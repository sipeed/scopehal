/***********************************************************************************************************************
*                                                                                                                      *
* libscopehal                                                                                                          *
*                                                                                                                      *
* Copyright (c) 2012-2026 Andrew D. Zonenberg and contributors                                                         *
* All rights reserved.                                                                                                 *
*                                                                                                                      *
* Redistribution and use in source and binary forms, with or without modification, are permitted provided that the     *
* following conditions are met:                                                                                        *
*                                                                                                                      *
*    * Redistributions of source code must retain the above copyright notice, this list of conditions, and the         *
*      following disclaimer.                                                                                           *
*                                                                                                                      *
*    * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the       *
*      following disclaimer in the documentation and/or other materials provided with the distribution.                *
*                                                                                                                      *
*    * Neither the name of the author nor the names of any contributors may be used to endorse or promote products     *
*      derived from this software without specific prior written permission.                                           *
*                                                                                                                      *
* THIS SOFTWARE IS PROVIDED BY THE AUTHORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED   *
* TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL *
* THE AUTHORS BE HELD LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES        *
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR       *
* BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT *
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE       *
* POSSIBILITY OF SUCH DAMAGE.                                                                                          *
*                                                                                                                      *
***********************************************************************************************************************/

/**
	@file
	@author Andrew D. Zonenberg and contributors
	@brief Implementation of SLogicOscilloscope

	@ingroup scopedrivers
 */

#include "scopehal.h"
#include "OscilloscopeChannel.h"
#include "EdgeTrigger.h"
#include "SLogicOscilloscope.h"

#include <algorithm>
#include <chrono>

using namespace std;

//The 8-color SLogic digital palette, repeating every 8 channels (D8k+0..D8k+7).
//Shared with PulseView / SLogicView / ALL-Logic and SigrokOscilloscope.
static const char* const g_slogicDigitalColors[8] =
{
	"#ff0000",	// Red
	"#ff8000",	// Orange
	"#ffe000",	// Yellow
	"#00c000",	// Green
	"#a0522d",	// Brown
	"#2080ff",	// Blue
	"#ffffff",	// White
	"#a0a0a0",	// Grey
};

//When a byte-group's 8 pins are shown as one analog channel, the 8-bit value maps LINEARLY to the
//device's fixed ±15 V analog range (0 -> -15 V, 128 -> ~0 V, 255 -> +15 V). This mapping is a
//physical property of the data and is independent of the display voltage range (V/div), so zooming
//the vertical scale rescales the view without changing the sample values.
static const float g_slogicAnalogVfs  = 15.0f;	//±full scale (volts)
static const float g_slogicAnalogSpan = 30.0f;	//total span, g_slogicAnalogVfs * 2

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Construction / destruction

/**
	@brief Initialize the driver

	Opens the SLogic USB device directly (no bridge). Step 2: USB layer only —
	channel model, config, acquisition and trigger arrive in later steps.

	@param transport	SCPINullTransport (the device is found by USB enumeration)
 */
SLogicOscilloscope::SLogicOscilloscope(SCPITransport* transport)
	: SCPIDevice(transport, false)
	, SCPIInstrument(transport, false)
	, m_extTrigger(nullptr)
{
	m_vendor = "Sipeed";
	SCPIDevice::m_model = "SLogic";
	m_serial = "";

	//Open the first attached SLogic device (a specific one can be selected by
	//serial later via the connection string).
	if(!OpenDevice())
		LogError("SLogicOscilloscope: no SLogic device found / open failed\n");
}

SLogicOscilloscope::~SLogicOscilloscope()
{
	CloseDevice();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// USB layer: the shared libslogic core owns the device (claim, event thread, ring)

void SLogicOscilloscope::OnLog(void* /*user*/, int level, const char* msg)
{
	switch(level)
	{
		case SLOGIC_LOG_ERR:	LogError("SLogic: %s\n", msg); break;
		case SLOGIC_LOG_WARN:	LogWarning("SLogic: %s\n", msg); break;
		case SLOGIC_LOG_INFO:	LogNotice("SLogic: %s\n", msg); break;
		case SLOGIC_LOG_DBG:	LogDebug("SLogic: %s\n", msg); break;
		default:				break;
	}
}

bool SLogicOscilloscope::OpenDevice(const string& serial)
{
	if(libusb_init(&m_usbctx) != LIBUSB_SUCCESS)
	{
		LogError("SLogicOscilloscope: libusb_init failed\n");
		m_usbctx = nullptr;
		return false;
	}

	libusb_device* dev = nullptr;
	if(slogic_find(m_usbctx, 0, nullptr, serial.empty() ? nullptr : serial.c_str(), &dev) != SLOGIC_OK ||
		slogic_dev_open(m_usbctx, dev, &SLogicOscilloscope::OnLog, this, &m_sdev) != SLOGIC_OK)
	{
		if(dev)
			libusb_unref_device(dev);
		libusb_exit(m_usbctx);
		m_usbctx = nullptr;
		m_sdev = nullptr;
		return false;
	}

	//Serial string, read through the core's open handle
	struct libusb_device_descriptor des;
	char sbuf[128] = {0};
	if(libusb_get_device_descriptor(dev, &des) == LIBUSB_SUCCESS && des.iSerialNumber)
		libusb_get_string_descriptor_ascii(slogic_dev_handle(m_sdev), des.iSerialNumber,
			(unsigned char*)sbuf, sizeof(sbuf));
	libusb_unref_device(dev);

	m_model = slogic_dev_model(m_sdev);
	m_channelCount = m_model->physical_channels;
	//"(10G) SLogic32U3": link tag first so it survives a truncated name
	const char* tag = slogic_link_tag(slogic_dev_link(m_sdev));
	SCPIDevice::m_model = tag ? (string("(") + tag + ") " + m_model->name) : string(m_model->name);
	m_serial = sbuf;

	//Pick sensible defaults, snapped to the nearest advertised value so
	//GetSampleRate()/GetSampleDepth() always match the advertised lists (a value
	//outside the list re-queries the timebase every frame).
	{
		auto nearest = [](const vector<uint64_t>& vals, uint64_t target) -> uint64_t
		{
			if(vals.empty())
				return target;
			uint64_t best = vals[0];
			uint64_t bestDist = (best > target) ? (best - target) : (target - best);
			for(auto v : vals)
			{
				uint64_t d = (v > target) ? (v - target) : (target - v);
				if(d < bestDist) { bestDist = d; best = v; }
			}
			return best;
		};
		auto rates = GetSampleRatesNonInterleaved();
		auto depths = GetSampleDepthsNonInterleaved();
		m_srate = nearest(rates, 40000000ULL);		//40 MS/s
		m_mdepth = nearest(depths, 100000ULL);		//100 kS
	}

	//Create channels + banks + trigger for the current layout
	BuildChannels();

	LogNotice("SLogicOscilloscope: opened %s (serial \"%s\", %d channels)\n",
		m_model->name, m_serial.c_str(), m_channelCount);
	return true;
}

void SLogicOscilloscope::CloseDevice()
{
	if(m_sdev)
	{
		slogic_dev_close(m_sdev);	//destroys a live capture first
		m_sdev = nullptr;
	}
	if(m_usbctx)
	{
		libusb_exit(m_usbctx);
		m_usbctx = nullptr;
	}
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Channel model

///@brief Analog channel color by index (matches SigrokOscilloscope)
string SLogicOscilloscope::GetChannelColor(size_t i)
{
	switch(i % 8)
	{
		case 0: return "#4040ff";
		case 1: return "#ff4040";
		case 2: return "#208020";
		case 3: return "#ffff00";
		case 4: return "#600080";
		case 5: return "#808080";
		case 6: return "#40a0a0";
		case 7:
		default: return "#e040e0";
	}
}

/**
	@brief Create the full physical channel set, then enable the active subset.

	Every byte-group g gets BOTH an analog channel A<g> and 8 digital channels
	D<8g+k>, at stable indices (group g occupies indices g*9 .. g*9+8: A at g*9,
	D<8g+0..7> at g*9+1..g*9+8). The active layout is expressed purely by which
	channels are enabled — so max-channels and per-group analog/digital can be
	changed later (Step 6/Phase 2) without recreating channel objects.
 */
void SLogicOscilloscope::BuildChannels()
{
	if(!m_model)
		return;

	size_t ngroups = m_model->physical_channels / 8;
	size_t nextChannel = 0;
	for(size_t g = 0; g < ngroups; g++)
	{
		//Analog channel A<g> covering the whole byte
		{
			size_t idx = nextChannel++;
			auto chan = new OscilloscopeChannel(
				this,
				"A" + to_string(g),
				GetChannelColor(idx),
				Unit(Unit::UNIT_FS),
				Unit(Unit::UNIT_VOLTS),
				Stream::STREAM_TYPE_ANALOG,
				idx);
			m_channels.push_back(chan);
			m_channelAttenuation[idx] = 1;	//1x = direct reading (±15 V); user picks 2x/etc. for an external probe
			m_channelCoupling[idx] = OscilloscopeChannel::COUPLE_DC_50;
			m_channelBandwidth[idx] = 0;
			m_channelVoltageRange[idx] = g_slogicAnalogSpan;	//show the full ±15 V by default
			m_channelOffset[idx] = 0;
		}
		//8 digital channels D<8g+k>, one per bit
		for(size_t k = 0; k < 8; k++)
		{
			size_t idx = nextChannel++;
			size_t dnum = 8*g + k;
			auto chan = new OscilloscopeChannel(
				this,
				"D" + to_string(dnum),
				g_slogicDigitalColors[dnum % 8],
				Unit(Unit::UNIT_FS),
				Unit(Unit::UNIT_COUNTS),
				Stream::STREAM_TYPE_DIGITAL,
				idx);
			m_channels.push_back(chan);
			m_channelAttenuation[idx] = 1;
			m_digitalHysteresis[idx] = 0.1;
		}
	}

	//Enable the active channel subset and build the analog/digital banks.
	ApplyChannelEnables();
	RebuildBanks();

	//Configure a rising-edge trigger on the first enabled channel
	size_t trigIdx = 0;
	for(size_t i = 0; i < m_channels.size(); i++)
	{
		if(IsChannelEnabled(i))
		{
			trigIdx = i;
			break;
		}
	}
	auto trig = new EdgeTrigger(this);
	trig->SetType(EdgeTrigger::EDGE_RISING);
	auto trigChan = GetOscilloscopeChannel(trigIdx);
	if(trigChan && trigChan->GetType(0) == Stream::STREAM_TYPE_ANALOG)
		trig->SetLevel(m_channelAttenuation[trigIdx] / 2.0);
	else
		trig->SetLevel(0);
	if(trigChan)
		trig->SetInput(0, StreamDescriptor(trigChan));
	SetTrigger(trig);
	SetTriggerOffset(1000000000000);	//1ms, to allow trigphase interpolation
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Information queries

string SLogicOscilloscope::IDPing()
{
	return "";
}

string SLogicOscilloscope::GetTransportName()
{
	return "null";
}

string SLogicOscilloscope::GetTransportConnectionString()
{
	return "";
}

///@brief Return the constant driver name "slogic"
string SLogicOscilloscope::GetDriverNameInternal()
{
	return "slogic";
}

unsigned int SLogicOscilloscope::GetInstrumentTypes() const
{
	return Instrument::INST_OSCILLOSCOPE;
}

uint32_t SLogicOscilloscope::GetInstrumentTypesForChannel(size_t /*i*/) const
{
	return Instrument::INST_OSCILLOSCOPE;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Triggering / capture control

Oscilloscope::TriggerMode SLogicOscilloscope::PollTrigger()
{
	//While armed, keep telling scopehal a trigger is ready so it calls AcquireData()
	//(the actual capture blocks inside AcquireData). Mirrors SigrokOscilloscope.
	return m_triggerArmed ? TRIGGER_MODE_TRIGGERED : TRIGGER_MODE_STOP;
}

void SLogicOscilloscope::Start()
{
	LogDebug("SLogic: Start (continuous)\n");
	m_lastTrigState = -1;	//new session: drop stale cross-buffer trigger state
	m_triggerArmed = true;
	m_triggerOneShot = false;
}

void SLogicOscilloscope::StartSingleTrigger()
{
	LogDebug("SLogic: StartSingleTrigger\n");
	m_lastTrigState = -1;
	m_triggerArmed = true;
	m_triggerOneShot = true;
}

void SLogicOscilloscope::Stop()
{
	LogDebug("SLogic: Stop\n");
	m_triggerArmed = false;
	m_captureAbort = true;	//break any in-progress capture wait
}

void SLogicOscilloscope::ForceTrigger()
{
	LogDebug("SLogic: ForceTrigger\n");
	m_lastTrigState = -1;
	m_triggerArmed = true;
	m_triggerOneShot = true;
}

bool SLogicOscilloscope::IsTriggerArmed()
{
	return m_triggerArmed;
}

void SLogicOscilloscope::PushTrigger()
{
	//Software trigger: nothing to program on the device, but the source/edge/level
	//may have changed, so drop the cross-buffer state so the next scan re-baselines.
	LogDebug("SLogic: PushTrigger (trigger source/edge/level changed)\n");
	m_lastTrigState = -1;
}

void SLogicOscilloscope::PullTrigger()
{
	//No hardware trigger to read back; the in-memory EdgeTrigger from BuildChannels stands.
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Acquisition (one finite capture through the libslogic capture engine)

int64_t SLogicOscilloscope::MonotonicUs()
{
	return chrono::duration_cast<chrono::microseconds>(
		chrono::steady_clock::now().time_since_epoch()).count();
}

int SLogicOscilloscope::OnSamples(void* user, const uint8_t* samples, size_t n, unsigned unitsize)
{
	auto* self = reinterpret_cast<SLogicOscilloscope*>(user);
	self->m_captureBuf.insert(self->m_captureBuf.end(), samples, samples + n * unitsize);
	return 0;
}

bool SLogicOscilloscope::AcquireData()
{
	if(!m_sdev || !m_model)
		return false;

	//Snapshot the user-facing config under m_mutex before using it. The UI thread mutates
	//these maps/scalars on clicks; reading them unlocked here races the UI and a concurrent
	//std::map mutation can spin/corrupt mid-traversal — the "click a few times -> freeze".
	//Take a consistent copy quickly, then release the lock (never hold it across the wait).
	uint64_t srate, mdepth;
	int chanCount, pattern;
	double vth = 1.6;
	{
		lock_guard<recursive_mutex> lock(m_mutex);
		srate = m_srate;
		mdepth = m_mdepth;
		chanCount = m_channelCount;
		pattern = m_patternMode;
		vth = m_digitalThreshold;	//single global comparator threshold for all digital inputs
	}

	unsigned unit = slogic_unitsize(chanCount);
	m_captureBuf.clear();
	m_captureBuf.reserve(mdepth * unit);
	m_captureAbort = false;

	slogic_capture_config cfg;
	memset(&cfg, 0, sizeof(cfg));
	cfg.dev.channel_count = chanCount;
	cfg.dev.samplerate_hz = srate;
	cfg.dev.threshold_v = vth;
	cfg.dev.pattern_mode = pattern;
	cfg.limit_samples = mdepth;
	cfg.sink.user = this;
	cfg.sink.data = &SLogicOscilloscope::OnSamples;
	m_captureCfg = cfg.dev;

	LogDebug("SLogic: AcquireData begin (ch=%d, rate=%llu Hz, depth=%llu, pattern=%d)\n",
		chanCount, (unsigned long long)srate, (unsigned long long)mdepth, pattern);

	slogic_capture* cap = nullptr;
	int r = slogic_capture_start(m_sdev, &cfg, &cap);
	if(r != SLOGIC_OK)
	{
		LogWarning("SLogic: capture start failed (%d)\n", r);
		return false;
	}

	//Poll on this (acquisition) thread until the capture ends, honoring Stop() and a guard
	int64_t startUs = MonotonicUs();
	int64_t guardUs = 5000000;	//5 s floor
	if(srate > 0)
	{
		int64_t expUs = (int64_t)((double)mdepth / (double)srate * 1e6);
		guardUs = max<int64_t>(guardUs, expUs * 4 + 2000000);
	}
	bool stopped = false;
	while(slogic_capture_poll(cap, 8, 10) == SLOGIC_CAP_RUNNING)
	{
		if(!stopped && (m_captureAbort || !m_triggerArmed || (MonotonicUs() - startUs) > guardUs))
		{
			if((MonotonicUs() - startUs) > guardUs)
				LogWarning("SLogic: capture guard timeout (%lld ms) — stopping\n",
					(long long)(guardUs / 1000));
			slogic_capture_stop(cap);
			stopped = true;
		}
	}
	slogic_report rep;
	slogic_capture_report(cap, &rep);
	slogic_capture_destroy(cap);	//the core already logged its slogic-report line

	if(rep.end == SLOGIC_END_ERROR || m_captureBuf.size() < unit)
		return false;

	ReshapeAndQueue();

	if(m_triggerOneShot)
		m_triggerArmed = false;
	return true;
}

/**
	@brief Scan the capture buffer for the active EdgeTrigger's edge.

	Ports the bridge's find_trigger_digital / find_trigger_analog including
	cross-buffer continuity (m_lastTrigState). Returns the trigger sample index,
	or -1 if no matching edge was found in this buffer.
 */
int64_t SLogicOscilloscope::FindTriggerSample(int unit)
{
	if(unit < 1)
		return -1;
	auto edge = dynamic_cast<EdgeTrigger*>(GetTrigger());
	if(!edge)
		return -1;
	auto in = edge->GetInput(0);
	if(!in.m_channel)
		return -1;
	size_t idx = in.m_channel->GetIndex();

	//Map channel index (g*9 base: analog A<g> at g*9, digital D<8g+k> at g*9+1+k) to byte-group + bit
	size_t g = idx / 9;
	size_t off = idx % 9;
	bool analog = (off == 0);
	size_t byteOffset = g;
	if(byteOffset >= (size_t)unit)
		return -1;

	uint64_t numSamples = m_captureBuf.size() / (size_t)unit;
	if(numSamples == 0)
		return -1;

	auto etype = edge->GetType();
	auto matched = [etype](bool curr) -> bool
	{
		switch(etype)
		{
			case EdgeTrigger::EDGE_RISING:	return curr;
			case EdgeTrigger::EDGE_FALLING:	return !curr;
			default:						return true;	//EDGE_ANY (and unsupported modes treated as any)
		}
	};

	int64_t found = -1;

	if(analog)
	{
		//scopehal trigger level is in displayed (post-attenuation) volts; reshape maps
		//0..255 -> (-15..+15 V) × attenuation, so invert: raw = ((level/atten) + Vfs) / span * 255.
		float lv = edge->GetLevel();
		float atten = m_channelAttenuation.count(idx) ? (float)m_channelAttenuation[idx] : 1.0f;
		if(atten == 0.0f)
			atten = 1.0f;
		int rawi = (int)((((lv / atten) + g_slogicAnalogVfs) / g_slogicAnalogSpan) * 255.0f + 0.5f);
		rawi = std::max(0, std::min(255, rawi));
		uint8_t thresh = (uint8_t)rawi;

		auto getVal = [&](uint64_t s) -> uint8_t { return m_captureBuf[s * (size_t)unit + byteOffset]; };

		bool prevAbove = false;
		uint64_t start = 0;
		if(m_lastTrigState >= 0)
			prevAbove = (m_lastTrigState != 0);
		else if(numSamples < 2)
		{
			m_lastTrigState = (getVal(numSamples - 1) >= thresh) ? 1 : 0;
			return -1;
		}
		else
		{
			prevAbove = (getVal(0) >= thresh);
			start = 1;
		}
		for(uint64_t i = start; i < numSamples; i++)
		{
			bool cur = (getVal(i) >= thresh);
			if(cur != prevAbove)
			{
				if(matched(cur)) { found = (int64_t)i; break; }
				prevAbove = cur;
			}
		}
		m_lastTrigState = (getVal(numSamples - 1) >= thresh) ? 1 : 0;
	}
	else
	{
		size_t bit = off - 1;
		uint8_t mask = (uint8_t)(1u << bit);
		auto getBit = [&](uint64_t s) -> bool { return (m_captureBuf[s * (size_t)unit + byteOffset] & mask) != 0; };

		bool prev = false;
		uint64_t start = 0;
		if(m_lastTrigState >= 0)
			prev = (m_lastTrigState != 0);
		else if(numSamples < 2)
		{
			m_lastTrigState = getBit(numSamples - 1) ? 1 : 0;
			return -1;
		}
		else
		{
			prev = getBit(0);
			start = 1;
		}
		for(uint64_t i = start; i < numSamples; i++)
		{
			bool cur = getBit(i);
			if(cur != prev)
			{
				if(matched(cur)) { found = (int64_t)i; break; }
				prev = cur;
			}
		}
		m_lastTrigState = getBit(numSamples - 1) ? 1 : 0;
	}

	return found;
}

/**
	@brief Reshape the sample-major capture buffer into scopehal waveforms.

	Runs the software trigger scan (FindTriggerSample) and crops the pre-trigger
	front so the emitted waveform's trigger sits at m_pretrigger (0 by default =
	trigger at the left edge, post-trigger only). No matching edge → the whole
	buffer is emitted untriggered from t=0 (matches the bridge policy). Analog
	byte-group → UniformAnalogWaveform (byteval/255*range); digital byte-group →
	8 SparseDigitalWaveform (RLE). Reuses SigrokOscilloscope's layout reshape.
 */
void SLogicOscilloscope::ReshapeAndQueue()
{
	//Reads config maps/scalars (channel count, sample rate, per-channel ranges, analog groups)
	//and the trigger config via FindTriggerSample. Hold m_mutex so a concurrent UI-thread setter
	//can't mutate these mid-read. Recursive mutex: FindTriggerSample below is covered by this.
	lock_guard<recursive_mutex> lock(m_mutex);

	//Use the channel count the buffer was actually captured with (m_captureCfg), not the live
	//m_channelCount — the user may have changed the channel mode between capture and reshape.
	int capCount = m_captureCfg.channel_count;
	if(capCount < 1)
		capCount = m_channelCount;
	int unit = capCount / 8;
	if(unit < 1)
		unit = 1;
	size_t activeGroups = (size_t)unit;
	uint64_t totalSamples = m_captureBuf.size() / unit;
	if(totalSamples == 0)
		return;

	//Software trigger scan + pretrigger crop
	uint64_t startSample = 0;
	int64_t trigSample = FindTriggerSample(unit);
	if(trigSample >= 0)
	{
		uint64_t pre = (m_pretrigger < (uint64_t)trigSample) ? m_pretrigger : (uint64_t)trigSample;
		startSample = (uint64_t)trigSample - pre;
	}
	uint64_t numSamples = totalSamples - startSample;
	if(numSamples == 0)
		return;

	int64_t fs_per_sample = (m_srate > 0) ? (int64_t)(1e15 / (double)m_srate) : 1;

	//Active layout: byte g of each sample → group g (analog A<g> at g*9, else D<8g+k> at g*9+1..8).
	//A partial last group emits only its valid low bits (e.g. 4 ch → D0..D3). Analog covers a whole
	//byte, so a group is analog only when fully inside the capture width.
	struct Grp { size_t byteOffset; bool analog; size_t firstChannel; size_t nbits; float atten; };
	vector<Grp> layout;
	for(size_t g=0; g<activeGroups; g++)
	{
		bool groupFull = ((g*8 + 8) <= (size_t)capCount);
		bool analog = (m_analogGroups.count(g) > 0) && groupFull;
		size_t base = g*9;
		size_t nbits = (size_t)capCount - g*8;
		if(nbits > 8)
			nbits = 8;
		//Probe attenuation for the analog view (A<g> at index base): displayed volts = raw × attenuation.
		float atten = m_channelAttenuation.count(base) ? (float)m_channelAttenuation[base] : 1.0f;
		layout.push_back({ g, analog, analog ? base : base+1, nbits, atten });
	}

	double t = GetTime();
	int64_t fs = (int64_t)((t - floor(t)) * FS_PER_SECOND);

	//Reshape sample-major bytes into per-channel waveforms. This is the heaviest step
	//of a capture, so the digital groups are converted on worker threads, and each
	//digital bit gets whichever representation is cheaper:
	//
	//  * Sparse (RLE) when transitions are rare — the normal case for real signals.
	//  * Dense (uniform, one byte per sample) when a bit toggles often. The Emulation
	//    counter's low bits flip almost every sample, so RLE is *larger* than dense
	//    there; at deep depths the old sparse-only, single-threaded path grew the run
	//    vectors to tens of GB of push_back()s and froze ngscopeclient. Dense is
	//    bounded and reallocation-free.
	//
	//To keep AcceleratorBuffer allocation off the worker threads, the work is phased:
	//count (parallel) -> allocate exact-size buffers (serial) -> fill (parallel).
	SequenceSet s;
	size_t totalCh = m_channels.size();

	if(numSamples > 100ull*1000*1000)
		LogWarning("SLogic: reshaping a deep capture (%llu samples x %d ch) may use several GB of RAM\n",
			(unsigned long long)numSamples, capCount);

	//Analog groups are always dense and cheap; fill them inline.
	for(auto& g : layout)
	{
		if(!g.analog)
			continue;
		auto cap = new UniformAnalogWaveform;
		cap->m_timescale = fs_per_sample;
		cap->m_triggerPhase = 0;
		cap->m_startTimestamp = time(NULL);
		cap->m_startFemtoseconds = fs;
		cap->PrepareForCpuAccess();
		cap->Resize(numSamples);
		for(uint64_t i=0; i<numSamples; i++)
		{
			uint8_t byteval = m_captureBuf[(size_t)(startSample + i) * unit + g.byteOffset];
			//Fixed physical mapping (0..255 -> -15..+15 V) scaled by the probe attenuation, and
			//independent of the display range so vertical zoom rescales the view, not the data.
			cap->m_samples[i] = (-g_slogicAnalogVfs + (byteval / 255.0f) * g_slogicAnalogSpan) * g.atten;
		}
		cap->MarkSamplesModifiedFromCpu();
		s[GetOscilloscopeChannel(g.firstChannel)] = cap;
	}

	//Digital groups: one worker per byte-group.
	vector<Grp> dgroups;
	for(auto& g : layout)
		if(!g.analog)
			dgroups.push_back(g);

	const uint8_t* buf = m_captureBuf.data();
	const size_t stride = (size_t)unit;
	const size_t base0 = (size_t)startSample * stride;

	//Per-channel conversion state, indexed by oscilloscope channel index.
	vector<uint64_t> runs(totalCh, 0);
	vector<SparseDigitalWaveform*> sparse(totalCh, nullptr);
	vector<UniformDigitalWaveform*> dense(totalCh, nullptr);
	vector<int64_t*> soff(totalCh, nullptr);
	vector<int64_t*> sdur(totalCh, nullptr);

	//Phase 1 (parallel): count transitions -> sparse run count, one scan per byte column.
	auto countGroup = [&](const Grp& g)
	{
		uint8_t prev[8];
		uint64_t r[8];
		for(size_t k=0; k<g.nbits; k++)
		{
			prev[k] = (buf[base0 + g.byteOffset] >> k) & 1;
			r[k] = 1;
		}
		for(uint64_t i=1; i<numSamples; i++)
		{
			uint8_t byteval = buf[base0 + i*stride + g.byteOffset];
			for(size_t k=0; k<g.nbits; k++)
			{
				uint8_t bit = (byteval >> k) & 1;
				if(bit != prev[k])
				{
					r[k]++;
					prev[k] = bit;
				}
			}
		}
		for(size_t k=0; k<g.nbits; k++)
			runs[g.firstChannel + k] = r[k];
	};
	{
		vector<thread> workers;
		for(size_t gi=0; gi<dgroups.size(); gi++)
			workers.emplace_back([&,gi]{ countGroup(dgroups[gi]); });
		for(auto& w : workers)
			w.join();
	}

	//Phase 2 (serial): allocate each bit with the cheaper representation at exact size.
	//A sparse run costs 17 bytes (two int64 offset+duration, one bool); dense costs 1 byte/sample.
	for(auto& g : dgroups)
	{
		for(size_t k=0; k<g.nbits; k++)
		{
			size_t idx = g.firstChannel + k;
			if(runs[idx] * 17 < numSamples)
			{
				auto cap = new SparseDigitalWaveform;
				cap->m_timescale = fs_per_sample;
				cap->m_triggerPhase = 0;
				cap->m_startTimestamp = time(NULL);
				cap->m_startFemtoseconds = fs;
				cap->PrepareForCpuAccess();
				cap->Resize(runs[idx]);
				sparse[idx] = cap;
				soff[idx] = cap->m_offsets.GetCpuPointer();
				sdur[idx] = cap->m_durations.GetCpuPointer();
			}
			else
			{
				auto cap = new UniformDigitalWaveform;
				cap->m_timescale = fs_per_sample;
				cap->m_triggerPhase = 0;
				cap->m_startTimestamp = time(NULL);
				cap->m_startFemtoseconds = fs;
				cap->PrepareForCpuAccess();
				cap->Resize(numSamples);
				dense[idx] = cap;
			}
		}
	}

	//Phase 3 (parallel): fill the pre-allocated buffers, one scan per byte column.
	auto fillGroup = [&](const Grp& g)
	{
		uint8_t sprev[8];
		int64_t sstart[8];
		uint64_t sr[8];
		for(size_t k=0; k<g.nbits; k++)
		{
			size_t idx = g.firstChannel + k;
			uint8_t bit = (buf[base0 + g.byteOffset] >> k) & 1;
			sprev[k] = bit;
			sstart[k] = 0;
			sr[k] = 0;
			if(dense[idx])
				dense[idx]->m_samples[0] = bit ? true : false;
		}
		for(uint64_t i=1; i<numSamples; i++)
		{
			uint8_t byteval = buf[base0 + i*stride + g.byteOffset];
			for(size_t k=0; k<g.nbits; k++)
			{
				size_t idx = g.firstChannel + k;
				uint8_t bit = (byteval >> k) & 1;
				if(dense[idx])
					dense[idx]->m_samples[i] = bit ? true : false;
				else if(bit != sprev[k])
				{
					soff[idx][sr[k]] = sstart[k];
					sdur[idx][sr[k]] = (int64_t)i - sstart[k];
					sparse[idx]->m_samples[sr[k]] = sprev[k] ? true : false;
					sr[k]++;
					sprev[k] = bit;
					sstart[k] = (int64_t)i;
				}
			}
		}
		//Flush the final run and mark buffers modified.
		for(size_t k=0; k<g.nbits; k++)
		{
			size_t idx = g.firstChannel + k;
			if(dense[idx])
				dense[idx]->MarkSamplesModifiedFromCpu();
			else
			{
				soff[idx][sr[k]] = sstart[k];
				sdur[idx][sr[k]] = (int64_t)numSamples - sstart[k];
				sparse[idx]->m_samples[sr[k]] = sprev[k] ? true : false;
				sparse[idx]->MarkSamplesModifiedFromCpu();
				sparse[idx]->MarkTimestampsModifiedFromCpu();
			}
		}
	};
	{
		vector<thread> workers;
		for(size_t gi=0; gi<dgroups.size(); gi++)
			workers.emplace_back([&,gi]{ fillGroup(dgroups[gi]); });
		for(auto& w : workers)
			w.join();
	}

	//Publish digital waveforms into the SequenceSet (serial; std::map is not thread-safe).
	for(auto& g : dgroups)
		for(size_t k=0; k<g.nbits; k++)
		{
			size_t idx = g.firstChannel + k;
			s[GetOscilloscopeChannel(idx)] =
				dense[idx] ? (WaveformBase*)dense[idx] : (WaveformBase*)sparse[idx];
		}

	//Queue the waveform set (keep at most 2 pending, like SigrokOscilloscope)
	m_pendingWaveformsMutex.lock();
	m_pendingWaveforms.push_back(s);
	while(m_pendingWaveforms.size() > 2)
	{
		SequenceSet set = *m_pendingWaveforms.begin();
		for(auto it : set)
			delete it.second;
		m_pendingWaveforms.pop_front();
	}
	m_pendingWaveformsMutex.unlock();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Serialization

void SLogicOscilloscope::LoadConfiguration(int version, const YAML::Node& node, IDTable& table)
{
	//Call the base class to configure everything
	Oscilloscope::LoadConfiguration(version, node, table);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Channel configuration. All trivial stubs for now.

bool SLogicOscilloscope::IsChannelEnabled(size_t i)
{
	lock_guard<recursive_mutex> lock(m_mutex);
	auto it = m_channelsEnabled.find(i);
	return (it != m_channelsEnabled.end()) ? it->second : false;
}

void SLogicOscilloscope::EnableChannel(size_t i)
{
	lock_guard<recursive_mutex> lock(m_mutex);
	m_channelsEnabled[i] = true;
	LogDebug("SLogic: EnableChannel(%zu)\n", i);
}

void SLogicOscilloscope::DisableChannel(size_t i)
{
	lock_guard<recursive_mutex> lock(m_mutex);
	m_channelsEnabled[i] = false;
	LogDebug("SLogic: DisableChannel(%zu)\n", i);
}

OscilloscopeChannel::CouplingType SLogicOscilloscope::GetChannelCoupling(size_t i)
{
	lock_guard<recursive_mutex> lock(m_mutex);
	auto it = m_channelCoupling.find(i);
	return (it != m_channelCoupling.end()) ? it->second : OscilloscopeChannel::COUPLE_DC_50;
}

vector<OscilloscopeChannel::CouplingType> SLogicOscilloscope::GetAvailableCouplings(size_t /*i*/)
{
	//Logic analyzer front end: fixed DC coupling
	return { OscilloscopeChannel::COUPLE_DC_50 };
}

void SLogicOscilloscope::SetChannelCoupling(size_t i, OscilloscopeChannel::CouplingType type)
{
	lock_guard<recursive_mutex> lock(m_mutex);
	m_channelCoupling[i] = type;
}

double SLogicOscilloscope::GetChannelAttenuation(size_t i)
{
	lock_guard<recursive_mutex> lock(m_mutex);
	auto it = m_channelAttenuation.find(i);
	return (it != m_channelAttenuation.end()) ? it->second : 1;
}

void SLogicOscilloscope::SetChannelAttenuation(size_t i, double atten)
{
	lock_guard<recursive_mutex> lock(m_mutex);
	m_channelAttenuation[i] = atten;
}

unsigned int SLogicOscilloscope::GetChannelBandwidthLimit(size_t i)
{
	lock_guard<recursive_mutex> lock(m_mutex);
	auto it = m_channelBandwidth.find(i);
	return (it != m_channelBandwidth.end()) ? it->second : 0;
}

void SLogicOscilloscope::SetChannelBandwidthLimit(size_t i, unsigned int limit_mhz)
{
	lock_guard<recursive_mutex> lock(m_mutex);
	m_channelBandwidth[i] = limit_mhz;
}

float SLogicOscilloscope::GetChannelVoltageRange(size_t i, size_t /*stream*/)
{
	lock_guard<recursive_mutex> lock(m_mutex);
	auto it = m_channelVoltageRange.find(i);
	return (it != m_channelVoltageRange.end()) ? it->second : 5;
}

void SLogicOscilloscope::SetChannelVoltageRange(size_t i, size_t /*stream*/, float range)
{
	lock_guard<recursive_mutex> lock(m_mutex);
	m_channelVoltageRange[i] = range;
}

bool SLogicOscilloscope::IsHighRateOffsetCapable(size_t /*i*/)
{
	return false;
}

OscilloscopeChannel* SLogicOscilloscope::GetExternalTrigger()
{
	return m_extTrigger;
}

float SLogicOscilloscope::GetChannelOffset(size_t i, size_t /*stream*/)
{
	lock_guard<recursive_mutex> lock(m_mutex);
	auto it = m_channelOffset.find(i);
	return (it != m_channelOffset.end()) ? it->second : 0;
}

void SLogicOscilloscope::SetChannelOffset(size_t i, size_t /*stream*/, float offset)
{
	lock_guard<recursive_mutex> lock(m_mutex);
	m_channelOffset[i] = offset;
}

vector<uint64_t> SLogicOscilloscope::GetSampleRatesNonInterleaved()
{
	//Model's advertised discrete rates, filtered to the channel-mode ceiling
	lock_guard<recursive_mutex> lock(m_mutex);
	vector<uint64_t> ret;
	if(!m_model)
		return ret;
	uint64_t ceil = m_sdev ? slogic_link_max_rate(m_model, m_channelCount, slogic_dev_link(m_sdev))
		: slogic_max_rate(m_model, m_channelCount);
	for(size_t i = 0; i < m_model->rate_count; i++)
	{
		uint64_t r = m_model->rates[i];
		if(!ceil || (r <= ceil))
			ret.push_back(r);
	}
	return ret;
}

vector<uint64_t> SLogicOscilloscope::GetSampleRatesInterleaved()
{
	return GetSampleRatesNonInterleaved();
}

set<Oscilloscope::InterleaveConflict> SLogicOscilloscope::GetInterleaveConflicts()
{
	return {};
}

vector<uint64_t> SLogicOscilloscope::GetSampleDepthsNonInterleaved()
{
	//1-2-5 decade sequence 1k..500M, matching the libsigrok slogic driver's
	//sampledepths_slogic list that the bridge advertised.
	static const uint64_t depths[] =
	{
		1000, 2000, 5000,
		10000, 20000, 50000,
		100000, 200000, 500000,
		1000000, 2000000, 5000000,
		10000000, 20000000, 50000000,
		100000000, 200000000, 500000000,
	};
	return vector<uint64_t>(depths, depths + sizeof(depths)/sizeof(depths[0]));
}

vector<uint64_t> SLogicOscilloscope::GetSampleDepthsInterleaved()
{
	return GetSampleDepthsNonInterleaved();
}

uint64_t SLogicOscilloscope::GetSampleRate()
{
	lock_guard<recursive_mutex> lock(m_mutex);
	return m_srate;
}

uint64_t SLogicOscilloscope::GetSampleDepth()
{
	lock_guard<recursive_mutex> lock(m_mutex);
	return m_mdepth;
}

void SLogicOscilloscope::SetSampleDepth(uint64_t depth)
{
	lock_guard<recursive_mutex> lock(m_mutex);
	m_mdepth = depth;
	LogDebug("SLogic: SetSampleDepth(%llu)\n", (unsigned long long)depth);
}

void SLogicOscilloscope::SetSampleRate(uint64_t rate)
{
	lock_guard<recursive_mutex> lock(m_mutex);
	m_srate = rate;
	LogDebug("SLogic: SetSampleRate(%llu)\n", (unsigned long long)rate);
}

void SLogicOscilloscope::SetTriggerOffset(int64_t offset)
{
	lock_guard<recursive_mutex> lock(m_mutex);
	m_triggerOffset = offset;
	LogDebug("SLogic: SetTriggerOffset(%lld)\n", (long long)offset);
}

int64_t SLogicOscilloscope::GetTriggerOffset()
{
	lock_guard<recursive_mutex> lock(m_mutex);
	return m_triggerOffset;
}

bool SLogicOscilloscope::CanInterleave()
{
	return false;
}

bool SLogicOscilloscope::IsInterleaving()
{
	return false;
}

bool SLogicOscilloscope::SetInterleaving(bool /*combine*/)
{
	return false;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Digital threshold / hysteresis

bool SLogicOscilloscope::IsDigitalThresholdConfigurable()
{
	//The device exposes a single programmable comparator threshold for all digital inputs.
	return true;
}

float SLogicOscilloscope::GetDigitalThreshold(size_t /*channel*/)
{
	//Global threshold: the device has one comparator Vth shared by every digital input.
	lock_guard<recursive_mutex> lock(m_mutex);
	return m_digitalThreshold;
}

void SLogicOscilloscope::SetDigitalThreshold(size_t /*channel*/, float level)
{
	//Global threshold: editing any digital channel retargets the one device-wide Vth, so every
	//channel reads back the same value and there is no per-channel threshold to drift out of sync.
	lock_guard<recursive_mutex> lock(m_mutex);
	m_digitalThreshold = level;
	m_lastTrigState = -1;	//threshold changes the captured bits; re-baseline the cross-buffer scan
	LogDebug("SLogic: SetDigitalThreshold(%.3f V, global)\n", level);
}

float SLogicOscilloscope::GetDigitalHysteresis(size_t channel)
{
	lock_guard<recursive_mutex> lock(m_mutex);
	auto it = m_digitalHysteresis.find(channel);
	return (it != m_digitalHysteresis.end()) ? it->second : 0.1;
}

void SLogicOscilloscope::SetDigitalHysteresis(size_t channel, float level)
{
	lock_guard<recursive_mutex> lock(m_mutex);
	m_digitalHysteresis[channel] = level;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Capture pattern

//Instrument-global capture pattern selector (Normal / USB connection test / Emulation).
//Emulation drives a structured on-device test pattern, giving a visible striped waveform
//as a workaround while the digital last-transition Vulkan render bug is unresolved.
//Applied on the next capture via slogic_configure/slogic_run (m_captureCfg.pattern_mode).
bool SLogicOscilloscope::IsPatternModeConfigurable()
{
	return true;
}

vector<string> SLogicOscilloscope::GetPatternModeNames()
{
	vector<string> names;
	for(int i=0; i<SLOGIC_PATTERN_COUNT; i++)
		names.push_back(slogic_pattern_names[i]);
	return names;
}

size_t SLogicOscilloscope::GetPatternMode()
{
	lock_guard<recursive_mutex> lock(m_mutex);
	return (size_t)m_patternMode;
}

void SLogicOscilloscope::SetPatternMode(size_t mode)
{
	lock_guard<recursive_mutex> lock(m_mutex);
	if(mode < (size_t)SLOGIC_PATTERN_COUNT)
	{
		m_patternMode = (int)mode;
		LogDebug("SLogic: SetPatternMode(%zu = %s)\n", mode,
			(mode < (size_t)SLOGIC_PATTERN_COUNT) ? slogic_pattern_names[mode] : "?");
	}
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Channel mode (capture width)

//The model's channel->max-rate limit table is the set of selectable capture widths: fewer active
//channels allow a higher sample rate (e.g. 32u3: 32 ch @ 200 MHz .. 4 ch @ 1400 MHz). Presented
//highest channel count first so full width is the default at the top of the list.
vector<slogic_rate_limit> SLogicOscilloscope::CaptureWidthOptions()
{
	vector<slogic_rate_limit> opts;
	if(!m_model)
		return opts;
	for(size_t i=0; i<m_model->limit_count; i++)
	{
		//Ceiling for this channel mode on the current link (32U3 on 5G: 4ch@800 ...)
		slogic_rate_limit l = m_model->limits[i];
		if(m_sdev)
			l.max_rate_hz = slogic_link_max_rate(m_model, l.channels, slogic_dev_link(m_sdev));
		opts.push_back(l);
	}
	sort(opts.begin(), opts.end(),
		[](const slogic_rate_limit& a, const slogic_rate_limit& b) { return a.channels > b.channels; });
	return opts;
}

bool SLogicOscilloscope::IsCaptureWidthConfigurable()
{
	return m_model && (m_model->limit_count > 1);
}

vector<string> SLogicOscilloscope::GetCaptureWidthNames()
{
	vector<string> names;
	for(auto& o : CaptureWidthOptions())
		names.push_back(to_string(o.channels) + " ch \xE2\x80\x94 max " +
			to_string((unsigned long long)(o.max_rate_hz / 1000000ULL)) + " MHz");
	return names;
}

size_t SLogicOscilloscope::GetCaptureWidth()
{
	lock_guard<recursive_mutex> lock(m_mutex);
	auto opts = CaptureWidthOptions();
	for(size_t i=0; i<opts.size(); i++)
	{
		if(opts[i].channels == m_channelCount)
			return i;
	}
	return 0;
}

void SLogicOscilloscope::SetCaptureWidth(size_t mode)
{
	lock_guard<recursive_mutex> lock(m_mutex);
	auto opts = CaptureWidthOptions();
	if(mode >= opts.size())
		return;
	int newCount = opts[mode].channels;
	uint64_t maxRate = opts[mode].max_rate_hz;
	m_channelCount = newCount;

	//Drop analog groups that are no longer fully inside the new width (analog covers a whole byte).
	for(auto it = m_analogGroups.begin(); it != m_analogGroups.end(); )
	{
		if((*it * 8 + 8) > (size_t)newCount)
			it = m_analogGroups.erase(it);
		else
			++it;
	}

	ApplyChannelEnables();
	RebuildBanks();

	//Clamp the sample rate to the new ceiling, snapping down to the highest advertised rate <= ceiling.
	if(m_srate > maxRate)
	{
		uint64_t best = maxRate;
		if(m_model)
		{
			best = 0;
			for(size_t i=0; i<m_model->rate_count; i++)
			{
				uint64_t r = m_model->rates[i];
				if((r <= maxRate) && (r > best))
					best = r;
			}
			if(best == 0)
				best = maxRate;
		}
		m_srate = best;
	}

	RetargetTriggerIfNeeded();

	m_lastTrigState = -1;	//trigger source/width changed: re-baseline the cross-buffer scan
	LogDebug("SLogic: SetCaptureWidth -> %d ch (max %llu MHz)\n",
		newCount, (unsigned long long)(maxRate / 1000000ULL));
}

size_t SLogicOscilloscope::GetChannelGroupCount()
{
	lock_guard<recursive_mutex> lock(m_mutex);
	//Only fully-active byte-groups can be analog; a partial last group (e.g. 4 ch) offers no toggle.
	return (size_t)(m_channelCount / 8);
}

bool SLogicOscilloscope::IsChannelGroupAnalog(size_t group)
{
	lock_guard<recursive_mutex> lock(m_mutex);
	return m_analogGroups.count(group) > 0;
}

void SLogicOscilloscope::SetChannelGroupAnalog(size_t group, bool analog)
{
	lock_guard<recursive_mutex> lock(m_mutex);
	//Analog covers a whole byte, so only a fully-active group can be toggled.
	if((group * 8 + 8) > (size_t)m_channelCount)
		return;
	if(analog)
		m_analogGroups.insert(group);
	else
		m_analogGroups.erase(group);

	//Analog/digital only changes interpretation of the same captured byte, not the capture width,
	//so no rate/re-plan change is needed — just re-enable channels and refresh banks + trigger.
	ApplyChannelEnables();
	RebuildBanks();
	RetargetTriggerIfNeeded();
	m_lastTrigState = -1;
	LogDebug("SLogic: SetChannelGroupAnalog(group=%zu, %s)\n", group, analog ? "analog" : "digital");
}

//Channel indices making up byte-group g: the analog A<g> at g*9, then the 8 digital D<8g+k> at g*9+1..8.
vector<size_t> SLogicOscilloscope::GetChannelGroupChannelIndices(size_t group)
{
	lock_guard<recursive_mutex> lock(m_mutex);
	vector<size_t> idx;
	size_t base = group * 9;
	if(base >= m_channels.size())
		return idx;
	for(size_t k=0; k<9 && (base + k) < m_channels.size(); k++)
		idx.push_back(base + k);
	return idx;
}

//Retarget the edge trigger to the first enabled channel if its current input channel is disabled.
void SLogicOscilloscope::RetargetTriggerIfNeeded()
{
	auto edge = dynamic_cast<EdgeTrigger*>(GetTrigger());
	if(!edge)
		return;
	auto in = edge->GetInput(0);
	for(size_t i=0; i<m_channels.size(); i++)
	{
		if((in.m_channel == GetOscilloscopeChannel(i)) && IsChannelEnabled(i))
			return;	//still enabled, nothing to do
	}
	for(size_t i=0; i<m_channels.size(); i++)
	{
		if(IsChannelEnabled(i))
		{
			edge->SetInput(0, StreamDescriptor(GetOscilloscopeChannel(i)));
			edge->SetLevel(0);
			return;
		}
	}
}

//Enable/disable the physical channel set from m_channelCount + m_analogGroups, handling a partial
//last byte-group (e.g. 4 ch enables only D0..D3). Analog covers a whole byte, so a group can be
//analog only when fully within the capture width.
void SLogicOscilloscope::ApplyChannelEnables()
{
	if(!m_model)
		return;
	size_t ngroups = (size_t)m_model->physical_channels / 8;
	for(size_t g=0; g<ngroups; g++)
	{
		size_t base = g * 9;	//A<g> at base; D<8g+k> at base+1+k
		bool analog = m_analogGroups.count(g) > 0;
		bool groupFull = ((g*8 + 8) <= (size_t)m_channelCount);
		bool asAnalog = analog && groupFull;
		if(asAnalog)
			EnableChannel(base);
		else
			DisableChannel(base);
		for(size_t k=0; k<8; k++)
		{
			size_t d = 8*g + k;
			if((d < (size_t)m_channelCount) && !asAnalog)
				EnableChannel(base + 1 + k);
			else
				DisableChannel(base + 1 + k);
		}
	}
}

void SLogicOscilloscope::RebuildBanks()
{
	m_analogBanks.clear();
	m_digitalBanks.clear();
	AnalogBank abank;
	DigitalBank dbank;
	for(size_t i=0; i<m_channels.size(); i++)
	{
		auto chan = GetOscilloscopeChannel(i);
		if(!chan)
			continue;
		if(chan->GetType(0) == Stream::STREAM_TYPE_ANALOG)
			abank.push_back(chan);
		else if(chan->GetType(0) == Stream::STREAM_TYPE_DIGITAL)
			dbank.push_back(chan);
	}
	if(!abank.empty())
		m_analogBanks.push_back(abank);
	if(!dbank.empty())
		m_digitalBanks.push_back(dbank);
}

vector<Oscilloscope::AnalogBank> SLogicOscilloscope::GetAnalogBanks()
{
	return m_analogBanks;
}

Oscilloscope::AnalogBank SLogicOscilloscope::GetAnalogBank(size_t i)
{
	for(auto b : m_analogBanks)
		if(std::find(b.begin(), b.end(), GetOscilloscopeChannel(i)) != b.end())
			return b;
	return {};
}

vector<Oscilloscope::DigitalBank> SLogicOscilloscope::GetDigitalBanks()
{
	return m_digitalBanks;
}
