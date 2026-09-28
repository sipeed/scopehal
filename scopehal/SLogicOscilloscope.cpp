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

	BindTransport();

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
// USB layer (transport shim + device open/claim + libusb event thread)

int SLogicOscilloscope::CtrlWrite(void* ctx, uint8_t bRequest, uint16_t wValue,
	uint16_t wIndex, const uint8_t* data, uint16_t len, unsigned timeoutMs)
{
	auto* self = reinterpret_cast<SLogicOscilloscope*>(ctx);
	if(!self || !self->m_devh)
		return LIBUSB_ERROR_NO_DEVICE;
	lock_guard<mutex> lk(self->m_transportMutex);
	return libusb_control_transfer(self->m_devh,
		LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_ENDPOINT_OUT, bRequest,
		wValue, wIndex, const_cast<uint8_t*>(data), len, (int)timeoutMs);
}

int SLogicOscilloscope::CtrlRead(void* ctx, uint8_t bRequest, uint16_t wValue,
	uint16_t wIndex, uint8_t* data, uint16_t len, unsigned timeoutMs)
{
	auto* self = reinterpret_cast<SLogicOscilloscope*>(ctx);
	if(!self || !self->m_devh)
		return LIBUSB_ERROR_NO_DEVICE;
	lock_guard<mutex> lk(self->m_transportMutex);
	return libusb_control_transfer(self->m_devh,
		LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_ENDPOINT_IN, bRequest,
		wValue, wIndex, data, len, (int)timeoutMs);
}

void SLogicOscilloscope::BindTransport()
{
	m_transport.ctx = this;
	m_transport.control_write = &SLogicOscilloscope::CtrlWrite;
	m_transport.control_read = &SLogicOscilloscope::CtrlRead;
}

bool SLogicOscilloscope::OpenDevice(const string& serial)
{
	if(libusb_init(&m_usbctx) != LIBUSB_SUCCESS)
	{
		LogError("SLogicOscilloscope: libusb_init failed\n");
		m_usbctx = nullptr;
		return false;
	}

	libusb_device** list = nullptr;
	ssize_t n = libusb_get_device_list(m_usbctx, &list);
	for(ssize_t i=0; i<n; i++)
	{
		libusb_device* dev = list[i];
		struct libusb_device_descriptor des;
		if(libusb_get_device_descriptor(dev, &des) != LIBUSB_SUCCESS)
			continue;
		if(des.idVendor != SLOGIC_VID)
			continue;
		const slogic_model* model = slogic_model_for_pid(des.idProduct);
		if(!model)
			continue;

		libusb_device_handle* h = nullptr;
		if(libusb_open(dev, &h) != LIBUSB_SUCCESS)
			continue;

		//Read serial + product strings
		char sbuf[128] = {0};
		if(des.iSerialNumber)
			libusb_get_string_descriptor_ascii(h, des.iSerialNumber, (unsigned char*)sbuf, sizeof(sbuf));
		string devSerial = sbuf;

		//If a specific serial was requested, skip non-matching devices
		if(!serial.empty() && devSerial != serial)
		{
			libusb_close(h);
			continue;
		}

		m_devh = h;
		m_model = model;
		m_channelCount = model->physical_channels;
		SCPIDevice::m_model = model->name;
		m_serial = devSerial;
		break;
	}
	if(list)
		libusb_free_device_list(list, 1);

	if(!m_devh)
	{
		libusb_exit(m_usbctx);
		m_usbctx = nullptr;
		return false;
	}

	//Detach any kernel driver, then claim interface 0
	libusb_set_auto_detach_kernel_driver(m_devh, 1);
	int cr = libusb_claim_interface(m_devh, 0);
	if(cr != LIBUSB_SUCCESS)
	{
		LogError("SLogicOscilloscope: claim interface 0 failed: %s\n", libusb_error_name(cr));
		libusb_close(m_devh);
		m_devh = nullptr;
		libusb_exit(m_usbctx);
		m_usbctx = nullptr;
		return false;
	}

	//Start the libusb event-handling thread before any async work
	m_eventThreadRun = true;
	m_eventThread = thread(&SLogicOscilloscope::EventThreadFunc, this);

	//Reset the device to a known state (model-aware; safe for all models)
	if(slogic_reset(m_model, &m_transport) != SLOGIC_OK)
		LogWarning("SLogicOscilloscope: slogic_reset failed\n");

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

void SLogicOscilloscope::EventThreadFunc()
{
	while(m_eventThreadRun)
	{
		struct timeval tv;
		tv.tv_sec = 0;
		tv.tv_usec = 100000;	//100 ms
		libusb_handle_events_timeout_completed(m_usbctx, &tv, nullptr);
	}
}

void SLogicOscilloscope::CloseDevice()
{
	m_eventThreadRun = false;
	if(m_eventThread.joinable())
		m_eventThread.join();

	if(m_devh)
	{
		libusb_release_interface(m_devh, 0);
		libusb_close(m_devh);
		m_devh = nullptr;
	}
	if(m_usbctx)
	{
		libusb_exit(m_usbctx);
		m_usbctx = nullptr;
	}
}

void SLogicOscilloscope::DrainEndpoint()
{
	if(!m_devh || !m_model)
		return;

	//Synchronously read + discard whatever is sitting in the bulk IN endpoint until
	//it runs dry (short reads that time out with 0 bytes) or a loop cap is hit.
	//Called between captures when no async ring transfer is in flight.
	vector<uint8_t> tmp(64 * 1024);
	int loops = 0;
	int xfer = 0;
	int r;
	do
	{
		xfer = 0;
		r = libusb_bulk_transfer(m_devh, m_model->ep_in, tmp.data(), (int)tmp.size(), &xfer, 50);
		loops++;
	} while((r == 0) && (xfer > 0) && (loops < 32));
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
			m_channelAttenuation[idx] = 10;
			m_channelCoupling[idx] = OscilloscopeChannel::COUPLE_DC_50;
			m_channelBandwidth[idx] = 0;
			m_channelVoltageRange[idx] = 5;
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
			m_digitalThresholds[idx] = 1.6;
			m_digitalHysteresis[idx] = 0.1;
		}
	}

	//Enable the active subset: groups 0..(m_channelCount/8 - 1); each active group
	//is analog (its A channel) or digital (its 8 D channels) per m_analogGroups.
	size_t activeGroups = m_channelCount / 8;
	for(size_t g = 0; g < ngroups; g++)
	{
		size_t base = g * 9;	//A<g> at base; D<8g+k> at base+1+k
		bool active = (g < activeGroups);
		bool analog = m_analogGroups.count(g) > 0;
		if(active && analog)
			EnableChannel(base);
		else
			DisableChannel(base);
		for(size_t k = 0; k < 8; k++)
		{
			if(active && !analog)
				EnableChannel(base + 1 + k);
			else
				DisableChannel(base + 1 + k);
		}
	}

	//Build analog + digital banks
	m_analogBanks.clear();
	m_digitalBanks.clear();
	{
		AnalogBank abank;
		DigitalBank dbank;
		for(size_t i = 0; i < m_channels.size(); i++)
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
// Acquisition (in-process finite capture via a libusb async bulk ring + the slogic core)

int64_t SLogicOscilloscope::MonotonicUs()
{
	return chrono::duration_cast<chrono::microseconds>(
		chrono::steady_clock::now().time_since_epoch()).count();
}

void LIBUSB_CALL SLogicOscilloscope::BulkCallback(struct libusb_transfer* xfer)
{
	auto* self = reinterpret_cast<SLogicOscilloscope*>(xfer->user_data);
	if(self)
		self->OnTransfer(xfer);
}

void SLogicOscilloscope::OnTransfer(struct libusb_transfer* xfer)
{
	int64_t now = MonotonicUs();
	int64_t dur = now - m_lastXferUs;
	m_lastXferUs = now;

	bool leaveFlight = true;

	if( (xfer->status == LIBUSB_TRANSFER_COMPLETED) ||
		(xfer->status == LIBUSB_TRANSFER_TIMED_OUT) )
	{
		size_t rawLen = (size_t)xfer->actual_length;

		//Drop the leading 4 bytes of the stream (hardware quirk) across transfers.
		//slogic_apply_first_drop shifts the buffer and returns the kept length.
		size_t kept = slogic_apply_first_drop(&m_stream, xfer->buffer, rawLen);

		{
			lock_guard<mutex> lk(m_captureMutex);
			if(kept && (m_captureBuf.size() < m_captureNeed))
			{
				size_t room = m_captureNeed - m_captureBuf.size();
				size_t take = min(room, kept);
				m_captureBuf.insert(m_captureBuf.end(), xfer->buffer, xfer->buffer + take);
			}
			if(m_captureBuf.size() >= m_captureNeed)
				m_captureDone = true;
		}

		//Fold the RAW received length into the stall watchdog (all-logic canonical policy)
		slogic_verdict v = slogic_stream_watch(&m_stream, rawLen, now, dur);
		if(v == SLOGIC_STREAM_ABORT)
			m_captureAbort = true;
		else if(v == SLOGIC_STREAM_RETRY_RUN)
			m_restartPending = true;

		//Resubmit unless finished / aborting / re-arming / stopped
		if(!m_captureDone && !m_captureAbort && !m_restartPending && m_triggerArmed)
		{
			xfer->actual_length = 0;
			if(libusb_submit_transfer(xfer) == 0)
				leaveFlight = false;
		}
	}
	else if(xfer->status == LIBUSB_TRANSFER_CANCELLED)
	{
		//Expected while the ring drains for a stop or re-arm.
	}
	else
		m_captureAbort = true;

	if(leaveFlight)
		m_inFlight--;
}

bool SLogicOscilloscope::AcquireData()
{
	if(!m_devh || !m_model)
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
		if(!m_digitalThresholds.empty())
			vth = m_digitalThresholds.begin()->second;
	}

	int unit = chanCount / 8;
	if(unit < 1)
		unit = 1;

	m_captureNeed = mdepth * (uint64_t)unit;
	{
		lock_guard<mutex> lk(m_captureMutex);
		m_captureBuf.clear();
		m_captureBuf.reserve(m_captureNeed);
	}
	m_captureDone = false;
	m_captureAbort = false;
	m_restartPending = false;
	m_inFlight = 0;

	//Build capture config from the snapshot (threshold: first digital channel's, else 1.6 V)
	m_captureCfg.channel_count = chanCount;
	m_captureCfg.samplerate_hz = srate;
	m_captureCfg.threshold_v = vth;
	m_captureCfg.pattern_mode = pattern;

	LogDebug("SLogic: AcquireData begin (ch=%d, rate=%llu Hz, depth=%llu, pattern=%d)\n",
		chanCount, (unsigned long long)srate, (unsigned long long)mdepth, pattern);

	if(slogic_plan_transfers(&m_captureCfg, SLOGIC_SIZING_ALLLOGIC, &m_plan) != SLOGIC_OK)
	{
		LogWarning("SLogic: slogic_plan_transfers failed\n");
		return false;
	}
	slogic_stream_init(&m_stream, &m_plan, m_captureNeed);
	m_lastXferUs = MonotonicUs();

	//Device (re)start sequence (control path on the session thread only — never from the bulk
	//callback: U3 returns BUSY). Order: reset -> clear the bulk IN endpoint's halt/data-toggle
	//-> configure -> arm the ring -> run.
	//
	//The clear_halt is the crucial recovery step. When a capture is aborted mid-stream (e.g. a
	//trigger re-arm), the device's endpoint data toggle resets on the next CTRL_RST but the
	//HOST's toggle does not — a vendor reset cannot touch it. The two then mismatch and the bulk
	//IN endpoint returns 0 bytes on every URB forever, even though reset/configure/run all report
	//success (exactly the "control OK but got=0/N, stream stalled" failure). libusb_clear_halt
	//resets the host-side toggle so it resyncs with the freshly-reset device.
	bool startOk = true;
	if(slogic_reset(m_model, &m_transport) != SLOGIC_OK)
	{
		LogWarning("SLogic: slogic_reset failed\n");
		startOk = false;
	}
	if(startOk)
	{
		int che = libusb_clear_halt(m_devh, m_model->ep_in);
		if(che != 0)
			LogDebug("SLogic: libusb_clear_halt(ep 0x%02x) = %d (%s)\n",
				m_model->ep_in, che, libusb_error_name(che));
	}
	if(startOk && (slogic_configure(m_model, &m_transport, &m_captureCfg) != SLOGIC_OK))
	{
		LogWarning("SLogic: slogic_configure failed\n");
		startOk = false;
	}

	//Arm the bulk ring (URBs wait for data), then RUN — URBs are live before the stream starts.
	int ring = m_plan.ring_count;
	if(ring < 1) ring = 1;
	if(ring > SLOGIC_MAX_TRANSFERS) ring = SLOGIC_MAX_TRANSFERS;
	m_ringBufs.assign(ring, vector<uint8_t>(m_plan.size_bytes));
	m_ring.clear();
	if(startOk)
	{
		for(int i=0; i<ring; i++)
		{
			auto* x = libusb_alloc_transfer(0);
			libusb_fill_bulk_transfer(x, m_devh, m_model->ep_in,
				m_ringBufs[i].data(), (int)m_plan.size_bytes,
				&SLogicOscilloscope::BulkCallback, this, m_plan.timeout_ms);
			m_ring.push_back(x);
			if(libusb_submit_transfer(x) == 0)
				m_inFlight++;
		}
	}

	if(startOk && (slogic_run(m_model, &m_transport, &m_captureCfg) != SLOGIC_OK))
	{
		LogWarning("SLogic: slogic_run failed\n");
		startOk = false;
	}
	if(!startOk)
		m_captureAbort = true;

	//Wait for the ring (driven by the event thread) to fill, honoring Stop() and a guard timeout
	int64_t startUs = MonotonicUs();
	int64_t guardUs = 5000000;	//5 s floor
	if(srate > 0)
	{
		int64_t expUs = (int64_t)((double)mdepth / (double)srate * 1e6);
		guardUs = max<int64_t>(guardUs, expUs * 4 + 2000000);
	}
	while(!m_captureDone && !m_captureAbort && m_triggerArmed)
	{
		if(m_restartPending)
		{
			//Stall re-arm on this thread: stop -> cancel + wait for the ring to fully drain
			//-> reset -> clear the endpoint halt/toggle -> configure -> resubmit -> run.
			//The full reset plus clear_halt is what makes the U3 stream restart reliably (a
			//bare RUN, or reset without clearing the host toggle, leaves the endpoint dead).
			LogWarning("SLogic: stream stalled, re-arming capture\n");
			slogic_stop(m_model, &m_transport);
			for(auto* x : m_ring)
				libusb_cancel_transfer(x);
			int64_t d0 = MonotonicUs();
			while((m_inFlight > 0) && ((MonotonicUs()-d0) < 500000))
				this_thread::sleep_for(chrono::milliseconds(1));
			m_restartPending = false;
			m_inFlight = 0;

			bool rearmOk = (slogic_reset(m_model, &m_transport) == SLOGIC_OK);
			if(rearmOk)
				libusb_clear_halt(m_devh, m_model->ep_in);
			if(rearmOk && (slogic_configure(m_model, &m_transport, &m_captureCfg) != SLOGIC_OK))
				rearmOk = false;
			if(rearmOk)
			{
				for(int i=0; i<ring; i++)
				{
					m_ring[i]->actual_length = 0;
					if(libusb_submit_transfer(m_ring[i]) == 0)
						m_inFlight++;
				}
				if(slogic_run(m_model, &m_transport, &m_captureCfg) != SLOGIC_OK)
					rearmOk = false;
			}
			if(!rearmOk)
				m_captureAbort = true;
		}
		this_thread::sleep_for(chrono::milliseconds(1));
		if((MonotonicUs() - startUs) > guardUs)
		{
			LogWarning("SLogic: capture guard timeout (%lld ms, got %zu/%llu bytes) — aborting\n",
				(long long)(guardUs / 1000), m_captureBuf.size(), (unsigned long long)m_captureNeed);
			m_captureAbort = true;
		}
	}

	//Log why the wait ended (done / stopped-by-UI / aborted) so a freeze shows its last state.
	LogDebug("SLogic: capture wait ended (done=%d, abort=%d, armed=%d, got=%zu/%llu)\n",
		(int)m_captureDone, (int)m_captureAbort, (int)m_triggerArmed,
		m_captureBuf.size(), (unsigned long long)m_captureNeed);

	//Stop the device, then cancel + free the ring. As in libsigrok slogic_dev_stop:
	//U3 issues CTRL=STOP; Combo 8 has no reliable stop, so draining the EP is its stop.
	if(m_model->proto == SLOGIC_PROTO_COMBO8)
		DrainEndpoint();
	else
		slogic_stop(m_model, &m_transport);
	for(auto* x : m_ring)
		libusb_cancel_transfer(x);
	int64_t d1 = MonotonicUs();
	while((m_inFlight > 0) && ((MonotonicUs()-d1) < 500000))
		this_thread::sleep_for(chrono::milliseconds(1));
	for(auto* x : m_ring)
		libusb_free_transfer(x);
	m_ring.clear();
	m_ringBufs.clear();

	if(m_captureBuf.size() < (size_t)unit)
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
		//scopehal trigger level is volts; reshape maps byteval/255*range → volts, so raw = level/range*255
		float range = m_channelVoltageRange.count(idx) ? m_channelVoltageRange[idx] : 5;
		float lv = edge->GetLevel();
		int rawi = (range > 0) ? (int)((lv / range) * 255.0f + 0.5f) : 128;
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

	int unit = m_channelCount / 8;
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

	//Active layout: byte g of each sample → group g (analog A<g> at g*9, else D<8g+k> at g*9+1..8)
	struct Grp { size_t byteOffset; bool analog; size_t firstChannel; };
	vector<Grp> layout;
	for(size_t g=0; g<activeGroups; g++)
	{
		bool analog = m_analogGroups.count(g) > 0;
		size_t base = g*9;
		layout.push_back({ g, analog, analog ? base : base+1 });
	}

	double t = GetTime();
	int64_t fs = (int64_t)((t - floor(t)) * FS_PER_SECOND);

	SequenceSet s;
	size_t totalCh = m_channels.size();
	vector< vector<int64_t> > rle_offsets(totalCh);
	vector< vector<int64_t> > rle_durations(totalCh);
	vector< vector<uint8_t> > rle_samples(totalCh);
	vector<uint8_t> last_val(totalCh, 0);
	vector<int64_t> last_start(totalCh, 0);

	//Create waveforms up front
	for(auto& g : layout)
	{
		if(g.analog)
		{
			auto cap = new UniformAnalogWaveform;
			s[GetOscilloscopeChannel(g.firstChannel)] = cap;
			cap->m_timescale = fs_per_sample;
			cap->m_triggerPhase = 0;
			cap->m_startTimestamp = time(NULL);
			cap->m_startFemtoseconds = fs;
			cap->PrepareForCpuAccess();
			cap->Resize(numSamples);
		}
		else
		{
			for(size_t k=0; k<8; k++)
			{
				auto cap = new SparseDigitalWaveform;
				s[GetOscilloscopeChannel(g.firstChannel + k)] = cap;
				cap->m_timescale = fs_per_sample;
				cap->m_triggerPhase = 0;
				cap->m_startTimestamp = time(NULL);
				cap->m_startFemtoseconds = fs;
			}
		}
	}

	//Fill
	for(uint64_t i=0; i<numSamples; i++)
	{
		size_t base = (size_t)(startSample + i) * unit;
		for(auto& g : layout)
		{
			uint8_t byteval = m_captureBuf[base + g.byteOffset];
			if(g.analog)
			{
				size_t idx = g.firstChannel;
				auto cap = static_cast<UniformAnalogWaveform*>(s[GetOscilloscopeChannel(idx)]);
				float range = m_channelVoltageRange.count(idx) ? m_channelVoltageRange[idx] : 5;
				cap->m_samples[i] = (byteval / 255.0f) * range;
			}
			else
			{
				for(size_t k=0; k<8; k++)
				{
					size_t idx = g.firstChannel + k;
					uint8_t bit = (byteval >> k) & 1;
					if(i == 0)
					{
						last_val[idx] = bit;
						last_start[idx] = 0;
					}
					else if(bit != last_val[idx])
					{
						rle_offsets[idx].push_back(last_start[idx]);
						rle_durations[idx].push_back((int64_t)i - last_start[idx]);
						rle_samples[idx].push_back(last_val[idx]);
						last_val[idx] = bit;
						last_start[idx] = (int64_t)i;
					}
				}
			}
		}
	}

	//Finish: flush analog + last RLE run per digital channel
	for(auto& g : layout)
	{
		if(g.analog)
		{
			auto cap = static_cast<UniformAnalogWaveform*>(s[GetOscilloscopeChannel(g.firstChannel)]);
			cap->MarkSamplesModifiedFromCpu();
		}
		else
		{
			for(size_t k=0; k<8; k++)
			{
				size_t idx = g.firstChannel + k;
				rle_offsets[idx].push_back(last_start[idx]);
				rle_durations[idx].push_back((int64_t)numSamples - last_start[idx]);
				rle_samples[idx].push_back(last_val[idx]);

				auto cap = static_cast<SparseDigitalWaveform*>(s[GetOscilloscopeChannel(idx)]);
				size_t memdepth = rle_offsets[idx].size();
				cap->PrepareForCpuAccess();
				cap->Resize(memdepth);
				memcpy(cap->m_offsets.GetCpuPointer(), rle_offsets[idx].data(), memdepth*sizeof(int64_t));
				memcpy(cap->m_durations.GetCpuPointer(), rle_durations[idx].data(), memdepth*sizeof(int64_t));
				for(size_t j=0; j<memdepth; j++)
					cap->m_samples[j] = rle_samples[idx][j] ? true : false;
				cap->MarkSamplesModifiedFromCpu();
				cap->MarkTimestampsModifiedFromCpu();
			}
		}
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
	uint64_t ceil = slogic_max_rate(m_model, m_channelCount);
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

float SLogicOscilloscope::GetDigitalThreshold(size_t channel)
{
	lock_guard<recursive_mutex> lock(m_mutex);
	auto it = m_digitalThresholds.find(channel);
	return (it != m_digitalThresholds.end()) ? it->second : 1.6;
}

void SLogicOscilloscope::SetDigitalThreshold(size_t channel, float level)
{
	lock_guard<recursive_mutex> lock(m_mutex);
	m_digitalThresholds[channel] = level;
	LogDebug("SLogic: SetDigitalThreshold(ch=%zu, %.3f V)\n", channel, level);
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
