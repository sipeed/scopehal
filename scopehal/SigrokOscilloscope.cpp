/***********************************************************************************************************************
*                                                                                                                      *
* libscopehal                                                                                                          *
*                                                                                                                      *
* Copyright (c) 2012-2025 Andrew D. Zonenberg and contributors                                                         *
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
	@brief Implementation of SigrokOscilloscope
	@ingroup scopedrivers
 */

#include "scopehal.h"
#include "SigrokOscilloscope.h"
#include "EdgeTrigger.h"

using namespace std;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Construction / destruction

SigrokOscilloscope::SigrokOscilloscope(SCPITransport* transport)
	: SCPIDevice(transport)
	, SCPIInstrument(transport)
	, RemoteBridgeOscilloscope(transport, true)
	, m_analogChannelCount(0)
	, m_digitalChannelBase(0)
	, m_digitalChannelCount(0)
	, m_digitalThreshold(1.6)
	, m_diag_hardwareWFMHz(FilterParameter::TYPE_FLOAT, Unit(Unit::UNIT_HZ))
	, m_diag_receivedWFMHz(FilterParameter::TYPE_FLOAT, Unit(Unit::UNIT_HZ))
	, m_diag_totalWFMs(FilterParameter::TYPE_INT, Unit(Unit::UNIT_COUNTS))
	, m_diag_droppedWFMs(FilterParameter::TYPE_INT, Unit(Unit::UNIT_COUNTS))
	, m_diag_droppedPercent(FilterParameter::TYPE_FLOAT, Unit(Unit::UNIT_PERCENT))
{
	//Verify we have a twinlan transport
	{
		auto csock = dynamic_cast<SCPITwinLanTransport*>(m_transport);
		if(!csock)
			LogFatal("SigrokOscilloscope expects a SCPITwinLanTransport\n");
	}

	//Query hardware to determine channel configuration
	IdentifyHardware();

	AddDiagnosticLog("Found Model: " + m_model);

	//Query available rates/depths and set the first available as the initial
	//value. Using a hardcoded default (like 1MHz) that isn't in the device's
	//supported list causes GetSampleRate() != GetTimebaseInfo().GetRate()
	//every frame, triggering infinite RATES?/DEPTHS? re-queries.
	{
		auto rates = GetSampleRatesNonInterleaved();
		auto depths = GetSampleDepthsNonInterleaved();
		if(!rates.empty())
			SetSampleRate(rates[0]);
		if(!depths.empty())
			SetSampleDepth(depths[0]);
	}

	//Add analog channel objects (only present when bridge is in analog ADC mode)
	for(size_t i = 0; i < m_analogChannelCount; i++)
	{
		string chname = "A" + to_string(i);

		auto chan = new OscilloscopeChannel(
			this,
			chname,
			GetChannelColor(i),
			Unit(Unit::UNIT_FS),
			Unit(Unit::UNIT_VOLTS),
			Stream::STREAM_TYPE_ANALOG,
			i);
		m_channels.push_back(chan);

		m_channelAttenuations[i] = 10;
		SetChannelCoupling(i, OscilloscopeChannel::COUPLE_AC_1M);
		SetChannelOffset(i, 0, 0);
		SetChannelVoltageRange(i, 0, 5);
		EnableChannel(i);
	}

	//Add digital channel objects (only present when bridge is in digital ADC mode)
	for(size_t i = 0; i < m_digitalChannelCount; i++)
	{
		size_t chnum = m_digitalChannelBase + i;
		string chname = "D" + to_string(i);

		auto chan = new OscilloscopeChannel(
			this,
			chname,
			GetChannelColor(i),
			Unit(Unit::UNIT_FS),
			Unit(Unit::UNIT_COUNTS),
			Stream::STREAM_TYPE_DIGITAL,
			chnum);
		m_channels.push_back(chan);

		m_channelAttenuations[chnum] = 1;
		SetDigitalHysteresis(chnum, 0.1);
		SetDigitalThreshold(chnum, 1.6);
		EnableChannel(chnum);
	}

	//Set up analog and digital banks
	if(m_analogChannelCount > 0)
	{
		AnalogBank bank;
		for(size_t i = 0; i < m_analogChannelCount; i++)
			bank.push_back(GetOscilloscopeChannel(i));
		m_analogBanks.push_back(bank);
	}
	if(m_digitalChannelCount > 0)
	{
		DigitalBank bank;
		for(size_t i = 0; i < m_digitalChannelCount; i++)
			bank.push_back(GetOscilloscopeChannel(m_digitalChannelBase + i));
		m_digitalBanks.push_back(bank);
	}

	//Configure the trigger on the first channel.
	//In analog mode, default to mid-scale (attenuation/2) so the software
	//trigger actually detects crossings. Level 0 maps to threshold_raw=0
	//which makes every sample "above" and no rising edge is ever found.
	auto trig = new EdgeTrigger(this);
	trig->SetType(EdgeTrigger::EDGE_RISING);
	if(m_analogChannelCount > 0 && m_digitalChannelCount == 0)
		trig->SetLevel(m_channelAttenuations[0] / 2.0);
	else
		trig->SetLevel(0);
	trig->SetInput(0, StreamDescriptor(GetOscilloscopeChannel(0)));
	SetTrigger(trig);
	SetTriggerOffset(1000000000000); //1ms to allow trigphase interpolation

	//Set up diagnostics
	m_diagnosticValues["Hardware WFM/s"] = &m_diag_hardwareWFMHz;
	m_diagnosticValues["Received WFM/s"] = &m_diag_receivedWFMHz;
	m_diagnosticValues["Total Waveforms Received"] = &m_diag_totalWFMs;
	m_diagnosticValues["Received Waveforms Dropped"] = &m_diag_droppedWFMs;
	m_diagnosticValues["% Received Waveforms Dropped"] = &m_diag_droppedPercent;

	ResetPerCaptureDiagnostics();
}

SigrokOscilloscope::~SigrokOscilloscope()
{
}

void SigrokOscilloscope::ResetPerCaptureDiagnostics()
{
	m_diag_hardwareWFMHz.SetFloatVal(0);
	m_diag_receivedWFMHz.SetFloatVal(0);
	m_diag_totalWFMs.SetIntVal(0);
	m_diag_droppedWFMs.SetIntVal(0);
	m_diag_droppedPercent.SetFloatVal(1);
	m_receiveClock.Reset();
}

string SigrokOscilloscope::GetChannelColor(size_t i)
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

void SigrokOscilloscope::IdentifyHardware()
{
	m_analogChannelCount = 0;
	m_digitalChannelBase = 0;
	m_digitalChannelCount = 0;

	LogDebug("ID Model \"%s\"\n", m_model.c_str());

	{
		lock_guard<recursive_mutex> lock(m_mutex);
		auto chans = m_transport->SendCommandQueuedWithReply("CHANS?");

		vector<uint16_t> ret;
		stringstream ss(chans);
		string token;

		while(getline(ss, token, ','))
		{
			if(!token.empty())
				ret.push_back(static_cast<uint16_t>(stoul(token)));
		}

		if(ret.size() != 2)
		{
			LogFatal("Unexpected CHANS? reply: %s\n", chans.c_str());
		}
		else
		{
			m_analogChannelCount = ret[0];
			m_digitalChannelCount = ret[1];
			m_digitalChannelBase = m_analogChannelCount;
		}
	}

	LogDebug("Detected %zu analog channels and %zu digital channels\n",
		m_analogChannelCount, m_digitalChannelCount);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Accessors

unsigned int SigrokOscilloscope::GetInstrumentTypes() const
{
	return Instrument::INST_OSCILLOSCOPE;
}

uint32_t SigrokOscilloscope::GetInstrumentTypesForChannel(size_t /*i*/) const
{
	return Instrument::INST_OSCILLOSCOPE;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Device interface functions

string SigrokOscilloscope::GetDriverNameInternal()
{
	return "sigrok";
}

void SigrokOscilloscope::FlushConfigCache()
{
	lock_guard<recursive_mutex> lock(m_cacheMutex);
}

double SigrokOscilloscope::GetChannelAttenuation(size_t i)
{
	lock_guard<recursive_mutex> lock(m_cacheMutex);
	return m_channelAttenuations[i];
}

void SigrokOscilloscope::SetChannelAttenuation(size_t i, double atten)
{
	lock_guard<recursive_mutex> lock(m_cacheMutex);
	double oldAtten = m_channelAttenuations[i];
	m_channelAttenuations[i] = atten;

	//Rescale channel voltage range and offset
	double delta = atten / oldAtten;
	m_channelVoltageRanges[i] *= delta;
	m_channelOffsets[i] *= delta;
}

unsigned int SigrokOscilloscope::GetChannelBandwidthLimit(size_t /*i*/)
{
	return 0;
}

void SigrokOscilloscope::SetChannelBandwidthLimit(size_t /*i*/, unsigned int /*limit_mhz*/)
{
}

OscilloscopeChannel* SigrokOscilloscope::GetExternalTrigger()
{
	return NULL;
}

Oscilloscope::TriggerMode SigrokOscilloscope::PollTrigger()
{
	//Always report "triggered" so we can block on AcquireData() in ScopeThread
	return TRIGGER_MODE_TRIGGERED;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Data acquisition

bool SigrokOscilloscope::AcquireData()
{
	//Flush pending START/SINGLE and configuration commands, then wait for a reply
	//on the command socket. This is the ordering barrier before requesting data
	//on the independent waveform socket.
	auto armed = Trim(m_transport->SendCommandQueuedWithReply("ARMED?"));
	if(armed != "1")
	{
		LogTrace("Bridge is not armed, skipping waveform request\n");
		return false;
	}

	//Signal to the bridge that we're ready for the next waveform
	const uint8_t r = 'K';
	m_transport->SendRawData(1, &r);

	//Read the unified waveform header
	uint32_t seqnum;
	uint16_t numChannels;
	uint64_t numSamples;
	int64_t fs_per_sample;
	int64_t trigger_fs;

	if(!m_transport->ReadRawData(sizeof(seqnum), (uint8_t*)&seqnum)) return false;
	if(!m_transport->ReadRawData(sizeof(numChannels), (uint8_t*)&numChannels)) return false;
	if(!m_transport->ReadRawData(sizeof(numSamples), (uint8_t*)&numSamples)) return false;
	if(!m_transport->ReadRawData(sizeof(fs_per_sample), (uint8_t*)&fs_per_sample)) return false;
	if(!m_transport->ReadRawData(sizeof(trigger_fs), (uint8_t*)&trigger_fs)) return false;

	double wfms_s;
	if(!m_transport->ReadRawData(sizeof(wfms_s), (uint8_t*)&wfms_s)) return false;

	//A zero-sample response means "no waveform this time" (trigger hasn't
	//fired yet, or acquisition was interrupted). Keep the previous waveform,
	//don't touch trigger offset, and don't disarm a one-shot arm — the next
	//polling loop will retry.
	if(numSamples == 0)
	{
		LogTrace("Bridge reported empty frame (SEQ#%u), trigger not fired yet\n", seqnum);
		return false;
	}

	{
		lock_guard<recursive_mutex> lock(m_mutex);
		if(m_triggerOffset != trigger_fs)
		{
			AddDiagnosticLog("Correcting trigger offset by " + to_string(m_triggerOffset - trigger_fs));
			m_triggerOffset = trigger_fs;
		}
	}

	m_diag_hardwareWFMHz.SetFloatVal(wfms_s);

	LogDebug("Receive header: SEQ#%u, %uch@%llu samples\n", seqnum, numChannels,
		static_cast<unsigned long long>(numSamples));

	//Prepare chunked read buffer
	vector<uint8_t> raw_buffer;
	uint16_t unit_size = (numChannels + 7) / 8;
	const size_t CHUNK_SAMPLES = 256 * 1024;
	raw_buffer.resize(CHUNK_SAMPLES * unit_size);

	SequenceSet s;
	double t = GetTime();
	int64_t fs = (t - floor(t)) * FS_PER_SECOND;

	if(m_analogChannelCount > 0 && m_digitalChannelCount == 0)
	{
		//Analog mode: reinterpret packed digital data as 8-bit ADC values
		uint32_t numAnalogChannels = numChannels / 8;
		for(uint32_t chnum = 0; chnum < numAnalogChannels; chnum++)
		{
			auto cap = new UniformAnalogWaveform;
			s[GetOscilloscopeChannel(chnum)] = cap;
			cap->m_timescale = fs_per_sample;
			cap->m_triggerPhase = 0;
			cap->m_startTimestamp = time(NULL);
			cap->m_startFemtoseconds = fs;
			cap->PrepareForCpuAccess();
			cap->Resize(numSamples);
		}

		for(uint64_t numSamples_recved = 0; numSamples_recved < numSamples;)
		{
			uint64_t samples_to_read = min((uint64_t)CHUNK_SAMPLES, numSamples - numSamples_recved);
			uint64_t bytes_to_read = samples_to_read * unit_size;

			if(!m_transport->ReadRawData(bytes_to_read, raw_buffer.data()))
			{
				LogWarning("Incomplete data read, expected %llu samples, got %llu samples\n",
					static_cast<unsigned long long>(numSamples),
					static_cast<unsigned long long>(numSamples_recved));
				numSamples = numSamples_recved;
				break;
			}

			//Process this chunk: extract per-channel byte values
			for(size_t i = 0; i < samples_to_read; i++)
			{
				for(uint32_t chnum = 0; chnum < numAnalogChannels; chnum++)
				{
					uint8_t raw_val = raw_buffer[i * unit_size + chnum];
					auto cap = static_cast<UniformAnalogWaveform*>(s[GetOscilloscopeChannel(chnum)]);

					float range = m_channelAttenuations[chnum];
					float offset = m_channelOffsets[chnum];
					cap->m_samples[numSamples_recved + i] = (raw_val / 255.0f) * range + offset;
				}
			}

			numSamples_recved += samples_to_read;
		}

		for(uint32_t chnum = 0; chnum < numAnalogChannels; chnum++)
		{
			auto cap = static_cast<UniformAnalogWaveform*>(s[GetOscilloscopeChannel(chnum)]);
			cap->MarkSamplesModifiedFromCpu();
		}
	}
	else
	{
		//Digital mode: extract per-channel bits and RLE encode

		//Temporary RLE buffers
		vector< vector<int64_t> > rle_offsets(numChannels);
		vector< vector<int64_t> > rle_durations(numChannels);
		vector< vector<uint8_t> > rle_samples(numChannels);

		//Tracking state for RLE deduplication
		vector<uint8_t> last_val(numChannels);
		vector<int64_t> last_start(numChannels, 0);
		bool first = true;

		for(uint32_t chnum = 0; chnum < numChannels; chnum++)
		{
			auto cap = new SparseDigitalWaveform;
			s[GetOscilloscopeChannel(m_digitalChannelBase + chnum)] = cap;
			cap->m_timescale = fs_per_sample;
			cap->m_triggerPhase = 0;
			cap->m_startTimestamp = time(NULL);
			cap->m_startFemtoseconds = fs;
		}

		for(uint64_t numSamples_recved = 0; numSamples_recved < numSamples;)
		{
			uint64_t samples_to_read = min((uint64_t)CHUNK_SAMPLES, numSamples - numSamples_recved);
			uint64_t bytes_to_read = samples_to_read * unit_size;

			if(!m_transport->ReadRawData(bytes_to_read, raw_buffer.data()))
			{
				LogWarning("Incomplete data read, expected %llu samples, got %llu samples\n",
					static_cast<unsigned long long>(numSamples),
					static_cast<unsigned long long>(numSamples_recved));
				numSamples = numSamples_recved;
				break;
			}

			//Process this chunk: extract individual channel bits
			uint8_t* p = raw_buffer.data();
			uint64_t sample_val = 0;

			for(size_t i = 0; i < samples_to_read; i++)
			{
				memcpy(&sample_val, p + i * unit_size, unit_size);
				int64_t global_index = numSamples_recved + i;

				for(uint32_t chnum = 0; chnum < numChannels; chnum++)
				{
					uint8_t bit = (sample_val >> chnum) & 1;

					if(first && i == 0)
					{
						last_val[chnum] = bit;
						last_start[chnum] = global_index;
					}
					else if(bit != last_val[chnum])
					{
						rle_offsets[chnum].push_back(last_start[chnum]);
						rle_durations[chnum].push_back(global_index - last_start[chnum]);
						rle_samples[chnum].push_back(last_val[chnum]);

						last_val[chnum] = bit;
						last_start[chnum] = global_index;
					}
				}
			}

			first = false;
			numSamples_recved += samples_to_read;
			LogDebug("Logic data: %llu/%llu samples\n",
				static_cast<unsigned long long>(numSamples_recved),
				static_cast<unsigned long long>(numSamples));
		}

		//Finish RLE coding: flush the last run for each channel
		for(uint32_t chnum = 0; chnum < numChannels; chnum++)
		{
			rle_offsets[chnum].push_back(last_start[chnum]);
			rle_durations[chnum].push_back(numSamples - last_start[chnum]);
			rle_samples[chnum].push_back(last_val[chnum]);

			auto cap = static_cast<SparseDigitalWaveform*>(s[GetOscilloscopeChannel(m_digitalChannelBase + chnum)]);
			size_t memdepth = rle_offsets[chnum].size();

			LogDebug("ch%u: Convert to %zu RLE entries\n", chnum, memdepth);

			cap->PrepareForCpuAccess();
			cap->Resize(memdepth);

			memcpy(cap->m_offsets.GetCpuPointer(), rle_offsets[chnum].data(), memdepth * sizeof(int64_t));
			memcpy(cap->m_durations.GetCpuPointer(), rle_durations[chnum].data(), memdepth * sizeof(int64_t));

			for(size_t j = 0; j < memdepth; j++)
				cap->m_samples[j] = rle_samples[chnum][j] ? true : false;

			cap->MarkSamplesModifiedFromCpu();
			cap->MarkTimestampsModifiedFromCpu();
		}
	}

	//Update diagnostics
	FilterParameter* param = &m_diag_totalWFMs;
	int total = param->GetIntVal() + 1;
	param->SetIntVal(total);

	param = &m_diag_droppedWFMs;
	int dropped = param->GetIntVal();

	//Save the waveforms to our queue
	m_pendingWaveformsMutex.lock();
	m_pendingWaveforms.push_back(s);

	while(m_pendingWaveforms.size() > 2)
	{
		SequenceSet set = *m_pendingWaveforms.begin();
		for(auto it : set)
			delete it.second;
		m_pendingWaveforms.pop_front();

		dropped++;
	}

	m_pendingWaveformsMutex.unlock();

	param->SetIntVal(dropped);

	param = &m_diag_droppedPercent;
	param->SetFloatVal((float)dropped / (float)total);

	m_receiveClock.Tick();
	m_diag_receivedWFMHz.SetFloatVal(m_receiveClock.GetAverageHz());

	//If this was a one-shot trigger we're no longer armed
	if(m_triggerOneShot)
		m_triggerArmed = false;

	return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Capture control

void SigrokOscilloscope::Start()
{
	RemoteBridgeOscilloscope::Start();
	ResetPerCaptureDiagnostics();
}

void SigrokOscilloscope::StartSingleTrigger()
{
	RemoteBridgeOscilloscope::StartSingleTrigger();
	ResetPerCaptureDiagnostics();
}

void SigrokOscilloscope::ForceTrigger()
{
	RemoteBridgeOscilloscope::ForceTrigger();
	ResetPerCaptureDiagnostics();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Timebase

vector<uint64_t> SigrokOscilloscope::GetSampleRatesNonInterleaved()
{
	vector<uint64_t> ret;

	string rates;
	{
		lock_guard<recursive_mutex> lock(m_mutex);
		rates = m_transport->SendCommandQueuedWithReply("RATES?");
	}

	stringstream ss(rates);
	string token;

	while(getline(ss, token, ','))
	{
		if(!token.empty())
			ret.push_back(static_cast<uint64_t>(stoull(token)));
	}

	return ret;
}

vector<uint64_t> SigrokOscilloscope::GetSampleRatesInterleaved()
{
	return {};
}

set<Oscilloscope::InterleaveConflict> SigrokOscilloscope::GetInterleaveConflicts()
{
	return {};
}

vector<uint64_t> SigrokOscilloscope::GetSampleDepthsNonInterleaved()
{
	vector<uint64_t> ret;

	string depths;
	{
		lock_guard<recursive_mutex> lock(m_mutex);
		depths = m_transport->SendCommandQueuedWithReply("DEPTHS?");
	}

	stringstream ss(depths);
	string token;

	while(getline(ss, token, ','))
	{
		if(!token.empty())
			ret.push_back(static_cast<uint64_t>(stoull(token)));
	}

	return ret;
}

vector<uint64_t> SigrokOscilloscope::GetSampleDepthsInterleaved()
{
	return {};
}

bool SigrokOscilloscope::IsInterleaving()
{
	return false;
}

bool SigrokOscilloscope::SetInterleaving(bool /*combine*/)
{
	return false;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Channel configuration

vector<OscilloscopeChannel::CouplingType> SigrokOscilloscope::GetAvailableCouplings(size_t /*i*/)
{
	vector<OscilloscopeChannel::CouplingType> ret;
	ret.push_back(OscilloscopeChannel::COUPLE_DC_1M);
	ret.push_back(OscilloscopeChannel::COUPLE_AC_1M);
	return ret;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// ADC configuration

vector<Oscilloscope::AnalogBank> SigrokOscilloscope::GetAnalogBanks()
{
	return m_analogBanks;
}

Oscilloscope::AnalogBank SigrokOscilloscope::GetAnalogBank(size_t i)
{
	for(auto b : m_analogBanks)
		if(std::find(b.begin(), b.end(), GetOscilloscopeChannel(i)) != b.end())
			return b;

	return {};
}

bool SigrokOscilloscope::IsADCModeConfigurable()
{
	return false;
}

vector<string> SigrokOscilloscope::GetADCModeNames(size_t /*channel*/)
{
	//ADC mode is fixed at bridge startup — not switchable from the client.
	//Return a single entry so the UI shows current mode without a dropdown.
	if(m_analogChannelCount > 0 && m_digitalChannelCount == 0)
		return {"8-bit Analog"};
	return {"Digital"};
}

size_t SigrokOscilloscope::GetADCMode(size_t /*channel*/)
{
	return (m_analogChannelCount > 0 && m_digitalChannelCount == 0) ? 1 : 0;
}

void SigrokOscilloscope::SetADCMode(size_t /*channel*/, size_t /*mode*/)
{
	//No-op: ADC mode is determined by bridge --adc-mode flag at startup.
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Logic analyzer configuration

vector<Oscilloscope::DigitalBank> SigrokOscilloscope::GetDigitalBanks()
{
	return m_digitalBanks;
}

Oscilloscope::DigitalBank SigrokOscilloscope::GetDigitalBank(size_t i)
{
	for(auto b : m_digitalBanks)
		if(std::find(b.begin(), b.end(), GetOscilloscopeChannel(i)) != b.end())
			return b;

	return {};
}

bool SigrokOscilloscope::IsDigitalHysteresisConfigurable()
{
	return false;
}

bool SigrokOscilloscope::IsDigitalThresholdConfigurable()
{
	return true;
}

float SigrokOscilloscope::GetDigitalHysteresis(size_t /*channel*/)
{
	return 0;
}

float SigrokOscilloscope::GetDigitalThreshold(size_t /*channel*/)
{
	lock_guard<recursive_mutex> lock(m_cacheMutex);
	return m_digitalThreshold;
}

void SigrokOscilloscope::SetDigitalHysteresis(size_t /*channel*/, float /*level*/)
{
}

void SigrokOscilloscope::SetDigitalThreshold(size_t i, float level)
{
	{
		lock_guard<recursive_mutex> lock(m_cacheMutex);
		if(m_digitalThreshold == level) return;

		m_digitalThreshold = level;
	}

	lock_guard<recursive_mutex> lock(m_mutex);
	m_transport->SendCommandQueued(":" + m_channels[i]->GetHwname() + ":THRESH " + to_string(level));
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Checking for validity of configurations

bool SigrokOscilloscope::CanEnableChannel(size_t /*i*/)
{
	//All channels are valid — mode is fixed at bridge startup, and only the
	//appropriate channel types are created based on CHANS? response.
	return true;
}
