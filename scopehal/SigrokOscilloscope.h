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
	@brief Declaration of SigrokOscilloscope
	@ingroup scopedrivers
 */

#ifndef SigrokOscilloscope_h
#define SigrokOscilloscope_h

#include "RemoteBridgeOscilloscope.h"
#include "../xptools/HzClock.h"

/**
	@brief SigrokOscilloscope - driver for talking to sigrok-bridge-server
	@ingroup scopedrivers
 */
class SigrokOscilloscope : public RemoteBridgeOscilloscope
{
public:
	SigrokOscilloscope(SCPITransport* transport);
	virtual ~SigrokOscilloscope();

	//not copyable or assignable
	SigrokOscilloscope(const SigrokOscilloscope& rhs) =delete;
	SigrokOscilloscope& operator=(const SigrokOscilloscope& rhs) =delete;

	//Device information
	virtual unsigned int GetInstrumentTypes() const override;
	virtual uint32_t GetInstrumentTypesForChannel(size_t i) const override;
	virtual void FlushConfigCache() override;

	//Channel configuration
	virtual std::vector<OscilloscopeChannel::CouplingType> GetAvailableCouplings(size_t i) override;
	virtual double GetChannelAttenuation(size_t i) override;
	virtual void SetChannelAttenuation(size_t i, double atten) override;
	virtual unsigned int GetChannelBandwidthLimit(size_t i) override;
	virtual void SetChannelBandwidthLimit(size_t i, unsigned int limit_mhz) override;
	virtual OscilloscopeChannel* GetExternalTrigger() override;
	virtual bool CanEnableChannel(size_t i) override;

	//Triggering
	virtual Oscilloscope::TriggerMode PollTrigger() override;
	virtual bool AcquireData() override;

	//Captures
	virtual void Start() override;
	virtual void StartSingleTrigger() override;
	virtual void ForceTrigger() override;

	//Timebase
	virtual std::vector<uint64_t> GetSampleRatesNonInterleaved() override;
	virtual std::vector<uint64_t> GetSampleRatesInterleaved() override;
	virtual std::set<InterleaveConflict> GetInterleaveConflicts() override;
	virtual std::vector<uint64_t> GetSampleDepthsNonInterleaved() override;
	virtual std::vector<uint64_t> GetSampleDepthsInterleaved() override;
	virtual bool IsInterleaving() override;
	virtual bool SetInterleaving(bool combine) override;

	//ADC configuration
	virtual std::vector<AnalogBank> GetAnalogBanks() override;
	virtual AnalogBank GetAnalogBank(size_t channel) override;
	virtual bool IsADCModeConfigurable() override;
	virtual std::vector<std::string> GetADCModeNames(size_t channel) override;
	virtual size_t GetADCMode(size_t channel) override;
	virtual void SetADCMode(size_t channel, size_t mode) override;

	//Logic analyzer configuration
	virtual std::vector<DigitalBank> GetDigitalBanks() override;
	virtual DigitalBank GetDigitalBank(size_t channel) override;
	virtual bool IsDigitalHysteresisConfigurable() override;
	virtual bool IsDigitalThresholdConfigurable() override;
	virtual float GetDigitalHysteresis(size_t channel) override;
	virtual float GetDigitalThreshold(size_t channel) override;
	virtual void SetDigitalHysteresis(size_t channel, float level) override;
	virtual void SetDigitalThreshold(size_t channel, float level) override;

protected:
	void IdentifyHardware();
	void ResetPerCaptureDiagnostics();
	std::string GetChannelColor(size_t i);

	size_t m_analogChannelCount;
	size_t m_digitalChannelBase;
	size_t m_digitalChannelCount;

	std::vector<AnalogBank> m_analogBanks;
	std::vector<DigitalBank> m_digitalBanks;
	std::map<size_t, double> m_channelAttenuations;
	float m_digitalThreshold;

	FilterParameter m_diag_hardwareWFMHz;
	FilterParameter m_diag_receivedWFMHz;
	FilterParameter m_diag_totalWFMs;
	FilterParameter m_diag_droppedWFMs;
	FilterParameter m_diag_droppedPercent;
	HzClock m_receiveClock;

public:
	static std::string GetDriverNameInternal();
	OSCILLOSCOPE_INITPROC(SigrokOscilloscope);
};

#endif
