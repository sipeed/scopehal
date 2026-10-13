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
	@brief Declaration of SLogicOscilloscope
	@ingroup scopedrivers
 */

#ifndef SLogicOscilloscope_h
#define SLogicOscilloscope_h

#include "SCPIOscilloscope.h"

#include <set>

//The vendored libslogic core is C. It already provides its own extern "C" guard,
//but we wrap the include defensively so this keeps compiling if that ever changes.
extern "C" {
#include "slogic/slogic.h"
}

#include <libusb-1.0/libusb.h>

#include <atomic>
#include <mutex>
#include <map>
#include <vector>

/**
	@brief Native driver for Sipeed SLogic USB logic analyzers
	@ingroup scopedrivers

	Talks to the device directly over USB via the vendored libslogic core — no
	sigrok-bridge process. Step 2 adds the USB layer (transport shim + device
	open/claim + libusb event thread); channels, config, acquisition and trigger
	come in later steps.
 */
class SLogicOscilloscope : public virtual SCPIOscilloscope
{
public:
	SLogicOscilloscope(SCPITransport* transport);
	virtual ~SLogicOscilloscope();

	//not copyable or assignable
	SLogicOscilloscope(const SLogicOscilloscope& rhs) =delete;
	SLogicOscilloscope& operator=(const SLogicOscilloscope& rhs) =delete;

	virtual std::string IDPing() override;

	virtual std::string GetTransportConnectionString() override;
	virtual std::string GetTransportName() override;
	virtual uint32_t GetInstrumentTypesForChannel(size_t i) const override;

	//Channel configuration
	virtual bool IsChannelEnabled(size_t i) override;
	virtual void EnableChannel(size_t i) override;
	virtual void DisableChannel(size_t i) override;
	virtual OscilloscopeChannel::CouplingType GetChannelCoupling(size_t i) override;
	virtual void SetChannelCoupling(size_t i, OscilloscopeChannel::CouplingType type) override;
	virtual std::vector<OscilloscopeChannel::CouplingType> GetAvailableCouplings(size_t i) override;
	virtual double GetChannelAttenuation(size_t i) override;
	virtual void SetChannelAttenuation(size_t i, double atten) override;
	virtual unsigned int GetChannelBandwidthLimit(size_t i) override;
	virtual void SetChannelBandwidthLimit(size_t i, unsigned int limit_mhz) override;
	virtual float GetChannelVoltageRange(size_t i, size_t stream) override;
	virtual void SetChannelVoltageRange(size_t i, size_t stream, float range) override;
	virtual OscilloscopeChannel* GetExternalTrigger() override;
	virtual float GetChannelOffset(size_t i, size_t stream) override;
	virtual void SetChannelOffset(size_t i, size_t stream, float offset) override;
	virtual bool IsHighRateOffsetCapable(size_t i) override;

	//Triggering
	virtual Oscilloscope::TriggerMode PollTrigger() override;
	virtual bool AcquireData() override;
	virtual void Start() override;
	virtual void StartSingleTrigger() override;
	virtual void Stop() override;
	virtual void ForceTrigger() override;
	virtual bool IsTriggerArmed() override;
	virtual void PushTrigger() override;
	virtual void PullTrigger() override;

	virtual std::vector<uint64_t> GetSampleRatesNonInterleaved() override;
	virtual std::vector<uint64_t> GetSampleRatesInterleaved() override;
	virtual std::set<InterleaveConflict> GetInterleaveConflicts() override;
	virtual std::vector<uint64_t> GetSampleDepthsNonInterleaved() override;
	virtual std::vector<uint64_t> GetSampleDepthsInterleaved() override;
	virtual uint64_t GetSampleRate() override;
	virtual uint64_t GetSampleDepth() override;
	virtual void SetSampleDepth(uint64_t depth) override;
	virtual void SetSampleRate(uint64_t rate) override;
	virtual void SetTriggerOffset(int64_t offset) override;
	virtual int64_t GetTriggerOffset() override;
	virtual bool IsInterleaving() override;
	virtual bool SetInterleaving(bool combine) override;
	virtual bool CanInterleave() override;

	virtual bool IsPatternModeConfigurable() override;
	virtual std::vector<std::string> GetPatternModeNames() override;
	virtual size_t GetPatternMode() override;
	virtual void SetPatternMode(size_t mode) override;
	virtual bool IsCaptureWidthConfigurable() override;
	virtual std::vector<std::string> GetCaptureWidthNames() override;
	virtual size_t GetCaptureWidth() override;
	virtual void SetCaptureWidth(size_t mode) override;
	virtual size_t GetChannelGroupCount() override;
	virtual bool IsChannelGroupAnalog(size_t group) override;
	virtual void SetChannelGroupAnalog(size_t group, bool analog) override;
	virtual std::vector<size_t> GetChannelGroupChannelIndices(size_t group) override;
	virtual std::vector<AnalogBank> GetAnalogBanks() override;
	virtual AnalogBank GetAnalogBank(size_t channel) override;

	virtual unsigned int GetInstrumentTypes() const override;
	virtual void LoadConfiguration(int version, const YAML::Node& node, IDTable& idmap) override;

	virtual std::vector<DigitalBank> GetDigitalBanks() override;

	//Digital threshold / hysteresis
	//The device has a single global comparator threshold (one DAC feeding every digital input),
	//so the threshold is instrument-wide: Set/Get ignore the channel argument and share one value.
	virtual bool IsDigitalThresholdConfigurable() override;
	virtual float GetDigitalHysteresis(size_t channel) override;
	virtual float GetDigitalThreshold(size_t channel) override;
	virtual void SetDigitalHysteresis(size_t channel, float level) override;
	virtual void SetDigitalThreshold(size_t channel, float level) override;

protected:

	///@brief Create the full physical channel set and enable the active subset
	void BuildChannels();
	///@brief Enable/disable the physical channel set from m_channelCount + m_analogGroups (handles a partial last byte-group)
	void ApplyChannelEnables();
	///@brief Rebuild m_analogBanks / m_digitalBanks from the current channels
	void RebuildBanks();
	///@brief Model channel-mode options (channel count + max rate), highest channel count first
	std::vector<slogic_rate_limit> CaptureWidthOptions();
	///@brief Retarget the edge trigger to the first enabled channel if its current channel is disabled
	void RetargetTriggerIfNeeded();
	///@brief Analog channel color by index
	std::string GetChannelColor(size_t i);

	///@brief External trigger
	OscilloscopeChannel* m_extTrigger = nullptr;

	//Channel state (map-backed, DemoOscilloscope style; SCPIOscilloscope base has no channel-state store)
	std::map<size_t, bool> m_channelsEnabled;
	std::map<size_t, OscilloscopeChannel::CouplingType> m_channelCoupling;
	std::map<size_t, double> m_channelAttenuation;
	std::map<size_t, unsigned int> m_channelBandwidth;
	std::map<size_t, float> m_channelVoltageRange;
	std::map<size_t, float> m_channelOffset;
	///@brief Global digital comparator threshold in volts (the device has a single Vth for all D channels)
	float m_digitalThreshold = 1.6f;
	std::map<size_t, float> m_digitalHysteresis;

	///@brief Current sample rate (Hz)
	uint64_t m_srate = 0;
	///@brief Current memory depth (samples)
	uint64_t m_mdepth = 0;
	///@brief Trigger offset (fs)
	int64_t m_triggerOffset = 0;

	///@brief Software-trigger pretrigger sample count (0 = trigger at the left edge, post-trigger only)
	uint64_t m_pretrigger = 0;
	///@brief Cross-buffer trigger state for continuous mode: -1 = unknown, else last-sample state (bit set / above threshold)
	int m_lastTrigState = -1;

	///@brief Analog channel banks
	std::vector<AnalogBank> m_analogBanks;
	///@brief Digital channel banks
	std::vector<DigitalBank> m_digitalBanks;

	///@brief libusb context owned by this driver
	libusb_context* m_usbctx = nullptr;

	///@brief Open device in the shared libslogic core (owns the handle, event thread and ring)
	slogic_dev* m_sdev = nullptr;

	///@brief Model descriptor from the vendored slogic core
	const slogic_model* m_model = nullptr;

	///@brief Number of channels in the current capture width
	int m_channelCount = 0;

	///@brief Set of analog channel group indices
	std::set<size_t> m_analogGroups;

	///@brief Capture pattern (SLOGIC_PATTERN_*), surfaced via the Pattern selector
	int m_patternMode = SLOGIC_PATTERN_NORMAL;

	//Acquisition: one finite capture per AcquireData() through the libslogic capture
	//engine; the engine delivers exact, sample-major samples (1 byte/sample below 8 ch).
	///@brief slogic config of the last capture (ReshapeAndQueue reads its channel count)
	slogic_config m_captureCfg;
	///@brief Captured samples, sample-major, unitsize max(1, ch/8) bytes each
	std::vector<uint8_t> m_captureBuf;
	///@brief Set by Stop() to end an in-progress capture
	std::atomic<bool> m_captureAbort{false};

	///@brief Engine sink: append delivered samples to m_captureBuf
	static int OnSamples(void* user, const uint8_t* samples, size_t n, unsigned unitsize);
	///@brief Engine log -> scopehal log
	static void OnLog(void* user, int level, const char* msg);
	///@brief Reshape m_captureBuf (sample-major) into scopehal waveforms and queue them
	void ReshapeAndQueue();
	///@brief Scan m_captureBuf for the active EdgeTrigger's edge; returns trigger sample index or -1 (none)
	int64_t FindTriggerSample(int unit);
	///@brief Monotonic clock in microseconds
	static int64_t MonotonicUs();

	///@brief Find + open the first SLogic (optionally by serial) through the core
	bool OpenDevice(const std::string& serial = "");
	///@brief Close the device (destroys a live capture) and the libusb context
	void CloseDevice();

public:
	static std::string GetDriverNameInternal();

	//This is intentionally not virtual since it's a static method used by enumeration
	//cppcheck-suppress duplInheritedMember
	static std::vector<SCPIInstrumentModel> GetDriverSupportedModels()
	{
		return
		{
			{"SLogic", {{ SCPITransportType::TRANSPORT_NULL, "" }}}
		};
	}
	OSCILLOSCOPE_INITPROC(SLogicOscilloscope)
};

#endif
