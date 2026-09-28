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

#include <thread>
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
	std::map<size_t, float> m_digitalThresholds;
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

	///@brief libusb context (unused in the scaffold; owned once the native path lands)
	libusb_context* m_usbctx = nullptr;

	///@brief libusb device handle (unused in the scaffold)
	libusb_device_handle* m_devh = nullptr;

	///@brief Model descriptor from the vendored slogic core (unused in the scaffold)
	const slogic_model* m_model = nullptr;

	///@brief Number of channels on the connected device (unused in the scaffold)
	int m_channelCount = 0;

	///@brief Set of analog channel group indices (unused in the scaffold)
	std::set<size_t> m_analogGroups;

	///@brief Capture pattern (SLOGIC_PATTERN_*), surfaced via the Pattern selector
	int m_patternMode = SLOGIC_PATTERN_NORMAL;

	///@brief slogic core transport (control-transfer callbacks bound to libusb)
	slogic_transport m_transport;

	///@brief Dedicated libusb event-handling thread
	std::thread m_eventThread;

	///@brief Run flag for the libusb event thread
	std::atomic<bool> m_eventThreadRun{false};

	///@brief Serializes control transfers against the bulk/event path
	std::mutex m_transportMutex;

	//Acquisition (Step 4): in-process finite capture via a libusb async bulk ring
	//driven by the event thread, using the slogic core's transfer plan + first-drop
	//+ stall watchdog. Software trigger scan / pretrigger crop is Step 5.
	///@brief slogic per-capture config
	slogic_config m_captureCfg;
	///@brief slogic per-transfer plan (size/ring/timeout)
	slogic_transfer_plan m_plan;
	///@brief slogic stall/first-drop bookkeeping for the current capture
	slogic_stream m_stream;
	///@brief Accumulated (post first-drop) capture bytes, sample-major
	std::vector<uint8_t> m_captureBuf;
	///@brief Target capture length in bytes (m_mdepth * unitsize)
	uint64_t m_captureNeed = 0;
	///@brief Protects m_captureBuf against the bulk callback
	std::mutex m_captureMutex;
	///@brief Set by the callback when m_captureNeed bytes are in hand
	std::atomic<bool> m_captureDone{false};
	///@brief Set on a fatal transfer error / watchdog ABORT
	std::atomic<bool> m_captureAbort{false};
	///@brief Watchdog RETRY_RUN: session thread re-arms the device once
	std::atomic<bool> m_restartPending{false};
	///@brief Number of URBs currently in flight
	std::atomic<int> m_inFlight{0};
	///@brief Monotonic microsecond timestamp of the previous transfer completion
	int64_t m_lastXferUs = 0;
	///@brief The libusb bulk transfer ring + its backing buffers
	std::vector<struct libusb_transfer*> m_ring;
	std::vector<std::vector<uint8_t>> m_ringBufs;

	///@brief libusb bulk completion callback (dispatches to OnTransfer)
	static void LIBUSB_CALL BulkCallback(struct libusb_transfer* xfer);
	///@brief Per-transfer handler: first-drop, append, watchdog, resubmit
	void OnTransfer(struct libusb_transfer* xfer);
	///@brief Reshape m_captureBuf (sample-major) into scopehal waveforms and queue them
	void ReshapeAndQueue();
	///@brief Scan m_captureBuf for the active EdgeTrigger's edge; returns trigger sample index or -1 (none)
	int64_t FindTriggerSample(int unit);
	///@brief Monotonic clock in microseconds
	static int64_t MonotonicUs();

	//USB layer (Step 2)
	///@brief slogic_transport control_write callback → libusb_control_transfer (OUT)
	static int CtrlWrite(void* ctx, uint8_t bRequest, uint16_t wValue,
		uint16_t wIndex, const uint8_t* data, uint16_t len, unsigned timeoutMs);
	///@brief slogic_transport control_read callback → libusb_control_transfer (IN)
	static int CtrlRead(void* ctx, uint8_t bRequest, uint16_t wValue,
		uint16_t wIndex, uint8_t* data, uint16_t len, unsigned timeoutMs);
	///@brief Bind m_transport to this instance's libusb handle
	void BindTransport();
	///@brief Enumerate + open the first matching SLogic USB device (optionally by serial); claim iface 0, start the event thread, reset
	bool OpenDevice(const std::string& serial = "");
	///@brief Stop the event thread, release the interface and close the device
	void CloseDevice();
	///@brief Read + discard any stale bytes left in the bulk IN endpoint between captures
	void DrainEndpoint();
	///@brief libusb event loop body
	void EventThreadFunc();

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
