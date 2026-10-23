/***********************************************************************************************************************
*                                                                                                                      *
* libscopeprotocols                                                                                                    *
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
	@brief Declaration of XYFilter
 */
#ifndef XYFilter_h
#define XYFilter_h

#include "../scopehal/ConstellationWaveform.h"

/**
	@brief X-Y display of two analog channels: every sample pair (x[i], y[i]) lands in a 2D density plot

	Output is drawn by the constellation renderer. The X axis is in micro-units of the X input (int64 X axis),
	the Y axis in the units of the Y input. Scale is automatic (fit to the data when the plot is cleared) or manual.
 */
class XYFilter : public Filter
{
public:
	XYFilter(const std::string& color);

	virtual void Refresh(vk::raii::CommandBuffer& cmdBuf, std::shared_ptr<QueueHandle> queue) override;

	static std::string GetProtocolName();

	virtual float GetVoltageRange(size_t stream) override;
	virtual float GetOffset(size_t stream) override;
	virtual void SetVoltageRange(float range, size_t stream) override;
	virtual void SetOffset(float offset, size_t stream) override;

	virtual void ClearSweeps() override;

	void SetWidth(size_t width)
	{
		if(m_width != width)
		{
			SetData(nullptr, 0);
			m_width = width;
		}
	}

	void SetHeight(size_t height)
	{
		if(m_height != height)
		{
			SetData(nullptr, 0);
			m_height = height;
		}
	}

	size_t GetWidth() const
	{ return m_width; }

	size_t GetHeight() const
	{ return m_height; }

	///@brief Full-scale span of the X axis, in X input units
	float GetXRange()
	{ return m_xrange.GetFloatVal(); }

	///@brief Value at the center of the X axis, in X input units
	float GetXCenter()
	{ return m_xcenter.GetFloatVal(); }

	PROTOCOL_DECODER_INITPROC(XYFilter)

	enum scale_t
	{
		SCALE_AUTO,
		SCALE_MANUAL
	};

	enum draw_t
	{
		DRAW_POINTS,
		DRAW_LINES
	};

	enum persistence_t
	{
		PERSIST_ACCUMULATE,
		PERSIST_LATEST
	};

protected:
	bool GetSamples(size_t i, const float*& p, size_t& len, int64_t& timescale);
	void AutoScale(const float* x, const float* y, size_t len);
	void Normalize(ConstellationWaveform* cap);

	///@brief Adds one hit at plot coordinates (x, y), in texels from the bottom left; off-plot hits are dropped
	void Plot(int64_t* data, float x, float y)
	{
		if( (x < 0) || (y < 0) )
			return;
		size_t ix = static_cast<size_t>(x);
		size_t iy = static_cast<size_t>(y);
		if( (ix >= m_width) || (iy >= m_height) )
			return;
		data[iy*m_width + ix] ++;
	}

	size_t m_height;
	size_t m_width;

	FilterParameter& m_scale;
	FilterParameter& m_persistence;
	FilterParameter& m_xrange;
	FilterParameter& m_xcenter;
	FilterParameter& m_yrange;
	FilterParameter& m_ycenter;
	FilterParameter& m_pointSize;
	FilterParameter& m_draw;

	///@brief Scratch row/column sums for the point-size box filter
	std::vector<int64_t> m_boxTemp;

	///@brief Scale the current accumulation was integrated with (x range, x center, y range, y center, draw mode)
	float m_accumScale[5];
};

#endif
