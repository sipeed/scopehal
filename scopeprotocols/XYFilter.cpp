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
	@brief Implementation of XYFilter
 */

#include "../scopehal/scopehal.h"
#include "XYFilter.h"

using namespace std;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Construction / destruction

XYFilter::XYFilter(const string& color)
	: Filter(color, CAT_MATH)
	, m_height(1)
	, m_width(1)
	, m_scale(m_parameters["Scale"])
	, m_persistence(m_parameters["Persistence"])
	, m_xrange(m_parameters["X Range"])
	, m_xcenter(m_parameters["X Center"])
	, m_yrange(m_parameters["Y Range"])
	, m_ycenter(m_parameters["Y Center"])
	, m_pointSize(m_parameters["Point Size"])
	, m_draw(m_parameters["Draw"])
	, m_accumScale{0, 0, 0, 0, 0}
{
	AddStream(Unit(Unit::UNIT_VOLTS), "data", Stream::STREAM_TYPE_CONSTELLATION);

	m_xAxisUnit = Unit(Unit::UNIT_MICROVOLTS);

	CreateInput<InputConstraintStreamType>("x", Stream::STREAM_TYPE_ANALOG);
	CreateInput<InputConstraintStreamType>("y", Stream::STREAM_TYPE_ANALOG);

	m_scale = FilterParameter(FilterParameter::TYPE_ENUM, Unit(Unit::UNIT_COUNTS));
	m_scale.AddEnumValue("Auto", SCALE_AUTO);
	m_scale.AddEnumValue("Manual", SCALE_MANUAL);
	m_scale.SetIntVal(SCALE_AUTO);

	m_persistence = FilterParameter(FilterParameter::TYPE_ENUM, Unit(Unit::UNIT_COUNTS));
	m_persistence.AddEnumValue("Accumulate", PERSIST_ACCUMULATE);
	m_persistence.AddEnumValue("Latest capture", PERSIST_LATEST);
	m_persistence.SetIntVal(PERSIST_ACCUMULATE);

	m_xrange = FilterParameter(FilterParameter::TYPE_FLOAT, Unit(Unit::UNIT_VOLTS));
	m_xrange.SetFloatVal(1);
	m_xcenter = FilterParameter(FilterParameter::TYPE_FLOAT, Unit(Unit::UNIT_VOLTS));
	m_xcenter.SetFloatVal(0);
	m_yrange = FilterParameter(FilterParameter::TYPE_FLOAT, Unit(Unit::UNIT_VOLTS));
	m_yrange.SetFloatVal(1);
	m_ycenter = FilterParameter(FilterParameter::TYPE_FLOAT, Unit(Unit::UNIT_VOLTS));
	m_ycenter.SetFloatVal(0);

	m_pointSize = FilterParameter(FilterParameter::TYPE_INT, Unit(Unit::UNIT_COUNTS));
	m_pointSize.SetIntVal(3);

	m_draw = FilterParameter(FilterParameter::TYPE_ENUM, Unit(Unit::UNIT_COUNTS));
	m_draw.AddEnumValue("Points", DRAW_POINTS);
	m_draw.AddEnumValue("Lines", DRAW_LINES);
	m_draw.SetIntVal(DRAW_POINTS);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Accessors

string XYFilter::GetProtocolName()
{
	return "X-Y";
}

float XYFilter::GetVoltageRange(size_t /*stream*/)
{
	return m_yrange.GetFloatVal();
}

float XYFilter::GetOffset(size_t /*stream*/)
{
	return -m_ycenter.GetFloatVal();
}

void XYFilter::SetVoltageRange(float /*range*/, size_t /*stream*/)
{
	//Scale comes from the filter parameters
}

void XYFilter::SetOffset(float /*offset*/, size_t /*stream*/)
{
	//Scale comes from the filter parameters
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Actual decoder logic

void XYFilter::ClearSweeps()
{
	SetData(nullptr, 0);
}

/**
	@brief Gets the CPU-side samples of an analog input (uniform or sparse)
 */
bool XYFilter::GetSamples(size_t i, const float*& p, size_t& len, int64_t& timescale)
{
	auto data = GetInputWaveform(i);
	auto u = dynamic_cast<UniformAnalogWaveform*>(data);
	auto s = dynamic_cast<SparseAnalogWaveform*>(data);
	if(u)
	{
		u->PrepareForCpuAccess();
		p = u->m_samples.GetCpuPointer();
		len = u->size();
	}
	else if(s)
	{
		s->PrepareForCpuAccess();
		p = s->m_samples.GetCpuPointer();
		len = s->size();
	}
	else
		return false;
	timescale = data->m_timescale;
	return true;
}

/**
	@brief Fits both axes to the data with a 10% margin
 */
void XYFilter::AutoScale(const float* x, const float* y, size_t len)
{
	float xmin = FLT_MAX, xmax = -FLT_MAX, ymin = FLT_MAX, ymax = -FLT_MAX;
	for(size_t i=0; i<len; i++)
	{
		xmin = min(xmin, x[i]);
		xmax = max(xmax, x[i]);
		ymin = min(ymin, y[i]);
		ymax = max(ymax, y[i]);
	}

	//A flat input still gets a visible span around its value
	auto span = [](float lo, float hi)
	{
		float r = (hi - lo) * 1.1f;
		return (r > 0) ? r : max(fabs(hi) * 0.2f, 1e-3f);
	};
	m_xrange.SetFloatVal(span(xmin, xmax));
	m_xcenter.SetFloatVal((xmin + xmax) / 2);
	m_yrange.SetFloatVal(span(ymin, ymax));
	m_ycenter.SetFloatVal((ymin + ymax) / 2);
}

void XYFilter::Refresh(
	[[maybe_unused]] vk::raii::CommandBuffer& cmdBuf,
	[[maybe_unused]] shared_ptr<QueueHandle> queue)
{
	#ifdef HAVE_NVTX
		nvtx3::scoped_range nrange("XYFilter::Refresh");
	#endif

	ClearMessages();

	//If an input goes momentarily bad, don't delete output - just stop updating
	const float* px;
	const float* py;
	size_t xlen, ylen;
	int64_t xts, yts;
	if(!GetSamples(0, px, xlen, xts))
	{
		AddErrorMessage("Missing inputs", "No analog waveform available at X input");
		return;
	}
	if(!GetSamples(1, py, ylen, yts))
	{
		AddErrorMessage("Missing inputs", "No analog waveform available at Y input");
		return;
	}
	if(xts != yts)
	{
		AddErrorMessage("Invalid inputs", "X and Y inputs must have the same sample rate");
		return;
	}
	size_t len = min(xlen, ylen);

	//Axis units follow the inputs (X axis is int64, so micro-units)
	auto xunit = GetInput(0).GetYAxisUnits();
	if(xunit == Unit::UNIT_AMPS)
		m_xAxisUnit = Unit(Unit::UNIT_MICROAMPS);
	else
		m_xAxisUnit = Unit(Unit::UNIT_MICROVOLTS);
	SetYAxisUnits(GetInput(1).GetYAxisUnits(), 0);

	auto cap = dynamic_cast<ConstellationWaveform*>(GetData(0));
	bool fresh = !cap || (m_persistence.GetIntVal() == PERSIST_LATEST);

	//Auto scale is fitted whenever the plot starts over, then held while accumulating
	if(fresh && (m_scale.GetIntVal() == SCALE_AUTO) && len)
		AutoScale(px, py, len);

	//A scale change invalidates what was integrated so far
	float scale[5] = { GetXRange(), GetXCenter(), m_yrange.GetFloatVal(), m_ycenter.GetFloatVal(),
		static_cast<float>(m_draw.GetIntVal()) };
	if(memcmp(scale, m_accumScale, sizeof(scale)) != 0)
		fresh = true;
	memcpy(m_accumScale, scale, sizeof(scale));

	if(fresh || !cap)
	{
		cap = new ConstellationWaveform(m_width, m_height);
		cap->m_timescale = 1;
		SetData(cap, 0);
	}

	float xscale = m_width / scale[0];
	float xlo = scale[1] - scale[0] / 2;
	float yscale = m_height / scale[2];
	float ylo = scale[3] - scale[2] / 2;

	cap->PrepareForCpuAccess();
	auto data = cap->GetAccumData();
	if( (m_draw.GetIntVal() == DRAW_LINES) && len)
	{
		//Join consecutive samples like an analog scope's beam. Each segment adds 1 to the texels it
		//crosses, its start excluded so a sample is not counted twice. Segments are stepped in
		//texels, so the work is bounded per refresh; past that the rest is drawn as points.
		const int64_t budget = 64 * 1024 * 1024;
		int64_t used = 0;
		bool clipped = false;
		float fx0 = (px[0] - xlo) * xscale;
		float fy0 = (py[0] - ylo) * yscale;
		for(size_t i=0; i<len; i++)
		{
			float fx1 = (px[i] - xlo) * xscale;
			float fy1 = (py[i] - ylo) * yscale;
			int64_t steps = (i == 0) ? 0 : static_cast<int64_t>(ceil(max(fabs(fx1 - fx0), fabs(fy1 - fy0))));
			if(used + steps > budget)
			{
				steps = 0;
				clipped = true;
			}
			used += steps;
			if(steps == 0)
				Plot(data, fx1, fy1);
			float dx = steps ? (fx1 - fx0) / steps : 0;
			float dy = steps ? (fy1 - fy0) / steps : 0;
			for(int64_t k=1; k<=steps; k++)
				Plot(data, fx0 + dx * k, fy0 + dy * k);
			fx0 = fx1;
			fy0 = fy1;
		}
		if(clipped)
			AddErrorMessage("Too many lines", "Line drawing stopped at its per-capture limit; the rest is drawn as points");
	}
	else
	{
		for(size_t i=0; i<len; i++)
			Plot(data, (px[i] - xlo) * xscale, (py[i] - ylo) * yscale);
	}
	cap->MarkModifiedFromCpu();

	cap->IntegrateSymbols(len);
	Normalize(cap);
	cap->m_revision ++;
}

/**
	@brief Normalizes the accumulated hits into the output, drawing each hit as a Point Size square

	The accumulation stays one texel per sample, so changing the point size needs no new data.
 */
void XYFilter::Normalize(ConstellationWaveform* cap)
{
	ssize_t w = m_width;
	ssize_t h = m_height;
	ssize_t size = max<int64_t>(1, min<int64_t>(m_pointSize.GetIntVal(), 64));
	ssize_t lo = (size - 1) / 2;		//window is [i - lo, i - lo + size); loops start where index 0 enters

	auto accum = cap->GetAccumData();
	m_boxTemp.resize(w * h);
	auto tmp = m_boxTemp.data();

	//Separable box sum: rows into tmp, then columns back into the output
	for(ssize_t y=0; y<h; y++)
	{
		auto src = accum + y*w;
		auto dst = tmp + y*w;
		int64_t sum = 0;
		for(ssize_t x=lo-size+1; x<w; x++)
		{
			ssize_t in = x - lo + size - 1;
			ssize_t out = x - lo - 1;
			if(in < w)
				sum += src[in];
			if(out >= 0)
				sum -= src[out];
			if(x >= 0)
				dst[x] = sum;
		}
	}

	auto& outbuf = cap->GetOutData();
	outbuf.PrepareForCpuAccess();
	auto out = outbuf.GetCpuPointer();
	int64_t nmax = 1;
	vector<int64_t> col(h);
	for(ssize_t x=0; x<w; x++)
	{
		int64_t sum = 0;
		for(ssize_t y=lo-size+1; y<h; y++)
		{
			ssize_t in = y - lo + size - 1;
			ssize_t gone = y - lo - 1;
			if(in < h)
				sum += tmp[in*w + x];
			if(gone >= 0)
				sum -= tmp[gone*w + x];
			if(y >= 0)
			{
				col[y] = sum;
				nmax = max(nmax, sum);
			}
		}
		for(ssize_t y=0; y<h; y++)
			out[y*w + x] = col[y];
	}

	//Same scaling as ConstellationWaveform::Normalize(): half of the peak count is full brightness
	float norm = 2.0f * cap->m_saturationLevel / nmax;
	for(ssize_t i=0; i<w*h; i++)
		out[i] = min(1.0f, out[i] * norm);
	outbuf.MarkModifiedFromCpu();
}
