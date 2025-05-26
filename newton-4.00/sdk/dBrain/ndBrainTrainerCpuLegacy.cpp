/* Copyright (c) <2003-2022> <Julio Jerez, Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 
* 3. This notice may not be removed or altered from any source distribution.
*/

#include "ndBrainStdafx.h"
#include "ndBrain.h"
#include "ndBrainLoss.h"
#include "ndBrainLayer.h"
#include "ndBrainVector.h"
#include "ndBrainMatrix.h"
#include "ndBrainTrainerCpuLegacy.h"
#include "ndBrainLayerActivationSoftmax.h"

class ndBrainTrainerCpuLegacy::ndLayerData : public ndClassAlloc
{
	public:
	ndLayerData(ndBrainLayer* const layer)
		:ndClassAlloc()
		,m_layer(layer)
		,m_gradient(nullptr)
	{
		if (layer->HasParameters())
		{
			m_gradient = layer->Clone();
		}
	}

	~ndLayerData()
	{
		if (m_gradient)
		{
			delete m_gradient;
		}
	}

	void Clear()
	{
		if (m_layer->HasParameters())
		{
			m_gradient->Clear();
		}
	}

	void Add(const ndLayerData& src)
	{
		if (m_layer->HasParameters())
		{
			m_gradient->Add(*src.m_gradient);
		}
	}

	void Copy(const ndLayerData& src)
	{
		if (m_layer->HasParameters())
		{
			m_gradient->Set(*src.m_gradient);
		}
	}

	void Scale(ndBrainFloat s)
	{
		if (m_layer->HasParameters())
		{
			m_gradient->Scale(s);
		}
	}

	ndBrainLayer* m_layer;
	ndBrainLayer* m_gradient;
};

ndBrainTrainerCpuLegacy::ndBrainTrainerCpuLegacy(const ndSharedPtr<ndBrain>& brain)
	:ndBrainTrainer(brain)
	,m_data()
	,m_workingBuffer()
	,m_prefixScan()
{
	for (ndInt32 i = 0; i < m_brain->GetCount(); ++i)
	{
		m_data.PushBack(new ndLayerData((**m_brain)[i]));
	}

	const ndArray<ndBrainLayer*>& layers = **brain;

	ndInt32 maxSize = layers[0]->GetInputSize();
	ndInt32 sizeAcc = (layers[0]->GetInputSize() + 7) & -8;

	m_prefixScan.PushBack(0);
	for (ndInt32 i = 0; i < m_brain->GetCount(); ++i)
	{
		m_prefixScan.PushBack(sizeAcc);
		ndInt32 outputSize = layers[i]->GetOutputBufferSize();
		sizeAcc += (outputSize + 7) & -8;
		maxSize = ndMax(maxSize, outputSize);
	}
	m_prefixScan.PushBack(sizeAcc + 32);

	m_maxLayerBufferSize = maxSize;
	m_workingBufferSize = sizeAcc + maxSize * 2 + 256;
	m_workingBuffer.SetCount(m_workingBufferSize);
}

ndBrainTrainerCpuLegacy::ndBrainTrainerCpuLegacy(const ndBrainTrainerCpuLegacy& src)
	:ndBrainTrainer(src)
	,m_data()
	,m_workingBuffer()
	,m_prefixScan(src.m_prefixScan)
	,m_workingBufferSize(src.m_workingBufferSize)
	,m_maxLayerBufferSize(src.m_maxLayerBufferSize)
{
	ndAssert(0);
	m_workingBuffer.SetCount(src.m_workingBuffer.GetCount());
	for (ndInt32 i = 0; i < m_brain->GetCount(); ++i)
	{
		m_data.PushBack(new ndLayerData((**m_brain)[i]));
	}
}

ndBrainTrainerCpuLegacy::~ndBrainTrainerCpuLegacy()
{
	for (ndInt32 i = 0; i < m_data.GetCount(); ++i)
	{
		delete (m_data[i]);
	}
}

ndBrainVector& ndBrainTrainerCpuLegacy::GetWorkingBuffer()
{
	return m_workingBuffer;
}

ndBrainLayer* ndBrainTrainerCpuLegacy::GetWeightsLayer(ndInt32 index) const
{
	return m_data[index]->m_layer;
}

ndBrainLayer* ndBrainTrainerCpuLegacy::GetGradientLayer(ndInt32 index) const
{
	return m_data[index]->m_gradient;
}

void ndBrainTrainerCpuLegacy::AcculumateGradients(const ndBrainTrainerCpuLegacy& src, ndInt32 index)
{
	ndLayerData* const dstData = m_data[index];
	ndAssert(dstData->m_layer->HasParameters());
	const ndLayerData* const srcData = src.m_data[index];
	dstData->Add(*srcData);
}

void ndBrainTrainerCpuLegacy::ClearGradients()
{
	for (ndInt32 i = ndInt32(m_data.GetCount() - 1); i >= 0; --i)
	{
		m_data[i]->Clear();
	}
}

void ndBrainTrainerCpuLegacy::AddGradients(const ndBrainTrainerCpuLegacy* const src)
{
	for (ndInt32 i = ndInt32(m_data.GetCount() - 1); i >= 0; --i)
	{
		m_data[i]->Add(*src->m_data[i]);
	}
}

void ndBrainTrainerCpuLegacy::CopyGradients(const ndBrainTrainerCpuLegacy* const src)
{
	for (ndInt32 i = ndInt32(m_data.GetCount() - 1); i >= 0; --i)
	{
		m_data[i]->Copy(*src->m_data[i]);
	}
}

void ndBrainTrainerCpuLegacy::ScaleWeights(const ndBrainFloat s)
{
	for (ndInt32 i = ndInt32(m_data.GetCount() - 1); i >= 0; --i)
	{
		m_data[i]->Scale(s);
	}
}

void ndBrainTrainerCpuLegacy::CalculateInputGradient(const ndBrainVector& input, ndBrainVector& inputGradientsOut, ndBrainLoss& loss)
{
	const ndInt32 layersCount = ndInt32(m_brain->GetCount());
	const ndArray<ndBrainLayer*>& layers = **m_brain;
	ndAssert(!(loss.IsCategorical() ^ (!strcmp(layers[layersCount - 1]->GetLabelId(), "ndBrainLayerActivationCategoricalSoftmax"))));

	if (m_workingBuffer.GetCount() < m_workingBufferSize)
	{
		m_workingBuffer.SetCount(m_workingBufferSize);
	}

	const ndInt32 gradientOffset = m_prefixScan[m_prefixScan.GetCount() - 1];
	const ndBrainFloat* const memBuffer = &m_workingBuffer[0];
	const ndBrainFloat* const gradientBuffer = &m_workingBuffer[gradientOffset];
	const ndInt32 maxSize = m_maxLayerBufferSize;

	ndBrainMemVector in0(memBuffer, input.GetCount());
	in0.Set(input);

	for (ndInt32 i = 0; i < layersCount; ++i)
	{
		const ndBrainMemVector in(memBuffer + m_prefixScan[i + 0], layers[i]->GetInputSize());
		ndBrainMemVector out(memBuffer + m_prefixScan[i + 1], layers[i]->GetOutputSize());
		layers[i]->MakePrediction(in, out);
	}
	const ndBrainMemVector output(memBuffer + m_prefixScan[layersCount], m_brain->GetOutputSize());

	ndBrainMemVector gradientIn(gradientBuffer, m_brain->GetOutputSize());
	ndBrainMemVector gradientOut(gradientBuffer + maxSize + 128, m_brain->GetOutputSize());
	loss.GetLoss(output, gradientOut);

	for (ndInt32 i = ndInt32(m_data.GetCount()) - 1; i >= 0; --i)
	{
		const ndBrainLayer* const layer = m_data[i]->m_layer;
		gradientIn.SetSize(layer->GetInputSize());
		const ndBrainMemVector in(memBuffer + m_prefixScan[i + 0], layer->GetInputSize());
		const ndBrainMemVector out(memBuffer + m_prefixScan[i + 1], layer->GetOutputSize());
		layer->InputDerivative(in, out, gradientOut, gradientIn);
		gradientIn.Swap(gradientOut);
	}
	inputGradientsOut.Set(gradientOut);
}

void ndBrainTrainerCpuLegacy::MakePrediction(const ndBrainVector&)
{
	ndAssert(0);
}

void ndBrainTrainerCpuLegacy::BackPropagate(const ndBrainVector&, const ndBrainVector&)
{
	ndAssert(0);
}

//#pragma optimize( "", off )
void ndBrainTrainerCpuLegacy::BackPropagate(const ndBrainVector& input, ndBrainLoss& loss)
{
	const ndInt32 layersCount = ndInt32(m_brain->GetCount());
	const ndArray<ndBrainLayer*>& layers = **m_brain;
	ndAssert(!(loss.IsCategorical() ^ (!strcmp(layers[layersCount - 1]->GetLabelId(), "ndBrainLayerActivationCategoricalSoftmax"))));

	if (m_workingBuffer.GetCount() < m_workingBufferSize)
	{
		m_workingBuffer.SetCount(m_workingBufferSize);
	}

	const ndInt32 gradientOffset = m_prefixScan[m_prefixScan.GetCount() - 1];
	const ndBrainFloat* const memBuffer = &m_workingBuffer[0];
	const ndBrainFloat* const gradientBuffer = &m_workingBuffer[gradientOffset];
	const ndInt32 maxSize = m_maxLayerBufferSize;

	ndBrainMemVector in0(memBuffer, input.GetCount());
	in0.Set(input);

	for (ndInt32 i = 0; i < layersCount; ++i)
	{
		const ndBrainMemVector in(memBuffer + m_prefixScan[i + 0], layers[i]->GetInputSize());
		ndBrainMemVector out(memBuffer + m_prefixScan[i + 1], layers[i]->GetOutputSize());
		layers[i]->MakePrediction(in, out);
	}
	const ndBrainMemVector output(memBuffer + m_prefixScan[layersCount], m_brain->GetOutputSize());
	
	ndBrainMemVector gradientIn(gradientBuffer, m_brain->GetOutputSize());
	ndBrainMemVector gradientOut(gradientBuffer + maxSize + 128, m_brain->GetOutputSize());
	loss.GetLoss(output, gradientOut);

	for (ndInt32 i = ndInt32(m_data.GetCount()) - 1; i >= 0; --i)
	{
		const ndBrainLayer* const layer = m_data[i]->m_layer;
		gradientIn.SetSize(layer->GetInputSize());
		const ndBrainMemVector in(memBuffer + m_prefixScan[i + 0], layer->GetInputSize());
		const ndBrainMemVector out(memBuffer + m_prefixScan[i + 1], layer->GetOutputSize());
		layer->CalculateParamGradients(in, out, gradientOut, gradientIn, m_data[i]->m_gradient);
		gradientIn.Swap(gradientOut);
	}
}
