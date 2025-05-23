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
#include "ndBrainGpuBuffer.h"
#include "ndBrainTrainerGpu.h"
#include "ndBrainLayerLinear.h"
#include "ndBrainGpuFloatBuffer.h"
#include "ndBrainLayerActivationSoftmax.h"

class ndBrainTrainerGpu::ndLayerData : public ndClassAlloc
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

ndBrainTrainerGpu::ndBrainTrainerGpu(const ndSharedPtr<ndBrain>& brain, const ndSharedPtr<ndBrainGpuContext>& context, ndInt32 minibatchSize)
	:ndBrainTrainer(brain)
	,m_context(context)
	,m_inputOutputBuffer()
	,m_weightAndBiasBuffer()
	,m_uniforms()
	,m_miniBatchSize(minibatchSize)
{
#if 0
	for (ndInt32 i = 0; i < m_brain->GetCount(); ++i)
	{
		m_data.PushBack(new ndLayerData((*m_brain)[i]));
	}

	const ndArray<ndBrainLayer*>& layers = *brain;

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
#endif

	InitWeightAndBiasBuffer();
	InitInputOutputBuffer();
}

ndBrainTrainerGpu::ndBrainTrainerGpu(const ndBrainTrainerGpu& src)
	:ndBrainTrainer(src)
	,m_context(src.m_context)
	,m_uniforms()
	,m_inputOutputBuffer()
	,m_weightAndBiasBuffer()
	//,m_data()
	//,m_workingBuffer()
	//,m_prefixScan(src.m_prefixScan)
	//,m_workingBufferSize(src.m_workingBufferSize)
	//,m_maxLayerBufferSize(src.m_maxLayerBufferSize)
{
	ndAssert(0);
#if 0
	m_workingBuffer.SetCount(src.m_workingBuffer.GetCount());
	for (ndInt32 i = 0; i < m_brain->GetCount(); ++i)
	{
		m_data.PushBack(new ndLayerData((*m_brain)[i]));
	}
#endif
}

ndBrainTrainerGpu::~ndBrainTrainerGpu()
{
	ndAssert(0);
	//for (ndInt32 i = 0; i < m_data.GetCount(); ++i)
	//{
	//	delete (m_data[i]);
	//}
}

void ndBrainTrainerGpu::InitWeightAndBiasBuffer()
{
	const ndBrain& network = **m_brain;

	for (ndInt32 i = 0; i < ndInt32(network.GetCount()); ++i)
	{
		const ndBrainLayer* const layer = network[i];
		ndBrainLayer::ndLayerUniformData uniformData(layer->GetLayerGpuUniformData(*m_context));
		m_uniforms.PushBack(uniformData);
	}
	m_uniforms.PushBack(ndBrainLayer::ndLayerUniformData());

	ndInt32 sizeAcc = 0;
	for (ndInt32 i = 0; i < m_uniforms.GetCount(); ++i)
	{
		ndInt32 blockSize = m_uniforms[i].m_blockSize;
		m_uniforms[i].m_blockSize = sizeAcc;
		sizeAcc += blockSize;
	}

	ndBrainVector parameters;
	parameters.SetCount(sizeAcc);
	for (ndInt32 i = 0; i < ndInt32(network.GetCount()); ++i)
	{
		const ndBrainLayer* const layer = network[i];
		ndBrainMemVector weights(&parameters[m_uniforms[i].m_blockSize], m_uniforms[i].m_parameterSize);
		layer->CopyGpuWeights(weights);
	}
	m_weightAndBiasBuffer = ndSharedPtr<ndBrainGpuFloatBuffer>(new ndBrainGpuFloatBuffer(*m_context, parameters, ndCpuMappable));
}

void ndBrainTrainerGpu::InitInputOutputBuffer()
{
	const ndBrain& network = **m_brain;
	ndInt32 bufferSize = network.GetInputSize();
	for (ndInt32 i = 0; i < ndInt32(network.GetCount()); ++i)
	{
		const ndBrainLayer* const layer = network[i];
		bufferSize += layer->GetOutputSize();
	}

	ndBrainVector buffer;
	buffer.SetCount(bufferSize * m_miniBatchSize);
	buffer.Set(ndBrainFloat(0.0f));
	m_inputOutputBuffer = ndSharedPtr<ndBrainGpuFloatBuffer>(new ndBrainGpuFloatBuffer(*m_context, buffer, ndCpuMappable));
}

ndBrainVector& ndBrainTrainerGpu::GetWorkingBuffer()
{
	ndAssert(0);
	static ndBrainVector xxx;
	return xxx;
	//return m_workingBuffer;
}

ndBrainLayer* ndBrainTrainerGpu::GetWeightsLayer(ndInt32 index) const
{
	ndAssert(0);
	return nullptr;
	//return m_data[index]->m_layer;
}

ndBrainLayer* ndBrainTrainerGpu::GetGradientLayer(ndInt32 index) const
{
	ndAssert(0);
	return nullptr;
	//return m_data[index]->m_gradient;
}

void ndBrainTrainerGpu::AcculumateGradients(const ndBrainTrainerGpu& src, ndInt32 index)
{
	ndAssert(0);
	//ndLayerData* const dstData = m_data[index];
	//ndAssert(dstData->m_layer->HasParameters());
	//const ndLayerData* const srcData = src.m_data[index];
	//dstData->Add(*srcData);
}

void ndBrainTrainerGpu::ClearGradients()
{
	ndAssert(0);
	//for (ndInt32 i = ndInt32(m_data.GetCount() - 1); i >= 0; --i)
	//{
	//	m_data[i]->Clear();
	//}
}

void ndBrainTrainerGpu::AddGradients(const ndBrainTrainerGpu* const src)
{
	ndAssert(0);
	//for (ndInt32 i = ndInt32(m_data.GetCount() - 1); i >= 0; --i)
	//{
	//	m_data[i]->Add(*src->m_data[i]);
	//}
}

void ndBrainTrainerGpu::CopyGradients(const ndBrainTrainerGpu* const src)
{
	ndAssert(0);
	//for (ndInt32 i = ndInt32(m_data.GetCount() - 1); i >= 0; --i)
	//{
	//	m_data[i]->Copy(*src->m_data[i]);
	//}
}

void ndBrainTrainerGpu::ScaleWeights(const ndBrainFloat s)
{
	ndAssert(0);
	//for (ndInt32 i = ndInt32(m_data.GetCount() - 1); i >= 0; --i)
	//{
	//	m_data[i]->Scale(s);
	//}
}

void ndBrainTrainerGpu::CalculateInputGradient(const ndBrainVector& input, ndBrainVector& inputGradientsOut, ndBrainLoss& loss)
{
	ndAssert(0);
#if 0
	const ndInt32 layersCount = ndInt32(m_brain->GetCount());
	const ndArray<ndBrainLayer*>& layers = *m_brain;
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
#endif
}

//#pragma optimize( "", off )
void ndBrainTrainerGpu::BackPropagate(const ndBrainVector& input, ndBrainLoss& loss)
{
	ndAssert(0);
#if 0
	const ndInt32 layersCount = ndInt32(m_brain->GetCount());
	const ndArray<ndBrainLayer*>& layers = *m_brain;
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
#endif
}
