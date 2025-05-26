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
#include "ndBrainTrainerCpuInference.h"
#include "ndBrainLayerActivationSoftmax.h"

#if 0
class ndBrainTrainerCpu::ndLayerData : public ndClassAlloc
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

ndBrainTrainerCpu::ndBrainTrainerCpu(const ndSharedPtr<ndBrain>& brain)
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

ndBrainTrainerCpu::ndBrainTrainerCpu(const ndBrainTrainerCpu& src)
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

ndBrainTrainerCpu::~ndBrainTrainerCpu()
{
	for (ndInt32 i = 0; i < m_data.GetCount(); ++i)
	{
		delete (m_data[i]);
	}
}

ndBrainVector& ndBrainTrainerCpu::GetWorkingBuffer()
{
	return m_workingBuffer;
}

ndBrainLayer* ndBrainTrainerCpu::GetWeightsLayer(ndInt32 index) const
{
	return m_data[index]->m_layer;
}

ndBrainLayer* ndBrainTrainerCpu::GetGradientLayer(ndInt32 index) const
{
	return m_data[index]->m_gradient;
}

void ndBrainTrainerCpu::AcculumateGradients(const ndBrainTrainerCpu& src, ndInt32 index)
{
	ndLayerData* const dstData = m_data[index];
	ndAssert(dstData->m_layer->HasParameters());
	const ndLayerData* const srcData = src.m_data[index];
	dstData->Add(*srcData);
}

void ndBrainTrainerCpu::ClearGradients()
{
	for (ndInt32 i = ndInt32(m_data.GetCount() - 1); i >= 0; --i)
	{
		m_data[i]->Clear();
	}
}

void ndBrainTrainerCpu::AddGradients(const ndBrainTrainerCpu* const src)
{
	for (ndInt32 i = ndInt32(m_data.GetCount() - 1); i >= 0; --i)
	{
		m_data[i]->Add(*src->m_data[i]);
	}
}

void ndBrainTrainerCpu::CopyGradients(const ndBrainTrainerCpu* const src)
{
	for (ndInt32 i = ndInt32(m_data.GetCount() - 1); i >= 0; --i)
	{
		m_data[i]->Copy(*src->m_data[i]);
	}
}

void ndBrainTrainerCpu::ScaleWeights(const ndBrainFloat s)
{
	for (ndInt32 i = ndInt32(m_data.GetCount() - 1); i >= 0; --i)
	{
		m_data[i]->Scale(s);
	}
}

void ndBrainTrainerCpu::CalculateInputGradient(const ndBrainVector& input, ndBrainVector& inputGradientsOut, ndBrainLoss& loss)
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

void ndBrainTrainerCpu::MakePrediction(const ndBrainVector&)
{
	ndAssert(0);
}

void ndBrainTrainerCpu::BackPropagate(const ndBrainVector&, const ndBrainVector&)
{
	ndAssert(0);
}

//#pragma optimize( "", off )
void ndBrainTrainerCpu::BackPropagate(const ndBrainVector& input, ndBrainLoss& loss)
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

#endif


ndBrainTrainerCpuInference::ndBrainTrainerCpuInference(const ndSharedPtr<ndBrain>& brain, ndBrainThreadPool* const threadPool, ndInt32 minibatchSize)
	:ndBrainTrainer(brain)
	,m_inputOutputBuffer()
	,m_weightAndBiasBuffer()
	,m_miniBatchInputBuffer()
	,m_miniBatchOutputBuffer()
	,m_threadPool(threadPool)
	,m_miniBatchSize(minibatchSize)
{
	InitInputOutputBuffer();
	InitWeightAndBiasBuffer();
}

ndBrainTrainerCpuInference::ndBrainTrainerCpuInference(const ndBrainTrainerCpuInference& src)
	:ndBrainTrainer(src)
	,m_inputOutputBuffer()
	,m_threadPool(src.m_threadPool)
	,m_miniBatchSize(src.m_miniBatchSize)
{
	ndAssert(0);
}

ndBrainTrainerCpuInference::~ndBrainTrainerCpuInference()
{
}

void ndBrainTrainerCpuInference::InitInputOutputBuffer()
{
	const ndBrain& brain = **m_brain;
	ndInt32 bufferSize = brain.GetInputSize();
	for (ndInt32 i = 0; i < ndInt32(brain.GetCount()); ++i)
	{
		const ndBrainLayer* const layer = brain[i];
		bufferSize += layer->GetOutputSize();
	}
	
	m_inputOutputBuffer.SetCount(bufferSize * m_miniBatchSize);
	m_inputOutputBuffer.Set(ndBrainFloat(0.0f));
}

void ndBrainTrainerCpuInference::InitWeightAndBiasBuffer()
{
	const ndBrain& brain = **m_brain;

	ndFixSizeArray<ndBrainLayer::ndLayerUniformDataCpu*, 256> uniformData;
	for (ndInt32 i = 0; i < ndInt32(brain.GetCount()); ++i)
	{
		const ndBrainLayer* const layer = brain[i];
		ndBrainLayer::ndLayerUniformDataCpu* const data = layer->GetLayerUniformDataCpu();
		data->m_owner = this;
		uniformData.PushBack(data);
	}

	class DummyLayerUniformDataCpu : public ndBrainLayer::ndLayerUniformDataCpu
	{
		public:
		DummyLayerUniformDataCpu()
			:ndBrainLayer::ndLayerUniformDataCpu(nullptr)
		{
		}

		virtual void Execute(ndInt32 threadid, ndInt32 threadCount)
		{
		}
	};

	DummyLayerUniformDataCpu sentinel;
	uniformData.PushBack(&sentinel);
	
	ndInt32 sizeAcc = 0;
	for (ndInt32 i = 0; i < uniformData.GetCount(); ++i)
	{
		uniformData[i]->m_parametersStartOffset = sizeAcc;
		sizeAcc += uniformData[i]->m_parametersSize;
	}
	
	ndInt32 padWeights = 32;
	m_weightAndBiasBuffer.SetCount(sizeAcc + padWeights);
	m_weightAndBiasBuffer.Set(ndBrainFloat(0.0f));
	for (ndInt32 i = 0; i < ndInt32(brain.GetCount()); ++i)
	{
		const ndBrainLayer* const layer = brain[i];
		ndBrainMemVector weights(&m_weightAndBiasBuffer[uniformData[i]->m_parametersStartOffset], uniformData[i]->m_parametersSize);
		layer->CopyWeights(weights);
	}
	
	m_miniBatchInputBuffer.SetCount(m_miniBatchSize * brain.GetInputSize());
	m_miniBatchInputBuffer.Set(ndBrainFloat(0.0f));
	
	m_miniBatchOutputBuffer.SetCount(m_miniBatchSize * brain.GetOutputSize());
	m_miniBatchOutputBuffer.Set(ndBrainFloat(0.0f));
	
	AddCopyInputCommand(uniformData[0]);
	//AddLayersCommands(uniformData);
	//AddCopyOutputCommand();
}

void ndBrainTrainerCpuInference::AddCopyInputCommand(const ndBrainLayer::ndLayerUniformDataCpu* const uniformData)
{
	//const ndBrain& brain = **m_brain;
	//ndInt32 inputOutputBufferSize = brain.GetInputSize();
	//for (ndInt32 i = 0; i < ndInt32(brain.GetCount()); ++i)
	//{
	//	const ndBrainLayer* const layer = brain[i];
	//	inputOutputBufferSize += layer->GetOutputSize();
	//}
	//
	//ndUniformBufferObject uniformParam;
	//uniformParam.m_inputSize = ndUnsigned32(uniformData.m_inputSize);
	//uniformParam.m_outputSize = ndUnsigned32(uniformData.m_outputSize);
	//uniformParam.m_weightsStartOffset = 0;
	//
	//uniformParam.m_inputOutputSize = ndUnsigned32(inputOutputBufferSize);
	//uniformParam.m_inputOutputStartOffset = 0;
	//ndSharedPtr<ndBrainGpuBuffer> uniformbuffer(new ndBrainGpuUniformBuffer(*m_context, sizeof(ndUniformBufferObject)));
	//uniformbuffer->LoadData(sizeof(ndUniformBufferObject), &uniformParam);
	//m_uniforms.Append(uniformbuffer);
	//
	//ndBrainGpuBuffer* const inputOutputBuffer = *m_inputOutputBuffer;
	//ndBrainGpuBuffer* const miniBatchInputBuffer = *m_miniBatchInputBuffer;
	//ndSharedPtr<ndBrainGpuCommand> command(new ndGpuCommand(*m_context, m_context->m_ndBrainCopyInput, m_miniBatchSize, *uniformbuffer, inputOutputBuffer, miniBatchInputBuffer));
	//m_commandBuffers.Append(command);
}

void ndBrainTrainerCpuInference::MakePrediction(const ndBrainVector& input)
{
	ndAssert(0);
}

// new method
void ndBrainTrainerCpuInference::BackPropagate(const ndBrainVector& outputGradients)
{
	ndAssert(0);
}
