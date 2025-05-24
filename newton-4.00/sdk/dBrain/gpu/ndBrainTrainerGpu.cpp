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
#include "ndBrainGpuContext.h"
#include "ndBrainGpuCommand.h"
#include "ndBrainLayerLinear.h"
#include "ndBrainGpuFloatBuffer.h"
#include "ndBrainGpuUniformBuffer.h"

ndBrainTrainerGpu::ndBrainTrainerGpu(const ndSharedPtr<ndBrain>& brain, const ndSharedPtr<ndBrainGpuContext>& context, ndInt32 minibatchSize)
	:ndBrainTrainer(brain)
	,m_context(context)
	,m_uniforms()
	,m_inputOutputBuffer()
	,m_weightAndBiasBuffer()
	,m_commandBuffers()
	,m_miniBatchSize(minibatchSize)
{
	InitInputOutputBuffer();
	InitWeightAndBiasBuffer();
}

ndBrainTrainerGpu::ndBrainTrainerGpu(const ndBrainTrainerGpu& src)
	:ndBrainTrainer(src)
	,m_context(src.m_context)
	,m_uniforms()
	,m_inputOutputBuffer()
	,m_weightAndBiasBuffer()
	,m_commandBuffers()
	,m_miniBatchSize(src.m_miniBatchSize)
{
	ndAssert(0);
}

ndBrainTrainerGpu::~ndBrainTrainerGpu()
{
}

void ndBrainTrainerGpu::InitWeightAndBiasBuffer()
{
	const ndBrain& network = **m_brain;

	ndFixSizeArray<ndBrainLayer::ndLayerUniformData, 256> uniformsData;
	for (ndInt32 i = 0; i < ndInt32(network.GetCount()); ++i)
	{
		const ndBrainLayer* const layer = network[i];
		ndBrainLayer::ndLayerUniformData uniformData(layer->GetLayerGpuUniformData(*m_context));
		uniformsData.PushBack(uniformData);
	}
	uniformsData.PushBack(ndBrainLayer::ndLayerUniformData());

	ndInt32 sizeAcc = 0;
	for (ndInt32 i = 0; i < uniformsData.GetCount(); ++i)
	{
		uniformsData[i].m_parametersStartOffset = sizeAcc;
		sizeAcc += uniformsData[i].m_parametersSize;
	}
	
	ndBrainVector parameters;
	parameters.SetCount(sizeAcc);
	for (ndInt32 i = 0; i < ndInt32(network.GetCount()); ++i)
	{
		const ndBrainLayer* const layer = network[i];
		ndBrainMemVector weights(&parameters[uniformsData[i].m_parametersStartOffset], uniformsData[i].m_parametersSize);
		layer->CopyGpuWeights(weights);
	}
	m_weightAndBiasBuffer = ndSharedPtr<ndBrainGpuBuffer>(new ndBrainGpuFloatBuffer(*m_context, parameters, ndCpuMappable));

	ndInt32 inputOutputBufferSize = network.GetInputSize();
	for (ndInt32 i = 0; i < ndInt32(network.GetCount()); ++i)
	{
		const ndBrainLayer* const layer = network[i];
		inputOutputBufferSize += layer->GetOutputSize();
	}

	// create all the uniform buffers
	struct UniformBufferObject
	{
		ndUnsigned32 m_inputSize;
		ndUnsigned32 m_outputSize;
		ndUnsigned32 m_weightsStartOffset;
		ndUnsigned32 m_inputOutputSize;
		ndUnsigned32 m_inputOutputStartOffset;
	};

	class ndGpuCommand : public ndBrainGpuCommand
	{
		public:
		ndGpuCommand(ndBrainGpuContext* const context, 
			VkShaderModule shader,
			ndInt32 numberOfinputs,
			ndBrainGpuBuffer* const uniforms,
			ndBrainGpuBuffer* const weights, 
			ndBrainGpuBuffer* const inputOutputs)
			:ndBrainGpuCommand(context)
		{
			ndFixSizeArray<ndBrainGpuBuffer*, 4> params;
			params.PushBack(uniforms);
			params.PushBack(inputOutputs);
			params.PushBack(weights);
			Assembly(shader, numberOfinputs, params.GetCount(), &params[0]);
		}
	};

	ndBrainGpuBuffer* const weightsBuffer = *m_weightAndBiasBuffer;
	ndBrainGpuBuffer* const inputOutputBuffer = *m_inputOutputBuffer;

	ndInt32 inputOutputStartOffset = 0;
	for (ndInt32 i = 0; i < ndInt32(network.GetCount()); ++i)
	{
		UniformBufferObject uniformParam;
		ndBrainLayer::ndLayerUniformData& data = uniformsData[i];
		uniformParam.m_inputSize = ndUnsigned32(data.m_inputSize);
		uniformParam.m_outputSize = ndUnsigned32(data.m_outputSize);
		uniformParam.m_weightsStartOffset = ndUnsigned32(data.m_parametersStartOffset);

		uniformParam.m_inputOutputSize = ndUnsigned32(inputOutputBufferSize);
		uniformParam.m_inputOutputStartOffset = ndUnsigned32(inputOutputStartOffset);
		ndSharedPtr<ndBrainGpuBuffer> buffer(new ndBrainGpuUniformBuffer(*m_context, sizeof(UniformBufferObject)));
		m_uniforms.Append(buffer);

		VkShaderModule shader = (VkShaderModule)data.m_shader;
		ndSharedPtr<ndBrainGpuCommand> command(new ndGpuCommand(*m_context, shader, m_miniBatchSize, *buffer, weightsBuffer, inputOutputBuffer));
		m_commandBuffers.Append(command);

		inputOutputStartOffset += data.m_inputSize;
	}
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
	m_inputOutputBuffer = ndSharedPtr<ndBrainGpuBuffer>(new ndBrainGpuFloatBuffer(*m_context, buffer, ndCpuMappable));
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
	for (ndList<ndSharedPtr<ndBrainGpuCommand>>::ndNode* node = m_commandBuffers.GetFirst(); node; node = node->GetNext())
	{
		ndSharedPtr<ndBrainGpuCommand>& command = node->GetInfo();
		m_context->AddCommandQueue(command);
	}
}
