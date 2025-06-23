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
#include "ndBrainBuffer.h"
#include "ndBrainTrainerGpu.h"
#include "ndBrainGpuContext.h"
#include "ndBrainGpuCommand.h"
#include "ndBrainLayerLinear.h"
#include "ndBrainGpuFloatBuffer.h"
#include "ndBrainGpuUniformBuffer.h"
#include "ndBrainOptimizerAdamGpu.h"

class ndBrainAdamUpdateCommand : public ndBrainGpuCommand
{
	public:
	ndBrainAdamUpdateCommand(ndBrainTrainerGpuInference* const owner,
		ndBrainGpuContext* const context,
		const ndSharedPtr<ndBrainGpuShader>& shader,
		ndInt32 miniBatchSize,
		ndBrainOptimizerAdamGpu::ndCommandShareInfo& info,
		const ndSharedPtr<ndBrainGpuUniformBuffer>& uniformBuffer)
		:ndBrainGpuCommand(context, ndBrainLayer::ndCommandShareInfo())
		,m_shader(shader)
		,m_uniformBuffer(uniformBuffer)
		,m_info(info)
		,m_owner(owner)
		,m_miniBatchSize(miniBatchSize)
	{
		ndFixSizeArray<ndBrainBuffer*, 8> params;
		params.PushBack(*m_uniformBuffer);
		Assembly(shader, m_miniBatchSize, params.GetCount(), &params[0]);
	}

	ndBrainAdamUpdateCommand(ndBrainTrainerGpuInference* const owner,
		ndBrainGpuContext* const context,
		const ndSharedPtr<ndBrainGpuShader>& shader,
		ndInt32 miniBatchSize,
		ndBrainOptimizerAdamGpu::ndCommandShareInfo& info,
		const ndSharedPtr<ndBrainGpuUniformBuffer>& uniformBuffer,
		ndSharedPtr<ndBrainGpuFloatBuffer>& weightAndBiasBuffer,
		ndSharedPtr<ndBrainGpuFloatBuffer>& weightAndBiasGradientBuffer,
		ndSharedPtr<ndBrainGpuFloatBuffer>& vdw,
		ndSharedPtr<ndBrainGpuFloatBuffer>& vdw2)
		:ndBrainGpuCommand(context, ndBrainLayer::ndCommandShareInfo())
		,m_shader(shader)
		,m_uniformBuffer(uniformBuffer)
		,m_info(info)
		,m_owner(owner)
		,m_miniBatchSize(miniBatchSize)
	{
		ndFixSizeArray<ndBrainBuffer*, 8> params;
		params.PushBack(*m_uniformBuffer);
		params.PushBack(*weightAndBiasBuffer);
		params.PushBack(*weightAndBiasGradientBuffer);
		params.PushBack(*vdw);
		params.PushBack(*vdw2);
		Assembly(shader, m_miniBatchSize, params.GetCount(), &params[0]);
	}

	ndSharedPtr<ndBrainGpuShader> m_shader;
	ndSharedPtr<ndBrainGpuUniformBuffer> m_uniformBuffer;
	ndBrainOptimizerAdamGpu::ndCommandShareInfo m_info;
	ndBrainTrainerGpuInference* m_owner;
	ndInt32 m_miniBatchSize;
};

ndBrainTrainerGpu::ndBrainTrainerGpu(const ndTrainerDescriptor& descriptor)
	:ndBrainTrainerGpuInference(descriptor)
	,m_optimizer(new ndBrainOptimizerAdamGpu(m_context))
	,m_inputOuputGradientsBuffer()
	,m_weightAndBiasGradientsBuffer()
	,m_miniBatchInputGradientBuffer()
	,m_miniBatchOutputGradientBuffer()
{
	ndBrainTrainerGpu::Initialize(descriptor);
}

ndBrainTrainerGpu::ndBrainTrainerGpu(const ndSharedPtr<ndBrain>& brain, const ndSharedPtr<ndBrainContext>& context, ndBrainFloat learnRate, ndInt32 minibatchSize)
	:ndBrainTrainerGpuInference(ndTrainerDescriptor(brain, context, minibatchSize, learnRate))
	,m_optimizer(new ndBrainOptimizerAdamGpu(m_context))
	,m_inputOuputGradientsBuffer()
	,m_weightAndBiasGradientsBuffer()
	,m_miniBatchInputGradientBuffer()
	,m_miniBatchOutputGradientBuffer()
{
	ndTrainerDescriptor descriptor(brain, context, minibatchSize, learnRate);
	ndBrainTrainerGpu::Initialize(descriptor);
}

ndBrainTrainerGpu::ndBrainTrainerGpu(const ndBrainTrainerGpu& src)
	:ndBrainTrainerGpuInference(src)
	,m_inputOuputGradientsBuffer()
	,m_weightAndBiasGradientsBuffer()
{
	ndAssert(0);
}

ndBrainTrainerGpu::~ndBrainTrainerGpu()
{
}

void ndBrainTrainerGpu::Initialize(const ndTrainerDescriptor& descriptor)
{
	ndBrainVector buffer;

	SaveInput(buffer);
	buffer.Set(ndReal(0.0f));
	m_miniBatchInputGradientBuffer = ndSharedPtr<ndBrainGpuFloatBuffer>(new ndBrainGpuFloatBuffer(*m_context, buffer));

	GetOutput(buffer);
	buffer.Set(ndReal(0.0f));
	m_miniBatchOutputGradientBuffer = ndSharedPtr<ndBrainGpuFloatBuffer>(new ndBrainGpuFloatBuffer(*m_context, buffer));

	GetWorkingBuffer(buffer);
	buffer.Set(ndReal(0.0f));
	m_inputOuputGradientsBuffer = ndSharedPtr<ndBrainGpuFloatBuffer>(new ndBrainGpuFloatBuffer(*m_context, buffer));

	GetParameterBuffer(buffer);
	buffer.SetCount(buffer.GetCount() * m_miniBatchSize);
	buffer.Set(ndReal(0.0f));
	m_weightAndBiasGradientsBuffer = ndSharedPtr<ndBrainGpuFloatBuffer>(new ndBrainGpuFloatBuffer(*m_context, buffer));

	AddCopyOutputGradientCommand();
	AddLayersGradientCommands();
	AddCopyInputGradientCommand();
	AddOptimizerGradientCommand(descriptor.m_learRate);
}

void ndBrainTrainerGpu::AddCopyInputGradientCommand()
{
	ndBrainTrainerGpuCommand* const firstCommand = FindCommand(m_inputId);
	ndAssert(firstCommand);
	
	ndBrainLayer::ndCommandShareInfo data(firstCommand->m_info);
	
	data.m_outputSize = 0;
	data.m_parametersBatchSize = 0;
	data.m_parametersStartOffset = 0;
	data.m_inputOutputStartOffset = 0;
	ndSharedPtr<ndBrainGpuUniformBuffer> uniformbuffer(new ndBrainGpuUniformBuffer(*m_context, sizeof(ndBrainLayer::ndCommandShareInfo)));
	uniformbuffer->MemoryToDevive(sizeof(ndBrainLayer::ndCommandShareInfo), &data);
	
	ndBrainGpuFloatBuffer* const inputOutputGradientBuffer = *m_inputOuputGradientsBuffer;
	ndBrainGpuFloatBuffer* const miniBatchInputGradientBuffer = *m_miniBatchInputGradientBuffer;
	ndSharedPtr<ndBrainGpuCommand>command(new ndBrainTrainerGpuCommand(this, data, m_inputId, m_context->GetAsGpuContext(), m_context->GetAsGpuContext()->m_brainCopyInputGradients, m_miniBatchSize, uniformbuffer, nullptr, miniBatchInputGradientBuffer, inputOutputGradientBuffer));
	m_backPropagateCommands.Append(command);
}

void ndBrainTrainerGpu::AddCopyOutputGradientCommand()
{
	ndBrainTrainerGpuCommand* const lastLayerCommand = FindCommand(m_outpuId);
	ndAssert(lastLayerCommand);
	
	ndBrainLayer::ndCommandShareInfo data(lastLayerCommand->m_info);
	
	data.m_inputSize = 0;
	data.m_parametersBatchSize = 0;
	data.m_parametersStartOffset = 0;
	data.m_inputOutputStartOffset += RoundoffOffset(data.m_inputSize);
	ndSharedPtr<ndBrainGpuUniformBuffer> uniformbuffer(new ndBrainGpuUniformBuffer(*m_context, sizeof(ndBrainLayer::ndCommandShareInfo)));
	//uniformbuffer->LoadData(sizeof(ndBrainLayer::ndCommandShareInfo), &data);
	uniformbuffer->MemoryToDevive(sizeof(ndBrainLayer::ndCommandShareInfo), &data);
	
	ndBrainGpuFloatBuffer* const inputOutputGradientBuffer = *m_inputOuputGradientsBuffer;
	ndBrainGpuFloatBuffer* const miniBatchOutputGradientBuffer = *m_miniBatchOutputGradientBuffer;
	ndSharedPtr<ndBrainGpuCommand>command(new ndBrainTrainerGpuCommand(this, data, m_outpuId, m_context->GetAsGpuContext(), m_context->GetAsGpuContext()->m_brainCopyOutputGradients, m_miniBatchSize, uniformbuffer, nullptr, miniBatchOutputGradientBuffer, inputOutputGradientBuffer));
	m_backPropagateCommands.Append(command);
}

void ndBrainTrainerGpu::AddOptimizerGradientCommand(ndBrainFloat learnRate)
{
	ndInt32 sizeInFloats = ndInt32(m_weightAndBiasBuffer->SizeInBytes() / sizeof(ndReal));
	m_optimizer->Init(sizeInFloats, learnRate);
	
	ndBrainLayer::ndCommandShareInfo data;
	data.m_inputSize = sizeInFloats;
	data.m_inputOutputSize = m_miniBatchSize;
	ndBrainGpuFloatBuffer* const weightAndBiasGradientsBuffer = *m_weightAndBiasGradientsBuffer;

#if 1
	ndSharedPtr<ndBrainGpuUniformBuffer> uniformbuffer(new ndBrainGpuUniformBuffer(*m_context, sizeof(ndBrainLayer::ndCommandShareInfo)));
	//uniformbuffer->LoadData(sizeof(ndBrainLayer::ndCommandShareInfo), &data);
	uniformbuffer->MemoryToDevive(sizeof(ndBrainLayer::ndCommandShareInfo), &data);

	ndSharedPtr<ndBrainGpuCommand> accumulateGradients(new ndBrainTrainerGpuCommand(this, data, 0, m_context->GetAsGpuContext(), m_context->GetAsGpuContext()->m_brainAccumulateGradientsAndAverage, m_miniBatchSize, uniformbuffer, weightAndBiasGradientsBuffer, nullptr, nullptr));
	accumulateGradients->m_numberOfWorkGroups = size_t(sizeInFloats / accumulateGradients->m_workGroupSize);
	m_accumulateGradientsCommands.Append(accumulateGradients);
#else
	// as usual my theory are all wrong, this make the update three rtime slower.
	for (ndInt32 i = m_miniBatchSize / 2; i > 0; i >>= 1)
	{
		for (ndInt32 j = 0; j < i; ++j)
		{
			data.m_parametersBatchSize = i * sizeInFloats;
			data.m_parametersStartOffset = j * sizeInFloats;
			ndSharedPtr<ndBrainGpuUniformBuffer> uniformbuffer(new ndBrainGpuUniformBuffer(m_context, sizeof(ndBrainLayer::ndCommandShareInfo)));
			uniformbuffer->LoadData(sizeof(ndBrainLayer::ndCommandShareInfo), &data);

			ndSharedPtr<ndBrainGpuCommand> accumulateGradients(new ndBrainTrainerGpuCommand(this, data, 0, m_context, m_context->m_brainAccumulateGradients, m_miniBatchSize, uniformbuffer, weightAndBiasGradientsBuffer, nullptr, nullptr));
			accumulateGradients->m_numberOfWorkGroups = size_t(sizeInFloats / accumulateGradients->m_workGroupSize);
			m_accumulateGradientsCommands.Append(accumulateGradients);
		}
	}

	ndSharedPtr<ndBrainGpuBuffer> uniformbuffer(new ndBrainGpuUniformBuffer(m_context, sizeof(ndBrainLayer::ndCommandShareInfo)));
	uniformbuffer->LoadData(sizeof(ndBrainLayer::ndCommandShareInfo), &data);

	ndSharedPtr<ndBrainGpuCommand> averageGradients(new ndBrainTrainerGpuCommand(this, data, 0, m_context, m_context->m_brainAverageGradients, m_miniBatchSize, uniformbuffer, weightAndBiasGradientsBuffer, nullptr, nullptr));
	averageGradients->m_numberOfWorkGroups = size_t(sizeInFloats / averageGradients->m_workGroupSize);
	m_accumulateGradientsCommands.Append(averageGradients);

#endif
	
	// add the adam optimizer kernel here
	ndBrainOptimizerAdamGpu::ndCommandShareInfo optimizerData(m_optimizer->m_parameters);
	ndSharedPtr<ndBrainGpuUniformBuffer> adamUniformbuffer(new ndBrainGpuUniformBuffer(*m_context, sizeof(ndBrainOptimizerAdamGpu::ndCommandShareInfo)));
	adamUniformbuffer->MemoryToDevive(sizeof(ndBrainOptimizerAdamGpu::ndCommandShareInfo), &optimizerData);
	
	if (m_optimizer->GetRegularizerType() == m_lasso)
	{
		m_adamOtimizerUpdate = ndSharedPtr<ndBrainGpuCommand>(new ndBrainAdamUpdateCommand(
			this, m_context->GetAsGpuContext(), m_context->GetAsGpuContext()->m_brainAdamLassoOptimizerUpdate, m_miniBatchSize, optimizerData,
			adamUniformbuffer, m_weightAndBiasBuffer, m_weightAndBiasGradientsBuffer,
			m_optimizer->m_vdw, m_optimizer->m_vdw2));
	}
	else
	{
		m_adamOtimizerUpdate = ndSharedPtr<ndBrainGpuCommand>(new ndBrainAdamUpdateCommand(
			this, m_context->GetAsGpuContext(), m_context->GetAsGpuContext()->m_brainAdamRidgeOptimizerUpdate, m_miniBatchSize, optimizerData,
			adamUniformbuffer, m_weightAndBiasBuffer, m_weightAndBiasGradientsBuffer,
			m_optimizer->m_vdw, m_optimizer->m_vdw2));
	}
	
	const ndSharedPtr<ndBrainGpuCommand>& gradientsData = m_accumulateGradientsCommands.GetFirst()->GetInfo();
	m_adamOtimizerUpdate->m_numberOfWorkGroups = size_t(sizeInFloats / gradientsData->m_workGroupSize);
	
	// add the momentum update command
	m_adamMomentumUpdate = ndSharedPtr<ndBrainGpuCommand>(new ndBrainAdamUpdateCommand(
		this, m_context->GetAsGpuContext(), m_context->GetAsGpuContext()->m_brainAdamMomentumUpdate, 1, optimizerData, adamUniformbuffer));
	m_adamMomentumUpdate->m_numberOfWorkGroups = 1;
}

void ndBrainTrainerGpu::AddLayersGradientCommands()
{
	const ndBrain& brain = **m_brain;
	for (ndInt32 i = ndInt32(brain.GetCount()) - 1; i >= 0; --i)
	{
		ndBrainLayer* const layer = brain[i];
		ndBrainTrainerGpuCommand* const feedForwardLayerCommand = FindCommand(size_t(layer));
		ndAssert(feedForwardLayerCommand);
	
		ndBrainLayer::ndCommandShareInfo info(feedForwardLayerCommand->m_info);
	
		ndSharedPtr<ndBrainGpuUniformBuffer> uniformBuffer(new ndBrainGpuUniformBuffer(*m_context, sizeof(ndBrainLayer::ndCommandShareInfo)));
		uniformBuffer->MemoryToDevive(sizeof(ndBrainLayer::ndCommandShareInfo), &info);
	
		ndBrainGpuFloatBuffer* const inputOutputBuffer = *m_inputOutputBuffer;
		ndBrainGpuFloatBuffer* const weightsAndBiasBuffer = *m_weightAndBiasBuffer;
		ndBrainGpuFloatBuffer* const inputOutputGradientBuffer = *m_inputOuputGradientsBuffer;
		ndBrainGpuFloatBuffer* const weightAndBiasGradientsBuffer = *m_weightAndBiasGradientsBuffer;
	
		ndBrainTrainerGpuCommand* const commandBuffer = layer->CreateGpuBackPropagateCommand(
			this, info, m_context->GetAsGpuContext(), m_miniBatchSize,
			uniformBuffer, inputOutputBuffer, weightsAndBiasBuffer, inputOutputGradientBuffer, weightAndBiasGradientsBuffer);
	
		ndSharedPtr<ndBrainGpuCommand>command(commandBuffer);
		command->m_layer = layer;
		m_backPropagateCommands.Append(command);
	}
}

void ndBrainTrainerGpu::GetGradientBuffer(ndBrainVector& output) const
{
	m_context->GetAsGpuContext()->SyncQueue();
	m_inputOuputGradientsBuffer->BrainVectorFromDevice(output);
}

void ndBrainTrainerGpu::BackPropagate(const ndBrainVector& outputGradients)
{
	//m_miniBatchOutputGradientBuffer->LoadData(outputGradients.GetCount() * sizeof(ndReal), &outputGradients[0]);
	m_miniBatchOutputGradientBuffer->BrainVectorToDevice(outputGradients);

	for (ndList<ndSharedPtr<ndBrainGpuCommand>>::ndNode* node = m_backPropagateCommands.GetFirst(); node; node = node->GetNext())
	{
		ndSharedPtr<ndBrainGpuCommand>& command = node->GetInfo();
		m_context->GetAsGpuContext()->AddCommandQueue(command);
	}
}

void ndBrainTrainerGpu::ApplyLearnRate()
{
	for (ndList<ndSharedPtr<ndBrainGpuCommand>>::ndNode* node = m_accumulateGradientsCommands.GetFirst(); node; node = node->GetNext())
	{
		ndSharedPtr<ndBrainGpuCommand>& command = node->GetInfo();
		m_context->GetAsGpuContext()->AddCommandQueue(command);
	}
	m_context->GetAsGpuContext()->AddCommandQueue(m_adamOtimizerUpdate);
}