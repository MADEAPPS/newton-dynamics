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
#include "ndBrainOptimizerAdamGpu.h"

class ndBrainAdamUpdateCommand : public ndBrainGpuCommand
{
	public:
	ndBrainAdamUpdateCommand(ndBrainTrainerGpuInference* const owner,
		ndBrainGpuContext* const context,
		const ndSharedPtr<ndBrainGpuShader>& shader,
		ndInt32 miniBatchSize,
		ndBrainOptimizerAdamGpu::ndCommandShareInfo& info,
		const ndSharedPtr<ndBrainGpuBuffer>& uniformBuffer)
		:ndBrainGpuCommand(context, ndBrainLayer::ndCommandShareInfo())
		,m_shader(shader)
		,m_uniformBuffer(uniformBuffer)
		,m_info(info)
		,m_owner(owner)
		,m_miniBatchSize(miniBatchSize)
	{
		ndFixSizeArray<ndBrainGpuBuffer*, 8> params;
		params.PushBack(*m_uniformBuffer);
		Assembly(shader, m_miniBatchSize, params.GetCount(), &params[0]);
	}

	ndBrainAdamUpdateCommand(ndBrainTrainerGpuInference* const owner,
		ndBrainGpuContext* const context,
		const ndSharedPtr<ndBrainGpuShader>& shader,
		ndInt32 miniBatchSize,
		ndBrainOptimizerAdamGpu::ndCommandShareInfo& info,
		const ndSharedPtr<ndBrainGpuBuffer>& uniformBuffer,
		ndSharedPtr<ndBrainGpuBuffer>& weightAndBiasBuffer,
		ndSharedPtr<ndBrainGpuBuffer>& weightAndBiasGradientBuffer,
		ndSharedPtr<ndBrainGpuBuffer>& vdw,
		ndSharedPtr<ndBrainGpuBuffer>& vdw2)
		:ndBrainGpuCommand(context, ndBrainLayer::ndCommandShareInfo())
		,m_shader(shader)
		,m_uniformBuffer(uniformBuffer)
		,m_info(info)
		,m_owner(owner)
		,m_miniBatchSize(miniBatchSize)
	{
		ndFixSizeArray<ndBrainGpuBuffer*, 8> params;
		params.PushBack(*m_uniformBuffer);
		params.PushBack(*weightAndBiasBuffer);
		params.PushBack(*weightAndBiasGradientBuffer);
		params.PushBack(*vdw);
		params.PushBack(*vdw2);
		Assembly(shader, m_miniBatchSize, params.GetCount(), &params[0]);
	}

	ndSharedPtr<ndBrainGpuShader> m_shader;
	ndSharedPtr<ndBrainGpuBuffer> m_uniformBuffer;
	ndBrainOptimizerAdamGpu::ndCommandShareInfo m_info;
	ndBrainTrainerGpuInference* m_owner;
	ndInt32 m_miniBatchSize;
};


ndBrainTrainerGpu::ndBrainTrainerGpu(const ndSharedPtr<ndBrain>& brain, const ndSharedPtr<ndBrainContext>& context, ndBrainFloat learnRate, ndInt32 minibatchSize)
	:ndBrainTrainerGpuInference(brain, context, minibatchSize)
	,m_optimizer(new ndBrainOptimizerAdamGpu(context))
	,m_inputOuputGradientsBuffer()
	,m_weightAndBiasGradientsBuffer()
	,m_miniBatchInputGradientBuffer()
	,m_miniBatchOutputGradientBuffer()
{
	ndBrainVector buffer;

	GetInput(buffer);
	buffer.Set(ndReal(0.0f));
	m_miniBatchInputGradientBuffer = ndSharedPtr<ndBrainGpuBuffer>(new ndBrainGpuFloatBuffer(m_context, buffer, ndCpuMappable));

	GetOutput(buffer);
	buffer.Set(ndReal(0.0f));
	m_miniBatchOutputGradientBuffer = ndSharedPtr<ndBrainGpuBuffer>(new ndBrainGpuFloatBuffer(m_context, buffer, ndCpuMappable));

	GetWorkingBuffer(buffer);
	buffer.SetCount(buffer.GetCount() * m_miniBatchSize);
	buffer.Set(ndReal(0.0f));
	m_inputOuputGradientsBuffer = ndSharedPtr<ndBrainGpuBuffer>(new ndBrainGpuFloatBuffer(m_context, buffer, ndCpuMappable));

	GetParameterBuffer(buffer);
	buffer.SetCount(buffer.GetCount() * m_miniBatchSize);
	buffer.Set(ndReal(0.0f));
	m_weightAndBiasGradientsBuffer = ndSharedPtr<ndBrainGpuBuffer>(new ndBrainGpuFloatBuffer(m_context, buffer, ndCpuMappable));

	AddCopyOutputGradientCommand();
	AddLayersGradientCommands();
	AddCopyInputGradientCommand();
	AddOptimizerGradientCommand(learnRate);
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

void ndBrainTrainerGpu::AddCopyInputGradientCommand()
{
	ndBrainTrainerGpuCommand* const firstCommand = FindCommand(m_inputId);
	ndAssert(firstCommand);

	ndBrainLayer::ndCommandShareInfo data(firstCommand->m_info);

	data.m_outputSize = 0;
	data.m_parametersBatchSize = 0;
	data.m_parametersStartOffset = 0;
	data.m_inputOutputStartOffset = 0;
	ndSharedPtr<ndBrainGpuBuffer> uniformbuffer(new ndBrainGpuUniformBuffer(m_context, sizeof(ndBrainLayer::ndCommandShareInfo)));
	uniformbuffer->LoadData(sizeof(ndBrainLayer::ndCommandShareInfo), &data);

	ndBrainGpuBuffer* const inputOutputGradientBuffer = *m_inputOuputGradientsBuffer;
	ndBrainGpuBuffer* const miniBatchInputGradientBuffer = *m_miniBatchInputGradientBuffer;
	ndSharedPtr<ndBrainGpuCommand>command(new ndBrainTrainerGpuCommand(this, data, m_inputId, m_context, m_context->m_ndBrainCopyInputGradients, m_miniBatchSize, uniformbuffer, nullptr, miniBatchInputGradientBuffer, inputOutputGradientBuffer));
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
	data.m_inputOutputStartOffset += data.m_inputSize;
	ndSharedPtr<ndBrainGpuBuffer> uniformbuffer(new ndBrainGpuUniformBuffer(m_context, sizeof(ndBrainLayer::ndCommandShareInfo)));
	uniformbuffer->LoadData(sizeof(ndBrainLayer::ndCommandShareInfo), &data);

	ndBrainGpuBuffer* const inputOutputGradientBuffer = *m_inputOuputGradientsBuffer;
	ndBrainGpuBuffer* const miniBatchOutputGradientBuffer = *m_miniBatchOutputGradientBuffer;
	ndSharedPtr<ndBrainGpuCommand>command(new ndBrainTrainerGpuCommand(this, data, m_outpuId, m_context, m_context->m_ndBrainCopyOutputGradients, m_miniBatchSize, uniformbuffer, nullptr, miniBatchOutputGradientBuffer, inputOutputGradientBuffer));
	m_backPropagateCommands.Append(command);
}

void ndBrainTrainerGpu::AddOptimizerGradientCommand(ndBrainFloat learnRate)
{
	ndInt32 sizeInFloats = ndInt32(m_weightAndBiasBuffer->SizeInBytes() / sizeof(ndReal));
	m_optimizer->Init(sizeInFloats, learnRate);

	ndBrainLayer::ndCommandShareInfo data;

	data.m_inputSize = sizeInFloats;
	data.m_inputOutputSize = m_miniBatchSize;
	ndSharedPtr<ndBrainGpuBuffer> uniformbuffer(new ndBrainGpuUniformBuffer(m_context, sizeof(ndBrainLayer::ndCommandShareInfo)));
	uniformbuffer->LoadData(sizeof(ndBrainLayer::ndCommandShareInfo), &data);

	ndBrainGpuBuffer* const weightAndBiasGradientsBuffer = *m_weightAndBiasGradientsBuffer;
	m_accumulateGradients = ndSharedPtr<ndBrainGpuCommand>(new ndBrainTrainerGpuCommand(this, data, 0, m_context, m_context->m_ndBrainAccumulateGradients, m_miniBatchSize, uniformbuffer, weightAndBiasGradientsBuffer, nullptr, nullptr));
	m_accumulateGradients->m_miniBatchSize = size_t(sizeInFloats / ND_KERNELS_WORKGROUP_SIZE);

	// add the adam optimizer kernel here
	ndBrainOptimizerAdamGpu::ndCommandShareInfo optimizerData(m_optimizer->m_parameters);
	optimizerData.m_minibatchSize = sizeInFloats / ND_KERNELS_WORKGROUP_SIZE;
	ndSharedPtr<ndBrainGpuBuffer> adamUniformbuffer(new ndBrainGpuUniformBuffer(m_context, sizeof(ndBrainOptimizerAdamGpu::ndCommandShareInfo)));
	adamUniformbuffer->LoadData(sizeof(ndBrainOptimizerAdamGpu::ndCommandShareInfo), &optimizerData);

	if (m_optimizer->GetRegularizerType() == ndBrainOptimizer::m_lasso)
	{
		m_adamOtimizerUpdate = ndSharedPtr<ndBrainGpuCommand>(new ndBrainAdamUpdateCommand(
			this, m_context, m_context->m_ndBrainAdamLassoOptimizerUpdate, m_miniBatchSize, optimizerData,
			adamUniformbuffer, m_weightAndBiasBuffer, m_weightAndBiasGradientsBuffer,
			m_optimizer->m_vdw, m_optimizer->m_vdw2));
		m_adamOtimizerUpdate->m_miniBatchSize = size_t(sizeInFloats / ND_KERNELS_WORKGROUP_SIZE);
	}
	else
	{
		m_adamOtimizerUpdate = ndSharedPtr<ndBrainGpuCommand>(new ndBrainAdamUpdateCommand(
			this, m_context, m_context->m_ndBrainAdamRidgeOptimizerUpdate, m_miniBatchSize, optimizerData,
			adamUniformbuffer, m_weightAndBiasBuffer, m_weightAndBiasGradientsBuffer,
			m_optimizer->m_vdw, m_optimizer->m_vdw2));
		m_adamOtimizerUpdate->m_miniBatchSize = size_t(sizeInFloats / ND_KERNELS_WORKGROUP_SIZE);
	}

	// add the momentum update command
	m_adamMomentumUpdate = ndSharedPtr<ndBrainGpuCommand>(new ndBrainAdamUpdateCommand(
		this, m_context, m_context->m_ndBrainAdamMomentumUpdate, 1, optimizerData, adamUniformbuffer));
	m_adamMomentumUpdate->m_miniBatchSize = 1;
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

		ndSharedPtr<ndBrainGpuBuffer> uniformBuffer(new ndBrainGpuUniformBuffer(m_context, sizeof(ndBrainLayer::ndCommandShareInfo)));
		uniformBuffer->LoadData(sizeof(ndBrainLayer::ndCommandShareInfo), &info);

		ndBrainGpuBuffer* const inputOutputBuffer = *m_inputOutputBuffer;
		ndBrainGpuBuffer* const weightsAndBiasBuffer = *m_weightAndBiasBuffer;
		ndBrainGpuBuffer* const inputOutputGradientBuffer = *m_inputOuputGradientsBuffer;
		ndBrainGpuBuffer* const weightAndBiasGradientsBuffer = *m_weightAndBiasGradientsBuffer;

		ndBrainTrainerGpuCommand* const commandBuffer = layer->CreateGpuBackPropagateCommand(
			this, info, m_context, m_miniBatchSize,
			uniformBuffer, inputOutputBuffer, weightsAndBiasBuffer, inputOutputGradientBuffer, weightAndBiasGradientsBuffer);

		ndSharedPtr<ndBrainGpuCommand>command(commandBuffer);
		command->m_layer = layer;
		m_backPropagateCommands.Append(command);
	}
}

void ndBrainTrainerGpu::ApplyLearnRate()
{
	m_context->AddCommandQueue(m_accumulateGradients);
	m_context->AddCommandQueue(m_adamOtimizerUpdate);
	m_context->AddCommandQueue(m_adamMomentumUpdate);
}

void ndBrainTrainerGpu::BackPropagate(const ndBrainVector& outputGradients, bool sync)
{
	m_miniBatchOutputGradientBuffer->LoadData(outputGradients.GetCount() * sizeof(ndReal), &outputGradients[0]);

	for (ndList<ndSharedPtr<ndBrainGpuCommand>>::ndNode* node = m_backPropagateCommands.GetFirst(); node; node = node->GetNext())
	{
		ndSharedPtr<ndBrainGpuCommand>& command = node->GetInfo();
		m_context->AddCommandQueue(command);
	}
	if (sync)
	{
		SyncQueue();
	}
}