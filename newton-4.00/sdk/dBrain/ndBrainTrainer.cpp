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
#include "ndBrainKernel.h"
#include "ndBrainVector.h"
#include "ndBrainMatrix.h"
#include "ndBrainBuffer.h"
#include "ndBrainTrainer.h"
#include "ndBrainContext.h"
#include "ndBrainGpuCommand.h"
#include "ndBrainLayerLinear.h"
#include "ndBrainFloatBuffer.h"
#include "ndBrainUniformBuffer.h"
#include "ndBrainOptimizerAdamGpu.h"

class ndBrainAdamUpdateCommand : public ndBrainGpuCommand
{
	public:
	ndBrainAdamUpdateCommand(ndBrainTrainerInference* const owner,
		ndBrainContext* const context,
		const ndSharedPtr<ndBrainKernel>& shader,
		ndInt32 miniBatchSize,
		ndBrainOptimizerAdamGpu::ndCommandShareInfo& info,
		const ndSharedPtr<ndBrainUniformBuffer>& uniformBuffer)
		:ndBrainGpuCommand(context, ndCommandShareInfo())
		,m_shader(shader)
		,m_uniformBuffer(uniformBuffer)
		,m_info(info)
		,m_owner(owner)
		,m_miniBatchSize(miniBatchSize)
	{
		ndAssert(0);
		//ndFixSizeArray<ndBrainBuffer*, 8> params;
		//params.PushBack(*m_uniformBuffer);
		//Assembly(shader, m_miniBatchSize, params.GetCount(), &params[0]);
	}

	ndBrainAdamUpdateCommand(ndBrainTrainerInference* const owner,
		ndBrainContext* const context,
		const ndSharedPtr<ndBrainKernel>& shader,
		ndInt32 miniBatchSize,
		ndBrainOptimizerAdamGpu::ndCommandShareInfo& info,
		const ndSharedPtr<ndBrainUniformBuffer>& uniformBuffer,
		ndSharedPtr<ndBrainFloatBuffer>& weightAndBiasBuffer,
		ndSharedPtr<ndBrainFloatBuffer>& weightAndBiasGradientBuffer,
		ndSharedPtr<ndBrainFloatBuffer>& vdw,
		ndSharedPtr<ndBrainFloatBuffer>& vdw2)
		:ndBrainGpuCommand(context, ndCommandShareInfo())
		,m_shader(shader)
		,m_uniformBuffer(uniformBuffer)
		,m_info(info)
		,m_owner(owner)
		,m_miniBatchSize(miniBatchSize)
	{
		ndAssert(0);
		//ndFixSizeArray<ndBrainBuffer*, 8> params;
		//params.PushBack(*m_uniformBuffer);
		//params.PushBack(*weightAndBiasBuffer);
		//params.PushBack(*weightAndBiasGradientBuffer);
		//params.PushBack(*vdw);
		//params.PushBack(*vdw2);
		//Assembly(shader, m_miniBatchSize, params.GetCount(), &params[0]);
	}

	ndSharedPtr<ndBrainKernel> m_shader;
	ndSharedPtr<ndBrainUniformBuffer> m_uniformBuffer;
	ndBrainOptimizerAdamGpu::ndCommandShareInfo m_info;
	ndBrainTrainerInference* m_owner;
	ndInt32 m_miniBatchSize;
};

ndBrainTrainer::ndBrainTrainer(const ndTrainerDescriptor& descriptor)
	:ndBrainTrainerInference(descriptor)
	//,m_optimizer(new ndBrainOptimizerAdamGpu(m_context))
	,m_inputOutputGradientsBuffer()
	,m_weightAndBiasGradientsBuffer()
	,m_miniBatchInputGradientBuffer()
	,m_miniBatchOutputGradientBuffer()
{
	Initialize();
}

ndBrainTrainer::ndBrainTrainer(const ndBrainTrainer& src)
	:ndBrainTrainerInference(src)
	,m_inputOutputGradientsBuffer()
	,m_weightAndBiasGradientsBuffer()
{
	ndAssert(0);
}

ndBrainTrainer::~ndBrainTrainer()
{
}

ndBrainFloatBuffer* ndBrainTrainer::GetOuputGradientBuffer()
{
	return *m_miniBatchOutputGradientBuffer;
}

ndBrainFloatBuffer* ndBrainTrainer::GetHiddenLayerGradientBuffer()
{
	return *m_inputOutputGradientsBuffer;
}

ndBrainFloatBuffer* ndBrainTrainer::GetWeightAndBiasGradientBuffer()
{
	return *m_weightAndBiasGradientsBuffer;
}

#if 0
void ndBrainTrainer::AddOptimizerGradientCommand(ndBrainFloat learnRate)
{
	ndInt32 sizeInFloats = ndInt32(m_weightAndBiasBuffer->SizeInBytes() / sizeof(ndReal));
	m_optimizer->Init(sizeInFloats, learnRate);
	
	ndCommandShareInfo data;
	data.m_inputSize = sizeInFloats;
	data.m_inputOutputSize = m_miniBatchSize;
	ndBrainFloatBuffer* const weightAndBiasGradientsBuffer = *m_weightAndBiasGradientsBuffer;

#if 1
	ndSharedPtr<ndBrainUniformBuffer> uniformbuffer(new ndBrainUniformBuffer(*m_descriptor.m_context, sizeof(ndCommandShareInfo)));
	uniformbuffer->MemoryToDevice(0, sizeof(ndCommandShareInfo), &data);

	ndSharedPtr<ndBrainGpuCommand> accumulateGradients(new ndBrainBufferCommand(this, data, 0, m_context->GetAsGpuContext(), m_context->GetAsGpuContext()->m_brainAccumulateGradientsAndAverage, m_miniBatchSize, uniformbuffer, weightAndBiasGradientsBuffer, nullptr, nullptr));
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
			ndSharedPtr<ndBrainUniformBuffer> uniformbuffer(new ndBrainUniformBuffer(m_context, sizeof(ndCommandShareInfo)));
			uniformbuffer->LoadData(sizeof(ndCommandShareInfo), &data);

			ndSharedPtr<ndBrainGpuCommand> accumulateGradients(new ndBrainBufferCommand(this, data, 0, m_context, m_context->m_brainAccumulateGradients, m_miniBatchSize, uniformbuffer, weightAndBiasGradientsBuffer, nullptr, nullptr));
			accumulateGradients->m_numberOfWorkGroups = size_t(sizeInFloats / accumulateGradients->m_workGroupSize);
			m_accumulateGradientsCommands.Append(accumulateGradients);
		}
	}

	ndSharedPtr<ndBrainGpuBuffer> uniformbuffer(new ndBrainUniformBuffer(m_context, sizeof(ndCommandShareInfo)));
	uniformbuffer->LoadData(sizeof(ndCommandShareInfo), &data);

	ndSharedPtr<ndBrainGpuCommand> averageGradients(new ndBrainBufferCommand(this, data, 0, m_context, m_context->m_brainAverageGradients, m_miniBatchSize, uniformbuffer, weightAndBiasGradientsBuffer, nullptr, nullptr));
	averageGradients->m_numberOfWorkGroups = size_t(sizeInFloats / averageGradients->m_workGroupSize);
	m_accumulateGradientsCommands.Append(averageGradients);

#endif
	
	// add the adam optimizer kernel here
	ndBrainOptimizerAdamGpu::ndCommandShareInfo optimizerData(m_optimizer->m_parameters);
	ndSharedPtr<ndBrainUniformBuffer> adamUniformbuffer(new ndBrainUniformBuffer(*m_descriptor.m_context, sizeof(ndBrainOptimizerAdamGpu::ndCommandShareInfo)));
	adamUniformbuffer->MemoryToDevice(0, sizeof(ndBrainOptimizerAdamGpu::ndCommandShareInfo), &optimizerData);
	
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


void ndBrainTrainer::GetGradientBuffer(ndBrainVector& output) const
{
	m_context->GetAsGpuContext()->SyncQueue();
	m_inputOutputGradientsBuffer->BrainVectorFromDevice(output);
}

void ndBrainTrainer::BackPropagate(const ndBrainVector& outputGradients)
{
	//m_miniBatchOutputGradientBuffer->LoadData(outputGradients.GetCount() * sizeof(ndReal), &outputGradients[0]);
	m_miniBatchOutputGradientBuffer->BrainVectorToDevice(outputGradients);

	for (ndList<ndSharedPtr<ndBrainGpuCommand>>::ndNode* node = m_backPropagateCommands.GetFirst(); node; node = node->GetNext())
	{
		ndSharedPtr<ndBrainGpuCommand>& command = node->GetInfo();
		m_context->GetAsGpuContext()->AddCommandQueue(command);
	}
}

void ndBrainTrainer::ApplyLearnRate()
{
	for (ndList<ndSharedPtr<ndBrainGpuCommand>>::ndNode* node = m_accumulateGradientsCommands.GetFirst(); node; node = node->GetNext())
	{
		ndSharedPtr<ndBrainGpuCommand>& command = node->GetInfo();
		m_context->GetAsGpuContext()->AddCommandQueue(command);
	}
	m_context->GetAsGpuContext()->AddCommandQueue(m_adamOtimizerUpdate);
}
#endif

void ndBrainTrainer::Initialize()
{
	ndBrainVector buffer;

	buffer.SetCount(ndInt64(m_miniBatchInputBuffer->GetCount()));
	buffer.Set(ndReal(0.0f));
	m_miniBatchInputGradientBuffer = ndSharedPtr<ndBrainFloatBuffer>(new ndBrainFloatBuffer(*m_descriptor.m_context, buffer));
	
	buffer.SetCount(ndInt64(m_miniBatchOutputBuffer->GetCount()));
	buffer.Set(ndReal(0.0f));
	m_miniBatchOutputGradientBuffer = ndSharedPtr<ndBrainFloatBuffer>(new ndBrainFloatBuffer(*m_descriptor.m_context, buffer));
	
	buffer.SetCount(ndInt64(m_inputOutputBuffer->GetCount()));
	buffer.Set(ndReal(0.0f));
	m_inputOutputGradientsBuffer = ndSharedPtr<ndBrainFloatBuffer>(new ndBrainFloatBuffer(*m_descriptor.m_context, buffer));
	
	buffer.SetCount(ndInt64(m_weightAndBiasBuffer->GetCount() * m_descriptor.m_minibatchSize));
	buffer.Set(ndReal(0.0f));
	m_weightAndBiasGradientsBuffer = ndSharedPtr<ndBrainFloatBuffer>(new ndBrainFloatBuffer(*m_descriptor.m_context, buffer));
	
	AddCopyOutputGradientCommand();
	AddLayersGradientCommands();
	AddCopyInputGradientCommand();
	 
	//AddOptimizerGradientCommand(descriptor.m_learRate);
}

void ndBrainTrainer::AddCopyOutputGradientCommand()
{
	ndBrainBufferCommand* const lastLayerCommand = FindCommand(m_outpuId);
	ndAssert(lastLayerCommand);

	ndBrainBufferCommandDesc& desc = lastLayerCommand->GetDescriptor();

	ndCommandShareInfo data(desc.m_info);
	data.m_inputSize = 0;
	data.m_parametersBatchSize = 0;
	data.m_parametersStartOffset = 0;
	data.m_inputOutputStartOffset += RoundoffOffset(data.m_inputSize);
	ndSharedPtr<ndBrainUniformBuffer> uniformbuffer(new ndBrainUniformBuffer(*m_descriptor.m_context, sizeof(ndCommandShareInfo), &data));

	ndBrainFloatBuffer* const inputOutputGradientBuffer = *m_inputOutputGradientsBuffer;
	ndBrainFloatBuffer* const miniBatchOutputGradientBuffer = *m_miniBatchOutputGradientBuffer;

	ndBrainBufferCommandDesc descritor(m_descriptor.m_minibatchSize);
	descritor.m_context = *m_descriptor.m_context;
	descritor.m_owner = this;
	descritor.m_id = m_outpuId;
	descritor.m_info = data;
	descritor.m_uniformBuffer = uniformbuffer;
	descritor.PushBack(miniBatchOutputGradientBuffer);
	descritor.PushBack(inputOutputGradientBuffer);

	if (descritor.m_context->GetAsCpuContext())
	{
		class ndCopyOutputGradientCommandCpu : public ndBrainBufferCommandCpu
		{
			public:
			ndCopyOutputGradientCommandCpu(const ndBrainBufferCommandDesc& desc)
				:ndBrainBufferCommandCpu(desc)
			{
			}

			virtual void Execute(ndInt32 miniBatchIndex) override
			{
				const ndCommandShareInfo& info = m_desc.m_info;
				ndBrainTrainer* const owner = (ndBrainTrainer*)m_desc.m_owner;

				ndBrainFloat* const dstPtr = (ndBrainFloat*)owner->m_inputOutputGradientsBuffer->GetCpuPtr();
				const ndBrainFloat* const srcPtr = (ndBrainFloat*)owner->m_miniBatchOutputGradientBuffer->GetCpuPtr();
				
				ndInt32 destOffset = miniBatchIndex * info.m_inputOutputSize;
				const ndBrainMemVector src(&srcPtr[miniBatchIndex * info.m_outputSize], info.m_outputSize);
				ndBrainMemVector dst(&dstPtr[destOffset + info.m_inputOutputStartOffset], info.m_outputSize);
				dst.Set(src);
			}
		};

		ndSharedPtr<ndBrainBufferCommand>command(new ndCopyOutputGradientCommandCpu(descritor));
		m_backPropagateCommands.Append(command);
	}
	else
	{
		ndAssert(0);
	}
}

void ndBrainTrainer::AddCopyInputGradientCommand()
{
	ndBrainBufferCommand* const firstCommand = FindCommand(m_inputId);
	ndAssert(firstCommand);

	ndBrainBufferCommandDesc& desc = firstCommand->GetDescriptor();
	ndCommandShareInfo data(desc.m_info);

	data.m_outputSize = 0;
	data.m_parametersBatchSize = 0;
	data.m_parametersStartOffset = 0;
	data.m_inputOutputStartOffset = 0;
	ndSharedPtr<ndBrainUniformBuffer> uniformbuffer(new ndBrainUniformBuffer(*m_descriptor.m_context, sizeof(ndCommandShareInfo), &data));

	ndBrainFloatBuffer* const inputOutputGradientBuffer = *m_inputOutputGradientsBuffer;
	ndBrainFloatBuffer* const miniBatchInputGradientBuffer = *m_miniBatchInputGradientBuffer;

	ndBrainBufferCommandDesc descritor(m_descriptor.m_minibatchSize);
	descritor.m_context = *m_descriptor.m_context;
	descritor.m_owner = this;
	descritor.m_id = m_inputId;
	descritor.m_info = data;
	descritor.m_uniformBuffer = uniformbuffer;
	descritor.PushBack(miniBatchInputGradientBuffer);
	descritor.PushBack(inputOutputGradientBuffer);

	if (descritor.m_context->GetAsCpuContext())
	{
		class ndCopyInputGradientCommandCpu : public ndBrainBufferCommandCpu
		{
			public:
			ndCopyInputGradientCommandCpu(const ndBrainBufferCommandDesc& desc)
				:ndBrainBufferCommandCpu(desc)
			{
			}

			virtual void Execute(ndInt32 miniBatchIndex) override
			{
				const ndCommandShareInfo& info = m_desc.m_info;
				ndBrainTrainer* const owner = (ndBrainTrainer*)m_desc.m_owner;

				ndBrainFloat* const dstPtr = (ndBrainFloat*)owner->m_miniBatchInputGradientBuffer->GetCpuPtr();
				const ndBrainFloat* const srcPtr = (ndBrainFloat*)owner->m_inputOutputGradientsBuffer->GetCpuPtr();

				const ndBrainMemVector src(&srcPtr[miniBatchIndex * info.m_inputOutputSize + info.m_inputOutputStartOffset], info.m_inputSize);
				ndBrainMemVector dst(&dstPtr[miniBatchIndex * info.m_inputSize], info.m_inputSize);
				dst.Set(src);
			}
		};

		ndSharedPtr<ndBrainBufferCommand>command(new ndCopyInputGradientCommandCpu(descritor));
		m_backPropagateCommands.Append(command);
	}
	else
	{
		ndAssert(0);
	}
}

void ndBrainTrainer::AddLayersGradientCommands()
{
	const ndBrain& brain = **m_descriptor.m_brain;
	for (ndInt32 i = ndInt32(brain.GetCount()) - 1; i >= 0; --i)
	{
		ndBrainLayer* const layer = brain[i];
		ndBrainBufferCommand* const feedForwardLayerCommand = FindCommand(size_t(layer));
		ndAssert(feedForwardLayerCommand);
		ndBrainBufferCommandDesc& desc = feedForwardLayerCommand->GetDescriptor();

		ndCommandShareInfo info(desc.m_info);
		ndSharedPtr<ndBrainUniformBuffer> uniformBuffer(new ndBrainUniformBuffer(*m_descriptor.m_context, sizeof(ndCommandShareInfo), &info));

		ndBrainFloatBuffer* const inputOutputBuffer = *m_inputOutputBuffer;
		ndBrainFloatBuffer* const weightsAndBiasBuffer = *m_weightAndBiasBuffer;
		ndBrainFloatBuffer* const inputOutputGradientBuffer = *m_inputOutputGradientsBuffer;
		ndBrainFloatBuffer* const weightAndBiasGradientsBuffer = *m_weightAndBiasGradientsBuffer;

		ndSharedPtr<ndBrainBufferCommand>command(layer->CreateGpuBackPropagateCommand(
			this, info, *m_descriptor.m_context, m_descriptor.m_minibatchSize,
			uniformBuffer, inputOutputBuffer, weightsAndBiasBuffer,
			inputOutputGradientBuffer, weightAndBiasGradientsBuffer));
		m_backPropagateCommands.Append(command);
	}
}

void ndBrainTrainer::BackPropagate()
{
	ndBrainContext* const context = *m_descriptor.m_context;
	for (ndList<ndSharedPtr<ndBrainBufferCommand>>::ndNode* node = m_backPropagateCommands.GetFirst(); node; node = node->GetNext())
	{
		ndSharedPtr<ndBrainBufferCommand>& command = node->GetInfo();
		context->SubmitBufferCommand(*command);
	}
}