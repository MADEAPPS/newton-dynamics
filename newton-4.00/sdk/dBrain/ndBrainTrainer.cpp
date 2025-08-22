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
#include "ndBrainLayerLinear.h"
#include "ndBrainFloatBuffer.h"
#include "ndBrainUniformBuffer.h"
#include "ndBrainOptimizerAdam.h"

ndBrainTrainer::ndBrainTrainer(const ndTrainerDescriptor& descriptor)
	:ndBrainTrainerInference(descriptor)
	,m_optimizer(new ndBrainOptimizerAdam(m_descriptor.m_context))
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

ndBrainFloatBuffer* ndBrainTrainer::GetInputGradientBuffer()
{
	return *m_miniBatchInputGradientBuffer;
}

ndBrainFloatBuffer* ndBrainTrainer::GetOuputGradientBuffer()
{
	return *m_miniBatchOutputGradientBuffer;
}

ndBrainFloatBuffer* ndBrainTrainer::GetPartialSumBiasGradientBuffer()
{
	return *m_biasPartialSumGradientsCacheBuffer;
}

ndBrainFloatBuffer* ndBrainTrainer::GetHiddenLayerGradientBuffer()
{
	return *m_inputOutputGradientsBuffer;
}

ndBrainFloatBuffer* ndBrainTrainer::GetWeightAndBiasGradientBuffer()
{
	return *m_weightAndBiasGradientsBuffer;
}

void ndBrainTrainer::Initialize()
{
	ndBrainVector buffer;
	ndInt64 maxSize = ndInt64(ndMax(m_weightAndBiasBuffer->GetCount(), m_inputOutputBuffer->GetCount()));
	buffer.Resize(maxSize);
	
	m_optimizer->SetRegularizer(m_descriptor.m_regularizer);
	m_optimizer->SetRegularizerType(m_descriptor.m_regularizerType);

	buffer.SetCount(ndInt64(m_inputOutputBuffer->GetCount()));
	buffer.Set(ndReal(0.0f));
	m_inputOutputGradientsBuffer = ndSharedPtr<ndBrainFloatBuffer>(new ndBrainFloatBuffer(*m_descriptor.m_context, buffer));

	ndInt32 partialSum = 0;
	for (ndInt32 i = 0; i < m_descriptor.m_brain->GetCount(); ++i)
	{	
		const ndBrainLayer* const layer = (**m_descriptor.m_brain)[0];
		if (partialSum < (layer->GetOutputSize() + 1024))
		{
			ndInt32 outSize = layer->GetOutputSize();
			partialSum = RoundOffOffset(outSize);
		}
	}
	buffer.SetCount(ndInt64(m_descriptor.m_minibatchSize) * partialSum);
	buffer.Set(ndReal(0.0f));
	m_biasPartialSumGradientsCacheBuffer = ndSharedPtr<ndBrainFloatBuffer>(new ndBrainFloatBuffer(*m_descriptor.m_context, buffer));

	buffer.SetCount(ndInt64(m_weightAndBiasBuffer->GetCount()));
	buffer.Set(ndReal(0.0f));
	m_weightAndBiasGradientsBuffer = ndSharedPtr<ndBrainFloatBuffer>(new ndBrainFloatBuffer(*m_descriptor.m_context, buffer));

	buffer.SetCount(ndInt64(m_miniBatchInputBuffer->GetCount()));
	buffer.Set(ndReal(0.0f));
	m_miniBatchInputGradientBuffer = ndSharedPtr<ndBrainFloatBuffer>(new ndBrainFloatBuffer(*m_descriptor.m_context, buffer));

	buffer.SetCount(ndInt64(m_miniBatchOutputBuffer->GetCount()));
	buffer.Set(ndReal(0.0f));
	m_miniBatchOutputGradientBuffer = ndSharedPtr<ndBrainFloatBuffer>(new ndBrainFloatBuffer(*m_descriptor.m_context, buffer));
	
	AddCopyOutputGradientCommand();
	AddLayersGradientCommands();
	AddCopyInputGradientCommand();
	AddOptimizerGradientCommand();
}

void ndBrainTrainer::AddCopyOutputGradientCommand()
{
	ndBrainBufferCommand* const lastLayerCommand = FindCommand(m_outpuId);
	ndAssert(lastLayerCommand);

	ndBrainBufferCommandDesc& desc = lastLayerCommand->GetDescriptor();

	ndCommandSharedInfo data(desc.m_info);
	data.m_inputSize = 0;
	data.m_parametersBatchSize = 0;
	data.m_parametersStartOffset = 0;
	data.m_inputOutputStartOffset += RoundOffOffset(data.m_inputSize);
	ndSharedPtr<ndBrainUniformBuffer> uniformbuffer(new ndBrainUniformBuffer(*m_descriptor.m_context, sizeof(ndCommandSharedInfo), &data));

	ndBrainFloatBuffer* const inputOutputGradientBuffer = *m_inputOutputGradientsBuffer;
	ndBrainFloatBuffer* const miniBatchOutputGradientBuffer = *m_miniBatchOutputGradientBuffer;

	ndBrainBufferCommandDesc descriptor(m_descriptor.m_minibatchSize);
	descriptor.m_context = *m_descriptor.m_context;
	descriptor.m_owner = this;
	descriptor.m_id = m_outpuId;
	descriptor.m_info = data;
	descriptor.m_uniformBuffer = uniformbuffer;
	descriptor.PushBack(*uniformbuffer);
	descriptor.PushBack(miniBatchOutputGradientBuffer);
	descriptor.PushBack(inputOutputGradientBuffer);

	if (descriptor.m_context->GetAsCpuContext())
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
				const ndCommandSharedInfo& info = m_desc.m_info;
				ndBrainTrainer* const owner = (ndBrainTrainer*)m_desc.m_owner;

				ndBrainFloat* const dstPtr = (ndBrainFloat*)owner->m_inputOutputGradientsBuffer->GetCpuPtr();
				const ndBrainFloat* const srcPtr = (ndBrainFloat*)owner->m_miniBatchOutputGradientBuffer->GetCpuPtr();
				
				ndInt32 destOffset = miniBatchIndex * info.m_inputOutputSize;
				const ndBrainMemVector src(&srcPtr[miniBatchIndex * info.m_outputSize], info.m_outputSize);
				ndBrainMemVector dst(&dstPtr[destOffset + info.m_inputOutputStartOffset], info.m_outputSize);
				dst.Set(src);
			}
		};

		ndSharedPtr<ndBrainBufferCommand>command(new ndCopyOutputGradientCommandCpu(descriptor));
		m_backPropagateCommands.Append(command);
	}
	else
	{
		descriptor.m_kernel = descriptor.m_context->GetAsGpuContext()->m_brainCopyOutputGradients;
		ndSharedPtr<ndBrainBufferCommand>command(new ndBrainGpuCommand(descriptor));
		m_backPropagateCommands.Append(command);
	}
}

void ndBrainTrainer::AddCopyInputGradientCommand()
{
	ndBrainBufferCommand* const firstCommand = FindCommand(m_inputId);
	ndAssert(firstCommand);

	ndBrainBufferCommandDesc& desc = firstCommand->GetDescriptor();
	ndCommandSharedInfo data(desc.m_info);

	data.m_outputSize = 0;
	data.m_parametersBatchSize = 0;
	data.m_parametersStartOffset = 0;
	data.m_inputOutputStartOffset = 0;
	ndSharedPtr<ndBrainUniformBuffer> uniformbuffer(new ndBrainUniformBuffer(*m_descriptor.m_context, sizeof(ndCommandSharedInfo), &data));

	ndBrainFloatBuffer* const inputOutputGradientBuffer = *m_inputOutputGradientsBuffer;
	ndBrainFloatBuffer* const miniBatchInputGradientBuffer = *m_miniBatchInputGradientBuffer;

	ndBrainBufferCommandDesc descriptor(m_descriptor.m_minibatchSize);
	descriptor.m_context = *m_descriptor.m_context;
	descriptor.m_owner = this;
	descriptor.m_id = m_inputId;
	descriptor.m_info = data;
	descriptor.m_uniformBuffer = uniformbuffer;
	descriptor.PushBack(*uniformbuffer);
	descriptor.PushBack(miniBatchInputGradientBuffer);
	descriptor.PushBack(inputOutputGradientBuffer);

	if (descriptor.m_context->GetAsCpuContext())
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
				const ndCommandSharedInfo& info = m_desc.m_info;
				ndBrainTrainer* const owner = (ndBrainTrainer*)m_desc.m_owner;

				ndBrainFloat* const dstPtr = (ndBrainFloat*)owner->m_miniBatchInputGradientBuffer->GetCpuPtr();
				const ndBrainFloat* const srcPtr = (ndBrainFloat*)owner->m_inputOutputGradientsBuffer->GetCpuPtr();

				const ndBrainMemVector src(&srcPtr[miniBatchIndex * info.m_inputOutputSize + info.m_inputOutputStartOffset], info.m_inputSize);
				ndBrainMemVector dst(&dstPtr[miniBatchIndex * info.m_inputSize], info.m_inputSize);
				dst.Set(src);
			}
		};

		ndSharedPtr<ndBrainBufferCommand>command(new ndCopyInputGradientCommandCpu(descriptor));
		m_backPropagateCommands.Append(command);
	}
	else
	{
		descriptor.m_kernel = descriptor.m_context->GetAsGpuContext()->m_brainCopyInputGradients;
		ndSharedPtr<ndBrainBufferCommand>command(new ndBrainGpuCommand(descriptor));
		m_backPropagateCommands.Append(command);
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

		const ndBrainBufferCommandDesc& desc = feedForwardLayerCommand->GetDescriptor();
		ndCommandSharedInfo info(desc.m_info);
		 
		ndBrainFloatBuffer* const inputOutputBuffer = *m_inputOutputBuffer;
		ndBrainFloatBuffer* const weightsAndBiasBuffer = *m_weightAndBiasBuffer;
		ndBrainFloatBuffer* const inputOutputGradientBuffer = *m_inputOutputGradientsBuffer;
		ndBrainFloatBuffer* const weightAndBiasGradientsBuffer = *m_weightAndBiasGradientsBuffer;

		ndCommandArray backCommands(layer->CreateGpuBackPropagateCommand(
			this, *m_descriptor.m_context, desc.m_info, m_descriptor.m_minibatchSize,
			inputOutputBuffer, weightsAndBiasBuffer, inputOutputGradientBuffer, weightAndBiasGradientsBuffer));

		for (ndInt32 j = 0; j < backCommands.GetCount(); ++j)
		{
			ndSharedPtr<ndBrainBufferCommand>command(backCommands[j]);
			m_backPropagateCommands.Append(command);
		}
	}
}

void ndBrainTrainer::AddOptimizerGradientCommand()
{
	ndBrainContext* const context = *m_descriptor.m_context;
	context->SetLeanRateCommandBuffers(**m_optimizer, m_descriptor.m_minibatchSize, **m_weightAndBiasBuffer, **m_weightAndBiasGradientsBuffer);
}

void ndBrainTrainer::ApplyLearnRate(ndBrainFloat learnRate)
{
	m_optimizer->ApplyLearnRate(learnRate);
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