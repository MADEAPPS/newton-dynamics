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

#if 0
class ndBrainAdamUpdateCommand : public ndBrainBufferCommand
{
	public:
	ndBrainAdamUpdateCommand(ndBrainTrainerInference* const owner,
		ndBrainContext* const context,
		const ndSharedPtr<ndBrainKernel>& shader,
		ndInt32 miniBatchSize,
		ndBrainOptimizerAdam::ndCommandShareInfo& info,
		const ndSharedPtr<ndBrainUniformBuffer>& uniformBuffer)
		:ndBrainBufferCommand(context, ndCommandShareInfo())
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
		ndBrainOptimizerAdam::ndCommandShareInfo& info,
		const ndSharedPtr<ndBrainUniformBuffer>& uniformBuffer,
		ndSharedPtr<ndBrainFloatBuffer>& weightAndBiasBuffer,
		ndSharedPtr<ndBrainFloatBuffer>& weightAndBiasGradientBuffer,
		ndSharedPtr<ndBrainFloatBuffer>& vdw,
		ndSharedPtr<ndBrainFloatBuffer>& vdw2)
		:ndBrainBufferCommand(context, ndCommandShareInfo())
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
	ndBrainOptimizerAdam::ndCommandShareInfo m_info;
	ndBrainTrainerInference* m_owner;
	ndInt32 m_miniBatchSize;
};
#endif

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
void ndBrainTrainer::GetGradientBuffer(ndBrainVector& output) const
{
	m_context->GetAsGpuContext()->SyncQueue();
	m_inputOutputGradientsBuffer->BrainVectorFromDevice(output);
}

void ndBrainTrainer::BackPropagate(const ndBrainVector& outputGradients)
{
	//m_miniBatchOutputGradientBuffer->LoadData(outputGradients.GetCount() * sizeof(ndReal), &outputGradients[0]);
	m_miniBatchOutputGradientBuffer->BrainVectorToDevice(outputGradients);

	for (ndList<ndSharedPtr<ndBrainBufferCommand>>::ndNode* node = m_backPropagateCommands.GetFirst(); node; node = node->GetNext())
	{
		ndSharedPtr<ndBrainBufferCommand>& command = node->GetInfo();
		m_context->GetAsGpuContext()->AddCommandQueue(command);
	}
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
	AddOptimizerGradientCommand();
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
	descritor.PushBack(*uniformbuffer);
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
	descritor.PushBack(*uniformbuffer);
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


void ndBrainTrainer::AddOptimizerGradientCommand()
{
	ndInt32 sizeInFloats = ndInt32(m_weightAndBiasBuffer->SizeInBytes() / sizeof(ndReal));
	m_optimizer->Init(sizeInFloats, m_descriptor.m_learnRate);

	ndCommandShareInfo data;
	data.m_inputSize = sizeInFloats;
	data.m_inputOutputSize = m_descriptor.m_minibatchSize;

	ndSharedPtr<ndBrainUniformBuffer> uniformbuffer(new ndBrainUniformBuffer(*m_descriptor.m_context, sizeof(ndCommandShareInfo), &data));

	ndBrainBufferCommandDesc descritor(0);
	descritor.m_context = *m_descriptor.m_context;
	descritor.m_owner = this;
	descritor.m_id = m_adamOptimizer;
	descritor.m_info = data;
	descritor.m_uniformBuffer = uniformbuffer;
	descritor.m_miniBatchSize = ndInt32(sizeInFloats / descritor.m_workGroupSize);

	descritor.PushBack(*uniformbuffer);
	descritor.PushBack(*m_weightAndBiasGradientsBuffer);

	if (m_descriptor.m_context->GetAsCpuContext())
	{
		class ndBrainAdamAddGradients : public ndBrainBufferCommandCpu
		{
			public:
			ndBrainAdamAddGradients(const ndBrainBufferCommandDesc& desc)
				:ndBrainBufferCommandCpu(desc)
			{
			}

			virtual void Execute(ndInt32 groupId) override
			{
				ndInt32 workGroupSize = m_desc.m_workGroupSize;

				const ndCommandShareInfo* const info = (ndCommandShareInfo*)m_desc[0]->GetCpuPtr();
				ndBrainFloat* const weightAndBiasGradient = (ndBrainFloat*)m_desc[1]->GetCpuPtr();

				ndInt32 inputSize = info->m_inputSize;
				ndInt32 miniBatchSize = info->m_inputOutputSize;
				ndInt32 start = groupId * workGroupSize;

				for (ndInt32 j = 1; j < miniBatchSize; ++j)
				{
					ndInt32 base = start + j * inputSize;
					for (ndInt32 itemId = 0; itemId < workGroupSize; ++itemId)
					{
						ndBrainFloat a = weightAndBiasGradient[base + itemId];
						weightAndBiasGradient[start + itemId] += a;
					}
				}
				
				ndBrainFloat weightFactor = ndBrainFloat(1.0f) / ndBrainFloat(miniBatchSize);
				for (ndInt32 itemId = 0; itemId < workGroupSize; ++itemId)
				{
					weightAndBiasGradient[start + itemId] *= weightFactor;
				}
			}
		};
		ndSharedPtr<ndBrainBufferCommand> accumulateGradients(new ndBrainAdamAddGradients(descritor));
		m_optimizerBufferCommands.Append(accumulateGradients);


		// add the adam optimizer kernel here
		ndBrainOptimizerAdam::ndCommandShareInfo optimizerData(m_optimizer->m_parameters);
		ndSharedPtr<ndBrainUniformBuffer> adamUniformbuffer(new ndBrainUniformBuffer(*m_descriptor.m_context, sizeof(ndBrainOptimizerAdam::ndCommandShareInfo), &optimizerData));

		if (m_optimizer->GetRegularizerType() != m_lasso)
		{
			ndBrainBufferCommandDesc updateGradientDescritor(0);
			updateGradientDescritor.m_context = *m_descriptor.m_context;
			updateGradientDescritor.m_owner = this;
			updateGradientDescritor.m_id = m_adamOptimizer;
			//updateGradientDescritor.m_info = data;
			updateGradientDescritor.m_uniformBuffer = adamUniformbuffer;
			updateGradientDescritor.m_miniBatchSize = ndInt32(sizeInFloats / updateGradientDescritor.m_workGroupSize);

			updateGradientDescritor.PushBack(*adamUniformbuffer);
			updateGradientDescritor.PushBack(*m_weightAndBiasBuffer);
			updateGradientDescritor.PushBack(*m_weightAndBiasGradientsBuffer);
			updateGradientDescritor.PushBack(*m_optimizer->m_vdw);
			updateGradientDescritor.PushBack(*m_optimizer->m_vdw2);

			class ndBrainAdamUpdateParametersRidge : public ndBrainBufferCommandCpu
			{
				public:
				ndBrainAdamUpdateParametersRidge(const ndBrainBufferCommandDesc& desc)
					:ndBrainBufferCommandCpu(desc)
				{
				}

				virtual void Execute(ndInt32 groupId) override
				{
					ndInt32 workGroupSize = m_desc.m_workGroupSize;

					const ndBrainOptimizerAdam::ndCommandShareInfo* const parameters = (ndBrainOptimizerAdam::ndCommandShareInfo*)m_desc[0]->GetCpuPtr();
					ndBrainFloat* const weightAndBiasBuffer = (ndBrainFloat*)m_desc[1]->GetCpuPtr();
					ndBrainFloat* const weightAndBiasGradientBuffer = (ndBrainFloat*)m_desc[2]->GetCpuPtr();
					ndBrainFloat* const vdw = (ndBrainFloat*)m_desc[3]->GetCpuPtr();
					ndBrainFloat* const vdw2 = (ndBrainFloat*)m_desc[4]->GetCpuPtr();
					
					ndBrainFloat descendRate = -parameters->m_learnRate;
					ndBrainFloat regularizer = -parameters->m_decayRegularizer;
					
					ndInt32 start = groupId * workGroupSize;
					for (ndInt32 itemId = 0; itemId < workGroupSize; ++itemId)
					{
						ndBrainFloat temp = weightAndBiasGradientBuffer[start + itemId];

						// calculate moving average
						ndBrainFloat a = vdw[start + itemId] * parameters->m_alpha + temp * (ndBrainFloat(1.0f) - parameters->m_alpha);
						vdw[start + itemId] = a;

						// caluate RMS
						ndBrainFloat b = vdw2[start + itemId] * parameters->m_beta + temp * temp * (ndBrainFloat(1.0f) - parameters->m_beta);
						vdw2[start + itemId] = b;

						ndBrainFloat vdwCorrected = a * parameters->m_invAlpha;
						ndBrainFloat vdw2Corrected = b * parameters->m_invBeta;

						if (vdw2Corrected == 0.0f)
						{
							vdw2Corrected *= 1;
						}

						ndBrainFloat bias_den = ndBrainFloat(1.0f) / (ndBrainFloat(ndSqrt(vdw2Corrected)) + parameters->m_epsilon);
						ndBrainFloat gradient = vdwCorrected * bias_den;

						ndBrainFloat weight = weightAndBiasBuffer[start + itemId];
						gradient += weight * regularizer;
						weightAndBiasBuffer[start + itemId] = weight + gradient * descendRate;
					}
				}
			};

			//m_adamOtimizerUpdate = ndSharedPtr<ndBrainBufferCommand>(new ndBrainAdamUpdateCommand(
			//	this, m_context->GetAsGpuContext(), m_context->GetAsGpuContext()->m_brainAdamRidgeOptimizerUpdate, 
			//  m_miniBatchSize, optimizerData,	adamUniformbuffer, 
			//  m_weightAndBiasBuffer, m_weightAndBiasGradientsBuffer,
			//	m_optimizer->m_vdw, m_optimizer->m_vdw2));
			ndSharedPtr<ndBrainBufferCommand> updateParameters(new ndBrainAdamUpdateParametersRidge(updateGradientDescritor));
			m_optimizerBufferCommands.Append(updateParameters);
		}
		else
		{
			ndAssert(0);
			//m_adamOtimizerUpdate = ndSharedPtr<ndBrainBufferCommand>(new ndBrainAdamUpdateCommand(
			//	this, m_context->GetAsGpuContext(), m_context->GetAsGpuContext()->m_brainAdamLassoOptimizerUpdate, m_miniBatchSize, optimizerData,
			//	adamUniformbuffer, m_weightAndBiasBuffer, m_weightAndBiasGradientsBuffer,
			//	m_optimizer->m_vdw, m_optimizer->m_vdw2));
		}

		//const ndSharedPtr<ndBrainBufferCommand>& gradientsData = m_optimizerBufferCommands.GetFirst()->GetInfo();
		//m_adamOtimizerUpdate->m_numberOfWorkGroups = size_t(sizeInFloats / gradientsData->m_workGroupSize);

		class ndBrainAdamMomentumUpdate : public ndBrainBufferCommandCpu
		{
			public:
			ndBrainAdamMomentumUpdate(const ndBrainBufferCommandDesc& desc)
				:ndBrainBufferCommandCpu(desc)
			{
			}

			virtual void Execute(ndInt32) override
			{
				ndBrainOptimizerAdam::ndCommandShareInfo* const parameters = (ndBrainOptimizerAdam::ndCommandShareInfo*)m_desc[0]->GetCpuPtr();

				parameters->m_betaAcc *= parameters->m_beta;
				parameters->m_alphaAcc *= parameters->m_alpha;
				if (parameters->m_betaAcc < ndBrainFloat(1.0e-7f))
				{
					parameters->m_betaAcc = ndBrainFloat(0.0f);
				}
				if (parameters->m_alphaAcc < ndBrainFloat(1.0e-7f))
				{
					parameters->m_alphaAcc = ndBrainFloat(0.0f);
				}
			}
		};

		ndBrainBufferCommandDesc momentumUpdateDescritor(0);
		momentumUpdateDescritor.m_context = *m_descriptor.m_context;
		momentumUpdateDescritor.m_owner = this;
		momentumUpdateDescritor.m_id = m_adamOptimizer;
		//momentumUpdateDescritor.m_info = data;
		momentumUpdateDescritor.m_uniformBuffer = adamUniformbuffer;
		momentumUpdateDescritor.m_miniBatchSize = 1;
		momentumUpdateDescritor.PushBack(*adamUniformbuffer);

		// add the momentum update command
		//m_adamMomentumUpdate = ndSharedPtr<ndBrainBufferCommand>(new ndBrainAdamUpdateCommand(
		//	this, m_context->GetAsGpuContext(), m_context->GetAsGpuContext()->m_brainAdamMomentumUpdate, 1, optimizerData, adamUniformbuffer));
		//m_adamMomentumUpdate->m_numberOfWorkGroups = 1;

		ndSharedPtr<ndBrainBufferCommand> momentumUpdate(new ndBrainAdamMomentumUpdate(momentumUpdateDescritor));
		m_optimizerBufferCommands.Append(momentumUpdate);
	}
	else
	{
		ndAssert(0);
	}
}

void ndBrainTrainer::ApplyLearnRate()
{
	ndBrainContext* const context = *m_descriptor.m_context;
	for (ndList<ndSharedPtr<ndBrainBufferCommand>>::ndNode* node = m_optimizerBufferCommands.GetFirst(); node; node = node->GetNext())
	{
		ndSharedPtr<ndBrainBufferCommand>& command = node->GetInfo();
		context->SubmitBufferCommand(*command);
	}
}
