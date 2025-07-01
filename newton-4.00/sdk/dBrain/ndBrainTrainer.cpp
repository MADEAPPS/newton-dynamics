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
		ndBrainOptimizerAdam::ndCommandSharedInfo& info,
		const ndSharedPtr<ndBrainUniformBuffer>& uniformBuffer)
		:ndBrainBufferCommand(context, ndCommandSharedInfo())
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
		ndBrainOptimizerAdam::ndCommandSharedInfo& info,
		const ndSharedPtr<ndBrainUniformBuffer>& uniformBuffer,
		ndSharedPtr<ndBrainFloatBuffer>& weightAndBiasBuffer,
		ndSharedPtr<ndBrainFloatBuffer>& weightAndBiasGradientBuffer,
		ndSharedPtr<ndBrainFloatBuffer>& vdw,
		ndSharedPtr<ndBrainFloatBuffer>& vdw2)
		:ndBrainBufferCommand(context, ndCommandSharedInfo())
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
	ndBrainOptimizerAdam::ndCommandSharedInfo m_info;
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

	ndCommandSharedInfo data(desc.m_info);
	data.m_inputSize = 0;
	data.m_parametersBatchSize = 0;
	data.m_parametersStartOffset = 0;
	data.m_inputOutputStartOffset += RoundoffOffset(data.m_inputSize);
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
		ndBrainBufferCommandDesc& desc = feedForwardLayerCommand->GetDescriptor();

		ndCommandSharedInfo info(desc.m_info);
		ndSharedPtr<ndBrainUniformBuffer> uniformBuffer(new ndBrainUniformBuffer(*m_descriptor.m_context, sizeof(ndCommandSharedInfo), &info));

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

	// add summation kernel
	{
		ndCommandSharedInfo data;
		data.m_inputSize = sizeInFloats;
		data.m_inputOutputSize = m_descriptor.m_minibatchSize;

		ndSharedPtr<ndBrainUniformBuffer> uniformbuffer(new ndBrainUniformBuffer(*m_descriptor.m_context, sizeof(ndCommandSharedInfo), &data));

		ndBrainBufferCommandDesc descriptor(0);
		descriptor.m_context = *m_descriptor.m_context;
		descriptor.m_owner = this;
		descriptor.m_id = m_adamOptimizerSum;
		descriptor.m_info = data;
		descriptor.m_uniformBuffer = uniformbuffer;
		descriptor.m_miniBatchSize = ndInt32(sizeInFloats / descriptor.m_workGroupSize);
		descriptor.PushBack(*uniformbuffer);
		descriptor.PushBack(*m_weightAndBiasGradientsBuffer);

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

					const ndCommandSharedInfo* const info = (ndCommandSharedInfo*)m_desc[0]->GetCpuPtr();
					ndBrainFloat* const weightAndBiasGradient = (ndBrainFloat*)m_desc[1]->GetCpuPtr();

					ndInt32 inputSize = info->m_inputSize;
					ndInt32 miniBatchSize = info->m_inputOutputSize;
					ndInt32 start = groupId * workGroupSize;

					for (ndInt32 j = 1; j < miniBatchSize; ++j)
					{
						ndInt32 base = start + j * inputSize;
						ndAssert(base > 0);
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
			ndSharedPtr<ndBrainBufferCommand> accumulateGradients(new ndBrainAdamAddGradients(descriptor));
			m_optimizerBufferCommands.Append(accumulateGradients);
		} 
		else
		{
			descriptor.m_kernel = descriptor.m_context->GetAsGpuContext()->m_brainAccumulateGradientsAndAverage;
			ndSharedPtr<ndBrainBufferCommand>command(new ndBrainGpuCommand(descriptor));
			m_optimizerBufferCommands.Append(command);
		}
	}

	// add the adam optimizer kernel here
	{
		ndBrainOptimizerAdam::ndCommandSharedInfo optimizerData(m_optimizer->m_parameters);
		ndSharedPtr<ndBrainUniformBuffer> adamUniformbuffer(new ndBrainUniformBuffer(*m_descriptor.m_context, sizeof(ndBrainOptimizerAdam::ndCommandSharedInfo), &optimizerData));

		ndBrainBufferCommandDesc descriptor(0);
		descriptor.m_context = *m_descriptor.m_context;
		descriptor.m_owner = this;
		descriptor.m_id = m_adamOptimizerUpdate;
		descriptor.m_uniformBuffer = adamUniformbuffer;
		descriptor.m_miniBatchSize = ndInt32(sizeInFloats / descriptor.m_workGroupSize);

		descriptor.PushBack(*adamUniformbuffer);
		descriptor.PushBack(*m_weightAndBiasBuffer);
		descriptor.PushBack(*m_weightAndBiasGradientsBuffer);
		descriptor.PushBack(*m_optimizer->m_vdw);
		descriptor.PushBack(*m_optimizer->m_vdw2);

		if (m_descriptor.m_context->GetAsCpuContext())
		{
			if (m_optimizer->GetRegularizerType() != m_lasso)
			{
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

						const ndBrainOptimizerAdam::ndCommandSharedInfo* const parameters = (ndBrainOptimizerAdam::ndCommandSharedInfo*)m_desc[0]->GetCpuPtr();
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

							ndBrainFloat bias_den = ndBrainFloat(1.0f) / (ndBrainFloat(ndSqrt(vdw2Corrected)) + parameters->m_epsilon);
							ndBrainFloat gradient = vdwCorrected * bias_den;

							ndBrainFloat weight = weightAndBiasBuffer[start + itemId];
							gradient += weight * regularizer;
							weightAndBiasBuffer[start + itemId] = weight + gradient * descendRate;
						}
					}
				};

				ndSharedPtr<ndBrainBufferCommand> updateParameters(new ndBrainAdamUpdateParametersRidge(descriptor));
				m_optimizerBufferCommands.Append(updateParameters);
			}
			else
			{
				ndAssert(0);
			}
		}
		else
		{
			if (m_optimizer->GetRegularizerType() != m_lasso)
			{
				descriptor.m_kernel = descriptor.m_context->GetAsGpuContext()->m_brainAdamRidgeOptimizerUpdate;
			}
			else
			{
				descriptor.m_kernel = descriptor.m_context->GetAsGpuContext()->m_brainAdamLassoOptimizerUpdate;
			}
			ndSharedPtr<ndBrainBufferCommand>command(new ndBrainGpuCommand(descriptor));
			m_optimizerBufferCommands.Append(command);
		}
	}

	// add adam momentum update
	{
		ndBrainOptimizerAdam::ndCommandSharedInfo optimizerData(m_optimizer->m_parameters);
		ndSharedPtr<ndBrainUniformBuffer> adamUniformbuffer(new ndBrainUniformBuffer(*m_descriptor.m_context, sizeof(ndBrainOptimizerAdam::ndCommandSharedInfo), &optimizerData));

		ndBrainBufferCommandDesc descriptor(0);
		descriptor.m_context = *m_descriptor.m_context;
		descriptor.m_owner = this;
		descriptor.m_id = m_adamOptimizerMomentum;
		descriptor.m_uniformBuffer = adamUniformbuffer;
		descriptor.m_miniBatchSize = 1;
		descriptor.PushBack(*adamUniformbuffer);

		if (m_descriptor.m_context->GetAsCpuContext())
		{
			class ndBrainAdamMomentumUpdate : public ndBrainBufferCommandCpu
			{
				public:
				ndBrainAdamMomentumUpdate(const ndBrainBufferCommandDesc& desc)
					:ndBrainBufferCommandCpu(desc)
				{
				}

				virtual void Execute(ndInt32) override
				{
					ndBrainOptimizerAdam::ndCommandSharedInfo* const parameters = (ndBrainOptimizerAdam::ndCommandSharedInfo*)m_desc[0]->GetCpuPtr();

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

			ndSharedPtr<ndBrainBufferCommand> momentumUpdate(new ndBrainAdamMomentumUpdate(descriptor));
			m_optimizerBufferCommands.Append(momentumUpdate);
		}
		else
		{
			descriptor.m_kernel = descriptor.m_context->GetAsGpuContext()->m_brainAdamMomentumUpdate;
			ndSharedPtr<ndBrainBufferCommand>command(new ndBrainGpuCommand(descriptor));
			m_optimizerBufferCommands.Append(command);
		}
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
