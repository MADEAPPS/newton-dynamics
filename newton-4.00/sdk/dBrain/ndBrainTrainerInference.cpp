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
#include "ndBrainGpuCommand.h"
#include "ndBrainCpuContext.h"
#include "ndBrainGpuContext.h"
#include "ndBrainLayerLinear.h"
#include "ndBrainFloatBuffer.h"
#include "ndBrainUniformBuffer.h"
#include "ndBrainTrainerInference.h"

ndTrainerDescriptor::ndTrainerDescriptor()
	:m_brain()
	,m_context()
	,m_learnRate(ndBrainFloat(1.0e-4f))
	,m_regularizer(ndBrainFloat(1.0e-4f))
	,m_minibatchSize(256)
	,m_regularizerType(m_ridge)
{
}

ndTrainerDescriptor::ndTrainerDescriptor(const ndSharedPtr<ndBrain>& brain, const ndSharedPtr<ndBrainContext>& context, ndInt32 minibatchSize, ndBrainFloat learnRate)
	:m_brain(brain)
	,m_context(context)
	,m_learnRate(learnRate)
	,m_regularizer(ndBrainFloat(1.0e-4f))
	,m_minibatchSize(minibatchSize)
	,m_regularizerType(m_ridge)
{
}

ndBrainTrainerInference::ndBrainTrainerInference(const ndTrainerDescriptor& descriptor)
	:ndClassAlloc()
	,m_descriptor(descriptor)
	,m_inputOutputBuffer()
	,m_weightAndBiasBuffer()
	,m_miniBatchInputBuffer()
	,m_miniBatchOutputBuffer()
	,m_feedForwardCommands()
{
	ndAssert((m_descriptor.m_minibatchSize & (ND_GPU_TILED_MATRIX_ROWS - 1)) == 0);
	InitInputOutputBuffer();
	InitWeightAndBiasBuffer();
}

ndBrainTrainerInference::ndBrainTrainerInference(const ndBrainTrainerInference& src)
	:ndClassAlloc()
	,m_descriptor(src.m_descriptor)
	,m_inputOutputBuffer()
	,m_weightAndBiasBuffer()
	,m_miniBatchInputBuffer()
	,m_miniBatchOutputBuffer()
	,m_feedForwardCommands()
{
	ndAssert(0);
}

ndBrainTrainerInference::~ndBrainTrainerInference()
{
}

ndInt32 ndBrainTrainerInference::RoundoffOffset(ndInt32 value) const
{
	return m_descriptor.m_context->GetAsGpuContext() ? (value + ND_DEFAULT_WORKGROUP_SIZE - 1) & -ND_DEFAULT_WORKGROUP_SIZE : value;
}

ndSharedPtr<ndBrain>& ndBrainTrainerInference::GetBrain()
{
	return m_descriptor.m_brain;
}

ndSharedPtr<ndBrainContext> ndBrainTrainerInference::GetContext()
{
	return m_descriptor.m_context;
}

ndBrainFloatBuffer* ndBrainTrainerInference::GetInputBuffer()
{
	return *m_miniBatchInputBuffer;
}

ndBrainFloatBuffer* ndBrainTrainerInference::GetOuputBuffer()
{
	return *m_miniBatchOutputBuffer;
}

ndBrainFloatBuffer* ndBrainTrainerInference::GetHiddenLayerBuffer()
{
	return *m_inputOutputBuffer;
}

ndBrainFloatBuffer* ndBrainTrainerInference::GetWeightAndBiasBuffer()
{
	return *m_weightAndBiasBuffer;
}

void ndBrainTrainerInference::InitInputOutputBuffer()
{
	const ndBrain& brain = **m_descriptor.m_brain;
	ndInt64 bufferSize = 0;
	if (m_descriptor.m_context->GetAsGpuContext())
	{
		bufferSize += RoundoffOffset(brain.GetInputSize());
		for (ndInt32 i = 0; i < ndInt32(brain.GetCount()); ++i)
		{
			const ndBrainLayer* const layer = brain[i];
			bufferSize += RoundoffOffset(layer->GetOutputSize());
		}
	}
	else
	{
		bufferSize += brain.GetInputSize();
		for (ndInt32 i = 0; i < ndInt32(brain.GetCount()); ++i)
		{
			const ndBrainLayer* const layer = brain[i];
			bufferSize += layer->GetOutputSize();
		}
	}
	
	ndBrainVector buffer;
	buffer.Resize(bufferSize * m_descriptor.m_minibatchSize);
	buffer.SetCount(bufferSize * m_descriptor.m_minibatchSize);
	buffer.Set(ndBrainFloat(0.0f));
	m_inputOutputBuffer = ndSharedPtr<ndBrainFloatBuffer>(new ndBrainFloatBuffer(*m_descriptor.m_context, buffer));
	
	//ndCopyBufferCommandInfo copyBufferInfo;
	//copyBufferInfo.m_dstOffsetInByte = 0;
	//copyBufferInfo.m_srcOffsetInByte = 0;
	//copyBufferInfo.m_strideInByte = ndInt32(brain.GetInputSize() * sizeof(ndInt32));
	//copyBufferInfo.m_srcStrideInByte = copyBufferInfo.m_strideInByte;
	//copyBufferInfo.m_dstStrideInByte = copyBufferInfo.m_strideInByte * m_descriptor.m_minibatchSize;
	//m_singlePredictionInputBufferParameters = ndSharedPtr<ndBrainUniformBuffer>(new ndBrainUniformBuffer(*m_descriptor.m_context, ndInt64(sizeof(ndCopyBufferCommandInfo)), &copyBufferInfo));
	//
	//copyBufferInfo.m_strideInByte = ndInt32(brain.GetOutputSize() * sizeof(ndInt32));
	//copyBufferInfo.m_srcStrideInByte = copyBufferInfo.m_strideInByte;
	//copyBufferInfo.m_dstStrideInByte = copyBufferInfo.m_strideInByte * m_descriptor.m_minibatchSize;
	//m_singlePredictionOutputBufferParameters = ndSharedPtr<ndBrainUniformBuffer>(new ndBrainUniformBuffer(*m_descriptor.m_context, ndInt64(sizeof(ndCopyBufferCommandInfo)), &copyBufferInfo));
	//
	//m_singlePredictionInputBuffer = ndSharedPtr<ndBrainFloatBuffer>(new ndBrainFloatBuffer(*m_descriptor.m_context, ndInt64(brain.GetInputSize()), true));
	//m_singlePredictionOutputBuffer = ndSharedPtr<ndBrainFloatBuffer>(new ndBrainFloatBuffer(*m_descriptor.m_context, ndInt64(brain.GetOutputSize()), true));
}	

void ndBrainTrainerInference::InitWeightAndBiasBuffer()
{
	const ndBrain& brain = **m_descriptor.m_brain;

	ndFixSizeArray<ndCommandSharedInfo, 256> uniformData;
	bool isCpu = m_descriptor.m_context->GetAsCpuContext() ? true : false;
	for (ndInt32 i = 0; i < ndInt32(brain.GetCount()); ++i)
	{
		ndBrainLayer* const layer = brain[i];
		ndCommandSharedInfo info(isCpu ? layer->GetCpuCommandSharedInfo(): layer->GetGpuCommandSharedInfo());
		info.m_parametersBatchSize = RoundoffOffset(info.m_parametersBatchSize);
		uniformData.PushBack(info);
	}
	uniformData.PushBack(ndCommandSharedInfo(nullptr));

	ndInt32 parametersSizeSum = 0;
	for (ndInt32 i = 0; i < uniformData.GetCount(); ++i)
	{
		uniformData[i].m_parametersStartOffset = parametersSizeSum;
		parametersSizeSum += uniformData[i].m_parametersBatchSize;
	}
	parametersSizeSum += ND_DEFAULT_WORKGROUP_SIZE;
	parametersSizeSum = (parametersSizeSum + ND_DEFAULT_WORKGROUP_SIZE - 1) & -ND_DEFAULT_WORKGROUP_SIZE;
	ndAssert((parametersSizeSum & (ND_DEFAULT_WORKGROUP_SIZE - 1)) == 0);

	ndBrainVector scratchBuffer;
	scratchBuffer.Resize(parametersSizeSum);
	scratchBuffer.SetCount(parametersSizeSum);
	scratchBuffer.Set(ndBrainFloat(0.0f));

	for (ndInt32 i = 0; i < ndInt32(brain.GetCount()); ++i)
	{
		const ndBrainLayer* const layer = brain[i];
		ndCommandSharedInfo& info = uniformData[i];

		ndInt32 size = uniformData[i].m_parametersBatchSize;
		if (size)
		{
			ndInt32 offset = uniformData[i].m_parametersStartOffset;
			ndBrainMemVector weights(&scratchBuffer[offset], size);
			isCpu ? layer->CopyCpuWeights(weights) : layer->CopyGpuWeights(weights);
			info.m_parametersBatchSize = parametersSizeSum;
		}
	}
	uniformData[uniformData.GetCount() - 1].m_parametersBatchSize = parametersSizeSum;
	m_weightAndBiasBuffer = ndSharedPtr<ndBrainFloatBuffer>(new ndBrainFloatBuffer(*m_descriptor.m_context, scratchBuffer));

	scratchBuffer.SetCount(m_descriptor.m_minibatchSize * brain.GetInputSize());
	scratchBuffer.Set(ndBrainFloat(0.0f));
	m_miniBatchInputBuffer = ndSharedPtr<ndBrainFloatBuffer>(new ndBrainFloatBuffer(*m_descriptor.m_context, scratchBuffer));

	scratchBuffer.SetCount(m_descriptor.m_minibatchSize * brain.GetOutputSize());
	scratchBuffer.Set(ndBrainFloat(0.0f));
	m_miniBatchOutputBuffer = ndSharedPtr<ndBrainFloatBuffer>(new ndBrainFloatBuffer(*m_descriptor.m_context, scratchBuffer));

	AddCopyInputCommand(uniformData[0]);
	AddLayersCommands(uniformData);
	AddCopyOutputCommand();
}

ndBrainBufferCommand* ndBrainTrainerInference::FindCommand(size_t id) const
{
	for (ndList<ndSharedPtr<ndBrainBufferCommand>>::ndNode* node = m_feedForwardCommands.GetFirst(); node; node = node->GetNext())
	{
		ndBrainBufferCommand* const command = (ndBrainBufferCommand*)*node->GetInfo();
		const ndBrainBufferCommandDesc& desc = command->GetDescriptor();
		if (desc.m_id == id)
		{
			return command;
		}
	}
	return nullptr;
}

void ndBrainTrainerInference::AddCopyInputCommand(const ndCommandSharedInfo& uniformData)
{
	const ndBrain& brain = **m_descriptor.m_brain;
	ndInt32 inputOutputBufferSize = RoundoffOffset(brain.GetInputSize());
	for (ndInt32 i = 0; i < ndInt32(brain.GetCount()); ++i)
	{
		const ndBrainLayer* const layer = brain[i];
		inputOutputBufferSize += RoundoffOffset(layer->GetOutputSize());
	}
	
	ndCommandSharedInfo uniformParam;
	uniformParam.m_inputSize = uniformData.m_inputSize;
	uniformParam.m_outputSize = uniformData.m_outputSize;
	uniformParam.m_parametersStartOffset = 0;
	uniformParam.m_inputOutputSize = inputOutputBufferSize;
	uniformParam.m_inputOutputStartOffset = 0;

	ndSharedPtr<ndBrainUniformBuffer> uniformbuffer(new ndBrainUniformBuffer(*m_descriptor.m_context, sizeof(ndCommandSharedInfo), &uniformParam));
	
	ndBrainFloatBuffer* const inputOutputBuffer = *m_inputOutputBuffer;
	ndBrainFloatBuffer* const miniBatchInputBuffer = *m_miniBatchInputBuffer;

	ndBrainBufferCommandDesc descriptor(m_descriptor.m_minibatchSize);
	descriptor.m_context = *m_descriptor.m_context;
	descriptor.m_owner = this;
	descriptor.m_id = m_inputId;
	descriptor.m_info = uniformParam;
	descriptor.m_uniformBuffer = uniformbuffer;
	descriptor.PushBack(*uniformbuffer);
	descriptor.PushBack(inputOutputBuffer);
	descriptor.PushBack(miniBatchInputBuffer);

	if (descriptor.m_context->GetAsCpuContext())
	{
		class ndCopyInputCommandCpu : public ndBrainBufferCommandCpu
		{
			public:
			ndCopyInputCommandCpu(const ndBrainBufferCommandDesc& desc)
				:ndBrainBufferCommandCpu(desc)
			{
			}

			virtual void Execute(ndInt32 groupId) override
			{
				const ndCommandSharedInfo& info = m_desc.m_info;
				ndBrainTrainerInference* const owner = m_desc.m_owner;

				ndBrainFloat* const dstPtr = (ndBrainFloat*)owner->m_inputOutputBuffer->GetCpuPtr();
				const ndBrainFloat* const srcPtr = (ndBrainFloat*)owner->m_miniBatchInputBuffer->GetCpuPtr();

				ndAssert(groupId * info.m_inputSize >= 0);
				ndAssert(groupId * info.m_inputOutputSize >= 0);
				const ndBrainMemVector src(&srcPtr[groupId * info.m_inputSize], info.m_inputSize);
				ndBrainMemVector dst(&dstPtr[groupId * info.m_inputOutputSize + info.m_inputOutputStartOffset], info.m_inputSize);
				dst.Set(src);
			}
		};

		ndSharedPtr<ndBrainBufferCommand>command(new ndCopyInputCommandCpu(descriptor));
		m_feedForwardCommands.Append(command);
	}
	else
	{
		descriptor.m_kernel = descriptor.m_context->GetAsGpuContext()->m_brainCopyInput;
		ndSharedPtr<ndBrainBufferCommand>command(new ndBrainGpuCommand(descriptor));
		m_feedForwardCommands.Append(command);
	}
}

void ndBrainTrainerInference::AddCopyOutputCommand()
{
	const ndBrain& brain = **m_descriptor.m_brain;
	size_t lastId = size_t(brain[brain.GetCount() - 1]);
	ndBrainBufferCommand* const lastLayerCommand = FindCommand(lastId);
	ndAssert(lastLayerCommand);
	
	const ndBrainBufferCommandDesc& desc = lastLayerCommand->GetDescriptor();
	ndCommandSharedInfo uniformParam(desc.m_info);
	
	uniformParam.m_parametersStartOffset = 0;
	uniformParam.m_inputOutputStartOffset += RoundoffOffset(uniformParam.m_inputSize);
	ndSharedPtr<ndBrainUniformBuffer> uniformbuffer(new ndBrainUniformBuffer(*m_descriptor.m_context, sizeof(ndCommandSharedInfo), &uniformParam));
	
	ndBrainFloatBuffer* const inputOutputBuffer = *m_inputOutputBuffer;
	ndBrainFloatBuffer* const miniBatchOutputBuffer = *m_miniBatchOutputBuffer;
	
	ndBrainBufferCommandDesc descriptor(m_descriptor.m_minibatchSize);
	descriptor.m_context = *m_descriptor.m_context;
	descriptor.m_owner = this;
	descriptor.m_id = m_outpuId;
	descriptor.m_info = uniformParam;
	descriptor.m_uniformBuffer = uniformbuffer;
	descriptor.PushBack(*uniformbuffer);
	descriptor.PushBack(inputOutputBuffer);
	descriptor.PushBack(miniBatchOutputBuffer);
	
	if (descriptor.m_context->GetAsCpuContext())
	{
		class ndCopyOutputCommandCpu : public ndBrainBufferCommandCpu
		{
			public:
			ndCopyOutputCommandCpu(const ndBrainBufferCommandDesc& desc)
				:ndBrainBufferCommandCpu(desc)
			{
			}

			virtual void Execute(ndInt32 groupId) override
			{
				const ndCommandSharedInfo& info = m_desc.m_info;
				ndBrainTrainerInference* const owner = m_desc.m_owner;

				ndBrainFloat* const dstPtr = (ndBrainFloat*)owner->m_miniBatchOutputBuffer->GetCpuPtr();
				const ndBrainFloat* const srcPtr = (ndBrainFloat*)owner->m_inputOutputBuffer->GetCpuPtr();

				ndAssert(groupId * info.m_outputSize >= 0);
				ndAssert(groupId * info.m_inputOutputSize >= 0);

				const ndBrainMemVector src(&srcPtr[groupId * info.m_inputOutputSize + info.m_inputOutputStartOffset], info.m_outputSize);
				ndBrainMemVector dst(&dstPtr[groupId * info.m_outputSize], info.m_outputSize);
				dst.Set(src);
			}
		};

		ndSharedPtr<ndBrainBufferCommand>command(new ndCopyOutputCommandCpu(descriptor));
		m_feedForwardCommands.Append(command);
	}
	else
	{
		descriptor.m_kernel = descriptor.m_context->GetAsGpuContext()->m_brainCopyOutput;
		ndSharedPtr<ndBrainBufferCommand>command(new ndBrainGpuCommand(descriptor));
		m_feedForwardCommands.Append(command);
	}
}

void ndBrainTrainerInference::UpdateParameters(const ndBrainVector& weightAndBias)
{
	bool isCpu = m_descriptor.m_context->GetAsCpuContext() ? true : false;
	for (ndList<ndSharedPtr<ndBrainBufferCommand>>::ndNode* node = m_feedForwardCommands.GetFirst(); node; node = node->GetNext())
	{
		ndBrainBufferCommand* const command = *node->GetInfo();
		const ndBrainBufferCommandDesc& desc = command->GetDescriptor();
		const ndCommandSharedInfo& info = desc.m_info;
		ndBrainLayer* const layer = (ndBrainLayer*)info.m_layer;
		if (layer)
		{
			ndInt32 width;
			ndInt32 height;
			ndBrainLayerLinear* const linearLayer = (ndBrainLayerLinear*)layer;
			linearLayer->CalculateRoundedSize(width, height);
			ndInt32 size = width * height + info.m_outputSize;
			ndAssert(size >= 0);
			const ndBrainMemVector weights(&weightAndBias[info.m_parametersStartOffset], size);
			isCpu ? layer->SetCpuWeights(weights) : layer->SetGpuWeights(weights);
		}
	}
}

void ndBrainTrainerInference::AddLayersCommands(ndFixSizeArray<ndCommandSharedInfo, 256>& layersUniformsData)
{
	// create all the uniform buffers 
	const ndBrain& brain = **m_descriptor.m_brain;

	ndInt32 inputOutputStartOffset = 0;
	ndInt32 inputOutputBufferSize = RoundoffOffset(brain.GetInputSize());
	for (ndInt32 i = 0; i < ndInt32(brain.GetCount()); ++i)
	{
		ndCommandSharedInfo& data = layersUniformsData[i];
		data.m_inputOutputStartOffset = inputOutputStartOffset;
		inputOutputBufferSize += RoundoffOffset(data.m_outputSize);
		inputOutputStartOffset += RoundoffOffset(data.m_inputSize);
	}

	ndBrainFloatBuffer* const weightsBuffer = *m_weightAndBiasBuffer;
	ndBrainFloatBuffer* const inputOutputBuffer = *m_inputOutputBuffer;
	for (ndInt32 i = 0; i < ndInt32(brain.GetCount()); ++i)
	{
		ndCommandSharedInfo uniformParam;

		ndBrainLayer* const layer = brain[i];
		const ndCommandSharedInfo& data = layersUniformsData[i];
		uniformParam.m_inputSize = data.m_inputSize;
		uniformParam.m_outputSize = data.m_outputSize;
		uniformParam.m_inputOutputSize = inputOutputBufferSize;
		uniformParam.m_inputOutputStartOffset = data.m_inputOutputStartOffset;
		uniformParam.m_parametersBatchSize = data.m_parametersBatchSize;
		uniformParam.m_parametersStartOffset = data.m_parametersStartOffset;
		uniformParam.m_layer = layer;
		ndSharedPtr<ndBrainUniformBuffer> uniformbuffer(new ndBrainUniformBuffer(*m_descriptor.m_context, sizeof(ndCommandSharedInfo), &uniformParam));

		ndSharedPtr<ndBrainBufferCommand> command(layer->CreateGpuFeedForwardCommand(this, uniformParam, *m_descriptor.m_context, m_descriptor.m_minibatchSize, uniformbuffer, inputOutputBuffer, weightsBuffer));
		m_feedForwardCommands.Append(command);
	}
}

void ndBrainTrainerInference::MakePrediction()
{
	ndBrainContext* const context = *m_descriptor.m_context;

	for (ndList<ndSharedPtr<ndBrainBufferCommand>>::ndNode* node = m_feedForwardCommands.GetFirst(); node; node = node->GetNext())
	{
		ndSharedPtr<ndBrainBufferCommand>& command = node->GetInfo();
		context->SubmitBufferCommand(*command);
	}
}
