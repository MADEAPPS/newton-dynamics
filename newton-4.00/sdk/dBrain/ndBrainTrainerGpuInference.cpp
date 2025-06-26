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
#include "ndBrainLayerLinear.h"
#include "ndBrainGpuFloatBuffer.h"
#include "ndBrainGpuUniformBuffer.h"
#include "ndBrainTrainerGpuInference.h"

ndBrainTrainerGpuCommand::ndBrainTrainerGpuCommand(
	ndBrainTrainerGpuInference* const owner,
	const ndBrainLayer::ndCommandShareInfo& info, size_t id,
	ndBrainGpuContext* const context,
	const ndSharedPtr<ndBrainGpuShader>& shader,
	ndInt32 numberOfinputs,
	const ndSharedPtr<ndBrainGpuUniformBuffer>& uniformBuffer,
	ndBrainGpuFloatBuffer* const inputOutputData,
	ndBrainGpuFloatBuffer* const parameters,
	ndBrainGpuFloatBuffer* const inputOutputGradients,
	ndBrainGpuFloatBuffer* const weightsAndBiasGradients)
	:ndBrainGpuCommand(context, info)
	,m_uniformBuffer(uniformBuffer)
	,m_owner(owner)
	,m_id(id)
{
	ndFixSizeArray<ndBrainBuffer*, 8> params;
	params.PushBack(*m_uniformBuffer);
	params.PushBack(inputOutputData);
	params.PushBack(parameters);
	params.PushBack(inputOutputGradients);
	params.PushBack(weightsAndBiasGradients);
	Assembly(shader, numberOfinputs, params.GetCount(), &params[0]);
}

ndBrainTrainerGpuInference::ndBrainTrainerGpuInference(const ndTrainerDescriptor& descriptor)
	:ndBrainTrainer(descriptor)
	,m_inputOutputBuffer()
	,m_weightAndBiasBuffer()
	,m_miniBatchInputBuffer()
	,m_miniBatchOutputBuffer()
	,m_feedForwardCommands()
	,m_miniBatchSize(descriptor.m_minibatchSize)
{
	ndAssert(descriptor.m_brain->IsGpuReady());
	ndAssert(descriptor.m_context->GetType() == ndBrainContext::m_gpu);
	ndBrainTrainerGpuInference::Initialize(descriptor);
}

ndBrainTrainerGpuInference::ndBrainTrainerGpuInference(const ndSharedPtr<ndBrain>& brain, const ndSharedPtr<ndBrainContext>& context, ndInt32 minibatchSize)
	:ndBrainTrainer(brain, context)
	,m_inputOutputBuffer()
	,m_weightAndBiasBuffer()
	,m_miniBatchInputBuffer()
	,m_miniBatchOutputBuffer()
	,m_feedForwardCommands()
	,m_miniBatchSize(minibatchSize)
{
	ndTrainerDescriptor descriptor(brain, context, m_miniBatchSize, ndBrainFloat(0.0f));
	ndAssert(descriptor.m_brain->IsGpuReady());
	ndAssert(descriptor.m_context->GetType() == ndBrainContext::m_gpu);
	ndBrainTrainerGpuInference::Initialize(descriptor);
}

ndBrainTrainerGpuInference::ndBrainTrainerGpuInference(const ndBrainTrainerGpuInference& src)
	:ndBrainTrainer(src)
	,m_inputOutputBuffer()
	,m_weightAndBiasBuffer()
	,m_miniBatchInputBuffer()
	,m_miniBatchOutputBuffer()
	,m_feedForwardCommands()
	,m_miniBatchSize(src.m_miniBatchSize)
{
	ndAssert(0);
}

ndBrainTrainerGpuInference::~ndBrainTrainerGpuInference()
{
}

void ndBrainTrainerGpuInference::Initialize(const ndTrainerDescriptor&)
{
	InitInputOutputBuffer();
	InitWeightAndBiasBuffer();
}

void ndBrainTrainerGpuInference::AddLayersCommands(ndFixSizeArray<ndBrainLayer::ndCommandShareInfo, 256>& layersUniformsData)
{
	// create all the uniform buffers 
	const ndBrain& brain = **m_brain;
	ndBrainGpuFloatBuffer* const weightsBuffer = *m_weightAndBiasBuffer;
	ndBrainGpuFloatBuffer* const inputOutputBuffer = *m_inputOutputBuffer;
	
	ndInt32 inputOutputStartOffset = 0;
	ndInt32 inputOutputBufferSize = RoundoffOffset(brain.GetInputSize());
	for (ndInt32 i = 0; i < ndInt32(brain.GetCount()); ++i)
	{
		ndBrainLayer::ndCommandShareInfo& data = layersUniformsData[i];
		data.m_inputOutputStartOffset = inputOutputStartOffset;
		inputOutputBufferSize += RoundoffOffset(data.m_outputSize);
		inputOutputStartOffset += RoundoffOffset(data.m_inputSize);
	}
	
	for (ndInt32 i = 0; i < ndInt32(brain.GetCount()); ++i)
	{
		ndBrainLayer::ndCommandShareInfo uniformParam;
	
		ndBrainLayer* const layer = brain[i];
		const ndBrainLayer::ndCommandShareInfo& data = layersUniformsData[i];
		uniformParam.m_inputSize = data.m_inputSize;
		uniformParam.m_outputSize = data.m_outputSize;
		uniformParam.m_inputOutputSize = inputOutputBufferSize;
		uniformParam.m_inputOutputStartOffset = data.m_inputOutputStartOffset;
		uniformParam.m_parametersBatchSize = data.m_parametersBatchSize;
		uniformParam.m_parametersStartOffset = data.m_parametersStartOffset;
		uniformParam.m_layer = layer;
		ndSharedPtr<ndBrainGpuUniformBuffer> uniformbuffer(new ndBrainGpuUniformBuffer(*m_context, sizeof(ndBrainLayer::ndCommandShareInfo)));
		uniformbuffer->MemoryToDevive(0, sizeof(ndBrainLayer::ndCommandShareInfo), &uniformParam);

		ndSharedPtr<ndBrainGpuCommand> command(layer->CreateGpuFeedForwardCommand(this, uniformParam, m_context->GetAsGpuContext(), m_miniBatchSize, uniformbuffer, inputOutputBuffer, weightsBuffer));
		command->m_layer = layer;
		m_feedForwardCommands.Append(command);
	}
}

void ndBrainTrainerGpuInference::AddCopyInputCommand(const ndBrainLayer::ndCommandShareInfo& uniformData)
{
	const ndBrain& brain = **m_brain;
	ndInt32 inputOutputBufferSize = RoundoffOffset(brain.GetInputSize());
	for (ndInt32 i = 0; i < ndInt32(brain.GetCount()); ++i)
	{
		const ndBrainLayer* const layer = brain[i];
		inputOutputBufferSize += RoundoffOffset(layer->GetOutputSize());
	}
	
	ndBrainLayer::ndCommandShareInfo uniformParam;
	uniformParam.m_inputSize = uniformData.m_inputSize;
	uniformParam.m_outputSize = uniformData.m_outputSize;
	uniformParam.m_parametersStartOffset = 0;
	uniformParam.m_inputOutputSize = inputOutputBufferSize;
	uniformParam.m_inputOutputStartOffset = 0;
	
	ndSharedPtr<ndBrainGpuUniformBuffer> uniformbuffer(new ndBrainGpuUniformBuffer(*m_context, sizeof(ndBrainLayer::ndCommandShareInfo)));
	uniformbuffer->MemoryToDevive(0, sizeof(ndBrainLayer::ndCommandShareInfo), &uniformParam);
	
	ndBrainGpuFloatBuffer* const inputOutputBuffer = *m_inputOutputBuffer;
	ndBrainGpuFloatBuffer* const miniBatchInputBuffer = *m_miniBatchInputBuffer;
	ndSharedPtr<ndBrainGpuCommand>command(new ndBrainTrainerGpuCommand(this, uniformParam, m_inputId, m_context->GetAsGpuContext(), m_context->GetAsGpuContext()->m_brainCopyInput, m_miniBatchSize, uniformbuffer, inputOutputBuffer, miniBatchInputBuffer));
	m_feedForwardCommands.Append(command);
}

void ndBrainTrainerGpuInference::AddCopyOutputCommand()
{
	const ndBrain& brain = **m_brain;
	size_t lastId = size_t(brain[brain.GetCount() - 1]);
	ndBrainTrainerGpuCommand* const lastLayerCommand = FindCommand(lastId);
	ndAssert(lastLayerCommand);

	ndBrainLayer::ndCommandShareInfo data(lastLayerCommand->m_info);
	
	data.m_parametersStartOffset = 0;
	data.m_inputOutputStartOffset += RoundoffOffset(data.m_inputSize);
	ndSharedPtr<ndBrainGpuUniformBuffer> uniformbuffer(new ndBrainGpuUniformBuffer(*m_context, sizeof(ndBrainLayer::ndCommandShareInfo)));
	uniformbuffer->MemoryToDevive(0, sizeof(ndBrainLayer::ndCommandShareInfo), &data);
	
	ndBrainGpuFloatBuffer* const inputOutputBuffer = *m_inputOutputBuffer;
	ndBrainGpuFloatBuffer* const miniBatchOutputBuffer = *m_miniBatchOutputBuffer;
	ndSharedPtr<ndBrainGpuCommand>command(new ndBrainTrainerGpuCommand(this, data, m_outpuId, m_context->GetAsGpuContext(), m_context->GetAsGpuContext()->m_brainCopyOutput, m_miniBatchSize, uniformbuffer, inputOutputBuffer, miniBatchOutputBuffer));
	m_feedForwardCommands.Append(command);
}

void ndBrainTrainerGpuInference::InitWeightAndBiasBuffer()
{
	const ndBrain& brain = **m_brain;
	
	ndFixSizeArray<ndBrainLayer::ndCommandShareInfo, 256> uniformData;
	for (ndInt32 i = 0; i < ndInt32(brain.GetCount()); ++i)
	{
		ndBrainLayer* const layer = brain[i];
		ndBrainLayer::ndCommandShareInfo info(layer->GetGpuCommandSharedInfo());
		info.m_parametersBatchSize = RoundoffOffset(info.m_parametersBatchSize);
		uniformData.PushBack(info);
	}
	uniformData.PushBack(ndBrainLayer::ndCommandShareInfo(nullptr));
	
	ndInt32 parametersSizeSum = 0;
	for (ndInt32 i = 0; i < uniformData.GetCount(); ++i)
	{
		uniformData[i].m_parametersStartOffset = parametersSizeSum;
		parametersSizeSum += uniformData[i].m_parametersBatchSize;
	}
	ndAssert((parametersSizeSum & (ND_DEFAULT_WORKGROUP_SIZE - 1)) == 0);
	if (!parametersSizeSum)
	{
		parametersSizeSum = ND_DEFAULT_WORKGROUP_SIZE;
	}
	
	ndBrainVector scratchBuffer;
	scratchBuffer.SetCount(parametersSizeSum);
	scratchBuffer.Set(ndBrainFloat(0.0f));
	
	for (ndInt32 i = 0; i < ndInt32(brain.GetCount()); ++i)
	{
		const ndBrainLayer* const layer = brain[i];
		ndBrainLayer::ndCommandShareInfo& info = uniformData[i];
	
		ndInt32 size = uniformData[i].m_parametersBatchSize;
		if (size)
		{
			ndInt32 offset = uniformData[i].m_parametersStartOffset;
			ndBrainMemVector weights(&scratchBuffer[offset], size);
			layer->CopyGpuWeights(weights);
			info.m_parametersBatchSize = parametersSizeSum;
		}
	}
	uniformData[uniformData.GetCount() - 1].m_parametersBatchSize = parametersSizeSum;
	m_weightAndBiasBuffer = ndSharedPtr<ndBrainGpuFloatBuffer>(new ndBrainGpuFloatBuffer(*m_context, scratchBuffer));
	
	scratchBuffer.SetCount(m_miniBatchSize * brain.GetInputSize());
	scratchBuffer.Set(ndBrainFloat(0.0f));
	m_miniBatchInputBuffer = ndSharedPtr<ndBrainGpuFloatBuffer>(new ndBrainGpuFloatBuffer(*m_context, scratchBuffer));
	
	scratchBuffer.SetCount(m_miniBatchSize * brain.GetOutputSize());
	scratchBuffer.Set(ndBrainFloat(0.0f));
	m_miniBatchOutputBuffer = ndSharedPtr<ndBrainGpuFloatBuffer>(new ndBrainGpuFloatBuffer(*m_context, scratchBuffer));
	
	AddCopyInputCommand(uniformData[0]);
	AddLayersCommands(uniformData);
	AddCopyOutputCommand();
}

ndInt32 ndBrainTrainerGpuInference::RoundoffOffset(ndInt32 value) const
{
	return (value + ND_DEFAULT_WORKGROUP_SIZE - 1) & -ND_DEFAULT_WORKGROUP_SIZE;
}

void ndBrainTrainerGpuInference::InitInputOutputBuffer()
{
	const ndBrain& brain = **m_brain;
	ndInt32 bufferSize = RoundoffOffset(brain.GetInputSize());
	for (ndInt32 i = 0; i < ndInt32(brain.GetCount()); ++i)
	{
		const ndBrainLayer* const layer = brain[i];
		bufferSize += RoundoffOffset(layer->GetOutputSize());
	}
	
	ndBrainVector buffer;
	buffer.SetCount(bufferSize * m_miniBatchSize);
	buffer.Set(ndBrainFloat(0.0f));
	m_inputOutputBuffer = ndSharedPtr<ndBrainGpuFloatBuffer>(new ndBrainGpuFloatBuffer(*m_context, buffer));
}

ndBrainBuffer* ndBrainTrainerGpuInference::GetInputBuffer()
{
	return *m_miniBatchInputBuffer;
}

const ndBrainBuffer* ndBrainTrainerGpuInference::GetOutputBuffer()
{
	return *m_miniBatchOutputBuffer;
}

void ndBrainTrainerGpuInference::SaveInput(ndBrainVector& output) const
{
	m_context->GetAsGpuContext()->SyncQueue();
	m_miniBatchInputBuffer->BrainVectorFromDevice(output);
}

void ndBrainTrainerGpuInference::GetOutput(ndBrainVector& output) const
{
	m_context->GetAsGpuContext()->SyncQueue();
	m_miniBatchOutputBuffer->BrainVectorFromDevice(output);
}

void ndBrainTrainerGpuInference::GetWorkingBuffer(ndBrainVector& output) const
{
	m_context->GetAsGpuContext()->SyncQueue();
	m_inputOutputBuffer->BrainVectorFromDevice(output);
}

void ndBrainTrainerGpuInference::GetParameterBuffer(ndBrainVector& output) const
{
	m_context->GetAsGpuContext()->SyncQueue();
	m_weightAndBiasBuffer->BrainVectorFromDevice(output);
}

void ndBrainTrainerGpuInference::BackPropagate(const ndBrainVector&)
{
}

ndBrainTrainerGpuCommand* ndBrainTrainerGpuInference::FindCommand(size_t id) const
{
	for (ndList<ndSharedPtr<ndBrainGpuCommand>>::ndNode* node = m_feedForwardCommands.GetFirst(); node; node = node->GetNext())
	{
		ndBrainTrainerGpuCommand* const command = (ndBrainTrainerGpuCommand*)*node->GetInfo();
		if (command->m_id == id)
		{
			return command;
		}
	}
	return nullptr;
}

void ndBrainTrainerGpuInference::ApplyLearnRate()
{
	ndAssert(0);
}

void ndBrainTrainerGpuInference::UpdateParameters(const ndBrainVector& weightAndBias)
{
	for (ndList<ndSharedPtr<ndBrainGpuCommand>>::ndNode* node = m_feedForwardCommands.GetFirst(); node; node = node->GetNext())
	{
		ndBrainGpuCommand* const command = *node->GetInfo();
		ndBrainLayer* const layer = command->m_layer;
		if (layer && command->m_info.m_parametersBatchSize)
		{
			const ndBrainLayer::ndCommandShareInfo info (layer->GetCpuCommandSharedInfo());
			ndInt32 height = (info.m_outputSize + ND_GPU_TILED_MATRIX_ROWS - 1) & -ND_GPU_TILED_MATRIX_ROWS;
			ndInt32 width = (info.m_inputSize + ND_GPU_TILED_MATRIX_COLUMNS - 1) & -ND_GPU_TILED_MATRIX_COLUMNS;
			ndInt32 size = width * height + height;
			const ndBrainMemVector weights(&weightAndBias[command->m_info.m_parametersStartOffset], size);
			layer->SetGpuWeights(weights);
		}
	}
}

void ndBrainTrainerGpuInference::MakeSinglePrediction(const ndBrainVector& input, ndBrainVector& output)
{
	ndBrainBuffer* const inputBuffer = GetInputBuffer();
	const ndBrainBuffer* const outputBuffer = GetOutputBuffer();
	inputBuffer->MemoryToDevive(0, input.GetCount() * sizeof (ndReal), &input[0]);
	MakePrediction();
	outputBuffer->MemoryFromDevive(0, output.GetCount() * sizeof(ndReal), &output[0]);
}

void ndBrainTrainerGpuInference::SyncQueue()
{
	m_context->GetAsGpuContext()->SyncQueue();
}

void ndBrainTrainerGpuInference::LoadInput(const ndBrainVector& input)
{
	m_context->GetAsGpuContext()->SyncQueue();
	m_miniBatchInputBuffer->BrainVectorToDevice(input);
}

void ndBrainTrainerGpuInference::MakePrediction()
{
	for (ndList<ndSharedPtr<ndBrainGpuCommand>>::ndNode* node = m_feedForwardCommands.GetFirst(); node; node = node->GetNext())
	{
		ndSharedPtr<ndBrainGpuCommand>& command = node->GetInfo();
		m_context->GetAsGpuContext()->AddCommandQueue(command);
	}
}

