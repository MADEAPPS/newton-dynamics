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
#include "ndBrainLayerLinear.h"
#include "ndBrainGpuFloatBuffer.h"
#include "ndBrainGpuUniformBuffer.h"
#include "ndBrainTrainerGpuInference.h"

ndBrainTrainerGpuCommand::ndBrainTrainerGpuCommand(ndBrainTrainerGpuInference* const owner,
	const ndBrainLayer::ndCommandShareInfo& info, size_t id,
	ndBrainGpuContext* const context,
	const ndSharedPtr<ndBrainGpuShader>& shader,
	ndInt32 numberOfinputs,
	const ndSharedPtr<ndBrainGpuBuffer>& uniformBuffer,
	ndBrainGpuBuffer* const inputOutputData,
	ndBrainGpuBuffer* const parameters,
	ndBrainGpuBuffer* const inputOutputGradients)
	:ndBrainGpuCommand(context)
	,m_uniformBuffer(uniformBuffer)
	,m_info(info)
	,m_owner(owner)
	,m_id(id)
{
	ndFixSizeArray<ndBrainGpuBuffer*, 4> params;
	params.PushBack(*m_uniformBuffer);
	params.PushBack(inputOutputData);
	params.PushBack(parameters);
	params.PushBack(inputOutputGradients);
	Assembly(shader, numberOfinputs, params.GetCount(), &params[0]);
}

ndBrainTrainerGpuInference::ndBrainTrainerGpuInference(const ndSharedPtr<ndBrain>& brain, const ndSharedPtr<ndBrainContext>& context, ndInt32 minibatchSize)
	:ndBrainTrainer(brain, context)
	,m_context(context->GetAsGpuContext())
	,m_contextRef(context)
	,m_inputOutputBuffer()
	,m_weightAndBiasBuffer()
	,m_miniBatchInputBuffer()
	,m_miniBatchOutputBuffer()
	,m_feedFowardCommands()
	,m_miniBatchSize(minibatchSize)
{
	ndAssert(brain->IsGpuReady());
	ndAssert(context->GetType() == ndBrainContext::m_gpu);
	InitInputOutputBuffer();
	InitWeightAndBiasBuffer();
}

ndBrainTrainerGpuInference::ndBrainTrainerGpuInference(const ndBrainTrainerGpuInference& src)
	:ndBrainTrainer(src)
	,m_context(src.m_context)
	,m_inputOutputBuffer()
	,m_weightAndBiasBuffer()
	,m_miniBatchInputBuffer()
	,m_miniBatchOutputBuffer()
	,m_feedFowardCommands()
	,m_miniBatchSize(src.m_miniBatchSize)
{
	ndAssert(0);
}

ndBrainTrainerGpuInference::~ndBrainTrainerGpuInference()
{
}

void ndBrainTrainerGpuInference::AddLayersCommands(ndFixSizeArray<ndBrainLayer::ndCommandShareInfo, 256>& layersUniformsData)
{
	// create all the uniform buffers 
	const ndBrain& brain = **m_brain;
	ndBrainGpuBuffer* const weightsBuffer = *m_weightAndBiasBuffer;
	ndBrainGpuBuffer* const inputOutputBuffer = *m_inputOutputBuffer;
	
	ndInt32 inputOutputStartOffset = 0;
	ndInt32 inputOutputBufferSize = brain.GetInputSize();
	for (ndInt32 i = 0; i < ndInt32(brain.GetCount()); ++i)
	{
		ndBrainLayer::ndCommandShareInfo& data = layersUniformsData[i];
		data.m_inputOutputStartOffset = inputOutputStartOffset;
		inputOutputBufferSize += data.m_outputSize;
		inputOutputStartOffset += data.m_inputSize;
	}
	
	for (ndInt32 i = 0; i < ndInt32(brain.GetCount()); ++i)
	{
		ndBrainLayer::ndCommandShareInfo uniformParam;
		const ndBrainLayer::ndCommandShareInfo& data = layersUniformsData[i];
		uniformParam.m_inputSize = data.m_inputSize;
		uniformParam.m_outputSize = data.m_outputSize;
		uniformParam.m_parametersBatchSize = data.m_parametersBatchSize;
		uniformParam.m_parametersStartOffset = data.m_parametersStartOffset;
		uniformParam.m_inputOutputSize = inputOutputBufferSize;
		uniformParam.m_inputOutputStartOffset = data.m_inputOutputStartOffset;
	
		ndSharedPtr<ndBrainGpuBuffer> uniformbuffer(new ndBrainGpuUniformBuffer(m_context, sizeof(ndBrainLayer::ndCommandShareInfo)));
		uniformbuffer->LoadData(sizeof(ndBrainLayer::ndCommandShareInfo), &uniformParam);
		
		const ndBrainLayer* const layer = brain[i];
		ndSharedPtr<ndBrainGpuCommand> command(layer->CreateGpuFeedForwardCommand(this, uniformParam, m_context, m_miniBatchSize, uniformbuffer, inputOutputBuffer, weightsBuffer));
		m_feedFowardCommands.Append(command);
	}
}

void ndBrainTrainerGpuInference::AddCopyInputCommand(const ndBrainLayer::ndCommandShareInfo& uniformData)
{
	const ndBrain& brain = **m_brain;
	ndInt32 inputOutputBufferSize = brain.GetInputSize();
	for (ndInt32 i = 0; i < ndInt32(brain.GetCount()); ++i)
	{
		const ndBrainLayer* const layer = brain[i];
		inputOutputBufferSize += layer->GetOutputSize();
	}
	
	ndBrainLayer::ndCommandShareInfo uniformParam;
	uniformParam.m_inputSize = uniformData.m_inputSize;
	uniformParam.m_outputSize = uniformData.m_outputSize;
	uniformParam.m_parametersStartOffset = 0;
	uniformParam.m_inputOutputSize = inputOutputBufferSize;
	uniformParam.m_inputOutputStartOffset = 0;
	
	ndSharedPtr<ndBrainGpuBuffer> uniformbuffer(new ndBrainGpuUniformBuffer(m_context, sizeof(ndBrainLayer::ndCommandShareInfo)));
	uniformbuffer->LoadData(sizeof(ndBrainLayer::ndCommandShareInfo), &uniformParam);
	
	ndBrainGpuBuffer* const inputOutputBuffer = *m_inputOutputBuffer;
	ndBrainGpuBuffer* const miniBatchInputBuffer = *m_miniBatchInputBuffer;
	ndSharedPtr<ndBrainGpuCommand>command(new ndBrainTrainerGpuCommand(this, uniformParam, m_inputId, m_context, m_context->m_ndBrainCopyInput, m_miniBatchSize, uniformbuffer, inputOutputBuffer, miniBatchInputBuffer));
	m_feedFowardCommands.Append(command);
}

void ndBrainTrainerGpuInference::AddCopyOutputCommand()
{
	const ndBrain& brain = **m_brain;
	size_t lastId = size_t(brain[brain.GetCount() - 1]);
	ndBrainTrainerGpuCommand* const lastLayerCommand = FindCommand(lastId);
	ndAssert(lastLayerCommand);

	ndBrainLayer::ndCommandShareInfo data(lastLayerCommand->m_info);
	
	data.m_parametersStartOffset = 0;
	data.m_inputOutputStartOffset += data.m_inputSize;
	ndSharedPtr<ndBrainGpuBuffer> uniformbuffer(new ndBrainGpuUniformBuffer(m_context, sizeof(ndBrainLayer::ndCommandShareInfo)));
	uniformbuffer->LoadData(sizeof(ndBrainLayer::ndCommandShareInfo), &data);
	
	ndBrainGpuBuffer* const inputOutputBuffer = *m_inputOutputBuffer;
	ndBrainGpuBuffer* const miniBatchOutputBuffer = *m_miniBatchOutputBuffer;
	ndSharedPtr<ndBrainGpuCommand>command(new ndBrainTrainerGpuCommand(this, data, m_outpuId, m_context, m_context->m_ndBrainCopyOutput, m_miniBatchSize, uniformbuffer, inputOutputBuffer, miniBatchOutputBuffer));
	m_feedFowardCommands.Append(command);
}

void ndBrainTrainerGpuInference::InitWeightAndBiasBuffer()
{
	const ndBrain& brain = **m_brain;
	
	ndFixSizeArray<ndBrainLayer::ndCommandShareInfo, 256> uniformData;
	for (ndInt32 i = 0; i < ndInt32(brain.GetCount()); ++i)
	{
		const ndBrainLayer* const layer = brain[i];
		ndBrainLayer::ndCommandShareInfo info(layer->GetCommandSharedInfo());
		uniformData.PushBack(info);
	}
	uniformData.PushBack(ndBrainLayer::ndCommandShareInfo(nullptr));
	
	ndInt32 sizeAcc = 0;
	for (ndInt32 i = 0; i < uniformData.GetCount(); ++i)
	{
		uniformData[i].m_parametersStartOffset = sizeAcc;
		sizeAcc += uniformData[i].m_parametersBatchSize;
	}
	sizeAcc += 32;

	ndBrainVector scratchBuffer;
	scratchBuffer.SetCount(sizeAcc);
	scratchBuffer.Set(ndBrainFloat(0.0f));
	for (ndInt32 i = 0; i < ndInt32(brain.GetCount()); ++i)
	{
		const ndBrainLayer* const layer = brain[i];
		ndBrainLayer::ndCommandShareInfo& info = uniformData[i];
		ndBrainMemVector weights(&scratchBuffer[uniformData[i].m_parametersStartOffset], uniformData[i].m_parametersBatchSize);
		info.m_parametersBatchSize = sizeAcc;
		layer->CopyWeights(weights);
	}
	uniformData[uniformData.GetCount() - 1].m_parametersBatchSize = sizeAcc;
	m_weightAndBiasBuffer = ndSharedPtr<ndBrainGpuBuffer>(new ndBrainGpuFloatBuffer(m_context, scratchBuffer, ndCpuMappable));

	scratchBuffer.SetCount(m_miniBatchSize * brain.GetInputSize());
	scratchBuffer.Set(ndBrainFloat(0.0f));
	m_miniBatchInputBuffer = ndSharedPtr<ndBrainGpuBuffer>(new ndBrainGpuFloatBuffer(m_context, scratchBuffer, ndCpuMappable));
	
	scratchBuffer.SetCount(m_miniBatchSize * brain.GetOutputSize());
	scratchBuffer.Set(ndBrainFloat(0.0f));
	m_miniBatchOutputBuffer = ndSharedPtr<ndBrainGpuBuffer>(new ndBrainGpuFloatBuffer(m_context, scratchBuffer, ndCpuMappable));
	
	AddCopyInputCommand(uniformData[0]);
	AddLayersCommands(uniformData);
	AddCopyOutputCommand();
}

void ndBrainTrainerGpuInference::InitInputOutputBuffer()
{
	const ndBrain& brain = **m_brain;
	ndInt32 bufferSize = brain.GetInputSize();
	for (ndInt32 i = 0; i < ndInt32(brain.GetCount()); ++i)
	{
		const ndBrainLayer* const layer = brain[i];
		bufferSize += layer->GetOutputSize();
	}

	ndBrainVector buffer;
	buffer.SetCount(bufferSize * m_miniBatchSize);
	buffer.Set(ndBrainFloat(0.0f));
	m_inputOutputBuffer = ndSharedPtr<ndBrainGpuBuffer>(new ndBrainGpuFloatBuffer(m_context, buffer, ndCpuMappable));
}

void ndBrainTrainerGpuInference::UnloadBuffer(ndBrainVector& ouput, const ndSharedPtr<ndBrainGpuBuffer>& gpuBuffer) const
{
	size_t sizeInBytes = gpuBuffer->SizeInBytes();
	ouput.SetCount(ndInt64(sizeInBytes / sizeof(ndReal)));
	gpuBuffer->UnloadData(sizeInBytes, &ouput[0]);
}

void ndBrainTrainerGpuInference::GetInput(ndBrainVector& ouput) const
{
	UnloadBuffer(ouput, m_miniBatchInputBuffer);
	m_context->SyncQueue();
}

void ndBrainTrainerGpuInference::GetOutput(ndBrainVector& ouput) const
{
	UnloadBuffer(ouput, m_miniBatchOutputBuffer);
	m_context->SyncQueue();
}

void ndBrainTrainerGpuInference::GetWorkingBuffer(ndBrainVector& ouput) const
{
	UnloadBuffer(ouput, m_inputOutputBuffer);
	m_context->SyncQueue();
}

void ndBrainTrainerGpuInference::GetParameterBuffer(ndBrainVector& ouput) const
{
	UnloadBuffer(ouput, m_weightAndBiasBuffer);
	m_context->SyncQueue();
}

void ndBrainTrainerGpuInference::BackPropagate(const ndBrainVector&, bool)
{
}

ndBrainTrainerGpuCommand* ndBrainTrainerGpuInference::FindCommand(size_t id) const
{
	for (ndList<ndSharedPtr<ndBrainGpuCommand>>::ndNode* node = m_feedFowardCommands.GetFirst(); node; node = node->GetNext())
	{
		ndBrainTrainerGpuCommand* const command = (ndBrainTrainerGpuCommand*)*node->GetInfo();
		if (command->m_id == id)
		{
			return command;
		}
	}
	return nullptr;
}

//void ndBrainTrainerGpuInference::ApplyLearnRate(ndBrainFloat learnRate)
void ndBrainTrainerGpuInference::ApplyLearnRate(ndBrainFloat)
{
	ndAssert(0);
}

void ndBrainTrainerGpuInference::UpdateParameters()
{
	//ndAssert(0);
}

//void ndBrainTrainerGpuInference::MakeSinglePrediction(const ndBrainVector& input, ndBrainVector& output)
void ndBrainTrainerGpuInference::MakeSinglePrediction(const ndBrainVector&, ndBrainVector&)
{
	ndAssert(0);
}

void ndBrainTrainerGpuInference::SyncQueue()
{
	m_context->SyncQueue();
}

void ndBrainTrainerGpuInference::MakePrediction(const ndBrainVector& input, bool sync)
{
	m_miniBatchInputBuffer->LoadData(input.GetCount() * sizeof(ndReal), &input[0]);
	for (ndList<ndSharedPtr<ndBrainGpuCommand>>::ndNode* node = m_feedFowardCommands.GetFirst(); node; node = node->GetNext())
	{
		ndSharedPtr<ndBrainGpuCommand>& command = node->GetInfo();
		m_context->AddCommandQueue(command);
	}
	if (sync)
	{
		SyncQueue();
	}
}

