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
#include "ndBrainTrainnerGpuInference.h"

ndBrainTrainerGpuCommand::ndBrainTrainerGpuCommand(ndBrainTrainnerGpuInference* const owner,
	const ndBrainLayer::ndCommandShareInfo& info, size_t m_id,
	ndBrainGpuContext* const context,
	ndVulkanShader m_shader,
	ndInt32 numberOfinputs,
	const ndSharedPtr<ndBrainGpuBuffer>& uniformBuffer,
	ndBrainGpuBuffer* const buffer1,
	ndBrainGpuBuffer* const buffer2)
	:ndBrainGpuCommand(context)
	,m_uniformBuffer(uniformBuffer)
	,m_info(info)
	,m_owner(owner)
	,m_id(m_id)
{
	ndFixSizeArray<ndBrainGpuBuffer*, 4> params;
	params.PushBack(*m_uniformBuffer);
	params.PushBack(buffer1);
	params.PushBack(buffer2);
	Assembly(m_shader, numberOfinputs, params.GetCount(), &params[0]);
}

ndBrainTrainnerGpuInference::ndBrainTrainnerGpuInference(const ndSharedPtr<ndBrain>& brain, const ndSharedPtr<ndBrainContext>& context, ndInt32 minibatchSize)
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

ndBrainTrainnerGpuInference::ndBrainTrainnerGpuInference(const ndBrainTrainnerGpuInference& src)
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

ndBrainTrainnerGpuInference::~ndBrainTrainnerGpuInference()
{
}

//void ndBrainTrainnerGpuInference::AddLayersCommands(ndFixSizeArray<ndLayerUniformDataGpu, 256>& layersUniformsData)
void ndBrainTrainnerGpuInference::AddLayersCommands(ndFixSizeArray<ndBrainLayer::ndCommandShareInfo, 256>& layersUniformsData)
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
		uniformParam.m_parametersStartOffset = data.m_parametersStartOffset;
		uniformParam.m_inputOutputSize = inputOutputBufferSize;
		uniformParam.m_inputOutputStartOffset = data.m_inputOutputStartOffset;
	
		ndSharedPtr<ndBrainGpuBuffer> uniformbuffer(new ndBrainGpuUniformBuffer(m_context, sizeof(ndBrainLayer::ndCommandShareInfo)));
		uniformbuffer->LoadData(sizeof(ndBrainLayer::ndCommandShareInfo), &uniformParam);
		
		const ndBrainLayer* const layer = brain[i];
		//size_t layerId = size_t(brain[i]);
		//ndSharedPtr<ndBrainGpuCommand> command(new ndGpuCommand(m_context, data.m_shader, m_miniBatchSize, uniformbuffer, inputOutputBuffer, weightsBuffer, layerId));
		ndSharedPtr<ndBrainTrainerGpuCommand> command(layer->CreateGpuFeedForwardCommand(this, uniformParam, m_context, m_miniBatchSize, uniformbuffer, inputOutputBuffer, weightsBuffer));
		m_feedFowardCommands.Append(command);
	}
}

void ndBrainTrainnerGpuInference::AddCopyInputCommand(const ndBrainLayer::ndCommandShareInfo& uniformData)
{
	const ndBrain& brain = **m_brain;
	ndInt32 inputOutputBufferSize = brain.GetInputSize();
	for (ndInt32 i = 0; i < ndInt32(brain.GetCount()); ++i)
	{
		const ndBrainLayer* const layer = brain[i];
		inputOutputBufferSize += layer->GetOutputSize();
	}
	
	//ndUniformBufferObject uniformParam;
	ndBrainLayer::ndCommandShareInfo uniformParam;
	uniformParam.m_inputSize = uniformData.m_inputSize;
	uniformParam.m_outputSize = uniformData.m_outputSize;
	uniformParam.m_parametersStartOffset = 0;
	uniformParam.m_inputOutputSize = inputOutputBufferSize;
	uniformParam.m_inputOutputStartOffset = 0;
	
	ndSharedPtr<ndBrainGpuBuffer> uniformbuffer(new ndBrainGpuUniformBuffer(m_context, sizeof(ndBrainLayer::ndCommandShareInfo)));
	uniformbuffer->LoadData(sizeof(ndBrainLayer::ndCommandShareInfo), &uniformParam);

	//ndBrainLayer::ndCommandShareInfo uniformParam1;
	//uniformbuffer->UnloadData(sizeof(ndBrainLayer::ndCommandShareInfo), &uniformParam1);
	
	ndBrainGpuBuffer* const inputOutputBuffer = *m_inputOutputBuffer;
	ndBrainGpuBuffer* const miniBatchInputBuffer = *m_miniBatchInputBuffer;
	ndSharedPtr<ndBrainTrainerGpuCommand>command(new ndBrainTrainerGpuCommand(this, uniformParam, m_inputId, m_context, m_context->m_ndBrainCopyInput, m_miniBatchSize, uniformbuffer, inputOutputBuffer, miniBatchInputBuffer));
	m_feedFowardCommands.Append(command);
}

void ndBrainTrainnerGpuInference::AddCopyOutputCommand()
{
	ndAssert(0);
	//ndUniformBufferObject data;
	//
	//const ndBrain& brain = **m_brain;
	//size_t lastId = size_t(brain[brain.GetCount() - 1]);
	//ndGpuCommand* const lastLayerCommand = (ndGpuCommand*)FindCommand(lastId);
	//ndAssert(lastLayerCommand);
	//const ndBrainGpuBuffer& uniformData = **lastLayerCommand->m_uniformBuffer;
	//uniformData.UnloadData(ndInt32 (sizeof(ndUniformBufferObject)), &data);
	//
	//data.m_parametersStartOffset = 0;
	//data.m_inputOutputStartOffset += data.m_inputSize;
	//ndSharedPtr<ndBrainGpuBuffer> uniformbuffer(new ndBrainGpuUniformBuffer(m_context, sizeof(ndUniformBufferObject)));
	//uniformbuffer->LoadData(sizeof(ndUniformBufferObject), &data);
	//
	//ndBrainGpuBuffer* const inputOutputBuffer = *m_inputOutputBuffer;
	//ndBrainGpuBuffer* const miniBatchOutputBuffer = *m_miniBatchOutputBuffer;
	//ndSharedPtr<ndBrainGpuCommand> command(new ndGpuCommand(m_context, m_context->m_ndBrainCopyOutput, m_miniBatchSize, uniformbuffer, inputOutputBuffer, miniBatchOutputBuffer, m_outpuId));
	//m_feedFowardCommands.Append(command);
}

void ndBrainTrainnerGpuInference::InitWeightAndBiasBuffer()
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
	
	//ndInt32 padWeights = 32;
	sizeAcc += 32;
	ndBrainVector scratchBuffer;
	//scratchBuffer.SetCount(sizeAcc + padWeights);
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
	//AddCopyOutputCommand();
}

void ndBrainTrainnerGpuInference::InitInputOutputBuffer()
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

void ndBrainTrainnerGpuInference::UnloadBuffer(ndBrainVector& ouput, const ndSharedPtr<ndBrainGpuBuffer>& gpuBuffer) const
{
	size_t sizeInBytes = gpuBuffer->SizeInBytes();
	ouput.SetCount(ndInt64(sizeInBytes / sizeof(ndReal)));
	gpuBuffer->UnloadData(sizeInBytes, &ouput[0]);
}

void ndBrainTrainnerGpuInference::GetInput(ndBrainVector& ouput) const
{
	UnloadBuffer(ouput, m_miniBatchInputBuffer);
}

void ndBrainTrainnerGpuInference::GetOutput(ndBrainVector& ouput) const
{
	UnloadBuffer(ouput, m_miniBatchOutputBuffer);
}

void ndBrainTrainnerGpuInference::GetWorkingBuffer(ndBrainVector& ouput) const
{
	UnloadBuffer(ouput, m_inputOutputBuffer);
}

void ndBrainTrainnerGpuInference::GetParameterBuffer(ndBrainVector& ouput) const
{
	UnloadBuffer(ouput, m_weightAndBiasBuffer);
}

void ndBrainTrainnerGpuInference::BackPropagate(const ndBrainVector&)
{
}

ndBrainTrainerGpuCommand* ndBrainTrainnerGpuInference::FindCommand(size_t id) const
{
	for (ndList<ndSharedPtr<ndBrainTrainerGpuCommand>>::ndNode* node = m_feedFowardCommands.GetFirst(); node; node = node->GetNext())
	{
		ndBrainTrainerGpuCommand* const command = *node->GetInfo();
		if (command->m_id == id)
		{
			return command;
		}
	}
	return nullptr;
}

void ndBrainTrainnerGpuInference::SubmitCommands()
{
	ndAssert(0);
	//for (ndList<ndSharedPtr<ndBrainGpuCommand>>::ndNode* node = m_feedFowardCommands.GetFirst(); node; node = node->GetNext())
	//{
	//	ndSharedPtr<ndBrainGpuCommand>& command = node->GetInfo();
	//	m_context->AddCommandQueue(command);
	//}
}

//void ndBrainTrainnerGpuInference::ApplyLearnRate(ndBrainFloat learnRate)
void ndBrainTrainnerGpuInference::ApplyLearnRate(ndBrainFloat)
{
	ndAssert(0);
}

void ndBrainTrainnerGpuInference::UpdateParameters()
{
	//ndAssert(0);
}

void ndBrainTrainnerGpuInference::MakePrediction(const ndBrainVector& input)
{
	m_context->BeginQueue();
	m_miniBatchInputBuffer->LoadData(input.GetCount() * sizeof(ndReal), &input[0]);
	SubmitCommands();
	m_context->EndQueue();
}

//void ndBrainTrainnerGpuInference::MakeSinglePrediction(const ndBrainVector& input, ndBrainVector& output)
void ndBrainTrainnerGpuInference::MakeSinglePrediction(const ndBrainVector&, ndBrainVector&)
{
	ndAssert(0);
}

