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
#include "ndBrainThreadPool.h"
#include "ndBrainCpuContext.h"
#include "ndBrainTrainerCpuInference.h"
#include "ndBrainLayerActivationSoftmax.h"

ndBrainTrainerCpuInference::ndBrainTrainerCpuInference(const ndSharedPtr<ndBrain>& brain, const ndSharedPtr<ndBrainContext>& context, ndInt32 minibatchSize)
	:ndBrainTrainer(brain, context)
	,m_inputOutputBuffer()
	,m_weightAndBiasBuffer()
	,m_miniBatchInputBuffer()
	,m_miniBatchOutputBuffer()
	,m_feedForwardCommands()
	,m_threadPool(m_context->GetAsCpuContext())
	,m_miniBatchSize(minibatchSize)
{
	InitInputOutputBuffer();
	InitWeightAndBiasBuffer();
}

ndBrainTrainerCpuInference::ndBrainTrainerCpuInference(const ndBrainTrainerCpuInference& src)
	:ndBrainTrainer(src)
	,m_inputOutputBuffer()
	,m_threadPool(src.m_threadPool)
	,m_miniBatchSize(src.m_miniBatchSize)
{
	ndAssert(0);
}

void ndBrainTrainerCpuInference::InitInputOutputBuffer()
{
	const ndBrain& brain = **m_brain;
	ndInt32 bufferSize = brain.GetInputSize();
	for (ndInt32 i = 0; i < ndInt32(brain.GetCount()); ++i)
	{
		const ndBrainLayer* const layer = brain[i];
		bufferSize += layer->GetOutputSize();
	}
	
	m_inputOutputBuffer.SetCount(bufferSize * m_miniBatchSize);
	m_inputOutputBuffer.Set(ndBrainFloat(0.0f));
}

void ndBrainTrainerCpuInference::InitWeightAndBiasBuffer()
{
	const ndBrain& brain = **m_brain;
	
	ndFixSizeArray<ndBrainTrainerCpuCommand*, 256> feedForwardCommands;
	for (ndInt32 i = 0; i < ndInt32(brain.GetCount()); ++i)
	{
		ndBrainLayer* const layer = brain[i];
		ndBrainLayerFeedForwardCpuCommand* const command = layer->GetLayerCpuFeedForwardCommand();
		command->m_owner = this;
		command->m_info.m_layer = layer;
		feedForwardCommands.PushBack(command);
	}
	
	class DummyLayerUniformDataCpu : public ndBrainTrainerCpuCommand
	{
		public:
		DummyLayerUniformDataCpu()
			:ndBrainTrainerCpuCommand(ndBrainLayer::ndCommandShareInfo(nullptr), 0)
		{
		}
	
		virtual void Execute(ndInt32)
		{
		}
	};
	DummyLayerUniformDataCpu sentinel;
	feedForwardCommands.PushBack(&sentinel);
	
	ndInt32 sizeAcc = 0;
	for (ndInt32 i = 0; i < feedForwardCommands.GetCount(); ++i)
	{
		feedForwardCommands[i]->m_info.m_parametersStartOffset = sizeAcc;
		sizeAcc += feedForwardCommands[i]->m_info.m_parametersBatchSize;
	}
	// add some padding for edge cases
	sizeAcc += 32;
	m_weightAndBiasBuffer.SetCount(sizeAcc);
	m_weightAndBiasBuffer.Set(ndBrainFloat(0.0f));
	for (ndInt32 i = 0; i < ndInt32(brain.GetCount()); ++i)
	{
		const ndBrainLayer* const layer = brain[i];
		ndBrainTrainerCpuCommand* const command = feedForwardCommands[i];
		ndBrainMemVector weights(&m_weightAndBiasBuffer[command->m_info.m_parametersStartOffset], command->m_info.m_parametersBatchSize);
		command->m_info.m_parametersBatchSize = sizeAcc;
		layer->CopyWeights(weights);
	}
	
	m_miniBatchInputBuffer.SetCount(m_miniBatchSize * brain.GetInputSize());
	m_miniBatchInputBuffer.Set(ndBrainFloat(0.0f));
	
	m_miniBatchOutputBuffer.SetCount(m_miniBatchSize * brain.GetOutputSize());
	m_miniBatchOutputBuffer.Set(ndBrainFloat(0.0f));
	
	AddCopyInputCommand(feedForwardCommands[0]);
	AddLayersCommands(feedForwardCommands);
	AddCopyOutputCommand();
}

void ndBrainTrainerCpuInference::AddCopyInputCommand(const ndBrainTrainerCpuCommand* const uniformData)
{
	const ndBrain& brain = **m_brain;
	ndInt32 inputOutputBufferSize = brain.GetInputSize();
	for (ndInt32 i = 0; i < ndInt32(brain.GetCount()); ++i)
	{
		const ndBrainLayer* const layer = brain[i];
		inputOutputBufferSize += layer->GetOutputSize();
	}
	
	ndCopyInputCommand* const inputCommand = new ndCopyInputCommand(this);
	
	inputCommand->m_info.m_inputSize = uniformData->m_info.m_inputSize;
	inputCommand->m_info.m_outputSize = uniformData->m_info.m_outputSize;
	inputCommand->m_info.m_inputOutputSize = inputOutputBufferSize;
	inputCommand->m_info.m_inputOutputStartOffset = 0;
	m_feedForwardCommands.Append(ndSharedPtr<ndBrainTrainerCpuCommand>(inputCommand));
}

void ndBrainTrainerCpuInference::AddCopyOutputCommand()
{
	const ndBrain& brain = **m_brain;
	ndCopyOutputCommand* const outputCommand = new ndCopyOutputCommand(this);
	
	//const ndBrainLayer::ndBrainLayerFeedForwardCpuCommand* const lastCommand = (ndBrainLayer::ndBrainLayerFeedForwardCpuCommand*)*m_feedForwardCommands.GetLast()->GetInfo();
	size_t lastId = (size_t)brain[brain.GetCount() - 1];
	ndAssert(FindCommand(lastId));
	ndBrainTrainerCpuCommand* const lastCommand = FindCommand(lastId);
	 
	outputCommand->m_info.m_inputSize = lastCommand->m_info.m_inputSize;
	outputCommand->m_info.m_outputSize = lastCommand->m_info.m_outputSize;
	//outputCommand->m_info.m_parametersSize = 0;
	
	outputCommand->m_info.m_inputOutputSize = lastCommand->m_info.m_inputOutputSize;
	outputCommand->m_info.m_inputOutputStartOffset = lastCommand->m_info.m_inputOutputStartOffset + lastCommand->m_info.m_inputSize;
	m_feedForwardCommands.Append(ndSharedPtr<ndBrainTrainerCpuCommand>(outputCommand));
}

ndBrainTrainerCpuCommand* ndBrainTrainerCpuInference::FindCommand(size_t id) const
{
	for (ndList<ndSharedPtr<ndBrainTrainerCpuCommand>>::ndNode* node = m_feedForwardCommands.GetFirst(); node; node = node->GetNext())
	{
		ndBrainTrainerCpuCommand* const command = *node->GetInfo();
		if (command->m_id == id)
		{
			return command;
		}
	}
	return nullptr;
}

void ndBrainTrainerCpuInference::AddLayersCommands(ndFixSizeArray<ndBrainTrainerCpuCommand*, 256>& layersCommands)
{
	// create all the uniform buffers 
	const ndBrain& brain = **m_brain;
	
	ndInt32 inputOutputStartOffset = 0;
	ndInt32 inputOutputBufferSize = brain.GetInputSize();
	for (ndInt32 i = 0; i < ndInt32(brain.GetCount()); ++i)
	{
		ndBrainTrainerCpuCommand* const data = layersCommands[i];
		data->m_info.m_inputOutputStartOffset = inputOutputStartOffset;
		inputOutputBufferSize += data->m_info.m_outputSize;
		inputOutputStartOffset += data->m_info.m_inputSize;
	}
	
	for (ndInt32 i = 0; i < ndInt32(brain.GetCount()); ++i)
	{
		ndBrainTrainerCpuCommand* const layerCommand = layersCommands[i];
		layerCommand->m_info.m_inputOutputSize = inputOutputBufferSize;
		m_feedForwardCommands.Append(ndSharedPtr<ndBrainTrainerCpuCommand>(layerCommand));
	}
}

void ndBrainTrainerCpuInference::GetOutput(ndBrainVector& ouput) const
{
	ouput.SetCount(m_miniBatchOutputBuffer.GetCount());
	ouput.Set(m_miniBatchOutputBuffer);
}

void ndBrainTrainerCpuInference::GetInput(ndBrainVector& input) const
{
	input.SetCount(m_miniBatchInputBuffer.GetCount());
	input.Set(m_miniBatchInputBuffer);
}

void ndBrainTrainerCpuInference::GetWorkingBuffer(ndBrainVector& inputOutputBuffer) const
{
	inputOutputBuffer.SetCount(m_inputOutputBuffer.GetCount());
	inputOutputBuffer.Set(m_miniBatchOutputBuffer);
}

void ndBrainTrainerCpuInference::GetParameterBuffer(ndBrainVector& parameters) const
{
	parameters.SetCount(m_weightAndBiasBuffer.GetCount());
	parameters.Set(m_weightAndBiasBuffer);
}

// new method
void ndBrainTrainerCpuInference::BackPropagate(const ndBrainVector&, bool)
{
}

void ndBrainTrainerCpuInference::ApplyLearnRate()
{
}

void ndBrainTrainerCpuInference::UpdateParameters(const ndBrainVector& weightAndBias)
{
	for (ndList<ndSharedPtr<ndBrainTrainerCpuCommand>>::ndNode* node = m_feedForwardCommands.GetFirst(); node; node = node->GetNext())
	{
		ndBrainTrainerCpuCommand* const command = *node->GetInfo();
		ndBrainLayer* const layer = command->m_info.m_layer;
		if (layer)
		{
			ndInt32 size = command->m_info.m_inputSize * command->m_info.m_outputSize + command->m_info.m_outputSize;
			const ndBrainMemVector weights(&weightAndBias[command->m_info.m_parametersStartOffset], size);
			layer->SetWeights(weights);
		}
	}
}

void ndBrainTrainerCpuInference::SoftCopyParameters(const ndBrainTrainer& src, ndBrainFloat blendFactor)
{
	//Scale(ndBrainFloat(1.0f) - blend);
	//ScaleAdd(target, blend);
	const ndBrainTrainerCpuInference* const blendSource = (ndBrainTrainerCpuInference*)&src;
	m_weightAndBiasBuffer.Scale(ndBrainFloat(1.0f) - blendFactor);
	m_weightAndBiasBuffer.ScaleAdd(blendSource->m_weightAndBiasBuffer, blendFactor);
}

void ndBrainTrainerCpuInference::SyncQueue()
{
	// do nothing
}

void ndBrainTrainerCpuInference::MakeSinglePrediction(const ndBrainVector& input, ndBrainVector& output)
{
	ndMemCpy(&m_miniBatchInputBuffer[0], &input[0], input.GetCount());
	for (ndList<ndSharedPtr<ndBrainTrainerCpuCommand>>::ndNode* node = m_feedForwardCommands.GetFirst(); node; node = node->GetNext())
	{
		ndBrainTrainerCpuCommand* const command = *node->GetInfo();
		command->Execute(0);
	}
	ndMemCpy(&output[0], &m_miniBatchOutputBuffer[0], output.GetCount());
}

void ndBrainTrainerCpuInference::MakePrediction(const ndBrainVector& input, bool sync)
{
	ndAssert(input.GetCount() == m_miniBatchInputBuffer.GetCount());
	m_miniBatchInputBuffer.Set(input);

	ndAtomic<ndInt32> iterator(0);
	for (ndList<ndSharedPtr<ndBrainTrainerCpuCommand>>::ndNode* node = m_feedForwardCommands.GetFirst(); node; node = node->GetNext())
	{
		ndBrainTrainerCpuCommand* const command = *node->GetInfo();

		auto ExecuteCommand = ndMakeObject::ndFunction([this, &iterator, command](ndInt32, ndInt32)
		{
			for (ndInt32 i = iterator++; i < m_miniBatchSize; i = iterator++)
			{
				command->Execute(i);
			}
		});
		iterator = 0;
		m_threadPool->ndBrainThreadPool::ParallelExecute(ExecuteCommand);
	}

	if (sync)
	{
		SyncQueue();
	}
}
