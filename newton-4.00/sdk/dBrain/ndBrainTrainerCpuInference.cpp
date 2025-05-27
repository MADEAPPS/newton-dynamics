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
#include "ndBrainTrainerCpuInference.h"
#include "ndBrainLayerActivationSoftmax.h"

ndBrainTrainerCpuInference::ndBrainTrainerCpuInference(const ndSharedPtr<ndBrain>& brain, ndBrainThreadPool* const threadPool, ndInt32 minibatchSize)
	:ndBrainTrainer(brain)
	,m_inputOutputBuffer()
	,m_weightAndBiasBuffer()
	,m_miniBatchInputBuffer()
	,m_miniBatchOutputBuffer()
	,m_feedFowardCommands()
	,m_threadPool(threadPool)
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

	ndFixSizeArray<ndBrainLayer::ndBrainLayerFeedFowardCpuCommand*, 256> feedFowardCommands;
	for (ndInt32 i = 0; i < ndInt32(brain.GetCount()); ++i)
	{
		const ndBrainLayer* const layer = brain[i];
		ndBrainLayer::ndBrainLayerFeedFowardCpuCommand* const data = layer->GetLayerCpuFeedForwardCommand();
		data->m_owner = this;
		feedFowardCommands.PushBack(data);
	}

	class DummyLayerUniformDataCpu : public ndBrainLayer::ndBrainLayerFeedFowardCpuCommand
	{
		public:
		DummyLayerUniformDataCpu()
			:ndBrainLayer::ndBrainLayerFeedFowardCpuCommand(nullptr)
		{
		}

		virtual void Execute(ndInt32, ndInt32)
		{
		}
	};

	DummyLayerUniformDataCpu sentinel;
	feedFowardCommands.PushBack(&sentinel);
	
	ndInt32 sizeAcc = 0;
	for (ndInt32 i = 0; i < feedFowardCommands.GetCount(); ++i)
	{
		feedFowardCommands[i]->m_parametersStartOffset = sizeAcc;
		sizeAcc += feedFowardCommands[i]->m_parametersBatchSize;
	}
	
	ndInt32 padWeights = 32;
	m_weightAndBiasBuffer.SetCount(sizeAcc + padWeights);
	m_weightAndBiasBuffer.Set(ndBrainFloat(0.0f));
	for (ndInt32 i = 0; i < ndInt32(brain.GetCount()); ++i)
	{
		const ndBrainLayer* const layer = brain[i];
		ndBrainLayer::ndBrainLayerFeedFowardCpuCommand* const command = feedFowardCommands[i];
		ndBrainMemVector weights(&m_weightAndBiasBuffer[command->m_parametersStartOffset], command->m_parametersBatchSize);
		command->m_parametersBatchSize = sizeAcc;
		layer->CopyWeights(weights);
	}
	
	m_miniBatchInputBuffer.SetCount(m_miniBatchSize * brain.GetInputSize());
	m_miniBatchInputBuffer.Set(ndBrainFloat(0.0f));
	
	m_miniBatchOutputBuffer.SetCount(m_miniBatchSize * brain.GetOutputSize());
	m_miniBatchOutputBuffer.Set(ndBrainFloat(0.0f));
	
	AddCopyInputCommand(feedFowardCommands[0]);
	AddLayersCommands(feedFowardCommands);
	AddCopyOutputCommand();
}

void ndBrainTrainerCpuInference::AddCopyInputCommand(const ndBrainLayer::ndBrainLayerFeedFowardCpuCommand* const uniformData)
{
	const ndBrain& brain = **m_brain;
	ndInt32 inputOutputBufferSize = brain.GetInputSize();
	for (ndInt32 i = 0; i < ndInt32(brain.GetCount()); ++i)
	{
		const ndBrainLayer* const layer = brain[i];
		inputOutputBufferSize += layer->GetOutputSize();
	}

	class ndCopyInputCommand : public ndBrainTrainerCpuCommand
	{
		public:
		ndCopyInputCommand(ndBrainTrainerCpuInference* const owner)
			:ndBrainTrainerCpuCommand(0)
			,m_owner(owner)
			,m_inputSize(0)
			,m_outputSize(0)
			,m_parametersSize(0)
			,m_inputOutputSize(0)
			,m_inputOutputStartOffset(0)
		{
		}

		virtual void Execute(ndInt32 miniBatchIndex)
		{
			const ndBrainMemVector src(&m_owner->m_miniBatchInputBuffer[miniBatchIndex * m_inputSize], m_inputSize);
			ndBrainMemVector dst(&m_owner->m_inputOutputBuffer[miniBatchIndex * m_inputOutputSize], m_inputSize);
			dst.Set(src);
		}

		ndBrainTrainerCpuInference* m_owner;
		ndInt32 m_inputSize;
		ndInt32 m_outputSize;
		ndInt32 m_parametersSize;
		ndInt32 m_inputOutputSize;
		ndInt32 m_inputOutputStartOffset;
	};
	ndCopyInputCommand* const inputCommand = new ndCopyInputCommand(this);
	
	inputCommand->m_inputSize = uniformData->m_inputSize;
	inputCommand->m_outputSize = uniformData->m_outputSize;
	inputCommand->m_parametersSize = 0;
	inputCommand->m_inputOutputSize = inputOutputBufferSize;
	inputCommand->m_inputOutputStartOffset = 0;

	m_feedFowardCommands.Append(ndSharedPtr<ndBrainTrainerCpuCommand>(inputCommand));
}

void ndBrainTrainerCpuInference::AddCopyOutputCommand()
{
	ndCopyOutputCommand* const outputCommand = new ndCopyOutputCommand(this);

	const ndBrainLayer::ndBrainLayerFeedFowardCpuCommand* const lastCommand = (ndBrainLayer::ndBrainLayerFeedFowardCpuCommand*)*m_feedFowardCommands.GetLast()->GetInfo();
	outputCommand->m_inputSize = lastCommand->m_inputSize;
	outputCommand->m_outputSize = lastCommand->m_outputSize;
	outputCommand->m_parametersSize = 0;

	outputCommand->m_inputOutputSize = lastCommand->m_inputOutputSize;
	outputCommand->m_inputOutputStartOffset = lastCommand->m_inputOutputStartOffset + lastCommand->m_inputSize;
	m_feedFowardCommands.Append(ndSharedPtr<ndBrainTrainerCpuCommand>(outputCommand));
}

void ndBrainTrainerCpuInference::AddLayersCommands(ndFixSizeArray<ndBrainLayer::ndBrainLayerFeedFowardCpuCommand*, 256>& layersCommands)
{
	// create all the uniform buffers 
	const ndBrain& brain = **m_brain;

	ndInt32 inputOutputStartOffset = 0;
	ndInt32 inputOutputBufferSize = brain.GetInputSize();
	for (ndInt32 i = 0; i < ndInt32(brain.GetCount()); ++i)
	{
		ndBrainLayer::ndBrainLayerFeedFowardCpuCommand* const data = layersCommands[i];
		data->m_inputOutputStartOffset = inputOutputStartOffset;
		inputOutputBufferSize += data->m_outputSize;
		inputOutputStartOffset += data->m_inputSize;
	}
	
	for (ndInt32 i = 0; i < ndInt32(brain.GetCount()); ++i)
	{
		ndBrainLayer::ndBrainLayerFeedFowardCpuCommand* const layerCommand = layersCommands[i];
		layerCommand->m_inputOutputSize = inputOutputBufferSize;
		m_feedFowardCommands.Append(ndSharedPtr<ndBrainTrainerCpuCommand>(layerCommand));
	}
}

void ndBrainTrainerCpuInference::GetOutput(ndBrainVector& ouput) const
{
	ouput.SetCount(m_miniBatchOutputBuffer.GetCount());
	ouput.Set(m_miniBatchOutputBuffer);
}

// new method
void ndBrainTrainerCpuInference::BackPropagate(const ndBrainVector&)
{
}

void ndBrainTrainerCpuInference::MakePrediction(const ndBrainVector& input)
{
	ndAssert(input.GetCount() == m_miniBatchInputBuffer.GetCount());
	m_miniBatchInputBuffer.Set(input);

	ndAtomic<ndInt32> iterator(0);
	for (ndList<ndSharedPtr<ndBrainTrainerCpuCommand>>::ndNode* node = m_feedFowardCommands.GetFirst(); node; node = node->GetNext())
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
}
