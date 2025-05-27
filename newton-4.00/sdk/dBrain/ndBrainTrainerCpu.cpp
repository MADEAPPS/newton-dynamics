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
#include "ndBrainTrainerCpu.h"
#include "ndBrainLayerLinear.h"

ndBrainTrainerCpu::ndBrainTrainerCpu(const ndSharedPtr<ndBrain>& brain, ndBrainThreadPool* const threadPool, ndInt32 minibatchSize)
	:ndBrainTrainerCpuInference(brain, threadPool, minibatchSize)
	,m_inputOuputGradientsBuffer()
	,m_weightAndBiasGradientsBuffer()
	,m_miniBatchInputGradientBuffer()
	,m_miniBatchOutputGradientBuffer()
	,m_backPropagateCommands()
{
	m_miniBatchInputGradientBuffer.SetCount(m_miniBatchInputBuffer.GetCount());
	m_miniBatchOutputGradientBuffer.SetCount(m_miniBatchOutputBuffer.GetCount());

	m_inputOuputGradientsBuffer.SetCount(m_inputOutputBuffer.GetCount() * m_miniBatchSize);
	m_inputOuputGradientsBuffer.Set(ndBrainFloat(0.0f));
	
	m_weightAndBiasGradientsBuffer.SetCount(m_weightAndBiasBuffer.GetCount() * m_miniBatchSize);
	m_weightAndBiasGradientsBuffer.Set(ndBrainFloat(0.0f));

	AddCopyOutputGradientCommand();
	AddLayersGradientCommands();
}

ndBrainTrainerCpu::ndBrainTrainerCpu(const ndBrainTrainerCpu& src)
	:ndBrainTrainerCpuInference(src)
	,m_inputOuputGradientsBuffer()
	,m_weightAndBiasGradientsBuffer()
{
	ndAssert(0);
}

void ndBrainTrainerCpu::AddCopyOutputGradientCommand()
{
	class ndCopyOutputGradientCommand : public ndBrainTrainerCpuCommand
	{
		public:
		ndCopyOutputGradientCommand(ndBrainTrainerCpu* const owner)
			:ndBrainTrainerCpuCommand(10)
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
			const ndBrainMemVector src(&m_owner->m_miniBatchOutputGradientBuffer[miniBatchIndex * m_outputSize], m_outputSize);
			ndInt32 destOffset = miniBatchIndex * m_inputOutputSize * m_owner->m_miniBatchSize;
			ndBrainMemVector dst(&m_owner->m_inputOuputGradientsBuffer[destOffset + m_inputOutputStartOffset], m_outputSize);
			dst.Set(src);
		}

		ndBrainTrainerCpu* m_owner;
		ndInt32 m_inputSize;
		ndInt32 m_outputSize;
		ndInt32 m_parametersSize;
		ndInt32 m_inputOutputSize;
		ndInt32 m_inputOutputStartOffset;
	};
	ndCopyOutputGradientCommand* const outputGradientCommand = new ndCopyOutputGradientCommand(this);

	const ndCopyOutputCommand* const lastCommand = (ndCopyOutputCommand*)*m_feedFowardCommands.GetLast()->GetInfo();
	outputGradientCommand->m_inputSize = lastCommand->m_inputSize;
	outputGradientCommand->m_outputSize = lastCommand->m_outputSize;
	outputGradientCommand->m_parametersSize = 0;
	
	outputGradientCommand->m_inputOutputSize = lastCommand->m_inputOutputSize;
	outputGradientCommand->m_inputOutputStartOffset = lastCommand->m_inputOutputStartOffset;
	m_backPropagateCommands.Append(ndSharedPtr<ndBrainTrainerCpuCommand>(outputGradientCommand));
}

void ndBrainTrainerCpu::AddLayersGradientCommands()
{
	const ndBrain& brain = **m_brain;

int xxxx = 0;
	for (ndInt32 i = ndInt32(brain.GetCount()) - 1; i >= 0; --i)
	{
		auto FindFeedFowardCommand = [this](size_t id)
		{
			for (ndList<ndSharedPtr<ndBrainTrainerCpuCommand>>::ndNode* node = m_feedFowardCommands.GetFirst(); node; node = node->GetNext())
			{
				const ndBrainTrainerCpuCommand* const command = *node->GetInfo();
				if (command->m_id == id)
				{
					return (ndBrainLayer::ndBrainLayerFeedFowardCpuCommand*)command;
				}
			}
			return (ndBrainLayer::ndBrainLayerFeedFowardCpuCommand*)nullptr;
		};
		const ndBrainLayer* const layer = brain[i];
		const ndBrainLayer::ndBrainLayerFeedFowardCpuCommand* const feedFowardCommand = FindFeedFowardCommand(size_t(layer));
		ndAssert(feedFowardCommand);

		ndBrainLayer::ndBrainLayerBackPropagateCpuCommand* const command = layer->GetLayerCpuBackPropagateCommand();
		command->m_owner = this;

		ndAssert(command->m_inputSize == feedFowardCommand->m_inputSize);
		ndAssert(command->m_outputSize == feedFowardCommand->m_outputSize);
		command->m_parametersSize = feedFowardCommand->m_parametersSize;
		command->m_parametersStartOffset = feedFowardCommand->m_parametersStartOffset;
		command->m_inputOutputSize = feedFowardCommand->m_inputOutputSize;
		command->m_inputOutputStartOffset = feedFowardCommand->m_inputOutputStartOffset;

		m_backPropagateCommands.Append(command);

		if (xxxx >= 1)
			break;
		xxxx++;
	}
}

void ndBrainTrainerCpu::BackPropagate(const ndBrainVector& outputGradients)
{
	ndAssert(outputGradients.GetCount() == m_miniBatchOutputGradientBuffer.GetCount());
	m_miniBatchOutputGradientBuffer.Set(outputGradients);

	ndAtomic<ndInt32> iterator(0);
	for (ndList<ndSharedPtr<ndBrainTrainerCpuCommand>>::ndNode* node = m_backPropagateCommands.GetFirst(); node; node = node->GetNext())
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