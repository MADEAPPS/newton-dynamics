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

ndBrainTrainerCpu::ndBrainTrainerCpu(
	const ndSharedPtr<ndBrain>& brain, 
	const ndSharedPtr<ndBrainContext>& context,
	ndBrainFloat learnRate,
	ndInt32 minibatchSize)
	:ndBrainTrainerCpuInference(brain, context, minibatchSize)
	,m_optimizer(new ndBrainOptimizerAdamCpu(context))
	,m_inputOuputGradientsBuffer()
	,m_weightAndBiasGradientsBuffer()
	,m_miniBatchInputGradientBuffer()
	,m_miniBatchOutputGradientBuffer()
	,m_backPropagateCommands()
{
	m_optimizer->Init(ndInt32(m_weightAndBiasBuffer.GetCount()), learnRate);

	m_miniBatchInputGradientBuffer.SetCount(m_miniBatchInputBuffer->m_buffer.GetCount());
	m_miniBatchOutputGradientBuffer.SetCount(m_miniBatchOutputBuffer.GetCount());

	m_inputOuputGradientsBuffer.SetCount(m_inputOutputBuffer.GetCount() * m_miniBatchSize);
	m_inputOuputGradientsBuffer.Set(ndBrainFloat(0.0f));
	
	m_weightAndBiasGradientsBuffer.SetCount(m_weightAndBiasBuffer.GetCount() * m_miniBatchSize);
	m_weightAndBiasGradientsBuffer.Set(ndBrainFloat(0.0f));

	AddCopyOutputGradientCommand();
	AddLayersGradientCommands();
	AddCopyInputGradientCommand();
}

ndBrainTrainerCpu::ndBrainTrainerCpu(const ndBrainTrainerCpu& src)
	:ndBrainTrainerCpuInference(src)
	,m_optimizer(src.m_optimizer)
	,m_inputOuputGradientsBuffer()
	,m_weightAndBiasGradientsBuffer()
	,m_miniBatchInputGradientBuffer()
	,m_miniBatchOutputGradientBuffer()
	,m_backPropagateCommands()
{
	ndAssert(0);
}

void ndBrainTrainerCpu::SaveInput(ndBrainVector& input) const
{
	input.SetCount(m_miniBatchInputGradientBuffer.GetCount());
	input.Set(m_miniBatchInputGradientBuffer);
}

void ndBrainTrainerCpu::AddCopyOutputGradientCommand()
{
	ndCopyOutputGradientCommand* const outputGradientCommand = new ndCopyOutputGradientCommand(this);
	ndAssert(FindCommand(ndBrainTrainerCpuInference::m_outpuId));
	const ndCopyOutputCommand* const lastCommand = (ndCopyOutputCommand*)FindCommand(ndBrainTrainerCpuInference::m_outpuId);
	outputGradientCommand->m_info.m_inputSize = 0;
	outputGradientCommand->m_info.m_outputSize = lastCommand->m_info.m_outputSize;
	outputGradientCommand->m_info.m_inputOutputSize = lastCommand->m_info.m_inputOutputSize;
	outputGradientCommand->m_info.m_inputOutputStartOffset = lastCommand->m_info.m_inputOutputStartOffset;
	m_backPropagateCommands.Append(ndSharedPtr<ndBrainTrainerCpuCommand>(outputGradientCommand));
}

void ndBrainTrainerCpu::AddCopyInputGradientCommand()
{
	ndCopyInputGradientCommand* const inputGradientCommand = new ndCopyInputGradientCommand(this);
	
	ndAssert(FindCommand(ndBrainTrainerCpuInference::m_inputId));
	const ndCopyInputCommand* const firstCommand = (ndCopyInputCommand*)FindCommand(ndBrainTrainerCpuInference::m_inputId);
	
	inputGradientCommand->m_info.m_inputSize = firstCommand->m_info.m_inputSize;
	//inputGradientCommand->m_outputSize = 0;
	//inputGradientCommand->m_parametersSize = 0;
	
	inputGradientCommand->m_info.m_inputOutputSize = firstCommand->m_info.m_inputOutputSize;
	inputGradientCommand->m_info.m_inputOutputStartOffset = firstCommand->m_info.m_inputOutputStartOffset;
	m_backPropagateCommands.Append(ndSharedPtr<ndBrainTrainerCpuCommand>(inputGradientCommand));
}

void ndBrainTrainerCpu::AddLayersGradientCommands()
{
	const ndBrain& brain = **m_brain;
	
	for (ndInt32 i = ndInt32(brain.GetCount()) - 1; i >= 0; --i)
	{
		ndBrainLayer* const layer = brain[i];
		ndAssert(FindCommand(size_t(layer)));
		const ndBrainTrainerCpuCommand* const feedForwardCommand = FindCommand(size_t(layer));
		ndAssert(feedForwardCommand);
	
		ndBrainLayerBackPropagateCpuCommand* const command = layer->GetLayerCpuBackPropagateCommand();
		ndAssert(command->m_info.m_inputSize == feedForwardCommand->m_info.m_inputSize);
		ndAssert(command->m_info.m_outputSize == feedForwardCommand->m_info.m_outputSize);

		command->m_owner = this;
		command->m_info.m_layer = layer;
		command->m_info.m_parametersBatchSize = feedForwardCommand->m_info.m_parametersBatchSize;
		command->m_info.m_parametersStartOffset = feedForwardCommand->m_info.m_parametersStartOffset;
		command->m_info.m_inputOutputSize = feedForwardCommand->m_info.m_inputOutputSize;
		command->m_info.m_inputOutputStartOffset = feedForwardCommand->m_info.m_inputOutputStartOffset;
	
		m_backPropagateCommands.Append(command);
	}
}

void ndBrainTrainerCpu::ApplyLearnRate()
{
	ndAtomic<ndInt32> iterator(0);
	auto AddGradients = ndMakeObject::ndFunction([this, &iterator](ndInt32, ndInt32)
	{
		ndFloat32 scale = ndFloat32(1.0f) / ndFloat32(m_miniBatchSize);

		const ndInt32 stride = 1024 * 4;
		const ndInt32 size = ndInt32(m_weightAndBiasBuffer.GetCount());
		const ndInt32 slices = (ndInt32(m_weightAndBiasBuffer.GetCount() + stride - 1)) / stride;
		for (ndInt32 i = iterator++; i < slices; i = iterator++)
		{
			ndInt32 start = i * stride;
			ndInt32 end = ((start + stride) > size) ? size : start + stride;
			ndInt32 count = end - start;

			ndBrainMemVector dst(&m_weightAndBiasGradientsBuffer[start], count);
			for (ndInt32 j = 1; j < m_miniBatchSize; ++j)
			{
				ndInt32 rowBase = j * size + start;
				const ndBrainMemVector src(&m_weightAndBiasGradientsBuffer[rowBase], count);
				dst.Add(src);
			}
			dst.Scale(scale);
		}
	});
	m_threadPool->ndBrainThreadPool::ParallelExecute(AddGradients);

	const ndBrainMemVector gradients(&m_weightAndBiasGradientsBuffer[0], m_weightAndBiasBuffer.GetCount());
	m_optimizer->Update(m_weightAndBiasBuffer, gradients);
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