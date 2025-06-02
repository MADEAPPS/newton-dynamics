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
#include "ndBrainTrainerGpu.h"
#include "ndBrainGpuContext.h"
#include "ndBrainGpuCommand.h"
#include "ndBrainLayerLinear.h"
#include "ndBrainGpuFloatBuffer.h"
#include "ndBrainGpuUniformBuffer.h"

ndBrainTrainerGpu::ndBrainTrainerGpu(const ndSharedPtr<ndBrain>& brain, const ndSharedPtr<ndBrainContext>& context, ndInt32 minibatchSize)
	:ndBrainTrainnerGpuInference(brain, context, minibatchSize)
	,m_inputOuputGradientsBuffer()
	,m_weightAndBiasGradientsBuffer()
	,m_miniBatchInputGradientBuffer()
	,m_miniBatchOutputGradientBuffer()
{
	ndBrainVector buffer;

	GetInput(buffer);
	buffer.Set(ndReal(0.0f));
	m_miniBatchInputGradientBuffer = ndSharedPtr<ndBrainGpuBuffer>(new ndBrainGpuFloatBuffer(m_context, buffer, ndCpuMappable));

	GetOutput(buffer);
	buffer.Set(ndReal(0.0f));
	m_miniBatchOutputGradientBuffer = ndSharedPtr<ndBrainGpuBuffer>(new ndBrainGpuFloatBuffer(m_context, buffer, ndCpuMappable));

	GetWorkingBuffer(buffer);
	buffer.SetCount(buffer.GetCount() * m_miniBatchSize);
	buffer.Set(ndReal(0.0f));
	m_inputOuputGradientsBuffer = ndSharedPtr<ndBrainGpuBuffer>(new ndBrainGpuFloatBuffer(m_context, buffer, ndCpuMappable));

	GetParameterBuffer(buffer);
	buffer.SetCount(buffer.GetCount() * m_miniBatchSize);
	buffer.Set(ndReal(0.0f));
	m_weightAndBiasGradientsBuffer = ndSharedPtr<ndBrainGpuBuffer>(new ndBrainGpuFloatBuffer(m_context, buffer, ndCpuMappable));

	AddCopyOutputGradientCommand();
}

ndBrainTrainerGpu::ndBrainTrainerGpu(const ndBrainTrainerGpu& src)
	:ndBrainTrainnerGpuInference(src)
	,m_inputOuputGradientsBuffer()
	,m_weightAndBiasGradientsBuffer()
{
	ndAssert(0);
}

ndBrainTrainerGpu::~ndBrainTrainerGpu()
{
}

void ndBrainTrainerGpu::AddCopyOutputGradientCommand()
{
	ndAssert(0);
	//ndUniformBufferObject data;
	//ndAssert(FindCommand(ndBrainTrainnerGpuInference::m_outpuId));
	//ndGpuCommand* const lastCommand = (ndGpuCommand*)FindCommand(m_outpuId);
	//
	//const ndBrainGpuBuffer& uniformData = **lastCommand->m_uniformBuffer;
	//uniformData.UnloadData(ndInt32(sizeof(ndUniformBufferObject)), &data);
	// 
	//data.m_inputSize = 0;
	//data.m_parametersStartOffset = 0;
	//data.m_inputOutputStartOffset += data.m_inputSize;
	//ndSharedPtr<ndBrainGpuBuffer> uniformbuffer(new ndBrainGpuUniformBuffer(m_context, sizeof(ndUniformBufferObject)));
	//uniformbuffer->LoadData(sizeof(ndUniformBufferObject), &data);
	//
	////ndBrainGpuBuffer* const inputOutputBuffer = *m_inputOutputBuffer;
	//ndBrainGpuBuffer* const miniBatchOutputBuffer = *m_miniBatchOutputGradientBuffer;
	////ndSharedPtr<ndBrainGpuCommand> command(new ndGpuCommand(m_context, m_context->m_ndBrainCopyOutput, m_miniBatchSize, *uniformbuffer, inputOutputBuffer, miniBatchOutputBuffer));
	////m_feedFowardCommands.Append(command);
}

//void ndBrainTrainerGpu::ApplyLearnRate(ndBrainFloat learnRate)
void ndBrainTrainerGpu::ApplyLearnRate(ndBrainFloat)
{
	//ndAssert(0);
}

void ndBrainTrainerGpu::SubmitBackwardCommands()
{
	//for (ndList<ndSharedPtr<ndBrainGpuCommand>>::ndNode* node = m_feedFowardCommands.GetFirst(); node; node = node->GetNext())
	//{
	//	ndSharedPtr<ndBrainGpuCommand>& command = node->GetInfo();
	//	m_context->AddCommandQueue(command);
	//}
}

void ndBrainTrainerGpu::BackPropagate(const ndBrainVector& outputGradients)
{
	m_context->BeginQueue();
	m_miniBatchOutputGradientBuffer->LoadData(outputGradients.GetCount() * sizeof(ndReal), &outputGradients[0]);
	SubmitBackwardCommands();
	m_context->EndQueue();
}