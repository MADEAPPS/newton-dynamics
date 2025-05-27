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

ndBrainTrainerGpu::ndBrainTrainerGpu(const ndSharedPtr<ndBrain>& brain, const ndSharedPtr<ndBrainGpuContext>& context, ndInt32 minibatchSize)
	:ndBrainTrainnerGpuInference(brain, context, minibatchSize)
	,m_inputOuputGradientsBuffer()
	,m_weightAndBiasGradientsBuffer()
{
	ndBrainVector buffer;
	GetWorkingBuffer(buffer);
	buffer.Set(ndReal(0.0f));
	m_inputOuputGradientsBuffer = ndSharedPtr<ndBrainGpuBuffer>(new ndBrainGpuFloatBuffer(*m_context, buffer, ndCpuMappable));

	GetParameterBuffer(buffer);
	buffer.SetCount(buffer.GetCount() * m_miniBatchSize);
	buffer.Set(ndReal(0.0f));
	m_weightAndBiasGradientsBuffer = ndSharedPtr<ndBrainGpuBuffer>(new ndBrainGpuFloatBuffer(*m_context, buffer, ndCpuMappable));
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

void ndBrainTrainerGpu::BackPropagate(const ndBrainVector& outputGradients)
{
	//m_context->BeginQueue();
	//m_miniBatchInputBuffer->LoadData(input.GetCount() * sizeof(ndReal), &input[0]);
	////m_groundTruth->LoadData(groundTruth.GetCount() * sizeof(ndReal), &groundTruth[0]);
	//for (ndList<ndSharedPtr<ndBrainGpuCommand>>::ndNode* node = m_feedFowardCommands.GetFirst(); node; node = node->GetNext())
	//{
	//	ndSharedPtr<ndBrainGpuCommand>& command = node->GetInfo();
	//	m_context->AddCommandQueue(command);
	//}
	//m_context->EndQueue();
}