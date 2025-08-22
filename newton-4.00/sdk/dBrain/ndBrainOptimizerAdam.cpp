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
#include "ndBrainMatrix.h"
#include "ndBrainCpuContext.h"
#include "ndBrainThreadPool.h"
#include "ndBrainFloatBuffer.h"
#include "ndBrainOptimizerAdam.h"
#include "ndBrainTrainerInference.h"

ndBrainOptimizerAdam::ndBrainOptimizerAdam(const ndSharedPtr<ndBrainContext>& context)
	:ndBrainOptimizer(context)
	,m_vdw()
	,m_vdw2()
	,m_parameters()
{
}

void ndBrainOptimizerAdam::Init(ndInt32 parametersBufferSizeInFloats)
{
	ndBrainVector buffer;
	buffer.SetCount(parametersBufferSizeInFloats);
	buffer.Set(ndReal(0.0f));

	m_parameters.m_decayRegularizer = GetRegularizer();
	m_vdw = ndSharedPtr<ndBrainFloatBuffer>(new ndBrainFloatBuffer(*m_context, buffer));
	m_vdw2 = ndSharedPtr<ndBrainFloatBuffer>(new ndBrainFloatBuffer(*m_context, buffer));
}

void ndBrainOptimizerAdam::ApplyLearnRate(ndBrainFloat learnRate)
{ 
	m_context->ApplyLeanRateCommands(*m_commands.GetFirst()->GetInfo(), learnRate);
	m_context->SubmitBufferCommand(*m_commands.GetFirst()->GetNext()->GetInfo());
}