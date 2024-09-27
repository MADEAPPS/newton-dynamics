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
#include "ndBrainVector.h"

#include "ndBrainAgentContinuePolicyGradient.h"

ndBrainAgentContinuePolicyGradient::ndBrainAgentContinuePolicyGradient(const ndSharedPtr<ndBrain>& actor)
	:ndBrainAgent()
	,m_policy(actor)
{
}

ndBrainAgentContinuePolicyGradient::ndBrainAgentContinuePolicyGradient(const ndBrainAgentContinuePolicyGradient& src)
	:ndBrainAgent(src)
	,m_policy(src.m_policy)
{
}

ndBrainAgentContinuePolicyGradient::~ndBrainAgentContinuePolicyGradient()
{
}

bool ndBrainAgentContinuePolicyGradient::IsTrainer() const
{
	return false;
}

void ndBrainAgentContinuePolicyGradient::InitWeights()
{
	ndAssert(0);
}

bool ndBrainAgentContinuePolicyGradient::IsTerminal() const
{
	ndAssert(0);
	return false;
}

ndBrainFloat ndBrainAgentContinuePolicyGradient::CalculateReward()
{
	ndAssert(0);
	return ndBrainFloat(0.0f);
}

void ndBrainAgentContinuePolicyGradient::ResetModel()
{
	ndAssert(0);
}

ndInt32 ndBrainAgentContinuePolicyGradient::GetEpisodeFrames() const
{
	ndAssert(0);
	return 0;
}


void ndBrainAgentContinuePolicyGradient::Save(ndBrainSave* const)
{
	ndAssert(0);
}

void ndBrainAgentContinuePolicyGradient::OptimizeStep()
{
}

void ndBrainAgentContinuePolicyGradient::Step()
{
	ndInt32 bufferSize = m_policy->CalculateWorkingBufferSize();
	ndBrainFloat* const bufferMem = ndAlloca(ndBrainFloat, bufferSize);
	ndBrainFloat* const actionBuffer = ndAlloca(ndBrainFloat, m_policy->GetOutputSize());
	ndBrainFloat* const observationBuffer = ndAlloca(ndBrainFloat, m_policy->GetInputSize());
	
	ndBrainMemVector workingBuffer(bufferMem, bufferSize);
	ndBrainMemVector actions(actionBuffer, m_policy->GetOutputSize());
	ndBrainMemVector observations(observationBuffer, m_policy->GetInputSize());
	
	GetObservation(observationBuffer);
	m_policy->MakePrediction(observations, actions, workingBuffer);
	ApplyActions(&actions[0]);
}
