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
#include "ndBrainVector.h"
#include "ndBrainAgentDeterministicPolicyGradient.h"

ndBrainAgentDeterministicPolicyGradient::ndBrainAgentDeterministicPolicyGradient(const ndSharedPtr<ndBrain>& actor)
	:ndBrainAgent()
	,m_actor(actor)
{
}

bool ndBrainAgentDeterministicPolicyGradient::IsTrainer() const
{
	return false;
}


bool ndBrainAgentDeterministicPolicyGradient::IsTerminal() const
{
	ndAssert(0);
	return false;
}

ndBrainFloat ndBrainAgentDeterministicPolicyGradient::CalculateReward()
{
	ndAssert(0);
	return ndBrainFloat(0.0f);
}

void ndBrainAgentDeterministicPolicyGradient::ResetModel()
{
	ndAssert(0);
}

void ndBrainAgentDeterministicPolicyGradient::InitWeights()
{
	ndAssert(0);
}


void ndBrainAgentDeterministicPolicyGradient::Save(ndBrainSave* const)
{
	ndAssert(0);
}

ndInt32 ndBrainAgentDeterministicPolicyGradient::GetEpisodeFrames() const
{
	ndAssert(0);
	return 0;
}

void ndBrainAgentDeterministicPolicyGradient::OptimizeStep()
{
}

void ndBrainAgentDeterministicPolicyGradient::Step()
{
	ndAssert(0);
	ndBrainFixSizeVector<256> actions;
	ndBrainFixSizeVector<256> observations;

	GetObservation(&observations[0]);
	m_actor->MakePrediction(observations, actions);
	ApplyActions(&actions[0]);
}


