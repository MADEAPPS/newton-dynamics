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

#ifndef _ND_BRAIN_AGENT_SOFT_ACTOR_CRITIC_TRAINER_H__
#define _ND_BRAIN_AGENT_SOFT_ACTOR_CRITIC_TRAINER_H__

#include "ndBrainStdafx.h"

#include "ndBrain.h"
#include "ndBrainAgent.h"
#include "ndBrainAgentDeterministicPolicyGradient_Trainer.h"

// this is an implementation of the soft deterministic actor critc 
// sac algorithm as described in: 
// https://spinningup.openai.com/en/latest/algorithms/sac.html

class ndBrainAgentSoftActorCritic_Trainer : public ndBrainAgentDeterministicPolicyGradient_Trainer
{
	public:
	ndBrainAgentSoftActorCritic_Trainer(const HyperParameters& parameters);
	virtual ~ndBrainAgentSoftActorCritic_Trainer();

	virtual void LearnPolicyFunction() override;
	virtual void CalculateExpectedRewards() override;

	ndBrainFloat CalculatePolicyProbability(ndInt32 index);
	ndBrainFloat CalculateReparametarizedPolicyProbability(ndInt32 index);
	ndBrainFloat CalculatePolicyProbability(ndInt32 index, const ndBrainVector& sampledActions);
};

#endif 
