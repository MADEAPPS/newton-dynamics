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

#ifndef _ND_AGENT_CONTINUE_POLICY_GRADIENT_H__
#define _ND_AGENT_CONTINUE_POLICY_GRADIENT_H__

#include "ndBrainStdafx.h"
#include "ndBrain.h"
#include "ndBrainAgent.h"

class ndBrainAgentContinuePolicyGradient: public ndBrainAgent
{
	public:
	ndBrainAgentContinuePolicyGradient(const ndSharedPtr<ndBrain>& actor);
	ndBrainAgentContinuePolicyGradient(const ndBrainAgentContinuePolicyGradient& src);
	~ndBrainAgentContinuePolicyGradient();

	void Step() override;

	protected:
	void InitWeights() override;
	void ResetModel() override;
	void OptimizeStep() override;
	bool IsTrainer() const override;
	bool IsTerminal() const override;
	ndBrainFloat CalculateReward() override;
	ndInt32 GetEpisodeFrames() const override;
	void Save(ndBrainSave* const loadSave) override;
	
	ndBrainVector m_actions;
	ndBrainVector m_observations;
};

#endif 
