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

#ifndef _ND_DQN_BRAIN_AGENT_DQN_H__
#define _ND_DQN_BRAIN_AGENT_DQN_H__

#include "ndBrainStdafx.h"
#include "ndBrain.h"
#include "ndBrainAgent.h"
#include "ndBrainTrainer.h"
//#include "ndBrainReplayBuffer.h"

template<ndInt32 statesDim, ndInt32 actionDim>
class ndBrainAgentDQN: public ndBrainAgent
{
	public:
	ndBrainAgentDQN(const ndSharedPtr<ndBrain>& actor);

	void Step();

	protected:
	void ResetModel();
	void OptimizeStep();
	bool IsTrainer() const;
	bool IsTerminal() const;
	ndBrainFloat CalculateReward();
	ndInt32 GetEpisodeFrames() const;
	
	void Save(ndBrainSave* const loadSave);

	void InitWeights();
	ndInt32 SelectBestAction(const ndBrainVector& actions) const;

	ndSharedPtr<ndBrain> m_actor;
};

template<ndInt32 statesDim, ndInt32 actionDim>
ndBrainAgentDQN<statesDim, actionDim>::ndBrainAgentDQN(const ndSharedPtr<ndBrain>& actor)
	:ndBrainAgent()
	,m_actor(actor)
{
}

template<ndInt32 statesDim, ndInt32 actionDim>
bool ndBrainAgentDQN<statesDim, actionDim>::IsTrainer() const
{
	return false;
}

template<ndInt32 statesDim, ndInt32 actionDim>
void ndBrainAgentDQN<statesDim, actionDim>::InitWeights()
{
	ndAssert(0);
}

template<ndInt32 statesDim, ndInt32 actionDim>
bool ndBrainAgentDQN<statesDim, actionDim>::IsTerminal() const
{
	ndAssert(0);
	return false;
}

template<ndInt32 statesDim, ndInt32 actionDim>
ndBrainFloat ndBrainAgentDQN<statesDim, actionDim>::CalculateReward()
{
	ndAssert(0);
	return ndBrainFloat(0.0f);
}

template<ndInt32 statesDim, ndInt32 actionDim>
void ndBrainAgentDQN<statesDim, actionDim>::ResetModel()
{
	ndAssert(0);
}

template<ndInt32 statesDim, ndInt32 actionDim>
ndInt32 ndBrainAgentDQN<statesDim, actionDim>::GetEpisodeFrames() const
{
	ndAssert(0);
	return 0;
}

template<ndInt32 statesDim, ndInt32 actionDim>
void ndBrainAgentDQN<statesDim, actionDim>::Save(ndBrainSave* const)
{
	ndAssert(0);
}

template<ndInt32 statesDim, ndInt32 actionDim>
void ndBrainAgentDQN<statesDim, actionDim>::OptimizeStep()
{
}

template<ndInt32 statesDim, ndInt32 actionDim>
void ndBrainAgentDQN<statesDim, actionDim>::Step()
{
	ndBrainFixSizeVector<actionDim> actions;
	ndBrainFixSizeVector<statesDim> observations;

	GetObservation(&observations[0]);
	m_actor->MakePrediction(observations, actions);

	ndBrainFloat bestAction = ndBrainFloat(actions.ArgMax());
	ApplyActions(&bestAction);
}

#endif 
