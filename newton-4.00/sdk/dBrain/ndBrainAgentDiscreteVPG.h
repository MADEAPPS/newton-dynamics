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

#ifndef _ND_AGENT_DESCRETE_VPG_H__
#define _ND_AGENT_DESCRETE_VPG_H__

#include "ndBrainStdafx.h"
#include "ndBrain.h"
#include "ndBrainAgent.h"
#include "ndBrainTrainer.h"
#include "ndBrainReplayBuffer.h"

template<ndInt32 statesDim, ndInt32 actionDim>
class ndBrainAgentDiscreteVPG: public ndBrainAgent
{
	public:
	ndBrainAgentDiscreteVPG(const ndSharedPtr<ndBrain>& actor);
	~ndBrainAgentDiscreteVPG();

	void Step();

	protected:
	void OptimizeStep();
	bool IsTrainer() const;
	void ResetModel() const;
	bool IsTerminal() const;
	ndBrainFloat GetReward() const;
	ndInt32 GetEpisodeFrames() const;
	void AddExploration(ndBrainFloat* const actions);

	void Save(ndBrainSave* const loadSave) const;

	void InitWeights();
	void InitWeights(ndBrainFloat weighVariance, ndBrainFloat biasVariance);

	ndSharedPtr<ndBrain> m_actor;
};

template<ndInt32 statesDim, ndInt32 actionDim>
ndBrainAgentDiscreteVPG<statesDim, actionDim>::ndBrainAgentDiscreteVPG(const ndSharedPtr<ndBrain>& actor)
	:ndBrainAgent()
	,m_actor(actor)
{
	ndAssert(0);
}

template<ndInt32 statesDim, ndInt32 actionDim>
ndBrainAgentDiscreteVPG<statesDim, actionDim>::~ndBrainAgentDiscreteVPG()
{
	ndAssert(0);
}

template<ndInt32 statesDim, ndInt32 actionDim>
bool ndBrainAgentDiscreteVPG<statesDim, actionDim>::IsTrainer() const
{
	return false;
}

template<ndInt32 statesDim, ndInt32 actionDim>
void ndBrainAgentDiscreteVPG<statesDim, actionDim>::InitWeights()
{
	ndAssert(0);
}

template<ndInt32 statesDim, ndInt32 actionDim>
void ndBrainAgentDiscreteVPG<statesDim, actionDim>::InitWeights(ndBrainFloat, ndBrainFloat)
{
	ndAssert(0);
}

template<ndInt32 statesDim, ndInt32 actionDim>
bool ndBrainAgentDiscreteVPG<statesDim, actionDim>::IsTerminal() const
{
	ndAssert(0);
	return false;
}

template<ndInt32 statesDim, ndInt32 actionDim>
void ndBrainAgentDiscreteVPG<statesDim, actionDim>::AddExploration(ndBrainFloat* const)
{
	ndAssert(0);
}

template<ndInt32 statesDim, ndInt32 actionDim>
ndBrainFloat ndBrainAgentDiscreteVPG<statesDim, actionDim>::GetReward() const
{
	ndAssert(0);
	return ndBrainFloat(0.0f);
}

template<ndInt32 statesDim, ndInt32 actionDim>
void ndBrainAgentDiscreteVPG<statesDim, actionDim>::ResetModel() const
{
	ndAssert(0);
}

template<ndInt32 statesDim, ndInt32 actionDim>
ndInt32 ndBrainAgentDiscreteVPG<statesDim, actionDim>::GetEpisodeFrames() const
{
	ndAssert(0);
	return 0;
}

template<ndInt32 statesDim, ndInt32 actionDim>
void ndBrainAgentDiscreteVPG<statesDim, actionDim>::Save(ndBrainSave* const) const
{
	ndAssert(0);
}

template<ndInt32 statesDim, ndInt32 actionDim>
void ndBrainAgentDiscreteVPG<statesDim, actionDim>::OptimizeStep()
{
}

template<ndInt32 statesDim, ndInt32 actionDim>
void ndBrainAgentDiscreteVPG<statesDim, actionDim>::Step()
{
	ndBrainFloat stateBuffer[statesDim * 2];
	ndBrainFloat actionBuffer[actionDim * 2];
	ndBrainMemVector state(stateBuffer, statesDim);
	ndBrainMemVector actions(actionBuffer, actionDim);

	GetObservation(&state[0]);
	m_actor->MakePrediction(state, actions);

	ndBrainFloat bestAction = ndBrainFloat(actions.ArgMax());
	ApplyActions(&bestAction);
}

#endif 
