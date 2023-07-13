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

#ifndef _ND_DQN_BRAIN_AGENT_DDPG_H__
#define _ND_DQN_BRAIN_AGENT_DDPG_H__

#include "ndBrainStdafx.h"
#include "ndBrain.h"
#include "ndBrainAgent.h"
#include "ndBrainTrainer.h"
#include "ndBrainReplayBuffer.h"

template<ndInt32 statesDim, ndInt32 actionDim>
class ndBrainAgentDDPG: public ndBrainAgent
{
	public:
	ndBrainAgentDDPG(const ndSharedPtr<ndBrain>& actor);

	void Step();

	protected:
	void OptimizeStep();
	void ResetModel() const;
	bool IsTerminal() const;
	ndReal GetReward() const;
	ndReal GetCurrentValue() const;
	void ApplyRandomAction() const;
	ndInt32 GetEpisodeFrames() const;
	ndInt32 GetOpmizationDelay() const;
	void SetOpmizationDelay(ndInt32 delay);
	void Save(ndBrainSave* const loadSave) const;

	ndSharedPtr<ndBrain> m_actor;
	ndBrainVector m_state;
	ndBrainVector m_actions;
};

template<ndInt32 statesDim, ndInt32 actionDim>
ndBrainAgentDDPG<statesDim, actionDim>::ndBrainAgentDDPG(const ndSharedPtr<ndBrain>& actor)
	:ndBrainAgent()
	,m_actor(actor)
{
	m_state.SetCount(statesDim);
	m_actions.SetCount(actionDim);
	m_state.Set(ndReal(0.0f));
	m_actions.Set(ndReal(0.0f));
}

template<ndInt32 statesDim, ndInt32 actionDim>
void ndBrainAgentDDPG<statesDim, actionDim>::ApplyRandomAction() const
{
	ndAssert(0);
}

template<ndInt32 statesDim, ndInt32 actionDim>
ndInt32 ndBrainAgentDDPG<statesDim, actionDim>::GetOpmizationDelay() const
{
	ndAssert(0);
	return 0;
}

template<ndInt32 statesDim, ndInt32 actionDim>
void ndBrainAgentDDPG<statesDim, actionDim>::SetOpmizationDelay(ndInt32)
{
	ndAssert(0);
}

template<ndInt32 statesDim, ndInt32 actionDim>
bool ndBrainAgentDDPG<statesDim, actionDim>::IsTerminal() const
{
	ndAssert(0);
	return false;
}

template<ndInt32 statesDim, ndInt32 actionDim>
ndReal ndBrainAgentDDPG<statesDim, actionDim>::GetReward() const
{
	ndAssert(0);
	return ndReal(0.0f);
}

template<ndInt32 statesDim, ndInt32 actionDim>
ndReal ndBrainAgentDDPG<statesDim, actionDim>::GetCurrentValue() const
{
	return ndReal(0.0f);
}

template<ndInt32 statesDim, ndInt32 actionDim>
void ndBrainAgentDDPG<statesDim, actionDim>::ResetModel() const
{
	ndAssert(0);
}

template<ndInt32 statesDim, ndInt32 actionDim>
void ndBrainAgentDDPG<statesDim, actionDim>::Save(ndBrainSave* const) const
{
	ndAssert(0);
}

template<ndInt32 statesDim, ndInt32 actionDim>
ndInt32 ndBrainAgentDDPG<statesDim, actionDim>::GetEpisodeFrames() const
{
	ndAssert(0);
	return 0;
}

template<ndInt32 statesDim, ndInt32 actionDim>
void ndBrainAgentDDPG<statesDim, actionDim>::OptimizeStep()
{
}

template<ndInt32 statesDim, ndInt32 actionDim>
void ndBrainAgentDDPG<statesDim, actionDim>::Step()
{
	GetObservation(&m_state[0]);
	m_actor->MakePrediction(m_state, m_actions);
	ApplyActions(&m_actions[0]);
}

#endif 
