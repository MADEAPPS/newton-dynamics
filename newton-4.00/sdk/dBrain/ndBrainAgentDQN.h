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

#ifndef _ND_BRAIN_AGENT_DQN_H__
#define _ND_BRAIN_AGENT_DQN_H__

#include "ndBrainStdafx.h"
#include "ndBrainAgent.h"

template<ndInt32 statesDim, ndInt32 actionDim>
class ndBrainAgentDQN: public ndBrainAgent
{
	public: 
	ndBrainAgentDQN();
	virtual ~ndBrainAgentDQN();

	ndBrainReplayTransitionMemory<ndInt32, statesDim, 1>& GetTransition();

	virtual void LearnStep();

	private:
	ndInt32 GetAction() const;

	protected:
	ndBrainReplayBuffer<ndInt32, statesDim, 1> m_replayBuffer;
	ndBrainReplayTransitionMemory<ndInt32, statesDim, 1> m_currentTransition;

	ndReal m_gamma;
	ndReal m_epsilonGreedy;
	ndReal m_epsilonGreedyStep;
	ndReal m_epsilonGreedyFloor;
	ndInt32 m_frameCount;
	ndInt32 m_eposideCount;
	ndInt32 m_epsilonGreedyFreq;
};

template<ndInt32 statesDim, ndInt32 actionDim>
ndBrainAgentDQN<statesDim, actionDim>::ndBrainAgentDQN()
	:ndBrainAgent()
	,m_replayBuffer()
	,m_gamma(ndReal(0.99f))
	,m_epsilonGreedy(ndReal(1.0f))
	,m_epsilonGreedyStep(ndReal(5.0e-4f))
	,m_epsilonGreedyFloor(ndReal(2.0e-3f))
	,m_frameCount(0)
	,m_eposideCount(0)
	,m_epsilonGreedyFreq(64)
{
}

template<ndInt32 statesDim, ndInt32 actionDim>
ndBrainAgentDQN<statesDim, actionDim>::~ndBrainAgentDQN()
{
}

template<ndInt32 statesDim, ndInt32 actionDim>
ndBrainReplayTransitionMemory<ndInt32, statesDim, 1>& ndBrainAgentDQN<statesDim, actionDim>::GetTransition()
{
	return m_currentTransition;
}

template<ndInt32 statesDim, ndInt32 actionDim>
ndInt32 ndBrainAgentDQN<statesDim, actionDim>::GetAction() const
{
	ndInt32 action = 0;
	ndFloat32 explore = ndRand();
	if (explore <= m_epsilonGreedy)
	{
		action = ndInt32(ndRandInt() % m_actionsCount);
	}
	else
	{
		ndAssert(0);
		//action = m_agent->GetMaxValueAction();
		//for (ndInt32 i = 0; i < m_stateCount; ++i)
		//{
		//	m_input[i] = m_currentTransition.m_state[i];
		//}
		//ndBrainInstance& instance = m_trainer.GetInstance();
		//instance.MakePrediction(m_input, m_output);
		//
		//ndReal maxReward = m_output[0];
		//for (ndInt32 i = 1; i < m_actionsCount; ++i)
		//{
		//	if (m_output[i] > maxReward)
		//	{
		//		action = i;
		//		maxReward = m_output[i];
		//	}
		//}
	}

	return action;
}

template<ndInt32 statesDim, ndInt32 actionDim>
void ndBrainAgentDQN<statesDim, actionDim>::LearnStep()
{
	if (!m_frameCount)
	{
		ResetModel();
	}

	GetObservation(&m_currentTransition.m_nextState[0]);
	m_currentTransition.m_reward = GetReward();
	m_currentTransition.m_terminalState = IsTerminal();

	m_replayBuffer.AddTransition(m_currentTransition);

	m_currentTransition.m_state = m_currentTransition.m_nextState;
	m_currentTransition.m_action[0] = GetAction();

	if (m_currentTransition.m_terminalState)
	{
		m_eposideCount++;
		ResetModel();
	}

	if (m_frameCount % m_epsilonGreedyFreq == (m_epsilonGreedyFreq - 1))
	{
		m_epsilonGreedy = ndMax(m_epsilonGreedy - m_epsilonGreedyStep, m_epsilonGreedyFloor);
	}

	m_frameCount++;
}

#endif 

