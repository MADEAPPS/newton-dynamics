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

#ifndef _ND_BRAIN_REPLAY_BUFFER_H__
#define _ND_BRAIN_REPLAY_BUFFER_H__

#include "ndBrainStdafx.h"
#include "ndBrainVector.h"
#include "ndBrainMatrix.h"
#include "ndBrainInstance.h"

template<class Action, ndInt32 statesCount>
class ndBrainReiforcementTransition
{
	public:
	ndBrainReiforcementTransition();

	void Clear();
	//void CopyFrom(const ndBrainReiforcementTransition& src);

	ndFixSizeArray<ndReal, statesCount> m_state;
	ndFixSizeArray<ndReal, statesCount> m_nextState;
	Action m_action;
	ndReal m_reward;
	bool m_terminalState;
};

template<class Action, ndInt32 statesCount>
class ndBrainReplayBuffer : public ndArray<ndBrainReiforcementTransition<Action, statesCount>>
{
	public:
	ndBrainReplayBuffer();
	~ndBrainReplayBuffer();

	//void SetCount(ndInt32 replayBufferSize, ndInt32 replayBatchSize, ndInt32 stateSize, ndInt32 actionSize);
	void SetCount(ndInt32 replayBufferSize, ndInt32 replayBatchSize);

	void AddTransition(const ndBrainReiforcementTransition<Action, statesCount>& transition);
	//ndBrainReiforcementTransition& GetTransitionEntry();
	//void MakeRandomBatch();

	ndArray<ndUnsigned32> m_randomShaffle;
	//ndBrainMatrix m_inputBatch;
	//ndBrainMatrix m_outputBatch;
	//ndBrainMatrix m_nextInputBatch;
	//ndBrainMatrix m_groundTruthBatch;
	//ndBrainVector n_rewardBatch;
	//ndBrainVector n_terminalBatch;
	//ndInt32 m_learnBatchSize;
	ndInt32 m_replayBufferIndex;
};

template<class Action, ndInt32 statesCount>
ndBrainReiforcementTransition<Action, statesCount>::ndBrainReiforcementTransition()
	:m_state()
	,m_nextState()
	,m_action()
	,m_reward(0.0f)
	,m_terminalState(false)
{
}

template<class Action, ndInt32 statesCount>
void ndBrainReiforcementTransition<Action, statesCount>::Clear()
{
	m_state.SetCount(0);
	m_nextState.SetCount(0);
	for (ndInt32 i = 0; i < statesCount; ++i)
	{
		m_state.PushBack(ndReal(0.0f));
		m_nextState.PushBack(ndReal(0.0f));
	}

	m_action = 0;
	m_reward = ndReal(0.0f);
	m_terminalState = false;
}

template<class Action, ndInt32 statesCount>
ndBrainReplayBuffer<Action, statesCount>::ndBrainReplayBuffer()
	:ndArray<ndBrainReiforcementTransition<Action, statesCount>>()
	,m_randomShaffle()
	//,m_inputBatch()
	//,m_outputBatch()
	//,m_nextInputBatch()
	//,m_groundTruthBatch()
	//,n_rewardBatch()
	//,n_terminalBatch()
	//,m_learnBatchSize(0)
	,m_replayBufferIndex(0)
{
}

template<class Action, ndInt32 statesCount>
ndBrainReplayBuffer<Action, statesCount>::~ndBrainReplayBuffer()
{
}

template<class Action, ndInt32 statesCount>
void ndBrainReplayBuffer<Action, statesCount>::SetCount(ndInt32 replayBufferSize, ndInt32)
{
	////ndAssert(GetCount() == 0);
	////ndAssert(m_learnBatchSize == 0);
	////ndAssert(replayBufferSize > replayBatchSize);
	
	m_replayBufferIndex = 0;
	////m_learnBatchSize = replayBatchSize;
	m_randomShaffle.Resize(replayBufferSize);
	ndArray<ndBrainReiforcementTransition<Action, statesCount>>::Resize(replayBufferSize);

	m_randomShaffle.SetCount(0);
	ndArray<ndBrainReiforcementTransition<Action, statesCount>>::SetCount(0);
	//for (ndInt32 i = 0; i < replayBufferSize; i++)
	//{
	//	ndBrainReiforcementTransition& transition = (*this)[i];
	//
	//	//	m_randomShaffle[i] = ndUnsigned32(i);
	//	transition.m_state = ndBrainVector();
	//	transition.m_nextState = ndBrainVector();
	//	transition.m_action = ndBrainVector();
	//	transition.m_reward = ndReal(1.0f);
	//	transition.m_terminalState = false;
	//
	//	transition.m_state.SetCount(stateSize);
	//	transition.m_nextState.SetCount(stateSize);
	//	transition.m_action.SetCount(actionSize);
	//}
	//
	////n_rewardBatch.SetCount(m_learnBatchSize);
	////n_terminalBatch.SetCount(m_learnBatchSize);
	////m_inputBatch.Init(m_learnBatchSize, stateSize);
	////m_outputBatch.Init(m_learnBatchSize, actionSize);
	////m_nextInputBatch.Init(m_learnBatchSize, stateSize);
	////m_groundTruthBatch.Init(m_learnBatchSize, actionSize);
}

template<class Action, ndInt32 statesCount>
void ndBrainReplayBuffer<Action, statesCount>::AddTransition(const ndBrainReiforcementTransition<Action, statesCount>& transition)
{
	ndInt32 count = ndArray<ndBrainReiforcementTransition<Action, statesCount>>::GetCount();
	if (count <= ndArray<ndBrainReiforcementTransition<Action, statesCount>>::GetCapacity())
	{
		ndAssert(count == m_replayBufferIndex);
		ndArray<ndBrainReiforcementTransition<Action, statesCount>>::PushBack(transition);
		m_replayBufferIndex = ndArray<ndBrainReiforcementTransition<Action, statesCount>>::GetCount();
	}
	else
	{
		ndAssert(0);
		//m_replayBufferIndex += (m_replayBufferIndex + 1) % GetCapacity();
		m_replayBufferIndex = (m_replayBufferIndex + 1) % ndArray<ndBrainReiforcementTransition<Action, statesCount>>::GetCapacity();
	}
}
#endif 

