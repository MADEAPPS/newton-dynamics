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

#include "ndDeepBrainStdafx.h"
#include "ndDeepBrainAgentReplayBuffer.h"

ndDeepBrainTransition::ndDeepBrainTransition()
	:m_state()
	,m_action()
	,m_nextState()
	,m_reward(0.0f)
	,m_terminalState(true)
{
}

void ndDeepBrainTransition::CopyFrom(const ndDeepBrainTransition& src)
{
	m_reward = src.m_reward;
	m_terminalState = src.m_terminalState;
	ndAssert(m_state.GetCount() == src.m_state.GetCount());
	ndAssert(m_action.GetCount() == src.m_action.GetCount());
	ndAssert(m_nextState.GetCount() == src.m_nextState.GetCount());
	
	memcpy(&m_state[0], &src.m_state[0], src.m_state.GetCount() * sizeof(ndReal));
	memcpy(&m_action[0], &src.m_action[0], src.m_action.GetCount() * sizeof(ndReal));
	memcpy(&m_nextState[0], &src.m_nextState[0], src.m_nextState.GetCount() * sizeof(ndReal));
}

ndDeepBrainReplayBuffer::ndDeepBrainReplayBuffer()
	:ndArray<ndDeepBrainTransition>()
	,m_randomShaffle()
	,m_inputBatch()
	,m_outputBatch()
	,m_nextInputBatch()
	,m_groundTruthBatch()
	,n_rewardBatch()
	,n_terminalBatch()
	,m_learnBashSize(0)
	,m_replayBufferIndex(0)
{
}

ndDeepBrainReplayBuffer::~ndDeepBrainReplayBuffer()
{
	for (ndInt32 i = 0; i < GetCount(); i++)
	{
		ndDeepBrainTransition& transition = (*this)[i];
		transition.m_state.~ndDeepBrainVector();
		transition.m_action.~ndDeepBrainVector();
		transition.m_nextState.~ndDeepBrainVector();
	}
}

void ndDeepBrainReplayBuffer::SetCount(ndInt32 count, ndInt32 stateSize, ndInt32 actionSize)
{
	ndAssert(count > 128);
	ndAssert(GetCount() == 0);
	ndAssert(m_learnBashSize == 0);

	m_learnBashSize = 128;
	m_replayBufferIndex = 0;

	m_randomShaffle.SetCount(count);
	ndArray<ndDeepBrainTransition>::SetCount(count);

	for (ndInt32 i = 0; i < count; i++)
	{
		ndDeepBrainTransition& transition = (*this)[i];

		m_randomShaffle[i] = i;
		transition.m_state = ndDeepBrainVector();
		transition.m_nextState = ndDeepBrainVector();
		transition.m_action = ndDeepBrainVector();
		transition.m_reward = 1.0f;
		transition.m_terminalState = false;

		transition.m_state.SetCount(stateSize);
		transition.m_nextState.SetCount(stateSize);
		transition.m_action.SetCount(actionSize);
	}

	n_rewardBatch.SetCount(m_learnBashSize);
	n_terminalBatch.SetCount(m_learnBashSize);
	m_inputBatch.Init(m_learnBashSize, stateSize);
	m_outputBatch.Init(m_learnBashSize, actionSize);
	m_nextInputBatch.Init(m_learnBashSize, stateSize);
	m_groundTruthBatch.Init(m_learnBashSize, actionSize);
}

ndDeepBrainTransition& ndDeepBrainReplayBuffer::GetTransitionEntry()
{
	ndInt32 replayIndex = m_replayBufferIndex % GetCount();
	ndDeepBrainTransition& transition = (*this)[replayIndex];
	m_replayBufferIndex++;
	return transition;
}

void ndDeepBrainReplayBuffer::MakeRandomBatch()
{
	ndInt32 count = ndMin(m_randomShaffle.GetCount(), m_replayBufferIndex);
	m_randomShaffle.RandomShuffle(count);
	ndAssert(m_learnBashSize == m_inputBatch.GetRows());
	for (ndInt32 i = 0; i < m_learnBashSize; ++i)
	{
		ndInt32 index = m_randomShaffle[i];
		const ndDeepBrainTransition& transition = (*this)[index];

		n_rewardBatch[i] = transition.m_reward;
		n_terminalBatch[i] = transition.m_terminalState ? 0.0f : 1.0f;
		for (ndInt32 j = 0; j < m_inputBatch.GetColumns(); ++j)
		{
			m_inputBatch[i][j] = transition.m_state[j];
			m_nextInputBatch[i][j] = transition.m_nextState[j];
		}

		for (ndInt32 j = 0; j < m_outputBatch.GetColumns(); ++j)
		{
			m_outputBatch[i][j] = transition.m_action[j];
		}
	}
}