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

template<class actionType, ndInt32 statesDim, ndInt32 actionDim = 1>
class ndBrainReplayTransitionMemory
{
	public:
	ndBrainReplayTransitionMemory();

	void Clear();
	ndFixSizeArray<ndReal, statesDim> m_state;
	ndFixSizeArray<ndReal, statesDim> m_nextState;
	ndFixSizeArray<actionType, actionDim> m_action;
	ndReal m_reward;
	bool m_terminalState;
};

template<class actionType, ndInt32 statesDim, ndInt32 actionDim = 1>
class ndBrainReplayBuffer : public ndArray<ndBrainReplayTransitionMemory<actionType, statesDim, actionDim>>
{
	public:
	ndBrainReplayBuffer(ndInt32 size);
	~ndBrainReplayBuffer();

	void Clear();
	void AddTransition(const ndBrainReplayTransitionMemory<actionType, statesDim, actionDim>& transition);
};

template<class actionType, ndInt32 statesDim, ndInt32 actionDim>
ndBrainReplayTransitionMemory<actionType, statesDim, actionDim>::ndBrainReplayTransitionMemory()
	:m_state()
	,m_nextState()
	,m_action()
	,m_reward(0.0f)
	,m_terminalState(false)
{
	Clear();
}

template<class actionType, ndInt32 statesDim, ndInt32 actionDim>
void ndBrainReplayTransitionMemory<actionType, statesDim, actionDim>::Clear()
{
	m_state.SetCount(0);
	m_action.SetCount(0);
	m_nextState.SetCount(0);
	for (ndInt32 i = 0; i < statesDim; ++i)
	{
		m_state.PushBack(ndReal(0.0f));
		m_nextState.PushBack(ndReal(0.0f));
	}

	for (ndInt32 i = 0; i < actionDim; ++i)
	{
		m_action.PushBack(actionType(0));
	}

	m_reward = ndReal(0.0f);
	m_terminalState = false;
}

template<class actionType, ndInt32 statesDim, ndInt32 actionDim>
ndBrainReplayBuffer<actionType, statesDim, actionDim>::ndBrainReplayBuffer(ndInt32 size)
	:ndArray<ndBrainReplayTransitionMemory<actionType, statesDim, actionDim>>()
{
	ndArray<ndBrainReplayTransitionMemory<actionType, statesDim, actionDim>>::SetCount(size);
	Clear();
}

template<class actionType, ndInt32 statesDim, ndInt32 actionDim>
ndBrainReplayBuffer<actionType, statesDim, actionDim>::~ndBrainReplayBuffer()
{
}

template<class actionType, ndInt32 statesDim, ndInt32 actionDim>
void ndBrainReplayBuffer<actionType, statesDim, actionDim>::Clear()
{
	ndArray<ndBrainReplayTransitionMemory<actionType, statesDim>>::SetCount(0);
}

template<class actionType, ndInt32 statesDim, ndInt32 actionDim>
void ndBrainReplayBuffer<actionType, statesDim, actionDim>::AddTransition(const ndBrainReplayTransitionMemory<actionType, statesDim, actionDim>& transition)
{
	ndInt32 count = ndArray<ndBrainReplayTransitionMemory<actionType, statesDim, actionDim>>::GetCount();
	if (count <= ndArray<ndBrainReplayTransitionMemory<actionType, statesDim, actionDim>>::GetCapacity())
	{
		ndArray<ndBrainReplayTransitionMemory<actionType, statesDim, actionDim>>::PushBack(transition);
	}
	else
	{
		ndAssert(0);
		//m_replayBufferIndex += (m_replayBufferIndex + 1) % GetCapacity();
		//m_replayBufferIndex = (m_replayBufferIndex + 1) % ndArray<ndBrainReiforcementTransition<Action, statesCount>>::GetCapacity();
	}
}
#endif 

