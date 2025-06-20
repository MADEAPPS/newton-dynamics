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

#if 0
template<ndInt32 statesDim, ndInt32 actionDim>
class ndBrainReplayTransitionMemory
{
	public:
	ndBrainReplayTransitionMemory();
	ndBrainReplayTransitionMemory(const ndBrainReplayTransitionMemory& src);

	void Clear();
	ndBrainReplayTransitionMemory& operator=(const ndBrainReplayTransitionMemory& src);

	ndBrainFixSizeVector<actionDim> m_action;
	ndBrainFixSizeVector<statesDim> m_observation;
	ndBrainFixSizeVector<statesDim> m_nextObservation;
	ndBrainFloat m_reward;
	bool m_terminalState;
};

template<ndInt32 statesDim, ndInt32 actionDim>
class ndBrainReplayBuffer : public ndArray<ndBrainReplayTransitionMemory<statesDim, actionDim>>
{
	public:
	ndBrainReplayBuffer();
	~ndBrainReplayBuffer();

	void Clear();
	void SetSize(ndInt32 size);
	void AddTransition(const ndBrainReplayTransitionMemory<statesDim, actionDim>& transition);

	ndInt32 m_replayBufferIndex;
};

template<ndInt32 statesDim, ndInt32 actionDim>
ndBrainReplayTransitionMemory<statesDim, actionDim>::ndBrainReplayTransitionMemory()
	:m_action()
	,m_observation()
	,m_nextObservation()
	,m_reward(ndBrainFloat (0.0f))
	,m_terminalState(false)
{
	Clear();
}

template<ndInt32 statesDim, ndInt32 actionDim>
ndBrainReplayTransitionMemory<statesDim, actionDim>::ndBrainReplayTransitionMemory(const ndBrainReplayTransitionMemory& src)
	:m_action(src.m_action)
	,m_observation(src.m_observation)
	,m_nextObservation(src.m_nextObservation)
	,m_reward(src.m_reward)
	,m_terminalState(src.m_terminalState)
{
}

template<ndInt32 statesDim, ndInt32 actionDim>
ndBrainReplayTransitionMemory<statesDim, actionDim>& ndBrainReplayTransitionMemory<statesDim, actionDim>::operator=(const ndBrainReplayTransitionMemory& src)
{
	new (this) ndBrainReplayTransitionMemory(src);
	return* this;
}

template<ndInt32 statesDim, ndInt32 actionDim>
void ndBrainReplayTransitionMemory<statesDim, actionDim>::Clear()
{
	m_action.SetCount(0);
	m_observation.SetCount(0);
	m_nextObservation.SetCount(0);
	for (ndInt32 i = 0; i < statesDim; ++i)
	{
		m_observation.PushBack(ndBrainFloat(0.0f));
		m_nextObservation.PushBack(ndBrainFloat(0.0f));
	}
	
	for (ndInt32 i = 0; i < actionDim; ++i)
	{
		m_action.PushBack(ndBrainFloat(0.0f));
	}
	
	m_reward = ndBrainFloat(0.0f);
	m_terminalState = false;
}

template<ndInt32 statesDim, ndInt32 actionDim>
ndBrainReplayBuffer<statesDim, actionDim>::ndBrainReplayBuffer()
	:ndArray<ndBrainReplayTransitionMemory<statesDim, actionDim>>()
	,m_replayBufferIndex(0)
{
}

template<ndInt32 statesDim, ndInt32 actionDim>
ndBrainReplayBuffer<statesDim, actionDim>::~ndBrainReplayBuffer()
{
}

template<ndInt32 statesDim, ndInt32 actionDim>
void ndBrainReplayBuffer<statesDim, actionDim>::Clear()
{
	m_replayBufferIndex = 0;
	ndArray<ndBrainReplayTransitionMemory<statesDim, actionDim>>::SetCount(0);
}

template<ndInt32 statesDim, ndInt32 actionDim>
void ndBrainReplayBuffer<statesDim, actionDim>::SetSize(ndInt32 size)
{
	ndArray<ndBrainReplayTransitionMemory<statesDim, actionDim>>::SetCount(size);
	Clear();
}

template<ndInt32 statesDim, ndInt32 actionDim>
void ndBrainReplayBuffer<statesDim, actionDim>::AddTransition(const ndBrainReplayTransitionMemory<statesDim, actionDim>& transition)
{
	ndInt32 count = ndArray<ndBrainReplayTransitionMemory<statesDim, actionDim>>::GetCount();
	if (count < ndArray<ndBrainReplayTransitionMemory<statesDim, actionDim>>::GetCapacity())
	{
		ndArray<ndBrainReplayTransitionMemory<statesDim, actionDim>>::PushBack(transition);
	}
	else
	{
		ndBrainReplayBuffer<statesDim, actionDim>& me = *this;
		me[m_replayBufferIndex] = transition;
		m_replayBufferIndex = (m_replayBufferIndex + 1) % ndArray<ndBrainReplayTransitionMemory<statesDim, actionDim>>::GetCapacity();
	}
}
#endif 
#endif 

