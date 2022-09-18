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


void ndDeepBrainTransition::CopyFrom(const ndDeepBrainTransition& src)
{
	m_reward = src.m_reward;
	m_state.SetCount(src.m_state.GetCount());
	m_action.SetCount(src.m_action.GetCount());
	m_nextState.SetCount(src.m_nextState.GetCount());
	
	memcpy(&m_state[0], &src.m_state[0], src.m_state.GetCount() * sizeof(ndReal));
	memcpy(&m_action[0], &src.m_action[0], src.m_action.GetCount() * sizeof(ndReal));
	memcpy(&m_nextState[0], &src.m_nextState[0], src.m_nextState.GetCount() * sizeof(ndReal));
}

ndDeepBrainReplayBuffer::ndDeepBrainReplayBuffer()
	:ndArray<ndDeepBrainTransition>()
{
}

ndDeepBrainReplayBuffer::~ndDeepBrainReplayBuffer()
{
	for (ndInt32 i = 0; i < GetCount(); i++)
	{
		ndDeepBrainTransition& transition = (*this)[i];
		transition.m_state.~ndArray<ndReal>();
		transition.m_action.~ndArray<ndReal>();
		transition.m_nextState.~ndArray<ndReal>();
	}
}

void ndDeepBrainReplayBuffer::SetCount(ndInt32 count)
{
	ndAssert(GetCount() == 0);
	ndArray<ndDeepBrainTransition>::SetCount(count);
	for (ndInt32 i = 0; i < count; i++)
	{
		ndDeepBrainTransition& transition = (*this)[i];
		transition.m_state = ndArray<ndReal>();
		transition.m_nextState = ndArray<ndReal>();
		transition.m_action = ndArray<ndReal>();
		transition.m_reward = 1.0f;
		transition.m_terminalState = false;
	}
}
