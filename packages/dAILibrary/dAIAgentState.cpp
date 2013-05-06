/* Copyright (c) <2009> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/


#include "dAI.h"
#include "dAIAgent.h"
#include "dAIAgentState.h"


dAIAgentState::dAIAgentState(dAIAgent* const agent)
	:m_state (NewtonAIAgentCreateState (agent->GetNewtonAIAgent()))
	,m_autoDestroy(false)
{
	NewtonAIAgentStateSetUserData (m_state, this);
	NewtonAIAgentStateSetDestroyCallback (m_state, Destroy);
	NewtonAIAgentStateSetUpdateCallback (m_state, Update);
	NewtonAIAgentStateSetEnterCallback (m_state, Enter);
	NewtonAIAgentStateSetExitCallback (m_state, Exit);
}

dAIAgentState::~dAIAgentState()
{
	if (!m_autoDestroy) {
		NewtonAIAgentStateSetUserData (m_state, NULL);  
		NewtonAIAgentStateSetDestroyCallback (m_state, NULL);  
		NewtonAIAgentDestroyState(NewtonAIAgentStateGetAgent(m_state), m_state);
	}
}

// call when the  AI agent is destroyed
void dAIAgentState::Destroy (NewtonAIAgentState* const agentState)
{
	dAIAgentState* const me = (dAIAgentState*) NewtonAIAgentStateGetUserData (agentState);
	me->m_autoDestroy = true;
	delete me;
}


NewtonAIAgent* dAIAgentState::GetNewtonAIAgent() const
{
	return NewtonAIAgentStateGetAgent(m_state);
}


dAIAgent* dAIAgentState::GetAgent() const
{
	NewtonAIAgent* const agent = GetNewtonAIAgent();
	dAssert (agent);
	return (dAIAgent*) NewtonAIAgentGetUserData(agent);
}


void dAIAgentState::Enter (NewtonAIAgentState* const agentState, int threadID)
{
	// go over every AI agent in the Game and send the to the start state
	dAIAgentState* const me = (dAIAgentState*) NewtonAIAgentStateGetUserData (agentState);
	me->Enter(threadID);
}

void dAIAgentState::Exit (NewtonAIAgentState* const agentState, int threadID)
{
	dAIAgentState* const me = (dAIAgentState*) NewtonAIAgentStateGetUserData (agentState);
	me->Exit(threadID);
}

void dAIAgentState::Update (NewtonAIAgentState* const agentState, dFloat timestep, int threadID)
{
	dAIAgentState* const me = (dAIAgentState*) NewtonAIAgentStateGetUserData (agentState);
	me->Update(timestep, threadID);
}


int dAIAgentState::GetAdjacentTranstions (int* const signalsBuffer, int maxCount) const
{
	return NewtonAgentStateGetAdjacentTransitions (m_state, signalsBuffer, maxCount);
}

int dAIAgentState::GetAdjacentStates (dAIAgentState** const agentsBuffer, int maxCount) const
{
	int count = NewtonAgentStateGetAdjacentStates(m_state, (NewtonAIAgentState**) agentsBuffer, maxCount);
	for (int i = 0; i < count; i ++) {
		agentsBuffer[i] = (dAIAgentState*) NewtonAIAgentStateGetUserData((NewtonAIAgentState*)agentsBuffer[i]);
	}
	return count;
}

