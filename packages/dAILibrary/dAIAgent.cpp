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

dAIAgent::dAIAgent (dAI* const manager)
	:m_agent (NewtonAICreateAgent (manager->GetAI ()))
	,m_autoDestroy(false)
{
	NewtonAIAgentSetUserData(m_agent, this);
	NewtonAIAgentSetUpdateCallback (m_agent, Update);
	NewtonAIAgentSetDestroyCallback (m_agent, Destroy);
}

dAIAgent::dAIAgent (NewtonAIAgent* const agent)
	:m_agent (agent)
	,m_autoDestroy(false)
{
	NewtonAIAgentSetUserData(m_agent, this);
	NewtonAIAgentSetUpdateCallback (m_agent, Update);
	NewtonAIAgentSetDestroyCallback (m_agent, Destroy);
}

dAIAgent::~dAIAgent ()
{
	if (!m_autoDestroy) {
		NewtonAIAgentSetUserData (m_agent, NULL);  
		NewtonAIAgentSetDestroyCallback (m_agent, NULL);  
		NewtonAIDestroyAgent(m_agent);
	}
}


void dAIAgent::Destroy (NewtonAIAgent* const agent)
{
	dAIAgent* const me = (dAIAgent*) NewtonAIAgentGetUserData (agent);
	me->m_autoDestroy = true;
	delete me;
}

dAI* dAIAgent::GetWorld() const
{
	return (dAI*) NewtonAIGetUserData (NewtonAIAgentGetAIWorld (m_agent));
}

dAIAgentState* dAIAgent::GetCurrentState () const
{
	NewtonAIAgentState* const state = NewtonAIAgentGetCurrentState(m_agent);
	if (state) {
		return (dAIAgentState*) NewtonAIAgentStateGetUserData(state);
	} 
	return NULL;
}


void dAIAgent::Update (NewtonAIAgent* const agent, dFloat timestep, int threadID)
{
	dAIAgent* const me = (dAIAgent*) NewtonAIAgentGetUserData (agent);
	me->Update(timestep, threadID);
}

void dAIAgent::GoToState (dAIAgentState* const state, int threadIndex)
{
	if (state) {
		NewtonAIAgentGotoState (m_agent, state->m_state, threadIndex);
	}
}

void dAIAgent::ApplyTransition (int transitionSymbol, int threadID)
{
	NewtonAIAgentApplyTransition (m_agent, transitionSymbol, threadID);
}


void dAIAgent::SetStartState (dAIAgentState* const state)
{
	NewtonAIAgentSetStartState (m_agent, state->m_state);
}

dAIAgentState* dAIAgent::GetStartState () const
{
//	NewtonAIAgent* const agent = NewtonAIGetFirstAgent (m_ai);
//	if (agent) {
//		return (dAIAgent*) NewtonAIAgentGetUserData (agent);
//	}
	return (dAIAgentState*) NewtonAIAgentStateGetUserData(NewtonAIAgentGetStartState (m_agent));
}

void* dAIAgent::LinkStates (dAIAgentState* const sourceState, dAIAgentState* const targetState, int transitionSignal)
{
	void* const transition = NewtonAIAgentConnectStates (sourceState->m_state, targetState->m_state, transitionSignal);
	return transition;
}

void dAIAgent::UnlinkStates (dAIAgentState* const sourceState, dAIAgentState* const targetState)
{
	NewtonAIAgentDisconnectStates (sourceState->m_state, targetState->m_state);
}