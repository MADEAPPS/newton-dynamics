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



#include "dStdAfxAI.h"
#include "dAI.h"
#include "dAIAgent.h"
#include "dAIAgentState.h"

dAI::dAI ()
	:m_ai (NewtonAICreate ())
{
	NewtonAISetUserData (m_ai, this);
}

dAI::~dAI ()
{
	NewtonAIDestroy (m_ai);
}

NewtonAIAgent* dAI::GetGameLogicNewtonAgent() const
{
	return NewtonAIGetGameStateAgent (m_ai);
}

dAIAgent* dAI::GetFirstAgent() const
{
	NewtonAIAgent* const agent = NewtonAIGetFirstAgent (m_ai);
	if (agent) {
		return (dAIAgent*) NewtonAIAgentGetUserData (agent);
	}
	return NULL;
}

dAIAgent* dAI::GetNextAgent(dAIAgent* const agent) const
{
	NewtonAIAgent* const nextAgent = NewtonAIGetNextAgent (m_ai, agent->m_agent);
	if (nextAgent) {
		return (dAIAgent*) NewtonAIAgentGetUserData (nextAgent);
	}
	return NULL;
	
}


dAIAgent* dAI::GetGameLogicAgent() const
{
	return (dAIAgent*) NewtonAIAgentGetUserData(GetGameLogicNewtonAgent());
}

void dAI::Update (dFloat timestep)
{
	NewtonAIUpdate (m_ai, timestep);
}
