/* Copyright (c) <2003-2011> <Julio Jerez, Newton Game Dynamics>
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

#include "dgAIStdafx.h"
#include "dgAIWorld.h"
#include "dgAIAgent.h"


dgAIAgentGraph::dgAIAgentGraph (dgMemoryAllocator* const allocator)
	:dgGraph<dgAIAgentAgentState, dgAIAgentAgentTransition>(allocator)
{
}

dgAIAgentGraph::~dgAIAgentGraph()
{
	// call on destroy on all agent before they go away
	for (dgListNode* node = GetFirst(); node; node = node->GetNext()) {
		dgAIAgentAgentState& agentProxy = node->GetInfo().m_nodeData;
		if (agentProxy.m_agent->GetDestroyFunction()) {
			agentProxy.m_agent->GetDestroyFunction()(agentProxy.m_agent);
		}
	}
}

dgAIAgent* dgAIAgentGraph::CreateAgent ()
{
	dgAIWorld* const world = (dgAIWorld*) this;
	dgAIAgent* const agent = new (GetAllocator()) dgAIAgent(GetAllocator());
	agent->m_world = world;

	dgListNode* const node = AddNode();
	dgAIAgentAgentState& aiAgentAgentState = node->GetInfo().m_nodeData;
	aiAgentAgentState.m_agent = agent;
	agent->m_myWorldNode = node;

	return agent;
}


void dgAIAgentGraph::DestroyAgent (dgAIAgent* const agent)
{
	dgListNode* const node = (dgListNode*) agent->m_myWorldNode;

	if (agent->GetDestroyFunction()) {
		agent->GetDestroyFunction()(agent);
	}
	DeleteNode (node);
}


void dgAIAgentGraph::Update (dgFloat32 timestep)
{
	// no rethreading updates for now, but very soon
	for (dgListNode* node = GetFirst(); node; node = node->GetNext()) {
		dgAIAgentAgentState& agentProxy = node->GetInfo().m_nodeData;
		agentProxy.m_agent->Update(timestep, 0);
	}
}


dgAIWorld::dgAIWorld(dgMemoryAllocator* const allocator)
	:dgAIAgentGraph (allocator)
	,m_gameState (allocator)
	,m_userData()
	,m_allocator(allocator)
{
	m_gameState.m_world = this;
}

dgAIWorld::~dgAIWorld(void)
{
	if (m_gameState.GetDestroyFunction()) {
		m_gameState.GetDestroyFunction()(&m_gameState);
	}
}


dgMemoryAllocator* dgAIWorld::GetAllocator() const
{
	return m_allocator;
}


void dgAIWorld::SetUserData (void* const userData)
{
	m_userData = userData;
}

void* dgAIWorld::GetUserData() const
{
	return m_userData;
}

dgAIAgent* dgAIWorld::GetGameStateAgent()
{
	return &m_gameState;
}


void dgAIWorld::Update (dgFloat32 timestep)
{
	m_gameState.Update(timestep, 0);
	dgAIAgentGraph::Update(timestep);
}