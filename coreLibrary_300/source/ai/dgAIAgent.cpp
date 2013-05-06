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


dgAIAgentState::dgAIAgentState ()
	:m_userData (NULL)
	,m_agent(NULL)
	,m_enter(NULL)
	,m_exit(NULL)
	,m_updateFnt (NULL)
	,m_destroyFnt (NULL)
{
}

dgAIAgentState::~dgAIAgentState ()
{
}


void dgAIAgentState::SetUserData (void* const userData)
{
	m_userData = userData;
}

void* dgAIAgentState::GetUserData () const
{
	return m_userData;
}

void dgAIAgentState::SetDestroyCallback (OnAIAgentStateDestroy callback)
{
	m_destroyFnt = callback;
}

OnAIAgentStateDestroy dgAIAgentState::GetDestroyCallback () const
{
	return m_destroyFnt;
}


void dgAIAgentState::SetUpdateCallback (OnAIAgentStateUpdate callback)
{
	m_updateFnt = callback;
}

OnAIAgentStateUpdate dgAIAgentState::GetUpdateCallback () const
{
	return m_updateFnt;
}


void dgAIAgentState::SetEnterCallback  (OnAIAgentStateEnterExit callback)
{
	m_enter = callback;
}

OnAIAgentStateEnterExit dgAIAgentState::GetEnterCallback () const
{
	return m_enter;
}

void dgAIAgentState::SetExitCallback (OnAIAgentStateEnterExit callback)
{
	m_exit = callback;
}

OnAIAgentStateEnterExit dgAIAgentState::GetExitCallback () const
{
	return m_exit;
}


dgAIAgentStatesGraph::dgAIAgentStatesGraph (dgMemoryAllocator* const allocator)
	:dgGraph<dgAIAgentState, dgAIAgentTransition> (allocator)
{
}

dgAIAgentStatesGraph::~dgAIAgentStatesGraph()
{
	// clean  up all state while all state are still alive
	for (dgListNode* node = GetFirst(); node; node = node->GetNext()) {
		dgAIAgentState& agentProxy = node->GetInfo().m_nodeData;
		if (agentProxy.m_destroyFnt) {
			agentProxy.m_destroyFnt (node);
		}
	}
}

dgAIAgent::dgListNode* dgAIAgentStatesGraph::CreateState()
{
	dgAIAgent* const agent = (dgAIAgent*) this;
	dgListNode* const node = AddNode ();
	dgAIAgentState& state = node->GetInfo().m_nodeData;
	state.m_agent = agent;
		
	return node;
}

void dgAIAgentStatesGraph::DestroyState(dgListNode* const stateNode)
{
	dgAIAgent* const agent = (dgAIAgent*) this;
	if (agent->m_currentState == stateNode) {
		dgAssert (0);
		agent->m_currentState = NULL;
	}
	if (agent->m_startState == stateNode) {
		dgAssert (0);
		agent->m_startState = NULL;
	}
	DeleteNode (stateNode);
}

void dgAIAgentStatesGraph::SetStateUserData (dgListNode* const stateNode, void* const userData)
{
	dgAIAgentState& state = stateNode->GetInfo().m_nodeData;
	state.SetUserData(userData);
}

void* dgAIAgentStatesGraph::GetStateUserData (dgListNode* const stateNode) const
{
	dgAIAgentState& state = stateNode->GetInfo().m_nodeData;
	return state.GetUserData();
}

void dgAIAgentStatesGraph::SetStateDestroyCallback (dgListNode* const stateNode, OnAIAgentStateDestroy callback)
{
	dgAIAgentState& state = stateNode->GetInfo().m_nodeData;
	state.SetDestroyCallback(callback);
}

OnAIAgentStateDestroy dgAIAgentStatesGraph::GetStateDestroyCallback (dgListNode* const stateNode) const
{
	dgAIAgentState& state = stateNode->GetInfo().m_nodeData;
	return state.GetDestroyCallback();
}

void dgAIAgentStatesGraph::SetStateUpdateCallback (dgListNode* const stateNode, OnAIAgentStateUpdate callback)
{
	dgAIAgentState& state = stateNode->GetInfo().m_nodeData;
	state.SetUpdateCallback(callback);
}

OnAIAgentStateUpdate dgAIAgentStatesGraph::GetStateUpdateCallback (dgListNode* const stateNode) const
{
	dgAIAgentState& state = stateNode->GetInfo().m_nodeData;
	return state.GetUpdateCallback();
}

void dgAIAgentStatesGraph::SetStateEnterCallback (dgListNode* const stateNode, OnAIAgentStateEnterExit callback)
{
	dgAIAgentState& state = stateNode->GetInfo().m_nodeData;
	state.SetEnterCallback(callback);
}

OnAIAgentStateEnterExit dgAIAgentStatesGraph::GetStateEnterCallback (dgListNode* const stateNode) const
{
	dgAIAgentState& state = stateNode->GetInfo().m_nodeData;
	return state.GetEnterCallback();
}

void dgAIAgentStatesGraph::SetStateExitCallback (dgListNode* const stateNode, OnAIAgentStateEnterExit callback)
{
	dgAIAgentState& state = stateNode->GetInfo().m_nodeData;
	state.SetExitCallback(callback);
}

OnAIAgentStateEnterExit dgAIAgentStatesGraph::GetStateExitCallback (dgListNode* const stateNode) const
{
	dgAIAgentState& state = stateNode->GetInfo().m_nodeData;
	return state.GetExitCallback();
}


void dgAIAgentStatesGraph::Enter (dgListNode* const stateNode, dgInt32 threadID)
{
	dgAIAgentState& state = stateNode->GetInfo().m_nodeData;
	if (state.m_enter) {
		state.m_enter (stateNode, threadID);
	}
}

void dgAIAgentStatesGraph::Exit (dgListNode* const stateNode, dgInt32 threadID)
{
	dgAIAgentState& state = stateNode->GetInfo().m_nodeData;
	if (state.m_exit) {
		state.m_exit (stateNode, threadID);
	}
}

void dgAIAgentStatesGraph::Update (dgListNode* const stateNode, dgFloat32 timestep, dgInt32 threadID)
{
	dgAIAgentState& state = stateNode->GetInfo().m_nodeData;
	if (state.m_updateFnt) {
		state.m_updateFnt (stateNode, timestep, threadID);
	}

}

dgAIAgent::dgAIAgent(dgMemoryAllocator* const allocator)
	:dgAIAgentStatesGraph (allocator) 
	,m_userData(NULL)
	,m_world(NULL)
	,m_myWorldNode(NULL)
	,m_startState(NULL)
	,m_currentState(NULL)
	,m_updateFnt(NULL)
	,m_destroyFnt(NULL)
{

}

dgAIAgent::~dgAIAgent(void)
{

}

void dgAIAgent::SetUserData (void* const userData)
{
	m_userData = userData;
}

void* dgAIAgent::GetUserData() const
{
	return m_userData;
}

void dgAIAgent::SetUpdateFunction (OnAIAgentUpdate fnt)
{
	m_updateFnt = fnt;
}

void dgAIAgent::SetDestroyFunction (OnAIAgentDestroy fnt)
{
	m_destroyFnt = fnt;
}

OnAIAgentUpdate dgAIAgent::GetUpdateFunction () const
{
	return m_updateFnt;
}

OnAIAgentDestroy dgAIAgent::GetDestroyFunction () const 
{
	return m_destroyFnt;
}


dgAIWorld* dgAIAgent::GetWorld() const
{
	return m_world;
}

void* dgAIAgent::GetAIWorldNode() const
{
	return m_myWorldNode;
}


dgGraphNode<dgAIAgentState, dgAIAgentTransition>::dgListNode* dgAIAgent::LinkStates(dgListNode* const srcStateNode, dgListNode* const trgStateNode, dgInt32 transitionSymbol)
{
	dgGraphNode<dgAIAgentState, dgAIAgentTransition>::dgListNode* const transitionNode = srcStateNode->GetInfo().AddEdge(trgStateNode);
	dgAIAgentTransition& transition = transitionNode->GetInfo().m_edgeData;
	transition.m_symbol = transitionSymbol;
	return transitionNode;
}

void dgAIAgent::UnlinkStates(dgListNode* const srcStateNode, dgListNode* const trgStateNode)
{
	dgGraphNode<dgAIAgentState, dgAIAgentTransition>::dgListNode* next;
	for (dgGraphNode<dgAIAgentState, dgAIAgentTransition>::dgListNode* edgeNode = srcStateNode->GetInfo().GetFirst(); edgeNode; edgeNode = next) {
		next = edgeNode->GetNext();
		if (edgeNode->GetInfo().m_node == trgStateNode) {
			srcStateNode->GetInfo().DeleteHalfEdge (edgeNode);
		}
	}
}

void dgAIAgent::SetStartState (dgListNode* const startState)
{
	m_startState = startState;
}

dgAIAgentStatesGraph::dgListNode* dgAIAgent::GetStartState () const
{
	return m_startState;
}

dgAIAgentStatesGraph::dgListNode* dgAIAgent::GetCurrentState () const
{
	return m_currentState;
}

dgInt32 dgAIAgent::GetAdjacentStates (dgListNode* const srcStateNode, dgListNode** const adjacentStates, int maxCount)
{
	dgInt32 count = 0;
	for (dgGraphNode<dgAIAgentState, dgAIAgentTransition>::dgListNode* edgeNode = srcStateNode->GetInfo().GetFirst(); edgeNode && (count < maxCount); edgeNode = edgeNode->GetNext()) {
		adjacentStates[count] = edgeNode->GetInfo().m_node;
		count ++;
	}
	return count;
}

dgInt32 dgAIAgent::GetAdjacentTransitions (dgListNode* const srcStateNode, dgInt32* const adjacentTransitions, dgInt32 maxCount)
{
	dgInt32 count = 0;
	for (dgGraphNode<dgAIAgentState, dgAIAgentTransition>::dgListNode* edgeNode = srcStateNode->GetInfo().GetFirst(); edgeNode && (count < maxCount); edgeNode = edgeNode->GetNext()) {
		const dgAIAgentTransition& transition = edgeNode->GetInfo().m_edgeData;
		adjacentTransitions[count] = transition.m_symbol;
		count ++;
	}
	return count;
}


void dgAIAgent::ApplyTransition (dgInt32 transitionSymbol, dgInt32 threadID)
{
	if (m_currentState) {
		for (dgGraphNode<dgAIAgentState, dgAIAgentTransition>::dgListNode* edgeNode = m_currentState->GetInfo().GetFirst(); edgeNode; edgeNode = edgeNode->GetNext()) {
			const dgAIAgentTransition& transition = edgeNode->GetInfo().m_edgeData;
			if (transition.m_symbol == transitionSymbol) {
				GotoState (edgeNode->GetInfo().m_node, threadID);
				break;
			}
		}
	}
}

void dgAIAgent::GotoState (dgListNode* const targetStateNode, dgInt32 threadID)
{
	if (m_currentState) {
		Exit(m_currentState, threadID);
	}

	m_currentState = targetStateNode;
	
	if (m_currentState) {
		Enter(m_currentState, threadID);
	}
}

void dgAIAgent::Update (dgFloat32 timestep, dgInt32 threadID)
{
	if (m_updateFnt) {
		m_updateFnt (this, timestep, threadID);
	}

	if (m_currentState) {
		dgAIAgentStatesGraph::Update (m_currentState, timestep, threadID);
	}
}
