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
#include "NewtonAI.h"
#include "dgAIWorld.h"
#include "dgAIAgent.h"


int NewtonAIGetVersion ()
{
	TRACE_FUNCTION(__FUNCTION__);
	return NEWTON_AI_MAJOR_VERSION * 100 + NEWTON_AI_MINOR_VERSION;
}


NewtonAIWorld* NewtonAICreate ()
{
	TRACE_FUNCTION(__FUNCTION__);
	dgMemoryAllocator* const allocator = new dgMemoryAllocator();

	NewtonAIWorld* const ai = (NewtonAIWorld*) new (allocator) dgAIWorld (allocator);
	return ai;
}

void NewtonAIDestroy (const NewtonAIWorld* const newtonAIWorld)
{
	TRACE_FUNCTION(__FUNCTION__);

	dgAIWorld* const ai = (dgAIWorld *) newtonAIWorld;
	dgMemoryAllocator* const allocator = ai->GetAllocator();

	delete ai;
	delete allocator;
}


void NewtonAISetUserData (const NewtonAIWorld* const newtonAIWorld, void* const userData)
{
	TRACE_FUNCTION(__FUNCTION__);
	dgAIWorld* const ai = (dgAIWorld *) newtonAIWorld;
	ai->SetUserData (userData);
}


void* NewtonAIGetUserData (const NewtonAIWorld* const newtonAIWorld)
{
	TRACE_FUNCTION(__FUNCTION__);
	dgAIWorld* const ai = (dgAIWorld *) newtonAIWorld;
	return ai->GetUserData ();
}


void NewtonAIDestroyAllAgents (const NewtonAIWorld* const newtonAIWorld)
{
	dgAssert (0);
}

void NewtonAIUpdate (const NewtonAIWorld* const newtonAIWorld, dFloat timestep)
{
	TRACE_FUNCTION(__FUNCTION__);
	dgAIWorld* const ai = (dgAIWorld *) newtonAIWorld;

	ai->Update(timestep);
}

NewtonAIAgent* NewtonAIGetGameStateAgent (const NewtonAIWorld* const newtonAIWorld)
{
	TRACE_FUNCTION(__FUNCTION__);
	dgAIWorld* const ai = (dgAIWorld *) newtonAIWorld;
	return (NewtonAIAgent*) ai->GetGameStateAgent();	
}

NewtonAIAgent* NewtonAICreateAgent (const NewtonAIWorld* const newtonAIWorld)
{
	TRACE_FUNCTION(__FUNCTION__);
	dgAIWorld* const ai = (dgAIWorld *) newtonAIWorld;
	return (NewtonAIAgent*) ai->CreateAgent();	
}

void NewtonAIDestroyAgent (NewtonAIAgent* const agent)
{
	TRACE_FUNCTION(__FUNCTION__);
	dgAIAgent* const aiAgent = (dgAIAgent*) agent;
	dgAIWorld* const ai = (dgAIWorld *) aiAgent->GetWorld();
	ai->DestroyAgent(aiAgent);
}

NewtonAIWorld* NewtonAIAgentGetAIWorld (NewtonAIAgent* const agent)
{
	TRACE_FUNCTION(__FUNCTION__);
	dgAIAgent* const aiAgent = (dgAIAgent*) agent;
	return (NewtonAIWorld*) aiAgent->GetWorld();
}

NewtonAIAgent* NewtonAIGetFirstAgent (const NewtonAIWorld* const newtonAIWorld)
{
	TRACE_FUNCTION(__FUNCTION__);
	dgAIWorld* const ai = (dgAIWorld *) newtonAIWorld;
	if (ai->GetFirst()) {
		return (NewtonAIAgent*) ai->GetFirst()->GetInfo().m_nodeData.m_agent;
	}
	return NULL;
}

NewtonAIAgent* NewtonAIGetNextAgent (const NewtonAIWorld* const newtonAIWorld, NewtonAIAgent* const thisAgent)
{
	TRACE_FUNCTION(__FUNCTION__);
//	dgAIWorld* const ai = (dgAIWorld *) newtonAIWorld;
	dgAIAgent* const aiAgent = (dgAIAgent*) thisAgent;
	dgAIWorld::dgListNode* const node = (dgAIWorld::dgListNode*) aiAgent->GetAIWorldNode();
	if ( node->GetNext()) {
		return (NewtonAIAgent*) node->GetNext()->GetInfo().m_nodeData.m_agent;
	}
	return NULL;
}


NewtonAIAgentState* NewtonAIAgentCreateState (NewtonAIAgent* const agent)
{
	TRACE_FUNCTION(__FUNCTION__);
	dgAIAgent* const aiAgent = (dgAIAgent*) agent;
	return (NewtonAIAgentState*) aiAgent->CreateState();
}

void NewtonAIAgentDestroyState (NewtonAIAgent* agent, NewtonAIAgentState* const state)
{
	TRACE_FUNCTION(__FUNCTION__);
	dgAIAgent* const aiAgent = (dgAIAgent*) agent;
	aiAgent->DestroyState((dgAIAgent::dgListNode*) state);
}

void NewtonAIAgentSetStartState (NewtonAIAgent* const agent, NewtonAIAgentState* const startState)
{
	TRACE_FUNCTION(__FUNCTION__);
	dgAIAgent* const aiAgent = (dgAIAgent*) agent;
	aiAgent->SetStartState((dgAIAgent::dgListNode*)startState);
}

NewtonAIAgentState* NewtonAIAgentGetStartState (NewtonAIAgent* const agent)
{
	TRACE_FUNCTION(__FUNCTION__);
	dgAIAgent* const aiAgent = (dgAIAgent*) agent;
	return (NewtonAIAgentState*) aiAgent->GetStartState();
}

NewtonAIAgentState* NewtonAIAgentGetCurrentState (NewtonAIAgent* const agent)
{
	TRACE_FUNCTION(__FUNCTION__);
	dgAIAgent* const aiAgent = (dgAIAgent*) agent;
	return (NewtonAIAgentState*) aiAgent->GetCurrentState();
}

void NewtonAIAgentApplyTransition (NewtonAIAgent* const agent, int transitionSymbol, int threadID)
{
	TRACE_FUNCTION(__FUNCTION__);
	dgAIAgent* const aiAgent = (dgAIAgent*) agent;
	aiAgent->ApplyTransition(transitionSymbol, threadID);
}

void NewtonAIAgentGotoState (NewtonAIAgent* const agent, NewtonAIAgentState* const agentState, int threadID)
{
	TRACE_FUNCTION(__FUNCTION__);
	dgAIAgent* const aiAgent = (dgAIAgent*) agent;
	aiAgent->GotoState((dgAIAgent::dgListNode*) agentState, threadID);
}

void NewtonAIAgentSetUserData (NewtonAIAgent* const agent, void* const userData)
{
	TRACE_FUNCTION(__FUNCTION__);
	dgAIAgent* const aiAgent = (dgAIAgent*) agent;
	aiAgent->SetUserData(userData);
}

void* NewtonAIAgentGetUserData (NewtonAIAgent* const agent)
{
	TRACE_FUNCTION(__FUNCTION__);
	dgAIAgent* const aiAgent = (dgAIAgent*) agent;
	return aiAgent->GetUserData();
}

void NewtonAIAgentSetDestroyCallback (NewtonAIAgent* const agent, OnNewtonAIAgentDestroyCallback callback)
{
	TRACE_FUNCTION(__FUNCTION__);
	dgAIAgent* const aiAgent = (dgAIAgent*) agent;
	aiAgent->SetDestroyFunction(OnAIAgentDestroy (callback));
}

OnNewtonAIAgentDestroyCallback NewtonAIAgentGetDestroyCallback (NewtonAIAgent* const agent)
{
	TRACE_FUNCTION(__FUNCTION__);
	dgAIAgent* const aiAgent = (dgAIAgent*) agent;
	return (OnNewtonAIAgentDestroyCallback) aiAgent->GetDestroyFunction();
}


void NewtonAIAgentSetUpdateCallback (NewtonAIAgent* const agent, OnNewtonAIAgentUpdateCallback callback)
{
	TRACE_FUNCTION(__FUNCTION__);
	dgAIAgent* const aiAgent = (dgAIAgent*) agent;
	aiAgent->SetUpdateFunction (OnAIAgentUpdate (callback));	
}

OnNewtonAIAgentUpdateCallback NewtonAIAgentGetUpdateCallback (NewtonAIAgent* const agent)
{
	TRACE_FUNCTION(__FUNCTION__);
	dgAIAgent* const aiAgent = (dgAIAgent*) agent;
	return OnNewtonAIAgentUpdateCallback (aiAgent->GetUpdateFunction ());	
}




NewtonAIAgent* NewtonAIAgentStateGetAgent (NewtonAIAgentState* const agentState)
{
	dgGraph<dgAIAgentState, dgAIAgentTransition>::dgListNode* const stateNode = (dgGraph<dgAIAgentState, dgAIAgentTransition>::dgListNode*) agentState;
	dgAIAgentState& state = stateNode->GetInfo().m_nodeData;	
	return (NewtonAIAgent*) state.m_agent;
}

void NewtonAIAgentStateSetUserData (NewtonAIAgentState* const agentState, void* const userData)
{
	dgGraph<dgAIAgentState, dgAIAgentTransition>::dgListNode* const stateNode = (dgGraph<dgAIAgentState, dgAIAgentTransition>::dgListNode*) agentState;
	dgAIAgentState& state = stateNode->GetInfo().m_nodeData;	
	state.m_agent->SetStateUserData (stateNode, userData);
}

void* NewtonAIAgentStateGetUserData (NewtonAIAgentState* const agentState)
{
	if (agentState) {
		dgGraph<dgAIAgentState, dgAIAgentTransition>::dgListNode* const stateNode = (dgGraph<dgAIAgentState, dgAIAgentTransition>::dgListNode*) agentState;
		dgAIAgentState& state = stateNode->GetInfo().m_nodeData;	
		return state.m_agent->GetStateUserData (stateNode);
	} 
	return NULL;
}

void NewtonAIAgentStateSetDestroyCallback (NewtonAIAgentState* const agentState, OnNewtonAIAgentStateDestroyCallback callback)
{
	dgGraph<dgAIAgentState, dgAIAgentTransition>::dgListNode* const stateNode = (dgGraph<dgAIAgentState, dgAIAgentTransition>::dgListNode*) agentState;
	dgAIAgentState& state = stateNode->GetInfo().m_nodeData;	
	state.m_agent->SetStateDestroyCallback (stateNode, OnAIAgentStateDestroy (callback));	
}

OnNewtonAIAgentStateDestroyCallback NewtonAIAgentStateGetDestroyCallback (NewtonAIAgentState* const agentState)
{
	dgGraph<dgAIAgentState, dgAIAgentTransition>::dgListNode* const stateNode = (dgGraph<dgAIAgentState, dgAIAgentTransition>::dgListNode*) agentState;
	dgAIAgentState& state = stateNode->GetInfo().m_nodeData;	
	return OnNewtonAIAgentStateDestroyCallback (state.m_agent->GetStateDestroyCallback (stateNode));	
}


void NewtonAIAgentStateSetUpdateCallback (NewtonAIAgentState* const agentState, OnNewtonAIAgentStateUpdateCallback callback)
{
	dgGraph<dgAIAgentState, dgAIAgentTransition>::dgListNode* const stateNode = (dgGraph<dgAIAgentState, dgAIAgentTransition>::dgListNode*) agentState;
	dgAIAgentState& state = stateNode->GetInfo().m_nodeData;	
	state.m_agent->SetStateUpdateCallback (stateNode, OnAIAgentStateUpdate (callback));	
}

OnNewtonAIAgentStateUpdateCallback NewtonAIAgentStateGetUpdateCallback (NewtonAIAgentState* const agentState)
{
	dgGraph<dgAIAgentState, dgAIAgentTransition>::dgListNode* const stateNode = (dgGraph<dgAIAgentState, dgAIAgentTransition>::dgListNode*) agentState;
	dgAIAgentState& state = stateNode->GetInfo().m_nodeData;	
	return OnNewtonAIAgentStateUpdateCallback (state.m_agent->GetStateUpdateCallback (stateNode));	
}


void NewtonAIAgentStateSetEnterCallback (NewtonAIAgentState* const agentState, OnNewtonAIAgentStateEnterExitCallback callback)
{
	dgGraph<dgAIAgentState, dgAIAgentTransition>::dgListNode* const stateNode = (dgGraph<dgAIAgentState, dgAIAgentTransition>::dgListNode*) agentState;
	dgAIAgentState& state = stateNode->GetInfo().m_nodeData;	
	state.m_agent->SetStateEnterCallback(stateNode, OnAIAgentStateEnterExit(callback));
}

OnNewtonAIAgentStateUpdateCallback NewtonAIAgentStateGetEnterCallback (NewtonAIAgentState* const agentState)
{
	dgGraph<dgAIAgentState, dgAIAgentTransition>::dgListNode* const stateNode = (dgGraph<dgAIAgentState, dgAIAgentTransition>::dgListNode*) agentState;
	dgAIAgentState& state = stateNode->GetInfo().m_nodeData;	
	return OnNewtonAIAgentStateUpdateCallback (state.m_agent->GetStateEnterCallback(stateNode));
}

void NewtonAIAgentStateSetExitCallback (NewtonAIAgentState* const agentState, OnNewtonAIAgentStateEnterExitCallback callback)
{
	dgGraph<dgAIAgentState, dgAIAgentTransition>::dgListNode* const stateNode = (dgGraph<dgAIAgentState, dgAIAgentTransition>::dgListNode*) agentState;
	dgAIAgentState& state = stateNode->GetInfo().m_nodeData;	
	state.m_agent->SetStateExitCallback(stateNode, OnAIAgentStateEnterExit(callback));
}

OnNewtonAIAgentStateUpdateCallback NewtonAIAgentStateGetExitCallback (NewtonAIAgentState* const agentState)
{
	dgGraph<dgAIAgentState, dgAIAgentTransition>::dgListNode* const stateNode = (dgGraph<dgAIAgentState, dgAIAgentTransition>::dgListNode*) agentState;
	dgAIAgentState& state = stateNode->GetInfo().m_nodeData;	
	return OnNewtonAIAgentStateUpdateCallback (state.m_agent->GetStateExitCallback(stateNode));
}

int NewtonAgentStateGetAdjacentStates (NewtonAIAgentState* const srcState, NewtonAIAgentState** const statesBuffer, int maxCount)
{
	dgGraph<dgAIAgentState, dgAIAgentTransition>::dgListNode* const stateNode = (dgGraph<dgAIAgentState, dgAIAgentTransition>::dgListNode*) srcState;
	dgAIAgentState& state = stateNode->GetInfo().m_nodeData;	
	return state.m_agent->GetAdjacentStates(stateNode, (dgGraph<dgAIAgentState, dgAIAgentTransition>::dgListNode**) statesBuffer, maxCount);
}


int NewtonAgentStateGetAdjacentTransitions (NewtonAIAgentState* const srcState, int* transitionSymbolsBuffer, int maxCount)
{
	dgGraph<dgAIAgentState, dgAIAgentTransition>::dgListNode* const stateNode = (dgGraph<dgAIAgentState, dgAIAgentTransition>::dgListNode*) srcState;
	dgAIAgentState& state = stateNode->GetInfo().m_nodeData;	
	return state.m_agent->GetAdjacentTransitions (stateNode, transitionSymbolsBuffer, maxCount);
}

NewtonAIAgentTransition* NewtonAIAgentConnectStates (NewtonAIAgentState* const srcState, NewtonAIAgentState* const trgState, int transitionsymbol)
{
	TRACE_FUNCTION(__FUNCTION__);
	dgGraph<dgAIAgentState, dgAIAgentTransition>::dgListNode* const stcNode = (dgGraph<dgAIAgentState, dgAIAgentTransition>::dgListNode*) srcState;
	dgGraph<dgAIAgentState, dgAIAgentTransition>::dgListNode* const trgNode = (dgGraph<dgAIAgentState, dgAIAgentTransition>::dgListNode*) trgState;
	dgAIAgentState& state = stcNode->GetInfo().m_nodeData;	
	if (state.m_agent == trgNode->GetInfo().m_nodeData.m_agent) {
		return (NewtonAIAgentTransition*) state.m_agent->LinkStates(stcNode, trgNode, transitionsymbol);
	}
	return NULL;
}

void NewtonAIAgentDisconnectStates (NewtonAIAgentState* const srcState, NewtonAIAgentState* const trgState)
{
	TRACE_FUNCTION(__FUNCTION__);
	dgGraph<dgAIAgentState, dgAIAgentTransition>::dgListNode* const stcNode = (dgGraph<dgAIAgentState, dgAIAgentTransition>::dgListNode*) srcState;
	dgGraph<dgAIAgentState, dgAIAgentTransition>::dgListNode* const trgNode = (dgGraph<dgAIAgentState, dgAIAgentTransition>::dgListNode*) trgState;
	dgAIAgentState& state = stcNode->GetInfo().m_nodeData;	
	if (state.m_agent == trgNode->GetInfo().m_nodeData.m_agent) {
		state.m_agent->UnlinkStates(stcNode, trgNode);
	}
}
