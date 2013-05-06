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

#ifndef __NEWTON_AI_H__
#define __NEWTON_AI_H__


#define NEWTON_AI_MAJOR_VERSION 1
#define NEWTON_AI_MINOR_VERSION 00


#ifndef dFloat
	#ifdef __USE_DOUBLE_PRECISION__
		#define dFloat double
	#else
		#define dFloat float
	#endif
#endif

#ifndef dFloat64
	#define dFloat64 double
#endif



#ifdef __cplusplus 
extern "C" {
#endif


	typedef struct NewtonAIWorld{} NewtonAIWorld;
	typedef struct NewtonAIAgent{} NewtonAIAgent;
	typedef struct NewtonAIAgentState{} NewtonAIAgentState;
	typedef struct NewtonAIAgentTransition{} NewtonAIAgentTransition;

	typedef void (*OnNewtonAIAgentDestroyCallback) (NewtonAIAgent* const agent);
	typedef void (*OnNewtonAIAgentUpdateCallback) (NewtonAIAgent* const agent, dFloat timestep, int threadID);

	typedef void (*OnNewtonAIAgentStateDestroyCallback) (NewtonAIAgentState* const agentState);
	typedef void (*OnNewtonAIAgentStateEnterExitCallback) (NewtonAIAgentState* const agentState, int threadID);
	typedef void (*OnNewtonAIAgentStateUpdateCallback) (NewtonAIAgentState* const agentState, dFloat timestep, int threadID);


	// **********************************************************************************************
	//
	// ai world control functions
	//
	// **********************************************************************************************
	int NewtonAIGetVersion ();

	NewtonAIWorld* NewtonAICreate ();
	void NewtonAIDestroy (const NewtonAIWorld* const newtonAIWorld);
	void NewtonAIDestroyAllAgents (const NewtonAIWorld* const newtonAIWorld);

	void NewtonAISetUserData (const NewtonAIWorld* const newtonAIWorld, void* const userData);
	void* NewtonAIGetUserData (const NewtonAIWorld* const newtonAIWorld);

	void NewtonAIUpdate (const NewtonAIWorld* const newtonAIWorld, dFloat timestep);

	NewtonAIAgent* NewtonAIGetGameStateAgent (const NewtonAIWorld* const newtonAIWorld);

	NewtonAIAgent* NewtonAIGetFirstAgent (const NewtonAIWorld* const newtonAIWorld);
	NewtonAIAgent* NewtonAIGetNextAgent (const NewtonAIWorld* const newtonAIWorld, NewtonAIAgent* const thisAgent);

	// **********************************************************************************************
	//
	// AI Agents interface
	//
	// **********************************************************************************************
	NewtonAIAgent* NewtonAICreateAgent (const NewtonAIWorld* const newtonAIWorld);
	void NewtonAIDestroyAgent (NewtonAIAgent* const agent);

	void NewtonAIAgentSetStartState (NewtonAIAgent* const agent, NewtonAIAgentState* const startState);
	NewtonAIAgentState* NewtonAIAgentGetStartState (NewtonAIAgent* const agent);
	NewtonAIAgentState* NewtonAIAgentGetCurrentState (NewtonAIAgent* const agent);

	void NewtonAIAgentApplyTransition (NewtonAIAgent* const agent, int transitionSymbol, int threadID);
	void NewtonAIAgentGotoState (NewtonAIAgent* const agent, NewtonAIAgentState* const agentState, int threadID);
	
	NewtonAIWorld* NewtonAIAgentGetAIWorld (NewtonAIAgent* const agent);

	void NewtonAIAgentSetUserData (NewtonAIAgent* const agent, void* const userData);
	void* NewtonAIAgentGetUserData (NewtonAIAgent* const agent);

	void NewtonAIAgentSetDestroyCallback (NewtonAIAgent* const agentState, OnNewtonAIAgentDestroyCallback callback);
	OnNewtonAIAgentDestroyCallback NewtonAIAgentGetDestroyCallback (NewtonAIAgent* const agentState);

	void NewtonAIAgentSetUpdateCallback (NewtonAIAgent* const agent, OnNewtonAIAgentUpdateCallback callback);
	OnNewtonAIAgentUpdateCallback NewtonAIAgentGetUpdateCallback (NewtonAIAgent* const agent);



	// **********************************************************************************************
	//
	// AI Agents State interface
	//
	// **********************************************************************************************
	NewtonAIAgentState* NewtonAIAgentCreateState (NewtonAIAgent* const agent);
	void NewtonAIAgentDestroyState (NewtonAIAgent* const agent, NewtonAIAgentState* const state);
	void NewtonAIAgentSetStartState (NewtonAIAgent* const agent, NewtonAIAgentState* const state);
	
	NewtonAIAgent* NewtonAIAgentStateGetAgent (NewtonAIAgentState* const agentState);

	NewtonAIAgentTransition* NewtonAIAgentConnectStates (NewtonAIAgentState* const srcState, NewtonAIAgentState* const trgState, int transitionsymbol);
	void NewtonAIAgentDisconnectStates (NewtonAIAgentState* const srcState, NewtonAIAgentState* const trgState);
	
	void NewtonAIAgentStateSetUserData (NewtonAIAgentState* const agentState, void* const userData);
	void* NewtonAIAgentStateGetUserData (NewtonAIAgentState* const agentState);

	void NewtonAIAgentStateSetDestroyCallback (NewtonAIAgentState* const agentState, OnNewtonAIAgentStateDestroyCallback callback);
	OnNewtonAIAgentStateDestroyCallback NewtonAIAgentStateGetDestroyCallback (NewtonAIAgentState* const agentState);

	void NewtonAIAgentStateSetUpdateCallback (NewtonAIAgentState* const agentState, OnNewtonAIAgentStateUpdateCallback callback);
	OnNewtonAIAgentStateUpdateCallback NewtonAIAgentStateGetUpdateCallback (NewtonAIAgentState* const agentState);

	void NewtonAIAgentStateSetEnterCallback (NewtonAIAgentState* const agentState, OnNewtonAIAgentStateEnterExitCallback callback);
	OnNewtonAIAgentStateUpdateCallback NewtonAIAgentStateGetEnterCallback (NewtonAIAgentState* const agentState);

	void NewtonAIAgentStateSetExitCallback (NewtonAIAgentState* const agentState, OnNewtonAIAgentStateEnterExitCallback callback);
	OnNewtonAIAgentStateUpdateCallback NewtonAIAgentStateGetExitCallback (NewtonAIAgentState* const agentState);

	int NewtonAgentStateGetAdjacentTransitions (NewtonAIAgentState* const srcState, int* transitionSymbolsBuffer, int maxCount);
	int NewtonAgentStateGetAdjacentStates (NewtonAIAgentState* const srcState, NewtonAIAgentState** const agentsBuffer, int maxCount);
	



#ifdef __cplusplus 
}
#endif
#endif



