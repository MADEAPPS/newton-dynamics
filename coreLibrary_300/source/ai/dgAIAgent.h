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

#ifndef __dgAIAgent__
#define __dgAIAgent__

class dgAIAgent;
class dgAIWorld;
class dgAIAgentState;



typedef void (*OnAIAgentUpdate) (dgAIAgent* const agent, dgFloat32 timestep, dgInt32 threadID);
typedef void (*OnAIAgentDestroy) (dgAIAgent* const agent);

typedef void (*OnAIAgentStateEnterExit) (void* const agentStateNode, dgInt32 threadID);
typedef void (*OnAIAgentStateUpdate) (void* const agentStateNode, dgFloat32 timestep, dgInt32 threadID);


typedef void (*OnAIAgentStateDestroy) (void* const agentStateNode);


class dgAIAgentTransition
{
	public:
	dgAIAgentTransition()
		:m_symbol(0)
	{
	}
	
	dgInt32 m_symbol;
};

class dgAIAgentState
{
	public:
	dgAIAgentState ();
	~dgAIAgentState ();

	void SetUserData (void* const userData);
	void* GetUserData () const;

	void SetDestroyCallback (OnAIAgentStateDestroy callback);
	OnAIAgentStateDestroy GetDestroyCallback () const;

	void SetUpdateCallback (OnAIAgentStateUpdate callback);
	OnAIAgentStateUpdate GetUpdateCallback () const;

	void SetEnterCallback  (OnAIAgentStateEnterExit callback);
	OnAIAgentStateEnterExit GetEnterCallback () const;

	void SetExitCallback  (OnAIAgentStateEnterExit callback);
	OnAIAgentStateEnterExit GetExitCallback () const;

	void* m_userData;
	dgAIAgent* m_agent;
	OnAIAgentStateEnterExit m_enter;
	OnAIAgentStateEnterExit m_exit;
	OnAIAgentStateUpdate m_updateFnt;
	OnAIAgentStateDestroy m_destroyFnt;
};

class dgAIAgentStatesGraph: public dgGraph<dgAIAgentState, dgAIAgentTransition> 
{
	public:
	dgAIAgentStatesGraph (dgMemoryAllocator* const allocator);
	~dgAIAgentStatesGraph();

	void SetStateUserData (dgListNode* const state, void* const userData);
	void* GetStateUserData (dgListNode* const state) const;

	void SetStateDestroyCallback (dgListNode* const state, OnAIAgentStateDestroy callback);
	OnAIAgentStateDestroy GetStateDestroyCallback (dgListNode* const state) const;

	void SetStateUpdateCallback (dgListNode* const state, OnAIAgentStateUpdate callback);
	OnAIAgentStateUpdate GetStateUpdateCallback (dgListNode* const state) const;

	void SetStateEnterCallback (dgListNode* const state, OnAIAgentStateEnterExit callback);
	OnAIAgentStateEnterExit GetStateEnterCallback (dgListNode* const state) const;

	void SetStateExitCallback (dgListNode* const state, OnAIAgentStateEnterExit callback);
	OnAIAgentStateEnterExit GetStateExitCallback (dgListNode* const state) const;

	void Update (dgListNode* const state, dgFloat32 timestep, dgInt32 threadID);

	void Enter (dgListNode* const state, dgInt32 threadID);
	void Exit (dgListNode* const state, dgInt32 threadID);

	virtual dgListNode* CreateState();
	virtual void DestroyState(dgListNode* const state);
};



DG_MSC_VECTOR_ALIGMENT
class dgAIAgent: public dgAIAgentStatesGraph
{
	public:
	DG_CLASS_ALLOCATOR(allocator)
	dgAIAgent(dgMemoryAllocator* const allocator);
	virtual ~dgAIAgent(void);

	void* GetUserData() const;
	void SetUserData (void* const userData);
	void SetUpdateFunction (OnAIAgentUpdate fnt);
	void SetDestroyFunction (OnAIAgentDestroy fnt);
	OnAIAgentUpdate GetUpdateFunction () const;
	OnAIAgentDestroy GetDestroyFunction () const;

	dgAIWorld* GetWorld() const;
	void* GetAIWorldNode() const;

	void SetStartState (dgListNode* const startState);
	dgListNode* GetStartState () const;
	dgListNode* GetCurrentState () const;
	void GotoState (dgListNode* const targetState, dgInt32 threadID);
	void ApplyTransition (dgInt32 transitionSymbol, dgInt32 threadID);

	dgInt32 GetAdjacentStates (dgListNode* const srcState, dgListNode** const adjacentStates, dgInt32 maxCount);
	dgInt32 GetAdjacentTransitions (dgListNode* const srcState, dgInt32* const adjacentTransitions, dgInt32 maxCount);

    void Update (dgFloat32 timestep, dgInt32 threadID);
	void UnlinkStates(dgListNode* const srcState, dgListNode* const trgState);
	dgGraphNode<dgAIAgentState, dgAIAgentTransition>::dgListNode* LinkStates(dgListNode* const srcState, dgListNode* const trgState, dgInt32 transitionSymbol);


	protected:
	void* m_userData;
	dgAIWorld* m_world;
	void* m_myWorldNode;
	dgListNode* m_startState;
	dgListNode* m_currentState;
	OnAIAgentUpdate m_updateFnt;
	OnAIAgentDestroy m_destroyFnt;

	friend class dgAIWorld;
	friend class dgAIAgentGraph;
	friend class dgAIAgentStatesGraph;
} DG_GCC_VECTOR_ALIGMENT ;

#endif