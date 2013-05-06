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


#ifndef _D_AI_AGENT_H_
#define _D_AI_AGENT_H_


class dAI;
class dAIAgentState;

class dAIAgent
{
	public: 
	dAIAgent (dAI* const manager);

	virtual ~dAIAgent ();
	virtual void Update (dFloat timestep, int threadID){}

	dAI* GetWorld() const;
	dAIAgentState* GetStartState () const;
	void SetStartState (dAIAgentState* const state);
	
	void GoToState (dAIAgentState* const state, int threadIndex);
	void ApplyTransition (int transitionSymbol, int threadID);



	NewtonAIAgent* GetNewtonAIAgent() const;
	dAIAgentState* GetCurrentState () const;



	void* LinkStates (dAIAgentState* const sourceState, dAIAgentState* const targetState, int thansitionSignal);
	void UnlinkStates (dAIAgentState* const sourceState, dAIAgentState* const targetState);

	protected:
	dAIAgent (NewtonAIAgent* const agent);

	private:
	static void Destroy (NewtonAIAgent* const agent);
	static void Update (NewtonAIAgent* const agent, dFloat timestep, int threadID);

	protected:
	NewtonAIAgent* m_agent;
	bool m_autoDestroy;

	friend class dAI;
	friend class dAIAgentState;
};



inline NewtonAIAgent* dAIAgent::GetNewtonAIAgent() const
{
	return m_agent;
}

#endif 

