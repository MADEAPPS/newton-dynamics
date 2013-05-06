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


#ifndef _D_AI_AGENT_STATE_H_
#define _D_AI_AGENT_STATE_H_

class dAIAgent;


class dAIAgentState
{
	public: 
	dAIAgentState(dAIAgent* const agent);
	virtual ~dAIAgentState();

	virtual void Enter(int threadID){}
	virtual void Exit(int threadID){}
	virtual void Update (dFloat timestep, int threadID){}

	dAIAgent* GetAgent() const;
	NewtonAIAgent* GetNewtonAIAgent() const;
	NewtonAIAgentState* GetNetonAIState() const; 

	int GetAdjacentTranstions (int* const signalBuffer, int maxCount) const;
	int GetAdjacentStates (dAIAgentState** const agentBuffer, int maxCount) const;

	private:
	// call when the  AI agent is destroyed
	static void Destroy (NewtonAIAgentState* const agentState);

	static void Enter (NewtonAIAgentState* const agentState, int threadID);
	static void Exit (NewtonAIAgentState* const agentState, int threadID);
	static void Update (NewtonAIAgentState* const agentState, dFloat timestep, int threadID);

	protected:
	NewtonAIAgentState* m_state;
	bool m_autoDestroy;

	friend class dAI; 
	friend class dAIAgent; 
};


inline NewtonAIAgentState* dAIAgentState::GetNetonAIState() const
{
	return m_state;
}


#endif 

