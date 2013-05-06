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


#ifndef _D_NEWTON_AI_AGENT_H_
#define _D_NEWTON_AI_AGENT_H_


class DemoEntityManager;



class DemoAIAgent: public dAIAgent
{
	public:
	DemoAIAgent (DemoEntityManager* const scene);
	DemoAIAgent (DemoEntityManager* const scene, NewtonAIAgent* const agent);

	protected:
	DemoEntityManager* m_scene;
};


class DemoAIState: public dAIAgentState
{
	public: 
	DemoAIState(DemoAIAgent* const agent, DemoEntityManager* const scene);
	virtual ~DemoAIState();

	DemoEntityManager* m_scene;
};



#endif 

