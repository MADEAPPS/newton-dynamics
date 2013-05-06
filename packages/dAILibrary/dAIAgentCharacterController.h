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


#ifndef _D_AI_AGENT_CHARACTER_CONTROLLER_H_
#define _D_AI_AGENT_CHARACTER_CONTROLLER_H_

#include "dAIAgent.h"

class dAIAgentCharacterController: public dAIAgent
{
	// locomotion states
	class dLocomotionAgent: public dAIAgent 
	{
		public:
		enum
		{
			m_idle,
			m_walk,
			m_strafe,
		};

		dLocomotionAgent (dAI* const manager);
		virtual ~dLocomotionAgent ();
		virtual void Update (dFloat timestep, int threadID);
	};

	public: 
	dAIAgentCharacterController (dAI* const manager, NewtonBody* const playerBody);
	virtual ~dAIAgentCharacterController ();
	virtual void Update (dFloat timestep, int threadID);


	dLocomotionAgent* m_locomotionSystem;
};

#endif 

