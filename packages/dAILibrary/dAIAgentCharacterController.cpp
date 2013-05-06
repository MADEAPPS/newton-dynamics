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


#include "dAI.h"
#include "dAIAgentCharacterController.h"



dAIAgentCharacterController::dLocomotionAgent::dLocomotionAgent (dAI* const manager)
	:dAIAgent (manager)
{

}

dAIAgentCharacterController::dLocomotionAgent::~dLocomotionAgent ()
{

}

void dAIAgentCharacterController::dLocomotionAgent::Update (dFloat timestep, int threadID)
{

}


dAIAgentCharacterController::dAIAgentCharacterController (dAI* const manager, NewtonBody* const playerBody)
	:dAIAgent (manager)
{
	m_locomotionSystem = new dLocomotionAgent (manager);
}

dAIAgentCharacterController::~dAIAgentCharacterController ()
{
	delete m_locomotionSystem;
}

void dAIAgentCharacterController::Update (dFloat timestep, int threadID)
{
}
