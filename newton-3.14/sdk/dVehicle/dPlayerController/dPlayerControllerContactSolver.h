/* Copyright (c) <2003-2019> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/


// NewtonPlayerControllerManager.h: interface for the NewtonPlayerControllerManager class.
//
//////////////////////////////////////////////////////////////////////

#ifndef D_PLAYER_CONTROLLER_CONTACT_SOLVER_H_
#define D_PLAYER_CONTROLLER_CONTACT_SOLVER_H_

#define D_PLAYER_MAX_CONTACTS		6
#define D_PLAYER_MAX_ROWS			(3 * D_PLAYER_MAX_CONTACTS) 

class dPlayerController;
class dPlayerControllerContactSolver
{
	public:
	dPlayerControllerContactSolver(dPlayerController* const controller)
		:m_controller(controller)
		,m_contactCount(0)
	{
	}

	void CalculateContacts();
	static unsigned PrefilterCallback(const NewtonBody* const body, const NewtonCollision* const collision, void* const userData);

	NewtonWorldConvexCastReturnInfo m_contactBuffer[D_PLAYER_MAX_ROWS];
	dPlayerController* m_controller;
	int m_contactCount;
};


#endif 

