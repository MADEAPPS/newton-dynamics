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


#ifndef _D_NEWTON_PLAYER_MANAGER_H_
#define _D_NEWTON_PLAYER_MANAGER_H_

#include "dStdAfxNewton.h"
//#include "dNewtonAlloc.h"

class dNewtonPlayerManager: public CustomPlayerControllerManager
{
	public:
	CNEWTON_API dNewtonPlayerManager (dNewton* const world);
	CNEWTON_API virtual ~dNewtonPlayerManager ();

//	virtual dPlayerController* CreatePlayer (Real mass, Real outerRadius, Real innerRadius, Real height, Real stairStep);
//	virtual void DestroyPlayer (dPlayerController* const player);
//	virtual void ApplyPlayerMove (CustomPlayerController* const controller, Real timestep);
};

/*
class dPlayerController: public dNewtonAlloc
{
	public:
	~dPlayerController();

	private:
	dPlayerController (CustomPlayerControllerManager::CustomController* const controller);
	CustomPlayerControllerManager::CustomController* m_controller;

	friend class dNewtonPlayerManager;
};
*/


#endif
