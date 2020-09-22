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

#include "ndSandboxStdafx.h"
#include "ndPhysicsWorld.h"
#include "ndDemoEntityManager.h"


#define MAX_PHYSICS_FPS				60.0f
#define MAX_PHYSICS_RECOVER_STEPS	2

ndPhysicsWorld::ndPhysicsWorld(ndDemoEntityManager* const manager)
	:ndWorld()
	,m_manager(manager)
	,m_timeAccumulator(0.0f)
{
	ClearCache();
}

ndPhysicsWorld::~ndPhysicsWorld()
{
}


void ndPhysicsWorld::AdvanceTime(dFloat32 timetep)
{
	const dFloat32 timeLimit = (1.0f / MAX_PHYSICS_FPS);
	m_timeAccumulator += timetep;
	if (m_timeAccumulator > timeLimit)
	{
		D_TRACKTIME();

		//dTrace (("%f\n", m_timeAccumulator));
		for (int recover = MAX_PHYSICS_RECOVER_STEPS; recover && (m_timeAccumulator > timeLimit); recover--)
		{
			Update(timeLimit);
			m_timeAccumulator -= timeLimit;
		}
		if (!m_manager->m_asynchronousPhysicsUpdate)
		{
			Sync();
		}
	}
}