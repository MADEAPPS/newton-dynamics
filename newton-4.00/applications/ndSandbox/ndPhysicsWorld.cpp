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

#define MAX_PHYSICS_FPS			60.0f
//#define MAX_PHYSICS_SUB_STEPS	2

ndPhysicsWorld::ndPhysicsWorld(ndDemoEntityManager* const manager)
	:ndWorld()
	,m_manager(manager)
{
}

ndPhysicsWorld::~ndPhysicsWorld()
{
}

void ndPhysicsWorld::ResetTimer()
{
	m_microsecunds = dGetTimeInMicrosenconds();
}


void ndPhysicsWorld::AdvanceTime()
{
/*
	dFloat32 timestepInSecunds = 1.0f / MAX_PHYSICS_FPS;
	dUnsigned64 timestepMicrosecunds = dUnsigned64(timestepInSecunds * 1000000.0f);

	dUnsigned64 currentTime = dGetTimeInMicrosenconds();
	dUnsigned64 nextTime = currentTime - m_microsecunds;
	if (nextTime > timestepMicrosecunds * 2)
	{
		m_microsecunds = currentTime - timestepMicrosecunds * 2;
		nextTime = currentTime - m_microsecunds;
	}

	bool newUpdate = false;
	dFloat32 physicsTime = 0.0f;
	//while (nextTime >= timestepMicrosecunds) 
	if (nextTime >= timestepMicrosecunds)
	{
		newUpdate = true;
		//ClearDebugDisplay(m_world);

		Update (timestep)
		if (m_manage->m_asynchronousPhysicsUpdate)
		{
			dAssert(0);
		}
		else
		{
			NewtonUpdate(m_world, timestepInSecunds);
		}

		physicsTime += NewtonGetLastUpdateTime(m_world);

		nextTime -= timestepMicrosecunds;
		m_microsecunds += timestepMicrosecunds;
	}

	if (newUpdate)
	{
		m_physicsFramesCount++;
		m_mainThreadPhysicsTimeAcc += physicsTime;
		if (m_physicsFramesCount >= 16)
		{
			m_mainThreadPhysicsTime = m_mainThreadPhysicsTimeAcc / m_physicsFramesCount;
			m_physicsFramesCount = 0;
			m_mainThreadPhysicsTimeAcc = 0.0f;
		}
	}
*/
	bool isSyncUpdate = !m_manager->m_asynchronousPhysicsUpdate;
	dUnsigned64 time1 = dGetTimeInMicrosenconds();
	dUnsigned64 currentTime = time1 - m_microsecunds;
	const dUnsigned64 deltaTime = dUnsigned64(1.0e6f / MAX_PHYSICS_FPS);
	if (currentTime > deltaTime)
	{
		D_TRACKTIME();
		Update(1.0f / MAX_PHYSICS_FPS);
		if (isSyncUpdate)
		{
			Sync();
		}

		m_microsecunds += deltaTime;
		dTrace (("%f\n", currentTime * 1.0e-3f));
	}

}