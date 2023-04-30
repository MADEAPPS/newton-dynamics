/* Copyright (c) <2003-2022> <Newton Game Dynamics>
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
#include "ndDemoMesh.h"
#include "ndDemoCamera.h"
#include "ndPhysicsWorld.h"
#include "ndSoundManager.h"
#include "ndContactCallback.h"
#include "ndDemoEntityNotify.h"
#include "ndDemoEntityManager.h"
#include "ndDemoCameraManager.h"
#include "ndDemoMeshInterface.h"
#include "ndBasicPlayerCapsule.h"
#include "ndArchimedesBuoyancyVolume.h"

#define MAX_PHYSICS_STEPS			1
#define MAX_PHYSICS_FPS				60.0f

ndPhysicsWorld::ndDefferentDeleteEntities::ndDefferentDeleteEntities(ndDemoEntityManager* const manager)
	:ndArray<ndDemoEntity*>()
	,m_manager(manager)
	,m_renderThreadId(std::this_thread::get_id())
{
	static ndPhysicsWorldFileLoadSave saveLoad;
}

void ndPhysicsWorld::ndDefferentDeleteEntities::Update()
{
	for (ndInt32 i = 0; i < GetCount(); ++i)
	{
		RemoveEntity((*this)[i]);
	}
	SetCount(0);
}

void ndPhysicsWorld::ndDefferentDeleteEntities::RemoveEntity(ndDemoEntity* const entity)
{
	ndAssert(entity->m_rootNode);
	if (m_renderThreadId == std::this_thread::get_id())
	{
		m_manager->RemoveEntity(entity);
		delete entity;
	}
	else
	{
		ndScopeSpinLock lock(entity->m_lock);
		if (!entity->m_isDead)
		{
			entity->m_isDead = true;
			PushBack(entity);
		}
	}
}

ndPhysicsWorld::ndPhysicsWorld(ndDemoEntityManager* const manager)
	:ndWorld()
	,m_manager(manager)
	,m_soundManager(new ndSoundManager(manager))
	,m_timeAccumulator(0.0f)
	,m_deadEntities(manager)
{
	ClearCache();
	SetContactNotify(new ndContactCallback);
}

ndPhysicsWorld::~ndPhysicsWorld()
{
	CleanUp();
}

void ndPhysicsWorld::CleanUp()
{
	ndWorld::CleanUp();
	if (m_soundManager)
	{
		delete m_soundManager;
		m_soundManager = nullptr;
	}
}

void ndPhysicsWorld::RemoveEntity(ndDemoEntity* const entity)
{
	ndAssert(entity->m_rootNode);
	m_deadEntities.RemoveEntity(entity);
}

ndDemoEntityManager* ndPhysicsWorld::GetManager() const
{
	return m_manager;
}

ndSoundManager* ndPhysicsWorld::GetSoundManager() const
{
	return m_soundManager;
}

void ndPhysicsWorld::PreUpdate(ndFloat32 timestep)
{
	ndWorld::PreUpdate(timestep);

	static ndInt32 xxxx = 0;
	xxxx++;
	if (xxxx % 200 == 0)
	{
		const ndBodyListView& bodyList = GetBodyList();
		if (bodyList.GetCount() > 1)
		{
			//ndInt32 index = ndInt32(ndRandInt() % (bodyList.GetCount() - 1) + 1);
			//RemoveBody(bodyList.GetView()[index]);
		}
	}
}

void ndPhysicsWorld::PostUpdate(ndFloat32 timestep)
{
	ndWorld::PostUpdate(timestep);
	m_manager->m_cameraManager->FixUpdate(m_manager, timestep);
	if (m_manager->m_updateCamera)
	{
		m_manager->m_updateCamera(m_manager, m_manager->m_updateCameraContext, timestep);
	}

	if (m_soundManager)
	{
		m_soundManager->Update(this, timestep);
	}
}

void ndPhysicsWorld::AdvanceTime(ndFloat32 timestep)
{
	D_TRACKTIME();
	const ndFloat32 descreteStep = (1.0f / MAX_PHYSICS_FPS);

	ndInt32 maxSteps = MAX_PHYSICS_STEPS;
	m_timeAccumulator += timestep;

	// if the time step is more than max timestep par frame, throw away the extra steps.
	if (m_timeAccumulator > descreteStep * (ndFloat32)maxSteps)
	{
		ndFloat32 steps = ndFloor(m_timeAccumulator / descreteStep) - (ndFloat32)maxSteps;
		ndAssert(steps >= 0.0f);
		m_timeAccumulator -= descreteStep * steps;
	}

	while (m_timeAccumulator > descreteStep)
	{
		Update(descreteStep);
		m_timeAccumulator -= descreteStep;
	}
	if (m_manager->m_synchronousPhysicsUpdate)
	{
		Sync();
	}

	m_deadEntities.Update();
}