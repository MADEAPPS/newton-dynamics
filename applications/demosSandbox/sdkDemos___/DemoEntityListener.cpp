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

#include "toolbox_stdafx.h"
#include "DemoEntity.h"
#include "DemoEntityManager.h"
#include "DemoEntityListener.h"


DemoEntityListener::DemoEntityListener(DemoEntityManager* const scene)
	:DemoListenerBase (scene, "entityListener")
{
}

DemoEntityListener::~DemoEntityListener()
{
}



void DemoEntityListener::PreUpdateKernel (NewtonWorld* const world, void* const userData, int threadIndex)
{
 	DemoEntityManager::dListNode* const node = (DemoEntityManager::dListNode*) userData;
	DemoEntityManager* const scene = (DemoEntityManager*) NewtonWorldGetUserData(world);

	dFloat timestep = scene->m_currentListenerTimestep;
	DemoEntity* const entity = node->GetInfo();
	entity->AddRef();
	entity->SimulationPreListener(scene, node, timestep);
	entity->Release();
}

void DemoEntityListener::PostUpdateKernel (NewtonWorld* const world, void* const userData, int threadIndex)
{
	DemoEntityManager::dListNode* const node = (DemoEntityManager::dListNode*) userData;
	DemoEntityManager* const scene = (DemoEntityManager*) NewtonWorldGetUserData(world);

	dFloat timestep = scene->m_currentListenerTimestep;
	DemoEntity* const entity = node->GetInfo();
	entity->AddRef();
	entity->SimulationPostListener(scene, node, timestep);
	entity->Release();
}



void DemoEntityListener::PreUpdate (const NewtonWorld* const world, dFloat timestep)
{
	DemoEntityManager::dListNode* nextNode;
	DemoEntityManager* const scene = (DemoEntityManager*) NewtonWorldGetUserData(world);
	scene->m_currentListenerTimestep = timestep;
	for (DemoEntityManager::dListNode* node = scene->dList<DemoEntity*>::GetFirst(); node; node = nextNode) {
		nextNode = node->GetNext();
		NewtonDispachThreadJob(world, PreUpdateKernel, node);
	}
	NewtonSyncThreadJobs(world);
}

void DemoEntityListener::PostUpdate (const NewtonWorld* const world, dFloat timestep)
{
	DemoEntityManager::dListNode* nextNode;
	DemoEntityManager* const scene = (DemoEntityManager*) NewtonWorldGetUserData(world);
	scene->m_currentListenerTimestep = timestep;
	for (DemoEntityManager::dListNode* node = scene->dList<DemoEntity*>::GetFirst(); node; node = nextNode) {
		nextNode = node->GetNext();
		NewtonDispachThreadJob(world, PostUpdateKernel, node);
	}
	NewtonSyncThreadJobs(world);
}


