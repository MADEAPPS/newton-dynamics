/* Copyright (c) <2003-2016> <Newton Game Dynamics>
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
	:dCustomListener (scene->GetNewton(), "entityListener")
{
}

DemoEntityListener::~DemoEntityListener()
{
}

void DemoEntityListener::PreUpdateKernel (NewtonWorld* const world, void* const userData, int threadIndex)
{
	const int threadCount = NewtonGetThreadsCount(world);
	DemoEntityManager* const scene = (DemoEntityManager*) NewtonWorldGetUserData(world);
	DemoEntityManager::dListNode* node = (DemoEntityManager::dListNode*) userData;
	if (node) {
		do {
			dFloat timestep = scene->m_currentListenerTimestep;
			DemoEntity* const entity = node->GetInfo();
			entity->AddRef();
			entity->SimulationPreListener(scene, node, timestep);
			entity->Release();
			for (int i = 0; i < threadCount; i++) {
				node = node ? node->GetNext() : NULL;
			}
		} while (node);
	}
}

void DemoEntityListener::PostUpdateKernel (NewtonWorld* const world, void* const userData, int threadIndex)
{
	const int threadCount = NewtonGetThreadsCount(world);
	DemoEntityManager* const scene = (DemoEntityManager*)NewtonWorldGetUserData(world);
	DemoEntityManager::dListNode* node = (DemoEntityManager::dListNode*) userData;
	if (node) {
		do {
			dFloat timestep = scene->m_currentListenerTimestep;
			DemoEntity* const entity = node->GetInfo();
			entity->AddRef();
			entity->SimulationPostListener(scene, node, timestep);
			entity->Release();
			for (int i = 0; i < threadCount; i++) {
				node = node ? node->GetNext() : NULL;
			}
		} while (node);
	}
}

void DemoEntityListener::PreUpdate (dFloat timestep)
{
	NewtonWorld* const world = GetWorld();
	const int threadCount = NewtonGetThreadsCount(world);
	DemoEntityManager* const scene = (DemoEntityManager*) NewtonWorldGetUserData(world);
	scene->m_currentListenerTimestep = timestep;
	DemoEntityManager::dListNode* node = scene->dList<DemoEntity*>::GetFirst();
	for (int i = 0; i < threadCount; i ++) {
		NewtonDispachThreadJob(world, PreUpdateKernel, node, "DemoEntityListener");
		node = node ? node->GetNext() : NULL;
	}
	NewtonSyncThreadJobs(world);
}

void DemoEntityListener::PostUpdate (dFloat timestep)
{
	NewtonWorld* const world = GetWorld();
	const int threadCount = NewtonGetThreadsCount(world);
	DemoEntityManager* const scene = (DemoEntityManager*)NewtonWorldGetUserData(world);
	scene->m_currentListenerTimestep = timestep;
	DemoEntityManager::dListNode* node = scene->dList<DemoEntity*>::GetFirst();
	for (int i = 0; i < threadCount; i++) {
		NewtonDispachThreadJob(world, PostUpdateKernel, node, "PostUpdateKernel");
		node = node ? node->GetNext() : NULL;
	}
	NewtonSyncThreadJobs(world);
}


