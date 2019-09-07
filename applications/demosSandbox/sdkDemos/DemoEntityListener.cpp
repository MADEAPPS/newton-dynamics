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

#include "toolbox_stdafx.h"
#include "DemoEntity.h"
#include "DemoEntityManager.h"
#include "DemoEntityListener.h"


DemoEntityListener::DemoEntityListener(DemoEntityManager* const scene)
	:dCustomParallelListener(scene->GetNewton(), "entityListener")
{
}

DemoEntityListener::~DemoEntityListener()
{
}

void DemoEntityListener::PreUpdate(dFloat timestep, int threadID)
{
	D_TRACKTIME();
	NewtonWorld* const world = GetWorld();
	const int threadCount = NewtonGetThreadsCount(world);
	DemoEntityManager* const scene = (DemoEntityManager*) NewtonWorldGetUserData(world);

	DemoEntityManager::dListNode* node = scene->dList<DemoEntity*>::GetFirst();
	for (int i = 0; i < threadID; i++) {
		node = node ? node->GetNext() : NULL;
	}
	if (node) {
		do {
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

void DemoEntityListener::PostUpdate(dFloat timestep, int threadID)
{
	D_TRACKTIME();
	NewtonWorld* const world = GetWorld();
	const int threadCount = NewtonGetThreadsCount(world);
	DemoEntityManager* const scene = (DemoEntityManager*)NewtonWorldGetUserData(world);

	DemoEntityManager::dListNode* node = scene->dList<DemoEntity*>::GetFirst();
	for (int i = 0; i < threadID; i++) {
		node = node ? node->GetNext() : NULL;
	}
	if (node) {
		do {
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


