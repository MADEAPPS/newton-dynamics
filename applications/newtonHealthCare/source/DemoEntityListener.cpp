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
#include "DemoCamera.h"
#include "DemoEntityManager.h"
#include "DemoEntityListener.h"


DemoEntityListener::DemoEntityListener(const char* const name)
	:DemoListener (name)
{
}

DemoEntityListener::~DemoEntityListener()
{
}

void DemoEntityListener::Update (DemoEntityManager* const scene, dFloat timestep)
{
	DemoEntityManager::dListNode* nextNode;
	for (DemoEntityManager::dListNode* node = scene->GetFirst(); node; node = nextNode) {
		nextNode = node->GetNext();

		DemoEntity* const entity = node->GetInfo();
		entity->AddRef();
		entity->SimulationLister(scene, node, timestep);
		entity->Release();
	}

}



