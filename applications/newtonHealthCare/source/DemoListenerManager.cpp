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
#include "DemoEntityManager.h"
#include "DemoListenerManager.h"

DemoListener::DemoListener(const char* const name)
{
	strncpy (m_name, name, sizeof (m_name) - 1);
}

DemoListener::~DemoListener()
{

}



DemoListenerManager::DemoListenerManager()
{
}

DemoListenerManager::~DemoListenerManager()
{
}


void DemoListenerManager::CleanUP (DemoEntityManager* const scene)
{
	for (dListNode* node = GetFirst(); node; node = node->GetNext()) {
		delete node->GetInfo();
	}
	RemoveAll();
}

void DemoListenerManager::Update(const NewtonWorld* const world, dFloat tiemestep)
{
	DemoEntityManager* const scene = (DemoEntityManager*) NewtonWorldGetUserData(world);

	for (dListNode* node = scene->m_listenerManager.GetFirst(); node; node = node->GetNext()) {
		
		node->GetInfo()->Update(scene, tiemestep);
	}
}

DemoListener* DemoListenerManager::Find (const char* const name)
{
	for (dListNode* node = GetFirst(); node; node = node->GetNext()) {
		DemoListener* const listener = node->GetInfo();
		if (!stricmp (listener->m_name, name)) {
			return listener;
		}
	}
	return NULL;
}
