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
#include "DemoListenerBase.h"

DemoListenerBase::DemoListenerBase(DemoEntityManager* const scene, const char* const listenerName)
//	:m_scene (scene)
{
	NewtonWorldAddPreListener (scene->GetNewton(), listenerName, this, PreUpdate, NULL);
	NewtonWorldAddPostListener (scene->GetNewton(), listenerName, this, PostUpdate, Destroy);
}

DemoListenerBase::~DemoListenerBase()
{
}


void DemoListenerBase::PreUpdate (const NewtonWorld* const world, void* const listenerUserData, dFloat timestep)
{
	DemoListenerBase* const me = (DemoListenerBase*) listenerUserData;
	me->PreUpdate(world, timestep);
}


void DemoListenerBase::PostUpdate (const NewtonWorld* const world, void* const listenerUserData, dFloat timestep)
{
	DemoListenerBase* const me = (DemoListenerBase*) listenerUserData;
	me->PostUpdate(world, timestep);
}

void DemoListenerBase::Destroy (const NewtonWorld* const world, void* const listenerUserData)
{
	DemoListenerBase* const me = (DemoListenerBase*) listenerUserData;
	delete me;
}

