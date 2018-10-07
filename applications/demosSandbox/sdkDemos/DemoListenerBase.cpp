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
#include "DemoListenerBase.h"
#include "DemoEntityManager.h"

DemoListenerBase::DemoListenerBase(DemoEntityManager* const scene, const char* const listenerName)
{
	NewtonWorld* const world = scene->GetNewton();
	void* const listener = NewtonWorldAddListener (world, listenerName, this);

	NewtonWorldListenerSetDestructorCallback (world, listener, Destroy);
	NewtonWorldListenerSetPreUpdateCallback (world, listener, PreUpdate);
	NewtonWorldListenerSetPostUpdateCallback (world, listener, PostUpdate);
	NewtonWorldListenerSetBodyDestroyCallback (world, listener, OnBodyDestroy);
}

DemoListenerBase::~DemoListenerBase()
{
}

void DemoListenerBase::OnBodyDestroy (NewtonBody* const body)
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

void DemoListenerBase::OnBodyDestroy (const NewtonWorld* const world, void* const listener, NewtonBody* const body)
{
	DemoListenerBase* const me = (DemoListenerBase*) NewtonWorldGetListenerUserData(world, listener);
	me->OnBodyDestroy(body);
}
