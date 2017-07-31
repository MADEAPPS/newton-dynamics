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

#ifndef __DEMO_LISTENER_BASE_MANAGER_H__
#define __DEMO_LISTENER_BASE_MANAGER_H__

#include "toolbox_stdafx.h"

class DemoEntityManager;

class DemoListenerBase
{
	public:
	DemoListenerBase(DemoEntityManager* const scene, const char* const listenerName);
	virtual ~DemoListenerBase();
	
	virtual void PreUpdate (const NewtonWorld* const world, dFloat timestep) = 0;
	virtual void PostUpdate (const NewtonWorld* const world, dFloat timestep) = 0;

	virtual void OnBodyDestroy (NewtonBody* const body);

	private:
	static void PreUpdate(const NewtonWorld* const world, void* const listenerUserData, dFloat tiemstep);
	static void PostUpdate(const NewtonWorld* const world, void* const listenerUserData, dFloat tiemstep);
	static void Destroy (const NewtonWorld* const world, void* const listenerUserData);
	static void OnBodyDestroy (const NewtonWorld* const world, void* const listener, NewtonBody* const body);
};

#endif