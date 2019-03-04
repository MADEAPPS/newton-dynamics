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

#ifndef __DEMO_ENTITY_LISTENER_H__
#define __DEMO_ENTITY_LISTENER_H__

#include "toolbox_stdafx.h"
#include "dCustomListener.h"

class DemoEntityListener: public dCustomListener
{
	public:
	DemoEntityListener(DemoEntityManager* const scene);
	~DemoEntityListener();

	private:
	virtual void PreUpdate (dFloat timestep);
	virtual void PostUpdate (dFloat timestep);

	static void PreUpdateKernel (NewtonWorld* const world, void* const userData, int threadIndex);
	static void PostUpdateKernel (NewtonWorld* const world, void* const userData, int threadIndex);
};

#endif