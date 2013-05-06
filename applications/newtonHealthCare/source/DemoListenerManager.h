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

#ifndef __DEMO_LISTENER_MANAGER_H__
#define __DEMO_LISTENER_MANAGER_H__

#include "toolbox_stdafx.h"

class DemoListener
{
	public:
	DemoListener(const char* const name);
	virtual ~DemoListener();
	virtual void Update (DemoEntityManager* const scene, dFloat timestep) = 0;

	char m_name[32];
};

class DemoListenerManager: public dList<DemoListener*> 
{
	public:
	DemoListenerManager();
	~DemoListenerManager();

	DemoListener* Find (const char* const name);
	void CleanUP (DemoEntityManager* const scene);
	static void Update(const NewtonWorld* const world, dFloat tiemstep);
};

#endif