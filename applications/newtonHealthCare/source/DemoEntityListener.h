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

#ifndef __DEMO_ENTITY_LISTENER_H__
#define __DEMO_ENTITY_LISTENER_H__

#include "toolbox_stdafx.h"
#include "DemoListenerManager.h"

class DemoEntityListener: public DemoListener
{
	public:
	DemoEntityListener(const char* const name);
	~DemoEntityListener();

	virtual void Update (DemoEntityManager* const scene, dFloat timestep);
};


#endif