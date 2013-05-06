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

#ifndef __DEMO_AI_LISTENER_H__
#define __DEMO_AI_LISTENER_H__

#include "toolbox_stdafx.h"
#include "DemoListenerManager.h"

class DemoAIListener: public DemoListener, public dAI
{
	public:
	DemoAIListener(const char* const name);
	~DemoAIListener();

	virtual void Update (DemoEntityManager* const scene, dFloat timestep);
};


#endif