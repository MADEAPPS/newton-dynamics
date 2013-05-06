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
#include "DemoAIListener.h"


DemoAIListener::DemoAIListener(const char* const name)
	:DemoListener(name)
	,dAI()
{
}

DemoAIListener::~DemoAIListener()
{
}

void DemoAIListener::Update (DemoEntityManager* const scene, dFloat timestep)
{
//	unsigned64 aiT0 = dGetTimeInMicrosenconds ();					
	dAI::Update(timestep);

//	unsigned64 aiT1 = dGetTimeInMicrosenconds ();					
}



