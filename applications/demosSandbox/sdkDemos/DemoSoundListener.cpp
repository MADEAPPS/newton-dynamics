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
#include "DemoSoundListener.h"


DemoSoundListener::DemoSoundListener(DemoEntityManager* const scene)
	:DemoListenerBase (scene, "soundListenr")
	,dSoundManager()
{
}

DemoSoundListener::~DemoSoundListener()
{
	// Clean all pending sounds
	DestroyAllSound();
}

void DemoSoundListener::PreUpdate (const NewtonWorld* const world, dFloat timestep)
{
	// do nothong here;
}

void DemoSoundListener::PostUpdate (const NewtonWorld* const world, dFloat timestep)
{
	DemoEntityManager* const scene = (DemoEntityManager*) NewtonWorldGetUserData(world);

	DemoCamera* const camera = scene->GetCamera();
	dMatrix matrix0 (camera->GetCurrentMatrix ());
	dMatrix matrix1 (camera->GetNextMatrix ());
	dVector veloc ((matrix1.m_posit - matrix0.m_posit).Scale (1.0f / timestep));
	//printf ("%f %f %f %f\n", veloc.m_x, veloc.m_y, veloc.m_z, timestepInSecunds);

	UpdateListener (matrix1.m_posit, veloc, matrix0.m_front, matrix0.m_up);
	dSoundManager::Update();
}



