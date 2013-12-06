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

#include "StdAfx.h"
#include "Entity.h"
#include "TutorialCode.h"
#include "SceneManager.h"
#include "CollectInputAndUpdateCamera.h"

#pragma warning (disable: 4100) //unreferenced formal parameter

void CreateScene (NewtonWorld* world, SceneManager* sceneManager)
{
	Entity* floor;
	Entity* smilly;
	Entity* frowny;

	// initialize the camera
	InitCamera (dVector (-15.0f, 5.0f, 0.0f, 0.0f), dVector (1.0f, 0.0f, 0.0f));

	// Create a large body to be the floor
	floor = sceneManager->CreateEntity();
	floor->LoadMesh ("FloorBox.dat");

	// add some visual entities.
	smilly = sceneManager->CreateEntity();
	smilly->LoadMesh ("Smilly.dat");
	smilly->m_curPosition.m_y = 10.0f;
	smilly->m_prevPosition = smilly->m_curPosition;

	// add some visual entities.
	frowny = sceneManager->CreateEntity();
	frowny->LoadMesh ("Frowny.dat");
	frowny->m_curPosition.m_z = 0.4f;
	frowny->m_curPosition.m_y = 10.0f + 0.4f;
	frowny->m_prevPosition = frowny->m_curPosition;

}




