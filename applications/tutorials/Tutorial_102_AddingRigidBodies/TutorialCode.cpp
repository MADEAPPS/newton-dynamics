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
#include "RigidBodyUtil.h"
#include "CollisionShapeUtil.h"
#include "CollectInputAndUpdateCamera.h"

void CreateScene (NewtonWorld* world, SceneManager* sceneManager)
{
	// initialize the camera
	InitCamera (dVector (-15.0f, 5.0f, 0.0f, 0.0f), dVector (1.0f, 0.0f, 0.0f));

	// Create a large body to be the floor
	Entity* const floor = sceneManager->CreateEntity();
	floor->LoadMesh ("FloorBox.dat");

	// add static floor Physics
	NewtonCollision* shape = CreateNewtonBox (world, floor, 0);
	NewtonBody* const floorBody = CreateRigidBody (world, floor, shape, 0.0f);
	NewtonDestroyCollision(shape);

	// set the Transformation Matrix for this rigid body
	dMatrix matrix (floor->m_curRotation, floor->m_curPosition);
	NewtonBodySetMatrix (floorBody, &matrix[0][0]);

	// add some visual entities.
	Entity* const smilly = sceneManager->CreateEntity();
	smilly->LoadMesh ("Smilly.dat");
	smilly->m_curPosition.m_y = 10.0f;
	smilly->m_prevPosition = smilly->m_curPosition;

	// add a body with a box shape
	shape = CreateNewtonBox (world, smilly, 0);
	CreateRigidBody (world, smilly, shape, 10.0f);
	NewtonDestroyCollision(shape);


	// add some visual entities.
	Entity* const frowny = sceneManager->CreateEntity();
	frowny->LoadMesh ("Frowny.dat");
	frowny->m_curPosition.m_z = 0.4f;
	frowny->m_curPosition.m_y = 10.0f + 0.4f;
	frowny->m_prevPosition = frowny->m_curPosition;

	// add a body with a Convex hull shape
	shape = CreateNewtonConvex (world, frowny, 0);
	CreateRigidBody (world, frowny, shape, 10.0f);
	NewtonDestroyCollision(shape);
}



