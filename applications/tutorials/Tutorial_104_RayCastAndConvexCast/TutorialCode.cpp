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
#include "FindFloor.h"
#include "CastFloor.h"
#include "TutorialCode.h"
#include "SceneManager.h"
#include "RigidBodyUtil.h"
#include "CollisionShapeUtil.h"
#include "CollectInputAndUpdateCamera.h"


#define PYRAMID_BASE	 20

void CreateScene (NewtonWorld* world, SceneManager* sceneManager)
{
	Entity* floor;
	NewtonBody* floorBody;
	NewtonCollision* shape;

	// initialize the camera
	InitCamera (dVector (-15.0f, 5.0f, 0.0f, 0.0f), dVector (1.0f, 0.0f, 0.0f));

	// Create a large body to be the floor
	floor = sceneManager->CreateEntity();
	floor->LoadMesh ("FloorBox.dat");

	// add static floor Physics
	shape = CreateNewtonBox (world, floor, 0);
	floorBody = CreateRigidBody (world, floor, shape, 0.0f);
	NewtonDestroyCollision(shape);

	// set the Transformation Matrix for this rigid body
	dMatrix matrix (floor->m_curRotation, floor->m_curPosition);
	NewtonBodySetMatrix (floorBody, &matrix[0][0]);


	// find the floor base, and add some distance up;
	dFloat floorY = FindFloor (world, 0.0f, 0.0f) + 2.0f;

	// this is the width of the Box;
	dFloat boxWidth = 0.42f;

	// Make a small pyramid of Boxes
	for (int i = 0; i < PYRAMID_BASE + 1; i ++) {
		for (int j = 0; j < PYRAMID_BASE - i; j ++) {
			Entity* smilly;
			NewtonBody* smillyBody;

			// crate a visual Box and place in a pyramid formation above the floor
			smilly = sceneManager->CreateEntity();
			smilly->LoadMesh ("Smilly.dat");
			smilly->m_curPosition.m_x = 0.0f;
			smilly->m_curPosition.m_z = dFloat (j) * boxWidth + dFloat (i) * (boxWidth * 0.5f);
			smilly->m_curPosition.m_y = floorY + dFloat (i) * boxWidth;
			smilly->m_prevPosition = smilly->m_curPosition;

			// add a body with a box shape
			shape = CreateNewtonBox (world, smilly, 0);
			smillyBody = CreateRigidBody (world, smilly, shape, 10.0f);
			NewtonDestroyCollision(shape);

			// we want some nice object placement, with zero penetration and and zero jitter
			// therefore we are going use use a Convex Cast function to snap the Box to the closest contact surface
			ConvexCastPlacement (smillyBody);
		}
	}
}



