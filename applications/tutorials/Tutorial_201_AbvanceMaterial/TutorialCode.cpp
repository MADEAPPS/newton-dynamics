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
#include "TutorialCode.h"
#include "SoundManager.h"
#include "SceneManager.h"
#include "RigidBodyUtil.h"
#include "MaterialManager.h"
#include "CollisionShapeUtil.h"
#include "CollectInputAndUpdateCamera.h"


enum MatrialID 
{
	m_mutiMaterial = 0,
	m_bricks,
	m_grass, 
	m_wood,
	m_metal,
};

void CreateScene (NewtonWorld* world, SceneManager* sceneManager)
{
	Entity* floor;
	NewtonCollision* shape;
	NewtonBody* floorBody;
	void* materialManager;
	SoundManager* sndManager;
	PhysicsMaterialInteration matInterations;
	
	sndManager = sceneManager->GetSoundManager();

	// Create the material for this scene, and attach it to the Newton World
	materialManager = CreateMaterialManager (world, sndManager);

	// add the Material table
	matInterations.m_restitution = 0.6f;
	matInterations.m_staticFriction = 0.6f;
	matInterations.m_kineticFriction = 0.3f;
	matInterations.m_scrapingSound = NULL;

	matInterations.m_impactSound = sndManager->LoadSound ("metalMetal.wav");
	AddMaterilInteraction (materialManager, m_metal, m_metal, &matInterations);

	matInterations.m_impactSound = sndManager->LoadSound ("boxBox.wav");
	AddMaterilInteraction (materialManager, m_wood, m_wood, &matInterations);

	matInterations.m_impactSound = sndManager->LoadSound ("metalBox.wav");
	AddMaterilInteraction (materialManager, m_metal, m_wood, &matInterations);
	
	matInterations.m_impactSound = sndManager->LoadSound ("grass0.wav");
	AddMaterilInteraction (materialManager, m_wood, m_grass, &matInterations);

	matInterations.m_impactSound = sndManager->LoadSound ("boxHit.wav");
	AddMaterilInteraction (materialManager, m_wood, m_bricks, &matInterations);

	matInterations.m_impactSound = sndManager->LoadSound ("grass1.wav");
	AddMaterilInteraction (materialManager, m_metal, m_grass, &matInterations);
	AddMaterilInteraction (materialManager, m_grass, m_bricks, &matInterations);

	matInterations.m_impactSound = sndManager->LoadSound ("metal.wav");
	AddMaterilInteraction (materialManager, m_metal, m_bricks, &matInterations);
	AddMaterilInteraction (materialManager, m_grass, m_grass, &matInterations);
	
	

	// Create a large body to be the floor
	floor = sceneManager->CreateEntity();
	floor->LoadMesh ("LevelMesh.dat");

	// add static floor Physics
	int materialMap[] = {m_bricks, m_grass, m_wood,	m_metal};
	shape = CreateMeshCollision (world, floor, materialMap);
	floorBody = CreateRigidBody (world, floor, shape, 0.0f);
	NewtonDestroyCollision(shape);


	// set the Transformation Matrix for this rigid body
	dMatrix matrix (floor->m_curRotation, floor->m_curPosition);
	NewtonBodySetMatrix (floorBody, &matrix[0][0]);

	// add some visual entities.
	dFloat y0 = FindFloor (world, 0.0f, 0.0f) + 10.0f;
	for (int i = 0; i < 5; i ++) {
		Entity* smilly;
		NewtonBody* smillyBody;
		smilly = sceneManager->CreateEntity();
		smilly->LoadMesh ("Smilly.dat");
		smilly->m_curPosition.m_y = y0;
		y0 += 2.0f;
		smilly->m_prevPosition = smilly->m_curPosition;

		// add a body with a box shape
		shape = CreateNewtonBox (world, smilly, m_metal);
		smillyBody = CreateRigidBody (world, smilly, shape, 10.0f);
		NewtonDestroyCollision(shape);
	}

	// add some visual entities.
	y0 = FindFloor (world, 0.0f, 0.4f) + 10.5f;
	for (int i = 0; i < 5; i ++) {
		Entity* frowny;
		NewtonBody* frownyBody;
		frowny = sceneManager->CreateEntity();
		frowny->LoadMesh ("Frowny.dat");
		frowny->m_curPosition.m_z = 0.4f;
		frowny->m_curPosition.m_y = y0;
		y0 += 2.0f;
		frowny->m_prevPosition = frowny->m_curPosition;

		// add a body with a Convex hull shape
		shape = CreateNewtonConvex (world, frowny, m_wood);
		frownyBody = CreateRigidBody (world, frowny, shape, 10.0f);
		NewtonDestroyCollision(shape);
	}

	y0 = FindFloor (world, 0.0f, 2.0f) + 10.5f;
	for (int i = 0; i < 5; i ++) {
		Entity* frowny;
		NewtonBody* frownyBody;

		frowny = sceneManager->CreateEntity();
		frowny->LoadMesh ("dumb.dat");
		frowny->m_curPosition.m_z = 2.0f;
		frowny->m_curPosition.m_y = y0;
		y0 += 2.0f;
		frowny->m_prevPosition = frowny->m_curPosition;

		// add a body with a Convex hull shape
		int materialMap[] = {m_grass, m_wood, m_metal, m_metal};
		shape = CreateNewtonCompoundFromEntitySubmesh (world, frowny, materialMap);
		frownyBody = CreateRigidBody (world, frowny, shape, 10.0f);
		NewtonDestroyCollision(shape);
	}


	// set the Camera EyePoint close to the scene action
	InitCamera (dVector (-15.0f, FindFloor (world, -15.0f, 0.0f) + 5.0f, 0.0f), dVector (1.0f, 0.0f, 0.0f));
}



