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
#include "CollisionShapeUtil.h"
#include "CreateHeightFieldEntity.h"
#include "CollectInputAndUpdateCamera.h"

#pragma warning (disable: 4100) //unreferenced formal parameter

int g_floorMaterial;
int g_woodMaterial;
int g_metalMaterial;
static void CreateMateials (NewtonWorld* world, SceneManager* sceneManager);

struct SoundEffect
{
	void *m_sound;
	SoundManager* m_manager;
};	

SoundEffect woodOnWood;
SoundEffect woodOnFloor;
SoundEffect woodOnMetal;
SoundEffect metalOnMetal;
SoundEffect metalOnFloor;

void CreateScene (NewtonWorld* world, SceneManager* sceneManager)
{
	Entity* floor;
	Entity* smilly;
	Entity* frowny;
	NewtonBody* floorBody;
	NewtonBody* smillyBody;
	NewtonBody* frownyBody;
	NewtonCollision* shape;

	// Create the material for this scene
	CreateMateials (world, sceneManager);

	// Create a large body to be the floor
	floor = sceneManager->CreateEntity();

	// add scene collision from a level mesh
	shape = CreateHeightFieldCollision (world, "h2.raw", 0);
	floorBody = CreateRigidBody (world, floor, shape, 0.0f);
	NewtonDestroyCollision(shape);

	// Get teh shape from the body
	shape = NewtonBodyGetCollision(floorBody);

	// make a visual mesh for the collision data
	CreateHeightFieldMesh (NewtonBodyGetCollision(floorBody), floor);

	// set the matrix at the origin
	dVector boxP0; 
	dVector boxP1; 
	dMatrix matrix (floor->m_curRotation, floor->m_curPosition);
	NewtonCollisionCalculateAABB (shape, &matrix[0][0], &boxP0.m_x, &boxP1.m_x); 

	// place the origin of the visual mesh at the center of the height field
	matrix.m_posit = (boxP0 + boxP1).Scale (-0.5f);
	matrix.m_posit.m_w = 1.0f;
	floor->m_curPosition = matrix.m_posit;
	floor->m_prevPosition = matrix.m_posit;

	// relocate the body;
	NewtonBodySetMatrix (floorBody, &matrix[0][0]);


	// assign an Material ID to this body
	NewtonBodySetMaterialGroupID (floorBody, g_floorMaterial);


	// add some visual entities.
	dFloat y0 = FindFloor (world, 0.0f, 0.0f) + 10.0f;
	for (int i = 0; i < 5; i ++) {
		smilly = sceneManager->CreateEntity();
		smilly->LoadMesh ("Smilly.dat");
		smilly->m_curPosition.m_y = y0;
		y0 += 2.0f;
		smilly->m_prevPosition = smilly->m_curPosition;

		// add a body with a box shape
		shape = CreateNewtonBox (world, smilly, 0);
		smillyBody = CreateRigidBody (world, smilly, shape, 10.0f);
		NewtonDestroyCollision(shape);

		// assign an Material ID to this body
		NewtonBodySetMaterialGroupID (smillyBody, g_metalMaterial);
	}


	// add some visual entities.
	y0 = FindFloor (world, 0.0f, 0.4f) + 10.5f;
	for (int i = 0; i < 5; i ++) {
		frowny = sceneManager->CreateEntity();
		frowny->LoadMesh ("Frowny.dat");
		frowny->m_curPosition.m_z = 0.4f;
		frowny->m_curPosition.m_y = y0;
		y0 += 2.0f;
		frowny->m_prevPosition = frowny->m_curPosition;

		// add a body with a Convex hull shape
		shape = CreateNewtonConvex (world, frowny, 0);
		frownyBody = CreateRigidBody (world, frowny, shape, 10.0f);
		NewtonDestroyCollision(shape);

		// assign an Material ID to this body
		NewtonBodySetMaterialGroupID (frownyBody, g_woodMaterial);
	}

	// set the Camera EyePoint close to the scene action
	InitCamera (dVector (-15.0f, FindFloor (world, -15.0f, 0.0f) + 5.0f, 0.0f), dVector (1.0f, 0.0f, 0.0f));
}


static void GenericContactProcess (const NewtonJoint* contactJoint, dFloat timestep, int threadIndex)
{
	dFloat contactBestSpeed;
	dVector contactPosit;
	SoundEffect* bestSound;

	bestSound = NULL;
	contactBestSpeed = 0.5f;
	NewtonBody* const body0 = NewtonJointGetBody0(contactJoint);
	for (void* contact = NewtonContactJointGetFirstContact (contactJoint); contact; contact = NewtonContactJointGetNextContact (contactJoint, contact)) {
		dFloat contactNormalSpeed;
		NewtonMaterial* material;

		// get the material for this contact;
		material = NewtonContactGetMaterial (contact);

		contactNormalSpeed = NewtonMaterialGetContactNormalSpeed (material);
		if (contactNormalSpeed > contactBestSpeed){
			dVector normal;
			contactBestSpeed = contactNormalSpeed;
			NewtonMaterialGetContactPositionAndNormal (material, body0, &contactPosit[0], &normal[0]);
			bestSound = (SoundEffect *)NewtonMaterialGetMaterialPairUserData (material);
		}
	}

	// now that we found we can play then
	if (bestSound) {
		// calculate the volume;
		dFloat volume;
		dFloat dist2;

		dVector eyePoint (GetCameraEyePoint() - contactPosit);
		dist2 = eyePoint % eyePoint;
		if (dist2 < (MAX_SOUND_DISTANCE * MAX_SOUND_DISTANCE)) {
			volume = 1.0f;
			if (dist2 > (MIN_SOUND_DISTANCE * MIN_SOUND_DISTANCE)) {
				volume = 1.0f - (dSqrt (dist2) - MIN_SOUND_DISTANCE) / (MAX_SOUND_DISTANCE -  MIN_SOUND_DISTANCE);
			}
			bestSound->m_manager->Play(bestSound->m_sound, volume, 0);
		}
	}
}


void CreateMateials (NewtonWorld* world, SceneManager* sceneManager)
{
	SoundManager* sndManager;
		
		
	sndManager = sceneManager->GetSoundManager();

	// create the Material IDs, 
	g_floorMaterial = NewtonMaterialCreateGroupID (world);
	g_woodMaterial = NewtonMaterialCreateGroupID (world);
	g_metalMaterial = NewtonMaterialCreateGroupID (world);

	// load the sound effects 
	woodOnFloor.m_sound = sndManager->LoadSound ("boxHit.wav");
	woodOnFloor.m_manager = sndManager;

	metalOnFloor.m_sound = sndManager->LoadSound ("metal.wav");
	metalOnFloor.m_manager = sndManager;

	woodOnMetal.m_sound = sndManager->LoadSound ("metalBox.wav");
	woodOnMetal.m_manager = sndManager;
	
	metalOnMetal.m_sound = sndManager->LoadSound ("metalMetal.wav");
	metalOnMetal.m_manager = sndManager;

	woodOnWood.m_sound = sndManager->LoadSound ("boxBox.wav");
	woodOnWood.m_manager = sndManager;


	//configure the Material interactions
	NewtonMaterialSetCollisionCallback (world, g_woodMaterial, g_floorMaterial, &woodOnFloor, NULL,  GenericContactProcess);
	NewtonMaterialSetCollisionCallback (world, g_metalMaterial, g_floorMaterial, &metalOnFloor, NULL,  GenericContactProcess);
	NewtonMaterialSetCollisionCallback (world, g_metalMaterial, g_woodMaterial, &woodOnMetal, NULL,  GenericContactProcess);

	NewtonMaterialSetCollisionCallback (world, g_woodMaterial, g_woodMaterial, &woodOnWood, NULL,  GenericContactProcess);
	NewtonMaterialSetCollisionCallback (world, g_metalMaterial, g_metalMaterial, &metalOnMetal, NULL,  GenericContactProcess);

}
