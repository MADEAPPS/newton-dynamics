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
#include "OpenGlUtil.h"
#include "TutorialCode.h"
#include "SoundManager.h"
#include "SceneManager.h"
#include "RigidBodyUtil.h"
#include "MaterialManager.h"
#include "CollisionShapeUtil.h"
#include "CreateHeightFieldEntity.h"
#include "CollectInputAndUpdateCamera.h"


enum MatrialID 
{
	m_mutiMaterial = 0,
	m_bricks,
	m_grass, 
	m_wood,
	m_metal,
};

//#define USE_HEIGHT_FIELD_LEVEL

#define PLAYER_SPEED		10.0f
#define PLAYER_SIDE_SPEED	 5.0f
#define PLAYER_JOINT_ID		0xEF38AB01
#define PLAYER_MAX_SLOPE	30.0f

struct PlayerController
{
	// destructor to be call when the joint is destroyed
	static void Destroy (const NewtonUserJoint* me)
	{
		PlayerController* path;

		path = (PlayerController*) CustomGetUserData(me);
		free (path);
	}

	// apply desire user control
	static void ApplyPlayerInput (const NewtonUserJoint* me, dFloat timestep, int threadIndex)
	{
		dFloat velocity;
		dFloat strafeVeloc;
		dFloat headinAngle;
		PlayerController* controller;

		controller = (PlayerController*) CustomGetUserData(me);

		velocity = 0;
		if (IsKeyDown ('W')) {
			velocity = PLAYER_SPEED;
		} else if (IsKeyDown  ('S')) {
			velocity = -PLAYER_SPEED;
		}

		strafeVeloc = 0.0f;
		if (IsKeyDown  ('D')) {
			strafeVeloc = PLAYER_SIDE_SPEED;
		} else if (IsKeyDown  ('A')) {
			strafeVeloc = -PLAYER_SIDE_SPEED;
		}

		// now set the desired player velocity and heading 
		headinAngle = GetCameraYawAngle ();

		// prevent player fro running faster when strafing and moving forward as the same time
		dFloat mag2 = velocity * velocity + strafeVeloc * strafeVeloc;
		if (mag2 > PLAYER_SPEED * PLAYER_SPEED) {
			mag2 = PLAYER_SPEED * dSqrt (1.0f / mag2);
			velocity *= mag2;
			strafeVeloc *= mag2;
		}	

		CustomPlayerControllerSetVelocity (me, velocity, strafeVeloc, headinAngle);
	}

	
	static void SetTransform (const NewtonBody* body, const dFloat* matrix, int threadId)
	{
		NewtonUserJoint* player;
		PlayerController* controller;

		// find the player joint;
		player = NULL;
		for (NewtonJoint* joint = NewtonBodyGetFirstJoint(body); joint; joint = NewtonBodyGetNextJoint(body, joint)) {
			NewtonUserJoint* tmp;
			tmp = (NewtonUserJoint*) NewtonJointGetUserData(joint);
			if (CustomGetJointID (tmp) == PLAYER_JOINT_ID) {
				player = tmp;
				break;
			}
		}

		// call the generic transform callback
		controller = (PlayerController*) CustomGetUserData(player);

#if 1
		// this will project the visual mesh to the ground
		dMatrix visualMatrix;
		CustomPlayerControllerGetVisualMaTrix (player, &visualMatrix[0][0]);
#else 
		// this will display the player at the collision shape position
		const dMatrix& visualMatrix = *((dMatrix*) matrix);
#endif

		controller->m_setTransformOriginal (body, &visualMatrix[0][0], threadId);
		
		// now we will set the camera to follow the player
		dVector eyePoint (visualMatrix.TransformVector(controller->m_point));
		
		// check if the player wants third person view
		static int prevCKeyDown = IsKeyDown  ('C');
		int isCkeyDwon = IsKeyDown  ('C');
		if (isCkeyDwon && !prevCKeyDown) {
			controller->m_isThirdView = !controller->m_isThirdView;
		}
		prevCKeyDown = isCkeyDwon;

		if (controller->m_isThirdView) {
			dVector dir (GetCameraDir ());
			eyePoint -= dir.Scale (8.0f);
		}

		SetCameraEyePoint (eyePoint);

//		NewtonBodyGetMatrix (body, &matrix[0][0]);
//		cameraEyepoint = matrix.m_posit;
//		cameraEyepoint.m_y += 1.0f;
	}


	int m_isThirdView;
	dVector m_point;
	void (*m_setTransformOriginal) (const NewtonBody* body, const dFloat* matrix, int threadIndex);

};




void CreateScene (NewtonWorld* world, SceneManager* sceneManager)
{
	Entity* floor;
	NewtonBody* floorBody;
	NewtonCollision* shape;

/*
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

	matInterations.m_impactSound = sndManager->LoadSound ("metal.wav");
	AddMaterilInteraction (materialManager, m_metal, m_bricks, &matInterations);
*/


	// Create a large body to be the floor
	floor = sceneManager->CreateEntity();
	int materialMap[] = {m_bricks, m_grass, m_wood,	m_metal};

#ifdef USE_HEIGHT_FIELD_LEVEL

	// add scene collision from a level m*esh
	shape = CreateHeightFieldCollision (world, "h2.raw", materialMap);
	floorBody = CreateRigidBody (world, floor, shape, 0.0f);
	NewtonDestroyCollision(shape);

	// make a visual mesh for the collision data
	CreateHeightFieldMesh (shape, floor);

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

#else

	floor->LoadMesh ("LevelMesh.dat");

	// add static floor Physics

	shape = CreateMeshCollision (world, floor, materialMap);
	floorBody = CreateRigidBody (world, floor, shape, 0.0f);
	NewtonDestroyCollision(shape);

	// set the Transformation Matrix for this rigid body
	dMatrix matrix (floor->m_curRotation, floor->m_curPosition);
	NewtonBodySetMatrix (floorBody, &matrix[0][0]);

#endif

	// now we will use the properties of this body to set a proper world size.
	dVector minBox;
	dVector maxBox;
	NewtonCollisionCalculateAABB (shape, &matrix[0][0], &minBox[0], &maxBox[0]);

	// add some extra padding
	minBox.m_x -=  50.0f;
	minBox.m_y -= 500.0f;
	minBox.m_z -=  50.0f;

	maxBox.m_x +=  50.0f;
	maxBox.m_y += 500.0f;
	maxBox.m_z +=  50.0f;

	// set the new world size
	NewtonSetWorldSize (world, &minBox[0], &maxBox[0]);


	// Create a Body and attach a player controller joint
	{
		dFloat y0;
		Entity* player;
		NewtonBody* playerBody;
		NewtonCollision* shape;

		// find  the a floor to place the player 
		y0 = FindFloor (world, 0.0f, 0.0f) + 1.0f;

		// load the player mesh
		player = sceneManager->CreateEntity();
		player->LoadMesh ("gymnast.dat");
		player->m_curPosition.m_y = y0;
		player->m_prevPosition = player->m_curPosition;

		// get the bounding Box of the player to get the collision shape dimensions
		dVector minBox;
		dVector maxBox;
		player->GetBBox (minBox, maxBox);

		// calculate player high and width
		dFloat padding = 1.0f / 64.0f;  // this si the default padding, for teh palye joint, we must subtract it from the shape 
		dFloat playerHigh = (maxBox.m_y - minBox.m_y) - padding;
		dFloat playerRadius0 = (maxBox.m_z - minBox.m_z) * 0.5f;
		dFloat playerRadius1 = (maxBox.m_x - minBox.m_x) * 0.5f;
		dFloat playerRadius = (playerRadius0 > playerRadius1 ? playerRadius0 : playerRadius1) - padding;

		// No we make and make a upright capsule for the collision mesh
		dMatrix orientation;
		orientation.m_front = dVector (0.0f, 1.0f, 0.0f, 0.0f);			// this is the player up direction
		orientation.m_up    = dVector (1.0f, 0.0f, 0.0f, 0.0f);			// this is the player front direction
		orientation.m_right = orientation.m_front * orientation.m_up;   // this is the player sideway direction
		orientation.m_posit = dVector (0.0f, 0.0f, 0.0f, 1.0f);

		// add a body with a box shape
		//shape = CreateNewtonCapsule (world, player, playerHigh, playerRadius, m_wood, orientation);
		shape = CreateNewtonCylinder (world, player, playerHigh, playerRadius, m_wood, orientation);
		playerBody = CreateRigidBody (world, player, shape, 10.0f);
		NewtonDestroyCollision(shape);

		// make sure the player does not go to sleep
		NewtonBodySetAutoSleep (playerBody, 0);

		// now we will attach a player controller to the body
		NewtonUserJoint* playerController;
		// the player can take step up to 0.7 units;
		dFloat maxStairStepFactor = 0.7f / playerHigh;
		playerController = CreateCustomPlayerController (&orientation[0][0], playerBody, maxStairStepFactor, padding);

		// set the Max Slope the player can climb to PLAYER_MAX_SLOPE degree
		CustomPlayerControllerSetMaxSlope (playerController, PLAYER_MAX_SLOPE * 3.1416f / 180.0f);


		// now we will append some application data for the application to control the player
		PlayerController* userControl = (PlayerController*) malloc (sizeof (PlayerController));
		userControl->m_isThirdView = 1;
		userControl->m_point = dVector (0.0f, playerHigh, 0.0f,0.0f);

		// set the user data for the application to control the player
		CustomSetUserData (playerController, userControl);

		// set the destruction call back so that the application can destroy local used data
		CustomSetDestructorCallback (playerController, PlayerController::Destroy);

		// set a call back to control the player
		CustomSetSubmitContraintCallback (playerController, PlayerController::ApplyPlayerInput);

		// we also need to set override the transform call back so the we can set the Camera
		userControl->m_setTransformOriginal = NewtonBodyGetTransformCallback(playerBody);
		NewtonBodySetTransformCallback (playerBody, PlayerController::SetTransform);

		// we will need some ID to fin this joint in the transform Callback
		CustomSetJointID (playerController, PLAYER_JOINT_ID);
	}

/*
	{
		// add some visual entities.
		dFloat y0;
		y0 = FindFloor (world, 0.0f, 0.4f) + 10.5f;
		for (int i = 0; i < 5; i ++) {
			Entity* frowny;
			NewtonBody* frownyBody;
			NewtonCollision* shape;

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
	}
*/
	// set the Camera EyePoint close to the scene action
	SetCameraEyePoint (dVector (-15.0f, FindFloor (world, -15.0f, 0.0f) + 5.0f, 0.0f));
}



