/* Copyright (c) <2003-2016> <Newton Game Dynamics>
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
#include "SkyBox.h"
#include "PhysicsUtils.h"
#include "TargaToOpenGl.h"
#include "DemoMesh.h"
#include "DemoEntityManager.h"
#include "DemoCamera.h"
#include "DebugDisplay.h"

#define PLAYER_MASS						80.0f
#define PLAYER_WALK_SPEED				4.0f
#define PLAYER_JUMP_SPEED				6.0f
#define PLAYER_THIRD_PERSON_VIEW_DIST	8.0f


class BasicPlayerControllerManager: public dCustomPlayerControllerManager
{
	public:
	BasicPlayerControllerManager (NewtonWorld* const world)
		:dCustomPlayerControllerManager (world)
	{
	}

	~BasicPlayerControllerManager ()
	{
	}

	dCustomPlayerController* CreatePlayer(const dMatrix& location, dFloat height, dFloat radius, dFloat mass)
	{
		// get the scene 
		DemoEntityManager* const scene = (DemoEntityManager*) NewtonWorldGetUserData(GetWorld());

		// set the play coordinate system
		dMatrix localAxis(dGetIdentityMatrix());

		//up is first vector
		localAxis[0] = dVector (0.0, 1.0f, 0.0f, 0.0f);
		// up is the second vector
		localAxis[1] = dVector (1.0, 0.0f, 0.0f, 0.0f);
		// size if the cross product
		localAxis[2] = localAxis[0].CrossProduct(localAxis[1]);

		// make a play controller with default values.
		dCustomPlayerController* const controller = CreatePlayerController(location, localAxis, mass, radius, height);

		// get body from player, and set some parameter
		NewtonBody* const body = controller->GetBody();

		// create the visual mesh from the player collision shape
		NewtonCollision* const collision = NewtonBodyGetCollision(body);
		DemoMesh* const geometry = new DemoMesh("player", scene->GetShaderCache(), collision, "smilli.tga", "smilli.tga", "smilli.tga");

		DemoEntity* const playerEntity = new DemoEntity(location, NULL);
		scene->Append(playerEntity);
		playerEntity->SetMesh(geometry, dGetIdentityMatrix());
		geometry->Release();

		// set the user data
		NewtonBodySetUserData(body, playerEntity);

		// set the transform callback
		NewtonBodySetTransformCallback(body, DemoEntity::TransformCallback);

		// save player model with the controller
		controller->SetUserData(playerEntity);

		return controller;
	}

	virtual int ProcessContacts (const dCustomPlayerController* const controller, NewtonWorldConvexCastReturnInfo* const contacts, int count) const 
	{
		count = dCustomPlayerControllerManager::ProcessContacts (controller, contacts, count); 
		return count;
	}

	// apply gravity 
	virtual void ApplyPlayerMove (dCustomPlayerController* const controller, dFloat timestep)
	{
		// calculate the gravity contribution to the velocity
		dVector gravityImpulse(0.0f, DEMO_GRAVITY * controller->GetMass() * timestep, 0.0f, 0.0f);

		dVector totalImpulse (controller->GetImpulse() + gravityImpulse);

		// set player linear and angular velocity
		//controller->SetPlayerVelocity (player->m_inputs.m_forwarSpeed, player->m_inputs.m_strafeSpeed, player->m_inputs.m_jumpSpeed, player->m_inputs.m_headinAngle, gravity, timestep);

		controller->SetImpulse(totalImpulse);
	}
};


void BasicPlayerController (DemoEntityManager* const scene)
{
	// load the sky box
	scene->CreateSkyBox();

	CreateLevelMesh (scene, "flatPlane.ngd", true);
	//CreateLevelMesh (scene, "playground.ngd", true);
	//CreateLevelMesh (scene, "castle.ngd", true);
	//CreateLevelMesh (scene, "sponza.ngd", true);
	//CreateLevelMesh (scene, "sibenik.ngd", true);

	NewtonWorld* const world = scene->GetNewton();


	// add an input Manage to manage the inputs and user interaction 
//	BasicPlayerInputManager* const inputManager = new BasicPlayerInputManager (scene);

	// create a character controller manager
	BasicPlayerControllerManager* const playerManager = new BasicPlayerControllerManager (world);

	// add main player
	dMatrix location (dGetIdentityMatrix());
	location.m_posit.m_x = -4.0f;
	location.m_posit.m_y = 5.0f;
	location.m_posit.m_z = 0.0f;

//	location.m_posit.m_x = 98.710999f;
//	location.m_posit.m_y =-0.96156919f; 
//	location.m_posit.m_z = 27.254711f;

	location.m_posit = FindFloor (scene->GetNewton(), location.m_posit, 10.0f);
	location.m_posit.m_y += 4.0f;
	dCustomPlayerController*  const player = playerManager->CreatePlayer(location, 1.9f, 0.5, 100.0f);

/*
	int defaultMaterialID = NewtonMaterialGetDefaultGroupID (scene->GetNewton());
	location.m_posit.m_x += 5.0f;
	dVector size (2.0f, 2.0f, 2.0f, 0.0f);

	int count = 0;
	dMatrix shapeOffsetMatrix (dGetIdentityMatrix());
	AddPrimitiveArray(scene, 100.0f, location.m_posit, size, count, count, 5.0f, _BOX_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix, 10.0f);
*/
	dVector origin (-10.0f, 2.0f, 0.0f, 0.0f);
	dQuaternion rot;
	scene->SetCameraMatrix(rot, origin);
}





