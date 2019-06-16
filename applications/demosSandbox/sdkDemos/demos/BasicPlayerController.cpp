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
#include "HeightFieldPrimitive.h"

#define PLAYER_MASS						80.0f
#define PLAYER_WALK_SPEED				8.0f
#define PLAYER_JUMP_SPEED				6.0f
#define PLAYER_THIRD_PERSON_VIEW_DIST	8.0f


class BasicPlayerControllerManager: public dCustomPlayerControllerManager
{
	public:
	BasicPlayerControllerManager (NewtonWorld* const world)
		:dCustomPlayerControllerManager (world)
		,m_player(NULL)
	{
		DemoEntityManager* const scene = (DemoEntityManager*) NewtonWorldGetUserData(GetWorld());

		scene->SetUpdateCameraFunction(UpdateCameraCallback, this);
		scene->Set2DDisplayRenderFunction (RenderPlayerHelp, NULL, this);
	}

	~BasicPlayerControllerManager ()
	{
	}

	void SetAsPlayer(dCustomPlayerController* const controller)
	{
		m_player = controller;
	}

	void RenderPlayerHelp(DemoEntityManager* const scene) const
	{
		dVector color(1.0f, 1.0f, 0.0f, 0.0f);
		scene->Print(color, "Navigation Keys");
		scene->Print(color, "walk forward:            W");
		scene->Print(color, "walk backward:           S");
		scene->Print(color, "strafe right:            D");
		scene->Print(color, "strafe left:             A");
		//scene->Print(color, "toggle camera mode:      C");
		//scene->Print(color, "jump:                    Space");
		//scene->Print(color, "hide help:               H");
		//scene->Print(color, "change player direction: Left mouse button");
	}

	static void RenderPlayerHelp(DemoEntityManager* const scene, void* const context)
	{
		BasicPlayerControllerManager* const me = (BasicPlayerControllerManager*)context;
		me->RenderPlayerHelp(scene);
	}

	static void UpdateCameraCallback(DemoEntityManager* const manager, void* const context, dFloat timestep)
	{
		BasicPlayerControllerManager* const me = (BasicPlayerControllerManager*)context;
		me->SetCamera();
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
		dCustomPlayerController* const controller = CreateController(location, localAxis, mass, radius, height, height / 3.0f);

		// Test Local Matrix manipulations
		//controller->SetFrame(dRollMatrix(60.0f * dDegreeToRad) * controller->GetFrame());

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

	void SetCamera ()
	{
		if (m_player) {
			DemoEntityManager* const scene = (DemoEntityManager*)NewtonWorldGetUserData(GetWorld());
			DemoCamera* const camera = scene->GetCamera();
			dMatrix camMatrix(camera->GetNextMatrix());

			DemoEntity* player = (DemoEntity*)NewtonBodyGetUserData(m_player->GetBody());
			dMatrix playerMatrix(player->GetNextMatrix());

			dFloat height = 2.0f;
			dVector frontDir(camMatrix[0]);
			dVector upDir(0.0f, 1.0f, 0.0f, 0.0f);
			dVector camOrigin = playerMatrix.TransformVector(upDir.Scale(height));
			camOrigin -= frontDir.Scale(PLAYER_THIRD_PERSON_VIEW_DIST);

			camera->SetNextMatrix(*scene, camMatrix, camOrigin);
		}
	}

	void ApplyInputs (dCustomPlayerController* const controller)
	{
		if (controller == m_player) {
			DemoEntityManager* const scene = (DemoEntityManager*)NewtonWorldGetUserData(GetWorld());
			dFloat forwarSpeed = (int(scene->GetKeyState('W')) - int(scene->GetKeyState('S'))) * PLAYER_WALK_SPEED;
			dFloat strafeSpeed = (int(scene->GetKeyState('D')) - int(scene->GetKeyState('A'))) * PLAYER_WALK_SPEED;

			if (forwarSpeed && strafeSpeed) {
				dFloat invMag = PLAYER_WALK_SPEED / dSqrt(forwarSpeed * forwarSpeed + strafeSpeed * strafeSpeed);
				forwarSpeed *= invMag;
				strafeSpeed *= invMag;
			}

			DemoCamera* const camera = scene->GetCamera();
			dMatrix camMatrix(camera->GetNextMatrix());

			controller->SetForwardSpeed(forwarSpeed);
			controller->SetLateralSpeed(strafeSpeed);
			controller->SetHeadingAngle(camera->GetYawAngle());
		}
	}

	bool ProccessContact(dCustomPlayerController* const controller, const dVector& position, const dVector& normal, const NewtonBody* const otherbody) const
	{
/*
		if (normal.m_y < 0.9f) {
			dMatrix matrix;
			NewtonBodyGetMatrix(controller->GetBody(), &matrix[0][0]);
			dFloat h = (position - matrix.m_posit).DotProduct3(matrix.m_up);
			return (h >= m_stepHigh) ? true : false;
		} 
*/
		return true;
	}

	dFloat ContactFriction(dCustomPlayerController* const controller, const dVector& position, const dVector& normal, int contactId, const NewtonBody* const otherbody) const
	{ 
		if (normal.m_y < 0.9f) {
			// steep slope are friction less
			return 0.0f;
		} else {
			//NewtonCollision* const collision = NewtonBodyGetCollision(otherbody);
			//int type = NewtonCollisionGetType (collision);
			//if ((type == SERIALIZE_ID_TREE) || (type == SERIALIZE_ID_TREE)) {
			//} else {
			switch (contactId)
			{
				case 1:
					// this the brick wall
					return 0.5f;
				case 2:
					// this the wood floor
					return 1.0f;
				case 3:
					// this the cement floor
					return 2.0f;
					//return 0.2f;
				default: 
					// this is everything else
					return 1.0f;
			}
		}
	}
	
	// apply gravity 
	virtual void ApplyMove (dCustomPlayerController* const controller, dFloat timestep)
	{
		// calculate the gravity contribution to the velocity
		dVector gravity(controller->GetFrame().RotateVector(dVector(DEMO_GRAVITY, 0.0f, 0.0f, 0.0f)));
		dVector totalImpulse(controller->GetImpulse() + gravity.Scale (controller->GetMass() * timestep));
		controller->SetImpulse(totalImpulse);

		// apply play movement
		ApplyInputs (controller);
	}

	dCustomPlayerController* m_player;
};


void BasicPlayerController (DemoEntityManager* const scene)
{
	// load the sky box
	scene->CreateSkyBox();

	//CreateLevelMesh (scene, "flatPlane.ngd", true);
	CreateLevelMesh (scene, "playerarena.ngd", true);
	//CreateHeightFieldTerrain(scene, 10, 2.0f, 1.5f, 0.3f, 200.0f, -50.0f);

	NewtonWorld* const world = scene->GetNewton();

	// create a character controller manager
	BasicPlayerControllerManager* const playerManager = new BasicPlayerControllerManager (world);

	// add main player
	dMatrix location (dGetIdentityMatrix());
	location.m_posit.m_x = 42.0f;
	location.m_posit.m_y = 5.0f;
	location.m_posit.m_z = -24.0f;

	location.m_posit.m_y = 15.0f;

	location.m_posit = FindFloor (scene->GetNewton(), location.m_posit, 20.0f);
	location.m_posit.m_y += 1.0f;
	dCustomPlayerController*  const player = playerManager->CreatePlayer(location, 1.9f, 0.5, 100.0f);
	playerManager->SetAsPlayer(player);

	int defaultMaterialID = NewtonMaterialGetDefaultGroupID (scene->GetNewton());
	location.m_posit.m_x += 5.0f;

	int count = 1;
	dMatrix shapeOffsetMatrix (dGetIdentityMatrix());
//	AddPrimitiveArray(scene, 100.0f, location.m_posit, dVector (2.0f, 2.0f, 2.0f, 0.0f), count, count, 5.0f, _BOX_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix, 10.0f);

	location.m_posit.m_x -= 10.0f;
	AddPrimitiveArray(scene, 100.0f, location.m_posit, dVector (2.0f, 0.5f, 2.0f, 0.0f), count, count, 5.0f, _BOX_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix, 10.0f);

	dVector origin (-10.0f, 2.0f, 0.0f, 0.0f);
	dQuaternion rot;
	scene->SetCameraMatrix(rot, origin);
}

