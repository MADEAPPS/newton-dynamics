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


#include <toolbox_stdafx.h>
#include "SkyBox.h"
#include "NewtonDemos.h"
#include "PhysicsUtils.h"
#include "TargaToOpenGl.h"
#include "DemoMesh.h"
#include "DemoEntityManager.h"
#include "DemoCamera.h"
#include "DebugDisplay.h"
#include "CustomPlayerControllerManager.h"

#define PLAYER_MASS						80.0f
#define PLAYER_WALK_SPEED				4.0f
#define PLAYER_THIRD_PERSON_VIEW_DIST	8.0f

class BasicPlayerEntity: public DemoEntity
{
	public: 
	BasicPlayerEntity (DemoEntityManager* const scene, CustomPlayerControllerManager* const manager, dFloat radius, dFloat height, const dMatrix& location)
		:DemoEntity (GetIdentityMatrix(), NULL)
		,m_isPlayer(false)
		,m_skipRender(true)
		,m_helpKey(true)
		,m_jumpKey(false)
		,m_headinAngle(0.0f)
		,m_forwarSpeed(0.0f)
		,m_strafeSpeed(0.0f)
		,m_jumpSpeed(0.0f)
		,m_shootProp(0)
	{
		// add this entity to the scene for rendering
		scene->Append(this);

		// now make a simple player controller, 
		dMatrix playerAxis; 
		playerAxis[0] = dVector (0.0f, 1.0f, 0.0f, 0.0f); // the y axis is the character up vector
		playerAxis[1] = dVector (1.0f, 0.0f, 0.0f, 0.0f); // the x axis is the character front direction
		playerAxis[2] = playerAxis[0] * playerAxis[1];
		playerAxis[3] = dVector (0.0f, 0.0f, 0.0f, 1.0f);

		// make the player controller, this function makes a kinematic body
		m_controller = manager->CreatePlayer(PLAYER_MASS, radius, radius * 0.5f, height, height * 0.33f, playerAxis);

		// set a restraining distance that the player can not get closet than
		m_controller->SetRestrainingDistance(0.1f);


		// players by default have the origin at the center of the lower bottom of the collision shape.
		// you can change this by calling SetPlayerOrigin, for example if a player has it origin at the center of the AABB you can call 
		//m_controller->SetPlayerOrigin (height * 0.5f);

		// get body from player, and set some parameter
		NewtonBody* const body = m_controller->GetBody();

		// set the user data
		NewtonBodySetUserData(body, this);

		// set the transform callback
		NewtonBodySetTransformCallback (body, DemoEntity::TransformCallback);

		// set the player matrix 
		NewtonBodySetMatrix(body, &location[0][0]);

		// create the visual mesh from the player collision shape
		NewtonCollision* const collision = NewtonBodyGetCollision(body);
		DemoMesh* const geometry = new DemoMesh("player", collision, "smilli.tga", "smilli.tga", "smilli.tga");
		SetMesh(geometry);
		geometry->Release(); 
	}

	~BasicPlayerEntity ()
	{
		// destroy the player controller and its rigid body
		m_controller->GetManager()->DestroyController(m_controller);
	}

	virtual void Render(dFloat timeStep) const
	{
		if (!(m_skipRender & m_isPlayer)) {
			// render only when external view mode
			DemoEntity::Render(timeStep);
		}
	}


	bool m_isPlayer;
	bool m_skipRender;
	DemoEntityManager::ButtonKey m_helpKey;
	DemoEntityManager::ButtonKey m_jumpKey;
	dFloat m_headinAngle;
	dFloat m_forwarSpeed;
	dFloat m_strafeSpeed;
	dFloat m_jumpSpeed;
	int m_shootProp;

	CustomPlayerControllerManager::CustomController* m_controller; 
};

class BasicPlayerControllerManager: public CustomPlayerControllerManager
{
	public:
	BasicPlayerControllerManager (NewtonWorld* const world)
		:CustomPlayerControllerManager (world)
		,m_player (NULL) 
		,m_shotProp(true)
		,m_cameraMode(false)
	{
		// hook a callback for 2d help display
		DemoEntityManager* const scene = (DemoEntityManager*) NewtonWorldGetUserData(world);
		scene->Set2DDisplayRenderFunction (RenderPlayerHelp, this);
	}

	~BasicPlayerControllerManager ()
	{
	}

	static void RenderPlayerHelp (DemoEntityManager* const scene, void* const context)
	{
		BasicPlayerControllerManager* const me = (BasicPlayerControllerManager*) context;
		me->RenderPlayerHelp (scene);
	}


	void SetAsPlayer (BasicPlayerEntity* const player)
	{
		if (m_player) {
			m_player->m_isPlayer = false;
		}

		m_player = player;
		player->m_isPlayer = true;
	}

	void RenderPlayerHelp (DemoEntityManager* const scene) const
	{
		if (m_player->m_helpKey.GetPushButtonState()) {
			dVector color(1.0f, 1.0f, 0.0f, 0.0f);
			scene->Print (color, 10, 200, "Navigation               Key");
			scene->Print (color, 10, 220, "walk forward:            W");
			scene->Print (color, 10, 240, "walk backward:           S");
			scene->Print (color, 10, 260, "strafe right:            D");
			scene->Print (color, 10, 280, "strafe left:             A");
			scene->Print (color, 10, 300, "toggle camera mode:      C");
			scene->Print (color, 10, 320, "jump:                    Space");
			scene->Print (color, 10, 340, "hide help:               H");
			scene->Print (color, 10, 360, "change player direction: Left mouse button");
		}
	}


	void PlayerInputs (const CustomPlayerController* const controller, dFloat timestep)
	{
		DemoEntityManager* const scene = (DemoEntityManager*) NewtonWorldGetUserData(GetWorld());

		DemoCamera* const camera = scene->GetCamera();

		NewtonDemos* const mainWindow = scene->GetRootWindow();
		NewtonBody* const body = controller->GetBody();
		BasicPlayerEntity* const player = (BasicPlayerEntity*) NewtonBodyGetUserData(body);

		player->m_headinAngle = camera->GetYawAngle();
		dFloat forwarSpeed = (int (mainWindow->GetKeyState ('W')) - int (mainWindow->GetKeyState ('S'))) * PLAYER_WALK_SPEED;
		dFloat strafeSpeed = (int (mainWindow->GetKeyState ('D')) - int (mainWindow->GetKeyState ('A'))) * PLAYER_WALK_SPEED;

		// normalize player speed
		dFloat mag2 = forwarSpeed * forwarSpeed + strafeSpeed * strafeSpeed;
		if (mag2 > 0.0f) {
			dFloat invMag = PLAYER_WALK_SPEED / dSqrt (mag2);
			forwarSpeed *= invMag;
			strafeSpeed *= invMag;
		}
		m_player->m_forwarSpeed = forwarSpeed;
		m_player->m_strafeSpeed = strafeSpeed;

		// set player is jumping speed
		const dFloat jumpSpeed = 8.0f;
		m_player->m_jumpSpeed = (m_player->m_jumpKey.UpdateTriggerButton(mainWindow, ' ')) ? jumpSpeed : 0.0f;

		// see if we are shooting
		m_player->m_shootProp = m_shotProp.UpdateTriggerButton(mainWindow, 0x0d) ? 1 : 0;

		// set the help key
		m_player->m_helpKey.UpdatePushButton (mainWindow, 'H');
	}

	// apply gravity 
	virtual void ApplyPlayerMove (CustomPlayerController* const controller, dFloat timestep)
	{
		NewtonBody* const body = controller->GetBody();

		BasicPlayerEntity* const player = (BasicPlayerEntity*) NewtonBodyGetUserData(body);
		if (player == m_player) {
			// velocity set by human player, get input from player camera
			PlayerInputs (controller, timestep);
		} else {
			// velocity is set by AI script
		}

#if 0
	#if 0
		static FILE* file = fopen ("log.bin", "wb");
		if (file) {
			fwrite (&player->m_headinAngle, sizeof (dFloat), 1, file);
			fwrite (&player->m_forwarSpeed, sizeof (dFloat), 1, file);
			fwrite (&player->m_strafeSpeed, sizeof (dFloat), 1, file);
			fwrite (&player->m_jumpSpeed, sizeof (dFloat), 1, file);
			fwrite (&player->m_shootProp, sizeof (int), 1, file);
			fflush(file);
		}
	#else 
		static FILE* file = fopen ("log.bin", "rb");
		if (file) {
			fread (&player->m_headinAngle, sizeof (dFloat), 1, file);
			fread (&player->m_forwarSpeed, sizeof (dFloat), 1, file);
			fread (&player->m_strafeSpeed, sizeof (dFloat), 1, file);
			fread (&player->m_jumpSpeed, sizeof (dFloat), 1, file);
			fread (&player->m_shootProp, sizeof (int), 1, file);
		}
	#endif
#endif
		
		
		// calculate desired linear and angular velocity form the input
		dVector gravity (0.0f, DEMO_GRAVITY, 0.0f, 0.0f);

		// set player linear and angular velocity
		controller->SetPlayerVelocity (player->m_forwarSpeed, player->m_strafeSpeed, player->m_jumpSpeed, player->m_headinAngle, gravity, timestep);
	}


	virtual int ProcessContacts (const CustomPlayerController* const controller, NewtonWorldConvexCastReturnInfo* const contacts, int count) const 
	{
		count = CustomPlayerControllerManager::ProcessContacts (controller, contacts, count); 
		return count;
	}

	void PostUpdate (dFloat timestep)
	{
		// update all characters physics
		CustomPlayerControllerManager::PostUpdate(timestep);

		// now overwrite the camera to match the player character location 
		if (m_player) {
			dMatrix playerMatrix (m_player->GetNextMatrix()); 
			dFloat height = m_player->m_controller->GetHight();
			dVector upDir (m_player->m_controller->GetUpDir());
			playerMatrix.m_posit += upDir.Scale(height);

			DemoEntityManager* const scene = (DemoEntityManager*) NewtonWorldGetUserData(GetWorld());
			DemoCamera* const camera = scene->GetCamera();
			camera->SetNavigationMode(false);


			// toggle between third and first person
			NewtonDemos* const mainWindow = scene->GetRootWindow();
			dMatrix camMatrix (camera->GetNextMatrix ());

			m_cameraMode.UpdatePushButton(mainWindow, 'C');
			if (m_cameraMode.GetPushButtonState()) {
				// first person view, player skip rendering
				m_player->m_skipRender = true;
			} else {
				m_player->m_skipRender = false;
				// third person view, set camera at some distance from payer
				dVector dist (camMatrix.m_front.Scale (PLAYER_THIRD_PERSON_VIEW_DIST));
				playerMatrix.m_posit -= dist;
				// raise a little more 
				playerMatrix.m_posit += upDir.Scale(height);
			}

			// smooth out the camera position 
			playerMatrix.m_posit = camMatrix.m_posit + (playerMatrix.m_posit - camMatrix.m_posit).Scale(0.5f);

			camera->SetNextMatrix(*scene, dQuaternion (camMatrix), playerMatrix.m_posit);

			// update the shot button
			if (m_player->m_shootProp) {
				SpawnRandomProp (camera->GetNextMatrix());
			}
		}
	}


	void SpawnRandomProp(const dMatrix& location) const
	{
		NewtonWorld* const world = GetWorld();
		DemoEntityManager* const scene = (DemoEntityManager*) NewtonWorldGetUserData(world);
		scene->SetCurrent();

		static PrimitiveType proSelection[] = {_SPHERE_PRIMITIVE, _BOX_PRIMITIVE, _CAPSULE_PRIMITIVE, _CYLINDER_PRIMITIVE, _CONE_PRIMITIVE, 
			                                   _TAPERED_CAPSULE_PRIMITIVE, _TAPERED_CYLINDER_PRIMITIVE, 
			                                   _CHAMFER_CYLINDER_PRIMITIVE, _RANDOM_CONVEX_HULL_PRIMITIVE, _REGULAR_CONVEX_HULL_PRIMITIVE};

		PrimitiveType type = PrimitiveType (dRand() % (sizeof (proSelection) / sizeof (proSelection[0])));

		dVector size (0.35f, 0.25f, 0.25f, 0.0f);
		NewtonCollision* const collision = CreateConvexCollision (world, GetIdentityMatrix(), size, type, 0);
		DemoMesh* const geometry = new DemoMesh("prop", collision, "smilli.tga", "smilli.tga", "smilli.tga");

		dMatrix matrix (location);

		NewtonBody* const prop = CreateSimpleSolid (scene, geometry, 30.0f, matrix, collision, 0);
		NewtonDestroyCollision(collision);
		geometry->Release();

		dFloat initialSpeed = 20.0f; 
		dVector veloc (matrix.m_front.Scale (initialSpeed));
		NewtonBodySetVelocity(prop, &veloc[0]);
		NewtonBodySetLinearDamping(prop, 0);

		// for now off until joints CCD is implemented!!
		//NewtonBodySetContinuousCollisionMode(prop, 1);
	}


	BasicPlayerEntity* m_player;
	DemoEntityManager::ButtonKey m_shotProp;
	DemoEntityManager::ButtonKey m_cameraMode;
};


void BasicPlayerController (DemoEntityManager* const scene)
{
	// load the sky box
	scene->CreateSkyBox();

	//CreateLevelMesh (scene, "flatPlane.ngd", true);
	//CreateLevelMesh (scene, "playground.ngd", true);
	//CreateLevelMesh (scene, "castle.ngd", true);
	//CreateLevelMesh (scene, "sponza.ngd", true);
	CreateLevelMesh (scene, "sibenik.ngd", true);


	NewtonWorld* const world = scene->GetNewton();

	// create a character controller manager
	BasicPlayerControllerManager* const manager = new BasicPlayerControllerManager (world);

	// add main player
	dMatrix location (GetIdentityMatrix());
	location.m_posit.m_x = -4.0f;
	location.m_posit.m_y = 5.0f;
	location.m_posit.m_z = 0.0f;
	location.m_posit = FindFloor (scene->GetNewton(), location.m_posit, 10.0f);
	BasicPlayerEntity* const player = new BasicPlayerEntity (scene, manager, 0.5f, 1.9f, location);

	// set as the player with the camera
	manager->SetAsPlayer(player);


	int defaultMaterialID = NewtonMaterialGetDefaultGroupID (scene->GetNewton());
	location.m_posit.m_x += 5.0f;
	dVector size (2.0f, 2.0f, 2.0f, 0.0f);
	int count = 1;
	dMatrix shapeOffsetMatrix (GetIdentityMatrix());
	AddPrimitiveArray(scene, 100.0f, location.m_posit, size, count, count, 5.0f, _BOX_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix, 10.0f);

	dVector origin (-10.0f, 2.0f, 0.0f, 0.0f);
	dQuaternion rot;
	scene->SetCameraMatrix(rot, origin);
}





