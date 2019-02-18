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

class BasicPlayerEntity: public DemoEntity
{
	public: 
	class InputRecord
	{
		public:
		InputRecord()
		{
			memset (this, 0, sizeof (InputRecord));
			m_cameraMode = 1;
		}
		
		dFloat m_headinAngle;
		dFloat m_forwarSpeed;
		dFloat m_strafeSpeed;
		dFloat m_jumpSpeed;
		int m_cameraMode;
	};

	BasicPlayerEntity (DemoEntityManager* const scene, dCustomPlayerControllerManager* const manager, dFloat radius, dFloat height, const dMatrix& location)
		:DemoEntity (dGetIdentityMatrix(), NULL)
	{
		// add this entity to the scene for rendering
		scene->Append(this);

		// now make a simple player controller, 
		dMatrix playerAxis; 
		playerAxis[0] = dVector (0.0f, 1.0f, 0.0f, 0.0f); // the y axis is the character up vector
		playerAxis[1] = dVector (1.0f, 0.0f, 0.0f, 0.0f); // the x axis is the character front direction
		playerAxis[2] = playerAxis[0].CrossProduct(playerAxis[1]);
		playerAxis[3] = dVector (0.0f, 0.0f, 0.0f, 1.0f);

		// make the player controller, this function makes a kinematic body
		m_controller = manager->CreatePlayer(PLAYER_MASS, radius, radius * 0.5f, height, height * 0.33f, playerAxis);
		//m_controller = manager->CreatePlayer(200.0f, 0.5f, 0.4f, 2.0f, 0.5f, playerAxis);

		// players by default have the origin at the center of mass of the collision shape.
		// you can change this by calling SetPlayerOrigin, for example if a player has its origin at the lower bottom of its AABB 
		m_controller->SetPlayerOrigin(height * 0.0f);

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
		DemoMesh* const geometry = new DemoMesh("player", scene->GetShaderCache(), collision, "smilli.tga", "smilli.tga", "smilli.tga");
		SetMesh(geometry, dGetIdentityMatrix());
		geometry->Release(); 
	}


	~BasicPlayerEntity ()
	{
		// destroy the player controller and its rigid body
		((dCustomPlayerControllerManager*)m_controller->GetManager())->DestroyController(m_controller);
	}

	void SetInput (const InputRecord& inputs)
	{
		m_inputs = inputs;
	}

	virtual void Render(dFloat timeStep, DemoEntityManager* const scene) const
	{
		if (m_inputs.m_cameraMode) {
			// render only when external view mode
			DemoEntity::Render(timeStep, scene);
		}
	}

	InputRecord	m_inputs;
	dCustomPlayerController* m_controller; 
};

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

	// apply gravity 
	virtual void ApplyPlayerMove (dCustomPlayerController* const controller, dFloat timestep)
	{
		NewtonBody* const body = controller->GetBody();
		BasicPlayerEntity* const player = (BasicPlayerEntity*) NewtonBodyGetUserData(body);

		// calculate desired linear and angular velocity form the input
		dVector gravity (0.0f, DEMO_GRAVITY, 0.0f, 0.0f);

		// set player linear and angular velocity
		controller->SetPlayerVelocity (player->m_inputs.m_forwarSpeed, player->m_inputs.m_strafeSpeed, player->m_inputs.m_jumpSpeed, player->m_inputs.m_headinAngle, gravity, timestep);
	}


	virtual int ProcessContacts (const dCustomPlayerController* const controller, NewtonWorldConvexCastReturnInfo* const contacts, int count) const 
	{
		count = dCustomPlayerControllerManager::ProcessContacts (controller, contacts, count); 
		return count;
	}
};

// we recommend using and input manage to control input for all games
class BasicPlayerInputManager: public dCustomInputManager
{
	public:
	BasicPlayerInputManager (DemoEntityManager* const scene)
		:dCustomInputManager(scene->GetNewton())
		,m_scene(scene)
		,m_player(NULL)
		,m_jumpKey(false)
		,m_cameraMode(true)
		,m_shootProp(false)
		,m_shootState(0)
	{
		// plug a callback for 2d help display
		scene->Set2DDisplayRenderFunction (RenderPlayerHelp, NULL, this);
	}

	void SpawnRandomProp(const dMatrix& location) const
	{
		NewtonWorld* const world = GetWorld();
		DemoEntityManager* const scene = (DemoEntityManager*) NewtonWorldGetUserData(world);
		//scene->SetCurrent();

		static PrimitiveType proSelection[] = {_SPHERE_PRIMITIVE, _BOX_PRIMITIVE, _CAPSULE_PRIMITIVE, _CYLINDER_PRIMITIVE, _CONE_PRIMITIVE, 
											   _CHAMFER_CYLINDER_PRIMITIVE, _RANDOM_CONVEX_HULL_PRIMITIVE, _REGULAR_CONVEX_HULL_PRIMITIVE};

		PrimitiveType type = PrimitiveType (dRand() % (sizeof (proSelection) / sizeof (proSelection[0])));

		dVector size (0.35f, 0.25f, 0.25f, 0.0f);
		NewtonCollision* const collision = CreateConvexCollision (world, dGetIdentityMatrix(), size, type, 0);
		DemoMesh* const geometry = new DemoMesh("prop", scene->GetShaderCache(), collision, "smilli.tga", "smilli.tga", "smilli.tga");

		dMatrix matrix (location);
		matrix.m_posit.m_y += 0.5f;

		NewtonBody* const prop = CreateSimpleSolid (scene, geometry, 30.0f, matrix, collision, 0);
		NewtonDestroyCollision(collision);
		geometry->Release();

		dFloat initialSpeed = 20.0f; 
		dVector veloc (matrix.m_front.Scale (initialSpeed));
		NewtonBodySetVelocity(prop, &veloc[0]);
		NewtonBodySetLinearDamping(prop, 0);
	}


	void OnBeginUpdate (dFloat timestepInSecunds)
	{
//		dAssert(0);
		/*
		BasicPlayerEntity::InputRecord inputs;

		DemoCamera* const camera = m_scene->GetCamera();

		// set the help key
		m_helpKey.UpdatePushButton (m_scene, 'H');

		// read the player inputs
		inputs.m_headinAngle = camera->GetYawAngle();
		inputs.m_cameraMode = m_cameraMode.UpdatePushButton(m_scene, 'C') ? 1 : 0;
		inputs.m_forwarSpeed = (int (m_scene->GetKeyState ('W')) - int (m_scene->GetKeyState ('S'))) * PLAYER_WALK_SPEED;
		inputs.m_strafeSpeed = (int (m_scene->GetKeyState ('D')) - int (m_scene->GetKeyState ('A'))) * PLAYER_WALK_SPEED;
		inputs.m_jumpSpeed = (m_jumpKey.UpdateTriggerButton(m_scene, ' ')) ? PLAYER_JUMP_SPEED : 0.0f;

		// normalize player speed
		dFloat mag2 = inputs.m_forwarSpeed * inputs.m_forwarSpeed + inputs.m_strafeSpeed * inputs.m_strafeSpeed;
		if (mag2 > 0.0f) {
			dFloat invMag = PLAYER_WALK_SPEED / dSqrt (mag2);
			inputs.m_forwarSpeed *= invMag;
			inputs.m_strafeSpeed *= invMag;
		}

		// see if we are shotting some props
		m_shootState = m_shootProp.UpdateTriggerButton(m_scene, 0x0d) ? 1 : 0;


#if 0
	#if 0
		static FILE* file = fopen ("log.bin", "wb");
		if (file) {
			fwrite (&inputs, sizeof (inputs), 1, file);
			fflush(file);
		}
	#else 
		static FILE* file = fopen ("log.bin", "rb");
		if (file) {
			fread (&inputs, sizeof (inputs), 1, file);
		}
	#endif
#endif

		m_player->SetInput(inputs);
*/
	}

	void OnEndUpdate (dFloat timestepInSecunds)
	{
		DemoCamera* const camera = m_scene->GetCamera();

		dMatrix camMatrix(camera->GetNextMatrix());
		dMatrix playerMatrix (m_player->GetNextMatrix());

		dVector frontDir (camMatrix[0]);

		dCustomPlayerController* const controller = m_player->m_controller; 
		dFloat height = controller->GetHigh();
		dVector upDir (controller->GetUpDir());

		dVector camOrigin(0.0f); 
		
		if (m_player->m_inputs.m_cameraMode) {
			// set third person view camera
			camOrigin = playerMatrix.TransformVector (upDir.Scale(height));
			camOrigin -= frontDir.Scale (PLAYER_THIRD_PERSON_VIEW_DIST);
		} else {
			// set first person view camera
			camMatrix = camMatrix * playerMatrix;
			camOrigin = playerMatrix.TransformVector (upDir.Scale(height));
		}

		camera->SetNextMatrix (*m_scene, camMatrix, camOrigin);

		// update the shot button
		if (m_shootState) {
			SpawnRandomProp (camera->GetNextMatrix());
		}
	}

	void AddPlayer (BasicPlayerEntity* const player)
	{
		m_player = player;
	}

	void RenderPlayerHelp (DemoEntityManager* const scene) const
	{
		dVector color(1.0f, 1.0f, 0.0f, 0.0f);
		scene->Print (color, "Navigation Keys");
		scene->Print (color, "walk forward:            W");
		scene->Print (color, "walk backward:           S");
		scene->Print (color, "strafe right:            D");
		scene->Print (color, "strafe left:             A");
		scene->Print (color, "toggle camera mode:      C");
		scene->Print (color, "jump:                    Space");
		scene->Print (color, "hide help:               H");
		scene->Print (color, "change player direction: Left mouse button");
	}


	static void RenderPlayerHelp (DemoEntityManager* const scene, void* const context)
	{
		BasicPlayerInputManager* const me = (BasicPlayerInputManager*) context;
		me->RenderPlayerHelp (scene);
	}


	DemoEntityManager* m_scene;
	BasicPlayerEntity* m_player;
	DemoEntityManager::ButtonKey m_jumpKey;
	DemoEntityManager::ButtonKey m_cameraMode;
	DemoEntityManager::ButtonKey m_shootProp;

	int m_shootState;
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
	BasicPlayerInputManager* const inputManager = new BasicPlayerInputManager (scene);

	// create a character controller manager
	BasicPlayerControllerManager* const playerManager = new BasicPlayerControllerManager (world);

	// add main player
	dMatrix location (dGetIdentityMatrix());
	location.m_posit.m_x = -4.0f;
	location.m_posit.m_y = 5.0f;
	location.m_posit.m_z = 0.0f;

	location.m_posit.m_x = 98.710999f;
	location.m_posit.m_y =-0.96156919f; 
	location.m_posit.m_z = 27.254711f;

	location.m_posit = FindFloor (scene->GetNewton(), location.m_posit, 10.0f);
	BasicPlayerEntity* const player = new BasicPlayerEntity (scene, playerManager, 0.5f, 1.9f, location);

	// set as the player with the camera
	inputManager->AddPlayer(player);

	int defaultMaterialID = NewtonMaterialGetDefaultGroupID (scene->GetNewton());
	location.m_posit.m_x += 5.0f;
	dVector size (2.0f, 2.0f, 2.0f, 0.0f);

	int count = 0;
	dMatrix shapeOffsetMatrix (dGetIdentityMatrix());
	AddPrimitiveArray(scene, 100.0f, location.m_posit, size, count, count, 5.0f, _BOX_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix, 10.0f);

	dVector origin (-10.0f, 2.0f, 0.0f, 0.0f);
	dQuaternion rot;
	scene->SetCameraMatrix(rot, origin);
}





