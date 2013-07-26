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
#include "CustomHinge.h"
#include "CustomTriggerManager.h"
#include "CustomKinematicController.h"
#include "DemoMesh.h"
#include "DemoEntityManager.h"
#include "DemoCamera.h"
#include "../toolBox/DebugDisplay.h"
#include "CustomPlayerControllerManager.h"


#define PLAYER_WALK_SPEED				4.0f
#define PLAYER_THIRD_PERSON_VIEW_DIST	8.0f

// define the massage we will use for this Game
#define ENTER_TRIGGER				1
#define EXIT_TRIGGER				2
#define INSIDE_TRIGGER				3

#define PLAYER_CALL_FERRY_DRIVER	4
#define PLAYER_BOARDED_FERRY_DRIVER	5


class PlaformEntityEntity;
class AdvancePlayerControllerManager;

// this demo use a trigger and player controller to automate the operation of a mini game environment
class TriggerManager: public CustomTriggerManager
{
	public:
	TriggerManager(NewtonWorld* const world, AdvancePlayerControllerManager* const playerManager)
		:CustomTriggerManager(world)
		,m_playerManager(playerManager)
	{
	}

	virtual void EventCallback (const CustomTriggerController* const me, TriggerEventType event, NewtonBody* const visitor) const
	{
		// send this message to the entity
		DemoEntity* const entity = (DemoEntity*) NewtonBodyGetUserData(visitor);
		if (entity) {
			// map the event massage 
			int message = -1;
			switch (event) {
				case m_enterTrigger:
					message = ENTER_TRIGGER;
					break;
				case m_exitTrigger:
					message = EXIT_TRIGGER;
					break;

				case m_inTrigger:
					message = INSIDE_TRIGGER;
					break;
			}
				
			// pass the controller pointer as the user data of this massage
			entity->MessageHandler(me->GetBody(), message, (void*)me);
		}
	}
	AdvancePlayerControllerManager* m_playerManager;
};


class AdvancePlayerEntity: public DemoEntity
{
	public: 

	AdvancePlayerEntity (DemoEntityManager* const scene, CustomPlayerControllerManager* const manager, dFloat radius, dFloat height, const dMatrix& location)
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
		m_controller = manager->CreatePlayer(80.0f, radius, radius * 0.5f, height, height * 0.33f, playerAxis);

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

	~AdvancePlayerEntity ()
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

	virtual void MessageHandler (NewtonBody* const sender, int message, void* const data)
	{
		switch (message) 
		{
			//a player can be in multiple triggers at the same time, it is important to see what trigger this is
			case ENTER_TRIGGER:
			{
				// the player enter a trigger, we most verify what trigger is this so that we can start some action
				CustomTriggerController* const trigger = (CustomTriggerController*) data;
				DemoEntity* const entity = (DemoEntity*) trigger->GetUserData();

				// we are simple going to signal to the entity controlled by this trigger that a player enter the trigger
				entity->MessageHandler(m_controller->GetBody(), PLAYER_CALL_FERRY_DRIVER, data);
				break;
			}

			case EXIT_TRIGGER:
			{
				// the player exit a trigger, we most verify what trigger is this so that we can finalized some action
				CustomTriggerController* const trigger = (CustomTriggerController*) data;
				DemoEntity* const entity = (DemoEntity*) trigger->GetUserData();

				// the player left the trigger and tell the ferry driver to start to move to the next destination
				entity->MessageHandler(m_controller->GetBody(), PLAYER_BOARDED_FERRY_DRIVER, data);
				break;
			}
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
	
	NewtonBody* m_currentTrigger;
	PlaformEntityEntity* m_currentPlatform;
	CustomPlayerControllerManager::CustomController* m_controller; 
};

class AdvancePlayerControllerManager: public CustomPlayerControllerManager
{
	public:
	AdvancePlayerControllerManager (NewtonWorld* const world)
		:CustomPlayerControllerManager (world)
		,m_player (NULL) 
		,m_shootProp(true)
		,m_cameraMode(false)
	{
		// hook a callback for 2d help display
		DemoEntityManager* const scene = (DemoEntityManager*) NewtonWorldGetUserData(world);
		scene->Set2DDisplayRenderFunction (RenderPlayerHelp, this);
	}

	~AdvancePlayerControllerManager ()
	{
	}

	static void RenderPlayerHelp (DemoEntityManager* const scene, void* const context)
	{
		AdvancePlayerControllerManager* const me = (AdvancePlayerControllerManager*) context;
		me->RenderPlayerHelp (scene);
	}


	void SetAsPlayer (AdvancePlayerEntity* const player)
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
			scene->Print (color, 10, 340, "shoot random prop:       Enter");
			scene->Print (color, 10, 360, "hide help:               H");
			scene->Print (color, 10, 380, "change player direction: Left mouse button");
		}
	}


	void PlayerInputs (const CustomPlayerController* const controller, dFloat timestep)
	{
		DemoEntityManager* const scene = (DemoEntityManager*) NewtonWorldGetUserData(GetWorld());

		DemoCamera* const camera = scene->GetCamera();

		NewtonDemos* const mainWindow = scene->GetRootWindow();
		NewtonBody* const body = controller->GetBody();
		AdvancePlayerEntity* const player = (AdvancePlayerEntity*) NewtonBodyGetUserData(body);

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

		// see if we are shoting
		m_player->m_shootProp = m_shootProp.UpdateTriggerButton(mainWindow, 0x0d) ? 1 : 0;

		// set the help key
		m_player->m_helpKey.UpdatePushButton (mainWindow, 'H');
	}


	// apply gravity 
	virtual void ApplyPlayerMove (CustomPlayerController* const controller, dFloat timestep)
	{
		NewtonBody* const body = controller->GetBody();

		AdvancePlayerEntity* const player = (AdvancePlayerEntity*) NewtonBodyGetUserData(body);
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
			fwrite (&player->m_shootProp, sizeof (m_shootProp), 1, file);
			fflush(file);
		}
	#else 
		static FILE* file = fopen ("log.bin", "rb");
		if (file) {
			fread (&player->m_headinAngle, sizeof (dFloat), 1, file);
			fread (&player->m_forwarSpeed, sizeof (dFloat), 1, file);
			fread (&player->m_strafeSpeed, sizeof (dFloat), 1, file);
			fread (&player->m_jumpSpeed, sizeof (dFloat), 1, file);
			fread (&player->m_shootProp, sizeof (m_shootProp), 1, file);
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
			dFloat height = m_player->m_controller->GetHigh();
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
//		matrix.m_posit += matrix.RotateVector (controller->GetUpDir().Scale (controller->GetHight()));

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

	
	AdvancePlayerEntity* m_player;
	DemoEntityManager::ButtonKey m_shootProp;
	DemoEntityManager::ButtonKey m_cameraMode;
};


class PlaformEntityEntity: public DemoEntity
{
	public:
	class FerryDriver: public CustomKinematicController
	{
		public:
		enum State
		{
			m_stop,
			m_driving,
		};

		FerryDriver (NewtonBody* const body, const dVector& pivot, NewtonBody* const triggerPort0, NewtonBody* const triggerPort1)
			:CustomKinematicController (body, pivot)
			,m_triggerPort0(triggerPort0)
			,m_triggerPort1(triggerPort1)
			,m_state(m_driving)
		{
			dMatrix matrix0;
			dMatrix matrix1;

			NewtonBodyGetMatrix(m_triggerPort0, &matrix0[0][0]);
			NewtonBodyGetMatrix(m_triggerPort1, &matrix1[0][0]);

			dVector dist0 (matrix0.m_posit - pivot);
			dVector dist1 (matrix1.m_posit - pivot);
			//dist0.m_y = 0.0f;
			//dist1.m_y = 0.0f;
			if ((dist0 % dist0) < (dist1 % dist1)) {
				m_target = matrix0.m_posit;
				m_currentPort = m_triggerPort0;
			} else {
				m_target = matrix1.m_posit;
				m_currentPort = m_triggerPort1;
			}
			//m_target.m_y = pivot.m_y;
		}

		virtual void SubmitConstraints (dFloat timestep, int threadIndex)
		{
			dMatrix matrix;
			dVector com;
			const dFloat speed = 3.0f;
			NewtonBody* const body = GetBody0();

			NewtonBodyGetCentreOfMass(body, &com[0]);
			NewtonBodyGetMatrix(body, &matrix[0][0]);
			com = matrix.TransformVector(com);

			switch (m_state)
			{
				case m_stop:
				{
					SetTargetPosit (com);
					break;
				}

				case m_driving:
				{
					dVector veloc (m_target - com);
					veloc = veloc.Scale (speed / dSqrt (veloc % veloc)); 
					dVector target = com + veloc.Scale(timestep);
					SetTargetPosit (target);
					break;
				}

				default:;
				dAssert (0);
			}

			CustomKinematicController::SubmitConstraints (timestep, threadIndex);
		}

		void Stop ()
		{
			m_state = m_stop;
		}


		void MoveToTarget (NewtonBody* const targetBody)
		{
			dAssert ((targetBody == m_triggerPort0) || (targetBody == m_triggerPort1));
			if (targetBody != m_currentPort) {
				m_currentPort = targetBody;

				// get the location of next target
				dMatrix targetMatrix;
				NewtonBodyGetMatrix(m_currentPort, &targetMatrix[0][0]);
				m_target = targetMatrix.m_posit;

				// the body might be sleep we need to activate the body by sett the sleep stet off
				NewtonBodySetSleepState(GetBody0(), 0);
				m_state = m_driving;
			}
		}

		void FlipTarget ()
		{
			MoveToTarget ((m_currentPort == m_triggerPort0) ? m_triggerPort1 : m_triggerPort0);
		}

		dVector m_target;	
		NewtonBody* m_currentPort;
		NewtonBody* m_triggerPort0;
		NewtonBody* m_triggerPort1;
		State m_state;
	};

	PlaformEntityEntity (DemoEntityManager* const scene, DemoEntity* const source, NewtonBody* const triggerPort0, NewtonBody* const triggerPort1)
		:DemoEntity (source->GetNextMatrix(), NULL)
	{
		scene->Append(this);

		DemoMesh* const mesh = source->GetMesh();
		SetMesh(mesh);

		const dFloat mass = 100.9f;
		dMatrix matrix (source->GetNextMatrix()) ;
		NewtonWorld* const world = scene->GetNewton();
		NewtonCollision* const collision = NewtonCreateConvexHull(world, mesh->m_vertexCount, mesh->m_vertex, 3 * sizeof (dFloat), 0, 0, NULL);
		NewtonBody* body = CreateSimpleBody (world, this, 100, matrix, collision, 0);
		NewtonDestroyCollision(collision);


		// attach a kinematic joint controller joint to move this body 
		dVector pivot;
		NewtonBodyGetCentreOfMass (body, &pivot[0]);
		pivot = matrix.TransformVector(pivot);
		m_driver = new FerryDriver (body, pivot, triggerPort0, triggerPort1);
		m_driver->SetMaxLinearFriction (50.0f * dAbs (mass * DEMO_GRAVITY)); 
		m_driver->SetMaxAngularFriction(50.0f * dAbs (mass * DEMO_GRAVITY)); 
	}


	virtual void MessageHandler (NewtonBody* const sender, int message, void* const data)
	{
		DemoEntity::MessageHandler (sender, message, data);
		switch (message) 
		{
			case ENTER_TRIGGER:
			{
				// when this signal is send by a trigger, it means this body reach his destination and should stops
				// m_enterTrigger signal is only send once
				m_driver->Stop ();
				m_driver->m_state = FerryDriver::m_stop;
				break;
			}
			case PLAYER_CALL_FERRY_DRIVER:
			{
				CustomTriggerController* const trigger = (CustomTriggerController*) data;
				dAssert (trigger->GetUserData() == this);
				m_driver->MoveToTarget (trigger->GetBody());
				break;
			}
			case PLAYER_BOARDED_FERRY_DRIVER:
			{
				m_driver->FlipTarget();
				break;
			}
		}
	}
	FerryDriver* m_driver;
};


static DemoEntityManager::dListNode* LoadScene(DemoEntityManager* const scene, const char* const name, const dMatrix& location)
{
	char fileName[2048];
	GetWorkingFileName (name, fileName);

	DemoEntityManager::dListNode* const lastEnt = scene->GetLast();
	scene->LoadScene (fileName);
	for (DemoEntityManager::dListNode* node = lastEnt->GetNext(); node; node = node->GetNext()) {
		DemoEntity* const entity = node->GetInfo();
		dMatrix matrix (entity->GetNextMatrix() * location);
		entity->ResetMatrix(*scene, matrix);
	}
	return lastEnt->GetNext(); 
}

static void LoadFloor(DemoEntityManager* const scene, NewtonCollision* const sceneCollision)
{
	NewtonWorld* const world = scene->GetNewton();

	// add a flat plane
	dMatrix matrix (GetIdentityMatrix());
	DemoEntityManager::dListNode* const floorNode = LoadScene(scene, "flatPlane.ngd", matrix);

	DemoEntity* const entity = floorNode->GetInfo();
	DemoMesh* const mesh = entity->GetMesh();

	NewtonCollision* const tree = NewtonCreateTreeCollision(world, 0);
	NewtonTreeCollisionBeginBuild(tree);

	dFloat* const vertex = mesh->m_vertex;
	for (DemoMesh::dListNode* node = mesh->GetFirst(); node; node = node->GetNext()){
		DemoSubMesh* const subMesh = &node->GetInfo();
		unsigned int* const indices = subMesh->m_indexes;
		int trianglesCount = subMesh->m_indexCount;
		for (int i = 0; i < trianglesCount; i += 3) {

			dVector face[3];
			int index = indices[i + 0] * 3;
			face[0] = dVector (vertex[index + 0], vertex[index + 1], vertex[index + 2]);

			index = indices[i + 1] * 3;
			face[1] = dVector (vertex[index + 0], vertex[index + 1], vertex[index + 2]);

			index = indices[i + 2] * 3;
			face[2] = dVector (vertex[index + 0], vertex[index + 1], vertex[index + 2]);

			int matID = 0;
			//matID = matID == 2 ? 1 : 2 ;
			NewtonTreeCollisionAddFace(tree, 3, &face[0].m_x, sizeof (dVector), matID);
		}
	}
	NewtonTreeCollisionEndBuild (tree, 1);

	// add the collision tree to the collision scene
	void* const proxy = NewtonSceneCollisionAddSubCollision (sceneCollision, tree);

	// destroy the original tree collision
	NewtonDestroyCollision (tree);


	// set the parameter on the added collision share
	matrix = entity->GetCurrentMatrix();
	NewtonCollision* const collisionTree = NewtonSceneCollisionGetCollisionFromNode (sceneCollision, proxy);

	NewtonSceneCollisionSetSubCollisionMatrix (sceneCollision, proxy, &matrix[0][0]);	
	NewtonCollisionSetUserData(collisionTree, entity);

	// set the application level callback, for debug display
	#ifdef USE_STATIC_MESHES_DEBUG_COLLISION
	NewtonStaticCollisionSetDebugCallback (collisionTree, ShowMeshCollidingFaces);
	#endif
}

static void LoadFerryBridge (DemoEntityManager* const scene, TriggerManager* const triggerManager, NewtonCollision* const sceneCollision, const char* const name, const dMatrix& location, NewtonBody* const playGroundBody)
{
	NewtonWorld* const world = scene->GetNewton();
	DemoEntityManager::dListNode* const bridgeNodes = scene->GetLast();
	LoadScene(scene, name, location);

	// add bridge foundations
	for (DemoEntityManager::dListNode* node = bridgeNodes->GetNext(); node; node = node->GetNext()) {
		DemoEntity* const entity = node->GetInfo();
		if (entity->GetName().Find("ramp") != -1) {
			DemoMesh* const mesh = entity->GetMesh();

			NewtonCollision* const collision = NewtonCreateConvexHull(world, mesh->m_vertexCount, mesh->m_vertex, 3 * sizeof (dFloat), 0, 0, NULL);
			void* const proxy = NewtonSceneCollisionAddSubCollision (sceneCollision, collision);
			NewtonDestroyCollision (collision);

			// get the location of this tire relative to the car chassis
			dMatrix matrix (entity->GetNextMatrix());

			NewtonCollision* const bridgeCollision = NewtonSceneCollisionGetCollisionFromNode (sceneCollision, proxy);
			NewtonSceneCollisionSetSubCollisionMatrix (sceneCollision, proxy, &matrix[0][0]);	
			NewtonCollisionSetUserData(bridgeCollision, entity);
		}
	}


	// add start triggers
	CustomTriggerController* trigger0 = NULL;
	CustomTriggerController* trigger1 = NULL;
	for (DemoEntityManager::dListNode* node = bridgeNodes->GetNext(); node;) {
		DemoEntity* const entity = node->GetInfo();
		node = node->GetNext() ;
		if (entity->GetName().Find("startTrigger") != -1) {
			DemoMesh* const mesh = entity->GetMesh();

			// create a trigger to match his mesh
			NewtonCollision* const collision = NewtonCreateConvexHull(world, mesh->m_vertexCount, mesh->m_vertex, 3 * sizeof (dFloat), 0, 0, NULL);
			dMatrix matrix (entity->GetNextMatrix());
			CustomTriggerController* const controller = triggerManager->CreateTrigger(matrix, collision, NULL);
			NewtonDestroyCollision (collision);

			if (!trigger0) {
				trigger0 = controller;
			} else {
				trigger1 = controller;
			}
			// remove this entity from the scene, since we do not wan the trigger to be visible
			// to see triggers click "show collision mesh, and or "show contact points" in the main menu.
			scene->RemoveEntity(entity);
		}
	}

	// add the platform object
	for (DemoEntityManager::dListNode* node = bridgeNodes->GetNext(); node;) {
		DemoEntity* const entity = node->GetInfo();
		node = node->GetNext() ;
		if (entity->GetName().Find("platform") != -1) {
			// make a platform object
			PlaformEntityEntity* const platformEntity = new PlaformEntityEntity (scene, entity, trigger0->GetBody(), trigger1->GetBody());

			// store the platform in the trigger use data
			trigger0->SetUserData(platformEntity);
			trigger1->SetUserData(platformEntity);

			// remove this entity from the scene
			scene->RemoveEntity(entity);
		}
	}
}


static void LoadSlide (DemoEntityManager* const scene, TriggerManager* const triggerManager, NewtonCollision* const sceneCollision, const char* const name, const dMatrix& location, NewtonBody* const playGroundBody)
{
	NewtonWorld* const world = scene->GetNewton();
	DemoEntityManager::dListNode* const bridgeNodes = scene->GetLast();
	LoadScene(scene, name, location);

	// add bridge foundations
	for (DemoEntityManager::dListNode* node = bridgeNodes->GetNext(); node; node = node->GetNext()) {
		DemoEntity* const entity = node->GetInfo();
		if (entity->GetName().Find("ramp") != -1) {
			DemoMesh* const mesh = entity->GetMesh();

			NewtonCollision* const collision = NewtonCreateConvexHull(world, mesh->m_vertexCount, mesh->m_vertex, 3 * sizeof (dFloat), 0, 0, NULL);
			void* const proxy = NewtonSceneCollisionAddSubCollision (sceneCollision, collision);
			NewtonDestroyCollision (collision);

			// get the location of this tire relative to the car chassis
			dMatrix matrix (entity->GetNextMatrix());

			NewtonCollision* const bridgeCollision = NewtonSceneCollisionGetCollisionFromNode (sceneCollision, proxy);
			NewtonSceneCollisionSetSubCollisionMatrix (sceneCollision, proxy, &matrix[0][0]);	
			NewtonCollisionSetUserData(bridgeCollision, entity);
		}
	}


	// add start triggers
	CustomTriggerController* trigger0 = NULL;
	CustomTriggerController* trigger1 = NULL;
	for (DemoEntityManager::dListNode* node = bridgeNodes->GetNext(); node;) {
		DemoEntity* const entity = node->GetInfo();
		node = node->GetNext() ;
		if (entity->GetName().Find("startTrigger") != -1) {
			DemoMesh* const mesh = entity->GetMesh();

			// create a trigger to match his mesh
			NewtonCollision* const collision = NewtonCreateConvexHull(world, mesh->m_vertexCount, mesh->m_vertex, 3 * sizeof (dFloat), 0, 0, NULL);
			dMatrix matrix (entity->GetNextMatrix());
			CustomTriggerController* const controller = triggerManager->CreateTrigger(matrix, collision, NULL);
			NewtonDestroyCollision (collision);

			if (!trigger0) {
				trigger0 = controller;
			} else {
				trigger1 = controller;
			}
			// remove this entity from the scene, since we do not wan the trigger to be visible
			// to see triggers click "show collision mesh, and or "show contact points" in the main menu.
			scene->RemoveEntity(entity);
		}
	}

	// add the platform object
	for (DemoEntityManager::dListNode* node = bridgeNodes->GetNext(); node;) {
		DemoEntity* const entity = node->GetInfo();
		node = node->GetNext() ;
		if (entity->GetName().Find("platform") != -1) {
			// make a platform object
			PlaformEntityEntity* const platformEntity = new PlaformEntityEntity (scene, entity, trigger0->GetBody(), trigger1->GetBody());

			// store the platform in the trigger use data
			trigger0->SetUserData(platformEntity);
			trigger1->SetUserData(platformEntity);

			// remove this entity from the scene
			scene->RemoveEntity(entity);
		}
	}

}


static void LoadHangingBridge (DemoEntityManager* const scene, TriggerManager* const triggerManager, NewtonCollision* const sceneCollision, const char* const name, const dMatrix& location, NewtonBody* const playGroundBody)
{
	NewtonWorld* const world = scene->GetNewton();
	DemoEntityManager::dListNode* const bridgeNodes = scene->GetLast();
	LoadScene(scene, name, location);

	// add bridge foundations
	for (DemoEntityManager::dListNode* node = bridgeNodes->GetNext(); node; node = node->GetNext()) {
		DemoEntity* const entity = node->GetInfo();
		if (entity->GetName().Find("ramp") != -1) {	
			DemoMesh* const mesh = entity->GetMesh();

			NewtonCollision* const collision = NewtonCreateConvexHull(world, mesh->m_vertexCount, mesh->m_vertex, 3 * sizeof (dFloat), 0, 0, NULL);
			void* const proxy = NewtonSceneCollisionAddSubCollision (sceneCollision, collision);
			NewtonDestroyCollision (collision);

			// get the location of this tire relative to the car chassis
			dMatrix matrix (entity->GetNextMatrix());

			NewtonCollision* const bridgeCollision = NewtonSceneCollisionGetCollisionFromNode (sceneCollision, proxy);
			NewtonSceneCollisionSetSubCollisionMatrix (sceneCollision, proxy, &matrix[0][0]);	
			NewtonCollisionSetUserData(bridgeCollision, entity);
		}
	}


	// add all the planks that form the bridge
	dTree<NewtonBody*, dString> planks;
	for (DemoEntityManager::dListNode* node = bridgeNodes->GetNext(); node; node = node->GetNext()) {
		DemoEntity* const entity = node->GetInfo();
		if (entity->GetName().Find("plank") != -1) {	
			DemoMesh* const mesh = entity->GetMesh();
			NewtonCollision* const collision = NewtonCreateConvexHull(world, mesh->m_vertexCount, mesh->m_vertex, 3 * sizeof (dFloat), 0, 0, NULL);
			NewtonBody* const body = CreateSimpleBody (world, entity, 30.0f, entity->GetNextMatrix(), collision, 0);
			NewtonDestroyCollision (collision);
			planks.Insert(body, entity->GetName());
		}
	}

	// connect each plant with a hinge
	// calculate the with of a plank
	dFloat plankwidth = 0.0f;
	dVector planksideDir (1.0f, 0.0f, 0.0f, 0.0f);
	{
		NewtonBody* const body0 = planks.Find("plank01")->GetInfo();
		NewtonBody* const body1 = planks.Find("plank02")->GetInfo();

		dMatrix matrix0;
		dMatrix matrix1;
		NewtonBodyGetMatrix(body0, &matrix0[0][0]);
		NewtonBodyGetMatrix(body1, &matrix1[0][0]);

		planksideDir = matrix1.m_posit - matrix0.m_posit;
		planksideDir = planksideDir.Scale (1.0f / dSqrt (planksideDir % planksideDir));

		dVector dir (matrix0.UnrotateVector(planksideDir));
		NewtonCollision* const shape = NewtonBodyGetCollision(body0);

		dVector p0;
		dVector p1;
		NewtonCollisionSupportVertex(shape, &dir[0], &p0[0]);
		dVector dir1 (dir.Scale (-1.0f));
		NewtonCollisionSupportVertex(shape, &dir1[0], &p1[0]);
		plankwidth = dAbs (0.5f * ((p1 - p0) % dir));
	}


	dTree<NewtonBody*, dString>::Iterator iter (planks);
	iter.Begin();

	dMatrix matrix0;
	NewtonBody* body0 = iter.GetNode()->GetInfo();
	NewtonBodyGetMatrix(body0, &matrix0[0][0]);

	for (iter ++; iter; iter ++) {
		NewtonBody* const body1 = iter.GetNode()->GetInfo();
		
		dMatrix matrix1;
		NewtonBodyGetMatrix(body1, &matrix1[0][0]);

		// calculate the hinge parameter form the matrix location of each plank
		dMatrix pinMatrix0 (GetIdentityMatrix());
		pinMatrix0[0] = pinMatrix0[1] * planksideDir;
		pinMatrix0[0] = pinMatrix0[0].Scale (1.0f / dSqrt (pinMatrix0[0] % pinMatrix0[0]));
		pinMatrix0[2] = pinMatrix0[0] * pinMatrix0[1];
		pinMatrix0[0][3] = 0.0f;
		pinMatrix0[1][3] = 0.0f;
		pinMatrix0[2][3] = 0.0f;

		// calculate the pivot
		pinMatrix0[3] = matrix0.m_posit + pinMatrix0[2].Scale (plankwidth);
		pinMatrix0[3][3] = 1.0f;

		dMatrix pinMatrix1 (pinMatrix0);
		pinMatrix1[3] = matrix1.m_posit - pinMatrix1[2].Scale (plankwidth);
		pinMatrix1[3][3] = 1.0f;

		// connect these two plank by a hinge, there a wiggle space between eh hinge that give therefore use the alternate hinge constructor
		new CustomHinge (pinMatrix0, pinMatrix1, body0, body1);

		body0 = body1;
		matrix0 = matrix1;
	}

	// connect the last and first plank to the bridge base
	{
		iter.Begin();
		body0 = iter.GetNode()->GetInfo();
		NewtonBodyGetMatrix(body0, &matrix0[0][0]);

		dMatrix pinMatrix0 (GetIdentityMatrix());
		pinMatrix0[0] = pinMatrix0[1] * planksideDir;
		pinMatrix0[0] = pinMatrix0[0].Scale (1.0f / dSqrt (pinMatrix0[0] % pinMatrix0[0]));
		pinMatrix0[2] = pinMatrix0[0] * pinMatrix0[1];
		pinMatrix0[0][3] = 0.0f;
		pinMatrix0[1][3] = 0.0f;
		pinMatrix0[2][3] = 0.0f;
		pinMatrix0[3] = matrix0.m_posit - pinMatrix0[2].Scale (plankwidth);

		new CustomHinge (pinMatrix0, body0, playGroundBody);
	}

	{
		iter.End();
		body0 = iter.GetNode()->GetInfo();
		NewtonBodyGetMatrix(body0, &matrix0[0][0]);

		dMatrix pinMatrix0 (GetIdentityMatrix());
		pinMatrix0[0] = pinMatrix0[1] * planksideDir;
		pinMatrix0[0] = pinMatrix0[0].Scale (1.0f / dSqrt (pinMatrix0[0] % pinMatrix0[0]));
		pinMatrix0[2] = pinMatrix0[0] * pinMatrix0[1];
		pinMatrix0[0][3] = 0.0f;
		pinMatrix0[1][3] = 0.0f;
		pinMatrix0[2][3] = 0.0f;
		pinMatrix0[3] = matrix0.m_posit + pinMatrix0[2].Scale (plankwidth);

		new CustomHinge (pinMatrix0, body0, playGroundBody);
	}


}



static void LoadPlayGroundScene(DemoEntityManager* const scene, TriggerManager* const triggerManager)
{
	NewtonWorld* const world = scene->GetNewton();
	//	CreateLevelMesh (scene, "flatPlane.ngd", true);
	//	return;

	// make the body with a dummy null collision, so that we can use for attaching world objects
	dMatrix matrix (GetIdentityMatrix());
	NewtonCollision* const dommyCollision = NewtonCreateNull(world);
	NewtonBody* const playGroundBody = NewtonCreateDynamicBody (world, dommyCollision, &matrix[0][0]);
	NewtonDestroyCollision (dommyCollision);	


	// use a multi shape static scene collision
	NewtonCollision* const sceneCollision = NewtonCreateSceneCollision (world, 0);

	// start adding shapes to this scene collisions 
	NewtonSceneCollisionBeginAddRemove (sceneCollision);

	// populate the scene collision
	{
		// load a flat floor
		LoadFloor(scene, sceneCollision);

		// load a slide platform
		dMatrix slideMatrix(GetIdentityMatrix());
		slideMatrix.m_posit.m_x += 80.0f;
		slideMatrix.m_posit.m_z = -20.0f;
		//LoadSlide(scene, triggerManager, sceneCollision, "slide.ngd", slideMatrix, playGroundBody);
		LoadFerryBridge(scene, triggerManager, sceneCollision, "platformBridge.ngd", slideMatrix, playGroundBody);

		// load another hanging bridge
		dMatrix bridgeMatrix(GetIdentityMatrix());
		bridgeMatrix.m_posit.m_x += 40.0f;
		LoadHangingBridge(scene, triggerManager, sceneCollision, "hangingBridge.ngd", bridgeMatrix, playGroundBody);

		// load another ferry bridge
		bridgeMatrix.m_posit.m_z += 20.0f;
		LoadFerryBridge(scene, triggerManager, sceneCollision, "platformBridge.ngd", bridgeMatrix, playGroundBody);
	}
	// finalize adding shapes to this scene collisions 
	NewtonSceneCollisionEndAddRemove (sceneCollision);

	// attach this collision to the playground Body
	NewtonBodySetCollision(playGroundBody, sceneCollision);


	// set the reference to the visual
	//NewtonBodySetUserData(level, visualMesh);

	// do not forget to release the collision
	NewtonDestroyCollision (sceneCollision);
}



void AdvancedPlayerController (DemoEntityManager* const scene)
{
	// load the sky box
	scene->CreateSkyBox();

	NewtonWorld* const world = scene->GetNewton();

	// create a character controller manager
	AdvancePlayerControllerManager* const playerManager = new AdvancePlayerControllerManager (world);

	// create a scene for player controller to work around
	TriggerManager* const triggerManager = new TriggerManager(world, playerManager);

	// create the background scene
	LoadPlayGroundScene(scene, triggerManager);

	// add main player
	dMatrix location (GetIdentityMatrix());
	location.m_posit.m_x = 45.0f;
	location.m_posit.m_y = 5.0f;
	location.m_posit.m_z = -10.0f;
	location.m_posit = FindFloor (scene->GetNewton(), location.m_posit, 10.0f);
	AdvancePlayerEntity* const player = new AdvancePlayerEntity (scene, playerManager, 0.5f, 1.9f, location);

	// set as the player with the camera
	playerManager->SetAsPlayer(player);


/*
	//dFloat x0 = location.m_posit.m_x;
	dFloat z0 = location.m_posit.m_z - 6.0f;
	for (int i = 0; i < 5; i ++) {
		location.m_posit.m_x += 2;
		location.m_posit.m_z = z0;
		for (int i = 0; i < 5; i ++) {
			location.m_posit.m_z += 2;
			location.m_posit.m_y = 5.0f;
			location.m_posit = FindFloor (scene->GetNewton(), location.m_posit, 10.0f);
			new AdvancePlayerEntity (scene, manager, 0.5f, 1.9f, location);
		}
	}
*/
	dQuaternion rot;
	dVector origin (-10.0f, 2.0f, 0.0f, 0.0f);
	scene->SetCameraMatrix(rot, origin);


	// configure Game Logic AI agent
//	new AIAgentGameLogic (scene, playerAgent);

//	AddBoxes(system, 10.0f, location, size, 3, 3, 10.0f, _SPHERE_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
//	AddBoxes(system, 10.0f, location, size, 3, 3, 10.0f, _BOX_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
//	AddBoxes(system, 10.0f, location, size, 3, 3, 10.0f, _CONE_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
//	AddBoxes(system, 10.0f, location, size, 3, 3, 10.0f, _CYLINDER_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
//	AddBoxes(system, 10.0f, location, size, 3, 3, 10.0f, _CAPSULE_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
//	AddBoxes(system, 10.0f, location, size, 3, 3, 10.0f, _CHAMFER_CYLINDER_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
//	AddBoxes(system, 10.0f, location, size, 3, 3, 10.0f, _RANDOM_CONVEX_HULL_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
//	AddBoxes(system, 10.0f, location, size, 3, 3, 10.0f, _REGULAR_CONVEX_HULL_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
//	InitEyePoint (dVector (1.0f, 0.0f, 0.0f), location.m_posit);
}





