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
#include "DemoMesh.h"
#include "DemoCamera.h"
#include "DebugDisplay.h"
#include "DemoEntityManager.h"


#define PLAYER_MASS						80.0f
#define PLAYER_WALK_SPEED				4.0f
#define PLAYER_JUMP_SPEED				6.0f
#define PLAYER_THIRD_PERSON_VIEW_DIST	8.0f


// define the massage we will use for this Game
#define ENTER_TRIGGER					1
#define EXIT_TRIGGER					2
#define INSIDE_TRIGGER					3

#define PLAYER_CALL_FERRY_DRIVER		4
#define PLAYER_BOARDED_FERRY_DRIVER		5


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
			switch (event) 
			{
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

	class InputRecord
	{
		public:
		InputRecord()
		{
			memset (this, 0, sizeof (InputRecord));
		}

		dFloat m_headinAngle;
		dFloat m_forwarSpeed;
		dFloat m_strafeSpeed;
		dFloat m_jumpSpeed;
		int m_cameraMode;
	};

	AdvancePlayerEntity (DemoEntityManager* const scene, CustomPlayerControllerManager* const manager, dFloat radius, dFloat height, const dMatrix& location)
		:DemoEntity (dGetIdentityMatrix(), NULL)
		,m_inputs()
		,m_currentTrigger(NULL)
		,m_controller(NULL) 
		,m_currentPlatform(NULL)
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

		// players by default have the origin at the center of mass of the collision shape.
		// you can change this by calling SetPlayerOrigin, for example if a player has it origin at the center of the AABB you can call 
		m_controller->SetPlayerOrigin (height * 0.5f);

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

		SetMesh(geometry, dGetIdentityMatrix());
		geometry->Release(); 
	}

	~AdvancePlayerEntity ()
	{
		// destroy the player controller and its rigid body
		((CustomPlayerControllerManager*)m_controller->GetManager())->DestroyController(m_controller);
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

	InputRecord	m_inputs;
	NewtonBody* m_currentTrigger;
	CustomPlayerController* m_controller; 
	PlaformEntityEntity* m_currentPlatform;
};

class AdvancePlayerControllerManager: public CustomPlayerControllerManager
{
	public:
	AdvancePlayerControllerManager (NewtonWorld* const world)
		:CustomPlayerControllerManager (world)
	{
	}

	~AdvancePlayerControllerManager ()
	{
	}

	// apply gravity 
	virtual void ApplyPlayerMove (CustomPlayerController* const controller, dFloat timestep)
	{
		NewtonBody* const body = controller->GetBody();
		AdvancePlayerEntity* const player = (AdvancePlayerEntity*) NewtonBodyGetUserData(body);
		
		// calculate desired linear and angular velocity form the input
		dVector gravity (0.0f, DEMO_GRAVITY, 0.0f, 0.0f);

		// set player linear and angular velocity
		controller->SetPlayerVelocity (player->m_inputs.m_forwarSpeed, player->m_inputs.m_strafeSpeed, player->m_inputs.m_jumpSpeed, player->m_inputs.m_headinAngle, gravity, timestep);
	}


	virtual int ProcessContacts (const CustomPlayerController* const controller, NewtonWorldConvexCastReturnInfo* const contacts, int count) const 
	{
		// here you need to process the contact and reject the one you do not need.
		//there are different ways to do this:
		// 1-by assigning a collision ID to the colliding shape in the contact
		// 2-by using the material system 
		// 3-by getting the user data from the shape or from the body and using any labeling system of the client application 
		// 4- by using any king of heuristic
		// if a contact is rejected then the [alway body will client with the object in the background and the collision system will take place 
		// for simplicity I will use the heuristic that I will let the  

		int newCount = count;
		for (int i = count - 1; i >= 0; i --) {
			const NewtonBody* const hitBody = contacts[i].m_hitBody;
			dFloat Ixx;
			dFloat Iyy;
			dFloat Izz;
			dFloat mass;
			NewtonBodyGetMass(hitBody, &mass, &Ixx, &Iyy, &Izz);
			if (mass > 0.0f) {
				contacts[i] = contacts[newCount - 1];
				newCount --;
			}
		}
		count = CustomPlayerControllerManager::ProcessContacts (controller, contacts, newCount); 
		return count;
	}
};


// we recommend using and input manage to control input for all games
class AdvancedPlayerInputManager: public CustomInputManager
{
	public:
	AdvancedPlayerInputManager (DemoEntityManager* const scene)
		:CustomInputManager(scene->GetNewton())
		,m_scene(scene)
		,m_player(NULL)
		,m_jumpKey(false)
		,m_cameraMode(true)
		,m_helpKey(true)
		,m_shootProp(false)
		,m_shootState(0)
	{
		// plug a callback for 2d help display
		scene->Set2DDisplayRenderFunction (RenderPlayerHelp, this);
	}

	void SpawnRandomProp(const dMatrix& location) const
	{
		NewtonWorld* const world = GetWorld();
		DemoEntityManager* const scene = (DemoEntityManager*) NewtonWorldGetUserData(world);
		//scene->SetCurrent();

		static PrimitiveType proSelection[] = {_SPHERE_PRIMITIVE, _BOX_PRIMITIVE, _CAPSULE_PRIMITIVE, _CYLINDER_PRIMITIVE, _CONE_PRIMITIVE, 
											   _CHAMFER_CYLINDER_PRIMITIVE, _RANDOM_CONVEX_HULL_PRIMITIVE, _REGULAR_CONVEX_HULL_PRIMITIVE};

		PrimitiveType type = PrimitiveType (dRand() % (sizeof (proSelection) / sizeof (proSelection[0])));

		//dVector size (0.35f, 0.25f, 0.25f, 0.0f);
		dVector size (2.0f, 2.0f, 2.0f, 0.0f);
		NewtonCollision* const collision = CreateConvexCollision (world, dGetIdentityMatrix(), size, type, 0);
		DemoMesh* const geometry = new DemoMesh("prop", collision, "smilli.tga", "smilli.tga", "smilli.tga");

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
        if (m_player) {
		    AdvancePlayerEntity::InputRecord inputs;

		    DemoCamera* const camera = m_scene->GetCamera();
		    NewtonDemos* const mainWindow = m_scene->GetRootWindow();

		    // set the help key
		    m_helpKey.UpdatePushButton (mainWindow, 'H');

		    // read the player inputs
		    inputs.m_headinAngle = camera->GetYawAngle();
		    inputs.m_cameraMode = m_cameraMode.UpdatePushButton(m_scene->GetRootWindow(), 'C') ? 1 : 0;
		    inputs.m_forwarSpeed = (int (mainWindow->GetKeyState ('W')) - int (mainWindow->GetKeyState ('S'))) * PLAYER_WALK_SPEED;
		    inputs.m_strafeSpeed = (int (mainWindow->GetKeyState ('D')) - int (mainWindow->GetKeyState ('A'))) * PLAYER_WALK_SPEED;
		    inputs.m_jumpSpeed = (m_jumpKey.UpdateTriggerButton(mainWindow, ' ')) ? PLAYER_JUMP_SPEED : 0.0f;

		    // normalize player speed
		    dFloat mag2 = inputs.m_forwarSpeed * inputs.m_forwarSpeed + inputs.m_strafeSpeed * inputs.m_strafeSpeed;
		    if (mag2 > 0.0f) {
			    dFloat invMag = PLAYER_WALK_SPEED / dSqrt (mag2);
			    inputs.m_forwarSpeed *= invMag;
			    inputs.m_strafeSpeed *= invMag;
		    }

		    // see if we are shotting some props
		    m_shootState = m_shootProp.UpdateTriggerButton(mainWindow, 0x0d) ? 1 : 0;


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
        }
	}

	void OnEndUpdate (dFloat timestepInSecunds)
	{
        if (m_player) {
		    DemoCamera* const camera = m_scene->GetCamera();

		    dMatrix camMatrix(camera->GetNextMatrix());
		    dMatrix playerMatrix (m_player->GetNextMatrix());

		    dVector frontDir (camMatrix[0]);

		    CustomPlayerController* const controller = m_player->m_controller; 
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
	}

	void AddPlayer (AdvancePlayerEntity* const player)
	{
		m_player = player;
	}

	void RenderPlayerHelp (DemoEntityManager* const scene, int lineNumber) const
	{
		if (m_helpKey.GetPushButtonState()) {
			dVector color(1.0f, 1.0f, 0.0f, 0.0f);
			lineNumber = scene->Print (color, 10, lineNumber + 20, "Navigation Keys");
			lineNumber = scene->Print (color, 10, lineNumber + 20, "walk forward:            W");
			lineNumber = scene->Print (color, 10, lineNumber + 20, "walk backward:           S");
			lineNumber = scene->Print (color, 10, lineNumber + 20, "strafe right:            D");
			lineNumber = scene->Print (color, 10, lineNumber + 20, "strafe left:             A");
			lineNumber = scene->Print (color, 10, lineNumber + 20, "toggle camera mode:      C");
			lineNumber = scene->Print (color, 10, lineNumber + 20, "jump:                    Space");
			lineNumber = scene->Print (color, 10, lineNumber + 20, "throw solids at player:  Enter");
			lineNumber = scene->Print (color, 10, lineNumber + 20, "hide help:               H");
			lineNumber = scene->Print (color, 10, lineNumber + 20, "change player direction: Left mouse button");
		}
	}

	static void RenderPlayerHelp (DemoEntityManager* const scene, void* const context, int lineNumber)
	{
		AdvancedPlayerInputManager* const me = (AdvancedPlayerInputManager*) context;
		me->RenderPlayerHelp (scene, lineNumber);
	}


	DemoEntityManager* m_scene;
	AdvancePlayerEntity* m_player;
	DemoEntityManager::ButtonKey m_jumpKey;
	DemoEntityManager::ButtonKey m_cameraMode;
	DemoEntityManager::ButtonKey m_helpKey;
	DemoEntityManager::ButtonKey m_shootProp;

	int m_shootState;
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
			,m_target(0.0f)
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
			dVector com(0.0f);
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

		DemoMesh* const mesh = (DemoMesh*)source->GetMesh();
		dAssert (mesh->IsType(DemoMesh::GetRttiType()));

		SetMesh(mesh, source->GetMeshMatrix());

		const dFloat mass = 100.0f;
		dMatrix matrix (source->GetNextMatrix()) ;
		NewtonWorld* const world = scene->GetNewton();

		// note: because the mesh matrix can have scale, for simplicity just apply the local mesh matrix to the vertex cloud
		dFloat pool[128][3];
		const dMatrix& meshMatrix = GetMeshMatrix();
		meshMatrix.TransformTriplex(&pool[0][0], 3 * sizeof (dFloat), mesh->m_vertex, 3 * sizeof (dFloat), mesh->m_vertexCount);
		NewtonCollision* const collision = NewtonCreateConvexHull(world, mesh->m_vertexCount, &pool[0][0], 3 * sizeof (dFloat), 0, 0, NULL);

		NewtonBody* body = CreateSimpleBody (world, this, 100, matrix, collision, 0);
		NewtonDestroyCollision(collision);


		// attach a kinematic joint controller joint to move this body 
		dVector pivot(0.0f);
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


//#define SPEK_CRASH_TEST
#ifdef SPEK_CRASH_TEST

// Make a 2-triangle flat test floor
NewtonCollision* testCreateFlatFloor(DemoEntityManager* const scene)
{
   NewtonCollision* tree = NewtonCreateTreeCollision(scene->GetNewton(), 0);
   NewtonTreeCollisionBeginBuild(tree);

   dFloat face[3][3];
   face[0][0] = +50; face[0][1] = 0; face[0][2] = -50;
   face[1][0] = -50; face[1][1] = 0; face[1][2] = -50;
   face[2][0] = -50; face[2][1] = 0; face[2][2] = +50;
// the stride in byte is the size of the vertex not the side of the array
   NewtonTreeCollisionAddFace(tree, 3, &face[0][0], 3 * sizeof (dFloat), 0);

   face[0][0] = +50; face[0][1] = 0; face[0][2] = -50;
   face[1][0] = -50; face[1][1] = 0; face[1][2] = +50;
   face[2][0] = +50; face[2][1] = 0; face[2][2] = +50;
   NewtonTreeCollisionAddFace(tree, 3, &face[0][0], 3 * sizeof (dFloat), 0);

   NewtonTreeCollisionEndBuild( tree, 0);

   return tree;
} // testCreateFlatFloor


static void LoadFloor(DemoEntityManager* const scene, NewtonCollision* const sceneCollision)
{
   NewtonWorld* const world = scene->GetNewton();

   // add a flat plane
   dMatrix matrix(dGetIdentityMatrix());
   DemoEntityManager::dListNode* const floorNode = LoadScene(scene, "flatPlane.ngd", matrix);

   DemoEntity* const entity = floorNode->GetInfo();
   DemoMesh* const mesh = (DemoMesh*)entity->GetMesh();
   dAssert (mesh->IsType(DemoMesh::GetRttiType()));

   // Replace original code with testFloor
   NewtonCollision* const tree = testCreateFlatFloor(scene);

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
#else 

static void LoadFloor(DemoEntityManager* const scene, NewtonCollision* const sceneCollision)
{
	NewtonWorld* const world = scene->GetNewton();

	// add a flat plane
	dMatrix matrix (dGetIdentityMatrix());
	DemoEntityManager::dListNode* const floorNode = LoadScene(scene, "flatPlane.ngd", matrix);

	DemoEntity* const entity = floorNode->GetInfo();
	DemoMesh* const mesh = (DemoMesh*)entity->GetMesh();
	dAssert (mesh->IsType(DemoMesh::GetRttiType()));

	NewtonCollision* const tree = NewtonCreateTreeCollision(world, 0);
	NewtonTreeCollisionBeginBuild(tree);

	dFloat* const vertex = mesh->m_vertex;
	for (DemoMesh::dListNode* node = mesh->GetFirst(); node; node = node->GetNext()){
		DemoSubMesh* const subMesh = &node->GetInfo();
		unsigned int* const indices = subMesh->m_indexes;
		int trianglesCount = subMesh->m_indexCount;
		for (int i = 0; i < trianglesCount; i += 3) {

			dFloat face[3][3];
			for (int j = 0; j < 3; j ++) {
				int index = indices[i + j] * 3;
				face[j][0] = vertex[index + 0];
				face[j][1] = vertex[index + 1];
				face[j][2] = vertex[index + 2];
			}
//			index = indices[i + 1] * 3;
//			face[1] = dVector (vertex[index + 0], vertex[index + 1], vertex[index + 2]);

//			index = indices[i + 2] * 3;
//			face[2] = dVector (vertex[index + 0], vertex[index + 1], vertex[index + 2]);

			int matID = 0;
			//matID = matID == 2 ? 1 : 2 ;
			NewtonTreeCollisionAddFace(tree, 3, &face[0][0], 3 * sizeof (dFloat), matID);
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
#endif




static void LoadFerryBridge (DemoEntityManager* const scene, TriggerManager* const triggerManager, NewtonCollision* const sceneCollision, const char* const name, const dMatrix& location, NewtonBody* const playGroundBody)
{
	NewtonWorld* const world = scene->GetNewton();
	DemoEntityManager::dListNode* const bridgeNodes = scene->GetLast();
	LoadScene(scene, name, location);

	// add bridge foundations
	for (DemoEntityManager::dListNode* node = bridgeNodes->GetNext(); node; node = node->GetNext()) {
		DemoEntity* const entity = node->GetInfo();
		if (entity->GetName().Find("ramp") != -1) {
			DemoMesh* const mesh = (DemoMesh*)entity->GetMesh();
			dAssert (mesh->IsType(DemoMesh::GetRttiType()));

			NewtonCollision* const collision = NewtonCreateConvexHull(world, mesh->m_vertexCount, mesh->m_vertex, 3 * sizeof (dFloat), 0, 0, NULL);
			void* const proxy = NewtonSceneCollisionAddSubCollision (sceneCollision, collision);
			NewtonDestroyCollision (collision);

			// set the matrix for this sub shape
			dMatrix matrix (entity->GetMeshMatrix() * entity->GetNextMatrix());
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
			DemoMesh* const mesh = (DemoMesh*)entity->GetMesh();
			dAssert (mesh->IsType(DemoMesh::GetRttiType()));

			const dMatrix& meshMatrix = entity->GetMeshMatrix();
			// create a trigger to match his mesh
			NewtonCollision* const collision = NewtonCreateConvexHull(world, mesh->m_vertexCount, mesh->m_vertex, 3 * sizeof (dFloat), 0, 0, &meshMatrix[0][0]);
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

/*
static void LoadSlide (DemoEntityManager* const scene, TriggerManager* const triggerManager, NewtonCollision* const sceneCollision, const char* const name, const dMatrix& location, NewtonBody* const playGroundBody)
{
	NewtonWorld* const world = scene->GetNewton();
	DemoEntityManager::dListNode* const bridgeNodes = scene->GetLast();
	LoadScene(scene, name, location);

	// add bridge foundations
	for (DemoEntityManager::dListNode* node = bridgeNodes->GetNext(); node; node = node->GetNext()) {
		DemoEntity* const entity = node->GetInfo();
		if (entity->GetName().Find("ramp") != -1) {
			DemoMesh* const mesh = (DemoMesh*)entity->GetMesh();
			dAssert (mesh->IsType(DemoMesh::GetRttiType()));

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
			DemoMesh* const mesh = (DemoMesh*)entity->GetMesh();
			dAssert (mesh->IsType(DemoMesh::GetRttiType()));

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
*/

static void LoadHangingBridge (DemoEntityManager* const scene, TriggerManager* const triggerManager, NewtonCollision* const sceneCollision, const char* const name, const dMatrix& location, NewtonBody* const playGroundBody)
{
	NewtonWorld* const world = scene->GetNewton();
	DemoEntityManager::dListNode* const bridgeNodes = scene->GetLast();
	LoadScene(scene, name, location);

	// add bridge foundations
	for (DemoEntityManager::dListNode* node = bridgeNodes->GetNext(); node; node = node->GetNext()) {
		DemoEntity* const entity = node->GetInfo();
		if (entity->GetName().Find("ramp") != -1) {	
			DemoMesh* const mesh = (DemoMesh*)entity->GetMesh();
			dAssert (mesh->IsType(DemoMesh::GetRttiType()));

			
			NewtonCollision* const collision = NewtonCreateConvexHull(world, mesh->m_vertexCount, mesh->m_vertex, 3 * sizeof (dFloat), 0, 0, NULL);
			void* const proxy = NewtonSceneCollisionAddSubCollision (sceneCollision, collision);
			NewtonDestroyCollision (collision);

			// get the location of this tire relative to the car chassis
			dMatrix matrix (entity->GetMeshMatrix() * entity->GetNextMatrix());

			NewtonCollision* const bridgeCollision = NewtonSceneCollisionGetCollisionFromNode (sceneCollision, proxy);
			NewtonSceneCollisionSetSubCollisionMatrix (sceneCollision, proxy, &matrix[0][0]);	
			NewtonCollisionSetUserData(bridgeCollision, entity);
		}
	}

	// add all the planks that form the bridge
	dTree<NewtonBody*, dString> planks;
	dFloat plankMass = 30.0f;
	for (DemoEntityManager::dListNode* node = bridgeNodes->GetNext(); node; node = node->GetNext()) {
		DemoEntity* const entity = node->GetInfo();
		if (entity->GetName().Find("plank") != -1) {	
			DemoMesh* const mesh = (DemoMesh*)entity->GetMesh();
			dAssert (mesh->IsType(DemoMesh::GetRttiType()));

			// note: because the mesh matrix can have scale, for simplicity just apply the local mesh matrix to the vertex cloud
			dVector pool[128];
			const dMatrix& meshMatrix = entity->GetMeshMatrix();
			meshMatrix.TransformTriplex(&pool[0].m_x, sizeof (dVector), mesh->m_vertex, 3 * sizeof (dFloat), mesh->m_vertexCount);

			NewtonCollision* const collision = NewtonCreateConvexHull(world, mesh->m_vertexCount, &pool[0].m_x, sizeof (dVector), 0, 0, NULL);
			NewtonBody* const body = CreateSimpleBody (world, entity, plankMass, entity->GetNextMatrix(), collision, 0);
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

		dVector p0(0.0f);
		dVector p1(0.0f);
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

	int jointCount = 0; 
	NewtonJoint* jointArray[256];
	// for more accuracy, wrap the bridge in a Newton skeleton 
	NewtonSkeletonContainer* const skeleton = NewtonSkeletonContainerCreate(scene->GetNewton(), body0, NULL);
	for (iter ++; iter; iter ++) {
		NewtonBody* const body1 = iter.GetNode()->GetInfo();
		
		dMatrix matrix1;
		NewtonBodyGetMatrix(body1, &matrix1[0][0]);

		// calculate the hinge parameter form the matrix location of each plank
		dMatrix pinMatrix0 (dGetIdentityMatrix());
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
		CustomHinge* const joint = new CustomHinge (pinMatrix0, pinMatrix1, body0, body1);
		joint->SetFriction(1000.0f);

		// save the joint for later used
		jointArray[jointCount] = joint->GetJoint();
		jointCount ++;

		body0 = body1;
		matrix0 = matrix1;
	}

	// add all the joint to the Newton Skeleton 
	NewtonSkeletonContainerAttachJointArray (skeleton, jointCount, jointArray);

	// complete the skeleton construction
	NewtonSkeletonContainerFinalize(skeleton);
	
	// connect the last and first plank to the bridge base
	{
		iter.Begin();
		body0 = iter.GetNode()->GetInfo();
		NewtonBodyGetMatrix(body0, &matrix0[0][0]);

		dMatrix pinMatrix0 (dGetIdentityMatrix());
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

		dMatrix pinMatrix0 (dGetIdentityMatrix());
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
	dMatrix matrix (dGetIdentityMatrix());
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

#ifndef SPEK_CRASH_TEST
		// load a slide platform
		dMatrix slideMatrix(dGetIdentityMatrix());
		slideMatrix.m_posit.m_x += 80.0f;
		slideMatrix.m_posit.m_z = -20.0f;
		//LoadSlide(scene, triggerManager, sceneCollision, "slide.ngd", slideMatrix, playGroundBody);
		LoadFerryBridge(scene, triggerManager, sceneCollision, "platformBridge.ngd", slideMatrix, playGroundBody);

		// load another hanging bridge
		dMatrix bridgeMatrix(dGetIdentityMatrix());
		bridgeMatrix.m_posit.m_x += 40.0f;
		LoadHangingBridge(scene, triggerManager, sceneCollision, "hangingBridge.ngd", bridgeMatrix, playGroundBody);

		// load another ferry bridge
		bridgeMatrix.m_posit.m_z += 20.0f;
		LoadFerryBridge(scene, triggerManager, sceneCollision, "platformBridge.ngd", bridgeMatrix, playGroundBody);
#endif	
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

	// add an input Manage to manage the inputs and user interaction 
	AdvancedPlayerInputManager* const inputManager = new AdvancedPlayerInputManager (scene);

	// create a character controller manager
	AdvancePlayerControllerManager* const playerManager = new AdvancePlayerControllerManager (world);

	// create a scene for player controller to work around
	TriggerManager* const triggerManager = new TriggerManager(world, playerManager);

	// create the background scene
	LoadPlayGroundScene(scene, triggerManager);

	// add main player
	dMatrix location (dGetIdentityMatrix());
	location.m_posit.m_x = 45.0f;
	location.m_posit.m_y = 5.0f;
	location.m_posit.m_z = -10.0f;
	location.m_posit = FindFloor (scene->GetNewton(), location.m_posit, 10.0f);
	AdvancePlayerEntity* const player = new AdvancePlayerEntity (scene, playerManager, 0.5f, 1.9f, location);

	// set as the player with the camera
	inputManager->AddPlayer(player);

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





