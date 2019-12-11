/* Copyright (c) <2003-2019> <Newton Game Dynamics>
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
#include "DemoMesh.h"
#include "DemoCamera.h"
#include "PhysicsUtils.h"
#include "TargaToOpenGl.h"
#include "DemoEntityManager.h"
#include "DebugDisplay.h"
#include "HeightFieldPrimitive.h"

#if 0
#define SERVO_VEHICLE_CAMERA_EYEPOINT			1.5f
#define SERVO_VEHICLE_CAMERA_HIGH_ABOVE_HEAD	2.0f
#define SERVO_VEHICLE_CAMERA_DISTANCE			8.0f

struct SERVO_VEHICLE_DEFINITION
{
	enum SHAPES_ID
	{
		m_terrain		= 1<<0,
		m_landPart		= 1<<1,
		m_bodyPart		= 1<<2,
		m_linkPart		= 1<<3,
		m_tirePart		= 1<<4,
		m_tireInnerRing = 1<<5,
	};

	char m_boneName[32];
	char m_shapeTypeName[32];
	dFloat m_mass;
	int m_bodyPartID;
	char m_articulationName[32];
};


static SERVO_VEHICLE_DEFINITION inverseKinematicsRidParts[] =
{
	{"body",		"convexHull",		   4096.0f, SERVO_VEHICLE_DEFINITION::m_bodyPart, "mainBody"},
	{"fr_tire",		"tireShape",			 64.0f, SERVO_VEHICLE_DEFINITION::m_tirePart, "frontTire"},
	{"fl_tire",		"tireShape",			 64.0f, SERVO_VEHICLE_DEFINITION::m_tirePart, "frontTire"},
	{"rr_tire",		"tireShape",			 64.0f, SERVO_VEHICLE_DEFINITION::m_tirePart, "rearTire"},
	{"rl_tire",		"tireShape",			 64.0f, SERVO_VEHICLE_DEFINITION::m_tirePart, "rearTire"},
	{"lift_1",		"convexHull",			 50.0f, SERVO_VEHICLE_DEFINITION::m_bodyPart, "hingeActuator"},
	{"lift_2",		"convexHull",			 40.0f, SERVO_VEHICLE_DEFINITION::m_bodyPart, "liftActuator"},
	{"lift_3",		"convexHull",			 30.0f, SERVO_VEHICLE_DEFINITION::m_bodyPart, "liftActuator"},
	{"lift_4",		"convexHull",			 20.0f, SERVO_VEHICLE_DEFINITION::m_bodyPart, "liftActuator"},
	{"left_teeth",  "convexHullAggregate",	 10.0f, SERVO_VEHICLE_DEFINITION::m_bodyPart, "paletteActuator"},
	{"right_teeth", "convexHullAggregate",	 10.0f, SERVO_VEHICLE_DEFINITION::m_bodyPart, "paletteActuator"},
};

class dLifterUserData: public DemoEntity::UserData
{
	public:
	class InputRecord
	{
		public:
		InputRecord()
		{
			memset(this, 0, sizeof(InputRecord));
		}

		dFloat m_liftValue;
		dFloat m_forkValue;
		dFloat m_paletteValue;
		int m_steerValue;
		int m_throttleValue;
	};


	dLifterUserData(DemoEntity* const vehicle)
		:DemoEntity::UserData()
		,m_inputs()
		,m_engineMotor(NULL)
		,m_engineJoint(NULL)
		,m_forkBase(NULL)
		,m_maxEngineSpeed(20.0f)
	{
		m_liftJoints[0] = NULL;
		m_liftJoints[1] = NULL;
		m_liftJoints[2] = NULL;
		m_paletteJoints[0] = NULL;
		m_paletteJoints[1] = NULL;
		m_rearTireJoints[0] = NULL;
		m_rearTireJoints[1] = NULL;
		m_frontTiresJoints[0] = NULL;
		m_frontTiresJoints[1] = NULL;
	}

	void OnRender(dFloat timestep) const
	{
	}

	void SetInput(const InputRecord& inputs)
	{
		m_inputs = inputs;
	}

	InputRecord m_inputs;
	dCustomMotor* m_engineMotor;
	dCustomDoubleHinge* m_engineJoint;
	dCustomHingeActuator* m_forkBase;
	dCustomWheel* m_rearTireJoints[2];
	dCustomWheel* m_frontTiresJoints[2];
	dCustomSliderActuator* m_liftJoints[3];
	dCustomSliderActuator* m_paletteJoints[2];
	dFloat m_maxEngineSpeed;
};

class ServoInputManager: public dCustomListener
{
	public:
	class InputRecord
	{
		public:
		InputRecord()
		{
			memset(this, 0, sizeof(InputRecord));
		}

		dFloat m_liftValue;
		dFloat m_forkValue;
		dFloat m_paletteValue;
		int m_steerValue;
		int m_throttleValue;
	};

	ServoInputManager(DemoEntityManager* const scene)
		:dCustomListener(scene->GetNewton(), "D_LISTENER")
		,m_scene(scene)
		,m_cameraMode(true)
		,m_changeVehicle(true)
		,m_palette(0.0f)
		,m_forkAngle(0.0f)
		,m_listPosit(0.0f)
		,m_playersCount(0)
		,m_currentPlayer(0)
		,m_needsWakeUp(false)
	{
		// plug a callback for 2d help display
		scene->Set2DDisplayRenderFunction(RenderPlayerHelp, NULL, this);
		scene->SetUpdateCameraFunction(UpdateCameraCallback, this);
		memset (m_player, 0, sizeof (m_player));
	}

	void PreUpdate(dFloat timestepInSecunds)
	{
		if (!(m_playersCount && m_player[m_currentPlayer % m_playersCount])) {
			return ;
		}
		dLifterUserData::InputRecord inputs;
		DemoEntity* const entity = (DemoEntity*)m_player[m_currentPlayer % m_playersCount]->GetUserData();
		dLifterUserData* const lifterData = (dLifterUserData*)(entity->GetUserData());

		inputs.m_steerValue = int(m_scene->GetKeyState('D')) - int(m_scene->GetKeyState('A'));
		inputs.m_throttleValue = int(m_scene->GetKeyState('W')) - int(m_scene->GetKeyState('S'));

		inputs.m_paletteValue = -m_palette;
		inputs.m_liftValue = m_listPosit;
		inputs.m_forkValue = m_forkAngle * dDegreeToRad;

		// check if we must activate the player
		if (m_needsWakeUp ||
			m_scene->GetKeyState('A') ||
			m_scene->GetKeyState('D') ||
			m_scene->GetKeyState('W') ||
			m_scene->GetKeyState('S')) {
			NewtonBody* const body = m_player[m_currentPlayer % m_playersCount]->GetBody();
			NewtonBodySetSleepState(body, false);
		}

#if 0
	#if 0
			static FILE* file = fopen("log.bin", "wb");
			if (file) {
				fwrite(&inputs, sizeof(inputs), 1, file);
				fflush(file);
			}
	#else 
			static FILE* file = fopen("log.bin", "rb");
			if (file) {
				fread(&inputs, sizeof(inputs), 1, file);
			}
	#endif
#endif
		lifterData->SetInput(inputs);
	}

	static void UpdateCameraCallback(DemoEntityManager* const manager, void* const context, dFloat timestep)
	{
		ServoInputManager* const me = (ServoInputManager*)context;
		me->UpdateCamera(timestep);
	}

	void UpdateCamera(dFloat timestepInSecunds)
	{
		if (!(m_playersCount && m_player[m_currentPlayer % m_playersCount])) {
			return;
		}

		DemoCamera* const camera = m_scene->GetCamera();
		DemoEntity* const entity = (DemoEntity*)m_player[m_currentPlayer % m_playersCount]->GetUserData();

		if (m_changeVehicle.UpdateTrigger(m_scene->GetKeyState('P'))) {
			m_currentPlayer++;
		}

		dMatrix camMatrix(camera->GetNextMatrix());
		dMatrix playerMatrix(entity->GetNextMatrix());

		dVector frontDir(camMatrix[0]);
		dVector camOrigin(0.0f);
		m_cameraMode.UpdatePushButton(m_scene->GetKeyState('C'));
		if (m_cameraMode.GetPushButtonState()) {
			camOrigin = playerMatrix.TransformVector(dVector(0.0f, SERVO_VEHICLE_CAMERA_HIGH_ABOVE_HEAD, 0.0f, 0.0f));
			camOrigin -= frontDir.Scale(SERVO_VEHICLE_CAMERA_DISTANCE);
		} else {
			camMatrix = camMatrix * playerMatrix;
			camOrigin = playerMatrix.TransformVector(dVector(-0.8f, SERVO_VEHICLE_CAMERA_EYEPOINT, 0.0f, 0.0f));
		}

		camera->SetNextMatrix(*m_scene, camMatrix, camOrigin);
	}

	void OnEndUpdate(dFloat timestepInSecunds)
	{
	}

	void AddPlayer(dModelRootNode* const player)
	{
		m_player[m_playersCount] = player;
		m_playersCount++;
	}

	void RenderPlayerHelp(DemoEntityManager* const scene)
	{
		dVector color(1.0f, 1.0f, 0.0f, 0.0f);
		scene->Print(color, "Navigation Keys");
		scene->Print(color, "drive forward:      W");
		scene->Print(color, "drive backward:     S");
		scene->Print(color, "turn right:         D");
		scene->Print(color, "turn left:          A");

		m_needsWakeUp = false;
		m_needsWakeUp = ImGui::SliderFloat("lift", &m_listPosit, -0.5f, 1.5f) || m_needsWakeUp;
		m_needsWakeUp = ImGui::SliderFloat("tilt", &m_forkAngle, -30.0f, 30.0f) || m_needsWakeUp;
		m_needsWakeUp = ImGui::SliderFloat("palette", &m_palette, -0.2f, 0.6f) || m_needsWakeUp;
	}

	static void RenderPlayerHelp(DemoEntityManager* const scene, void* const context)
	{
		ServoInputManager* const me = (ServoInputManager*)context;
		me->RenderPlayerHelp(scene);
	}

	DemoEntityManager* m_scene;
	dModelRootNode* m_player[2];
	DemoEntityManager::ButtonKey m_cameraMode;
	DemoEntityManager::ButtonKey m_changeVehicle;
	dFloat32 m_palette;
	dFloat32 m_forkAngle;
	dFloat32 m_listPosit;
	int m_playersCount;
	int m_currentPlayer;
	bool m_needsWakeUp;
};


class ServoVehicleManagerManager: public dModelManager
{
	public:
	ServoVehicleManagerManager(DemoEntityManager* const scene)
		:dModelManager(scene->GetNewton())
	{
		// create a material for early collision culling
		int material = NewtonMaterialGetDefaultGroupID(scene->GetNewton());
		NewtonMaterialSetCallbackUserData(scene->GetNewton(), material, material, this);
		NewtonMaterialSetCompoundCollisionCallback(scene->GetNewton(), material, material, CompoundSubCollisionAABBOverlap);
		NewtonMaterialSetCollisionCallback(scene->GetNewton(), material, material, OnBoneAABBOverlap, OnContactsProcess);
	}

	virtual void OnDebug(dCustomJoint::dDebugDisplay* const debugContext)
	{
	}

//	static int OnBoneAABBOverlap(const NewtonMaterial* const material, const NewtonBody* const body0, const NewtonBody* const body1, int threadIndex)
	static int OnBoneAABBOverlap(const NewtonJoint* const contactJoint, dFloat timestep, int threadIndex)
	{
		const NewtonBody* const body0 = NewtonJointGetBody0(contactJoint);
		const NewtonBody* const body1 = NewtonJointGetBody1(contactJoint);
		const NewtonCollision* const collision0 = NewtonBodyGetCollision(body0);
		const NewtonCollision* const collision1 = NewtonBodyGetCollision(body1);
		SERVO_VEHICLE_DEFINITION::SHAPES_ID id0 = SERVO_VEHICLE_DEFINITION::SHAPES_ID(NewtonCollisionGetUserID(collision0));
		SERVO_VEHICLE_DEFINITION::SHAPES_ID id1 = SERVO_VEHICLE_DEFINITION::SHAPES_ID(NewtonCollisionGetUserID(collision1));
		dAssert(id0 != SERVO_VEHICLE_DEFINITION::m_tireInnerRing);
		dAssert(id1 != SERVO_VEHICLE_DEFINITION::m_tireInnerRing);

		switch (id0 | id1) 
		{
			//case SERVO_VEHICLE_DEFINITION::m_linkPart | SERVO_VEHICLE_DEFINITION::m_linkPart:
			//case SERVO_VEHICLE_DEFINITION::m_linkPart | SERVO_VEHICLE_DEFINITION::m_bodyPart:
			case SERVO_VEHICLE_DEFINITION::m_tirePart | SERVO_VEHICLE_DEFINITION::m_bodyPart:
			case SERVO_VEHICLE_DEFINITION::m_tirePart | SERVO_VEHICLE_DEFINITION::m_tirePart:
			case SERVO_VEHICLE_DEFINITION::m_bodyPart | SERVO_VEHICLE_DEFINITION::m_bodyPart:
			{
				return 0;
				break;
			}

			case SERVO_VEHICLE_DEFINITION::m_terrain | SERVO_VEHICLE_DEFINITION::m_bodyPart:
			case SERVO_VEHICLE_DEFINITION::m_terrain | SERVO_VEHICLE_DEFINITION::m_tirePart:
			case SERVO_VEHICLE_DEFINITION::m_terrain | SERVO_VEHICLE_DEFINITION::m_landPart:
			case SERVO_VEHICLE_DEFINITION::m_tirePart | SERVO_VEHICLE_DEFINITION::m_landPart:
			//case SERVO_VEHICLE_DEFINITION::m_terrain | SERVO_VEHICLE_DEFINITION::m_linkPart:
			//case SERVO_VEHICLE_DEFINITION::m_tirePart | SERVO_VEHICLE_DEFINITION::m_bodyPart:
			//case SERVO_VEHICLE_DEFINITION::m_tirePart | SERVO_VEHICLE_DEFINITION::m_linkPart:
			//case SERVO_VEHICLE_DEFINITION::m_tirePart | SERVO_VEHICLE_DEFINITION::m_landPart:
			//case SERVO_VEHICLE_DEFINITION::m_landPart | SERVO_VEHICLE_DEFINITION::m_bodyPart:
			//case SERVO_VEHICLE_DEFINITION::m_landPart | SERVO_VEHICLE_DEFINITION::m_linkPart:
			case SERVO_VEHICLE_DEFINITION::m_landPart | SERVO_VEHICLE_DEFINITION::m_landPart:
			case SERVO_VEHICLE_DEFINITION::m_landPart | SERVO_VEHICLE_DEFINITION::m_bodyPart:
			{
				return 1;
				break;
			}
			default:
			{
				dAssert(0);
				return 1;
			}
		}
	}

	static int CompoundSubCollisionAABBOverlap (const NewtonJoint* const contact, dFloat timestep, const NewtonBody* const body0, const void* const collisionNode0, const NewtonBody* const body1, const void* const collisionNode1, int threadIndex)
	{
		dAssert(collisionNode0);
		NewtonCollision* const collision0 = NewtonCompoundCollisionGetCollisionFromNode (NewtonBodyGetCollision(body0), collisionNode0);
		NewtonCollision* const collision1 = collisionNode1 ? NewtonCompoundCollisionGetCollisionFromNode(NewtonBodyGetCollision(body1), collisionNode1) : NewtonBodyGetCollision(body1);

		SERVO_VEHICLE_DEFINITION::SHAPES_ID id0 = SERVO_VEHICLE_DEFINITION::SHAPES_ID(NewtonCollisionGetUserID(collision0));
		SERVO_VEHICLE_DEFINITION::SHAPES_ID id1 = SERVO_VEHICLE_DEFINITION::SHAPES_ID(NewtonCollisionGetUserID(collision1));
//		dAssert(id0 != SERVO_VEHICLE_DEFINITION::m_tireInnerRing);
//		dAssert(id1 != SERVO_VEHICLE_DEFINITION::m_tireInnerRing);

		switch (id0 | id1)
		{
			//case SERVO_VEHICLE_DEFINITION::m_terrain | SERVO_VEHICLE_DEFINITION::m_linkPart:			
			//case SERVO_VEHICLE_DEFINITION::m_terrain | SERVO_VEHICLE_DEFINITION::m_tireInnerRing:
			//case SERVO_VEHICLE_DEFINITION::m_linkPart | SERVO_VEHICLE_DEFINITION::m_tireInnerRing:
			case SERVO_VEHICLE_DEFINITION::m_terrain | SERVO_VEHICLE_DEFINITION::m_landPart:
			case SERVO_VEHICLE_DEFINITION::m_tirePart | SERVO_VEHICLE_DEFINITION::m_terrain:
			case SERVO_VEHICLE_DEFINITION::m_tirePart | SERVO_VEHICLE_DEFINITION::m_landPart:
			case SERVO_VEHICLE_DEFINITION::m_bodyPart | SERVO_VEHICLE_DEFINITION::m_landPart:
			case SERVO_VEHICLE_DEFINITION::m_tireInnerRing | SERVO_VEHICLE_DEFINITION::m_linkPart:
			{
				return 1;
				break;
			}

			case SERVO_VEHICLE_DEFINITION::m_tirePart | SERVO_VEHICLE_DEFINITION::m_linkPart:
			case SERVO_VEHICLE_DEFINITION::m_tireInnerRing | SERVO_VEHICLE_DEFINITION::m_landPart:
			case SERVO_VEHICLE_DEFINITION::m_tireInnerRing | SERVO_VEHICLE_DEFINITION::m_terrain:
			{
				return 0;
				break;
			}
			default:
			{
				dAssert(0);
				return 0;
			}
		}
	}
	
	static void OnContactsProcess (const NewtonJoint* const contactJoint, dFloat timestep, int threadIndex)
	{
//		dAssert(0);
/*
		int countCount = 0;
		void* contactList[32];

		for (void* contact = NewtonContactJointGetFirstContact(contactJoint); contact; contact = NewtonContactJointGetNextContact(contactJoint, contact)) {
			contactList[countCount] = contact;
			countCount++;
		}

		NewtonBody* const body0 = NewtonJointGetBody0(contactJoint);
		NewtonBody* const body1 = NewtonJointGetBody1(contactJoint);
		for (int i = 0; i < countCount; i ++) {
			NewtonMaterial* const material = NewtonContactGetMaterial (contactList[i]);
			NewtonCollision* const collision0 = NewtonMaterialGetBodyCollidingShape(material, body0);
			NewtonCollision* const collision1 = NewtonMaterialGetBodyCollidingShape(material, body1);

			SERVO_VEHICLE_DEFINITION::SHAPES_ID id0 = SERVO_VEHICLE_DEFINITION::SHAPES_ID(NewtonCollisionGetUserID(collision0));
			SERVO_VEHICLE_DEFINITION::SHAPES_ID id1 = SERVO_VEHICLE_DEFINITION::SHAPES_ID(NewtonCollisionGetUserID(collision1));

			switch (id0 | id1) 
			{
				//case SERVO_VEHICLE_DEFINITION::m_terrain | SERVO_VEHICLE_DEFINITION::m_tireInnerRing:
				//case SERVO_VEHICLE_DEFINITION::m_linkPart | SERVO_VEHICLE_DEFINITION::m_tirePart:
//				{
//					NewtonContactJointRemoveContact(contactJoint, contactList[i]);
//					break;
//				}
				
				case SERVO_VEHICLE_DEFINITION::m_bodyPart | SERVO_VEHICLE_DEFINITION::m_terrain:
				case SERVO_VEHICLE_DEFINITION::m_tirePart | SERVO_VEHICLE_DEFINITION::m_terrain:
				case SERVO_VEHICLE_DEFINITION::m_tirePart | SERVO_VEHICLE_DEFINITION::m_linkPart:
				case SERVO_VEHICLE_DEFINITION::m_linkPart | SERVO_VEHICLE_DEFINITION::m_terrain:
				case SERVO_VEHICLE_DEFINITION::m_linkPart | SERVO_VEHICLE_DEFINITION::m_tireInnerRing:
				case SERVO_VEHICLE_DEFINITION::m_landPart | SERVO_VEHICLE_DEFINITION::m_tirePart:
				case SERVO_VEHICLE_DEFINITION::m_landPart | SERVO_VEHICLE_DEFINITION::m_bodyPart:
				case SERVO_VEHICLE_DEFINITION::m_landPart | SERVO_VEHICLE_DEFINITION::m_linkPart:
				case SERVO_VEHICLE_DEFINITION::m_landPart | SERVO_VEHICLE_DEFINITION::m_terrain:
				case SERVO_VEHICLE_DEFINITION::m_landPart | SERVO_VEHICLE_DEFINITION::m_landPart:
				{
					break;
				}
				default:
				{
					dAssert(0);
				}
			}
		}
*/
	}

	virtual void OnUpdateTransform(const dModelNode* const bone, const dMatrix& localMatrix) const
	{
		NewtonBody* const body = bone->GetBody();
		DemoEntity* const ent = (DemoEntity*)NewtonBodyGetUserData(body);
		if (ent) {
			DemoEntityManager* const scene = (DemoEntityManager*)NewtonWorldGetUserData(NewtonBodyGetWorld(body));

			dQuaternion rot(localMatrix);
			ent->SetMatrix(*scene, rot, localMatrix.m_posit);
		}
	}

	//virtual void OnPreUpdate (dCustomTransformController* const controller, dFloat timestep, int threadIndex) const
	virtual void OnPreUpdate(dModelRootNode* const controller, dFloat timestep) const
	{
		dLifterUserData* const lifterData = (dLifterUserData*) ((DemoEntity*)controller->GetUserData())->GetUserData();
		if (!lifterData) {
			return;
		}
		
		dFloat brakeTorque = 10000.0f;
		if (lifterData->m_engineJoint) {
			dFloat engineRPM = 0.0f;
			if (lifterData->m_inputs.m_throttleValue > 0) {
				brakeTorque = 0.0f;
				engineRPM = -lifterData->m_maxEngineSpeed;
			} else if (lifterData->m_inputs.m_throttleValue < 0) {
				brakeTorque = 0.0f;
				engineRPM = lifterData->m_maxEngineSpeed;
			}

			// apply DC engine torque
			lifterData->m_engineMotor->SetSpeed(engineRPM);
		}

		//apply breaks
		if (lifterData->m_frontTiresJoints[0] && lifterData->m_frontTiresJoints[1]) {
			lifterData->m_frontTiresJoints[0]->SetAngularFriction(brakeTorque);
			lifterData->m_frontTiresJoints[1]->SetAngularFriction(brakeTorque);
		}

		// update steering wheels
		if (lifterData->m_rearTireJoints[0] && lifterData->m_rearTireJoints[1]) {
			dFloat steeringAngle = lifterData->m_rearTireJoints[0]->GetSteerAngle();
			if (lifterData->m_inputs.m_steerValue > 0) {
				steeringAngle = 30.0f * dDegreeToRad;
			} else if (lifterData->m_inputs.m_steerValue < 0) {
				steeringAngle = -30.0f * dDegreeToRad;
			}
			lifterData->m_rearTireJoints[0]->SetAngularFriction(brakeTorque);
			lifterData->m_rearTireJoints[1]->SetAngularFriction(brakeTorque);
			lifterData->m_rearTireJoints[0]->SetTargetSteerAngle(steeringAngle);
			lifterData->m_rearTireJoints[1]->SetTargetSteerAngle(steeringAngle);
		}

		if (lifterData->m_forkBase) {
			lifterData->m_forkBase->SetTargetAngle(lifterData->m_inputs.m_forkValue);
		}
		if (lifterData->m_liftJoints[0]) {
			lifterData->m_liftJoints[0]->SetTargetPosit(lifterData->m_inputs.m_liftValue);
		}
		if (lifterData->m_liftJoints[1]) {
			lifterData->m_liftJoints[1]->SetTargetPosit(lifterData->m_inputs.m_liftValue);
		}
		if (lifterData->m_liftJoints[2]) {
			lifterData->m_liftJoints[2]->SetTargetPosit(lifterData->m_inputs.m_liftValue);
		}
		if (lifterData->m_paletteJoints[0]) {
			lifterData->m_paletteJoints[0]->SetTargetPosit(lifterData->m_inputs.m_paletteValue);
		}
		if (lifterData->m_paletteJoints[1]) {
			lifterData->m_paletteJoints[1]->SetTargetPosit(lifterData->m_inputs.m_paletteValue);
		}
	}

	NewtonCollision* MakeConvexHull(DemoEntity* const bodyPart) const
	{
		dVector points[1024 * 16];

		DemoMesh* const mesh = (DemoMesh*)bodyPart->GetMesh();
		dAssert(mesh->IsType(DemoMesh::GetRttiType()));
		dAssert(mesh->m_vertexCount && (mesh->m_vertexCount < int(sizeof(points) / sizeof(points[0]))));

		// go over the vertex array and find and collect all vertices's weighted by this bone.
		const dFloat* const array = mesh->m_vertex;
		for (int i = 0; i < mesh->m_vertexCount; i++) {
			points[i][0] = array[i * 3 + 0];
			points[i][1] = array[i * 3 + 1];
			points[i][2] = array[i * 3 + 2];
			points[i][3] = 0.0f;
		}
		bodyPart->GetMeshMatrix().TransformTriplex(&points[0][0], sizeof(dVector), &points[0][0], sizeof(dVector), mesh->m_vertexCount);
		return NewtonCreateConvexHull(GetWorld(), mesh->m_vertexCount, &points[0][0], sizeof(dVector), 1.0e-3f, SERVO_VEHICLE_DEFINITION::m_bodyPart, NULL);
	}

	NewtonCollision* MakePalleteShape(DemoEntity* const bodyPart) const
	{
		DemoMesh* const mesh = (DemoMesh*)bodyPart->GetMesh();
		dAssert(mesh->IsType(DemoMesh::GetRttiType()));

		// go over the vertex array and find and collect all vertices's weighted by this bone.
		dVector minExtend(1.0e10f);
		dVector maxExtend(-1.0e10f);
		const dFloat* const array = mesh->m_vertex;
		for (int i = 0; i < mesh->m_vertexCount; i++) {
			minExtend.m_x = dMin(minExtend.m_x, array[i * 3 + 0]);
			minExtend.m_y = dMin(minExtend.m_y, array[i * 3 + 1]);
			minExtend.m_z = dMin(minExtend.m_z, array[i * 3 + 2]);

			maxExtend.m_x = dMax(maxExtend.m_x, array[i * 3 + 0]);
			maxExtend.m_y = dMax(maxExtend.m_y, array[i * 3 + 1]);
			maxExtend.m_z = dMax(maxExtend.m_z, array[i * 3 + 2]);
		}

		dMatrix oririn(dGetIdentityMatrix());
		oririn.m_posit = (minExtend + maxExtend).Scale(0.5f);
		oririn.m_posit.m_w = 1.0f;
		maxExtend -= minExtend;

		oririn.m_posit.m_y -= maxExtend.m_y * 0.43f;
		maxExtend.m_y *= 0.125f;
		return NewtonCreateBox(GetWorld(), maxExtend.m_x, maxExtend.m_y, maxExtend.m_z, SERVO_VEHICLE_DEFINITION::m_bodyPart, &oririn[0][0]);
	}

	NewtonCollision* MakeForkLiftTireShape(DemoEntity* const bodyPart) const
	{
		dFloat radius = 0.0f;
		dFloat maxWidth = 0.0f;
		dFloat minWidth = 0.0f;

		DemoMesh* const mesh = (DemoMesh*)bodyPart->GetMesh();
		dAssert(mesh->IsType(DemoMesh::GetRttiType()));
		const dMatrix& matrix = bodyPart->GetMeshMatrix();
		dFloat* const array = mesh->m_vertex;
		for (int i = 0; i < mesh->m_vertexCount; i++) {
			dVector p(matrix.TransformVector(dVector(array[i * 3 + 0], array[i * 3 + 1], array[i * 3 + 2], 1.0f)));
			maxWidth = dMax(p.m_x, maxWidth);
			minWidth = dMin(p.m_x, minWidth);
			radius = dMax(p.m_y, radius);
		}
		dFloat width = maxWidth - minWidth;
		radius -= width * 0.5f;
		return NewtonCreateChamferCylinder(GetWorld(), radius, width, 0, NULL);
	}

	NewtonBody* CreateBodyPart(DemoEntity* const bodyPart, const SERVO_VEHICLE_DEFINITION& definition)
	{
		NewtonCollision* shape = NULL;
		if (!strcmp(definition.m_shapeTypeName, "tireShape")) {
			shape = MakeForkLiftTireShape(bodyPart);
		} else if (!strcmp(definition.m_shapeTypeName, "convexHull")) {
			shape = MakeConvexHull(bodyPart);
		} else if (!strcmp(definition.m_shapeTypeName, "convexHullAggregate")) {
			shape = MakePalleteShape(bodyPart);
		} else {
			dAssert(0);
		}

		// calculate the bone matrix
		dMatrix matrix(bodyPart->CalculateGlobalMatrix());

		NewtonWorld* const world = GetWorld();

		// create the rigid body that will make this bone
		NewtonBody* const body = NewtonCreateDynamicBody(world, shape, &matrix[0][0]);

		// destroy the collision helper shape 
		NewtonDestroyCollision(shape);

		// get the collision from body
		NewtonCollision* const collision = NewtonBodyGetCollision(body);

		// calculate the moment of inertia and the relative center of mass of the solid
		NewtonBodySetMassProperties(body, definition.m_mass, collision);

		// save the user lifterData with the bone body (usually the visual geometry)
		NewtonBodySetUserData(body, bodyPart);

		// assign a body part id
		NewtonCollisionSetUserID(collision, definition.m_bodyPartID);

		// make sure to set the tranform callback to null because this is a hierachical model;
		NewtonBodySetTransformCallback(body, NULL);

		// set the bod part force and torque call back to the gravity force, skip the transform callback
		NewtonBodySetForceAndTorqueCallback(body, PhysicsApplyGravityForce);
		return body;
	}

	dCustomHingeActuator* LinkHingeActuator (NewtonBody* const parent, NewtonBody* const child)
	{
		dMatrix baseMatrix;
		NewtonBodyGetMatrix (child, &baseMatrix[0][0]);

		dFloat minAngleLimit = -20.0f * dDegreeToRad;
		dFloat maxAngleLimit =  20.0f * dDegreeToRad;
		dFloat angularRate = 15.0f * dDegreeToRad;
		return new dCustomHingeActuator (&baseMatrix[0][0], angularRate, minAngleLimit, maxAngleLimit, child, parent);
	}

	dCustomSliderActuator* LinkLiftActuator(NewtonBody* const parent, NewtonBody* const child)
	{
		dMatrix baseMatrix;
		NewtonBodyGetMatrix(child, &baseMatrix[0][0]);

		dFloat minLimit = -2.0f;
		dFloat maxLimit = 2.5f;
		dFloat linearRate = 0.20f;
		dCustomSliderActuator* const joint = new dCustomSliderActuator(&baseMatrix[0][0], linearRate, minLimit, maxLimit, child, parent);
		joint->SetMinForce(-100.0f);
		return joint;
	}

	dCustomWheel* LinkTireJoint(NewtonBody* const chassis, NewtonBody* const tire)
	{
		dMatrix tireMatrix;
		dMatrix chassisMatrix;

		// calculate the tire location matrix
		NewtonBodyGetMatrix(tire, &tireMatrix[0][0]);
		NewtonBodyGetMatrix(chassis, &chassisMatrix[0][0]);

		chassisMatrix = dRollMatrix(90.0f * dDegreeToRad) * chassisMatrix;
		chassisMatrix = dPitchMatrix(90.0f * dDegreeToRad) * chassisMatrix;
		chassisMatrix.m_posit = tireMatrix.m_posit;

		dFloat angularRate = 60.0f * dDegreeToRad;
		dCustomWheel* const wheel = new dCustomWheel(&chassisMatrix[0][0], tire, chassis);
		wheel->SetSteerRate(angularRate);
		wheel->EnableLimits(true);
		wheel->SetLimits(0.0f, 0.0f);
		return wheel;
	}

	dCustomWheel* LinkFrontTireJoint(dLifterUserData* const lifterData, NewtonBody* const chassis, NewtonBody* const tire)
	{
		dCustomWheel* const wheel = LinkTireJoint(chassis, tire);

		// link traction tire to the engine using a differential gear
		dMatrix matrix;
		dMatrix engineMatrix;
		dMatrix chassisMatrix;
		
		NewtonBodyGetMatrix(tire, &matrix[0][0]);
		dMatrix tireHingeMatrix(dRollMatrix(0.0f * dDegreeToRad) * matrix);
		tireHingeMatrix = wheel->GetMatrix0() * tireHingeMatrix;

		dAssert (chassis == lifterData->m_engineJoint->GetBody1());
		NewtonBody* const engine = lifterData->m_engineJoint->GetBody0();
		lifterData->m_engineJoint->CalculateGlobalMatrix(engineMatrix, chassisMatrix);

		dAssert (0);
		//dFloat sign = dSign(engineMatrix.m_up.DotProduct3(tireHingeMatrix.m_posit - engineMatrix.m_posit));
		//new dCustomDifferentialGear(5.0f, tireHingeMatrix.m_up.Scale(-1.0f), engineMatrix.m_front.Scale(sign), chassisMatrix.m_up, tire, engine, chassis);

		return wheel;
	}

	void ConnectBodyPart(dLifterUserData* const lifterData,  NewtonBody* const parent, NewtonBody* const child, const dString& jointArticulation, dList<dCustomJoint*>& cycleLinks)
	{
		if (jointArticulation == "") {
			// this is the root body do nothing
		} else if (jointArticulation == "frontTire") {
			dCustomWheel* const tire = LinkFrontTireJoint(lifterData, parent, child);
			if (!lifterData->m_frontTiresJoints[0]) {
				lifterData->m_frontTiresJoints[0] = tire;
			} else {
				dAssert(!lifterData->m_frontTiresJoints[1]);
				lifterData->m_frontTiresJoints[1] = tire;
			}
		} else if (jointArticulation == "rearTire") {
			dCustomWheel* const tire = LinkTireJoint(parent, child);
			if (!lifterData->m_rearTireJoints[0]) {
				lifterData->m_rearTireJoints[0] = tire;
			} else {
				dAssert(!lifterData->m_rearTireJoints[1]);
				lifterData->m_rearTireJoints[1] = tire;
			}
		} else if (jointArticulation == "hingeActuator") {
			lifterData->m_forkBase = LinkHingeActuator(parent, child);
		} else if (jointArticulation == "liftActuator") {
			dCustomSliderActuator* const lift = LinkLiftActuator(parent, child);;
			if (!lifterData->m_liftJoints[0]) {
				lifterData->m_liftJoints[0] = lift;
			} else if (!lifterData->m_liftJoints[1]) {
				dAssert(!lifterData->m_liftJoints[1]);
				lifterData->m_liftJoints[1] = lift;
			} else {
				dAssert(!lifterData->m_liftJoints[2]);
				lifterData->m_liftJoints[2] = lift;
			}

		} else if (jointArticulation == "paletteActuator") {
			dCustomSliderActuator* const lift = LinkLiftActuator(parent, child);;
			if (!lifterData->m_paletteJoints[0]) {
				lifterData->m_paletteJoints[0] = lift;
			} else {
				dAssert(!lifterData->m_paletteJoints[1]);
				lifterData->m_paletteJoints[1] = lift;
			}
		} else {
			dAssert(0);
		}
	}

	dModelNode* CreateEngineNode(dModelNode* const chassisBone)
	{
		NewtonWorld* const world = GetWorld();
		NewtonCollision* const shape = NewtonCreateCylinder(world, 0.125f, 0.125f, 0.75f, 0, NULL);

		NewtonBody* const chassis = chassisBone->GetBody();

		// create the rigid body that will make this bone
		dMatrix engineMatrix;
		NewtonBodyGetMatrix(chassis, &engineMatrix[0][0]);
		engineMatrix = dRollMatrix(0.5f * dPi) * engineMatrix;
		engineMatrix.m_posit.m_y += 1.0f;

		NewtonBody* const engineBody = NewtonCreateDynamicBody(world, shape, &engineMatrix[0][0]);

		// destroy the collision helper shape 
		NewtonDestroyCollision(shape);

		// get the collision from body
		NewtonCollision* const collision = NewtonBodyGetCollision(engineBody);
		NewtonCollisionSetMode(collision, 0);

		// calculate the moment of inertia and the relative center of mass of the solid
		//NewtonBodySetMassProperties(engineBody, 50.0f, collision);
		dFloat mass = 50.0f;
		dFloat radius = 1.0f;
		dFloat Inertia = 2.0f * mass * radius * radius / 5.0f;
		NewtonBodySetMassMatrix(engineBody, mass, Inertia, Inertia, Inertia);

		// set the bod part force and torque call back to the gravity force, skip the transform callback
		NewtonBodySetForceAndTorqueCallback(engineBody, PhysicsApplyGravityForce);

		// connect engine to chassis with a hinge
		dMatrix engineAxis;
		engineAxis.m_front = engineMatrix.m_front;
		engineAxis.m_up = engineMatrix.m_right;
		engineAxis.m_right = engineAxis.m_front.CrossProduct(engineAxis.m_up);
		engineAxis.m_posit = engineMatrix.m_posit;

		dCustomDoubleHinge* const engineJoint = new dCustomDoubleHinge(engineAxis, engineBody, chassis);
		engineJoint->EnableLimits(false);
		engineJoint->EnableLimits1(false);
		return new dModelNode(engineBody, dGetIdentityMatrix(), chassisBone);
	}

	dCustomMotor* CreateEngineMotor(dCustomDoubleHinge* const engineJoint)
	{
		dMatrix engineMatrix;
		dMatrix chassisMatrix;
		NewtonBody* const engine = engineJoint->GetBody0();
		NewtonBody* const chassis = engineJoint->GetBody1();
		engineJoint->CalculateGlobalMatrix(engineMatrix, chassisMatrix);
		return new dCustomMotor(engineMatrix.m_up, engine, chassis);
	}

	dModelRootNode* CreateForklift(const dMatrix& location, const char* const filename, int bodyPartsCount, SERVO_VEHICLE_DEFINITION* const definition)
	{
		NewtonWorld* const world = GetWorld();
		DemoEntityManager* const scene = (DemoEntityManager*)NewtonWorldGetUserData(world);

		// make a clone of the mesh 
		DemoEntity* const vehicleModel = DemoEntity::LoadNGD_mesh(filename, scene->GetNewton(), scene->GetShaderCache());
		scene->Append(vehicleModel);

		// plane the model at its location
		vehicleModel->ResetMatrix(*scene, location);

		DemoEntity* const rootEntity = (DemoEntity*)vehicleModel->Find(definition[0].m_boneName);
		NewtonBody* const rootBody = CreateBodyPart(rootEntity, definition[0]);
		dModelRootNode* const controller = new dModelRootNode(rootBody, dGetIdentityMatrix());

		// add this root model 
		AddRoot(controller);

		//controller->SetCalculateLocalTransforms(true);
		controller->SetTranformMode(true);

		//controller->SetSelfCollision(0);
		controller->SetUserData(vehicleModel);

		// move the center of mass a lithe to the back, and lower
		dVector com(0.0f);
		NewtonBodyGetCentreOfMass(rootBody, &com[0]);
		//com.m_x -= 0.25f;
		com.m_y -= 0.25f;
		NewtonBodySetCentreOfMass(rootBody, &com[0]);

//new dCustom6dof (location, rootBody);
//NewtonBodySetMassMatrix(rootBody, 0.0f, 0.0f, 0.0f, 0.0f);

		// attach a user lifterData for controlling the vehicle
		dLifterUserData* const lifterData = new dLifterUserData(vehicleModel);
		vehicleModel->SetUserData(lifterData);

		// add engine
		dModelNode* const engineBone = CreateEngineNode(controller);
		//lifterData->m_engineJoint = (dCustomDoubleHinge*)engineBone->GetParentJoint();
		lifterData->m_engineJoint = (dCustomDoubleHinge*) FindJoint(controller->GetBody(), engineBone->GetBody());
		dAssert(lifterData->m_engineJoint->IsType(dCustomDoubleHinge::GetType()));
		lifterData->m_engineMotor = CreateEngineMotor(lifterData->m_engineJoint);

		// set power parameter for a simple DC engine
		lifterData->m_maxEngineSpeed = 20.0f;
		lifterData->m_engineMotor->SetTorque(3000.0f);

		// walk down the model hierarchy an add all the components 
		int stackIndex = 0;
		dModelNode* parentBones[32];
		DemoEntity* childEntities[32];
		for (DemoEntity* child = rootEntity->GetChild(); child; child = child->GetSibling()) {
			parentBones[stackIndex] = controller;
			childEntities[stackIndex] = child;
			stackIndex++;
		}

		dList<dCustomJoint*> cycleLinks;
		while (stackIndex) {
			stackIndex--;
			DemoEntity* const entity = childEntities[stackIndex];
			dModelNode* parentBone = parentBones[stackIndex];

			const char* const name = entity->GetName().GetStr();
			for (int i = 0; i < bodyPartsCount; i++) {
				if (!strcmp(definition[i].m_boneName, name)) {
					NewtonBody* const bone = CreateBodyPart(entity, definition[i]);

					// connect this body part to its parent with a vehicle joint
					NewtonBody* const parentBody = parentBone->GetBody();
					ConnectBodyPart(lifterData, parentBody, bone, definition[i].m_articulationName, cycleLinks);

					dMatrix bindMatrix(entity->GetParent()->CalculateGlobalMatrix((DemoEntity*)NewtonBodyGetUserData(parentBody)).Inverse());
					parentBone = new dModelNode(bone, bindMatrix, parentBone);
					break;
				}
			}

			for (DemoEntity* child = entity->GetChild(); child; child = child->GetSibling()) {
				parentBones[stackIndex] = parentBone;
				childEntities[stackIndex] = child;
				stackIndex++;
			}
		}

		return controller;
	}
};

static void MakeHeavyLoad (DemoEntityManager* const scene, const dMatrix& location)
{
	dFloat mass = 2500.0f;
//	dFloat mass = 400.0f;

	dMatrix matrix (location);
	matrix.m_posit.m_x += 1.5f;

	NewtonWorld* const world = scene->GetNewton();
	NewtonCollision* const bar = NewtonCreateBox (world, 0.25f, 0.25f, 2.4f, SERVO_VEHICLE_DEFINITION::m_landPart, NULL); 
	NewtonCollision* const bell = NewtonCreateBox (world, 1.0f, 1.0f, 0.25f, SERVO_VEHICLE_DEFINITION::m_landPart, NULL); 

	NewtonCollision* const collision = NewtonCreateCompoundCollision(scene->GetNewton(), SERVO_VEHICLE_DEFINITION::m_landPart);
	NewtonCompoundCollisionBeginAddRemove(collision);

	NewtonCompoundCollisionAddSubCollision(collision, bar);

	dMatrix offset (dGetIdentityMatrix());
	void* const leftWing = NewtonCompoundCollisionAddSubCollision (collision, bell);
	offset.m_posit.m_z = -1.0f;
	NewtonCompoundCollisionSetSubCollisionMatrix (collision, leftWing, &offset[0][0]);	

	void* const rightWing = NewtonCompoundCollisionAddSubCollision (collision, bell);
	offset.m_posit.m_z = 1.0f;
	NewtonCompoundCollisionSetSubCollisionMatrix(collision, rightWing, &offset[0][0]);

	NewtonCompoundCollisionEndAddRemove(collision);

	NewtonDestroyCollision(bar);
	NewtonDestroyCollision(bell);

	DemoMesh* const mesh = new DemoMesh ("weight", scene->GetShaderCache(), collision, "wood_1.tga", "wood_1.tga", "wood_1.tga");
	CreateSimpleSolid (scene, mesh, mass, matrix, collision, 0, false);

	mesh->Release();
	NewtonDestroyCollision(collision);
}
#endif


class ServoVehicleManagerManager: public dModelManager
{
	public:
	ServoVehicleManagerManager(DemoEntityManager* const scene, int threadMaterialID)
		:dModelManager(scene->GetNewton())
		//,m_player(NULL)
		//,m_threadMaterialID(threadMaterialID)
		//,m_cabinSpeed(0)
		//,m_bucket_x(0.0f)
		//,m_bucket_y(0.0f)
		//,m_bucket_angle(0.0f)
	{
/*
		scene->SetUpdateCameraFunction(UpdateCameraCallback, this);
		scene->Set2DDisplayRenderFunction(RenderPlayerHelp, NULL, this);

		// create a material for early collision culling
		NewtonWorld* const world = scene->GetNewton();
		int material = NewtonMaterialGetDefaultGroupID(world);

		NewtonMaterialSetCallbackUserData(world, material, material, this);
		NewtonMaterialSetCollisionCallback(world, material, material, StandardAABBOverlapTest, NULL);
		NewtonMaterialSetDefaultElasticity(world, material, material, 0.1f);
		NewtonMaterialSetDefaultFriction(world, material, material, 0.9f, 0.9f);

		NewtonMaterialSetCallbackUserData(world, material, threadMaterialID, this);
		NewtonMaterialSetCollisionCallback(world, material, threadMaterialID, StandardAABBOverlapTest, NULL);
		NewtonMaterialSetDefaultElasticity(world, material, threadMaterialID, 0.1f);
		NewtonMaterialSetDefaultFriction(world, material, threadMaterialID, 0.9f, 0.9f);

		NewtonMaterialSetCallbackUserData(world, threadMaterialID, threadMaterialID, this);
		NewtonMaterialSetCollisionCallback(world, threadMaterialID, threadMaterialID, StandardAABBOverlapTest, NULL);
		NewtonMaterialSetContactGenerationCallback(world, threadMaterialID, threadMaterialID, ThreadStaticContactsGeneration);
		NewtonMaterialSetDefaultElasticity(world, threadMaterialID, threadMaterialID, 0.1f);
		NewtonMaterialSetDefaultFriction(world, threadMaterialID, threadMaterialID, 0.9f, 0.9f);
*/
	}

	static void UpdateCameraCallback(DemoEntityManager* const manager, void* const context, dFloat timestep)
	{
		ServoVehicleManagerManager* const me = (ServoVehicleManagerManager*)context;
		me->UpdateCamera(timestep);
	}

	static void RenderPlayerHelp(DemoEntityManager* const scene, void* const context)
	{
		ServoVehicleManagerManager* const me = (ServoVehicleManagerManager*)context;
		me->RenderPlayerHelp(scene);
	}

	void RenderPlayerHelp(DemoEntityManager* const scene)
	{
/*
		dVector color(1.0f, 1.0f, 0.0f, 0.0f);
		scene->Print(color, "Navigation Keys");
		scene->Print(color, "drive forward:      w");
		scene->Print(color, "drive backward:     s");
		scene->Print(color, "turn right:         d");
		scene->Print(color, "turn left:          a");

		scene->Print(color, "bucket controls");
		ImGui::SliderInt("cabin rotation", &m_cabinSpeed, -3, 3);
		ImGui::SliderFloat("bucket x", &m_bucket_x, -2.0f, 4.0f);
		ImGui::SliderFloat("bucket y", &m_bucket_y, -6.0f, 6.0f);
		ImGui::SliderFloat("bucket angle", &m_bucket_angle, -60.0f, 130.0f);
*/
	}

	void UpdateCamera(dFloat timestep)
	{
		//return;
		//if (!m_player) {
		//	return;
		//}
/*
		DemoEntity* const player = (DemoEntity*)NewtonBodyGetUserData(m_player->GetBody());
		DemoEntityManager* const scene = (DemoEntityManager*)NewtonWorldGetUserData(GetWorld());
		DemoCamera* const camera = scene->GetCamera();
		dMatrix camMatrix(camera->GetNextMatrix());
		dMatrix playerMatrix(player->GetNextMatrix());

		dVector frontDir(camMatrix[0]);
		dVector camOrigin(playerMatrix.m_posit + dVector(0.0f, ARTICULATED_VEHICLE_CAMERA_HIGH_ABOVE_HEAD, 0.0f, 0.0f));
		camOrigin -= frontDir.Scale(ARTICULATED_VEHICLE_CAMERA_DISTANCE);
		camera->SetNextMatrix(*scene, camMatrix, camOrigin);
*/
	}
/*
	NewtonBody* CreateBodyPart(DemoEntity* const bodyPart, dFloat mass)
	{
		NewtonWorld* const world = GetWorld();
		NewtonCollision* const shape = bodyPart->CreateCollisionFromchildren(world);
		dAssert(shape);

		// calculate the bone matrix
		dMatrix matrix(bodyPart->CalculateGlobalMatrix());

		// create the rigid body that will make this bone
		NewtonBody* const body = NewtonCreateDynamicBody(world, shape, &matrix[0][0]);

		// assign the material ID
		NewtonBodySetMaterialGroupID(body, NewtonMaterialGetDefaultGroupID(world));

		// destroy the collision helper shape 
		NewtonDestroyCollision(shape);

		// get the collision from body
		NewtonCollision* const collision = NewtonBodyGetCollision(body);

		// save the root node as the use data
		NewtonCollisionSetUserData(collision, this);

		// set collision filter
		// set the material properties for each link
		NewtonCollisionMaterial material;
		NewtonCollisionGetMaterial(collision, &material);
		material.m_userId = ARTICULATED_VEHICLE_DEFINITION::m_bodyPart;
		material.m_userParam[0].m_int =
			ARTICULATED_VEHICLE_DEFINITION::m_terrain |
			ARTICULATED_VEHICLE_DEFINITION::m_bodyPart |
			ARTICULATED_VEHICLE_DEFINITION::m_linkPart |
			ARTICULATED_VEHICLE_DEFINITION::m_tirePart |
			ARTICULATED_VEHICLE_DEFINITION::m_propBody;
		NewtonCollisionSetMaterial(collision, &material);

		// calculate the moment of inertia and the relative center of mass of the solid
		NewtonBodySetMassProperties(body, mass, collision);

		// save the user data with the bone body (usually the visual geometry)
		NewtonBodySetUserData(body, bodyPart);

		// set the bod part force and torque call back to the gravity force, skip the transform callback
		NewtonBodySetForceAndTorqueCallback(body, PhysicsApplyGravityForce);
		return body;
	}
*/

	dModelRootNode* CreateExcavator(const char* const modelName, const dMatrix& location)
	{

		NewtonWorld* const world = GetWorld();
		DemoEntityManager* const scene = (DemoEntityManager*)NewtonWorldGetUserData(world);

		// make a clone of the mesh 
		DemoEntity* const vehicleModel = DemoEntity::LoadNGD_mesh(modelName, world, scene->GetShaderCache());
		scene->Append(vehicleModel);
/*
		// place the model at its location
		dMatrix matrix(vehicleModel->GetCurrentMatrix());
		matrix.m_posit = location.m_posit;
		vehicleModel->ResetMatrix(*scene, matrix);

		DemoEntity* const rootEntity = (DemoEntity*)vehicleModel->Find("base");
		NewtonBody* const rootBody = CreateBodyPart(rootEntity, 4000.0f);
		dExcavatorModel* const controller = new dExcavatorModel(rootBody, m_threadMaterialID);

		// the the model to calculate the local transformation
		controller->SetTranformMode(true);

		// add the model to the manager
		AddRoot(controller);

		m_player = controller;
		return controller;
*/
		return NULL;
	}

/*
	virtual void OnDebug(dModelRootNode* const model, dCustomJoint::dDebugDisplay* const debugContext)
	{
		dExcavatorModel* const excavator = (dExcavatorModel*)model;
		excavator->OnDebug(debugContext);
	}

	virtual void OnPreUpdate(dModelRootNode* const model, dFloat timestep) const
	{
		dExcavatorModel* const excavator = (dExcavatorModel*)model;

		NewtonWorld* const world = NewtonBodyGetWorld(excavator->GetBody());
		DemoEntityManager* const scene = (DemoEntityManager*)NewtonWorldGetUserData(world);

		dExcavatorControls controls;
		controls.m_throttle = (dFloat(scene->GetKeyState('W')) - dFloat(scene->GetKeyState('S')));
		controls.m_steeringValue = (dFloat(scene->GetKeyState('A')) - dFloat(scene->GetKeyState('D')));
		controls.m_cabinSpeed = m_cabinSpeed;
		controls.m_bucket_x = m_bucket_x;
		controls.m_bucket_y = m_bucket_y;
		controls.m_bucket_angle = m_bucket_angle;

		excavator->ApplyControls(controls, timestep);
	}

	virtual void OnUpdateTransform(const dModelNode* const bone, const dMatrix& localMatrix) const
	{
		NewtonBody* const body = bone->GetBody();
		DemoEntity* const ent = (DemoEntity*)NewtonBodyGetUserData(body);
		if (ent) {
			DemoEntityManager* const scene = (DemoEntityManager*)NewtonWorldGetUserData(NewtonBodyGetWorld(body));

			dQuaternion rot(localMatrix);
			ent->SetMatrix(*scene, rot, localMatrix.m_posit);
		}
	}

	static int StandardAABBOverlapTest(const NewtonJoint* const contactJoint, dFloat timestep, int threadIndex)
	{
		const NewtonBody* const body0 = NewtonJointGetBody0(contactJoint);
		const NewtonBody* const body1 = NewtonJointGetBody1(contactJoint);
		const NewtonCollision* const collision0 = NewtonBodyGetCollision(body0);
		const NewtonCollision* const collision1 = NewtonBodyGetCollision(body1);

		if (NewtonCollisionGetUserData(collision0) != NewtonCollisionGetUserData(collision1)) {
			return 1;
		}

		NewtonCollisionMaterial material0;
		NewtonCollisionMaterial material1;
		NewtonCollisionGetMaterial(collision0, &material0);
		NewtonCollisionGetMaterial(collision1, &material1);

		//m_terrain	 = 1 << 0,
		//m_bodyPart = 1 << 1,
		//m_tirePart = 1 << 2,
		//m_linkPart = 1 << 3,
		//m_propBody = 1 << 4,

		const dLong mask0 = material0.m_userId & material1.m_userParam[0].m_int;
		const dLong mask1 = material1.m_userId & material0.m_userParam[0].m_int;
		return (mask0 && mask1) ? 1 : 0;
	}

	static int ThreadStaticContactsGeneration(const NewtonMaterial* const material, const NewtonBody* const body0, const NewtonCollision* const collision0, const NewtonBody* const body1, const NewtonCollision* const collision1, NewtonUserContactPoint* const contactBuffer, int maxCount, int threadIndex)
	{
		dAssert(NewtonBodyGetMaterialGroupID(body0) == NewtonBodyGetMaterialGroupID(body1));
		dAssert(NewtonBodyGetMaterialGroupID(body0) != NewtonMaterialGetDefaultGroupID(NewtonBodyGetWorld(body0)));
		dAssert(NewtonBodyGetMaterialGroupID(body1) != NewtonMaterialGetDefaultGroupID(NewtonBodyGetWorld(body1)));

		dAssert(NewtonCollisionGetUserID(collision0) == ARTICULATED_VEHICLE_DEFINITION::m_linkPart);
		dAssert(NewtonCollisionGetUserID(collision1) == ARTICULATED_VEHICLE_DEFINITION::m_terrain);

		dExcavatorModel* const excavator = (dExcavatorModel*)NewtonCollisionGetUserData(collision0);
		dAssert(excavator);
		return excavator->CollideLink(material, body0, body1, contactBuffer);
	}

	dExcavatorModel* m_player;
	int m_threadMaterialID;
	int m_cabinSpeed;
	dFloat32 m_bucket_x;
	dFloat32 m_bucket_y;
	dFloat32 m_bucket_angle;
*/
};

void ServoJoints (DemoEntityManager* const scene)
{
	// load the sky box
	scene->CreateSkyBox();

	NewtonBody* const floor = CreateLevelMesh (scene, "flatPlane.ngd", true);
	//CreateHeightFieldTerrain (scene, 9, 8.0f, 1.5f, 0.2f, 200.0f, -50.0f);
	//NewtonCollision* const floorCollision = NewtonBodyGetCollision(floor);
	//NewtonCollisionSetUserID(floorCollision, SERVO_VEHICLE_DEFINITION::m_terrain);

	ServoVehicleManagerManager* const vehicleManager = new ServoVehicleManagerManager (scene, 0);

	NewtonWorld* const world = scene->GetNewton();
	dVector origin(FindFloor(world, dVector(-10.0f, 50.0f, 0.0f, 1.0f), 2.0f * 50.0f));

	dMatrix matrix(dGetIdentityMatrix());
	matrix.m_posit = FindFloor(world, origin, 100.0f);
	matrix.m_posit.m_y += 1.5f;
	vehicleManager->CreateExcavator("tractor.ngd", matrix);
/*
	NewtonWorld* const world = scene->GetNewton();
	dVector origin (FindFloor (world, dVector (-10.0f, 50.0f, 0.0f, 1.0f), 2.0f * 50.0f));

	// add an input Manage to manage the inputs and user interaction 
	ServoInputManager* const inputManager = new ServoInputManager (scene);
	inputManager;

	//  create a skeletal transform controller for controlling rag doll
	ServoVehicleManagerManager* const vehicleManager = new ServoVehicleManagerManager (scene);

	dMatrix matrix (dGetIdentityMatrix());
	matrix.m_posit = FindFloor (world, origin, 100.0f);
	matrix.m_posit.m_y += 0.5f;
	
	// load a the mesh of the articulate vehicle
	dModelRootNode* const forklift = vehicleManager->CreateForklift(matrix, "forklift.ngd", sizeof(inverseKinematicsRidParts) / sizeof (inverseKinematicsRidParts[0]), inverseKinematicsRidParts);
	inputManager->AddPlayer(forklift);
*/

#if 1
	//place heavy load to show reproduce black bird dream problems
	matrix.m_posit.m_x += 2.0f;	
	matrix.m_posit.m_z -= 2.0f;	
//	MakeHeavyLoad (scene, matrix);

	// add some object to play with
	LoadLumberYardMesh(scene, dVector(5.0f, 0.0f, 0.0f, 0.0f), 0);
//	LoadLumberYardMesh(scene, dVector(5.0f, 0.0f, 6.0f, 0.0f), 0);
//	LoadLumberYardMesh(scene, dVector(10.0f, 0.0f, -4.0f, 0.0f), 0);
//	LoadLumberYardMesh(scene, dVector(10.0f, 0.0f,  2.0f, 0.0f), 0);
//	LoadLumberYardMesh(scene, dVector(15.0f, 0.0f, 0.0f, 0.0f), 0);
//	LoadLumberYardMesh(scene, dVector(15.0f, 0.0f, 6.0f, 0.0f), 0);
#endif	
	origin.m_x -= 2.0f;
	origin.m_y += 4.0f;
	//origin.m_z = 6.0f;
	dQuaternion rot (dVector (0.0f, 1.0f, 0.0f, 0.0f), 0.0f * dDegreeToRad);  
	scene->SetCameraMatrix(rot, origin);
}



