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
#include "DemoMesh.h"
#include "DemoCamera.h"
#include "PhysicsUtils.h"
#include "TargaToOpenGl.h"
#include "DemoEntityManager.h"
#include "DebugDisplay.h"
#include "HeightFieldPrimitive.h"


#define SERVO_VEHICLE_CAMERA_EYEPOINT			1.5f
#define SERVO_VEHICLE_CAMERA_HIGH_ABOVE_HEAD	2.0f
#define SERVO_VEHICLE_CAMERA_DISTANCE			7.0f

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


static SERVO_VEHICLE_DEFINITION forkliftDefinition[] =
{
	{"body",		"convexHull",			900.0f, SERVO_VEHICLE_DEFINITION::m_bodyPart, "mainBody"},
	{"fr_tire",		"tireShape",			 50.0f, SERVO_VEHICLE_DEFINITION::m_tirePart, "frontTire"},
	{"fl_tire",		"tireShape",			 50.0f, SERVO_VEHICLE_DEFINITION::m_tirePart, "frontTire"},
	{"rr_tire",		"tireShape",			 50.0f, SERVO_VEHICLE_DEFINITION::m_tirePart, "rearTire"},
	{"rl_tire",		"tireShape",			 50.0f, SERVO_VEHICLE_DEFINITION::m_tirePart, "rearTire"},
	{"lift_1",		"convexHull",			 50.0f, SERVO_VEHICLE_DEFINITION::m_bodyPart, "hingeActuator"},
	{"lift_2",		"convexHull",			 40.0f, SERVO_VEHICLE_DEFINITION::m_bodyPart, "liftActuator"},
	{"lift_3",		"convexHull",			 30.0f, SERVO_VEHICLE_DEFINITION::m_bodyPart, "liftActuator"},
	{"lift_4",		"convexHull",			 20.0f, SERVO_VEHICLE_DEFINITION::m_bodyPart, "liftActuator"},
	{"left_teeth",  "convexHullAggregate",	 10.0f, SERVO_VEHICLE_DEFINITION::m_bodyPart, "paletteActuator"},
	{"right_teeth", "convexHullAggregate",	 10.0f, SERVO_VEHICLE_DEFINITION::m_bodyPart, "paletteActuator"},
};

#if 0
class ServoEntityModel: public DemoEntity
{
	public:

	class InputRecord
	{
		public:
		InputRecord()
		{
			memset (this, 0, sizeof (InputRecord));
		}

		dFloat m_slideValue;
		dFloat m_turnValue;
		dFloat m_liftValue;
		int m_openValue;
		int m_wristAxis0;
		int m_wristAxis1;
		int m_steerValue;
		int m_throttleValue;
	};

	void SetInput (const InputRecord& inputs)
	{
		m_inputs = inputs;
	}

	int m_rearTiresCount;
	int m_tractionTiresCount;
	int m_liftActuatorsCount;
	int m_universalActuatorsCount;
	int m_paletteActuatorsCount;
	dFloat m_maxEngineTorque;
	dFloat m_maxEngineSpeed;
	dFloat m_maxTurmVelocity;
	dFloat m_tiltAngle;
	dFloat m_liftPosit;
	dFloat m_openPosit;
	dFloat m_steeringTorque;
	dFloat m_wristAxis0;
	dFloat m_wristAxis1;

	dCustomMotor2* m_engineMotor;
	dCustomUniversal* m_engineJoint;
	InputRecord m_inputs;
};

class ServoVehicleManagerManager: public dCustomArticulaledTransformManager
{
	public:

	ServoVehicleManagerManager (DemoEntityManager* const scene)
		:dCustomArticulaledTransformManager (scene->GetNewton())
	{
		// create a material for early collision culling
		int material = NewtonMaterialGetDefaultGroupID (scene->GetNewton());
		NewtonMaterialSetCallbackUserData (scene->GetNewton(), material, material, this);
		NewtonMaterialSetCompoundCollisionCallback(scene->GetNewton(), material, material, CompoundSubCollisionAABBOverlap);
		NewtonMaterialSetCollisionCallback (scene->GetNewton(), material, material, OnBoneAABBOverlap, OnContactsProcess);
	}

	virtual void OnDebug(dCustomJoint::dDebugDisplay* const debugContext)
	{
	}

	virtual void OnPreUpdate (dCustomArticulatedTransformController* const controller, dFloat timestep, int threadIndex) const
	{
		ServoEntityModel* const vehicleModel = (ServoEntityModel*)controller->GetUserData();

		if (vehicleModel->m_engineJoint) {
			dFloat brakeTorque = 2000.0f;
			//dFloat engineTorque = 0.0f;
			dFloat engineRPM = 0.0f;
			if (vehicleModel->m_inputs.m_throttleValue > 0) {
				brakeTorque = 0.0f;
				engineRPM = -vehicleModel->m_maxEngineSpeed;
			} else if (vehicleModel->m_inputs.m_throttleValue < 0) {
				brakeTorque = 0.0f;
				engineRPM = vehicleModel->m_maxEngineSpeed;
			}

			// apply DC engine torque
			vehicleModel->m_engineMotor->SetSpeed1(engineRPM);

			if (!vehicleModel->m_rearTiresCount) {
				// apply DC rate turn Motor 
				if (vehicleModel->m_inputs.m_steerValue > 0) {
					brakeTorque = 0.0f;
					vehicleModel->m_engineMotor->SetSpeed(vehicleModel->m_maxTurmVelocity);
				} else if (vehicleModel->m_inputs.m_steerValue < 0){
					brakeTorque = 0.0f;
					vehicleModel->m_engineMotor->SetSpeed(-vehicleModel->m_maxTurmVelocity);
				} else {
					vehicleModel->m_engineMotor->SetSpeed(0.0f);
				}
			}

			// apply breaks
			for (int i = 0; i < vehicleModel->m_tractionTiresCount; i ++) {
				vehicleModel->m_tractionTiresJoints[i]->SetFriction(brakeTorque);
			}
		}
		
		// update steering wheels
		if (vehicleModel->m_rearTiresCount) {
			dAssert (0);
/*
			dFloat steeringAngle = vehicleModel->m_rearTireJoints[0]->GetJointAngle_1();
			if (vehicleModel->m_inputs.m_steerValue > 0) {
				//steeringAngle = vehicleModel->m_rearTireJoints[0]->GetMinAngularLimit0(); 
				steeringAngle = vehicleModel->m_rearTireJoints[0]->GetMinAngularLimit_1();
			} else if (vehicleModel->m_inputs.m_steerValue < 0) {
				//steeringAngle = vehicleModel->m_rearTireJoints[0]->GetMaxAngularLimit0(); 
				steeringAngle = vehicleModel->m_rearTireJoints[0]->GetMaxAngularLimit_1();
			}
			for (int i = 0; i < vehicleModel->m_rearTiresCount; i ++) {
				vehicleModel->m_rearTireJoints[i]->SetTargetAngle1(steeringAngle);
			}
*/
		}

		// set the base turn angle
		vehicleModel->m_angularActuator1->SetTargetAngle(vehicleModel->m_inputs.m_turnValue);


		// set the tilt angle
/*
		dFloat tiltAngle = vehicleModel->m_tiltAngle;
		if (vehicleModel->m_inputs.m_liftValue > 0) {
			tiltAngle = vehicleModel->m_angularActuator0[0]->GetMinAngularLimit();
			vehicleModel->m_tiltAngle = vehicleModel->m_angularActuator0[0]->GetActuatorAngle();
		} else if (vehicleModel->m_inputs.m_liftValue < 0) {
			tiltAngle = vehicleModel->m_angularActuator0[0]->GetMaxAngularLimit();
			vehicleModel->m_tiltAngle = vehicleModel->m_angularActuator0[0]->GetActuatorAngle();
		}

		for (int i = 0; i < vehicleModel->m_angularActuatorsCount0; i ++) {
			vehicleModel->m_angularActuator0[i]->SetTargetAngle (tiltAngle);
		}
*/


		vehicleModel->m_angularActuator0->SetTargetAngle(vehicleModel->m_inputs.m_liftValue);


		// set the lift position
//		if (vehicleModel->m_liftActuatorsCount) 
		{
/*
			dFloat liftPosit = vehicleModel->m_liftPosit;
			if (vehicleModel->m_inputs.m_liftValue > 0) {
				liftPosit = vehicleModel->m_liftJoints[0]->GetMinPositLimit();
				vehicleModel->m_liftPosit = vehicleModel->m_liftJoints[0]->GetActuatorPosit();
			} else if (vehicleModel->m_inputs.m_liftValue < 0) {
				liftPosit = vehicleModel->m_liftJoints[0]->GetMaxPositLimit();
				vehicleModel->m_liftPosit = vehicleModel->m_liftJoints[0]->GetActuatorPosit();
			}
			for (int i = 0; i < vehicleModel->m_liftActuatorsCount; i ++) {
				vehicleModel->m_liftJoints[i]->SetTargetPosit(liftPosit);
			}
*/
		}

		vehicleModel->m_liftJoints[0]->SetTargetPosit(vehicleModel->m_inputs.m_slideValue);
		vehicleModel->m_liftJoints[1]->SetTargetPosit(vehicleModel->m_inputs.m_slideValue);
		

		// open Close palette position
		if (vehicleModel->m_paletteActuatorsCount) {
			dFloat openPosit = vehicleModel->m_openPosit;
			if (vehicleModel->m_inputs.m_openValue > 0) {
				openPosit = vehicleModel->m_paletteJoints[0]->GetMinPositLimit();
				vehicleModel->m_openPosit = vehicleModel->m_paletteJoints[0]->GetActuatorPosit();
			} else if (vehicleModel->m_inputs.m_openValue < 0) {
				openPosit = vehicleModel->m_paletteJoints[0]->GetMaxPositLimit();
				vehicleModel->m_openPosit = vehicleModel->m_paletteJoints[0]->GetActuatorPosit();
			}
			for (int i = 0; i < vehicleModel->m_paletteActuatorsCount; i ++) {
				vehicleModel->m_paletteJoints[i]->SetTargetPosit(openPosit);
			}
		}

		// open Close palette position
		if (vehicleModel->m_universalActuatorsCount) {
			dAssert (0);
/*
			dFloat posit0 = vehicleModel->m_wristAxis0;
			if (vehicleModel->m_inputs.m_wristAxis0 > 0) {
				posit0 = vehicleModel->m_universalActuator[0]->GetMinAngularLimit_0();
				vehicleModel->m_wristAxis0 = vehicleModel->m_universalActuator[0]->GetJointAngle_0();
			} else if (vehicleModel->m_inputs.m_wristAxis0 < 0) {
				posit0 = vehicleModel->m_universalActuator[0]->GetMaxAngularLimit_0();
				vehicleModel->m_wristAxis0 = vehicleModel->m_universalActuator[0]->GetJointAngle_1();
			}

			dFloat posit1 = vehicleModel->m_wristAxis1;
			if (vehicleModel->m_inputs.m_wristAxis1 > 0) {
				posit1 = vehicleModel->m_universalActuator[0]->GetMinAngularLimit_1();
				vehicleModel->m_wristAxis1 = vehicleModel->m_universalActuator[0]->GetJointAngle_1();
			} else if (vehicleModel->m_inputs.m_wristAxis1 < 0) {
				posit1 = vehicleModel->m_universalActuator[0]->GetMaxAngularLimit_1();
				vehicleModel->m_wristAxis1 = vehicleModel->m_universalActuator[0]->GetJointAngle_1();
			}

			for (int i = 0; i < vehicleModel->m_universalActuatorsCount; i ++) {
				vehicleModel->m_universalActuator[i]->SetTargetAngle0(posit0);
				vehicleModel->m_universalActuator[i]->SetTargetAngle1(posit1);
			}
*/
		}
	}

	static int OnBoneAABBOverlap (const NewtonMaterial* const material, const NewtonBody* const body0, const NewtonBody* const body1, int threadIndex)
	{
		NewtonCollision* const collision0 = NewtonBodyGetCollision(body0);
		NewtonCollision* const collision1 = NewtonBodyGetCollision(body1);
		SERVO_VEHICLE_DEFINITION::SHAPES_ID id0 = SERVO_VEHICLE_DEFINITION::SHAPES_ID (NewtonCollisionGetUserID(collision0));
		SERVO_VEHICLE_DEFINITION::SHAPES_ID id1 = SERVO_VEHICLE_DEFINITION::SHAPES_ID (NewtonCollisionGetUserID(collision1));
		dAssert(id0 != SERVO_VEHICLE_DEFINITION::m_tireInnerRing);
		dAssert(id1 != SERVO_VEHICLE_DEFINITION::m_tireInnerRing);

		switch (id0 | id1)
		{
			case SERVO_VEHICLE_DEFINITION::m_linkPart | SERVO_VEHICLE_DEFINITION::m_linkPart:
			case SERVO_VEHICLE_DEFINITION::m_linkPart | SERVO_VEHICLE_DEFINITION::m_bodyPart:
			case SERVO_VEHICLE_DEFINITION::m_tirePart | SERVO_VEHICLE_DEFINITION::m_tirePart:
			{
				return 0;
				break;
			}

			case SERVO_VEHICLE_DEFINITION::m_terrain | SERVO_VEHICLE_DEFINITION::m_bodyPart:
			case SERVO_VEHICLE_DEFINITION::m_terrain | SERVO_VEHICLE_DEFINITION::m_landPart:
			case SERVO_VEHICLE_DEFINITION::m_terrain | SERVO_VEHICLE_DEFINITION::m_linkPart:
			case SERVO_VEHICLE_DEFINITION::m_tirePart | SERVO_VEHICLE_DEFINITION::m_bodyPart:
			case SERVO_VEHICLE_DEFINITION::m_tirePart | SERVO_VEHICLE_DEFINITION::m_linkPart:
			case SERVO_VEHICLE_DEFINITION::m_tirePart | SERVO_VEHICLE_DEFINITION::m_terrain:
			case SERVO_VEHICLE_DEFINITION::m_tirePart | SERVO_VEHICLE_DEFINITION::m_landPart:
			case SERVO_VEHICLE_DEFINITION::m_landPart | SERVO_VEHICLE_DEFINITION::m_bodyPart:
			case SERVO_VEHICLE_DEFINITION::m_landPart | SERVO_VEHICLE_DEFINITION::m_linkPart:
			case SERVO_VEHICLE_DEFINITION::m_landPart | SERVO_VEHICLE_DEFINITION::m_landPart:
			{
				return 1;
				break;
			}
			default:
			{
				dAssert (0);
				return 1;
			}
		}
	}

	static int CompoundSubCollisionAABBOverlap (const NewtonMaterial* const material, const NewtonBody* const body0, const void* const collisionNode0, const NewtonBody* const body1, const void* const collisionNode1, int threadIndex)
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
			case SERVO_VEHICLE_DEFINITION::m_tirePart | SERVO_VEHICLE_DEFINITION::m_terrain:
			case SERVO_VEHICLE_DEFINITION::m_tirePart | SERVO_VEHICLE_DEFINITION::m_landPart:
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


//dAssert (0);
/*
		if (linkBody && !tireBody) {
			// find the root body from the articulated structure 
			NewtonCollision* const linkCollision = NewtonBodyGetCollision(linkBody);
			const dCustomArticulatedTransformController::dSkeletonBone* const rootbone = (dCustomArticulatedTransformController::dSkeletonBone*)NewtonCollisionGetUserData(linkCollision);
			dCustomArticulatedTransformController* const controller = rootbone->m_controller;
			NewtonBody* const chassiBody = controller->GetBoneBody(rootbone);

			int countCount = 0;
			void* contactList[32];
			for (void* contact = NewtonContactJointGetFirstContact(contactJoint); contact; contact = NewtonContactJointGetNextContact(contactJoint, contact)) {
				contactList[countCount] = contact;
				countCount ++;
			}

			for (int i = 1; i < countCount; i++) {
				NewtonContactJointRemoveContact(contactJoint, contactList[i]);
			}

			dMatrix tireMatrix;
			dMatrix chassisMatrix;
			NewtonBodyGetMatrix(linkBody, &tireMatrix[0][0]);
			NewtonBodyGetMatrix(chassiBody, &chassisMatrix[0][0]);
			dVector upDir(chassisMatrix.RotateVector(dVector(0.0f, 1.0f, 0.0f, 0.0f)));
			dVector tireAxis(tireMatrix.RotateVector(dVector(1.0f, 0.0f, 0.0f, 0.0f)));
			dVector contactDirection(upDir.CrossProduct(tireAxis));

			NewtonMaterial* const material = NewtonContactGetMaterial(contactList[0]);
			NewtonMaterialContactRotateTangentDirections(material, &contactDirection[0]);
			NewtonMaterialSetContactFrictionCoef(material, 0.5f, 0.5f, 0);
			NewtonMaterialSetContactFrictionCoef(material, 1.0f, 1.0f, 1);

		} else if (tireBody) {

			// find the root body from the articulated structure 
			NewtonCollision* const tireCollsion = NewtonBodyGetCollision(tireBody);
			const dCustomArticulatedTransformController::dSkeletonBone* const bone = (dCustomArticulatedTransformController::dSkeletonBone*)NewtonCollisionGetUserData (tireCollsion);

			dCustomArticulatedTransformController* const controller = bone->m_controller;
			const dCustomArticulatedTransformController::dSkeletonBone* const rootbone = controller->GetParent(bone);
			NewtonBody* const chassiBody = controller->GetBoneBody(rootbone);

			// Get the root and tire matrices
			dMatrix tireMatrix;
			dMatrix chassisMatrix;
			NewtonBodyGetMatrix(tireBody, &tireMatrix[0][0]);
			NewtonBodyGetMatrix(chassiBody, &chassisMatrix[0][0]);

			dVector upDir (chassisMatrix.RotateVector(dVector (0.0f, 1.0f, 0.0f, 0.0f)));
			dVector tireAxis (tireMatrix.RotateVector(dVector (1.0f, 0.0f, 0.0f, 0.0f)));

			dVector contactDirection (upDir.CrossProduct(tireAxis));
			for (void* contact = NewtonContactJointGetFirstContact (contactJoint); contact; contact = NewtonContactJointGetNextContact (contactJoint, contact)) {
				NewtonMaterial* const material = NewtonContactGetMaterial (contact);
				NewtonMaterialContactRotateTangentDirections (material, &contactDirection[0]);
				NewtonMaterialSetContactFrictionCoef (material, 1.0f, 1.0f, 0);
				NewtonMaterialSetContactFrictionCoef (material, 1.0f, 1.0f, 1);
			}
		}
*/
	}
	
	dCustomArticulatedTransformController::dSkeletonBone* CreateEngineNode(dCustomArticulatedTransformController* const controller, dCustomArticulatedTransformController::dSkeletonBone* const chassisBone)
	{
		NewtonWorld* const world = GetWorld();
		NewtonCollision* const shape = NewtonCreateCylinder (world, 0.125f, 0.125f, 0.75f, 0, NULL);

		NewtonBody* const chassis = chassisBone->m_body;

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
		NewtonCollisionSetMode (collision, 0);

		// calculate the moment of inertia and the relative center of mass of the solid
		//NewtonBodySetMassProperties(engineBody, 50.0f, collision);
		dFloat mass = 50.0f;
		dFloat radius = 1.0f;
		dFloat Inertia = 2.0f * mass * radius * radius / 5.0f;
		NewtonBodySetMassMatrix (engineBody, mass, Inertia, Inertia, Inertia);

		// set the bod part force and torque call back to the gravity force, skip the transform callback
		NewtonBodySetForceAndTorqueCallback(engineBody, PhysicsApplyGravityForce);

		// connect engine to chassis with a hinge
		dMatrix engineAxis;
		engineAxis.m_front = engineMatrix.m_front;
		engineAxis.m_up = engineMatrix.m_right;
		engineAxis.m_right = engineAxis.m_front.CrossProduct(engineAxis.m_up);
		engineAxis.m_posit = engineMatrix.m_posit;

		dCustomUniversal* const engineJoint = new dCustomUniversal(engineAxis, engineBody, chassis);
		engineJoint->EnableLimits(false);
		engineJoint->EnableLimits2(false);
		return controller->AddBone(engineBody, dGetIdentityMatrix(), chassisBone);
	}

	dCustomMotor2* CreateEngineMotor(dCustomArticulatedTransformController* const controller, dCustomUniversal* const engineJoint)
	{
		dMatrix engineMatrix;
		dMatrix chassisMatrix;
		NewtonBody* const engine = engineJoint->GetBody0();
		NewtonBody* const chassis = engineJoint->GetBody1();
		engineJoint->CalculateGlobalMatrix(engineMatrix, chassisMatrix);
		return new dCustomMotor2(engineMatrix.m_front, engineMatrix.m_up, engine, chassis);
	}
};


// we recommend using and input manage to control input for all games
class ServoInputManager: public dCustomInputManager
{
	public:
	ServoInputManager (DemoEntityManager* const scene)
		:dCustomInputManager(scene->GetNewton())
		,m_scene(scene)
		,m_cameraMode(true)
		,m_changeVehicle(true)
		,m_playersCount(0)
		,m_currentPlayer(0)
		,m_liftboom(0.0f) 
		,m_slideboom(0.0f) 
		,m_rotatebase(0.0f)
	{
		// plug a callback for 2d help display
		scene->Set2DDisplayRenderFunction (RenderPlayerHelp, NULL, this);
		scene->SetUpdateCameraFunction(UpdateCameraCallback, this);
	}

	void OnBeginUpdate (dFloat timestepInSecunds)
	{
		ServoEntityModel::InputRecord inputs;

		ServoEntityModel* const vehicleModel = (ServoEntityModel*) m_player[m_currentPlayer % m_playersCount]->GetUserData();

		inputs.m_wristAxis0 = int(m_scene->GetKeyState('Y')) - int(m_scene->GetKeyState('U'));
		inputs.m_wristAxis1 = int(m_scene->GetKeyState('I')) - int(m_scene->GetKeyState('O'));
		inputs.m_openValue = int (m_scene->GetKeyState ('F')) - int (m_scene->GetKeyState ('G'));
		inputs.m_steerValue = int (m_scene->GetKeyState ('D')) - int (m_scene->GetKeyState ('A'));
		inputs.m_throttleValue = int (m_scene->GetKeyState ('W')) - int (m_scene->GetKeyState ('S'));

		inputs.m_slideValue = m_slideboom;
		inputs.m_liftValue = m_liftboom * dDegreeToRad;
		inputs.m_turnValue = m_rotatebase * dDegreeToRad;

		// check if we must activate the player
		if (m_needsWakeUp ||
			m_scene->GetKeyState ('A') || 
			m_scene->GetKeyState ('D') ||
			m_scene->GetKeyState ('W') ||
			m_scene->GetKeyState ('S'))
		{
			NewtonBody* const body = m_player[m_currentPlayer % m_playersCount]->GetRoot()->m_body;
			NewtonBodySetSleepState(body, false);
		}

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
		vehicleModel->SetInput (inputs);
	}

	static void UpdateCameraCallback(DemoEntityManager* const manager, void* const context, dFloat timestep)
	{
		ServoInputManager* const me = (ServoInputManager*)context;
		me->UpdateCamera(timestep);
	}

	void UpdateCamera (dFloat timestepInSecunds)
	{
		DemoCamera* const camera = m_scene->GetCamera();
		ServoEntityModel* const vehicleModel = (ServoEntityModel*) m_player[m_currentPlayer % m_playersCount]->GetUserData();

		if (m_changeVehicle.UpdateTrigger(m_scene->GetKeyState ('P'))) {
			m_currentPlayer ++;
		}
		
		dMatrix camMatrix(camera->GetNextMatrix());
		dMatrix playerMatrix (vehicleModel->GetNextMatrix());

		dVector frontDir (camMatrix[0]);
		dVector camOrigin(0.0f); 
		m_cameraMode.UpdatePushButton(m_scene->GetKeyState('C'));
		if (m_cameraMode.GetPushButtonState()) {
			camOrigin = playerMatrix.TransformVector( dVector(0.0f, SERVO_VEHICLE_CAMERA_HIGH_ABOVE_HEAD, 0.0f, 0.0f));
			camOrigin -= frontDir.Scale(SERVO_VEHICLE_CAMERA_DISTANCE);
		} else {
			camMatrix = camMatrix * playerMatrix;
			camOrigin = playerMatrix.TransformVector(dVector(-0.8f, SERVO_VEHICLE_CAMERA_EYEPOINT, 0.0f, 0.0f));
		}

		camera->SetNextMatrix (*m_scene, camMatrix, camOrigin);
	}

	void OnEndUpdate (dFloat timestepInSecunds)
	{
	}

	void AddPlayer(dCustomArticulatedTransformController* const player)
	{
		m_player[m_playersCount] = player;
		m_playersCount ++;
	}

	void RenderPlayerHelp (DemoEntityManager* const scene)
	{
		dVector color(1.0f, 1.0f, 0.0f, 0.0f);
		scene->Print (color, "Navigation Keys");
		scene->Print (color, "drive forward:      W");
		scene->Print (color, "drive backward:     S");
		scene->Print (color, "turn right:         D");
		scene->Print (color, "turn left:          A");

		m_needsWakeUp = false;
		m_needsWakeUp = ImGui::SliderFloat("BoomSlide", &m_slideboom, 0.0f, 3.0f) || m_needsWakeUp;
		m_needsWakeUp = ImGui::SliderFloat("BoomLift", &m_liftboom, -180.0f, 180.0f) || m_needsWakeUp;
		m_needsWakeUp = ImGui::SliderFloat("RotateBase", &m_rotatebase, -180.0f, 180.0f) || m_needsWakeUp;
		

		//scene->Print (color, "open palette:       F");
		//scene->Print (color, "close palette       G");
		//scene->Print (color, "lift palette:       E");
		//scene->Print (color, "lower palette       Q");
		//scene->Print (color, "tilt forward:       Z");
		//scene->Print (color, "tilt backward:      X");
		//scene->Print (color, "turn base left:     R");
		//scene->Print (color, "turn base right:    T");
		//scene->Print (color, "toggle camera mode: C");
		//scene->Print (color, "switch vehicle:     P");
	}				

	static void RenderPlayerHelp (DemoEntityManager* const scene, void* const context)
	{
		ServoInputManager* const me = (ServoInputManager*) context;
		me->RenderPlayerHelp (scene);
	}


	DemoEntityManager* m_scene;
	dCustomArticulatedTransformController* m_player[2];
	DemoEntityManager::ButtonKey m_cameraMode;
	DemoEntityManager::ButtonKey m_changeVehicle;
	int m_playersCount;
	int m_currentPlayer;
	bool m_needsWakeUp;
	dFloat32 m_liftboom; 
	dFloat32 m_slideboom; 
	dFloat32 m_rotatebase; 
};

static void LoadLumberYardMesh (DemoEntityManager* const scene, const DemoEntity& entity, const dVector& location)
{
	dTree<NewtonCollision*, DemoMesh*> filter;
	NewtonWorld* const world = scene->GetNewton();

	dFloat density = 15.0f; 

	int defaultMaterialID = NewtonMaterialGetDefaultGroupID (scene->GetNewton());	
	for (DemoEntity* child = entity.GetFirst(); child; child = child->GetNext()) {
		DemoMesh* const mesh = (DemoMesh*)child->GetMesh();
		if (mesh) {
            dAssert (mesh->IsType(DemoMesh::GetRttiType()));
  			dTree<NewtonCollision*, DemoMesh*>::dTreeNode* node = filter.Find(mesh);
			if (!node) {
				// make a collision shape only for and instance
				dFloat* const array = mesh->m_vertex;
				dVector minBox(1.0e10f, 1.0e10f, 1.0e10f, 1.0f);
				dVector maxBox(-1.0e10f, -1.0e10f, -1.0e10f, 1.0f);
				
				for (int i = 0; i < mesh->m_vertexCount; i ++) {
					dVector p (array[i * 3 + 0], array[i * 3 + 1], array[i * 3 + 2], 1.0f);
					minBox.m_x = dMin (p.m_x, minBox.m_x);
					minBox.m_y = dMin (p.m_y, minBox.m_y);
					minBox.m_z = dMin (p.m_z, minBox.m_z);

					maxBox.m_x = dMax (p.m_x, maxBox.m_x);
					maxBox.m_y = dMax (p.m_y, maxBox.m_y);
					maxBox.m_z = dMax (p.m_z, maxBox.m_z);
				}

				dVector size (maxBox - minBox);
				dMatrix offset (dGetIdentityMatrix());
				offset.m_posit = (maxBox + minBox).Scale (0.5f);
				NewtonCollision* const shape = NewtonCreateBox(world, size.m_x, size.m_y, size.m_z, SERVO_VEHICLE_DEFINITION::m_landPart, &offset[0][0]);
				node = filter.Insert (shape, mesh);
			}

			// create a body and add to the world
			NewtonCollision* const shape = node->GetInfo();
			dMatrix matrix (child->GetMeshMatrix() * child->CalculateGlobalMatrix());
			matrix.m_posit += location;
			dFloat mass = density * NewtonConvexCollisionCalculateVolume(shape);
			CreateSimpleSolid (scene, mesh, mass, matrix, shape, defaultMaterialID);
		}
	}

	// destroy all shapes
	while (filter.GetRoot()) {
		NewtonCollision* const shape = filter.GetRoot()->GetInfo();
		NewtonDestroyCollision(shape);
		filter.Remove(filter.GetRoot());
	}
}
#endif


class ServoEntityModel: public DemoEntity
{
	public:
	class InputRecord
	{
		public:
		InputRecord()
		{
			memset(this, 0, sizeof(InputRecord));
		}

		//dFloat m_slideValue;
		//dFloat m_turnValue;
		//dFloat m_liftValue;
		//int m_openValue;
		//int m_wristAxis0;
		//int m_wristAxis1;
		int m_steerValue;
		//int m_throttleValue;
	};

	ServoEntityModel(DemoEntityManager* const scene, const char* const name)
		:DemoEntity(dGetIdentityMatrix(), NULL)
		,m_inputs()
		,m_angularActuator0(NULL)
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
		// load the vehicle model
		LoadNGD_mesh(name, scene->GetNewton());
	}

	ServoEntityModel(const ServoEntityModel& copy)
		:DemoEntity(copy)
		,m_inputs()
		,m_angularActuator0(NULL)
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

	DemoEntity* CreateClone() const
	{
		return new ServoEntityModel(*this);
	}

	void SetInput(const InputRecord& inputs)
	{
		m_inputs = inputs;
	}

	InputRecord m_inputs;
	dCustomWheel* m_rearTireJoints[2];
	dCustomWheel* m_frontTiresJoints[2];
	dCustomSliderActuator* m_liftJoints[3];
	dCustomSliderActuator* m_paletteJoints[2];
	dCustomHingeActuator* m_angularActuator0;
};


class ServoInputManager: public dCustomInputManager
{
	public:
	ServoInputManager(DemoEntityManager* const scene)
		:dCustomInputManager(scene->GetNewton())
		,m_scene(scene)
		,m_cameraMode(true)
		,m_changeVehicle(true)
		,m_playersCount(0)
		,m_currentPlayer(0)
//		,m_liftboom(0.0f)
//		,m_slideboom(0.0f)
//		,m_rotatebase(0.0f)
	{
		// plug a callback for 2d help display
		scene->Set2DDisplayRenderFunction(RenderPlayerHelp, NULL, this);
		scene->SetUpdateCameraFunction(UpdateCameraCallback, this);
	}

	void OnBeginUpdate(dFloat timestepInSecunds)
	{
		ServoEntityModel::InputRecord inputs;
		ServoEntityModel* const vehicleModel = (ServoEntityModel*)m_player[m_currentPlayer % m_playersCount]->GetUserData();

		//inputs.m_wristAxis0 = int(m_scene->GetKeyState('Y')) - int(m_scene->GetKeyState('U'));
		//inputs.m_wristAxis1 = int(m_scene->GetKeyState('I')) - int(m_scene->GetKeyState('O'));
		//inputs.m_openValue = int(m_scene->GetKeyState('F')) - int(m_scene->GetKeyState('G'));
		inputs.m_steerValue = int(m_scene->GetKeyState('D')) - int(m_scene->GetKeyState('A'));
		//inputs.m_throttleValue = int(m_scene->GetKeyState('W')) - int(m_scene->GetKeyState('S'));

		//inputs.m_slideValue = m_slideboom;
		//inputs.m_liftValue = m_liftboom * dDegreeToRad;
		//inputs.m_turnValue = m_rotatebase * dDegreeToRad;

		// check if we must activate the player
		if (m_needsWakeUp ||
			m_scene->GetKeyState('A') ||
			m_scene->GetKeyState('D') ||
			m_scene->GetKeyState('W') ||
			m_scene->GetKeyState('S')) {
			NewtonBody* const body = m_player[m_currentPlayer % m_playersCount]->GetRoot()->m_body;
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
		vehicleModel->SetInput(inputs);
	}

	static void UpdateCameraCallback(DemoEntityManager* const manager, void* const context, dFloat timestep)
	{
		ServoInputManager* const me = (ServoInputManager*)context;
		me->UpdateCamera(timestep);
	}

	void UpdateCamera(dFloat timestepInSecunds)
	{
		DemoCamera* const camera = m_scene->GetCamera();
		ServoEntityModel* const vehicleModel = (ServoEntityModel*)m_player[m_currentPlayer % m_playersCount]->GetUserData();

		//if (!m_currentPlayer) {
		//	return;
		//}

		if (m_changeVehicle.UpdateTrigger(m_scene->GetKeyState('P'))) {
			m_currentPlayer++;
		}

		dMatrix camMatrix(camera->GetNextMatrix());
		dMatrix playerMatrix(vehicleModel->GetNextMatrix());

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

	void AddPlayer(dCustomArticulatedTransformController* const player)
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
		//m_needsWakeUp = ImGui::SliderFloat("BoomSlide", &m_slideboom, 0.0f, 3.0f) || m_needsWakeUp;
		//m_needsWakeUp = ImGui::SliderFloat("BoomLift", &m_liftboom, -180.0f, 180.0f) || m_needsWakeUp;
		//m_needsWakeUp = ImGui::SliderFloat("RotateBase", &m_rotatebase, -180.0f, 180.0f) || m_needsWakeUp;
	}

	static void RenderPlayerHelp(DemoEntityManager* const scene, void* const context)
	{
		ServoInputManager* const me = (ServoInputManager*)context;
		me->RenderPlayerHelp(scene);
	}


	DemoEntityManager* m_scene;
	dCustomArticulatedTransformController* m_player[2];
	DemoEntityManager::ButtonKey m_cameraMode;
	DemoEntityManager::ButtonKey m_changeVehicle;
	int m_playersCount;
	int m_currentPlayer;
	bool m_needsWakeUp;
//	dFloat32 m_liftboom;
//	dFloat32 m_slideboom;
//	dFloat32 m_rotatebase;
};


class ServoVehicleManagerManager: public dCustomArticulaledTransformManager
{
	public:
	ServoVehicleManagerManager(DemoEntityManager* const scene)
		:dCustomArticulaledTransformManager(scene->GetNewton())
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

	static int OnBoneAABBOverlap(const NewtonMaterial* const material, const NewtonBody* const body0, const NewtonBody* const body1, int threadIndex)
	{
		NewtonCollision* const collision0 = NewtonBodyGetCollision(body0);
		NewtonCollision* const collision1 = NewtonBodyGetCollision(body1);
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
			//case SERVO_VEHICLE_DEFINITION::m_terrain | SERVO_VEHICLE_DEFINITION::m_landPart:
			//case SERVO_VEHICLE_DEFINITION::m_terrain | SERVO_VEHICLE_DEFINITION::m_linkPart:
			//case SERVO_VEHICLE_DEFINITION::m_tirePart | SERVO_VEHICLE_DEFINITION::m_bodyPart:
			//case SERVO_VEHICLE_DEFINITION::m_tirePart | SERVO_VEHICLE_DEFINITION::m_linkPart:
			//case SERVO_VEHICLE_DEFINITION::m_tirePart | SERVO_VEHICLE_DEFINITION::m_landPart:
			//case SERVO_VEHICLE_DEFINITION::m_landPart | SERVO_VEHICLE_DEFINITION::m_bodyPart:
			//case SERVO_VEHICLE_DEFINITION::m_landPart | SERVO_VEHICLE_DEFINITION::m_linkPart:
			//case SERVO_VEHICLE_DEFINITION::m_landPart | SERVO_VEHICLE_DEFINITION::m_landPart:
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


	static int CompoundSubCollisionAABBOverlap (const NewtonMaterial* const material, const NewtonBody* const body0, const void* const collisionNode0, const NewtonBody* const body1, const void* const collisionNode1, int threadIndex)
	{
		dAssert(0);
		return 0;
/*
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
			case SERVO_VEHICLE_DEFINITION::m_tirePart | SERVO_VEHICLE_DEFINITION::m_terrain:
			case SERVO_VEHICLE_DEFINITION::m_tirePart | SERVO_VEHICLE_DEFINITION::m_landPart:
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
*/
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

	virtual void OnUpdateTransform(const dCustomArticulatedTransformController::dSkeletonBone* const bone, const dMatrix& localMatrix) const
	{
		DemoEntity* const ent = (DemoEntity*)NewtonBodyGetUserData(bone->m_body);
		if (ent) {
			DemoEntityManager* const scene = (DemoEntityManager*)NewtonWorldGetUserData(NewtonBodyGetWorld(bone->m_body));

			dQuaternion rot(localMatrix);
			ent->SetMatrix(*scene, rot, localMatrix.m_posit);
		}
	}


	virtual void OnPreUpdate (dCustomArticulatedTransformController* const controller, dFloat timestep, int threadIndex) const
	{
		ServoEntityModel* const vehicleModel = (ServoEntityModel*)controller->GetUserData();
/*
		if (vehicleModel->m_engineJoint) {
			dFloat brakeTorque = 2000.0f;
			//dFloat engineTorque = 0.0f;
			dFloat engineRPM = 0.0f;
			if (vehicleModel->m_inputs.m_throttleValue > 0) {
				brakeTorque = 0.0f;
				engineRPM = -vehicleModel->m_maxEngineSpeed;
			} else if (vehicleModel->m_inputs.m_throttleValue < 0) {
				brakeTorque = 0.0f;
				engineRPM = vehicleModel->m_maxEngineSpeed;
			}

			// apply DC engine torque
			vehicleModel->m_engineMotor->SetSpeed1(engineRPM);

			if (!vehicleModel->m_rearTiresCount) {
				// apply DC rate turn Motor 
				if (vehicleModel->m_inputs.m_steerValue > 0) {
					brakeTorque = 0.0f;
					vehicleModel->m_engineMotor->SetSpeed(vehicleModel->m_maxTurmVelocity);
				} else if (vehicleModel->m_inputs.m_steerValue < 0){
					brakeTorque = 0.0f;
					vehicleModel->m_engineMotor->SetSpeed(-vehicleModel->m_maxTurmVelocity);
				} else {
					vehicleModel->m_engineMotor->SetSpeed(0.0f);
				}
			}

			// apply breaks
			for (int i = 0; i < vehicleModel->m_tractionTiresCount; i ++) {
				vehicleModel->m_tractionTiresJoints[i]->SetFriction(brakeTorque);
			}
		}
*/

		// update steering wheels
		dFloat steeringAngle = vehicleModel->m_rearTireJoints[0]->GetSteerAngle();
		if (vehicleModel->m_inputs.m_steerValue > 0) {
			steeringAngle = 30.0f * dDegreeToRad;
		} else if (vehicleModel->m_inputs.m_steerValue < 0) {
			steeringAngle = -30.0f * dDegreeToRad;
		}
		vehicleModel->m_rearTireJoints[0]->SetTargetSteerAngle(steeringAngle);
		vehicleModel->m_rearTireJoints[1]->SetTargetSteerAngle(steeringAngle);

#if 0
		// set the base turn angle
//		vehicleModel->m_angularActuator1->SetTargetAngle(vehicleModel->m_inputs.m_turnValue);

		// set the tilt angle
/*
		dFloat tiltAngle = vehicleModel->m_tiltAngle;
		if (vehicleModel->m_inputs.m_liftValue > 0) {
			tiltAngle = vehicleModel->m_angularActuator0[0]->GetMinAngularLimit();
			vehicleModel->m_tiltAngle = vehicleModel->m_angularActuator0[0]->GetActuatorAngle();
		} else if (vehicleModel->m_inputs.m_liftValue < 0) {
			tiltAngle = vehicleModel->m_angularActuator0[0]->GetMaxAngularLimit();
			vehicleModel->m_tiltAngle = vehicleModel->m_angularActuator0[0]->GetActuatorAngle();
		}

		for (int i = 0; i < vehicleModel->m_angularActuatorsCount0; i ++) {
			vehicleModel->m_angularActuator0[i]->SetTargetAngle (tiltAngle);
		}
*/



		vehicleModel->m_angularActuator0->SetTargetAngle(vehicleModel->m_inputs.m_liftValue);


		// set the lift position
//		if (vehicleModel->m_liftActuatorsCount) 
		{
/*
			dFloat liftPosit = vehicleModel->m_liftPosit;
			if (vehicleModel->m_inputs.m_liftValue > 0) {
				liftPosit = vehicleModel->m_liftJoints[0]->GetMinPositLimit();
				vehicleModel->m_liftPosit = vehicleModel->m_liftJoints[0]->GetActuatorPosit();
			} else if (vehicleModel->m_inputs.m_liftValue < 0) {
				liftPosit = vehicleModel->m_liftJoints[0]->GetMaxPositLimit();
				vehicleModel->m_liftPosit = vehicleModel->m_liftJoints[0]->GetActuatorPosit();
			}
			for (int i = 0; i < vehicleModel->m_liftActuatorsCount; i ++) {
				vehicleModel->m_liftJoints[i]->SetTargetPosit(liftPosit);
			}
*/
		}

		vehicleModel->m_liftJoints[0]->SetTargetPosit(vehicleModel->m_inputs.m_slideValue);
		vehicleModel->m_liftJoints[1]->SetTargetPosit(vehicleModel->m_inputs.m_slideValue);
		

		// open Close palette position
		if (vehicleModel->m_paletteActuatorsCount) {
			dFloat openPosit = vehicleModel->m_openPosit;
			if (vehicleModel->m_inputs.m_openValue > 0) {
				openPosit = vehicleModel->m_paletteJoints[0]->GetMinPositLimit();
				vehicleModel->m_openPosit = vehicleModel->m_paletteJoints[0]->GetActuatorPosit();
			} else if (vehicleModel->m_inputs.m_openValue < 0) {
				openPosit = vehicleModel->m_paletteJoints[0]->GetMaxPositLimit();
				vehicleModel->m_openPosit = vehicleModel->m_paletteJoints[0]->GetActuatorPosit();
			}
			for (int i = 0; i < vehicleModel->m_paletteActuatorsCount; i ++) {
				vehicleModel->m_paletteJoints[i]->SetTargetPosit(openPosit);
			}
		}

		// open Close palette position
		if (vehicleModel->m_universalActuatorsCount) {
			dAssert (0);
/*
			dFloat posit0 = vehicleModel->m_wristAxis0;
			if (vehicleModel->m_inputs.m_wristAxis0 > 0) {
				posit0 = vehicleModel->m_universalActuator[0]->GetMinAngularLimit_0();
				vehicleModel->m_wristAxis0 = vehicleModel->m_universalActuator[0]->GetJointAngle_0();
			} else if (vehicleModel->m_inputs.m_wristAxis0 < 0) {
				posit0 = vehicleModel->m_universalActuator[0]->GetMaxAngularLimit_0();
				vehicleModel->m_wristAxis0 = vehicleModel->m_universalActuator[0]->GetJointAngle_1();
			}

			dFloat posit1 = vehicleModel->m_wristAxis1;
			if (vehicleModel->m_inputs.m_wristAxis1 > 0) {
				posit1 = vehicleModel->m_universalActuator[0]->GetMinAngularLimit_1();
				vehicleModel->m_wristAxis1 = vehicleModel->m_universalActuator[0]->GetJointAngle_1();
			} else if (vehicleModel->m_inputs.m_wristAxis1 < 0) {
				posit1 = vehicleModel->m_universalActuator[0]->GetMaxAngularLimit_1();
				vehicleModel->m_wristAxis1 = vehicleModel->m_universalActuator[0]->GetJointAngle_1();
			}

			for (int i = 0; i < vehicleModel->m_universalActuatorsCount; i ++) {
				vehicleModel->m_universalActuator[i]->SetTargetAngle0(posit0);
				vehicleModel->m_universalActuator[i]->SetTargetAngle1(posit1);
			}
*/
		}
#endif
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

		oririn.m_posit.m_y -= maxExtend.m_y * 0.5f;
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

		// save the user data with the bone body (usually the visual geometry)
		NewtonBodySetUserData(body, bodyPart);

		// assign a body part id
		NewtonCollisionSetUserID(collision, definition.m_bodyPartID);

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
		dFloat angularRate = 10.0f * dDegreeToRad;
		return new dCustomHingeActuator (&baseMatrix[0][0], angularRate, minAngleLimit, maxAngleLimit, child, parent);
	}

	dCustomSliderActuator* LinkLiftActuator(NewtonBody* const parent, NewtonBody* const child)
	{
		dMatrix baseMatrix;
		NewtonBodyGetMatrix(child, &baseMatrix[0][0]);

		dFloat minLimit = -0.25f;
		dFloat maxLimit = 1.5f;
		dFloat linearRate = 0.125f;
		return new dCustomSliderActuator(&baseMatrix[0][0], linearRate, minLimit, maxLimit, child, parent);
	}

	dCustomSliderActuator* LinkPaletteActuator(NewtonBody* const parent, NewtonBody* const child)
	{
		dMatrix baseMatrix;
		NewtonBodyGetMatrix(child, &baseMatrix[0][0]);

		dFloat minLimit = -0.25f;
		dFloat maxLimit = 0.2f;
		dFloat linearRate = 0.25f;
		return new dCustomSliderActuator(&baseMatrix[0][0], linearRate, minLimit, maxLimit, child, parent);
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

		//dFloat angleLimit = 30.0f * dDegreeToRad;
		dFloat angularRate = 60.0f * dDegreeToRad;
		dCustomWheel* const wheel = new dCustomWheel(&chassisMatrix[0][0], tire, chassis);
		wheel->SetSteerRate(angularRate);
		wheel->EnableLimits(true);
		wheel->SetLimits(0.0f, 0.0f);
		return wheel;
	}

	void ConnectBodyPart(ServoEntityModel* const vehicleModel, NewtonBody* const parent, NewtonBody* const child, const dString& jointArticulation, dList<dCustomJoint*>& cycleLinks)
	{
		if (jointArticulation == "") {
			// this is the root body do nothing
		} else if (jointArticulation == "frontTire") {
			//vehicleModel->LinkFrontTire(parent, child, cycleLinks);
			dCustomWheel* const tire = LinkTireJoint(parent, child);
			if (!vehicleModel->m_frontTiresJoints[0]) {
				vehicleModel->m_frontTiresJoints[0] = tire;
			} else {
				dAssert(!vehicleModel->m_frontTiresJoints[1]);
				vehicleModel->m_frontTiresJoints[1] = tire;
			}

		} else if (jointArticulation == "rearTire") {
			dCustomWheel* const tire = LinkTireJoint(parent, child);
			if (!vehicleModel->m_rearTireJoints[0]) {
				vehicleModel->m_rearTireJoints[0] = tire;
			} else {
				dAssert(!vehicleModel->m_rearTireJoints[1]);
				vehicleModel->m_rearTireJoints[1] = tire;
			}
		} else if (jointArticulation == "hingeActuator") {
			vehicleModel->m_angularActuator0 = LinkHingeActuator(parent, child);
		} else if (jointArticulation == "liftActuator") {
			dCustomSliderActuator* const lift = LinkLiftActuator(parent, child);;
			if (!vehicleModel->m_liftJoints[0]) {
				vehicleModel->m_liftJoints[0] = lift;
			} else if (!vehicleModel->m_liftJoints[1]) {
				dAssert(!vehicleModel->m_liftJoints[1]);
				vehicleModel->m_liftJoints[1] = lift;
			} else {
				dAssert(!vehicleModel->m_liftJoints[2]);
				vehicleModel->m_liftJoints[2] = lift;
			}

		} else if (jointArticulation == "paletteActuator") {
			//vehicleModel->LinkPaletteActuator(parent, child);
			dCustomSliderActuator* const lift = LinkLiftActuator(parent, child);;
			if (!vehicleModel->m_paletteJoints[0]) {
				vehicleModel->m_paletteJoints[0] = lift;
			} else {
				dAssert(!vehicleModel->m_paletteJoints[1]);
				vehicleModel->m_paletteJoints[1] = lift;
			}
		} else {
			dAssert(0);
		}
	}

	dCustomArticulatedTransformController* CreateForklift(const dMatrix& location, const DemoEntity* const model, int bodyPartsCount, SERVO_VEHICLE_DEFINITION* const definition)
	{
		NewtonWorld* const world = GetWorld();
		DemoEntityManager* const scene = (DemoEntityManager*)NewtonWorldGetUserData(world);

		// make a clone of the mesh 
		ServoEntityModel* const vehicleModel = (ServoEntityModel*)model->CreateClone();
		scene->Append(vehicleModel);

		// plane the model at its location
		vehicleModel->ResetMatrix(*scene, location);

		dCustomArticulatedTransformController* const controller = CreateTransformController();

		controller->SetUserData(vehicleModel);
		controller->SetCalculateLocalTransforms(true);

		DemoEntity* const rootEntity = (DemoEntity*)vehicleModel->Find(definition[0].m_boneName);

		NewtonBody* const rootBody = CreateBodyPart(rootEntity, definition[0]);

		// move the center of mass a lithe to the back, and lower
		dVector com(0.0f);
		NewtonBodyGetCentreOfMass(rootBody, &com[0]);
		//com.m_x -= 0.25f;
		com.m_y -= 0.25f;
		NewtonBodySetCentreOfMass(rootBody, &com[0]);

//NewtonBodySetMassMatrix(rootBody, 0.0f, 0.0f, 0.0f, 0.0f);

		// add the root bone to the articulation manager
		dCustomArticulatedTransformController::dSkeletonBone* const chassisBone = controller->AddRoot(rootBody, dGetIdentityMatrix());

		// add engine
		//dCustomArticulatedTransformController::dSkeletonBone* const engineBone = CreateEngineNode(controller, chassisBone);
		//vehicleModel->m_engineJoint = (dCustomDoubleHinge*)engineBone->FindJoint();
		//vehicleModel->m_engineMotor = CreateEngineMotor(controller, vehicleModel->m_engineJoint);

		// set power parameter for a simple DC engine
		//		dFloat maxOmega = 40.0f;
		//		vehicleModel->m_maxEngineTorque = -400.0f;
		//		vehicleModel->m_omegaResistance = 1.0f / maxOmega;
		//		vehicleModel->m_maxTurmDamp = 0.0f;
		//		vehicleModel->m_maxTurmVelocity = 0.0f;

		// walk down the model hierarchy an add all the components 
		int stackIndex = 0;
		DemoEntity* childEntities[32];
		dCustomArticulatedTransformController::dSkeletonBone* parentBones[32];
		for (DemoEntity* child = rootEntity->GetChild(); child; child = child->GetSibling()) {
			parentBones[stackIndex] = chassisBone;
			childEntities[stackIndex] = child;
			stackIndex++;
		}

		dList<dCustomJoint*> cycleLinks;
		while (stackIndex) {
			stackIndex--;
			DemoEntity* const entity = childEntities[stackIndex];
			dCustomArticulatedTransformController::dSkeletonBone* parentBone = parentBones[stackIndex];

			const char* const name = entity->GetName().GetStr();
			for (int i = 0; i < bodyPartsCount; i++) {
				if (!strcmp(definition[i].m_boneName, name)) {
					NewtonBody* const bone = CreateBodyPart(entity, definition[i]);

					// connect this body part to its parent with a vehicle joint
					ConnectBodyPart(vehicleModel, parentBone->m_body, bone, definition[i].m_articulationName, cycleLinks);

					dMatrix bindMatrix(entity->GetParent()->CalculateGlobalMatrix((DemoEntity*)NewtonBodyGetUserData(parentBone->m_body)).Inverse());
					parentBone = controller->AddBone(bone, bindMatrix, parentBone);
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


void ServoJoints (DemoEntityManager* const scene)
{
	// load the sky box
	scene->CreateSkyBox();
	NewtonBody* const floor = CreateLevelMesh (scene, "flatPlane.ngd", true);
	//CreateHeightFieldTerrain (scene, 9, 8.0f, 1.5f, 0.2f, 200.0f, -50.0f);
	NewtonCollision* const floorCollision = NewtonBodyGetCollision(floor);
	NewtonCollisionSetUserID(floorCollision, SERVO_VEHICLE_DEFINITION::m_terrain);

	NewtonWorld* const world = scene->GetNewton();
	dVector origin (FindFloor (world, dVector (-10.0f, 50.0f, 0.0f, 1.0f), 2.0f * 50.0f));

	// add an input Manage to manage the inputs and user interaction 
	ServoInputManager* const inputManager = new ServoInputManager (scene);

	//  create a skeletal transform controller for controlling rag doll
	ServoVehicleManagerManager* const vehicleManager = new ServoVehicleManagerManager (scene);

	dMatrix matrix (dGetIdentityMatrix());
	matrix.m_posit = FindFloor (world, origin, 100.0f);
	matrix.m_posit.m_y += 1.5f;

	// load a the mesh of the articulate vehicle
	ServoEntityModel forkliftModel(scene, "forklift.ngd");
	dCustomArticulatedTransformController* const forklift = vehicleManager->CreateForklift(matrix, &forkliftModel, sizeof(forkliftDefinition) / sizeof (forkliftDefinition[0]), forkliftDefinition);
	inputManager->AddPlayer(forklift);
/*
	// add some object to play with
//	DemoEntity entity (dGetIdentityMatrix(), NULL);
//	entity.LoadNGD_mesh ("lumber.ngd", scene->GetNewton());
//	LoadLumberYardMesh (scene, entity, dVector(10.0f, 0.0f, 0.0f, 0.0f));
//	LoadLumberYardMesh (scene, entity, dVector(40.0f, 0.0f, 0.0f, 0.0f));
//	LoadLumberYardMesh (scene, entity, dVector(10.0f, 0.0f, 10.0f, 0.0f));
//	LoadLumberYardMesh (scene, entity, dVector(20.0f, 0.0f, 10.0f, 0.0f));
//	LoadLumberYardMesh (scene, entity, dVector(10.0f, 0.0f, 20.0f, 0.0f));
//	LoadLumberYardMesh (scene, entity, dVector(20.0f, 0.0f, 20.0f, 0.0f));
*/
//	origin.m_x -= 5.0f;
	origin.m_y += 2.0f;
	origin.m_z -= 4.0f;
	dQuaternion rot (dVector (0.0f, 1.0f, 0.0f, 0.0f), -45.0f * dDegreeToRad);  
	scene->SetCameraMatrix(rot, origin);
}



