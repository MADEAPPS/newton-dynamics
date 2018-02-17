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


#define ARTICULATED_VEHICLE_CAMERA_EYEPOINT			1.5f
#define ARTICULATED_VEHICLE_CAMERA_HIGH_ABOVE_HEAD	2.0f
//#define ARTICULATED_VEHICLE_CAMERA_DISTANCE		7.0f
#define ARTICULATED_VEHICLE_CAMERA_DISTANCE			10.0f

struct ARTICULATED_VEHICLE_DEFINITION
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


static ARTICULATED_VEHICLE_DEFINITION forkliftDefinition[] =
{
	{"body",		"convexHull",			900.0f, ARTICULATED_VEHICLE_DEFINITION::m_bodyPart, "mainBody"},
	{"fr_tire",		"tireShape",			 50.0f, ARTICULATED_VEHICLE_DEFINITION::m_tirePart, "frontTire"},
	{"fl_tire",		"tireShape",			 50.0f, ARTICULATED_VEHICLE_DEFINITION::m_tirePart, "frontTire"},
	{"rr_tire",		"tireShape",			 50.0f, ARTICULATED_VEHICLE_DEFINITION::m_tirePart, "rearTire"},
	{"rl_tire",		"tireShape",			 50.0f, ARTICULATED_VEHICLE_DEFINITION::m_tirePart, "rearTire"},
	{"lift_1",		"convexHull",			 50.0f, ARTICULATED_VEHICLE_DEFINITION::m_bodyPart, "hingeActuator"},
	{"lift_2",		"convexHull",			 40.0f, ARTICULATED_VEHICLE_DEFINITION::m_bodyPart, "liftActuator"},
	{"lift_3",		"convexHull",			 30.0f, ARTICULATED_VEHICLE_DEFINITION::m_bodyPart, "liftActuator"},
	{"lift_4",		"convexHull",			 20.0f, ARTICULATED_VEHICLE_DEFINITION::m_bodyPart, "liftActuator"},
	{"left_teeth",  "convexHullAggregate",	 10.0f, ARTICULATED_VEHICLE_DEFINITION::m_bodyPart, "paletteActuator"},
	{"right_teeth", "convexHullAggregate",	 10.0f, ARTICULATED_VEHICLE_DEFINITION::m_bodyPart, "paletteActuator"},
};

class ArticulatedEntityModel: public DemoEntity
{
	public:

	class InputRecord
	{
		public:
		InputRecord()
		{
			memset (this, 0, sizeof (InputRecord));
		}

		int m_turnValue;
		int m_tiltValue;
		int m_liftValue;
		int m_openValue;
		int m_wristAxis0;
		int m_wristAxis1;
		int m_steerValue;
		int m_throttleValue;
	};

	ArticulatedEntityModel (DemoEntityManager* const scene, const char* const name)
		:DemoEntity(dGetIdentityMatrix(), NULL)
		,m_rearTiresCount(0)
		,m_tractionTiresCount(0)
		,m_angularActuatorsCount0(0)
		,m_angularActuatorsCount1(0)
		,m_liftActuatorsCount(0)
		,m_paletteActuatorsCount(0)
		,m_maxEngineTorque(0.0f)
		,m_engineMotor(NULL)
		,m_engineJoint(NULL)
	{
		// load the vehicle model
		LoadNGD_mesh (name, scene->GetNewton());
	}

	ArticulatedEntityModel (const ArticulatedEntityModel& copy)
		:DemoEntity(copy)
		,m_rearTiresCount(0)
		,m_tractionTiresCount(0)
		,m_angularActuatorsCount0(0)
		,m_angularActuatorsCount1(0)
		,m_liftActuatorsCount(0)
		,m_universalActuatorsCount(0)
		,m_paletteActuatorsCount(0)
		,m_maxEngineTorque(0.0f)
		,m_maxEngineSpeed(30.0f)
		,m_maxTurmVelocity(10.0f)
		,m_turnAngle(0.0f)
		,m_tiltAngle(0.0f)
		,m_liftPosit(0.0f)
		,m_openPosit(0.0f)
		,m_steeringTorque(0.0f)
		,m_wristAxis0(0.0f)
		,m_wristAxis1(0.0f)
		,m_engineMotor(NULL)
		,m_engineJoint(NULL)
	{
	}

	DemoEntity* CreateClone() const
	{
		return new ArticulatedEntityModel(*this);
	}

	void SetInput (const InputRecord& inputs)
	{
		m_inputs = inputs;
	}

	void LinkFrontTire (NewtonBody* const chassis, NewtonBody* const tire, dList<dCustomJoint*>& cycleLinks)
	{
		dMatrix tireMatrix;
		dMatrix chassisMatrix;

		// calculate the tire location matrix
		NewtonBodyGetMatrix(tire, &tireMatrix[0][0]);
		NewtonBodyGetMatrix(chassis, &chassisMatrix[0][0]);

		chassisMatrix = dYawMatrix(90.0f * dDegreeToRad) * chassisMatrix;
		chassisMatrix.m_posit = tireMatrix.m_posit;

		dCustomGear* const axel = new dCustomGear(5.0f, chassisMatrix.m_front, chassisMatrix.m_front, tire, m_engineJoint->GetBody0());
		cycleLinks.Append(axel);

		dCustomSlidingContact* const suspenssionTire = new dCustomSlidingContact(&chassisMatrix[0][0], tire, chassis);
		suspenssionTire->EnableLimits(true);
		suspenssionTire->SetLimits(-0.5f, 0.01f);
		suspenssionTire->SetAsSpringDamper(true, 0.9f, 1550.0f, 150.0f);

		m_tractionTiresJoints[m_tractionTiresCount] = suspenssionTire;
		m_tractionTires[m_tractionTiresCount] = tire;
		m_tractionTiresCount ++;
	}

	void LinkRearTire (NewtonBody* const chassis, NewtonBody* const tire)
	{
		dMatrix tireMatrix;
		dMatrix chassisMatrix;

		// calculate the tire location matrix
		NewtonBodyGetMatrix(tire, &tireMatrix[0][0]);
		NewtonBodyGetMatrix(chassis, &chassisMatrix[0][0]);

		chassisMatrix = dYawMatrix(90.0f * dDegreeToRad) * chassisMatrix;
		chassisMatrix.m_posit = tireMatrix.m_posit;

		dFloat angleLimit = 30.0f * dDegreeToRad;
		dFloat angularRate = 60.0f * dDegreeToRad;
		m_rearTireJoints[m_rearTiresCount] = new dCustomUniversalActuator (&chassisMatrix[0][0], angularRate, -angleLimit, angleLimit, angularRate, -angleLimit, angleLimit, tire, chassis);
		m_rearTireJoints[m_rearTiresCount]->SetEnableFlag0 (false);
		m_rearTiresCount ++;
	}

	void LinkHingeActuator (NewtonBody* const parent, NewtonBody* const child)
	{
		dMatrix baseMatrix;
		NewtonBodyGetMatrix (child, &baseMatrix[0][0]);

		dFloat minAngleLimit = -20.0f * dDegreeToRad;
		dFloat maxAngleLimit =  20.0f * dDegreeToRad;
		dFloat angularRate = 10.0f * dDegreeToRad;
		m_angularActuator0[m_angularActuatorsCount0] = new dCustomHingeActuator (&baseMatrix[0][0], angularRate, minAngleLimit, maxAngleLimit, child, parent);
		m_angularActuatorsCount0 ++;
	}

	void LinkLiftActuator (NewtonBody* const parent, NewtonBody* const child)
	{
		dMatrix baseMatrix;
		NewtonBodyGetMatrix (child, &baseMatrix[0][0]);

		dFloat minLimit = -0.25f;
		dFloat maxLimit = 1.5f;
		dFloat linearRate = 0.125f;
		m_liftJoints[m_liftActuatorsCount] = new dCustomSliderActuator (&baseMatrix[0][0], linearRate, minLimit, maxLimit, child, parent);
		m_liftActuatorsCount ++;
	}

	void LinkPaletteActuator (NewtonBody* const parent, NewtonBody* const child)
	{
		dMatrix baseMatrix;
		NewtonBodyGetMatrix (child, &baseMatrix[0][0]);

		dFloat minLimit = -0.25f;
		dFloat maxLimit = 0.2f;
		dFloat linearRate = 0.25f;
		m_paletteJoints[m_paletteActuatorsCount] = new dCustomSliderActuator (&baseMatrix[0][0], linearRate, minLimit, maxLimit, child, parent);
		m_paletteActuatorsCount ++;
	}

	int m_rearTiresCount;
	int m_tractionTiresCount;
	int m_angularActuatorsCount0;
	int m_angularActuatorsCount1;
	int m_liftActuatorsCount;
	int m_universalActuatorsCount;
	int m_paletteActuatorsCount;
	dFloat m_maxEngineTorque;
	dFloat m_maxEngineSpeed;
	dFloat m_maxTurmVelocity;
	dFloat m_turnAngle;
	dFloat m_tiltAngle;
	dFloat m_liftPosit;
	dFloat m_openPosit;
	dFloat m_steeringTorque;
	dFloat m_wristAxis0;
	dFloat m_wristAxis1;
		
	NewtonBody* m_tractionTires[4];
	dCustomSlidingContact* m_tractionTiresJoints[4];
	
	dCustomSliderActuator* m_liftJoints[4];
	dCustomSliderActuator* m_paletteJoints[4];
	dCustomHingeActuator* m_angularActuator0[4];
	dCustomHingeActuator* m_angularActuator1[4];
	dCustomUniversalActuator* m_rearTireJoints[4];
	dCustomUniversalActuator* m_universalActuator[4];

	dCustomMotor2* m_engineMotor;
	dCustomUniversal* m_engineJoint;

	InputRecord m_inputs;
};

class ArticulatedVehicleManagerManager: public dCustomArticulaledTransformManager
{
	public:

	ArticulatedVehicleManagerManager (DemoEntityManager* const scene)
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
		ArticulatedEntityModel* const vehicleModel = (ArticulatedEntityModel*)controller->GetUserData();

		if (vehicleModel->m_engineJoint) {
			dFloat brakeTorque = 2000.0f;
			//dFloat engineTorque = 0.0f;
			dFloat engineRPM = 0.0f;
			if (vehicleModel->m_inputs.m_throttleValue > 0) {
				brakeTorque = 0.0f;
				//engineTorque = -vehicleModel->m_maxEngineTorque; 
				engineRPM = -vehicleModel->m_maxEngineSpeed;
			} else if (vehicleModel->m_inputs.m_throttleValue < 0) {
				brakeTorque = 0.0f;
				//engineTorque = vehicleModel->m_maxEngineTorque; 
				engineRPM = vehicleModel->m_maxEngineSpeed;
			}

			// apply DC engine torque
/*
			dMatrix chassisMatrix;
			dVector engineOmega(0.0f);
			NewtonBody* const engineBody = vehicleModel->m_engineJoint->GetBody0();
			NewtonBody* const chassisBody = vehicleModel->m_engineJoint->GetBody1();
			NewtonBodyGetOmega(engineBody, &engineOmega[0]);
			NewtonBodyGetMatrix(chassisBody, &chassisMatrix[0][0]);
			chassisMatrix = vehicleModel->m_engineJoint->GetMatrix1() * chassisMatrix;
			engineTorque -= (engineOmega.DotProduct3(chassisMatrix.m_up)) * vehicleModel->m_omegaResistance;
			dVector torque (chassisMatrix.m_up.Scale(engineTorque));
			NewtonBodyAddTorque (engineBody, &torque[0]);
*/			

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
		if (vehicleModel->m_angularActuatorsCount1) {
			dFloat turnAngle = vehicleModel->m_turnAngle;
			if (vehicleModel->m_inputs.m_turnValue > 0) {
				turnAngle = vehicleModel->m_angularActuator1[0]->GetMinAngularLimit();
				vehicleModel->m_turnAngle = vehicleModel->m_angularActuator1[0]->GetActuatorAngle();
			} else if (vehicleModel->m_inputs.m_turnValue < 0) {
				turnAngle = vehicleModel->m_angularActuator1[0]->GetMaxAngularLimit();
				vehicleModel->m_turnAngle = vehicleModel->m_angularActuator1[0]->GetActuatorAngle();
			}

			for (int i = 0; i < vehicleModel->m_angularActuatorsCount1; i++) {
				vehicleModel->m_angularActuator1[i]->SetTargetAngle(turnAngle);
			}
		}

		// set the tilt angle
		if (vehicleModel->m_angularActuatorsCount0) {
			dFloat tiltAngle = vehicleModel->m_tiltAngle;
			if (vehicleModel->m_inputs.m_tiltValue > 0) {
				tiltAngle = vehicleModel->m_angularActuator0[0]->GetMinAngularLimit();
				vehicleModel->m_tiltAngle = vehicleModel->m_angularActuator0[0]->GetActuatorAngle();
			} else if (vehicleModel->m_inputs.m_tiltValue < 0) {
				tiltAngle = vehicleModel->m_angularActuator0[0]->GetMaxAngularLimit();
				vehicleModel->m_tiltAngle = vehicleModel->m_angularActuator0[0]->GetActuatorAngle();
			}

			for (int i = 0; i < vehicleModel->m_angularActuatorsCount0; i ++) {
				vehicleModel->m_angularActuator0[i]->SetTargetAngle (tiltAngle);
			}
		}

		// set the lift position
		if (vehicleModel->m_liftActuatorsCount) {
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
		}

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
		ARTICULATED_VEHICLE_DEFINITION::SHAPES_ID id0 = ARTICULATED_VEHICLE_DEFINITION::SHAPES_ID (NewtonCollisionGetUserID(collision0));
		ARTICULATED_VEHICLE_DEFINITION::SHAPES_ID id1 = ARTICULATED_VEHICLE_DEFINITION::SHAPES_ID (NewtonCollisionGetUserID(collision1));
		dAssert(id0 != ARTICULATED_VEHICLE_DEFINITION::m_tireInnerRing);
		dAssert(id1 != ARTICULATED_VEHICLE_DEFINITION::m_tireInnerRing);

		switch (id0 | id1)
		{
			case ARTICULATED_VEHICLE_DEFINITION::m_linkPart | ARTICULATED_VEHICLE_DEFINITION::m_linkPart:
			case ARTICULATED_VEHICLE_DEFINITION::m_linkPart | ARTICULATED_VEHICLE_DEFINITION::m_bodyPart:
			case ARTICULATED_VEHICLE_DEFINITION::m_tirePart | ARTICULATED_VEHICLE_DEFINITION::m_tirePart:
			{
				return 0;
				break;
			}

			case ARTICULATED_VEHICLE_DEFINITION::m_terrain | ARTICULATED_VEHICLE_DEFINITION::m_bodyPart:
			case ARTICULATED_VEHICLE_DEFINITION::m_terrain | ARTICULATED_VEHICLE_DEFINITION::m_landPart:
			case ARTICULATED_VEHICLE_DEFINITION::m_terrain | ARTICULATED_VEHICLE_DEFINITION::m_linkPart:
			case ARTICULATED_VEHICLE_DEFINITION::m_tirePart | ARTICULATED_VEHICLE_DEFINITION::m_linkPart:
			case ARTICULATED_VEHICLE_DEFINITION::m_tirePart | ARTICULATED_VEHICLE_DEFINITION::m_terrain:
			case ARTICULATED_VEHICLE_DEFINITION::m_tirePart | ARTICULATED_VEHICLE_DEFINITION::m_landPart:
			case ARTICULATED_VEHICLE_DEFINITION::m_landPart | ARTICULATED_VEHICLE_DEFINITION::m_bodyPart:
			case ARTICULATED_VEHICLE_DEFINITION::m_landPart | ARTICULATED_VEHICLE_DEFINITION::m_linkPart:
			case ARTICULATED_VEHICLE_DEFINITION::m_landPart | ARTICULATED_VEHICLE_DEFINITION::m_landPart:
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

		ARTICULATED_VEHICLE_DEFINITION::SHAPES_ID id0 = ARTICULATED_VEHICLE_DEFINITION::SHAPES_ID(NewtonCollisionGetUserID(collision0));
		ARTICULATED_VEHICLE_DEFINITION::SHAPES_ID id1 = ARTICULATED_VEHICLE_DEFINITION::SHAPES_ID(NewtonCollisionGetUserID(collision1));
//		dAssert(id0 != ARTICULATED_VEHICLE_DEFINITION::m_tireInnerRing);
//		dAssert(id1 != ARTICULATED_VEHICLE_DEFINITION::m_tireInnerRing);

		switch (id0 | id1)
		{
			//case ARTICULATED_VEHICLE_DEFINITION::m_terrain | ARTICULATED_VEHICLE_DEFINITION::m_linkPart:			
			//case ARTICULATED_VEHICLE_DEFINITION::m_terrain | ARTICULATED_VEHICLE_DEFINITION::m_tireInnerRing:
			//case ARTICULATED_VEHICLE_DEFINITION::m_linkPart | ARTICULATED_VEHICLE_DEFINITION::m_tireInnerRing:
			case ARTICULATED_VEHICLE_DEFINITION::m_tirePart | ARTICULATED_VEHICLE_DEFINITION::m_terrain:
			case ARTICULATED_VEHICLE_DEFINITION::m_tirePart | ARTICULATED_VEHICLE_DEFINITION::m_landPart:
			case ARTICULATED_VEHICLE_DEFINITION::m_tireInnerRing | ARTICULATED_VEHICLE_DEFINITION::m_linkPart:
			{
				return 1;
				break;
			}

			case ARTICULATED_VEHICLE_DEFINITION::m_tirePart | ARTICULATED_VEHICLE_DEFINITION::m_linkPart:
			case ARTICULATED_VEHICLE_DEFINITION::m_tireInnerRing | ARTICULATED_VEHICLE_DEFINITION::m_landPart:
			case ARTICULATED_VEHICLE_DEFINITION::m_tireInnerRing | ARTICULATED_VEHICLE_DEFINITION::m_terrain:
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

			ARTICULATED_VEHICLE_DEFINITION::SHAPES_ID id0 = ARTICULATED_VEHICLE_DEFINITION::SHAPES_ID(NewtonCollisionGetUserID(collision0));
			ARTICULATED_VEHICLE_DEFINITION::SHAPES_ID id1 = ARTICULATED_VEHICLE_DEFINITION::SHAPES_ID(NewtonCollisionGetUserID(collision1));

			switch (id0 | id1) 
			{
				//case ARTICULATED_VEHICLE_DEFINITION::m_terrain | ARTICULATED_VEHICLE_DEFINITION::m_tireInnerRing:
				//case ARTICULATED_VEHICLE_DEFINITION::m_linkPart | ARTICULATED_VEHICLE_DEFINITION::m_tirePart:
//				{
//					NewtonContactJointRemoveContact(contactJoint, contactList[i]);
//					break;
//				}
				
				case ARTICULATED_VEHICLE_DEFINITION::m_tirePart | ARTICULATED_VEHICLE_DEFINITION::m_terrain:
				case ARTICULATED_VEHICLE_DEFINITION::m_tirePart | ARTICULATED_VEHICLE_DEFINITION::m_linkPart:
				case ARTICULATED_VEHICLE_DEFINITION::m_linkPart | ARTICULATED_VEHICLE_DEFINITION::m_terrain:
				case ARTICULATED_VEHICLE_DEFINITION::m_linkPart | ARTICULATED_VEHICLE_DEFINITION::m_tireInnerRing:
				case ARTICULATED_VEHICLE_DEFINITION::m_landPart | ARTICULATED_VEHICLE_DEFINITION::m_tirePart:
				case ARTICULATED_VEHICLE_DEFINITION::m_landPart | ARTICULATED_VEHICLE_DEFINITION::m_bodyPart:
				case ARTICULATED_VEHICLE_DEFINITION::m_landPart | ARTICULATED_VEHICLE_DEFINITION::m_linkPart:
				case ARTICULATED_VEHICLE_DEFINITION::m_landPart | ARTICULATED_VEHICLE_DEFINITION::m_terrain:
				case ARTICULATED_VEHICLE_DEFINITION::m_landPart | ARTICULATED_VEHICLE_DEFINITION::m_landPart:
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
	
	virtual void OnUpdateTransform (const dCustomArticulatedTransformController::dSkeletonBone* const bone, const dMatrix& localMatrix) const
	{
		DemoEntity* const ent = (DemoEntity*) NewtonBodyGetUserData(bone->m_body);
		if (ent) {
			DemoEntityManager* const scene = (DemoEntityManager*) NewtonWorldGetUserData(NewtonBodyGetWorld(bone->m_body));

			dQuaternion rot (localMatrix);
			ent->SetMatrix (*scene, rot, localMatrix.m_posit);
		}
	}

	NewtonCollision* MakeForkLiftTireShape (DemoEntity* const bodyPart) const
	{
		dFloat radius = 0.0f;
		dFloat maxWidth = 0.0f;
		dFloat minWidth = 0.0f;

		DemoMesh* const mesh = (DemoMesh*)bodyPart->GetMesh();
		dAssert (mesh->IsType(DemoMesh::GetRttiType()));
		const dMatrix& matrix = bodyPart->GetMeshMatrix();
		dFloat* const array = mesh->m_vertex;
		for (int i = 0; i < mesh->m_vertexCount; i ++) {
			dVector p (matrix.TransformVector(dVector(array[i * 3 + 0], array[i * 3 + 1], array[i * 3 + 2], 1.0f)));
			maxWidth = dMax (p.m_x, maxWidth);
			minWidth = dMin (p.m_x, minWidth);
			radius = dMax (p.m_y, radius);
		}
		dFloat width = maxWidth - minWidth;
		radius -= width * 0.5f;
		return NewtonCreateChamferCylinder (GetWorld(), radius, width, 0, NULL);
	}


	NewtonCollision* MakeConvexHull(DemoEntity* const bodyPart) const
	{
		dVector points[1024 * 16];

		DemoMesh* const mesh = (DemoMesh*)bodyPart->GetMesh();
		dAssert (mesh->IsType(DemoMesh::GetRttiType()));
		dAssert (mesh->m_vertexCount && (mesh->m_vertexCount < int (sizeof (points) / sizeof (points[0]))));

		// go over the vertex array and find and collect all vertices's weighted by this bone.
		const dFloat* const array = mesh->m_vertex;
		for (int i = 0; i < mesh->m_vertexCount; i ++) {
			points[i][0] = array[i * 3 + 0];
			points[i][1] = array[i * 3 + 1];
			points[i][2] = array[i * 3 + 2];
			points[i][3] = 0.0f;
		}
		bodyPart->GetMeshMatrix().TransformTriplex(&points[0][0], sizeof (dVector), &points[0][0], sizeof (dVector), mesh->m_vertexCount) ;
		return NewtonCreateConvexHull (GetWorld(), mesh->m_vertexCount, &points[0][0], sizeof (dVector), 1.0e-3f, 0, NULL);
	}

	NewtonCollision* MakeConvexHullAggregate(DemoEntity* const bodyPart) const
	{
		dAssert (bodyPart->GetMesh()->IsType(DemoMesh::GetRttiType()));
		NewtonMesh* const mesh = ((DemoMesh*)bodyPart->GetMesh())->CreateNewtonMesh (GetWorld(), bodyPart->GetMeshMatrix());
		
		NewtonMesh* const convexApproximation = NewtonMeshApproximateConvexDecomposition (mesh, 0.01f, 0.2f, 32, 100, NULL, NULL);
		
		NewtonCollision* const compound = NewtonCreateCompoundCollisionFromMesh (GetWorld(), convexApproximation, 0.001f, 0, 0);

		NewtonMeshDestroy(convexApproximation);
		NewtonMeshDestroy(mesh);
		return compound;
	}

	NewtonBody* CreateBodyPart (DemoEntity* const bodyPart, const ARTICULATED_VEHICLE_DEFINITION& definition) 
	{
		NewtonCollision* shape = NULL;
		if (!strcmp (definition.m_shapeTypeName, "tireShape")) {
			shape = MakeForkLiftTireShape(bodyPart);
		} else if (!strcmp (definition.m_shapeTypeName, "convexHull")) {
			shape = MakeConvexHull(bodyPart);
		} else if (!strcmp (definition.m_shapeTypeName, "convexHullAggregate")) {
			//shape = MakeConvexHullAggregate(bodyPart);
			shape = MakeConvexHull(bodyPart);
		} else {
			dAssert (0);
		}

		// calculate the bone matrix
		dMatrix matrix (bodyPart->CalculateGlobalMatrix());

		NewtonWorld* const world = GetWorld();

		// create the rigid body that will make this bone
		NewtonBody* const body = NewtonCreateDynamicBody (world, shape, &matrix[0][0]);

		// destroy the collision helper shape 
		NewtonDestroyCollision (shape);

		// get the collision from body
		NewtonCollision* const collision = NewtonBodyGetCollision(body);

		// calculate the moment of inertia and the relative center of mass of the solid
		NewtonBodySetMassProperties (body, definition.m_mass, collision);

		// save the user data with the bone body (usually the visual geometry)
		NewtonBodySetUserData(body, bodyPart);

		// assign a body part id
		NewtonCollisionSetUserID(collision, definition.m_bodyPartID);

		// set the bod part force and torque call back to the gravity force, skip the transform callback
		NewtonBodySetForceAndTorqueCallback (body, PhysicsApplyGravityForce);
		return body;
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

	void ConnectBodyPart (ArticulatedEntityModel* const vehicleModel, NewtonBody* const parent, NewtonBody* const child, const dString& jointArticulation, dList<dCustomJoint*>& cycleLinks)
	{
		if (jointArticulation == "") {
			// this is the root body do nothing

		} else if (jointArticulation == "frontTire") {
			vehicleModel->LinkFrontTire (parent, child, cycleLinks);
		} else if (jointArticulation == "rearTire") {
			vehicleModel->LinkRearTire (parent, child);
		} else if (jointArticulation == "hingeActuator") {
			vehicleModel->LinkHingeActuator (parent, child);
		} else if (jointArticulation == "liftActuator") {
			vehicleModel->LinkLiftActuator (parent, child);
		} else if (jointArticulation == "paletteActuator") {
			vehicleModel->LinkPaletteActuator (parent, child);
		} else {
			dAssert (0);
		}
	}

	dCustomArticulatedTransformController* CreateForklift (const dMatrix& location, const DemoEntity* const model, int bodyPartsCount, ARTICULATED_VEHICLE_DEFINITION* const definition)
	{
		NewtonWorld* const world = GetWorld(); 
		DemoEntityManager* const scene = (DemoEntityManager*) NewtonWorldGetUserData(world);

		// make a clone of the mesh 
		ArticulatedEntityModel* const vehicleModel = (ArticulatedEntityModel*) model->CreateClone();
		scene->Append(vehicleModel);

		// plane the model at its location
		vehicleModel->ResetMatrix (*scene, location);

		//dCustomArticulatedTransformController* const controller = CreateTransformController (vehicleModel);
		dCustomArticulatedTransformController* const controller = CreateTransformController ();
		controller->SetUserData(vehicleModel);
		controller->SetCalculateLocalTransforms (true);

		DemoEntity* const rootEntity = (DemoEntity*) vehicleModel->Find (definition[0].m_boneName);
		NewtonBody* const rootBody = CreateBodyPart (rootEntity, definition[0]);

		// move the center of mass a lithe to the back, and lower
		dVector com(0.0f);
		NewtonBodyGetCentreOfMass(rootBody, &com[0]);
		//com.m_x -= 0.25f;
		com.m_y -= 0.25f;
		NewtonBodySetCentreOfMass(rootBody, &com[0]);

		// add the root bone to the articulation manager
		dCustomArticulatedTransformController::dSkeletonBone* const chassisBone = controller->AddRoot (rootBody, dGetIdentityMatrix());

		// add engine
		dCustomArticulatedTransformController::dSkeletonBone* const engineBone = CreateEngineNode(controller, chassisBone);
		vehicleModel->m_engineJoint = (dCustomUniversal*) engineBone->FindJoint();
		vehicleModel->m_engineMotor = CreateEngineMotor(controller, vehicleModel->m_engineJoint);

		dAssert(0);
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
			stackIndex ++;
		}

		dList<dCustomJoint*> cycleLinks;
		while (stackIndex) {
			stackIndex --;
			DemoEntity* const entity = childEntities[stackIndex];
			dCustomArticulatedTransformController::dSkeletonBone* parentBone = parentBones[stackIndex];

			const char* const name = entity->GetName().GetStr();
			for (int i = 0; i < bodyPartsCount; i ++) {
				if (!strcmp (definition[i].m_boneName, name)) {
					NewtonBody* const bone = CreateBodyPart (entity, definition[i]);

					// connect this body part to its parent with a vehicle joint
					ConnectBodyPart (vehicleModel, parentBone->m_body, bone, definition[i].m_articulationName, cycleLinks);

					dMatrix bindMatrix (entity->GetParent()->CalculateGlobalMatrix ((DemoEntity*)NewtonBodyGetUserData (parentBone->m_body)).Inverse());
					parentBone = controller->AddBone (bone, bindMatrix, parentBone);
					break;
				}
			}

			for (DemoEntity* child = entity->GetChild(); child; child = child->GetSibling()) {
				parentBones[stackIndex] = parentBone;
				childEntities[stackIndex] = child;
				stackIndex ++;
			}
		}

		// disable self collision between all body parts
		controller->DisableAllSelfCollision();
		return controller;
	}

	NewtonCollision* MakeRobotTireShape(DemoEntity* const bodyPart, bool hasInnerRing) const
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
			maxWidth = dMax(p.m_y, maxWidth);
			minWidth = dMin(p.m_y, minWidth);
			radius = dMax(p.m_x, radius);
		}
		dFloat width = maxWidth - minWidth;
		radius -= width * 0.5f;

		NewtonCollision* tireShape = NULL;
		dMatrix align (dRollMatrix(90.0f * dDegreeToRad));
		if (hasInnerRing) {
			NewtonCollision* const innerRing = NewtonCreateChamferCylinder(GetWorld(), radius, width, ARTICULATED_VEHICLE_DEFINITION::m_tireInnerRing, &align[0][0]);
			NewtonCollision* const outerRing = NewtonCreateChamferCylinder(GetWorld(), radius + 0.07f, width, ARTICULATED_VEHICLE_DEFINITION::m_tirePart, &align[0][0]);

			tireShape = NewtonCreateCompoundCollision(GetWorld(), ARTICULATED_VEHICLE_DEFINITION::m_tirePart);
			NewtonCompoundCollisionBeginAddRemove(tireShape);
			NewtonCompoundCollisionAddSubCollision(tireShape, innerRing);
			NewtonCompoundCollisionAddSubCollision(tireShape, outerRing);
			NewtonCompoundCollisionEndAddRemove (tireShape);	

			NewtonDestroyCollision(innerRing);
			NewtonDestroyCollision(outerRing);
			
		} else {
			//tireShape =  NewtonCreateChamferCylinder(GetWorld(), radius, width, ARTICULATED_VEHICLE_DEFINITION::m_tireInnerRing, &align[0][0]);
			tireShape = NewtonCreateChamferCylinder(GetWorld(), radius, width, ARTICULATED_VEHICLE_DEFINITION::m_tirePart, &align[0][0]);
		}

		return tireShape;
	}


	dCustomArticulatedTransformController::dSkeletonBone* MakeTireBody(const char* const entName, const char* const tireName, dCustomArticulatedTransformController* const controller, dCustomArticulatedTransformController::dSkeletonBone* const parentBone, bool hasInnerRing)
	{
		NewtonBody* const parentBody = parentBone->m_body;
		DemoEntity* const parentModel = (DemoEntity*)NewtonBodyGetUserData(parentBody);
		DemoEntity* const tireModel = parentModel->Find(entName);

		NewtonCollision* const tireCollision = MakeRobotTireShape(tireModel, hasInnerRing);

		// calculate the bone matrix
		dMatrix matrix(tireModel->CalculateGlobalMatrix());

		NewtonWorld* const world = GetWorld();

		// create the rigid body that will make this bone
		NewtonBody* const tireBody = NewtonCreateDynamicBody(world, tireCollision, &matrix[0][0]);

		// destroy the collision helper shape 
		NewtonDestroyCollision(tireCollision);

		// get the collision from body
		NewtonCollision* const collision = NewtonBodyGetCollision(tireBody);

		// calculate the moment of inertia and the relative center of mass of the solid
		NewtonBodySetMassProperties(tireBody, 30.0f, collision);

		// save the user data with the bone body (usually the visual geometry)
		NewtonBodySetUserData(tireBody, tireModel);

		// set the bod part force and torque call back to the gravity force, skip the transform callback
		NewtonBodySetForceAndTorqueCallback(tireBody, PhysicsApplyGravityForce);
		
		return controller->AddBone(tireBody, dGetIdentityMatrix(), parentBone);
	}

	dCustomArticulatedTransformController::dSkeletonBone* MakeTire(const char* const entName, const char* const tireName, dCustomArticulatedTransformController* const controller, dCustomArticulatedTransformController::dSkeletonBone* const parentBone, bool hasInnerRing)
	{
		dCustomArticulatedTransformController::dSkeletonBone* const bone = MakeTireBody (entName, tireName, controller, parentBone, hasInnerRing);

		// connect the tire the body with a hinge
		dMatrix matrix;
		NewtonBodyGetMatrix (bone->m_body, &matrix[0][0]);
		//dMatrix hingeFrame (dRollMatrix(90.0f * dDegreeToRad) * matrix);
		dMatrix hingeFrame(dRollMatrix(0.0f * dDegreeToRad) * matrix);

		dCustomSlidingContact* const tire = new dCustomSlidingContact(hingeFrame, bone->m_body, parentBone->m_body);
		tire->EnableLimits(true);
		tire->SetLimits(0.0f, 0.0f);
		tire->SetAsSpringDamper(false, 1.0f, 0.0f, 0.0f);
		return bone;
	}

	dCustomArticulatedTransformController::dSkeletonBone* MakeSuspensionTire(const char* const entName, const char* const tireName, dCustomArticulatedTransformController* const controller, dCustomArticulatedTransformController::dSkeletonBone* const parentBone)
	{
		dCustomArticulatedTransformController::dSkeletonBone* const bone = MakeTireBody(entName, tireName, controller, parentBone, true);

		// connect the tire tp the body with a hinge
		dMatrix matrix;
		NewtonBodyGetMatrix(bone->m_body, &matrix[0][0]);
		dMatrix hingeFrame(dRollMatrix (0.0f * dDegreeToRad) * matrix);
		dCustomSlidingContact* const joint = new dCustomSlidingContact(hingeFrame, bone->m_body, parentBone->m_body);
		joint->EnableLimits(true);
		joint->SetLimits(-0.5f, 0.01f);
		joint->SetAsSpringDamper(true, 0.9f, 1550.0f, 150.0f);
		return bone;
	}

	dCustomArticulatedTransformController::dSkeletonBone* MakeTractionTire(const char* const entName, const char* const tireName, dCustomArticulatedTransformController* const controller, dCustomArticulatedTransformController::dSkeletonBone* const parentBone)
	{
		ArticulatedEntityModel* const vehicleModel = (ArticulatedEntityModel*)controller->GetUserData();
		dCustomArticulatedTransformController::dSkeletonBone* const bone = MakeTireBody(entName, tireName, controller, parentBone, false);

		// connect the tire tp the body with a hinge
		dMatrix matrix;
		NewtonBodyGetMatrix(bone->m_body, &matrix[0][0]);
		dMatrix tireHingeMatrix(dRollMatrix(0.0f * dDegreeToRad) * matrix);

		// save tractions tires 
		vehicleModel->m_tractionTires[vehicleModel->m_tractionTiresCount] = bone->m_body;
		vehicleModel->m_tractionTiresJoints[vehicleModel->m_tractionTiresCount] = new dCustomSlidingContact(tireHingeMatrix, bone->m_body, parentBone->m_body);
		vehicleModel->m_tractionTiresJoints[vehicleModel->m_tractionTiresCount]->EnableLimits(true);
		vehicleModel->m_tractionTiresJoints[vehicleModel->m_tractionTiresCount]->SetLimits(0.0f, 0.0f);
		vehicleModel->m_tractionTiresJoints[vehicleModel->m_tractionTiresCount]->SetAsSpringDamper(false, 1.0f, 0.0f, 0.0f);
		vehicleModel->m_tractionTiresCount++;

		// link traction tire to the engine using a differential gear
		dMatrix engineMatrix;
		dMatrix chassisMatrix;
		
		NewtonBody* const tire = bone->m_body;
		NewtonBody* const engine = vehicleModel->m_engineJoint->GetBody0();
		NewtonBody* const chassis = vehicleModel->m_engineJoint->GetBody1();
		vehicleModel->m_engineJoint->CalculateGlobalMatrix(engineMatrix, chassisMatrix);

//dFloat m, x, y, z;
//NewtonBodyGetMass(engine, &m, &x, &y, &z);

		dFloat sign = dSign(engineMatrix.m_up.DotProduct3(tireHingeMatrix.m_posit - engineMatrix.m_posit));
		new dCustomDifferentialGear(5.0f, tireHingeMatrix.m_up, engineMatrix.m_front.Scale(sign), chassisMatrix.m_up, tire, engine, chassis);
		return bone;
	}

	dCustomJoint* LinkTires(dCustomArticulatedTransformController::dSkeletonBone* const master, dCustomArticulatedTransformController::dSkeletonBone* const slave, dCustomArticulatedTransformController::dSkeletonBone* const root)
	{
		NewtonCollisionInfoRecord masterTire;
		NewtonCollisionInfoRecord slaveTire;

		NewtonCollisionGetInfo(NewtonBodyGetCollision(master->m_body), &masterTire);
		NewtonCollisionGetInfo(NewtonBodyGetCollision(slave->m_body), &slaveTire);
		if (slaveTire.m_collisionType == SERIALIZE_ID_COMPOUND) {
			NewtonCollision* const tire = NewtonBodyGetCollision(slave->m_body);
			void* const node = NewtonCompoundCollisionGetNodeByIndex (tire, 1);
			NewtonCollision* const innerRing = NewtonCompoundCollisionGetCollisionFromNode (tire, node);
			NewtonCollisionGetInfo(innerRing, &slaveTire);
		}

		dAssert(masterTire.m_collisionType == SERIALIZE_ID_CHAMFERCYLINDER);
		dAssert(slaveTire.m_collisionType == SERIALIZE_ID_CHAMFERCYLINDER);
		dAssert(masterTire.m_collisionUserID == ARTICULATED_VEHICLE_DEFINITION::m_tirePart);
		dAssert(slaveTire.m_collisionUserID == ARTICULATED_VEHICLE_DEFINITION::m_tirePart);

		dFloat masterRadio = masterTire.m_chamferCylinder.m_height * 0.5f + masterTire.m_chamferCylinder.m_radio;
		dFloat slaveRadio = slaveTire.m_chamferCylinder.m_height * 0.5f + slaveTire.m_chamferCylinder.m_radio;

		dMatrix pinMatrix;
		NewtonBodyGetMatrix(root->m_body, &pinMatrix[0][0]);
		return new dCustomGear(slaveRadio / masterRadio, pinMatrix[2], pinMatrix[2].Scale(-1.0f), slave->m_body, master->m_body);
	}

	void MakeLeftTrack(dCustomArticulatedTransformController* const controller)
	{
		dCustomArticulatedTransformController::dSkeletonBone* const chassisBone = controller->GetRoot();
		
		dCustomArticulatedTransformController::dSkeletonBone* const leftTire_0 = MakeTractionTire ("leftTire_0", "tractionLeftTire", controller, chassisBone);
		dCustomArticulatedTransformController::dSkeletonBone* const leftTire_7 = MakeTire ("leftTire_7", "tire", controller, chassisBone, false);
		LinkTires (leftTire_0, leftTire_7, chassisBone);

		MakeTire ("leftTireSuport_0", "suportTire", controller, chassisBone, false);
		MakeTire ("leftTireSuport_1", "suportTire", controller, chassisBone, false);

		for (int i = 1; i < 7; i++) {
			char name[64];
			sprintf(name, "leftTire_%d", i);
			dCustomArticulatedTransformController::dSkeletonBone* const childBone = MakeSuspensionTire(name, "tire", controller, chassisBone);
			LinkTires (leftTire_0, childBone, chassisBone);
		}
	}

	void MakeRightTrack(dCustomArticulatedTransformController* const controller)
	{
		dCustomArticulatedTransformController::dSkeletonBone* const chassisBone = controller->GetRoot();

		dCustomArticulatedTransformController::dSkeletonBone* const rightTire_0 = MakeTractionTire("rightTire_0", "tractionrightTire", controller, chassisBone);
		dCustomArticulatedTransformController::dSkeletonBone* const rightTire_7 = MakeTire("rightTire_7", "tire", controller, chassisBone, false);
		LinkTires (rightTire_0, rightTire_7, chassisBone);

		MakeTire("rightTireSuport_0", "suportTire", controller, chassisBone, false);
		MakeTire("rightTireSuport_1", "suportTire", controller, chassisBone, false);

		for (int i = 1; i < 7; i++) {
			char name[64];
			sprintf(name, "rightTire_%d", i);
			dCustomArticulatedTransformController::dSkeletonBone* const childBone = MakeSuspensionTire(name, "tire", controller, chassisBone);
			LinkTires(rightTire_0, childBone, chassisBone);
		}
	}
		
	class ConstantSpeedKnotInterpolant
	{
		public:
		dFloat m_u;
		dFloat m_dist;
	};

	dFloat CalculateKnotParam(const ConstantSpeedKnotInterpolant* const interpolants, int interpolantsCount, dFloat t) const
	{
		int low = 0;
		int high = interpolantsCount - 1;

		while ((high - low) >= 4) {
			int mid = (low + high) >> 1;
			if (t > interpolants[mid].m_dist) {
				low = mid;
			} else {
				high = mid;
			}
		}

		dAssert(interpolants[low].m_dist <= t);
		for (int i = low; i < interpolantsCount; i++) {
			if (interpolants[i + 1].m_dist >= t) {
				low = i;
				break;
			}
		}

		dFloat u0 = interpolants[low].m_u;
		dFloat h0 = interpolants[low].m_dist;
		dFloat du = interpolants[low + 1].m_u - u0;
		dFloat dh = interpolants[low + 1].m_dist - h0;
		return dMod(u0 + du * (t - h0) / dh, 1.0f);
	}

	void CalculaterUniformSpaceSamples(DemoEntity* const chassis, dFloat offset, dCustomArticulatedTransformController::dSkeletonBone* const rootNode)
	{
		dFloat linkLength = 0.33f;

		DemoEntity* const threadPath = chassis->Find("trackPath");
		dAssert(threadPath);
		DemoBezierCurve* const bezierPath = (DemoBezierCurve*) threadPath->GetMesh();

		dFloat length = dFloat (bezierPath->m_curve.CalculateLength (0.01f));
		int linksCount = int(dFloor (length / linkLength)) + 1;
		linkLength = length / linksCount;

		// create the uniform speed knot interpolation table
		ConstantSpeedKnotInterpolant steps[2048];
		steps[0].m_u = 0.0f;
		steps[0].m_dist = 0.0f;

		int count = 0;
		steps[count].m_u = 0.0f;
		steps[count].m_dist = 0.0f;
		count++;
		int samplingRate = linksCount * 20;
		dFloat distAcc = 0.0f;

		dFloat stepAcc = linkLength;
		dVector p0(bezierPath->m_curve.CurvePoint(0.0f));
		for (int i = 1; i < samplingRate + 45; i++) {
			dFloat u = dFloat(i) / samplingRate;
			dVector p1(bezierPath->m_curve.CurvePoint(dMod(u, 1.0f)));
			dVector err(p1 - p0);
			dFloat errMag = dSqrt(err.DotProduct3(err));
			distAcc += errMag;
			if (distAcc >= stepAcc) {
				stepAcc += linkLength;
				steps[count].m_u = u;
				steps[count].m_dist = distAcc;
				count++;
				dAssert(count < int(sizeof (steps) / sizeof (steps[0])));
			}
			p0 = p1;
		}

		dMatrix rootMatrix (chassis->GetCurrentMatrix());

		dMatrix aligmentMatrix (dRollMatrix(180.0f * dDegreeToRad));

		DemoEntity* const threadLink = chassis->Find("link");
		dMatrix shapeMatrix (threadPath->GetMeshMatrix() * threadPath->GetCurrentMatrix());

		DemoEntity* const threadLinkChild = chassis->Find("Object001");
		dMatrix shapeChildMatrix (threadLinkChild->GetCurrentMatrix());

		dFloat s = 0.0f;
		dMatrix matrix(dGetIdentityMatrix());
		dFloat u0 = CalculateKnotParam(steps, linksCount, s);
		dVector r0(bezierPath->m_curve.CurvePoint(u0));

		NewtonWorld* const world = GetWorld();
		DemoEntityManager* const scene = (DemoEntityManager*)NewtonWorldGetUserData(world);
		dMatrix linkMatrix (dGetIdentityMatrix());
		linkMatrix.m_posit.m_y = linkLength * 0.5f;
		NewtonCollision* const collision = NewtonCreateBox (GetWorld(), 0.06f, linkLength, 0.5f, 0, &linkMatrix[0][0]);
		NewtonCollisionSetUserID(collision, ARTICULATED_VEHICLE_DEFINITION::m_linkPart);

		NewtonBody* linkArray[1024];

		int bodyCount = 0;
		// add extra link for wiggle room
		//linksCount += 2;
		linksCount += 1;

		void* const aggregate = NewtonCollisionAggregateCreate(world);
		NewtonCollisionAggregateSetSelfCollision (aggregate, 0);
		for (int i = 1; i < linksCount + 1; i++) {
			s += linkLength;
			dFloat u1 = CalculateKnotParam(steps, linksCount, dMod (s, length));
			dVector r1(bezierPath->m_curve.CurvePoint(u1));
			dVector dir(r1 - r0);

			dir = dir.Scale(1.0f / dSqrt(dir.DotProduct3(dir)));
			matrix.m_front = dVector(dir.m_z, -dir.m_y, 0.0f, 0.0f);
			matrix.m_up = dVector(dir.m_y, dir.m_z, 0.0f, 0.0f);
			matrix.m_right = dVector(0.0f, 0.0f, 1.0f, 0.0f);
			matrix = aligmentMatrix * matrix;
			matrix.m_posit = shapeMatrix.TransformVector(r0);
			matrix.m_posit.m_z = offset;

			matrix = matrix * rootMatrix;
			NewtonBody* const link = CreateSimpleSolid (scene, (DemoMesh*)threadLink->GetMesh(), 10.0f, matrix, collision, 0);
			NewtonCollisionAggregateAddBody (aggregate, link);
			linkArray[bodyCount] = link;
			bodyCount ++;
			DemoEntity* const threadPart = (DemoEntity*) NewtonBodyGetUserData(link);

			DemoEntity* const threadPartChild = new DemoEntity(shapeChildMatrix, threadPart);
			threadPartChild->SetMesh(threadLinkChild->GetMesh(), dGetIdentityMatrix());

			r0 = r1;
			u0 = u1;
		}
		NewtonDestroyCollision(collision);

		dMatrix aligment (dYawMatrix(90.0f * dDegreeToRad));
		NewtonBody* link0 = linkArray[0];

		NewtonJoint* hingeArray[1024];
		for (int i = 1; i < bodyCount; i++) {
			NewtonBody* const link1 = linkArray[i];

			dMatrix matrix;
			NewtonBodyGetMatrix(link1, &matrix[0][0]);
			dMatrix franmeMatrix (aligment * matrix);
			dCustomHinge* const hinge = new dCustomHinge (franmeMatrix, link1, link0);
			hinge->SetStiffness(0.99f);
			hinge->SetAsSpringDamper(true, 0.9f, 0.0f, 20.0f);

			hingeArray[i-1] = hinge->GetJoint();
			link0 = link1;
		}

		dMatrix matrix0;
		dMatrix matrix1;
		dMatrix matrix2;
		NewtonBodyGetMatrix(linkArray[0], &matrix0[0][0]);
		NewtonBodyGetMatrix(linkArray[1], &matrix2[0][0]);
		NewtonBodyGetMatrix(linkArray[bodyCount - 1], &matrix1[0][0]);
		
		dVector dist (matrix2.m_posit - matrix0.m_posit);
		matrix1.m_posit += dist;
		dCustomHinge* const hinge = new dCustomHinge (aligment * matrix0, aligment * matrix1, linkArray[0], linkArray[bodyCount - 1]);
		hinge->SetStiffness(0.99f);
		hinge->SetAsSpringDamper(true, 0.9f, 0.0f, 20.0f);

		dVector com (0.0f);
		for (int i = 0; i < bodyCount; i++) {
			dMatrix matrix;
			NewtonBody* const link = linkArray[i];
			NewtonBodyGetMatrix(link, &matrix[0][0]);
			com += matrix.m_posit;
		}
		com = com.Scale (1.0f / bodyCount);
		com.m_w = 1.0f;

		NewtonBody* const chassisBody = rootNode->m_body;
		NewtonBody* const chainBody = linkArray[0];
		dMatrix planeMatrix;
		NewtonBodyGetMatrix(chassisBody, &planeMatrix[0][0]);
		new dCustomPlane (com, planeMatrix.m_right, chainBody, chassisBody);
	}

	void MakeLeftThread(dCustomArticulatedTransformController* const controller)
	{
		dCustomArticulatedTransformController::dSkeletonBone* const chassisBone = controller->GetRoot();
		DemoEntity* const chassis = (DemoEntity*) NewtonBodyGetUserData (chassisBone->m_body);
		DemoEntity* const pivot = chassis->Find ("leftTire_0");
		CalculaterUniformSpaceSamples (chassis, pivot->GetCurrentMatrix().m_posit.m_z, chassisBone);
	}

	void MakeRightThread(dCustomArticulatedTransformController* const controller)
	{
		dCustomArticulatedTransformController::dSkeletonBone* const chassisBone = controller->GetRoot();
		DemoEntity* const chassis = (DemoEntity*)NewtonBodyGetUserData(chassisBone->m_body);
		DemoEntity* const pivot = chassis->Find("rightTire_0");
		CalculaterUniformSpaceSamples(chassis, pivot->GetCurrentMatrix().m_posit.m_z, chassisBone);
	}
	
	dCustomArticulatedTransformController::dSkeletonBone* AddCraneBoom(dCustomArticulatedTransformController* const controller, dCustomArticulatedTransformController::dSkeletonBone* const baseBone, const char* name)
	{
		dCustomArticulatedTransformController::dSkeletonBone* const chassisBone = controller->GetRoot();
		ArticulatedEntityModel* const vehicleModel = (ArticulatedEntityModel*)NewtonBodyGetUserData(chassisBone->m_body);

		DemoEntity* const boom = vehicleModel->Find(name);

		ARTICULATED_VEHICLE_DEFINITION definition;
		strcpy(definition.m_boneName, name);
		strcpy(definition.m_shapeTypeName, "convexHull");
		definition.m_mass = 10.0f;
		definition.m_bodyPartID = ARTICULATED_VEHICLE_DEFINITION::m_bodyPart;
		strcpy(definition.m_articulationName, name);
		NewtonBody* const boomBody = CreateBodyPart(boom, definition);

		dMatrix matrix;
		NewtonBodyGetMatrix(boomBody, &matrix[0][0]);
		matrix = dRollMatrix(0.5f * dPi) * matrix;

		dFloat minLimit =  0.0f;
		dFloat maxLimit =  4.0f;
		dFloat linearRate = 2.0f;
		vehicleModel->m_liftJoints[vehicleModel->m_liftActuatorsCount] = new dCustomSliderActuator(&matrix[0][0], linearRate, minLimit, maxLimit, boomBody, baseBone->m_body);
		vehicleModel->m_liftActuatorsCount++;
		dCustomArticulatedTransformController::dSkeletonBone* const boomBone = controller->AddBone(boomBody, dGetIdentityMatrix(), baseBone);
		return boomBone;
	}

	void AddCranekPaletteActuator(dCustomArticulatedTransformController* const controller, dCustomArticulatedTransformController::dSkeletonBone* const baseBone, const char* const name)
	{
		dCustomArticulatedTransformController::dSkeletonBone* const chassisBone = controller->GetRoot();
		ArticulatedEntityModel* const vehicleModel = (ArticulatedEntityModel*)NewtonBodyGetUserData(chassisBone->m_body);

		DemoEntity* const palette = vehicleModel->Find(name);

		ARTICULATED_VEHICLE_DEFINITION definition;
		strcpy(definition.m_boneName, name);
		strcpy(definition.m_shapeTypeName, "convexHull");
		definition.m_mass = 10.0f;
		definition.m_bodyPartID = ARTICULATED_VEHICLE_DEFINITION::m_bodyPart;
		strcpy(definition.m_articulationName, name);
		NewtonBody* const paletteBody = CreateBodyPart(palette, definition);

		dMatrix matrix;
		NewtonBodyGetMatrix(paletteBody, &matrix[0][0]);
		matrix = dRollMatrix(0.5f * dPi) * matrix;

		dFloat minLimit = -0.01f;
		dFloat maxLimit =  1.25f;
		dFloat rate = 2.0f;

		vehicleModel->m_paletteJoints[vehicleModel->m_paletteActuatorsCount] = new dCustomSliderActuator(&matrix[0][0], rate, minLimit, maxLimit, paletteBody, baseBone->m_body);
		vehicleModel->m_paletteActuatorsCount++;
		controller->AddBone(paletteBody, dGetIdentityMatrix(), baseBone);
	}

	void AddCraneWrist(dCustomArticulatedTransformController* const controller, dCustomArticulatedTransformController::dSkeletonBone* const baseBone)
	{
		dCustomArticulatedTransformController::dSkeletonBone* const chassisBone = controller->GetRoot();
		ArticulatedEntityModel* const vehicleModel = (ArticulatedEntityModel*)NewtonBodyGetUserData(chassisBone->m_body);

		DemoEntity* const wrist = vehicleModel->Find("effector");

		ARTICULATED_VEHICLE_DEFINITION definition;
		strcpy(definition.m_boneName, "effector");
		strcpy(definition.m_shapeTypeName, "convexHull");
		definition.m_mass = 10.0f;
		definition.m_bodyPartID = ARTICULATED_VEHICLE_DEFINITION::m_bodyPart;
		strcpy(definition.m_articulationName, "effector");
		NewtonBody* const wristBody = CreateBodyPart(wrist, definition);

		dMatrix matrix;
		NewtonBodyGetMatrix(wristBody, &matrix[0][0]);
		matrix = dPitchMatrix(0.0f * dPi) * dYawMatrix(0.5f * dPi) * matrix;

		dFloat minAngleLimit = -120.0f * dDegreeToRad;
		dFloat maxAngleLimit =  120.0f * dDegreeToRad;
		dFloat angularRate = 30.0f * dDegreeToRad;

		vehicleModel->m_universalActuator[vehicleModel->m_universalActuatorsCount] = new dCustomUniversalActuator(&matrix[0][0], angularRate, minAngleLimit * 2.0f, maxAngleLimit * 2.0f, angularRate, minAngleLimit, maxAngleLimit, wristBody, baseBone->m_body);
		vehicleModel->m_universalActuatorsCount++;
		dCustomArticulatedTransformController::dSkeletonBone* const wristBone = controller->AddBone(wristBody, dGetIdentityMatrix(), baseBone);
		AddCranekPaletteActuator (controller, wristBone, "leftHand");
		AddCranekPaletteActuator (controller, wristBone, "rightHand");
	}

	void AddCraneLift(dCustomArticulatedTransformController* const controller, dCustomArticulatedTransformController::dSkeletonBone* const baseBone)
	{
		dCustomArticulatedTransformController::dSkeletonBone* const chassisBone = controller->GetRoot();
		ArticulatedEntityModel* const vehicleModel = (ArticulatedEntityModel*)NewtonBodyGetUserData(chassisBone->m_body);

		DemoEntity* const boom = vehicleModel->Find("Boom1");

		ARTICULATED_VEHICLE_DEFINITION definition;
		strcpy(definition.m_boneName, "Boom1");
		strcpy(definition.m_shapeTypeName, "convexHull");
		definition.m_mass = 10.0f;
		definition.m_bodyPartID = ARTICULATED_VEHICLE_DEFINITION::m_bodyPart;
		strcpy(definition.m_articulationName, "Boom1");
		NewtonBody* const boomBody = CreateBodyPart(boom, definition);

		dMatrix matrix;
		NewtonBodyGetMatrix(boomBody, &matrix[0][0]);
		matrix = dRollMatrix(0.5f * dPi) * matrix;

		dFloat minAngleLimit = -60.0f * dDegreeToRad;
		dFloat maxAngleLimit =  10.0f * dDegreeToRad;
		dFloat angularRate = 20.0f * dDegreeToRad;
		vehicleModel->m_angularActuator0[vehicleModel->m_angularActuatorsCount0] = new dCustomHingeActuator(&matrix[0][0], angularRate, minAngleLimit, maxAngleLimit, boomBody, baseBone->m_body);
		vehicleModel->m_angularActuatorsCount0++;
		dCustomArticulatedTransformController::dSkeletonBone* const boomBone1 = controller->AddBone(boomBody, dGetIdentityMatrix(), baseBone);
		dCustomArticulatedTransformController::dSkeletonBone* const boomBone2 = AddCraneBoom (controller, boomBone1, "Boom2");
		dCustomArticulatedTransformController::dSkeletonBone* const boomBone3 = AddCraneBoom (controller, boomBone2, "Boom3");
		AddCraneWrist(controller, boomBone3);
	}

	void AddCraneBase(dCustomArticulatedTransformController* const controller)
	{
		dCustomArticulatedTransformController::dSkeletonBone* const chassisBone = controller->GetRoot();
		ArticulatedEntityModel* const vehicleModel = (ArticulatedEntityModel*)NewtonBodyGetUserData(chassisBone->m_body);

		DemoEntity* const base = vehicleModel->Find("base");

		ARTICULATED_VEHICLE_DEFINITION definition;
		strcpy(definition.m_boneName, "base");
		strcpy(definition.m_shapeTypeName, "convexHull");
		definition.m_mass = 30.0f;
		definition.m_bodyPartID = ARTICULATED_VEHICLE_DEFINITION::m_bodyPart;
		strcpy(definition.m_articulationName, "base");
		NewtonBody* const baseBody = CreateBodyPart(base, definition);

		dMatrix matrix;
		NewtonBodyGetMatrix (baseBody, &matrix[0][0]);
		matrix = dRollMatrix (0.5f * dPi) * matrix;

		dFloat minAngleLimit = -1.0e10f;
		dFloat maxAngleLimit = 1.0e10f;
		dFloat angularRate = 20.0f * dDegreeToRad;
		vehicleModel->m_angularActuator1[vehicleModel->m_angularActuatorsCount1] = new dCustomHingeActuator(&matrix[0][0], angularRate, minAngleLimit, maxAngleLimit, baseBody, chassisBone->m_body);
		vehicleModel->m_angularActuatorsCount1++;
		dCustomArticulatedTransformController::dSkeletonBone* const baseBone = controller->AddBone(baseBody, dGetIdentityMatrix(), chassisBone);
		AddCraneLift(controller, baseBone);
	}

	dCustomArticulatedTransformController* CreateRobot (const dMatrix& location, const DemoEntity* const model, int , ARTICULATED_VEHICLE_DEFINITION* const )
	{
		NewtonWorld* const world = GetWorld();
		DemoEntityManager* const scene = (DemoEntityManager*)NewtonWorldGetUserData(world);

		// make a clone of the mesh 
		ArticulatedEntityModel* const vehicleModel = (ArticulatedEntityModel*)model->CreateClone();
		scene->Append(vehicleModel);

		// plane the model at its location
		dMatrix matrix (vehicleModel->GetCurrentMatrix());
		matrix.m_posit = location.m_posit;
		vehicleModel->ResetMatrix(*scene, matrix);

		dCustomArticulatedTransformController* const controller = CreateTransformController();
		controller->SetUserData(vehicleModel);
		controller->SetCalculateLocalTransforms (true);

		ARTICULATED_VEHICLE_DEFINITION definition;
		strcpy (definition.m_boneName, "body");
		strcpy (definition.m_shapeTypeName, "convexHull");
		definition.m_mass = 800.0f;
		definition.m_bodyPartID = ARTICULATED_VEHICLE_DEFINITION::m_bodyPart;
		strcpy (definition.m_articulationName, "mainBody");
		NewtonBody* const chassis = CreateBodyPart (vehicleModel, definition);

		// hide the thread fenders
		for (int i = 0; i < 4; i ++) {
			char name[64];
			sprintf (name, "fender%d", i);
			DemoEntity* const fender = vehicleModel->Find(name);
			fender->SetMesh(NULL, dGetIdentityMatrix());
		}
		
		// adding fenders to protect the threads
		#if 0
			NewtonCollision* const compound = NewtonCreateCompoundCollision (world, 0);
			NewtonCompoundCollisionBeginAddRemove(compound);
			NewtonCompoundCollisionAddSubCollision (compound, NewtonBodyGetCollision(chassis));
			for (int i = 0; i < 4; i ++) {
				char name[64];
				sprintf (name, "fender%d", i);
				DemoEntity* const fender = vehicleModel->Find(name);
				NewtonCollision* const collision = MakeConvexHull (fender);
				NewtonCollisionSetUserID (collision, ARTICULATED_VEHICLE_DEFINITION::m_fenderPart);
				dMatrix matrix(fender->GetCurrentMatrix());
				NewtonCollisionSetMatrix (collision, &matrix[0][0]);
				NewtonCompoundCollisionAddSubCollision (compound, collision);
				NewtonDestroyCollision(collision);
			}
			NewtonCompoundCollisionEndAddRemove(compound);	
			NewtonBodySetCollision(chassis, compound);
			NewtonDestroyCollision(compound);
		#endif

		dCustomArticulatedTransformController::dSkeletonBone* const chassisBone = controller->AddRoot(chassis, dGetIdentityMatrix());

		// add engine
		dCustomArticulatedTransformController::dSkeletonBone* const engineBone = CreateEngineNode(controller, chassisBone);
		vehicleModel->m_engineJoint = (dCustomUniversal*) engineBone->FindJoint();
		vehicleModel->m_engineMotor = CreateEngineMotor(controller, vehicleModel->m_engineJoint);

		// set power parameter for a simple DC engine
		vehicleModel->m_maxTurmVelocity = 10.0f;
		vehicleModel->m_maxEngineSpeed = 30.0f;
		vehicleModel->m_engineMotor->SetTorque(2000.0f);
		vehicleModel->m_engineMotor->SetTorque1(2500.0f);

		// set the steering torque 
		vehicleModel->m_engineJoint->SetFriction(500.0f);

//		AddCraneBase (controller);

		MakeLeftTrack (controller);
		MakeRightTrack (controller);

//		MakeLeftThread(controller);
//		MakeRightThread(controller);
/*
		// disable self collision between all body parts
		controller->DisableAllSelfCollision();

		// wrap the skeleton in a newton skeleton for exact accuracy
		controller->DisableAllSelfCollision();
//		for (int i = 0; i < controller->GetBoneCount(); i ++) {
//			dCustomArticulatedTransformController::dSkeletonBone* const bone = controller->GetBone(i);
//			NewtonCollisionSetUserData (NewtonBodyGetCollision(bone->m_body), bone);
//		}
*/
		return controller;
	}
};


// we recommend using and input manage to control input for all games
class AriculatedJointInputManager: public dCustomInputManager
{
	public:
	AriculatedJointInputManager (DemoEntityManager* const scene)
		:dCustomInputManager(scene->GetNewton())
		,m_scene(scene)
		,m_cameraMode(true)
		,m_changeVehicle(true)
		,m_playersCount(0)
		,m_currentPlayer(0)
	{
		// plug a callback for 2d help display
		scene->Set2DDisplayRenderFunction (RenderPlayerHelp, NULL, this);
		scene->SetUpdateCameraFunction(UpdateCameraCallback, this);
	}

	void OnBeginUpdate (dFloat timestepInSecunds)
	{
		ArticulatedEntityModel::InputRecord inputs;

		ArticulatedEntityModel* const vehicleModel = (ArticulatedEntityModel*) m_player[m_currentPlayer % m_playersCount]->GetUserData();

		inputs.m_wristAxis0 = int(m_scene->GetKeyState('Y')) - int(m_scene->GetKeyState('U'));
		inputs.m_wristAxis1 = int(m_scene->GetKeyState('I')) - int(m_scene->GetKeyState('O'));
		inputs.m_turnValue = int (m_scene->GetKeyState ('R')) - int (m_scene->GetKeyState ('T'));
		inputs.m_tiltValue = int (m_scene->GetKeyState ('Z')) - int (m_scene->GetKeyState ('X'));
		inputs.m_liftValue = int (m_scene->GetKeyState ('Q')) - int (m_scene->GetKeyState ('E'));
		inputs.m_openValue = int (m_scene->GetKeyState ('F')) - int (m_scene->GetKeyState ('G'));
		inputs.m_steerValue = int (m_scene->GetKeyState ('D')) - int (m_scene->GetKeyState ('A'));
		inputs.m_throttleValue = int (m_scene->GetKeyState ('W')) - int (m_scene->GetKeyState ('S'));

		// check if we must activate the player
		if (m_scene->GetKeyState ('A') || 
			m_scene->GetKeyState ('D') ||
			m_scene->GetKeyState ('W') ||
			m_scene->GetKeyState ('S') ||

			m_scene->GetKeyState ('R') ||
			m_scene->GetKeyState ('T') ||
			m_scene->GetKeyState ('I') ||
			m_scene->GetKeyState ('O') ||
			m_scene->GetKeyState ('Y') ||
			m_scene->GetKeyState ('U') ||
			m_scene->GetKeyState ('F') ||
			m_scene->GetKeyState ('G') ||
			m_scene->GetKeyState ('Q') ||
			m_scene->GetKeyState ('E') ||
			m_scene->GetKeyState ('Z') ||
			m_scene->GetKeyState ('X')) 
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
		AriculatedJointInputManager* const me = (AriculatedJointInputManager*)context;
		me->UpdateCamera(timestep);
	}

	void UpdateCamera (dFloat timestepInSecunds)
	{
		DemoCamera* const camera = m_scene->GetCamera();
		ArticulatedEntityModel* const vehicleModel = (ArticulatedEntityModel*) m_player[m_currentPlayer % m_playersCount]->GetUserData();

		if (m_changeVehicle.UpdateTrigger(m_scene->GetKeyState ('P'))) {
			m_currentPlayer ++;
		}
		
		dMatrix camMatrix(camera->GetNextMatrix());
		dMatrix playerMatrix (vehicleModel->GetNextMatrix());

		dVector frontDir (camMatrix[0]);
		dVector camOrigin(0.0f); 
		m_cameraMode.UpdatePushButton(m_scene->GetKeyState('C'));
		if (m_cameraMode.GetPushButtonState()) {
			camOrigin = playerMatrix.TransformVector( dVector(0.0f, ARTICULATED_VEHICLE_CAMERA_HIGH_ABOVE_HEAD, 0.0f, 0.0f));
			camOrigin -= frontDir.Scale(ARTICULATED_VEHICLE_CAMERA_DISTANCE);
		} else {
			camMatrix = camMatrix * playerMatrix;
			camOrigin = playerMatrix.TransformVector(dVector(-0.8f, ARTICULATED_VEHICLE_CAMERA_EYEPOINT, 0.0f, 0.0f));
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

	void RenderPlayerHelp (DemoEntityManager* const scene) const
	{
		dVector color(1.0f, 1.0f, 0.0f, 0.0f);
		scene->Print (color, "Navigation Keys");
		scene->Print (color, "drive forward:      W");
		scene->Print (color, "drive backward:     S");
		scene->Print (color, "turn right:         D");
		scene->Print (color, "turn left:          A");
		scene->Print (color, "open palette:       F");
		scene->Print (color, "close palette       G");
		scene->Print (color, "lift palette:       E");
		scene->Print (color, "lower palette       Q");
		scene->Print (color, "tilt forward:       Z");
		scene->Print (color, "tilt backward:      X");
		scene->Print (color, "turn base left:     R");
		scene->Print (color, "turn base right:    T");
		scene->Print (color, "toggle camera mode: C");
		scene->Print (color, "switch vehicle:     P");
	}				

	static void RenderPlayerHelp (DemoEntityManager* const scene, void* const context)
	{
		AriculatedJointInputManager* const me = (AriculatedJointInputManager*) context;
		me->RenderPlayerHelp (scene);
	}


	DemoEntityManager* m_scene;
	dCustomArticulatedTransformController* m_player[2];
	DemoEntityManager::ButtonKey m_cameraMode;
	DemoEntityManager::ButtonKey m_changeVehicle;
	int m_playersCount;
	int m_currentPlayer;
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
				NewtonCollision* const shape = NewtonCreateBox(world, size.m_x, size.m_y, size.m_z, ARTICULATED_VEHICLE_DEFINITION::m_landPart, &offset[0][0]);
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

void ArticulatedJoints (DemoEntityManager* const scene)
{
	// load the sky box
	scene->CreateSkyBox();
	NewtonBody* const floor = CreateLevelMesh (scene, "flatPlane.ngd", true);
	//CreateHeightFieldTerrain (scene, 9, 8.0f, 1.5f, 0.2f, 200.0f, -50.0f);
	NewtonCollision* const floorCollision = NewtonBodyGetCollision(floor);
	NewtonCollisionSetUserID(floorCollision, ARTICULATED_VEHICLE_DEFINITION::m_terrain);

	// add an input Manage to manage the inputs and user interaction 
	AriculatedJointInputManager* const inputManager = new AriculatedJointInputManager (scene);

	//  create a skeletal transform controller for controlling rag doll
	ArticulatedVehicleManagerManager* const vehicleManager = new ArticulatedVehicleManagerManager (scene);

	NewtonWorld* const world = scene->GetNewton();
	dVector origin (FindFloor (world, dVector (-10.0f, 50.0f, 0.0f, 1.0f), 2.0f * 50.0f));

	dMatrix matrix (dGetIdentityMatrix());
	matrix.m_posit = FindFloor (world, origin, 100.0f);
	matrix.m_posit.m_y += 1.5f;

	ArticulatedEntityModel robotModel(scene, "robot.ngd");
	dCustomArticulatedTransformController* const robot = vehicleManager->CreateRobot (matrix, &robotModel, 0, NULL);
	inputManager->AddPlayer (robot);

	matrix.m_posit.m_z += 4.0f;
	// load a the mesh of the articulate vehicle
//	ArticulatedEntityModel forkliftModel(scene, "forklift.ngd");
//	dCustomArticulatedTransformController* const forklift = vehicleManager->CreateForklift(matrix, &forkliftModel, sizeof(forkliftDefinition) / sizeof (forkliftDefinition[0]), forkliftDefinition);
//	inputManager->AddPlayer(forklift);

	// add some object to play with
	DemoEntity entity (dGetIdentityMatrix(), NULL);
	entity.LoadNGD_mesh ("lumber.ngd", scene->GetNewton());
//	LoadLumberYardMesh (scene, entity, dVector(10.0f, 0.0f, 0.0f, 0.0f));
//	LoadLumberYardMesh (scene, entity, dVector(40.0f, 0.0f, 0.0f, 0.0f));
//	LoadLumberYardMesh (scene, entity, dVector(10.0f, 0.0f, 10.0f, 0.0f));
//	LoadLumberYardMesh (scene, entity, dVector(20.0f, 0.0f, 10.0f, 0.0f));
//	LoadLumberYardMesh (scene, entity, dVector(10.0f, 0.0f, 20.0f, 0.0f));
//	LoadLumberYardMesh (scene, entity, dVector(20.0f, 0.0f, 20.0f, 0.0f));

	origin.m_x -= 5.0f;
	origin.m_y += 5.0f;
	dQuaternion rot (dVector (0.0f, 1.0f, 0.0f, 0.0f), -30.0f * dDegreeToRad);  
	scene->SetCameraMatrix(rot, origin);
}



