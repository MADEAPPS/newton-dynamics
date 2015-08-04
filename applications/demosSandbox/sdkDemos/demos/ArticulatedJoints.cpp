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
#include "DemoMesh.h"
#include "DemoCamera.h"
#include "NewtonDemos.h"
#include "PhysicsUtils.h"
#include "TargaToOpenGl.h"
#include "DemoEntityManager.h"

#include "DebugDisplay.h"
#include "HeightFieldPrimitive.h"


#define ARTICULATED_VEHICLE_CAMERA_EYEPOINT			1.5f
#define ARTICULATED_VEHICLE_CAMERA_HIGH_ABOVE_HEAD	2.0f
//#define ARTICULATED_VEHICLE_CAMERA_DISTANCE			7.0f
#define ARTICULATED_VEHICLE_CAMERA_DISTANCE			10.0f

struct ARTICULATED_VEHICLE_DEFINITION
{
	enum
	{
		m_tireID = 1<<0,
		m_bodyPart = 1<<1,
		m_LinkPart = 1<<2,
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
	{"fr_tire",		"tireShape",			 50.0f, ARTICULATED_VEHICLE_DEFINITION::m_tireID, "frontTire"},
	{"fl_tire",		"tireShape",			 50.0f, ARTICULATED_VEHICLE_DEFINITION::m_tireID, "frontTire"},
	{"rr_tire",		"tireShape",			 50.0f, ARTICULATED_VEHICLE_DEFINITION::m_tireID, "rearTire"},
	{"rl_tire",		"tireShape",			 50.0f, ARTICULATED_VEHICLE_DEFINITION::m_tireID, "rearTire"},
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
	class EngineTire: public CustomHinge
	{
		public:
		EngineTire(const dMatrix& pinAndPivotFrame, NewtonBody* const child, NewtonBody* const parent)
			:CustomHinge(pinAndPivotFrame, child, parent)
			,m_velocity (5.0f) 
		{
		}

		void SubmitConstraintsFreeDof(dFloat timestep, const dMatrix& matrix0, const dMatrix& matrix1)
		{
			dFloat velocity = 0.1f * (m_velocity - GetJointOmega()) / timestep;
			NewtonUserJointAddAngularRow(m_joint, 0.0f, &matrix1.m_front[0]);
			NewtonUserJointSetRowAcceleration (m_joint, velocity);
		}

		dFloat m_velocity;
	};


	class InputRecord
	{
		public:
		InputRecord()
		{
			memset (this, 0, sizeof (InputRecord));
		}

		int m_steerValue;
		int m_throttleValue;
		int m_tiltValue;
		int m_liftValue;
		int m_openValue;
	};

	ArticulatedEntityModel (DemoEntityManager* const scene, const char* const name)
		:DemoEntity(dGetIdentityMatrix(), NULL)
		,m_rearTiresCount(0)
		,m_frontTiresCount(0)
		,m_angularActuatorsCount(0)
		,m_liftActuatorsCount(0)
		,m_paletteActuatorsCount(0)
	{
		// load the vehicle model
		LoadNGD_mesh (name, scene->GetNewton());
	}

	ArticulatedEntityModel (const ArticulatedEntityModel& copy)
		:DemoEntity(copy)
		,m_rearTiresCount(0)
		,m_frontTiresCount(0)
		,m_angularActuatorsCount(0)
		,m_liftActuatorsCount(0)
		,m_paletteActuatorsCount(0)
		,m_maxEngineSpeed(5.0f)
		,m_maxTurnSpeed(5.0f)
		,m_tiltAngle(0.0f)
		,m_liftPosit(0.0f)
		,m_openPosit(0.0f)
		,m_steeringTorque(0.0f)
		,m_engineLeftTire(NULL)
		,m_engineRightTire(NULL)
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

	void LinkFrontTire (NewtonBody* const chassis, NewtonBody* const tire)
	{
		dMatrix tireMatrix;
		dMatrix chassisMatrix;

		// calculate the tire location matrix
		NewtonBodyGetMatrix(tire, &tireMatrix[0][0]);
		NewtonBodyGetMatrix(chassis, &chassisMatrix[0][0]);

		chassisMatrix = dYawMatrix(90.0f * 3.141592f / 180.0f) * chassisMatrix;
		chassisMatrix.m_posit = tireMatrix.m_posit;

		m_frontTireJoints[m_frontTiresCount] = new CustomHinge (&chassisMatrix[0][0], tire, chassis);
		m_fronTires[m_frontTiresCount] = tire;
		m_frontTiresCount ++;
	}

	void LinkRearTire (NewtonBody* const chassis, NewtonBody* const tire)
	{
		dMatrix tireMatrix;
		dMatrix chassisMatrix;

		// calculate the tire location matrix
		NewtonBodyGetMatrix(tire, &tireMatrix[0][0]);
		NewtonBodyGetMatrix(chassis, &chassisMatrix[0][0]);

		chassisMatrix = dYawMatrix(90.0f * 3.141592f / 180.0f) * chassisMatrix;
		chassisMatrix.m_posit = tireMatrix.m_posit;

		dFloat angleLimit = 30.0f * 3.141592f / 180.0f;
		dFloat angularRate = 60.0f * 3.141592f / 180.0f;
		m_rearTireJoints[m_rearTiresCount] = new CustomUniversalActuator (&chassisMatrix[0][0], angularRate, -angleLimit, angleLimit, angularRate, -angleLimit, angleLimit, tire, chassis);
		m_rearTireJoints[m_rearTiresCount]->SetEnableFlag0 (false);
		m_rearTiresCount ++;
	}

	void LinkHingeActuator (NewtonBody* const parent, NewtonBody* const child)
	{
		dMatrix baseMatrix;
		NewtonBodyGetMatrix (child, &baseMatrix[0][0]);

		dFloat minAngleLimit = -20.0f * 3.141592f / 180.0f;
		dFloat maxAngleLimit =  20.0f * 3.141592f / 180.0f;
		dFloat angularRate = 10.0f * 3.141592f / 180.0f;
		m_angularActuator[m_angularActuatorsCount] = new CustomHingeActuator (&baseMatrix[0][0], angularRate, minAngleLimit, maxAngleLimit, child, parent);
		m_angularActuatorsCount ++;
	}

	void LinkLiftActuator (NewtonBody* const parent, NewtonBody* const child)
	{
		dMatrix baseMatrix;
		NewtonBodyGetMatrix (child, &baseMatrix[0][0]);

		dFloat minLimit = -0.25f;
		dFloat maxLimit = 1.5f;
		dFloat linearRate = 0.125f;
		m_liftJoints[m_liftActuatorsCount] = new CustomSliderActuator (&baseMatrix[0][0], linearRate, minLimit, maxLimit, child, parent);
		m_liftActuatorsCount ++;
	}

	void LinkPaletteActuator (NewtonBody* const parent, NewtonBody* const child)
	{
		dMatrix baseMatrix;
		NewtonBodyGetMatrix (child, &baseMatrix[0][0]);

		dFloat minLimit = -0.25f;
		dFloat maxLimit = 0.2f;
		dFloat linearRate = 0.25f;
		m_paletteJoints[m_paletteActuatorsCount] = new CustomSliderActuator (&baseMatrix[0][0], linearRate, minLimit, maxLimit, child, parent);
		m_paletteActuatorsCount ++;
	}

	int m_rearTiresCount;
	int m_frontTiresCount;
	int m_angularActuatorsCount;
	int m_liftActuatorsCount;
	int m_paletteActuatorsCount;
	dFloat m_maxEngineSpeed;
	dFloat m_maxTurnSpeed;
	dFloat m_tiltAngle;
	dFloat m_liftPosit;
	dFloat m_openPosit;
	dFloat m_steeringTorque;
		
	NewtonBody* m_fronTires[4];
	CustomHinge* m_frontTireJoints[4];
	CustomSliderActuator* m_liftJoints[4];
	CustomSliderActuator* m_paletteJoints[4];
	CustomUniversalActuator* m_rearTireJoints[4];
	CustomHingeActuator* m_angularActuator[4];
	EngineTire* m_engineLeftTire;
	EngineTire* m_engineRightTire;

	InputRecord m_inputs;
};

class ArticulatedVehicleManagerManager: public CustomArticulaledTransformManager
{
	public:

	ArticulatedVehicleManagerManager (DemoEntityManager* const scene)
		:CustomArticulaledTransformManager (scene->GetNewton(), true)
	{
		// create a material for early collision culling
		int material = NewtonMaterialGetDefaultGroupID (scene->GetNewton());
		NewtonMaterialSetCollisionCallback (scene->GetNewton(), material, material, this, OnBoneAABBOverlap, OnContactsProcess);
	}

	virtual void OnPreUpdate (CustomArticulatedTransformController* const controller, dFloat timestep, int threadIndex) const
	{
		ArticulatedEntityModel* const vehicleModel = (ArticulatedEntityModel*)controller->GetUserData();

		// apply engine torque
		if (vehicleModel->m_engineLeftTire && vehicleModel->m_engineRightTire) {

			dFloat speed = 0.0f; 
			if (vehicleModel->m_inputs.m_throttleValue > 0) {
				speed = vehicleModel->m_maxEngineSpeed;
			} else if (vehicleModel->m_inputs.m_throttleValue < 0) {
				speed = -vehicleModel->m_maxEngineSpeed;
			}
			vehicleModel->m_engineLeftTire->m_velocity = speed; 
			vehicleModel->m_engineRightTire->m_velocity = speed; 

			if (vehicleModel->m_inputs.m_steerValue > 0) {
				vehicleModel->m_engineLeftTire->m_velocity += vehicleModel->m_maxTurnSpeed;
				vehicleModel->m_engineRightTire->m_velocity -= vehicleModel->m_maxTurnSpeed;
			}
			else if (vehicleModel->m_inputs.m_steerValue < 0) {
				vehicleModel->m_engineLeftTire->m_velocity -= vehicleModel->m_maxTurnSpeed;
				vehicleModel->m_engineRightTire->m_velocity += vehicleModel->m_maxTurnSpeed;
			}
		}

		// update steering wheels
		if (vehicleModel->m_rearTiresCount) {
/*
			dFloat steeringAngle = vehicleModel->m_rearTireJoints[0]->GetActuatorAngle1();
			if (vehicleModel->m_inputs.m_steerValue > 0) {
				steeringAngle = vehicleModel->m_rearTireJoints[0]->GetMinAngularLimit0(); 
			} else if (vehicleModel->m_inputs.m_steerValue < 0) {
				steeringAngle = vehicleModel->m_rearTireJoints[0]->GetMaxAngularLimit0(); 
			}
			for (int i = 0; i < vehicleModel->m_rearTiresCount; i ++) {
				vehicleModel->m_rearTireJoints[i]->SetTargetAngle1(steeringAngle);
			}
*/			
		}

		// set the tilt angle
		if (vehicleModel->m_angularActuatorsCount) {
			dFloat tiltAngle = vehicleModel->m_tiltAngle;
			if (vehicleModel->m_inputs.m_tiltValue > 0) {
				tiltAngle = vehicleModel->m_angularActuator[0]->GetMinAngularLimit();
				vehicleModel->m_tiltAngle = vehicleModel->m_angularActuator[0]->GetActuatorAngle();
			} else if (vehicleModel->m_inputs.m_tiltValue < 0) {
				tiltAngle = vehicleModel->m_angularActuator[0]->GetMaxAngularLimit();
				vehicleModel->m_tiltAngle = vehicleModel->m_angularActuator[0]->GetActuatorAngle();
			}

			for (int i = 0; i < vehicleModel->m_angularActuatorsCount; i ++) {
				vehicleModel->m_angularActuator[i]->SetTargetAngle (tiltAngle);
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
	}

	static int OnBoneAABBOverlap (const NewtonMaterial* const material, const NewtonBody* const body0, const NewtonBody* const body1, int threadIndex)
	{
		NewtonCollision* const collision0 = NewtonBodyGetCollision(body0);
		NewtonCollision* const collision1 = NewtonBodyGetCollision(body1);
		CustomArticulatedTransformController::dSkeletonBone* const bone0 = (CustomArticulatedTransformController::dSkeletonBone*)NewtonCollisionGetUserData (collision0);
		CustomArticulatedTransformController::dSkeletonBone* const bone1 = (CustomArticulatedTransformController::dSkeletonBone*)NewtonCollisionGetUserData (collision1);
		if (bone0 && bone1 && (bone0->m_myController == bone1->m_myController)) {
//			dAssert (!bone0->m_myController->SelfCollisionTest (bone0, bone1));
//			return bone0->m_myController->SelfCollisionTest (bone0, bone1) ? 1 : 0;
		}
		return 1;
	}

	
	static void OnContactsProcess (const NewtonJoint* const contactJoint, dFloat timestep, int threadIndex)
	{
		NewtonBody* const body0 = NewtonJointGetBody0(contactJoint);
		NewtonBody* const body1 = NewtonJointGetBody1(contactJoint);

		NewtonCollision* const collision0 = NewtonBodyGetCollision(body0);
		NewtonCollision* const collision1 = NewtonBodyGetCollision(body1);

		int id0 = NewtonCollisionGetUserID (collision0);
		int id1 = NewtonCollisionGetUserID (collision1);

		NewtonBody* const tireBody = (id0 & ARTICULATED_VEHICLE_DEFINITION::m_tireID) ? body0 : ((id1 & ARTICULATED_VEHICLE_DEFINITION::m_tireID) ? body1 : NULL);
		NewtonBody* const linkBody = (id0 & ARTICULATED_VEHICLE_DEFINITION::m_LinkPart) ? body0 : ((id1 & ARTICULATED_VEHICLE_DEFINITION::m_LinkPart) ? body1 : NULL);

		if (linkBody && !tireBody) {
			// find the root body from the articulated structure 
			NewtonCollision* const linkCollsion = NewtonBodyGetCollision(linkBody);
			const CustomArticulatedTransformController::dSkeletonBone* const rootbone = (CustomArticulatedTransformController::dSkeletonBone*)NewtonCollisionGetUserData(linkCollsion);
			CustomArticulatedTransformController* const controller = rootbone->m_myController;
			NewtonBody* const chassiBody = controller->GetBoneBody(rootbone);

			// Get the root and tire matrices
			dMatrix tireMatrix;
			dMatrix chassisMatrix;
			NewtonBodyGetMatrix(linkBody, &tireMatrix[0][0]);
			NewtonBodyGetMatrix(chassiBody, &chassisMatrix[0][0]);

			dVector upDir(chassisMatrix.RotateVector(dVector(0.0f, 1.0f, 0.0f, 0.0f)));
			dVector tireAxis(tireMatrix.RotateVector(dVector(1.0f, 0.0f, 0.0f, 0.0f)));

			dVector contactDirection(upDir * tireAxis);
			for (void* contact = NewtonContactJointGetFirstContact(contactJoint); contact; contact = NewtonContactJointGetNextContact(contactJoint, contact)) {
				NewtonMaterial* const material = NewtonContactGetMaterial(contact);
				NewtonMaterialContactRotateTangentDirections(material, &contactDirection[0]);
				NewtonMaterialSetContactFrictionCoef(material, 5.0f, 5.0f, 0);
				NewtonMaterialSetContactFrictionCoef(material, 1.0f, 1.0f, 1);
			}

		} else if (tireBody) {

			// find the root body from the articulated structure 
			NewtonCollision* const tireCollsion = NewtonBodyGetCollision(tireBody);
			const CustomArticulatedTransformController::dSkeletonBone* const bone = (CustomArticulatedTransformController::dSkeletonBone*)NewtonCollisionGetUserData (tireCollsion);

			CustomArticulatedTransformController* const controller = bone->m_myController;
			const CustomArticulatedTransformController::dSkeletonBone* const rootbone = controller->GetParent(bone);
			NewtonBody* const chassiBody = controller->GetBoneBody(rootbone);

			// Get the root and tire matrices
			dMatrix tireMatrix;
			dMatrix chassisMatrix;
			NewtonBodyGetMatrix(tireBody, &tireMatrix[0][0]);
			NewtonBodyGetMatrix(chassiBody, &chassisMatrix[0][0]);

			dVector upDir (chassisMatrix.RotateVector(dVector (0.0f, 1.0f, 0.0f, 0.0f)));
			dVector tireAxis (tireMatrix.RotateVector(dVector (1.0f, 0.0f, 0.0f, 0.0f)));

			dVector contactDirection (upDir * tireAxis);
			for (void* contact = NewtonContactJointGetFirstContact (contactJoint); contact; contact = NewtonContactJointGetNextContact (contactJoint, contact)) {
				NewtonMaterial* const material = NewtonContactGetMaterial (contact);
				NewtonMaterialContactRotateTangentDirections (material, &contactDirection[0]);
				NewtonMaterialSetContactFrictionCoef (material, 1.0f, 1.0f, 0);
				NewtonMaterialSetContactFrictionCoef (material, 1.0f, 1.0f, 1);
			}
		}
	}
	

	virtual void OnUpdateTransform (const CustomArticulatedTransformController::dSkeletonBone* const bone, const dMatrix& localMatrix) const
	{
		DemoEntity* const ent = (DemoEntity*) NewtonBodyGetUserData(bone->m_body);
		DemoEntityManager* const scene = (DemoEntityManager*) NewtonWorldGetUserData(NewtonBodyGetWorld(bone->m_body));

		dQuaternion rot (localMatrix);
		ent->SetMatrix (*scene, rot, localMatrix.m_posit);
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
		dAssert (mesh->m_vertexCount && (mesh->m_vertexCount < int (sizeof (points)/ sizeof (points[0]))));

		// go over the vertex array and find and collect all vertices's weighted by this bone.
		dFloat* const array = mesh->m_vertex;
		for (int i = 0; i < mesh->m_vertexCount; i ++) {
			points[i].m_x = array[i * 3 + 0];
			points[i].m_y = array[i * 3 + 1];
			points[i].m_z = array[i * 3 + 2];
		}
		bodyPart->GetMeshMatrix().TransformTriplex(&points[0].m_x, sizeof (dVector), &points[0].m_x, sizeof (dVector), mesh->m_vertexCount) ;

		return NewtonCreateConvexHull (GetWorld(), mesh->m_vertexCount, &points[0].m_x, sizeof (dVector), 1.0e-3f, 0, NULL);
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
			shape = MakeConvexHullAggregate(bodyPart);
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

		//NewtonBodySetMaterialGroupID (body, m_material);
		NewtonCollisionSetUserID(collision, definition.m_bodyPartID);

		// set the bod part force and torque call back to the gravity force, skip the transform callback
		NewtonBodySetForceAndTorqueCallback (body, PhysicsApplyGravityForce);
		return body;
	}

	void ConnectBodyPart (ArticulatedEntityModel* const vehicleModel, NewtonBody* const parent, NewtonBody* const child, const dString& jointArticulation)
	{
		if (jointArticulation == "") {
			// this is the root body do nothing

		} else if (jointArticulation == "frontTire") {
			vehicleModel->LinkFrontTire (parent, child);
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


	CustomArticulatedTransformController* CreateForklift (const dMatrix& location, const DemoEntity* const model, int bodyPartsCount, ARTICULATED_VEHICLE_DEFINITION* const definition)
	{
		NewtonWorld* const world = GetWorld(); 
		DemoEntityManager* const scene = (DemoEntityManager*) NewtonWorldGetUserData(world);

		// make a clone of the mesh 
		ArticulatedEntityModel* const vehicleModel = (ArticulatedEntityModel*) model->CreateClone();
		scene->Append(vehicleModel);

		// plane the model at its location
		vehicleModel->ResetMatrix (*scene, location);

		CustomArticulatedTransformController* const controller = CreateTransformController (vehicleModel);

		DemoEntity* const rootEntity = (DemoEntity*) vehicleModel->Find (definition[0].m_boneName);
		NewtonBody* const rootBody = CreateBodyPart (rootEntity, definition[0]);

		// move the center of mass a lithe to the back, and lower
		dVector com;
		NewtonBodyGetCentreOfMass(rootBody, &com[0]);
		//com.m_x -= 0.25f;
		com.m_y -= 0.25f;
		NewtonBodySetCentreOfMass(rootBody, &com[0]);

		// for debugging joints
		//NewtonBodySetMassMatrix(rootBody, 0,0,0,0);

		// add the root bone to the articulation manager
		CustomArticulatedTransformController::dSkeletonBone* const bone = controller->AddBone (rootBody, dGetIdentityMatrix());
		// save the bone as the shape use data for self collision test
		NewtonCollisionSetUserData (NewtonBodyGetCollision(rootBody), bone);

		// walk down the model hierarchy an add all the components 
		int stackIndex = 0;
		DemoEntity* childEntities[32];
		CustomArticulatedTransformController::dSkeletonBone* parentBones[32];
		for (DemoEntity* child = rootEntity->GetChild(); child; child = child->GetSibling()) {
			parentBones[stackIndex] = bone;
			childEntities[stackIndex] = child;
			stackIndex ++;
		}

		while (stackIndex) {
			stackIndex --;
			DemoEntity* const entity = childEntities[stackIndex];
			CustomArticulatedTransformController::dSkeletonBone* parentBone = parentBones[stackIndex];

			const char* const name = entity->GetName().GetStr();
			for (int i = 0; i < bodyPartsCount; i ++) {
				if (!strcmp (definition[i].m_boneName, name)) {
					NewtonBody* const bone = CreateBodyPart (entity, definition[i]);

					// connect this body part to its parent with a vehicle joint
					ConnectBodyPart (vehicleModel, parentBone->m_body, bone, definition[i].m_articulationName);

					dMatrix bindMatrix (entity->GetParent()->CalculateGlobalMatrix ((DemoEntity*)NewtonBodyGetUserData (parentBone->m_body)).Inverse());
					parentBone = controller->AddBone (bone, bindMatrix, parentBone);

					// save the controller as the collision user data, for collision culling
					NewtonCollisionSetUserData (NewtonBodyGetCollision(bone), parentBone);
					break;
				}
			}

			for (DemoEntity* child = entity->GetChild(); child; child = child->GetSibling()) {
				parentBones[stackIndex] = parentBone;
				childEntities[stackIndex] = child;
				stackIndex ++;
			}
		}

		// calculate the engine parameters
//		if (vehicleModel->m_frontTiresCount) {
//			CalculateEngine (vehicleModel, rootBody, vehicleModel->m_fronTires[0]);
//		}

		// disable self collision between all body parts
		controller->DisableAllSelfCollision();

		// wrap the skeleton in a newton skeleton for exact accuracy
		controller->MakeNewtonSkeleton();

		return controller;
	}

	NewtonCollision* MakeRobotTireShape(DemoEntity* const bodyPart) const
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

		dMatrix align (dRollMatrix(90.0f * 3.141592f / 180.0f));
		return NewtonCreateChamferCylinder(GetWorld(), radius, width, 0, &align[0][0]);
	}


	CustomArticulatedTransformController::dSkeletonBone* MakeTireBody(const char* const entName, const char* const tireName, CustomArticulatedTransformController* const controller, CustomArticulatedTransformController::dSkeletonBone* const parentBone)
	{
		ARTICULATED_VEHICLE_DEFINITION definition;
		strcpy(definition.m_boneName, entName);
		strcpy(definition.m_shapeTypeName, "tireShape");
		strcpy(definition.m_articulationName, tireName);
		definition.m_mass = 30.0f;
		definition.m_bodyPartID = ARTICULATED_VEHICLE_DEFINITION::m_tireID;

		NewtonBody* const parentBody = parentBone->m_body;
		DemoEntity* const parentModel = (DemoEntity*)NewtonBodyGetUserData(parentBody);
		DemoEntity* const tireModel = parentModel->Find(entName);

		NewtonCollision* const tireCollision = MakeRobotTireShape(tireModel);

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
		NewtonBodySetMassProperties(tireBody, definition.m_mass, collision);

		// save the user data with the bone body (usually the visual geometry)
		NewtonBodySetUserData(tireBody, tireModel);

		//NewtonBodySetMaterialGroupID (body, m_material);
		NewtonCollisionSetUserID(collision, definition.m_bodyPartID);

		// set the bod part force and torque call back to the gravity force, skip the transform callback
		NewtonBodySetForceAndTorqueCallback(tireBody, PhysicsApplyGravityForce);

		CustomArticulatedTransformController::dSkeletonBone* const bone = controller->AddBone(tireBody, dGetIdentityMatrix(), parentBone);
		NewtonCollisionSetUserData(NewtonBodyGetCollision(tireBody), bone);

		return bone;
	}

	CustomArticulatedTransformController::dSkeletonBone* MakeTire(const char* const entName, const char* const tireName, CustomArticulatedTransformController* const controller, CustomArticulatedTransformController::dSkeletonBone* const parentBone)
	{
		CustomArticulatedTransformController::dSkeletonBone* const bone = MakeTireBody (entName, tireName, controller, parentBone);

		// connect the tire tp the body with a hinge
		dMatrix matrix;
		NewtonBodyGetMatrix (bone->m_body, &matrix[0][0]);
		dMatrix hingeFrame (dRollMatrix(90.0f * 3.141592f / 180.0f) * matrix);
		new CustomHinge (hingeFrame, bone->m_body, parentBone->m_body);

		return bone;
	}

	CustomArticulatedTransformController::dSkeletonBone* MakeEngineTire(const char* const entName, const char* const tireName, CustomArticulatedTransformController* const controller, CustomArticulatedTransformController::dSkeletonBone* const parentBone)
	{
		CustomArticulatedTransformController::dSkeletonBone* const bone = MakeTireBody(entName, tireName, controller, parentBone);

		// connect the tire tp the body with a hinge
		dMatrix matrix;
		NewtonBodyGetMatrix(bone->m_body, &matrix[0][0]);
		dMatrix hingeFrame(dRollMatrix(90.0f * 3.141592f / 180.0f) * matrix);
		new ArticulatedEntityModel::EngineTire(hingeFrame, bone->m_body, parentBone->m_body);

		return bone;
	}

	void LinkTires(CustomArticulatedTransformController::dSkeletonBone* const master, CustomArticulatedTransformController::dSkeletonBone* const slave, CustomArticulatedTransformController::dSkeletonBone* const root)
	{
		NewtonCollisionInfoRecord masterTire;
		NewtonCollisionInfoRecord slaveTire;

		NewtonCollisionGetInfo(NewtonBodyGetCollision(master->m_body), &masterTire);
		NewtonCollisionGetInfo(NewtonBodyGetCollision(slave->m_body), &slaveTire);

		dAssert(masterTire.m_collisionType == SERIALIZE_ID_CHAMFERCYLINDER);
		dAssert(slaveTire.m_collisionType == SERIALIZE_ID_CHAMFERCYLINDER);

		dFloat masterRadio = masterTire.m_chamferCylinder.m_height * 0.5f + masterTire.m_chamferCylinder.m_radio;
		dFloat slaveRadio = slaveTire.m_chamferCylinder.m_height * 0.5f + slaveTire.m_chamferCylinder.m_radio;

		dMatrix pinMatrix;
		NewtonBodyGetMatrix(root->m_body, &pinMatrix[0][0]);
		new CustomGear(masterRadio / slaveRadio, pinMatrix[2], pinMatrix[2].Scale(-1.0f), slave->m_body, master->m_body);
	}
	

	void MakeSuspension(CustomArticulatedTransformController* const controller, const char* const entName, CustomArticulatedTransformController::dSkeletonBone* const masterTire)
	{
		CustomArticulatedTransformController::dSkeletonBone* const chassisBone = controller->GetBone(0);

		ARTICULATED_VEHICLE_DEFINITION definition;
		strcpy(definition.m_boneName, entName);
		strcpy(definition.m_shapeTypeName, "convexHull");
		strcpy(definition.m_articulationName, "suspension");
		definition.m_mass = 50.0f;
		definition.m_bodyPartID = ARTICULATED_VEHICLE_DEFINITION::m_bodyPart;
	
		NewtonBody* const parentBody = chassisBone->m_body;
		DemoEntity* const parentModel = (DemoEntity*)NewtonBodyGetUserData(parentBody);
		DemoEntity* const partModel = parentModel->Find(entName);

		NewtonBody* const suspestionBody = CreateBodyPart(partModel, definition);

		CustomArticulatedTransformController::dSkeletonBone* const suspBone = controller->AddBone(suspestionBody, dGetIdentityMatrix(), chassisBone);
		NewtonCollisionSetUserData(NewtonBodyGetCollision(suspestionBody), suspBone);

		dMatrix matrix;
		NewtonBodyGetMatrix(suspestionBody, &matrix[0][0]);
		dMatrix hingeFrame(dYawMatrix(90.0f * 3.141592f / 180.0f) * matrix);
		CustomHinge* const hinge = new CustomHinge(hingeFrame, suspestionBody, parentBody);
		hinge->EnableLimits(true);
		hinge->SetLimits(-45.0f * 3.1416f / 180.0f, 45.0f * 3.1416f / 180.0f);

		for (DemoEntity* child = partModel->GetChild(); child; child = child->GetSibling()) {
			CustomArticulatedTransformController::dSkeletonBone* const childBone = MakeTire (child->GetName().GetStr(), "tire", controller, suspBone);
			 //MakeTire (child->GetName().GetStr(), "tire", controller, suspBone);
			LinkTires (masterTire, childBone, chassisBone);
		}
	}

	void MakeLeftTrack(CustomArticulatedTransformController* const controller)
	{
		CustomArticulatedTransformController::dSkeletonBone* const chassisBone = controller->GetBone(0);
		
		CustomArticulatedTransformController::dSkeletonBone* const leftTire_0 = MakeEngineTire ("leftTire_0", "tractionLeftTire", controller, chassisBone);
		CustomArticulatedTransformController::dSkeletonBone* const leftTire_7 = MakeTire ("leftTire_7", "tire", controller, chassisBone);
		LinkTires (leftTire_0, leftTire_7, chassisBone);

		MakeTire ("leftTireSuport_0", "suportTire", controller, chassisBone);
		MakeTire ("leftTireSuport_1", "suportTire", controller, chassisBone);

		MakeSuspension (controller, "leftSuspension_0", leftTire_0);
		MakeSuspension (controller, "leftSuspension_1", leftTire_0);
		MakeSuspension (controller, "leftSuspension_2", leftTire_0);

		ArticulatedEntityModel* const model = (ArticulatedEntityModel*) NewtonBodyGetUserData (chassisBone->m_body);
		model->m_fronTires[model->m_frontTiresCount] = leftTire_0->m_body;
		model->m_frontTiresCount += 1;

		ArticulatedEntityModel::EngineTire* const engineJoint = (ArticulatedEntityModel::EngineTire*) NewtonJointGetUserData(NewtonBodyGetFirstJoint(leftTire_0->m_body));
		dAssert (dString (engineJoint->GetTypeName()) == dString ("CustomHinge"));
		model->m_engineLeftTire = engineJoint;
	}

	void MakeRightTrack(CustomArticulatedTransformController* const controller)
	{
		CustomArticulatedTransformController::dSkeletonBone* const chassisBone = controller->GetBone(0);

		CustomArticulatedTransformController::dSkeletonBone* const rightTire_0 = MakeEngineTire("rightTire_0", "tractionrightTire", controller, chassisBone);
		CustomArticulatedTransformController::dSkeletonBone* const rightTire_7 = MakeTire("rightTire_7", "tire", controller, chassisBone);
		LinkTires (rightTire_0, rightTire_7, chassisBone);

		MakeTire("rightTireSuport_0", "suportTire", controller, chassisBone);
		MakeTire("rightTireSuport_1", "suportTire", controller, chassisBone);

		MakeSuspension(controller, "rightSuspension_0", rightTire_0);
		MakeSuspension(controller, "rightSuspension_1", rightTire_0);
		MakeSuspension(controller, "rightSuspension_2", rightTire_0);

		ArticulatedEntityModel* const model = (ArticulatedEntityModel*)NewtonBodyGetUserData(chassisBone->m_body);
		model->m_fronTires[model->m_frontTiresCount] = rightTire_0->m_body;
		model->m_frontTiresCount += 1;

		ArticulatedEntityModel::EngineTire* const engineJoint = (ArticulatedEntityModel::EngineTire*)NewtonJointGetUserData(NewtonBodyGetFirstJoint(rightTire_0->m_body));
		dAssert(dString(engineJoint->GetTypeName()) == dString("CustomHinge"));
		model->m_engineRightTire = engineJoint;
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
			}
			else {
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


	class RealJoint: public CustomJoint
	{
		public:
		RealJoint(NewtonBody* const link, NewtonBody* const chassis)
			:CustomJoint(1, link, chassis)
		{
			dMatrix linkMatrix;
			dMatrix chassisMatrix;
			
			NewtonBodyGetMatrix (link, &linkMatrix[0][0]);
			NewtonBodyGetMatrix (link, &chassisMatrix[0][0]);

			dMatrix pinAndPivotFrame (dGrammSchmidt (chassisMatrix[2]));
			pinAndPivotFrame.m_posit = linkMatrix.m_posit;
			CalculateLocalMatrix (pinAndPivotFrame, m_localMatrix0, m_localMatrix1);
		}
			
		void SubmitConstraints(dFloat timestep, int threadIndex)
		{
			dMatrix matrix0;
			dMatrix matrix1;

			// calculate the position of the pivot point and the Jacobian direction vectors, in global space. 
			CalculateGlobalMatrix(matrix0, matrix1);

			// Restrict the movement on the pivot point along all three orthonormal directions
			NewtonUserJointAddLinearRow(m_joint, &matrix0.m_posit[0], &matrix1.m_posit[0], &matrix1.m_front[0]);
		}
	};

	void CalculaterUniformSpaceSamples(DemoEntity* const chassis, float offset, CustomArticulatedTransformController::dSkeletonBone* const rootNode)
	{
		dFloat linkLength = 0.32f;

		DemoEntity* const threadPath = chassis->Find("trackPath");
		dAssert(threadPath);
		DemoBezierCurve* const bezierPath = (DemoBezierCurve*) threadPath->GetMesh();

		dFloat length = bezierPath->m_curve.CalculateLength (0.01f);
		int linksCount = int(length / linkLength) + 1;
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
		dBigVector q(bezierPath->m_curve.CurvePoint(0.0f));
		dVector p0(dVector(q.m_x, q.m_y, q.m_z, q.m_w));
		for (int i = 1; i < samplingRate + 45; i++) {
			dFloat u = dFloat(i) / samplingRate;
			dBigVector q(bezierPath->m_curve.CurvePoint(dMod(u, 1.0f)));
			dVector p1(dVector(q.m_x, q.m_y, q.m_z, q.m_w));
			dVector err(p1 - p0);
			dFloat errMag = dSqrt(err % err);
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

		dMatrix aligmentMatrix (dRollMatrix(180.0f * 3.141592f / 180.0f));

		DemoEntity* const threadLink = chassis->Find("link");
		dMatrix shapeMatrix (threadPath->GetMeshMatrix() * threadPath->GetCurrentMatrix());

		DemoEntity* const threadLinkChild = chassis->Find("Object001");
		dMatrix shapeChildMatrix (threadLinkChild->GetCurrentMatrix());

		dFloat s = 0.0f;
		dMatrix matrix(dGetIdentityMatrix());
		dFloat u0 = CalculateKnotParam(steps, linksCount, s);
		dBigVector r(bezierPath->m_curve.CurvePoint(u0));
		dVector r0(dVector(r.m_x, r.m_y, r.m_z, 1.0f));

		NewtonWorld* const world = GetWorld();
		DemoEntityManager* const scene = (DemoEntityManager*)NewtonWorldGetUserData(world);
		NewtonCollision* const collision = MakeConvexHull(threadLink);
		NewtonCollisionSetUserID(collision, ARTICULATED_VEHICLE_DEFINITION::m_LinkPart);
		NewtonCollisionSetUserData (collision, rootNode);

		NewtonBody* linkArray[1024];

		int bodyCount = 0;
		linksCount += 1;

		void* const aggregate = NewtonCollisionAggregateCreate(world);
		NewtonCollisionAggregateSetSelfCollision (aggregate, 0);
		for (int i = 1; i < linksCount + 1; i++) {
			s += linkLength;
			dFloat u1 = CalculateKnotParam(steps, linksCount, dMod (s, length));
			dBigVector r(bezierPath->m_curve.CurvePoint(u1));
			dVector r1(dVector(r.m_x, r.m_y, r.m_z, 1.0f));
			dVector dir(r1 - r0);

			dir = dir.Scale(1.0f / dSqrt(dir % dir));
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

		dMatrix aligment (dYawMatrix(90.0f * 3.1416f / 180.0f));
		NewtonBody* link0 = linkArray[0];

		NewtonJoint* hingeArray[1024];
		for (int i = 1; i < bodyCount; i++) {
			NewtonBody* const link1 = linkArray[i];

			dMatrix matrix;
			NewtonBodyGetMatrix(link1, &matrix[0][0]);
			CustomHinge* const hinge = new CustomHinge (aligment * matrix, link1, link0);
			hinge->SetFriction(20.0f);
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
		CustomHinge* const hinge = new CustomHinge (matrix0, matrix1, linkArray[0], linkArray[bodyCount - 1]);
		hinge->SetFriction(20.0f);

		int reeltPoints = 8;
		for (int i = 0; i < reeltPoints; i ++) {
			NewtonBody* const linkBody = linkArray[i * bodyCount / reeltPoints];
			new RealJoint (linkBody, rootNode->m_body);
		}

		NewtonSkeletonContainer* const skeleton = NewtonSkeletonContainerCreate (world, link0, NULL);
		NewtonSkeletonContainerAttachJointArray (skeleton, bodyCount - 1, hingeArray);
		NewtonSkeletonContainerFinalize (skeleton);
	}


	void MakeLeftThread(CustomArticulatedTransformController* const controller)
	{
		CustomArticulatedTransformController::dSkeletonBone* const chassisBone = controller->GetBone(0);
		DemoEntity* const chassis = (DemoEntity*) NewtonBodyGetUserData (chassisBone->m_body);
		DemoEntity* const pivot = chassis->Find ("leftTire_0");
		CalculaterUniformSpaceSamples (chassis, pivot->GetCurrentMatrix().m_posit.m_z, chassisBone);
	}


	void MakeRightThread(CustomArticulatedTransformController* const controller)
	{
		CustomArticulatedTransformController::dSkeletonBone* const chassisBone = controller->GetBone(0);
		DemoEntity* const chassis = (DemoEntity*)NewtonBodyGetUserData(chassisBone->m_body);
		DemoEntity* const pivot = chassis->Find("rightTire_0");
		CalculaterUniformSpaceSamples(chassis, pivot->GetCurrentMatrix().m_posit.m_z, chassisBone);
	}


	CustomArticulatedTransformController* CreateRobot (const dMatrix& location, const DemoEntity* const model, int , ARTICULATED_VEHICLE_DEFINITION* const )
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

		CustomArticulatedTransformController* const controller = CreateTransformController(vehicleModel);


		ARTICULATED_VEHICLE_DEFINITION definition;
		strcpy (definition.m_boneName, "body");
		strcpy (definition.m_shapeTypeName, "convexHull");
		definition.m_mass = 800.0f;
		definition.m_bodyPartID = ARTICULATED_VEHICLE_DEFINITION::m_bodyPart;
		strcpy (definition.m_articulationName, "mainBody");
		NewtonBody* const chassis = CreateBodyPart (vehicleModel, definition);

		CustomArticulatedTransformController::dSkeletonBone* const bone = controller->AddBone(chassis, dGetIdentityMatrix());
		NewtonCollisionSetUserData(NewtonBodyGetCollision(chassis), bone);

		MakeLeftTrack (controller);
		MakeRightTrack (controller);
		MakeLeftThread (controller);
		MakeRightThread(controller);

		// disable self collision between all body parts
		controller->DisableAllSelfCollision();

		// wrap the skeleton in a newton skeleton for exact accuracy
		controller->MakeNewtonSkeleton();

		return controller;
	}
};


// we recommend using and input manage to control input for all games
class AriculatedJointInputManager: public CustomInputManager
{
	public:
	AriculatedJointInputManager (DemoEntityManager* const scene)
		:CustomInputManager(scene->GetNewton())
		,m_scene(scene)
		,m_player(NULL)
		,m_cameraMode(true)
	{
		// plug a callback for 2d help display
		scene->Set2DDisplayRenderFunction (RenderPlayerHelp, this);
	}

	void OnBeginUpdate (dFloat timestepInSecunds)
	{
		ArticulatedEntityModel::InputRecord inputs;

		NewtonDemos* const mainWindow = m_scene->GetRootWindow();
		ArticulatedEntityModel* const vehicleModel = (ArticulatedEntityModel*) m_player->GetUserData();
		
		inputs.m_tiltValue = int (mainWindow->GetKeyState ('Z')) - int (mainWindow->GetKeyState ('X'));
		inputs.m_liftValue = int (mainWindow->GetKeyState ('Q')) - int (mainWindow->GetKeyState ('E'));
		inputs.m_openValue = int (mainWindow->GetKeyState ('F')) - int (mainWindow->GetKeyState ('G'));
		inputs.m_steerValue = int (mainWindow->GetKeyState ('D')) - int (mainWindow->GetKeyState ('A'));
		inputs.m_throttleValue = int (mainWindow->GetKeyState ('W')) - int (mainWindow->GetKeyState ('S'));

		// check if we must activate the player
		if (mainWindow->GetKeyState ('A') || 
			mainWindow->GetKeyState ('D') ||
			mainWindow->GetKeyState ('W') ||
			mainWindow->GetKeyState ('S') ||
			mainWindow->GetKeyState ('F') ||
			mainWindow->GetKeyState ('G') ||
			mainWindow->GetKeyState ('Q') ||
			mainWindow->GetKeyState ('E') ||
			mainWindow->GetKeyState ('Z') ||
			mainWindow->GetKeyState ('X')) 
		{
			NewtonBody* const body = m_player->GetBoneBody(0);
			NewtonBodySetSleepState(body, false);
		}

//NewtonBody* const body = m_player->GetBoneBody(0);
//NewtonBodySetSleepState(body, false);

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

	void OnEndUpdate (dFloat timestepInSecunds)
	{
		DemoCamera* const camera = m_scene->GetCamera();
		ArticulatedEntityModel* const vehicleModel = (ArticulatedEntityModel*) m_player->GetUserData();
		
		dMatrix camMatrix(camera->GetNextMatrix());
		dMatrix playerMatrix (vehicleModel->GetNextMatrix());

		dVector frontDir (camMatrix[0]);

		dVector camOrigin; 
		m_cameraMode.UpdatePushButton(m_scene->GetRootWindow(), 'C');
		if (m_cameraMode.GetPushButtonState()) {
			camOrigin = playerMatrix.TransformVector( dVector(0.0f, ARTICULATED_VEHICLE_CAMERA_HIGH_ABOVE_HEAD, 0.0f, 0.0f));
			camOrigin -= frontDir.Scale(ARTICULATED_VEHICLE_CAMERA_DISTANCE);
		} else {
			camMatrix = camMatrix * playerMatrix;
			camOrigin = playerMatrix.TransformVector(dVector(-0.8f, ARTICULATED_VEHICLE_CAMERA_EYEPOINT, 0.0f, 0.0f));
		}

		camera->SetNextMatrix (*m_scene, camMatrix, camOrigin);
	}

	void AddPlayer(CustomArticulatedTransformController* const player)
	{
		m_player = player;
	}


	void RenderPlayerHelp (DemoEntityManager* const scene, int lineNumber) const
	{
		//		if (m_player->m_helpKey.GetPushButtonState()) {
		dVector color(1.0f, 1.0f, 0.0f, 0.0f);
		lineNumber = scene->Print (color, 10, lineNumber + 20, "Navigation Keys");
		lineNumber = scene->Print (color, 10, lineNumber + 20, "drive forward:           W");
		lineNumber = scene->Print (color, 10, lineNumber + 20, "drive backward:          S");
		lineNumber = scene->Print (color, 10, lineNumber + 20, "turn right:              D");
		lineNumber = scene->Print (color, 10, lineNumber + 20, "turn left:               A");
		lineNumber = scene->Print (color, 10, lineNumber + 20, "open palette:            F");
		lineNumber = scene->Print (color, 10, lineNumber + 20, "close palette            G");
		lineNumber = scene->Print (color, 10, lineNumber + 20, "lift palette:            E");
		lineNumber = scene->Print (color, 10, lineNumber + 20, "lower palette            Q");
		lineNumber = scene->Print (color, 10, lineNumber + 20, "tilt forward:            Z");
		lineNumber = scene->Print (color, 10, lineNumber + 20, "tilt backward:           X");
		lineNumber = scene->Print (color, 10, lineNumber + 20, "toggle camera mode:      C");
	}


	static void RenderPlayerHelp (DemoEntityManager* const scene, void* const context, int lineNumber)
	{
		AriculatedJointInputManager* const me = (AriculatedJointInputManager*) context;
		me->RenderPlayerHelp (scene, lineNumber);
	}


	DemoEntityManager* m_scene;
	CustomArticulatedTransformController* m_player;
	DemoEntityManager::ButtonKey m_cameraMode;
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
				//NewtonCollision* const shape = NewtonCreateBox(world, size.m_x, size.m_y, size.m_z, 0, NULL);
				NewtonCollision* const shape = NewtonCreateBox(world, size.m_x, size.m_y, size.m_z, 0, &offset[0][0]);
				node = filter.Insert (shape, mesh);
			}

			// create a body and add to the world
			NewtonCollision* const shape = node->GetInfo();
			dMatrix matrix (child->GetMeshMatrix() * child->CalculateGlobalMatrix());
			matrix.m_posit += location;
			float mass = density * NewtonConvexCollisionCalculateVolume(shape);
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
	CreateLevelMesh (scene, "flatPlane.ngd", true);
	//CreateHeightFieldTerrain (scene, 9, 8.0f, 1.5f, 0.2f, 200.0f, -50.0f);

	// add an input Manage to manage the inputs and user interaction 
	AriculatedJointInputManager* const inputManager = new AriculatedJointInputManager (scene);

	//  create a skeletal transform controller for controlling rag doll
	ArticulatedVehicleManagerManager* const vehicleManager = new ArticulatedVehicleManagerManager (scene);

	NewtonWorld* const world = scene->GetNewton();
	dVector origin (FindFloor (world, dVector (-10.0f, 50.0f, 0.0f, 1.0f), 2.0f * 50.0f));

	dMatrix matrix (dGetIdentityMatrix());
	matrix.m_posit = FindFloor (world, origin, 100.0f);
	matrix.m_posit.m_y += 0.75f;

	// load a the mesh of the articulate vehicle
//	ArticulatedEntityModel forkliftModel(scene, "forklift.ngd");
//	CustomArticulatedTransformController* const forklift = vehicleManager->CreateForklift (matrix, &forkliftModel, sizeof(forkliftDefinition) / sizeof (forkliftDefinition[0]), forkliftDefinition);
//	inputManager->AddPlayer (forklift);
	
	matrix.m_posit.m_z += 4.0f;
	ArticulatedEntityModel robotModel(scene, "robot.ngd");
	CustomArticulatedTransformController* const robot = vehicleManager->CreateRobot (matrix, &robotModel, 0, NULL);
	inputManager->AddPlayer (robot);

	// add some object to play with
	DemoEntity entity (dGetIdentityMatrix(), NULL);
	entity.LoadNGD_mesh ("lumber.ngd", scene->GetNewton());
	LoadLumberYardMesh (scene, entity, dVector(10.0f, 0.0f, 0.0f, 0.0f));
	
	origin.m_x -= 5.0f;
	origin.m_y += 5.0f;
	dQuaternion rot (dVector (0.0f, 1.0f, 0.0f, 0.0f), -30.0f * 3.141592f / 180.0f);  
	scene->SetCameraMatrix(rot, origin);

}



