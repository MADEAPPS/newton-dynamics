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
#define ARTICULATED_VEHICLE_CAMERA_DISTANCE			7.0f

struct ARTICULATED_VEHICLE_DEFINITION
{
	enum
	{
		m_tireID = 1<<0,
		m_bodyPart = 2<<0,
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
		,m_maxEngineTorque(0.0f)
		,m_omegaResistance(0.0f)
		,m_tiltAngle(0.0f)
		,m_liftPosit(0.0f)
		,m_openPosit(0.0f)
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
	dFloat m_maxEngineTorque;
	dFloat m_omegaResistance;
	dFloat m_tiltAngle;
	dFloat m_liftPosit;
	dFloat m_openPosit;
		
	NewtonBody* m_fronTires[4];
	CustomHinge* m_frontTireJoints[4];
	CustomSliderActuator* m_liftJoints[4];
	CustomSliderActuator* m_paletteJoints[4];
	CustomUniversalActuator* m_rearTireJoints[4];
	CustomHingeActuator* m_angularActuator[4];

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
		if (vehicleModel->m_frontTiresCount) {
			dFloat brakeTorque = 0.0f;
			dFloat engineTorque = 0.0f;
			if (vehicleModel->m_inputs.m_throttleValue > 0) {
				engineTorque = -vehicleModel->m_maxEngineTorque; 
			} else if (vehicleModel->m_inputs.m_throttleValue < 0) {
				engineTorque = vehicleModel->m_maxEngineTorque; 
			} else {
				brakeTorque = 1000.0f;
			}
			for (int i = 0; i < vehicleModel->m_frontTiresCount; i ++) {
				vehicleModel->m_frontTireJoints[i]->SetFriction(brakeTorque);
			}

			dMatrix matrix;
			NewtonBody* const rootBody = controller->GetBoneBody(0);
			NewtonBodyGetMatrix(rootBody, &matrix[0][0]);
			
			dVector tirePing (matrix.RotateVector(dVector (0.0, 0.0, 1.0, 0.0)));
			if (engineTorque != 0.0f) {
				dVector torque (tirePing.Scale(engineTorque));
				for (int i = 0; i < vehicleModel->m_frontTiresCount; i ++) {
					NewtonBodyAddTorque (vehicleModel->m_fronTires[i], &torque[0]);
				}
			}

			for (int i = 0; i < vehicleModel->m_frontTiresCount; i ++) {
				dVector omega;
				NewtonBodyGetOmega(vehicleModel->m_fronTires[i], &omega[0]);
				dFloat omegaMag = omega % tirePing;
				dFloat sign = (omegaMag >= 0.0f) ? 1.0 : -1.0f;
				omega -= tirePing.Scale(sign * omegaMag * omegaMag * vehicleModel->m_omegaResistance);
				NewtonBodySetOmega(vehicleModel->m_fronTires[i], &omega[0]);
			}
		}

		// update steering wheels
		if (vehicleModel->m_rearTiresCount) {
			dFloat steeringAngle = vehicleModel->m_rearTireJoints[0]->GetActuatorAngle1();
			if (vehicleModel->m_inputs.m_steerValue > 0) {
				steeringAngle = vehicleModel->m_rearTireJoints[0]->GetMinAngularLimit0(); 
			} else if (vehicleModel->m_inputs.m_steerValue < 0) {
				steeringAngle = vehicleModel->m_rearTireJoints[0]->GetMaxAngularLimit0(); 
			}
			for (int i = 0; i < vehicleModel->m_rearTiresCount; i ++) {
				vehicleModel->m_rearTireJoints[i]->SetTargetAngle1(steeringAngle);
			}
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
			dAssert (!bone0->m_myController->SelfCollisionTest (bone0, bone1));
			return bone0->m_myController->SelfCollisionTest (bone0, bone1) ? 1 : 0;
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
		if (tireBody) {

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

	NewtonCollision* MakeTireShape (DemoEntity* const bodyPart) const
	{
		dFloat radius = 0.0f;
		dFloat maxWidth = 0.0f;
		dFloat minWidth = 0.0f;

		DemoMesh* const mesh = bodyPart->GetMesh();
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

		DemoMesh* const mesh = bodyPart->GetMesh();
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
		NewtonMesh* const mesh = bodyPart->GetMesh()->CreateNewtonMesh (GetWorld(), bodyPart->GetMeshMatrix());
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
			shape = MakeTireShape(bodyPart);
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


	void CalculateEngine (ArticulatedEntityModel* const vehicleModel, NewtonBody* const chassiBody, NewtonBody* const tireBody)
	{
		// calculate the maximum torque that the engine will produce
		NewtonCollision* const tireShape = NewtonBodyGetCollision(tireBody);
		dAssert (NewtonCollisionGetType(tireShape) == SERIALIZE_ID_CHAMFERCYLINDER);

		dVector p0;
		dVector p1;
		CalculateAABB (tireShape, dGetIdentityMatrix(), p0, p1);

		dFloat Ixx;
		dFloat Iyy;
		dFloat Izz;
		dFloat mass;
		NewtonBodyGetMassMatrix(chassiBody, &mass, &Ixx, &Iyy, &Izz);
		dFloat radius = (p1.m_y - p0.m_y) * 0.5f;

		// calculate a torque the will produce a 0.5f of the force of gravity
		vehicleModel->m_maxEngineTorque = 0.25f * mass * radius * dAbs(DEMO_GRAVITY);

		// calculate the coefficient of drag for top speed of 20 m/s
		dFloat maxOmega = 100.0f / radius;
		vehicleModel->m_omegaResistance = 1.0f / maxOmega;
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

		CustomArticulatedTransformController* const controller = CreateTransformController (vehicleModel, true);

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
		if (vehicleModel->m_frontTiresCount) {
			CalculateEngine (vehicleModel, rootBody, vehicleModel->m_fronTires[0]);
		}

		// disable self collision between all body parts
		controller->DisableAllSelfCollision();

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
		DemoMesh* const mesh = child->GetMesh();
		if (mesh) {
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

	// load a the mesh of the articulate vehicle
	ArticulatedEntityModel forkliffModel(scene, "forklift.ngd");

	// add an input Manage to manage the inputs and user interaction 
	AriculatedJointInputManager* const inputManager = new AriculatedJointInputManager (scene);

	//  create a skeletal transform controller for controlling rag doll
	ArticulatedVehicleManagerManager* const vehicleManager = new ArticulatedVehicleManagerManager (scene);

	NewtonWorld* const world = scene->GetNewton();
	dVector origin (FindFloor (world, dVector (-10.0f, 50.0f, 0.0f, 1.0f), 2.0f * 50.0f));

	dMatrix matrix (dGetIdentityMatrix());
	matrix.m_posit = FindFloor (world, origin, 100.0f);
	matrix.m_posit.m_y += 0.75f;
	inputManager->AddPlayer (vehicleManager->CreateForklift (matrix, &forkliffModel, sizeof(forkliftDefinition) / sizeof (forkliftDefinition[0]), forkliftDefinition));


	// add some object to play with
	DemoEntity entity (dGetIdentityMatrix(), NULL);
	entity.LoadNGD_mesh ("lumber.ngd", scene->GetNewton());
	LoadLumberYardMesh (scene, entity, dVector(10.0f, 0.0f, 0.0f, 0.0f));
	
	origin.m_x -= 5.0f;
	origin.m_y += 5.0f;
	dQuaternion rot (dVector (0.0f, 1.0f, 0.0f, 0.0f), -30.0f * 3.141592f / 180.0f);  
	scene->SetCameraMatrix(rot, origin);

}



