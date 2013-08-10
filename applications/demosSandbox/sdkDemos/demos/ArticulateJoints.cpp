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
#include "CustomBallAndSocket.h"
#include "DebugDisplay.h"
#include "HeightFieldPrimitive.h"
#include "CustomArcticulatedTransformManager.h"


struct ARTICULATED_VEHICLE_DEFINITION
{
	char m_boneName[32];
	char m_shapeType[32];

	dFloat m_mass;

//	dFloat m_shapePitch;
//	dFloat m_shapeYaw;
//	dFloat m_shapeRoll;

//	dFloat m_shape_x;
//	dFloat m_shape_y;
//	dFloat m_shape_z;

//	dFloat m_radio;
//	dFloat m_height;

	

//	dFloat m_coneAngle;
//	dFloat m_minTwistAngle;
//	dFloat m_maxTwistAngle;

//	dFloat m_childPitch;
//	dFloat m_childYaw;
//	dFloat m_childRoll;

//	dFloat m_parentPitch;
//	dFloat m_parentYaw;
//	dFloat m_parentRoll;
};


static ARTICULATED_VEHICLE_DEFINITION forkliftDefinition[] =
{
	{"body",		"convexHull",			600.0f}, 
	{"lift_1",		"convexHull",			50.0f}, 
	{"lift_2",		"convexHull",			50.0f}, 
	{"lift_3",		"convexHull",			50.0f}, 
	{"lift_4",		"convexHull",			50.0f}, 
	{"left_teeth",  "convexHullAggregate",	50.0f}, 
	{"right_teeth", "convexHullAggregate",	50.0f}, 
	{"rr_tire",		"tireShape",			40.0f}, 
	{"rl_tire",		"tireShape",			40.0f}, 
	{"fr_tire",		"tireShape",			40.0f}, 
	{"fl_tire",		"tireShape",			40.0f}, 
};

class ArticulatedVehicleManagerManager: public CustomArticulaledTransformManager
{
	public:
	ArticulatedVehicleManagerManager (DemoEntityManager* const scene)
		:CustomArticulaledTransformManager (scene->GetNewton())
	{
		// create a material for early collision culling
		m_material = NewtonMaterialCreateGroupID(scene->GetNewton());
		NewtonMaterialSetCollisionCallback (scene->GetNewton(), m_material, m_material, this, OnBoneAABBOverlap, NULL);
	}

	virtual void OnPreUpdate (CustomArcticulatedTransformController* const constroller, dFloat timestep, int threadIndex) const
	{
	}

	static int OnBoneAABBOverlap (const NewtonMaterial* const material, const NewtonBody* const body0, const NewtonBody* const body1, int threadIndex)
	{
		NewtonCollision* const collision0 = NewtonBodyGetCollision(body0);
		NewtonCollision* const collision1 = NewtonBodyGetCollision(body1);
		CustomArcticulatedTransformController::dSkeletonBone* const bone0 = (CustomArcticulatedTransformController::dSkeletonBone*)NewtonCollisionGetUserData (collision0);
		CustomArcticulatedTransformController::dSkeletonBone* const bone1 = (CustomArcticulatedTransformController::dSkeletonBone*)NewtonCollisionGetUserData (collision1);
		dAssert (bone0);
		dAssert (bone1);
		if (bone0->m_myController == bone1->m_myController) {
			return bone0->m_myController->SelfCollisionTest (bone0, bone1) ? 1 : 0;
		}
		return 1;
	}

	virtual void OnUpdateTransform (const CustomArcticulatedTransformController::dSkeletonBone* const bone, const dMatrix& localMatrix) const
	{
		DemoEntity* const ent = (DemoEntity*) NewtonBodyGetUserData(bone->m_body);
		DemoEntityManager* const scene = (DemoEntityManager*) NewtonWorldGetUserData(NewtonBodyGetWorld(bone->m_body));

		dQuaternion rot (localMatrix);
		ent->SetMatrix (*scene, rot, localMatrix.m_posit);
	}

	NewtonCollision* MakeTireShape (DemoEntity* const bodyPart) const
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
		return NewtonCreateConvexHull (GetWorld(), mesh->m_vertexCount, &points[0].m_x, sizeof (dVector), 1.0e-3f, 0, NULL);
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
		return NewtonCreateConvexHull (GetWorld(), mesh->m_vertexCount, &points[0].m_x, sizeof (dVector), 1.0e-3f, 0, NULL);
	}

	NewtonCollision* MakeConvexHullAggregate(DemoEntity* const bodyPart) const
	{
		NewtonMesh* const mesh = bodyPart->GetMesh()->CreateNewtonMesh (GetWorld());
		NewtonMesh* const convexApproximation = NewtonMeshApproximateConvexDecomposition (mesh, 0.01f, 0.2f, 32, 100, NULL);
		
		NewtonCollision* const compound = NewtonCreateCompoundCollisionFromMesh (GetWorld(), convexApproximation, 0.001f, 0, 0);

		NewtonMeshDestroy(convexApproximation);
		NewtonMeshDestroy(mesh);
		return compound;
	}

	NewtonBody* CreateBodyPart (DemoEntity* const bodyPart, const ARTICULATED_VEHICLE_DEFINITION& definition) 
	{
		NewtonCollision* shape = NULL;
		if (!strcmp (definition.m_shapeType, "sphere")) {
			dAssert (0);
//			shape = MakeSphere (bodyPart, definition);
		} else if (!strcmp (definition.m_shapeType, "tireShape")) {
			shape = MakeTireShape(bodyPart);
		} else if (!strcmp (definition.m_shapeType, "capsule")) {
			dAssert (0);
//			shape = MakeCapsule(bodyPart, definition);
		} else if (!strcmp (definition.m_shapeType, "box")) {
			dAssert (0);
//			shape = MakeBox (bodyPart);
		} else if (!strcmp (definition.m_shapeType, "convexHull")) {
			shape = MakeConvexHull(bodyPart);
		} else if (!strcmp (definition.m_shapeType, "convexHullAggregate")) {
			shape = MakeConvexHullAggregate(bodyPart);
		} else {
			dAssert (0);
		}

		// calculate the bone matrix
		dMatrix matrix (bodyPart->CalculateGlobalMatrix());

		NewtonWorld* const world = GetWorld();

		// create the rigid body that will make this bone
		NewtonBody* const bone = NewtonCreateDynamicBody (world, shape, &matrix[0][0]);

		// calculate the moment of inertia and the relative center of mass of the solid
		NewtonBodySetMassProperties (bone, definition.m_mass, shape);

		// save the user data with the bone body (usually the visual geometry)
		NewtonBodySetUserData(bone, bodyPart);

		// assign the material for early collision culling
		NewtonBodySetMaterialGroupID (bone, m_material);

		// set the bod part force and torque call back to the gravity force, skip the transform callback
		NewtonBodySetForceAndTorqueCallback (bone, PhysicsApplyGravityForce);

		// destroy the collision helper shape 
		NewtonDestroyCollision (shape);
		return bone;
	}



	void CreateForklift (const dMatrix& location, const DemoEntity* const model, int bodyPartsCount, ARTICULATED_VEHICLE_DEFINITION* const definition)
	{
		NewtonWorld* const world = GetWorld(); 
		DemoEntityManager* const scene = (DemoEntityManager*) NewtonWorldGetUserData(world);

		// make a clone of the mesh 
		DemoEntity* const vehicleModel = (DemoEntity*) model->CreateClone();
		scene->Append(vehicleModel);

		CustomArcticulatedTransformController* const controller = CreateTransformController (vehicleModel, true);

		DemoEntity* const rootEntity = (DemoEntity*) vehicleModel->Find (definition[0].m_boneName);
		NewtonBody* const rootBone = CreateBodyPart (rootEntity, definition[0]);


		// add the root bone to the articulation manager
		CustomArcticulatedTransformController::dSkeletonBone* const bone = controller->AddBone (rootBone, GetIdentityMatrix());
		// save the bone as the shape use data for self collision test
		NewtonCollisionSetUserData (NewtonBodyGetCollision(rootBone), bone);

		// walk down the model hierarchy an add all the components 
		int stackIndex = 0;
		DemoEntity* childEntities[32];
		CustomArcticulatedTransformController::dSkeletonBone* parentBones[32];
		for (DemoEntity* child = rootEntity->GetChild(); child; child = child->GetSibling()) {
			parentBones[stackIndex] = bone;
			childEntities[stackIndex] = child;
			stackIndex ++;
		}

		while (stackIndex) {
			stackIndex --;
			DemoEntity* const entity = childEntities[stackIndex];
			CustomArcticulatedTransformController::dSkeletonBone* parentBone = parentBones[stackIndex];

			const char* const name = entity->GetName().GetStr();
			for (int i = 0; i < bodyPartsCount; i ++) {
				if (!strcmp (definition[i].m_boneName, name)) {
					NewtonBody* const bone = CreateBodyPart (entity, definition[i]);

					// connect this body part to its parent with a vehicle joint
//					ConnectBodyParts (bone, parentBone->m_body, definition[i]);

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
	}

	int m_material;
};


void ArticulatedJoints (DemoEntityManager* const scene)
{
	// load the sky box
	scene->CreateSkyBox();
	CreateLevelMesh (scene, "flatPlane.ngd", true);
	//CreateHeightFieldTerrain (scene, 9, 8.0f, 1.5f, 0.2f, 200.0f, -50.0f);

	// load a the mesh of the articulate vehicle
	DemoEntity forkliffModel(GetIdentityMatrix(), NULL);
	forkliffModel.LoadNGD_mesh ("forklift.ngd", scene->GetNewton());

	//  create a skeletal transform controller for controlling rag doll
	ArticulatedVehicleManagerManager* const manager = new ArticulatedVehicleManagerManager (scene);

	NewtonWorld* const world = scene->GetNewton();
	dMatrix matrix (GetIdentityMatrix());

	dVector origin (FindFloor (world, dVector (-10.0f, 50.0f, 0.0f, 1.0f), 2.0f * 50.0f));

	int count = 1;
	for (int x = 0; x < count; x ++) {
		for (int z = 0; z < count; z ++) {
			dVector p (origin + dVector ((x - count / 2) * 3.0f - count / 2, 0.0f, (z - count / 2) * 3.0f, 0.0f));
			matrix.m_posit = FindFloor (world, p, 100.0f);
			matrix.m_posit.m_y += 3.0f;
			manager->CreateForklift (matrix, &forkliffModel, sizeof(forkliftDefinition) / sizeof (forkliftDefinition[0]), forkliftDefinition);
		}
	}
	
	origin.m_x -= 5.0f;
	origin.m_y += 5.0f;
	dQuaternion rot;
	scene->SetCameraMatrix(rot, origin);
}



