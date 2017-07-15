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

#include <toolbox_stdafx.h>
#include "SkyBox.h"
#include "DemoMesh.h"
#include "DemoCamera.h"
#include "NewtonDemos.h"
#include "PhysicsUtils.h"
#include "TargaToOpenGl.h"
#include "DemoEntityManager.h"
#include "dCustomBallAndSocket.h"
#include "DebugDisplay.h"
#include "HeightFieldPrimitive.h"
#include "dCustomActiveCharacterManager.h"



class DynamicRagdollManager: public dCustomActiveCharacterManager
{
	public: 
	DynamicRagdollManager (DemoEntityManager* const scene)
		:dCustomActiveCharacterManager (scene->GetNewton())
	{
		// create a material for early collision culling
		m_material = NewtonMaterialCreateGroupID(scene->GetNewton());
		//NewtonMaterialSetCallbackUserData (scene->GetNewton(), m_material, m_material, this);
		//NewtonMaterialSetCollisionCallback (scene->GetNewton(), m_material, m_material, OnBoneAABBOverlap, NULL);
	}


#if 0
	static int OnBoneAABBOverlap (const NewtonMaterial* const material, const NewtonBody* const body0, const NewtonBody* const body1, int threadIndex)
	{
		dAssert (0);

		NewtonCollision* const collision0 = NewtonBodyGetCollision(body0);
		NewtonCollision* const collision1 = NewtonBodyGetCollision(body1);
		dCustomActiveCharacterController::dSkeletonBone* const bone0 = (dCustomActiveCharacterController::dSkeletonBone*)NewtonCollisionGetUserData (collision0);
		dCustomActiveCharacterController::dSkeletonBone* const bone1 = (dCustomActiveCharacterController::dSkeletonBone*)NewtonCollisionGetUserData (collision1);

		dAssert (bone0);
		dAssert (bone1);
		if (bone0->m_myController && bone1->m_myController) {
			return bone0->m_myController->SelfCollisionTest (bone0, bone1) ? 1 : 0;
		}
*/
		return 1;
	}

	virtual void OnUpdateTransform (const dCustomActiveCharacterController::dSkeletonBone* const bone, const dMatrix& localMatrix) const
	{
		DemoEntity* const ent = (DemoEntity*) NewtonBodyGetUserData(bone->m_body);
		DemoEntityManager* const scene = (DemoEntityManager*) NewtonWorldGetUserData(NewtonBodyGetWorld(bone->m_body));
		
		dQuaternion rot (localMatrix);
		ent->SetMatrix (*scene, rot, localMatrix.m_posit);

		dCustomActiveCharacterControllerManager::OnUpdateTransform (bone, localMatrix);
	}

#endif

	NewtonCollision* ParseCollisonShape(FILE* const file)
	{
		char token[256];
		NewtonWorld* const world = GetWorld();
		NewtonCollision* collision = NULL;

		fscanf (file, "%s", token);

		if (!strcmp(token, "sphere")) {
			dFloat radio;
			fscanf(file, "%s %f", token, &radio);
			collision = NewtonCreateSphere(world, radio, 0, NULL);
		} else if (!strcmp (token, "capsule")) {
			dFloat radio0;
			dFloat radio1;
			dFloat height;
			fscanf (file, "%s %f %s %f %s %f", token, &radio0, token, &radio1, token, &height);
			collision = NewtonCreateCapsule(world, radio0, radio1, height, 0, NULL);
		} else if (!strcmp (token, "convexHull")) {
			dVector array[1024];
			int pointCount;
			fscanf (file, "%s %d", token, &pointCount);
			for (int i = 0; i < pointCount; i ++) {
				fscanf (file, "%s %f %f %f", token, &array[i].m_x, &array[i].m_y, &array[i].m_z);
			}
			collision = NewtonCreateConvexHull(world, pointCount, &array[0][0], sizeof(dVector), 1.0e-3f, 0, NULL);
		} else {
			dAssert (0);
		}

		dVector scale;
		dVector posit;
		dVector euler;
		fscanf (file, "%s %f %f %f", token, &scale.m_x, &scale.m_y, &scale.m_z);
		fscanf (file, "%s %f %f %f", token, &posit.m_x, &posit.m_y, &posit.m_z);
		fscanf (file, "%s %f %f %f", token, &euler.m_x, &euler.m_y, &euler.m_z);

		euler = euler.Scale (3.141592f / 180.0f);
		dMatrix matrix (euler.m_x, euler.m_y, euler.m_z, posit);
		NewtonCollisionSetMatrix(collision, &matrix[0][0]);
NewtonCollisionSetMode(collision, 0);
		return collision;
	}

	NewtonBody* MakeRigidBody (const dVector& posit, const dVector& euler, dFloat mass, NewtonCollision* const collision)
	{
		dMatrix matrix(euler.m_x, euler.m_y, euler.m_z, posit);
		NewtonWorld* const world = GetWorld();
		DemoEntityManager* const scene = (DemoEntityManager*)NewtonWorldGetUserData(world);
		DemoMesh* const mesh = new DemoMesh("calf", collision, "smilli.tga", "smilli.tga", "smilli.tga");
		NewtonBody* const body = CreateSimpleSolid(scene, mesh, mass, matrix, collision, m_material);
		mesh->Release();
		NewtonDestroyCollision(collision);
		return body;
	}

	void ParseRigidBody(FILE* const file, dTree<NewtonBody*, dString> bodyMap)
	{
		dVector posit(0.0f);
		dVector euler(0.0f);
		NewtonBody* body = NULL;
		NewtonCollision* collision = NULL;
		dFloat mass;

		char token[256];
		char nodeName[256];

		while (!feof(file)) {
			fscanf(file, "%s", token);

			if (!strcmp(token, "node:")) {
				fscanf(file, "%s", &nodeName);
			} else if (!strcmp(token, "mass:")) {
				fscanf(file, "%f", &mass);
			} else if (!strcmp(token, "position:")) {
				fscanf(file, "%f %f %f", &posit.m_x, &posit.m_y, &posit.m_z);
			} else if (!strcmp(token, "eulerAngles:")) {
				fscanf(file, "%f %f %f", &euler.m_x, &euler.m_y, &euler.m_z);
				euler = euler.Scale(3.141592f / 180.0f);
			} else if (!strcmp(token, "shapeType:")) {
				collision = ParseCollisonShape(file);
			} else if (!strcmp(token, "nodeEnd:")) {
				body = MakeRigidBody(posit, euler, mass, collision);
				bodyMap.Insert(body, nodeName);
				break;
			} else {
				dAssert(0);
			}
		}
	}

	NewtonBody* ParseRigidBodies(FILE* const file, NewtonBody* const parentNode)
	{
/*
		dVector posit(0.0f);
		dVector euler(0.0f);
		NewtonBody* body = NULL;
		NewtonCollision* collision = NULL;
		
		dFloat mass;

		char token[256];
		char nodeName[256];

		while(!feof (file)) {
			fscanf (file, "%s", token);

			if (!strcmp(token, "mass:")) {
				fscanf(file, "%f", &mass);
			} else if (!strcmp(token, "position:")) {
				fscanf(file, "%f %f %f", &posit.m_x, &posit.m_y, &posit.m_z);
			} else if (!strcmp(token, "eulerAngles:")) {
				fscanf(file, "%f %f %f", &euler.m_x, &euler.m_y, &euler.m_z);
				euler = euler.Scale (3.141592f / 180.0f);
			} else if (!strcmp(token, "shapeType:")) {
				collision = ParseCollisonShape(file);
			} else if (!strcmp(token, "node:") || !strcmp(token, "endNode:")) {
				body = MakeRigidBody (posit, euler, mass, collision, parentNode);
				if (!strcmp(token, "node:")) {
					fscanf(file, "%s", nodeName);
					return ParseRigidBodies(file, body);
				} else {
					fscanf (file, "%s", token);
					if (!strcmp(token, "node:")) {
						fscanf(file, "%s", nodeName);
						return ParseRigidBodies(file, parentNode);
					} else {
						return body;
					}
				}
			} else {
				dAssert (0);
			}
		}
		return body;
*/

		char token[256];
		//char nodeName[256];
		char rootBodyName[256];

		dTree<NewtonBody*, dString> bodyMap;

		int nodesCount;
		fscanf(file, "%s %d", token, &nodesCount);
		fscanf(file, "%s %s", token, rootBodyName);

		for (int i = 0; i < nodesCount; i ++) {
			ParseRigidBody(file, bodyMap);
		}

		return NULL;
	}




	void ParseRagdollFile(const char* const name, dCustomActiveCharacterController* const controller)
	{
		char fileName[2048];
		dGetWorkingFileName(name, fileName);

		FILE* const file = fopen(fileName, "rt");
		dAssert(file);

		NewtonBody* const rootBone = ParseRigidBodies (file, NULL);

		fclose (file);
	}


	void CreateBasicGait (dMatrix& matrix)
	{
#if 0
/*
		NewtonWorld* const world = GetWorld();
		DemoEntityManager* const scene = (DemoEntityManager*)NewtonWorldGetUserData(world);

		dMatrix offsetMatrix(dGetIdentityMatrix());

		// make the pelvis
		dFloat pelvisMass = 12.0f;
		dVector pelvisSize(0.25f, 0.15f, 0.25f, 0.0f);
		NewtonCollision* const pelvisShape = CreateConvexCollision(world, offsetMatrix, pelvisSize, _CAPSULE_PRIMITIVE, 0);
NewtonCollisionSetMode(pelvisShape, 0);
		DemoMesh* const pelvisMesh = new DemoMesh("pelvis", pelvisShape, "smilli.tga", "smilli.tga", "smilli.tga");
		NewtonBody* const pelvisBody = CreateSimpleSolid (scene, pelvisMesh, pelvisMass, matrix, pelvisShape, m_material);

		dCustomActiveCharacterController* const controller = CreateTransformController(pelvisBody);

		// add right legs
		dFloat legMass = 8.0f;
		dVector legSize(0.25f, 0.4f, 0.20f, 0.0f);
		NewtonCollision* const legShape = CreateConvexCollision(world, offsetMatrix, legSize, _CAPSULE_PRIMITIVE, 0);
NewtonCollisionSetMode(legShape, 0);
		DemoMesh* const legMesh = new DemoMesh("leg", legShape, "smilli.tga", "smilli.tga", "smilli.tga");
		{
			dMatrix rightLegMatrix (dRollMatrix(-100.0f * 3.14159f / 180.0f) * matrix);
			rightLegMatrix.m_posit.m_y -= 0.30f;
			rightLegMatrix.m_posit.m_z += 0.20f;
			NewtonBody* const rightLegBody = CreateSimpleSolid(scene, legMesh, legMass, rightLegMatrix, legShape, m_material);
			dCustomActiveCharacterJoint* const joint = ConnectBodyParts(rightLegBody, pelvisBody);
			controller->AddBone (joint, controller->GetRoot());
		}

		{
			dMatrix leftLegMatrix(dRollMatrix(-80.0f * 3.14159f / 180.0f) * matrix);
			leftLegMatrix.m_posit.m_y -= 0.30f;
			leftLegMatrix.m_posit.m_z -= 0.20f;
			//NewtonBody* const leftLegBody = CreateSimpleSolid(scene, legMesh, legMass, leftLegMatrix, legShape, m_material);
		}

		// add calfs
		dFloat calfMass = 6.0f;
		dVector calfSize(0.20f, 0.4f, 0.20f, 0.0f);
		NewtonCollision* const calfShape = CreateConvexCollision(world, offsetMatrix, calfSize, _CAPSULE_PRIMITIVE, 0);
NewtonCollisionSetMode(calfShape, 0);
		DemoMesh* const calfMesh = new DemoMesh("calf", calfShape, "smilli.tga", "smilli.tga", "smilli.tga");

		{
			dMatrix rightCalfMatrix (rightLegMatrix);
			rightCalfMatrix.m_posit.m_y -= 0.46f;
			rightCalfMatrix.m_posit.m_z += 0.09f;
			NewtonBody* const rightCalfBody = CreateSimpleSolid(scene, calfMesh, calfMass, rightCalfMatrix, calfShape, m_material);
		}

		{
			dMatrix leftCalfMatrix(leftLegMatrix);
			leftCalfMatrix.m_posit.m_y -= 0.46f;
			leftCalfMatrix.m_posit.m_z -= 0.09f;
			NewtonBody* const leftCalfBody = CreateSimpleSolid(scene, calfMesh, calfMass, leftCalfMatrix, calfShape, m_material);
		}
		// add foots
		legMesh->Release();
		//calfMesh->Release();
		pelvisMesh->Release();
		NewtonDestroyCollision(legShape);
		//NewtonDestroyCollision(calfShape);
		NewtonDestroyCollision(pelvisShape);

		controller->Finalize();
*/
#endif

		dCustomActiveCharacterController* const controller = CreateTransformController();
		ParseRagdollFile("balancingGait.txt", controller);
	}

	int m_material;
};


void DynamicRagDoll (DemoEntityManager* const scene)
{
	// load the sky box
	scene->CreateSkyBox();
	CreateLevelMesh (scene, "flatPlane.ngd", true);
	//CreateHeightFieldTerrain(scene, HEIGHTFIELD_DEFAULT_SIZE, HEIGHTFIELD_DEFAULT_CELLSIZE, 1.5f, 0.2f, 200.0f, -50.0f);

	// load a skeleton mesh for using as a ragdoll manager
	DemoEntity ragDollModel(dGetIdentityMatrix(), NULL);
	ragDollModel.LoadNGD_mesh ("skeleton.ngd", scene->GetNewton());
//	ragDollModel.LoadNGD_mesh ("gymnast.ngd", scene->GetNewton());

	//  create a skeletal transform controller for controlling rag doll
	DynamicRagdollManager* const manager = new DynamicRagdollManager (scene);

	NewtonWorld* const world = scene->GetNewton();
	//dMatrix matrix (dGetIdentityMatrix());
	dMatrix matrix (dYawMatrix(3.141592f * 0.5f));

//	dVector origin (-10.0f, 1.0f, 0.0f, 1.0f);
	dVector origin (FindFloor (world, dVector (-4.0f, 50.0f, 0.0f, 1.0f), 2.0f * 50.0f));

//	int count = 10;
	int count = 1;
	for (int x = 0; x < count; x ++) {
		for (int z = 0; z < count; z ++) {
			dVector p (origin + dVector ((x - count / 2) * 3.0f - count / 2, 0.0f, (z - count / 2) * 3.0f, 0.0f));
			matrix.m_posit = FindFloor (world, p, 100.0f);
			matrix.m_posit.m_y += 1.3f;
			//manager->CreateRagDoll (matrix, &ragDollModel, skeletonRagDoll, sizeof (skeletonRagDoll) / sizeof (skeletonRagDoll[0]));
			manager->CreateBasicGait (matrix);

		}
	}
/*
	const int defaultMaterialID = NewtonMaterialGetDefaultGroupID(scene->GetNewton());
	const dVector location(origin);
	const dVector size(0.25f, 0.25f, 0.375f, 0.0f);
	const int count1 = 5;
	const dMatrix shapeOffsetMatrix(dGetIdentityMatrix());
	AddPrimitiveArray(scene, 10.0f, location, size, count1, count1, 5.0f, _BOX_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
*/

	origin.m_x = -14.0f;
//	origin.m_x -= 2.0f;
	origin.m_y  = 1.5f;
	dQuaternion rot;
	scene->SetCameraMatrix(rot, origin);
}



