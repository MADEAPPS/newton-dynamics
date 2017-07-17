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

		scene->Set2DDisplayRenderFunction (Debug, this);
	}


	static void Debug (DemoEntityManager* const scene, void* const context, int lineNumber)
	{
//		SuperCarVehicleControllerManager* const me = (SuperCarVehicleControllerManager*) context;
//		me->RenderVehicleHud (scene, lineNumber);
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
//NewtonCollisionSetMode(collision, 0);
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

	void ParseRigidBody(FILE* const file, dTree<NewtonBody*, const dString>& bodyMap)
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

	void ParseJoint(FILE* const file, const dTree<NewtonBody*, const dString>& bodyMap)
	{
		char token[256];
		char jointType[256];
		char childName[256];
		char parentName[256];

		NewtonBody* child;
		NewtonBody* parent;

		dVector childPivot;
		dVector parentPivot;
		dVector childEuler;
		dVector parentEuler;
		dFloat coneAngle;
		dFloat minTwistAngle;
		dFloat maxTwistAngle;

		while (!feof(file)) {
			fscanf(file, "%s", token);
			if (!strcmp(token, "joint:")) {
				fscanf(file, "%s", &jointType);
			} else if (!strcmp(token, "childBody:")) {
				fscanf(file, "%s", childName);
				child = bodyMap.Find(childName)->GetInfo();
			} else if (!strcmp(token, "parentBody:")) {
				fscanf(file, "%s", parentName);
				parent = bodyMap.Find(parentName)->GetInfo();
			} else if (!strcmp(token, "childPivot:")) {
				fscanf(file, "%f %f %f", &childPivot.m_x, &childPivot.m_y, &childPivot.m_z);
			} else if (!strcmp(token, "childEulers:")) {
				fscanf(file, "%f %f %f", &childEuler.m_x, &childEuler.m_y, &childEuler.m_z);
			} else if (!strcmp(token, "parentPivot:")) {
				fscanf(file, "%f %f %f", &parentPivot.m_x, &parentPivot.m_y, &parentPivot.m_z);
			} else if (!strcmp(token, "parentEulers:")) {
				fscanf(file, "%f %f %f", &parentEuler.m_x, &parentEuler.m_y, &parentEuler.m_z);
			} else if (!strcmp(token, "coneAngle:")) {
				fscanf(file, "%f", &coneAngle);
			} else if (!strcmp(token, "minTwistAngle:")) {
				fscanf(file, "%f", &minTwistAngle);
			} else if (!strcmp(token, "maxTwistAngle:")) {
				fscanf(file, "%f", &maxTwistAngle);
			} else if (!strcmp(token, "jointEnd:")) {
				break;
			} else {
				dAssert(0);
			}
		}


		dMatrix childBodyMatrix;
		dMatrix parentBodyMatrix;

		childEuler = childEuler.Scale (3.141592f / 180.0f);
		parentEuler = parentEuler.Scale(3.141592f / 180.0f);

		NewtonBodyGetMatrix(child, &childBodyMatrix[0][0]);
		NewtonBodyGetMatrix(parent, &parentBodyMatrix[0][0]);
		dMatrix childPinAndPivotInGlobalSpace (childEuler.m_x, childEuler.m_y, childEuler.m_z, childPivot);
		dMatrix parentPinAndPivotInGlobalSpace (parentEuler.m_x, parentEuler.m_y, parentEuler.m_z, parentPivot);

		childPinAndPivotInGlobalSpace = childPinAndPivotInGlobalSpace * childBodyMatrix;
		parentPinAndPivotInGlobalSpace = parentPinAndPivotInGlobalSpace * parentBodyMatrix;

		if (!strcmp(jointType, "dCustomRagdollMotor")) {
			dCustomRagdollMotor* const joint = new dCustomRagdollMotor(childPinAndPivotInGlobalSpace, child, parentPinAndPivotInGlobalSpace, parent);
			joint->SetAngle0(-coneAngle * 3.141592f / 180.0f, coneAngle * 3.141592f / 180.0f);
			joint->SetAngle1(-coneAngle * 3.141592f / 180.0f, coneAngle * 3.141592f / 180.0f);
			joint->SetTwistAngle(minTwistAngle * 3.141592f / 180.0f, maxTwistAngle * 3.141592f / 180.0f);

		} else {
			dAssert(0);
		}
	}

	NewtonBody* ParseRigidBodies(FILE* const file, NewtonBody* const parentNode)
	{
		char token[256];
		char rootBodyName[256];

		dTree<NewtonBody*, const dString> bodyMap;

		int nodesCount;
		fscanf(file, "%s %d", token, &nodesCount);
		fscanf(file, "%s %s", token, rootBodyName);
		for (int i = 0; i < nodesCount; i ++) {
			ParseRigidBody(file, bodyMap);
		}

		int jointsCount;
		fscanf(file, "%s %d", token, &jointsCount);
		for (int i = 0; i < jointsCount; i++) {
			ParseJoint(file, bodyMap);
		}

		return bodyMap.Find(rootBodyName)->GetInfo();
	}


	NewtonBody* ParseRagdollFile(const char* const name, dCustomActiveCharacterController* const controller)
	{
		char fileName[2048];
		dGetWorkingFileName(name, fileName);

		char* const oldloc = setlocale( LC_ALL, 0 );
		setlocale( LC_ALL, "C" );
		FILE* const file = fopen(fileName, "rt");
		dAssert(file);

		NewtonBody* const rootBone = ParseRigidBodies (file, NULL);

		fclose (file);
		setlocale (LC_ALL, oldloc);

		return rootBone;
	}


	void CreateBasicGait (const dVector& posit, dFloat angle)
	{
		dCustomActiveCharacterController* const controller = CreateTransformController();
		NewtonBody* const root = ParseRagdollFile("balancingGait.txt", controller);

		dMatrix bodyMatrix;
		NewtonBodyGetMatrix (root, &bodyMatrix[0][0]);

		bodyMatrix = dYawMatrix(angle) * bodyMatrix;
		bodyMatrix.m_posit = posit;
		bodyMatrix.m_posit.m_w = 1.0f;
		NewtonBodySetMatrixRecursive (root, &bodyMatrix[0][0]);

		void* const rootNode = controller->AddRoot(root);

		int stack = 1;
		void* stackPool[128];
		stackPool[0] = rootNode;

		dTree<int, NewtonJoint*> filter; 
		while (stack) {
			stack --;
			void* const node = stackPool[stack];
			NewtonBody* const rootBody = controller->GetBody(node);

			for (NewtonJoint* joint = NewtonBodyGetFirstJoint(rootBody); joint; joint = NewtonBodyGetNextJoint(rootBody, joint)) {
				if (!filter.Find(joint)) {
					filter.Insert(joint);
					dCustomJoint* const cJoint = (dCustomJoint*)NewtonJointGetUserData(joint);
					void* const bone = controller->AddBone(cJoint, node);
					stackPool[stack] = bone;
					stack ++;
				}
			}
		}


		controller->Finalize();
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

//	dVector origin (-10.0f, 1.0f, 0.0f, 1.0f);
	dVector origin (FindFloor (world, dVector (-4.0f, 50.0f, 0.0f, 1.0f), 2.0f * 50.0f));

//	int count = 10;
	int count = 1;
	for (int x = 0; x < count; x ++) {
		for (int z = 0; z < count; z ++) {
			dVector p (origin + dVector ((x - count / 2) * 3.0f - count / 2, 0.0f, (z - count / 2) * 3.0f, 0.0f));
			p = FindFloor (world, p, 100.0f);
			p.m_y += 0.9f;
			manager->CreateBasicGait (p, 0.0f);
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

	origin.m_x = -8.0f;
//	origin.m_x -= 2.0f;
	origin.m_y  = 1.5f;
	dQuaternion rot;
	scene->SetCameraMatrix(rot, origin);
}



