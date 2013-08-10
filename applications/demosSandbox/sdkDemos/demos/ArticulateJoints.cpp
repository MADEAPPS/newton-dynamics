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


class ArticulatedVehicleManagerManager: public CustomArticulaledTransformManager
{
	public:
	ArticulatedVehicleManagerManager (DemoEntityManager* const scene)
		:CustomArticulaledTransformManager (scene->GetNewton())
	{
		// create a material for early collision culling
//		m_material = NewtonMaterialCreateGroupID(scene->GetNewton());
//		NewtonMaterialSetCollisionCallback (scene->GetNewton(), m_material, m_material, this, OnBoneAABBOverlap, NULL);
	}

	virtual void OnPreUpdate (CustomArcticulatedTransformController* const constroller, dFloat timestep, int threadIndex) const
	{
	}

	static int OnBoneAABBOverlap (const NewtonMaterial* const material, const NewtonBody* const body0, const NewtonBody* const body1, int threadIndex)
	{
		dAssert (0);
//		NewtonCollision* const collision0 = NewtonBodyGetCollision(body0);
//		NewtonCollision* const collision1 = NewtonBodyGetCollision(body1);
//		CustomArcticulatedTransformController::dSkeletonBone* const bone0 = (CustomArcticulatedTransformController::dSkeletonBone*)NewtonCollisionGetUserData (collision0);
//		CustomArcticulatedTransformController::dSkeletonBone* const bone1 = (CustomArcticulatedTransformController::dSkeletonBone*)NewtonCollisionGetUserData (collision1);
//		dAssert (bone0);
//		dAssert (bone1);
//		if (bone0->m_myController == bone1->m_myController) {
//			return bone0->m_myController->SelfCollisionTest (bone0, bone1) ? 1 : 0;
//		}
		return 1;
	}

	virtual void OnUpdateTransform (const CustomArcticulatedTransformController::dSkeletonBone* const bone, const dMatrix& localMatrix) const
	{
		dAssert (0);
		DemoEntity* const ent = (DemoEntity*) NewtonBodyGetUserData(bone->m_body);
		DemoEntityManager* const scene = (DemoEntityManager*) NewtonWorldGetUserData(NewtonBodyGetWorld(bone->m_body));

		dQuaternion rot (localMatrix);
		ent->SetMatrix (*scene, rot, localMatrix.m_posit);
	}


	void CreateForklift (const dMatrix& location, const DemoEntity* const model)
	{
		NewtonWorld* const world = GetWorld(); 
		DemoEntityManager* const scene = (DemoEntityManager*) NewtonWorldGetUserData(world);

		// make a clone of the mesh 
		DemoEntity* const vehicleModel = (DemoEntity*) model->CreateClone();
		scene->Append(vehicleModel);


	}
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
			manager->CreateForklift (matrix, &forkliffModel);
		}
	}
	
	origin.m_x -= 25.0f;
	origin.m_y += 5.0f;
	dQuaternion rot;
	scene->SetCameraMatrix(rot, origin);
}



