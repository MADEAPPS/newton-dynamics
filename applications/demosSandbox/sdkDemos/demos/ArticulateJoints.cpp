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



void ArticulatedJoints (DemoEntityManager* const scene)
{
	// load the sky box
	scene->CreateSkyBox();
	CreateLevelMesh (scene, "flatPlane.ngd", true);
	//CreateHeightFieldTerrain (scene, 9, 8.0f, 1.5f, 0.2f, 200.0f, -50.0f);

	// load a skeleton mesh for using as a ragdoll manager
	DemoEntity ragDollModel(GetIdentityMatrix(), NULL);
	ragDollModel.LoadNGD_mesh ("skeleton.ngd", scene->GetNewton());

	//  create a skeletal transform controller for controlling rag doll
//	RagDollManager* const manager = new RagDollManager (scene);

	NewtonWorld* const world = scene->GetNewton();
	dMatrix matrix (GetIdentityMatrix());

	dVector origin (FindFloor (world, dVector (-10.0f, 50.0f, 0.0f, 1.0f), 2.0f * 50.0f));

	int count = 1;
	for (int x = 0; x < count; x ++) {
		for (int z = 0; z < count; z ++) {
			dVector p (origin + dVector ((x - count / 2) * 3.0f - count / 2, 0.0f, (z - count / 2) * 3.0f, 0.0f));
			matrix.m_posit = FindFloor (world, p, 100.0f);
			matrix.m_posit.m_y += 3.0f;
//			manager->CreateRagDoll (matrix, &ragDollModel, skeletonRagDoll, sizeof (skeletonRagDoll) / sizeof (skeletonRagDoll[0]));
		}
	}
	
	origin.m_x -= 25.0f;
	origin.m_y += 5.0f;
	dQuaternion rot;
	scene->SetCameraMatrix(rot, origin);
}



