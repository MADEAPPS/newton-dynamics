/* Copyright (c) <2003-2021> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/

#include "ndSandboxStdafx.h"
#include "ndSkyBox.h"
#include "ndTargaToOpenGl.h"
#include "ndDemoMesh.h"
#include "ndDemoCamera.h"
#include "ndLoadFbxMesh.h"
#include "ndPhysicsUtils.h"
#include "ndPhysicsWorld.h"
#include "ndMakeStaticMap.h"
#include "ndDemoEntityManager.h"
#include "ndDemoInstanceEntity.h"

void ndBasicRagdoll (ndDemoEntityManager* const scene)
{
	// build a floor
	BuildFloorBox(scene, dGetIdentityMatrix());

	dVector origin1(0.0f, 0.0f, 0.0f, 0.0f);
	//AddCapsulesStacks(scene, origin1, 10.0f, 0.5f, 0.5f, 1.0f, 10, 10, 7);
	//fbxDemoEntity* const ragdoll = LoadFbxMesh("whiteMan.fbx");
	fbxDemoEntity* const ragdoll = LoadFbxMesh("skinTest0.fbx");

	ragdoll->BuildRenderMeshes(scene);
	scene->AddEntity(ragdoll);

	dQuaternion rot;
	dVector origin(-10.0f, 1.0f, 0.0f, 0.0f);
	scene->SetCameraMatrix(rot, origin);
}
