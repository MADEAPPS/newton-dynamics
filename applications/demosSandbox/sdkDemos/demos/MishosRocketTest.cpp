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
#include "DemoEntityManager.h"
#include "DemoCamera.h"
#include "PhysicsUtils.h"
#include "DemoMesh.h"
#include "OpenGlUtil.h"

void MishosRocketTest(DemoEntityManager* const scene)
{
	// set DEMO_GRAVITY constant to turn on off gravity

	scene->CreateSkyBox();

	//NewtonBody* const levelMeshBody = CreateLevelMesh(scene, "flatPlane.ngd", 1);
	//dMatrix offsetPlane(dGetIdentityMatrix());
	////offsetPlane.m_posit.m_y = 6378135.0f;
	//offsetPlane.m_posit.m_y = 20.0f;
	//NewtonBodySetMatrix(levelMeshBody, &offsetPlane[0][0]);

	NewtonWorld* const world = scene->GetNewton();
	dMatrix offset(dGetIdentityMatrix());
	dFloat vertOffset = 6378135.0f;
	//dFloat vertOffset = 0.0f;
	dVector damp(0.0f);

	// Make Launch platform as a solid static box...
	NewtonCollision* const LP0 = NewtonCreateBox(world, 20.0f, 10.0f, 20.0f, 0, &offset[0][0]);
	NewtonCollision* const LaunchPlatformShape = NewtonCreateSceneCollision(world, 0);
	NewtonSceneCollisionBeginAddRemove(LaunchPlatformShape);
	NewtonSceneCollisionAddSubCollision(LaunchPlatformShape, LP0);
	NewtonSceneCollisionEndAddRemove(LaunchPlatformShape);
	dMatrix matrixLP(0.0f, 0.0f, 0.0f, dVector(0.0f, vertOffset - 5.0f, 0.0f));
	dFloat LPMass = 979452.0f; // irrelevant, this is a static object, here for conformity
	DemoMesh* const geometryLP = new DemoMesh("Launch Platform", LaunchPlatformShape, "metal_30.tga", "metal_30.tga", "metal_30.tga");
	NewtonBody* const LPBody = CreateSimpleSolid(scene, geometryLP, LPMass, matrixLP, LaunchPlatformShape, 0);
	//NewtonBodySetMassProperties(coreStageBody, coreStageMass, CoreStageShape);
	NewtonBodySetLinearDamping(LPBody, 0.0f);
	NewtonBodySetAngularDamping(LPBody, &damp[0]);
	geometryLP->Release();
	NewtonDestroyCollision(LaunchPlatformShape);
	NewtonDestroyCollision(LP0);

	// Make Core Stage
	NewtonCollision* const CS0 = NewtonCreateCylinder(world, 4.25f, 4.25f, 59.8f, 1, &offset[0][0]);
	NewtonCollision* const CoreStageShape = NewtonCreateCompoundCollision(world, 0);
	NewtonCompoundCollisionBeginAddRemove(CoreStageShape);
	NewtonCompoundCollisionAddSubCollision(CoreStageShape, CS0);
	NewtonCompoundCollisionEndAddRemove(CoreStageShape);
	dMatrix matrixCS(0.0f, 0.0f, 1.5708f, dVector(0.0f, vertOffset + 30.1f, 0.0f));
	dFloat coreStageMass = 979452.0f;
	DemoMesh* const geometryCoreStage = new DemoMesh("CoreStage", CoreStageShape, "metal_30.tga", "metal_30.tga", "metal_30.tga");
	NewtonBody* const coreStageBody = CreateSimpleSolid(scene, geometryCoreStage, coreStageMass, matrixCS, CoreStageShape, 0);
	//NewtonBodySetMassProperties(coreStageBody, coreStageMass, CoreStageShape);
	NewtonBodySetLinearDamping(coreStageBody, 0.0f);
	NewtonBodySetAngularDamping(coreStageBody, &damp[0]);
	geometryCoreStage->Release();
	NewtonDestroyCollision(CoreStageShape);
	NewtonDestroyCollision(CS0);

	// place camera into position and rotate towards objects
	dMatrix camMatrix(dGetIdentityMatrix());
	dQuaternion dqRotY(dVector(0.0f, 1.0f, 0.0f), 90.0f * 3.141592f / 180.0f);
	dQuaternion rot(camMatrix);
	dVector origin(0.0, vertOffset + 5.0f, 120.0f, 0.0f);
	scene->SetCameraMatrix(rot*dqRotY, origin);

	NewtonSerializeToFile(scene->GetNewton(), "RocketLaunchFar.bin", NULL, NULL);

}