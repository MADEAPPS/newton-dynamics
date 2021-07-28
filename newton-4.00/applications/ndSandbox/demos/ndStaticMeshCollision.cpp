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
#include "ndDemoMesh.h"
#include "ndDemoCamera.h"
#include "ndLoadFbxMesh.h"
#include "ndPhysicsUtils.h"
#include "ndPhysicsWorld.h"
#include "ndCompoundScene.h"
#include "ndTargaToOpenGl.h"
#include "ndMakeStaticMap.h"
#include "ndDemoEntityManager.h"
#include "ndBasicPlayerCapsule.h"
#include "ndHeightFieldPrimitive.h"

void ndStaticMeshCollisionDemo (ndDemoEntityManager* const scene)
{
	dMatrix heighfieldLocation (dGetIdentityMatrix());
	heighfieldLocation.m_posit.m_x = -200.0f;
	heighfieldLocation.m_posit.m_z = -200.0f;

	//BuildFlatPlane(scene, true);
	//BuildCompoundScene(scene, heighfieldLocation);
	BuildHeightFieldTerrain(scene, heighfieldLocation);
	//BuildStaticMesh(scene, "flatPlane.fbx", false);
	//BuildStaticMesh(scene, "track.fbx", false);
	//BuildStaticMesh(scene, "playerarena.fbx", true);
	//BuildStaticMesh(scene, "excavator.fbx", false);

	dMatrix location(dGetIdentityMatrix());
	location.m_posit.m_y += 2.0f;

	dMatrix localAxis(dGetIdentityMatrix());
	localAxis[0] = dVector(0.0, 1.0f, 0.0f, 0.0f);
	localAxis[1] = dVector(1.0, 0.0f, 0.0f, 0.0f);
	localAxis[2] = localAxis[0].CrossProduct(localAxis[1]);

	LoadFbxAnimation("whiteMan_idle.fbx");

	fbxDemoEntity* const man = scene->LoadFbxMesh("whiteMan.fbx");

	dFloat32 height = 1.9f;
	dFloat32 radio = 0.5f;
	dFloat32 mass = 100.0f;
	new ndBasicPlayerCapsule(scene, man, localAxis, location, mass, radio, height, height/4.0f, true);
	
	location.m_posit.m_x += 8.0f;
	location.m_posit.m_z -= 2.0f;
	new ndBasicPlayerCapsule(scene, man, localAxis, location, mass, radio, height, height / 4.0f);
	
	location.m_posit.m_z += 4.0f;
	new ndBasicPlayerCapsule(scene, man, localAxis, location, mass, radio, height, height / 4.0f);

	AddBox(scene, dVector(10.0f, 1.0f, 0.0f, 0.0f), 30.0f, 2.0f, 0.25f, 2.5f);
	AddBox(scene, dVector(10.0f, 1.5f, 1.125f, 0.0f), 30.0f, 2.0f, 0.25f, 2.5f);
	AddBox(scene, dVector(10.0f, 2.0f, 1.250f, 0.0f), 30.0f, 2.0f, 0.25f, 2.5f);
	AddConvexHull(scene, dVector(8.0f, 1.0f, -3.0f, 0.0f), 10.0f, 0.6f, 1.0f, 15);
	AddConvexHull(scene, dVector(7.0f, 1.0f, -3.0f, 0.0f), 10.0f, 0.7f, 1.0f, 10);
	AddConvexHull(scene, dVector(6.0f, 1.0f, -3.0f, 0.0f), 10.0f, 0.5f, 1.2f, 6);
	AddCapsulesStacks(scene, dVector(45.0f, 0.0f, 0.0f, 0.0f), 10.0f, 0.5f, 0.5f, 1.0f, 10, 10, 7);

	delete man;
	dQuaternion rot;
	dVector origin(-5.0f, 4.0f, 0.0f, 0.0f);
	scene->SetCameraMatrix(rot, origin);
}
