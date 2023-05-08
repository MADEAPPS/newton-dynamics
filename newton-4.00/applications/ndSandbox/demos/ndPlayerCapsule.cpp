/* Copyright (c) <2003-2022> <Newton Game Dynamics>
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
#include "ndMakeStaticMap.h"
#include "ndCompoundScene.h"
#include "ndDemoEntityManager.h"
#include "ndDemoInstanceEntity.h"
#include "ndBasicPlayerCapsule.h"

void ndPlayerCapsuleDemo (ndDemoEntityManager* const scene)
{
	// build a floor
	//BuildPlayArena(scene);
	BuildFloorBox(scene, ndGetIdentityMatrix());
	//BuildCompoundScene(scene, ndGetIdentityMatrix());

	ndMatrix location(ndGetIdentityMatrix());
	location.m_posit.m_y += 2.0f;

	ndMatrix localAxis(ndGetIdentityMatrix());
	localAxis[0] = ndVector(0.0f, 1.0f, 0.0f, 0.0f);
	localAxis[1] = ndVector(1.0f, 0.0f, 0.0f, 0.0f);
	localAxis[2] = localAxis[0].CrossProduct(localAxis[1]);

	ndFloat32 height = 1.9f;
	ndFloat32 radio = 0.5f;
	ndFloat32 mass = 100.0f;

	ndPhysicsWorld* const world = scene->GetWorld();
	ndSharedPtr<ndDemoEntity> entity(ndDemoEntity::LoadFbx("walker.fbx", scene));
	
	ndSharedPtr<ndBody> player0(new ndBasicPlayerCapsule(scene, *entity, localAxis, location, mass, radio, height, height / 4.0f, true));
	world->AddBody(player0);

	ndSharedPtr<ndBody> player1(new ndBasicPlayerCapsule(scene, *entity, localAxis, location, mass, radio, height, height/4.0f));
	//world->AddBody(player1);

	location.m_posit.m_z += 2.0f;
	ndSharedPtr<ndBody> player2(new ndBasicPlayerCapsule(scene, *entity, localAxis, location, mass, radio, height, height / 4.0f));
	//world->AddBody(player2);
	
	location.m_posit.m_z += 2.0f;
	ndSharedPtr<ndBody> player3(new ndBasicPlayerCapsule(scene, *entity, localAxis, location, mass, radio, height, height / 4.0f));
	//world->AddBody(player3);

	class PlaceMatrix : public ndMatrix
	{
		public:
		PlaceMatrix(ndFloat32 x, ndFloat32 y, ndFloat32 z)
			:ndMatrix(ndGetIdentityMatrix())
		{
			m_posit.m_x = x;
			m_posit.m_y = y;
			m_posit.m_z = z;
		}
	};

	//AddCapsulesStacks___(scene, PlaceMatrix(32.0f, 0.0f, 0.0f), 10.0f, 0.5f, 0.5f, 1.0f, 10, 10, 7);
	//AddBox(scene, PlaceMatrix(10.0f, 0.0f, 0.0f), 30.0f, 2.0f, 0.25f, 2.5f);
	//AddBox(scene, PlaceMatrix(10.0f, 0.5f, 1.125f), 30.0f, 2.0f, 0.25f, 2.5f);
	//AddBox(scene, PlaceMatrix(10.0f, 1.0f, 1.250f), 30.0f, 2.0f, 0.25f, 2.5f);

	ndQuaternion rot;
	ndVector origin(-10.0f, 5.0f, 0.0f, 1.0f);
	scene->SetCameraMatrix(rot, origin);
}
