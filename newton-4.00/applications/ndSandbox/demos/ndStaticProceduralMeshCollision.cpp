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
#include "ndCompoundScene.h"
#include "ndDemoEntityManager.h"
#include "ndBasicPlayerCapsule.h"
#include "ndHeightFieldPrimitive.h"
#include "ndMakeProceduralStaticMap.h"

void ndStaticUserMeshCollisionDemo (ndDemoEntityManager* const scene)
{
	BuildProceduralMap(scene, 500, 4.0f, 0.0f);

	ndMatrix location(ndGetIdentityMatrix());
	location.m_posit.m_y += 2.0f;

	ndMatrix localAxis(ndGetIdentityMatrix());
	localAxis[0] = ndVector(0.0, 1.0f, 0.0f, 0.0f);
	localAxis[1] = ndVector(1.0, 0.0f, 0.0f, 0.0f);
	localAxis[2] = localAxis[0].CrossProduct(localAxis[1]);

	ndSharedPtr<ndDemoEntity> man (ndDemoEntity::LoadFbx("walker.fbx", scene));

	ndFloat32 height = 1.9f;
	ndFloat32 radio = 0.5f;
	ndFloat32 mass = 100.0f;
	new ndBasicPlayerCapsule(scene, *man, localAxis, location, mass, radio, height, height/4.0f, true);
	
	location.m_posit.m_x += 8.0f;
	location.m_posit.m_z -= 2.0f;
	//new ndBasicPlayerCapsule(scene, *man, localAxis, location, mass, radio, height, height / 4.0f);
	
	location.m_posit.m_z += 4.0f;
	//new ndBasicPlayerCapsule(scene, *man, localAxis, location, mass, radio, height, height / 4.0f);

	class PlaceMatrix: public ndMatrix
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

	//AddBox(scene, PlaceMatrix(10.0f, 1.0f, 0.0f), 30.0f, 2.0f, 0.25f, 2.5f);
	//AddBox(scene, PlaceMatrix(10.0f, 1.5f, 1.125f), 30.0f, 2.0f, 0.25f, 2.5f);
	//AddBox(scene, PlaceMatrix(10.0f, 2.0f, 1.250f), 30.0f, 2.0f, 0.25f, 2.5f);
	//AddConvexHull(scene, PlaceMatrix(8.0f, 1.0f, -3.0f), 10.0f, 0.6f, 1.0f, 15);
	//AddConvexHull(scene, PlaceMatrix(7.0f, 1.0f, -3.0f), 10.0f, 0.7f, 1.0f, 10);
	//AddConvexHull(scene, PlaceMatrix(6.0f, 1.0f, -3.0f), 10.0f, 0.5f, 1.2f, 6);
	//AddCapsulesStacks(scene, PlaceMatrix(45.0f, 0.0f, 0.0f), 10.0f, 0.5f, 0.5f, 1.0f, 10, 10, 7);

	ndQuaternion rot;
	ndVector origin(-5.0f, 4.0f, 0.0f, 1.0f);
	scene->SetCameraMatrix(rot, origin);

	//ndFileFormatSave xxxxSave;
	//xxxxSave.SaveWorld(scene->GetWorld(), "xxxx.nd");
	ndFileFormatLoad xxxxLoad;
	xxxxLoad.Load("xxxx.nd");
	// offset bodies positions for calibration;
	const ndList<ndSharedPtr<ndBody>>& bodyList = xxxxLoad.GetBodyList();
	for (ndList<ndSharedPtr<ndBody>>::ndNode* node = bodyList.GetFirst(); node; node = node->GetNext())
	{
		ndSharedPtr<ndBody>& body = node->GetInfo();
		ndMatrix bodyMatrix(body->GetMatrix());
		bodyMatrix.m_posit.m_x += 4.0f;
		body->SetMatrix(bodyMatrix);
	}
	xxxxLoad.AddToWorld(scene->GetWorld());

}
