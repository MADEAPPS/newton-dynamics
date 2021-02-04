/* Copyright (c) <2003-2019> <Newton Game Dynamics>
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
#include "ndPhysicsUtils.h"
#include "ndPhysicsWorld.h"
#include "ndMakeStaticMap.h"
#include "ndDemoEntityManager.h"
#include "ndSimpleConvexFracture.h"

static void makePointCloud(ndSimpleConvexFracture::ndDesc& desc)
{
	//	dVector pMin;
	//	dVector pMax;
	//	desc.m_shape->CalculateAABB(dGetIdentityMatrix(), pMin, pMax);
	//
	//	dInt32 steps = 2;
	//	dVector size (pMax - pMin);
	//	for (dInt32 z = 0; z <= steps; z++)
	//	{
	//		dFloat32 z0 = pMin.m_z + z * size.m_z / steps + dGaussianRandom(size.m_z * 0.1f);
	//		for (dInt32 y = 0; y <= steps; y++)
	//		{
	//			dFloat32 y0 = pMin.m_y + y * size.m_y / steps + dGaussianRandom(size.m_y * 0.1f);
	//			for (dInt32 x = 0; x <= steps; x++)
	//			{
	//				dFloat32 x0 = pMin.m_x + x * size.m_x / steps + dGaussianRandom(size.m_x * 0.1f);
	//				dVector point(x0, y0, z0, dFloat32(0.0f));
	//				desc.m_pointCloud.PushBack(point);
	//			}
	//		}
	//	}
	//}

	dVector pMin;
	dVector pMax;
	desc.m_shape->CalculateAABB(dGetIdentityMatrix(), pMin, pMax);
	dVector size((pMax - pMin).Scale(0.25f));

	desc.m_pointCloud.PushBack(dVector::m_zero);

	desc.m_pointCloud.PushBack(dVector(-size.m_x, -size.m_y, -size.m_z, dFloat32(0.0f)));
	desc.m_pointCloud.PushBack(dVector(-size.m_x, -size.m_y, size.m_z, dFloat32(0.0f)));
	desc.m_pointCloud.PushBack(dVector(-size.m_x, size.m_y, -size.m_z, dFloat32(0.0f)));
	desc.m_pointCloud.PushBack(dVector(-size.m_x, size.m_y, size.m_z, dFloat32(0.0f)));

	desc.m_pointCloud.PushBack(dVector(size.m_x, -size.m_y, -size.m_z, dFloat32(0.0f)));
	desc.m_pointCloud.PushBack(dVector(size.m_x, -size.m_y, size.m_z, dFloat32(0.0f)));
	desc.m_pointCloud.PushBack(dVector(size.m_x, size.m_y, -size.m_z, dFloat32(0.0f)));
	desc.m_pointCloud.PushBack(dVector(size.m_x, size.m_y, size.m_z, dFloat32(0.0f)));

	for (dInt32 i = 0; i < desc.m_pointCloud.GetCount(); i++)
	{
		dFloat32 x = dGaussianRandom(size.m_x);
		dFloat32 y = dGaussianRandom(size.m_y);
		dFloat32 z = dGaussianRandom(size.m_y);
		desc.m_pointCloud[i] += dVector(x, y, z, dFloat32(0.0f));
	}
}


static void AddEffect0(ndSimpleConvexFracture* const manager, const dMatrix& matrix)
{
	ndSimpleConvexFracture::ndDesc desc;

	ndShapeInstance shape(new ndShapeBox(0.5f, 0.5f, 3.0f));

	desc.m_shape = &shape;
	desc.m_outTexture = "reljef.tga";
	desc.m_innerTexture = "concreteBrick.tga";
	desc.m_breakImpactSpeed = 10.0f;

	dMatrix location(matrix);
	ndWorld* const world = manager->m_scene->GetWorld();
	dVector floor(FindFloor(*world, dVector(location.m_posit.m_x, 100.0f, location.m_posit.m_z, dFloat32(0.0f)), 2.0f * 100.0f));
	location.m_posit.m_y = floor.m_y + 0.25f;

	makePointCloud(desc);
	ndSimpleConvexFracture::ndEffect effect(manager, desc);

//	for (dInt32 j = 0; j < 3; j++)
	for (dInt32 i = 0; i < 3; i++)
	{
		manager->AddEffect(effect, 200.0f, location);
	}
}

void ndBasicConvexFracturing(ndDemoEntityManager* const scene)
{
	// build a floor
	BuildFloorBox(scene);

	ndPhysicsWorld* const world = scene->GetWorld();
	ndSimpleConvexFracture* const fractureManager = new ndSimpleConvexFracture(scene);
	world->AddModel(fractureManager);
	world->RegisterModelUpdate(fractureManager);

	//dInt32 woodX = 3;
	//dInt32 woodZ = 3;
	dMatrix matrix(dGetIdentityMatrix());
	matrix.m_posit.m_x += 10.0f;
	matrix.m_posit.m_y += 2.0f;

	AddEffect0(fractureManager, matrix);
	//ndShapeInstance shape2(new ndShapeBox(0.5f, 0.5f, 3.0f));
	//fractureManager->AddFracturedWoodPrimitive(scene, shape2, 
	//	"wood_4.tga", "wood_0.tga",
	//	10.0f, 1000.0f, matrix.m_posit, woodX, woodZ, 1.0f, 0, 0);


	//ndShapeInstance shape0(new ndShapeCylinder(0.5f, 0.5f, 3.0f));
	//fractureManager->AddFracturedWoodPrimitive(scene, shape0, 
	//	"reljef.tga", "concreteBrick.tga",
	//	10.0f, 1000.0f, matrix.m_posit, woodX, woodZ, 1.0f, 0, 0);
	//
	//matrix.m_posit.m_x += 10.0f;
	//matrix.m_posit.m_z += 4.0f;
	//ndShapeInstance shape1(new ndShapeCapsule(0.5f, 0.5f, 3.0f));
	//fractureManager->AddFracturedWoodPrimitive(scene, shape1, 
	//	"wood_0.tga", "wood_1.tga",
	//	10.0f, 1000.0f, matrix.m_posit, woodX, woodZ, 1.0f, 0, 0);



	dQuaternion rot;
	//dVector origin(-80.0f, 5.0f, 0.0f, 0.0f);
	//dVector origin(-60.0f, 5.0f, 0.0f, 0.0f);
	dVector origin(-10.0f, 5.0f, 0.0f, 0.0f);
	scene->SetCameraMatrix(rot, origin);
}
