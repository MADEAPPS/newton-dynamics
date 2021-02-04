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

static void AddBoxEffect(ndSimpleConvexFracture* const manager, const dMatrix& matrix)
{
	ndSimpleConvexFracture::ndDesc desc;

	// first ww make a collision shape that we wnat to brake to pieces
	ndShapeInstance shape(new ndShapeBox(3.0f, 0.5f, 0.5f));

	// next we populate the descriptor for how the shape is going toi be broken in pieces.
	desc.m_shape = &shape;
	desc.m_outTexture = "reljef.tga";
	desc.m_innerTexture = "concreteBrick.tga";
	desc.m_breakImpactSpeed = 10.0f;
	makePointCloud(desc);

	// now with make a template effect that we can place 
	// in the scene many time.
	ndSimpleConvexFracture::ndEffect effect(manager, desc);

	// get a locatiobn in the scene
	dMatrix location(matrix);
	ndWorld* const world = manager->m_scene->GetWorld();
	dVector floor(FindFloor(*world, dVector(location.m_posit.m_x, 100.0f, location.m_posit.m_z, dFloat32(0.0f)), 2.0f * 100.0f));
	location.m_posit.m_y = floor.m_y + 0.25f;

	// place few instance of teh same effect in teh scane.
	const dFloat32 z0 = location.m_posit.m_z;
	for (dInt32 j = 0; j < 5; j++)
	{
		location.m_posit.m_z = z0;
		for (dInt32 i = 0; i < 5; i++)
		{
			location.m_posit.m_z += 0.5f;
			manager->AddEffect(effect, 200.0f, location);
		}
		location.m_posit.m_y += 0.5f;
	}
}

static void AddCapsuleEffect(ndSimpleConvexFracture* const manager, const dMatrix& matrix)
{
	ndSimpleConvexFracture::ndDesc desc;

	ndShapeInstance shape(new ndShapeCapsule(0.25f, 0.25f, 4.0f));

	desc.m_shape = &shape;
	desc.m_outTexture = "wood_0.tga";
	desc.m_innerTexture = "wood_1.tga";
	desc.m_breakImpactSpeed = 10.0f;

	dMatrix location(matrix);
	ndWorld* const world = manager->m_scene->GetWorld();
	dVector floor(FindFloor(*world, dVector(location.m_posit.m_x, 100.0f, location.m_posit.m_z, dFloat32(0.0f)), 2.0f * 100.0f));
	location.m_posit.m_y = floor.m_y + 0.25f;

	makePointCloud(desc);
	ndSimpleConvexFracture::ndEffect effect(manager, desc);

	const dFloat32 z0 = location.m_posit.m_z;
	for (dInt32 j = 0; j < 5; j++)
	{
		location.m_posit.m_z = z0;
		for (dInt32 i = 0; i < 5; i++)
		{
			location.m_posit.m_z += 0.5f;
			manager->AddEffect(effect, 200.0f, location);
		}
		location.m_posit.m_y += 0.5f;
	}
}

static void AddCylinderEffect(ndSimpleConvexFracture* const manager, const dMatrix& matrix)
{
	ndSimpleConvexFracture::ndDesc desc;

	ndShapeInstance shape(new ndShapeCylinder(0.25f, 0.25f, 4.0f));

	desc.m_shape = &shape;
	desc.m_outTexture = "wood_3.tga";
	desc.m_innerTexture = "wood_4.tga";
	desc.m_breakImpactSpeed = 10.0f;

	dMatrix location(matrix);
	ndWorld* const world = manager->m_scene->GetWorld();
	dVector floor(FindFloor(*world, dVector(location.m_posit.m_x, 100.0f, location.m_posit.m_z, dFloat32(0.0f)), 2.0f * 100.0f));
	location.m_posit.m_y = floor.m_y + 0.25f;

	makePointCloud(desc);
	ndSimpleConvexFracture::ndEffect effect(manager, desc);

	const dFloat32 z0 = location.m_posit.m_z;
	for (dInt32 j = 0; j < 5; j++)
	{
		location.m_posit.m_z = z0;
		for (dInt32 i = 0; i < 5; i++)
		{
			location.m_posit.m_z += 0.5f;
			manager->AddEffect(effect, 200.0f, location);
		}
		location.m_posit.m_y += 0.5f;
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

	dMatrix matrix(dGetIdentityMatrix());

	matrix.m_posit.m_x += 10.0f;
	matrix.m_posit.m_y += 2.0f;
	matrix.m_posit.m_z -= 5.0f;
	AddBoxEffect(fractureManager, matrix);

	matrix.m_posit.m_z += 5.0f;
	AddCapsuleEffect(fractureManager, matrix);

	matrix.m_posit.m_z += 5.0f;
	AddCylinderEffect(fractureManager, matrix);

	dQuaternion rot;
	//dVector origin(-80.0f, 5.0f, 0.0f, 0.0f);
	//dVector origin(-60.0f, 5.0f, 0.0f, 0.0f);
	dVector origin(-10.0f, 5.0f, 0.0f, 0.0f);
	scene->SetCameraMatrix(rot, origin);
}
