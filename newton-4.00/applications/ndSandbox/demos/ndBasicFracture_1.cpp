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
#include "ndPhysicsUtils.h"
#include "ndPhysicsWorld.h"
#include "ndMakeStaticMap.h"
#include "ndDemoEntityManager.h"
#include "ndConvexFractureModel_1.h"

static void makePointCloud(ndConvexFractureModel_1::ndDesc& desc)
{
	//dVector pMin;
	//dVector pMax;
	//desc.m_shape->CalculateAABB(dGetIdentityMatrix(), pMin, pMax);
	//dVector size((pMax - pMin).Scale(0.25f));
	//
	//desc.m_pointCloud.PushBack(dVector::m_zero);
	//
	//desc.m_pointCloud.PushBack(dVector(-size.m_x, -size.m_y, -size.m_z, dFloat32(0.0f)));
	//desc.m_pointCloud.PushBack(dVector(-size.m_x, -size.m_y, size.m_z, dFloat32(0.0f)));
	//desc.m_pointCloud.PushBack(dVector(-size.m_x, size.m_y, -size.m_z, dFloat32(0.0f)));
	//desc.m_pointCloud.PushBack(dVector(-size.m_x, size.m_y, size.m_z, dFloat32(0.0f)));
	//
	//desc.m_pointCloud.PushBack(dVector(size.m_x, -size.m_y, -size.m_z, dFloat32(0.0f)));
	//desc.m_pointCloud.PushBack(dVector(size.m_x, -size.m_y, size.m_z, dFloat32(0.0f)));
	//desc.m_pointCloud.PushBack(dVector(size.m_x, size.m_y, -size.m_z, dFloat32(0.0f)));
	//desc.m_pointCloud.PushBack(dVector(size.m_x, size.m_y, size.m_z, dFloat32(0.0f)));
	//
	//for (dInt32 i = 0; i < desc.m_pointCloud.GetCount(); i++)
	//{
	//	dFloat32 x = dGaussianRandom(size.m_x);
	//	dFloat32 y = dGaussianRandom(size.m_y);
	//	dFloat32 z = dGaussianRandom(size.m_y);
	//	desc.m_pointCloud[i] += dVector(x, y, z, dFloat32(0.0f));
	//}

	dVector pMin;
	dVector pMax;
	desc.m_shape->CalculateAABB(dGetIdentityMatrix(), pMin, pMax);
	dVector size(pMax - pMin);

	const dInt32 count = 20;

	dSetRandSeed(0);
	const dVector scale(0.2f);
	const dVector invScale(size * scale.Reciproc());
	for (dInt32 i = 0; i < count; i++)
	{
		dFloat32 x = dRand();
		dFloat32 y = dRand();
		dFloat32 z = dRand();
		dVector randPoint(x, y, z, dFloat32(0.0f));
		randPoint *= invScale;
		randPoint = pMin + scale * randPoint.Floor();
		desc.m_pointCloud.PushBack(randPoint);
	}
}

static dVector CalculateLocation(ndConvexFractureModel_1* const manager, const dMatrix& matrix, const ndShapeInstance& shape)
{
	dVector minBox;
	dVector maxBox;
	shape.CalculateAABB(dGetIdentityMatrix(), minBox, maxBox);

	ndWorld* const world = manager->m_scene->GetWorld();
	dVector floor(FindFloor(*world, dVector(matrix.m_posit.m_x, 100.0f, matrix.m_posit.m_z, dFloat32(0.0f)), 2.0f * 100.0f));

	dVector boxPadding(ndShapeInstance::GetBoxPadding());
	floor.m_y += (maxBox.m_y - minBox.m_y) * 0.5f - boxPadding.m_y;
	return floor;
}

static void AddBoxEffect(ndConvexFractureModel_1* const manager, const dMatrix& matrix)
{
	ndConvexFractureModel_1::ndDesc desc;

	// first make a collision shape that we want to brake to pieces
	ndShapeInstance shape(new ndShapeBox(3.0f, 0.5f, 0.5f));

	// next we populate the descriptor for how the shape is going to be broken in pieces.
	desc.m_shape = &shape;
	desc.m_outTexture = "reljef.tga";
	desc.m_innerTexture = "concreteBrick.tga";
	desc.m_breakImpactSpeed = 10.0f;
	makePointCloud(desc);

	// now with make a template effect that we can place 
	// in the scene many time.
	ndConvexFractureModel_1::ndEffect effect(manager, desc);

	// get a location in the scene
	dMatrix location(matrix);
	location.m_posit = CalculateLocation(manager, matrix, shape);

	// place few instance of the same effect in the scene.
	const dInt32 count = 5;
	const dFloat32 z0 = location.m_posit.m_z;
	for (dInt32 j = 0; j < count; j++)
	{
		location.m_posit.m_z = z0;
		for (dInt32 i = 0; i < count; i++)
		{
			location.m_posit.m_z += 0.5f;
			manager->AddEffect(effect, 200.0f, location);
		}
		location.m_posit.m_y += 0.5f;
	}
}

static void AddCapsuleEffect(ndConvexFractureModel_1* const manager, const dMatrix& matrix)
{
	ndConvexFractureModel_1::ndDesc desc;

	ndShapeInstance shape(new ndShapeCapsule(0.25f, 0.25f, 4.0f));

	desc.m_shape = &shape;
	desc.m_outTexture = "wood_0.tga";
	desc.m_innerTexture = "wood_1.tga";
	desc.m_breakImpactSpeed = 10.0f;

	dMatrix location(matrix);
	location.m_posit = CalculateLocation(manager, matrix, shape);

	makePointCloud(desc);
	ndConvexFractureModel_1::ndEffect effect(manager, desc);

	const dInt32 count = 5;
	const dFloat32 z0 = location.m_posit.m_z;
	for (dInt32 j = 0; j < count; j++)
	{
		location.m_posit.m_z = z0;
		for (dInt32 i = 0; i < count; i++)
		{
			location.m_posit.m_z += 0.5f;
			manager->AddEffect(effect, 200.0f, location);
		}
		location.m_posit.m_y += 0.5f;
	}
}

static void AddCylinderEffect(ndConvexFractureModel_1* const manager, const dMatrix& matrix)
{
	ndConvexFractureModel_1::ndDesc desc;

	ndShapeInstance shape(new ndShapeCylinder(0.25f, 0.25f, 4.0f));

	desc.m_shape = &shape;
	desc.m_outTexture = "wood_3.tga";
	desc.m_innerTexture = "wood_4.tga";
	desc.m_breakImpactSpeed = 10.0f;

	dMatrix location(matrix);
	location.m_posit = CalculateLocation(manager, matrix, shape);

	makePointCloud(desc);
	ndConvexFractureModel_1::ndEffect effect(manager, desc);

	const dInt32 count = 5;
	const dFloat32 z0 = location.m_posit.m_z;
	for (dInt32 j = 0; j < count; j++)
	{
		location.m_posit.m_z = z0;
		for (dInt32 i = 0; i < count; i++)
		{
			location.m_posit.m_z += 0.5f;
			manager->AddEffect(effect, 200.0f, location);
		}
		location.m_posit.m_y += 0.5f;
	}
}

void ndBasicFracture_1(ndDemoEntityManager* const scene)
{
	// build a floor
	BuildFloorBox(scene, dGetIdentityMatrix());

	ndPhysicsWorld* const world = scene->GetWorld();
	ndConvexFractureModel_1* const fractureManager = new ndConvexFractureModel_1(scene);
	world->AddModel(fractureManager);

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
	dVector origin(-10.0f, 5.0f, 0.0f, 0.0f);
	scene->SetCameraMatrix(rot, origin);
}
