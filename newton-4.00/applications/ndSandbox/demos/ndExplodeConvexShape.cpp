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
#include "ndExplodeConvexShapeModel.h"

static void makePointCloud(ndExplodeConvexShapeModel::ndDesc& desc)
{
	//ndVector pMin;
	//ndVector pMax;
	//desc.m_shape->CalculateAabb(dGetIdentityMatrix(), pMin, pMax);
	//ndVector size((pMax - pMin).Scale(0.25f));
	//
	//desc.m_pointCloud.PushBack(ndVector::m_zero);
	//
	//desc.m_pointCloud.PushBack(ndVector(-size.m_x, -size.m_y, -size.m_z, ndFloat32(0.0f)));
	//desc.m_pointCloud.PushBack(ndVector(-size.m_x, -size.m_y, size.m_z, ndFloat32(0.0f)));
	//desc.m_pointCloud.PushBack(ndVector(-size.m_x, size.m_y, -size.m_z, ndFloat32(0.0f)));
	//desc.m_pointCloud.PushBack(ndVector(-size.m_x, size.m_y, size.m_z, ndFloat32(0.0f)));
	//
	//desc.m_pointCloud.PushBack(ndVector(size.m_x, -size.m_y, -size.m_z, ndFloat32(0.0f)));
	//desc.m_pointCloud.PushBack(ndVector(size.m_x, -size.m_y, size.m_z, ndFloat32(0.0f)));
	//desc.m_pointCloud.PushBack(ndVector(size.m_x, size.m_y, -size.m_z, ndFloat32(0.0f)));
	//desc.m_pointCloud.PushBack(ndVector(size.m_x, size.m_y, size.m_z, ndFloat32(0.0f)));
	//
	//for (ndInt32 i = 0; i < desc.m_pointCloud.GetCount(); i++)
	//{
	//	ndFloat32 x = dGaussianRandom(size.m_x);
	//	ndFloat32 y = dGaussianRandom(size.m_y);
	//	ndFloat32 z = dGaussianRandom(size.m_y);
	//	desc.m_pointCloud[i] += ndVector(x, y, z, ndFloat32(0.0f));
	//}

	ndVector pMin;
	ndVector pMax;
	desc.m_shape->CalculateAabb(dGetIdentityMatrix(), pMin, pMax);
	ndVector size(pMax - pMin);

	const ndInt32 count = 20;

	dSetRandSeed(0);
	const ndVector scale(0.2f);
	const ndVector invScale(size * scale.Reciproc());
	for (ndInt32 i = 0; i < count; i++)
	{
		ndFloat32 x = dRand();
		ndFloat32 y = dRand();
		ndFloat32 z = dRand();
		ndVector randPoint(x, y, z, ndFloat32(0.0f));
		randPoint *= invScale;
		randPoint = pMin + scale * randPoint.Floor();
		desc.m_pointCloud.PushBack(randPoint);
	}
}

static ndVector CalculateLocation(ndExplodeConvexShapeModel* const manager, const ndMatrix& matrix, const ndShapeInstance& shape)
{
	ndVector minBox;
	ndVector maxBox;
	shape.CalculateAabb(dGetIdentityMatrix(), minBox, maxBox);

	ndWorld* const world = manager->m_scene->GetWorld();
	ndVector floor(FindFloor(*world, ndVector(matrix.m_posit.m_x, 100.0f, matrix.m_posit.m_z, ndFloat32(0.0f)), 2.0f * 100.0f));

	ndVector boxPadding(ndShapeInstance::GetBoxPadding());
	floor.m_y += (maxBox.m_y - minBox.m_y) * 0.5f - boxPadding.m_y;
	return floor;
}

static void AddBoxEffect(ndExplodeConvexShapeModel* const manager, const ndMatrix& matrix)
{
	ndExplodeConvexShapeModel::ndDesc desc;

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
	ndExplodeConvexShapeModel::ndEffect effect(manager, desc);

	// get a location in the scene
	ndMatrix location(matrix);
	location.m_posit = CalculateLocation(manager, matrix, shape);

	// place few instance of the same effect in the scene.
	const ndInt32 count = 5;
	const ndFloat32 z0 = location.m_posit.m_z;
	for (ndInt32 j = 0; j < count; j++)
	{
		location.m_posit.m_z = z0;
		for (ndInt32 i = 0; i < count; i++)
		{
			location.m_posit.m_z += 0.5f;
			manager->AddEffect(effect, 200.0f, location);
		}
		location.m_posit.m_y += 0.5f;
	}
}

static void AddCapsuleEffect(ndExplodeConvexShapeModel* const manager, const ndMatrix& matrix)
{
	ndExplodeConvexShapeModel::ndDesc desc;

	ndShapeInstance shape(new ndShapeCapsule(0.25f, 0.25f, 4.0f));

	desc.m_shape = &shape;
	desc.m_outTexture = "wood_0.tga";
	desc.m_innerTexture = "wood_1.tga";
	desc.m_breakImpactSpeed = 10.0f;

	ndMatrix location(matrix);
	location.m_posit = CalculateLocation(manager, matrix, shape);

	makePointCloud(desc);
	ndExplodeConvexShapeModel::ndEffect effect(manager, desc);

	const ndInt32 count = 5;
	const ndFloat32 z0 = location.m_posit.m_z;
	for (ndInt32 j = 0; j < count; j++)
	{
		location.m_posit.m_z = z0;
		for (ndInt32 i = 0; i < count; i++)
		{
			location.m_posit.m_z += 0.5f;
			manager->AddEffect(effect, 200.0f, location);
		}
		location.m_posit.m_y += 0.5f;
	}
}

static void AddCylinderEffect(ndExplodeConvexShapeModel* const manager, const ndMatrix& matrix)
{
	ndExplodeConvexShapeModel::ndDesc desc;

	ndShapeInstance shape(new ndShapeCylinder(0.25f, 0.25f, 4.0f));

	desc.m_shape = &shape;
	desc.m_outTexture = "wood_3.tga";
	desc.m_innerTexture = "wood_4.tga";
	desc.m_breakImpactSpeed = 10.0f;

	ndMatrix location(matrix);
	location.m_posit = CalculateLocation(manager, matrix, shape);

	makePointCloud(desc);
	ndExplodeConvexShapeModel::ndEffect effect(manager, desc);

	const ndInt32 count = 5;
	const ndFloat32 z0 = location.m_posit.m_z;
	for (ndInt32 j = 0; j < count; j++)
	{
		location.m_posit.m_z = z0;
		for (ndInt32 i = 0; i < count; i++)
		{
			location.m_posit.m_z += 0.5f;
			manager->AddEffect(effect, 200.0f, location);
		}
		location.m_posit.m_y += 0.5f;
	}
}

void ndBasicExplodeConvexShape(ndDemoEntityManager* const scene)
{
	// build a floor
	BuildFloorBox(scene, dGetIdentityMatrix());

	ndPhysicsWorld* const world = scene->GetWorld();
	ndExplodeConvexShapeModel* const fractureManager = new ndExplodeConvexShapeModel(scene);
	world->AddModel(fractureManager);

	ndMatrix matrix(dGetIdentityMatrix());

	matrix.m_posit.m_x += 10.0f;
	matrix.m_posit.m_y += 2.0f;
	matrix.m_posit.m_z -= 5.0f;
	AddBoxEffect(fractureManager, matrix);

	matrix.m_posit.m_z += 5.0f;
	AddCapsuleEffect(fractureManager, matrix);

	matrix.m_posit.m_z += 5.0f;
	AddCylinderEffect(fractureManager, matrix);

	ndQuaternion rot;
	ndVector origin(-10.0f, 5.0f, 0.0f, 0.0f);
	scene->SetCameraMatrix(rot, origin);
}
