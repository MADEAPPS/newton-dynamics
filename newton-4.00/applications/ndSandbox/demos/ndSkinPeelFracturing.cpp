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
#include "ndSkinPeelFracture.h"
#include "ndDemoEntityManager.h"

static dVector CalculateLocation(ndSkinPeelFracture* const manager, const dMatrix& matrix, const ndShapeInstance& shape)
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

static void makePointCloud(ndSkinPeelFracture::ndDesc& desc)
{
	dVector pMin;
	dVector pMax;
	desc.m_outerShape->CalculateAABB(dGetIdentityMatrix(), pMin, pMax);
	dVector size(pMax - pMin);

	//const dInt32 count = 10;
	const dInt32 count = 100;
	dFloat32 scale = 0.2f;
	dFloat32 invScale = 1.0f / scale;
	for (dInt32 i = 0; i < count; i++)
	{
		dFloat32 x = pMin.m_x + scale * dFloor(dRand() * size.m_x * invScale);
		dFloat32 y = pMin.m_y + scale * dFloor(dRand() * size.m_y * invScale);
		dFloat32 z = pMin.m_z + scale * dFloor(dRand() * size.m_z * invScale);

		desc.m_pointCloud.PushBack (dVector(x, y, z, dFloat32(0.0f)));
	}
}

static void AddBoxEffect(ndSkinPeelFracture* const manager, const dMatrix& matrix)
{
	ndSkinPeelFracture::ndDesc desc;

	// first make a collision shape that we want to brake to pieces

	dVector shapeBox(1.0f, 5.0f, 20.0f, 0.0f);
	ndShapeInstance outerShape(new ndShapeBox(shapeBox.m_x, shapeBox.m_y, shapeBox.m_z));

	shapeBox -= dVector(0.25f);
	ndShapeInstance innerShape(new ndShapeBox(shapeBox.m_x, shapeBox.m_y, shapeBox.m_z));

	// next we populate the descriptor for how the shape is going to be broken in pieces.
	desc.m_outerShape = &outerShape;
	desc.m_innerShape = &innerShape;
	desc.m_outTexture = "reljef.tga";
	desc.m_innerTexture = "concreteBrick.tga";
	desc.m_breakImpactSpeed = 10.0f;
	desc.m_breakImpactSpeed = 0.0f;
	makePointCloud(desc);

	// now with make a template effect that we can place 
	// in the scene many time.
	ndSkinPeelFracture::ndEffect effect(manager, desc);

	// get a location in the scene
	dMatrix location(matrix);
	location.m_posit = CalculateLocation(manager, matrix, outerShape);
location.m_posit.m_y += 0.5f;

	// place few instance of the same effect in the scene.
	const dInt32 count = 1;
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

void ndSkinPeelFracturing(ndDemoEntityManager* const scene)
{
	// build a floor
	BuildFloorBox(scene);

	ndPhysicsWorld* const world = scene->GetWorld();
	ndSkinPeelFracture* const fractureManager = new ndSkinPeelFracture(scene);
	world->AddModel(fractureManager);
	world->RegisterModelUpdate(fractureManager);

	dMatrix matrix(dGetIdentityMatrix());
	matrix.m_posit.m_x += 10.0f;
	matrix.m_posit.m_y += 2.0f;

	AddBoxEffect(fractureManager, matrix);

	dQuaternion rot;
	dVector origin(-10.0f, 5.0f, 0.0f, 0.0f);
	scene->SetCameraMatrix(rot, origin);
}
