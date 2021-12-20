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
#include "ndDemoInstanceEntity.h"

static void AddRigidBody(ndDemoEntityManager* const scene, 
	const ndMatrix& matrix, const ndShapeInstance& shape, 
	ndDemoInstanceEntity* const rootEntity, ndFloat32 mass)
{
	ndBodyDynamic* const body = new ndBodyDynamic();
	ndDemoEntity* const entity = new ndDemoEntity(matrix, rootEntity);

	body->SetNotifyCallback(new ndDemoEntityNotify(scene, entity));
	body->SetMatrix(matrix);
	body->SetCollisionShape(shape);
	body->SetMassMatrix(mass, shape);

	ndWorld* const world = scene->GetWorld();
	world->AddBody(body);
}

static void BuildSphereColumn(ndDemoEntityManager* const scene, ndFloat32 mass, const ndVector& origin, const ndVector& size, ndInt32 count)
{
	// build a standard block stack of 20 * 3 boxes for a total of 60
	ndWorld* const world = scene->GetWorld();

	ndVector blockBoxSize(size);

	// create the stack
	ndMatrix baseMatrix(dGetIdentityMatrix());

	// for the elevation of the floor at the stack position
	baseMatrix.m_posit.m_x = origin.m_x;
	baseMatrix.m_posit.m_z = origin.m_z;

	ndVector floor(FindFloor(*world, baseMatrix.m_posit + ndVector(0.0f, 100.0f, 0.0f, 0.0f), 200.0f));
	baseMatrix.m_posit.m_y = floor.m_y + blockBoxSize.m_x;

	ndShapeInstance shape(new ndShapeSphere(blockBoxSize.m_x));
	ndDemoMeshIntance* const geometry = new ndDemoMeshIntance("shape", scene->GetShaderCache(), &shape, "earthmap.tga", "earthmap.tga", "earthmap.tga", 1.0f, dRollMatrix (ndFloat32 (-90.0f) * ndDegreeToRad));

	ndDemoInstanceEntity* const rootEntity = new ndDemoInstanceEntity(geometry);
	scene->AddEntity(rootEntity);
	
	for (ndInt32 i = 0; i < count; i++)
	{
		AddRigidBody(scene, baseMatrix, shape, rootEntity, mass);
		baseMatrix.m_posit += baseMatrix.m_up.Scale(blockBoxSize.m_x * 2.0f);
	}

	geometry->Release();
}

static void BuildBoxColumn(ndDemoEntityManager* const scene, ndFloat32 mass, const ndVector& origin, const ndVector& size, ndInt32 count)
{
	// build a standard block stack of 20 * 3 boxes for a total of 60
	ndWorld* const world = scene->GetWorld();

	ndVector blockBoxSize(size);
	blockBoxSize = blockBoxSize.Scale(2.0f);

	// create the stack
	ndMatrix baseMatrix(dGetIdentityMatrix());

	// for the elevation of the floor at the stack position
	baseMatrix.m_posit.m_x = origin.m_x;
	baseMatrix.m_posit.m_z = origin.m_z;

	ndVector floor(FindFloor(*world, baseMatrix.m_posit + ndVector(0.0f, 100.0f, 0.0f, 0.0f), 200.0f));
	baseMatrix.m_posit.m_y = floor.m_y + blockBoxSize.m_y * 0.5f;

	ndShapeInstance shape(new ndShapeBox(blockBoxSize.m_x, blockBoxSize.m_y, blockBoxSize.m_z));
	ndDemoMeshIntance* const geometry = new ndDemoMeshIntance("shape", scene->GetShaderCache(), &shape, "wood_0.tga", "wood_0.tga", "wood_0.tga");

	ndDemoInstanceEntity* const rootEntity = new ndDemoInstanceEntity(geometry);
	scene->AddEntity(rootEntity);

//baseMatrix.m_posit.m_y -= 0.02f;
	ndMatrix rotation(dYawMatrix(20.0f * ndDegreeToRad));
	for (ndInt32 i = 0; i < count; i++) 
	{
		AddRigidBody(scene, baseMatrix, shape, rootEntity, mass);
		baseMatrix.m_posit += baseMatrix.m_up.Scale(blockBoxSize.m_x);
		baseMatrix = rotation * baseMatrix;
	}

	geometry->Release();
}

static void BuildCylinderColumn(ndDemoEntityManager* const scene, ndFloat32 mass, const ndVector& origin, const ndVector& size, ndInt32 count)
{
	// build a standard block stack of 20 * 3 boxes for a total of 60
	ndWorld* const world = scene->GetWorld();

	ndVector blockBoxSize(size);

	// create the stack
	ndMatrix baseMatrix(dGetIdentityMatrix());

	// for the elevation of the floor at the stack position
	baseMatrix.m_posit.m_x = origin.m_x;
	baseMatrix.m_posit.m_z = origin.m_z;

	ndVector floor(FindFloor(*world, baseMatrix.m_posit + ndVector(0.0f, 100.0f, 0.0f, 0.0f), 200.0f));
	baseMatrix.m_posit.m_y = floor.m_y + blockBoxSize.m_z * 0.5f;

	ndShapeInstance shape(new ndShapeCylinder(blockBoxSize.m_x, blockBoxSize.m_y, blockBoxSize.m_z));
	shape.SetLocalMatrix(dRollMatrix(ndPi * 0.5f));
	ndDemoMeshIntance* const geometry = new ndDemoMeshIntance("shape", scene->GetShaderCache(), &shape, "wood_0.tga", "wood_0.tga", "wood_0.tga");

	ndDemoInstanceEntity* const rootEntity = new ndDemoInstanceEntity(geometry);
	scene->AddEntity(rootEntity);

	ndMatrix rotation(dYawMatrix(20.0f * ndDegreeToRad));

	for (ndInt32 i = 0; i < count; i++)
	{
		AddRigidBody(scene, baseMatrix, shape, rootEntity, mass);
		baseMatrix.m_posit += baseMatrix.m_up.Scale(blockBoxSize.m_z);
		baseMatrix = rotation * baseMatrix;
	}

	geometry->Release();
}

static void BuildPyramid(ndDemoEntityManager* const scene, 
	ndDemoInstanceEntity* const rootEntity, const ndShapeInstance& shape,
	ndFloat32 mass, const ndVector& origin, const ndVector& boxSize, ndInt32 count)
{
	ndMatrix matrix(dGetIdentityMatrix());
	matrix.m_posit = origin;
	matrix.m_posit.m_w = 1.0f;

	// create the shape and visual mesh as a common data to be re used
	ndWorld* const world = scene->GetWorld();

	ndVector floor(FindFloor(*world, origin + ndVector(0.0f, 100.0f, 0.0f, 0.0f), 200.0f));
	matrix.m_posit.m_y = floor.m_y;
	
	ndFloat32 stepz = boxSize.m_z + 1.0e-2f;
	ndFloat32 stepy = boxSize.m_y + 1.0e-2f;
	stepy = boxSize.m_y;
	
	ndFloat32 y0 = matrix.m_posit.m_y + stepy / 2.0f;
	ndFloat32 z0 = matrix.m_posit.m_z - stepz * count / 2;

	matrix.m_posit.m_y = y0;
	matrix.m_posit.m_y -= 0.01f;

	for (ndInt32 j = 0; j < count; j++) 
	{
		matrix.m_posit.m_z = z0;
		for (ndInt32 i = 0; i < (count - j); i++) 
		{
			AddRigidBody(scene, matrix, shape, rootEntity, mass);
			matrix.m_posit.m_z += stepz;
		}
		z0 += stepz * 0.5f;
		matrix.m_posit.m_y += stepy;
	}
}

void BuildPyramidStacks(ndDemoEntityManager* const scene, ndFloat32 mass, const ndVector& origin, const ndVector& boxSize, ndInt32 stackHigh)
{
	ndVector origin1(origin);

	ndVector size(boxSize.Scale(1.0f));
	ndShapeInstance shape(new ndShapeBox(size.m_x, size.m_y, size.m_z));
	ndDemoMeshIntance* const geometry = new ndDemoMeshIntance("shape", scene->GetShaderCache(), &shape, "wood_0.tga", "wood_0.tga", "wood_0.tga");

	ndDemoInstanceEntity* const rootEntity = new ndDemoInstanceEntity(geometry);
	scene->AddEntity(rootEntity);

	origin1.m_z = 0.0f;
	origin1.m_x += 3.0f;
	BuildPyramid(scene, rootEntity, shape, mass, origin1, boxSize, stackHigh);
	geometry->Release();
}

void ndBasicStacks (ndDemoEntityManager* const scene)
{
	// build a floor
	BuildFlatPlane(scene, true);
	ndVector origin(ndVector::m_zero);

	//ndInt32 pyramidHigh = 60;
	//ndInt32 pyramidHigh = 18;
	ndInt32 pyramidHigh = 30;
	//ndInt32 pyramidHigh = 8;
	for (ndInt32 i = 0; i < 10; i++)
	{
		BuildPyramidStacks(scene, 1.0f, origin, ndVector(0.5f, 0.25f, 0.8f, 0.0f), pyramidHigh);
		origin.m_x += 4.0f;
	}
	
	origin = ndVector::m_zero;
	origin.m_x -= 2.0f;
	origin.m_z -= 3.0f;
	BuildSphereColumn(scene, 10.0f, origin, ndVector(0.5f, 0.5f, 0.5f, 0.0f), 20);

	origin.m_z += 6.0f;
	//BuildBoxColumn(scene, 10.0f, origin, ndVector(0.5f, 0.5f, 0.5f, 0.0f), 7);
	BuildBoxColumn(scene, 10.0f, origin, ndVector(0.5f, 0.5f, 0.5f, 0.0f), 20);
	
	origin.m_z += 6.0f;
	//BuildCylinderColumn(scene, 10.0f, origin, ndVector(0.75f, 0.6f, 1.0f, 0.0f), 40);

	ndQuaternion rot(dYawMatrix (45.0f * ndDegreeToRad));
	origin = ndVector::m_zero;
	origin.m_x -= 3.0f;
	origin.m_y += 5.0f;

	origin.m_x -= 15.0f;
	origin.m_z += 15.0f;

	//origin.m_x -= 20.0f;
	//origin.m_z += 20.0f;
	scene->SetCameraMatrix(rot, origin);
}
