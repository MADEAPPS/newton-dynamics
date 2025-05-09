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
#include "ndPhysicsUtils.h"
#include "ndPhysicsWorld.h"
#include "ndMakeStaticMap.h"
#include "ndDemoEntityNotify.h"
#include "ndDemoEntityManager.h"
#include "ndDemoInstanceEntity.h"

static ndBodyDynamic* AddRigidBody(ndDemoEntityManager* const scene,
	const ndMatrix& matrix, const ndShapeInstance& shape, 
	ndDemoEntity* const rootEntity, ndFloat32 mass)
{
	ndSharedPtr<ndBody> body (new ndBodyDynamic());
	ndSharedPtr<ndDemoEntity> entity (new ndDemoEntity(matrix));
	rootEntity->AddChild(entity);

	body->SetNotifyCallback(new ndDemoEntityNotify(scene, entity));
	body->SetMatrix(matrix);
	body->GetAsBodyDynamic()->SetCollisionShape(shape);
	body->GetAsBodyDynamic()->SetMassMatrix(mass, shape);

	ndWorld* const world = scene->GetWorld();
	world->AddBody(body);
	return body->GetAsBodyDynamic();
}

static void BuildSphereColumn(ndDemoEntityManager* const scene, ndFloat32 mass, const ndVector& origin, const ndVector& size, ndInt32 count)
{
	// build a standard block stack of 20 * 3 boxes for a total of 60
	ndWorld* const world = scene->GetWorld();

	ndVector blockBoxSize(size);

	// create the stack
	ndMatrix baseMatrix(ndGetIdentityMatrix());

	ndShapeInstance shape(new ndShapeSphere(blockBoxSize.m_x));

	// for the elevation of the floor at the stack position
	baseMatrix.m_posit.m_x = origin.m_x;
	baseMatrix.m_posit.m_z = origin.m_z;

	ndSharedPtr<ndDemoMeshIntance> geometry(new ndDemoMeshIntance("shape", scene->GetShaderCache(), &shape, "earthmap.png", "earthmap.png", "earthmap.png", 1.0f, ndRollMatrix(ndFloat32(-90.0f) * ndDegreeToRad)));

	ndSharedPtr<ndDemoEntity> rootEntity = new ndDemoInstanceEntity(geometry);
	scene->AddEntity(rootEntity);
	
	for (ndInt32 i = 0; i < count; ++i)
	{
		baseMatrix = FindFloor(*world, baseMatrix, shape, 100.0f);
		AddRigidBody(scene, baseMatrix, shape, *rootEntity, mass);
	}
}

static void BuildBoxColumn(ndDemoEntityManager* const scene, ndFloat32 mass, const ndVector& origin, const ndVector& size, ndInt32 count)
{
	// build a standard block stack of 20 * 3 boxes for a total of 60
	ndWorld* const world = scene->GetWorld();

	ndVector blockBoxSize(size);
	blockBoxSize = blockBoxSize.Scale(2.0f);

	// create the stack
	ndMatrix baseMatrix(ndGetIdentityMatrix());

	ndShapeInstance shape(new ndShapeBox(blockBoxSize.m_x, blockBoxSize.m_y, blockBoxSize.m_z));
	ndSharedPtr<ndDemoMeshIntance> geometry(new ndDemoMeshIntance("shape", scene->GetShaderCache(), &shape, "wood_0.png", "wood_0.png", "wood_0.png"));

	// for the elevation of the floor at the stack position
	baseMatrix.m_posit.m_x = origin.m_x;
	baseMatrix.m_posit.m_z = origin.m_z;

	ndSharedPtr<ndDemoEntity> rootEntity = new ndDemoInstanceEntity(geometry);
	scene->AddEntity(rootEntity);

baseMatrix.m_posit.m_y -= 0.01;
	ndMatrix rotation(ndYawMatrix(20.0f * ndDegreeToRad));
	for (ndInt32 i = 0; i < count; ++i) 
	{
		baseMatrix = FindFloor(*world, baseMatrix, shape, 100.0f);
		AddRigidBody(scene, baseMatrix, shape, *rootEntity, mass);
		baseMatrix.m_posit += baseMatrix.m_up.Scale(blockBoxSize.m_x);
		baseMatrix = rotation * baseMatrix;
	}
}

static void BuildCylinderColumn(ndDemoEntityManager* const scene, ndFloat32 mass, const ndVector& origin, const ndVector& size, ndInt32 count)
{
	// build a standard block stack of 20 * 3 boxes for a total of 60
	ndWorld* const world = scene->GetWorld();

	ndVector blockBoxSize(size);

	// create the stack
	ndMatrix baseMatrix(ndGetIdentityMatrix());

	// for the elevation of the floor at the stack position
	baseMatrix.m_posit.m_x = origin.m_x;
	baseMatrix.m_posit.m_z = origin.m_z;

	ndShapeInstance shape(new ndShapeCylinder(blockBoxSize.m_x, blockBoxSize.m_y, blockBoxSize.m_z));
	shape.SetLocalMatrix(ndRollMatrix(ndPi * 0.5f));
	ndSharedPtr<ndDemoMeshIntance> geometry(new ndDemoMeshIntance("shape", scene->GetShaderCache(), &shape, "wood_0.png", "wood_0.png", "wood_0.png"));

	ndSharedPtr<ndDemoEntity> rootEntity = new ndDemoInstanceEntity(geometry);
	scene->AddEntity(rootEntity);

	ndMatrix rotation(ndYawMatrix(20.0f * ndDegreeToRad));

	for (ndInt32 i = 0; i < count; ++i)
	{
		baseMatrix = FindFloor(*world, baseMatrix, shape, 100.0f);
		AddRigidBody(scene, baseMatrix, shape, *rootEntity, mass);
		baseMatrix.m_posit += baseMatrix.m_up.Scale(blockBoxSize.m_z);
		baseMatrix = rotation * baseMatrix;
	}
}

static void BuildPyramid(ndDemoEntityManager* const scene, 
	ndDemoEntity* const rootEntity, const ndShapeInstance& shape,
	ndFloat32 mass, const ndVector& origin, const ndVector& boxSize, ndInt32 count)
{
	ndMatrix matrix(ndGetIdentityMatrix());
	matrix.m_posit = origin;
	matrix.m_posit.m_w = 1.0f;

	// create the shape and visual mesh as a common data to be re used
	ndWorld* const world = scene->GetWorld();
	
	ndFloat32 stepz = boxSize.m_z + 1.0e-2f;
	ndFloat32 stepy = boxSize.m_y + 1.0e-2f;
	stepy = boxSize.m_y;
	
	ndFloat32 y0 = matrix.m_posit.m_y;
	ndFloat32 z0 = matrix.m_posit.m_z - stepz * (ndFloat32)count / 2;

	matrix.m_posit.m_y = y0;
	matrix.m_posit.m_y -= 0.01f;

	for (ndInt32 j = 0; j < count; ++j) 
	{
		matrix.m_posit.m_z = z0;
		matrix = FindFloor(*world, matrix, shape, 100.0f);
		for (ndInt32 i = 0; i < (count - j); ++i) 
		{
			AddRigidBody(scene, matrix, shape, rootEntity, mass);
			matrix.m_posit.m_z += stepz;
		}
		z0 += stepz * 0.5f;
	}
}

void BuildPyramidStacks(ndDemoEntityManager* const scene, ndFloat32 mass, const ndVector& origin, const ndVector& boxSize, ndInt32 stackHigh)
{
	ndVector origin1(origin);

	ndVector size(boxSize.Scale(1.0f));
	ndShapeInstance shape(new ndShapeBox(size.m_x, size.m_y, size.m_z));
	ndSharedPtr<ndDemoMeshIntance> geometry (new ndDemoMeshIntance("shape", scene->GetShaderCache(), &shape, "wood_0.png", "wood_0.png", "wood_0.png"));

	//ndDemoInstanceEntity* const rootEntity = new ndDemoInstanceEntity(geometry);
	ndSharedPtr<ndDemoEntity> rootEntity = new ndDemoInstanceEntity(geometry);
	scene->AddEntity(rootEntity);

	origin1.m_z = 0.0f;
	origin1.m_x += 3.0f;
	BuildPyramid(scene, *rootEntity, shape, mass, origin1, boxSize, stackHigh);
}

static void BuildCapsuleStack(ndDemoEntityManager* const scene, ndFloat32 mass, const ndVector& origin, const ndVector& size, ndInt32 stackHigh)
{
	// build a standard block stack of 20 * 3 boxes for a total of 60
	ndWorld* const world = scene->GetWorld();

	ndVector blockBoxSize(size);

	ndMatrix uvMatrix(ndPitchMatrix(ndPi));
	ndShapeInstance collision(new ndShapeCapsule(blockBoxSize.m_x, blockBoxSize.m_x, blockBoxSize.m_z));

	ndSharedPtr<ndDemoMeshIntance> geometry (new ndDemoMeshIntance("shape", scene->GetShaderCache(), &collision, "smilli.png", "smilli.png", "smilli.png"));

	ndFloat32 vertialStep = blockBoxSize.m_x * 2.0f;
	ndFloat32 horizontalStep = blockBoxSize.m_z * 0.8f;

	ndMatrix matrix0(ndGetIdentityMatrix());
	matrix0.m_posit = origin;
	matrix0.m_posit.m_y += blockBoxSize.m_x;
	matrix0.m_posit.m_w = 1.0f;
	matrix0 = FindFloor(*world, matrix0, collision, 100.0f);

	ndMatrix matrix1(matrix0);
	matrix1.m_posit.m_z += horizontalStep;

	ndMatrix matrix2(ndYawMatrix(ndPi * 0.5f) * matrix0);
	matrix2.m_posit.m_x += horizontalStep * 0.5f;
	matrix2.m_posit.m_z += horizontalStep * 0.5f;
	matrix2.m_posit.m_y += vertialStep;

	ndMatrix matrix3(matrix2);
	matrix3.m_posit.m_x -= horizontalStep;

	ndSharedPtr<ndDemoEntity> rootEntity = new ndDemoInstanceEntity(geometry);
	scene->AddEntity(rootEntity);

	for (ndInt32 i = 0; i < stackHigh / 2; ++i)
	{
		AddRigidBody(scene, matrix0, collision, *rootEntity, mass);
		AddRigidBody(scene, matrix1, collision, *rootEntity, mass);
		AddRigidBody(scene, matrix2, collision, *rootEntity, mass);
		AddRigidBody(scene, matrix3, collision, *rootEntity, mass);

		matrix0.m_posit.m_y += vertialStep * 2.0f;
		matrix1.m_posit.m_y += vertialStep * 2.0f;
		matrix2.m_posit.m_y += vertialStep * 2.0f;
		matrix3.m_posit.m_y += vertialStep * 2.0f;
	}
}

void ndBasicStacks (ndDemoEntityManager* const scene)
{
	// build a floor
	BuildFlatPlane(scene, true);
	ndVector origin(ndVector::m_zero);

	//ndInt32 pyramidHigh = 2;
	//ndInt32 pyramidHigh = 10;
	//ndInt32 pyramidHigh = 20;
	ndInt32 pyramidHigh = 30;
	//ndInt32 pyramidHigh = 50;
	for (ndInt32 i = 0; i < 4; ++i)
	{
		BuildPyramidStacks(scene, 1.0f, origin, ndVector(0.5f, 0.25f, 0.8f, 0.0f), pyramidHigh);
		origin.m_x += 4.0f;
	}
	
	origin = ndVector::m_zero;
	origin.m_x -= 2.0f;
	origin.m_z -= 3.0f;
	BuildSphereColumn(scene, 10.0f, origin, ndVector(0.5f, 0.5f, 0.5f, 0.0f), 20);

	origin.m_z += 6.0f;
	BuildBoxColumn(scene, 10.0f, origin, ndVector(0.5f, 0.5f, 0.5f, 0.0f), 20);
	//BuildBoxColumn(scene, 10.0f, origin, ndVector(0.5f, 0.5f, 0.5f, 0.0f), 1);
	
	origin.m_z += 6.0f;
	BuildCylinderColumn(scene, 10.0f, origin, ndVector(0.75f, 0.6f, 1.0f, 0.0f), 20);

	origin.m_x -= 6.0f;
	origin.m_z -= 6.0f;
	BuildCapsuleStack(scene, 10.0f, origin, ndVector(0.25f, 0.25f, 2.0f, 0.0f), 20);

	origin = ndVector::m_wOne;
	origin.m_x -= 3.0f;
	origin.m_y += 5.0f;

	origin.m_x -= 15.0f;
	origin.m_z += 15.0f;

	ndQuaternion rot(ndYawMatrix(45.0f * ndDegreeToRad));
	scene->SetCameraMatrix(rot, origin);
}
