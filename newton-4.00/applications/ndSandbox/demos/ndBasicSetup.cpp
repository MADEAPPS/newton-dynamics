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
#include "ndDemoEntityManager.h"

static void BuildFloor(ndDemoEntityManager* const scene)
{
	ndPhysicsWorld* const world = scene->GetWorld();

	ndShapeInstance box(new ndShapeBox(200.0f, 1.0f, 200.f));
	dMatrix uvMatrix(dGetIdentityMatrix());
	uvMatrix[0][0] *= 0.025f;
	uvMatrix[1][1] *= 0.025f;
	uvMatrix[2][2] *= 0.025f;
	ndDemoMesh* const geometry = new ndDemoMesh("box", scene->GetShaderCache(), &box, "wood_0.tga", "marbleCheckBoard.tga", "wood_0.tga", 1.0f, uvMatrix);
	
	dMatrix matrix(dGetIdentityMatrix());
	matrix.m_posit.m_y = -0.5f;
	ndDemoEntity* const entity = new ndDemoEntity(matrix, nullptr);
	entity->SetMesh(geometry, dGetIdentityMatrix());

	ndBodyDynamic* const body = new ndBodyDynamic();
	body->SetNotifyCallback(new ndDemoEntityNotify(entity));
	body->SetMatrix(matrix);
	body->SetCollisionShape(box);
	
	world->AddBody(body);

	scene->AddEntity(entity);
	geometry->Release();
}

static void BuildSphere(ndDemoEntityManager* const scene, 
	ndDemoMesh* const sphereMesh, const ndShapeInstance& sphereShape,
	dFloat32 mass, const dVector& origin, const dFloat32 diameter, int count, dFloat32 xxxx)
{
	//dMatrix matrix(dGetIdentityMatrix());
	dMatrix matrix(dRollMatrix(90.0f * dDegreeToRad));
	//dMatrix matrix(dYawMatrix(90.0f * dDegreeToRad) * dPitchMatrix(-45.0f * dDegreeToRad));
	matrix.m_posit = origin;
	matrix.m_posit.m_w = 1.0f;

	ndPhysicsWorld* const world = scene->GetWorld();

	dVector floor(FindFloor(*world, matrix.m_posit + dVector(0.0f, 100.0f, 0.0f, 0.0f), 200.0f));
	matrix.m_posit.m_y = floor.m_y + diameter * 0.5f * 0.99f;

	//matrix.m_posit.m_y += xxxx;
matrix.m_posit.m_y += 7.0f;

	for (dInt32 i = 0; i < count; i++)
	{
		ndBodyDynamic* const body = new ndBodyDynamic();
		ndDemoEntity* const entity = new ndDemoEntity(matrix, nullptr);
		entity->SetMesh(sphereMesh, dGetIdentityMatrix());

		body->SetNotifyCallback(new ndDemoEntityNotify(entity));
		body->SetMatrix(matrix);
		body->SetCollisionShape(sphereShape);
		body->SetMassMatrix(mass, sphereShape);
		body->SetGyroMode(true);

		world->AddBody(body);
		scene->AddEntity(entity);

		//matrix.m_posit.m_y += diameter * 0.99f;
		matrix.m_posit.m_y += diameter * 0.99f * 3.0f;
	}
}

static void BuildSphereStacks(ndDemoEntityManager* const scene, const dVector& origin)
{
	dFloat32 diameter = 1.0f;
	//ndShapeInstance sphere(new ndShapeSphere(diameter * 0.5f));
	ndShapeInstance sphere(new ndShapeCapsule(diameter * 0.5f, diameter * 0.5f, diameter * 1.0f));
	ndDemoMesh* const mesh = new ndDemoMesh("sphere", scene->GetShaderCache(), &sphere, "wood_0.tga", "wood_0.tga", "wood_0.tga");

	// get the dimension from shape itself
	//dVector minP(0.0f);
	//dVector maxP(0.0f);
	//sphere.CalculateAABB(dGetIdentityMatrix(), minP, maxP);

	const int n = 10;
	const int stackHigh = 7;
	for (dInt32 i = 0; i < n; i++)
	{
		for (dInt32 j = 0; j < n; j++)
		{
			dVector location((j - n / 2) * 4.0f, 0.0f, (i - n / 2) * 4.0f, 0.0f);
			BuildSphere(scene, mesh, sphere, 10.0f, location + origin, 1.0f, stackHigh, 2.0f);
		}
	}

	mesh->Release();
}

void ndBasicSetup (ndDemoEntityManager* const scene)
{
	// load the sky box
	scene->CreateSkyBox();

	// sync just in case we are on a pending update
	scene->GetWorld()->Sync();

	// build a floor
	BuildFloor(scene);

	dVector origin1(0.0f, 0.0f, 0.0f, 0.0f);
	BuildSphereStacks(scene, origin1);

	dQuaternion rot;
	//dVector origin(-80.0f, 5.0f, 0.0f, 0.0f);
	dVector origin(-40.0f, 5.0f, 0.0f, 0.0f);
	scene->SetCameraMatrix(rot, origin);
}
