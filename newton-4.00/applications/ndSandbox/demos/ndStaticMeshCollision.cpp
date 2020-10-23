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
#include "ndDemoMesh.h"
#include "ndDemoCamera.h"
#include "ndPhysicsUtils.h"
#include "ndPhysicsWorld.h"
#include "ndTargaToOpenGl.h"
#include "ndDemoEntityManager.h"
#include "ndBasicPlayerCapsule.h"


/*
static void AddShape(ndDemoEntityManager* const scene,
	ndDemoMesh* const mesh, const ndShapeInstance& shape,
	dFloat32 mass, const dVector& origin, const dFloat32 diameter, int count)
{
	dMatrix matrix(dRollMatrix(90.0f * dDegreeToRad));
	matrix.m_posit = origin;
	matrix.m_posit.m_w = 1.0f;

	ndPhysicsWorld* const world = scene->GetWorld();

	dVector floor(FindFloor(*world, matrix.m_posit + dVector(0.0f, 100.0f, 0.0f, 0.0f), 200.0f));
	matrix.m_posit.m_y = floor.m_y + diameter * 0.5f * 0.99f;

	matrix.m_posit.m_y += 10.0f;

	for (dInt32 i = 0; i < count; i++)
	{
		ndBodyDynamic* const body = new ndBodyDynamic();
		ndDemoEntity* const entity = new ndDemoEntity(matrix, nullptr);
		entity->SetMesh(mesh, dGetIdentityMatrix());

		body->SetNotifyCallback(new ndDemoEntityNotify(scene, entity));
		body->SetMatrix(matrix);
		body->SetCollisionShape(shape);
		body->SetMassMatrix(mass, shape);
		body->SetGyroMode(true);

		world->AddBody(body);
		scene->AddEntity(entity);

		matrix.m_posit.m_y += diameter * 0.99f * 3.0f;
	}
}

static void AddShapes(ndDemoEntityManager* const scene, const dVector& origin)
{
	dFloat32 diameter = 1.0f;
	ndShapeInstance shape(new ndShapeCapsule(diameter * 0.5f, diameter * 0.5f, diameter * 1.0f));
	ndDemoMesh* const mesh = new ndDemoMesh("shape", scene->GetShaderCache(), &shape, "marble.tga", "marble.tga", "marble.tga");

	const int n = 5;
	const int stackHigh = 5;
	//const int n = 10;
	//const int stackHigh = 7;
	for (dInt32 i = 0; i < n; i++)
	{
		for (dInt32 j = 0; j < n; j++)
		{
			dVector location((j - n / 2) * 4.0f, 0.0f, (i - n / 2) * 4.0f, 0.0f);
			AddShape(scene, mesh, shape, 10.0f, location + origin, 1.0f, stackHigh);
		}
	}

	mesh->Release();
}

static void AddPlatform(ndDemoEntityManager* const scene, dFloat32 mass, const dVector& origin)
{
	dMatrix matrix(dGetIdentityMatrix());
	matrix.m_posit = origin;
	matrix.m_posit.m_w = 1.0f;

	ndPhysicsWorld* const world = scene->GetWorld();

	dVector floor(FindFloor(*world, matrix.m_posit + dVector(0.0f, 100.0f, 0.0f, 0.0f), 200.0f));
	matrix.m_posit.m_y += 1.0f;

	ndShapeInstance shape(new ndShapeBox(2.0f, 0.25f, 2.5f));
	ndDemoMesh* const mesh = new ndDemoMesh("shape", scene->GetShaderCache(), &shape, "marble.tga", "marble.tga", "marble.tga");

	ndBodyDynamic* const body = new ndBodyDynamic();
	ndDemoEntity* const entity = new ndDemoEntity(matrix, nullptr);
	entity->SetMesh(mesh, dGetIdentityMatrix());
	body->SetNotifyCallback(new ndDemoEntityNotify(scene, entity));

	body->SetMatrix(matrix);
	body->SetCollisionShape(shape);
	body->SetMassMatrix(mass, shape);
	body->SetGyroMode(true);

	world->AddBody(body);
	scene->AddEntity(entity);

	mesh->Release();
}
*/

static void BuildFloor(ndDemoEntityManager* const scene)
{
	ndPhysicsWorld* const world = scene->GetWorld();
	dVector floor[] =
	{ 
		{ 100.0f, 0.0f,  100.0f, 1.0f},
		{ 100.0f, 0.0f, -100.0f, 1.0f},
		{-100.0f, 0.0f, -100.0f, 1.0f},
		{-100.0f, 0.0f,  100.0f, 1.0f},
	};
	dInt32 index[][3] = { { 0, 1, 2}, { 0, 2, 3 } };
	//dInt32 index[][4] = {{ 0, 1, 2, 3 }};

	dPolygonSoupBuilder meshBuilder;
	meshBuilder.Begin();
	meshBuilder.AddFaceIndirect(&floor[0].m_x, sizeof(dVector), 31, &index[0][0], 3);
	meshBuilder.AddFaceIndirect(&floor[0].m_x, sizeof(dVector), 31, &index[1][0], 3);
	//meshBuilder.AddFace(&floor[0].m_x, sizeof(dVector), 4, 31);
	meshBuilder.End(true);

	ndShapeInstance box(new ndShapeStaticBVH(meshBuilder));
	dMatrix uvMatrix(dGetIdentityMatrix());
	uvMatrix[0][0] *= 0.025f;
	uvMatrix[1][1] *= 0.025f;
	uvMatrix[2][2] *= 0.025f;
	ndDemoMesh* const geometry = new ndDemoMesh("box", scene->GetShaderCache(), &box, "marbleCheckBoard.tga", "marbleCheckBoard.tga", "marbleCheckBoard.tga", 1.0f, uvMatrix);

	dMatrix matrix(dGetIdentityMatrix());
	matrix.m_posit.m_y = -0.5f;
	ndDemoEntity* const entity = new ndDemoEntity(matrix, nullptr);
	entity->SetMesh(geometry, dGetIdentityMatrix());

	ndBodyDynamic* const body = new ndBodyDynamic();
	body->SetNotifyCallback(new ndDemoEntityNotify(scene, entity));
	body->SetMatrix(matrix);
	body->SetCollisionShape(box);

	world->AddBody(body);

	scene->AddEntity(entity);
	geometry->Release();
}
void ndStaticMeshCollisionDemo (ndDemoEntityManager* const scene)
{
	// build a floor
	BuildFloor(scene);

	dMatrix location(dGetIdentityMatrix());
	location.m_posit.m_y += 2.0f;

	dMatrix localAxis(dGetIdentityMatrix());
	localAxis[0] = dVector(0.0, 1.0f, 0.0f, 0.0f);
	localAxis[1] = dVector(1.0, 0.0f, 0.0f, 0.0f);
	localAxis[2] = localAxis[0].CrossProduct(localAxis[1]);

	dFloat32 height = 1.9f;
	dFloat32 radio = 0.5f;
	dFloat32 mass = 100.0f;
//	ndBasicPlayerCapsule* const player = new ndBasicPlayerCapsule(
//		scene, localAxis, location, mass, radio, height, height/4.0f);
//
//	player->m_isPlayer = true;
//	scene->SetUpdateCameraFunction(ndBasicPlayerCapsule::UpdateCameraCallback, player);

	//AddShapes(scene, dVector (22.0f, 0.0f, 0.0f, 0.0f));
	//AddPlatform(scene, 30.0f, dVector(10.0f, 0.0f, 0.0f, 0.0f));
	//AddPlatform(scene, 30.0f, dVector(10.0f, 0.5f, 1.125f, 0.0f));
	//AddPlatform(scene, 30.0f, dVector(10.0f, 1.0f, 1.250f, 0.0f));

	//location.m_posit.m_z += 2.0f;
	//new ndBasicPlayerCapsule(scene, localAxis, location, mass, radio, height, height / 4.0f);
	//
	//location.m_posit.m_z += 2.0f;
	//new ndBasicPlayerCapsule(scene, localAxis, location, mass, radio, height, height / 4.0f);

	dQuaternion rot;
	dVector origin(-10.0f, 5.0f, 0.0f, 0.0f);
	scene->SetCameraMatrix(rot, origin);
}
