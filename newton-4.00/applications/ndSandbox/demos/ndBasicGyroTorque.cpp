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

static void DzhanibekovEffect(ndDemoEntityManager* const scene, dFloat32 mass, dFloat32 angularSpeed, const dVector& origin)
{
	dMatrix matrix(dGetIdentityMatrix());
	matrix.m_posit = origin;
	matrix.m_posit.m_w = 1.0f;

	ndPhysicsWorld* const world = scene->GetWorld();

	dVector floor(FindFloor(*world, matrix.m_posit + dVector(0.0f, 100.0f, 0.0f, 0.0f), 200.0f));
	matrix.m_posit.m_y += 10.0f;

	ndShapeInstance shape(new ndShapeBox(1.0f, 0.25f, 0.5));
	ndDemoMesh* const mesh = new ndDemoMesh("shape", scene->GetShaderCache(), &shape, "marble.tga", "marble.tga", "marble.tga");

	dVector omega(0.1f, 0.0f, angularSpeed, 0.0f);
	ndBodyDynamic* const body = new ndBodyDynamic();
	ndDemoEntity* const entity = new ndDemoEntity(matrix, nullptr);
	entity->SetMesh(mesh, dGetIdentityMatrix());
	body->SetNotifyCallback(new ndDemoEntityNotify(scene, entity, 0.0f));

	body->SetOmega(omega);
	body->SetMatrix(matrix);
	body->SetCollisionShape(shape);
	body->SetMassMatrix(mass, shape);
	body->SetGyroMode(true);

	world->AddBody(body);
	scene->AddEntity(entity);

	mesh->Release();
}

static void Phitop(ndDemoEntityManager* const scene, dFloat32 mass, dFloat32 angularSpeed, const dVector& origin)
{
	//dMatrix matrix(dGetIdentityMatrix());
	dMatrix matrix(dPitchMatrix(15.0f * dDegreeToRad));
	matrix.m_posit = origin;
	matrix.m_posit.m_w = 1.0f;

	ndPhysicsWorld* const world = scene->GetWorld();

	dVector floor(FindFloor(*world, matrix.m_posit + dVector(0.0f, 100.0f, 0.0f, 0.0f), 200.0f));
	matrix.m_posit.m_y += 0.5f;

	ndShapeInstance shape(new ndShapeSphere(1.0f));
	shape.SetScale(dVector (0.5f, 0.5f, 1.0f, 0.0f));

	ndDemoMesh* const mesh = new ndDemoMesh("shape", scene->GetShaderCache(), &shape, "marble.tga", "marble.tga", "marble.tga");

	//dVector omega(0.1f, 0.0f, angularSpeed, 0.0f);
	dVector omega(0.0f, angularSpeed, 0.0f, 0.0f);
	ndBodyDynamic* const body = new ndBodyDynamic();
	ndDemoEntity* const entity = new ndDemoEntity(matrix, nullptr);
	entity->SetMesh(mesh, dGetIdentityMatrix());
	body->SetNotifyCallback(new ndDemoEntityNotify(scene, entity));
	//body->SetNotifyCallback(new ndDemoEntityNotify(scene, entity, 0.0f));

	body->SetOmega(omega);
	body->SetMatrix(matrix);
	body->SetCollisionShape(shape);
	body->SetMassMatrix(mass, shape);
	body->SetGyroMode(true);

	world->AddBody(body);
	scene->AddEntity(entity);

	mesh->Release();
}


void ndBasicAngularMomentum (ndDemoEntityManager* const scene)
{
	// build a floor
	BuildFloor(scene);

	DzhanibekovEffect(scene, 10.0f,  5.0f, dVector(0.0f, 0.0f, -2.0f, 0.0f));
	DzhanibekovEffect(scene, 10.0f, -5.0f, dVector(0.0f, 0.0f, 0.0f, 0.0f));
	DzhanibekovEffect(scene, 10.0f, 10.0f, dVector(0.0f, 0.0f, 2.0f, 0.0f));

	Phitop(scene, 10.0f, 25.0f, dVector(0.0f, 0.0f, -4.0f, 0.0f));
	Phitop(scene, 10.0f, -25.0f, dVector(0.0f, 0.0f, 0.0f, 0.0f));
	Phitop(scene, 10.0f, 35.0f, dVector(0.0f, 0.0f,  4.0f, 0.0f));

	dQuaternion rot;
	dVector origin(-25.0f, 5.0f, 0.0f, 0.0f);
	scene->SetCameraMatrix(rot, origin);
}
