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

static void BuildBallSocket(ndDemoEntityManager* const scene, const dVector& origin)
{
	dFloat32 mass = 1.0f;
	dFloat32 diameter = 0.5f;
	//ndShapeInstance capsule(new ndShapeSphere(diameter * 0.5f));
	ndShapeInstance capsule(new ndShapeCapsule(diameter * 0.5f, diameter * 0.5f, diameter * 1.0f));
	ndDemoMesh* const mesh = new ndDemoMesh("capsule", scene->GetShaderCache(), &capsule, "marble.tga", "marble.tga", "marble.tga");

	//dMatrix matrix(dGetIdentityMatrix());
	dMatrix matrix(dRollMatrix(90.0f * dDegreeToRad));
	//dMatrix matrix(dYawMatrix(90.0f * dDegreeToRad) * dPitchMatrix(-45.0f * dDegreeToRad));
	matrix.m_posit = origin;
	matrix.m_posit.m_w = 1.0f;

	ndPhysicsWorld* const world = scene->GetWorld();

	dVector floor(FindFloor(*world, matrix.m_posit + dVector(0.0f, 100.0f, 0.0f, 0.0f), 200.0f));
	matrix.m_posit.m_y = floor.m_y;

	//matrix.m_posit.m_y += xxxx;
	matrix.m_posit.m_y += 5.0f;
	ndDemoEntity* const entity0 = new ndDemoEntity(matrix, nullptr);
	entity0->SetMesh(mesh, dGetIdentityMatrix());
	ndBodyDynamic* const body0 = new ndBodyDynamic();
	body0->SetNotifyCallback(new ndDemoEntityNotify(entity0));
	body0->SetMatrix(matrix);
	body0->SetCollisionShape(capsule);
	body0->SetMassMatrix(mass, capsule);
	world->AddBody(body0);
	scene->AddEntity(entity0);

	matrix.m_posit.m_y += 0.5f;
	ndDemoEntity* const entity1 = new ndDemoEntity(matrix, nullptr);
	entity1->SetMesh(mesh, dGetIdentityMatrix());
	ndBodyDynamic* const body1 = new ndBodyDynamic();
	body1->SetNotifyCallback(new ndDemoEntityNotify(entity1));
	body1->SetMatrix(matrix);
	body1->SetCollisionShape(capsule);
	body1->SetMassMatrix(mass, capsule);
	world->AddBody(body1);
	scene->AddEntity(entity1);

	dMatrix bodyMatrix0(body0->GetMatrix());
	dMatrix bodyMatrix1(body1->GetMatrix());
	dMatrix pinMatrix(bodyMatrix0);
	pinMatrix.m_posit = (bodyMatrix0.m_posit + bodyMatrix1.m_posit).Scale(0.5f);
	ndJointBallAndSocket* const joint = new ndJointBallAndSocket(pinMatrix, body0, body1);
	world->AddJoint(joint);

	//ndBodyDynamic* const fixBody = world->GetSentinelBody();
	//pinMatrix.m_posit.m_y += diameter * 2.0f;

	mesh->Release();
}


void ndBasicJoints (ndDemoEntityManager* const scene)
{
	// load the sky box
	scene->CreateSkyBox();

	// sync just in case we are on a pending update
	scene->GetWorld()->Sync();

	// build a floor
	BuildFloor(scene);

	BuildBallSocket(scene, dVector(0.0f, 0.0f, 0.0f, 0.0f));

	dQuaternion rot;
	//dVector origin(-80.0f, 5.0f, 0.0f, 0.0f);
	dVector origin(-20.0f, 5.0f, 0.0f, 0.0f);
	scene->SetCameraMatrix(rot, origin);
}
