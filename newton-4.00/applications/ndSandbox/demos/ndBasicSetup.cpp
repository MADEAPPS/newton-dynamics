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
	ndBodyDynamic* const body = new ndBodyDynamic();

	dMatrix matrix(dGetIdentityMatrix());
	matrix.m_posit.m_y = -0.5f;

	ndDemoEntity* const entity = new ndDemoEntity(matrix, nullptr);

	body->SetNotifyCallback(new ndDemoEntityNotify(entity));
	body->SetMatrix(matrix);
	body->SetCollisionShape(box);
	world->AddBody(body);

	scene->AddEntity(entity);
}

void ndBasicSetup (ndDemoEntityManager* const scene)
{
	// load the skybox
	scene->CreateSkyBox();

	// sync just in case we are on a pending update
	scene->GetWorld()->Sync();

	// build a floor
	BuildFloor(scene);

	dQuaternion rot;
	dVector origin(-10.0f, 5.0f, 0.0f, 0.0f);
	scene->SetCameraMatrix(rot, origin);
}
