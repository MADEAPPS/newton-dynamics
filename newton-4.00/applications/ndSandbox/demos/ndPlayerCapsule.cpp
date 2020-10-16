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

class ndBasicPlayer: public ndBodyPlayerCapsule
{
	public:
	ndBasicPlayer(const dMatrix& localAxis, dFloat32 mass, dFloat32 radius, dFloat32 height, dFloat32 stepHeight)
		//:ndBodyPlayerCapsule(localAxis, mass, radius, height, stepHeight)
		:ndBodyPlayerCapsule()
	{
		dAssert(0);
	}

	void ApplyInputs(dFloat32 timestep)
	{
		//// calculate the gravity contribution to the velocity, 
		//// use twice the gravity 
		//dFloat32 g = 2.0f * DEMO_GRAVITY;
		////dVector gravity(m_localFrame.RotateVector(dVector(g, 0.0f, 0.0f, 0.0f)));
		//dVector gravity(0.0f, g, 0.0f, 0.0f);
		//dVector totalImpulse(m_impulse + gravity.Scale(m_mass * timestep));
		//m_impulse = totalImpulse;
	}

};

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

static ndBasicPlayer* AddPlayer(ndDemoEntityManager* const scene, const dMatrix& location, dFloat32 mass, dFloat32 radius, dFloat32 height)
{
	// set the play coordinate system
	dMatrix localAxis(dGetIdentityMatrix());

	//up is first vector
	localAxis[0] = dVector(0.0, 1.0f, 0.0f, 0.0f);
	// up is the second vector
	localAxis[1] = dVector(1.0, 0.0f, 0.0f, 0.0f);
	// size if the cross product
	localAxis[2] = localAxis[0].CrossProduct(localAxis[1]);

	dMatrix matrix(dGetIdentityMatrix());
	matrix.m_posit.m_y = 2.0f;

	ndBasicPlayer* const player = new ndBasicPlayer(localAxis, mass, radius, height, height / 3.0f);
	player->SetMatrix(matrix);

	ndDemoEntity* const entity = new ndDemoEntity(player->GetMatrix(), nullptr);

	const ndShapeInstance& shape = player->GetCollisionShape();
	ndDemoMesh* const mesh = new ndDemoMesh("shape", scene->GetShaderCache(), &shape, "marble.tga", "marble.tga", "marble.tga");
	entity->SetMesh(mesh, dGetIdentityMatrix());
	mesh->Release();

	player->SetNotifyCallback(new ndDemoEntityNotify(scene, entity));

	ndPhysicsWorld* const world = scene->GetWorld();
	world->AddBody(player);
	scene->AddEntity(entity);
	return player;
}

void ndPlayerCapsuleDemo (ndDemoEntityManager* const scene)
{
	// build a floor
	BuildFloor(scene);

	dVector origin1(0.0f, 0.0f, 0.0f, 0.0f);
	dMatrix location(dGetIdentityMatrix());
	ndBasicPlayer* const player = AddPlayer(scene, location, 100.0f, 0.5f, 1.9f);
	player;

	dQuaternion rot;
	//dVector origin(-80.0f, 5.0f, 0.0f, 0.0f);
	dVector origin(-10.0f, 5.0f, 0.0f, 0.0f);
	scene->SetCameraMatrix(rot, origin);
}
