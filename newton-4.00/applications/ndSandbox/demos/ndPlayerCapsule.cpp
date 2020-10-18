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

#define PLAYER_WALK_SPEED				8.0f
#define PLAYER_THIRD_PERSON_VIEW_DIST	8.0f
#define PLAYER_JUMP_SPEED				5.0f

class ndBasicPlayer: public ndBodyPlayerCapsule
{
	public:
	ndBasicPlayer(ndDemoEntityManager* const scene,
		const dMatrix& localAxis, const dMatrix& location,
		dFloat32 mass, dFloat32 radius, dFloat32 height, dFloat32 stepHeight)
		:ndBodyPlayerCapsule(localAxis, mass, radius, height, stepHeight)
		,m_scene(scene)
	{
		SetMatrix(location);
		ndDemoEntity* const entity = new ndDemoEntity(location, nullptr);

		const ndShapeInstance& shape = GetCollisionShape();
		ndDemoMesh* const mesh = new ndDemoMesh("shape", scene->GetShaderCache(), &shape, "smilli.tga", "marble.tga", "marble.tga");
		entity->SetMesh(mesh, dGetIdentityMatrix());
		mesh->Release();
		
		SetNotifyCallback(new ndDemoEntityNotify(scene, entity));
		
		ndPhysicsWorld* const world = scene->GetWorld();
		world->AddBody(this);
		scene->AddEntity(entity);
	}

	void ApplyInputs(dFloat32 timestep)
	{
		//calculate the gravity contribution to the velocity, 
		//use twice the gravity 
		//dVector gravity(m_localFrame.RotateVector(dVector(g, 0.0f, 0.0f, 0.0f)));

		dVector gravity(0.0f, 2.0f * DEMO_GRAVITY, 0.0f, 0.0f);
		dVector totalImpulse(m_impulse + gravity.Scale(m_mass * timestep));
		m_impulse = totalImpulse;

		dFloat32 forwarSpeed = (dInt32(m_scene->GetKeyState('W')) - dInt32(m_scene->GetKeyState('S'))) * PLAYER_WALK_SPEED;
		dFloat32 strafeSpeed = (dInt32(m_scene->GetKeyState('D')) - dInt32(m_scene->GetKeyState('A'))) * PLAYER_WALK_SPEED;

		//bool crowchKey = scene->GetKeyState('C') ? true : false;
		//if (m_crowchKey.UpdateTrigger(crowchKey))
		//{
		//	controller->ToggleCrouch();
		//	DemoEntity* const playerEntity = (DemoEntity*)NewtonBodyGetUserData(controller->GetBody());
		//	if (controller->IsCrouched()) {
		//		playerEntity->SetMesh(m_crouchMesh, dGetIdentityMatrix());
		//	}
		//	else {
		//		playerEntity->SetMesh(m_standingMesh, dGetIdentityMatrix());
		//	}
		//}

		if (m_scene->GetKeyState(' ') && IsOnFloor()) 
		{
			//dVector jumpImpule(controller->GetLocalFrame().RotateVector(dVector(PLAYER_JUMP_SPEED * controller->GetMass(), 0.0f, 0.0f, 0.0f)));
			dVector jumpImpule(0.0f, PLAYER_JUMP_SPEED * m_mass, 0.0f, 0.0f);
			m_impulse += jumpImpule;
		}

		//if (forwarSpeed && strafeSpeed) {
		//	dFloat32 invMag = PLAYER_WALK_SPEED / dSqrt(forwarSpeed * forwarSpeed + strafeSpeed * strafeSpeed);
		//	forwarSpeed *= invMag;
		//	strafeSpeed *= invMag;
		//}
		//
		ndDemoCamera* const camera = m_scene->GetCamera();

		SetForwardSpeed(forwarSpeed);
		SetLateralSpeed(strafeSpeed);
		SetHeadingAngle(camera->GetYawAngle());
	}

	dFloat32 ContactFrictionCallback(const dVector& position, const dVector& normal, dInt32 contactId, const ndBodyKinematic* const otherbody) const
	{
		return dFloat32(2.0f);
	}

	void SetCamera()
	{
		ndDemoCamera* const camera = m_scene->GetCamera();
		dMatrix camMatrix(camera->GetNextMatrix());

		ndDemoEntityNotify* const notify = (ndDemoEntityNotify*)GetNotifyCallback();
		ndDemoEntity* const player = (ndDemoEntity*)notify->GetUserData();
		dMatrix playerMatrix(player->GetNextMatrix());
		
		dFloat32 height = 2.0f;
		dVector frontDir(camMatrix[0]);
		dVector upDir(0.0f, 1.0f, 0.0f, 0.0f);
		dVector camOrigin = playerMatrix.TransformVector(upDir.Scale(height));
		camOrigin -= frontDir.Scale(PLAYER_THIRD_PERSON_VIEW_DIST);
		
		camera->SetNextMatrix(*m_scene, camMatrix, camOrigin);

		dFloat32 angle0 = camera->GetYawAngle();
		dFloat32 angle1 = GetHeadingAngle();
		dFloat32 error(angle1 - angle0);

		if ((dAbs (error) > 1.0e-3f) ||
			m_scene->GetKeyState(' ') ||
			m_scene->GetKeyState('A') ||
			m_scene->GetKeyState('D') ||
			m_scene->GetKeyState('W') ||
			m_scene->GetKeyState('S'))
		{
			SetSleepState(false);
		}
	}

	static void UpdateCameraCallback(ndDemoEntityManager* const manager, void* const context, dFloat32 timestep)
	{
		ndBasicPlayer* const me = (ndBasicPlayer*)context;
		me->SetCamera();
	}

	ndDemoEntityManager* m_scene;
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


static void AddShape(ndDemoEntityManager* const scene,
	ndDemoMesh* const sphereMesh, const ndShapeInstance& sphereShape,
	dFloat32 mass, const dVector& origin, const dFloat32 diameter, int count, dFloat32 xxxx)
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
		entity->SetMesh(sphereMesh, dGetIdentityMatrix());

		body->SetNotifyCallback(new ndDemoEntityNotify(scene, entity));
		body->SetMatrix(matrix);
		body->SetCollisionShape(sphereShape);
		body->SetMassMatrix(mass, sphereShape);
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
			AddShape(scene, mesh, shape, 10.0f, location + origin, 1.0f, stackHigh, 2.0f);
		}
	}

	mesh->Release();
}

void ndPlayerCapsuleDemo (ndDemoEntityManager* const scene)
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
	ndBasicPlayer* const player = new ndBasicPlayer(
		scene, localAxis, location, mass, radio, height, height/4.0f);

	scene->SetUpdateCameraFunction(ndBasicPlayer::UpdateCameraCallback, player);

	dVector origin1(14.0f, 0.0f, 0.0f, 0.0f);
	AddShapes(scene, origin1);

	dQuaternion rot;
	dVector origin(-10.0f, 5.0f, 0.0f, 0.0f);
	scene->SetCameraMatrix(rot, origin);
}
