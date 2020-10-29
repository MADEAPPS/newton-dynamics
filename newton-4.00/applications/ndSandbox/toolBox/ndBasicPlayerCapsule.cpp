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
#include "ndTargaToOpenGl.h"
#include "ndDemoMesh.h"
#include "ndDemoCamera.h"
#include "ndPhysicsUtils.h"
#include "ndPhysicsWorld.h"
#include "ndDemoEntityManager.h"
#include "ndBasicPlayerCapsule.h"

#define PLAYER_WALK_SPEED				8.0f
#define PLAYER_THIRD_PERSON_VIEW_DIST	8.0f
#define PLAYER_JUMP_SPEED				5.0f

ndBasicPlayerCapsule::ndBasicPlayerCapsule(ndDemoEntityManager* const scene, const dMatrix& localAxis, const dMatrix& location,
	dFloat32 mass, dFloat32 radius, dFloat32 height, dFloat32 stepHeight)
	:ndBodyPlayerCapsule(localAxis, mass, radius, height, stepHeight)
	,m_scene(scene)
	,m_isPlayer(false)
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

void ndBasicPlayerCapsule::ApplyInputs(dFloat32 timestep)
{
	//calculate the gravity contribution to the velocity, 
	//use twice the gravity 
	//dVector gravity(m_localFrame.RotateVector(dVector(g, 0.0f, 0.0f, 0.0f)));

	dVector gravity(0.0f, 2.0f * DEMO_GRAVITY, 0.0f, 0.0f);
	dVector totalImpulse(m_impulse + gravity.Scale(m_mass * timestep));
	m_impulse = totalImpulse;

	//dTrace(("  frame: %d    player camera: %f\n", m_scene->GetWorld()->GetFrameIndex(), m_playerInput.m_heading * dRadToDegree));
	if (m_playerInput.m_jump)
	{
		dVector jumpImpule(0.0f, PLAYER_JUMP_SPEED * m_mass, 0.0f, 0.0f);
		m_impulse += jumpImpule;
		m_playerInput.m_jump = false;
	}

	SetForwardSpeed(m_playerInput.m_forwardSpeed);
	SetLateralSpeed(m_playerInput.m_strafeSpeed);
	SetHeadingAngle(m_playerInput.m_heading);
}

dFloat32 ndBasicPlayerCapsule::ContactFrictionCallback(const dVector& position, const dVector& normal, dInt32 contactId, const ndBodyKinematic* const otherbody) const
{
	//return dFloat32(2.0f);
	if (dAbs(normal.m_y) < 0.8f)
	{
		return 0.4f;
	}
	return dFloat32(2.0f);
}

void ndBasicPlayerCapsule::SetCamera()
{
	if (m_isPlayer)
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
		dFloat32 error = AnglesAdd(angle1, -angle0);
		//dFloat32 error = 1.0f;

		if ((dAbs (error) > 1.0e-3f) ||
			m_scene->GetKeyState(' ') ||
			m_scene->GetKeyState('A') ||
			m_scene->GetKeyState('D') ||
			m_scene->GetKeyState('W') ||
			m_scene->GetKeyState('S'))
		{
			SetSleepState(false);
		}

		m_playerInput.m_heading = camera->GetYawAngle();
		m_playerInput.m_forwardSpeed = (dInt32(m_scene->GetKeyState('W')) - dInt32(m_scene->GetKeyState('S'))) * PLAYER_WALK_SPEED;
		m_playerInput.m_strafeSpeed = (dInt32(m_scene->GetKeyState('D')) - dInt32(m_scene->GetKeyState('A'))) * PLAYER_WALK_SPEED;
		m_playerInput.m_jump = m_scene->GetKeyState(' ') && IsOnFloor();

		if (m_playerInput.m_forwardSpeed && m_playerInput.m_strafeSpeed)
		{
			dFloat32 invMag = PLAYER_WALK_SPEED / dSqrt(m_playerInput.m_forwardSpeed * m_playerInput.m_forwardSpeed + m_playerInput.m_strafeSpeed * m_playerInput.m_strafeSpeed);
			m_playerInput.m_forwardSpeed *= invMag;
			m_playerInput.m_strafeSpeed *= invMag;
		}
	}
}

void ndBasicPlayerCapsule::UpdateCameraCallback(ndDemoEntityManager* const manager, void* const context, dFloat32 timestep)
{
	ndBasicPlayerCapsule* const me = (ndBasicPlayerCapsule*)context;
	me->SetCamera();
}
