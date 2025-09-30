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
#include "ndMeshLoader.h"
#include "ndPhysicsUtils.h"
#include "ndPhysicsWorld.h"
#include "ndDemoEntityNotify.h"
#include "ndDemoEntityManager.h"
#include "ndBasicPlayerCapsule.h"

//#define PLAYER_WALK_SPEED				8.0f
#define PLAYER_JUMP_SPEED				5.0f
//
////#define PLAYER_FIRST_PERSON	
//#ifdef PLAYER_FIRST_PERSON	
//	#define PLAYER_THIRD_PERSON_VIEW_DIST	0.0f
//#else
//	//#define PLAYER_THIRD_PERSON_VIEW_DIST	6.0f
//	#define PLAYER_THIRD_PERSON_VIEW_DIST	5.0f
//#endif

ndBasicPlayerCapsule::ndBasicPlayerCapsule()
	:ndBodyPlayerCapsule()
{
}

ndBasicPlayerCapsule::ndBasicPlayerCapsule(
	ndDemoEntityManager* const scene, 
	ndSharedPtr<ndRenderSceneNode>& entity, 
	const ndMatrix& localAxis, const ndMatrix& location,
	ndFloat32 mass, ndFloat32 radius, ndFloat32 height, ndFloat32 stepHeight)
	:ndBodyPlayerCapsule(localAxis, mass, radius, height, stepHeight)
{
	ndMatrix matrix(location);
	ndPhysicsWorld* const world = scene->GetWorld();
	ndVector floor(FindFloor(*world, matrix.m_posit + ndVector(0.0f, 100.0f, 0.0f, 0.0f), 200.0f));
	matrix.m_posit.m_y = floor.m_y;

	SetMatrix(matrix);
	SetNotifyCallback(new ndDemoEntityNotify(scene, entity));
}

ndBasicPlayerCapsule::~ndBasicPlayerCapsule()
{
}

void ndBasicPlayerCapsule::ApplyInputs(ndFloat32 timestep)
{
	//calculate the gravity contribution to the velocity, 
	const ndVector gravity(GetNotifyCallback()->GetGravity());
	const ndVector totalImpulse(m_impulse + gravity.Scale(m_mass * timestep));
	m_impulse = totalImpulse;

	//dTrace(("  frame: %d    player camera: %f\n", m_scene->GetWorld()->GetFrameIndex(), m_playerInput.m_heading * dRadToDegree));
	if (m_playerInput.m_jump)
	{
		ndVector jumpImpule(0.0f, PLAYER_JUMP_SPEED * m_mass, 0.0f, 0.0f);
		m_impulse += jumpImpule;
		m_playerInput.m_jump = false;
	}

	SetForwardSpeed(m_playerInput.m_forwardSpeed);
	SetLateralSpeed(m_playerInput.m_strafeSpeed);
	SetHeadingAngle(m_playerInput.m_heading);
}

//ndFloat32 ndBasicPlayerCapsule::ContactFrictionCallback(const ndVector& position, const ndVector& normal, ndInt32 contactId, const ndBodyKinematic* const otherbody) const
ndFloat32 ndBasicPlayerCapsule::ContactFrictionCallback(const ndVector&, const ndVector& normal, ndInt32, const ndBodyKinematic* const) const
{
	//return ndFloat32(2.0f);
	if (ndAbs(normal.m_y) < 0.8f)
	{
		return 0.4f;
	}
	return ndFloat32(2.0f);
}

