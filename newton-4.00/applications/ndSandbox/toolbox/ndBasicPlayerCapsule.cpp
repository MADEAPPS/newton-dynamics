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

#define PLAYER_WALK_SPEED				8.0f
#define PLAYER_JUMP_SPEED				5.0f

//#define PLAYER_FIRST_PERSON	

#ifdef PLAYER_FIRST_PERSON	
	#define PLAYER_THIRD_PERSON_VIEW_DIST	0.0f
#else
	//#define PLAYER_THIRD_PERSON_VIEW_DIST	6.0f
	#define PLAYER_THIRD_PERSON_VIEW_DIST	5.0f
#endif

class ndBasicPlayerCapsuleNotify : public ndDemoEntityNotify
{
	public:
	ndBasicPlayerCapsuleNotify(
		ndDemoEntityManager* const manager, 
		const ndSharedPtr<ndRenderSceneNode>& entity)
		:ndDemoEntityNotify(manager, entity)
		//,m_meshOrigin(entity->GetRenderMatrix().m_posit)
		//,m_localRotation(entity->GetRenderMatrix())
		//,m_veloc(ndVector::m_zero)
	{
		ndAssert(0);
	}

	void OnTransform(ndInt32, const ndMatrix& matrix)
	{
		ndAssert(0);
		//const ndBody* const body = GetBody();
		//const ndQuaternion rot(body->GetRotation());
		//ndWorld* const word = m_manager->GetWorld();
		//
		//m_entity->SetMatrix(m_localRotation * rot, matrix.TransformVector(m_meshOrigin));
		//ndBasicPlayerCapsule* const player = (ndBasicPlayerCapsule*)GetBody();
		//
		//ndFloat32 timestep = word->GetScene()->GetTimestep();
		//player->m_animBlendTree->Update(timestep);
		//player->m_animBlendTree->Evaluate(player->m_output, m_veloc);
		
		//for (ndInt32 i = 0; i < player->m_output.GetCount(); ++i)
		//{
		//	const ndAnimKeyframe& keyFrame = player->m_output[i];
		//	ndDemoEntity* const entity = (ndDemoEntity*)keyFrame.m_userData;
		//	if (entity)
		//	{
		//		entity->SetMatrix(keyFrame.m_rotation, keyFrame.m_posit);
		//	}
		//}
	}
	
	ndQuaternion m_meshOrigin;
	ndQuaternion m_localRotation;
	ndVector m_veloc;
};

ndBasicPlayerCapsule::ndBasicPlayerCapsule()
	:ndBodyPlayerCapsule()
	,m_isPlayer(false)
{
}

class ndAnimationBlendTansition: public ndAnimationTwoWayBlend
{
	public:
	ndAnimationBlendTansition(ndAnimationBlendTreeNode* const node0, ndAnimationBlendTreeNode* const node1)
		:ndAnimationTwoWayBlend(node0, node1)
		,m_paramMemory(0.0f)
	{
	}

	void SetTransition(ndFloat32 param)
	{
		m_paramMemory = m_paramMemory + 0.15f * (param - m_paramMemory);
		SetParam(m_paramMemory);
	}

	ndFloat32 m_paramMemory;
};

ndBasicPlayerCapsule::ndBasicPlayerCapsule(
	ndDemoEntityManager* const scene, 
	ndMeshLoader& loader, ndSharedPtr<ndRenderSceneNode>& entity, 
	const ndMatrix& localAxis, const ndMatrix& location,
	ndFloat32 mass, ndFloat32 radius, ndFloat32 height, ndFloat32 stepHeight, bool isPlayer)
	:ndBodyPlayerCapsule(localAxis, mass, radius, height, stepHeight)
	,m_isPlayer(isPlayer)
	,m_output()
	,m_idleWalkBlend(nullptr)
	,m_animBlendTree(nullptr)
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

