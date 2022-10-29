/* Copyright (c) <2003-2021> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/

#include "ndNewton.h"
#include "ndWorldGlue.h"
#include "ndMatrixGlue.h"
#include "ndRigidBodyGlue.h"
#include "ndBodyNotifyGlue.h"
#include "ndShapeInstanceGlue.h"

ndRigidBodyGlue::ndRigidBodyGlue(nRigidBodyType type)
	:ndContainersFreeListAlloc<ndRigidBodyGlue>()
	,m_body()
	,m_shapeInstance(nullptr)
{
	switch (type)
	{
		case m_dynamic:
		{
			m_body = new ndBodyDynamic();
			break;
		}

		case m_triggerVolume:
		{
			m_body = new ndBodyTriggerVolume();
			break;
		}

		case m_playerCapsule:
		{
			ndAssert(0);
			break;
		}

		default:
		{
			ndAssert(0);
		}
	}

	m_shapeInstance = new ndShapeInstanceGlue(&m_body->GetCollisionShape());
}

ndRigidBodyGlue::~ndRigidBodyGlue()
{
	ndBodyNotifyGlue* const notification = GetNotifyCallback();
	if (notification && notification->m_world)
	{
		notification->m_world->RemoveBody(this);
	}

	delete m_body;
	delete m_shapeInstance;
}

int ndRigidBodyGlue::GetId() const
{
	return m_body->GetId();
}

void ndRigidBodyGlue::SetMatrix(const ndMatrixGlue* const matrix)
{
	m_body->SetMatrix(*matrix);
}

void ndRigidBodyGlue::SetMassMatrix(float mass, const ndShapeInstanceGlue* const shapeInstance)
{
	const ndShapeInstance& instance = *shapeInstance->m_shapeInstance;
	m_body->SetMassMatrix(mass, instance);
}

void ndRigidBodyGlue::SetNotifyCallback(ndBodyNotifyGlue* const notify)
{
	ndAssert(notify->m_notify);
	m_body->SetNotifyCallback((ndBodyNotify*)notify->m_notify);
}

const ndShapeInstanceGlue* ndRigidBodyGlue::GetCollisionShape() const
{
	return m_shapeInstance;
}

void ndRigidBodyGlue::SetCollisionShape(const ndShapeInstanceGlue* const shapeInstance)
{
	delete m_shapeInstance;
	m_body->SetCollisionShape(*shapeInstance->m_shapeInstance);
	m_shapeInstance = new ndShapeInstanceGlue(&m_body->GetCollisionShape());
}

ndBodyNotifyGlue* ndRigidBodyGlue::GetNotifyCallback() const
{
	ndAssert(m_body->GetNotifyCallback());
	return (ndBodyNotifyGlue*)m_body->GetNotifyCallback()->GetUserData();
}


