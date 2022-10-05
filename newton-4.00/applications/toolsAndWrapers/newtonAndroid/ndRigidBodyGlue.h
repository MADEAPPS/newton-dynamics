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

#ifndef _ND_RIGIB_BODY_GLUE_H_
#define _ND_RIGIB_BODY_GLUE_H_

#include "ndMatrixGlue.h"
#include "ndBodyKinematic.h"
#include "ndBodyNotifyGlue.h"
#include "ndBodyKinematicBase.h"
#include "ndBodyTriggerVolume.h"
#include "ndBodyPlayerCapsule.h"
#include "ndShapeInstanceGlue.h"

enum nRigidBodyType
{
	m_dynamic,
	m_triggerVolume,
	m_playerCapsule,
};

class ndRigidBodyGlue
{
	public:
	ndRigidBodyGlue(nRigidBodyType type)
		:m_body()
		,m_shapeInstance(nullptr)
	{
		switch (type)
		{
			case m_dynamic:
			{
				m_body = ndSharedPtr<ndBodyKinematic>(new ndBodyDynamic());
				break;
			}

			case m_triggerVolume:
			{
				m_body = ndSharedPtr<ndBodyKinematic>(new ndBodyTriggerVolume());
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

	~ndRigidBodyGlue()
	{
		delete m_shapeInstance;
	}

	int GetId()
	{
		return m_body->GetId();
	}

	void SetMatrix(const ndMatrixGlue* const matrix)
	{
		m_body->SetMatrix(*matrix);
	}

	void SetMassMatrix(float mass, const ndShapeInstanceGlue* const shapeInstance)
	{
		const ndShapeInstance& instance = *shapeInstance->m_shapeInstance;
		m_body->SetMassMatrix(mass, instance);
	}

	void SetNotifyCallback(ndBodyNotifyGlue* const notify)
	{
		m_body->SetNotifyCallback(notify);
	}

	const ndShapeInstanceGlue* GetCollisionShape() const
	{
		return m_shapeInstance;
	}

	void SetCollisionShape(const ndShapeInstanceGlue* const shapeInstance)
	{
		delete m_shapeInstance;
		m_body->SetCollisionShape(*shapeInstance->m_shapeInstance);
		m_shapeInstance = new ndShapeInstanceGlue(&m_body->GetCollisionShape());
	}

	private:
	ndSharedPtr<ndBodyKinematic> m_body;
	ndShapeInstanceGlue* m_shapeInstance;
	friend class ndWorldGlue;
};

#endif 

