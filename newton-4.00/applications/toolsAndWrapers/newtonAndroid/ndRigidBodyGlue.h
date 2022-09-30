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
		:m_body(nullptr)
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
	}

	virtual ~ndRigidBodyGlue()
	{
		//if (m_body)
		//{
		//	delete m_body;
		//}
	}

	int GetId() const
	{
		return m_body->GetId();
	}

	virtual void SetMatrix(const ndMatrixGlue* const matrix)
	{
		m_body->SetMatrix(*matrix);
	}

	virtual ndShapeInstanceGlue* GetCollisionShape() const
	{
		return (ndShapeInstanceGlue*)(&m_body->GetCollisionShape());
	}

	virtual void SetCollisionShape(const ndShapeInstanceGlue* const shapeInstance)
	{
		m_body->SetCollisionShape(*shapeInstance);
	}

	virtual void SetMassMatrix(float mass, const ndShapeInstanceGlue* const shapeInstance)
	{
		m_body->SetMassMatrix(mass, *shapeInstance);
	}

	virtual void SetNotifyCallback(ndBodyNotifyGlue* const notify)
	{
		m_body->SetNotifyCallback(notify);
	}

	private:
	ndBodyKinematic* m_body;
	friend class ndWorldGlue;
};

#endif 

