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

#ifndef _N_RIGIB_BODY_H_
#define _N_RIGIB_BODY_H_

#include "nMatrix.h"
#include "nBodyNotify.h"
#include "nShapeInstance.h"
#include "ndBodyKinematic.h"
#include "ndBodyKinematicBase.h"
#include "ndBodyTriggerVolume.h"
#include "ndBodyPlayerCapsule.h"


class nRigidBody
{
	public:
	enum Type
	{
		m_dynamic,
		m_triggerVolume,
		m_playerCapsule,
	};

	nRigidBody(Type type)
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

	virtual ~nRigidBody()
	{
		if (m_body)
		{
			delete m_body;
		}
	}

	int GetId() const
	{
		return m_body->GetId();
	}

	virtual void SetMatrix(const nMatrix* const matrix)
	{
		m_body->SetMatrix(*matrix);
	}

	virtual void SetCollisionShape(const nShapeInstance* const shapeInstance)
	{
		m_body->SetCollisionShape(*shapeInstance);
	}

	virtual void SetNotifyCallback(nBodyNotify* const notify)
	{
		m_body->SetNotifyCallback(notify);
	}

	private:
	ndBodyKinematic* m_body;
	friend class nWorld;
};

#endif 

