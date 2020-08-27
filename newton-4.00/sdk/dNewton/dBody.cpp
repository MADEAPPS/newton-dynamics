/* Copyright (c) <2003-2019> <Julio Jerez, Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 
* 3. This notice may not be removed or altered from any source distribution.
*/

#include "dNewtonStdafx.h"
#include "dBody.h"
#include "dShapeNull.h"

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////


class dDummyCollision: public dShapeNull
{
	public: 
	dDummyCollision()
		:dShapeNull()
	{
		m_refCount.fetch_add(1);
	}

	~dDummyCollision()
	{
		m_refCount.fetch_add(-1);
	}

	static dShapeNull* GetNullShape()
	{
		static dDummyCollision nullShape;
		return &nullShape;
	}
};

dBody::dBody()
	:m_matrix(dGetIdentityMatrix())
	,m_invWorldInertiaMatrix(dGetZeroMatrix())
	,m_veloc(dVector::m_zero)
	,m_omega(dVector::m_zero)
	,m_localCentreOfMass(dVector::m_zero)
	,m_globalCentreOfMass(dVector::m_zero)
	,m_rotation()
	,m_notifyCallback(nullptr)
	,m_shapeInstance(dDummyCollision::GetNullShape())
	,m_newton(nullptr)
	,m_newtonNode(nullptr)
{
}

dBody::~dBody()
{
	if (m_notifyCallback)
	{
		delete m_notifyCallback;
	}
}

void dBody::SetCentreOfMass(const dVector& com)
{
	m_localCentreOfMass.m_x = com.m_x;
	m_localCentreOfMass.m_y = com.m_y;
	m_localCentreOfMass.m_z = com.m_z;
	m_localCentreOfMass.m_w = dFloat32(1.0f);
	m_globalCentreOfMass = m_matrix.TransformVector(m_localCentreOfMass);
}


const dShapeInstance& dBody::GetCollisionShape() const
{
	return m_shapeInstance;
}

void dBody::SetCollisionShape(const dShapeInstance& shapeInstance)
{
	m_shapeInstance = shapeInstance;
}

void dBody::SetNotifyCallback(dBodyNotify* const notify)
{
	//dAssert(notify->m_body == nullptr);
	if (m_notifyCallback)
	{
		delete m_notifyCallback;
	}
	m_notifyCallback = notify;
	m_notifyCallback->m_body = this;
}

dBodyNotify* dBody::GetNotifyCallback(dBodyNotify* const notify) const
{
	return m_notifyCallback;
}

inline dNewton* dBody::GetNewton() const
{
	return m_newton;
}

void dBody::SetNewtonNode(dNewton* const newton, dList<dBody*>::dListNode* const node)
{
	m_newton = newton;
	m_newtonNode = node;
}

dList<dBody*>::dListNode* dBody::GetNewtonNode() const
{
	return m_newtonNode;
}

dVector dBody::GetOmega() const
{
	return m_omega;
}

void dBody::SetOmega(const dVector& veloc)
{
	m_omega = veloc;
}

dVector dBody::GetVelocity() const
{
	return m_veloc;
}

void dBody::SetVelocity(const dVector& veloc)
{
	m_veloc = veloc;
}

dMatrix dBody::GetMatrix() const
{
	return m_matrix;
}

void dBody::SetMatrix(const dMatrix& matrix)
{
	m_matrix = matrix;
	dAssert(m_matrix.TestOrthogonal(dFloat32(1.0e-4f)));

	m_rotation = dQuaternion(m_matrix);
	m_globalCentreOfMass = m_matrix.TransformVector(m_localCentreOfMass);
}
