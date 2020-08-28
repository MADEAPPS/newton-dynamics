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


dUnsigned32 dBody::m_uniqueIDCount = 0;

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
	,m_minAABB(dVector::m_zero)
	,m_maxAABB(dVector::m_zero)
	,m_rotation()
	,m_notifyCallback(nullptr)
	,m_shapeInstance(dDummyCollision::GetNullShape())
	,m_newton(nullptr)
	,m_broadPhaseNode(nullptr)
	,m_newtonNode(nullptr)
	,m_broadPhaseAggregateNode(nullptr)
	,m_flags(0)
	,m_uniqueID(m_uniqueIDCount)
{
	m_equilibrium = 0;
	m_uniqueIDCount++;
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

dInt32 dBody::GetId() const
{
	return m_uniqueID;
}

dNewton* dBody::GetNewton() const
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

dBroadPhaseBodyNode* dBody::GetBroadPhaseNode() const
{
	return m_broadPhaseNode;
}

void dBody::SetBroadPhaseNode(dBroadPhaseBodyNode* const node)
{
	m_broadPhaseNode = node;
}

dBroadPhaseAggregate* dBody::GetBroadPhaseAggregate() const
{
	return m_broadPhaseAggregateNode;
}

void dBody::SetBroadPhaseAggregate(dBroadPhaseAggregate* const node)
{
	m_broadPhaseAggregateNode = node;
}

dVector dBody::GetOmega() const
{
	return m_omega;
}

void dBody::SetOmega(const dVector& veloc)
{
	m_equilibrium = 0;
	m_omega = veloc;
}

dVector dBody::GetVelocity() const
{
	return m_veloc;
}

void dBody::SetVelocity(const dVector& veloc)
{
	m_equilibrium = 0;
	m_veloc = veloc;
}

dMatrix dBody::GetMatrix() const
{
	return m_matrix;
}

void dBody::UpdateCollisionMatrix()
{
	m_shapeInstance.SetGlobalMatrix(m_shapeInstance.GetLocalMatrix() * m_matrix);
	m_shapeInstance.CalculateFastAABB(m_shapeInstance.GetGlobalMatrix(), m_minAABB, m_maxAABB);
}

void dBody::SetMatrix(const dMatrix& matrix)
{
	m_equilibrium = 0;
	m_matrix = matrix;
	dAssert(m_matrix.TestOrthogonal(dFloat32(1.0e-4f)));

	m_rotation = dQuaternion(m_matrix);
	m_globalCentreOfMass = m_matrix.TransformVector(m_localCentreOfMass);
}
