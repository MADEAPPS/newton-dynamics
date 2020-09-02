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

#include "ntStdafx.h"
#include "ntBody.h"
#include "ntContact.h"
#include "ntShapeNull.h"
#include "ntBodyNotify.h"
#include "ntRayCastNotify.h"

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////


dUnsigned32 ntBody::m_uniqueIDCount = 0;

class ntDummyCollision: public ntShapeNull
{
	public: 
	ntDummyCollision()
		:ntShapeNull()
	{
		m_refCount.fetch_add(1);
	}

	~ntDummyCollision()
	{
		m_refCount.fetch_add(-1);
	}

	static ntShapeNull* GetNullShape()
	{
		static ntDummyCollision nullShape;
		return &nullShape;
	}
};

ntBody::ntBody()
	:m_matrix(dGetIdentityMatrix())
	,m_invWorldInertiaMatrix(dGetZeroMatrix())
	,m_shapeInstance(ntDummyCollision::GetNullShape())
	,m_veloc(dVector::m_zero)
	,m_omega(dVector::m_zero)
	,m_localCentreOfMass(dVector::m_zero)
	,m_globalCentreOfMass(dVector::m_zero)
	,m_minAABB(dVector::m_zero)
	,m_maxAABB(dVector::m_zero)
	,m_rotation()
	,m_world(nullptr)
	,m_notifyCallback(nullptr)
	,m_broadPhaseNode(nullptr)
	,m_worldNode(nullptr)
	,m_broadPhaseAggregateNode(nullptr)
	,m_flags(0)
	,m_uniqueID(m_uniqueIDCount)
{
	m_equilibrium = 0;
	m_collideWithLinkedBodies = 1;
	m_uniqueIDCount++;
}

ntBody::~ntBody()
{
	if (m_notifyCallback)
	{
		delete m_notifyCallback;
	}
}

void ntBody::SetCentreOfMass(const dVector& com)
{
	m_localCentreOfMass.m_x = com.m_x;
	m_localCentreOfMass.m_y = com.m_y;
	m_localCentreOfMass.m_z = com.m_z;
	m_localCentreOfMass.m_w = dFloat32(1.0f);
	m_globalCentreOfMass = m_matrix.TransformVector(m_localCentreOfMass);
}

ntShapeInstance& ntBody::GetCollisionShape()
{
	//return *((ntShapeInstance*)&m_shapeInstance);
	return (ntShapeInstance&)m_shapeInstance;
}

const ntShapeInstance& ntBody::GetCollisionShape() const
{
	return m_shapeInstance;
}

void ntBody::SetCollisionShape(const ntShapeInstance& shapeInstance)
{
	m_shapeInstance = shapeInstance;
}

void ntBody::SetNotifyCallback(ntBodyNotify* const notify)
{
	//dAssert(notify->m_body == nullptr);
	if (m_notifyCallback)
	{
		delete m_notifyCallback;
	}
	m_notifyCallback = notify;
	if (m_notifyCallback)
	{
		m_notifyCallback->m_body = this;
	}
}

ntBodyNotify* ntBody::GetNotifyCallback(ntBodyNotify* const notify) const
{
	return m_notifyCallback;
}

dInt32 ntBody::GetId() const
{
	return m_uniqueID;
}

ntWorld* ntBody::GetWorld() const
{
	return m_world;
}

void ntBody::SetWorldNode(ntWorld* const world, dList<ntBody*>::dListNode* const node)
{
	m_world = world;
	m_worldNode = node;
}

dList<ntBody*>::dListNode* ntBody::GetNewtonNode() const
{
	return m_worldNode;
}

ntBroadPhaseBodyNode* ntBody::GetBroadPhaseNode() const
{
	return m_broadPhaseNode;
}

void ntBody::SetBroadPhaseNode(ntBroadPhaseBodyNode* const node)
{
	m_broadPhaseNode = node;
}

ntBroadPhaseAggregate* ntBody::GetBroadPhaseAggregate() const
{
	return m_broadPhaseAggregateNode;
}

void ntBody::SetBroadPhaseAggregate(ntBroadPhaseAggregate* const node)
{
	m_broadPhaseAggregateNode = node;
}

dVector ntBody::GetOmega() const
{
	return m_omega;
}

void ntBody::SetOmega(const dVector& veloc)
{
	m_equilibrium = 0;
	m_omega = veloc;
}

dVector ntBody::GetVelocity() const
{
	return m_veloc;
}

void ntBody::SetVelocity(const dVector& veloc)
{
	m_equilibrium = 0;
	m_veloc = veloc;
}

dMatrix ntBody::GetMatrix() const
{
	return m_matrix;
}

void ntBody::UpdateCollisionMatrix()
{
	m_shapeInstance.SetGlobalMatrix(m_shapeInstance.GetLocalMatrix() * m_matrix);
	m_shapeInstance.CalculateFastAABB(m_shapeInstance.GetGlobalMatrix(), m_minAABB, m_maxAABB);
}

void ntBody::SetMatrix(const dMatrix& matrix)
{
	m_equilibrium = 0;
	m_matrix = matrix;
	dAssert(m_matrix.TestOrthogonal(dFloat32(1.0e-4f)));

	m_rotation = dQuaternion(m_matrix);
	m_globalCentreOfMass = m_matrix.TransformVector(m_localCentreOfMass);
}

dFloat32 ntBody::RayCast(ntRayCastNotify& callback, const dFastRayTest& ray, dFloat32 maxT) const
{
	dVector l0(ray.m_p0);
	dVector l1(ray.m_p0 + ray.m_diff.Scale(dMin(maxT, dFloat32(1.0f))));

	if (dRayBoxClip(l0, l1, m_minAABB, m_maxAABB))
	{
		const dMatrix& globalMatrix = m_shapeInstance.GetGlobalMatrix();
		dVector localP0(globalMatrix.UntransformVector(l0));
		dVector localP1(globalMatrix.UntransformVector(l1));
		dVector p1p0(localP1 - localP0);
		dAssert(p1p0.m_w == dFloat32(0.0f));
		if (p1p0.DotProduct(p1p0).GetScalar() > dFloat32(1.0e-12f)) 
		{
			ntContactPoint contactOut;
			dFloat32 t = m_shapeInstance.RayCast(callback, localP0, localP1, maxT, this, contactOut);
			if (t < dFloat32(1.0f)) 
			{
				dAssert(localP0.m_w == dFloat32(0.0f));
				dAssert(localP1.m_w == dFloat32(0.0f));
				dVector p(globalMatrix.TransformVector(localP0 + (localP1 - localP0).Scale(t)));
				t = ray.m_diff.DotProduct(p - ray.m_p0).GetScalar() / ray.m_diff.DotProduct(ray.m_diff).GetScalar();
				if (t < maxT) 
				{
					dAssert(t >= dFloat32(0.0f));
					dAssert(t <= dFloat32(1.0f));
					contactOut.m_body0 = this;
					contactOut.m_body1 = this;
					contactOut.m_point = p;
					contactOut.m_normal = globalMatrix.RotateVector(contactOut.m_normal);
					maxT = callback.OnRayCastAction(contactOut, t);
				}
			}
		}
	}
	return maxT;
}