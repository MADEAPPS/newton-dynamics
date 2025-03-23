/* Copyright (c) <2003-2022> <Julio Jerez, Newton Game Dynamics>
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

#include "ndCoreStdafx.h"
#include "ndCollisionStdafx.h"
#include "ndBody.h"
#include "ndContact.h"
#include "ndBodyNotify.h"

ndUnsigned32 ndBody::m_uniqueIdCount = 0;

ndBody::ndBody()
	:ndContainersFreeListAlloc<ndBody>()
	,m_matrix(ndGetIdentityMatrix())
	,m_rotation()
	,m_veloc(ndVector::m_zero)
	,m_omega(ndVector::m_zero)
	,m_localCentreOfMass(ndVector::m_wOne)
	,m_globalCentreOfMass(ndVector::m_wOne)
	,m_minAabb(ndVector::m_wOne)
	,m_maxAabb(ndVector::m_wOne)
	,m_notifyCallback(nullptr)
	//,m_deletedNode(nullptr)
	,m_uniqueId(m_uniqueIdCount)
	,m_flags(0)
	,m_isStatic(0)
	,m_autoSleep(1)
	,m_equilibrium(0)
	,m_equilibrium0(0)
	,m_isJointFence0(0)
	,m_isJointFence1(0)
	,m_isConstrained(0)
	,m_sceneForceUpdate(1)
	,m_sceneEquilibrium(0)
	,m_skeletonSelfCollision(0)
{
	m_uniqueIdCount++;
	m_transformIsDirty = 1;
}

ndBody::ndBody(const ndBody& src)
	:ndContainersFreeListAlloc<ndBody>()
	,m_matrix(src.m_matrix)
	,m_rotation(src.m_rotation)
	,m_veloc(src.m_veloc)
	,m_omega(src.m_omega)
	,m_localCentreOfMass(src.m_localCentreOfMass)
	,m_globalCentreOfMass(src.m_globalCentreOfMass)
	,m_minAabb(src.m_minAabb)
	,m_maxAabb(src.m_maxAabb)
	,m_notifyCallback(nullptr)
	//,m_deletedNode(nullptr)
	,m_uniqueId(m_uniqueIdCount)
	,m_flags(0)
	,m_isStatic(0)
	,m_autoSleep(src.m_autoSleep)
	,m_equilibrium(0)
	,m_equilibrium0(0)
	,m_isJointFence0(0)
	,m_isJointFence1(0)
	,m_isConstrained(0)
	,m_sceneForceUpdate(1)
	,m_sceneEquilibrium(0)
{
	m_uniqueIdCount++;
	m_transformIsDirty = 1;
	if (src.m_notifyCallback)
	{
		SetNotifyCallback(src.m_notifyCallback->Clone());
	}
}

ndBody::~ndBody()
{
	//ndAssert(!m_deletedNode);
	if (m_notifyCallback)
	{
		delete m_notifyCallback;
	}
}

bool ndBody::GetSeletonSelfCollision() const
{
	return m_skeletonSelfCollision ? true : false;
}

void ndBody::SetSeletonSelfCollision(bool state)
{
	m_skeletonSelfCollision = ndUnsigned8(state ? 1 : 0);
}

ndUnsigned32 ndBody::GetId() const
{
	return m_uniqueId;
}

ndBodyNotify* ndBody::GetNotifyCallback() const
{
	return m_notifyCallback;
}

ndMatrix ndBody::GetMatrix() const
{
	return m_matrix;
}

ndVector ndBody::GetPosition() const
{
	return m_matrix.m_posit;
}

ndQuaternion ndBody::GetRotation() const
{
	return m_rotation;
}

ndVector ndBody::GetGlobalGetCentreOfMass() const
{
	return m_globalCentreOfMass;
}

ndVector ndBody::GetVelocity() const
{
	return m_veloc;
}

ndVector ndBody::GetOmega() const
{
	return m_omega;
}

void ndBody::GetAABB(ndVector& p0, ndVector& p1) const
{
	p0 = m_minAabb;
	p1 = m_maxAabb;
}

const ndVector& ndBody::GetCentreOfMass() const
{
	return m_localCentreOfMass;
}

ndVector ndBody::GetVelocityAtPoint(const ndVector& point) const
{
	return m_veloc + m_omega.CrossProduct(point - m_globalCentreOfMass);
}

ndFloat32 ndBody::GetInvMass() const
{
	return ndFloat32(0.0f);
}

void ndBody::SetCentreOfMass(const ndVector& com)
{
	m_localCentreOfMass.m_x = com.m_x;
	m_localCentreOfMass.m_y = com.m_y;
	m_localCentreOfMass.m_z = com.m_z;
	m_localCentreOfMass.m_w = ndFloat32(1.0f);
	m_globalCentreOfMass = m_matrix.TransformVector(m_localCentreOfMass);
}

void ndBody::SetNotifyCallback(ndBodyNotify* const notify)
{
	if (notify != m_notifyCallback)
	{
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
}

void ndBody::SetOmegaNoSleep(const ndVector& omega)
{
	m_omega = omega;
}

void ndBody::SetOmega(const ndVector& omega)
{
	m_equilibrium = 0;
	SetOmegaNoSleep(omega);
}

void ndBody::SetVelocityNoSleep(const ndVector& veloc)
{
	m_veloc = veloc;
}

void ndBody::SetVelocity(const ndVector& veloc)
{
	m_equilibrium = 0;
	SetVelocityNoSleep(veloc);
}

void ndBody::SetMatrixNoSleep(const ndMatrix& matrix)
{
	m_matrix = matrix;
	ndAssert(m_matrix.TestOrthogonal(ndFloat32(1.0e-4f)));

	m_rotation = ndQuaternion(m_matrix);
	m_globalCentreOfMass = m_matrix.TransformVector(m_localCentreOfMass);
}

void ndBody::SetMatrixAndCentreOfMass(const ndQuaternion& rotation, const ndVector& globalcom)
{
	m_rotation = rotation;
	ndAssert(m_rotation.DotProduct(m_rotation).GetScalar() > ndFloat32(0.9999f));
	m_globalCentreOfMass = globalcom;
	m_matrix = ndCalculateMatrix(rotation, m_matrix.m_posit);
	m_matrix.m_posit = m_globalCentreOfMass - m_matrix.RotateVector(m_localCentreOfMass);
}

void ndBody::SetMatrix(const ndMatrix& matrix)
{
	m_equilibrium = 0;
	m_transformIsDirty = 1;
	m_sceneForceUpdate = 1;
	SetMatrixNoSleep(matrix);
}

