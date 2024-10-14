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
#include "ndShape.h"

ndVector ndShape::m_flushZero(ndFloat32(1.0e-7f));

ndShape::ndShape(ndShapeID id)
	:ndContainersFreeListAlloc<ndShape>()
	,m_inertia(ndVector::m_zero)
	,m_crossInertia(ndVector::m_zero)
	,m_centerOfMass(ndVector::m_zero)
	,m_boxSize(ndVector::m_zero)
	,m_boxOrigin(ndVector::m_zero)
	,m_refCount(0)
	,m_collisionId(id)
{
}

ndShape::ndShape(const ndShape& source)
	:ndContainersFreeListAlloc<ndShape>()
	,m_inertia(source.m_inertia)
	,m_crossInertia(source.m_crossInertia)
	,m_centerOfMass(source.m_centerOfMass)
	,m_boxSize(source.m_boxSize)
	,m_boxOrigin(source.m_boxOrigin)
	,m_refCount(0)
	,m_collisionId(source.m_collisionId)
{
}

ndShape::~ndShape()
{
	ndAssert(m_refCount.load() == 0);
}

void ndShape::MassProperties()
{
	// using general central theorem, to extract the Inertia relative to the center of mass 
	//IImatrix = IIorigin + unitmass * [(displacemnet % displacemnet) * identityMatrix - transpose(displacement) * displacement)];

	ndMatrix inertia(ndGetIdentityMatrix());
	inertia[0][0] = m_inertia[0];
	inertia[1][1] = m_inertia[1];
	inertia[2][2] = m_inertia[2];
	inertia[0][1] = m_crossInertia[2];
	inertia[1][0] = m_crossInertia[2];
	inertia[0][2] = m_crossInertia[1];
	inertia[2][0] = m_crossInertia[1];
	inertia[1][2] = m_crossInertia[0];
	inertia[2][1] = m_crossInertia[0];

	ndVector origin(m_centerOfMass);
	ndFloat32 originMag2 = origin.DotProduct(origin & ndVector::m_triplexMask).GetScalar();

	ndMatrix Covariance(ndCovarianceMatrix(origin, origin));
	ndMatrix parallel(ndGetIdentityMatrix());
	for (ndInt32 i = 0; i < 3; ++i) 
	{
		parallel[i][i] = originMag2;
		inertia[i] += (parallel[i] - Covariance[i]);
		ndAssert(inertia[i][i] > ndFloat32(0.0f));
	}

	m_inertia[0] = inertia[0][0];
	m_inertia[1] = inertia[1][1];
	m_inertia[2] = inertia[2][2];
	m_crossInertia[0] = inertia[2][1];
	m_crossInertia[1] = inertia[2][0];
	m_crossInertia[2] = inertia[1][0];
}

ndShapeInfo ndShape::GetShapeInfo() const
{
	ndShapeInfo info;
	info.m_collisionType = m_collisionId;
	return info;
}

const ndShape* ndShape::AddRef() const
{
	m_refCount.fetch_add(1);
	return this;
}

ndInt32 ndShape::Release() const
{
	ndInt32 count = m_refCount.fetch_add(-1);
	ndAssert(count >= 1);
	if (count == 1)
	{
		delete this;
	}
	return count;
}

ndInt32 ndShape::GetRefCount() const
{
	return m_refCount.load();
}
