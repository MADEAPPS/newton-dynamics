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
#include "ndShapeNull.h"

ndShapeNull::ndShapeNull()
	:ndShape(m_nullCollision)
{
	m_inertia = ndVector::m_one | ndVector::m_triplexMask;
	ndAssert(ndMemory::CheckMemory(this));
}

ndShapeNull::~ndShapeNull()
{
	ndAssert(ndMemory::CheckMemory(this));
}

ndShapeNull* ndShapeNull::GetAsShapeNull()
{
	return this;
}

ndVector ndShapeNull::SupportVertex(const ndVector&) const
{
	return ndVector::m_zero;
}

ndVector ndShapeNull::SupportVertexSpecial(const ndVector&, ndFloat32) const
{
	return ndVector::m_zero;
}

ndFloat32 ndShapeNull::RayCast(ndRayCastNotify&, const ndVector&, const ndVector&, ndFloat32, const ndBody* const, ndContactPoint&) const
{
	return ndFloat32(1.2f);
}

void ndShapeNull::DebugShape(const ndMatrix&, ndShapeDebugNotify&) const
{
}

void ndShapeNull::CalculateAabb(const ndMatrix&, ndVector& p0, ndVector& p1) const
{
	p0 = ndVector::m_zero;
	p1 = ndVector::m_zero;
}

ndShapeInfo ndShapeNull::GetShapeInfo() const
{
	ndAssert(0);
	ndShapeInfo info;
	return info;
}

ndFloat32 ndShapeNull::GetVolume() const
{
	return ndFloat32(0.0f);
}

ndFloat32 ndShapeNull::GetBoxMinRadius() const
{
	return ndFloat32(0.0f);
}

ndFloat32 ndShapeNull::GetBoxMaxRadius() const
{
	return ndFloat32(0.0f);
}

ndVector ndShapeNull::CalculateVolumeIntegral(const ndMatrix&, const ndVector&, const ndShapeInstance&) const
{
	return ndVector::m_zero;
}

ndVector ndShapeNull::SupportVertexSpecialProjectPoint(const ndVector& point, const ndVector&) const
{
	return point;
}

ndInt32 ndShapeNull::CalculatePlaneIntersection(const ndVector&, const ndVector&, ndVector* const) const
{
	return 0;
}

ndUnsigned64 ndShapeNull::GetHash(ndUnsigned64) const
{
	return 12345678;
}

