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
#include "ntShapeNull.h"

#if 0
ntShapeNull::ntShapeNull(dgMemoryAllocator* const allocator, dgUnsigned32 signature)
	:dShape(allocator, signature, m_nullCollision) 
{
	m_rtti |= ntShapeNull_RTTI;
	m_inertia = dVector (dFloat32 (1.0f), dFloat32 (1.0f), dFloat32 (1.0f), dFloat32 (0.0f));
}

ntShapeNull::ntShapeNull(dgWorld* const world, dgDeserialize deserialization, void* const userData, dgInt32 revisionNumber)
	:dShape (world, deserialization, userData, revisionNumber)
{
	m_rtti |= ntShapeNull_RTTI;
}

void ntShapeNull::Serialize(dgSerialize callback, void* const userData) const
{
	SerializeLow(callback, userData);
}

ntShapeNull::~ntShapeNull()
{
}

void ntShapeNull::SetCollisionBBox (const dVector& p0, const dVector& p1)
{
	dgAssert (0);
}


dgInt32 ntShapeNull::CalculateSignature () const
{
	return dgInt32 (GetSignature());
}

void ntShapeNull::CalcAABB (const dgMatrix& matrix, dVector &p0, dVector &p1) const
{
	p0 = matrix[3] & dVector::m_triplexMask;
	p1 = matrix[3] & dVector::m_triplexMask;
}

dVector ntShapeNull::SupportVertex (const dVector& dir, dgInt32* const vertexIndex) const
{
	dgAssert (0);
	return dVector::m_zero;
}

dVector ntShapeNull::SupportVertexSpecial (const dVector& dir, dFloat32 skinThickness, dgInt32* const vertexIndex) const
{
	dgAssert(0);
	return dVector::m_zero;
}

dFloat32 ntShapeNull::GetVolume () const
{
	return dFloat32 (0.0f);
}

dFloat32 ntShapeNull::RayCast (const dVector& localP0, const dVector& localP1, dFloat32 maxT, dgContactPoint& contactOut, const dgBody* const body, void* const userData, OnRayPrecastAction preFilter) const
{
	return dFloat32 (1.2f);
}


dVector ntShapeNull::CalculateVolumeIntegral (const dgMatrix& globalMatrix, const dVector& plane, const dgCollisionInstance& parentScale) const
{
	dgAssert (0);
	return dVector::m_zero;
}
#endif


ntShapeNull::ntShapeNull()
	:ntShapeConvex(m_nullCollision)
{
	m_inertia = dVector(dFloat32(1.0f), dFloat32(1.0f), dFloat32(1.0f), dFloat32(0.0f));
}
