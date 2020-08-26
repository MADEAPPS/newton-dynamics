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
#include "dShapeNull.h"

#if 0
dShapeNull::dShapeNull(dgMemoryAllocator* const allocator, dgUnsigned32 signature)
	:dShape(allocator, signature, m_nullCollision) 
{
	m_rtti |= dShapeNull_RTTI;
	m_inertia = dVector (dFloat32 (1.0f), dFloat32 (1.0f), dFloat32 (1.0f), dFloat32 (0.0f));
}

dShapeNull::dShapeNull(dgWorld* const world, dgDeserialize deserialization, void* const userData, dgInt32 revisionNumber)
	:dShape (world, deserialization, userData, revisionNumber)
{
	m_rtti |= dShapeNull_RTTI;
}

void dShapeNull::Serialize(dgSerialize callback, void* const userData) const
{
	SerializeLow(callback, userData);
}

dShapeNull::~dShapeNull()
{
}

void dShapeNull::SetCollisionBBox (const dVector& p0, const dVector& p1)
{
	dgAssert (0);
}

void dShapeNull::DebugCollision (const dgMatrix& matrixPtr, dgCollision::OnDebugCollisionMeshCallback callback, void* const userData) const
{
}

dgInt32 dShapeNull::CalculateSignature () const
{
	return dgInt32 (GetSignature());
}

void dShapeNull::CalcAABB (const dgMatrix& matrix, dVector &p0, dVector &p1) const
{
	p0 = matrix[3] & dVector::m_triplexMask;
	p1 = matrix[3] & dVector::m_triplexMask;
}

dVector dShapeNull::SupportVertex (const dVector& dir, dgInt32* const vertexIndex) const
{
	dgAssert (0);
	return dVector::m_zero;
}

dVector dShapeNull::SupportVertexSpecial (const dVector& dir, dFloat32 skinThickness, dgInt32* const vertexIndex) const
{
	dgAssert(0);
	return dVector::m_zero;
}

dFloat32 dShapeNull::GetVolume () const
{
	return dFloat32 (0.0f);
}

dFloat32 dShapeNull::RayCast (const dVector& localP0, const dVector& localP1, dFloat32 maxT, dgContactPoint& contactOut, const dgBody* const body, void* const userData, OnRayPrecastAction preFilter) const
{
	return dFloat32 (1.2f);
}


dVector dShapeNull::CalculateVolumeIntegral (const dgMatrix& globalMatrix, const dVector& plane, const dgCollisionInstance& parentScale) const
{
	dgAssert (0);
	return dVector::m_zero;
}
#endif


dShapeNull::dShapeNull()
	:dShapeConvex(m_nullCollision)
{
	m_inertia = dVector(dFloat32(1.0f), dFloat32(1.0f), dFloat32(1.0f), dFloat32(0.0f));
}
