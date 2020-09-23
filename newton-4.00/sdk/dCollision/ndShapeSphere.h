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

#ifndef _D_SHAPE_SPHERE_H__
#define _D_SHAPE_SPHERE_H__

#include "ndShapeConvex.h"

#define D_SPHERE_VERTEX_COUNT 18

D_MSV_NEWTON_ALIGN_32
class ndShapeSphere: public ndShapeConvex
{
	public:
	//ndShapeSphere(dgMemoryAllocator* allocator, dUnsigned32 signature, dFloat32 radius);
	//ndShapeSphere(dgWorld* const world, dgDeserialize deserialization, void* const userData, dInt32 revisionNumber);
	//virtual ~ndShapeSphere();
	D_COLLISION_API ndShapeSphere(dFloat32 radius);
	virtual ~ndShapeSphere();

	virtual ndShapeSphere* GetAsShapeSphere() { return this; }

	protected:
	D_COLLISION_API void Init(dFloat32 radius);
	D_COLLISION_API virtual void MassProperties();

	D_COLLISION_API virtual void CalcAABB(const dMatrix& matrix, dVector& p0, dVector& p1) const;
	D_COLLISION_API virtual dVector SupportVertexSpecialProjectPoint(const dVector& point, const dVector& dir) const;
	D_COLLISION_API virtual dVector SupportVertex(const dVector& dir, dInt32* const vertexIndex) const;
	D_COLLISION_API virtual dVector SupportVertexSpecial(const dVector& dir, dFloat32 skinThickness, dInt32* const vertexIndex) const;
	D_COLLISION_API virtual dFloat32 RayCast(ndRayCastNotify& callback, const dVector& localP0, const dVector& localP1, dFloat32 maxT, const ndBody* const body, ndContactPoint& contactOut) const;

	virtual dInt32 CalculatePlaneIntersection(const dVector& normal, const dVector& point, dVector* const contactsOut) const;

	void TesselateTriangle(dInt32 level, const dVector& p0, const dVector& p1, const dVector& p2, dInt32& count, dVector* const ouput) const;

	dVector m_vertex[D_SPHERE_VERTEX_COUNT];
	dFloat32 m_radius;

	static dInt32 m_shapeRefCount;
	static dVector m_unitSphere[];
	static ndConvexSimplexEdge m_edgeArray[];

} D_GCC_NEWTON_ALIGN_32;

/*
class dgCollisionPoint: public ndShapeSphere
{
	public:
	dgCollisionPoint (dgMemoryAllocator* const allocator) 
		:ndShapeSphere(allocator, 0x12344321, dFloat32 (0.25f))
	{
	}

	virtual dFloat32 GetVolume () const;
	virtual dVector SupportVertex (const dVector& dir, dInt32* const vertexIndex) const;
	virtual dVector SupportVertexSpecial (const dVector& dir, dFloat32 skinThickness, dInt32* const vertexIndex) const;
};
*/

#endif 

