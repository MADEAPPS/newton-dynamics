/* Copyright (c) <2003-2011> <Julio Jerez, Newton Game Dynamics>
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

#ifndef _DG_COLLISION_CAPSULE_H_
#define _DG_COLLISION_CAPSULE_H_

#include "dgCollisionConvex.h"

#define DG_CAP_SEGMENTS 4
#define DG_CAPSULE_SEGMENTS 6

class dgCollisionCapsule: public dgCollisionConvex  
{
	public:
	dgCollisionCapsule (dgMemoryAllocator* const allocator, dgUnsigned32 signature, dgFloat32 radius, dgFloat32 height);
	dgCollisionCapsule(dgWorld* const world, dgDeserialize deserialization, void* const userData);
	virtual ~dgCollisionCapsule();

	dgFloat32 GetRadius() const {return m_radius;}
	dgFloat32 GetHeight() const {return m_height;}
	

	private:
	void Init (dgFloat32 radius, dgFloat32 height);
	virtual void CalcAABB (const dgMatrix& matrix, dgVector& p0, dgVector& p1) const;
	virtual dgFloat32 RayCast (const dgVector& localP0, const dgVector& localP1, dgFloat32 maxT, dgContactPoint& contactOut, const dgBody* const body, void* const userData, OnRayPrecastAction preFilter) const;
	virtual dgVector SupportVertex (const dgVector& dir, dgInt32* const vertexIndex) const;

	virtual dgInt32 CalculatePlaneIntersection (const dgVector& normal, const dgVector& point, dgVector* const contactsOut, dgFloat32 normalSign) const;

	virtual void DebugCollision  (const dgMatrix& matrix, dgCollision::OnDebugCollisionMeshCallback callback, void* const userData) const;

	virtual dgInt32 CalculateSignature () const;
	virtual void SetCollisionBBox (const dgVector& p0, const dgVector& p1);
	virtual void MassProperties ();

	virtual void GetCollisionInfo(dgCollisionInfo* const info) const;
	virtual void Serialize(dgSerialize callback, void* const userData) const;

	static dgInt32 CalculateSignature (dgFloat32 radius, dgFloat32 height);

	// special feature based contact calculation for conics convex (ex spheres, capsules, tapered capsules, and chamfered cylinders)
	// in newton we only deal with sub set of conic function, that can be expressed by the equation
	// ((x - x0) / a)^2 + ((y - y0) / b)^2 + ((z - z0) / c)^2  = 1   and possible a linear or circular sweep of the same equation
	// this preclude parabolic and hyperbolic conics 
	virtual dgVector ConvexConicSupporVertex (const dgVector& dir) const;
	virtual dgVector ConvexConicSupporVertex (const dgVector& point, const dgVector& dir) const;
	//dgInt32 CalculateSphereConicContacts (dgFloat32 posit, const dgVector& normal, const dgVector& point, dgVector* const contact) const;
	virtual dgInt32 CalculateContacts (const dgVector& point, const dgVector& normal, dgCollisionParamProxy& proxy, dgVector* const contactsOut) const;

	void TesselateTriangle (dgInt32 level, const dgVector& p0, const dgVector& p1, const dgVector& p2, dgInt32& count, dgVector* ouput) const;

	dgFloat32 m_radius;
	dgFloat32 m_height;
	dgVector m_vertex[2 * DG_CAP_SEGMENTS * DG_CAPSULE_SEGMENTS];
	static dgInt32 m_shapeRefCount;
	static dgConvexSimplexEdge m_edgeArray[];

	friend class dgWorld;
};

#endif 

