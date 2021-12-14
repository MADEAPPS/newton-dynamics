/* Copyright (c) <2003-2021> <Julio Jerez, Newton Game Dynamics>
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

#ifndef __ND_COLLISION_CONVEX_POLYGON_H__
#define __ND_COLLISION_CONVEX_POLYGON_H__

#include "ndShapeConvex.h"
class ndShapeInstance;

#define D_CONVEX_POLYGON_MAX_VERTEX_COUNT	64

D_MSV_NEWTON_ALIGN_32
class ndShapeConvexPolygon: public ndShapeConvex
{
	public:
	class dgClippedFaceEdge
	{
		public:
		dgClippedFaceEdge* m_next;
		dgClippedFaceEdge* m_twin;
		ndInt32 m_incidentNormal;
		ndInt32 m_incidentVertex;
	};

	ndShapeConvexPolygon ();
	~ndShapeConvexPolygon ();

	virtual ndShapeConvexPolygon* GetAsShapeAsConvexPolygon();

	ndVector CalculateGlobalNormal(const ndShapeInstance* const parentMesh, const ndVector& localNormal) const;
	ndInt32 CalculateContactToConvexHullDescrete(const ndShapeInstance* const parentMesh, ndContactSolver& proxy);
	ndInt32 CalculateContactToConvexHullContinue(const ndShapeInstance* const parentMesh, ndContactSolver& proxy);

	virtual ndFloat32 GetVolume() const;
	virtual ndFloat32 GetBoxMinRadius() const;
	virtual ndFloat32 GetBoxMaxRadius() const;
	virtual ndVector SupportVertex(const ndVector& dir, ndInt32* const vertexIndex) const;
	bool BeamClipping(const ndVector& origin, ndFloat32 size, const ndShapeInstance* const parentMesh);
	virtual ndInt32 CalculatePlaneIntersection(const ndVector& normal, const ndVector& point, ndVector* const contactsOut) const;

	virtual ndFloat32 RayCast(ndRayCastNotify& callback, const ndVector& localP0, const ndVector& localP1, ndFloat32 maxT, const ndBody* const body, ndContactPoint& contactOut) const;

	virtual ndInt32 Release() const;

	ndVector m_normal;
	ndVector m_localPoly[D_CONVEX_POLYGON_MAX_VERTEX_COUNT];
	ndInt32 m_clippEdgeNormal[D_CONVEX_POLYGON_MAX_VERTEX_COUNT];
	ndFloat32 m_faceClipSize;
	ndInt32 m_count;
	ndInt32 m_paddedCount;
	ndInt32 m_faceId;
	ndInt32 m_stride;
	ndInt32 m_faceNormalIndex;
	
	const ndFloat32* m_vertex;
	const ndInt32* m_vertexIndex;
	const ndInt32* m_adjacentFaceEdgeNormalIndex;
} D_GCC_NEWTON_ALIGN_32;

inline ndShapeConvexPolygon* ndShapeConvexPolygon::GetAsShapeAsConvexPolygon()
{
	return this; 
}

inline ndFloat32 ndShapeConvexPolygon::RayCast(ndRayCastNotify&, const ndVector&, const ndVector&, ndFloat32, const ndBody* const, ndContactPoint&) const
{
	return ndFloat32(1.2f);
}

inline ndFloat32 ndShapeConvexPolygon::GetVolume() const
{
	return ndFloat32(0.0f);
}

inline ndFloat32 ndShapeConvexPolygon::GetBoxMinRadius() const
{
	return m_faceClipSize;
}

inline ndFloat32 ndShapeConvexPolygon::GetBoxMaxRadius() const
{
	return GetBoxMinRadius();
}

#endif

