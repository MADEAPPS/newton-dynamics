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

#ifndef __D_COLLISION_CONVEX_POLYGON_H__
#define __D_COLLISION_CONVEX_POLYGON_H__

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
		dInt32 m_incidentNormal;
		dInt32 m_incidentVertex;
	};

	ndShapeConvexPolygon ();
	~ndShapeConvexPolygon ();

	virtual ndShapeConvexPolygon* GetAsShapeAsConvexPolygon();

	dVector CalculateGlobalNormal(const ndShapeInstance* const parentMesh, const dVector& localNormal) const;
	dInt32 CalculateContactToConvexHullDescrete(const ndShapeInstance* const parentMesh, ndContactSolver& proxy);
	dInt32 CalculateContactToConvexHullContinue(const ndShapeInstance* const parentMesh, ndContactSolver& proxy);

	virtual dFloat32 GetVolume() const;
	virtual dFloat32 GetBoxMinRadius() const;
	virtual dFloat32 GetBoxMaxRadius() const;
	virtual dVector SupportVertex(const dVector& dir, dInt32* const vertexIndex) const;
	bool BeamClipping(const dVector& origin, dFloat32 size, const ndShapeInstance* const parentMesh);
	virtual dInt32 CalculatePlaneIntersection(const dVector& normal, const dVector& point, dVector* const contactsOut) const;

	virtual dFloat32 RayCast(ndRayCastNotify& callback, const dVector& localP0, const dVector& localP1, dFloat32 maxT, const ndBody* const body, ndContactPoint& contactOut) const;

	virtual dInt32 Release() const;

	dVector m_normal;
	dVector m_localPoly[D_CONVEX_POLYGON_MAX_VERTEX_COUNT];
	dInt32 m_clippEdgeNormal[D_CONVEX_POLYGON_MAX_VERTEX_COUNT];
	dFloat32 m_faceClipSize;
	dInt32 m_count;
	dInt32 m_paddedCount;
	dInt32 m_faceId;
	dInt32 m_stride;
	dInt32 m_faceNormalIndex;
	
	const dFloat32* m_vertex;
	const dInt32* m_vertexIndex;
	const dInt32* m_adjacentFaceEdgeNormalIndex;
} D_GCC_NEWTON_ALIGN_32;

inline ndShapeConvexPolygon* ndShapeConvexPolygon::GetAsShapeAsConvexPolygon()
{
	return this; 
}

inline dFloat32 ndShapeConvexPolygon::RayCast(ndRayCastNotify&, const dVector&, const dVector&, dFloat32, const ndBody* const, ndContactPoint&) const
{
	return dFloat32(1.2f);
}

inline dFloat32 ndShapeConvexPolygon::GetVolume() const
{
	return dFloat32(0.0f);
}

inline dFloat32 ndShapeConvexPolygon::GetBoxMinRadius() const
{
	return m_faceClipSize;
}

inline dFloat32 ndShapeConvexPolygon::GetBoxMaxRadius() const
{
	return GetBoxMinRadius();
}

#endif

