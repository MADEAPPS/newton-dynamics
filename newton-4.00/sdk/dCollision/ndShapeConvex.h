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

#ifndef __ND_SHAPE_CONVEX_H__
#define __ND_SHAPE_CONVEX_H__

#include "ndCollisionStdafx.h"
#include "ndShape.h"

#define D_CLIP_MAX_COUNT				512
#define D_CLIP_MAX_POINT_COUNT			64
#define D_MIN_CONVEX_SHAPE_SIZE			ndFloat32 (1.0f/128.0f)

D_MSV_NEWTON_ALIGN_32
class ndShapeConvex: public ndShape
{
	public:
	D_CLASS_REFLECTION(ndShapeConvex);
	class ndConvexSimplexEdge
	{
		public:
		ndConvexSimplexEdge* m_twin;
		ndConvexSimplexEdge* m_next;
		ndConvexSimplexEdge* m_prev;
		ndInt32 m_vertex;
	};

	protected:
	D_COLLISION_API ndShapeConvex (ndShapeID id);
	D_COLLISION_API ~ndShapeConvex ();

	virtual ndShapeConvex* GetAsShapeConvex() { return this; }

	D_COLLISION_API void SetVolumeAndCG();
	D_COLLISION_API virtual void MassProperties();
	D_COLLISION_API virtual void DebugShape(const ndMatrix& matrix, ndShapeDebugNotify& debugCallback) const;
	D_COLLISION_API virtual ndFloat32 CalculateMassProperties(const ndMatrix& offset, ndVector& inertia, ndVector& crossInertia, ndVector& centerOfMass) const;
	D_COLLISION_API virtual ndMatrix CalculateInertiaAndCenterOfMass(const ndMatrix& alignMatrix, const ndVector& localScale, const ndMatrix& matrix) const;

	D_COLLISION_API virtual ndShapeInfo GetShapeInfo() const;
	D_COLLISION_API virtual void CalculateAabb(const ndMatrix& matrix, ndVector& p0, ndVector& p1) const;
	D_COLLISION_API virtual ndVector SupportVertex(const ndVector& dir, ndInt32* const vertexIndex) const;
	D_COLLISION_API virtual ndInt32 CalculatePlaneIntersection(const ndVector& normal, const ndVector& point, ndVector* const contactsOut) const;
	D_COLLISION_API virtual ndVector CalculateVolumeIntegral(const ndMatrix& globalMatrix, const ndVector& globalPlane, const ndShapeInstance& parentScale) const;
	D_COLLISION_API virtual ndFloat32 RayCast(ndRayCastNotify& callback, const ndVector& localP0, const ndVector& localP1, ndFloat32 maxT, const ndBody* const body, ndContactPoint& contactOut) const;

	D_COLLISION_API virtual void Save(const ndLoadSaveBase::ndSaveDescriptor& desc) const;
	bool SanityCheck(ndPolyhedra& hull) const;
	bool SanityCheck(ndInt32 count, const ndVector& normal, ndVector* const contactsOut) const;
	ndInt32 RectifyConvexSlice(ndInt32 count, const ndVector& normal, ndVector* const contactsOut) const;
	virtual ndInt32 GetConvexVertexCount() const { return m_vertexCount; }
	virtual ndVector SupportVertexSpecial(const ndVector& dir, ndFloat32, ndInt32* const vertexIndex) const
	{
		return SupportVertex(dir, vertexIndex);
	}

	virtual ndVector SupportVertexSpecialProjectPoint(const ndVector& point, const ndVector&) const
	{
		return point;
	}

	virtual const ndConvexSimplexEdge** GetVertexToEdgeMapping() const 
	{ 
		return nullptr; 
	}

	virtual ndFloat32 GetVolume() const;
	virtual ndFloat32 GetBoxMinRadius() const;
	virtual ndFloat32 GetBoxMaxRadius() const;

	ndVector CalculateVolumeIntegral(const ndPlane& plane) const;
	ndInt32 BuildCylinderCapPoly (ndFloat32 radius, const ndMatrix& transform, ndVector* const vertexOut) const;

	ndVector* m_vertex;
	ndConvexSimplexEdge* m_simplex;

	ndFloat32 m_boxMinRadius;
	ndFloat32 m_boxMaxRadius;
	ndFloat32 m_simplexVolume;
	ndUnsigned16 m_edgeCount;
	ndUnsigned16 m_vertexCount;
	friend class ndMeshEffect;
	friend class ndContactSolver;
} D_GCC_NEWTON_ALIGN_32 ;

#endif 


