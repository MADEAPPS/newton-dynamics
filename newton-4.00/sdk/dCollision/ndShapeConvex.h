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

#ifndef __D_SHAPE_CONVEX_H__
#define __D_SHAPE_CONVEX_H__

#include "ndCollisionStdafx.h"
#include "ndShape.h"

#define D_CLIP_MAX_COUNT				512
#define D_CLIP_MAX_POINT_COUNT			64
#define D_MIN_CONVEX_SHAPE_SIZE			dFloat32 (1.0f/128.0f)

D_MSV_NEWTON_ALIGN_32
class ndShapeConvex: public ndShape
{
	public:

	//bool IntesectionTest (dShapeParamProxy& proxy) const;

	class ndConvexSimplexEdge
	{
		public:
		ndConvexSimplexEdge* m_twin;
		ndConvexSimplexEdge* m_next;
		ndConvexSimplexEdge* m_prev;
		dInt32 m_vertex;
	};

	protected:
	D_COLLISION_API ndShapeConvex (ndShapeID id);
	D_COLLISION_API ~ndShapeConvex ();

	virtual ndShapeConvex* GetAsShapeConvex() { return this; }

	D_COLLISION_API void SetVolumeAndCG();
	D_COLLISION_API virtual void MassProperties();
	D_COLLISION_API virtual void DebugShape(const dMatrix& matrix, ndShapeDebugCallback& debugCallback) const;
	D_COLLISION_API virtual dFloat32 CalculateMassProperties(const dMatrix& offset, dVector& inertia, dVector& crossInertia, dVector& centerOfMass) const;
	D_COLLISION_API virtual dMatrix CalculateInertiaAndCenterOfMass(const dMatrix& alignMatrix, const dVector& localScale, const dMatrix& matrix) const;

	D_COLLISION_API virtual ndShapeInfo GetShapeInfo() const;
	D_COLLISION_API virtual void CalcAABB(const dMatrix& matrix, dVector& p0, dVector& p1) const;
	D_COLLISION_API virtual dVector SupportVertex(const dVector& dir, dInt32* const vertexIndex) const;
	D_COLLISION_API virtual dInt32 CalculatePlaneIntersection(const dVector& normal, const dVector& point, dVector* const contactsOut) const;
	D_COLLISION_API virtual dVector CalculateVolumeIntegral(const dMatrix& globalMatrix, const dVector& globalPlane, const ndShapeInstance& parentScale) const;
	D_COLLISION_API virtual dFloat32 RayCast(ndRayCastNotify& callback, const dVector& localP0, const dVector& localP1, const ndBody* const body, ndContactPoint& contactOut) const;

	bool SanityCheck(dPolyhedra& hull) const;
	bool SanityCheck(dInt32 count, const dVector& normal, dVector* const contactsOut) const;
	dInt32 RectifyConvexSlice(dInt32 count, const dVector& normal, dVector* const contactsOut) const;
	virtual dInt32 GetConvexVertexCount() const { return m_vertexCount; }
	virtual dVector SupportVertexSpecial(const dVector& dir, dFloat32 skinThickness, dInt32* const vertexIndex) const
	{
		return SupportVertex(dir, vertexIndex);
	}

	virtual dVector SupportVertexSpecialProjectPoint(const dVector& point, const dVector& dir) const
	{
		return point;
	}

	virtual const ndConvexSimplexEdge** GetVertexToEdgeMapping() const 
	{ 
		return nullptr; 
	}

	virtual dFloat32 GetVolume() const;
	virtual dFloat32 GetBoxMinRadius() const;
	virtual dFloat32 GetBoxMaxRadius() const;

	dVector CalculateVolumeIntegral(const dPlane& plane) const;

	//virtual void SerializeLow(dgSerialize callback, void* const userData) const;
	//dInt32 RayCastClosestFace (dVector* tetrahedrum, const dVector& origin, dFloat32& pointDist) const;
	dInt32 BuildCylinderCapPoly (dFloat32 radius, const dMatrix& transform, dVector* const vertexOut) const;

	dVector* m_vertex;
	ndConvexSimplexEdge* m_simplex;

	dFloat32 m_boxMinRadius;
	dFloat32 m_boxMaxRadius;
	dFloat32 m_simplexVolume;
	dUnsigned16 m_edgeCount;
	dUnsigned16 m_vertexCount;
	friend class ndContactSolver;
} D_GCC_NEWTON_ALIGN_32 ;

#endif 


