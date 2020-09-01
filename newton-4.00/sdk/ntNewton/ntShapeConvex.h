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

#include "ntStdafx.h"
#include "ntBody.h"
#include "ntWorld.h"

#define D_CLIP_MAX_COUNT				512
#define D_CLIP_MAX_POINT_COUNT			64
#define D_MIN_CONVEX_SHAPE_SIZE			dFloat32 (1.0f/128.0f)

D_MSV_NEWTON_ALIGN_32
class ntShapeConvex: public ntShape
{
	public:
/*	
	virtual dInt32 GetConvexVertexCount() const { return m_vertexCount;}
	virtual void CalcAABB (const dMatrix& matrix, dVector& p0, dVector& p1) const;
	virtual dVector SupportVertex (const dVector& dir, dInt32* const vertexIndex) const;
	virtual dInt32 CalculatePlaneIntersection (const dVector& normal, const dVector& point, dVector* const contactsOut) const;
	virtual dFloat32 RayCast (const dVector& localP0, const dVector& localP1, dFloat32 maxT, dgContactPoint& contactOut, const dgBody* const body, void* const userData, OnRayPrecastAction preFilter) const;

	bool IntesectionTest (dShapeParamProxy& proxy) const;
*/
	protected:
	class dConvexSimplexEdge
	{
		public:
		dConvexSimplexEdge* m_twin;
		dConvexSimplexEdge* m_next;
		dConvexSimplexEdge* m_prev;
		dInt32 m_vertex;
	};

	D_NEWTON_API ntShapeConvex (dShapeID id);
//	dShapeConvex (dgWorld* const world, dgDeserialize deserialization, void* const userData, dInt32 revisionNumber);
	D_NEWTON_API ~ntShapeConvex ();

	virtual ntShapeConvex* GetAsShapeConvex() { return this; }

	D_NEWTON_API void SetVolumeAndCG();
	D_NEWTON_API virtual void MassProperties();
	D_NEWTON_API virtual void DebugShape(const dMatrix& matrix, ntShapeDebugCallback& debugCallback) const;
	D_NEWTON_API virtual dFloat32 CalculateMassProperties(const dMatrix& offset, dVector& inertia, dVector& crossInertia, dVector& centerOfMass) const;
	D_NEWTON_API virtual dMatrix CalculateInertiaAndCenterOfMass(const dMatrix& alignMatrix, const dVector& localScale, const dMatrix& matrix) const;
	

/*
	virtual void SerializeLow(dgSerialize callback, void* const userData) const;

	virtual dVector CalculateVolumeIntegral (const dMatrix& globalMatrix, const dVector& plane, const dShapeInstance& parentScale) const;
	virtual dFloat32 GetVolume () const;

	virtual dFloat32 GetBoxMinRadius () const; 
	virtual dFloat32 GetBoxMaxRadius () const;

	dInt32 RayCastClosestFace (dVector* tetrahedrum, const dVector& origin, dFloat32& pointDist) const;
	dVector CalculateVolumeIntegral (const dgPlane& plane) const; 
	
	bool SanityCheck (dgPolyhedra& hull) const;
	bool SanityCheck(dInt32 count, const dVector& normal, dVector* const contactsOut) const;

	dInt32 RectifyConvexSlice (dInt32 count, const dVector& normal, dVector* const contactsOut) const;

	virtual dVector SupportVertexSpecial (const dVector& dir, dFloat32 skinThickness, dInt32* const vertexIndex) const;
	virtual dVector SupportVertexSpecialProjectPoint (const dVector& point, const dVector& dir) const;
	virtual const dConvexSimplexEdge** GetVertexToEdgeMapping() const {return NULL;}

	dInt32 BuildCylinderCapPoly (dFloat32 radius, const dMatrix& transform, dVector* const vertexOut) const;
	
	
	friend class dgWorld;
	friend class dgBroadPhase;
	friend class dgMinkowskiConv;
	friend class dShapeCompound;
	friend class dShapeConvexModifier;
*/

	dVector* m_vertex;
	dConvexSimplexEdge* m_simplex;

	dFloat32 m_boxMinRadius;
	dFloat32 m_boxMaxRadius;
	dFloat32 m_simplexVolume;
	dUnsigned16 m_edgeCount;
	dUnsigned16 m_vertexCount;

} D_GCC_NEWTON_ALIGN_32 ;

#endif 


