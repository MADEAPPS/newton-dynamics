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

#ifndef __D_SHAPE_BOX_H__
#define __D_SHAPE_BOX_H__

#include "dShapeConvex.h"

D_MSC_VECTOR_ALIGNMENT
class dShapeBox: public dShapeConvex
{
	public:
	D_NEWTON_API dShapeBox(dFloat32 size_x, dFloat32 size_y, dFloat32 size_z);
	//dShapeBox(dgWorld* const world, dgDeserialize deserialization, void* const userData, dInt32 revisionNumber);
	virtual ~dShapeBox();

	virtual dShapeBox* GetAsShapeBox() { return this; }

	protected:
	D_NEWTON_API void Init (dFloat32 size_x, dFloat32 size_y, dFloat32 size_z);
	D_NEWTON_API virtual void MassProperties();
	D_NEWTON_API virtual dVector SupportVertex(const dVector& dir, dInt32* const vertexIndex) const;
/*
	virtual void CalcAABB (const dgMatrix& matrix, dVector& p0, dVector& p1) const;
	virtual dFloat32 RayCast (const dVector& localP0, const dVector& localP1, dFloat32 maxT, dgContactPoint& contactOut, const dgBody* const body, void* const userData, OnRayPrecastAction preFilter) const;
	virtual dVector SupportVertexSpecial(const dVector& dir, dFloat32 skinThickness, dInt32* const vertexIndex) const;
	virtual dVector SupportVertexSpecialProjectPoint(const dVector& point, const dVector& dir) const;
	virtual dInt32 CalculatePlaneIntersection (const dVector& normal, const dVector& point, dVector* const contactsOut) const;
	virtual const dConvexSimplexEdge** GetVertexToEdgeMapping() const;
	
	virtual dInt32 CalculateSignature () const;
	virtual void SetCollisionBBox (const dVector& p0, const dVector& p1);
	

	virtual void GetCollisionInfo(dgCollisionInfo* const info) const;
	virtual void Serialize(dgSerialize callback, void* const userData) const;

	static dInt32 CalculateSignature (dFloat32 dx, dFloat32 dy, dFloat32 dz);
*/

	dVector m_size[2];
	dVector m_vertex[8];

	static dInt32 m_initSimplex;
	static dInt32 m_faces[][4];
	static dVector m_indexMark;
	//static dVector m_penetrationTol;
	static dConvexSimplexEdge m_edgeArray[];
	static dConvexSimplexEdge* m_edgeEdgeMap[];
	static dConvexSimplexEdge* m_vertexToEdgeMap[];

} D_GCC_VECTOR_ALIGNMENT;

#endif 

