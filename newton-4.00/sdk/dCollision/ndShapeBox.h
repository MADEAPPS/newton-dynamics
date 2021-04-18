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

#include "ndShapeConvex.h"

D_MSV_NEWTON_ALIGN_32
class ndShapeBox: public ndShapeConvex
{
	public:
	D_COLLISION_API ndShapeBox(const nd::TiXmlNode* const xmlNode);
	D_COLLISION_API ndShapeBox(dFloat32 size_x, dFloat32 size_y, dFloat32 size_z);
	D_COLLISION_API virtual ~ndShapeBox();

	virtual ndShapeBox* GetAsShapeBox() { return this; }

	protected:
	D_COLLISION_API void Init (dFloat32 size_x, dFloat32 size_y, dFloat32 size_z);
	D_COLLISION_API virtual void MassProperties();

	D_COLLISION_API virtual ndShapeInfo GetShapeInfo() const;
	D_COLLISION_API virtual void CalcAABB(const dMatrix& matrix, dVector& p0, dVector& p1) const;
	D_COLLISION_API virtual dVector SupportVertexSpecialProjectPoint(const dVector& point, const dVector& dir) const;
	D_COLLISION_API virtual dVector SupportVertex(const dVector& dir, dInt32* const vertexIndex) const;
	D_COLLISION_API virtual dVector SupportVertexSpecial(const dVector& dir, dFloat32 skinThickness, dInt32* const vertexIndex) const;
	D_COLLISION_API virtual dFloat32 RayCast(ndRayCastNotify& callback, const dVector& localP0, const dVector& localP1, dFloat32 maxT, const ndBody* const body, ndContactPoint& contactOut) const;

	const ndConvexSimplexEdge** GetVertexToEdgeMapping() const;
	virtual dInt32 CalculatePlaneIntersection(const dVector& normal, const dVector& point, dVector* const contactsOut) const;
	D_COLLISION_API virtual void Save(nd::TiXmlElement* const xmlNode, const char* const assetPath, dInt32 nodeid) const;

	dVector m_size[2];
	dVector m_vertex[8];

	static dInt32 m_initSimplex;
	static dInt32 m_faces[][4];
	static dVector m_indexMark;
	static dVector m_penetrationTol;
	static ndConvexSimplexEdge m_edgeArray[];
	static ndConvexSimplexEdge* m_edgeEdgeMap[];
	static ndConvexSimplexEdge* m_vertexToEdgeMap[];

} D_GCC_NEWTON_ALIGN_32;

#endif 

