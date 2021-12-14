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

#ifndef __ND_SHAPE_CONE_H__
#define __ND_SHAPE_CONE_H__

#include "ndShapeConvex.h"

#define D_CONE_SEGMENTS 12

D_MSV_NEWTON_ALIGN_32
class ndShapeCone : public ndShapeConvex
{
	public:
	D_CLASS_REFLECTION(ndShapeCone);
	D_COLLISION_API ndShapeCone(ndFloat32 radio, ndFloat32 height);
	D_COLLISION_API ndShapeCone(const ndLoadSaveBase::ndLoadDescriptor& desc);
	D_COLLISION_API ~ndShapeCone();

	virtual ndShapeCone* GetAsShapeCone() { return this; }

	protected:
	D_COLLISION_API void Init (ndFloat32 radio, ndFloat32 height);

	D_COLLISION_API virtual ndShapeInfo GetShapeInfo() const;
	D_COLLISION_API virtual void CalculateAabb(const ndMatrix& matrix, ndVector& p0, ndVector& p1) const;
	D_COLLISION_API virtual void DebugShape(const ndMatrix& matrix, ndShapeDebugNotify& debugCallback) const;
	D_COLLISION_API virtual ndVector SupportVertexSpecialProjectPoint(const ndVector& point, const ndVector& dir) const;
	D_COLLISION_API virtual ndVector SupportVertex(const ndVector& dir, ndInt32* const vertexIndex) const;
	D_COLLISION_API virtual ndVector SupportVertexSpecial(const ndVector& dir, ndFloat32 skinThickness, ndInt32* const vertexIndex) const;
	D_COLLISION_API virtual ndFloat32 RayCast(ndRayCastNotify& callback, const ndVector& localP0, const ndVector& localP1, ndFloat32 maxT, const ndBody* const body, ndContactPoint& contactOut) const;
	D_COLLISION_API virtual void Save(const ndLoadSaveBase::ndSaveDescriptor& desc) const;

	virtual ndInt32 CalculatePlaneIntersection(const ndVector& normal, const ndVector& point, ndVector* const contactsOut) const;

	ndVector m_profile[3];
	ndFloat32 m_height;
	ndFloat32 m_radius;
	ndVector m_vertex[D_CONE_SEGMENTS + 1];

	static ndInt32 m_shapeRefCount;
	static ndConvexSimplexEdge m_edgeArray[];

} D_GCC_NEWTON_ALIGN_32;

#endif 

