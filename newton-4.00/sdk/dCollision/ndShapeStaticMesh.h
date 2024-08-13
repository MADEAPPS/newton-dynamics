/* Copyright (c) <2003-2022> <Julio Jerez, Newton Game Dynamics>
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

#ifndef __ND_SHAPE_STATIC_MESH_H__
#define __ND_SHAPE_STATIC_MESH_H__

#include "ndCollisionStdafx.h"
#include "ndShape.h"

class ndShapeInstance;
class ndPolygonMeshDesc;

class ndShapeStaticMesh: public ndShape
{
	public:
	D_CLASS_REFLECTION(ndShapeStaticMesh,ndShape)
	D_COLLISION_API ndShapeStaticMesh(ndShapeID id);
	D_COLLISION_API virtual ~ndShapeStaticMesh();

	virtual void DebugShape(const ndMatrix& matrix, ndShapeDebugNotify& debugCallback) const;
	virtual ndFloat32 RayCast(ndRayCastNotify& callback, const ndVector& localP0, const ndVector& localP1, ndFloat32 maxT, const ndBody* const body, ndContactPoint& contactOut) const;
	virtual void GetCollidingFaces(ndPolygonMeshDesc* const data) const;

	protected:
	virtual ndFloat32 GetVolume() const;
	virtual ndFloat32 GetBoxMinRadius() const;
	virtual ndFloat32 GetBoxMaxRadius() const;
	virtual ndShapeStaticMesh* GetAsShapeStaticMesh();
	virtual ndVector SupportVertex(const ndVector& dir) const;
	virtual ndVector SupportVertexSpecial(const ndVector& dir, ndFloat32 skinMargin) const;
	virtual ndVector SupportVertexSpecialProjectPoint(const ndVector& point, const ndVector& dir) const;
	virtual ndInt32 CalculatePlaneIntersection(const ndVector& normal, const ndVector& point, ndVector* const contactsOut) const;
	virtual ndVector CalculateVolumeIntegral(const ndMatrix& globalMatrix, const ndVector& plane, const ndShapeInstance& parentScale) const;

	D_COLLISION_API virtual void CalculateAabb(const ndMatrix& matrix, ndVector& p0, ndVector& p1) const;
	D_COLLISION_API ndInt32 CalculatePlaneIntersection(const ndFloat32* const vertex, const ndInt32* const index, ndInt32 indexCount, ndInt32 strideInFloat, const ndPlane& localPlane, ndVector* const contactsOut) const;

	D_MSV_NEWTON_ALIGN_32 
	class ndMeshVertexListIndexList
	{
		public:
		ndInt32* m_indexList;
		ndInt32* m_userDataList;
		ndFloat32* m_veterxArray;
		ndInt32 m_triangleCount; 
		ndInt32 m_maxIndexCount;
		ndInt32 m_vertexCount;
		ndInt32 m_vertexStrideInBytes;
	} D_GCC_NEWTON_ALIGN_32;
};

inline ndFloat32 ndShapeStaticMesh::GetVolume() const
{
	return ndFloat32(0.0f);
}

inline ndFloat32 ndShapeStaticMesh::GetBoxMinRadius() const
{
	return ndFloat32(0.0f);
}

inline ndFloat32 ndShapeStaticMesh::GetBoxMaxRadius() const
{
	return ndFloat32(0.0f);
}

inline ndVector ndShapeStaticMesh::SupportVertex(const ndVector&) const
{
	ndAssert(0);
	return ndVector::m_zero;
}

inline ndVector ndShapeStaticMesh::SupportVertexSpecial(const ndVector& dir, ndFloat32) const
{
	ndAssert(0);
	return SupportVertex(dir);
}

inline ndVector ndShapeStaticMesh::SupportVertexSpecialProjectPoint(const ndVector& point, const ndVector&) const
{ 
	return point; 
}

inline ndInt32 ndShapeStaticMesh::CalculatePlaneIntersection(const ndVector&, const ndVector&, ndVector* const) const
{
	return 0;
}

inline ndVector ndShapeStaticMesh::CalculateVolumeIntegral(const ndMatrix&, const ndVector&, const ndShapeInstance&) const
{
	return ndVector::m_zero;
}

inline ndShapeStaticMesh* ndShapeStaticMesh::GetAsShapeStaticMesh()
{ 
	return this; 
}


inline void ndShapeStaticMesh::DebugShape(const ndMatrix&, ndShapeDebugNotify&) const
{
}

inline ndFloat32 ndShapeStaticMesh::RayCast(ndRayCastNotify&, const ndVector&, const ndVector&, ndFloat32, const ndBody* const, ndContactPoint&) const
{
	return ndFloat32(1.2f);
}

inline void ndShapeStaticMesh::GetCollidingFaces(ndPolygonMeshDesc* const) const
{
}

#endif 



