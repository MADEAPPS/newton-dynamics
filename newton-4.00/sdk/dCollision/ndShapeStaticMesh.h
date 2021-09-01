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

#ifndef __D_SHAPE_STATIC_MESH_H__
#define __D_SHAPE_STATIC_MESH_H__

#include "ndCollisionStdafx.h"
#include "ndShape.h"

class ndBodyKinematic;
class ndShapeInstance;
class ndContactSolver;

#define D_MAX_COLLIDING_FACES		512
#define D_MAX_COLLIDING_INDICES		(D_MAX_COLLIDING_FACES * (4 * 2 + 3))


class ndShapeStaticMesh;
typedef void (*dgCollisionMeshCollisionCallback) (const ndBodyKinematic* const bodyWithTreeCollision, const ndBodyKinematic* const body, dInt32 faceID, 
												  dInt32 vertexCount, const dFloat32* const vertex, dInt32 vertexStrideInBytes); 

D_MSV_NEWTON_ALIGN_32 
class ndPolygonMeshDesc: public dFastAabb
{
	public:
	class ndMesh
	{
		public:
		dInt32 m_globalFaceIndexCount[D_MAX_COLLIDING_FACES];
		dInt32 m_globalFaceIndexStart[D_MAX_COLLIDING_FACES];
		dFloat32 m_globalHitDistance[D_MAX_COLLIDING_FACES];
	};

	// colliding box in polygonSoup local space
	ndPolygonMeshDesc()
		:dFastAabb()
		,m_boxDistanceTravelInMeshSpace(dFloat32 (0.0f))
		,m_maxT(dFloat32 (1.0f))
		,m_doContinuesCollisionTest(false)
	{
	}

	D_COLLISION_API ndPolygonMeshDesc(ndContactSolver& proxy, bool ccdMode);

	D_COLLISION_API void SortFaceArray();
	dFloat32 GetSeparetionDistance() const;
	void SetDistanceTravel(const dVector& distanceInGlobalSpace);

	dInt32 GetFaceIndexCount(dInt32 indexCount) const
	{
		return indexCount * 2 + 3;
	}

	const dInt32* GetAdjacentFaceEdgeNormalArray(const dInt32* const faceIndexArray, dInt32 indexCount) const
	{
		return &faceIndexArray[indexCount + 2];
	}

	dInt32 GetNormalIndex(const dInt32* const faceIndexArray, dInt32 indexCount) const
	{
		return faceIndexArray[indexCount + 1];
	}

	dInt32 GetFaceId(const dInt32* const faceIndexArray, dInt32 indexCount) const
	{
		return faceIndexArray[indexCount];
	}

	dFloat32 GetFaceSize(const dInt32* const faceIndexArray, dInt32 indexCount) const
	{
		dInt32 size = faceIndexArray[indexCount * 2 + 2];
		return dFloat32 ((size >= 1) ? size : dFloat32 (1.0f));
	}

	dVector m_boxDistanceTravelInMeshSpace;
	dInt32 m_faceCount;
	dInt32 m_vertexStrideInBytes;
	dFloat32 m_skinThickness;
	void* m_userData;
	ndShapeInstance* m_convexInstance;
	ndShapeInstance* m_polySoupInstance;
	dFloat32* m_vertex;
	dInt32* m_faceIndexCount;
	dInt32* m_faceVertexIndex;

	// private data;
	dInt32* m_faceIndexStart;
	dFloat32* m_hitDistance;
	const ndShapeStaticMesh* m_me;
	dInt32 m_globalIndexCount;
	dFloat32 m_maxT;
	bool m_doContinuesCollisionTest;
	dInt32 m_globalFaceVertexIndex[D_MAX_COLLIDING_INDICES];
	ndMesh m_meshData;
} D_GCC_NEWTON_ALIGN_32;

class ndShapeStaticMesh: public ndShape
{
	public:
	D_CLASS_REFLECTION(ndShapeStaticMesh);
	D_COLLISION_API ndShapeStaticMesh(ndShapeID id);
	D_COLLISION_API ndShapeStaticMesh(const dLoadSaveBase::dLoadDescriptor& desc);
	D_COLLISION_API virtual ~ndShapeStaticMesh();

	virtual void GetCollidingFaces(ndPolygonMeshDesc* const data) const;

	protected:
	virtual dFloat32 GetVolume() const;
	virtual dFloat32 GetBoxMinRadius() const;
	virtual dFloat32 GetBoxMaxRadius() const;
	virtual ndShapeStaticMesh* GetAsShapeStaticMesh();
	virtual dVector SupportVertex(const dVector& dir, dInt32* const vertexIndex) const;
	virtual dVector SupportVertexSpecial(const dVector& dir, dFloat32 skinThickness, dInt32* const vertexIndex) const;
	virtual dVector SupportVertexSpecialProjectPoint(const dVector& point, const dVector& dir) const;
	virtual dInt32 CalculatePlaneIntersection(const dVector& normal, const dVector& point, dVector* const contactsOut) const;
	virtual dVector CalculateVolumeIntegral(const dMatrix& globalMatrix, const dVector& plane, const ndShapeInstance& parentScale) const;

	D_COLLISION_API virtual void CalculateAabb(const dMatrix& matrix, dVector& p0, dVector& p1) const;
	D_COLLISION_API dInt32 CalculatePlaneIntersection(const dFloat32* const vertex, const dInt32* const index, dInt32 indexCount, dInt32 strideInFloat, const dPlane& localPlane, dVector* const contactsOut) const;
	D_COLLISION_API void Save(const dLoadSaveBase::dSaveDescriptor& desc) const;

	virtual void DebugShape(const dMatrix& matrix, ndShapeDebugCallback& debugCallback) const;
	virtual dFloat32 RayCast(ndRayCastNotify& callback, const dVector& localP0, const dVector& localP1, dFloat32 maxT, const ndBody* const body, ndContactPoint& contactOut) const;

	D_MSV_NEWTON_ALIGN_32 
	class ndMeshVertexListIndexList
	{
		public:
		dInt32* m_indexList;
		dInt32* m_userDataList;
		dFloat32* m_veterxArray;
		dInt32 m_triangleCount; 
		dInt32 m_maxIndexCount;
		dInt32 m_vertexCount;
		dInt32 m_vertexStrideInBytes;
	} D_GCC_NEWTON_ALIGN_32;
};

inline dFloat32 ndShapeStaticMesh::GetVolume() const
{
	return dFloat32(0.0f);
}

inline dFloat32 ndShapeStaticMesh::GetBoxMinRadius() const
{
	return dFloat32(0.0f);
}

inline dFloat32 ndShapeStaticMesh::GetBoxMaxRadius() const
{
	return dFloat32(0.0f);
}

inline dVector ndShapeStaticMesh::SupportVertex(const dVector&, dInt32* const) const
{
	dAssert(0);
	return dVector::m_zero;
}

inline dVector ndShapeStaticMesh::SupportVertexSpecial(const dVector& dir, dFloat32, dInt32* const vertexIndex) const
{
	dAssert(0);
	return SupportVertex(dir, vertexIndex);
}

inline dVector ndShapeStaticMesh::SupportVertexSpecialProjectPoint(const dVector& point, const dVector&) const
{ 
	return point; 
}

inline dInt32 ndShapeStaticMesh::CalculatePlaneIntersection(const dVector&, const dVector&, dVector* const) const
{
	return 0;
}

inline dVector ndShapeStaticMesh::CalculateVolumeIntegral(const dMatrix&, const dVector&, const ndShapeInstance&) const
{
	return dVector::m_zero;
}

inline ndShapeStaticMesh* ndShapeStaticMesh::GetAsShapeStaticMesh()
{ 
	return this; 
}

inline dFloat32 ndPolygonMeshDesc::GetSeparetionDistance() const
{
	return m_separationDistance[0] * m_polySoupInstance->GetScale().GetScalar();
}

inline void ndPolygonMeshDesc::SetDistanceTravel(const dVector& distanceInGlobalSpace)
{
	const dMatrix& soupMatrix = m_polySoupInstance->GetGlobalMatrix();
	m_boxDistanceTravelInMeshSpace = m_polySoupInstance->GetInvScale() * soupMatrix.UnrotateVector(distanceInGlobalSpace * m_convexInstance->GetInvScale());
	if (m_boxDistanceTravelInMeshSpace.DotProduct(m_boxDistanceTravelInMeshSpace).GetScalar() < dFloat32(1.0e-2f))
	{
		m_doContinuesCollisionTest = false;
	}
}

inline void ndShapeStaticMesh::DebugShape(const dMatrix&, ndShapeDebugCallback&) const
{
}

inline dFloat32 ndShapeStaticMesh::RayCast(ndRayCastNotify&, const dVector&, const dVector&, dFloat32, const ndBody* const, ndContactPoint&) const
{
	return dFloat32(1.2f);
}

inline void ndShapeStaticMesh::GetCollidingFaces(ndPolygonMeshDesc* const) const
{
}

#endif 



