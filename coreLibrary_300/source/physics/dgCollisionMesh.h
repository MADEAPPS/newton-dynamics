/* Copyright (c) <2003-2011> <Julio Jerez, Newton Game Dynamics>
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

#ifndef __DGCOLLISION_MESH_H__
#define __DGCOLLISION_MESH_H__


#include "dgCollision.h"
#include "dgCollisionConvex.h"
#include "dgCollisionInstance.h"


#define DG_MAX_COLLIDING_FACES			512
#define DG_MAX_COLLIDING_INDICES		(DG_MAX_COLLIDING_FACES * (4 * 2 + 3))


class dgCollisionMesh;
typedef void (*dgCollisionMeshCollisionCallback) (const dgBody* const bodyWithTreeCollision, const dgBody* const body, dgInt32 faceID, 
												  dgInt32 vertexCount, const dgFloat32* const vertex, dgInt32 vertexStrideInBytes); 



DG_MSC_VECTOR_ALIGMENT 
class dgPolygonMeshDesc: public dgFastAABBInfo
{
	public:
	class dgMesh
	{
		public:
		dgInt32 m_globalFaceIndexCount[DG_MAX_COLLIDING_FACES];
		dgInt32 m_globalFaceIndexStart[DG_MAX_COLLIDING_FACES];
		dgFloat32 m_globalHitDistance[DG_MAX_COLLIDING_FACES];
	};

	// colliding box in polygonSoup local space
	DG_INLINE dgPolygonMeshDesc()
		:dgFastAABBInfo()
		,m_boxDistanceTravelInMeshSpace(dgFloat32 (0.0f))
		,m_maxT(dgFloat32 (1.0f))
		,m_doContinuesCollisionTest(false)
	{
	}

	dgPolygonMeshDesc(dgCollisionParamProxy& proxy, void* const userData);

	DG_INLINE void SetDistanceTravel (const dgVector& distanceInGlobalSpace)
	{
		//const dgMatrix& soupMatrix = data.m_polySoupCollision->GetGlobalMatrix();
		const dgMatrix& soupMatrix = m_polySoupCollision->GetGlobalMatrix();
		//data.m_boxDistanceTravelInMeshSpace = data.m_polySoupCollision->GetInvScale().CompProduct4(soupMatrix.UnrotateVector(upperBoundVeloc.CompProduct4(data.m_objCollision->GetInvScale())));
		m_boxDistanceTravelInMeshSpace = m_polySoupCollision->GetInvScale().CompProduct4(soupMatrix.UnrotateVector(distanceInGlobalSpace.CompProduct4(m_objCollision->GetInvScale())));
	}


	DG_INLINE dgInt32 GetFaceIndexCount(dgInt32 indexCount) const
	{
		return indexCount * 2 + 3;
	}

	DG_INLINE const dgInt32* GetAdjacentFaceEdgeNormalArray(const dgInt32* const faceIndexArray, dgInt32 indexCount) const
	{
		return &faceIndexArray[indexCount + 2];
	}


	DG_INLINE dgInt32 GetNormalIndex(const dgInt32* const faceIndexArray, dgInt32 indexCount) const
	{
		return faceIndexArray[indexCount + 1];
	}

	DG_INLINE dgInt32 GetFaceId(const dgInt32* const faceIndexArray, dgInt32 indexCount) const
	{
		return faceIndexArray[indexCount];
	}

	DG_INLINE dgFloat32 GetFaceSize(const dgInt32* const faceIndexArray, dgInt32 indexCount) const
	{
		dgInt32 size = faceIndexArray[indexCount * 2 + 2];
		return dgFloat32 ((size >= 1) ? size : dgFloat32 (1.0f));
	}

	void SortFaceArray ();

	dgVector m_boxDistanceTravelInMeshSpace;
	dgInt32 m_threadNumber;
	dgInt32 m_faceCount;
	dgInt32 m_vertexStrideInBytes;
	dgFloat32 m_skinThickness;
	void* m_userData;
	dgBody *m_objBody;
	dgBody *m_polySoupBody;
	dgCollisionInstance* m_objCollision;
	dgCollisionInstance* m_polySoupCollision;
	dgFloat32* m_vertex;
	dgInt32* m_faceIndexCount;
	dgInt32* m_faceVertexIndex;

	// private data;
	dgInt32* m_faceIndexStart;
	dgFloat32* m_hitDistance;
	const dgCollisionMesh* m_me;
	dgInt32 m_globalIndexCount;
	dgFloat32 m_maxT;
	bool m_doContinuesCollisionTest;
	dgInt32 m_globalFaceVertexIndex[DG_MAX_COLLIDING_INDICES];
	dgMesh m_meshData;
} DG_GCC_VECTOR_ALIGMENT;

DG_MSC_VECTOR_ALIGMENT 
class dgCollisionMeshRayHitDesc
{
	public:
	dgCollisionMeshRayHitDesc ()
		:m_matrix (dgGetIdentityMatrix())
	{
	}

	dgVector m_localP0; 
	dgVector m_localP1; 
	dgVector m_normal;
	dgUnsigned64 m_userId;
	void*  m_userData;
	void*  m_altenateUserData;
	dgMatrix m_matrix;
}DG_GCC_VECTOR_ALIGMENT;



class dgCollisionMesh: public dgCollision  
{
	public:

	DG_MSC_VECTOR_ALIGMENT 
	class dgMeshVertexListIndexList
	{
		public:
		dgInt32* m_indexList;
		dgInt32* m_userDataList;
		dgFloat32* m_veterxArray;
		dgInt32 m_triangleCount; 
		dgInt32 m_maxIndexCount;
		dgInt32 m_vertexCount;
		dgInt32 m_vertexStrideInBytes;
	}DG_GCC_VECTOR_ALIGMENT;


	dgCollisionMesh (dgWorld* const world, dgCollisionID type);
	dgCollisionMesh (dgWorld* const world, dgDeserialize deserialization, void* const userData);
	virtual ~dgCollisionMesh();

	


	virtual dgFloat32 GetVolume () const;
	virtual dgFloat32 GetBoxMinRadius () const; 
	virtual dgFloat32 GetBoxMaxRadius () const;
	virtual void GetVertexListIndexList (const dgVector& p0, const dgVector& p1, dgMeshVertexListIndexList &data) const = 0;

	virtual void GetCollidingFaces (dgPolygonMeshDesc* const data) const = 0;

	void SetDebugCollisionCallback (dgCollisionMeshCollisionCallback debugCallback);
	dgCollisionMeshCollisionCallback GetDebugCollisionCallback() const { return m_debugCallback;} 

	protected:
	virtual void SetCollisionBBox (const dgVector& p0, const dgVector& p1);

	private:
	virtual dgInt32 CalculateSignature () const;
	virtual dgVector SupportVertex (const dgVector& dir, dgInt32* const vertexIndex) const;

	virtual void CalcAABB (const dgMatrix& matrix, dgVector& p0, dgVector& p1) const;
	virtual void DebugCollision  (const dgMatrix& matrix, dgCollision::OnDebugCollisionMeshCallback callback, void* const userData) const;

	virtual dgVector CalculateVolumeIntegral (const dgMatrix& globalMatrix, const dgVector& plane, const dgCollisionInstance& parentScale) const;
	virtual dgFloat32 RayCast (const dgVector& localP0, const dgVector& localP1, dgFloat32 maxT, dgContactPoint& contactOut, const dgBody* const body, void* const userData, OnRayPrecastAction preFilter) const = 0;
	virtual dgFloat32 ConvexRayCast (const dgCollisionInstance* const convexShape, const dgMatrix& origin, const dgVector& veloc, dgFloat32 minT, dgContactPoint& contactOut, const dgBody* const referenceBody, const dgCollisionInstance* const referenceInstance, void* const userData, dgInt32 threadId) const;

	dgInt32 CalculatePlaneIntersection (const dgFloat32* const vertex, const dgInt32* const index, dgInt32 indexCount, dgInt32 strideInFloat, const dgPlane& localPlane, dgVector* const contactsOut) const;

	dgInt32 CalculatePlaneIntersection (const dgVector& normal, const dgVector& point, dgVector* const contactsOut, dgFloat32 normalSign) const;

	virtual void GetCollisionInfo(dgCollisionInfo* const info) const;
	virtual void Serialize(dgSerialize callback, void* const userData) const;


#ifdef DG_DEBUG_AABB
	dgVector BoxSupportMapping  (const dgVector& dir) const;
#endif

	protected:
	dgCollisionMeshCollisionCallback m_debugCallback;


	friend class dgWorld;
	friend class dgCollisionInstance;
};



#endif 

