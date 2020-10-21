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

#ifndef __D_SHAPE_STATIC_BVH__
#define __D_SHAPE_STATIC_BVH__

#include "ndCollisionStdafx.h"
#include "ndShapeStaticMesh.h"

//class ndShapeStaticBVH;
//typedef dFloat32 (*dgCollisionBVHUserRayCastCallback) (const dgBody* const body, const ndShapeStaticBVH* const heightFieldCollision, dFloat32 interception, dFloat32* normal, dInt32 faceId, void* usedData);

//class ndShapeStaticBVH: public ndShapeStaticMesh, public dgAABBPolygonSoup
class ndShapeStaticBVH : public ndShapeStaticMesh
{
	public:
	D_COLLISION_API ndShapeStaticBVH();
	D_COLLISION_API virtual ~ndShapeStaticBVH(void);

	protected:
	virtual void DebugShape(const dMatrix& matrix, ndShapeDebugCallback& debugCallback) const;

	//virtual dFloat32 RayCast(const dVector& localP0, const dVector& localP1, dFloat32 maxT, dgContactPoint& contactOut, const dgBody* const body, void* const userData, OnRayPrecastAction preFilter) const;
	D_COLLISION_API virtual dFloat32 RayCast(ndRayCastNotify& callback, const dVector& localP0, const dVector& localP1, dFloat32 maxT, const ndBody* const body, ndContactPoint& contactOut) const;
#if 0
	public:
	D_MSV_NEWTON_ALIGN_32 
	class dgBVHRay: public dgFastRayTest 
	{
		public:
		dgBVHRay(const dVector& l0, const dVector& l1)
			:dgFastRayTest (l0, l1)
		{
		}

		dMatrix m_matrix;
		dVector m_normal;
		dgUnsigned32 m_id;
		dFloat32 m_t;
		void* m_userData;
		const dgBody* m_myBody;
		const ndShapeStaticBVH* m_me;
	} D_GCC_NEWTON_ALIGN_32;

	
	ndShapeStaticBVH (dgWorld* const world, dgDeserialize deserialization, void* const userData, dInt32 revisionNumber);
	

	void BeginBuild();
	void AddFace (dInt32 vertexCount, const dFloat32* const vertexPtr, dInt32 strideInBytes, dInt32 faceAttribute);
	void EndBuild(dInt32 optimize);

	void SetCollisionRayCastCallback (dgCollisionBVHUserRayCastCallback rayCastCallback);
	dgCollisionBVHUserRayCastCallback GetDebugRayCastCallback() const { return m_userRayCastCallback;} 
	void GetVertexListIndexList (const dVector& p0, const dVector& p1, dgMeshVertexListIndexList &data) const;

	void ForEachFace (dgAABBIntersectCallback callback, void* const context) const;

	private:
	static dFloat32 RayHit (void* const context, const dFloat32* const polygon, dInt32 strideInBytes, const dInt32* const indexArray, dInt32 indexCount);
	static dFloat32 RayHitUser (void* const context, const dFloat32* const polygon, dInt32 strideInBytes, const dInt32* const indexArray, dInt32 indexCount);
	static dgIntersectStatus GetPolygon (void* const context, const dFloat32* const polygon, dInt32 strideInBytes, const dInt32* const indexArray, dInt32 indexCount, dFloat32 hitDistance);
	static dgIntersectStatus ShowDebugPolygon (void* const context, const dFloat32* const polygon, dInt32 strideInBytes, const dInt32* const indexArray, dInt32 indexCount, dFloat32 hitDistance);
	static dgIntersectStatus GetTriangleCount (void* const context, const dFloat32* const polygon, dInt32 strideInBytes, const dInt32* const indexArray, dInt32 indexCount, dFloat32 hitDistance);
	static dgIntersectStatus CollectVertexListIndexList (void* const context, const dFloat32* const polygon, dInt32 strideInBytes, const dInt32* const indexArray, dInt32 indexCount, dFloat32 hitDistance);

	void Serialize(dgSerialize callback, void* const userData) const;
	virtual dVector SupportVertex (const dVector& dir) const;
	
	virtual void GetCollidingFaces (dgPolygonMeshDesc* const data) const;
	virtual void GetCollisionInfo(dgCollisionInfo* const info) const;

	virtual void GetLocalAABB (const dVector& p0, const dVector& p1, dVector& boxP0, dVector& boxP1) const;

	virtual dVector SupportVertex (const dVector& dir, dInt32* const vertexIndex) const;
	virtual dVector SupportVertexSpecial (const dVector& dir, dFloat32 skinThickness, dInt32* const vertexIndex) const;
	virtual dVector SupportVertexSpecialProjectPoint (const dVector& point, const dVector& dir) const {return point;}

	dgPolygonSoupDatabaseBuilder* m_builder;
	dgCollisionBVHUserRayCastCallback m_userRayCastCallback;

	dInt32 m_trianglesCount;
	friend class dgCollisionCompound;
	friend class dgCollisionDeformableMesh;
#endif
};


#endif
