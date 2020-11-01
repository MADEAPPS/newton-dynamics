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

class ndBodyKinematic;
class ndShapeStaticBVH;
typedef dFloat32 (*dgCollisionBVHUserRayCastCallback) (const ndBodyKinematic* const body, const ndShapeStaticBVH* const collsionShape, dFloat32 interception, dFloat32* normal, dInt32 faceId, void* usedData);

class ndShapeStaticBVH : public ndShapeStaticMesh, public dAabbPolygonSoup
{
	public:
	D_COLLISION_API ndShapeStaticBVH(const dPolygonSoupBuilder& builder);
	D_COLLISION_API virtual ~ndShapeStaticBVH();

	protected:
	virtual ndShapeInfo GetShapeInfo() const;
	virtual void DebugShape(const dMatrix& matrix, ndShapeDebugCallback& debugCallback) const;
	virtual dFloat32 RayCast(ndRayCastNotify& callback, const dVector& localP0, const dVector& localP1, const ndBody* const body, ndContactPoint& contactOut) const;
	virtual void GetCollidingFaces(ndPolygonMeshDesc* const data) const;
	
	static dFloat32 RayHit(void* const context, const dFloat32* const polygon, dInt32 strideInBytes, const dInt32* const indexArray, dInt32 indexCount);
	static dIntersectStatus ShowDebugPolygon(void* const context, const dFloat32* const polygon, dInt32 strideInBytes, const dInt32* const indexArray, dInt32 indexCount, dFloat32 hitDistance);
	static dIntersectStatus GetTriangleCount(void* const context, const dFloat32* const polygon, dInt32 strideInBytes, const dInt32* const indexArray, dInt32 indexCount, dFloat32 hitDistance);
	static dIntersectStatus GetPolygon(void* const context, const dFloat32* const polygon, dInt32 strideInBytes, const dInt32* const indexArray, dInt32 indexCount, dFloat32 hitDistance);
	public:
	D_MSV_NEWTON_ALIGN_32 
	class ndBvhRay: public dFastRayTest 
	{
		public:
		ndBvhRay(const dVector& l0, const dVector& l1)
			:dFastRayTest (l0, l1)
		{
		}

		dMatrix m_matrix;
		dVector m_normal;
		dUnsigned32 m_id;
		dFloat32 m_t;
		ndRayCastNotify* m_callback;
		const ndBodyKinematic* m_myBody;
		const ndShapeStaticBVH* m_me;
	} D_GCC_NEWTON_ALIGN_32;

#if 0	
	void GetVertexListIndexList (const dVector& p0, const dVector& p1, ndMeshVertexListIndexList &data) const;
	void ForEachFace (dgAABBIntersectCallback callback, void* const context) const;

	private:
	
	static dFloat32 RayHitUser (void* const context, const dFloat32* const polygon, dInt32 strideInBytes, const dInt32* const indexArray, dInt32 indexCount);
	static dIntersectStatus CollectVertexListIndexList (void* const context, const dFloat32* const polygon, dInt32 strideInBytes, const dInt32* const indexArray, dInt32 indexCount, dFloat32 hitDistance);
	virtual dVector SupportVertex (const dVector& dir) const;

	virtual void GetLocalAABB (const dVector& p0, const dVector& p1, dVector& boxP0, dVector& boxP1) const;

	virtual dVector SupportVertex (const dVector& dir, dInt32* const vertexIndex) const;
	virtual dVector SupportVertexSpecial (const dVector& dir, dFloat32 skinThickness, dInt32* const vertexIndex) const;
	virtual dVector SupportVertexSpecialProjectPoint (const dVector& point, const dVector& dir) const {return point;}
	friend class dgCollisionCompound;
	friend class dgCollisionDeformableMesh;
#endif

	private: 
	D_COLLISION_API virtual void Save(nd::TiXmlElement* const xmlNode, const char* const assetPath, dInt32 nodeid) const;
	dInt32 m_trianglesCount;
};


#endif
