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

#ifndef __ND_SHAPE_STATIC_BVH__
#define __ND_SHAPE_STATIC_BVH__

#include "ndCollisionStdafx.h"
#include "ndShapeStaticMesh.h"

class ndShapeStatic_bvh: public ndShapeStaticMesh, public ndAabbPolygonSoup
{
	public:
	D_CLASS_REFLECTION(ndShapeStatic_bvh);
	D_COLLISION_API ndShapeStatic_bvh(const ndPolygonSoupBuilder& builder);
	D_COLLISION_API ndShapeStatic_bvh(const ndLoadSaveBase::dLoadDescriptor& desc);
	D_COLLISION_API virtual ~ndShapeStatic_bvh();

	protected:
	virtual ndShapeInfo GetShapeInfo() const;
	virtual ndShapeStatic_bvh* GetAsShapeStaticBVH() { return this; }
	virtual void DebugShape(const ndMatrix& matrix, ndShapeDebugNotify& debugCallback) const;
	virtual dFloat32 RayCast(ndRayCastNotify& callback, const ndVector& localP0, const ndVector& localP1, dFloat32 maxT, const ndBody* const body, ndContactPoint& contactOut) const;
	virtual void GetCollidingFaces(ndPolygonMeshDesc* const data) const;
	virtual void Save(const ndLoadSaveBase::ndSaveDescriptor& desc) const;
	
	static dFloat32 RayHit(void* const context, const dFloat32* const polygon, dInt32 strideInBytes, const dInt32* const indexArray, dInt32 indexCount);
	static dIntersectStatus ShowDebugPolygon(void* const context, const dFloat32* const polygon, dInt32 strideInBytes, const dInt32* const indexArray, dInt32 indexCount, dFloat32 hitDistance);
	static dIntersectStatus GetTriangleCount(void* const context, const dFloat32* const polygon, dInt32 strideInBytes, const dInt32* const indexArray, dInt32 indexCount, dFloat32 hitDistance);
	static dIntersectStatus GetPolygon(void* const context, const dFloat32* const polygon, dInt32 strideInBytes, const dInt32* const indexArray, dInt32 indexCount, dFloat32 hitDistance);

	private: 
	dInt32 m_trianglesCount;

	friend class ndContactSolver;
};


#endif
