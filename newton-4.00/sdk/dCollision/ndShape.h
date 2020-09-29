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

#ifndef __D_SHAPE_H__ 
#define __D_SHAPE_H__ 

#include "ndCollisionStdafx.h"

class ndBody;
class ndShape;
class ndShapeBox;
class ndShapeNull;
class ndShapeSphere;
class ndShapeConvex;
class ndContactPoint;
class ndShapeCompound;
class ndRayCastNotify;
class ndShapeDebugCallback;

#ifdef _DEBUG
//	#define DG_DEBUG_AABB
#endif

enum ndShapeID
{
	// do not change the order of these enum
	m_sphereCollision = 0,
//	m_capsuleCollision,
//	m_cylinderCollision,
//	m_chamferCylinderCollision,
	m_boxCollision,
//	m_coneCollision,
//	m_convexHullCollision,
//	// this must be the last convex shape ID
	m_nullCollision,

//	m_compoundCollision,
//	m_boundingBoxHierachy,
//	m_heightField,
//	m_deformableClothPatch,
//	m_deformableSolidMesh,
//	m_userMesh,
//	m_sceneCollision,
//	m_compoundFracturedCollision,
//
//	// these are for internal use only	
//	m_contactCloud,
//	m_polygonCollision,
//	m_lumpedMassCollision
};

D_MSV_NEWTON_ALIGN_32
class ndShapeInfo
{
	public:
	class ndInstanceMaterial
	{
		public:
		ndInstanceMaterial()
		{
			memset(this, 0, sizeof(ndInstanceMaterial));
		}

		dInt64 m_userId;
		union 
		{
			void* m_userData;
			dUnsigned64 m_alignPad;
		};
		union 
		{
			dUnsigned64 m_intData;
			dFloat32 m_floatData;
		} m_userParam[6];
	};

	struct dgBoxData
	{
		dFloat32 m_x;
		dFloat32 m_y;
		dFloat32 m_z;
	};

	struct dgSphereData
	{
		dFloat32 m_radius;
	};

	struct dgCylinderData
	{
		dFloat32 m_radio0;
		dFloat32 m_radio1;
		dFloat32 m_height;
	};

	struct dgCapsuleData
	{
		dFloat32 m_radio0;
		dFloat32 m_radio1;
		dFloat32 m_height;
	};

	struct dgConeData
	{
		dFloat32 m_r;
		dFloat32 m_height;
	};

	struct dgChamferCylinderData
	{
		dFloat32 m_r;
		dFloat32 m_height;
	};

	struct dgConvexHullData
	{
		dInt32 m_vertexCount;
		dInt32 m_strideInBytes;
		dInt32 m_faceCount;
		dVector* m_vertex;
	};

	struct dgConvexModifierData
	{
		ndShape* m_child;
	};

	struct dgCoumpountCollisionData
	{
		dInt32 m_chidrenCount;
		//dgCollision** m_chidren;
	};

	struct dgCollisionBVHData
	{
		dInt32 m_vertexCount;
		dInt32 m_indexCount;
	};

	struct dgDeformableMeshData
	{
		dInt32 m_vertexCount;
		dInt32 m_triangleCount;
		dInt32 m_vertexStrideInBytes;
		dUnsigned16* m_indexList;
		dFloat32* m_vertexList;
	};

	struct dgHeightMapCollisionData
	{
		dInt32 m_width;
		dInt32 m_height;
		dInt32 m_gridsDiagonals;
		dInt32 m_elevationDataType;		// 0 = 32 bit floats, 1 = unsigned 16 bit intergers
		dFloat32 m_verticalScale;
		dFloat32 m_horizonalScale_x;
		dFloat32 m_horizonalScale_z;
		void* m_elevation;
		dInt8* m_atributes;
	};

	struct dgSceneData
	{
		dInt32 m_childrenProxyCount;
	};

	dMatrix m_offsetMatrix;
	ndInstanceMaterial m_collisionMaterial;
	ndShapeID m_collisionType;
	union
	{
		dgBoxData m_box;
		dgConeData m_cone;
		dgSphereData m_sphere;
		dgCapsuleData m_capsule;
		dgCylinderData m_cylinder;
		dgChamferCylinderData m_chamferCylinder;
		dgConvexHullData m_convexHull;
		dgDeformableMeshData m_deformableMesh;
		dgConvexModifierData m_convexModifierData;
		dgCoumpountCollisionData m_compoundCollision;
		dgCollisionBVHData m_bvhCollision;
		dgHeightMapCollisionData m_heightFieldCollision;
		dgSceneData m_sceneCollision;
		dFloat32 m_paramArray[32];
	};
} D_GCC_NEWTON_ALIGN_32;


D_MSV_NEWTON_ALIGN_32
class ndShape: public dClassAlloc
{
	public:
	const ndShape* AddRef() const;
	dInt32 GetRefCount() const;
	virtual dInt32 Release() const;

	virtual ndShapeBox* GetAsShapeBox() { return nullptr; }
	virtual ndShapeSphere* GetAsShapeSphere() { return nullptr; }
	virtual ndShapeCompound* GetAsShapeCompound() { return nullptr; }

	virtual ndShapeNull* GetAsShapeNull() { return nullptr; }
	virtual ndShapeConvex* GetAsShapeConvex() { return nullptr; }

	virtual dInt32 GetConvexVertexCount() const;

	D_COLLISION_API virtual void MassProperties();

	virtual void DebugShape(const dMatrix& matrix, ndShapeDebugCallback& debugCallback) const = 0;

	virtual ndShapeInfo GetShapeInfo() const;
	virtual void CalcAABB(const dMatrix& matrix, dVector& p0, dVector& p1) const = 0;
	virtual dVector SupportVertex(const dVector& dir, dInt32* const vertexIndex) const = 0;
	virtual dVector SupportVertexSpecialProjectPoint(const dVector& point, const dVector& dir) const = 0;
	virtual dVector SupportVertexSpecial(const dVector& dir, dFloat32 skinSkinThickness, dInt32* const vertexIndex) const = 0;
	virtual dInt32 CalculatePlaneIntersection(const dVector& normal, const dVector& point, dVector* const contactsOut) const = 0;
	virtual dFloat32 RayCast(ndRayCastNotify& callback, const dVector& localP0, const dVector& localP1, dFloat32 maxT, const ndBody* const body, ndContactPoint& contactOut) const = 0;

	virtual dMatrix CalculateInertiaAndCenterOfMass(const dMatrix& alignMatrix, const dVector& localScale, const dMatrix& matrix) const;
	virtual dFloat32 CalculateMassProperties(const dMatrix& offset, dVector& inertia, dVector& crossInertia, dVector& centerOfMass) const;

	protected:
	D_COLLISION_API ndShape(ndShapeID id);
	D_COLLISION_API ndShape (const ndShape& source);
	D_COLLISION_API virtual ~ndShape();

	dVector m_inertia;	
	dVector m_crossInertia;	
	dVector m_centerOfMass;
	dVector m_boxSize;
	dVector m_boxOrigin;
	mutable dAtomic<dInt32> m_refCount;
	ndShapeID m_collisionId;
	static dVector m_flushZero;

} D_GCC_NEWTON_ALIGN_32;

inline dInt32 ndShape::GetConvexVertexCount() const
{
	return 0;
}

inline const ndShape* ndShape::AddRef() const
{
	m_refCount.fetch_add(1);
	return this;
}

inline dInt32 ndShape::Release() const
{
	dInt32 count = m_refCount.fetch_add(-1);
	dAssert(count >= 1);
	if (count == 1) 
	{
		delete this;
	}
	return count;
}

inline dInt32 ndShape::GetRefCount() const
{
	return m_refCount.load();
}

inline dFloat32 ndShape::CalculateMassProperties(const dMatrix& offset, dVector& inertia, dVector& crossInertia, dVector& centerOfMass) const
{ 
	dAssert(0); 
	return 0; 
}

inline dMatrix ndShape::CalculateInertiaAndCenterOfMass(const dMatrix& alignMatrix, const dVector& localScale, const dMatrix& matrix) const
{
	dAssert(0);
	return dGetZeroMatrix();
}


#endif 


