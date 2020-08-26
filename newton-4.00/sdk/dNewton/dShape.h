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

//class dgBody;
//class dgWorld;
//class dShape;
//class dgMeshEffect;
//class dgContactPoint;
//class dgPolygonSoupDesc;
//class dShapeConvex;
//class dgPolygonMeshDesc;
//class dShapeInstance;
//class dShapeConvexHull;
//class dgPolygonSoupRayHitDesc;

class dShapeBox;
class dShapeNull;
class dShapeConvex;

#ifdef _DEBUG
//	#define DG_DEBUG_AABB
#endif

//#define PREFILTER_RAYCAST(filter,body,collision,userData) (filter && !filter(body,collision,userData)) 
//typedef dInt32(dgApi *OnBodiesInAABB) (dgBody* body, void* const userData);
//typedef dUnsigned32(dgApi *OnRayPrecastAction) (const dgBody* const body, const dShapeInstance* const collision, void* const userData);
//typedef dFloat32(dgApi *OnRayCastAction) (const dgBody* const body, const dShapeInstance* const collision, const dVector& contact, const dVector& normal, dgInt64 collisionID, void* const userData, dFloat32 intersetParam);


/*
D_MSC_VECTOR_ALIGNMENT
class dShapeInfo
{
	public:
	class dgInstanceMaterial
	{
		public:
		dgInstanceMaterial()
		{
			memset(this, 0, sizeof (dgInstanceMaterial));
		}

		dgInt64 m_userId;
		union {
			void* m_userData;
			dgUnsigned64 m_alignPad;
		};
		union {
			dgUnsigned64 m_intData;
			dFloat32 m_floatData;
		} m_userParam[6];
	};

	struct dgBoxData
	{
		dFloat32 m_x;
		dFloat32 m_y;
		dFloat32 m_z;
	} ;

	struct dgSphereData
	{
		dFloat32 m_radius;
	} ;

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
		dShape* m_child;
	};

	struct dgCoumpountCollisionData
	{
		dInt32 m_chidrenCount;
		//dShape** m_chidren;
	};

	struct dShapeBVHData
	{
		dInt32 m_vertexCount;
		dInt32 m_indexCount;
	};

	struct dgDeformableMeshData
	{
		dInt32 m_vertexCount;
		dInt32 m_triangleCount;
		dInt32 m_vertexStrideInBytes;
		dgUnsigned16* m_indexList;
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
		dgInt8* m_atributes;
	};

	struct dgSceneData
	{
		dInt32 m_childrenProxyCount;
	};

	dMatrix m_offsetMatrix;
	dgInstanceMaterial m_collisionMaterial;
	dInt32 m_collisionType;
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
		dShapeBVHData m_bvhCollision;
		dgHeightMapCollisionData m_heightFieldCollision;
		dgSceneData m_sceneCollision;
		dFloat32 m_paramArray[32];
	};
} D_GCC_VECTOR_ALIGNMENT;
*/


enum dShapeID
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

D_MSC_VECTOR_ALIGNMENT
class dShape: public dClassAlloc
{
	public:
//	typedef void (dgApi *OnDebugCollisionMeshCallback) (void* userData, int vertexCount, const dFloat32* faceArray, int faceId);

/*	
	static dUnsigned32 Quantize (dFloat32 value);
	static dUnsigned32 Quantize(const void* const buffer, int size);

	// these function should be be virtual
	dInt32 IsType (dgRTTI type) const; 
	dUnsigned32 GetSignature () const;
	dShapeID GetCollisionPrimityType () const;

	virtual dVector SupportVertex (const dVector& dir, dInt32* const vertexIndex) const = 0;
	virtual dVector SupportVertexSpecialProjectPoint (const dVector& point, const dVector& dir) const = 0;
	virtual dVector SupportVertexSpecial (const dVector& dir, dFloat32 skinSkinThickness, dInt32* const vertexIndex) const = 0;

	virtual dInt32 CalculatePlaneIntersection (const dVector& normal, const dVector& point, dVector* const contactsOut) const = 0;

	virtual void SetCollisionBBox (const dVector& p0, const dVector& p1) = 0;
	virtual void CalcAABB (const dMatrix& matrix, dVector& p0, dVector& p1) const = 0;

	virtual void DebugCollision  (const dMatrix& matrix, dShape::OnDebugCollisionMeshCallback callback, void* const userData) const = 0;
	virtual dFloat32 RayCast (const dVector& localP0, const dVector& localP1, dFloat32 maxT, dgContactPoint& contactOut, const dgBody* const body, void* const userData, OnRayPrecastAction preFilter) const = 0;

	virtual dFloat32 GetVolume () const = 0;
	
	virtual void MassProperties ();
	virtual dFloat32 CalculateMassProperties (const dMatrix& offset, dVector& inertia, dVector& crossInertia, dVector& centerOfMass) const {dAssert (0); return 0;}

	virtual dFloat32 GetBoxMinRadius () const = 0; 
	virtual dFloat32 GetBoxMaxRadius () const = 0; 
	
	virtual dVector CalculateVolumeIntegral (const dMatrix& globalMatrix, const dVector& globalPlane, const dShapeInstance& parentScale) const = 0;
	virtual void Serialize(dgSerialize callback, void* const userData) const = 0;

	virtual void GetCollisionInfo(dShapeInfo* const info) const;
	virtual void SerializeLow(dgSerialize callback, void* const userData) const;

	virtual void CalculateImplicitContacts(dInt32 count, dgContactPoint* const contactPoints) const {dAssert (0);}


	virtual dInt32 GetConvexVertexCount() const; 

	
	const dVector& GetObbOrigin() const; 
	virtual dVector GetObbSize() const; 

	dFloat32 GetUmbraClipSize () const;
	dgMemoryAllocator* GetAllocator() const;
*/

	const dShape* AddRef() const;
	dInt32 GetRefCount() const;
	virtual dInt32 Release() const;

	virtual dShape* GetAsShape() { return this; }
	virtual dShapeBox* GetAsShapeBox() { return nullptr; }
	virtual dShapeNull* GetAsShapeNull() { return nullptr; }
	virtual dShapeConvex* GetAsShapeConvex() { return nullptr; }

	virtual dMatrix CalculateInertiaAndCenterOfMass(const dMatrix& alignMatrix, const dVector& localScale, const dMatrix& matrix) const;

	protected:
	D_NEWTON_API dShape(dShapeID id);
	D_NEWTON_API dShape (const dShape& source);
//	D_NEWTON_API dShape (dgWorld* const world, dgDeserialize deserialization, void* const userData, dInt32 revision);
	D_NEWTON_API virtual ~dShape();

	//void SetSignature (dInt32 signature);
	//virtual dInt32 CalculateSignature () const = 0;

	dVector m_inertia;	
	//dVector m_crossInertia;	
	//dVector m_centerOfMass;
	//dVector m_boxSize;
	//dVector m_boxOrigin;
	mutable std::atomic<dInt32> m_refCount;
	//dUnsigned32 m_signature;
	//dShapeID m_collisionId;
	//dgMemoryAllocator* m_allocator;
	//
	//static dVector m_flushZero;
	//friend class dgBody;
	//friend class dgWorld;
	//friend class dgMinkowskiConv;
	//friend class dShapeInstance;
	//friend class dShapeCompound;

} D_GCC_VECTOR_ALIGNMENT;

/*
inline dShapeID dShape::GetCollisionPrimityType () const
{
	return m_collisionId;
}

inline dUnsigned32 dShape::GetSignature () const
{
	return m_signature;
}

inline void dShape::SetSignature (dInt32 signature)
{
	m_signature = dUnsigned32 (signature);
}

inline dgMemoryAllocator* dShape::GetAllocator() const
{
	return m_allocator;
}


inline dFloat32 dShape::GetUmbraClipSize () const
{
//	return GetMax (GetBoxMaxRadius() * dFloat32 (2.0f) + dFloat32 (1.0f), dFloat32 (16.0f));
	return dFloat32 (3.0f) * GetBoxMaxRadius();
}

inline dInt32 dShape::GetConvexVertexCount() const 
{ 
	return 0;
}

inline dInt32 dShape::IsType (dgRTTI type) const 
{
	return type & m_rtti;
}

inline dVector dShape::GetObbSize() const
{
	return m_boxSize;
}

inline const dVector& dShape::GetObbOrigin() const
{
	return m_boxOrigin;
}
*/


inline const dShape* dShape::AddRef() const
{
	m_refCount.fetch_add(1);
	return this;
}

inline dInt32 dShape::Release() const
{
	dInt32 count = m_refCount.fetch_add(-1);
	dAssert(count >= 1);
	if (count == 1) 
	{
		delete this;
	}
	return count;
}

inline dInt32 dShape::GetRefCount() const
{
	return m_refCount.load();
}

inline dMatrix dShape::CalculateInertiaAndCenterOfMass(const dMatrix& alignMatrix, const dVector& localScale, const dMatrix& matrix) const
{
	dAssert(0);
	return dGetZeroMatrix();
}


#endif 


