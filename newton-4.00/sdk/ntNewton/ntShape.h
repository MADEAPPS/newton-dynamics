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

class ntShapeBox;
class ntShapeNull;
class ntShapeConvex;
class ntShapeDebugCallback;

#ifdef _DEBUG
//	#define DG_DEBUG_AABB
#endif


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
class ntShape: public dClassAlloc
{
	public:
//	typedef void (*OnDebugCollisionMeshCallback) (void* userData, int vertexCount, const dFloat32* faceArray, int faceId);

/*	
	static dUnsigned32 Quantize (dFloat32 value);
	static dUnsigned32 Quantize(const void* const buffer, int size);

	// these function should be be virtual
	dInt32 IsType (dgRTTI type) const; 
	dUnsigned32 GetSignature () const;
	dShapeID GetCollisionPrimityType () const;

	virtual dVector SupportVertexSpecialProjectPoint (const dVector& point, const dVector& dir) const = 0;
	virtual dVector SupportVertexSpecial (const dVector& dir, dFloat32 skinSkinThickness, dInt32* const vertexIndex) const = 0;

	virtual dInt32 CalculatePlaneIntersection (const dVector& normal, const dVector& point, dVector* const contactsOut) const = 0;

	virtual void SetCollisionBBox (const dVector& p0, const dVector& p1) = 0;
	
	virtual dFloat32 RayCast (const dVector& localP0, const dVector& localP1, dFloat32 maxT, dgContactPoint& contactOut, const dgBody* const body, void* const userData, OnRayPrecastAction preFilter) const = 0;

	virtual dFloat32 GetVolume () const = 0;
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

	const ntShape* AddRef() const;
	dInt32 GetRefCount() const;
	virtual dInt32 Release() const;

	virtual ntShape* GetAsShape() { return this; }
	virtual ntShapeBox* GetAsShapeBox() { return nullptr; }
	virtual ntShapeNull* GetAsShapeNull() { return nullptr; }
	virtual ntShapeConvex* GetAsShapeConvex() { return nullptr; }

	D_NEWTON_API virtual void MassProperties();

	virtual void DebugShape(const dMatrix& matrix, ntShapeDebugCallback& debugCallback) const = 0;

	virtual void CalcAABB(const dMatrix& matrix, dVector& p0, dVector& p1) const = 0;
	virtual dVector SupportVertex(const dVector& dir, dInt32* const vertexIndex) const = 0;

	virtual dMatrix CalculateInertiaAndCenterOfMass(const dMatrix& alignMatrix, const dVector& localScale, const dMatrix& matrix) const;
	virtual dFloat32 CalculateMassProperties(const dMatrix& offset, dVector& inertia, dVector& crossInertia, dVector& centerOfMass) const;

	protected:
	D_NEWTON_API ntShape(dShapeID id);
	D_NEWTON_API ntShape (const ntShape& source);
//	D_NEWTON_API dShape (dgWorld* const world, dgDeserialize deserialization, void* const userData, dInt32 revision);
	D_NEWTON_API virtual ~ntShape();

	//void SetSignature (dInt32 signature);
	//virtual dInt32 CalculateSignature () const = 0;

	dVector m_inertia;	
	dVector m_crossInertia;	
	dVector m_centerOfMass;
	dVector m_boxSize;
	dVector m_boxOrigin;
	mutable std::atomic<dInt32> m_refCount;
	//dShapeID m_collisionId;
	static dVector m_flushZero;

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


inline const ntShape* ntShape::AddRef() const
{
	m_refCount.fetch_add(1);
	return this;
}

inline dInt32 ntShape::Release() const
{
	dInt32 count = m_refCount.fetch_add(-1);
	dAssert(count >= 1);
	if (count == 1) 
	{
		delete this;
	}
	return count;
}

inline dInt32 ntShape::GetRefCount() const
{
	return m_refCount.load();
}

inline dFloat32 ntShape::CalculateMassProperties(const dMatrix& offset, dVector& inertia, dVector& crossInertia, dVector& centerOfMass) const
{ 
	dAssert(0); 
	return 0; 
}

inline dMatrix ntShape::CalculateInertiaAndCenterOfMass(const dMatrix& alignMatrix, const dVector& localScale, const dMatrix& matrix) const
{
	dAssert(0);
	return dGetZeroMatrix();
}


#endif 


