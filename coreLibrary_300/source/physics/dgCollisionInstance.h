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

#ifndef _DG_COLLISION_INSTANCE_H_ 
#define _DG_COLLISION_INSTANCE_H_ 


#define DG_MAX_COLLISION_AABB_PADDING		dgFloat32 (1.0f / 16.0f)

#include "dgCollision.h"

class dgCollisionInstance
{
	public:
	enum dgScaleType
	{
		m_unit,
		m_uniform,
		m_nonUniform,
	};

	DG_CLASS_ALLOCATOR(allocator)
	dgCollisionInstance();
	dgCollisionInstance(const dgCollisionInstance& instance);
	dgCollisionInstance(const dgCollisionInstance& meshInstance, const dgCollision* const shape);
	dgCollisionInstance(const dgWorld* const world, const dgCollision* const childCollision, dgInt32 shapeID, const dgMatrix &matrix);
	dgCollisionInstance(const dgWorld* const world, dgDeserialize deserialization, void* const userData);
	~dgCollisionInstance();

	dgCollisionInstance* AddRef ();
	dgInt32 Release ();

	void SetScale (const dgVector& scale);
	const dgVector& GetScale () const;
	const dgVector& GetInvScale () const;

	const dgMatrix& GetLocalMatrix () const;
	const dgMatrix& GetGlobalMatrix () const;
	void SetLocalMatrix (const dgMatrix& matrix);
	void SetGlobalMatrix (const dgMatrix& matrix);

	dgUnsigned32 GetUserDataID () const;
	void SetUserDataID (dgUnsigned32 userData);

	void* GetUserData () const;
	void SetUserData (void* const userData);

	dgVector GetBoxSize() const;
	dgVector GetBoxOrigin() const;

	dgFloat32 GetUmbraClipSize () const;
	
	const dgWorld* GetWorld() const;
	const dgCollision* GetChildShape() const;

	void SetWorld (dgWorld* const world);
	void SetChildShape (dgCollision* const shape);

	dgFloat32 GetVolume () const;
	void GetCollisionInfo(dgCollisionInfo* const info) const;

	dgInt32 IsType (dgCollision::dgRTTI type) const ;
	dgMemoryAllocator* GetAllocator() const;

	bool GetCollisionMode() const;
	void SetCollisionMode(bool mode);

	void SetBreakImpulse(dgFloat32 force);
	dgFloat32 GetBreakImpulse() const;

	dgUnsigned32 GetSignature () const;
	dgCollisionID GetCollisionPrimityType () const;

	void CalcAABB (const dgMatrix &matrix, dgVector& p0, dgVector& p1) const;
	dgFloat32 RayCast (const dgVector& localP0, const dgVector& localP1, dgContactPoint& contactOut, OnRayPrecastAction preFilter, const dgBody* const body, void* const userData) const;
	dgFloat32 ConvexRayCast (const dgCollisionInstance* const convexShape, const dgMatrix& localMatrix, const dgVector& localVeloc, dgFloat32 maxT, dgContactPoint& contactOut, OnRayPrecastAction preFilter, const dgBody* const referenceBody, void* const userData, dgInt32 threadId) const; 

	dgFloat32 GetBoxMinRadius () const; 
	dgFloat32 GetBoxMaxRadius () const; 
	dgMatrix CalculateInertia () const;
	void DebugCollision  (const dgMatrix& matrix, OnDebugCollisionMeshCallback callback, void* const userData) const;
	dgVector CalculateVolumeIntegral (const dgMatrix& globalMatrix, GetBuoyancyPlane bouyancyPlane, void* const context) const;

	dgVector SupportVertex (const dgVector& dir, dgInt32* const vertexIndex) const;
	dgInt32 CalculatePlaneIntersection (const dgVector& normal, const dgVector& point, dgVector* const contactsOut) const;

	dgInt32 CalculateSignature () const;
	void SetCollisionBBox (const dgVector& p0, const dgVector& p1);

	dgInt32 GetConvexVertexCount() const; 

	void Serialize(dgSerialize callback, void* const userData, bool saveShape = true) const;
//	dgMatrix CalculateSpaceMatrix (const dgCollisionInstance* const instance) const;

	dgMatrix m_globalMatrix;
	dgMatrix m_localMatrix;
	dgVector m_scale;
	dgVector m_invScale;
	dgVector m_maxScale;
	dgUnsigned32 m_userDataID;
	dgInt32 m_refCount;
	
	void* m_userData;
	const dgWorld* m_world;
	const dgCollision* m_childShape;
	union 
	{
		dgUnsigned32 m_flags;
		struct {
			dgInt32 m_scaleIsUnit		:1;
			dgInt32 m_scaleIsUniform	:1;
			dgInt32 m_collisionMode		:1;
		};
	};

	static dgVector m_padding;
};

DG_INLINE dgCollisionInstance::dgCollisionInstance(const dgCollisionInstance& meshInstance, const dgCollision* const shape)
	:m_globalMatrix(meshInstance.m_globalMatrix)
	,m_localMatrix (meshInstance.m_localMatrix)
	,m_scale(meshInstance.m_scale)
	,m_invScale(meshInstance.m_invScale)
	,m_maxScale(meshInstance.m_maxScale)
	,m_userDataID(meshInstance.m_userDataID)
	,m_refCount(1)
//	,m_userData(meshInstance.m_userData)
	,m_userData(NULL)
	,m_world(meshInstance.m_world)
	,m_childShape (shape)
	,m_scaleIsUnit (meshInstance.m_scaleIsUnit)
	,m_scaleIsUniform (meshInstance.m_scaleIsUniform)
	,m_collisionMode(meshInstance.m_collisionMode)
{
	m_childShape->AddRef();
}


DG_INLINE dgCollisionInstance* dgCollisionInstance::AddRef () 
{
	m_refCount ++;
	return this;
}

DG_INLINE dgInt32 dgCollisionInstance::Release ()
{
	m_refCount --;
	if (m_refCount) {
		return m_refCount;
	}
	delete this;
	return 0;
}


DG_INLINE dgInt32 dgCollisionInstance::IsType (dgCollision::dgRTTI type) const 
{
	return m_childShape->IsType (type);
}

DG_INLINE const dgWorld* dgCollisionInstance::GetWorld() const
{
	return m_world;
}

DG_INLINE const dgCollision* dgCollisionInstance::GetChildShape() const 
{
	return m_childShape;
}


DG_INLINE void dgCollisionInstance::SetWorld (dgWorld* const world)
{
	m_world = world;
}

DG_INLINE void dgCollisionInstance::SetChildShape (dgCollision* const shape)
{
	shape->AddRef();
	if (m_childShape) {
		m_childShape->Release();
	}

	m_childShape = shape;
}


DG_INLINE void dgCollisionInstance::GetCollisionInfo(dgCollisionInfo* const info) const
{
	info->m_offsetMatrix = m_localMatrix;
	info->m_userDadaID = m_userDataID;
//	info->m_scale = m_scale;
	m_childShape->GetCollisionInfo(info);
}


DG_INLINE const dgVector& dgCollisionInstance::GetScale () const
{
	return m_scale;
}

DG_INLINE const dgVector& dgCollisionInstance::GetInvScale () const
{
	return m_invScale;
}


DG_INLINE const dgMatrix& dgCollisionInstance::GetLocalMatrix () const
{
	return m_localMatrix;
}

DG_INLINE const dgMatrix& dgCollisionInstance::GetGlobalMatrix () const
{
	return m_globalMatrix;
}


DG_INLINE void dgCollisionInstance::SetGlobalMatrix (const dgMatrix& matrix)
{
	m_globalMatrix = matrix;
}


DG_INLINE dgMemoryAllocator* dgCollisionInstance::GetAllocator() const
{
	return m_childShape->GetAllocator();
}

DG_INLINE dgFloat32 dgCollisionInstance::GetVolume () const
{
	return m_childShape->GetVolume() * m_scale.m_x * m_scale.m_y * m_scale.m_z;
}


DG_INLINE bool dgCollisionInstance::GetCollisionMode() const
{
	return m_collisionMode ? true : false;
}

DG_INLINE void dgCollisionInstance::SetCollisionMode(bool mode)
{
	m_collisionMode = mode;
}




DG_INLINE void dgCollisionInstance::SetBreakImpulse(dgFloat32 force)
{
	dgAssert (0);
//	m_destructionImpulse = force;
}

DG_INLINE dgFloat32 dgCollisionInstance::GetBreakImpulse() const
{
//	return m_destructionImpulse;
	return dgFloat32 (1.0e20f);
}

DG_INLINE void* dgCollisionInstance::GetUserData () const
{
	return m_userData;
}

DG_INLINE void dgCollisionInstance::SetUserData (void* const userData)
{
	m_userData = userData;
}


DG_INLINE dgUnsigned32 dgCollisionInstance::GetUserDataID () const
{
	return m_userDataID;
}

DG_INLINE void dgCollisionInstance::SetUserDataID (dgUnsigned32 userDataId)
{
	m_userDataID = userDataId;
}

DG_INLINE dgUnsigned32 dgCollisionInstance::GetSignature () const
{
	return m_childShape->GetSignature();
}

DG_INLINE dgCollisionID dgCollisionInstance::GetCollisionPrimityType () const
{
	return m_childShape->GetCollisionPrimityType();
}

DG_INLINE dgFloat32 dgCollisionInstance::GetBoxMinRadius () const
{
	return m_childShape->GetBoxMinRadius() * m_maxScale.m_x;
} 

DG_INLINE dgFloat32 dgCollisionInstance::GetBoxMaxRadius () const
{
	return m_childShape->GetBoxMaxRadius() * m_maxScale.m_x;
} 



DG_INLINE dgVector dgCollisionInstance::CalculateVolumeIntegral (const dgMatrix& globalMatrix, GetBuoyancyPlane bouyancyPlane, void* const context) const
{
	return m_childShape->CalculateVolumeIntegral(globalMatrix,bouyancyPlane,context);
}

DG_INLINE dgVector dgCollisionInstance::SupportVertex(const dgVector& dir, dgInt32* const vertexIndex) const
{
	dgAssert (dgAbsf(dir % dir - dgFloat32 (1.0f)) < dgFloat32 (1.0e-2f));

	if (m_scaleIsUniform) {
		return m_scale.CompProduct3 (m_childShape->SupportVertex (dir, vertexIndex));
	} else {
		// support((p * m), n) = m * support (p, n * trans(m)) 
		dgVector dir1 (m_scale.CompProduct3(dir));
		dir1 = dir1.Scale3 (dgRsqrt (dir1 % dir1));
		return m_scale.CompProduct3(m_childShape->SupportVertex (dir1, vertexIndex));
	}

}



DG_INLINE void dgCollisionInstance::SetCollisionBBox (const dgVector& p0, const dgVector& p1)
{
	dgAssert (0);
}

DG_INLINE dgInt32 dgCollisionInstance::CalculateSignature () const
{
	dgAssert (0);
	return 0;
}


DG_INLINE dgInt32 dgCollisionInstance::GetConvexVertexCount() const 
{ 
	return m_childShape->GetConvexVertexCount();
}


DG_INLINE dgVector dgCollisionInstance::GetBoxSize() const
{
	return m_childShape->m_boxSize.CompProduct3(m_scale);
}

DG_INLINE dgVector dgCollisionInstance::GetBoxOrigin() const
{
	return m_childShape->m_boxOrigin.CompProduct3(m_scale);
}

DG_INLINE dgFloat32 dgCollisionInstance::GetUmbraClipSize () const
{
	return m_childShape->GetUmbraClipSize() * m_maxScale.m_x;
}
/*
DG_INLINE dgMatrix dgCollisionInstance::CalculateSpaceMatrix (const dgCollisionInstance* const instance) const
{
	dgMatrix matrix (instance->m_globalMatrix * m_globalMatrix.Inverse());

	const dgVector& scale = instance->m_scale;
	matrix[0] = matrix[0].Scale4(scale[0]);
	matrix[1] = matrix[1].Scale4(scale[1]);
	matrix[2] = matrix[2].Scale4(scale[2]);

	matrix[0] = m_invScale.CompProduct4 (matrix[0]);
	matrix[1] = m_invScale.CompProduct4 (matrix[1]);
	matrix[2] = m_invScale.CompProduct4 (matrix[2]);
	matrix[3] = m_invScale.CompProduct4 (matrix[3]);
	return matrix;
}
*/

#endif 

