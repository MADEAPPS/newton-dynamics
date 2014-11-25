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
		m_global,
	};

	DG_CLASS_ALLOCATOR(allocator)
	dgCollisionInstance();
	dgCollisionInstance(const dgCollisionInstance& instance);
	dgCollisionInstance(const dgCollisionInstance& meshInstance, const dgCollision* const shape);
	dgCollisionInstance(const dgWorld* const world, const dgCollision* const childCollision, dgInt32 shapeID, const dgMatrix& matrix);
	dgCollisionInstance(const dgWorld* const world, dgDeserialize deserialization, void* const userData);
	~dgCollisionInstance();

	dgCollisionInstance* AddRef ();
	dgInt32 Release ();

	void SetScale (const dgVector& scale);
	const dgVector& GetScale () const;
	const dgVector& GetInvScale () const;

	void SetGlobalScale (const dgVector& scale);

	dgScaleType GetScaleType() const;
	dgScaleType GetCombinedScaleType(dgScaleType type) const;

	const dgMatrix& GetLocalMatrix () const;
	const dgMatrix& GetGlobalMatrix () const;
	const dgMatrix& GetAlignMatrix () const;
	void SetLocalMatrix (const dgMatrix& matrix);
	void SetGlobalMatrix (const dgMatrix& matrix);

	dgUnsigned32 GetUserDataID () const;
	void SetUserDataID (dgUnsigned32 userData);

	void* GetUserData () const;
	void SetUserData (void* const userData);

	const void* GetCollisionHandle () const;
	const dgCollisionInstance* GetParent () const;

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

	void CalcObb (dgVector& origin, dgVector& size) const;
	void CalcAABB (const dgMatrix& matrix, dgVector& p0, dgVector& p1) const;
	dgFloat32 RayCast (const dgVector& localP0, const dgVector& localP1, dgFloat32 maxT, dgContactPoint& contactOut, OnRayPrecastAction preFilter, const dgBody* const body, void* const userData) const;
	dgFloat32 ConvexRayCast (const dgCollisionInstance* const convexShape, const dgMatrix& localMatrix, const dgVector& localVeloc, dgFloat32 maxT, dgContactPoint& contactOut, OnRayPrecastAction preFilter, const dgBody* const referenceBody, void* const userData, dgInt32 threadId) const; 

	dgFloat32 GetBoxMinRadius () const; 
	dgFloat32 GetBoxMaxRadius () const; 
	dgMatrix CalculateInertia () const;
	void DebugCollision  (const dgMatrix& matrix, dgCollision::OnDebugCollisionMeshCallback callback, void* const userData) const;

	dgVector SupportVertex (const dgVector& dir, dgInt32* const vertexIndex) const;
	dgInt32 CalculatePlaneIntersection (const dgVector& normal, const dgVector& point, dgVector* const contactsOut, dgFloat32 normalSign) const;

	dgInt32 CalculateSignature () const;
	void SetCollisionBBox (const dgVector& p0, const dgVector& p1);

	dgInt32 GetConvexVertexCount() const; 

	void Serialize(dgSerialize callback, void* const userData, bool saveShape = true) const;
	void CalculateBuoyancyAcceleration (const dgMatrix& matrix, const dgVector& origin, const dgVector& gravity, const dgVector& fluidPlane, dgFloat32 fluidDensity, dgFloat32 fluidViscosity, dgVector& accel, dgVector& alpha);
	

	dgMatrix m_globalMatrix;
	dgMatrix m_localMatrix;
	dgMatrix m_aligmentMatrix;
	dgVector m_scale;
	dgVector m_invScale;
	dgVector m_maxScale;
	dgUnsigned32 m_userDataID;
	dgInt32 m_refCount;
	
	void* m_userData;
	const dgWorld* m_world;
	const dgCollision* m_childShape;
	const void* m_subCollisionHandle;
	const dgCollisionInstance* m_parent;
	dgInt32 m_collisionMode;
	dgScaleType m_scaleType;

	static dgVector m_padding;
};

DG_INLINE dgCollisionInstance::dgCollisionInstance(const dgCollisionInstance& meshInstance, const dgCollision* const shape)
	:m_globalMatrix(meshInstance.m_globalMatrix)
	,m_localMatrix (meshInstance.m_localMatrix)
	,m_aligmentMatrix (meshInstance.m_aligmentMatrix)
	,m_scale(meshInstance.m_scale)
	,m_invScale(meshInstance.m_invScale)
	,m_maxScale(meshInstance.m_maxScale)
	,m_userDataID(meshInstance.m_userDataID)
	,m_refCount(1)
	,m_userData(NULL)
	,m_world(meshInstance.m_world)
	,m_childShape (shape)
	,m_subCollisionHandle(NULL)
	,m_parent(NULL)
	,m_collisionMode(meshInstance.m_collisionMode)
	,m_scaleType(meshInstance.m_scaleType)
{
	if (m_childShape) {
		m_childShape->AddRef();
	}
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

DG_INLINE const dgMatrix& dgCollisionInstance::GetAlignMatrix () const
{
	return m_aligmentMatrix;
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
	m_collisionMode = mode ? 1 : 0;
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


DG_INLINE const void* dgCollisionInstance::GetCollisionHandle () const
{
	return m_subCollisionHandle;
}

DG_INLINE const dgCollisionInstance* dgCollisionInstance::GetParent () const
{
	return m_parent;
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
	//return m_childShape->GetBoxMaxRadius() * m_maxScale.m_x;
	return GetBoxMinRadius ();
} 


DG_INLINE dgVector dgCollisionInstance::SupportVertex(const dgVector& dir, dgInt32* const vertexIndex) const
{
	dgAssert (dgAbsf(dir % dir - dgFloat32 (1.0f)) < dgFloat32 (1.0e-2f));
	dgAssert (dir.m_w == dgFloat32 (0.0f));
	switch (m_scaleType)
	{
		case m_unit:
		{
			return m_childShape->SupportVertex (dir, vertexIndex);
		}
		case m_uniform:
		{
			return m_scale.CompProduct4 (m_childShape->SupportVertex (dir, vertexIndex));
		}
		case m_nonUniform:
		{
			// support((p * S), n) = S * support (p, n * transp(S)) 
			dgVector dir1 (m_scale.CompProduct4(dir));
			dir1 = dir1.CompProduct4(dir1.InvMagSqrt());
			return m_scale.CompProduct4(m_childShape->SupportVertex (dir1, vertexIndex));
		}

		case m_global:
		default:	
		{
			dgVector dir1 (m_aligmentMatrix.UnrotateVector(m_scale.CompProduct4(dir)));
			dir1 = dir1.CompProduct4(dir1.InvMagSqrt());
			return m_scale.CompProduct4(m_aligmentMatrix.TransformVector (m_childShape->SupportVertex (dir1, vertexIndex)));
		}
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
	switch (m_scaleType)
	{
		case m_unit:
		case m_uniform:
		case m_nonUniform:
			return m_childShape->m_boxSize.CompProduct4(m_scale);

		case m_global:
		default:
			return m_childShape->m_boxSize.CompProduct4(m_maxScale);
	}
}

DG_INLINE dgVector dgCollisionInstance::GetBoxOrigin() const
{
	switch (m_scaleType)
	{
		case m_unit:
		case m_uniform:
		case m_nonUniform:
			return m_childShape->m_boxOrigin.CompProduct4(m_scale);

		case m_global:
		default:
			return m_aligmentMatrix.TransformVector(m_childShape->m_boxOrigin).CompProduct4(m_scale);
	}
}

DG_INLINE dgFloat32 dgCollisionInstance::GetUmbraClipSize () const
{
	return m_childShape->GetUmbraClipSize() * m_maxScale.m_x;
}

DG_INLINE dgCollisionInstance::dgScaleType dgCollisionInstance::GetScaleType() const
{
	return m_scaleType;
}

DG_INLINE dgCollisionInstance::dgScaleType dgCollisionInstance::GetCombinedScaleType(dgCollisionInstance::dgScaleType type) const
{
	return dgMax(m_scaleType, type);
}

DG_INLINE void dgCollisionInstance::CalcObb (dgVector& origin, dgVector& size) const
{
	size = m_childShape->GetObbSize(); 
	origin = m_childShape->GetObbOrigin(); 

	switch (m_scaleType)
	{
		case m_unit:
		{
			size += m_padding;
			break;
		}

		case m_uniform:
		case m_nonUniform:
		{
			size = size.CompProduct4(m_scale) + m_padding;
			origin = origin.CompProduct4(m_scale);
			break;
		}
		case m_global:
		{
			dgAssert (0);
//			dgMatrix matrix1 (matrix);
//			matrix1[0] = matrix1[0].Scale4(m_scale.m_x);
//			matrix1[1] = matrix1[1].Scale4(m_scale.m_y);
//			matrix1[2] = matrix1[2].Scale4(m_scale.m_z);
//			m_childShape->CalcAABB (m_aligmentMatrix * matrix1, p0, p1);
//			p0 -= m_padding;
//			p1 += m_padding;
			break;
		}
	}

	dgAssert (size.m_w == dgFloat32 (0.0f));
	dgAssert (origin.m_w == dgFloat32 (0.0f));
}

#endif 

