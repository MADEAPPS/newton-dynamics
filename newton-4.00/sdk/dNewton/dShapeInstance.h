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

#ifndef __D_SHAPE_INSTANCE_H__ 
#define __D_SHAPE_INSTANCE_H__ 

#define DG_MAX_SHAPE_AABB_PADDING dFloat32 (1.0f / 16.0f)

#include "dShape.h"

class dBody;

D_MSC_VECTOR_ALIGNMENT
class dShapeInstance: public dClassAlloc
{
	public:
	enum dScaleType
	{
		m_unit,
		m_uniform,
		m_nonUniform,
		m_global,
	};

	D_NEWTON_API dShapeInstance(dShape* const shape);
	D_NEWTON_API dShapeInstance(const dShapeInstance& instance);
	//dShapeInstance(const dShapeInstance& meshInstance, const dShape* const shape);
	//dShapeInstance(const dgWorld* const world, const dShape* const childCollision, dInt32 shapeID, const dMatrix& matrix);
	//dShapeInstance(const dgWorld* const world, dgDeserialize deserialization, void* const userData, dInt32 revisionNumber);
	D_NEWTON_API ~dShapeInstance();
	D_NEWTON_API dShapeInstance& operator=(const dShapeInstance& src);

	const dShape* GetShape() const { return m_shape; }

	D_NEWTON_API dMatrix CalculateInertia() const;
#if 0
	dShapeInstance* AddRef ();
	dInt32 Release ();

	void SetScale (const dVector& scale);
	const dVector& GetScale () const;
	const dVector& GetInvScale () const;

	void SetGlobalScale (const dVector& scale);

	dScaleType GetScaleType() const;
	dScaleType GetCombinedScaleType(dScaleType type) const;

	const dMatrix& GetLocalMatrix () const;
	const dMatrix& GetGlobalMatrix () const;
	const dMatrix& GetAlignMatrix () const;
	void SetLocalMatrix (const dMatrix& matrix);
	void SetGlobalMatrix (const dMatrix& matrix);

	dUnsigned64 GetUserDataID () const;
	void SetUserDataID (dUnsigned64 userData);

	void* GetUserData () const;
	void SetUserData (void* const userData);

	dShapeInfo::dgInstanceMaterial GetMaterial () const;
	void SetMaterial (const dShapeInfo::dgInstanceMaterial& userData);

	const void* GetCollisionHandle () const;
	const dShapeInstance* GetParent () const;

	dVector GetBoxSize() const;
	dVector GetBoxOrigin() const;

	dFloat32 GetUmbraClipSize () const;
	
	const dgWorld* GetWorld() const;
	const dShape* GetChildShape() const;

	void SetWorld (dgWorld* const world);
	void SetChildShape (dShape* const shape);

	dFloat32 GetVolume () const;
	void GetCollisionInfo(dShapeInfo* const info) const;

	dInt32 IsType (dShape::dgRTTI type) const;
	dgMemoryAllocator* GetAllocator() const;

	bool GetCollisionMode() const;
	void SetCollisionMode(bool mode);

	void SetBreakImpulse(dFloat32 force);
	dFloat32 GetBreakImpulse() const;

	dUnsigned32 GetSignature () const;
	dShapeID GetCollisionPrimityType () const;

	void CalcObb (dVector& origin, dVector& size) const;
	void CalcAABB (const dMatrix& matrix, dVector& p0, dVector& p1) const;
	dFloat32 RayCast (const dVector& localP0, const dVector& localP1, dFloat32 maxT, dgContactPoint& contactOut, OnRayPrecastAction preFilter, const dgBody* const body, void* const userData) const;

	dFloat32 GetBoxMinRadius () const; 
	dFloat32 GetBoxMaxRadius () const; 
	
	dMatrix GetScaledTransform(const dMatrix& matrix) const;
	void DebugCollision  (const dMatrix& matrix, dShape::OnDebugCollisionMeshCallback callback, void* const userData) const;

	dVector SupportVertex (const dVector& dir) const;
	dInt32 CalculatePlaneIntersection (const dVector& normal, const dVector& point, dVector* const contactsOut) const;

	dInt32 CalculateSignature () const;
	void SetCollisionBBox (const dVector& p0, const dVector& p1);

	dInt32 GetConvexVertexCount() const; 

	void Serialize(dgSerialize callback, void* const userData, bool saveShape = true) const;
	dVector CalculateBuoyancyVolume (const dMatrix& matrix, const dVector& fluidPlane) const;
	
	dVector SupportVertexSpecial (const dVector& dir, dInt32* const vertexIndex) const;
	dVector SupportVertexSpecialProjectPoint (const dVector& point, const dVector& dir) const;

	dFloat32 GetSkinThickness() const;
	void SetSkinThickness(dFloat32 thickness);

	void CalculateImplicitContacts(dInt32 count, dgContactPoint* const contactPoints) const;

	
	dShapeInfo::dgInstanceMaterial m_material;
	const dgWorld* m_world;
	const dShape* m_childShape;
	const void* m_subCollisionHandle;
	const dShapeInstance* m_parent;
	dFloat32 m_skinThickness;
	dInt32 m_collisionMode;
	dInt32 m_refCount;
	dScaleType m_scaleType;
	bool m_isExternal;

	static dVector m_padding;
#endif

	dMatrix m_globalMatrix;
	dMatrix m_localMatrix;
	dMatrix m_aligmentMatrix;
	dVector m_scale;
	dVector m_invScale;
	dVector m_maxScale;

	const dShape* m_shape;
	const dBody* m_ownerBody;
} D_GCC_VECTOR_ALIGNMENT;

#if 0
D_INLINE dShapeInstance::dShapeInstance(const dShapeInstance& meshInstance, const dShape* const shape)
	:m_globalMatrix(meshInstance.m_globalMatrix)
	,m_localMatrix (meshInstance.m_localMatrix)
	,m_aligmentMatrix (meshInstance.m_aligmentMatrix)
	,m_scale(meshInstance.m_scale)
	,m_invScale(meshInstance.m_invScale)
	,m_maxScale(meshInstance.m_maxScale)
	,m_material(meshInstance.m_material)
	,m_world(meshInstance.m_world)
	,m_childShape (shape)
	,m_subCollisionHandle(NULL)
	,m_parent(NULL)
	,m_skinThickness(meshInstance.m_skinThickness)
	,m_collisionMode(meshInstance.m_collisionMode)
	,m_refCount(1)
	,m_scaleType(meshInstance.m_scaleType)
	,m_isExternal(false)
{
	if (m_childShape) {
		m_childShape->AddRef();
	}
}

D_INLINE dShapeInstance* dShapeInstance::AddRef () 
{
	m_refCount ++;
	return this;
}

D_INLINE dInt32 dShapeInstance::Release ()
{
	m_refCount --;
	if (m_refCount) {
		return m_refCount;
	}
	delete this;
	return 0;
}

D_INLINE dInt32 dShapeInstance::IsType (dShape::dgRTTI type) const 
{
	return m_childShape->IsType (type);
}

D_INLINE const dgWorld* dShapeInstance::GetWorld() const
{
	return m_world;
}

D_INLINE const dShape* dShapeInstance::GetChildShape() const 
{
	return m_childShape;
}

D_INLINE void dShapeInstance::SetWorld (dgWorld* const world)
{
	m_world = world;
}

D_INLINE void dShapeInstance::SetChildShape (dShape* const shape)
{
	shape->AddRef();
	if (m_childShape) {
		m_childShape->Release();
	}

	m_childShape = shape;
}

D_INLINE void dShapeInstance::GetCollisionInfo(dShapeInfo* const info) const
{
	info->m_offsetMatrix = m_localMatrix;
	info->m_collisionMaterial = m_material;
	m_childShape->GetCollisionInfo(info);
}

D_INLINE const dVector& dShapeInstance::GetScale () const
{
	return m_scale;
}

D_INLINE const dVector& dShapeInstance::GetInvScale () const
{
	return m_invScale;
}

D_INLINE const dMatrix& dShapeInstance::GetLocalMatrix () const
{
	return m_localMatrix;
}

D_INLINE const dMatrix& dShapeInstance::GetGlobalMatrix () const
{
	return m_globalMatrix;
}

D_INLINE const dMatrix& dShapeInstance::GetAlignMatrix () const
{
	return m_aligmentMatrix;
}

D_INLINE void dShapeInstance::SetGlobalMatrix (const dMatrix& matrix)
{
	m_globalMatrix = matrix;
}

D_INLINE dgMemoryAllocator* dShapeInstance::GetAllocator() const
{
	return m_childShape->GetAllocator();
}

D_INLINE dFloat32 dShapeInstance::GetVolume () const
{
	return m_childShape->GetVolume() * m_scale.m_x * m_scale.m_y * m_scale.m_z;
}

D_INLINE bool dShapeInstance::GetCollisionMode() const
{
	return m_collisionMode ? true : false;
}

D_INLINE void dShapeInstance::SetCollisionMode(bool mode)
{
	m_collisionMode = mode ? 1 : 0;
}

D_INLINE void dShapeInstance::SetBreakImpulse(dFloat32 force)
{
	dAssert (0);
//	m_destructionImpulse = force;
}

D_INLINE dFloat32 dShapeInstance::GetBreakImpulse() const
{
//	return m_destructionImpulse;
	return dFloat32 (1.0e20f);
}

D_INLINE const void* dShapeInstance::GetCollisionHandle () const
{
	return m_subCollisionHandle;
}

D_INLINE const dShapeInstance* dShapeInstance::GetParent () const
{
	return m_parent;
}

D_INLINE dUnsigned64 dShapeInstance::GetUserDataID () const
{
	return m_material.m_userId;
}

D_INLINE void dShapeInstance::SetUserDataID (dUnsigned64 userDataId)
{
	m_material.m_userId = userDataId;
}

D_INLINE void* dShapeInstance::GetUserData () const
{
	return m_material.m_userData;
}

D_INLINE void dShapeInstance::SetUserData (void* const userData)
{
	m_material.m_userData = userData;
}

D_INLINE dShapeInfo::dgInstanceMaterial dShapeInstance::GetMaterial () const
{
	return m_material;
}

D_INLINE void dShapeInstance::SetMaterial(const dShapeInfo::dgInstanceMaterial& userData)
{
	m_material = userData;
}

D_INLINE dUnsigned32 dShapeInstance::GetSignature () const
{
	return m_childShape->GetSignature();
}

D_INLINE dShapeID dShapeInstance::GetCollisionPrimityType () const
{
	return m_childShape->GetCollisionPrimityType();
}

D_INLINE dFloat32 dShapeInstance::GetBoxMinRadius () const
{
	return m_childShape->GetBoxMinRadius() * m_maxScale.m_x;
} 

D_INLINE dFloat32 dShapeInstance::GetBoxMaxRadius () const
{
	return m_childShape->GetBoxMaxRadius() * m_maxScale.m_x;
} 

D_INLINE dVector dShapeInstance::SupportVertex(const dVector& dir) const
{
	dAssert (dir.m_w == dFloat32 (0.0f));
	dAssert (dgAbs(dir.DotProduct(dir).GetScalar() - dFloat32 (1.0f)) < dFloat32 (1.0e-2f));
	dAssert (dir.m_w == dFloat32 (0.0f));
	switch (m_scaleType)
	{
		case m_unit:
		{
			return m_childShape->SupportVertex (dir, NULL);
		}
		case m_uniform:
		{
			return m_scale * m_childShape->SupportVertex (dir, NULL);
		}
		case m_nonUniform:
		{
			// support((p * S), n) = S * support (p, n * transp(S)) 
			dVector dir1 ((m_scale * dir).Normalize());
			return m_scale * m_childShape->SupportVertex (dir1, NULL);
		}

		case m_global:
		default:	
		{
			dVector dir1 (m_aligmentMatrix.UnrotateVector((m_scale * dir).Normalize()));
			return m_scale * m_aligmentMatrix.TransformVector (m_childShape->SupportVertex (dir1, NULL));
		}
	}
}

D_INLINE dVector dShapeInstance::SupportVertexSpecial (const dVector& dir, dInt32* const vertexIndex) const
{
	dAssert (dir.m_w == dFloat32 (0.0f));
	dAssert(dgAbs(dir.DotProduct(dir).GetScalar() - dFloat32(1.0f)) < dFloat32(1.0e-2f));
	dAssert(dir.m_w == dFloat32(0.0f));
	switch (m_scaleType) 
	{
		case m_unit:
		{
		   return m_childShape->SupportVertexSpecial(dir, m_skinThickness, vertexIndex);
		}
		case m_uniform:
		{
			return m_scale * m_childShape->SupportVertexSpecial(dir, m_skinThickness, vertexIndex);
		}

		default:
			return SupportVertex(dir);

#if 0
		case m_nonUniform:
		{
			// support((p * S), n) = S * support (p, n * transp(S))
			dVector dir1((m_scale * dir).Normalize());
			return m_scale * m_childShape->SupportVertexSpecial(dir1, m_skinThickness, vertexIndex);
		}

		default:
		{
			dVector dir1(m_aligmentMatrix.UnrotateVector((m_scale * dir).Normalize()));
			return m_scale * m_aligmentMatrix.TransformVector(m_childShape->SupportVertexSpecial(dir1, vertexIndex));
		}
#endif
	}
}

D_INLINE dVector dShapeInstance::SupportVertexSpecialProjectPoint (const dVector& point, const dVector& dir) const
{
	dAssert(dir.m_w == dFloat32(0.0f));
	dAssert(dgAbs(dir.DotProduct(dir).GetScalar() - dFloat32(1.0f)) < dFloat32(1.0e-2f));
	switch (m_scaleType) 
	{
		case m_unit:
		{
		   return m_childShape->SupportVertexSpecialProjectPoint(point, dir);
		}
		case m_uniform:
		{
			return m_scale * m_childShape->SupportVertexSpecialProjectPoint(point * m_invScale, dir);
		}

		default:
			return point;

#if 0
		case m_nonUniform:
		{
			// support((p * S), n) = S * support (p/S, n * transp(S)) 
			dVector dir1((m_scale * dir).Normalize());
			return m_scale * m_childShape->SupportVertexSpecialProjectPoint(point * m_invScale, dir1);
		}

		case m_global:
		default:
		{
			dVector dir1(m_aligmentMatrix.UnrotateVector((m_scale * dir).Normalize()));
			return m_scale * m_aligmentMatrix.TransformVector(m_childShape->SupportVertexSpecialProjectPoint(m_aligmentMatrix.UntransformVector(point * m_invScale), dir1));
		}
#endif
	}
}

D_INLINE void dShapeInstance::SetCollisionBBox (const dVector& p0, const dVector& p1)
{
	dAssert (0);
}

D_INLINE dInt32 dShapeInstance::CalculateSignature () const
{
	dAssert (0);
	return 0;
}

D_INLINE dInt32 dShapeInstance::GetConvexVertexCount() const 
{ 
	return m_childShape->GetConvexVertexCount();
}


D_INLINE dVector dShapeInstance::GetBoxSize() const
{
	switch (m_scaleType)
	{
		case m_unit:
		case m_uniform:
		case m_nonUniform:
			return m_childShape->m_boxSize * m_scale;

		case m_global:
		default:
			return m_childShape->m_boxSize * m_maxScale;
	}
}

D_INLINE dVector dShapeInstance::GetBoxOrigin() const
{
	switch (m_scaleType)
	{
		case m_unit:
		case m_uniform:
		case m_nonUniform:
			return m_childShape->m_boxOrigin * m_scale;

		case m_global:
		default:
			return m_aligmentMatrix.TransformVector(m_childShape->m_boxOrigin) * m_scale;
	}
}

D_INLINE dFloat32 dShapeInstance::GetUmbraClipSize () const
{
	return m_childShape->GetUmbraClipSize() * m_maxScale.m_x;
}

D_INLINE dShapeInstance::dScaleType dShapeInstance::GetScaleType() const
{
	return m_scaleType;
}

D_INLINE dShapeInstance::dScaleType dShapeInstance::GetCombinedScaleType(dShapeInstance::dScaleType type) const
{
	dAssert (0);
	return dgMax(m_scaleType, type);
}

D_INLINE dMatrix dShapeInstance::GetScaledTransform(const dMatrix& matrix) const
{
	dMatrix scaledMatrix(m_localMatrix * matrix);
	scaledMatrix[0] = scaledMatrix[0].Scale(m_scale[0]);
	scaledMatrix[1] = scaledMatrix[1].Scale(m_scale[1]);
	scaledMatrix[2] = scaledMatrix[2].Scale(m_scale[2]);
	return m_aligmentMatrix * scaledMatrix;
}

D_INLINE void dShapeInstance::CalcObb (dVector& origin, dVector& size) const
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
			size = size * m_scale + m_padding;
			origin = origin * m_scale;
			break;
		}
		case m_global:
		{
//			dMatrix matrix1 (matrix);
//			matrix1[0] = matrix1[0].Scale(m_scale.m_x);
//			matrix1[1] = matrix1[1].Scale(m_scale.m_y);
//			matrix1[2] = matrix1[2].Scale(m_scale.m_z);
//			m_childShape->CalcAABB (m_aligmentMatrix * matrix1, p0, p1);
//			p0 -= m_padding;
//			p1 += m_padding;

			dVector p0;
			dVector p1;
			m_childShape->CalcAABB(m_aligmentMatrix, p0, p1);
			size = (dVector::m_half * (p1 - p0) * m_scale + m_padding) & dVector::m_triplexMask;
			origin = (dVector::m_half * (p1 + p0) * m_scale) & dVector::m_triplexMask;;
			break;
		}
	}

	dAssert (size.m_w == dFloat32 (0.0f));
	dAssert (origin.m_w == dFloat32 (0.0f));
}

D_INLINE dFloat32 dShapeInstance::GetSkinThickness() const
{
	return m_skinThickness;
}

D_INLINE void dShapeInstance::SetSkinThickness(dFloat32 thickness)
{
	m_skinThickness = dgAbs (thickness);
}

D_INLINE dVector dShapeInstance::CalculateBuoyancyVolume(const dMatrix& matrix, const dVector& fluidPlane) const
{
	return m_childShape->CalculateVolumeIntegral(m_localMatrix * matrix, fluidPlane, *this);
}
#endif

#endif 

