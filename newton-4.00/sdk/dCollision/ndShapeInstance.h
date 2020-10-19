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

#define D_MAX_SHAPE_AABB_PADDING dFloat32 (1.0f / 16.0f)

#include "ndShape.h"

class ndBody;
class ndShapeInfo;
class ndContactPoint;
class ndShapeInstance;
class ndRayCastNotify;

D_MSV_NEWTON_ALIGN_32
class ndShapeDebugCallback : public dClassAlloc
{
	public: 
	ndShapeDebugCallback()
		:m_instance(nullptr)
	{
	}

	virtual ~ndShapeDebugCallback()
	{
	}

	virtual void DrawPolygon(dInt32 vertexCount, const dVector* const faceArray) = 0;

	const ndShapeInstance* m_instance;
} D_GCC_NEWTON_ALIGN_32;

D_MSV_NEWTON_ALIGN_32
class ndShapeInstance: public dClassAlloc
{
	public:
	enum ndScaleType
	{
		m_unit,
		m_uniform,
		m_nonUniform,
		m_global,
	};

	D_COLLISION_API ndShapeInstance(ndShape* const shape);
	D_COLLISION_API ndShapeInstance(const ndShapeInstance& instance);
	//ndShapeInstance(const ndShapeInstance& meshInstance, const dShape* const shape);
	//ndShapeInstance(const dgWorld* const world, const dShape* const childCollision, dInt32 shapeID, const dMatrix& matrix);
	//ndShapeInstance(const dgWorld* const world, dgDeserialize deserialization, void* const userData, dInt32 revisionNumber);
	D_COLLISION_API ~ndShapeInstance();
	D_COLLISION_API ndShapeInstance& operator=(const ndShapeInstance& src);

	D_COLLISION_API dMatrix CalculateInertia() const;
	D_COLLISION_API void CalculateAABB(const dMatrix& matrix, dVector& minP, dVector& maxP) const;
	D_COLLISION_API void DebugShape(const dMatrix& matrix, ndShapeDebugCallback& debugCallback) const;
	D_COLLISION_API dFloat32 RayCast(ndRayCastNotify& callback, const dVector& localP0, const dVector& localP1, dFloat32 maxT, const ndBody* const body, ndContactPoint& contactOut) const;

	D_COLLISION_API ndShapeInfo GetShapeInfo() const;

	D_COLLISION_API dFloat32 CalculateBuoyancyCenterOfPresure(dVector& com, const dMatrix& matrix, const dVector& fluidPlane) const;

	ndShape* GetShape();
	const ndShape* GetShape() const;
	dVector SupportVertex(const dVector& dir) const;
	dMatrix GetScaledTransform(const dMatrix& matrix) const;
	dVector SupportVertexSpecial(const dVector& dir, dInt32* const vertexIndex) const;
	dVector SupportVertexSpecialProjectPoint(const dVector& point, const dVector& dir) const;
	dInt32 CalculatePlaneIntersection(const dVector& normal, const dVector& point, dVector* const contactsOut) const;

	const dMatrix& GetLocalMatrix() const;
	void SetLocalMatrix(const dMatrix& matrix);

	const dMatrix& GetGlobalMatrix() const;
	void SetGlobalMatrix(const dMatrix& scale);

	bool GetCollisionMode() const;
	void SetCollisionMode(bool mode);
	dInt32 GetConvexVertexCount() const;

	ndShapeMaterial GetMaterial() const;
	void SetMaterial(const ndShapeMaterial& material);
	
	const dVector& GetScale() const;
	const dVector& GetInvScale() const;
	D_COLLISION_API void SetScale(const dVector& scale);

	dFloat32 GetVolume() const;
	dFloat32 GetBoxMinRadius() const;
	dFloat32 GetBoxMaxRadius() const;

#if 0
	ndShapeInstance* AddRef ();
	dInt32 Release ();

	void SetGlobalScale (const dVector& scale);

	dScaleType GetScaleType() const;
	dScaleType GetCombinedScaleType(dScaleType type) const;

	const dMatrix& GetAlignMatrix () const;
	dUnsigned64 GetUserDataID () const;
	void SetUserDataID (dUnsigned64 userData);

	void* GetUserData () const;
	void SetUserData (void* const userData);

	const void* GetCollisionHandle () const;
	const ndShapeInstance* GetParent () const;

	dVector GetBoxSize() const;
	dVector GetBoxOrigin() const;

	dFloat32 GetUmbraClipSize () const;
	
	const dgWorld* GetWorld() const;
	const dShape* GetChildShape() const;

	void SetWorld (dgWorld* const world);
	void SetChildShape (dShape* const shape);

	dInt32 IsType (dShape::dgRTTI type) const;
	dgMemoryAllocator* GetAllocator() const;

	void SetBreakImpulse(dFloat32 force);
	dFloat32 GetBreakImpulse() const;

	dUnsigned32 GetSignature () const;
	ndShapeID GetCollisionPrimityType () const;

	void CalcObb (dVector& origin, dVector& size) const;

	dInt32 CalculateSignature () const;
	void SetCollisionBBox (const dVector& p0, const dVector& p1);

	void Serialize(dgSerialize callback, void* const userData, bool saveShape = true) const;
	

	dFloat32 GetSkinThickness() const;
	void SetSkinThickness(dFloat32 thickness);

	void CalculateImplicitContacts(dInt32 count, dgContactPoint* const contactPoints) const;
	ndShapeInfo::dgInstanceMaterial m_material;

	const dgWorld* m_world;
	const dShape* m_shape;
	const void* m_subCollisionHandle;
	const ndShapeInstance* m_parent;
	dInt32 m_refCount;
	
	bool m_isExternal;
#endif

	dMatrix m_globalMatrix;
	dMatrix m_localMatrix;
	dMatrix m_aligmentMatrix;
	dVector m_scale;
	dVector m_invScale;
	dVector m_maxScale;

	ndShapeMaterial m_shapeMaterial;
	const ndShape* m_shape;
	const ndBody* m_ownerBody;
	dFloat32 m_skinThickness;
	ndScaleType m_scaleType;
	bool m_collisionMode;

	static dVector m_padding;
} D_GCC_NEWTON_ALIGN_32 ;

#if 0
D_INLINE ndShapeInstance::ndShapeInstance(const ndShapeInstance& meshInstance, const dShape* const shape)
	:m_globalMatrix(meshInstance.m_globalMatrix)
	,m_localMatrix (meshInstance.m_localMatrix)
	,m_aligmentMatrix (meshInstance.m_aligmentMatrix)
	,m_scale(meshInstance.m_scale)
	,m_invScale(meshInstance.m_invScale)
	,m_maxScale(meshInstance.m_maxScale)
	,m_material(meshInstance.m_material)
	,m_world(meshInstance.m_world)
	,m_shape (shape)
	,m_subCollisionHandle(nullptr)
	,m_parent(nullptr)
	,m_skinThickness(meshInstance.m_skinThickness)
	,m_collisionMode(meshInstance.m_collisionMode)
	,m_refCount(1)
	,m_scaleType(meshInstance.m_scaleType)
	,m_isExternal(false)
{
	if (m_shape) {
		m_shape->AddRef();
	}
}

D_INLINE ndShapeInstance* ndShapeInstance::AddRef () 
{
	m_refCount ++;
	return this;
}

D_INLINE dInt32 ndShapeInstance::Release ()
{
	m_refCount --;
	if (m_refCount) {
		return m_refCount;
	}
	delete this;
	return 0;
}

D_INLINE dInt32 ndShapeInstance::IsType (dShape::dgRTTI type) const 
{
	return m_shape->IsType (type);
}

D_INLINE const dgWorld* ndShapeInstance::GetWorld() const
{
	return m_world;
}

D_INLINE const dShape* ndShapeInstance::GetChildShape() const 
{
	return m_shape;
}

D_INLINE void ndShapeInstance::SetWorld (dgWorld* const world)
{
	m_world = world;
}

D_INLINE void ndShapeInstance::SetChildShape (dShape* const shape)
{
	shape->AddRef();
	if (m_shape) {
		m_shape->Release();
	}

	m_shape = shape;
}

D_INLINE void ndShapeInstance::GetCollisionInfo(ndShapeInfo* const info) const
{
	info->m_offsetMatrix = m_localMatrix;
	info->m_collisionMaterial = m_material;
	m_shape->GetCollisionInfo(info);
}


D_INLINE const dMatrix& ndShapeInstance::GetAlignMatrix () const
{
	return m_aligmentMatrix;
}


D_INLINE dgMemoryAllocator* ndShapeInstance::GetAllocator() const
{
	return m_shape->GetAllocator();
}


D_INLINE void ndShapeInstance::SetBreakImpulse(dFloat32 force)
{
	dAssert (0);
//	m_destructionImpulse = force;
}

D_INLINE dFloat32 ndShapeInstance::GetBreakImpulse() const
{
//	return m_destructionImpulse;
	return dFloat32 (1.0e20f);
}

D_INLINE const void* ndShapeInstance::GetCollisionHandle () const
{
	return m_subCollisionHandle;
}

D_INLINE const ndShapeInstance* ndShapeInstance::GetParent () const
{
	return m_parent;
}

D_INLINE dUnsigned64 ndShapeInstance::GetUserDataID () const
{
	return m_material.m_userId;
}

D_INLINE void ndShapeInstance::SetUserDataID (dUnsigned64 userDataId)
{
	m_material.m_userId = userDataId;
}

D_INLINE void* ndShapeInstance::GetUserData () const
{
	return m_material.m_userData;
}

D_INLINE void ndShapeInstance::SetUserData (void* const userData)
{
	m_material.m_userData = userData;
}

D_INLINE dUnsigned32 ndShapeInstance::GetSignature () const
{
	return m_shape->GetSignature();
}

D_INLINE ndShapeID ndShapeInstance::GetCollisionPrimityType () const
{
	return m_shape->GetCollisionPrimityType();
}

D_INLINE void ndShapeInstance::SetCollisionBBox (const dVector& p0, const dVector& p1)
{
	dAssert (0);
}

D_INLINE dInt32 ndShapeInstance::CalculateSignature () const
{
	dAssert (0);
	return 0;
}

D_INLINE dVector ndShapeInstance::GetBoxSize() const
{
	switch (m_scaleType)
	{
		case m_unit:
		case m_uniform:
		case m_nonUniform:
			return m_shape->m_boxSize * m_scale;

		case m_global:
		default:
			return m_shape->m_boxSize * m_maxScale;
	}
}

D_INLINE dVector ndShapeInstance::GetBoxOrigin() const
{
	switch (m_scaleType)
	{
		case m_unit:
		case m_uniform:
		case m_nonUniform:
			return m_shape->m_boxOrigin * m_scale;

		case m_global:
		default:
			return m_aligmentMatrix.TransformVector(m_shape->m_boxOrigin) * m_scale;
	}
}

D_INLINE dFloat32 ndShapeInstance::GetUmbraClipSize () const
{
	return m_shape->GetUmbraClipSize() * m_maxScale.m_x;
}

D_INLINE ndShapeInstance::ndScaleType ndShapeInstance::GetScaleType() const
{
	return m_scaleType;
}

D_INLINE ndShapeInstance::ndScaleType ndShapeInstance::GetCombinedScaleType(ndShapeInstance::ndScaleType type) const
{
	dAssert (0);
	return dMax(m_scaleType, type);
}


D_INLINE void ndShapeInstance::CalcObb (dVector& origin, dVector& size) const
{
	size = m_shape->GetObbSize(); 
	origin = m_shape->GetObbOrigin(); 

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
//			m_shape->CalcAABB (m_aligmentMatrix * matrix1, p0, p1);
//			p0 -= m_padding;
//			p1 += m_padding;

			dVector p0;
			dVector p1;
			m_shape->CalcAABB(m_aligmentMatrix, p0, p1);
			size = (dVector::m_half * (p1 - p0) * m_scale + m_padding) & dVector::m_triplexMask;
			origin = (dVector::m_half * (p1 + p0) * m_scale) & dVector::m_triplexMask;;
			break;
		}
	}

	dAssert (size.m_w == dFloat32 (0.0f));
	dAssert (origin.m_w == dFloat32 (0.0f));
}

D_INLINE dFloat32 ndShapeInstance::GetSkinThickness() const
{
	return m_skinThickness;
}

D_INLINE void ndShapeInstance::SetSkinThickness(dFloat32 thickness)
{
	m_skinThickness = dAbs (thickness);
}

#endif

D_INLINE ndShape* ndShapeInstance::GetShape()
{
	return (ndShape*)m_shape;
}

D_INLINE const ndShape* ndShapeInstance::GetShape() const 
{ 
	return m_shape; 
}

D_INLINE const dMatrix& ndShapeInstance::GetLocalMatrix() const
{
	return m_localMatrix;
}

D_INLINE void ndShapeInstance::SetLocalMatrix(const dMatrix& matrix)
{
	m_localMatrix = matrix;
}

D_INLINE dInt32 ndShapeInstance::GetConvexVertexCount() const
{
	return m_shape->GetConvexVertexCount();
}

D_INLINE const dMatrix& ndShapeInstance::GetGlobalMatrix() const
{
	return m_globalMatrix;
}

D_INLINE void ndShapeInstance::SetGlobalMatrix(const dMatrix& matrix)
{
	m_globalMatrix = matrix;
}

D_INLINE dMatrix ndShapeInstance::GetScaledTransform(const dMatrix& matrix) const
{
	dMatrix scaledMatrix(m_localMatrix * matrix);
	scaledMatrix[0] = scaledMatrix[0].Scale(m_scale[0]);
	scaledMatrix[1] = scaledMatrix[1].Scale(m_scale[1]);
	scaledMatrix[2] = scaledMatrix[2].Scale(m_scale[2]);
	return m_aligmentMatrix * scaledMatrix;
}


D_INLINE dVector ndShapeInstance::SupportVertex(const dVector& dir) const
{
	dAssert(dir.m_w == dFloat32(0.0f));
	dAssert(dAbs(dir.DotProduct(dir).GetScalar() - dFloat32(1.0f)) < dFloat32(1.0e-2f));
	dAssert(dir.m_w == dFloat32(0.0f));
	switch (m_scaleType)
	{
		case m_unit:
		{
			return m_shape->SupportVertex(dir, nullptr);
		}
		case m_uniform:
		{
			return m_scale * m_shape->SupportVertex(dir, nullptr);
		}
		case m_nonUniform:
		{
			// support((p * S), n) = S * support (p, n * transp(S)) 
			dVector dir1((m_scale * dir).Normalize());
			return m_scale * m_shape->SupportVertex(dir1, nullptr);
		}

		case m_global:
		default:
		{
			dVector dir1(m_aligmentMatrix.UnrotateVector((m_scale * dir).Normalize()));
			return m_scale * m_aligmentMatrix.TransformVector(m_shape->SupportVertex(dir1, nullptr));
		}
	}
}

D_INLINE dVector ndShapeInstance::SupportVertexSpecial(const dVector& dir, dInt32* const vertexIndex) const
{
	dAssert(dir.m_w == dFloat32(0.0f));
	dAssert(dAbs(dir.DotProduct(dir).GetScalar() - dFloat32(1.0f)) < dFloat32(1.0e-2f));
	dAssert(dir.m_w == dFloat32(0.0f));
	switch (m_scaleType)
	{
		case m_unit:
		{
			return m_shape->SupportVertexSpecial(dir, m_skinThickness, vertexIndex);
		}
		case m_uniform:
		{
			return m_scale * m_shape->SupportVertexSpecial(dir, m_skinThickness, vertexIndex);
		}

		default:
			return SupportVertex(dir);
	}
}

D_INLINE dVector ndShapeInstance::SupportVertexSpecialProjectPoint(const dVector& point, const dVector& dir) const
{
	dAssert(dir.m_w == dFloat32(0.0f));
	dAssert(dAbs(dir.DotProduct(dir).GetScalar() - dFloat32(1.0f)) < dFloat32(1.0e-2f));
	switch (m_scaleType)
	{
		case m_unit:
		{
			return m_shape->SupportVertexSpecialProjectPoint(point, dir);
		}
		case m_uniform:
		{
			return m_scale * m_shape->SupportVertexSpecialProjectPoint(point * m_invScale, dir);
		}

		default:
			return point;

#if 0
	case m_nonUniform:
	{
		// support((p * S), n) = S * support (p/S, n * transp(S)) 
		dVector dir1((m_scale * dir).Normalize());
		return m_scale * m_shape->SupportVertexSpecialProjectPoint(point * m_invScale, dir1);
	}

	case m_global:
	default:
	{
		dVector dir1(m_aligmentMatrix.UnrotateVector((m_scale * dir).Normalize()));
		return m_scale * m_aligmentMatrix.TransformVector(m_shape->SupportVertexSpecialProjectPoint(m_aligmentMatrix.UntransformVector(point * m_invScale), dir1));
	}
#endif
	}
}

D_INLINE bool ndShapeInstance::GetCollisionMode() const
{
	return m_collisionMode;
}

D_INLINE void ndShapeInstance::SetCollisionMode(bool mode)
{
	m_collisionMode = mode;
}

D_INLINE const dVector& ndShapeInstance::GetScale() const
{
	return m_scale;
}

D_INLINE const dVector& ndShapeInstance::GetInvScale() const
{
	return m_invScale;
}

D_INLINE dFloat32 ndShapeInstance::GetBoxMinRadius() const
{
	return m_shape->GetBoxMinRadius() * m_maxScale.m_x;
}

D_INLINE dFloat32 ndShapeInstance::GetBoxMaxRadius() const
{
	return m_shape->GetBoxMaxRadius() * m_maxScale.m_x;
}

D_INLINE dFloat32 ndShapeInstance::GetVolume() const
{
	return m_shape->GetVolume() * m_scale.m_x * m_scale.m_y * m_scale.m_z;
}

D_INLINE ndShapeMaterial ndShapeInstance::GetMaterial() const
{
	return m_shapeMaterial;
}

D_INLINE void ndShapeInstance::SetMaterial(const ndShapeMaterial& material)
{
	m_shapeMaterial = material;
}


#endif 

