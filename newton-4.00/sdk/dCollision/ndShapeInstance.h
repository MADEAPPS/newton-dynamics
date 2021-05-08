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
	class ndDistanceCalculator
	{
		public:
		ndDistanceCalculator()
		{
		}

		ndDistanceCalculator(
			ndShapeInstance* const shape0, const dMatrix& matrix0,
			ndShapeInstance* const shape1, const dMatrix& matrix1)
			:m_matrix0(matrix0)
			,m_matrix1(matrix1)
			,m_point0(dVector::m_wOne)
			,m_point1(dVector::m_wOne)
			,m_normal(dVector::m_zero)
			,m_shape0(shape0)
			,m_shape1(shape1)
		{
		}

		D_COLLISION_API bool ClosestPoint();

		dMatrix m_matrix0;
		dMatrix m_matrix1;
		dVector m_point0;
		dVector m_point1;
		dVector m_normal;
		ndShapeInstance* m_shape0;
		ndShapeInstance* m_shape1;
	};

	enum ndScaleType
	{
		m_unit,
		m_uniform,
		m_nonUniform,
		m_global,
	};

	D_COLLISION_API ndShapeInstance(ndShape* const shape);
	D_COLLISION_API ndShapeInstance(const ndShapeInstance& instance);
	D_COLLISION_API ndShapeInstance(const nd::TiXmlNode* const xmlNode, const dTree<const ndShape*, dUnsigned32>& shapesCache);
	D_COLLISION_API ~ndShapeInstance();
	D_COLLISION_API ndShapeInstance& operator=(const ndShapeInstance& src);

	D_COLLISION_API dMatrix CalculateInertia() const;
	D_COLLISION_API void CalculateObb(dVector& origin, dVector& size) const;
	D_COLLISION_API void CalculateAABB(const dMatrix& matrix, dVector& minP, dVector& maxP) const;
	D_COLLISION_API void DebugShape(const dMatrix& matrix, ndShapeDebugCallback& debugCallback) const;
	D_COLLISION_API dFloat32 RayCast(ndRayCastNotify& callback, const dVector& localP0, const dVector& localP1, const ndBody* const body, ndContactPoint& contactOut) const;

	//D_COLLISION_API dInt32 ClosestPoint(const dMatrix& matrix, const dVector& point, dVector& contactPoint) const;

	D_COLLISION_API ndShapeInfo GetShapeInfo() const;

	D_COLLISION_API dFloat32 CalculateBuoyancyCenterOfPresure(dVector& com, const dMatrix& matrix, const dVector& fluidPlane) const;
	D_COLLISION_API void Save(nd::TiXmlElement* const rootNode, const dTree<dUnsigned32, const ndShape*>& shapesCache) const;

	D_COLLISION_API static dVector GetBoxPadding();

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

	ndScaleType GetScaleType() const;
	dFloat32 GetUmbraClipSize() const;
	dUnsigned64 GetUserDataID() const;

	dMatrix m_globalMatrix;
	dMatrix m_localMatrix;
	dMatrix m_aligmentMatrix;
	dVector m_scale;
	dVector m_invScale;
	dVector m_maxScale;

	ndShapeMaterial m_shapeMaterial;
	const ndShape* m_shape;
	const ndBody* m_ownerBody;
	const void* m_subCollisionHandle;
	const ndShapeInstance* m_parent;

	dFloat32 m_skinThickness;
	ndScaleType m_scaleType;
	bool m_collisionMode;

	static dVector m_padding;
	friend class ndShapeConvexPolygon;
} D_GCC_NEWTON_ALIGN_32 ;

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

D_INLINE ndShapeInstance::ndScaleType ndShapeInstance::GetScaleType() const
{
	return m_scaleType;
}

D_INLINE dFloat32 ndShapeInstance::GetUmbraClipSize() const
{
	return m_shape->GetUmbraClipSize() * m_maxScale.m_x;
}

D_INLINE dUnsigned64 ndShapeInstance::GetUserDataID() const
{
	return m_shapeMaterial.m_userId;
}

#endif 


