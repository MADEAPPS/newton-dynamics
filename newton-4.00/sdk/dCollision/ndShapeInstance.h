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

#ifndef __ND_SHAPE_INSTANCE_H__ 
#define __ND_SHAPE_INSTANCE_H__ 

#define D_MAX_SHAPE_AABB_PADDING ndFloat32 (1.0f / 16.0f)

#include "ndShape.h"

class ndBody;
class ndScene;
class ndShapeInfo;
class ndContactPoint;
class ndShapeInstance;
class ndRayCastNotify;

D_MSV_NEWTON_ALIGN_32
class ndShapeDebugNotify : public ndClassAlloc
{
	public: 
	enum ndEdgeType
	{
		m_shared,
		m_open,
	};

	ndShapeDebugNotify()
		:m_instance(nullptr)
	{
	}

	virtual ~ndShapeDebugNotify()
	{
	}

	virtual void DrawPolygon(ndInt32 vertexCount, const ndVector* const faceArray, const ndEdgeType* const edgeType) = 0;

	const ndShapeInstance* m_instance;
} D_GCC_NEWTON_ALIGN_32;

D_MSV_NEWTON_ALIGN_32
class ndShapeInstance: public ndClassAlloc
{
	public:
	class ndDistanceCalculator
	{
		public:
		ndDistanceCalculator(ndScene* const scene)
		{
			m_scene = scene;
		}

		ndDistanceCalculator(ndScene* const scene,
			ndShapeInstance* const shape0, const ndMatrix& matrix0,
			ndShapeInstance* const shape1, const ndMatrix& matrix1)
			:m_matrix0(matrix0)
			,m_matrix1(matrix1)
			,m_point0(ndVector::m_wOne)
			,m_point1(ndVector::m_wOne)
			,m_normal(ndVector::m_zero)
			,m_scene(scene)
			,m_shape0(shape0)
			,m_shape1(shape1)
		{
		}

		D_COLLISION_API bool ClosestPoint();

		ndMatrix m_matrix0;
		ndMatrix m_matrix1;
		ndVector m_point0;
		ndVector m_point1;
		ndVector m_normal;
		ndScene* m_scene;
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
	D_COLLISION_API ndShapeInstance(const ndShapeInstance& instance, ndShape* const shape);
	D_COLLISION_API ndShapeInstance(const nd::TiXmlNode* const xmlNode, const ndShapeLoaderCache& shapesMap);
	D_COLLISION_API ~ndShapeInstance();

	D_COLLISION_API ndShapeInstance& operator=(const ndShapeInstance& src);

	D_COLLISION_API ndMatrix CalculateInertia() const;
	D_COLLISION_API void CalculateObb(ndVector& origin, ndVector& size) const;
	D_COLLISION_API void CalculateAabb(const ndMatrix& matrix, ndVector& minP, ndVector& maxP) const;
	D_COLLISION_API void DebugShape(const ndMatrix& matrix, ndShapeDebugNotify& debugCallback) const;
	D_COLLISION_API ndFloat32 RayCast(ndRayCastNotify& callback, const ndVector& localP0, const ndVector& localP1, const ndBody* const body, ndContactPoint& contactOut) const;

	//D_COLLISION_API ndInt32 ClosestPoint(const ndMatrix& matrix, const ndVector& point, ndVector& contactPoint) const;

	D_COLLISION_API ndShapeInfo GetShapeInfo() const;
	D_COLLISION_API void Save(const ndLoadSaveBase::ndSaveDescriptor& desc) const;
	D_COLLISION_API ndFloat32 CalculateBuoyancyCenterOfPresure(ndVector& com, const ndMatrix& matrix, const ndVector& fluidPlane) const;

	D_COLLISION_API static ndVector GetBoxPadding();
	

	ndShape* GetShape();
	const ndShape* GetShape() const;
	void SetShape(ndShape* const shape);
	ndVector SupportVertex(const ndVector& dir) const;
	ndMatrix GetScaledTransform(const ndMatrix& matrix) const;
	ndVector SupportVertexSpecial(const ndVector& dir, ndInt32* const vertexIndex) const;
	ndVector SupportVertexSpecialProjectPoint(const ndVector& point, const ndVector& dir) const;

	const ndMatrix& GetLocalMatrix() const;
	void SetLocalMatrix(const ndMatrix& matrix);

	const ndMatrix& GetGlobalMatrix() const;
	void SetGlobalMatrix(const ndMatrix& scale);

	bool GetCollisionMode() const;
	void SetCollisionMode(bool mode);
	ndInt32 GetConvexVertexCount() const;

	ndShapeMaterial GetMaterial() const;
	void SetMaterial(const ndShapeMaterial& material);
	
	const ndVector& GetScale() const;
	const ndVector& GetInvScale() const;
	D_COLLISION_API void SetScale(const ndVector& scale);
	D_COLLISION_API void SetGlobalScale(const ndVector& scale);
	D_COLLISION_API void SetGlobalScale(const ndMatrix& scaleMatrix);
	D_COLLISION_API ndInt32 CalculatePlaneIntersection(const ndVector& normal, const ndVector& point, ndVector* const contactsOut) const;

	ndFloat32 GetVolume() const;
	ndFloat32 GetBoxMinRadius() const;
	ndFloat32 GetBoxMaxRadius() const;

	ndScaleType GetScaleType() const;
	ndFloat32 GetUmbraClipSize() const;
	ndUnsigned64 GetUserDataID() const;

	ndMatrix m_globalMatrix;
	ndMatrix m_localMatrix;
	ndMatrix m_aligmentMatrix;
	ndVector m_scale;
	ndVector m_invScale;
	ndVector m_maxScale;

	ndShapeMaterial m_shapeMaterial;
	const ndShape* m_shape;
	const ndBody* m_ownerBody;
	const void* m_subCollisionHandle;
	const ndShapeInstance* m_parent;

	ndFloat32 m_skinThickness;
	ndScaleType m_scaleType;
	bool m_collisionMode;

	private:
	static ndVector m_padding;
} D_GCC_NEWTON_ALIGN_32 ;

inline ndShape* ndShapeInstance::GetShape()
{
	return (ndShape*)m_shape;
}

inline const ndShape* ndShapeInstance::GetShape() const 
{ 
	return m_shape; 
}

inline const ndMatrix& ndShapeInstance::GetLocalMatrix() const
{
	return m_localMatrix;
}

inline void ndShapeInstance::SetLocalMatrix(const ndMatrix& matrix)
{
	m_localMatrix = matrix;
}

inline ndInt32 ndShapeInstance::GetConvexVertexCount() const
{
	return m_shape->GetConvexVertexCount();
}

inline const ndMatrix& ndShapeInstance::GetGlobalMatrix() const
{
	return m_globalMatrix;
}

inline void ndShapeInstance::SetGlobalMatrix(const ndMatrix& matrix)
{
	m_globalMatrix = matrix;
}

inline ndMatrix ndShapeInstance::GetScaledTransform(const ndMatrix& matrix) const
{
	ndMatrix scale(dGetIdentityMatrix());
	scale[0][0] = m_scale.m_x;
	scale[1][1] = m_scale.m_y;
	scale[2][2] = m_scale.m_z;
	//ndMatrix scaledMatrix(m_localMatrix * matrix);
	return m_aligmentMatrix * scale * m_localMatrix * matrix;
}

inline ndVector ndShapeInstance::SupportVertex(const ndVector& inDir) const
{
	const ndVector dir(inDir & ndVector::m_triplexMask);
	dAssert(dir.m_w == ndFloat32(0.0f));
	dAssert(dAbs(dir.DotProduct(dir).GetScalar() - ndFloat32(1.0f)) < ndFloat32(1.0e-2f));
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
			const ndVector dir1((m_scale * dir).Normalize());
			return m_scale * m_shape->SupportVertex(dir1, nullptr);
		}

		case m_global:
		default:
		{
			const ndVector dir1(m_aligmentMatrix.UnrotateVector((m_scale * dir).Normalize()));
			return m_scale * m_aligmentMatrix.TransformVector(m_shape->SupportVertex(dir1, nullptr));
		}
	}
}

inline ndVector ndShapeInstance::SupportVertexSpecial(const ndVector& inDir, ndInt32* const vertexIndex) const
{
	const ndVector dir(inDir & ndVector::m_triplexMask);
	dAssert(dir.m_w == ndFloat32(0.0f));
	dAssert(dAbs(dir.DotProduct(dir).GetScalar() - ndFloat32(1.0f)) < ndFloat32(1.0e-2f));
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

inline ndVector ndShapeInstance::SupportVertexSpecialProjectPoint(const ndVector& point, const ndVector& inDir) const
{
	const ndVector dir(inDir & ndVector::m_triplexMask);
	dAssert(dir.m_w == ndFloat32(0.0f));
	dAssert(dAbs(dir.DotProduct(dir).GetScalar() - ndFloat32(1.0f)) < ndFloat32(1.0e-2f));
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

inline bool ndShapeInstance::GetCollisionMode() const
{
	return m_collisionMode;
}

inline void ndShapeInstance::SetCollisionMode(bool mode)
{
	m_collisionMode = mode;
}

inline const ndVector& ndShapeInstance::GetScale() const
{
	return m_scale;
}

inline const ndVector& ndShapeInstance::GetInvScale() const
{
	return m_invScale;
}

inline ndFloat32 ndShapeInstance::GetBoxMinRadius() const
{
	return m_shape->GetBoxMinRadius() * m_maxScale.m_x;
}

inline ndFloat32 ndShapeInstance::GetBoxMaxRadius() const
{
	return m_shape->GetBoxMaxRadius() * m_maxScale.m_x;
}

inline ndFloat32 ndShapeInstance::GetVolume() const
{
	return m_shape->GetVolume() * m_scale.m_x * m_scale.m_y * m_scale.m_z;
}

inline ndShapeMaterial ndShapeInstance::GetMaterial() const
{
	return m_shapeMaterial;
}

inline void ndShapeInstance::SetMaterial(const ndShapeMaterial& material)
{
	m_shapeMaterial = material;
}

inline ndShapeInstance::ndScaleType ndShapeInstance::GetScaleType() const
{
	return m_scaleType;
}

inline ndFloat32 ndShapeInstance::GetUmbraClipSize() const
{
	return m_shape->GetUmbraClipSize() * m_maxScale.m_x;
}

inline ndUnsigned64 ndShapeInstance::GetUserDataID() const
{
	return m_shapeMaterial.m_userId;
}

inline void ndShapeInstance::SetShape(ndShape* const shape)
{
	if (m_shape)
	{
		m_shape->Release();
	}
	m_shape = shape ? shape->AddRef() : shape;
}
#endif 


