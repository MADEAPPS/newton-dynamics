/* Copyright (c) <2003-2022> <Julio Jerez, Newton Game Dynamics>
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

D_MSV_NEWTON_CLASS_ALIGN_32
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

} D_GCC_NEWTON_CLASS_ALIGN_32;

D_MSV_NEWTON_CLASS_ALIGN_32
class ndShapeInstance: public ndContainersFreeListAlloc<ndShapeInstance>
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
	D_COLLISION_API ~ndShapeInstance();

	D_COLLISION_API ndShapeInstance& operator=(const ndShapeInstance& src);

	D_COLLISION_API ndMatrix CalculateInertia() const;
	D_COLLISION_API void CalculateObb(ndVector& origin, ndVector& size) const;
	D_COLLISION_API void CalculateAabb(const ndMatrix& matrix, ndVector& minP, ndVector& maxP) const;
	D_COLLISION_API void DebugShape(const ndMatrix& matrix, ndShapeDebugNotify& debugCallback) const;
	D_COLLISION_API ndFloat32 RayCast(ndRayCastNotify& callback, const ndVector& localP0, const ndVector& localP1, const ndBody* const body, ndContactPoint& contactOut) const;

	//D_COLLISION_API ndInt32 ClosestPoint(const ndMatrix& matrix, const ndVector& point, ndVector& contactPoint) const;

	D_COLLISION_API ndShapeInfo GetShapeInfo() const;
	D_COLLISION_API ndFloat32 CalculateBuoyancyCenterOfPresure(ndVector& com, const ndMatrix& matrix, const ndVector& fluidPlane) const;

	D_COLLISION_API static ndFloat32 GetBoxPadding();

	D_COLLISION_API void SavePLY(const char* const fileName) const;

	const char* ClassName() const;
	static const char* StaticClassName();
	const char* SuperClassName() const;
	
	D_COLLISION_API ndShape* GetShape();
	D_COLLISION_API const ndShape* GetShape() const;
	D_COLLISION_API void SetShape(ndShape* const shape);
	D_COLLISION_API ndVector SupportVertex(const ndVector& dir) const;
	D_COLLISION_API ndMatrix GetScaledTransform(const ndMatrix& matrix) const;
	D_COLLISION_API ndVector SupportVertexSpecial(const ndVector& dir) const;
	D_COLLISION_API ndVector SupportVertexSpecialProjectPoint(const ndVector& point, const ndVector& dir) const;

	D_COLLISION_API const ndMatrix& GetLocalMatrix() const;
	D_COLLISION_API void SetLocalMatrix(const ndMatrix& matrix);

	D_COLLISION_API const ndMatrix& GetGlobalMatrix() const;
	D_COLLISION_API void SetGlobalMatrix(const ndMatrix& scale);

	D_COLLISION_API bool GetCollisionMode() const;
	D_COLLISION_API void SetCollisionMode(bool mode);
	D_COLLISION_API ndInt32 GetConvexVertexCount() const;

	D_COLLISION_API ndShapeMaterial GetMaterial() const;
	D_COLLISION_API void SetMaterial(const ndShapeMaterial& material);
	
	D_COLLISION_API const ndVector& GetScale() const;
	D_COLLISION_API const ndVector& GetInvScale() const;
	D_COLLISION_API const ndMatrix& GetAlignmentMatrix() const;

	D_COLLISION_API void SetScale(const ndVector& scale);
	D_COLLISION_API void SetGlobalScale(const ndVector& scale);
	D_COLLISION_API void SetGlobalScale(const ndMatrix& scaleMatrix);
	D_COLLISION_API ndInt32 CalculatePlaneIntersection(const ndVector& normal, const ndVector& point, ndVector* const contactsOut) const;

	D_COLLISION_API ndFloat32 GetVolume() const;
	D_COLLISION_API ndFloat32 GetBoxMinRadius() const;
	D_COLLISION_API ndFloat32 GetBoxMaxRadius() const;

	D_COLLISION_API ndScaleType GetScaleType() const;
	D_COLLISION_API ndFloat32 GetUmbraClipSize() const;
	D_COLLISION_API ndUnsigned64 GetUserDataID() const;

	ndMatrix m_globalMatrix;
	ndMatrix m_localMatrix;
	ndMatrix m_alignmentMatrix;
	ndVector m_scale;
	ndVector m_invScale;
	ndVector m_maxScale;

	ndShapeMaterial m_shapeMaterial;
	const ndShape* m_shape;
	const ndBody* m_ownerBody;
	const void* m_subCollisionHandle;
	ndFloat32 m_skinMargin;
	ndScaleType m_scaleType;
	bool m_collisionMode;

	private:
	static ndVector m_padding;
} D_GCC_NEWTON_CLASS_ALIGN_32 ;

#endif 


