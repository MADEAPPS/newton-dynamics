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

#ifndef __ND_SHAPE_HEIGHT_FIELD__
#define __ND_SHAPE_HEIGHT_FIELD__

#include "ndCollisionStdafx.h"
#include "ndShapeStaticMesh.h"

D_MSV_NEWTON_CLASS_ALIGN_32
class ndShapeHeightfield: public ndShapeStaticMesh
{
	public:
	class ndTriangle
	{
		public:	
		ndInt32 m_i0;
		ndInt32 m_i1;
		ndInt32 m_i2;
		ndInt32 m_material;
		ndInt32 m_normal;
		ndInt32 m_normal_edge01;
		ndInt32 m_normal_edge12;
		ndInt32 m_normal_edge20;
		ndInt32 m_area;
	};

	class ndGridQuad
	{
		public:
		ndTriangle m_triangle0;
		ndTriangle m_triangle1;
	};

	enum ndGridConstruction
	{
		m_normalDiagonals = 0,
		m_invertedDiagonals,
	};

	D_CLASS_REFLECTION(ndShapeHeightfield,ndShapeStaticMesh)
	D_COLLISION_API ndShapeHeightfield(ndInt32 width, ndInt32 height, ndGridConstruction constructionMode,ndFloat32 horizontalScale_x, ndFloat32 horizontalScale_z);
	D_COLLISION_API virtual ~ndShapeHeightfield();

	D_COLLISION_API ndArray<ndReal>& GetElevationMap();
	D_COLLISION_API const ndArray<ndReal>& GetElevationMap() const;

	D_COLLISION_API ndArray<ndInt8>& GetAttributeMap();
	D_COLLISION_API const ndArray<ndInt8>& GetAttributeMap() const;

	D_COLLISION_API void UpdateElevationMapAabb();
	D_COLLISION_API void GetLocalAabb(const ndVector& p0, const ndVector& p1, ndVector& boxP0, ndVector& boxP1) const;

	protected:
	D_COLLISION_API virtual ndShapeInfo GetShapeInfo() const;
	D_COLLISION_API virtual ndUnsigned64 GetHash(ndUnsigned64 hash) const;
	D_COLLISION_API virtual ndShapeHeightfield* GetAsShapeHeightfield() { return this; }
	D_COLLISION_API virtual void DebugShape(const ndMatrix& matrix, ndShapeDebugNotify& debugCallback) const;
	D_COLLISION_API virtual ndFloat32 RayCast(ndRayCastNotify& callback, const ndVector& localP0, const ndVector& localP1, ndFloat32 maxT, const ndBody* const body, ndContactPoint& contactOut) const;
	D_COLLISION_API virtual void GetCollidingFaces(ndPolygonMeshDesc* const data) const;

	private: 
	void CalculateLocalObb();
	ndInt32 FastInt(ndFloat32 x) const;
	const ndInt32* GetIndexList() const;
	void CalculateMinExtend2d(const ndVector& p0, const ndVector& p1, ndVector& boxP0, ndVector& boxP1) const;
	void CalculateMinExtend3d(const ndVector& p0, const ndVector& p1, ndVector& boxP0, ndVector& boxP1) const;
	ndFloat32 RayCastCell(const ndFastRay& ray, ndInt32 xIndex0, ndInt32 zIndex0, ndVector& normalOut, ndFloat32 maxT) const;
	void CalculateMinAndMaxElevation(ndInt32 x0, ndInt32 x1, ndInt32 z0, ndInt32 z1, ndFloat32& minHeight, ndFloat32& maxHeight) const;

	ndArray<ndInt8> m_attributeMap;
	ndArray<ndReal> m_elevationMap;
	ndFloat32 m_horizontalScale_x;
	ndFloat32 m_horizontalScale_z;
	ndFloat32 m_horizontalScaleInv_x;
	ndFloat32 m_horizontalScaleInv_z;
	ndInt32 m_width;
	ndInt32 m_height;
	ndGridConstruction m_diagonalMode;

	ndVector m_minBox;
	ndVector m_maxBox;

	static ndVector m_yMask;
	static ndVector m_padding;
	static ndVector m_elevationPadding;
	static ndInt32 m_cellIndices[][4];

	friend class ndContactSolver;
} D_GCC_NEWTON_CLASS_ALIGN_32;

#endif
