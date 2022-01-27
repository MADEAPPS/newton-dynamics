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

#ifndef __ND_SHAPE_HEIGHT_FIELD__
#define __ND_SHAPE_HEIGHT_FIELD__

#include "ndCollisionStdafx.h"
#include "ndShapeStaticMesh.h"

class ndShapeHeightfield: public ndShapeStaticMesh
{
	public:
	enum ndGridConstruction
	{
		m_normalDiagonals = 0,
		m_invertedDiagonals,
	};

	D_CLASS_REFLECTION(ndShapeHeightfield);
	D_COLLISION_API ndShapeHeightfield(
		ndInt32 width, ndInt32 height, ndGridConstruction contructionMode,
		ndFloat32 verticalScale, ndFloat32 horizontalScale_x, ndFloat32 horizontalScale_z);
	D_COLLISION_API ndShapeHeightfield(const ndLoadSaveBase::ndLoadDescriptor& desc);
	D_COLLISION_API virtual ~ndShapeHeightfield();

	ndArray<ndInt16>& GetElevationMap();
	const ndArray<ndInt16>& GetElevationMap() const;

	D_COLLISION_API void UpdateElevationMapAabb();
	D_COLLISION_API void GetLocalAabb(const ndVector& p0, const ndVector& p1, ndVector& boxP0, ndVector& boxP1) const;

	protected:
	virtual ndShapeInfo GetShapeInfo() const;
	virtual ndShapeHeightfield* GetAsShapeHeightfield() { return this; }
	virtual void DebugShape(const ndMatrix& matrix, ndShapeDebugNotify& debugCallback) const;
	virtual ndFloat32 RayCast(ndRayCastNotify& callback, const ndVector& localP0, const ndVector& localP1, ndFloat32 maxT, const ndBody* const body, ndContactPoint& contactOut) const;
	virtual void GetCollidingFaces(ndPolygonMeshDesc* const data) const;
	virtual void Save(const ndLoadSaveBase::ndSaveDescriptor& desc) const;

	private: 
	class ndLocalThreadData
	{
		public:
		ndLocalThreadData()
			:m_threadId()
		{
		}

		ndArray<ndVector> m_vertex;
		ndThreadId m_threadId;
	};

	void CalculateLocalObb();
	ndInt32 FastInt(ndFloat32 x) const;
	const ndInt32* GetIndexList() const;
	void CalculateMinExtend2d(const ndVector& p0, const ndVector& p1, ndVector& boxP0, ndVector& boxP1) const;
	void CalculateMinExtend3d(const ndVector& p0, const ndVector& p1, ndVector& boxP0, ndVector& boxP1) const;
	ndFloat32 RayCastCell(const ndFastRay& ray, ndInt32 xIndex0, ndInt32 zIndex0, ndVector& normalOut, ndFloat32 maxT) const;
	void CalculateMinAndMaxElevation(ndInt32 x0, ndInt32 x1, ndInt32 z0, ndInt32 z1, ndFloat32& minHeight, ndFloat32& maxHeight) const;

	ndVector m_minBox;
	ndVector m_maxBox;
	ndArray<ndInt8> m_atributeMap;
	ndArray<ndInt16> m_elevationMap;
	ndFloat32 m_verticalScale;
	ndFloat32 m_horizontalScale_x;
	ndFloat32 m_horizontalScale_z;
	ndFloat32 m_horizontalScaleInv_x;
	ndFloat32 m_horizontalScaleInv_z;
	ndInt32 m_width;
	ndInt32 m_height;
	ndGridConstruction m_diagonalMode;
	mutable ndList<ndLocalThreadData> m_localData;

	static ndVector m_yMask;
	static ndVector m_padding;
	static ndVector m_elevationPadding;
	static ndInt32 m_cellIndices[][4];
	static ndInt32 m_verticalEdgeMap[][7];
	static ndInt32 m_horizontalEdgeMap[][7];
	friend class ndContactSolver;
};

inline ndArray<ndInt16>& ndShapeHeightfield::GetElevationMap()
{
	return m_elevationMap;
}

inline const ndArray<ndInt16>& ndShapeHeightfield::GetElevationMap() const
{
	return m_elevationMap;
}

inline ndInt32 ndShapeHeightfield::FastInt(ndFloat32 x) const
{
	ndInt32 i = ndInt32(x);
	if (ndFloat32(i) > x) 
	{
		i--;
	}
	return i;
}

#endif
