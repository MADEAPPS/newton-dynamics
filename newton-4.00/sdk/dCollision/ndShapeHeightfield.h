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

#ifndef __D_SHAPE_HEIGHT_FIELD__
#define __D_SHAPE_HEIGHT_FIELD__

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

	D_COLLISION_API ndShapeHeightfield(
		dInt32 width, dInt32 height, ndGridConstruction contructionMode,
		dFloat32 verticalScale, dFloat32 horizontalScale_x, dFloat32 horizontalScale_z);

	D_COLLISION_API ndShapeHeightfield(const nd::TiXmlNode* const xmlNode, const char* const assetPath);
	D_COLLISION_API virtual ~ndShapeHeightfield();

	dArray<dInt16>& GetElevationMap();
	const dArray<dInt16>& GetElevationMap() const;
	D_COLLISION_API void UpdateElevationMapAabb();

	void GetLocalAabb(const dVector& p0, const dVector& p1, dVector& boxP0, dVector& boxP1) const;

	protected:
	virtual ndShapeInfo GetShapeInfo() const;
	virtual ndShapeHeightfield* GetAsShapeHeightfield() { return this; }
	virtual void DebugShape(const dMatrix& matrix, ndShapeDebugCallback& debugCallback) const;
	virtual dFloat32 RayCast(ndRayCastNotify& callback, const dVector& localP0, const dVector& localP1, dFloat32 maxT, const ndBody* const body, ndContactPoint& contactOut) const;
	virtual void GetCollidingFaces(ndPolygonMeshDesc* const data) const;
	virtual void Save(nd::TiXmlElement* const xmlNode, const char* const assetPath, dInt32 nodeid) const;

	private: 
	class ndLocalThreadData
	{
		public:
		ndLocalThreadData()
			:m_threadId()
		{
		}

		dArray<dVector> m_vertex;
		std::thread::id m_threadId;
	};

	void CalculateAABB();
	dInt32 FastInt(dFloat32 x) const;
	const dInt32* GetIndexList() const;
	void CalculateMinExtend2d(const dVector& p0, const dVector& p1, dVector& boxP0, dVector& boxP1) const;
	void CalculateMinExtend3d(const dVector& p0, const dVector& p1, dVector& boxP0, dVector& boxP1) const;
	dFloat32 RayCastCell(const dFastRay& ray, dInt32 xIndex0, dInt32 zIndex0, dVector& normalOut, dFloat32 maxT) const;
	void CalculateMinAndMaxElevation(dInt32 x0, dInt32 x1, dInt32 z0, dInt32 z1, dFloat32& minHeight, dFloat32& maxHeight) const;

	dVector m_minBox;
	dVector m_maxBox;
	dArray<dInt8> m_atributeMap;
	dArray<dInt16> m_elevationMap;
	dFloat32 m_verticalScale;
	dFloat32 m_horizontalScale_x;
	dFloat32 m_horizontalScale_z;
	dFloat32 m_horizontalScaleInv_x;
	dFloat32 m_horizontalScaleInv_z;
	dInt32 m_width;
	dInt32 m_height;
	ndGridConstruction m_diagonalMode;
	mutable dList<ndLocalThreadData> m_localData;

	static dVector m_yMask;
	static dVector m_padding;
	static dVector m_elevationPadding;
	static dInt32 m_cellIndices[][4];
	static dInt32 m_verticalEdgeMap[][7];
	static dInt32 m_horizontalEdgeMap[][7];
	friend class ndContactSolver;
};

inline dArray<dInt16>& ndShapeHeightfield::GetElevationMap()
{
	return m_elevationMap;
}

inline const dArray<dInt16>& ndShapeHeightfield::GetElevationMap() const
{
	return m_elevationMap;
}

inline dInt32 ndShapeHeightfield::FastInt(dFloat32 x) const
{
	dInt32 i = dInt32(x);
	if (dFloat32(i) > x) 
	{
		i--;
	}
	return i;
}

#endif
