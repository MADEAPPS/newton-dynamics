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

#ifndef __DGCOLLISION_HEIGHT_FIELD__
#define __DGCOLLISION_HEIGHT_FIELD__

#include "dgCollision.h"
#include "dgCollisionMesh.h"

class dgCollisionHeightField;
typedef dgFloat32 (*dgCollisionHeightFieldRayCastCallback) (const dgBody* const body, const dgCollisionHeightField* const heightFieldCollision, dgFloat32 interception, dgInt32 row, dgInt32 col, dgVector* const normal, int faceId, void* const usedData);


class dgCollisionHeightField: public dgCollisionMesh
{
	public:
	enum dgElevationType
	{
		m_float32Bit = 0,
		m_unsigned16Bit,
	};

	enum dgCollisionHeightFieldGridConstruction
	{
		m_normalDiagonals = 0,
		m_invertedDiagonals,
		m_alternateOddRowsDiagonals,
		m_alternateEvenRowsDiagonals,
		m_alternateOddColumsDiagonals,
		m_alternateEvenColumsDiagonals,
		m_starDiagonals,
		m_starInvertexDiagonals,
	};
	dgCollisionHeightField (dgWorld* const world, dgInt32 width, dgInt32 height, dgInt32 contructionMode, 
							const void* const elevationMap, dgElevationType elevationDataType, dgFloat32 verticalScale, 
							const dgInt8* const atributeMap, dgFloat32 horizontalScale);

	dgCollisionHeightField (dgWorld* const world, dgDeserialize deserialization, void* const userData, dgInt32 revisionNumber);

	virtual ~dgCollisionHeightField(void);

	void SetCollisionRayCastCallback (dgCollisionHeightFieldRayCastCallback rayCastCallback);
	dgCollisionHeightFieldRayCastCallback GetDebugRayCastCallback() const { return m_userRayCastCallback;} 

	void SetHorizontalDisplacement (const dgUnsigned16* const displacemnet, dgFloat32 scale);

	private:
	class dgPerIntanceData
	{
		public:
		dgWorld* m_world;
		dgInt32 m_refCount;
		dgInt32 m_vertexCount[DG_MAX_THREADS_HIVE_COUNT];
		dgVector *m_vertex[DG_MAX_THREADS_HIVE_COUNT];
	};

	void CalculateAABB();
	
	void AllocateVertex(dgWorld* const world, dgInt32 thread) const;
	void CalculateMinExtend2d (const dgVector& p0, const dgVector& p1, dgVector& boxP0, dgVector& boxP1) const;
	void CalculateMinExtend3d (const dgVector& p0, const dgVector& p1, dgVector& boxP0, dgVector& boxP1) const;
	dgFloat32 RayCastCell (const dgFastRayTest& ray, dgInt32 xIndex0, dgInt32 zIndex0, dgVector& normalOut, dgFloat32 maxT) const;

	virtual void Serialize(dgSerialize callback, void* const userData) const;
	virtual dgFloat32 RayCast (const dgVector& localP0, const dgVector& localP1, dgFloat32 maxT, dgContactPoint& contactOut, const dgBody* const body, void* const userData, OnRayPrecastAction preFilter) const;
	virtual void GetCollidingFaces (dgPolygonMeshDesc* const data) const;

	virtual void GetCollisionInfo(dgCollisionInfo* const info) const;
	virtual dgVector SupportVertex (const dgVector& dir, dgInt32* const vertexIndex) const;
	virtual dgVector SupportVertexSpecial (const dgVector& dir, dgInt32* const vertexIndex) const;
	virtual dgVector SupportVertexSpecialProjectPoint (const dgVector& point, const dgVector& dir) const {return point;};

	virtual void DebugCollision (const dgMatrix& matrixPtr, dgCollision::OnDebugCollisionMeshCallback callback, void* const userData) const;
	void GetVertexListIndexList (const dgVector& p0, const dgVector& p1, dgMeshVertexListIndexList &data) const;
	void GetLocalAABB (const dgVector& p0, const dgVector& p1, dgVector& boxP0, dgVector& boxP1) const;
	
	void AddDisplacement (dgVector* const vertex, dgInt32 x0, dgInt32 x1, dgInt32 z0, dgInt32 z1) const;

	dgVector m_minBox;
	dgVector m_maxBox;

	dgInt32 m_width;
	dgInt32 m_height;
	dgInt32 m_diagonalMode;
	dgInt8* m_atributeMap;
	dgInt8* m_diagonals;
	void* m_elevationMap;
	dgUnsigned16* m_horizontalDisplacement;
	dgFloat32 m_verticalScale;
	dgFloat32 m_horizontalScale;
	dgFloat32 m_horizontalScaleInv;
	dgFloat32 m_horizontalDisplacementScale;
	dgCollisionHeightFieldRayCastCallback m_userRayCastCallback;
	dgElevationType m_elevationDataType;

	static dgVector m_yMask;
	static dgVector m_padding;
	static dgInt32 m_cellIndices[][4];
	static dgInt32 m_verticalEdgeMap[][7];
	static dgInt32 m_horizontalEdgeMap[][7];
	
	dgPerIntanceData* m_instanceData;
	friend class dgCollisionCompound;
};


DG_INLINE void dgCollisionHeightField::CalculateMinExtend2d (const dgVector& p0, const dgVector& p1, dgVector& boxP0, dgVector& boxP1) const
{
	dgVector q0 (p0.GetMin(p1) - m_padding);
	dgVector q1 (p0.GetMax(p1) + m_padding);

	boxP0 = ((q0.Scale4(m_horizontalScaleInv).Floor().Scale4(m_horizontalScale) & m_yMask) + dgVector(dgFloat32 (-1.0e10f)).AndNot(m_yMask)) & dgVector::m_triplexMask;
	boxP1 = (((q1.Scale4(m_horizontalScaleInv).Floor().Scale4(m_horizontalScale) + dgVector(m_horizontalScale)) & m_yMask) + dgVector(dgFloat32 (1.0e10f)).AndNot(m_yMask)) & dgVector::m_triplexMask;

	dgVector minBox ((m_minBox & m_yMask) + boxP0.AndNot(m_yMask));
	dgVector maxBox ((m_maxBox & m_yMask) + boxP1.AndNot(m_yMask));
	boxP0 = boxP0.GetMax (minBox);
	boxP1 = boxP1.GetMin (maxBox);
}


DG_INLINE void dgCollisionHeightField::CalculateMinExtend3d (const dgVector& p0, const dgVector& p1, dgVector& boxP0, dgVector& boxP1) const
{
	dgAssert (p0.m_x <= p1.m_x);
	dgAssert (p0.m_y <= p1.m_y);
	dgAssert (p0.m_z <= p1.m_z);
	dgAssert (p0.m_w == dgFloat32 (0.0f));
	dgAssert (p1.m_w == dgFloat32 (0.0f));

	dgVector q0 (p0 - m_padding);
	dgVector q1 (p1 + m_padding);

	boxP0 = (q0.Scale4(m_horizontalScaleInv).Floor().Scale4(m_horizontalScale) & m_yMask) + q0.AndNot(m_yMask);
	boxP1 = ((q1.Scale4(m_horizontalScaleInv).Floor().Scale4(m_horizontalScale) + dgVector(m_horizontalScale)) & m_yMask) + q1.AndNot(m_yMask);

	if (m_horizontalDisplacement) {
		boxP0 -= dgVector (m_horizontalScale, dgFloat32 (0.0f), m_horizontalScale, dgFloat32 (0.0f)); 
		boxP1 += dgVector (m_horizontalScale, dgFloat32 (0.0f), m_horizontalScale, dgFloat32 (0.0f)); 
	}

	dgVector minBox ((m_minBox & m_yMask) + boxP0.AndNot(m_yMask));
	dgVector maxBox ((m_maxBox & m_yMask) + boxP1.AndNot(m_yMask));
	boxP0 = boxP0.GetMax (minBox);
	boxP1 = boxP1.GetMin (maxBox);
}


#endif
