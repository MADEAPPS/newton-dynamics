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

#include "dgPhysicsStdafx.h"
#include "dgBody.h"
#include "dgWorld.h"
#include "dgCollisionHeightField.h"


#define DG_HIGHTFILD_DATA_ID 0x45AF5E07

dgVector dgCollisionHeightField::m_yMask (0xffffffff, 0, 0xffffffff, 0);
dgVector dgCollisionHeightField::m_padding (dgFloat32 (0.25f), dgFloat32 (0.25f), dgFloat32 (0.25f), dgFloat32 (0.0f));

dgInt32 dgCollisionHeightField::m_cellIndices[][4] =
{
	{0, 1, 2, 3},
	{1, 3, 0, 2}
};

dgInt32 dgCollisionHeightField::m_verticalEdgeMap[][7] = 
{
	{1 * 9 + 1, 0 * 9 + 0, 1 * 9 + 4, 1 * 9 + 6, 0 * 9 + 6, 1 * 9 + 4, 0 * 9 + 4},
	{1 * 9 + 2, 0 * 9 + 1, 1 * 9 + 4, 1 * 9 + 6, 0 * 9 + 7, 1 * 9 + 4, 0 * 9 + 4},
	{1 * 9 + 2, 0 * 9 + 0, 1 * 9 + 4, 1 * 9 + 7, 0 * 9 + 6, 1 * 9 + 4, 0 * 9 + 4},
	{1 * 9 + 0, 0 * 9 + 1, 1 * 9 + 4, 1 * 9 + 7, 0 * 9 + 7, 1 * 9 + 4, 0 * 9 + 4}
};

dgInt32 dgCollisionHeightField::m_horizontalEdgeMap[][7] =
{
	{1 * 9 + 0, 2 * 9 + 1, 1 * 9 + 4, 1 * 9 + 7, 2 * 9 + 7, 1 * 9 + 4, 2 * 9 + 4},
	{1 * 9 + 0, 3 * 9 + 0, 1 * 9 + 4, 1 * 9 + 7, 3 * 9 + 6, 1 * 9 + 4, 3 * 9 + 4},
	{0 * 9 + 1, 2 * 9 + 1, 0 * 9 + 4, 1 * 9 + 7, 2 * 9 + 7, 0 * 9 + 4, 2 * 9 + 4},
	{0 * 9 + 1, 3 * 9 + 0, 0 * 9 + 4, 0 * 9 + 6, 3 * 9 + 6, 0 * 9 + 4, 3 * 9 + 4}
};


dgCollisionHeightField::dgCollisionHeightField(
	dgWorld* const world, dgInt32 width, dgInt32 height, dgInt32 contructionMode, 
	const void* const elevationMap, dgElevationType elevationDataType, dgFloat32 verticalScale, 
	const dgInt8* const atributeMap, dgFloat32 horizontalScale)
	:dgCollisionMesh (world, m_heightField)
	,m_width(width)
	,m_height(height)
	,m_diagonalMode (dgCollisionHeightFieldGridConstruction  (dgClamp (contructionMode, dgInt32 (m_normalDiagonals), dgInt32 (m_starInvertexDiagonals))))
	,m_verticalScale(verticalScale)
	,m_horizontalScale(horizontalScale)
	,m_horizontalScaleInv (dgFloat32 (1.0f) / m_horizontalScale)
	,m_userRayCastCallback(NULL)
	,m_elevationDataType(elevationDataType)
{
	m_rtti |= dgCollisionHeightField_RTTI;

	switch (m_elevationDataType) 
	{
		case m_float32Bit:
		{
			m_elevationMap = dgMallocStack(m_width * m_height * sizeof (dgFloat32));
			memcpy (m_elevationMap, elevationMap, m_width * m_height * sizeof (dgFloat32));
			break;
		}

		case m_unsigned16Bit:
		{
			m_elevationMap = dgMallocStack(m_width * m_height * sizeof (dgUnsigned16));
			memcpy (m_elevationMap, elevationMap, m_width * m_height * sizeof (dgUnsigned16));
		}
	}

	dgInt32 attibutePaddedMapSize = (m_width * m_height + 4) & -4; 
	m_atributeMap = (dgInt8 *)dgMallocStack(attibutePaddedMapSize * sizeof (dgInt8));
	m_diagonals = (dgInt8 *)dgMallocStack(attibutePaddedMapSize * sizeof (dgInt8));

	switch (m_diagonalMode)
	{
		case m_normalDiagonals:
		{
			memset (m_diagonals, 0, m_width * m_height * sizeof (dgInt8));
			break;
		}
		case m_invertedDiagonals:
		{
			memset (m_diagonals, 1, m_width * m_height * sizeof (dgInt8));
			break;
		}

		case m_alternateOddRowsDiagonals:
		{
			for (dgInt32 j = 0; j < m_height; j += 2) {
				dgInt32 index = j * m_width;
				for (dgInt32 i = 0; i < m_width; i ++) {
					m_diagonals[index + i] = 0;
				}
			}

			for (dgInt32 j = 1; j < m_height; j += 2) {
				dgInt32 index = j * m_width;
				for (dgInt32 i = 0; i < m_width; i ++) {
					m_diagonals[index + i] = 1;
				}
			}
			break;
		}

		case m_alternateEvenRowsDiagonals:
		{
			for (dgInt32 j = 0; j < m_height; j += 2) {
				dgInt32 index = j * m_width;
				for (dgInt32 i = 0; i < m_width; i ++) {
					m_diagonals[index + i] = 1;
				}
			}

			for (dgInt32 j = 1; j < m_height; j += 2) {
				dgInt32 index = j * m_width;
				for (dgInt32 i = 0; i < m_width; i ++) {
					m_diagonals[index + i] = 0;
				}
			}
			break;
		}


		case m_alternateOddColumsDiagonals:
		{
			for (dgInt32 j = 0; j < m_height; j ++) {
				dgInt32 index = j * m_width;
				for (dgInt32 i = 0; i < m_width; i += 2) {
					m_diagonals[index + i] = 0;
				}

				for (dgInt32 i = 1; i < m_width; i += 2) {
					m_diagonals[index + i] = 1;
				}
			}
			break;
		}

		case m_alternateEvenColumsDiagonals:
		{
			for (dgInt32 j = 0; j < m_height; j ++) {
				dgInt32 index = j * m_width;
				for (dgInt32 i = 0; i < m_width; i += 2) {
					m_diagonals[index + i] = 1;
				}

				for (dgInt32 i = 1; i < m_width; i += 2) {
					m_diagonals[index + i] = 0;
				}
			}
			break;
		}

		case m_starDiagonals:
		{
			for (dgInt32 j = 0; j < m_height; j += 2) {
				dgInt32 index = j * m_width;
				for (dgInt32 i = 0; i < m_width; i += 2) {
					m_diagonals[index + i] = 0;
				}
				for (dgInt32 i = 1; i < m_width; i += 2) {
					m_diagonals[index + i] = 1;
				}
			}

			for (dgInt32 j = 1; j < m_height; j += 2) {
				dgInt32 index = j * m_width;
				for (dgInt32 i = 0; i < m_width; i += 2) {
					m_diagonals[index + i] = 1;
				}
				for (dgInt32 i = 1; i < m_width; i += 2) {
					m_diagonals[index + i] = 0;
				}
			}
			break;
		}

		case m_starInvertexDiagonals:
		{
			for (dgInt32 j = 0; j < m_height; j += 2) {
				dgInt32 index = j * m_width;
				for (dgInt32 i = 0; i < m_width; i += 2) {
					m_diagonals[index + i] = 1;
				}
				for (dgInt32 i = 1; i < m_width; i += 2) {
					m_diagonals[index + i] = 0;
				}
			}

			for (dgInt32 j = 1; j < m_height; j += 2) {
				dgInt32 index = j * m_width;
				for (dgInt32 i = 0; i < m_width; i += 2) {
					m_diagonals[index + i] = 0;
				}
				for (dgInt32 i = 1; i < m_width; i += 2) {
					m_diagonals[index + i] = 1;
				}
			}

			break;
		}

		default:
			dgAssert (0);
		
	}
	memcpy (m_atributeMap, atributeMap, m_width * m_height * sizeof (dgInt8));


	dgTree<void*, unsigned>::dgTreeNode* nodeData = world->m_perInstanceData.Find(DG_HIGHTFILD_DATA_ID);
	if (!nodeData) {
		m_instanceData = (dgPerIntanceData*) dgMallocStack (sizeof (dgPerIntanceData));
		m_instanceData->m_refCount = 0;
		m_instanceData->m_world = world;
		for (dgInt32 i = 0 ; i < DG_MAX_THREADS_HIVE_COUNT; i ++) {
			m_instanceData->m_vertex[i] = NULL;
			m_instanceData->m_vertexCount[i] = 8 * 8;
			AllocateVertex(world, i);
		}
		nodeData = world->m_perInstanceData.Insert (m_instanceData, DG_HIGHTFILD_DATA_ID);
	}
	m_instanceData = (dgPerIntanceData*) nodeData->GetInfo();

	m_instanceData->m_refCount ++;

	CalculateAABB();
	SetCollisionBBox(m_minBox, m_maxBox);
}

dgCollisionHeightField::dgCollisionHeightField (dgWorld* const world, dgDeserialize deserialization, void* const userData)
	:dgCollisionMesh (world, deserialization, userData)
{
	dgAssert (m_rtti | dgCollisionHeightField_RTTI);
	
	dgInt32 elevationDataType;

	m_userRayCastCallback = NULL;
	deserialization (userData, &m_width, sizeof (dgInt32));
	deserialization (userData, &m_height, sizeof (dgInt32));
	deserialization (userData, &m_diagonalMode, sizeof (dgInt32));
	deserialization (userData, &elevationDataType, sizeof (dgInt32));
	deserialization (userData, &m_verticalScale, sizeof (dgFloat32));
	deserialization (userData, &m_horizontalScale, sizeof (dgFloat32));
	deserialization (userData, &m_minBox.m_x, sizeof (dgVector)); 
	deserialization (userData, &m_maxBox.m_x, sizeof (dgVector)); 

	m_elevationDataType = dgElevationType (elevationDataType);

	dgInt32 attibutePaddedMapSize = (m_width * m_height + 4) & -4; 
	m_atributeMap = (dgInt8 *)dgMallocStack(attibutePaddedMapSize * sizeof (dgInt8));
	m_diagonals = (dgInt8 *)dgMallocStack(attibutePaddedMapSize * sizeof (dgInt8));

	switch (m_elevationDataType) 
	{
		case m_float32Bit:
		{
			m_elevationMap = dgMallocStack(m_width * m_height * sizeof (dgFloat32));
			deserialization (userData, m_elevationMap, m_width * m_height * sizeof (dgFloat32));
			break;
		}

		case m_unsigned16Bit:
		{
			m_elevationMap = dgMallocStack(m_width * m_height * sizeof (dgUnsigned16));
			deserialization (userData, m_elevationMap, m_width * m_height * sizeof (dgUnsigned16));
			break;
		}
	}
	
	deserialization (userData, m_atributeMap, attibutePaddedMapSize * sizeof (dgInt8));
	deserialization (userData, m_diagonals, attibutePaddedMapSize * sizeof (dgInt8));

	m_horizontalScaleInv = dgFloat32 (1.0f) / m_horizontalScale;
	dgTree<void*, unsigned>::dgTreeNode* nodeData = world->m_perInstanceData.Find(DG_HIGHTFILD_DATA_ID);
	if (!nodeData) {
		m_instanceData = (dgPerIntanceData*) dgMallocStack (sizeof (dgPerIntanceData));
		m_instanceData->m_refCount = 0;
		m_instanceData->m_world = world;
		for (dgInt32 i = 0 ; i < DG_MAX_THREADS_HIVE_COUNT; i ++) {
			m_instanceData->m_vertex[i] = NULL;
			m_instanceData->m_vertexCount[i] = 8 * 8;
			AllocateVertex(world, i);
		}
		nodeData = world->m_perInstanceData.Insert (m_instanceData, DG_HIGHTFILD_DATA_ID);
	}
	m_instanceData = (dgPerIntanceData*) nodeData->GetInfo();

	m_instanceData->m_refCount ++;
	SetCollisionBBox(m_minBox, m_maxBox);
}

dgCollisionHeightField::~dgCollisionHeightField(void)
{
	m_instanceData->m_refCount --;
	if (!m_instanceData->m_refCount) {
		dgWorld* world = m_instanceData->m_world;

		for (dgInt32 i = 0 ; i < DG_MAX_THREADS_HIVE_COUNT; i ++) {
			dgFreeStack(m_instanceData->m_vertex[i]);
		}
		dgFreeStack(m_instanceData);
		world->m_perInstanceData.Remove(DG_HIGHTFILD_DATA_ID);
	}
	dgFreeStack(m_elevationMap);
	dgFreeStack(m_atributeMap);
	dgFreeStack(m_diagonals);
}

void dgCollisionHeightField::Serialize(dgSerialize callback, void* const userData) const
{
	SerializeLow(callback, userData);

	dgInt32 elevationDataType = m_elevationDataType;

	callback (userData, &m_width, sizeof (dgInt32));
	callback (userData, &m_height, sizeof (dgInt32));
	callback (userData, &m_diagonalMode, sizeof (dgInt32));
	callback (userData, &elevationDataType, sizeof (dgInt32));
	callback (userData, &m_verticalScale, sizeof (dgFloat32));
	callback (userData, &m_horizontalScale, sizeof (dgFloat32));
	callback (userData, &m_minBox.m_x, sizeof (dgVector)); 
	callback (userData, &m_maxBox.m_x, sizeof (dgVector)); 

	switch (m_elevationDataType) 
	{
		case m_float32Bit:
		{
			callback (userData, m_elevationMap, m_width * m_height * sizeof (dgFloat32));
			break;
		}
		case m_unsigned16Bit:
		{
			callback (userData, m_elevationMap, m_width * m_height * sizeof (dgUnsigned16));
			break;
		}
	}

	dgInt32 attibutePaddedMapSize = (m_width * m_height + 4) & -4; 
	callback (userData, m_atributeMap, attibutePaddedMapSize * sizeof (dgInt8));
	callback (userData, m_diagonals, attibutePaddedMapSize * sizeof (dgInt8));
}

void dgCollisionHeightField::SetCollisionRayCastCallback (dgCollisionHeightFieldRayCastCallback rayCastCallback)
{
	m_userRayCastCallback = rayCastCallback;
}

void dgCollisionHeightField::AllocateVertex(dgWorld* const world, dgInt32 threadIndex) const
{
	dgVector *vertex;
	vertex = (dgVector *)dgMallocStack(2 * m_instanceData->m_vertexCount[threadIndex] * sizeof (dgVector));
	if (m_instanceData->m_vertex[threadIndex]) {
		memcpy (vertex, m_instanceData->m_vertex[threadIndex], m_instanceData->m_vertexCount[threadIndex] * sizeof (dgVector));
		dgFreeStack(m_instanceData->m_vertex[threadIndex]);
	}

	m_instanceData->m_vertexCount[threadIndex] *= 2;
	m_instanceData->m_vertex[threadIndex] = vertex;
}


void dgCollisionHeightField::CalculateAABB()
{
	dgFloat32 y0 = dgFloat32 (dgFloat32 (1.0e10f));
	dgFloat32 y1 = dgFloat32 (-dgFloat32 (1.0e10f));
	switch (m_elevationDataType) 
	{
		case m_float32Bit:
		{
			const dgFloat32* const elevation = (dgFloat32*)m_elevationMap;
			for (dgInt32 i = 0; i < m_width * m_height; i ++) {
				y0 = dgMin(y0, elevation[i]);
				y1 = dgMax(y1, elevation[i]);
			}
			break;
		}

		case m_unsigned16Bit:
		{
			const dgUnsigned16* const elevation = (dgUnsigned16*)m_elevationMap;
			for (dgInt32 i = 0; i < m_width * m_height; i ++) {
				y0 = dgMin(y0, dgFloat32 (elevation[i]));
				y1 = dgMax(y1, dgFloat32 (elevation[i]));
			}
		}
	}
	m_minBox = dgVector (dgFloat32 (dgFloat32 (0.0f)),                y0 * m_verticalScale, dgFloat32 (dgFloat32 (0.0f)),               dgFloat32 (0.0f)); 
	m_maxBox = dgVector (dgFloat32 (m_width - 1) * m_horizontalScale, y1 * m_verticalScale, dgFloat32 (m_height-1) * m_horizontalScale, dgFloat32 (0.0f)); 
}


void dgCollisionHeightField::GetCollisionInfo(dgCollisionInfo* const info) const
{
	dgCollision::GetCollisionInfo(info);

	dgCollisionInfo::dgHeightMapCollisionData& data = info->m_heightFieldCollision;
	data.m_width = m_width;
	data.m_height = m_height;
	data.m_gridsDiagonals = m_diagonalMode;
	data.m_elevationDataType = m_elevationDataType;
	data.m_verticalScale = m_verticalScale;
	data.m_horizonalScale = m_horizontalScale;
	data.m_atributes = m_atributeMap;
	data.m_elevation = m_elevationMap;
}





dgFloat32 dgCollisionHeightField::RayCastCell (const dgFastRayTest& ray, dgInt32 xIndex0, dgInt32 zIndex0, dgVector& normalOut, dgFloat32 maxT) const
{
	dgVector points[4];
	dgInt32 triangle[3];

	// get the 3d point at the corner of the cell
	if ((xIndex0 < 0) || (zIndex0 < 0) || (xIndex0 >= (m_width - 1)) || (zIndex0 >= (m_height - 1))) {
		return dgFloat32 (1.2f);
	}
	
	dgAssert (maxT <= 1.0);

	dgInt32 base = zIndex0 * m_width + xIndex0;
	
	switch (m_elevationDataType) 
	{
		case m_float32Bit:
		{
			const dgFloat32* const elevation = (dgFloat32*)m_elevationMap;
			points[0 * 2 + 0] = dgVector ((xIndex0 + 0) * m_horizontalScale, m_verticalScale * elevation[base],			      (zIndex0 + 0) * m_horizontalScale, dgFloat32 (0.0f));
			points[0 * 2 + 1] = dgVector ((xIndex0 + 1) * m_horizontalScale, m_verticalScale * elevation[base + 1],           (zIndex0 + 0) * m_horizontalScale, dgFloat32 (0.0f));
			points[1 * 2 + 1] = dgVector ((xIndex0 + 1) * m_horizontalScale, m_verticalScale * elevation[base + m_width + 1], (zIndex0 + 1) * m_horizontalScale, dgFloat32 (0.0f));
			points[1 * 2 + 0] = dgVector ((xIndex0 + 0) * m_horizontalScale, m_verticalScale * elevation[base + m_width + 0], (zIndex0 + 1) * m_horizontalScale, dgFloat32 (0.0f));
			break;
		}

		case m_unsigned16Bit:
		default:
		{
			const dgUnsigned16* const elevation = (dgUnsigned16*)m_elevationMap;
			points[0 * 2 + 0] = dgVector ((xIndex0 + 0) * m_horizontalScale,  m_verticalScale * dgFloat32 (elevation[base]),			   (zIndex0 + 0) * m_horizontalScale, dgFloat32 (0.0f));
			points[0 * 2 + 1] = dgVector ((xIndex0 + 1) * m_horizontalScale,  m_verticalScale * dgFloat32 (elevation[base + 1]),           (zIndex0 + 0) * m_horizontalScale, dgFloat32 (0.0f));
			points[1 * 2 + 1] = dgVector ((xIndex0 + 1) * m_horizontalScale,  m_verticalScale * dgFloat32 (elevation[base + m_width + 1]), (zIndex0 + 1) * m_horizontalScale, dgFloat32 (0.0f));
			points[1 * 2 + 0] = dgVector ((xIndex0 + 0) * m_horizontalScale,  m_verticalScale * dgFloat32 (elevation[base + m_width + 0]), (zIndex0 + 1) * m_horizontalScale, dgFloat32 (0.0f));
			break;
		}
	}
	
	dgFloat32 t = dgFloat32 (1.2f);
	if (!m_diagonals[base]) {
		triangle[0] = 1;
		triangle[1] = 2;
		triangle[2] = 3;

		dgVector e10 (points[2] - points[1]);
		dgVector e20 (points[3] - points[1]);
		dgVector normal (e10 * e20);
		normal = normal.CompProduct4(normal.DotProduct4(normal).InvSqrt());
		t = ray.PolygonIntersect (normal, maxT, &points[0].m_x, sizeof (dgVector), triangle, 3);
		if (t < maxT){
			normalOut = normal;
			return t;
		}

		triangle[0] = 1;
		triangle[1] = 0;
		triangle[2] = 2;

		dgVector e30 (points[0] - points[1]);
		normal = e30 * e10;
		//normal = normal.Scale3 (dgRsqrt (normal % normal));
		normal = normal.CompProduct4(normal.DotProduct4(normal).InvSqrt());
		t = ray.PolygonIntersect (normal, maxT, &points[0].m_x, sizeof (dgVector), triangle, 3);
		if (t < maxT){
			normalOut = normal;
			return t;
		}

	} else {
		triangle[0] = 0;
		triangle[1] = 2;
		triangle[2] = 3;

		dgVector e10 (points[2] - points[0]);
		dgVector e20 (points[3] - points[0]);
		dgVector normal (e10 * e20);
		normal = normal.CompProduct4(normal.DotProduct4(normal).InvSqrt());
		t = ray.PolygonIntersect (normal, maxT, &points[0].m_x, sizeof (dgVector), triangle, 3);
		if (t < maxT){
			normalOut = normal;
			return t;
		}

		triangle[0] = 0;
		triangle[1] = 3;
		triangle[2] = 1;

		dgVector e30 (points[1] - points[0]);
		normal = e20 * e30;
		normal = normal.CompProduct4(normal.DotProduct4(normal).InvSqrt());
		t = ray.PolygonIntersect (normal, maxT, &points[0].m_x, sizeof (dgVector), triangle, 3);
		if (t < maxT){
			normalOut = normal;
			return t;
		}
	}
	return t;
}


dgFloat32 dgCollisionHeightField::RayCast (const dgVector& q0, const dgVector& q1, dgFloat32 maxT, dgContactPoint& contactOut, const dgBody* const body, void* const userData, OnRayPrecastAction preFilter) const
{
	dgVector boxP0;
	dgVector boxP1;

	// calculate the ray bounding box
	CalculateMinExtend2d (q0, q1, boxP0, boxP1);

	dgVector p0 (q0);
	dgVector p1 (q1);

	// clip the line against the bounding box
	if (dgRayBoxClip (p0, p1, boxP0, boxP1)) { 
		dgVector dp (p1 - p0);
		dgVector normalOut (dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f));

		dgFloat32 scale = m_horizontalScale;
		dgFloat32 invScale = m_horizontalScaleInv;
		dgInt32 ix0 = dgFastInt (p0.m_x * invScale);
		dgInt32 iz0 = dgFastInt (p0.m_z * invScale);

		// implement a 3ddda line algorithm 
		dgInt32 xInc;
		dgFloat32 tx;
		dgFloat32 stepX;
		if (dp.m_x > dgFloat32 (0.0f)) {
			xInc = 1;
			dgFloat32 val = dgFloat32 (1.0f) / dp.m_x;
			stepX = scale * val;
			tx = (scale * (ix0 + dgFloat32 (1.0f)) - p0.m_x) * val;
		} else if (dp.m_x < dgFloat32 (0.0f)) {
			xInc = -1;
			dgFloat32 val = -dgFloat32 (1.0f) / dp.m_x;
			stepX = scale * val;
			tx = -(scale * ix0 - p0.m_x) * val;
		} else {
			xInc = 0;
			stepX = dgFloat32 (0.0f);
			tx = dgFloat32 (1.0e10f);
		}

		dgInt32 zInc;
		dgFloat32 tz;
		dgFloat32 stepZ;
		
		if (dp.m_z > dgFloat32 (0.0f)) {
			zInc = 1;
			dgFloat32 val = dgFloat32 (1.0f) / dp.m_z;
			stepZ = scale * val;
			tz = (scale * (iz0 + dgFloat32 (1.0f)) - p0.m_z) * val;
		} else if (dp.m_z < dgFloat32 (0.0f)) {
			zInc = -1;
			dgFloat32 val = -dgFloat32 (1.0f) / dp.m_z;
			stepZ = scale * val;
			tz = -(scale * iz0 - p0.m_z) * val;
		} else {
			zInc = 0;
			stepZ = dgFloat32 (0.0f);
			tz = dgFloat32 (1.0e10f);
		}

		dgFloat32 txAcc = tx;
		dgFloat32 tzAcc = tz;
		dgInt32 xIndex0 = ix0;
		dgInt32 zIndex0 = iz0;
		dgFastRayTest ray (q0, q1); 

		// for each cell touched by the line
		do {
			dgFloat32 t = RayCastCell (ray, xIndex0, zIndex0, normalOut, maxT);
			if (t < maxT) {
				// bail out at the first intersection and copy the data into the descriptor
				contactOut.m_normal = normalOut.Scale3 (dgRsqrt (normalOut % normalOut));
				contactOut.m_shapeId0 = m_atributeMap[zIndex0 * m_width + xIndex0];
				contactOut.m_shapeId1 = m_atributeMap[zIndex0 * m_width + xIndex0];

				if (m_userRayCastCallback) {
					dgVector normal (body->GetCollision()->GetGlobalMatrix().RotateVector (contactOut.m_normal));
					m_userRayCastCallback (body, this, t, xIndex0, zIndex0, &normal, dgInt32 (contactOut.m_shapeId0), userData);
				}

				return t;
			}

			if (txAcc < tzAcc) {
				xIndex0 += xInc;
				tx = txAcc;
				txAcc += stepX;
			} else {
				zIndex0 += zInc;
				tz = tzAcc;
				tzAcc += stepZ;
			}
		} while ((tx <= dgFloat32 (1.0f)) || (tz <= dgFloat32 (1.0f)));
	}

	// if no cell was hit, return a large value
	return dgFloat32 (1.2f);

}


void dgCollisionHeightField::GetVertexListIndexList (const dgVector& p0, const dgVector& p1, dgMeshVertexListIndexList &data) const
{
	dgAssert (0);
	data.m_vertexCount = 0;
}

struct dgCollisionHeightFieldShowPolyContext
{
	dgMatrix m_matrix;
	void* m_userData;
	dgCollision::OnDebugCollisionMeshCallback m_callback;
};

dgVector dgCollisionHeightField::SupportVertex (const dgVector& dir, dgInt32* const vertexIndex) const
{
	dgFloat32 maxProject (dgFloat32 (-1.e-20f));
	dgVector support (dgFloat32 (0.0f));
	if (m_elevationDataType == m_float32Bit)  {
		const dgFloat32* const elevation = (dgFloat32*)m_elevationMap;
		for (dgInt32 z = 0; z < m_height - 1; z ++) {
			dgInt32 base = z * m_width;
			dgFloat32 zVal = m_horizontalScale * z;
			for (dgInt32 x = 0; x < m_width; x ++) {
				dgVector p (m_horizontalScale * x, m_verticalScale * elevation[base + x], zVal, dgFloat32 (0.0f));
				dgFloat32 project = dir.DotProduct4(p).m_x;
				if (project > maxProject) {
					maxProject = project;
					support = p;
				}
			}
		}

	} else {
		const dgUnsigned16* const elevation = (dgUnsigned16*)m_elevationMap;
		for (dgInt32 z = 0; z < m_height - 1; z ++) {
			dgInt32 base = z * m_width;
			dgFloat32 zVal = m_horizontalScale * z;
			for (dgInt32 x = 0; x < m_width; x ++) {
				dgVector p (m_horizontalScale * x, m_verticalScale * elevation[base + x], zVal, dgFloat32 (0.0f));
				dgFloat32 project = dir.DotProduct4(p).m_x;
				if (project > maxProject) {
					maxProject = project;
					support = p;
				}
			}
		}
	}
	return support;
}

void dgCollisionHeightField::DebugCollision (const dgMatrix& matrix, dgCollision::OnDebugCollisionMeshCallback callback, void* const userData) const
{
	dgVector points[4];

	dgInt32 base = 0;
	for (dgInt32 z = 0; z < m_height - 1; z ++) {
		switch (m_elevationDataType) 
		{
			case m_float32Bit:
			{
				const dgFloat32* const elevation = (dgFloat32*)m_elevationMap;
				points[0 * 2 + 0] = dgVector ((0 + 0) * m_horizontalScale, m_verticalScale * elevation[base + 0              ], (z + 0) * m_horizontalScale, dgFloat32 (0.0f));
				points[1 * 2 + 0] = dgVector ((0 + 0) * m_horizontalScale, m_verticalScale * elevation[base + 0 + m_width + 0], (z + 1) * m_horizontalScale, dgFloat32 (0.0f));
				break;
			}

			case m_unsigned16Bit:
			{
				const dgUnsigned16* const elevation = (dgUnsigned16*)m_elevationMap;
				points[0 * 2 + 0] = dgVector ((0 + 0) * m_horizontalScale, m_verticalScale * dgFloat32 (elevation[base + 0              ]), (z + 0) * m_horizontalScale, dgFloat32 (0.0f));
				points[1 * 2 + 0] = dgVector ((0 + 0) * m_horizontalScale, m_verticalScale * dgFloat32 (elevation[base + 0 + m_width + 0]), (z + 1) * m_horizontalScale, dgFloat32 (0.0f));
				break;
			}
		}
		points[0 * 2 + 0] = matrix.TransformVector(points[0 * 2 + 0]);
		points[1 * 2 + 0] = matrix.TransformVector(points[1 * 2 + 0]);

		for (dgInt32 x = 0; x < m_width - 1; x ++) {
			dgTriplex triangle[3];
			switch (m_elevationDataType) 
			{
				case m_float32Bit:
				{
					const dgFloat32* const elevation = (dgFloat32*)m_elevationMap;
					points[0 * 2 + 1] = dgVector ((x + 1) * m_horizontalScale, m_verticalScale * elevation[base + x +           1], (z + 0) * m_horizontalScale, dgFloat32 (0.0f));
					points[1 * 2 + 1] = dgVector ((x + 1) * m_horizontalScale, m_verticalScale * elevation[base + x + m_width + 1], (z + 1) * m_horizontalScale, dgFloat32 (0.0f));
					break;
				}


				case m_unsigned16Bit:
				{
					const dgUnsigned16* const elevation = (dgUnsigned16*)m_elevationMap;
					points[0 * 2 + 1] = dgVector ((x + 1) * m_horizontalScale, m_verticalScale * dgFloat32 (elevation[base + x +           1]), (z + 0) * m_horizontalScale, dgFloat32 (0.0f));
					points[1 * 2 + 1] = dgVector ((x + 1) * m_horizontalScale, m_verticalScale * dgFloat32 (elevation[base + x + m_width + 1]), (z + 1) * m_horizontalScale, dgFloat32 (0.0f));
					break;
				}
			}
			points[0 * 2 + 1] = matrix.TransformVector(points[0 * 2 + 1]);
			points[1 * 2 + 1] = matrix.TransformVector(points[1 * 2 + 1]);


			const dgInt32* const indirectIndex = &m_cellIndices[dgInt32 (m_diagonals[z * m_width + x])][0];

			dgInt32 i0 = indirectIndex[0];
			dgInt32 i1 = indirectIndex[1];
			dgInt32 i2 = indirectIndex[2];
			dgInt32 i3 = indirectIndex[3];

			triangle[0].m_x = points[i1].m_x;
			triangle[0].m_y = points[i1].m_y;
			triangle[0].m_z = points[i1].m_z;

			triangle[1].m_x = points[i0].m_x;
			triangle[1].m_y = points[i0].m_y;
			triangle[1].m_z = points[i0].m_z;

			triangle[2].m_x = points[i2].m_x;
			triangle[2].m_y = points[i2].m_y;
			triangle[2].m_z = points[i2].m_z;
			callback (userData, 3, &triangle[0].m_x, m_atributeMap[base]);

			triangle[0].m_x = points[i1].m_x;
			triangle[0].m_y = points[i1].m_y;
			triangle[0].m_z = points[i1].m_z;

			triangle[1].m_x = points[i2].m_x;
			triangle[1].m_y = points[i2].m_y;
			triangle[1].m_z = points[i2].m_z;

			triangle[2].m_x = points[i3].m_x;
			triangle[2].m_y = points[i3].m_y;
			triangle[2].m_z = points[i3].m_z;
			callback (userData, 3, &triangle[0].m_x, m_atributeMap[base]);

			points[0 * 2 + 0] = points[0 * 2 + 1];
			points[1 * 2 + 0] = points[1 * 2 + 1];
		}
		base += m_width;
	}
}


void dgCollisionHeightField::GetLocalAABB (const dgVector& q0, const dgVector& q1, dgVector& boxP0, dgVector& boxP1) const
{
	// the user data is the pointer to the collision geometry
	CalculateMinExtend3d (q0, q1, boxP0, boxP1);

	dgVector p0 (boxP0.Scale4(m_horizontalScaleInv).GetInt());
	dgVector p1 (boxP1.Scale4(m_horizontalScaleInv).GetInt());

	dgAssert (p0.m_ix == dgFastInt (boxP0.m_x * m_horizontalScaleInv));
	dgAssert (p0.m_iz == dgFastInt (boxP0.m_z * m_horizontalScaleInv));
	dgAssert (p1.m_ix == dgFastInt (boxP1.m_x * m_horizontalScaleInv));
	dgAssert (p1.m_iz == dgFastInt (boxP1.m_z * m_horizontalScaleInv));

	dgInt32 x0 = p0.m_ix;
	dgInt32 x1 = p1.m_ix;
	dgInt32 z0 = p0.m_iz;
	dgInt32 z1 = p1.m_iz;

	dgFloat32 minHeight = dgFloat32 (1.0e10f);
	dgFloat32 maxHeight = dgFloat32 (-1.0e10f);
	dgInt32 base = z0 * m_width;
	switch (m_elevationDataType) 
	{
		case m_float32Bit:
		{
			const dgFloat32* const elevation = (dgFloat32*)m_elevationMap;
			for (dgInt32 z = z0; z <= z1; z ++) {
				for (dgInt32 x = x0; x <= x1; x ++) {
					dgFloat32 high = elevation[base + x];
					if (high < minHeight) {
						minHeight = high;
					}
					if (high > maxHeight) {
						maxHeight = high;
					}
				}
				base += m_width;
			}
			break;
		}

		case m_unsigned16Bit:
		{
			const dgUnsigned16* const elevation = (dgUnsigned16*)m_elevationMap;
			for (dgInt32 z = z0; z <= z1; z ++) {
				for (dgInt32 x = x0; x <= x1; x ++) {
					dgFloat32 high = dgFloat32 (elevation[base + x]);
					if (high < minHeight) {
						minHeight = high;
					}
					if (high > maxHeight) {
						maxHeight = high;
					}
				}
				base += m_width;
			}
			break;
		}
	}

	boxP0.m_y = m_verticalScale * minHeight;
	boxP1.m_y = m_verticalScale * maxHeight;
}


void dgCollisionHeightField::GetCollidingFaces (dgPolygonMeshDesc* const data) const
{
	dgVector boxP0;
	dgVector boxP1;

	dgWorld* const world = data->m_objBody->GetWorld();

	// the user data is the pointer to the collision geometry
	CalculateMinExtend3d (data->m_p0, data->m_p1, boxP0, boxP1);
	boxP0 += data->m_boxDistanceTravelInMeshSpace & (data->m_boxDistanceTravelInMeshSpace < dgVector (dgFloat32 (0.0f)));  
	boxP1 += data->m_boxDistanceTravelInMeshSpace & (data->m_boxDistanceTravelInMeshSpace > dgVector (dgFloat32 (0.0f)));  

	boxP0 = (boxP0.GetMax(dgVector (dgFloat32 (0.0f))) & m_yMask) + boxP0.AndNot(m_yMask);
	boxP1 = (boxP1.GetMax(dgVector (dgFloat32 (0.0f))) & m_yMask) + boxP1.AndNot(m_yMask);

	dgVector p0 (boxP0.Scale4(m_horizontalScaleInv).GetInt());
	dgVector p1 (boxP1.Scale4(m_horizontalScaleInv).GetInt());

	dgAssert (p0.m_ix == dgFastInt (boxP0.m_x * m_horizontalScaleInv));
	dgAssert (p0.m_iz == dgFastInt (boxP0.m_z * m_horizontalScaleInv));
	dgAssert (p1.m_ix == dgFastInt (boxP1.m_x * m_horizontalScaleInv));
	dgAssert (p1.m_iz == dgFastInt (boxP1.m_z * m_horizontalScaleInv));

	dgInt32 x0 = p0.m_ix;
	dgInt32 x1 = p1.m_ix;
	dgInt32 z0 = p0.m_iz;
	dgInt32 z1 = p1.m_iz;

	dgFloat32 minHeight = dgFloat32 (1.0e10f);
	dgFloat32 maxHeight = dgFloat32 (-1.0e10f);
	dgInt32 base = z0 * m_width;
	switch (m_elevationDataType) 
	{
		case m_float32Bit:
		{
			const dgFloat32* const elevation = (dgFloat32*)m_elevationMap;
			for (dgInt32 z = z0; z <= z1; z ++) {
				for (dgInt32 x = x0; x <= x1; x ++) {
					dgFloat32 high = elevation[base + x];
					if (high < minHeight) {
						minHeight = high;
					}
					if (high > maxHeight) {
						maxHeight = high;
					}
				}
				base += m_width;
			}
			break;
		}

		case m_unsigned16Bit:
		{
			const dgUnsigned16* const elevation = (dgUnsigned16*)m_elevationMap;
			for (dgInt32 z = z0; z <= z1; z ++) {
				for (dgInt32 x = x0; x <= x1; x ++) {
					dgFloat32 high = dgFloat32 (elevation[base + x]);
					if (high < minHeight) {
						minHeight = high;
					}
					if (high > maxHeight) {
						maxHeight = high;
					}
				}
				base += m_width;
			}
			break;
		}
	}

	minHeight *= m_verticalScale;
	maxHeight *= m_verticalScale;

	if (!((maxHeight < boxP0.m_y) || (minHeight > boxP1.m_y))) {
		// scan the vertices's intersected by the box extend
		base = (z1 - z0 + 1) * (x1 - x0 + 1) + 2 * (z1 - z0) * (x1 - x0);
		while (base > m_instanceData->m_vertexCount[data->m_threadNumber]) {
			AllocateVertex(world, data->m_threadNumber);
		}

		dgInt32 vertexIndex = 0;
		base = z0 * m_width;
		dgVector* const vertex = m_instanceData->m_vertex[data->m_threadNumber];

		switch (m_elevationDataType) 
		{
			case m_float32Bit:
			{
				const dgFloat32* const elevation = (dgFloat32*)m_elevationMap;
				for (dgInt32 z = z0; z <= z1; z ++) {
					dgFloat32 zVal = m_horizontalScale * z;
					for (dgInt32 x = x0; x <= x1; x ++) {
						vertex[vertexIndex] = dgVector(m_horizontalScale * x, m_verticalScale * elevation[base + x], zVal, dgFloat32 (0.0f));
						vertexIndex ++;
						dgAssert (vertexIndex <= m_instanceData->m_vertexCount[data->m_threadNumber]); 
					}
					base += m_width;
				}
				break;
			}

			case m_unsigned16Bit:
			{
				const dgUnsigned16* const elevation = (dgUnsigned16*)m_elevationMap;
				for (dgInt32 z = z0; z <= z1; z ++) {
					dgFloat32 zVal = m_horizontalScale * z;
					for (dgInt32 x = x0; x <= x1; x ++) {
						vertex[vertexIndex] = dgVector(m_horizontalScale * x, m_verticalScale * dgFloat32 (elevation[base + x]), zVal, dgFloat32 (0.0f));
						vertexIndex ++;
						dgAssert (vertexIndex <= m_instanceData->m_vertexCount[data->m_threadNumber]); 
					}
					base += m_width;
				}
				break;
			}
		}
	
		dgInt32 normalBase = vertexIndex;
		vertexIndex = 0;
		dgInt32 index = 0;
		dgInt32 faceCount = 0;
		dgInt32 step = x1 - x0 + 1;
		dgInt32* const indices = data->m_globalFaceVertexIndex;
		dgInt32* const faceIndexCount = data->m_meshData.m_globalFaceIndexCount;
		dgInt32 faceSize = dgInt32 (m_horizontalScale * dgFloat32 (2.0f)); 

		for (dgInt32 z = z0; (z < z1) && (faceCount < DG_MAX_COLLIDING_FACES); z ++) {
			dgInt32 zStep = z * m_width;
			for (dgInt32 x = x0; (x < x1) && (faceCount < DG_MAX_COLLIDING_FACES); x ++) {
				const dgInt32* const indirectIndex = &m_cellIndices[dgInt32 (m_diagonals[zStep + x])][0];

				dgInt32 vIndex[4];
				vIndex[0] = vertexIndex;
				vIndex[1] = vertexIndex + 1;
				vIndex[2] = vertexIndex + step;
				vIndex[3] = vertexIndex + step + 1;

				const dgInt32 i0 = vIndex[indirectIndex[0]];
				const dgInt32 i1 = vIndex[indirectIndex[1]];
				const dgInt32 i2 = vIndex[indirectIndex[2]];
				const dgInt32 i3 = vIndex[indirectIndex[3]];

				const dgVector e0 (vertex[i0] - vertex[i1]);
				const dgVector e1 (vertex[i2] - vertex[i1]);
				const dgVector e2 (vertex[i3] - vertex[i1]);
				dgVector n0 (e0 *  e1);
				dgVector n1 (e1 *  e2);

				//normalBase 
				const dgInt32 normalIndex0 = normalBase;
				vertex[normalIndex0] = n0.CompProduct4(n0.InvMagSqrt());
				dgAssert  (dgAbsf(vertex[normalIndex0] % vertex[normalIndex0] - dgFloat32 (1.0f)) < dgFloat32 (1.0e-6f));

				const dgInt32 normalIndex1 = normalBase + 1;
				vertex[normalIndex1] = n1.CompProduct4(n1.InvMagSqrt());
				dgAssert  (dgAbsf(vertex[normalIndex1] % vertex[normalIndex1] - dgFloat32 (1.0f)) < dgFloat32 (1.0e-6f));

				faceIndexCount[faceCount] = 3;
				indices[index + 0 + 0] = i2;
				indices[index + 0 + 1] = i1;
				indices[index + 0 + 2] = i0;
				indices[index + 0 + 3] = m_atributeMap[zStep + x];
				indices[index + 0 + 4] = normalIndex0;
				indices[index + 0 + 5] = normalIndex0;
				indices[index + 0 + 6] = normalIndex0;
				indices[index + 0 + 7] = normalIndex0;
				indices[index + 0 + 8] = faceSize;

				faceIndexCount[faceCount + 1] = 3;
				indices[index + 9 + 0] = i1;
				indices[index + 9 + 1] = i2;
				indices[index + 9 + 2] = i3;
				indices[index + 9 + 3] = m_atributeMap[zStep + x];
				indices[index + 9 + 4] = normalIndex1;
				indices[index + 9 + 5] = normalIndex1;
				indices[index + 9 + 6] = normalIndex1;
				indices[index + 9 + 7] = normalIndex1;
				indices[index + 9 + 8] = faceSize;

				dgFloat32 dist (vertex[normalIndex0] % (vertex[i3] - vertex[i1]));
				if (dist < -dgFloat32 (1.0e-3f)) {
					indices[index + 0 + 5] = normalIndex1;
					indices[index + 9 + 5] = normalIndex0;
				}

				index += 9 * 2;
				normalBase += 2;
				faceCount += 2;
				vertexIndex ++;
			}
			vertexIndex ++;
		}

		#ifdef _DEBUG
			dgAssert (faceCount < DG_MAX_COLLIDING_FACES);
			if (faceCount >= DG_MAX_COLLIDING_FACES) {
				dgTrace (("buffer Over float, try using a lower resolution mesh for collision\n"));
			}
		#endif

		const int maxIndex = index;
		dgInt32 stepBase = (x1 - x0) * (2 * 9);
		for (dgInt32 z = z0; z < z1; z ++) {
			const dgInt32 diagBase = m_width * z;
			//const dgInt32 vertexBase = (z - z0) * step;
			//const dgInt32 triangleIndexBase = vertexBase * (2 * 9);
			const dgInt32 triangleIndexBase = (z - z0) * stepBase;
			for (dgInt32 x = x0; x < (x1 - 1); x ++) {
				int index = (x - x0) * (2 * 9) + triangleIndexBase;
				if (index < maxIndex) {
					const dgInt32 code = (m_diagonals[diagBase + x] << 1) + m_diagonals[diagBase + x + 1];
					const dgInt32* const edgeMap = &m_horizontalEdgeMap[code][0];
				
					//dgInt32* const triangles = &indices[(x - x0) * (2 * 9) + triangleIndexBase];
					dgInt32* const triangles = &indices[index];
					const dgInt32 i0 = triangles[edgeMap[0]];
					const dgInt32 i1 = triangles[edgeMap[1]];
					const dgInt32 i2 = triangles[edgeMap[2]];

					const dgVector& origin = vertex[i0];
					const dgVector& testPoint = vertex[i1];
					const dgVector& normal = vertex[i2];
					dgFloat32 dist (normal % (testPoint - origin));

					if (dist < -dgFloat32 (1.0e-3f)) {
						const dgInt32 i3 = edgeMap[3];
						const dgInt32 i4 = edgeMap[4];
						const dgInt32 i5 = edgeMap[5];
						const dgInt32 i6 = edgeMap[6];
						triangles[i3] = triangles[i6];
						triangles[i4] = triangles[i5];
					}
				}
			}
		}


		//dgInt32 stepBase = step * (2 * 9);
		for (dgInt32 x = x0; x < x1; x ++) {
			const dgInt32 triangleIndexBase = (x - x0) * (2 * 9);
			for (dgInt32 z = z0; z < (z1 - 1); z ++) {	
				int index = (z - z0) * stepBase + triangleIndexBase;
				if (index < maxIndex) {
					const dgInt32 diagBase = m_width * z;
					const dgInt32 code = (m_diagonals[diagBase + x] << 1) + m_diagonals[diagBase + m_width + x];
					const dgInt32* const edgeMap = &m_verticalEdgeMap[code][0];

					//dgInt32* const triangles = &indices[(z - z0) * stepBase + triangleIndexBase];
					dgInt32* const triangles = &indices[index];
					const dgInt32 i0 = triangles[edgeMap[0]];
					const dgInt32 i1 = triangles[edgeMap[1] + stepBase];
					const dgInt32 i2 = triangles[edgeMap[2]];

					const dgVector& origin = vertex[i0];
					const dgVector& testPoint = vertex[i1];
					const dgVector& normal = vertex[i2];
					dgFloat32 dist (normal % (testPoint - origin));

					if (dist < -dgFloat32 (1.0e-3f)) {
						const dgInt32 i3 = edgeMap[3];
						const dgInt32 i4 = edgeMap[4] + stepBase;
						const dgInt32 i5 = edgeMap[5];
						const dgInt32 i6 = edgeMap[6] + stepBase;
						triangles[i3] = triangles[i6];
						triangles[i4] = triangles[i5];
					}
				}
			}
		}

		dgInt32 stride = sizeof (dgVector) / sizeof (dgFloat32);
		dgInt32 faceCount0 = 0; 
		dgInt32 faceIndexCount0 = 0; 
		dgInt32 faceIndexCount1 = 0; 

		dgInt32* const address = data->m_meshData.m_globalFaceIndexStart;
		dgFloat32* const hitDistance = data->m_meshData.m_globalHitDistance;
		
		if (data->m_doContinuesCollisionTest) {
			dgFastRayTest ray (dgVector (dgFloat32 (0.0f)), data->m_boxDistanceTravelInMeshSpace);
			for (dgInt32 i = 0; i < faceCount; i ++) {
				const dgInt32* const indexArray = &indices[faceIndexCount1]; 
				const dgVector& faceNormal = vertex[indexArray[4]];
				dgFloat32 dist = data->PolygonBoxRayDistance (faceNormal, 3, indexArray, stride, &vertex[0].m_x, ray);
				if (dist < dgFloat32 (1.0f)) {
					hitDistance[faceCount0] = dist;
					address[faceCount0] = faceIndexCount0;
					memcpy (&indices[faceIndexCount0], indexArray, 9 * sizeof (dgInt32));
					faceCount0 ++;
					faceIndexCount0 += 9;
				}
				faceIndexCount1 += 9;
			}
		} else {
			for (dgInt32 i = 0; i < faceCount; i ++) {
				const dgInt32* const indexArray = &indices[faceIndexCount1]; 
				const dgVector& faceNormal = vertex[indexArray[4]];
				dgFloat32 dist = data->PolygonBoxDistance (faceNormal, 3, indexArray, stride, &vertex[0].m_x);
				if (dist > dgFloat32 (0.0f)) {
					hitDistance[faceCount0] = dist;
					address[faceCount0] = faceIndexCount0;
					memcpy (&indices[faceIndexCount0], indexArray, 9 * sizeof (dgInt32));
					faceCount0 ++;
					faceIndexCount0 += 9;
				}
				faceIndexCount1 += 9;
			}
		}

		if (faceCount0) {
			// initialize the callback data structure
			data->m_faceCount = faceCount0;
			data->m_vertex = &vertex[0].m_x;
			data->m_faceVertexIndex = indices;
			data->m_faceIndexStart = address;
			data->m_hitDistance = hitDistance;
			data->m_faceIndexCount = faceIndexCount;
			data->m_vertexStrideInBytes = sizeof (dgVector);

			if (GetDebugCollisionCallback()) { 
				dgTriplex triplex[3];
				const dgMatrix& matrix = data->m_polySoupCollision->GetGlobalMatrix();
				for (dgInt32 i = 0; i < data->m_faceCount; i ++) {
					dgInt32 base = address[i];
					for (dgInt32 j = 0; j < 3; j ++) {
						dgInt32 index = data->m_faceVertexIndex[base + j];
						dgVector p (matrix.TransformVector(vertex[index]));
						triplex[j].m_x = p.m_x;
						triplex[j].m_y = p.m_y;
						triplex[j].m_z = p.m_z;
					}
					GetDebugCollisionCallback() (data->m_polySoupBody, data->m_objBody, data->m_faceVertexIndex[base + 4], 3, &triplex[0].m_x, sizeof (dgTriplex));
				}
			}
		}


		#ifdef _DEBUG
			for (dgInt32 i = 0; i < data->m_faceCount; i ++) {
				dgInt32 base = address[i];
				const dgInt32* const localIndexArray = &data->m_faceVertexIndex[base];

				int index = data->GetNormalIndex (localIndexArray, 3);
				dgVector n (vertex[index]);
				dgVector p0 (vertex[data->m_faceVertexIndex[base + 0]]);
				dgVector p1 (vertex[data->m_faceVertexIndex[base + 1]]);
				dgVector p2 (vertex[data->m_faceVertexIndex[base + 2]]);

				dgVector n1 ((p1 - p0) * (p2 - p0));
				n1 = n1.CompProduct4(n1.InvMagSqrt());

				dgMatrix polygonMatrix;
				polygonMatrix[0] = p1 - p0;
				polygonMatrix[0] = polygonMatrix[0].CompProduct4 (polygonMatrix[0].DotProduct4(polygonMatrix[0]).InvSqrt());
				polygonMatrix[1] = n * polygonMatrix[0];
				polygonMatrix[2] = n;
				polygonMatrix[3] = dgVector::m_wOne;
				dgAssert (polygonMatrix.TestOrthogonal());
			}
		#endif
	}
}

