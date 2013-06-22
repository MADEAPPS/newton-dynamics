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

dgCollisionHeightField::dgCollisionHeightField(
	dgWorld* const world,
	dgInt32 width, 
	dgInt32 height, 
	dgInt32 contructionMode, 
	const dgFloat32* const elevationMap, 
	const dgInt8* const atributeMap, 
	dgFloat32 horizontalScale)
	:dgCollisionMesh (world, m_heightField)
{
	m_userRayCastCallback = NULL;
	m_rtti |= dgCollisionHeightField_RTTI;
	m_width = width;
	m_height = height;
	m_diagonalMode = contructionMode;
	m_horizontalScale = horizontalScale;

	m_elevationMap = (dgFloat32 *)dgMallocStack(m_width * m_height * sizeof (dgFloat32));
	memcpy (m_elevationMap, elevationMap, m_width * m_height * sizeof (dgFloat32));

	dgInt32 attibutePaddedMapSize = (m_width * m_height + 4) & -4; 
	m_atributeMap = (dgInt8 *)dgMallocStack(attibutePaddedMapSize * sizeof (dgInt8));
	memcpy (m_atributeMap, atributeMap, m_width * m_height * sizeof (dgInt8));

	dgFloat32 y0 = dgFloat32 (dgFloat32 (1.0e10f));
	dgFloat32 y1 = dgFloat32 (-dgFloat32 (1.0e10f));
	for (dgInt32 i = 0; i < m_width * m_height; i ++) {
		y0 = dgMin(y0, dgFloat32 (m_elevationMap[i]));
		y1 = dgMax(y1, dgFloat32 (m_elevationMap[i]));
	}


	m_minBox = dgVector (dgFloat32 (dgFloat32 (0.0f)), y0, dgFloat32 (dgFloat32 (0.0f)), dgFloat32 (0.0f)); 
	m_maxBox = dgVector (dgFloat32 (m_width-1) * m_horizontalScale, y1, dgFloat32 (m_height-1) * m_horizontalScale, dgFloat32 (0.0f)); 

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

dgCollisionHeightField::dgCollisionHeightField (dgWorld* const world, dgDeserialize deserialization, void* const userData)
	:dgCollisionMesh (world, deserialization, userData)
{
	dgAssert (m_rtti | dgCollisionHeightField_RTTI);

	m_userRayCastCallback = NULL;
	deserialization (userData, &m_width, sizeof (dgInt32));
	deserialization (userData, &m_height, sizeof (dgInt32));
	deserialization (userData, &m_diagonalMode, sizeof (dgInt32));
	deserialization (userData, &m_horizontalScale, sizeof (dgFloat32));
	deserialization (userData, &m_minBox.m_x, sizeof (dgVector)); 
	deserialization (userData, &m_maxBox.m_x, sizeof (dgVector)); 

	m_elevationMap = (dgFloat32 *)dgMallocStack(m_width * m_height * sizeof (dgFloat32));

	dgInt32 attibutePaddedMapSize = (m_width * m_height + 4) & -4; 
	m_atributeMap = (dgInt8 *)dgMallocStack(attibutePaddedMapSize * sizeof (dgInt8));

	deserialization (userData, m_elevationMap, m_width * m_height * sizeof (dgFloat32));
	deserialization (userData, m_atributeMap, attibutePaddedMapSize * sizeof (dgInt8));


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
}

void dgCollisionHeightField::Serialize(dgSerialize callback, void* const userData) const
{
	SerializeLow(callback, userData);

	callback (userData, &m_width, sizeof (dgInt32));
	callback (userData, &m_height, sizeof (dgInt32));
	callback (userData, &m_diagonalMode, sizeof (dgInt32));
	callback (userData, &m_horizontalScale, sizeof (dgFloat32));
	callback (userData, &m_minBox.m_x, sizeof (dgVector)); 
	callback (userData, &m_maxBox.m_x, sizeof (dgVector)); 

	callback (userData, m_elevationMap, m_width * m_height * sizeof (dgFloat32));

	dgInt32 attibutePaddedMapSize = (m_width * m_height + 4) & -4; 
	callback (userData, m_atributeMap, attibutePaddedMapSize * sizeof (dgInt8));
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



void dgCollisionHeightField::GetCollisionInfo(dgCollisionInfo* const info) const
{
	dgCollision::GetCollisionInfo(info);

	dgCollisionInfo::dgHeightMapCollisionData& data = info->m_heightFieldCollision;
	data.m_width = m_width;
	data.m_height = m_height;
	data.m_gridsDiagonals = m_diagonalMode;
	data.m_horizonalScale = m_horizontalScale;
	data.m_verticalScale = dgFloat32 (1.0f);
	data.m_atributes = m_atributeMap;
	data.m_elevation = m_elevationMap;
}


void dgCollisionHeightField::CalculateMinExtend2d (const dgVector& p0, const dgVector& p1, dgVector& boxP0, dgVector& boxP1) const
{
	dgFloat32 x0 = dgMin (p0.m_x, p1.m_x) - dgFloat32 (1.0e-3f);
	dgFloat32 z0 = dgMin (p0.m_z, p1.m_z) - dgFloat32 (1.0e-3f);

	dgFloat32 x1 = dgMax (p0.m_x, p1.m_x) + dgFloat32 (1.0e-3f);
	dgFloat32 z1 = dgMax (p0.m_z, p1.m_z) + dgFloat32 (1.0e-3f);

	x0 = m_horizontalScale * dgFloor (x0 * m_horizontalScaleInv);
	z0 = m_horizontalScale * dgFloor (z0 * m_horizontalScaleInv);
	x1 = m_horizontalScale * dgFloor (x1 * m_horizontalScaleInv) + m_horizontalScale;
	z1 = m_horizontalScale * dgFloor (z1 * m_horizontalScaleInv) + m_horizontalScale;

	boxP0.m_x = dgMax (x0, m_minBox.m_x);
	boxP0.m_z = dgMax (z0, m_minBox.m_z);
	boxP0.m_y = -dgFloat32 (1.0e10f);
	boxP0.m_w = dgFloat32 (0.0f);

	boxP1.m_x = dgMin (x1, m_maxBox.m_x);
	boxP1.m_z = dgMin (z1, m_maxBox.m_z);
	boxP1.m_y = dgFloat32 (1.0e10f);
	boxP1.m_w = dgFloat32 (0.0f);
}

void dgCollisionHeightField::CalculateMinExtend3d (const dgVector& p0, const dgVector& p1, dgVector& boxP0, dgVector& boxP1) const
{
	dgAssert (p0.m_x <= p1.m_x);
	dgAssert (p0.m_y <= p1.m_y);
	dgAssert (p0.m_z <= p1.m_z);

	dgFloat32 x0 = m_horizontalScale * dgFloor ((p0.m_x - dgFloat32 (1.0e-3f)) * m_horizontalScaleInv);
	dgFloat32 z0 = m_horizontalScale * dgFloor ((p0.m_z - dgFloat32 (1.0e-3f)) * m_horizontalScaleInv);
	dgFloat32 x1 = m_horizontalScale * dgFloor ((p1.m_x + dgFloat32 (1.0e-3f)) * m_horizontalScaleInv) + m_horizontalScale;
	dgFloat32 z1 = m_horizontalScale * dgFloor ((p1.m_z + dgFloat32 (1.0e-3f)) * m_horizontalScaleInv) + m_horizontalScale;

	boxP0.m_x = dgMax (x0, m_minBox.m_x);
	boxP0.m_z = dgMax (z0, m_minBox.m_z);
	boxP0.m_y = p0.m_y - dgFloat32 (1.0e-3f);
	boxP0.m_w = dgFloat32 (0.0f);

	boxP1.m_x = dgMin (x1, m_maxBox.m_x);
	boxP1.m_z = dgMin (z1, m_maxBox.m_z);
	boxP1.m_y = p1.m_y + dgFloat32 (1.0e-3f);
	boxP1.m_w = dgFloat32 (0.0f);
}


dgFloat32 dgCollisionHeightField::RayCastCell (const dgFastRayTest& ray, dgInt32 xIndex0, dgInt32 zIndex0, dgVector& normalOut) const
{
	dgVector points[4];
	dgInt32 triangle[3];

	// get the 3d point at the corner of the cell
	if ((xIndex0 < 0) || (zIndex0 < 0) || (xIndex0 >= (m_width - 1)) || (zIndex0 >= (m_height - 1))) {
		return dgFloat32 (1.2f);
	}
	
	dgInt32 base = zIndex0 * m_width + xIndex0;
	
	points[0 * 2 + 0] = dgVector ((xIndex0 + 0) * m_horizontalScale, dgFloat32 (m_elevationMap[base]),			     (zIndex0 + 0) * m_horizontalScale, dgFloat32 (0.0f));
	points[0 * 2 + 1] = dgVector ((xIndex0 + 1) * m_horizontalScale, dgFloat32 (m_elevationMap[base + 1]),           (zIndex0 + 0) * m_horizontalScale, dgFloat32 (0.0f));
	points[1 * 2 + 1] = dgVector ((xIndex0 + 1) * m_horizontalScale, dgFloat32 (m_elevationMap[base + m_width + 1]), (zIndex0 + 1) * m_horizontalScale, dgFloat32 (0.0f));
	points[1 * 2 + 0] = dgVector ((xIndex0 + 0) * m_horizontalScale, dgFloat32 (m_elevationMap[base + m_width + 0]), (zIndex0 + 1) * m_horizontalScale, dgFloat32 (0.0f));
	
	dgFloat32 t = dgFloat32 (1.2f);
	if (!m_diagonalMode) {
		triangle[0] = 1;
		triangle[1] = 2;
		triangle[2] = 3;

		dgVector e10 (points[2] - points[1]);
		dgVector e20 (points[3] - points[1]);
		dgVector normal (e10 * e20);
		normal = normal.Scale3 (dgRsqrt (normal % normal));
		t = ray.PolygonIntersect (normal, &points[0].m_x, sizeof (dgVector), triangle, 3);
		if (t < dgFloat32 (1.0f)){
			normalOut = normal;
			return t;
		}

		triangle[0] = 1;
		triangle[1] = 0;
		triangle[2] = 2;

		dgVector e30 (points[0] - points[1]);
		normal = e30 * e10;
		normal = normal.Scale3 (dgRsqrt (normal % normal));
		t = ray.PolygonIntersect (normal, &points[0].m_x, sizeof (dgVector), triangle, 3);
		if (t < dgFloat32 (1.0f)){
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
		normal = normal.Scale3 (dgRsqrt (normal % normal));
		t = ray.PolygonIntersect (normal, &points[0].m_x, sizeof (dgVector), triangle, 3);
		if (t < dgFloat32 (1.0f)){
			normalOut = normal;
			return t;
		}

		triangle[0] = 0;
		triangle[1] = 3;
		triangle[2] = 1;

		dgVector e30 (points[1] - points[0]);
		normal = e20 * e30;
		normal = normal.Scale3 (dgRsqrt (normal % normal));
		t = ray.PolygonIntersect (normal, &points[0].m_x, sizeof (dgVector), triangle, 3);
		if (t < dgFloat32 (1.0f)){
			normalOut = normal;
			return t;
		}
	}
	return t;
}


dgFloat32 dgCollisionHeightField::RayCast (const dgVector& q0, const dgVector& q1, dgContactPoint& contactOut, const dgBody* const body, void* const userData) const
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
		dgFloat32 stepZ;
		dgFloat32 tz;
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
			dgFloat32 t = RayCastCell (ray, xIndex0, zIndex0, normalOut);
			if (t < dgFloat32 (1.0f)) {
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
				tz = txAcc;
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
	OnDebugCollisionMeshCallback m_callback;
};


void dgCollisionHeightField::DebugCollision (const dgMatrix& matrix, OnDebugCollisionMeshCallback callback, void* const userData) const
{
	dgVector points[4];

	dgInt32 base = 0;
	for (dgInt32 z = 0; z < m_height - 1; z ++) {
		points[0 * 2 + 0] = matrix.TransformVector(dgVector ((0 + 0) * m_horizontalScale, dgFloat32 (m_elevationMap[base + 0              ]), (z + 0) * m_horizontalScale, dgFloat32 (0.0f)));
		points[1 * 2 + 0] = matrix.TransformVector(dgVector ((0 + 0) * m_horizontalScale, dgFloat32 (m_elevationMap[base + 0 + m_width + 0]), (z + 1) * m_horizontalScale, dgFloat32 (0.0f)));

		for (dgInt32 x = 0; x < m_width - 1; x ++) {
			dgTriplex triangle[3];
			points[0 * 2 + 1] = matrix.TransformVector(dgVector ((x + 1) * m_horizontalScale, dgFloat32 (m_elevationMap[base + x +           1]), (z + 0) * m_horizontalScale, dgFloat32 (0.0f)));
			points[1 * 2 + 1] = matrix.TransformVector(dgVector ((x + 1) * m_horizontalScale, dgFloat32 (m_elevationMap[base + x + m_width + 1]), (z + 1) * m_horizontalScale, dgFloat32 (0.0f)));
			
			if (m_diagonalMode) {
				triangle[0].m_x = points[0].m_x;
				triangle[0].m_y = points[0].m_y;
				triangle[0].m_z = points[0].m_z;

				triangle[1].m_x = points[2].m_x;
				triangle[1].m_y = points[2].m_y;
				triangle[1].m_z = points[2].m_z;

				triangle[2].m_x = points[1].m_x;
				triangle[2].m_y = points[1].m_y;
				triangle[2].m_z = points[1].m_z;
				callback (userData, 3, &triangle[0].m_x, m_atributeMap[base]);

				triangle[0].m_x = points[1].m_x;
				triangle[0].m_y = points[1].m_y;
				triangle[0].m_z = points[1].m_z;

				triangle[1].m_x = points[2].m_x;
				triangle[1].m_y = points[2].m_y;
				triangle[1].m_z = points[2].m_z;

				triangle[2].m_x = points[3].m_x;
				triangle[2].m_y = points[3].m_y;
				triangle[2].m_z = points[3].m_z;
				callback (userData, 3, &triangle[0].m_x, m_atributeMap[base]);

			} else {
				triangle[0].m_x = points[0].m_x;
				triangle[0].m_y = points[0].m_y;
				triangle[0].m_z = points[0].m_z;

				triangle[1].m_x = points[2].m_x;
				triangle[1].m_y = points[2].m_y;
				triangle[1].m_z = points[2].m_z;

				triangle[2].m_x = points[3].m_x;
				triangle[2].m_y = points[3].m_y;
				triangle[2].m_z = points[3].m_z;
				callback (userData, 3, &triangle[0].m_x, m_atributeMap[base]);

				triangle[0].m_x = points[0].m_x;
				triangle[0].m_y = points[0].m_y;
				triangle[0].m_z = points[0].m_z;

				triangle[1].m_x = points[3].m_x;
				triangle[1].m_y = points[3].m_y;
				triangle[1].m_z = points[3].m_z;

				triangle[2].m_x = points[1].m_x;
				triangle[2].m_y = points[1].m_y;
				triangle[2].m_z = points[1].m_z;
				callback (userData, 3, &triangle[0].m_x, m_atributeMap[base]);
			}
			points[0 * 2 + 0] = points[0 * 2 + 1];
			points[1 * 2 + 0] = points[1 * 2 + 1];
		}
		base += m_width;
	}
}


void dgCollisionHeightField::GetLocalAABB (const dgVector& p0, const dgVector& p1, dgVector& boxP0, dgVector& boxP1) const
{
	// the user data is the pointer to the collision geometry
	CalculateMinExtend3d (p0, p1, boxP0, boxP1);

	dgInt32 x0 = dgFastInt (boxP0.m_x * m_horizontalScaleInv);
	dgInt32 x1 = dgFastInt (boxP1.m_x * m_horizontalScaleInv);
	dgInt32 z0 = dgFastInt (boxP0.m_z * m_horizontalScaleInv);
	dgInt32 z1 = dgFastInt (boxP1.m_z * m_horizontalScaleInv);

	dgFloat32 minHeight = dgFloat32 (1.0e10f);
	dgFloat32 maxHeight = dgFloat32 (-1.0e10f);
	dgInt32 base = z0 * m_width;
	for (dgInt32 z = z0; z <= z1; z ++) {
		for (dgInt32 x = x0; x <= x1; x ++) {
			dgFloat32 high = m_elevationMap[base + x];
			if (high < minHeight) {
				minHeight = high;
			}
			if (high > maxHeight) {
				maxHeight = high;
			}
		}
		base += m_width;
	}

	boxP0.m_y = minHeight;
	boxP1.m_y = maxHeight;
}


void dgCollisionHeightField::GetCollidingFaces (dgPolygonMeshDesc* const data) const
{
	dgVector boxP0;
	dgVector boxP1;

	// the user data is the pointer to the collision geometry
	CalculateMinExtend3d (data->m_boxP0, data->m_boxP1, boxP0, boxP1);

	dgWorld* const world = data->m_objBody->GetWorld();
	dgInt32 x0 = dgFastInt (boxP0.m_x * m_horizontalScaleInv);
	dgInt32 x1 = dgFastInt (boxP1.m_x * m_horizontalScaleInv);
	dgInt32 z0 = dgFastInt (boxP0.m_z * m_horizontalScaleInv);
	dgInt32 z1 = dgFastInt (boxP1.m_z * m_horizontalScaleInv);

	dgFloat32 minHeight = dgFloat32 (1.0e10f);
	dgFloat32 maxHeight = dgFloat32 (-1.0e10f);

	dgInt32 base = z0 * m_width;
	for (dgInt32 z = z0; z <= z1; z ++) {
		for (dgInt32 x = x0; x <= x1; x ++) {
			dgFloat32 high = m_elevationMap[base + x];
			if (high < minHeight) {
				minHeight = high;
			}
			if (high > maxHeight) {
				maxHeight = high;
			}
		}
		base += m_width;
	}



	if (!((maxHeight < boxP0.m_y) || (minHeight > boxP1.m_y))) {
		// scan the vertices's intersected by the box extend
		base = (z1 - z0 + 1) * (x1 - x0 + 1) + 2 * (z1 - z0) * (x1 - x0);
		while (base > m_instanceData->m_vertexCount[data->m_threadNumber]) {
			AllocateVertex(world, data->m_threadNumber);
		}

		dgInt32 vertexIndex = 0;
		base = z0 * m_width;
		dgVector* const vertex = m_instanceData->m_vertex[data->m_threadNumber];

		for (dgInt32 z = z0; z <= z1; z ++) {
			for (dgInt32 x = x0; x <= x1; x ++) {
				vertex[vertexIndex] = dgVector(m_horizontalScale * x, m_elevationMap[base + x], m_horizontalScale * z, dgFloat32 (0.0f));
				vertexIndex ++;
				dgAssert (vertexIndex <= m_instanceData->m_vertexCount[data->m_threadNumber]); 
			}
			base += m_width;
		}
	

		dgInt32 normalBase = vertexIndex;
		vertexIndex = 0;
		dgInt32 index = 0;
		dgInt32 faceCount = 0;
		dgInt32 step = x1 - x0 + 1;
		dgInt32* const indices = data->m_globalFaceVertexIndex;
		//dgInt32* const faceIndexCount = data->m_globalFaceIndexCount;
		dgInt32* const faceIndexCount = data->m_meshData.m_globalFaceIndexCount;
		dgInt32 faceSize = dgInt32 (m_horizontalScale * dgFloat32 (2.0f)); 
		if (m_diagonalMode) {
			for (dgInt32 z = z0; z < z1; z ++) {
				dgInt32 zStep = z * m_width;
				for (dgInt32 x = x0; x < x1; x ++) {
					dgInt32 i0 = vertexIndex;
					dgInt32 i1 = vertexIndex + step;
					dgInt32 i2 = vertexIndex + 1;
					dgInt32 i3 = vertexIndex + step + 1;

					// calculate the the normal
					dgVector e0 (vertex[i1] - vertex[i0]);
					dgVector e1 (vertex[i3] - vertex[i0]);
					dgVector e2 (vertex[i2] - vertex[i0]);
					dgVector n1 (e0 *  e1);
					dgVector n0 (e1 *  e2);

					//normalBase 
					//n0 = n0.Scale3 (dgRsqrt(n0 % n0));
					n0 = n0.CompProduct4(n0.DotProduct4(n0).InvSqrt());
					vertex[normalBase] = n0;
					
					//n1 = n1.Scale3 (dgRsqrt(n1 % n1));
					n1 = n1.CompProduct4(n1.DotProduct4(n1).InvSqrt());
					vertex[normalBase + 1] = n1;

					faceIndexCount[faceCount] = 3;
					indices[index + 0 + 0] = i0;
					indices[index + 0 + 1] = i3;
					indices[index + 0 + 2] = i2;
					indices[index + 0 + 3] = m_atributeMap[zStep + x];
					indices[index + 0 + 4] = normalBase;
					indices[index + 0 + 5] = normalBase;
					indices[index + 0 + 6] = normalBase;
					indices[index + 0 + 7] = normalBase;
					indices[index + 0 + 8] = faceSize;

					faceIndexCount[faceCount + 1] = 3;
					indices[index + 9 + 0] = i3;
					indices[index + 9 + 1] = i0;
					indices[index + 9 + 2] = i1;
					indices[index + 9 + 3] = m_atributeMap[zStep + x];
					indices[index + 9 + 4] = normalBase + 1;
					indices[index + 9 + 5] = normalBase + 1;
					indices[index + 9 + 6] = normalBase + 1;
					indices[index + 9 + 7] = normalBase + 1;
					indices[index + 9 + 8] = faceSize;
					
					dgFloat32 dist (n0 % (vertex[i1] - vertex[i2]));
					if (dist < -dgFloat32 (1.0e-3f)) {
						indices[index + 0 + 5] = normalBase + 1;
						indices[index + 9 + 5] = normalBase;
					}

					index += 9 * 2;
					normalBase += 2;
					faceCount += 2;
					vertexIndex ++;
				}
				vertexIndex ++;
			}

			step = x1 - x0;
			for (dgInt32 z = z0; z < z1; z ++) {
				dgInt32 triangleIndexBase = (z - z0) * step * 18;
				for (dgInt32 x = x0; x < (x1 - 1); x ++) {
					dgInt32* const triangles = &indices[triangleIndexBase + (x - x0) * 18];
					const dgVector& origin = vertex[triangles[0]];
					const dgVector& testPoint = vertex[triangles[27]];
					const dgVector& normal = vertex[triangles[4]];

					dgFloat32 dist (normal % (testPoint - origin));
					if (dist < -dgFloat32 (1.0e-3f)) {
						dgInt32 i0 = 6;
						dgInt32 i1 = 27 + 6;
						triangles[i0] = triangles[27 + 4];
						triangles[i1] = triangles[9 + 4];
					}
				}
			}

			for (dgInt32 x = x0; x < x1; x ++) {
				dgInt32 triangleIndexBase = (x - x0) * 18;
				for (dgInt32 z = z0; z < (z1 - 1); z ++) {	
					dgInt32* const triangles = &indices[triangleIndexBase + (z - z0) * step * 18];
					const dgVector& origin = vertex[triangles[9]];
					const dgVector& testPoint = vertex[triangles[step * 18 + 1]];
					const dgVector& normal = vertex[triangles[9 + 4]];
					dgFloat32 dist (normal % (testPoint - origin));

					if (dist < -dgFloat32 (1.0e-3f)) {
						dgInt32 i0 = 9 + 7;
						dgInt32 i1 = step * 18 + 7;
						triangles[i0] = triangles[step * 18 + 4];
						triangles[i1] = triangles[9 + 4];
					}
				}
			}
		} else {
			for (dgInt32 z = z0; z < z1; z ++) {
				dgInt32 zStep = z * m_width;
				for (dgInt32 x = x0; x < x1; x ++) {
					dgInt32 i0 = vertexIndex;
					dgInt32 i1 = vertexIndex + step;
					dgInt32 i2 = vertexIndex + 1;
					dgInt32 i3 = vertexIndex + step + 1;

					// calculate the the normal
					dgVector e0 (vertex[i0] - vertex[i2]);
					dgVector e1 (vertex[i1] - vertex[i2]);
					dgVector e2 (vertex[i3] - vertex[i2]);
					dgVector n1 (e0 *  e1);
					dgVector n0 (e1 *  e2);

					//normalBase 
					//n0 = n0.Scale3 (dgRsqrt(n0 % n0));
					n0 = n0.CompProduct4(n0.DotProduct4(n0).InvSqrt());
					vertex[normalBase] = n0;

					//n1 = n1.Scale3 (dgRsqrt(n1 % n1));
					n1 = n1.CompProduct4(n1.DotProduct4(n1).InvSqrt());
					vertex[normalBase + 1] = n1;

					faceIndexCount[faceCount] = 3;
					indices[index + 0 + 0] = i0;
					indices[index + 0 + 1] = i1;
					indices[index + 0 + 2] = i2;
					indices[index + 0 + 3] = m_atributeMap[zStep + x];
					indices[index + 0 + 4] = normalBase;
					indices[index + 0 + 5] = normalBase;
					indices[index + 0 + 6] = normalBase;
					indices[index + 0 + 7] = normalBase;
					indices[index + 0 + 8] = faceSize;


					faceIndexCount[faceCount + 1] = 3;
					indices[index + 9 + 0] = i2;
					indices[index + 9 + 1] = i1;
					indices[index + 9 + 2] = i3;
					indices[index + 9 + 3] = m_atributeMap[zStep + x];
					indices[index + 9 + 4] = normalBase + 1;
					indices[index + 9 + 5] = normalBase + 1;
					indices[index + 9 + 6] = normalBase + 1;
					indices[index + 9 + 7] = normalBase + 1;
					indices[index + 9 + 8] = faceSize;

					dgFloat32 dist (n0 % (vertex[i3] - vertex[i0]));
					if (dist < -dgFloat32 (1.0e-3f)) {
						indices[index + 0 + 6] = normalBase + 1;
						indices[index + 9 + 5] = normalBase;
					}

					index += 9 * 2;
					normalBase += 2;
					faceCount += 2;
					vertexIndex ++;
				}
				vertexIndex ++;
			}

			step = x1 - x0;
			for (dgInt32 z = z0; z < z1; z ++) {
				dgInt32 triangleIndexBase = (z - z0) * step * 18;
				for (dgInt32 x = x0; x < (x1 - 1); x ++) {
					dgInt32* const triangles = &indices[triangleIndexBase + (x - x0) * 18];
					const dgVector& origin = vertex[triangles[9 + 0]];
					const dgVector& testPoint = vertex[triangles[20]];
					const dgVector& normal = vertex[triangles[9 + 4]];

					dgFloat32 dist (normal % (testPoint - origin));
					if (dist < -dgFloat32 (1.0e-3f)) {
						dgInt32 i0 = 9 + 7;
						dgInt32 i1 = 18 + 5;
						triangles[i0] = triangles[18 + 4];
						triangles[i1] = triangles[9 + 4];
					}
				}
			}

			for (dgInt32 x = x0; x < x1; x ++) {
				dgInt32 triangleIndexBase = (x - x0) * 18;
				for (dgInt32 z = z0; z < (z1 - 1); z ++) {	
					dgInt32* const triangles = &indices[triangleIndexBase + (z - z0) * step * 18];
					const dgVector& origin = vertex[triangles[9]];
					const dgVector& testPoint = vertex[triangles[step * 18 + 1]];
					const dgVector& normal = vertex[triangles[9 + 4]];
					dgFloat32 dist (normal % (testPoint - origin));

					if (dist < -dgFloat32 (1.0e-3f)) {
						dgInt32 i0 = 9 + 6;
						dgInt32 i1 = step * 18 + 7;
						triangles[i0] = triangles[step * 18 + 4];
						triangles[i1] = triangles[9 + 4];
					}
				}
			}
		}

		dgInt32 stride = sizeof (dgVector) / sizeof (dgFloat32);
		dgInt32 faceCount0 = 0; 
		dgInt32 faceIndexCount0 = 0; 
		dgInt32 faceIndexCount1 = 0; 

		dgFastAABBInfo aabb (data->m_boxP0, data->m_boxP1);
		dgInt32* const address = data->m_meshData.m_globalFaceIndexStart;
		dgFloat32* const hitDistance = data->m_meshData.m_globalHitDistance;
		
		if (data->m_doContinuesCollisionTest) {
			dgFastRayTest ray (dgVector (dgFloat32 (0.0f)), data->m_boxDistanceTravelInMeshSpace);
			for (dgInt32 i = 0; i < faceCount; i ++) {
				const dgInt32* const indexArray = &indices[faceIndexCount1]; 
				const dgVector& faceNormal = vertex[indexArray[4]];
				dgFloat32 dist = aabb.PolygonBoxRayDistance (faceNormal, 3, indexArray, stride, &vertex[0].m_x, ray);
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
				dgFloat32 dist = aabb.PolygonBoxDistance (faceNormal, 3, indexArray, stride, &vertex[0].m_x);
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
	}
}

