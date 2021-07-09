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

#include "dCoreStdafx.h"
#include "ndCollisionStdafx.h"
#include "ndContact.h"
#include "ndBodyKinematic.h"
#include "ndShapeInstance.h"
#include "ndShapeHeightfield.h"

dVector ndShapeHeightfield::m_yMask(0xffffffff, 0, 0xffffffff, 0);
dVector ndShapeHeightfield::m_padding(dFloat32(0.25f), dFloat32(0.25f), dFloat32(0.25f), dFloat32(0.0f));
dVector ndShapeHeightfield::m_elevationPadding(dFloat32(0.0f), dFloat32(1.0e10f), dFloat32(0.0f), dFloat32(0.0f));

dInt32 ndShapeHeightfield::m_cellIndices[][4] =
{
	{ 0, 1, 2, 3 },
	{ 1, 3, 0, 2 }
};

ndShapeHeightfield::ndShapeHeightfield(
	dInt32 width, dInt32 height, ndGridConstruction constructionMode,
	dFloat32 verticalScale, dFloat32 horizontalScale_x, dFloat32 horizontalScale_z)
	:ndShapeStaticMesh(m_heightField)
	,m_minBox(dVector::m_zero)
	,m_maxBox(dVector::m_zero)
	,m_atributeMap(width * height)
	,m_elevationMap(width * height)
	,m_verticalScale(verticalScale)
	,m_horizontalScale_x(horizontalScale_x)
	,m_horizontalScale_z(horizontalScale_z)
	,m_horizontalScaleInv_x(dFloat32(1.0f) / horizontalScale_x)
	,m_horizontalScaleInv_z(dFloat32(1.0f) / horizontalScale_z)

	,m_width(width)
	,m_height(height)
	,m_diagonalMode(constructionMode)
{
	dAssert(width >= 2);
	dAssert(height >= 2);
	m_atributeMap.SetCount(width * height);
	m_elevationMap.SetCount(width * height);

	memset(&m_atributeMap[0], 0, sizeof(dInt8) * m_atributeMap.GetCount());
	memset(&m_elevationMap[0], 0, sizeof(dInt16) * m_elevationMap.GetCount());

	CalculateAABB();
}

//ndShapeHeightfield::ndShapeHeightfield(const nd::TiXmlNode* const xmlNode, const char* const assetPath)
ndShapeHeightfield::ndShapeHeightfield(const nd::TiXmlNode* const, const char* const)
	:ndShapeStaticMesh(m_heightField)
{
	dAssert(0);
}

ndShapeHeightfield::~ndShapeHeightfield(void)
{
}

//void ndShapeHeightfield::Save(nd::TiXmlElement* const xmlNode, const char* const assetPath, dInt32 nodeid) const
void ndShapeHeightfield::Save(nd::TiXmlElement* const, const char* const, dInt32) const
{
	dAssert(0);
}

ndShapeInfo ndShapeHeightfield::GetShapeInfo() const
{
	ndShapeInfo info(ndShapeStaticMesh::GetShapeInfo());

	info.m_heightfield.m_width = m_width;
	info.m_heightfield.m_height = m_height;
	info.m_heightfield.m_gridsDiagonals = m_diagonalMode;
	info.m_heightfield.m_verticalScale = m_verticalScale;
	info.m_heightfield.m_horizonalScale_x = m_horizontalScale_x;
	info.m_heightfield.m_horizonalScale_z = m_horizontalScale_z;
	info.m_heightfield.m_elevation = (dInt16*)&m_elevationMap[0];
	info.m_heightfield.m_atributes = (dInt8*)&m_atributeMap[0];

	return info;
}

void ndShapeHeightfield::CalculateAABB()
{
	dInt16 y0 = dInt16(0x7fff);
	dInt16 y1 = dInt16 (-0x7fff);
	for (dInt32 i = m_elevationMap.GetCount()-1; i >= 0; i--)
	{
		y0 = dMin(y0, m_elevationMap[i]);
		y1 = dMax(y1, m_elevationMap[i]);
	}

	m_minBox = dVector(dFloat32(dFloat32(0.0f)), dFloat32 (y0) * m_verticalScale, dFloat32(0.0f), dFloat32(0.0f));
	m_maxBox = dVector(dFloat32(m_width) * m_horizontalScale_x, dFloat32(y1) * m_verticalScale, dFloat32(m_height) * m_horizontalScale_z, dFloat32(0.0f));

	m_boxSize = (m_maxBox - m_minBox) * dVector::m_half;
	m_boxOrigin = (m_maxBox + m_minBox) * dVector::m_half;
}

void ndShapeHeightfield::UpdateElevationMapAabb()
{
	CalculateAABB();
}
/*
dIntersectStatus ndShapeHeightfield::GetTriangleCount(void* const context, const dFloat32* const, dInt32, const dInt32* const, dInt32 indexCount, dFloat32)
{
	ndMeshVertexListIndexList& data = (*(ndMeshVertexListIndexList*)context);

	if ((data.m_triangleCount + indexCount - 2) * 3 > data.m_maxIndexCount) 
	{
		return t_StopSearh;
	}

	data.m_triangleCount += (indexCount - 2);
	dAssert((data.m_triangleCount * 3) <= data.m_maxIndexCount);
	return t_ContinueSearh;
}


dIntersectStatus ndShapeHeightfield::ShowDebugPolygon(void* const context, const dFloat32* const polygon, dInt32 strideInBytes, const dInt32* const indexArray, dInt32 indexCount, dFloat32)
{
	dVector poly[128];
	ndShapeDebugCallback::ndEdgeType edgeType[128];

	dInt32 stride = dInt32(strideInBytes / sizeof(dFloat32));

	ndCollisionBVHShowPolyContext& data = *(ndCollisionBVHShowPolyContext*)context;
	for (dInt32 i = 0; i < indexCount; i++) 
	{
		dVector p(&polygon[indexArray[i] * stride]);
		poly[i] = data.m_matrix.TransformVector(p & dVector::m_triplexMask);
		dInt32 edgeIndexType = (indexArray[i + indexCount + 2]) & D_CONCAVE_EDGE_MASK;
		edgeType[i] = edgeIndexType ? ndShapeDebugCallback::m_open : ndShapeDebugCallback::m_shared;
	}
	//dAssert(0);
	data.m_callback->DrawPolygon(indexCount, poly, edgeType);
	return t_ContinueSearh;
}

dFloat32 ndShapeHeightfield::RayHit(void* const context, const dFloat32* const polygon, dInt32 strideInBytes, const dInt32* const indexArray, dInt32 indexCount)
{
	ndBvhRay& me = *((ndBvhRay*)context);
	dVector normal(&polygon[indexArray[indexCount + 1] * (strideInBytes / sizeof(dFloat32))]);
	normal = normal & dVector::m_triplexMask;
	dFloat32 t = me.PolygonIntersect(normal, me.m_t, polygon, strideInBytes, indexArray, indexCount);
	if (t <= (me.m_t * dFloat32(1.0001f))) 
	{
		me.m_t = t;
		me.m_normal = normal;
		me.m_id = me.m_me->GetTagId(indexArray, indexCount);
	}
	return t;
}
*/

const dInt32* ndShapeHeightfield::GetIndexList() const
{
	return &m_cellIndices[(m_diagonalMode == m_normalDiagonals) ? 0 : 1][0];
}

void ndShapeHeightfield::DebugShape(const dMatrix& matrix, ndShapeDebugCallback& debugCallback) const
{
	dVector points[4];
	dVector triangle[3];

	ndShapeDebugCallback::ndEdgeType edgeType[4];
	memset(edgeType, ndShapeDebugCallback::m_shared, sizeof(edgeType));

	const dInt32* const indirectIndex = GetIndexList();
	const dInt32 i0 = indirectIndex[0];
	const dInt32 i1 = indirectIndex[1];
	const dInt32 i2 = indirectIndex[2];
	const dInt32 i3 = indirectIndex[3];

	dInt32 base = 0;
	for (dInt32 z = 0; z < m_height - 1; z++) 
	{
		const dVector p0 ((0 + 0) * m_horizontalScale_x, m_verticalScale * dFloat32(m_elevationMap[base + 0]),               (z + 0) * m_horizontalScale_z, dFloat32(0.0f));
		const dVector p1 ((0 + 0) * m_horizontalScale_x, m_verticalScale * dFloat32(m_elevationMap[base + 0 + m_width + 0]), (z + 1) * m_horizontalScale_z, dFloat32(0.0f));

		points[0 * 2 + 0] = matrix.TransformVector(p0);
		points[1 * 2 + 0] = matrix.TransformVector(p1);

		for (dInt32 x = 0; x < m_width - 1; x++) 
		{
			const dVector p2 ((x + 1) * m_horizontalScale_x, m_verticalScale * dFloat32(m_elevationMap[base + x + 1]),			(z + 0) * m_horizontalScale_z, dFloat32(0.0f));
			const dVector p3 ((x + 1) * m_horizontalScale_x, m_verticalScale * dFloat32(m_elevationMap[base + x + m_width + 1]), (z + 1) * m_horizontalScale_z, dFloat32(0.0f));

			points[0 * 2 + 1] = matrix.TransformVector(p2);
			points[1 * 2 + 1] = matrix.TransformVector(p3);

			triangle[0] = points[i1];
			triangle[1] = points[i0];
			triangle[2] = points[i2];
			debugCallback.DrawPolygon(3, triangle, edgeType);

			triangle[0] = points[i1];
			triangle[1] = points[i2];
			triangle[2] = points[i3];
			debugCallback.DrawPolygon(3, triangle, edgeType);

			points[0 * 2 + 0] = points[0 * 2 + 1];
			points[1 * 2 + 0] = points[1 * 2 + 1];
		}
		base += m_width;
	}
}

void ndShapeHeightfield::CalculateMinExtend2d(const dVector& p0, const dVector& p1, dVector& boxP0, dVector& boxP1) const
{
	dVector scale(m_horizontalScale_x, dFloat32(0.0f), m_horizontalScale_z, dFloat32(0.0f));
	dVector q0(p0.GetMin(p1) - m_padding);
	dVector q1(p0.GetMax(p1) + scale + m_padding);

	dVector invScale(m_horizontalScaleInv_x, dFloat32(0.0f), m_horizontalScaleInv_z, dFloat32(0.0f));
	boxP0 = (((q0 * invScale).Floor() * scale)         & m_yMask) - m_elevationPadding;
	boxP1 = (((q1 * invScale).Floor() * scale + scale) & m_yMask) + m_elevationPadding;
	dAssert(boxP0.m_w == dFloat32(0.0f));
	dAssert(boxP1.m_w == dFloat32(0.0f));

	dVector minBox(boxP0.Select(m_minBox, m_yMask));
	dVector maxBox(boxP1.Select(m_maxBox, m_yMask));

	boxP0 = boxP0.GetMax(minBox);
	boxP1 = boxP1.GetMin(maxBox);
}

dFloat32 ndShapeHeightfield::RayCastCell(const dFastRayTest& ray, dInt32 xIndex0, dInt32 zIndex0, dVector& normalOut, dFloat32 maxT) const
{
	dVector points[4];
	dInt32 triangle[3];

	// get the 3d point at the corner of the cell
	if ((xIndex0 < 0) || (zIndex0 < 0) || (xIndex0 >= (m_width - 1)) || (zIndex0 >= (m_height - 1))) 
	{
		return dFloat32(1.2f);
	}

	dAssert(maxT <= 1.0);

	dInt32 base = zIndex0 * m_width + xIndex0;

	//const dFloat32* const elevation = (dFloat32*)m_elevationMap;
	points[0 * 2 + 0] = dVector((xIndex0 + 0) * m_horizontalScale_x, m_verticalScale * dFloat32 (m_elevationMap[base + 0]),			  (zIndex0 + 0) * m_horizontalScale_z, dFloat32(0.0f));
	points[0 * 2 + 1] = dVector((xIndex0 + 1) * m_horizontalScale_x, m_verticalScale * dFloat32 (m_elevationMap[base + 1]),			  (zIndex0 + 0) * m_horizontalScale_z, dFloat32(0.0f));
	points[1 * 2 + 1] = dVector((xIndex0 + 1) * m_horizontalScale_x, m_verticalScale * dFloat32 (m_elevationMap[base + m_width + 1]), (zIndex0 + 1) * m_horizontalScale_z, dFloat32(0.0f));
	points[1 * 2 + 0] = dVector((xIndex0 + 0) * m_horizontalScale_x, m_verticalScale * dFloat32 (m_elevationMap[base + m_width + 0]), (zIndex0 + 1) * m_horizontalScale_z, dFloat32(0.0f));

	dFloat32 t = dFloat32(1.2f);
	//if (!m_diagonals[base]) 
	if (m_diagonalMode == m_normalDiagonals)
	{
		triangle[0] = 1;
		triangle[1] = 2;
		triangle[2] = 3;

		dVector e10(points[2] - points[1]);
		dVector e20(points[3] - points[1]);
		dVector normal(e10.CrossProduct(e20));
		normal = normal.Normalize();
		t = ray.PolygonIntersect(normal, maxT, &points[0].m_x, sizeof(dVector), triangle, 3);
		if (t < maxT) 
		{
			normalOut = normal;
			return t;
		}

		triangle[0] = 1;
		triangle[1] = 0;
		triangle[2] = 2;

		dVector e30(points[0] - points[1]);
		normal = e30.CrossProduct(e10);
		normal = normal.Normalize();
		t = ray.PolygonIntersect(normal, maxT, &points[0].m_x, sizeof(dVector), triangle, 3);
		if (t < maxT) 
		{
			normalOut = normal;
			return t;
		}
	}
	else 
	{
		triangle[0] = 0;
		triangle[1] = 2;
		triangle[2] = 3;

		dVector e10(points[2] - points[0]);
		dVector e20(points[3] - points[0]);
		dVector normal(e10.CrossProduct(e20));
		normal = normal.Normalize();
		t = ray.PolygonIntersect(normal, maxT, &points[0].m_x, sizeof(dVector), triangle, 3);
		if (t < maxT) 
		{
			normalOut = normal;
			return t;
		}

		triangle[0] = 0;
		triangle[1] = 3;
		triangle[2] = 1;

		dVector e30(points[1] - points[0]);
		normal = e20.CrossProduct(e30);
		normal = normal.Normalize();
		t = ray.PolygonIntersect(normal, maxT, &points[0].m_x, sizeof(dVector), triangle, 3);
		if (t < maxT) 
		{
			normalOut = normal;
			return t;
		}
	}
	return t;
}


//dFloat32 ndShapeHeightfield::RayCast(ndRayCastNotify& callback, const dVector& localP0, const dVector& localP1, dFloat32 maxT, const ndBody* const body, ndContactPoint& contactOut) const
dFloat32 ndShapeHeightfield::RayCast(ndRayCastNotify&, const dVector& localP0, const dVector& localP1, dFloat32 maxT, const ndBody* const, ndContactPoint& contactOut) const
{
	dVector boxP0;
	dVector boxP1;

	// calculate the ray bounding box
	CalculateMinExtend2d(localP0, localP1, boxP0, boxP1);

	dVector p0(localP0);
	dVector p1(localP1);
	
	// clip the line against the bounding box
	if (dRayBoxClip(p0, p1, boxP0, boxP1)) 
	{
		dVector dp(p1 - p0);
		dVector normalOut(dVector::m_zero);
	
		dFloat32 scale_x = m_horizontalScale_x;
		dFloat32 invScale_x = m_horizontalScaleInv_x;
		dFloat32 scale_z = m_horizontalScale_z;
		dFloat32 invScale_z = m_horizontalScaleInv_z;
		dInt32 ix0 = FastInt(p0.m_x * invScale_x);
		dInt32 iz0 = FastInt(p0.m_z * invScale_z);
	
		// implement a 3ddda line algorithm 
		dInt32 xInc;
		dFloat32 tx;
		dFloat32 stepX;
		if (dp.m_x > dFloat32(0.0f)) 
		{
			xInc = 1;
			dFloat32 val = dFloat32(1.0f) / dp.m_x;
			stepX = scale_x * val;
			tx = (scale_x * (ix0 + dFloat32(1.0f)) - p0.m_x) * val;
		}
		else if (dp.m_x < dFloat32(0.0f)) 
		{
			xInc = -1;
			dFloat32 val = -dFloat32(1.0f) / dp.m_x;
			stepX = scale_x * val;
			tx = -(scale_x * ix0 - p0.m_x) * val;
		}
		else 
		{
			xInc = 0;
			stepX = dFloat32(0.0f);
			tx = dFloat32(1.0e10f);
		}
	
		dInt32 zInc;
		dFloat32 tz;
		dFloat32 stepZ;
		if (dp.m_z > dFloat32(0.0f)) 
		{
			zInc = 1;
			dFloat32 val = dFloat32(1.0f) / dp.m_z;
			stepZ = scale_z * val;
			tz = (scale_z * (iz0 + dFloat32(1.0f)) - p0.m_z) * val;
		}
		else if (dp.m_z < dFloat32(0.0f)) 
		{
			zInc = -1;
			dFloat32 val = -dFloat32(1.0f) / dp.m_z;
			stepZ = scale_z * val;
			tz = -(scale_z * iz0 - p0.m_z) * val;
		}
		else 
		{
			zInc = 0;
			stepZ = dFloat32(0.0f);
			tz = dFloat32(1.0e10f);
		}
	
		dFloat32 txAcc = tx;
		dFloat32 tzAcc = tz;
		dInt32 xIndex0 = ix0;
		dInt32 zIndex0 = iz0;
		dFastRayTest ray(localP0, localP1);
	
		// for each cell touched by the line
		do 
		{
			dFloat32 t = RayCastCell(ray, xIndex0, zIndex0, normalOut, maxT);
			if (t < maxT) 
			{
				// bail out at the first intersection and copy the data into the descriptor
				dAssert(normalOut.m_w == dFloat32(0.0f));
				contactOut.m_normal = normalOut.Normalize();
				contactOut.m_shapeId0 = m_atributeMap[zIndex0 * m_width + xIndex0];
				contactOut.m_shapeId1 = m_atributeMap[zIndex0 * m_width + xIndex0];
	
				return t;
			}
	
			if (txAcc < tzAcc) 
			{
				xIndex0 += xInc;
				tx = txAcc;
				txAcc += stepX;
			}
			else 
			{
				zIndex0 += zInc;
				tz = tzAcc;
				tzAcc += stepZ;
			}
		} while ((tx <= dFloat32(1.0f)) || (tz <= dFloat32(1.0f)));
	}
	
	// if no cell was hit, return a large value
	return dFloat32(1.2f);
}

/*
dIntersectStatus ndShapeHeightfield::GetPolygon(void* const context, const dFloat32* const, dInt32, const dInt32* const indexArray, dInt32 indexCount, dFloat32 hitDistance)
{
	ndPolygonMeshDesc& data = (*(ndPolygonMeshDesc*)context);
	if (data.m_faceCount >= D_MAX_COLLIDING_FACES) 
	{
		dTrace(("buffer Over float, try using a lower resolution mesh for collision\n"));
		return t_StopSearh;
	}
	if ((data.m_globalIndexCount + indexCount * 2 + 3) >= D_MAX_COLLIDING_INDICES) 
	{
		dTrace(("buffer Over float, try using a lower resolution mesh for collision\n"));
		return t_StopSearh;
	}

	dInt32 count = indexCount * 2 + 3;

	data.m_faceIndexCount[data.m_faceCount] = indexCount;
	data.m_faceIndexStart[data.m_faceCount] = data.m_globalIndexCount;
	data.m_hitDistance[data.m_faceCount] = hitDistance;
	data.m_faceCount++;
	dInt32* const dst = &data.m_faceVertexIndex[data.m_globalIndexCount];

	//the docks say memcpy is an intrinsic function but as usual this is another Microsoft lied
	for (dInt32 i = 0; i < count; i++) 
	{
		dst[i] = indexArray[i];
	}

	data.m_globalIndexCount += count;

	return t_ContinueSearh;
}
*/

//void ndShapeHeightfield::GetCollidingFaces(ndPolygonMeshDesc* const data) const
void ndShapeHeightfield::GetCollidingFaces(ndPolygonMeshDesc* const) const
{
	dAssert(0);
}

