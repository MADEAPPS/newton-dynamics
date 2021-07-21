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

dInt32 ndShapeHeightfield::m_horizontalEdgeMap[][7] =
{
	{ 1 * 9 + 0, 2 * 9 + 1, 1 * 9 + 4, 1 * 9 + 7, 2 * 9 + 7, 1 * 9 + 4, 2 * 9 + 4 },
	{ 0 * 9 + 1, 3 * 9 + 0, 0 * 9 + 4, 0 * 9 + 6, 3 * 9 + 6, 0 * 9 + 4, 3 * 9 + 4 }
};

dInt32 ndShapeHeightfield::m_verticalEdgeMap[][7] =
{
	{ 1 * 9 + 1, 0 * 9 + 0, 1 * 9 + 4, 1 * 9 + 6, 0 * 9 + 6, 1 * 9 + 4, 0 * 9 + 4 },
	{ 1 * 9 + 0, 0 * 9 + 1, 1 * 9 + 4, 1 * 9 + 7, 0 * 9 + 7, 1 * 9 + 4, 0 * 9 + 4 }
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
	,m_localData()
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
	m_maxBox = dVector(dFloat32(m_width-1) * m_horizontalScale_x, dFloat32(y1) * m_verticalScale, dFloat32(m_height-1) * m_horizontalScale_z, dFloat32(0.0f));

	m_boxSize = (m_maxBox - m_minBox) * dVector::m_half;
	m_boxOrigin = (m_maxBox + m_minBox) * dVector::m_half;
}

void ndShapeHeightfield::UpdateElevationMapAabb()
{
	CalculateAABB();
}

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
	const dVector scale(m_horizontalScale_x, dFloat32(0.0f), m_horizontalScale_z, dFloat32(0.0f));
	const dVector q0(p0.GetMin(p1) - m_padding);
	const dVector q1(p0.GetMax(p1) + scale + m_padding);

	const dVector invScale(m_horizontalScaleInv_x, dFloat32(0.0f), m_horizontalScaleInv_z, dFloat32(0.0f));
	boxP0 = (((q0 * invScale).Floor() * scale)         & m_yMask) - m_elevationPadding;
	boxP1 = (((q1 * invScale).Floor() * scale + scale) & m_yMask) + m_elevationPadding;
	dAssert(boxP0.m_w == dFloat32(0.0f));
	dAssert(boxP1.m_w == dFloat32(0.0f));

	const dVector minBox(boxP0.Select(m_minBox, m_yMask));
	const dVector maxBox(boxP1.Select(m_maxBox, m_yMask));

	boxP0 = boxP0.GetMax(minBox);
	boxP1 = boxP1.GetMin(maxBox);
}

void ndShapeHeightfield::CalculateMinExtend3d(const dVector& p0, const dVector& p1, dVector& boxP0, dVector& boxP1) const
{
	dAssert(p0.m_x <= p1.m_x);
	dAssert(p0.m_y <= p1.m_y);
	dAssert(p0.m_z <= p1.m_z);
	dAssert(p0.m_w == dFloat32(0.0f));
	dAssert(p1.m_w == dFloat32(0.0f));

	const dVector scale(m_horizontalScale_x, dFloat32(0.0f), m_horizontalScale_z, dFloat32(0.0f));
	const dVector q0(p0.GetMin(p1) - m_padding);
	const dVector q1(p0.GetMax(p1) + scale + m_padding);
	const dVector invScale(m_horizontalScaleInv_x, dFloat32(0.0f), m_horizontalScaleInv_z, dFloat32(0.0f));

	boxP0 = q0.Select((q0 * invScale).Floor() * scale, m_yMask);
	boxP1 = q1.Select((q1 * invScale).Floor() * scale + scale, m_yMask);
	
	boxP0 = boxP0.Select(boxP0.GetMax(m_minBox), m_yMask);
	boxP1 = boxP1.Select(boxP1.GetMin(m_maxBox), m_yMask);
}

void ndShapeHeightfield::GetLocalAabb(const dVector& q0, const dVector& q1, dVector& boxP0, dVector& boxP1) const
{
	// the user data is the pointer to the collision geometry
	CalculateMinExtend3d(q0, q1, boxP0, boxP1);
	
	const dVector p0(boxP0.Scale(m_horizontalScaleInv_x).GetInt());
	const dVector p1(boxP1.Scale(m_horizontalScaleInv_x).GetInt());
	
	dAssert(p0.m_ix == FastInt(boxP0.m_x * m_horizontalScaleInv_x));
	dAssert(p0.m_iz == FastInt(boxP0.m_z * m_horizontalScaleInv_x));
	dAssert(p1.m_ix == FastInt(boxP1.m_x * m_horizontalScaleInv_x));
	dAssert(p1.m_iz == FastInt(boxP1.m_z * m_horizontalScaleInv_x));
	
	dInt32 x0 = dInt32(p0.m_ix);
	dInt32 x1 = dInt32(p1.m_ix);
	dInt32 z0 = dInt32(p0.m_iz);
	dInt32 z1 = dInt32(p1.m_iz);
	
	dFloat32 minHeight = dFloat32(1.0e10f);
	dFloat32 maxHeight = dFloat32(-1.0e10f);
	CalculateMinAndMaxElevation(x0, x1, z0, z1, minHeight, maxHeight);
	boxP0.m_y = minHeight;
	boxP1.m_y = maxHeight;
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

void ndShapeHeightfield::CalculateMinAndMaxElevation(dInt32 x0, dInt32 x1, dInt32 z0, dInt32 z1, dFloat32& minHeight, dFloat32& maxHeight) const
{
	dInt16 minVal = 0x7fff;
	dInt16 maxVal = -0x7fff;
	dInt32 base = z0 * m_width;
	for (dInt32 z = z0; z <= z1; z++) 
	{
		for (dInt32 x = x0; x <= x1; x++) 
		{
			dInt16 high = m_elevationMap[base + x];
			minVal = dMin(high, minVal);
			maxVal = dMax(high, maxVal);
		}
		base += m_width;
	}

	minHeight = minVal * m_verticalScale;
	maxHeight = maxVal * m_verticalScale;
}

void ndShapeHeightfield::GetCollidingFaces(ndPolygonMeshDesc* const data) const
{
	dVector boxP0;
	dVector boxP1;

	//dgWorld* const world = data->m_objBody->GetWorld();

	// the user data is the pointer to the collision geometry
	CalculateMinExtend3d(data->GetOrigin(), data->GetTarget(), boxP0, boxP1);
	boxP0 += data->m_boxDistanceTravelInMeshSpace & (data->m_boxDistanceTravelInMeshSpace < dVector::m_zero);
	boxP1 += data->m_boxDistanceTravelInMeshSpace & (data->m_boxDistanceTravelInMeshSpace > dVector::m_zero);

	boxP0 = boxP0.Select(boxP0.GetMax(m_minBox), m_yMask);
	boxP1 = boxP1.Select(boxP1.GetMin(m_maxBox), m_yMask);

	dVector p0(boxP0.Scale(m_horizontalScaleInv_x).GetInt());
	dVector p1(boxP1.Scale(m_horizontalScaleInv_x).GetInt());

	dAssert(p0.m_ix == FastInt(boxP0.m_x * m_horizontalScaleInv_x));
	dAssert(p0.m_iz == FastInt(boxP0.m_z * m_horizontalScaleInv_x));
	dAssert(p1.m_ix == FastInt(boxP1.m_x * m_horizontalScaleInv_x));
	dAssert(p1.m_iz == FastInt(boxP1.m_z * m_horizontalScaleInv_x));

	dInt32 x0 = dInt32(p0.m_ix);
	dInt32 x1 = dInt32(p1.m_ix);
	dInt32 z0 = dInt32(p0.m_iz);
	dInt32 z1 = dInt32(p1.m_iz);

	dFloat32 minHeight = dFloat32(1.0e10f);
	dFloat32 maxHeight = dFloat32(-1.0e10f);
	data->SetSeparatingDistance(dFloat32(0.0f));
	CalculateMinAndMaxElevation(x0, x1, z0, z1, minHeight, maxHeight);

	if (!((maxHeight < boxP0.m_y) || (minHeight > boxP1.m_y))) 
	{
		std::thread::id threadId = std::this_thread::get_id();
		dList<ndLocalThreadData>::dNode* localDataNode = nullptr;
		for (dList<ndLocalThreadData>::dNode* node = m_localData.GetFirst(); node; node = node->GetNext())
		{
			if (node->GetInfo().m_threadId == threadId)
			{
				localDataNode = node;
				break;
			}
		}

		if (!localDataNode)
		{
			localDataNode = m_localData.Append();
			localDataNode->GetInfo().m_threadId = threadId;
		}

		// scan the vertices's intersected by the box extend
		dInt32 vertexCount = (z1 - z0 + 1) * (x1 - x0 + 1) + 2 * (z1 - z0) * (x1 - x0);
		dArray<dVector>& vertex = localDataNode->GetInfo().m_vertex;
		vertex.SetCount(vertexCount);
	
		dInt32 vertexIndex = 0;
		dInt32 base = z0 * m_width;
		for (dInt32 z = z0; z <= z1; z++) 
		{
			dFloat32 zVal = m_horizontalScale_z * z;
			for (dInt32 x = x0; x <= x1; x++) 
			{
				vertex[vertexIndex] = dVector(m_horizontalScale_x * x, m_verticalScale * dFloat32(m_elevationMap[base + x]), zVal, dFloat32(0.0f));
				vertexIndex++;
				dAssert(vertexIndex <= vertex.GetCount());
			}
			base += m_width;
		}

		dInt32 normalBase = vertexIndex;
		vertexIndex = 0;
		dInt32 index = 0;
		dInt32 faceCount = 0;
		dInt32 step = x1 - x0 + 1;
		dInt32* const indices = data->m_globalFaceVertexIndex;
		dInt32* const faceIndexCount = data->m_meshData.m_globalFaceIndexCount;
		dInt32 faceSize = dInt32(dMax(m_horizontalScale_x, m_horizontalScale_z) * dFloat32(2.0f));

		const dInt32* const indirectIndex = GetIndexList();
		for (dInt32 z = z0; (z < z1) && (faceCount < D_MAX_COLLIDING_FACES); z++) 
		{
			dInt32 zStep = z * m_width;
			for (dInt32 x = x0; (x < x1) && (faceCount < D_MAX_COLLIDING_FACES); x++) 
			{
				dInt32 vIndex[4];
				vIndex[0] = vertexIndex;
				vIndex[1] = vertexIndex + 1;
				vIndex[2] = vertexIndex + step;
				vIndex[3] = vertexIndex + step + 1;
	
				const dInt32 i0 = vIndex[indirectIndex[0]];
				const dInt32 i1 = vIndex[indirectIndex[1]];
				const dInt32 i2 = vIndex[indirectIndex[2]];
				const dInt32 i3 = vIndex[indirectIndex[3]];
	
				const dVector e0(vertex[i0] - vertex[i1]);
				const dVector e1(vertex[i2] - vertex[i1]);
				const dVector e2(vertex[i3] - vertex[i1]);
				dVector n0(e0.CrossProduct(e1));
				dVector n1(e1.CrossProduct(e2));
				dAssert(n0.m_w == dFloat32(0.0f));
				dAssert(n1.m_w == dFloat32(0.0f));
	
				dAssert(n0.DotProduct(n0).GetScalar() > dFloat32(0.0f));
				dAssert(n1.DotProduct(n1).GetScalar() > dFloat32(0.0f));
	
				//normalBase 
				const dInt32 normalIndex0 = normalBase;
				const dInt32 normalIndex1 = normalBase + 1;
				vertex[normalIndex0] = n0.Normalize();
				vertex[normalIndex1] = n1.Normalize();
	
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
	
				dVector dp(vertex[i3] - vertex[i1]);
				dAssert(dp.m_w == dFloat32(0.0f));
				dFloat32 dist(vertex[normalIndex0].DotProduct(dp).GetScalar());
				if (dist < -dFloat32(1.0e-3f)) 
				{
					indices[index + 0 + 5] = normalIndex1;
					indices[index + 9 + 5] = normalIndex0;
				}
	
				index += 9 * 2;
				normalBase += 2;
				faceCount += 2;
				vertexIndex++;
			}
			vertexIndex++;
		}
		
		const int maxIndex = index;
		dInt32 stepBase = (x1 - x0) * (2 * 9);
		for (dInt32 z = z0; z < z1; z++) 
		{
			//const dInt32 diagBase = m_width * z;
			const dInt32 triangleIndexBase = (z - z0) * stepBase;
			const dInt32* const horizontalEdgeMap = &m_horizontalEdgeMap[m_diagonalMode == m_normalDiagonals ? 0 : 1][0];
			for (dInt32 x = x0; x < (x1 - 1); x++) 
			{
				dInt32 index1 = (x - x0) * (2 * 9) + triangleIndexBase;
				if (index1 < maxIndex) 
				{
					dInt32* const triangles = &indices[index1];
					const dInt32 i0 = triangles[horizontalEdgeMap[0]];
					const dInt32 i1 = triangles[horizontalEdgeMap[1]];
					const dInt32 i2 = triangles[horizontalEdgeMap[2]];
					
					const dVector& origin = vertex[i0];
					const dVector& testPoint = vertex[i1];
					const dVector& normal = vertex[i2];
					dAssert(normal.m_w == dFloat32(0.0f));
					dFloat32 dist(normal.DotProduct(testPoint - origin).GetScalar());
					
					if (dist < -dFloat32(1.0e-3f)) 
					{
						const dInt32 i3 = horizontalEdgeMap[3];
						const dInt32 i4 = horizontalEdgeMap[4];
						const dInt32 i5 = horizontalEdgeMap[5];
						const dInt32 i6 = horizontalEdgeMap[6];
						triangles[i3] = triangles[i6];
						triangles[i4] = triangles[i5];
					}
				}
			}
		}
	
		const dInt32* const verticalEdgeMap = &m_verticalEdgeMap[m_diagonalMode == m_normalDiagonals ? 0 : 1][0];
		for (dInt32 x = x0; x < x1; x++) 
		{
			const dInt32 triangleIndexBase = (x - x0) * (2 * 9);
			for (dInt32 z = z0; z < (z1 - 1); z++) 
			{
				dInt32 index1 = (z - z0) * stepBase + triangleIndexBase;
				if (index1 < maxIndex) 
				{
					//const dInt32 diagBase = m_width * z;
					//const dInt32 code = (m_diagonals[diagBase + x] << 1) + m_diagonals[diagBase + m_width + x];
					//const dInt32* const edgeMap = &m_verticalEdgeMap[code][0];
	
					dInt32* const triangles = &indices[index1];
					const dInt32 i0 = triangles[verticalEdgeMap[0]];
					const dInt32 i1 = triangles[verticalEdgeMap[1] + stepBase];
					const dInt32 i2 = triangles[verticalEdgeMap[2]];
	
					const dVector& origin = vertex[i0];
					const dVector& testPoint = vertex[i1];
					const dVector& normal = vertex[i2];
					dAssert(normal.m_w == dFloat32(0.0f));
					dFloat32 dist(normal.DotProduct(testPoint - origin).GetScalar());
	
					if (dist < -dFloat32(1.0e-3f)) 
					{
						const dInt32 i3 = verticalEdgeMap[3];
						const dInt32 i4 = verticalEdgeMap[4] + stepBase;
						const dInt32 i5 = verticalEdgeMap[5];
						const dInt32 i6 = verticalEdgeMap[6] + stepBase;
						triangles[i3] = triangles[i6];
						triangles[i4] = triangles[i5];
					}
				}
			}
		}
	
		dInt32 stride = sizeof(dVector) / sizeof(dFloat32);
		dInt32 faceCount0 = 0;
		dInt32 faceIndexCount0 = 0;
		dInt32 faceIndexCount1 = 0;
	
		dInt32* const address = data->m_meshData.m_globalFaceIndexStart;
		dFloat32* const hitDistance = data->m_meshData.m_globalHitDistance;
	
		if (data->m_doContinuesCollisionTest) 
		{
			dFastRayTest ray(dVector::m_zero, data->m_boxDistanceTravelInMeshSpace);
			for (dInt32 i = 0; i < faceCount; i++) 
			{
				const dInt32* const indexArray = &indices[faceIndexCount1];
				const dVector& faceNormal = vertex[indexArray[4]];
				dFloat32 dist = data->PolygonBoxRayDistance(faceNormal, 3, indexArray, stride, &vertex[0].m_x, ray);
				if (dist < dFloat32(1.0f)) 
				{
					hitDistance[faceCount0] = dist;
					address[faceCount0] = faceIndexCount0;
					memcpy(&indices[faceIndexCount0], indexArray, 9 * sizeof(dInt32));
					faceCount0++;
					faceIndexCount0 += 9;
				}
				faceIndexCount1 += 9;
			}
		}
		else 
		{
			for (dInt32 i = 0; i < faceCount; i++) 
			{
				const dInt32* const indexArray = &indices[faceIndexCount1];
				const dVector& faceNormal = vertex[indexArray[4]];
				dFloat32 dist = data->PolygonBoxDistance(faceNormal, 3, indexArray, stride, &vertex[0].m_x);
				if (dist > dFloat32(0.0f)) 
				{
					hitDistance[faceCount0] = dist;
					address[faceCount0] = faceIndexCount0;
					memcpy(&indices[faceIndexCount0], indexArray, 9 * sizeof(dInt32));
					faceCount0++;
					faceIndexCount0 += 9;
				}
				faceIndexCount1 += 9;
			}
		}
	
		if (faceCount0) 
		{
			// initialize the callback data structure
			data->m_faceCount = faceCount0;
			data->m_vertex = &vertex[0].m_x;
			data->m_faceVertexIndex = indices;
			data->m_faceIndexStart = address;
			data->m_hitDistance = hitDistance;
			data->m_faceIndexCount = faceIndexCount;
			data->m_vertexStrideInBytes = sizeof(dVector);
		}
	}
}

