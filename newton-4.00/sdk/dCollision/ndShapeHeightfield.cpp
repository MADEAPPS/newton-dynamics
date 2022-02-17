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

#include "ndCoreStdafx.h"
#include "ndCollisionStdafx.h"
#include "ndContact.h"
#include "ndBodyKinematic.h"
#include "ndShapeInstance.h"
#include "ndShapeHeightfield.h"

D_CLASS_REFLECTION_IMPLEMENT_LOADER(ndShapeHeightfield)

ndVector ndShapeHeightfield::m_yMask(0xffffffff, 0, 0xffffffff, 0);
ndVector ndShapeHeightfield::m_padding(ndFloat32(0.25f), ndFloat32(0.25f), ndFloat32(0.25f), ndFloat32(0.0f));
ndVector ndShapeHeightfield::m_elevationPadding(ndFloat32(0.0f), ndFloat32(1.0e10f), ndFloat32(0.0f), ndFloat32(0.0f));

ndInt32 ndShapeHeightfield::m_cellIndices[][4] =
{
	{ 0, 1, 2, 3 },
	{ 1, 3, 0, 2 }
};

ndInt32 ndShapeHeightfield::m_horizontalEdgeMap[][7] =
{
	{ 1 * 9 + 0, 2 * 9 + 1, 1 * 9 + 4, 1 * 9 + 7, 2 * 9 + 7, 1 * 9 + 4, 2 * 9 + 4 },
	{ 0 * 9 + 1, 3 * 9 + 0, 0 * 9 + 4, 0 * 9 + 6, 3 * 9 + 6, 0 * 9 + 4, 3 * 9 + 4 }
};

ndInt32 ndShapeHeightfield::m_verticalEdgeMap[][7] =
{
	{ 1 * 9 + 1, 0 * 9 + 0, 1 * 9 + 4, 1 * 9 + 6, 0 * 9 + 6, 1 * 9 + 4, 0 * 9 + 4 },
	{ 1 * 9 + 0, 0 * 9 + 1, 1 * 9 + 4, 1 * 9 + 7, 0 * 9 + 7, 1 * 9 + 4, 0 * 9 + 4 }
};

ndShapeHeightfield::ndShapeHeightfield(
	ndInt32 width, ndInt32 height, ndGridConstruction constructionMode,
	ndFloat32 verticalScale, ndFloat32 horizontalScale_x, ndFloat32 horizontalScale_z)
	:ndShapeStaticMesh(m_heightField)
	,m_minBox(ndVector::m_zero)
	,m_maxBox(ndVector::m_zero)
	,m_atributeMap(width * height)
	,m_elevationMap(width * height)
	,m_verticalScale(verticalScale)
	,m_horizontalScale_x(horizontalScale_x)
	,m_horizontalScale_z(horizontalScale_z)
	,m_horizontalScaleInv_x(ndFloat32(1.0f) / horizontalScale_x)
	,m_horizontalScaleInv_z(ndFloat32(1.0f) / horizontalScale_z)
	,m_width(width)
	,m_height(height)
	,m_diagonalMode(constructionMode)
	,m_localData()
{
	dAssert(width >= 2);
	dAssert(height >= 2);
	m_atributeMap.SetCount(width * height);
	m_elevationMap.SetCount(width * height);

	memset(&m_atributeMap[0], 0, sizeof(ndInt8) * m_atributeMap.GetCount());
	memset(&m_elevationMap[0], 0, sizeof(ndInt16) * m_elevationMap.GetCount());

	CalculateLocalObb();
}

ndShapeHeightfield::ndShapeHeightfield(const ndLoadSaveBase::ndLoadDescriptor& desc)
	:ndShapeStaticMesh(m_heightField)
	,m_minBox(ndVector::m_zero)
	,m_maxBox(ndVector::m_zero)
	,m_atributeMap(0)
	,m_elevationMap(0)
	,m_verticalScale(ndFloat32 (0.0f))
	,m_horizontalScale_x(ndFloat32(0.0f))
	,m_horizontalScale_z(ndFloat32(0.0f))
	,m_horizontalScaleInv_x(ndFloat32(0.0f))
	,m_horizontalScaleInv_z(ndFloat32(0.0f))
	,m_width(0)
	,m_height(0)
	,m_diagonalMode(m_normalDiagonals)
	,m_localData()
{
	const nd::TiXmlNode* const xmlNode = desc.m_rootNode;
	const char* const assetName = xmlGetString(xmlNode, "assetName");

	m_minBox = xmlGetVector3(xmlNode, "minBox");
	m_maxBox = xmlGetVector3(xmlNode, "maxBox");
	m_verticalScale = xmlGetFloat(xmlNode, "verticalScale");
	m_horizontalScale_x = xmlGetFloat(xmlNode, "horizontalScale_x");
	m_horizontalScale_z = xmlGetFloat(xmlNode, "horizontalScale_z");
	m_width = xmlGetInt(xmlNode, "width");
	m_height = xmlGetInt(xmlNode, "height");
	m_diagonalMode = ndGridConstruction(xmlGetInt(xmlNode, "diagonalMode"));

	m_horizontalScaleInv_x = ndFloat32(1.0f) / m_horizontalScale_x;
	m_horizontalScaleInv_z = ndFloat32(1.0f) / m_horizontalScale_z;

	char filePathName[1024 * 2];
	sprintf(filePathName, "%s/%s", desc.m_assetPath, assetName);
	FILE* const file = fopen(filePathName, "rb");
	if (file)
	{
		ndInt64 readBytes;
		m_elevationMap.SetCount(m_width * m_height);
		m_atributeMap.SetCount(m_width * m_height);
		readBytes = fread(&m_elevationMap[0], sizeof(ndInt16), m_elevationMap.GetCount(), file);
		readBytes = fread(&m_atributeMap[0], sizeof(ndInt8), m_atributeMap.GetCount(), file);
		fclose(file);
	}
	CalculateLocalObb();
}

ndShapeHeightfield::~ndShapeHeightfield(void)
{
}

void ndShapeHeightfield::Save(const ndLoadSaveBase::ndSaveDescriptor& desc) const
{
	nd::TiXmlElement* const childNode = new nd::TiXmlElement(ClassName());
	desc.m_rootNode->LinkEndChild(childNode);
	childNode->SetAttribute("hashId", desc.m_nodeNodeHash);
	ndShapeStaticMesh::Save(ndLoadSaveBase::ndSaveDescriptor(desc, childNode));

	char fileName[1024];
	sprintf(fileName, "%s_%d.bin", desc.m_assetName, desc.m_assetIndex);
	xmlSaveParam(childNode, "assetName", "string", fileName);
	xmlSaveParam(childNode, "minBox", m_minBox);
	xmlSaveParam(childNode, "maxBox", m_maxBox);
	xmlSaveParam(childNode, "verticalScale", m_verticalScale);
	xmlSaveParam(childNode, "horizontalScale_x", m_horizontalScale_x);
	xmlSaveParam(childNode, "horizontalScale_z", m_horizontalScale_z);
	xmlSaveParam(childNode, "width", m_width);
	xmlSaveParam(childNode, "height", m_height);
	xmlSaveParam(childNode, "diagonalMode", ndInt32 (m_diagonalMode));

	char filePathName[1024 * 2];
	sprintf(filePathName, "%s/%s", desc.m_assetPath, fileName);
	desc.m_assetIndex++;

	FILE* const file = fopen(filePathName, "wb");
	if (file) 
	{
		fwrite(&m_elevationMap[0], sizeof(ndInt16), m_elevationMap.GetCount(), file);
		fwrite(&m_atributeMap[0], sizeof(ndInt8), m_atributeMap.GetCount(), file);
		fclose(file);
	}
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
	info.m_heightfield.m_elevation = (ndInt16*)&m_elevationMap[0];
	info.m_heightfield.m_atributes = (ndInt8*)&m_atributeMap[0];

	return info;
}

void ndShapeHeightfield::CalculateLocalObb()
{
	ndInt16 y0 = ndInt16(0x7fff);
	ndInt16 y1 = ndInt16 (-0x7fff);
	for (ndInt32 i = m_elevationMap.GetCount()-1; i >= 0; i--)
	{
		y0 = dMin(y0, m_elevationMap[i]);
		y1 = dMax(y1, m_elevationMap[i]);
	}

	m_minBox = ndVector(ndFloat32(0.0f), ndFloat32 (y0) * m_verticalScale, ndFloat32(0.0f), ndFloat32(0.0f));
	m_maxBox = ndVector(ndFloat32(m_width-1) * m_horizontalScale_x, ndFloat32(y1) * m_verticalScale, ndFloat32(m_height-1) * m_horizontalScale_z, ndFloat32(0.0f));

	m_boxSize = (m_maxBox - m_minBox) * ndVector::m_half;
	m_boxOrigin = (m_maxBox + m_minBox) * ndVector::m_half;
}

void ndShapeHeightfield::UpdateElevationMapAabb()
{
	CalculateLocalObb();
}

const ndInt32* ndShapeHeightfield::GetIndexList() const
{
	return &m_cellIndices[(m_diagonalMode == m_normalDiagonals) ? 0 : 1][0];
}

void ndShapeHeightfield::DebugShape(const ndMatrix& matrix, ndShapeDebugNotify& debugCallback) const
{
	ndVector points[4];
	ndVector triangle[3];

	ndShapeDebugNotify::ndEdgeType edgeType[4];
	memset(edgeType, ndShapeDebugNotify::m_shared, sizeof(edgeType));

	const ndInt32* const indirectIndex = GetIndexList();
	const ndInt32 i0 = indirectIndex[0];
	const ndInt32 i1 = indirectIndex[1];
	const ndInt32 i2 = indirectIndex[2];
	const ndInt32 i3 = indirectIndex[3];

	ndInt32 base = 0;
	for (ndInt32 z = 0; z < m_height - 1; z++) 
	{
		const ndVector p0 ((0 + 0) * m_horizontalScale_x, m_verticalScale * ndFloat32(m_elevationMap[base + 0]),               (z + 0) * m_horizontalScale_z, ndFloat32(0.0f));
		const ndVector p1 ((0 + 0) * m_horizontalScale_x, m_verticalScale * ndFloat32(m_elevationMap[base + 0 + m_width + 0]), (z + 1) * m_horizontalScale_z, ndFloat32(0.0f));

		points[0 * 2 + 0] = matrix.TransformVector(p0);
		points[1 * 2 + 0] = matrix.TransformVector(p1);

		for (ndInt32 x = 0; x < m_width - 1; x++) 
		{
			const ndVector p2 ((x + 1) * m_horizontalScale_x, m_verticalScale * ndFloat32(m_elevationMap[base + x + 1]),			(z + 0) * m_horizontalScale_z, ndFloat32(0.0f));
			const ndVector p3 ((x + 1) * m_horizontalScale_x, m_verticalScale * ndFloat32(m_elevationMap[base + x + m_width + 1]), (z + 1) * m_horizontalScale_z, ndFloat32(0.0f));

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

void ndShapeHeightfield::CalculateMinExtend2d(const ndVector& p0, const ndVector& p1, ndVector& boxP0, ndVector& boxP1) const
{
	const ndVector scale(m_horizontalScale_x, ndFloat32(0.0f), m_horizontalScale_z, ndFloat32(0.0f));
	const ndVector q0(p0.GetMin(p1) - m_padding);
	const ndVector q1(p0.GetMax(p1) + scale + m_padding);

	const ndVector invScale(m_horizontalScaleInv_x, ndFloat32(0.0f), m_horizontalScaleInv_z, ndFloat32(0.0f));
	boxP0 = (((q0 * invScale).Floor() * scale)         & m_yMask) - m_elevationPadding;
	boxP1 = (((q1 * invScale).Floor() * scale + scale) & m_yMask) + m_elevationPadding;
	dAssert(boxP0.m_w == ndFloat32(0.0f));
	dAssert(boxP1.m_w == ndFloat32(0.0f));

	const ndVector minBox(boxP0.Select(m_minBox, m_yMask));
	const ndVector maxBox(boxP1.Select(m_maxBox, m_yMask));

	boxP0 = boxP0.GetMax(minBox);
	boxP1 = boxP1.GetMin(maxBox);
}

void ndShapeHeightfield::CalculateMinExtend3d(const ndVector& p0, const ndVector& p1, ndVector& boxP0, ndVector& boxP1) const
{
	dAssert(p0.m_x <= p1.m_x);
	dAssert(p0.m_y <= p1.m_y);
	dAssert(p0.m_z <= p1.m_z);
	dAssert(p0.m_w == ndFloat32(0.0f));
	dAssert(p1.m_w == ndFloat32(0.0f));

	const ndVector scale(m_horizontalScale_x, ndFloat32(0.0f), m_horizontalScale_z, ndFloat32(0.0f));
	const ndVector q0(p0.GetMin(p1) - m_padding);
	const ndVector q1(p0.GetMax(p1) + scale + m_padding);
	const ndVector invScale(m_horizontalScaleInv_x, ndFloat32(0.0f), m_horizontalScaleInv_z, ndFloat32(0.0f));

	boxP0 = q0.Select((q0 * invScale).Floor() * scale, m_yMask);
	boxP1 = q1.Select((q1 * invScale).Floor() * scale + scale, m_yMask);
	
	boxP0 = boxP0.Select(boxP0.GetMax(m_minBox), m_yMask);
	boxP1 = boxP1.Select(boxP1.GetMax(m_minBox), m_yMask);

	boxP0 = boxP0.Select(boxP0.GetMin(m_maxBox), m_yMask);
	boxP1 = boxP1.Select(boxP1.GetMin(m_maxBox), m_yMask);

	dAssert(boxP0.m_x <= boxP1.m_x);
	dAssert(boxP0.m_z <= boxP1.m_z);
}

void ndShapeHeightfield::GetLocalAabb(const ndVector& q0, const ndVector& q1, ndVector& boxP0, ndVector& boxP1) const
{
	// the user data is the pointer to the collision geometry
	CalculateMinExtend3d(q0, q1, boxP0, boxP1);
	
	const ndVector p0(boxP0.Scale(m_horizontalScaleInv_x).GetInt());
	const ndVector p1(boxP1.Scale(m_horizontalScaleInv_x).GetInt());
	
	dAssert(p0.m_ix == FastInt(boxP0.m_x * m_horizontalScaleInv_x));
	dAssert(p0.m_iz == FastInt(boxP0.m_z * m_horizontalScaleInv_x));
	dAssert(p1.m_ix == FastInt(boxP1.m_x * m_horizontalScaleInv_x));
	dAssert(p1.m_iz == FastInt(boxP1.m_z * m_horizontalScaleInv_x));
	
	ndInt32 x0 = ndInt32(p0.m_ix);
	ndInt32 x1 = ndInt32(p1.m_ix);
	ndInt32 z0 = ndInt32(p0.m_iz);
	ndInt32 z1 = ndInt32(p1.m_iz);
	
	ndFloat32 minHeight = ndFloat32(1.0e10f);
	ndFloat32 maxHeight = ndFloat32(-1.0e10f);
	CalculateMinAndMaxElevation(x0, x1, z0, z1, minHeight, maxHeight);
	boxP0.m_y = minHeight;
	boxP1.m_y = maxHeight;
	dAssert(boxP0.m_x <= boxP1.m_x);
	dAssert(boxP0.m_y <= boxP1.m_y);
	dAssert(boxP0.m_z <= boxP1.m_z);
}

ndFloat32 ndShapeHeightfield::RayCastCell(const ndFastRay& ray, ndInt32 xIndex0, ndInt32 zIndex0, ndVector& normalOut, ndFloat32 maxT) const
{
	ndVector points[4];
	ndInt32 triangle[3];

	// get the 3d point at the corner of the cell
	if ((xIndex0 < 0) || (zIndex0 < 0) || (xIndex0 >= (m_width - 1)) || (zIndex0 >= (m_height - 1))) 
	{
		return ndFloat32(1.2f);
	}

	dAssert(maxT <= 1.0);

	ndInt32 base = zIndex0 * m_width + xIndex0;

	//const ndFloat32* const elevation = (ndFloat32*)m_elevationMap;
	points[0 * 2 + 0] = ndVector((xIndex0 + 0) * m_horizontalScale_x, m_verticalScale * ndFloat32 (m_elevationMap[base + 0]),			  (zIndex0 + 0) * m_horizontalScale_z, ndFloat32(0.0f));
	points[0 * 2 + 1] = ndVector((xIndex0 + 1) * m_horizontalScale_x, m_verticalScale * ndFloat32 (m_elevationMap[base + 1]),			  (zIndex0 + 0) * m_horizontalScale_z, ndFloat32(0.0f));
	points[1 * 2 + 1] = ndVector((xIndex0 + 1) * m_horizontalScale_x, m_verticalScale * ndFloat32 (m_elevationMap[base + m_width + 1]), (zIndex0 + 1) * m_horizontalScale_z, ndFloat32(0.0f));
	points[1 * 2 + 0] = ndVector((xIndex0 + 0) * m_horizontalScale_x, m_verticalScale * ndFloat32 (m_elevationMap[base + m_width + 0]), (zIndex0 + 1) * m_horizontalScale_z, ndFloat32(0.0f));

	ndFloat32 t = ndFloat32(1.2f);
	if (m_diagonalMode == m_normalDiagonals)
	{
		triangle[0] = 1;
		triangle[1] = 2;
		triangle[2] = 3;

		ndVector e10(points[2] - points[1]);
		ndVector e20(points[3] - points[1]);
		ndVector normal(e10.CrossProduct(e20));
		normal = normal.Normalize();
		t = ray.PolygonIntersect(normal, maxT, &points[0].m_x, sizeof(ndVector), triangle, 3);
		if (t < maxT) 
		{
			normalOut = normal;
			return t;
		}

		triangle[0] = 1;
		triangle[1] = 0;
		triangle[2] = 2;

		ndVector e30(points[0] - points[1]);
		normal = e30.CrossProduct(e10);
		normal = normal.Normalize();
		t = ray.PolygonIntersect(normal, maxT, &points[0].m_x, sizeof(ndVector), triangle, 3);
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

		ndVector e10(points[2] - points[0]);
		ndVector e20(points[3] - points[0]);
		ndVector normal(e10.CrossProduct(e20));
		normal = normal.Normalize();
		t = ray.PolygonIntersect(normal, maxT, &points[0].m_x, sizeof(ndVector), triangle, 3);
		if (t < maxT) 
		{
			normalOut = normal;
			return t;
		}

		triangle[0] = 0;
		triangle[1] = 3;
		triangle[2] = 1;

		ndVector e30(points[1] - points[0]);
		normal = e20.CrossProduct(e30);
		normal = normal.Normalize();
		t = ray.PolygonIntersect(normal, maxT, &points[0].m_x, sizeof(ndVector), triangle, 3);
		if (t < maxT) 
		{
			normalOut = normal;
			return t;
		}
	}
	return t;
}

ndFloat32 ndShapeHeightfield::RayCast(ndRayCastNotify&, const ndVector& localP0, const ndVector& localP1, ndFloat32 maxT, const ndBody* const, ndContactPoint& contactOut) const
{
	ndVector boxP0;
	ndVector boxP1;

	// calculate the ray bounding box
	CalculateMinExtend2d(localP0, localP1, boxP0, boxP1);

	ndVector p0(localP0);
	ndVector p1(localP1);
	
	// clip the line against the bounding box
	if (dRayBoxClip(p0, p1, boxP0, boxP1)) 
	{
		ndVector dp(p1 - p0);
		ndVector normalOut(ndVector::m_zero);
	
		ndFloat32 scale_x = m_horizontalScale_x;
		ndFloat32 invScale_x = m_horizontalScaleInv_x;
		ndFloat32 scale_z = m_horizontalScale_z;
		ndFloat32 invScale_z = m_horizontalScaleInv_z;
		ndInt32 ix0 = FastInt(p0.m_x * invScale_x);
		ndInt32 iz0 = FastInt(p0.m_z * invScale_z);
	
		// implement a 3ddda line algorithm 
		ndInt32 xInc;
		ndFloat32 tx;
		ndFloat32 stepX;
		if (dp.m_x > ndFloat32(0.0f)) 
		{
			xInc = 1;
			ndFloat32 val = ndFloat32(1.0f) / dp.m_x;
			stepX = scale_x * val;
			tx = (scale_x * (ix0 + ndFloat32(1.0f)) - p0.m_x) * val;
		}
		else if (dp.m_x < ndFloat32(0.0f)) 
		{
			xInc = -1;
			ndFloat32 val = -ndFloat32(1.0f) / dp.m_x;
			stepX = scale_x * val;
			tx = -(scale_x * ix0 - p0.m_x) * val;
		}
		else 
		{
			xInc = 0;
			stepX = ndFloat32(0.0f);
			tx = ndFloat32(1.0e10f);
		}
	
		ndInt32 zInc;
		ndFloat32 tz;
		ndFloat32 stepZ;
		if (dp.m_z > ndFloat32(0.0f)) 
		{
			zInc = 1;
			ndFloat32 val = ndFloat32(1.0f) / dp.m_z;
			stepZ = scale_z * val;
			tz = (scale_z * (iz0 + ndFloat32(1.0f)) - p0.m_z) * val;
		}
		else if (dp.m_z < ndFloat32(0.0f)) 
		{
			zInc = -1;
			ndFloat32 val = -ndFloat32(1.0f) / dp.m_z;
			stepZ = scale_z * val;
			tz = -(scale_z * iz0 - p0.m_z) * val;
		}
		else 
		{
			zInc = 0;
			stepZ = ndFloat32(0.0f);
			tz = ndFloat32(1.0e10f);
		}
	
		ndFloat32 txAcc = tx;
		ndFloat32 tzAcc = tz;
		ndInt32 xIndex0 = ix0;
		ndInt32 zIndex0 = iz0;
		ndFastRay ray(localP0, localP1);
	
		// for each cell touched by the line
		do 
		{
			ndFloat32 t = RayCastCell(ray, xIndex0, zIndex0, normalOut, maxT);
			if (t < maxT) 
			{
				// bail out at the first intersection and copy the data into the descriptor
				dAssert(normalOut.m_w == ndFloat32(0.0f));
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
		} while ((tx <= ndFloat32(1.0f)) || (tz <= ndFloat32(1.0f)));
	}
	
	// if no cell was hit, return a large value
	return ndFloat32(1.2f);
}

void ndShapeHeightfield::CalculateMinAndMaxElevation(ndInt32 x0, ndInt32 x1, ndInt32 z0, ndInt32 z1, ndFloat32& minHeight, ndFloat32& maxHeight) const
{
	ndInt16 minVal = 0x7fff;
	ndInt16 maxVal = -0x7fff;
	ndInt32 base = z0 * m_width;
	for (ndInt32 z = z0; z <= z1; z++) 
	{
		for (ndInt32 x = x0; x <= x1; x++) 
		{
			ndInt16 high = m_elevationMap[base + x];
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
	ndVector boxP0;
	ndVector boxP1;

	//dgWorld* const world = data->m_objBody->GetWorld();

	// the user data is the pointer to the collision geometry
	CalculateMinExtend3d(data->GetOrigin(), data->GetTarget(), boxP0, boxP1);
	boxP0 += data->m_boxDistanceTravelInMeshSpace & (data->m_boxDistanceTravelInMeshSpace < ndVector::m_zero);
	boxP1 += data->m_boxDistanceTravelInMeshSpace & (data->m_boxDistanceTravelInMeshSpace > ndVector::m_zero);

	boxP0 = boxP0.Select(boxP0.GetMax(m_minBox), m_yMask);
	boxP1 = boxP1.Select(boxP1.GetMin(m_maxBox), m_yMask);

	ndVector p0(boxP0.Scale(m_horizontalScaleInv_x).GetInt());
	ndVector p1(boxP1.Scale(m_horizontalScaleInv_x).GetInt());

	dAssert(p0.m_ix == FastInt(boxP0.m_x * m_horizontalScaleInv_x));
	dAssert(p0.m_iz == FastInt(boxP0.m_z * m_horizontalScaleInv_x));
	dAssert(p1.m_ix == FastInt(boxP1.m_x * m_horizontalScaleInv_x));
	dAssert(p1.m_iz == FastInt(boxP1.m_z * m_horizontalScaleInv_x));

	ndInt32 x0 = ndInt32(p0.m_ix);
	ndInt32 x1 = ndInt32(p1.m_ix);
	ndInt32 z0 = ndInt32(p0.m_iz);
	ndInt32 z1 = ndInt32(p1.m_iz);

	ndFloat32 minHeight = ndFloat32(1.0e10f);
	ndFloat32 maxHeight = ndFloat32(-1.0e10f);
	data->SetSeparatingDistance(ndFloat32(0.0f));
	CalculateMinAndMaxElevation(x0, x1, z0, z1, minHeight, maxHeight);

	if (!((maxHeight < boxP0.m_y) || (minHeight > boxP1.m_y))) 
	{
		ndThreadId threadId = ndGetThreadId();
		ndList<ndLocalThreadData>::ndNode* localDataNode = nullptr;
		for (ndList<ndLocalThreadData>::ndNode* node = m_localData.GetFirst(); node; node = node->GetNext())
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
		ndInt32 vertexCount = (z1 - z0 + 1) * (x1 - x0 + 1) + 2 * (z1 - z0) * (x1 - x0);
		ndArray<ndVector>& vertex = localDataNode->GetInfo().m_vertex;
		vertex.SetCount(vertexCount);
	
		ndInt32 vertexIndex = 0;
		ndInt32 base = z0 * m_width;
		for (ndInt32 z = z0; z <= z1; z++) 
		{
			ndFloat32 zVal = m_horizontalScale_z * z;
			for (ndInt32 x = x0; x <= x1; x++) 
			{
				vertex[vertexIndex] = ndVector(m_horizontalScale_x * x, m_verticalScale * ndFloat32(m_elevationMap[base + x]), zVal, ndFloat32(0.0f));
				vertexIndex++;
				dAssert(vertexIndex <= vertex.GetCount());
			}
			base += m_width;
		}

		ndInt32 normalBase = vertexIndex;
		vertexIndex = 0;
		ndInt32 index = 0;
		ndInt32 faceCount = 0;
		ndInt32 step = x1 - x0 + 1;
		ndInt32* const indices = data->m_globalFaceVertexIndex;
		ndInt32* const faceIndexCount = data->m_meshData.m_globalFaceIndexCount;
		ndInt32 faceSize = ndInt32(dMax(m_horizontalScale_x, m_horizontalScale_z) * ndFloat32(2.0f));

		const ndInt32* const indirectIndex = GetIndexList();
		for (ndInt32 z = z0; (z < z1) && (faceCount < D_MAX_COLLIDING_FACES); z++) 
		{
			ndInt32 zStep = z * m_width;
			for (ndInt32 x = x0; (x < x1) && (faceCount < D_MAX_COLLIDING_FACES); x++) 
			{
				ndInt32 vIndex[4];
				vIndex[0] = vertexIndex;
				vIndex[1] = vertexIndex + 1;
				vIndex[2] = vertexIndex + step;
				vIndex[3] = vertexIndex + step + 1;
	
				const ndInt32 i0 = vIndex[indirectIndex[0]];
				const ndInt32 i1 = vIndex[indirectIndex[1]];
				const ndInt32 i2 = vIndex[indirectIndex[2]];
				const ndInt32 i3 = vIndex[indirectIndex[3]];
	
				const ndVector e0(vertex[i0] - vertex[i1]);
				const ndVector e1(vertex[i2] - vertex[i1]);
				const ndVector e2(vertex[i3] - vertex[i1]);
				ndVector n0(e0.CrossProduct(e1));
				ndVector n1(e1.CrossProduct(e2));
				dAssert(n0.m_w == ndFloat32(0.0f));
				dAssert(n1.m_w == ndFloat32(0.0f));
	
				dAssert(n0.DotProduct(n0).GetScalar() > ndFloat32(0.0f));
				dAssert(n1.DotProduct(n1).GetScalar() > ndFloat32(0.0f));
	
				//normalBase 
				const ndInt32 normalIndex0 = normalBase;
				const ndInt32 normalIndex1 = normalBase + 1;
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
	
				ndVector dp(vertex[i3] - vertex[i1]);
				dAssert(dp.m_w == ndFloat32(0.0f));
				ndFloat32 dist(vertex[normalIndex0].DotProduct(dp).GetScalar());
				if (dist < -ndFloat32(1.0e-3f)) 
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
		ndInt32 stepBase = (x1 - x0) * (2 * 9);
		for (ndInt32 z = z0; z < z1; z++) 
		{
			//const ndInt32 diagBase = m_width * z;
			const ndInt32 triangleIndexBase = (z - z0) * stepBase;
			const ndInt32* const horizontalEdgeMap = &m_horizontalEdgeMap[m_diagonalMode == m_normalDiagonals ? 0 : 1][0];
			for (ndInt32 x = x0; x < (x1 - 1); x++) 
			{
				ndInt32 index1 = (x - x0) * (2 * 9) + triangleIndexBase;
				if (index1 < maxIndex) 
				{
					ndInt32* const triangles = &indices[index1];
					const ndInt32 i0 = triangles[horizontalEdgeMap[0]];
					const ndInt32 i1 = triangles[horizontalEdgeMap[1]];
					const ndInt32 i2 = triangles[horizontalEdgeMap[2]];
					
					const ndVector& origin = vertex[i0];
					const ndVector& testPoint = vertex[i1];
					const ndVector& normal = vertex[i2];
					dAssert(normal.m_w == ndFloat32(0.0f));
					ndFloat32 dist(normal.DotProduct(testPoint - origin).GetScalar());
					
					if (dist < -ndFloat32(1.0e-3f)) 
					{
						const ndInt32 i3 = horizontalEdgeMap[3];
						const ndInt32 i4 = horizontalEdgeMap[4];
						const ndInt32 i5 = horizontalEdgeMap[5];
						const ndInt32 i6 = horizontalEdgeMap[6];
						triangles[i3] = triangles[i6];
						triangles[i4] = triangles[i5];
					}
				}
			}
		}
	
		const ndInt32* const verticalEdgeMap = &m_verticalEdgeMap[m_diagonalMode == m_normalDiagonals ? 0 : 1][0];
		for (ndInt32 x = x0; x < x1; x++) 
		{
			const ndInt32 triangleIndexBase = (x - x0) * (2 * 9);
			for (ndInt32 z = z0; z < (z1 - 1); z++) 
			{
				ndInt32 index1 = (z - z0) * stepBase + triangleIndexBase;
				if (index1 < maxIndex) 
				{
					ndInt32* const triangles = &indices[index1];
					const ndInt32 i0 = triangles[verticalEdgeMap[0]];
					const ndInt32 i1 = triangles[verticalEdgeMap[1] + stepBase];
					const ndInt32 i2 = triangles[verticalEdgeMap[2]];
	
					const ndVector& origin = vertex[i0];
					const ndVector& testPoint = vertex[i1];
					const ndVector& normal = vertex[i2];
					dAssert(normal.m_w == ndFloat32(0.0f));
					ndFloat32 dist(normal.DotProduct(testPoint - origin).GetScalar());
	
					if (dist < -ndFloat32(1.0e-3f)) 
					{
						const ndInt32 i3 = verticalEdgeMap[3];
						const ndInt32 i4 = verticalEdgeMap[4] + stepBase;
						const ndInt32 i5 = verticalEdgeMap[5];
						const ndInt32 i6 = verticalEdgeMap[6] + stepBase;
						triangles[i3] = triangles[i6];
						triangles[i4] = triangles[i5];
					}
				}
			}
		}
	
		ndInt32 stride = sizeof(ndVector) / sizeof(ndFloat32);
		ndInt32 faceCount0 = 0;
		ndInt32 faceIndexCount0 = 0;
		ndInt32 faceIndexCount1 = 0;
	
		ndInt32* const address = data->m_meshData.m_globalFaceIndexStart;
		ndFloat32* const hitDistance = data->m_meshData.m_globalHitDistance;
	
		if (data->m_doContinueCollisionTest) 
		{
			ndFastRay ray(ndVector::m_zero, data->m_boxDistanceTravelInMeshSpace);
			for (ndInt32 i = 0; i < faceCount; i++) 
			{
				const ndInt32* const indexArray = &indices[faceIndexCount1];
				const ndVector& faceNormal = vertex[indexArray[4]];
				ndFloat32 dist = data->PolygonBoxRayDistance(faceNormal, 3, indexArray, stride, &vertex[0].m_x, ray);
				if (dist < ndFloat32(1.0f)) 
				{
					hitDistance[faceCount0] = dist;
					address[faceCount0] = faceIndexCount0;
					memcpy(&indices[faceIndexCount0], indexArray, 9 * sizeof(ndInt32));
					faceCount0++;
					faceIndexCount0 += 9;
				}
				faceIndexCount1 += 9;
			}
		}
		else 
		{
			for (ndInt32 i = 0; i < faceCount; i++) 
			{
				const ndInt32* const indexArray = &indices[faceIndexCount1];
				const ndVector& faceNormal = vertex[indexArray[4]];
				ndFloat32 dist = data->PolygonBoxDistance(faceNormal, 3, indexArray, stride, &vertex[0].m_x);
				if (dist > ndFloat32(0.0f)) 
				{
					hitDistance[faceCount0] = dist;
					address[faceCount0] = faceIndexCount0;
					memcpy(&indices[faceIndexCount0], indexArray, 9 * sizeof(ndInt32));
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
			data->m_vertexStrideInBytes = sizeof(ndVector);
		}
	}
}

