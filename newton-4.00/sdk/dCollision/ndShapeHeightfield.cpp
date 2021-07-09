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

dInt32 ndShapeHeightfield::m_cellIndices[][4] =
{
	{ 0, 1, 2, 3 },
	{ 1, 3, 0, 2 }
};

ndShapeHeightfield::ndShapeHeightfield(
	dInt32 width, dInt32 height, ndGridConstruction constructionMode,
	dFloat32 verticalScale, dFloat32 horizontalScale_x, dFloat32 horizontalScale_z)
	:ndShapeStaticMesh(m_heightField)
	,m_atributeMap(width * height)
	,m_elevationMap(width * height)
	,m_verticalScale(verticalScale)
	,m_horizontalScale_x(horizontalScale_x)
	,m_horizontalScale_z(horizontalScale_z)
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

	//Create(builder);
	//CalculateAdjacendy();
	//
	//dVector p0;
	//dVector p1;
	//GetAABB(p0, p1);
	//m_boxSize = (p1 - p0) * dVector::m_half;
	//m_boxOrigin = (p1 + p0) * dVector::m_half;
	//
	//ndMeshVertexListIndexList data;
	//data.m_indexList = nullptr;
	//data.m_userDataList = nullptr;
	//data.m_maxIndexCount = 1000000000;
	//data.m_triangleCount = 0;
	//dVector zero(dVector::m_zero);
	//dFastAabbInfo box(dGetIdentityMatrix(), dVector(dFloat32(1.0e15f)));
	//ForAllSectors(box, zero, dFloat32(1.0f), GetTriangleCount, &data);
	//m_trianglesCount = data.m_triangleCount;



	//dInt32 attibutePaddedMapSize = (m_width * m_height + 4) & -4;
	//m_diagonals = (dInt8 *)dgMallocStack(attibutePaddedMapSize * sizeof(dInt8));
	//switch (m_diagonalMode)
	//{
	//	case m_normalDiagonals:
	//	{
	//		memset(m_diagonals, 0, m_width * m_height * sizeof(dInt8));
	//		break;
	//	}
	//	case m_invertedDiagonals:
	//	{
	//		memset(m_diagonals, 1, m_width * m_height * sizeof(dInt8));
	//		break;
	//	}
	//
	//	case m_alternateOddRowsDiagonals:
	//	{
	//		for (dInt32 j = 0; j < m_height; j += 2) {
	//			dInt32 index = j * m_width;
	//			for (dInt32 i = 0; i < m_width; i++) {
	//				m_diagonals[index + i] = 0;
	//			}
	//		}
	//
	//		for (dInt32 j = 1; j < m_height; j += 2) {
	//			dInt32 index = j * m_width;
	//			for (dInt32 i = 0; i < m_width; i++) {
	//				m_diagonals[index + i] = 1;
	//			}
	//		}
	//		break;
	//	}
	//
	//	case m_alternateEvenRowsDiagonals:
	//	{
	//		for (dInt32 j = 0; j < m_height; j += 2) {
	//			dInt32 index = j * m_width;
	//			for (dInt32 i = 0; i < m_width; i++) {
	//				m_diagonals[index + i] = 1;
	//			}
	//		}
	//
	//		for (dInt32 j = 1; j < m_height; j += 2) {
	//			dInt32 index = j * m_width;
	//			for (dInt32 i = 0; i < m_width; i++) {
	//				m_diagonals[index + i] = 0;
	//			}
	//		}
	//		break;
	//	}
	//
	//
	//	case m_alternateOddColumsDiagonals:
	//	{
	//		for (dInt32 j = 0; j < m_height; j++) {
	//			dInt32 index = j * m_width;
	//			for (dInt32 i = 0; i < m_width; i += 2) {
	//				m_diagonals[index + i] = 0;
	//			}
	//
	//			for (dInt32 i = 1; i < m_width; i += 2) {
	//				m_diagonals[index + i] = 1;
	//			}
	//		}
	//		break;
	//	}
	//
	//	case m_alternateEvenColumsDiagonals:
	//	{
	//		for (dInt32 j = 0; j < m_height; j++) {
	//			dInt32 index = j * m_width;
	//			for (dInt32 i = 0; i < m_width; i += 2) {
	//				m_diagonals[index + i] = 1;
	//			}
	//
	//			for (dInt32 i = 1; i < m_width; i += 2) {
	//				m_diagonals[index + i] = 0;
	//			}
	//		}
	//		break;
	//	}
	//
	//	case m_starDiagonals:
	//	{
	//		for (dInt32 j = 0; j < m_height; j += 2) {
	//			dInt32 index = j * m_width;
	//			for (dInt32 i = 0; i < m_width; i += 2) {
	//				m_diagonals[index + i] = 0;
	//			}
	//			for (dInt32 i = 1; i < m_width; i += 2) {
	//				m_diagonals[index + i] = 1;
	//			}
	//		}
	//
	//		for (dInt32 j = 1; j < m_height; j += 2) {
	//			dInt32 index = j * m_width;
	//			for (dInt32 i = 0; i < m_width; i += 2) {
	//				m_diagonals[index + i] = 1;
	//			}
	//			for (dInt32 i = 1; i < m_width; i += 2) {
	//				m_diagonals[index + i] = 0;
	//			}
	//		}
	//		break;
	//	}
	//
	//	case m_starInvertexDiagonals:
	//	{
	//		for (dInt32 j = 0; j < m_height; j += 2) {
	//			dInt32 index = j * m_width;
	//			for (dInt32 i = 0; i < m_width; i += 2) {
	//				m_diagonals[index + i] = 1;
	//			}
	//			for (dInt32 i = 1; i < m_width; i += 2) {
	//				m_diagonals[index + i] = 0;
	//			}
	//		}
	//
	//		for (dInt32 j = 1; j < m_height; j += 2) {
	//			dInt32 index = j * m_width;
	//			for (dInt32 i = 0; i < m_width; i += 2) {
	//				m_diagonals[index + i] = 0;
	//			}
	//			for (dInt32 i = 1; i < m_width; i += 2) {
	//				m_diagonals[index + i] = 1;
	//			}
	//		}
	//
	//		break;
	//	}
	//
	//	default:
	//		dAssert(0);
	//
	//}

	//dgTree<void*, unsigned>::dgTreeNode* nodeData = world->m_perInstanceData.Find(DG_HIGHTFIELD_DATA_ID);
	//if (!nodeData) {
	//	m_instanceData = (dgPerIntanceData*) new dgPerIntanceData();
	//	m_instanceData->m_refCount = 0;
	//	m_instanceData->m_world = world;
	//	for (dInt32 i = 0; i < DG_MAX_THREADS_HIVE_COUNT; i++) {
	//		m_instanceData->m_vertex[i] = NULL;
	//		m_instanceData->m_vertexCount[i] = 0;
	//		m_instanceData->m_vertex[i].SetAllocator(world->GetAllocator());
	//		AllocateVertex(world, i);
	//	}
	//	nodeData = world->m_perInstanceData.Insert(m_instanceData, DG_HIGHTFIELD_DATA_ID);
	//}
	//m_instanceData = (dgPerIntanceData*)nodeData->GetInfo();
	//m_instanceData->m_refCount++;
	CalculateAABB();
}

//ndShapeHeightfield::ndShapeHeightfield(const nd::TiXmlNode* const xmlNode, const char* const assetPath)
ndShapeHeightfield::ndShapeHeightfield(const nd::TiXmlNode* const, const char* const)
	:ndShapeStaticMesh(m_heightField)
{
	dAssert(0);
	//D_CORE_API const char* xmlGetString(const nd::TiXmlNode* const rootNode, const char* const name);
	//const char* const assetName = xmlGetString(xmlNode, "assetName");
	//char pathCopy[1024];
	//sprintf(pathCopy, "%s/%s", assetPath, assetName);
	//Deserialize(pathCopy);
	//
	//dVector p0;
	//dVector p1;
	//GetAABB(p0, p1);
	//m_boxSize = (p1 - p0) * dVector::m_half;
	//m_boxOrigin = (p1 + p0) * dVector::m_half;
	//
	//ndMeshVertexListIndexList data;
	//data.m_indexList = nullptr;
	//data.m_userDataList = nullptr;
	//data.m_maxIndexCount = 1000000000;
	//data.m_triangleCount = 0;
	//dVector zero(dVector::m_zero);
	//dFastAabbInfo box(dGetIdentityMatrix(), dVector(dFloat32(1.0e15f)));
	//ForAllSectors(box, zero, dFloat32(1.0f), GetTriangleCount, &data);
	//m_trianglesCount = data.m_triangleCount;
}

ndShapeHeightfield::~ndShapeHeightfield(void)
{
}

//void ndShapeHeightfield::Save(nd::TiXmlElement* const xmlNode, const char* const assetPath, dInt32 nodeid) const
void ndShapeHeightfield::Save(nd::TiXmlElement* const, const char* const, dInt32) const
{
	dAssert(0);
	//nd::TiXmlElement* const paramNode = new nd::TiXmlElement("ndShapeHeightfield");
	//xmlNode->LinkEndChild(paramNode);
	//
	//paramNode->SetAttribute("nodeId", nodeid);
	//
	//char pathCopy[1024];
	//sprintf(pathCopy, "%s/asset%d.bin", assetPath, nodeid);
	//Serialize(pathCopy);
	//
	//sprintf(pathCopy, "asset%d.bin", nodeid);
	//xmlSaveParam(paramNode, "assetName", "string", pathCopy);
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

	dVector p0 (dFloat32(dFloat32(0.0f)), dFloat32 (y0) * m_verticalScale, dFloat32(0.0f), dFloat32(0.0f));
	dVector p1 (dFloat32(m_width) * m_horizontalScale_x, dFloat32(y1) * m_verticalScale, dFloat32(m_height) * m_horizontalScale_z, dFloat32(0.0f));

	m_boxSize = (p1 - p0) * dVector::m_half;
	m_boxOrigin = (p1 + p0) * dVector::m_half;
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

void ndShapeHeightfield::DebugShape(const dMatrix& matrix, ndShapeDebugCallback& debugCallback) const
{
	dVector points[4];
	dVector triangle[3];

	ndShapeDebugCallback::ndEdgeType edgeType[4];
	memset(edgeType, ndShapeDebugCallback::m_shared, sizeof(edgeType));

	//const dInt32* const indirectIndex = &m_cellIndices[dInt32(m_diagonals[z * m_width + x])][0];
	const dInt32* const indirectIndex = &m_cellIndices[(m_diagonalMode == m_normalDiagonals) ? 0 : 1][0];
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

//dFloat32 ndShapeHeightfield::RayCast(ndRayCastNotify& callback, const dVector& localP0, const dVector& localP1, dFloat32 maxT, const ndBody* const body, ndContactPoint& contactOut) const
dFloat32 ndShapeHeightfield::RayCast(ndRayCastNotify&, const dVector&, const dVector&, dFloat32, const ndBody* const, ndContactPoint&) const
{
	dAssert(0);
	return 0;
	//ndBvhRay ray(localP0, localP1);
	//ray.m_t = dFloat32(1.0f);
	//ray.m_me = this;
	//ray.m_myBody = ((ndBody*)body)->GetAsBodyKinematic();
	//ray.m_callback = &callback;
	//dFloat32 t = dFloat32 (1.2f);
	//ForAllSectorsRayHit(ray, maxT, RayHit, &ray);
	//if (ray.m_t < maxT)
	//{
	//	t = ray.m_t;
	//	dAssert(ray.m_normal.m_w == dFloat32(0.0f));
	//	dAssert(ray.m_normal.DotProduct(ray.m_normal).GetScalar() > dFloat32(0.0f));
	//	contactOut.m_normal = ray.m_normal.Normalize();
	//	contactOut.m_shapeId0 = ray.m_id;
	//	contactOut.m_shapeId1 = ray.m_id;
	//}
	//return t;
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
	//data->m_me = this;
	//data->m_vertex = GetLocalVertexPool();
	//data->m_vertexStrideInBytes = GetStrideInBytes();
	//
	//data->m_faceCount = 0;
	//data->m_globalIndexCount = 0;
	//data->m_faceIndexCount = data->m_meshData.m_globalFaceIndexCount;
	//data->m_faceIndexStart = data->m_meshData.m_globalFaceIndexStart;
	//data->m_faceVertexIndex = data->m_globalFaceVertexIndex;
	//data->m_hitDistance = data->m_meshData.m_globalHitDistance;
	//ForAllSectors(*data, data->m_boxDistanceTravelInMeshSpace, data->m_maxT, GetPolygon, data);
}

