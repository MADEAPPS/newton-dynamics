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
#include "ndShapeStatic_bvh.h"

D_CLASS_REFLECTION_IMPLEMENT_LOADER(ndShapeStatic_bvh)

class ndCollisionBvhShowPolyContext
{
	public:
	ndMatrix m_matrix;
	void* m_userData;
	ndShapeDebugNotify* m_callback;
};

D_MSV_NEWTON_ALIGN_32 
class ndBvhRay: public ndFastRay 
{
	public:
	ndBvhRay(const ndVector& l0, const ndVector& l1)
		:ndFastRay (l0, l1)
	{
	}

	ndMatrix m_matrix;
	ndVector m_normal;
	ndUnsigned32 m_id;
	ndFloat32 m_t;
	ndRayCastNotify* m_callback;
	const ndBodyKinematic* m_myBody;
	const ndShapeStatic_bvh* m_me;
} D_GCC_NEWTON_ALIGN_32;

ndShapeStatic_bvh::ndShapeStatic_bvh(const ndPolygonSoupBuilder& builder)
	:ndShapeStaticMesh(m_boundingBoxHierachy)
	,ndAabbPolygonSoup()
	,m_trianglesCount(0)
{
	Create(builder);
	CalculateAdjacendy();

	ndVector p0;
	ndVector p1;
	GetAABB(p0, p1);
	m_boxSize = (p1 - p0) * ndVector::m_half;
	m_boxOrigin = (p1 + p0) * ndVector::m_half;

	ndMeshVertexListIndexList data;
	data.m_indexList = nullptr;
	data.m_userDataList = nullptr;
	data.m_maxIndexCount = 1000000000;
	data.m_triangleCount = 0;
	ndVector zero(ndVector::m_zero);
	ndFastAabb box(dGetIdentityMatrix(), ndVector(ndFloat32(1.0e15f)));
	ForAllSectors(box, zero, ndFloat32(1.0f), GetTriangleCount, &data);
	m_trianglesCount = data.m_triangleCount;
}

ndShapeStatic_bvh::ndShapeStatic_bvh(const ndLoadSaveBase::dLoadDescriptor& desc)
	:ndShapeStaticMesh(m_boundingBoxHierachy)
	,ndAabbPolygonSoup()
	,m_trianglesCount(0)
{
	const nd::TiXmlNode* const xmlNode = desc.m_rootNode;
	const char* const assetName = xmlGetString(xmlNode, "assetName");
	char pathCopy[1024];
	sprintf(pathCopy, "%s/%s", desc.m_assetPath, assetName);
	Deserialize(pathCopy);

	ndVector p0;
	ndVector p1;
	GetAABB(p0, p1);
	m_boxSize = (p1 - p0) * ndVector::m_half;
	m_boxOrigin = (p1 + p0) * ndVector::m_half;

	ndMeshVertexListIndexList data;
	data.m_indexList = nullptr;
	data.m_userDataList = nullptr;
	data.m_maxIndexCount = 1000000000;
	data.m_triangleCount = 0;
	ndVector zero(ndVector::m_zero);
	ndFastAabb box(dGetIdentityMatrix(), ndVector(ndFloat32(1.0e15f)));
	ForAllSectors(box, zero, ndFloat32(1.0f), GetTriangleCount, &data);
	m_trianglesCount = data.m_triangleCount;
}

ndShapeStatic_bvh::~ndShapeStatic_bvh(void)
{
}

dIntersectStatus ndShapeStatic_bvh::GetTriangleCount(void* const context, const ndFloat32* const, ndInt32, const ndInt32* const, ndInt32 indexCount, ndFloat32)
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

ndShapeInfo ndShapeStatic_bvh::GetShapeInfo() const
{
	ndShapeInfo info(ndShapeStaticMesh::GetShapeInfo());

	info.m_bvh.m_vertexCount = GetVertexCount();
	info.m_bvh.m_indexCount = m_trianglesCount * 3;
	return info;
}

dIntersectStatus ndShapeStatic_bvh::ShowDebugPolygon(void* const context, const ndFloat32* const polygon, ndInt32 strideInBytes, const ndInt32* const indexArray, ndInt32 indexCount, ndFloat32)
{
	ndVector poly[128];
	ndShapeDebugNotify::ndEdgeType edgeType[128];

	ndInt32 stride = ndInt32(strideInBytes / sizeof(ndFloat32));

	ndCollisionBvhShowPolyContext& data = *(ndCollisionBvhShowPolyContext*)context;
	for (ndInt32 i = 0; i < indexCount; i++) 
	{
		ndVector p(&polygon[indexArray[i] * stride]);
		poly[i] = data.m_matrix.TransformVector(p & ndVector::m_triplexMask);
		ndInt32 edgeIndexType = (indexArray[i + indexCount + 2]) & D_CONCAVE_EDGE_MASK;
		edgeType[i] = edgeIndexType ? ndShapeDebugNotify::m_open : ndShapeDebugNotify::m_shared;
	}
	//dAssert(0);
	data.m_callback->DrawPolygon(indexCount, poly, edgeType);
	return t_ContinueSearh;
}

void ndShapeStatic_bvh::DebugShape(const ndMatrix& matrix, ndShapeDebugNotify& debugCallback) const
{
	ndCollisionBvhShowPolyContext context;

	context.m_matrix = matrix;
	context.m_userData = (void*)this;
	context.m_callback = &debugCallback;

	ndFastAabb box(dGetIdentityMatrix(), ndVector(1.0e15f));
	ForAllSectors(box, ndVector::m_zero, ndFloat32(1.0f), ShowDebugPolygon, &context);
}

ndFloat32 ndShapeStatic_bvh::RayHit(void* const context, const ndFloat32* const polygon, ndInt32 strideInBytes, const ndInt32* const indexArray, ndInt32 indexCount)
{
	ndBvhRay& me = *((ndBvhRay*)context);
	ndVector normal(&polygon[indexArray[indexCount + 1] * (strideInBytes / sizeof(ndFloat32))]);
	normal = normal & ndVector::m_triplexMask;
	ndFloat32 t = me.PolygonIntersect(normal, me.m_t, polygon, strideInBytes, indexArray, indexCount);
	if (t <= (me.m_t * ndFloat32(1.0001f))) 
	{
		me.m_t = t;
		me.m_normal = normal;
		me.m_id = me.m_me->GetTagId(indexArray, indexCount);
	}
	return t;
}

ndFloat32 ndShapeStatic_bvh::RayCast(ndRayCastNotify& callback, const ndVector& localP0, const ndVector& localP1, ndFloat32 maxT, const ndBody* const body, ndContactPoint& contactOut) const
{
	ndBvhRay ray(localP0, localP1);
	ray.m_t = ndFloat32(1.0f);
	ray.m_me = this;
	ray.m_myBody = ((ndBody*)body)->GetAsBodyKinematic();
	ray.m_callback = &callback;
	ndFloat32 t = ndFloat32 (1.2f);
	ForAllSectorsRayHit(ray, maxT, RayHit, &ray);
	if (ray.m_t < maxT)
	{
		t = ray.m_t;
		dAssert(ray.m_normal.m_w == ndFloat32(0.0f));
		dAssert(ray.m_normal.DotProduct(ray.m_normal).GetScalar() > ndFloat32(0.0f));
		contactOut.m_normal = ray.m_normal.Normalize();
		contactOut.m_shapeId0 = ray.m_id;
		contactOut.m_shapeId1 = ray.m_id;
	}
	return t;
}

dIntersectStatus ndShapeStatic_bvh::GetPolygon(void* const context, const ndFloat32* const, ndInt32, const ndInt32* const indexArray, ndInt32 indexCount, ndFloat32 hitDistance)
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

	ndInt32 count = indexCount * 2 + 3;

	data.m_faceIndexCount[data.m_faceCount] = indexCount;
	data.m_faceIndexStart[data.m_faceCount] = data.m_globalIndexCount;
	data.m_hitDistance[data.m_faceCount] = hitDistance;
	data.m_faceCount++;
	ndInt32* const dst = &data.m_faceVertexIndex[data.m_globalIndexCount];

	//the docks say memcpy is an intrinsic function but as usual this is another Microsoft lied
	for (ndInt32 i = 0; i < count; i++) 
	{
		dst[i] = indexArray[i];
	}

	data.m_globalIndexCount += count;

	return t_ContinueSearh;
}

void ndShapeStatic_bvh::GetCollidingFaces(ndPolygonMeshDesc* const data) const
{
	data->m_me = this;
	data->m_vertex = GetLocalVertexPool();
	data->m_vertexStrideInBytes = GetStrideInBytes();

	data->m_faceCount = 0;
	data->m_globalIndexCount = 0;
	data->m_faceIndexCount = data->m_meshData.m_globalFaceIndexCount;
	data->m_faceIndexStart = data->m_meshData.m_globalFaceIndexStart;
	data->m_faceVertexIndex = data->m_globalFaceVertexIndex;
	data->m_hitDistance = data->m_meshData.m_globalHitDistance;
	ForAllSectors(*data, data->m_boxDistanceTravelInMeshSpace, data->m_maxT, GetPolygon, data);
}

void ndShapeStatic_bvh::Save(const ndLoadSaveBase::ndSaveDescriptor& desc) const
{
	nd::TiXmlElement* const childNode = new nd::TiXmlElement(ClassName());
	desc.m_rootNode->LinkEndChild(childNode);
	childNode->SetAttribute("hashId", desc.m_nodeNodeHash);
	ndShapeStaticMesh::Save(ndLoadSaveBase::ndSaveDescriptor(desc, childNode));

	char fileName[1024];
	sprintf(fileName, "%s_%d.bin", desc.m_assetName, desc.m_assetIndex);
	xmlSaveParam(childNode, "assetName", "string", fileName);

	char filePathName[2 * 1024];
	sprintf(filePathName, "%s/%s", desc.m_assetPath, fileName);
	desc.m_assetIndex++;
	Serialize(filePathName);
}
