/* Copyright (c) <2003-2019> <Julio Jerez, Newton Game Dynamics>
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
#include "ndShapeStaticBVH.h"

#if 0
void ndShapeStaticBVH::ForEachFace (dAaabbIntersectCallback callback, void* const context) const
{
	dVector p0 (-1.0e10f, -1.0e10f, -1.0e10f, 1.0f);
	dVector p1 ( 1.0e10f,  1.0e10f,  1.0e10f, 1.0f);
	dVector zero (dFloat32 (0.0f));
	dFastAabbInfo box (dGetIdentityMatrix(), dVector (dFloat32 (1.0e15f)));
	//ForAllSectors (p0, p1, zero, dFloat32 (1.0f), callback, context);
	ForAllSectors (box, zero, dFloat32 (1.0f), callback, context);
}

dIntersectStatus ndShapeStaticBVH::CollectVertexListIndexList (void* const context, const dFloat32* const polygon, dInt32 strideInBytes, const dInt32* const indexArray, dInt32 indexCount, dFloat32 hitDistance)
{
	ndMeshVertexListIndexList& data = (*(ndMeshVertexListIndexList*) context);

	if ((data.m_triangleCount + indexCount - 2) * 3 > data.m_maxIndexCount) {
		return t_StopSearh;
	}

	dInt32 k = data.m_triangleCount;
	dInt32 j = data.m_triangleCount * 3;
	dInt32 index0 = indexArray[0];
	dInt32 index1 = indexArray[1];
	dInt32 attribute = indexArray[indexCount];
	for (dInt32 i = 2; i < indexCount; i ++) {
		dInt32 index2 = indexArray[i];
		data.m_indexList[j + 0] = index0;
		data.m_indexList[j + 1] = index1;
		data.m_indexList[j + 2] = index2;
		data.m_userDataList[k] = attribute;
		index1 = index2; 
		k ++;
		j += 3;
	}

	dAssert (j <= data.m_maxIndexCount);
	data.m_triangleCount = k;
	dAssert ((data.m_triangleCount * 3) <= data.m_maxIndexCount);

	return t_ContinueSearh;
}

void ndShapeStaticBVH::GetVertexListIndexList (const dVector& p0, const dVector& p1, ndMeshVertexListIndexList &data) const
{
	dFastAabbInfo box (p0, p1);
	ForAllSectors (box, dVector (dFloat32 (0.0f)), dFloat32 (1.0f), CollectVertexListIndexList, &data);

	data.m_veterxArray = GetLocalVertexPool(); 
	data.m_vertexCount = GetVertexCount(); 
	data.m_vertexStrideInBytes = GetStrideInBytes(); 
}


dFloat32 ndShapeStaticBVH::RayHitUser (void* const context, const dFloat32* const polygon, dInt32 strideInBytes, const dInt32* const indexArray, dInt32 indexCount)
{
	dAssert (0);
	dFloat32 t = dFloat32 (1.2f);
	ndBvhRay& me = *((ndBvhRay*) context);
	dVector normal (&polygon[indexArray[indexCount + 1] * (strideInBytes / sizeof (dFloat32))]);
	normal = normal & dVector::m_triplexMask;
dAssert (0);
	t = me.PolygonIntersect (normal, me.m_t, polygon, strideInBytes, indexArray, indexCount);
	if (t < dFloat32 (1.0f)) {
		if (t < me.m_t) {
			me.m_t = t;
			me.m_normal = normal;
			me.m_id = me.m_me->GetTagId(indexArray, indexCount);
		}
		normal = me.m_matrix.RotateVector(normal);
		t = me.m_me->GetDebugRayCastCallback() (me.m_myBody, me.m_me, t, &normal[0], dInt32 (me.m_me->GetTagId(indexArray, indexCount)), me.m_userData);
	}
	return t;
}


dVector ndShapeStaticBVH::SupportVertex (const dVector& dir) const
{
	return ForAllSectorsSupportVectex (dir);
}

dVector ndShapeStaticBVH::SupportVertex(const dVector& dir, dInt32* const vertexIndex) const
{
	return ForAllSectorsSupportVectex(dir);
}

dVector ndShapeStaticBVH::SupportVertexSpecial(const dVector& dir, dFloat32 skinThickness, dInt32* const vertexIndex) const
{
	dAssert(0);
	return SupportVertex(dir, vertexIndex);
}

void ndShapeStaticBVH::GetLocalAABB (const dVector& p0, const dVector& p1, dVector& boxP0, dVector& boxP1) const
{
	dAssert (0);
}
#endif

struct dgCollisionBVHShowPolyContext
{
	dMatrix m_matrix;
	void* m_userData;
	ndShapeDebugCallback* m_callback;
};

ndShapeStaticBVH::ndShapeStaticBVH(const dPolygonSoupBuilder& builder)
	:ndShapeStaticMesh(m_boundingBoxHierachy)
	,dAabbPolygonSoup()
	,m_trianglesCount(0)
{
	Create(builder);
	CalculateAdjacendy();

	dVector p0;
	dVector p1;
	GetAABB(p0, p1);
	m_boxSize = (p1 - p0) * dVector::m_half;
	m_boxOrigin = (p1 + p0) * dVector::m_half;

	ndMeshVertexListIndexList data;
	data.m_indexList = nullptr;
	data.m_userDataList = nullptr;
	data.m_maxIndexCount = 1000000000;
	data.m_triangleCount = 0;
	dVector zero(dVector::m_zero);
	dFastAabbInfo box(dGetIdentityMatrix(), dVector(dFloat32(1.0e15f)));
	ForAllSectors(box, zero, dFloat32(1.0f), GetTriangleCount, &data);
	m_trianglesCount = data.m_triangleCount;
}

ndShapeStaticBVH::~ndShapeStaticBVH(void)
{
}

dIntersectStatus ndShapeStaticBVH::GetTriangleCount(void* const context, const dFloat32* const polygon, dInt32 strideInBytes, const dInt32* const indexArray, dInt32 indexCount, dFloat32 hitDistance)
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

ndShapeInfo ndShapeStaticBVH::GetShapeInfo() const
{
	ndShapeInfo info(ndShapeStaticMesh::GetShapeInfo());

	info.m_bvhCollision.m_vertexCount = GetVertexCount();
	info.m_bvhCollision.m_indexCount = m_trianglesCount * 3;
	return info;
}

dIntersectStatus ndShapeStaticBVH::ShowDebugPolygon(void* const context, const dFloat32* const polygon, dInt32 strideInBytes, const dInt32* const indexArray, dInt32 indexCount, dFloat32 hitDistance)
{
	dVector poly[128];
	dInt32 stride = dInt32(strideInBytes / sizeof(dFloat32));

	dgCollisionBVHShowPolyContext& data = *(dgCollisionBVHShowPolyContext*)context;
	for (dInt32 i = 0; i < indexCount; i++) 
	{
		dVector p(&polygon[indexArray[i] * stride]);
		poly[i] = data.m_matrix.TransformVector(p & dVector::m_triplexMask);
	}
	data.m_callback->DrawPolygon(indexCount, poly);
	return t_ContinueSearh;
}

void ndShapeStaticBVH::DebugShape(const dMatrix& matrix, ndShapeDebugCallback& debugCallback) const
{
	dgCollisionBVHShowPolyContext context;

	context.m_matrix = matrix;
	context.m_userData = (void*)this;
	context.m_callback = &debugCallback;

	dFastAabbInfo box(dGetIdentityMatrix(), dVector(1.0e15f));
	ForAllSectors(box, dVector::m_zero, dFloat32(1.0f), ShowDebugPolygon, &context);
}

dFloat32 ndShapeStaticBVH::RayHit(void* const context, const dFloat32* const polygon, dInt32 strideInBytes, const dInt32* const indexArray, dInt32 indexCount)
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

dFloat32 ndShapeStaticBVH::RayCast(ndRayCastNotify& callback, const dVector& localP0, const dVector& localP1, const ndBody* const body, ndContactPoint& contactOut) const
{
	ndBvhRay ray(localP0, localP1);
	ray.m_t = dFloat32(1.0f);
	ray.m_me = this;
	ray.m_myBody = ((ndBody*)body)->GetAsBodyKinematic();
	ray.m_callback = &callback;
	dFloat32 t = dFloat32 (1.2f);
	ForAllSectorsRayHit(ray, dFloat32(1.0f), RayHit, &ray);
	if (ray.m_t < dFloat32 (1.0f))
	{
		t = ray.m_t;
		dAssert(ray.m_normal.m_w == dFloat32(0.0f));
		dAssert(ray.m_normal.DotProduct(ray.m_normal).GetScalar() > dFloat32(0.0f));
		contactOut.m_normal = ray.m_normal.Normalize();
		contactOut.m_shapeId0 = ray.m_id;
		contactOut.m_shapeId1 = ray.m_id;
	}
	return t;
}

dIntersectStatus ndShapeStaticBVH::GetPolygon(void* const context, const dFloat32* const polygon, dInt32 strideInBytes, const dInt32* const indexArray, dInt32 indexCount, dFloat32 hitDistance)
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

	//if (data.m_me->GetDebugCollisionCallback()) 
	//{
	//	dTriplex triplex[128];
	//	dInt32 stride = dInt32(strideInBytes / sizeof(dFloat32));
	//	const dVector scale = data.m_polySoupInstance->GetScale();
	//	dMatrix matrix(data.m_polySoupInstance->GetLocalMatrix() * data.m_polySoupBody->GetMatrix());
	//	for (dInt32 i = 0; i < indexCount; i++) {
	//		dVector p(matrix.TransformVector(scale * (dVector(&polygon[indexArray[i] * stride]) & dVector::m_triplexMask)));
	//		triplex[i].m_x = p.m_x;
	//		triplex[i].m_y = p.m_y;
	//		triplex[i].m_z = p.m_z;
	//	}
	//	if (data.m_polySoupBody) 
	//	{
	//		data.m_me->GetDebugCollisionCallback() (data.m_polySoupBody, data.m_objBody, indexArray[indexCount], indexCount, &triplex[0].m_x, sizeof(dTriplex));
	//	}
	//}

	dAssert(data.m_vertex == polygon);
	dInt32 count = indexCount * 2 + 3;

	data.m_faceIndexCount[data.m_faceCount] = indexCount;
	//data.m_faceIndexStart[data.m_faceCount] = data.m_faceCount ? (data.m_faceIndexStart[data.m_faceCount - 1] + data.m_faceIndexCount[data.m_faceCount - 1]) : 0;
	data.m_faceIndexStart[data.m_faceCount] = data.m_globalIndexCount;
	data.m_hitDistance[data.m_faceCount] = hitDistance;
	data.m_faceCount++;
	dInt32* const dst = &data.m_faceVertexIndex[data.m_globalIndexCount];

	//the docks say memcpy is an intrinsic function but as usual this is another Microsoft lied
	//memcpy (dst, indexArray, sizeof (dInt32) * count);
	for (dInt32 i = 0; i < count; i++) 
	{
		dst[i] = indexArray[i];
	}

	data.m_globalIndexCount += count;

	return t_ContinueSearh;
}


void ndShapeStaticBVH::GetCollidingFaces(ndPolygonMeshDesc* const data) const
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

void ndShapeStaticBVH::Save(void* const xmlNode, dInt32 nodeid) const
{


}