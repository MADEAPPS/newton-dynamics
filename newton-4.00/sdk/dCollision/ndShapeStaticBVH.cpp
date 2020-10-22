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
#include "ndShapeStaticBVH.h"

#if 0
#include "dgPhysicsStdafx.h"
#include "dgBody.h"
#include "dgWorld.h"
#include "ndShapeStaticBVH.h"



ndShapeStaticBVH::ndShapeStaticBVH (dgWorld* const world, dgDeserialize deserialization, void* const userData, dInt32 revisionNumber)
	:ndShapeStaticMesh (world, deserialization, userData, revisionNumber)
	,dgAABBPolygonSoup()
	,m_trianglesCount(0)
{
	dgAssert (m_rtti | dgCollisionBVH_RTTI);
	m_builder = NULL;;
	m_userRayCastCallback = NULL;

	dgAABBPolygonSoup::Deserialize (deserialization, userData, revisionNumber);

	dVector p0; 
	dVector p1; 
	GetAABB (p0, p1);
	SetCollisionBBox(p0, p1);

	deserialization(userData, &m_trianglesCount, sizeof (dInt32));
}

void ndShapeStaticBVH::Serialize(dgSerialize callback, void* const userData) const
{
	SerializeLow(callback, userData);
	dgAABBPolygonSoup::Serialize ((dgSerialize) callback, userData);
	callback(userData, &m_trianglesCount, sizeof (dInt32));
}

void ndShapeStaticBVH::BeginBuild()
{
	m_builder = new (m_allocator) dgPolygonSoupDatabaseBuilder(m_allocator);
	m_builder->Begin();
}

void ndShapeStaticBVH::AddFace(dInt32 vertexCount, const dFloat32* const vertexPtr, dInt32 strideInBytes, dInt32 faceAttribute)
{
	dInt32 faceArray;
	dInt32 indexList[256];

	faceArray = vertexCount;
	dgAssert (vertexCount < dInt32 (sizeof (indexList) / sizeof (indexList[0])));
	for (dInt32 i = 0; i < vertexCount; i ++) {
		indexList[i] = i;
	}
	m_builder->AddMesh (vertexPtr, vertexCount, strideInBytes, 1, &faceArray, indexList, &faceAttribute, dGetIdentityMatrix());
}

void ndShapeStaticBVH::SetCollisionRayCastCallback (dgCollisionBVHUserRayCastCallback rayCastCallback)
{
	m_userRayCastCallback = rayCastCallback;
}

void ndShapeStaticBVH::EndBuild(dInt32 optimize)
{
	dVector p0;
	dVector p1;

	bool state = optimize ? true : false;

#ifdef _DEBUG
	if (state && (optimize >> 1)) {
		char debugMesh[256];
		sprintf (debugMesh, "debugMesh_%d.ply", optimize-1);
		m_builder->SavePLY(debugMesh);
	}
#endif

	m_builder->End(state);
	Create (*m_builder, state);
	CalculateAdjacendy();
	
	GetAABB (p0, p1);
	SetCollisionBBox (p0, p1);

	delete m_builder;
	m_builder = NULL;
		
	dgMeshVertexListIndexList data;
	data.m_indexList = NULL;
	data.m_userDataList = NULL;
	data.m_maxIndexCount = 1000000000;
	data.m_triangleCount = 0; 
	dVector zero (dFloat32 (0.0f));
	dFastAabbInfo box (dGetIdentityMatrix(), dVector (dFloat32 (1.0e15f)));
	ForAllSectors (box, zero, dFloat32 (1.0f), GetTriangleCount, &data);
	m_trianglesCount = data.m_triangleCount;
}


void ndShapeStaticBVH::GetCollisionInfo(dgCollisionInfo* const info) const
{
	dgCollision::GetCollisionInfo(info);

	info->m_bvhCollision.m_vertexCount = GetVertexCount();
	info->m_bvhCollision.m_indexCount = m_trianglesCount * 3;
}

void ndShapeStaticBVH::ForEachFace (dAaabbIntersectCallback callback, void* const context) const
{
	dVector p0 (-1.0e10f, -1.0e10f, -1.0e10f, 1.0f);
	dVector p1 ( 1.0e10f,  1.0e10f,  1.0e10f, 1.0f);
	dVector zero (dFloat32 (0.0f));
	dFastAabbInfo box (dGetIdentityMatrix(), dVector (dFloat32 (1.0e15f)));
	//ForAllSectors (p0, p1, zero, dFloat32 (1.0f), callback, context);
	ForAllSectors (box, zero, dFloat32 (1.0f), callback, context);
}


dgIntersectStatus ndShapeStaticBVH::CollectVertexListIndexList (void* const context, const dFloat32* const polygon, dInt32 strideInBytes, const dInt32* const indexArray, dInt32 indexCount, dFloat32 hitDistance)
{
	dgMeshVertexListIndexList& data = (*(dgMeshVertexListIndexList*) context);

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

	dgAssert (j <= data.m_maxIndexCount);
	data.m_triangleCount = k;
	dgAssert ((data.m_triangleCount * 3) <= data.m_maxIndexCount);

	return t_ContinueSearh;
}



dgIntersectStatus ndShapeStaticBVH::GetTriangleCount (void* const context, const dFloat32* const polygon, dInt32 strideInBytes, const dInt32* const indexArray, dInt32 indexCount, dFloat32 hitDistance)
{
	dgMeshVertexListIndexList& data = (*(dgMeshVertexListIndexList*) context);

	if ((data.m_triangleCount + indexCount - 2) * 3 > data.m_maxIndexCount) {
		return t_StopSearh;
	}

	data.m_triangleCount += (indexCount - 2);
	dgAssert ((data.m_triangleCount * 3) <= data.m_maxIndexCount);
	return t_ContinueSearh;
}


void ndShapeStaticBVH::GetVertexListIndexList (const dVector& p0, const dVector& p1, dgMeshVertexListIndexList &data) const
{
	dFastAabbInfo box (p0, p1);
	ForAllSectors (box, dVector (dFloat32 (0.0f)), dFloat32 (1.0f), CollectVertexListIndexList, &data);

	data.m_veterxArray = GetLocalVertexPool(); 
	data.m_vertexCount = GetVertexCount(); 
	data.m_vertexStrideInBytes = GetStrideInBytes(); 

}



dFloat32 ndShapeStaticBVH::RayHit (void* const context, const dFloat32* const polygon, dInt32 strideInBytes, const dInt32* const indexArray, dInt32 indexCount)
{
	dgBVHRay& me = *((dgBVHRay*) context);
	dVector normal (&polygon[indexArray[indexCount + 1] * (strideInBytes / sizeof (dFloat32))]);
	normal = normal & dVector::m_triplexMask;
	dFloat32 t = me.PolygonIntersect (normal, me.m_t, polygon, strideInBytes, indexArray, indexCount);
	if (t <= (me.m_t * dFloat32 (1.0001f))) {
//		if ((t * dFloat32 (1.0001f)) >= me.m_t) {
//			dFloat32 dist0;
//			dFloat32 dist1;
//			dist0 = me.m_diff % normal;
//			dist1 = me.m_diff % me.m_normal;
//			if (dist0 < dist1) {
//				me.m_t = t;
//				me.m_normal = normal;
//				me.m_id = me.m_me->GetTagId(indexArray, indexCount);
//			} else {
//				t = me.m_t;
//			}
//		} else {
			me.m_t = t;
			me.m_normal = normal;
			me.m_id = me.m_me->GetTagId(indexArray, indexCount);
//		}
	}
	return t;
}

dFloat32 ndShapeStaticBVH::RayHitUser (void* const context, const dFloat32* const polygon, dInt32 strideInBytes, const dInt32* const indexArray, dInt32 indexCount)
{
	dgAssert (0);
	dFloat32 t = dFloat32 (1.2f);
	dgBVHRay& me = *((dgBVHRay*) context);
	dVector normal (&polygon[indexArray[indexCount + 1] * (strideInBytes / sizeof (dFloat32))]);
	normal = normal & dVector::m_triplexMask;
dgAssert (0);
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




dgIntersectStatus ndShapeStaticBVH::GetPolygon (void* const context, const dFloat32* const polygon, dInt32 strideInBytes, const dInt32* const indexArray, dInt32 indexCount, dFloat32 hitDistance)
{
	dgPolygonMeshDesc& data = (*(dgPolygonMeshDesc*) context);
	if (data.m_faceCount >= DG_MAX_COLLIDING_FACES) {
		dgTrace (("buffer Over float, try using a lower resolution mesh for collision\n"));
		return t_StopSearh;
	}
	if ((data.m_globalIndexCount + indexCount * 2 + 3) >= DG_MAX_COLLIDING_INDICES) {
		dgTrace (("buffer Over float, try using a lower resolution mesh for collision\n"));
		return t_StopSearh;
	}

	if (data.m_me->GetDebugCollisionCallback()) { 
		dgTriplex triplex[128];
		dInt32 stride = dInt32 (strideInBytes / sizeof (dFloat32));
		const dVector scale = data.m_polySoupInstance->GetScale();
		dMatrix matrix (data.m_polySoupInstance->GetLocalMatrix() * data.m_polySoupBody->GetMatrix());
		for (dInt32 i = 0; i < indexCount; i ++ ) {
			dVector p (matrix.TransformVector(scale * (dVector(&polygon[indexArray[i] * stride]) & dVector::m_triplexMask))); 
			triplex[i].m_x = p.m_x;
			triplex[i].m_y = p.m_y;
			triplex[i].m_z = p.m_z;
		}
		if (data.m_polySoupBody) {
			data.m_me->GetDebugCollisionCallback() (data.m_polySoupBody, data.m_objBody, indexArray[indexCount], indexCount, &triplex[0].m_x, sizeof (dgTriplex));
		}
	}

	dgAssert (data.m_vertex == polygon);
	dInt32 count = indexCount * 2 + 3;

	data.m_faceIndexCount[data.m_faceCount] = indexCount;
//	data.m_faceIndexStart[data.m_faceCount] = data.m_faceCount ? (data.m_faceIndexStart[data.m_faceCount - 1] + data.m_faceIndexCount[data.m_faceCount - 1]) : 0;
	data.m_faceIndexStart[data.m_faceCount] = data.m_globalIndexCount;
	data.m_hitDistance[data.m_faceCount] = hitDistance;
	data.m_faceCount ++;
	dInt32* const dst = &data.m_faceVertexIndex[data.m_globalIndexCount];

	//the docks say memcpy is an intrinsic function but as usual this is another Microsoft lied
	//memcpy (dst, indexArray, sizeof (dInt32) * count);
	for (dInt32 i = 0; i < count; i ++) {
		dst[i] = indexArray[i];
	}
	
	data.m_globalIndexCount += count;
	
	return t_ContinueSearh;
}

void ndShapeStaticBVH::GetCollidingFaces (dgPolygonMeshDesc* const data) const
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
	ForAllSectors (*data, data->m_boxDistanceTravelInMeshSpace, data->m_maxT, GetPolygon, data);
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
	dgAssert(0);
	return SupportVertex(dir, vertexIndex);
}



void ndShapeStaticBVH::GetLocalAABB (const dVector& p0, const dVector& p1, dVector& boxP0, dVector& boxP1) const
{
	dgAssert (0);
}

struct dgCollisionBVHShowPolyContext
{
	dMatrix m_matrix;
	void* m_userData;
	dgCollision::OnDebugCollisionMeshCallback m_callback;
};

dgIntersectStatus ndShapeStaticBVH::ShowDebugPolygon (void* const context, const dFloat32* const polygon, dInt32 strideInBytes, const dInt32* const indexArray, dInt32 indexCount, dFloat32 hitDistance)
{
	dgTriplex triplex[128];
	dInt32 stride = dInt32 (strideInBytes / sizeof (dFloat32));

	dgCollisionBVHShowPolyContext& data = *(dgCollisionBVHShowPolyContext*) context;
	for (dInt32 i = 0; i < indexCount; i ++ ) {
		dVector p (&polygon[indexArray[i] * stride]);
		p = p & dVector::m_triplexMask;
		p = data.m_matrix.TransformVector(p);
		triplex[i].m_x = p.m_x;
		triplex[i].m_y = p.m_y;
		triplex[i].m_z = p.m_z;
	}
//	data.m_callback (data.m_userData, indexCount, &triplex[0].m_x, indexArray[-1]);
	data.m_callback (data.m_userData, indexCount, &triplex[0].m_x, indexArray[indexCount]);

	return t_ContinueSearh;
}

void ndShapeStaticBVH::DebugCollision (const dMatrix& matrixPtr, dgCollision::OnDebugCollisionMeshCallback callback, void* const userData) const
{
	dgCollisionBVHShowPolyContext context;

	context.m_matrix = matrixPtr;
	context.m_userData = userData;;
	context.m_callback = callback;

	dFastAabbInfo box (dGetIdentityMatrix(), dVector (1.0e15f));
	ForAllSectors (box, dVector(dFloat32 (0.0f)), dFloat32 (1.0f), ShowDebugPolygon, &context);
}
#endif

ndShapeStaticBVH::ndShapeStaticBVH(const dPolygonSoupBuilder& builder)
	:ndShapeStaticMesh(m_boundingBoxHierachy)
	,dAabbPolygonSoup()
//	,m_trianglesCount(0)
{
	Create(builder, true);
	CalculateAdjacendy();

	dVector p0;
	dVector p1;
	GetAABB(p0, p1);
	//SetCollisionBBox(p0, p1);
	m_boxSize = (p1 - p0) * dVector::m_half;
	m_boxOrigin = (p1 + p0) * dVector::m_half;
	//m_boxMinRadius = dMin(m_boxSize.m_x, m_boxSize.m_y, m_boxSize.m_z);
	//m_boxMaxRadius = dSqrt((m_boxSize.DotProduct(m_boxSize)).GetScalar());
}

ndShapeStaticBVH::~ndShapeStaticBVH(void)
{
	dAssert(0);
}

void ndShapeStaticBVH::DebugShape(const dMatrix& matrix, ndShapeDebugCallback& debugCallback) const
{
	dAssert(0);
}

//dFloat32 ndShapeStaticBVH::RayCast(const dVector& localP0, const dVector& localP1, dFloat32 maxT, dgContactPoint& contactOut, const dgBody* const body, void* const userData, OnRayPrecastAction preFilter) const
dFloat32 ndShapeStaticBVH::RayCast(ndRayCastNotify& callback, const dVector& localP0, const dVector& localP1, dFloat32 maxT, const ndBody* const body, ndContactPoint& contactOut) const
{
	dAssert(0);
	return 0;
	//dgBVHRay ray(localP0, localP1);
	//ray.m_t = dgMin(maxT, dFloat32(1.0f));
	//ray.m_me = this;
	//ray.m_userData = userData;
	//if (!m_userRayCastCallback) {
	//	ForAllSectorsRayHit(ray, maxT, RayHit, &ray);
	//	if (ray.m_t < maxT) {
	//		maxT = ray.m_t;
	//		dgAssert(ray.m_normal.m_w == dFloat32(0.0f));
	//		dgAssert(ray.m_normal.DotProduct(ray.m_normal).GetScalar() > dFloat32(0.0f));
	//		//contactOut.m_normal = ray.m_normal.Scale (dgRsqrt (ray.m_normal.DotProduct(ray.m_normal).GetScalar() + dFloat32 (1.0e-8f)));
	//		contactOut.m_normal = ray.m_normal.Normalize();
	//		//contactOut.m_userId = ray.m_id;
	//		contactOut.m_shapeId0 = ray.m_id;
	//		contactOut.m_shapeId1 = ray.m_id;
	//	}
	//}
	//else {
	//	if (body) {
	//		//ray.m_matrix = body->m_collisionWorldMatrix;
	//		ray.m_matrix = body->m_collision->GetGlobalMatrix();
	//	}
	//
	//	ForAllSectorsRayHit(ray, maxT, RayHitUser, &ray);
	//	if (ray.m_t < dFloat32(1.0f)) {
	//		maxT = ray.m_t;
	//		dgAssert(ray.m_normal.m_w == dFloat32(0.0f));
	//		dgAssert(ray.m_normal.DotProduct(ray.m_normal).GetScalar() > dFloat32(0.0f));
	//		//contactOut.m_normal = ray.m_normal.Scale (dgRsqrt (ray.m_normal.DotProduct(ray.m_normal).GetScalar() + dFloat32 (1.0e-8f)));
	//		contactOut.m_normal = ray.m_normal.Normalize();
	//		//contactOut.m_userId = ray.m_id;
	//		contactOut.m_shapeId0 = ray.m_id;
	//		contactOut.m_shapeId1 = ray.m_id;
	//	}
	//}
	//return maxT;
}
