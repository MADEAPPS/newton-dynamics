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
#include "dgCollisionUserMesh.h"


//dgCollisionUserMesh::dgCollisionUserMesh(dgMemoryAllocator* allocator, const dgVector& boxP0, const dgVector& boxP1, const dgUserMeshCreation& data)
//	:dgCollisionMesh (allocator, m_userMesh)
dgCollisionUserMesh::dgCollisionUserMesh(dgWorld* const world, const dgVector& boxP0, const dgVector& boxP1, const dgUserMeshCreation& data)
	:dgCollisionMesh (world, m_userMesh)
{
	m_rtti |= dgCollisionUserMesh_RTTI;

	m_userData = data.m_userData;
	m_getInfoCallback = data.m_getInfoCallback;
	m_faceInAABBCalback = data.m_faceInAABBCallback;
	m_rayHitCallback = data.m_rayHitCallback;
	m_collideCallback = data.m_collideCallback;
	m_destroyCallback = data.m_destroyCallback;
	m_serializeCallback = data.m_serializeCallback;
	m_getAABBOvelapTestCallback = data.m_getAABBOvelapTestCallback;

	SetCollisionBBox(boxP0, boxP1);
}

dgCollisionUserMesh::dgCollisionUserMesh (dgWorld* const world, dgDeserialize deserialization, void* const userData)
	:dgCollisionMesh (world, deserialization, userData)
{
dgAssert (0);
	m_rtti |= dgCollisionUserMesh_RTTI;
	
/*
	dgAABBPolygonSoup::Deserialize (deserialization, userData);

	dgVector p0; 
	dgVector p1; 
	GetAABB (p0, p1);
	SetCollisionBBox(p0, p1);
*/
}

dgCollisionUserMesh::~dgCollisionUserMesh(void)
{
	if (m_destroyCallback) {
		m_destroyCallback (m_userData);
	}
}

void dgCollisionUserMesh::Serialize(dgSerialize callback, void* const userData) const
{
	SerializeLow(callback, userData);
	if (m_serializeCallback) {
		m_serializeCallback (m_userData, callback, userData);
	}
}


void dgCollisionUserMesh::GetVertexListIndexList (const dgVector& p0, const dgVector& p1, dgMeshVertexListIndexList &data) const
{
	if (m_faceInAABBCalback) {
		return m_faceInAABBCalback (m_userData, &p0[0], &p1[0], (const dgFloat32**) &data.m_veterxArray, &data.m_vertexCount, &data.m_vertexStrideInBytes,
							 data.m_indexList, data.m_maxIndexCount, data.m_userDataList);

	} else {
		data.m_triangleCount = 0;
	}
}

void dgCollisionUserMesh::GetCollisionInfo(dgCollisionInfo* const info) const
{
	dgCollision::GetCollisionInfo(info);
	if (m_getInfoCallback) {
		m_getInfoCallback (m_userData, info);
	}
}

bool dgCollisionUserMesh::AABBOvelapTest (const dgVector& boxP0, const dgVector& boxP1) const
{
	return (m_getAABBOvelapTestCallback && m_getAABBOvelapTestCallback (m_userData, boxP0, boxP1)) ? true : false;
}


dgFloat32 dgCollisionUserMesh::RayCast (const dgVector& localP0, const dgVector& localP1, dgFloat32 maxT, dgContactPoint& contactOut, const dgBody* const body,	void* const userData) const
{
	dgFloat32 param = dgFloat32 (1.2f);
	if (m_rayHitCallback) {
		dgCollisionMeshRayHitDesc data;
		data.m_localP0 = localP0;
		data.m_localP1 = localP1;
		data.m_userId = body->m_collision->GetUserDataID();
		data.m_userData = m_userData;
		data.m_altenateUserData = userData;
		if (body) {
			data.m_matrix = body->m_collision->GetGlobalMatrix();
		}

		dgFloat32 t = m_rayHitCallback (data);
		if ((t < dgFloat32 (1.0f)) && (t > dgFloat32 (0.0f))) {
			param = t;
			contactOut.m_normal = data.m_normal;
//			contactOut.m_userId = data.m_userId;
			contactOut.m_shapeId0 = dgInt64 (data.m_userId);
		} 
	}
	return param;
}


dgVector dgCollisionUserMesh::SupportVertex (const dgVector& dir) const
{
	dgAssert (0);
	return dgVector (0, 0, 0, 0);
}


void dgCollisionUserMesh::GetCollidingFaces (dgPolygonMeshDesc* const data) const
{
	data->m_faceCount = 0;

	if (m_collideCallback) {
		data->m_me = this;
		data->m_userData = m_userData;
		m_collideCallback (&data->m_p0);

		dgInt32 faceCount0 = 0; 
		dgInt32 faceIndexCount0 = 0; 
		dgInt32 faceIndexCount1 = 0; 
		dgInt32 stride = data->m_vertexStrideInBytes / sizeof (dgFloat32);
		dgFloat32* const vertex = data->m_vertex;
		dgInt32* const address = data->m_meshData.m_globalFaceIndexStart;
		dgFloat32* const hitDistance = data->m_meshData.m_globalHitDistance;
		dgInt32* const indices = data->m_faceVertexIndex;
		dgInt32* const faceIndexCount = data->m_faceIndexCount; 

		for (dgInt32 i = 0; i < data->m_faceCount; i ++) {
			dgInt32 indexCount = faceIndexCount[i];
			const dgInt32* const indexArray = &indices[faceIndexCount1]; 

			dgInt32 normalIndex = data->GetNormalIndex (indexArray, indexCount);
			const dgVector& faceNormal = vertex[normalIndex * stride];
			dgFloat32 dist = data->PolygonBoxDistance (faceNormal, indexCount, indexArray, stride, vertex);

			const dgInt32 faceIndexCount = data->GetFaceIndexCount(indexCount); 
			if (dist > dgFloat32 (0.0f)) {
				hitDistance[faceCount0] = dist;
				address[faceCount0] = faceIndexCount0;
				memcpy (&indices[faceIndexCount0], indexArray, faceIndexCount * sizeof (dgInt32));
				faceCount0 ++;
				faceIndexCount0 += faceIndexCount;
			}
			faceIndexCount1 += faceIndexCount;
		}

		data->m_faceCount = 0;
		if (faceCount0) {
			data->m_faceCount = faceCount0;
			data->m_faceIndexStart = address;
			data->m_hitDistance = hitDistance;
		}
	}
}


void dgCollisionUserMesh::DebugCollision (const dgMatrix& matrixPtr, OnDebugCollisionMeshCallback callback, void* const userData) const
{
/*
	dgCollisionUserMeshShowPolyContext context;

	context.m_matrix = matrixPtr;
	context.m_userData = userData;;
	context.m_callback = callback;

	dgVector p0 (dgFloat32 (-1.0e20f), dgFloat32 (-1.0e20f), dgFloat32 (-1.0e20f), dgFloat32 (0.0f));
	dgVector p1 (dgFloat32 ( 1.0e20f), dgFloat32 ( 1.0e20f), dgFloat32 ( 1.0e20f), dgFloat32 (0.0f));
	ForAllSectors (p0, p1, ShowDebugPolygon, &context);
*/
}
