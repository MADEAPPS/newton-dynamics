/* Copyright (c) <2003-2016> <Julio Jerez, Newton Game Dynamics>
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
#include "dgContact.h"
#include "dgMeshEffect.h"
#include "dgCollisionBVH.h"
#include "dgCollisionConvexPolygon.h"
#include "dgCollisionDeformableSolidMesh.h"


dgCollisionDeformableSolidMesh::dgCollisionDeformableSolidMesh(dgWorld* const world, dgMeshEffect* const mesh)
	:dgCollisionDeformableMesh(world, m_deformableSolidMesh)
{
	m_rtti |= dgCollisionDeformableSolidMesh_RTTI;

//	dgArray<dgVector> m_posit;
//	dgArray<dgVector> m_veloc;
//	dgArray<dgVector> m_accel;
//	dgArray<dgVector> m_externalforce;

	dgInt32 count = mesh->GetVertexCount();
	dgVector* const points = dgAlloca (dgVector, count);

	for (dgInt32 i = 0; i < count; i++) {
		dgBigVector p(mesh->GetVertex(i));
		points[i] = p;
	}
	m_indexToVertexMap.Resize(count);
	dgInt32* const indexToVertexMap = &m_indexToVertexMap[0];
	m_particlesCount = dgVertexListToIndexList (&points[0].m_x, sizeof (dgVector), 3 * sizeof (dgFloat32), 0, count, indexToVertexMap, dgFloat32 (1.0e-5f));
	for (dgInt32 i = 0; i < m_particlesCount; i++) {
		m_posit[i] = points[i];
	}

	dgInt32 edgeCount = 0;
	dgSoftLink* const links = dgAlloca (dgSoftLink, mesh->GetCount() / 2);
	for (void* edgePtr = mesh->GetFirstEdge(); edgePtr; edgePtr = mesh->GetNextEdge(edgePtr)) {
		dgInt32 v0;
		dgInt32 v1;
		mesh->GetEdgeIndex(edgePtr, v0, v1);
		v0 = indexToVertexMap[v0];
		v1 = indexToVertexMap[v1];
		links[edgeCount].m_m0 = dgInt16(dgMin (v0, v1));
		links[edgeCount].m_m1 = dgInt16(dgMax (v0, v1));
		edgeCount ++;
	}
	dgAssert(edgeCount == mesh->GetCount() / 2);
	dgSort(links, edgeCount, CompareEdges);

	dgInt32 uniqueEdgeCount = 0;
	for (dgInt32 i = 1; i < edgeCount; i ++) {
		if (CompareEdges (&links[i], &links[uniqueEdgeCount], NULL) > 0) {
			uniqueEdgeCount++;
			links[uniqueEdgeCount] = links[i];
		}
	}
	uniqueEdgeCount++;
	m_linksCount = uniqueEdgeCount;
	m_indexToVertexMap.Resize(m_linksCount);
	for (dgInt32 i = 0; i < m_linksCount; i ++) {
		m_linkList[i] = links[i];
	}
	FinalizeBuild();
}

dgCollisionDeformableSolidMesh::dgCollisionDeformableSolidMesh(const dgCollisionDeformableSolidMesh& source)
	:dgCollisionDeformableMesh(source)
{
	m_rtti |= source.m_rtti;
}

dgCollisionDeformableSolidMesh::dgCollisionDeformableSolidMesh(dgWorld* const world, dgDeserialize deserialization, void* const userData, dgInt32 revisionNumber)
	:dgCollisionDeformableMesh(world, deserialization, userData, revisionNumber)
{
}


dgCollisionDeformableSolidMesh::~dgCollisionDeformableSolidMesh(void)
{
}


dgInt32 dgCollisionDeformableSolidMesh::CompareEdges (const dgSoftLink* const A, const dgSoftLink* const B, void* const context)
{
	dgInt64 m0 = (dgInt64(A->m_m0)<<32) + A->m_m1;
	dgInt64 m1 = (dgInt64(B->m_m0)<<32) + B->m_m1;
	if (m0 > m1) {
		return 1;
	} else if (m0 < m1) {
		return -1;
	} 
	return 0;
}