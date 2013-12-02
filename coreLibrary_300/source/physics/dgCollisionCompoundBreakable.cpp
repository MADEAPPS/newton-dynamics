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
#include "dgWorld.h"
#include "dgMeshEffect.h"
#include "dgCollisionConvexHull.h"
#include "dgCollisionCompoundBreakable.h"

#if 0

#define DG_DYNAMINIC_ISLAND_COST		0x7fffffff

dgCollisionCompoundBreakable::dgCollisionConvexIntance::dgCollisionConvexIntance (
	dgCollisionConvex* convexChild, 
	dgCollisionCompoundBreakable::dgDebriGraph::dgListNode* node,
	dgFloat32 density)
	:dgCollisionConvex(convexChild->GetAllocator(), 0, dgCollisionID(0)) 
{
	dgAssert (0);
/*
	m_treeNode = NULL;
	m_graphNode = node;
	m_myShape = convexChild;
	m_myShape->AddRef();
	SetOffsetMatrix (m_myShape->GetLocalMatrix());

	((dgCollision*)convexChild)->CalculateInertia(m_inertia, m_volume);
	m_volume.m_w = ((dgCollision*)convexChild)->GetVolume();
	
	m_inertia = m_inertia.Scale (density * m_volume.m_w);
	m_inertia.m_w = density * m_volume.m_w;
	

SetBreakImpulse(2500);

	m_vertexCount = 1;
*/
}

dgCollisionCompoundBreakable::dgCollisionConvexIntance::dgCollisionConvexIntance (const dgCollisionConvexIntance& source, dgCollisionCompoundBreakable::dgDebriGraph::dgListNode* node)
	:dgCollisionConvex (source.GetAllocator(), 0, dgCollisionID(0)), m_inertia(source.m_inertia) 
{
	dgAssert (0);
/*
	m_treeNode = NULL;
	m_graphNode = node;
	m_myShape = source.m_myShape;
	m_myShape->AddRef();
	m_vertexCount = 1;
	m_volume = source.m_volume;
	m_destructionImpulse = source.m_destructionImpulse;
*/
}

dgCollisionCompoundBreakable::dgCollisionConvexIntance::dgCollisionConvexIntance (dgWorld* const world, dgDeserialize deserialization, void* const userData)
	:dgCollisionConvex (world, deserialization, userData) 
{
	dgAssert (0);
/*
	m_graphNode = NULL;
	m_treeNode = NULL;
	m_vertexCount = 1;

	deserialization (userData, &m_volume, sizeof (m_volume));
	deserialization (userData, &m_inertia, sizeof (m_inertia));
	deserialization (userData, &m_destructionImpulse, sizeof (m_destructionImpulse));
	
  	m_myShape = new (world->GetAllocator()) dgCollisionConvexHull(world, deserialization, userData);
*/
}

dgCollisionCompoundBreakable::dgCollisionConvexIntance::~dgCollisionConvexIntance()
{
	m_myShape->Release();
}


dgFloat32 dgCollisionCompoundBreakable::dgCollisionConvexIntance::GetVolume () const
{
	return ((dgCollision*)m_myShape)->GetVolume ();
}


dgVector dgCollisionCompoundBreakable::dgCollisionConvexIntance::SupportVertex (const dgVector& dir) const
{
	return m_myShape->SupportVertex (dir);
}



void dgCollisionCompoundBreakable::dgCollisionConvexIntance::CalcAABB (const dgMatrix& matrix, dgVector& p0, dgVector& p1) const
{
	m_myShape->CalcAABB (matrix, p0, p1);
}


void dgCollisionCompoundBreakable::dgCollisionConvexIntance::DebugCollision (const dgMatrix& matrix, OnDebugCollisionMeshCallback callback, void* const userData) const
{
	((dgCollision*)m_myShape)->DebugCollision (matrix, callback, userData);
}

dgFloat32 dgCollisionCompoundBreakable::dgCollisionConvexIntance::RayCast (const dgVector& localP0, const dgVector& localP1, dgContactPoint& contactOut, const dgBody* const body, void* const userData) const
{
	return m_myShape->RayCast (localP0, localP1, contactOut, body, userData);
}


dgVector dgCollisionCompoundBreakable::dgCollisionConvexIntance::CalculateVolumeIntegral (const dgMatrix& globalMatrix, const dgVector& plane) const
{
	return ((dgCollision*)m_myShape)->CalculateVolumeIntegral (globalMatrix, bouyancyPlane, context);
}


dgInt32 dgCollisionCompoundBreakable::dgCollisionConvexIntance::CalculateSignature () const
{
	return 0;
//		return ((dgCollision*)m_myShape)->CalculateSignature ();
}

void dgCollisionCompoundBreakable::dgCollisionConvexIntance::SetCollisionBBox (const dgVector& p0, const dgVector& p1)
{
	m_myShape->SetCollisionBBox (p0, p1);
}


dgFloat32 dgCollisionCompoundBreakable::dgCollisionConvexIntance::GetBoxMinRadius () const
{
	return ((dgCollision*)m_myShape)->GetBoxMinRadius ();
}

dgFloat32 dgCollisionCompoundBreakable::dgCollisionConvexIntance::GetBoxMaxRadius () const
{
	return ((dgCollision*)m_myShape)->GetBoxMaxRadius ();
}


dgInt32 dgCollisionCompoundBreakable::dgCollisionConvexIntance::CalculatePlaneIntersection (const dgVector& normal, const dgVector& point, dgVector* const contactsOut)  const
{
	return m_myShape->CalculatePlaneIntersection (normal, point, contactsOut);
}



void dgCollisionCompoundBreakable::dgCollisionConvexIntance::GetCollisionInfo(dgCollisionInfo* const info) const
{
	m_myShape->GetCollisionInfo(info);
}

void dgCollisionCompoundBreakable::dgCollisionConvexIntance::Serialize(dgSerialize callback, void* const userData) const
{
dgAssert (0);
/*
	SerializeLow(callback, userData);

	callback (userData, &m_volume, sizeof (m_volume));
	callback (userData, &m_inertia, sizeof (m_inertia));
	callback (userData, &m_destructionImpulse, sizeof (m_destructionImpulse));

	m_myShape->Serialize(callback, userData);
*/
}

void dgCollisionCompoundBreakable::dgCollisionConvexIntance::SetBreakImpulse(dgFloat32 force)
{
	m_destructionImpulse = force;
}

dgFloat32 dgCollisionCompoundBreakable::dgCollisionConvexIntance::GetBreakImpulse() const
{
	return m_destructionImpulse;
}


dgCollisionCompoundBreakable::dgVertexBuffer::dgVertexBuffer(dgInt32 vertsCount, dgMemoryAllocator* allocator)
{
	m_allocator = allocator;
	m_vertexCount = vertsCount;
	m_uv = (dgFloat32 *) m_allocator->Malloc (2 * vertsCount * dgInt32 (sizeof (dgFloat32))); 
	m_vertex = (dgFloat32 *) m_allocator->Malloc (3 * vertsCount * dgInt32 (sizeof (dgFloat32))); 
	m_normal = (dgFloat32 *) m_allocator->Malloc (3 * vertsCount * dgInt32 (sizeof (dgFloat32))); 
}

dgCollisionCompoundBreakable::dgVertexBuffer::~dgVertexBuffer ()
{
	m_allocator->Free (m_normal);
	m_allocator->Free (m_vertex);
	m_allocator->Free (m_uv);
}

dgCollisionCompoundBreakable::dgVertexBuffer::dgVertexBuffer (dgMemoryAllocator* const allocator, dgDeserialize callback, void* const userData)
{
	m_allocator = allocator;
	callback (userData, &m_vertexCount, dgInt32 (sizeof (dgInt32)));

	m_uv = (dgFloat32 *) m_allocator->Malloc (2 * m_vertexCount * dgInt32 (sizeof (dgFloat32))); 
	m_vertex = (dgFloat32 *) m_allocator->Malloc (3 * m_vertexCount * dgInt32 (sizeof (dgFloat32))); 
	m_normal = (dgFloat32 *) m_allocator->Malloc (3 * m_vertexCount * dgInt32 (sizeof (dgFloat32))); 

	callback (userData, m_vertex, size_t (3 * m_vertexCount * dgInt32 (sizeof (dgFloat32))));
	callback (userData, m_normal, size_t (3 * m_vertexCount * dgInt32 (sizeof (dgFloat32))));
	callback (userData, m_uv, size_t (2 * m_vertexCount * dgInt32 (sizeof (dgFloat32))));
}

void dgCollisionCompoundBreakable::dgVertexBuffer::Serialize(dgSerialize callback, void* const userData) const
{
	callback (userData, &m_vertexCount, dgInt32 (sizeof (dgInt32)));
	callback (userData, m_vertex, size_t (3 * m_vertexCount * dgInt32 (sizeof (dgFloat32))));
	callback (userData, m_normal, size_t (3 * m_vertexCount * dgInt32 (sizeof (dgFloat32))));
	callback (userData, m_uv, size_t (2 * m_vertexCount * dgInt32 (sizeof (dgFloat32))));
}


void dgCollisionCompoundBreakable::dgVertexBuffer::GetVertexStreams (dgInt32 vertexStrideInByte, dgFloat32* vertex, dgInt32 normalStrideInByte, dgFloat32* normal,
	dgInt32 uvStrideInByte, dgFloat32* uv) const
{
	uvStrideInByte /= dgInt32 (sizeof (dgFloat32));
	vertexStrideInByte /= dgInt32 (sizeof (dgFloat32));
	normalStrideInByte /= dgInt32 (sizeof (dgFloat32));
	for (dgInt32 i = 0; i < m_vertexCount; i ++)	{
		dgInt32 j = i * vertexStrideInByte;
		vertex[j + 0] = m_vertex[i * 3 + 0];
		vertex[j + 1] = m_vertex[i * 3 + 1];
		vertex[j + 2] = m_vertex[i * 3 + 2];

		j = i * normalStrideInByte;
		normal[j + 0] = m_normal[i * 3 + 0];
		normal[j + 1] = m_normal[i * 3 + 1];
		normal[j + 2] = m_normal[i * 3 + 2];

		j = i * uvStrideInByte;
		uv[j + 0] = m_uv[i * 2 + 0];
		uv[j + 1] = m_uv[i * 2 + 1];
	}
}



dgCollisionCompoundBreakable::dgSubMesh::dgSubMesh (dgMemoryAllocator* const allocator)
{
	m_material = 0;
	m_faceCount = 0;
	m_indexes = NULL;
	m_faceOffset = 0;
	m_visibleFaces = 1;
	m_allocator = allocator;
}

dgCollisionCompoundBreakable::dgSubMesh::~dgSubMesh ()
{
	if (m_indexes) {
//		m_allocator->Free (m_visibilityMap);
		m_allocator->Free (m_indexes);
	}
}

void dgCollisionCompoundBreakable::dgSubMesh::Serialize(dgSerialize callback, void* const userData) const
{
	callback (userData, &m_material, dgInt32 (sizeof (dgInt32)));
	callback (userData, &m_faceCount, dgInt32 (sizeof (dgInt32)));
	callback (userData, &m_faceOffset, dgInt32 (sizeof (dgInt32)));
	callback (userData, &m_visibleFaces, dgInt32 (sizeof (dgInt32)));
	callback (userData, m_indexes, size_t (3 * m_faceCount * dgInt32 (sizeof (dgInt32))));
}

dgCollisionCompoundBreakable::dgMesh::dgMesh(dgMemoryAllocator* const allocator)
	:dgList<dgSubMesh>(allocator)
{
	m_IsVisible = 1;
}

dgCollisionCompoundBreakable::dgMesh::~dgMesh()
{
}

dgCollisionCompoundBreakable::dgMesh::dgMesh (dgMemoryAllocator* const allocator, dgDeserialize callback, void* const userData)
	:dgList<dgSubMesh>(allocator), dgRefCounter ()
{
	dgInt32 count;

	callback (userData, &m_IsVisible, dgInt32 (sizeof (dgInt32)));
	callback (userData, &count, dgInt32 (sizeof (dgInt32)));
	for (dgInt32 i = 0; i < count; i ++) {
		dgInt32 material;
		dgInt32 faceCount;
		dgInt32 faceOffset;
		dgInt32 exteriorFaces;

		callback (userData, &material, dgInt32 (sizeof (dgInt32)));
		callback (userData, &faceCount, dgInt32 (sizeof (dgInt32)));
		callback (userData, &faceOffset, dgInt32 (sizeof (dgInt32)));
		callback (userData, &exteriorFaces, dgInt32 (sizeof (dgInt32)));
		dgSubMesh* const subMesh = AddgSubMesh (faceCount * 3, material);

		subMesh->m_faceOffset = faceOffset;
		subMesh->m_visibleFaces = exteriorFaces;

//		callback (userData, subMesh->m_visibilityMap, faceCount * dgInt32 (sizeof (dgInt32)));
		callback (userData, subMesh->m_indexes, size_t (3 * faceCount * sizeof (dgInt32)));
	}
}

void dgCollisionCompoundBreakable::dgMesh::Serialize(dgSerialize callback, void* const userData) const
{
	dgInt32 count;
	count = GetCount();
	
	callback (userData, &m_IsVisible, dgInt32 (sizeof (dgInt32)));
	callback (userData, &count, dgInt32 (sizeof (dgInt32)));
	for (dgListNode* node = GetFirst(); node; node = node->GetNext()) {
		dgSubMesh& subMesh = node->GetInfo();
		subMesh.Serialize(callback, userData);
	}
}

dgCollisionCompoundBreakable::dgSubMesh* dgCollisionCompoundBreakable::dgMesh::AddgSubMesh(dgInt32 indexCount, dgInt32 material)
{
	dgSubMesh tmp (GetAllocator());
	dgSubMesh& subMesh = Append(tmp)->GetInfo();

	subMesh.m_faceOffset = 0;
	subMesh.m_visibleFaces = 1;

	subMesh.m_material = material;
	subMesh.m_faceCount = indexCount / 3;

	subMesh.m_indexes = (dgInt32 *) subMesh.m_allocator->Malloc (indexCount * dgInt32 (sizeof (dgInt32))); 
	return &subMesh;
}



dgCollisionCompoundBreakable::dgDebriNodeInfo::dgDebriNodeInfo ()
{
	m_mesh = NULL;
	m_shape = NULL;
	memset (&m_commonData, 0, sizeof (m_commonData));
}

dgCollisionCompoundBreakable::dgDebriNodeInfo::~dgDebriNodeInfo ()
{
	if (m_shape) {
		m_shape->Release();
	}
	
	if (m_mesh) {
		m_mesh->Release();
	}
}

dgCollisionCompoundBreakable::dgSharedNodeMesh::dgSharedNodeMesh ()
{
}

dgCollisionCompoundBreakable::dgSharedNodeMesh::~dgSharedNodeMesh ()
{
}


dgCollisionCompoundBreakable::dgDebriGraph::dgDebriGraph (dgMemoryAllocator* const allocator)
	:dgGraph<dgDebriNodeInfo, dgSharedNodeMesh>(allocator)
{
}


dgCollisionCompoundBreakable::dgDebriGraph::dgDebriGraph (dgMemoryAllocator* const allocator, dgDeserialize callback, void* const userData)
	:dgGraph<dgDebriNodeInfo, dgSharedNodeMesh>(allocator)
{
	dgInt32 count;
	dgGraph<dgDebriNodeInfo, dgSharedNodeMesh>::dgListNode* node;

	callback (userData, &count, dgInt32 (sizeof (dgInt32)));
	dgStack<dgListNode*> nodesMap(count);

	node = dgGraph<dgDebriNodeInfo, dgSharedNodeMesh>::AddNode();
	dgDebriNodeInfo& data = GetFirst()->GetInfo().m_nodeData;
	callback (userData, &data.m_commonData, sizeof (data.m_commonData));

	nodesMap[0] = node;
	for (dgInt32 i = 1; i < count; i ++) {
		dgGraph<dgDebriNodeInfo, dgSharedNodeMesh>::dgListNode* node;

		node = dgGraph<dgDebriNodeInfo, dgSharedNodeMesh>::AddNode();
		dgDebriNodeInfo& data = node->GetInfo().m_nodeData;
		callback (userData, &data.m_commonData, sizeof (data.m_commonData));
		data.m_mesh = new (GetAllocator()) dgMesh (GetAllocator(), callback, userData);
		nodesMap[i] = node;
	}

	for (dgInt32 i = 0; i < count - 1; i ++) {
		dgInt32 edges;
		callback (userData, &edges, dgInt32 (sizeof (dgInt32)));
		dgStack<dgInt32> pool(edges);
		callback (userData, &pool[0], size_t (edges * dgInt32 (sizeof (dgInt32))));
		for (dgInt32 j = 0; j < edges; j ++) {
			nodesMap[i]->GetInfo().AddEdge (nodesMap[pool[j]]);
		}
	}
}



dgCollisionCompoundBreakable::dgDebriGraph::dgDebriGraph (const dgDebriGraph& source)
	:dgGraph<dgDebriNodeInfo, dgSharedNodeMesh>(source.GetAllocator())
{
	dgGraph<dgDebriNodeInfo, dgSharedNodeMesh>::dgListNode* newNode;
	dgTree<dgListNode*,dgListNode*> filter(GetAllocator());   

	newNode = dgGraph<dgDebriNodeInfo, dgSharedNodeMesh>::AddNode();
	dgDebriNodeInfo& data = newNode->GetInfo().m_nodeData;
	dgDebriNodeInfo& srcData = source.GetFirst()->GetInfo().m_nodeData;

	data.m_commonData = srcData.m_commonData;

	filter.Insert(newNode, source.GetFirst());
	for (dgGraph<dgDebriNodeInfo, dgSharedNodeMesh>::dgListNode* node = source.GetFirst()->GetNext(); node; node = node->GetNext()) {
		newNode = dgGraph<dgDebriNodeInfo, dgSharedNodeMesh>::AddNode();

		dgDebriNodeInfo& srcData = node->GetInfo().m_nodeData;
		dgDebriNodeInfo& data = newNode->GetInfo().m_nodeData;

		data.m_commonData = srcData.m_commonData;
		data.m_mesh = srcData.m_mesh;
		data.m_mesh->AddRef();

		filter.Insert(newNode, node);
	}

	for (dgListNode* node = source.GetFirst(); node; node = node->GetNext()) {
		dgListNode* myNode = filter.Find(node)->GetInfo();
		for (dgGraphNode<dgDebriNodeInfo, dgSharedNodeMesh>::dgListNode* edgeNode = node->GetInfo().GetFirst(); edgeNode; edgeNode = edgeNode->GetNext()) {
			dgListNode* otherNode;
			otherNode = filter.Find(edgeNode->GetInfo().m_node)->GetInfo();
			myNode->GetInfo().AddEdge (otherNode);
		}
	}
}





dgCollisionCompoundBreakable::dgDebriGraph::~dgDebriGraph ()
{
}



void dgCollisionCompoundBreakable::dgDebriGraph::Serialize(dgSerialize callback, void* const userData) const
{
	dgInt32 lru;
	dgInt32 index;
	dgInt32 count;
	dgTree<dgInt32,dgListNode*> enumerator(GetAllocator());   

	lru = 0;
	index = 1;
	count = GetCount();
	callback (userData, &count, dgInt32 (sizeof (dgInt32)));
	dgDebriNodeInfo& data = GetFirst()->GetInfo().m_nodeData;

	dgDebriNodeInfo::PackedSaveData packedData (data.m_commonData);
	packedData.m_lru = 0;
	callback (userData, &packedData, sizeof (packedData));
	enumerator.Insert(0, GetFirst());

	for (dgListNode* node = GetFirst()->GetNext(); node; node = node->GetNext()) {
		dgDebriNodeInfo& data = node->GetInfo().m_nodeData;
		dgDebriNodeInfo::PackedSaveData packedData (data.m_commonData);
		packedData.m_lru = 0;
		callback (userData, &packedData, sizeof (packedData));

		data.m_mesh->Serialize(callback, userData);
		enumerator.Insert(index, node);
		index ++;
	}
	
	for (dgListNode* node = GetFirst(); node != GetLast(); node = node->GetNext()) {
		dgInt32 count;

		index = 0;
		count = node->GetInfo().GetCount();
		callback (userData, &count, dgInt32 (sizeof (dgInt32)));
		dgStack<dgInt32> buffer(count);
		dgInt32* const pool = &buffer[0];
		for (dgGraphNode<dgDebriNodeInfo, dgSharedNodeMesh>::dgListNode* edgeNode = node->GetInfo().GetFirst(); edgeNode; edgeNode = edgeNode->GetNext()) {
			pool[index] = enumerator.Find(edgeNode->GetInfo().m_node)->GetInfo();
			index ++;
		}
		callback (userData, &pool[0], size_t (count * sizeof (dgInt32)));
	}
}


void dgCollisionCompoundBreakable::dgDebriGraph::AddNode (
	dgFlatVertexArray& flatArray, 
	dgMeshEffect* solid, 
	dgInt32 clipperMaterial, 
	dgInt32 id,
	dgFloat32 density,
	dgFloat32 padding)
{
	dgAssert (0);
/*
	dgCollision* collision;

	collision =  solid->CreateConvexCollision(dgFloat32 (0.005f), id, padding, dgGetIdentityMatrix());
	dgAssert (collision);
	if (collision) {
		dgGraph<dgDebriNodeInfo, dgSharedNodeMesh>::dgListNode* node;
		dgInt32 vertexCount;
		dgInt32 baseVertexCount;
		dgCollision* collisionInstance;
		dgMeshEffect::dgIndexArray* geometryHandle;

		node =  dgGraph<dgDebriNodeInfo, dgSharedNodeMesh>::AddNode ();
		dgDebriNodeInfo& data = node->GetInfo().m_nodeData;
		
		collisionInstance = new (GetAllocator()) dgCollisionConvexIntance ((dgCollisionConvex*) collision, node, density);
		collision->Release();

		data.m_mesh = new (GetAllocator()) dgMesh(GetAllocator());
		data.m_shape = (dgCollisionConvex*)collisionInstance;

		vertexCount = solid->GetPropertiesCount() * 6;
		dgStack<dgVector>vertex (vertexCount);
		dgStack<dgVector>normal (vertexCount);
		dgStack<dgVector>uv0 (vertexCount);
		dgStack<dgVector>uv1 (vertexCount);

		solid->GetVertexStreams (sizeof (dgVector), &vertex[0].m_x, sizeof (dgVector), &normal[0].m_x, sizeof (dgVector), &uv0[0].m_x, sizeof (dgVector), &uv1[0].m_x);
		
		// extract the materials index array for mesh
		geometryHandle = solid->MaterialGeomteryBegin();

		data.m_mesh->m_IsVisible = 0;
		vertexCount = flatArray.m_count;
		baseVertexCount = flatArray.m_count;
		for (dgInt32 handle = solid->GetFirstMaterial (geometryHandle); handle != -1; handle = solid->GetNextMaterial (geometryHandle, handle)) {
			dgInt32 isVisible;
			dgInt32 material; 
			dgInt32 indexCount; 
			dgSubMesh* segment;

//			isVisible = 1;
			material = solid->GetMaterialID (geometryHandle, handle);
//			if (material == clipperMaterial) {
//				isVisible = 0;
//			}
			isVisible = (material != clipperMaterial) ? 1 : 0;
			data.m_mesh->m_IsVisible |= isVisible;

			indexCount = solid->GetMaterialIndexCount (geometryHandle, handle);
			segment = data.m_mesh->AddgSubMesh(indexCount, material);
			segment->m_visibleFaces = isVisible;

			solid->GetMaterialGetIndexStream (geometryHandle, handle, segment->m_indexes);
			for (dgInt32 i = 0; i < indexCount; i ++) {
				dgInt32 j;
				j = segment->m_indexes[i];
				flatArray[vertexCount].m_point[0] = vertex[j].m_x;
				flatArray[vertexCount].m_point[1] = vertex[j].m_y;
				flatArray[vertexCount].m_point[2] = vertex[j].m_z;
				flatArray[vertexCount].m_point[3] = normal[j].m_x;
				flatArray[vertexCount].m_point[4] = normal[j].m_y;
				flatArray[vertexCount].m_point[5] = normal[j].m_z;
				flatArray[vertexCount].m_point[6] = uv0[j].m_x;
				flatArray[vertexCount].m_point[7] = uv0[j].m_y;
				flatArray[vertexCount].m_point[8] = uv1[j].m_x;
				flatArray[vertexCount].m_point[9] = uv1[j].m_y;
				segment->m_indexes[i] = vertexCount - baseVertexCount; 
				vertexCount ++;
		//		dgAssert ((vertexCount - baseVertexCount) < dgInt32 (indexBuffer.GetElementsCount()));
			}
		}
		solid->MaterialGeomteryEnd(geometryHandle);

		dgStack<dgInt32> indexBuffer (vertexCount - baseVertexCount);
		flatArray.m_count += dgVertexListToIndexList (&flatArray[baseVertexCount].m_point[0], sizeof (dgFlatVertex), sizeof (dgFlatVertex), 0, vertexCount - baseVertexCount, &indexBuffer[0], dgFloat32 (1.0e-6f));

		for (dgMesh::dgListNode* meshSgement = data.m_mesh->GetFirst(); meshSgement; meshSgement = meshSgement->GetNext()) {
			dgSubMesh* const subMesh = &meshSgement->GetInfo();
			for (dgInt32 i = 0; i < subMesh->m_faceCount * 3; i ++) {
				dgInt32 j;
				j = subMesh->m_indexes[i];  
				subMesh->m_indexes[i] = baseVertexCount + indexBuffer[j];
			}
		}
	}
*/
}


void dgCollisionCompoundBreakable::dgDebriGraph::AddMeshes (
	dgFlatVertexArray& vertexArray, 
	dgInt32 count, 
	const dgMeshEffect* const solidArray[], 
	const dgInt32* const idArray,
	const dgFloat32* const densities,
	const dgInt32* const internalFaceMaterial,
	dgFloat32 padding)
{
//padding = 0.0f;

	dgGraph<dgDebriNodeInfo, dgSharedNodeMesh>::dgListNode* fixNode;

	fixNode =  dgGraph<dgDebriNodeInfo, dgSharedNodeMesh>::AddNode ();
	for (dgInt32 i = 0; i < count; i ++) {
		AddNode(vertexArray, (dgMeshEffect*) solidArray[i], internalFaceMaterial[i], idArray[i], densities[i], padding);
	}
}





dgCollisionCompoundBreakable::dgCollisionCompoundBreakable (const dgCollisionCompoundBreakable& source)
	:dgCollisionCompound(source), m_conectivity(source.m_conectivity), m_detachedIslands(source.m_conectivity.GetAllocator())
{
	dgAssert (0);
/*
	dgInt32 stack;
	dgNodeBase* stackPool[DG_COMPOUND_STACK_DEPTH];
	dgTree<dgCompoundBreakableFilterData, dgCollisionConvex*> graphNodeMap(m_allocator);

	m_lru = source.m_lru;
	m_lastIslandColor = source.m_lastIslandColor;

	m_visibilityMapIndexCount = source.m_visibilityMapIndexCount;
	m_visibilityMap = (dgInt8 *) m_allocator->Malloc (dgInt32 (source.m_visibilityMapIndexCount * sizeof (dgInt8)));
	memcpy (m_visibilityMap, source.m_visibilityMap, size_t (source.m_visibilityMapIndexCount * sizeof (dgInt8)));

	m_visibilityInderectMap = (dgInt32 *) m_allocator->Malloc (source.m_visibilityMapIndexCount * dgInt32 (sizeof (dgInt32)));
	memcpy (m_visibilityInderectMap, source.m_visibilityInderectMap, size_t (source.m_visibilityMapIndexCount * dgInt32 (sizeof (dgInt32))));
	 

	m_vertexBuffer = source.m_vertexBuffer;
	m_vertexBuffer->AddRef();
	m_collisionId = m_compoundBreakable;
	m_rtti |= dgCollisionCompoundBreakable_RTTI;

	stack = 0;
	dgDebriGraph::dgListNode* myNode = m_conectivity.GetFirst()->GetNext();
	for (dgDebriGraph::dgListNode* node = source.m_conectivity.GetFirst()->GetNext(); node != source.m_conectivity.GetLast(); myNode = myNode->GetNext(), node = node->GetNext() ) {
		 dgCompoundBreakableFilterData info;
		 dgDebriNodeInfo& nodeInfo = node->GetInfo().m_nodeData;
		 		 
		 info.m_index = stack;
		 info.m_node = myNode;
		 graphNodeMap.Insert (info, nodeInfo.m_shape);
		dgAssert (m_array[stack] == nodeInfo.m_shape);
		stack ++;
	}


	stack = 1;
	stackPool[0] = m_root;
	while (stack) {
		dgNodeBase *me;

		stack --;
		me = stackPool[stack];

		if (me->m_type == m_leaf) {
			dgInt32 index;
			dgCollisionConvexIntance* newShape;

			dgCompoundBreakableFilterData& info = graphNodeMap.Find(me->m_shape)->GetInfo();
			index = info.m_index;
			dgAssert (m_array[index] == me->m_shape);
			dgAssert (!info.m_node->GetInfo().m_nodeData.m_shape);

//			newShape = new dgCollisionConvexIntance (((dgCollisionConvexIntance*) me->m_shape)->m_myShape, info.m_node, 0);
			newShape = new (m_allocator) dgCollisionConvexIntance (*((dgCollisionConvexIntance*) me->m_shape), info.m_node);
			info.m_node->GetInfo().m_nodeData.m_shape = newShape;

			me->m_shape->Release();
			me->m_shape = newShape;
			newShape->AddRef();

			m_array[index]->Release();
			m_array[index] = newShape;
			newShape->AddRef();

		} else {
			dgAssert (me->m_type == m_node);
			stackPool[stack] = me->m_left;
			stack++;

			stackPool[stack] = me->m_right;
			stack++;
		}
	}

	LinkNodes ();
*/
}


dgCollisionCompoundBreakable::dgCollisionCompoundBreakable (dgWorld* const world, dgDeserialize deserialization, void* const userData)
	:dgCollisionCompound (world, deserialization, userData), m_conectivity(world->GetAllocator(), deserialization, userData),
	 m_detachedIslands(world->GetAllocator())
{
	dgAssert (0);
/*
	dgInt32 stack;
	m_collisionId = m_compoundBreakable;
	m_rtti |= dgCollisionCompoundBreakable_RTTI;

	deserialization (userData, &m_lru, dgInt32 (sizeof (dgInt32)));
	deserialization (userData, &m_lastIslandColor, dgInt32 (sizeof (dgInt32)));
	deserialization (userData, &m_visibilityMapIndexCount, dgInt32 (sizeof (dgInt32)));

	m_visibilityMap = (dgInt8 *) m_allocator->Malloc (dgInt32 (m_visibilityMapIndexCount * sizeof (dgInt8)));
	deserialization (userData, m_visibilityMap, m_visibilityMapIndexCount * sizeof (dgInt8));

	m_visibilityInderectMap = (dgInt32 *) m_allocator->Malloc (m_visibilityMapIndexCount * dgInt32 (sizeof (dgInt32)));
	deserialization (userData, m_visibilityInderectMap, size_t (m_visibilityMapIndexCount * dgInt32 (sizeof (dgInt32))));

	m_vertexBuffer = new (m_allocator) dgVertexBuffer (m_allocator, deserialization, userData);

	stack = 0;
	for (dgDebriGraph::dgListNode* node = m_conectivity.GetFirst()->GetNext(); node != m_conectivity.GetLast(); node = node->GetNext() ) {
		dgCollisionConvexIntance* instance;
		instance = (dgCollisionConvexIntance*) m_array[stack];
		node->GetInfo().m_nodeData.m_shape = instance;
		instance->AddRef();
		instance->m_graphNode = node;
		stack ++;
	}

	LinkNodes ();
*/
}


void dgCollisionCompoundBreakable::LinkNodes ()
{
	dgAssert (0);
/*
	dgInt32 stack;
	dgNodeBase* pool[DG_COMPOUND_STACK_DEPTH];

#ifdef _DEBUG
	dgInt32 count = 0;
#endif

	pool[0] = m_root;
	stack = 1;
	while (stack) {
		dgNodeBase* node;

		stack --;
		node = pool[stack];
		if (node->m_type == m_leaf) {
			dgCollisionConvexIntance* shape;
			shape = (dgCollisionConvexIntance*) node->m_shape;
			shape->m_treeNode = node;

			#ifdef _DEBUG
			shape->m_ordinal = count;
			count ++;
			#endif
		} else {
			dgAssert (node->m_type == m_node);
		//	if (node->m_type == m_node) {
			pool[stack] = node->m_right;
			stack ++;

			pool[stack] = node->m_left;
			stack ++;
		} 
	}
*/
}


void dgCollisionCompoundBreakable::Serialize(dgSerialize callback, void* const userData) const
{
	dgInt32 lru;
	dgCollisionCompound::Serialize(callback, userData);

	m_conectivity.Serialize(callback, userData);

	lru = 0;
	callback (userData, &lru, dgInt32 (sizeof (dgInt32)));
	callback (userData, &m_lastIslandColor, dgInt32 (sizeof (dgInt32)));
	callback (userData, &m_visibilityMapIndexCount, dgInt32 (sizeof (dgInt32)));
	callback (userData, m_visibilityMap, m_visibilityMapIndexCount * sizeof (dgInt8));
	callback (userData, m_visibilityInderectMap, size_t (m_visibilityMapIndexCount * dgInt32 (sizeof (dgInt32))));

	m_vertexBuffer->Serialize(callback, userData);
}

void dgCollisionCompoundBreakable::GetCollisionInfo(dgCollisionInfo* const info) const
{
	dgCollisionCompound::GetCollisionInfo (info);
	info->m_collisionType = m_compoundBreakable;
}





dgInt32 dgCollisionCompoundBreakable::GetSegmentIndexStreamShort (dgDebriGraph::dgListNode* const node, dgMesh::dgListNode* subMeshNode, dgInt16* const index) const
{
	dgInt32 currentIndex;
	dgSubMesh* const subMesh = &subMeshNode->GetInfo();
	const dgInt32* const indexes = subMesh->m_indexes;	

	currentIndex = 0;
	if (node == m_conectivity.GetLast()) {
		dgInt32 faceCount;
		
		const dgInt8* const visbilityMap = m_visibilityMap;
		const dgInt32* const visibilityInderectMap = &m_visibilityInderectMap[subMesh->m_faceOffset];

		faceCount = subMesh->m_faceCount;
		for (dgInt32 i = 0; i < faceCount; i ++) {
			if (visbilityMap[visibilityInderectMap[i]]) {
				index[currentIndex + 0] = dgInt16 (indexes[i * 3 + 0]);
				index[currentIndex + 1] = dgInt16 (indexes[i * 3 + 1]);
				index[currentIndex + 2] = dgInt16 (indexes[i * 3 + 2]);
				currentIndex += 3;
			}
		}

	} else {
		dgInt32 indexCount;
		indexCount = subMesh->m_faceCount * 3;
		for (dgInt32 i = 0; i < indexCount; i ++) {
			index[i] = dgInt16 (indexes[i]);
		}
		currentIndex = indexCount;
	}

	return currentIndex;
}

dgInt32 dgCollisionCompoundBreakable::GetSegmentIndexStream (dgDebriGraph::dgListNode* const node, dgMesh::dgListNode* subMeshNode, dgInt32* const index) const
{
	dgInt32 currentIndex;
	dgSubMesh* const subMesh = &subMeshNode->GetInfo();
	const dgInt32* const indexes = subMesh->m_indexes;

	currentIndex = 0;
	if (node == m_conectivity.GetLast()) {
		dgInt32 faceCount;

		const dgInt8* const visbilityMap = m_visibilityMap;
		const dgInt32* const visibilityInderectMap = &m_visibilityInderectMap[subMesh->m_faceOffset];

		faceCount = subMesh->m_faceCount;
		for (dgInt32 i = 0; i < faceCount; i ++) {
			if (visbilityMap[visibilityInderectMap[i]]) {
				index[currentIndex + 0] = indexes[i * 3 + 0];
				index[currentIndex + 1] = indexes[i * 3 + 1];
				index[currentIndex + 2] = indexes[i * 3 + 2];
				currentIndex += 3;
			}
		}

	} else {
		dgInt32 indexCount;
		indexCount = subMesh->m_faceCount * 3;
		for (dgInt32 i = 0; i < indexCount; i ++) {
			index[i] = indexes[i];
		}
		currentIndex = indexCount;
	}
	return currentIndex;
}








dgInt32 dgCollisionCompoundBreakable::GetSegmentsInRadius (const dgVector& origin, dgFloat32 radius, dgDebriGraph::dgListNode** segments, dgInt32 maxCount)
{
	dgAssert (0);
	return 0;
/*
	dgInt32 count;
	dgInt32 stack;
	dgNodeBase* stackPool[DG_COMPOUND_STACK_DEPTH];

	dgTriplex& p = *((dgTriplex*) &origin.m_x);
	dgVector p0 (origin - dgVector (radius, radius, radius, dgFloat32 (0.0f)));
	dgVector p1 (origin + dgVector (radius, radius, radius, dgFloat32 (0.0f)));

	count = 0;
	stack = 1;
	stackPool[0] = m_root;


	while (stack) {
		const dgNodeBase *me;

		stack --;
		me = stackPool[stack];
		dgAssert (me);

		if (dgOverlapTest (me->m_p0, me->m_p1, p0, p1)) {

			if (me->m_type == m_leaf) {
				
				dgCollisionConvexIntance* const shape = (dgCollisionConvexIntance*) me->m_shape;
				dgDebriGraph::dgListNode* const node = shape->m_graphNode;

				if (node->GetInfo().m_nodeData.m_mesh->m_IsVisible) {
					dgTriplex normal;
					dgTriplex contact; 
					m_world->ClosestPoint (p, shape, dgGetIdentityMatrix(), contact, normal, 0);
					dgVector q (contact.m_x, contact.m_y, contact.m_z, dgFloat32 (0.0f));
					dgVector dist (q - origin);
					if ((dist % dist) < radius * radius) {
						segments[count] = node;
						count ++;
						if (count >= maxCount) {
							break;
						}
					}
				}
			} else {
				dgAssert (me->m_type == m_node);
				stackPool[stack] = me->m_left;
				stack++;
				dgAssert (stack < dgInt32 (sizeof (stackPool) / sizeof (dgNodeBase*)));

				stackPool[stack] = me->m_right;
				stack++;
				dgAssert (stack < dgInt32 (sizeof (stackPool) / sizeof (dgNodeBase*)));
			}
		}
	}


//segments[0] = segments[2];
//count = 1;

	return count;
*/
}


/*
dgInt32 dgCollisionCompoundBreakable::GetDetachedPieces (dgCollision** shapes, dgInt32 maxCount)
{

}
*/

void dgCollisionCompoundBreakable::EnumerateIslands ()
{
//	dgInt32 islandId;

	m_lastIslandColor = 0;
//	m_island.RemoveAll();
	for (dgDebriGraph::dgListNode* node = m_conectivity.GetFirst(); node != m_conectivity.GetLast(); node = node->GetNext()) {
		node->GetInfo().m_nodeData.m_commonData.m_islandIndex = -1;
	}
	for (dgDebriGraph::dgListNode* node = m_conectivity.GetFirst(); node != m_conectivity.GetLast(); node = node->GetNext()) {
		dgDebriNodeInfo& data = node->GetInfo().m_nodeData;
		data.m_commonData.m_distanceToFixNode = DG_DYNAMINIC_ISLAND_COST;
		if (data.m_commonData.m_islandIndex == -1) {
			dgInt32 count;	
			dgDebriGraph::dgListNode* stack[1024 * 4];

			count = 1;
			stack[0] = node;
			node->GetInfo().m_nodeData.m_commonData.m_islandIndex = m_lastIslandColor;
			while (count) {
				dgDebriGraph::dgListNode* rootNode;

				count --;
				rootNode = stack[count];
				for (dgGraphNode<dgDebriNodeInfo, dgSharedNodeMesh>::dgListNode* edgeNode = rootNode->GetInfo().GetFirst(); edgeNode; edgeNode = edgeNode->GetNext()) {
					dgDebriGraph::dgListNode* childNode;
					childNode = edgeNode->GetInfo().m_node;
					if (childNode->GetInfo().m_nodeData.m_commonData.m_islandIndex != m_lastIslandColor) {
						childNode->GetInfo().m_nodeData.m_commonData.m_islandIndex = m_lastIslandColor;
						stack[count] = childNode;
						count ++;
					}
				}
			}
			m_lastIslandColor ++;
		}
	}

	m_conectivity.GetFirst()->GetInfo().m_nodeData.m_commonData.m_distanceToFixNode = 0;
}


void dgCollisionCompoundBreakable::ResetAnchor ()
{
	dgDebriGraph::dgListNode* fixNode;
	dgGraphNode<dgDebriNodeInfo, dgSharedNodeMesh>::dgListNode* nextEdge;
	
	fixNode = m_conectivity.GetFirst();
	for (dgGraphNode<dgDebriNodeInfo, dgSharedNodeMesh>::dgListNode* edgeNode = fixNode->GetInfo().GetFirst(); edgeNode; edgeNode = nextEdge) {
		nextEdge = edgeNode->GetNext();
		fixNode->GetInfo().DeleteEdge(edgeNode);
	}
	EnumerateIslands ();

}


void dgCollisionCompoundBreakable::SetAnchoredParts (dgInt32 count, const dgMatrix* const matrixArray, const dgCollision** collisionArray)
{
	dgAssert(0);
/*
	dgDebriGraph::dgListNode* fixNode;
	dgMatrix matrix (dgGetIdentityMatrix());

	fixNode = m_conectivity.GetFirst();
	for (dgDebriGraph::dgListNode* node = m_conectivity.GetFirst()->GetNext(); node != m_conectivity.GetLast(); node = node->GetNext()) {
		dgGraphNode<dgDebriNodeInfo, dgSharedNodeMesh>::dgListNode* edgeNode;

		for (edgeNode = fixNode->GetInfo().GetFirst(); edgeNode; edgeNode = edgeNode->GetNext()) {
			if (edgeNode->GetInfo().m_node == fixNode) {
				break;
			}
		}
		if (!edgeNode) {
			dgCollision* shape;
			shape = node->GetInfo().m_nodeData.m_shape;
			for (dgInt32 i = 0; i < count; i ++) {
				dgInt32 contactCount;
				dgFloat32 penetration[16];
				dgTriplex points[16];
				dgTriplex normals[16];
				contactCount = m_world->Collide (shape,matrix, (dgCollision*)collisionArray[i], matrixArray[i], points, normals, penetration, 4, 0);
				if (contactCount) {
					node->GetInfo().AddEdge(fixNode);
					fixNode->GetInfo().AddEdge(node);
					break;
				}
			}
		}
	}
//m_conectivity.Trace();

	EnumerateIslands ();

	if (fixNode->GetInfo().GetCount() > 0) {
		dgInt32 leadingIndex;	
		dgInt32 trealingIndex;	
		dgDebriGraph::dgListNode* queue[1024 * 4];

		leadingIndex = 1;
		trealingIndex = 0;
		queue[0] = fixNode;
		fixNode->GetInfo().m_nodeData.m_commonData.m_distanceToFixNode = 0;
		while (trealingIndex != leadingIndex) {
			dgInt32 cost;
			dgDebriGraph::dgListNode* rootNode;
			
			rootNode = queue[trealingIndex];
			trealingIndex ++;
			if (trealingIndex >= dgInt32 (sizeof (queue)/ sizeof (queue[0]))) {
				trealingIndex = 0;
			}


			cost = rootNode->GetInfo().m_nodeData.m_commonData.m_distanceToFixNode + 1;
			for (dgGraphNode<dgDebriNodeInfo, dgSharedNodeMesh>::dgListNode* edgeNode = rootNode->GetInfo().GetFirst(); edgeNode; edgeNode = edgeNode->GetNext()) {
				dgDebriGraph::dgListNode* childNode;
				childNode = edgeNode->GetInfo().m_node;
				if (childNode->GetInfo().m_nodeData.m_commonData.m_distanceToFixNode > cost) {
					childNode->GetInfo().m_nodeData.m_commonData.m_distanceToFixNode = cost;
					queue[leadingIndex] = childNode;
					leadingIndex ++;
					if (leadingIndex >= dgInt32 (sizeof (queue)/ sizeof (queue[0]))) {
						leadingIndex = 0;
					}
					dgAssert (leadingIndex != trealingIndex);
				}
			}
		}
	}
*/
}


void dgCollisionCompoundBreakable::DeleteComponent (dgDebriGraph::dgListNode* node)
{
	dgAssert (0);
/*
	dgMesh* mesh;
	dgNodeBase* treeNode;
	dgCollisionConvexIntance* shape;

	mesh = node->GetInfo().m_nodeData.m_mesh;
	for (dgMesh::dgListNode* meshSgement = mesh->GetFirst(); meshSgement; meshSgement = meshSgement->GetNext()) {
		dgSubMesh* const subMesh = &meshSgement->GetInfo();
		if (subMesh->m_visibleFaces) {
			memset (&m_visibilityMap[subMesh->m_faceOffset], 0, size_t (subMesh->m_faceCount));
		}
	}

	for (dgGraphNode<dgDebriNodeInfo, dgSharedNodeMesh>::dgListNode* edgeNode = node->GetInfo().GetFirst(); edgeNode; edgeNode = edgeNode->GetNext()) {
		dgDebriGraph::dgListNode* adjecentNode;
		adjecentNode = edgeNode->GetInfo().m_node;

		mesh = adjecentNode->GetInfo().m_nodeData.m_mesh;
		if (mesh) {
			mesh->m_IsVisible = 1;
			for (dgMesh::dgListNode* meshSgement = mesh->GetFirst(); meshSgement; meshSgement = meshSgement->GetNext()) {
				dgSubMesh* const subMesh = &meshSgement->GetInfo();
				if (!subMesh->m_visibleFaces) {
					subMesh->m_visibleFaces = 1;
					memset (&m_visibilityMap[subMesh->m_faceOffset], 1, size_t (subMesh->m_faceCount));
				}
			}
		}
	}

	for (dgGraphNode<dgDebriNodeInfo, dgSharedNodeMesh>::dgListNode* edgeNode = node->GetInfo().GetFirst(); edgeNode; edgeNode = edgeNode->GetNext()) {
		dgDebriGraph::dgListNode* neighborg; 
		neighborg = edgeNode->GetInfo().m_node;
		if (neighborg->GetInfo().m_nodeData.m_commonData.m_lru < m_lru) {
			neighborg->GetInfo().m_nodeData.m_commonData.m_lru = m_lru;
			m_detachedIslands.Append(neighborg);
		}
	}

	shape = (dgCollisionConvexIntance*) node->GetInfo().m_nodeData.m_shape;
	dgAssert (shape->m_graphNode->GetInfo().GetFirst() == node->GetInfo().GetFirst());
	m_conectivity.DeleteNode (node);

	for (dgIsland::dgListNode* islandNode = m_detachedIslands.GetFirst(); islandNode; islandNode = islandNode->GetNext()) {
		if (node == islandNode->GetInfo()) {
			m_detachedIslands.Remove(islandNode);
			break;
		}
	}

	treeNode = shape->m_treeNode;
	((dgCollisionConvexIntance*) m_array[m_count - 1])->m_treeNode->m_id = treeNode->m_id;
	RemoveCollision (treeNode);
*/
}


dgBody* dgCollisionCompoundBreakable::CreateComponentBody (dgDebriGraph::dgListNode* node) const
{
	dgAssert (0);
	return NULL;
/*
	dgFloat32 Ixx;
	dgFloat32 Iyy;
	dgFloat32 Izz;
	dgFloat32 Ixx1;
	dgFloat32 Iyy1;
	dgFloat32 Izz1;
	dgFloat32 mass;
	dgBody* body;
	dgCollisionConvexIntance* shape;

	shape = (dgCollisionConvexIntance*) node->GetInfo().m_nodeData.m_shape;
	dgAssert (shape->m_graphNode->GetInfo().GetFirst() == node->GetInfo().GetFirst());


	body = m_world->CreateBody(shape->m_myShape);
	body->SetCentreOfMass(shape->m_volume);
	
	mass = shape->m_inertia.m_w;
	Ixx = shape->m_inertia.m_x;
	Iyy = shape->m_inertia.m_y;
	Izz = shape->m_inertia.m_z;

dgAssert (0);
mass = 1.0f;
Ixx = shape->m_inertia.m_x/shape->m_inertia.m_w;
Iyy = shape->m_inertia.m_y/shape->m_inertia.m_w;
Izz = shape->m_inertia.m_z/shape->m_inertia.m_w;


	Ixx1 = ClampValue (Ixx, dgFloat32 (0.001f) * mass, dgFloat32 (100.0f) * mass);
	Iyy1 = ClampValue (Iyy, dgFloat32 (0.001f) * mass, dgFloat32 (100.0f) * mass);
	Izz1 = ClampValue (Izz, dgFloat32 (0.001f) * mass, dgFloat32 (100.0f) * mass);
	if (mass < dgFloat32 (1.0e-3f)) {
		mass = DG_INFINITE_MASS * dgFloat32 (1.5f);
	}

	body->SetMassMatrix (mass, Ixx1, Iyy1, Izz1);
	body->SetAparentMassMatrix (dgVector (Ixx, Iyy, Izz, mass));

	return body;
*/
}



void dgCollisionCompoundBreakable::DeleteComponentBegin ()
{
	m_lru ++;
	m_detachedIslands.RemoveAll();

//m_conectivity.Trace();
}


void dgCollisionCompoundBreakable::DeleteComponentEnd ()
{
	dgInt32 baseLru;
	dgIsland::dgListNode* nextIslandNode;

int xxxx = 0;
dgDebriGraph::dgListNode* xxx[1024 * 8];

	m_lru ++;
	baseLru	= m_lru;
	for (dgIsland::dgListNode* islandNode = m_detachedIslands.GetFirst(); islandNode; islandNode = nextIslandNode) {
		dgDebriGraph::dgListNode* node; 

		nextIslandNode = islandNode->GetNext();
		node = islandNode->GetInfo();

		m_lru ++;
		if (node->GetInfo().m_nodeData.m_commonData.m_lru > baseLru) {
			m_detachedIslands.Remove(node);
		} else {
			dgInt32 count;	
			dgInt32 piecesCount;
			dgDebriGraph::dgListNode* stack[1024 * 4];
			dgDebriGraph::dgListNode* newIslandPieces[1024 * 8];

			count = 1;
			stack[0] = node;
			piecesCount = 0;


			node->GetInfo().m_nodeData.m_commonData.m_lru = m_lru;

			while (count) {
				dgInt32 stackOrigin;
				dgDebriGraph::dgListNode* rootNode;

				count --;
				rootNode = stack[count];
				dgAssert (node->GetInfo().m_nodeData.m_commonData.m_lru == m_lru);

				newIslandPieces[piecesCount] = rootNode;
				piecesCount ++;
				if (rootNode->GetInfo().m_nodeData.m_commonData.m_distanceToFixNode == 0) {
					piecesCount = 0;
					m_detachedIslands.Remove(node);
					break;
				}

				stackOrigin = count;
				for (dgGraphNode<dgDebriNodeInfo, dgSharedNodeMesh>::dgListNode* edgeNode = rootNode->GetInfo().GetFirst(); edgeNode; edgeNode = edgeNode->GetNext()) {
					dgInt32 cost;
					dgInt32 index;
					dgDebriGraph::dgListNode* childNode;

					childNode = edgeNode->GetInfo().m_node;
					dgDebriNodeInfo& data = childNode->GetInfo().m_nodeData;

					if (data.m_commonData.m_lru < baseLru) {
						data.m_commonData.m_lru = m_lru;
						cost = data.m_commonData.m_distanceToFixNode;
						for (index = count; (index > stackOrigin) && (stack[index - 1]->GetInfo().m_nodeData.m_commonData.m_distanceToFixNode < cost); index --) {
							stack[index] = stack[index - 1];
						}
						stack[index] = childNode;
						count ++;
					} else if (data.m_commonData.m_lru < m_lru) {
						count = 0;
						piecesCount = 0;
						m_detachedIslands.Remove(node);
						break;
					}
				}
			}

			if (piecesCount) {
				for (dgInt32 i = 0; i < piecesCount; i ++) {
					newIslandPieces[i]->GetInfo().m_nodeData.m_commonData.m_islandIndex = m_lastIslandColor;

xxx[xxxx] = newIslandPieces[i];
xxxx ++;
#ifdef _DEBUG
					for (dgInt32 j = i + 1; j < piecesCount; j ++) {
						dgAssert (newIslandPieces[i] != newIslandPieces[j]);
					}
#endif
				}
				m_lastIslandColor ++;
				piecesCount = 0;
			}
		}
	}

	for (int i = 0; i < xxxx; i ++) {
		DeleteComponent (xxx[i]);
	}


/*
m_conectivity.GetFirst()->GetInfo().Trace();
dgInt32 count;	
dgDebriGraph::dgListNode* stack[1024 * 4];
count = 1;
stack[0] = m_conectivity.GetFirst();
m_conectivity.GetFirst()->GetInfo().m_nodeData.m_commonData.m_lru = m_lru;
while (count) {
	dgDebriGraph::dgListNode* rootNode;

	count --;
	rootNode = stack[count];
	for (dgGraphNode<dgDebriNodeInfo, dgSharedNodeMesh>::dgListNode* edgeNode = rootNode->GetInfo().GetFirst(); edgeNode; edgeNode = edgeNode->GetNext()) {
		dgDebriGraph::dgListNode* childNode;

		childNode = edgeNode->GetInfo().m_node;
		dgDebriNodeInfo& data = childNode->GetInfo().m_nodeData;

		if (data.m_commonData.m_lru != m_lru) {
			data.m_commonData.m_lru = m_lru;
			stack[count] = childNode;
			count ++;
		}
	}
}

for (dgDebriGraph::dgListNode* node = m_conectivity.GetFirst(); node != m_conectivity.GetLast(); node = node->GetNext()) {
//	dgAssert (node->GetInfo().m_nodeData.m_commonData.m_lru == m_lru);
}
*/

	

}


#endif


dgCollisionCompoundBreakable::~dgCollisionCompoundBreakable(void)
{
	dgAssert (0);
/*
	if (m_visibilityMap) {
		m_allocator->Free (m_visibilityMap);
		m_allocator->Free (m_visibilityInderectMap);
	}

	if (m_vertexBuffer) {
		m_vertexBuffer->Release();
	}
*/
}


class dgCollisionCompoundBreakable::dgFractureBuilder: public dgTree<dgMeshEffect*, dgInt32>
{
	public:
    class dgPerimenterEdge
    {
        public:
        dgPerimenterEdge* m_next;
        dgPerimenterEdge* m_prev;
        const dgBigVector* m_vertex;
    };

	class dgFractureConectivity: public dgGraph<int, int>
	{
		public:
		dgFractureConectivity(dgMemoryAllocator* const allocator)
			:dgGraph<int, int> (allocator)
		{
		}
	};

	class dgConvexSolidVertices: public dgList<dgInt32>  
	{
		public:
		dgConvexSolidVertices(dgMemoryAllocator* const allocator)
			:dgList<dgInt32> (allocator)
		{
		}
	};

	class dgConvexSolidArray: public dgTree<dgConvexSolidVertices, dgInt32>  
	{
		public:
		dgConvexSolidArray(dgMemoryAllocator* const allocator)
			:dgTree<dgConvexSolidVertices, dgInt32> (allocator) 
		{
		}
	};


	dgFractureBuilder (dgMemoryAllocator* const allocator, dgMeshEffect* const solidMesh, dgInt32 pointcloudCount, const dgFloat32* const vertexCloud, dgInt32 strideInBytes, int materialId, const dgMatrix& textureProjectionMatrix)
		:dgTree<dgMeshEffect*, dgInt32>(allocator)
		,m_conectivity(allocator)
	{
		dgStack<dgBigVector> buffer(pointcloudCount + 16);
		dgBigVector* const pool = &buffer[0];
		dgFloat64 quantizeFactor = dgFloat64 (16.0f);
		dgFloat64 invQuantizeFactor = dgFloat64 (1.0f) / quantizeFactor;
		dgInt32 stride = strideInBytes / sizeof (dgFloat32); 

		dgBigVector minAABB;
		dgBigVector maxAABB;
		solidMesh->CalculateAABB (minAABB, maxAABB);
		for (dgInt32 i = 0; i < pointcloudCount; i ++) {
			dgFloat64 x = vertexCloud[i * stride + 0];
			dgFloat64 y	= vertexCloud[i * stride + 1];
			dgFloat64 z	= vertexCloud[i * stride + 2];
			x = floor (x * quantizeFactor) * invQuantizeFactor;
			y = floor (y * quantizeFactor) * invQuantizeFactor;
			z = floor (z * quantizeFactor) * invQuantizeFactor;
			pool[i] = dgBigVector (x, y, z, dgFloat64 (0.0f));
		}

		// add the bbox as a barrier
		int count = pointcloudCount;
		pool[count + 0] = dgBigVector ( minAABB.m_x, minAABB.m_y, minAABB.m_z, dgFloat64 (0.0f));
		pool[count + 1] = dgBigVector ( maxAABB.m_x, minAABB.m_y, minAABB.m_z, dgFloat64 (0.0f));
		pool[count + 2] = dgBigVector ( minAABB.m_x, maxAABB.m_y, minAABB.m_z, dgFloat64 (0.0f));
		pool[count + 3] = dgBigVector ( maxAABB.m_x, maxAABB.m_y, minAABB.m_z, dgFloat64 (0.0f));
		pool[count + 4] = dgBigVector ( minAABB.m_x, minAABB.m_y, maxAABB.m_z, dgFloat64 (0.0f));
		pool[count + 5] = dgBigVector ( maxAABB.m_x, minAABB.m_y, maxAABB.m_z, dgFloat64 (0.0f));
		pool[count + 6] = dgBigVector ( minAABB.m_x, maxAABB.m_y, maxAABB.m_z, dgFloat64 (0.0f));
		pool[count + 7] = dgBigVector ( maxAABB.m_x, maxAABB.m_y, maxAABB.m_z, dgFloat64 (0.0f));
		count += 8;

		dgStack<dgInt32> indexList(count);
		count = dgVertexListToIndexList(&pool[0].m_x, sizeof (dgBigVector), 3, count, &indexList[0], dgFloat64 (5.0e-2f));	
		dgAssert (count >= 8);

		dgFloat64 maxSize = dgMax(maxAABB.m_x - minAABB.m_x, maxAABB.m_y - minAABB.m_y, maxAABB.m_z - minAABB.m_z);
		minAABB -= dgBigVector (maxSize, maxSize, maxSize, dgFloat64 (0.0f));
		maxAABB += dgBigVector (maxSize, maxSize, maxSize, dgFloat64 (0.0f));

		// add the a guard zone, so that we do not have to clip
		dgInt32 guadVertexKey = count;
		pool[count + 0] = dgBigVector ( minAABB.m_x, minAABB.m_y, minAABB.m_z, dgFloat64 (0.0f));
		pool[count + 1] = dgBigVector ( maxAABB.m_x, minAABB.m_y, minAABB.m_z, dgFloat64 (0.0f));
		pool[count + 2] = dgBigVector ( minAABB.m_x, maxAABB.m_y, minAABB.m_z, dgFloat64 (0.0f));
		pool[count + 3] = dgBigVector ( maxAABB.m_x, maxAABB.m_y, minAABB.m_z, dgFloat64 (0.0f));
		pool[count + 4] = dgBigVector ( minAABB.m_x, minAABB.m_y, maxAABB.m_z, dgFloat64 (0.0f));
		pool[count + 5] = dgBigVector ( maxAABB.m_x, minAABB.m_y, maxAABB.m_z, dgFloat64 (0.0f));
		pool[count + 6] = dgBigVector ( minAABB.m_x, maxAABB.m_y, maxAABB.m_z, dgFloat64 (0.0f));
		pool[count + 7] = dgBigVector ( maxAABB.m_x, maxAABB.m_y, maxAABB.m_z, dgFloat64 (0.0f));
		count += 8; 

		dgDelaunayTetrahedralization delaunayTetrahedras (allocator, &pool[0].m_x, count, sizeof (dgBigVector), dgFloat32 (0.0f));
		delaunayTetrahedras.RemoveUpperHull ();

		dgInt32 tetraCount = delaunayTetrahedras.GetCount();
		dgStack<dgBigVector> voronoiPoints(tetraCount + 32);
		dgStack<dgDelaunayTetrahedralization::dgListNode*> tetradrumNode(tetraCount);
		dgConvexSolidArray delanayNodes (allocator);	

		dgInt32 index = 0;
		const dgHullVector* const delanayPoints = delaunayTetrahedras.GetHullVertexArray();
		for (dgDelaunayTetrahedralization::dgListNode* node = delaunayTetrahedras.GetFirst(); node; node = node->GetNext()) {
			dgConvexHull4dTetraherum& tetra = node->GetInfo();
			voronoiPoints[index] = tetra.CircumSphereCenter (delanayPoints);
			tetradrumNode[index] = node;

			for (dgInt32 i = 0; i < 4; i ++) {
				dgConvexSolidArray::dgTreeNode* header = delanayNodes.Find(tetra.m_faces[0].m_index[i]);
				if (!header) {
					dgConvexSolidVertices list (allocator);
					header = delanayNodes.Insert(list, tetra.m_faces[0].m_index[i]);
				}
				header->GetInfo().Append (index);
			}
			index ++;
		}

		dgConvexSolidArray::Iterator iter (delanayNodes);
		dgFloat32 normalAngleInRadians = 30.0f * 3.1416f / 180.0f;

		dgTree<dgFractureConectivity::dgListNode*, int> graphMap(allocator);
		for (iter.Begin(); iter; iter ++) {

			dgConvexSolidArray::dgTreeNode* const nodeNode = iter.GetNode();
			const dgList<dgInt32>& list = nodeNode->GetInfo();
			dgInt32 key = nodeNode->GetKey();
			if (key < guadVertexKey) {
				dgBigVector pointArray[512];
				dgInt32 indexArray[512];

				dgInt32 count = 0;
				for (dgList<dgInt32>::dgListNode* ptr = list.GetFirst(); ptr; ptr = ptr->GetNext()) {
					dgInt32 i = ptr->GetInfo();
					pointArray[count] = voronoiPoints[i];
					count ++;
					dgAssert (count < dgInt32 (sizeof (pointArray) / sizeof (pointArray[0])));
				}

				count = dgVertexListToIndexList(&pointArray[0].m_x, sizeof (dgBigVector), 3, count, &indexArray[0], dgFloat64 (1.0e-3f));	
				if (count >= 4) {
					dgMeshEffect* const convexMesh =  new (allocator) dgMeshEffect (allocator, &pointArray[0].m_x, count, sizeof (dgBigVector), dgFloat64 (0.0f));
					if (convexMesh->GetCount()) {
						convexMesh->AddRef();
						Insert (convexMesh, key);

						convexMesh->ConvertToPolygons();
						convexMesh->CalculateNormals(normalAngleInRadians);
						convexMesh->UniformBoxMapping (materialId, textureProjectionMatrix);

						dgInt32 vCount = convexMesh->GetVertexCount();
						for (dgInt32 i = 0; i < vCount; i ++) {
							dgBigVector& point = convexMesh->GetVertex (i);
							point.m_w = key;
						}
						dgInt32 aCount = convexMesh->GetPropertiesCount();
						for (dgInt32 i = 0; i < aCount; i ++) {
							dgMeshEffect::dgVertexAtribute& attib = convexMesh->GetAttribute (i);
							attib.m_vertex.m_w = key;
						}

						dgFractureConectivity::dgListNode* const node = m_conectivity.AddNode ();
						node->GetInfo().m_nodeData = key;
						graphMap.Insert(node, key);
					}
					convexMesh->Release();
				}
			}
		}
		delanayNodes.RemoveAll();

		for (dgDelaunayTetrahedralization::dgListNode* node = delaunayTetrahedras.GetFirst(); node; node = node->GetNext()) {
			dgConvexHull4dTetraherum& tetra = node->GetInfo();
			for (dgInt32 i = 0; i < 3; i ++) {
				dgInt32 nodeindex0 = tetra.m_faces[0].m_index[i];
				if (nodeindex0 < guadVertexKey) {
					for (dgInt32 j = i + 1; j < 4; j ++) {
						dgInt32 nodeindex1 = tetra.m_faces[0].m_index[j];
						if (nodeindex1 < guadVertexKey) {
							dgAssert (graphMap.Find(nodeindex0));
							dgAssert (graphMap.Find(nodeindex1));
							dgFractureConectivity::dgListNode* const node0 = graphMap.Find(nodeindex0)->GetInfo();
							dgFractureConectivity::dgListNode* const node1 = graphMap.Find(nodeindex1)->GetInfo();
							if (!IsPairConnected (node0, node1)) {
								if (AreSolidNeigborg (nodeindex0, nodeindex1)) {
									node0->GetInfo().AddEdge(node1);
									node1->GetInfo().AddEdge(node0);
								}
							}
						}
					}
				}
			}
		}

for (dgFractureConectivity::dgListNode* node = m_conectivity.GetFirst(); node; node = node->GetNext()) {
	dgInt32 index = node->GetInfo().m_nodeData;
	dgTrace (("node %d: ", index));
	for (dgGraphNode<int, int>::dgListNode* edge = node->GetInfo().GetFirst(); edge; edge = edge->GetNext()) {
		dgFractureConectivity::dgListNode* const otherNode = edge->GetInfo().m_node;
		dgInt32 index1 = otherNode->GetInfo().m_nodeData;
		dgTrace (("%d ", index1));
	}
	dgTrace (("\n"));
}


//solidMesh->SaveOFF("xxx0.off");

	}

    ~dgFractureBuilder()
    {
        Iterator iter (*this);
        for (iter.Begin(); iter; iter ++) {
            dgMeshEffect* const mesh = iter.GetNode()->GetInfo();
            mesh->Release();
        }
    }


	bool IsPairConnected (dgFractureConectivity::dgListNode* const nodeA, dgFractureConectivity::dgListNode* const nodeB) const
	{
		for (dgGraphNode<int, int>::dgListNode* edgeNodeAB = nodeA->GetInfo().GetFirst(); edgeNodeAB; edgeNodeAB = edgeNodeAB->GetNext()) {
			dgFractureConectivity::dgListNode* const otherNode = edgeNodeAB->GetInfo().m_node;
			if (otherNode == nodeB)
				return true;
		}
		return false;
	}

	bool ArePlaneCoplanar (dgMeshEffect* const meshA, void* faceA, const dgBigVector& planeA, dgMeshEffect* const meshB, void* faceB, const dgBigVector& planeB) const
	{
		if (((planeA % planeB) < dgFloat64 (-1.0 + 1.0e-6f)) && ((fabs(planeA.m_w + planeB.m_w) < dgFloat64(1.0e-6f)))) {
			const dgBigVector* const pointsA = (dgBigVector*) meshA->GetVertexPool();
			const dgBigVector* const pointsB = (dgBigVector*) meshB->GetVertexPool();

			dgInt32 indexA[128];
			dgInt32 indexB[128];

			dgInt32 indexCountA = meshA->GetFaceIndexCount(faceA);
			dgInt32 indexCountB = meshB->GetFaceIndexCount(faceB);
			dgAssert (indexCountA < sizeof (indexA)/ sizeof(indexA[0]));
			dgAssert (indexCountB < sizeof (indexB)/ sizeof(indexB[0]));

			meshA->GetFaceIndex(faceA, indexA);
			meshA->GetFaceIndex(faceB, indexB);

//dgTrace (("faceA:\n"));
			dgPerimenterEdge subdivision[256];
			dgAssert ((2 * (indexCountA + indexCountB)) < dgInt32 (sizeof (subdivision) / sizeof (subdivision[0])));
			for (dgInt32 i = 0; i < indexCountB; i ++) {
				subdivision[i].m_vertex = &pointsB[indexB[i]];
				subdivision[i].m_prev = &subdivision[i - 1];
				subdivision[i].m_next = &subdivision[i + 1];

//dgTrace (("%f %f %f\n", pointsB[indexB[i]].m_x, pointsB[indexB[i]].m_y, pointsB[indexB[i]].m_z));
			}
			subdivision[0].m_prev = &subdivision[indexCountB - 1];
			subdivision[indexCountB - 1].m_next = &subdivision[0];

            dgInt32 edgeIndex = indexCountB;

			dgBigVector outputPool[128];
			dgPerimenterEdge* edgeClipped[2];
			dgBigVector* output = &outputPool[0];
			edgeClipped[0] = NULL;
			edgeClipped[1] = NULL;

//dgTrace (("faceB:\n"));
			dgPerimenterEdge* poly = &subdivision[0];
			dgInt32 i0 = indexCountA - 1;
			for (dgInt32 i1 = 0; i1 < indexCountA; i1 ++) {
                const dgBigVector& q0 = pointsA[indexA[i0]];
                const dgBigVector& q1 = pointsA[indexA[i1]];
				dgBigVector n (planeA * (q1 - q0));
				dgBigPlane plane (n, - (n % q0));
				i0 = i1;
//dgTrace (("%f %f %f\n", q0.m_x, q0.m_y, q0.m_z));

				dgInt32 count = 0;
				dgPerimenterEdge* tmp = poly;
				dgInt32 isInside = 0;
				dgFloat64 test0 = plane.Evalue (*tmp->m_vertex);
				do {
					dgFloat64 test1 = plane.Evalue (*tmp->m_next->m_vertex);
					if (test0 >= dgFloat32 (0.0f)) {
						isInside |= 1;
						if (test1 < dgFloat32 (0.0f)) {
							const dgBigVector& p0 = *tmp->m_vertex;
							const dgBigVector& p1 = *tmp->m_next->m_vertex;

							dgBigVector dp (p1 - p0); 
							dgFloat64 den = plane % dp;
							if (fabs(den) < dgFloat32 (1.0e-24f)) {
								den = (den >= dgFloat32 (0.0f)) ?  dgFloat32 (1.0e-24f) :  dgFloat32 (-1.0e-24f);
							}

							den = test0 / den;
							if (den >= dgFloat32 (0.0f)) {
								den = dgFloat32 (0.0f);
							} else if (den <= -1.0f) {
								den = dgFloat32 (-1.0f);
							}
							output[0] = p0 - dp.Scale3 (den);
							edgeClipped[0] = tmp;
							count ++;
						}
					} else if (test1 >= dgFloat32 (0.0f)) {
						const dgBigVector& p0 = *tmp->m_vertex;
						const dgBigVector& p1 = *tmp->m_next->m_vertex;
						isInside |= 1;
						dgBigVector dp (p1 - p0); 
						dgFloat64 den = plane % dp;
						if (fabs(den) < dgFloat32 (1.0e-24f)) {
							den = (den >= dgFloat32 (0.0f)) ?  dgFloat32 (1.0e-24f) :  dgFloat32 (-1.0e-24f);
						}
						den = test0 / den;
						if (den >= dgFloat32 (0.0f)) {
							den = dgFloat32 (0.0f);
						} else if (den <= -1.0f) {
							den = dgFloat32 (-1.0f);
						}
						output[1] = p0 - dp.Scale3 (den);
						edgeClipped[1] = tmp;
						count ++;
					}
					test0 = test1;
					tmp = tmp->m_next;
				} while ((tmp != poly) && count < 2);

				if (!isInside) {
					return false;
				}

				if (count == 2) {
					dgPerimenterEdge* const newEdge = &subdivision[edgeIndex];
					newEdge->m_next = edgeClipped[1];
					newEdge->m_prev = edgeClipped[0];
					edgeClipped[0]->m_next = newEdge;
					edgeClipped[1]->m_prev = newEdge;

					newEdge->m_vertex = &output[0];
					edgeClipped[1]->m_vertex = &output[1];
					poly = newEdge;

					output += 2;
					edgeIndex ++;
					dgAssert (edgeIndex < dgInt32 (sizeof (subdivision) / sizeof (subdivision[0])));
				}
			}
//dgTrace (("\n"));
			dgAssert (poly);
            dgBigVector area(dgFloat32 (0.0f));
            dgBigVector r0 (*poly->m_vertex);
            dgBigVector r1 (*poly->m_next->m_vertex);
            dgBigVector r1r0 (r1 - r0);
            dgPerimenterEdge* polyPtr = poly->m_next->m_next;
            do {
                dgBigVector r2 (*polyPtr->m_vertex);
                dgBigVector r2r0 (r2 - r0);
                area += r2r0 * r1r0;
                r1r0 = r2r0;
                polyPtr = polyPtr->m_next;
            } while (polyPtr != poly);
            return fabs (area % planeA) > dgFloat32 (1.0e-5f);
		}

		return false;
	}


    bool AreSolidNeigborg (int indexA, int indexB) const
    {
        dgMeshEffect* const meshA = Find(indexA)->GetInfo();
        dgMeshEffect* const meshB = Find(indexB)->GetInfo();

        const dgBigVector* const pointsA = (dgBigVector*) meshA->GetVertexPool();
        const dgBigVector* const pointsB = (dgBigVector*) meshB->GetVertexPool();

        dgBigVector planeB_array[512];

        dgInt32 planeB_Count = 0;
        for (void* faceB = meshB->GetFirstFace(); faceB; faceB = meshB->GetNextFace(faceB)) {
            dgAssert (!meshB->IsFaceOpen (faceB));
            dgInt32 vertexIndexB = meshB->GetVertexIndex (faceB);
            dgBigVector planeB (meshB->CalculateFaceNormal (faceB));
            planeB.m_w = -(planeB % pointsB[vertexIndexB]);
            planeB_array[planeB_Count] = planeB;
            planeB_Count ++;
            dgAssert (planeB_Count < sizeof (planeB_array) / sizeof (planeB_array[0]));
        }

        for (void* faceA = meshA->GetFirstFace(); faceA; faceA = meshA->GetNextFace(faceA)) {
            dgAssert (!meshA->IsFaceOpen (faceA));
            dgInt32 vertexIndexA = meshA->GetVertexIndex (faceA);
            dgBigVector planeA (meshA->CalculateFaceNormal (faceA));
            planeA.m_w = -(planeA % pointsA[vertexIndexA]);

            dgInt32 indexB = 0;
            for (void* faceB = meshB->GetFirstFace(); faceB; faceB = meshB->GetNextFace(faceB)) {
                if (ArePlaneCoplanar (meshA, faceA, planeA, meshB, faceB, planeB_array[indexB])) {
                    return true;
                }
                indexB ++;
            }
        }
        return false;
    }

	dgFractureConectivity m_conectivity;
};


dgCollisionCompoundBreakable::dgCollisionCompoundBreakable (dgWorld* const world, dgMeshEffect* const solidMesh, int fracturePhysicsMaterialID, int pointcloudCount, const dgFloat32* const vertexCloud, int strideInBytes, int materialID, const dgMatrix& offsetMatrix)
	:dgCollisionCompound (world)
//	dgInt32 count, const dgMeshEffect* const solidArray[],
//	const dgInt32* const idArray, const dgFloat32* const densities, const dgInt32* const internalFaceMaterial,
//	dgInt32 debriiId, dgFloat32 collisionPadding,
//	dgWorld* world)
//	:dgCollisionCompound(world), m_conectivity (world->GetAllocator()), m_detachedIslands(world->GetAllocator())
{
	m_rtti |= dgCollisionCompoundBreakable_RTTI;

//	dgInt32 acc;
//	dgInt32 indexCount;
//	dgInt32 vertsCount;
//	dgInt32 materialHitogram[256];
//	dgInt32 faceOffsetHitogram[256];
//	dgSubMesh* mainSegmenst[256];
//	dgMesh* mainMesh;
//	dgFlatVertexArray vertexArray(m_world->GetAllocator());
 
//	m_lru = 0;
//	m_lastIslandColor = 0;
//	m_visibilityMapIndexCount = 0;
//	m_vertexBuffer = NULL;
//	m_visibilityMap = NULL;
//	m_visibilityInderectMap = NULL;
//	m_collisionId = m_compoundBreakable;
//	m_rtti |= dgCollisionCompoundBreakable_RTTI;
//	if (collisionPadding < dgFloat32 (0.01f)) {
//		collisionPadding = dgFloat32 (0.01f);
//	}
//	m_conectivity.AddMeshes (vertexArray, count, solidArray, idArray, densities, internalFaceMaterial, collisionPadding);

//	NewtonMesh* const debriMeshPieces = NewtonMeshCreateVoronoiConvexDecomposition (m_world, count, &points[0].m_x, sizeof (dVector), interiorMaterial, &textureMatrix[0][0]);
//	dAssert (debriMeshPieces);
//	dgFloat32 normalAngleInRadians = 30.0f * 3.1416f / 180.0f;

pointcloudCount = 0;
	dgFractureBuilder fractureBuilder (GetAllocator(), solidMesh, pointcloudCount, vertexCloud, strideInBytes, materialID, offsetMatrix);
/*
	for (iter.Begin(); iter; iter ++) {
		dgTree<dgList<dgInt32>, dgInt32>::dgTreeNode* const nodeNode = iter.GetNode();
		const dgList<dgInt32>& list = nodeNode->GetInfo();
		dgInt32 key = nodeNode->GetKey();

		if (key < guadVertexKey) {
			dgBigVector pointArray[512];
			dgInt32 indexArray[512];

			dgInt32 count = 0;
			for (dgList<dgInt32>::dgListNode* ptr = list.GetFirst(); ptr; ptr = ptr->GetNext()) {
				dgInt32 i = ptr->GetInfo();
				pointArray[count] = voronoiPoints[i];
				count ++;
				dgAssert (count < dgInt32 (sizeof (pointArray) / sizeof (pointArray[0])));
			}

			count = dgVertexListToIndexList(&pointArray[0].m_x, sizeof (dgBigVector), 3, count, &indexArray[0], dgFloat64 (1.0e-3f));	
			if (count >= 4) {
				dgMeshEffect convexMesh (allocator, &pointArray[0].m_x, count, sizeof (dgBigVector), dgFloat64 (0.0f));
				if (convexMesh.GetCount()) {
					convexMesh.CalculateNormals(normalAngleInRadians);
					convexMesh.UniformBoxMapping (materialId, textureProjectionMatrix);

					for (dgInt32 i = 0; i < convexMesh.m_pointCount; i ++) {
						convexMesh.m_points[i].m_w = layer;
					}
					for (dgInt32 i = 0; i < convexMesh.m_atribCount; i ++) {
						convexMesh.m_attrib[i].m_vertex.m_w = layer;
					}
					voronoiPartition->MergeFaces(&convexMesh);
					layer += dgFloat64 (1.0f);

				}
			}
		}
	}
	voronoiPartition->EndPolygon(dgFloat64 (1.0e-8f), false);

	//	voronoiPartition->SaveOFF("xxx0.off");

	//voronoiPartition->ConvertToPolygons();
	return voronoiPartition;
*/

/*
	dgStack<dgCollisionConvex*> shapeArrayPool (m_conectivity.GetCount());
	dgCollisionConvex** const shapeArray = &shapeArrayPool[0];
	
	indexCount = 0;
	for (dgDebriGraph::dgListNode* node = m_conectivity.GetFirst()->GetNext(); node; node = node->GetNext()) {
		dgDebriNodeInfo& data = node->GetInfo().m_nodeData;
		shapeArray[indexCount] = data.m_shape;
		indexCount ++;
	}

	if (indexCount) {
		m_root = BuildTree(indexCount, &shapeArray[0]);
	}
	Init (indexCount, &shapeArray[0]);

	dgStack<dgInt32> indexBuffer (vertexArray.m_count);
	vertsCount = dgVertexListToIndexList (&vertexArray[0].m_point[0], sizeof (dgFlatVertex), sizeof (dgFlatVertex), 0, vertexArray.m_count, &indexBuffer[0], dgFloat32 (1.0e-6f));

	m_vertexBuffer = new (m_world->GetAllocator()) dgVertexBuffer(vertsCount, m_world->GetAllocator());
	for (dgInt32 i = 0; i < vertsCount; i ++) {
		m_vertexBuffer->m_vertex[i * 3 + 0] = vertexArray[i].m_point[0];
		m_vertexBuffer->m_vertex[i * 3 + 1] = vertexArray[i].m_point[1];
		m_vertexBuffer->m_vertex[i * 3 + 2] = vertexArray[i].m_point[2];
		m_vertexBuffer->m_normal[i * 3 + 0] = vertexArray[i].m_point[3];
		m_vertexBuffer->m_normal[i * 3 + 1] = vertexArray[i].m_point[4];
		m_vertexBuffer->m_normal[i * 3 + 2] = vertexArray[i].m_point[5];
		m_vertexBuffer->m_uv[i * 2 + 0] = vertexArray[i].m_point[6];
		m_vertexBuffer->m_uv[i * 2 + 1] = vertexArray[i].m_point[7];
	}

	memset (materialHitogram, 0, sizeof (materialHitogram));
	memset (faceOffsetHitogram, 0, sizeof (faceOffsetHitogram));
	memset (mainSegmenst, 0, sizeof (mainSegmenst));
	for (dgDebriGraph::dgListNode* node = m_conectivity.GetFirst()->GetNext(); node; node = node->GetNext()) {
		dgDebriNodeInfo& data = node->GetInfo().m_nodeData;
		for (dgMesh::dgListNode* meshSgement = data.m_mesh->GetFirst(); meshSgement; meshSgement = meshSgement->GetNext()) {
			dgInt32 material;
			dgSubMesh* const subMesh = &meshSgement->GetInfo();
			material = subMesh->m_material;
			materialHitogram[material] += subMesh->m_faceCount;
		}
	}

	dgDebriNodeInfo& mainNodeData = m_conectivity.dgGraph<dgDebriNodeInfo, dgSharedNodeMesh>::AddNode()->GetInfo().m_nodeData;

	acc = 0;
	mainMesh = new (m_world->GetAllocator()) dgMesh(m_world->GetAllocator());
	mainNodeData.m_mesh = mainMesh;
	for (dgInt32 i = 0; i < 256; i ++) {
		if (materialHitogram[i]) {
			dgSubMesh* segment;
			segment = mainMesh->AddgSubMesh(materialHitogram[i] * 3, i);
			segment->m_faceOffset = acc;
			segment->m_faceCount = 0;
			mainSegmenst[i] = segment;
		}
		faceOffsetHitogram[i] = acc;
		acc += materialHitogram[i];
	}


	m_visibilityMapIndexCount = acc;
	m_visibilityMap = (dgInt8*) m_allocator->Malloc (dgInt32 (acc * sizeof (dgInt8)));
	m_visibilityInderectMap = (dgInt32*) m_allocator->Malloc (acc * dgInt32 (sizeof (dgInt32)));
	acc = 0;

	for (dgDebriGraph::dgListNode* node = m_conectivity.GetFirst()->GetNext(); node != m_conectivity.GetLast(); node = node->GetNext() ) {
		dgDebriNodeInfo& data = node->GetInfo().m_nodeData;

		for (dgMesh::dgListNode* node = data.m_mesh->GetFirst(); node; node = node->GetNext()) {
			dgInt8 visbility;
			dgInt32 rootIndexCount;

			dgSubMesh& segment = node->GetInfo();
			dgSubMesh& rootSegment = *mainSegmenst[segment.m_material];

			visbility = dgInt8 (segment.m_visibleFaces);

			memset (&m_visibilityMap[acc], visbility, size_t (segment.m_faceCount));
			for (dgInt32 i = 0; i < segment.m_faceCount; i ++) {
				m_visibilityInderectMap[faceOffsetHitogram[segment.m_material] + i] = acc + i;
			}
			faceOffsetHitogram[segment.m_material] += segment.m_faceCount;

			rootIndexCount = rootSegment.m_faceCount * 3;
			for (dgInt32 i = 0; i < segment.m_faceCount * 3; i ++) {
				dgInt32 j;

				j = segment.m_indexes[i];
				segment.m_indexes[i] = indexBuffer[j];
				rootSegment.m_indexes[rootIndexCount] = indexBuffer[j];
				rootIndexCount ++;
			}
			rootSegment.m_faceCount = rootIndexCount / 3;

			segment.m_faceOffset = acc;
//			dgAssert (acc == segment.m_faceOffset);
			acc += segment.m_faceCount;
		}
	}
	LinkNodes ();	

	dgMatrix matrix (dgGetIdentityMatrix());
	for (dgDebriGraph::dgListNode* node = m_conectivity.GetFirst()->GetNext(); node != m_conectivity.GetLast(); node = node->GetNext()) {
		dgInt32 stack;
		dgCollisionConvexIntance* myShape;
		dgNodeBase* pool[DG_COMPOUND_STACK_DEPTH];

		myShape = (dgCollisionConvexIntance*) node->GetInfo().m_nodeData.m_shape;
		pool[0] = m_root;
		stack = 1;

		dgVector p0 (myShape->m_treeNode->m_p0);
		dgVector p1 (myShape->m_treeNode->m_p1);

		p0 -= dgVector (dgFloat32 (1.0e-2f), dgFloat32 (1.0e-2f), dgFloat32 (1.0e-2f), dgFloat32 (0.0f));
		p1 += dgVector (dgFloat32 (1.0e-2f), dgFloat32 (1.0e-2f), dgFloat32 (1.0e-2f), dgFloat32 (0.0f));

		while (stack) {
			dgNodeBase* treeNode;
			stack --;
			treeNode = pool[stack];
			if (dgOverlapTest (p0, p1, treeNode->m_p0, treeNode->m_p1)) {
				if (treeNode->m_type == m_leaf) {
					dgCollisionConvexIntance* otherShape;
					otherShape = (dgCollisionConvexIntance*) treeNode->m_shape;
					if (otherShape->m_graphNode != node) {
						dgGraphNode<dgDebriNodeInfo, dgSharedNodeMesh>::dgListNode* adjacentEdge;
	//					dgAssert (otherShape->m_graphNode != node) 
						for (adjacentEdge = node->GetInfo().GetFirst(); adjacentEdge; adjacentEdge = adjacentEdge->GetNext()) {
							if (adjacentEdge->GetInfo().m_node == otherShape->m_graphNode) {
								break;
							}
						}
						if (!adjacentEdge) {
							dgInt32 disjoint;
							dgTriplex normal;
							dgTriplex contactA; 
							dgTriplex contactB; 
							const dgFloat32 minSeparation = dgFloat32 (2.0f) * collisionPadding + dgFloat32 (0.05f);
							const dgFloat32 minSeparation2 = minSeparation * minSeparation;

							disjoint = world->ClosestPoint (myShape, matrix, otherShape, matrix, contactA, contactB, normal, 0);
							if (disjoint) {
								dgVector qA (contactA.m_x, contactA.m_y, contactA.m_z, dgFloat32 (0.0f));
								dgVector qB (contactB.m_x, contactB.m_y, contactB.m_z, dgFloat32 (0.0f));
								dgVector dist (qB - qA);
								disjoint = (dist % dist) > minSeparation2;
							}

							if (!disjoint) {
								otherShape->m_graphNode->GetInfo().AddEdge(node);
								node->GetInfo().AddEdge(otherShape->m_graphNode);
							}
						}
					}
	
				} else {
					dgAssert (treeNode->m_type == m_node);
					pool[stack] = treeNode->m_right;
					stack ++;
					pool[stack] = treeNode->m_left;
					stack ++;
				}
			}
		}
	}
	
	ResetAnchor ();
//m_conectivity.Trace();
*/
}







