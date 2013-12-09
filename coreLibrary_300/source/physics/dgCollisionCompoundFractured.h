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

#ifndef DG_COLLISION_COMPOUND_FRACTURED_H
#define DG_COLLISION_COMPOUND_FRACTURED_H

#include "dgCollisionCompound.h"


class dgMeshEffect;

class dgCollisionCompoundFractured: public dgCollisionCompound
{
	class dgFractureBuilder;

	public:
	class dgFlatVertex
	{
		public:
		dgFloat32 m_point[10]; // 3 point, 3 normal. 2 uv0, 2 uv1
	};

	class dgFlatVertexArray: public dgArray<dgFlatVertex>
	{
		public:
		dgFlatVertexArray(dgMemoryAllocator* const allocator)
			:dgArray<dgFlatVertex> (1024 * 4, allocator)
		{
			m_count = 0;
		}

		dgInt32 m_count;
	};

	class dgVertexBuffer: public dgRefCounter  
	{
		public: 
		DG_CLASS_ALLOCATOR(allocator)
		dgVertexBuffer (dgInt32 count, dgMemoryAllocator* allocator);
		dgVertexBuffer (dgMemoryAllocator* const allocator, dgDeserialize deserialization, void* const userData);
		~dgVertexBuffer ();

		void Serialize(dgSerialize callback, void* const userData) const;
//		void GetVertexStreams (dgInt32 vertexStrideInByte, dgFloat32* const vertex, dgInt32 normalStrideInByte, dgFloat32* const normal, dgInt32 uvStrideInByte, dgFloat32* const uv) const;
		const dgFloat32* GetVertexPositions() const {return m_vertex;};
		const dgFloat32* GetVertexNormals() const {return m_normal;};
		const dgFloat32* GetVertexUVs() const {return m_uv;};
		

		dgFloat32 *m_uv;
		dgFloat32 *m_vertex;
		dgFloat32 *m_normal;
		dgMemoryAllocator* m_allocator;
		dgInt32 m_vertexCount;
	};

	class dgSubMesh
	{
		public:
		dgSubMesh (dgMemoryAllocator* const allocator);
		~dgSubMesh ();
		void Serialize(dgSerialize callback, void* const userData) const;

		dgInt32 *m_indexes;
		dgMemoryAllocator* m_allocator;
		dgInt32 m_faceOffset;
		dgInt32 m_material;
		dgInt32 m_faceCount;
		bool m_visibleFaces;
	};

	class dgMesh: public dgList<dgSubMesh>, public dgRefCounter 
	{
		public:
		dgMesh(dgMemoryAllocator* const allocator);
		dgMesh (dgMemoryAllocator* const allocator, dgDeserialize deserialization, void* const userData);
		~dgMesh();
		void Serialize(dgSerialize callback, void* const userData) const;
		dgSubMesh* AddgSubMesh(dgInt32 indexCount, dgInt32 material);

		bool m_IsVisible;
	};

	class dgDebriNodeInfo
	{
		public:
		dgDebriNodeInfo ();
		~dgDebriNodeInfo ();

//		struct PackedSaveData {
//			union {
//				dgInt32 m_lru;
//				dgInt32 m_shapeID;
//			};
//			dgInt32 m_distanceToFixNode;
//			dgInt32 m_islandIndex; 
//		} m_commonData;

		dgMesh* m_mesh;
		dgTreeArray::dgTreeNode* m_shapeNode;
		dgInt32 m_lru;
	};


	class dgSharedNodeMesh
	{
		public:
		dgSharedNodeMesh ();
		~dgSharedNodeMesh ();
	};

	class dgConectivityGraph: public dgGraph<dgDebriNodeInfo, dgSharedNodeMesh>
	{
		public:
		dgConectivityGraph (dgMemoryAllocator* const allocator);
		dgConectivityGraph (const dgConectivityGraph& source);
		dgConectivityGraph (dgMemoryAllocator* const allocator, dgDeserialize deserialization, void* const userData);
		~dgConectivityGraph ();

//		void AddToHeap (dgDownHeap<dgMeshEffect*, dgFloat32>& heap, dgMeshEffect* front);
		dgListNode* AddNode (dgFlatVertexArray& vertexArray, dgMeshEffect* const factureVisualMesh, dgTreeArray::dgTreeNode* const collisionNode, dgInt32 interiorMaterialBase);
//		void SplitAndAddNodes (dgFlatVertexArray& vertexArray, dgMeshEffect* solid, dgMeshEffect* const clipper, 
//							   dgInt32 clipperMaterial, dgInt32 id, dgFloat32 density, dgCollisionCompoundBreakableCallback callback, void* buildUsedData);

//		void AddMeshes (dgFlatVertexArray& vertexArray, dgInt32 count, dgMeshEffect* const solidArray[], dgMeshEffect* const clipperArray[], 
//						dgMatrix* const matrixArray, dgInt32* const idArray, dgFloat32* const densities, dgCollisionCompoundBreakableCallback callback, void* buildUsedData);

		void Serialize(dgSerialize callback, void* const userData) const;
	};

	class dgConectivityGraphMap: public dgTree<dgConectivityGraph::dgListNode*, const dgCollisionInstance*>
	{
		public:
		dgConectivityGraphMap (const dgConectivityGraphMap& source)
			:dgTree<dgConectivityGraph::dgListNode*, const dgCollisionInstance*>(source.GetAllocator())
		{
		}

		dgConectivityGraphMap (dgMemoryAllocator* const allocator)
			:dgTree<dgConectivityGraph::dgListNode*, const dgCollisionInstance*>(allocator)
		{
		}

		void Pupolate(const dgConectivityGraph& graph)
		{
			for (dgConectivityGraph::dgListNode* node = graph.GetFirst(); node != graph.GetLast(); node = node->GetNext() ) {
				dgDebriNodeInfo& nodeInfo = node->GetInfo().m_nodeData;
				Insert(node, nodeInfo.m_shapeNode->GetInfo()->GetShape());
			}
		}
	};

/*
	class dgCompoundBreakableFilterData
	{
		public:
		dgInt32 m_index;
		dgDebriGraph::dgListNode* m_node;
	};
	
	class dgIsland: public dgList<dgDebriGraph::dgListNode*> 
	{
		public:
		dgIsland(dgMemoryAllocator* const allocator)
			:dgList<dgDebriGraph::dgListNode*> (allocator)
		{
		}
	};
#endif
*/

	public:
	dgCollisionCompoundFractured (const dgCollisionCompoundFractured& source);
	dgCollisionCompoundFractured (dgWorld* const world, dgMeshEffect* const solidMesh, int fracturePhysicsMaterialID, int pointcloudCount, const dgFloat32* const vertexCloud, int strideInBytes, int materialID, const dgMatrix& offsetMatrix);

	dgCollisionCompoundFractured (dgWorld* const world, dgDeserialize deserialization, void* const userData);
	virtual ~dgCollisionCompoundFractured(void);

	dgConectivityGraph::dgListNode* GetMainMesh() const {return m_conectivity.GetLast();}

	dgInt32 GetVertecCount() const {return m_vertexBuffer->m_vertexCount;}

	const dgFloat32* GetVertexPositions () const {return m_vertexBuffer->GetVertexPositions();}
	const dgFloat32* GetVertexNormal () const {return m_vertexBuffer->GetVertexNormals();}
	const dgFloat32* GetVertexUVs () const {return m_vertexBuffer->GetVertexUVs();}
	
	dgInt32 GetSegmentIndexStream (dgConectivityGraph::dgListNode* const node, dgMesh::dgListNode* const segment, dgInt32* const index) const;
	dgInt32 GetSegmentIndexStreamShort (dgConectivityGraph::dgListNode* const node, dgMesh::dgListNode* segment, dgInt16* const index) const;

	void SetImpulseStrength(dgFloat32 impulseStrength);
	dgFloat32 GetImpulseStrength() const;

	void SetImpulsePropgationFactor(dgFloat32 factor);
	dgFloat32 GetSetImpulsePropgationFactor() const;


/*
	void DeleteComponentBegin ();
	dgBody* CreateComponentBody (dgDebriGraph::dgListNode* node) const;
	void DeleteComponent (dgDebriGraph::dgListNode* node);
	void DeleteComponentEnd ();
	dgDebriGraph::dgListNode* GetFirstComponentMesh () const {return (m_conectivity.GetCount() > 2) ? m_conectivity.GetFirst()->GetNext() : NULL;}
	dgInt32 GetSegmentsInRadius (const dgVector& origin, dgFloat32 radius, dgDebriGraph::dgListNode** segments, dgInt32 maxCount);

	void ResetAnchor ();
	void SetAnchoredParts (dgInt32 count, const dgMatrix* const matrixArray, const dgCollision** collisionArray);
	void EnumerateIslands ();
//	dgInt32 GetSegmentsInRadius (const dgVector& origin, dgFloat32 radius, dgDebriGraph::dgListNode** segments, dgInt32 maxCount);
//	dgInt32 GetDetachedPieces (dgCollision** shapes, dgInt32 maxCount);
	private:
	void LinkNodes ();	
	virtual void GetCollisionInfo(dgCollisionInfo* const info) const;
	virtual void Serialize(dgSerialize callback, void* const userData) const;
	
	dgInt32 m_lru;
	dgInt32 m_lastIslandColor;
	dgIsland m_detachedIslands;
*/	
	private:
	dgVector GetObbSize() const;
    virtual void CalcAABB (const dgMatrix& matrix, dgVector& p0, dgVector& p1) const;
	dgInt32 CalculateContacts (dgCollidingPairCollector::dgPair* const pair, dgCollisionParamProxy& proxy) const;

	void SpawnSingleDrebree (dgBody* const myBody, dgConectivityGraph::dgListNode* const rootNode, dgFloat32 impulseStimate2, dgFloat32 impulseStimateCut2) const;

	dgConectivityGraph m_conectivity;
	dgConectivityGraphMap m_conectivityMap;
	dgVertexBuffer* m_vertexBuffer;
	dgInt8* m_visibilityMap;
	dgInt32* m_visibilityIndirectMap;
	mutable dgInt32 m_lru;
	dgInt32 m_visibilityMapIndexCount;
	dgFloat32 m_impulseStrengthPerUnitMass;
	dgFloat32 m_impulseAbsortionFactor;
};
#endif
