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

#ifndef __DG_AABB_POLYGON_SOUP_H_
#define __DG_AABB_POLYGON_SOUP_H_

#include "dgStdafx.h"
#include "dgPolygonSoupDatabase.h"


class dgPolygonSoupDatabaseBuilder;


class dgAABBPolygonSoup: public dgPolygonSoupDatabase
{
	class dgConstructionTree;

	class dgNode
	{
		class dgLeafNodePtr
		{
			#define DG_INDEX_COUNT_BITS 6

			public:
			DG_INLINE dgLeafNodePtr ()
			{
				dgAssert (0);
			}

			DG_INLINE dgLeafNodePtr (dgUnsigned32 node)
			{
				m_node = node;
				dgAssert (!IsLeaf());
			}

			DG_INLINE dgUnsigned32 IsLeaf () const 
			{
				return m_node & 0x80000000;
			}

			DG_INLINE dgUnsigned32 GetCount() const 
			{
				dgAssert (IsLeaf());
				return (m_node & (~0x80000000)) >> (32 - DG_INDEX_COUNT_BITS - 1);
			}

			DG_INLINE dgUnsigned32 GetIndex() const 
			{
				dgAssert (IsLeaf());
				return m_node & (~(-(1 << (32 - DG_INDEX_COUNT_BITS - 1))));
			}


			DG_INLINE dgLeafNodePtr (dgUnsigned32 faceIndexCount, dgUnsigned32 faceIndexStart)
			{
				dgAssert (faceIndexCount < (1<<DG_INDEX_COUNT_BITS));
				m_node = 0x80000000 | (faceIndexCount << (32 - DG_INDEX_COUNT_BITS - 1)) | faceIndexStart;
			}

			DG_INLINE dgNode* GetNode (const void* root) const
			{
				return ((dgNode*) root) + m_node;
			}

			union {
				dgUnsigned32 m_node;
			};
		};

		void CalcExtends (dgVector* const vertex, dgInt32 indexCount, const dgInt32* const indexArray) 
		{
			dgVector minP ( dgFloat32 (1.0e15f),  dgFloat32 (1.0e15f),  dgFloat32 (1.0e15f), dgFloat32 (0.0f)); 
			dgVector maxP (-dgFloat32 (1.0e15f), -dgFloat32 (1.0e15f), -dgFloat32 (1.0e15f), dgFloat32 (0.0f)); 
			for (dgInt32 i = 0; i < indexCount; i ++) {
				dgInt32 index = indexArray[i];
				const dgVector p = vertex[index];
				minP = minP.GetMin(p);
				maxP = maxP.GetMax(p);
			}
			vertex[m_minIndex] = (minP - dgVector (dgFloat32 (1.0e-3f))) & dgVector::m_triplexMask;
			vertex[m_maxIndex] = (maxP + dgVector (dgFloat32 (1.0e-3f))) & dgVector::m_triplexMask;
		}


		DG_INLINE dgInt32 BoxTest (const dgTriplex* const vertexArray, const dgVector& boxP0, const dgVector& boxP1) const
		{
			dgVector minBox (&vertexArray[m_minIndex].m_x);
			dgVector maxBox (&vertexArray[m_maxIndex].m_x);
			return dgOverlapTest (minBox, maxBox, boxP0, boxP1);
		}

		DG_INLINE dgFloat32 BoxPenetration (const dgTriplex* const vertexArray, const dgVector& boxP0, const dgVector& boxP1) const
		{
			dgVector p0 (&vertexArray[m_minIndex].m_x);
			dgVector p1 (&vertexArray[m_maxIndex].m_x);
			dgVector minBox (p0 - boxP1);
			dgVector maxBox (p1 - boxP0);
			return ::BoxPenetration (minBox, maxBox);
		}


		DG_INLINE dgFloat32 BoxIntersect (const dgFastRayTest& ray, const dgTriplex* const vertexArray, const dgVector& boxP0, const dgVector& boxP1) const
		{
			dgVector p0 (&vertexArray[m_minIndex].m_x);
			dgVector p1 (&vertexArray[m_maxIndex].m_x);

			dgVector minBox (p0 - boxP1);
			dgVector maxBox (p1 - boxP0);
			return ray.BoxIntersect(minBox, maxBox);
		}


		DG_INLINE dgInt32 RayTest (const dgFastRayTest& ray, const dgTriplex* const vertexArray) const
		{
			dgVector minBox (&vertexArray[m_minIndex].m_x);
			dgVector maxBox (&vertexArray[m_maxIndex].m_x);
			//return RayTest (ray, minBox, maxBox);
			return ray.BoxTest (minBox, maxBox);
		}


		dgVector ForAllSectorsSupportVertex (const dgVector& dir, const dgInt32* const indexArray, const dgFloat32* const vertexArray) const;
		void ForAllSectorsRayHit (const dgFastRayTest& raySrc, const dgInt32* const indexArray, const dgFloat32* const vertexArray, dgRayIntersectCallback callback, void* const context) const;
	

		dgInt32 m_minIndex;
		dgInt32 m_maxIndex;
		dgLeafNodePtr m_back;
		dgLeafNodePtr m_front;
		friend class dgAABBPolygonSoup;
	};

	

	public:
	virtual void GetAABB (dgVector& p0, dgVector& p1) const;
	virtual void Serialize (dgSerialize callback, void* const userData) const;
	virtual void Deserialize (dgDeserialize callback, void* const userData);

	protected:
	dgAABBPolygonSoup ();
	~dgAABBPolygonSoup ();

	void* GetRootNode() const;
	void* GetBackNode(const void* const root) const;
	void* GetFrontNode(const void* const root) const;
	void GetNodeAABB(const void* const root, dgVector& p0, dgVector& p1) const;

	void CalculateAdjacendy ();
	void Create (const dgPolygonSoupDatabaseBuilder& builder, bool optimizedBuild);
	dgFloat32 CalculateFaceMaxSize (const dgVector* const vertex, dgInt32 indexCount, const dgInt32* const indexArray) const;

	virtual void ForAllSectors (const dgVector& minBox, const dgVector& maxBox, const dgVector& boxDistanceTravel, dgFloat32 m_maxT, dgAABBIntersectCallback callback, void* const context) const;
	virtual void ForAllSectorsRayHit (const dgFastRayTest& ray, dgRayIntersectCallback callback, void* const context) const;
	virtual dgVector ForAllSectorsSupportVectex (const dgVector& dir) const;	

	private:
	dgInt32 BuildTopDown (dgMemoryAllocator* const allocator, dgInt32 boxCount, dgNode* const boxArray, dgVector* const vertexArrayOut, dgInt32 &treeVCount, bool optimizedBuild);
	static dgIntersectStatus CalculateManifoldFaceEdgeNormals (void* const context, const dgFloat32* const polygon, dgInt32 strideInBytes, const dgInt32* const indexArray, dgInt32 indexCount);
	static dgIntersectStatus CalculateDisjointedFaceEdgeNormals (void* const context, const dgFloat32* const polygon, dgInt32 strideInBytes, const dgInt32* const indexArray, dgInt32 indexCount, dgFloat32 hitDistance);
	static dgIntersectStatus CalculateAllFaceEdgeNormals (void* const context, const dgFloat32* const polygon, dgInt32 strideInBytes, const dgInt32* const indexArray, dgInt32 indexCount, dgFloat32 hitDistance);

	dgInt32 m_nodesCount;
	dgInt32 m_indexCount;
	dgInt32 *m_indices;
	dgNode* m_aabb;
};

#endif


