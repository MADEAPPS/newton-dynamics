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

#ifndef __NDG_AABB_POLYGONSOUP_H_
#define __NDG_AABB_POLYGONSOUP_H_

#include "ndCoreStdafx.h"
#include "ndTypes.h"
#include "ndUtils.h"
#include "ndFastRay.h"
#include "ndFastAabb.h"
#include "ndIntersections.h"
#include "ndPolygonSoupDatabase.h"

class ndPolygonSoupBuilder;

// index format: i0, i1, i2, ... , id, normal, e0Normal, e1Normal, e2Normal, ..., faceSize
#define D_CONCAVE_EDGE_MASK	(1<<31)

class ndAabbPolygonSoup: public ndPolygonSoupDatabase
{
	public:
	class ndNode
	{
		public:
		enum ndNodeType
		{
			m_binary = 0,
			m_leaf,
		};

		class ndLeafNodePtr
		{
			#define DG_INDEX_COUNT_BITS 6

			public:
			inline ndLeafNodePtr ()
			{
				dAssert (0);
			}

			inline ndLeafNodePtr (ndUnsigned32 node)
			{
				m_node = node;
				dAssert (!IsLeaf());
			}

			inline ndUnsigned32 IsLeaf () const 
			{
				return m_node & 0x80000000;
			}

			inline ndUnsigned32 GetCount() const 
			{
				dAssert (IsLeaf());
				return (m_node & (~0x80000000)) >> (32 - DG_INDEX_COUNT_BITS - 1);
			}

			inline ndUnsigned32 GetIndex() const 
			{
				dAssert (IsLeaf());
				return m_node & (~(-(1 << (32 - DG_INDEX_COUNT_BITS - 1))));
			}

			inline ndLeafNodePtr (ndUnsigned32 faceIndexCount, ndUnsigned32 faceIndexStart)
			{
				dAssert (faceIndexCount < (1<<DG_INDEX_COUNT_BITS));
				m_node = 0x80000000 | (faceIndexCount << (32 - DG_INDEX_COUNT_BITS - 1)) | faceIndexStart;
			}

			inline ndNode* GetNode (const void* const root) const
			{
				return ((ndNode*) root) + m_node;
			}

			ndUnsigned32 m_node;
		};

		ndNode ()
			:m_indexBox0(0)
			,m_indexBox1(0)
			,m_left(0)
			,m_right(0)
		{
		}

		inline ndFloat32 RayDistance (const ndFastRay& ray, const ndTriplex* const vertexArray) const
		{
			ndVector minBox (&vertexArray[m_indexBox0].m_x);
			ndVector maxBox (&vertexArray[m_indexBox1].m_x);
			minBox = minBox & ndVector::m_triplexMask;
			maxBox = maxBox & ndVector::m_triplexMask;
			return ray.BoxIntersect(minBox, maxBox);
		}

		inline ndFloat32 BoxPenetration (const ndFastAabb& obb, const ndTriplex* const vertexArray) const
		{
			ndVector p0 (&vertexArray[m_indexBox0].m_x);
			ndVector p1 (&vertexArray[m_indexBox1].m_x);
			p0 = p0 & ndVector::m_triplexMask;
			p1 = p1 & ndVector::m_triplexMask;
			ndVector minBox (p0 - obb.m_p1);
			ndVector maxBox (p1 - obb.m_p0);
			dAssert(maxBox.m_x >= minBox.m_x);
			dAssert(maxBox.m_y >= minBox.m_y);
			dAssert(maxBox.m_z >= minBox.m_z);

			ndVector mask ((minBox * maxBox) < ndVector::m_zero);
			ndVector dist (maxBox.GetMin (minBox.Abs()) & mask);
			dist = dist.GetMin(dist.ShiftTripleRight());
			dist = dist.GetMin(dist.ShiftTripleRight());

			if (dist.GetScalar() > ndFloat32 (0.0f)) 
			{
				ndVector origin (ndVector::m_half * (p1 + p0));
				ndVector size (ndVector::m_half * (p1 - p0));

				origin = obb.UntransformVector(origin);
				size = obb.m_absDir.RotateVector(size);
				ndVector q0 (origin - size);
				ndVector q1 (origin + size);
				ndVector minBox1 (q0 - obb.m_size);
				ndVector maxBox1 (q1 + obb.m_size);
				dAssert(maxBox1.m_x >= minBox1.m_x);
				dAssert(maxBox1.m_y >= minBox1.m_y);
				dAssert(maxBox1.m_z >= minBox1.m_z);
				ndVector mask1 ((minBox1 * maxBox1) < ndVector::m_zero);
				ndVector dist1 (maxBox1.GetMin (minBox1.Abs()) & mask1);
				dist1 = dist1.GetMin(dist1.ShiftTripleRight());
				dist1 = dist1.GetMin(dist1.ShiftTripleRight());
				dist = dist.GetMin(dist1);
			} 
			else 
			{
				ndVector p1p0((minBox.Abs()).GetMin(maxBox.Abs()).AndNot(mask));
				dist = p1p0.DotProduct(p1p0);
				dist = dist.Sqrt() * ndVector::m_negOne;
			}
			return	dist.GetScalar();
		}

		inline ndFloat32 BoxIntersect (const ndFastRay& ray, const ndFastRay& obbRay, const ndFastAabb& obb, const ndTriplex* const vertexArray) const
		{
			ndVector p0 (&vertexArray[m_indexBox0].m_x);
			ndVector p1 (&vertexArray[m_indexBox1].m_x);
			p0 = p0 & ndVector::m_triplexMask;
			p1 = p1 & ndVector::m_triplexMask;

			ndVector minBox (p0 - obb.m_p1);
			ndVector maxBox (p1 - obb.m_p0);
			ndFloat32 dist = ray.BoxIntersect(minBox, maxBox);
			if (dist < ndFloat32 (1.0f)) 
			{
				ndVector origin (ndVector::m_half * (p1 + p0));
				ndVector size (ndVector::m_half * (p1 - p0));

				origin = obb.UntransformVector(origin);
				size = obb.m_absDir.RotateVector(size);
				ndVector q0 (origin - size);
				ndVector q1 (origin + size);

				ndVector minBox1 (q0 - obb.m_size);
				ndVector maxBox1 (q1 + obb.m_size);
				ndFloat32 dist1 = obbRay.BoxIntersect(minBox1, maxBox1);
				dist = (dist1  > ndFloat32 (1.0f)) ? dist1 : dMax (dist1, dist);
			}
			return dist;
		}

		ndInt32 m_indexBox0;
		ndInt32 m_indexBox1;
		ndLeafNodePtr m_left;
		ndLeafNodePtr m_right;
	};

	class ndSpliteInfo;
	class ndNodeBuilder;

	D_CORE_API virtual void GetAABB (ndVector& p0, ndVector& p1) const;
	D_CORE_API virtual void Serialize (const char* const path) const;
	D_CORE_API virtual void Deserialize (const char* const path);

	protected:
	D_CORE_API ndAabbPolygonSoup ();
	D_CORE_API virtual ~ndAabbPolygonSoup ();

	D_CORE_API void Create (const ndPolygonSoupBuilder& builder);
	D_CORE_API void CalculateAdjacendy ();
	D_CORE_API virtual ndVector ForAllSectorsSupportVectex(const ndVector& dir) const;
	D_CORE_API virtual void ForAllSectorsRayHit (const ndFastRay& ray, ndFloat32 maxT, dRayIntersectCallback callback, void* const context) const;
	D_CORE_API virtual void ForAllSectors (const ndFastAabb& obbAabb, const ndVector& boxDistanceTravel, ndFloat32 maxT, dAaabbIntersectCallback callback, void* const context) const;
	D_CORE_API virtual void ForThisSector(const ndAabbPolygonSoup::ndNode* const node, const ndFastAabb& obbAabb, const ndVector& boxDistanceTravel, ndFloat32 maxT, dAaabbIntersectCallback callback, void* const context) const;

	public:
	inline ndNode* GetRootNode() const
	{
		return m_aabb;
	}

	inline ndNode* GetBackNode(const ndNode* const node) const
	{
		return node->m_left.IsLeaf() ? nullptr : node->m_left.GetNode(m_aabb);
	}

	inline ndNode* GetFrontNode(const ndNode* const node) const
	{
		return node->m_right.IsLeaf() ? nullptr : node->m_right.GetNode(m_aabb);
	}

	inline void GetNodeAabb(const ndNode* const node, ndVector& p0, ndVector& p1) const
	{
		p0 = ndVector (&((ndTriplex*)m_localVertex)[node->m_indexBox0].m_x);
		p1 = ndVector (&((ndTriplex*)m_localVertex)[node->m_indexBox1].m_x);
		p0 = p0 & ndVector::m_triplexMask;
		p1 = p1 & ndVector::m_triplexMask;
	}

	private:
	ndNodeBuilder* BuildTopDown (ndNodeBuilder* const leafArray, ndInt32 firstBox, ndInt32 lastBox, ndNodeBuilder** const allocator) const;
	ndFloat32 CalculateFaceMaxSize (const ndVector* const vertex, ndInt32 indexCount, const ndInt32* const indexArray) const;
	static dIntersectStatus CalculateDisjointedFaceEdgeNormals (void* const context, const ndFloat32* const polygon, ndInt32 strideInBytes, const ndInt32* const indexArray, ndInt32 indexCount, ndFloat32 hitDistance);
	static dIntersectStatus CalculateAllFaceEdgeNormals(void* const context, const ndFloat32* const polygon, ndInt32 strideInBytes, const ndInt32* const indexArray, ndInt32 indexCount, ndFloat32 hitDistance);
	void ImproveNodeFitness (ndNodeBuilder* const node) const;

	ndNode* m_aabb;
	ndInt32* m_indices;
	ndInt32 m_nodesCount;
	ndInt32 m_indexCount;
	friend class ndContactSolver;
};

#endif


