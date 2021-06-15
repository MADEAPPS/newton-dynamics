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

#ifndef __DG_AABB_POLYGONSOUP_H_
#define __DG_AABB_POLYGONSOUP_H_

#include "dCoreStdafx.h"
#include "dTypes.h"
#include "dIntersections.h"
#include "dPolygonSoupDatabase.h"

class dPolygonSoupBuilder;

// index format: i0, i1, i2, ... , id, normal, e0Normal, e1Normal, e2Normal, ..., faceSize
#define D_CONCAVE_EDGE_MASK	(1<<31)

class dAabbPolygonSoup: public dPolygonSoupDatabase
{
	public:
	class dNode
	{
		public:
		enum dNodeType
		{
			m_binary = 0,
			m_leaf,
		};

		class dgLeafNodePtr
		{
			#define DG_INDEX_COUNT_BITS 6

			public:
			inline dgLeafNodePtr ()
			{
				dAssert (0);
			}

			inline dgLeafNodePtr (dUnsigned32 node)
			{
				m_node = node;
				dAssert (!IsLeaf());
			}

			inline dUnsigned32 IsLeaf () const 
			{
				return m_node & 0x80000000;
			}

			inline dUnsigned32 GetCount() const 
			{
				dAssert (IsLeaf());
				return (m_node & (~0x80000000)) >> (32 - DG_INDEX_COUNT_BITS - 1);
			}

			inline dUnsigned32 GetIndex() const 
			{
				dAssert (IsLeaf());
				return m_node & (~(-(1 << (32 - DG_INDEX_COUNT_BITS - 1))));
			}

			inline dgLeafNodePtr (dUnsigned32 faceIndexCount, dUnsigned32 faceIndexStart)
			{
				dAssert (faceIndexCount < (1<<DG_INDEX_COUNT_BITS));
				m_node = 0x80000000 | (faceIndexCount << (32 - DG_INDEX_COUNT_BITS - 1)) | faceIndexStart;
			}

			inline dNode* GetNode (const void* const root) const
			{
				return ((dNode*) root) + m_node;
			}

			dUnsigned32 m_node;
		};

		dNode ()
			:m_indexBox0(0)
			,m_indexBox1(0)
			,m_left(0)
			,m_right(0)
		{
		}

		inline dFloat32 RayDistance (const dFastRayTest& ray, const dTriplex* const vertexArray) const
		{
			dVector minBox (&vertexArray[m_indexBox0].m_x);
			dVector maxBox (&vertexArray[m_indexBox1].m_x);
			minBox = minBox & dVector::m_triplexMask;
			maxBox = maxBox & dVector::m_triplexMask;
			return ray.BoxIntersect(minBox, maxBox);
		}

		inline dFloat32 BoxPenetration (const dFastAabbInfo& obb, const dTriplex* const vertexArray) const
		{
			dVector p0 (&vertexArray[m_indexBox0].m_x);
			dVector p1 (&vertexArray[m_indexBox1].m_x);
			p0 = p0 & dVector::m_triplexMask;
			p1 = p1 & dVector::m_triplexMask;
			dVector minBox (p0 - obb.m_p1);
			dVector maxBox (p1 - obb.m_p0);
			dAssert(maxBox.m_x >= minBox.m_x);
			dAssert(maxBox.m_y >= minBox.m_y);
			dAssert(maxBox.m_z >= minBox.m_z);

			dVector mask ((minBox * maxBox) < dVector::m_zero);
			dVector dist (maxBox.GetMin (minBox.Abs()) & mask);
			dist = dist.GetMin(dist.ShiftTripleRight());
			dist = dist.GetMin(dist.ShiftTripleRight());

			if (dist.GetScalar() > dFloat32 (0.0f)) 
			{
				dVector origin (dVector::m_half * (p1 + p0));
				dVector size (dVector::m_half * (p1 - p0));

				origin = obb.UntransformVector(origin);
				size = obb.m_absDir.RotateVector(size);
				dVector q0 (origin - size);
				dVector q1 (origin + size);
				dVector minBox1 (q0 - obb.m_size);
				dVector maxBox1 (q1 + obb.m_size);
				dAssert(maxBox1.m_x >= minBox1.m_x);
				dAssert(maxBox1.m_y >= minBox1.m_y);
				dAssert(maxBox1.m_z >= minBox1.m_z);
				dVector mask1 ((minBox1 * maxBox1) < dVector::m_zero);
				dVector dist1 (maxBox1.GetMin (minBox1.Abs()) & mask1);
				dist1 = dist1.GetMin(dist1.ShiftTripleRight());
				dist1 = dist1.GetMin(dist1.ShiftTripleRight());
				dist = dist.GetMin(dist1);
			} 
			else 
			{
				dVector p1p0((minBox.Abs()).GetMin(maxBox.Abs()).AndNot(mask));
				dist = p1p0.DotProduct(p1p0);
				dist = dist.Sqrt() * dVector::m_negOne;
			}
			return	dist.GetScalar();
		}

		inline dFloat32 BoxIntersect (const dFastRayTest& ray, const dFastRayTest& obbRay, const dFastAabbInfo& obb, const dTriplex* const vertexArray) const
		{
			dVector p0 (&vertexArray[m_indexBox0].m_x);
			dVector p1 (&vertexArray[m_indexBox1].m_x);
			p0 = p0 & dVector::m_triplexMask;
			p1 = p1 & dVector::m_triplexMask;

			dVector minBox (p0 - obb.m_p1);
			dVector maxBox (p1 - obb.m_p0);
			dFloat32 dist = ray.BoxIntersect(minBox, maxBox);
			if (dist < dFloat32 (1.0f)) 
			{
				dVector origin (dVector::m_half * (p1 + p0));
				dVector size (dVector::m_half * (p1 - p0));

				origin = obb.UntransformVector(origin);
				size = obb.m_absDir.RotateVector(size);
				dVector q0 (origin - size);
				dVector q1 (origin + size);

				dVector minBox1 (q0 - obb.m_size);
				dVector maxBox1 (q1 + obb.m_size);
				dFloat32 dist1 = obbRay.BoxIntersect(minBox1, maxBox1);
				dist = (dist1  > dFloat32 (1.0f)) ? dist1 : dMax (dist1, dist);
			}
			return dist;
		}

		dInt32 m_indexBox0;
		dInt32 m_indexBox1;
		dgLeafNodePtr m_left;
		dgLeafNodePtr m_right;
	};

	class dgSpliteInfo;
	class dgNodeBuilder;

	D_CORE_API virtual void GetAABB (dVector& p0, dVector& p1) const;
	D_CORE_API virtual void Serialize (const char* const path) const;
	D_CORE_API virtual void Deserialize (const char* const path);

	protected:
	D_CORE_API dAabbPolygonSoup ();
	D_CORE_API virtual ~dAabbPolygonSoup ();

	D_CORE_API void Create (const dPolygonSoupBuilder& builder);
	D_CORE_API void CalculateAdjacendy ();
	D_CORE_API virtual dVector ForAllSectorsSupportVectex(const dVector& dir) const;
	D_CORE_API virtual void ForAllSectorsRayHit (const dFastRayTest& ray, dFloat32 maxT, dRayIntersectCallback callback, void* const context) const;
	D_CORE_API virtual void ForAllSectors (const dFastAabbInfo& obbAabb, const dVector& boxDistanceTravel, dFloat32 maxT, dAaabbIntersectCallback callback, void* const context) const;
	D_CORE_API virtual void ForThisSector(const dAabbPolygonSoup::dNode* const node, const dFastAabbInfo& obbAabb, const dVector& boxDistanceTravel, dFloat32 maxT, dAaabbIntersectCallback callback, void* const context) const;

	inline dNode* GetRootNode() const
	{
		return m_aabb;
	}

	inline dNode* GetBackNode(const void* const root) const
	{
		dNode* const node = (dNode*) root;
		return node->m_left.IsLeaf() ? nullptr : node->m_left.GetNode(m_aabb);
	}

	inline dNode* GetFrontNode(const void* const root) const
	{
		dNode* const node = (dNode*) root;
		return node->m_right.IsLeaf() ? nullptr : node->m_right.GetNode(m_aabb);
	}

	inline void GetNodeAABB(const void* const root, dVector& p0, dVector& p1) const 
	{
		const dNode* const node = (dNode*)root;
		p0 = dVector (&((dTriplex*)m_localVertex)[node->m_indexBox0].m_x);
		p1 = dVector (&((dTriplex*)m_localVertex)[node->m_indexBox1].m_x);
		p0 = p0 & dVector::m_triplexMask;
		p1 = p1 & dVector::m_triplexMask;
	}

	private:
	dgNodeBuilder* BuildTopDown (dgNodeBuilder* const leafArray, dInt32 firstBox, dInt32 lastBox, dgNodeBuilder** const allocator) const;
	dFloat32 CalculateFaceMaxSize (const dVector* const vertex, dInt32 indexCount, const dInt32* const indexArray) const;
	static dIntersectStatus CalculateDisjointedFaceEdgeNormals (void* const context, const dFloat32* const polygon, dInt32 strideInBytes, const dInt32* const indexArray, dInt32 indexCount, dFloat32 hitDistance);
	static dIntersectStatus CalculateAllFaceEdgeNormals(void* const context, const dFloat32* const polygon, dInt32 strideInBytes, const dInt32* const indexArray, dInt32 indexCount, dFloat32 hitDistance);
	void ImproveNodeFitness (dgNodeBuilder* const node) const;

	dNode* m_aabb;
	dInt32* m_indices;
	dInt32 m_nodesCount;
	dInt32 m_indexCount;
	friend class ndContactSolver;
};


#endif


