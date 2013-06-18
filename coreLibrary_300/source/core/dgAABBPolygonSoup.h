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

/****************************************************************************
*
*  File Name  : Bitmap.C
*  Visual C++ 4.0 base by Julio Jerez
*
****************************************************************************/
#ifndef __DG_AABB_POLYGON_SOUP_H_
#define __DG_AABB_POLYGON_SOUP_H_

#include "dgStdafx.h"
#include "dgPolygonSoupDatabase.h"


class dgPolygonSoupDatabaseBuilder;

//#define DG_NEW_AABB_TREE

#ifdef DG_NEW_AABB_TREE

class dgAABBPolygonSoup: public dgPolygonSoupDatabase
{
	public:
	DG_MSC_VECTOR_AVX_ALIGMENT
	class dgNode
	{
		public:
		enum dgNodeType
		{
			m_binary = 0,
			m_leaf,
		};

		dgNode ()
			:m_indexBox0(0)
			,m_indexBox1(0)
			,m_left(NULL)
			,m_right(NULL)
		{
		}

		DG_INLINE dgInt32 RayTest (const dgFastRayTest& ray, const dgTriplex* const vertexArray) const
		{
			dgVector p0 (&vertexArray[m_indexBox0].m_x);
			dgVector p1 (&vertexArray[m_indexBox1].m_x);
			return ray.BoxTest (p0, p1);
		}

		DG_INLINE dgFloat32 BoxIntersect (const dgFastRayTest& ray, const dgTriplex* const vertexArray, const dgVector& boxP0, const dgVector& boxP1) const
		{
			dgVector p0 (&vertexArray[m_indexBox0].m_x);
			dgVector p1 (&vertexArray[m_indexBox1].m_x);
			dgVector minBox (p0 - boxP1);
			dgVector maxBox (p1 - boxP0);
			return ray.BoxIntersect(minBox, maxBox);
		}

		DG_INLINE dgFloat32 BoxPenetration (const dgTriplex* const vertexArray, const dgVector& boxP0, const dgVector& boxP1) const
		{
			dgVector p0 (&vertexArray[m_indexBox0].m_x);
			dgVector p1 (&vertexArray[m_indexBox1].m_x);
			dgVector minBox (p0 - boxP1);
			dgVector maxBox (p1 - boxP0);
			dgVector mask ((minBox.CompProduct4(maxBox)) < dgVector (dgFloat32 (0.0f)));
			mask = mask & mask.ShiftTripleRight();
			mask = mask & mask.ShiftTripleRight();
			dgVector dist (maxBox.GetMin (minBox.Abs()) & mask);
			dist = dist.GetMin(dist.ShiftTripleRight());
			dist = dist.GetMin(dist.ShiftTripleRight());
			return dist.m_x;
		}



		dgInt32 m_indexBox0;
		dgInt32 m_indexBox1 : 31;
		dgInt32 m_nodeType	:  1;
		dgNode* m_left;
		dgNode* m_right;
	} DG_GCC_VECTOR_AVX_ALIGMENT;

	class dgNodeBuilder;

//	dgInt32 GetIndexCount() const
//	{
//		return m_indexCount;
//	}

//	dgInt32* GetIndexPool() const
//	{
//		return m_indices;
//	}

	virtual void GetAABB (dgVector& p0, dgVector& p1) const;
	virtual void Serialize (dgSerialize callback, void* const userData) const {dgAssert (0);}
	virtual void Deserialize (dgDeserialize callback, void* const userData) {dgAssert (0);}

	protected:
	dgAABBPolygonSoup ();
	virtual ~dgAABBPolygonSoup ();

	void Create (dgMemoryAllocator* const allocator, const dgPolygonSoupDatabaseBuilder& builder, bool optimizedBuild);
	virtual void ForAllSectorsRayHit (const dgFastRayTest& ray, dgRayIntersectCallback callback, void* const context) const;
	virtual void ForAllSectors (const dgVector& minBox, const dgVector& maxBox, const dgVector& boxDistanceTravel, dgFloat32 m_maxT, dgAABBIntersectCallback callback, void* const context) const;

	void* GetRootNode() const 
	{
		return m_aabb;
	}

	void* GetBackNode(const void* const root) const 
	{
		dgNode* const node = ((dgNode*)root)->m_left;
		return (node->m_nodeType & dgNode::m_leaf) ? NULL: node;
	}

	void* GetFrontNode(const void* const root) const 
	{
		dgNode* const node = ((dgNode*)root)->m_right;
		return (node->m_nodeType & dgNode::m_leaf) ? NULL: node;
	}

	void GetNodeAABB(const void* const root, dgVector& p0, dgVector& p1) const 
	{
		dgNode* const node = (dgNode*)root;
		p0 = dgVector (&m_aabbPoints[node->m_indexBox0].m_x);
		p1 = dgVector (&m_aabbPoints[node->m_indexBox1].m_x);
	}
	virtual dgVector ForAllSectorsSupportVectex (const dgVector& dir) const {dgAssert (0); return dgVector(0.0f);}	
	

	private:
	void CalculateAdjacendy ();
	dgFloat32 CalculateFaceMaxSize (dgTriplex* const vertex, dgInt32 indexCount, const dgInt32* const indexArray) const;
//	static dgIntersectStatus CalculateManifoldFaceEdgeNormals (void* const context, const dgFloat32* const polygon, dgInt32 strideInBytes, const dgInt32* const indexArray, dgInt32 indexCount);
	static dgIntersectStatus CalculateDisjointedFaceEdgeNormals (void* const context, const dgFloat32* const polygon, dgInt32 strideInBytes, const dgInt32* const indexArray, dgInt32 indexCount, dgFloat32 hitDistance);
	static dgIntersectStatus CalculateAllFaceEdgeNormals (void* const context, const dgFloat32* const polygon, dgInt32 strideInBytes, const dgInt32* const indexArray, dgInt32 indexCount, dgFloat32 hitDistance);
	void ImproveNodeFitness (dgNodeBuilder* const node) const;

	dgInt32 m_nodesCount;
	dgInt32 m_indexCount;
	dgNode* m_aabb;
	dgInt32* m_indices;
	dgTriplex* m_aabbPoints;
};

#else
class dgAABBPolygonSoup: public dgPolygonSoupDatabase
{
	public:
	dgInt32 GetIndexCount() const
	{
		return m_indexCount;
	}

	dgInt32* GetIndexPool() const
	{
		return m_indices;
	}

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
	void Create (dgMemoryAllocator* const allocator, const dgPolygonSoupDatabaseBuilder& builder, bool optimizedBuild);
	dgFloat32 CalculateFaceMaxSize (dgTriplex* const vertex, dgInt32 indexCount, const dgInt32* const indexArray) const;

	virtual void ForAllSectors (const dgVector& minBox, const dgVector& maxBox, const dgVector& boxDistanceTravel, dgFloat32 m_maxT, dgAABBIntersectCallback callback, void* const context) const;
	virtual void ForAllSectorsRayHit (const dgFastRayTest& ray, dgRayIntersectCallback callback, void* const context) const;
	virtual dgVector ForAllSectorsSupportVectex (const dgVector& dir) const;	

	private:
	static dgIntersectStatus CalculateManifoldFaceEdgeNormals (void* const context, const dgFloat32* const polygon, dgInt32 strideInBytes, const dgInt32* const indexArray, dgInt32 indexCount);
	static dgIntersectStatus CalculateDisjointedFaceEdgeNormals (void* const context, const dgFloat32* const polygon, dgInt32 strideInBytes, const dgInt32* const indexArray, dgInt32 indexCount, dgFloat32 hitDistance);
	static dgIntersectStatus CalculateAllFaceEdgeNormals (void* const context, const dgFloat32* const polygon, dgInt32 strideInBytes, const dgInt32* const indexArray, dgInt32 indexCount, dgFloat32 hitDistance);

	dgInt32 m_nodesCount;
	dgInt32 m_indexCount;
	dgInt32 *m_indices;
	void* m_aabb;
};

#endif

#endif


