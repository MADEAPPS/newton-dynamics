/* Copyright (c) <2003-2022> <Julio Jerez, Newton Game Dynamics>
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

#include "ndCoreStdafx.h"
#include "ndTypes.h"
#include "ndHeap.h"
#include "ndStack.h"
#include "ndList.h"
#include "ndMatrix.h"
#include "ndPolyhedra.h"
#include "ndAabbPolygonSoup.h"
#include "ndPolygonSoupBuilder.h"

#define DG_STACK_DEPTH 512

D_MSV_NEWTON_ALIGN_32
class ndAabbPolygonSoup::ndNodeBuilder: public ndAabbPolygonSoup::ndNode
{
	public:
	ndNodeBuilder (const ndVector& p0, const ndVector& p1)
		:ndNode()
		,m_left (nullptr)
		,m_right (nullptr)
		,m_parent (nullptr)
		,m_indexBox0(0)
		,m_indexBox1(0)
		,m_enumeration(-1)
		,m_faceIndex(0)
		,m_indexCount(0)
		,m_faceIndices(nullptr)
	{
		SetBox (p0, p1);
	}

	ndNodeBuilder (const ndVector* const vertexArray, ndInt32 faceIndex, ndInt32 indexCount, const ndInt32* const indexArray)
		:ndNode()
		,m_left (nullptr)
		,m_right (nullptr)
		,m_parent (nullptr)
		,m_indexBox0(0)
		,m_indexBox1(0)
		,m_enumeration(-1)
		,m_faceIndex(faceIndex)
		,m_indexCount(indexCount)
		,m_faceIndices(indexArray)
	{
		ndVector minP ( ndFloat32 (1.0e15f)); 
		ndVector maxP (-ndFloat32 (1.0e15f)); 
		for (ndInt32 i = 0; i < indexCount; ++i) 
		{
			ndInt32 index = indexArray[i];
			const ndVector& p (vertexArray[index]);
			minP = p.GetMin(minP); 
			maxP = p.GetMax(maxP); 
		}
		minP -= ndVector (ndFloat32 (1.0e-3f));
		maxP += ndVector (ndFloat32 (1.0e-3f));
		minP = minP & ndVector::m_triplexMask;
		maxP = maxP & ndVector::m_triplexMask;
		SetBox (minP, maxP);
	}

	ndNodeBuilder (ndNodeBuilder* const left, ndNodeBuilder* const right)
		:ndNode()
		,m_left(left)
		,m_right(right)
		,m_parent(nullptr)
		,m_indexBox0(0)
		,m_indexBox1(0)
		,m_enumeration(-1)
		,m_faceIndex(0)
		,m_indexCount(0)
		,m_faceIndices(nullptr)
	{
		m_left->m_parent = this;
		m_right->m_parent = this;

		ndVector p0 (left->m_p0.GetMin(right->m_p0));
		ndVector p1 (left->m_p1.GetMax(right->m_p1));
		SetBox(p0, p1);
	}

	void SetBox (const ndVector& p0, const ndVector& p1)
	{
		m_p0 = p0;
		m_p1 = p1;
	}

	ndVector m_p0;
	ndVector m_p1;
	ndNodeBuilder* m_left;
	ndNodeBuilder* m_right;
	ndNodeBuilder* m_parent;
	ndInt32 m_indexBox0;
	ndInt32 m_indexBox1;
	ndInt32 m_enumeration;
	ndInt32 m_faceIndex;
	ndInt32 m_indexCount;
	const ndInt32* m_faceIndices;
} D_GCC_NEWTON_ALIGN_32;

class ndAabbPolygonSoup::ndSplitInfo
{
	public:
	ndSplitInfo (ndNodeBuilder* const boxArray, ndInt32 boxCount)
	{
		ndVector minP ( ndFloat32 (1.0e15f)); 
		ndVector maxP (-ndFloat32 (1.0e15f)); 

		if (boxCount == 2) 
		{
			m_axis = 1;
			for (ndInt32 i = 0; i < boxCount; ++i) 
			{
				const ndNodeBuilder& box = boxArray[i];
				const ndVector& p0 = box.m_p0;
				const ndVector& p1 = box.m_p1;
				minP = minP.GetMin (p0); 
				maxP = maxP.GetMax (p1); 
			}

		} 
		else 
		{
			ndVector median (ndVector::m_zero);
			ndVector varian (ndVector::m_zero);
			for (ndInt32 i = 0; i < boxCount; ++i) 
			{
				const ndNodeBuilder& box = boxArray[i];

				const ndVector& p0 = box.m_p0;
				const ndVector& p1 = box.m_p1;

				minP = minP.GetMin (p0); 
				maxP = maxP.GetMax (p1); 
				ndVector p (ndVector::m_half * (p0 + p1));

				median += p;
				varian += p * p;
			}

			varian = varian.Scale (ndFloat32 (boxCount)) - median * median;

			ndInt32 index = 0;
			ndFloat32 maxVarian = ndFloat32 (-1.0e10f);
			for (ndInt32 i = 0; i < 3; ++i) 
			{
				if (varian[i] > maxVarian) 
				{
					index = i;
					maxVarian = varian[i];
				}
			}

			ndVector center = median.Scale (ndFloat32 (1.0f) / ndFloat32 (boxCount));
			ndFloat32 test = center[index];
			ndInt32 i0 = 0;
			ndInt32 i1 = boxCount - 1;
			do 
			{    
				for (; i0 <= i1; ++i0) 
				{
					const ndNodeBuilder& box = boxArray[i0];
					ndFloat32 val = (box.m_p0[index] + box.m_p1[index]) * ndFloat32 (0.5f);
					if (val > test) 
					{
						break;
					}
				}

				for (; i1 >= i0; --i1) 
				{
					const ndNodeBuilder& box = boxArray[i1];
					ndFloat32 val = (box.m_p0[index] + box.m_p1[index]) * ndFloat32 (0.5f);
					if (val < test) 
					{
						break;
					}
				}

				if (i0 < i1)
				{
					ndSwap(boxArray[i0], boxArray[i1]);
					i0++; 
					i1--;
				}
			} while (i0 <= i1);

			if (i0 > 0)
			{
				i0 --;
			}
			if ((i0 + 1) >= boxCount) 
			{
				i0 = boxCount - 2;
			}

			m_axis = i0 + 1;
		}

		ndAssert (maxP.m_x - minP.m_x >= ndFloat32 (0.0f));
		ndAssert (maxP.m_y - minP.m_y >= ndFloat32 (0.0f));
		ndAssert (maxP.m_z - minP.m_z >= ndFloat32 (0.0f));
		m_p0 = minP;
		m_p1 = maxP;
	}

	ndInt32 m_axis;
	ndVector m_p0;
	ndVector m_p1;
};

ndAabbPolygonSoup::ndAabbPolygonSoup ()
	:ndPolygonSoupDatabase()
	,m_aabb(nullptr)
	,m_indices(nullptr)
	,m_nodesCount(0)
	,m_indexCount(0)
{
}

ndAabbPolygonSoup::~ndAabbPolygonSoup ()
{
	if (m_aabb) 
	{
		ndMemory::Free(m_aabb);
		ndMemory::Free(m_indices);
	}
}

ndFloat32 ndAabbPolygonSoup::CalculateFaceMaxDiagonal (const ndVector* const vertex, ndInt32 indexCount, const ndInt32* const indexArray) const
{
	ndFloat32 maxSize = ndFloat32 (0.0f);
	ndInt32 index = indexArray[indexCount - 1];
	ndVector p0 (vertex[index]);
	for (ndInt32 i = 0; i < indexCount; ++i) 
	{
		ndInt32 index1 = indexArray[i];
		ndVector p1 (vertex[index1]);

		ndVector dir (p1 - p0);
		ndAssert (dir.m_w == ndFloat32 (0.0f));
		dir = dir.Normalize();

		ndFloat32 maxVal = ndFloat32 (-1.0e10f);
		ndFloat32 minVal = ndFloat32 ( 1.0e10f);
		for (ndInt32 j = 0; j < indexCount; ++j) 
		{
			ndInt32 index2 = indexArray[j];
			ndVector q (vertex[index2]);
			ndFloat32 val = dir.DotProduct(q).GetScalar();
			minVal = ndMin(minVal, val);
			maxVal = ndMax(maxVal, val);
		}

		ndFloat32 size = maxVal - minVal;
		maxSize = ndMax(maxSize, size);
		p0 = p1;
	}

	return maxSize;
}

void ndAabbPolygonSoup::GetAABB (ndVector& p0, ndVector& p1) const
{
	if (m_aabb) 
	{ 
		GetNodeAabb (m_aabb, p0, p1);
	} 
	else 
	{
		p0 = ndVector::m_zero;
		p1 = ndVector::m_zero;
	}
}

void ndAabbPolygonSoup::CalculateAdjacent ()
{
	ndVector p0;
	ndVector p1;
	GetAABB (p0, p1);
	ndFastAabb box (p0, p1);

	ndPolyhedra adjacentMesh;
	adjacentMesh.BeginFace();
	ForAllSectors(box, ndVector::m_zero, ndFloat32(1.0f), CalculateAllFaceEdgeNormals, &adjacentMesh);
	adjacentMesh.EndFace();

	ndInt32 mark = adjacentMesh.IncLRU();
	ndPolyhedra::Iterator iter(adjacentMesh);
	const ndTriplex* const vertexArray = (ndTriplex*)GetLocalVertexPool();
	for (iter.Begin(); iter; iter++)
	{
		ndEdge* const edge = &(*iter);
		if ((edge->m_mark != mark) && (edge->m_incidentFace >= 0) && (edge->m_twin->m_incidentFace >= 0))
		{
			ndInt32 indexCount0 = 0;
			ndInt32 indexCount1 = 0;
			ndInt32 offsetIndex0 = -1;
			ndInt32 offsetIndex1 = -1;
			ndInt32* const indexArray0 = (ndInt32*)edge->m_userData;
			ndInt32* const indexArray1 = (ndInt32*)edge->m_twin->m_userData;
			ndEdge* ptr = edge;
			do
			{
				indexCount0++;
				ptr = ptr->m_next;
			} while (ptr != edge);

			ptr = edge->m_twin;
			do
			{
				indexCount1++;
				ptr = ptr->m_next;
			} while (ptr != edge->m_twin);

			ndVector n0(&vertexArray[indexArray0[indexCount0 + 1]].m_x);
			ndVector q0(&vertexArray[indexArray0[0]].m_x);
			n0 = n0 & ndVector::m_triplexMask;
			q0 = q0 & ndVector::m_triplexMask;

			ndVector n1(&vertexArray[indexArray1[indexCount1 + 1]].m_x);
			ndVector q1(&vertexArray[indexArray1[0]].m_x);
			n1 = n1 & ndVector::m_triplexMask;
			q1 = q1 & ndVector::m_triplexMask;

			ndPlane plane0(n0, -n0.DotProduct(q0).GetScalar());
			ndPlane plane1(n1, -n1.DotProduct(q1).GetScalar());

			ndFloat32 maxDist0 = ndFloat32(-1.0f);
			for (ndInt32 i = 0; i < indexCount1; ++i)
			{
				if (edge->m_twin->m_incidentVertex == indexArray1[i])
				{
					offsetIndex1 = i;
				}
				ndVector point(&vertexArray[indexArray1[i]].m_x);
				ndFloat32 dist(plane0.Evalue(point & ndVector::m_triplexMask));
				maxDist0 = ndMax(maxDist0, dist);
			}

			ndFloat32 maxDist1 = ndFloat32(-1.0f);
			for (ndInt32 i = 0; i < indexCount0; ++i)
			{
				if (edge->m_incidentVertex == indexArray0[i])
				{
					offsetIndex0 = i;
				}
				ndVector point(&vertexArray[indexArray0[i]].m_x);
				ndFloat32 dist(plane1.Evalue(point & ndVector::m_triplexMask));
				maxDist1 = ndMax(maxDist1, dist);
			}

			bool edgeIsConvex = (maxDist0 <= ndFloat32(1.0e-3f));
			edgeIsConvex = edgeIsConvex && (maxDist1 <= ndFloat32(1.0e-3f));
			edgeIsConvex = edgeIsConvex || (n0.DotProduct(n1).GetScalar() > ndFloat32(0.9991f));

			//hacks for testing adjacency
			//edgeIsConvex = edgeIsConvex || (n0.DotProduct(n1).GetScalar() > ndFloat32(0.5f));
			//edgeIsConvex = true;
			if (edgeIsConvex)
			{
				ndAssert(offsetIndex0 >= 0);
				ndAssert(offsetIndex1 >= 0);
				indexArray0[indexCount0 + 2 + offsetIndex0] = indexArray1[indexCount1 + 1];
				indexArray1[indexCount1 + 2 + offsetIndex1] = indexArray0[indexCount0 + 1];
			}
		}
		edge->m_mark = mark;
		edge->m_twin->m_mark = mark;
	}

	ndStack<ndTriplex> pool ((m_indexCount / 2) - 1);
	ndInt32 normalCount = 0;
	for (ndInt32 i = 0; i < m_nodesCount; ++i) 
	{
		const ndNode* const node = &m_aabb[i];
		if (node->m_left.IsLeaf()) 
		{
			ndInt32 vCount = ndInt32 (node->m_left.GetCount());
			if (vCount) 
			{
				ndInt32 index = ndInt32 (node->m_left.GetIndex());
				ndInt32* const face = &m_indices[index];
	
				ndInt32 j0 = 2 * (vCount + 1) - 1;
				ndVector normal (&vertexArray[face[vCount + 1]].m_x);
				normal = normal & ndVector::m_triplexMask;
				ndAssert (ndAbs (normal.DotProduct(normal).GetScalar() - ndFloat32 (1.0f)) < ndFloat32 (1.0e-6f));
				ndVector q0 (&vertexArray[face[vCount - 1]].m_x);
				q0 = q0 & ndVector::m_triplexMask;
				for (ndInt32 j = 0; j < vCount; ++j) 
				{
					ndInt32 j1 = vCount + 2 + j;
					ndVector q1 (&vertexArray[face[j]].m_x);
					q1 = q1 & ndVector::m_triplexMask;
					if (face[j0] & D_CONCAVE_EDGE_MASK)
					{
						ndVector e (q1 - q0);
						ndVector n (e.CrossProduct(normal).Normalize());
						ndAssert (ndAbs (n.DotProduct(n).GetScalar() - ndFloat32 (1.0f)) < ndFloat32 (1.0e-6f));
						pool[normalCount].m_x = n.m_x;
						pool[normalCount].m_y = n.m_y;
						pool[normalCount].m_z = n.m_z;
						face[j0] = normalCount | D_CONCAVE_EDGE_MASK;
						normalCount ++;
					}
					q0 = q1;
					j0 = j1;
				}
			}
		}
	
		if (node->m_right.IsLeaf()) 
		{
			ndInt32 vCount = ndInt32 (node->m_right.GetCount());
			if (vCount) 
			{
				ndInt32 index = ndInt32 (node->m_right.GetIndex());
				ndInt32* const face = &m_indices[index];
	
				ndInt32 j0 = 2 * (vCount + 1) - 1;
				ndVector normal (&vertexArray[face[vCount + 1]].m_x);
				normal = normal & ndVector::m_triplexMask;
				ndAssert (ndAbs (normal.DotProduct(normal).GetScalar() - ndFloat32 (1.0f)) < ndFloat32 (1.0e-6f));
				ndVector q0 (&vertexArray[face[vCount - 1]].m_x);
				q0 = q0 & ndVector::m_triplexMask;
				for (ndInt32 j = 0; j < vCount; ++j) 
				{
					ndInt32 j1 = vCount + 2 + j;
					ndVector q1 (&vertexArray[face[j]].m_x);
					q1 = q1 & ndVector::m_triplexMask;
					if (face[j0] & D_CONCAVE_EDGE_MASK)
					{
						ndVector e (q1 - q0);
						ndVector n (e.CrossProduct(normal).Normalize());
						ndAssert (ndAbs (n.DotProduct(n).GetScalar() - ndFloat32 (1.0f)) < ndFloat32 (1.0e-6f));
						pool[normalCount].m_x = n.m_x;
						pool[normalCount].m_y = n.m_y;
						pool[normalCount].m_z = n.m_z;
						face[j0] = normalCount | D_CONCAVE_EDGE_MASK;
						normalCount ++;
					}
					q0 = q1;
					j0 = j1;
				}
			}
		}
	}
	
	if (normalCount) 
	{
		ndStack<ndInt32> indexArray (normalCount);
		ndInt32 newNormalCount = ndVertexListToIndexList (&pool[0].m_x, sizeof (ndTriplex), 3, normalCount, &indexArray[0], ndFloat32 (1.0e-6f));
	
		ndInt32 oldCount = GetVertexCount();
		ndTriplex* const vertexArray1 = (ndTriplex*)ndMemory::Malloc (sizeof (ndTriplex) * (oldCount + newNormalCount));
		ndMemCpy(vertexArray1, (ndTriplex*)GetLocalVertexPool(), oldCount);
		ndMemCpy(&vertexArray1[oldCount], &pool[0], newNormalCount);

		ndMemory::Free(GetLocalVertexPool());
	
		m_localVertex = &vertexArray1[0].m_x;
		m_vertexCount = oldCount + newNormalCount;
	
		for (ndInt32 i = 0; i < m_nodesCount; ++i) 
		{
			const ndNode* const node = &m_aabb[i];
			if (node->m_left.IsLeaf()) 
			{
				ndInt32 vCount = ndInt32 (node->m_left.GetCount());
				ndInt32 index = ndInt32 (node->m_left.GetIndex());
				ndInt32* const face = &m_indices[index];
				for (ndInt32 j = 0; j < vCount; ++j) 
				{
					ndInt32 edgeIndexNormal = face[vCount + 2 + j];
					if (edgeIndexNormal & D_CONCAVE_EDGE_MASK)
					{
						ndInt32 k = edgeIndexNormal & (~D_CONCAVE_EDGE_MASK);
						face[vCount + 2 + j] = (indexArray[k] + oldCount) | D_CONCAVE_EDGE_MASK;
					}
					#ifdef _DEBUG	
						//ndVector normal (&vertexArray1[face[vCount + 2 + j]].m_x);
						ndVector normal (&vertexArray1[face[vCount + 2 + j] & (~D_CONCAVE_EDGE_MASK)].m_x);
						normal = normal & ndVector::m_triplexMask;
						ndAssert (ndAbs (normal.DotProduct(normal).GetScalar() - ndFloat32 (1.0f)) < ndFloat32 (1.0e-6f));
					#endif
				}
			}
	
			if (node->m_right.IsLeaf()) 
			{
				ndInt32 vCount = ndInt32 (node->m_right.GetCount());
				ndInt32 index = ndInt32 (node->m_right.GetIndex());
				ndInt32* const face = &m_indices[index];
				for (ndInt32 j = 0; j < vCount; ++j) 
				{
					ndInt32 edgeIndexNormal = face[vCount + 2 + j];
					if (edgeIndexNormal & D_CONCAVE_EDGE_MASK)
					{
						ndInt32 k = edgeIndexNormal & (~D_CONCAVE_EDGE_MASK);
						face[vCount + 2 + j] = (indexArray[k] + oldCount) | D_CONCAVE_EDGE_MASK;
					}
	
					#ifdef _DEBUG	
						//ndVector normal (&vertexArray1[face[vCount + 2 + j]].m_x);
						ndVector normal(&vertexArray1[face[vCount + 2 + j] & (~D_CONCAVE_EDGE_MASK)].m_x);
						normal = normal & ndVector::m_triplexMask;
						ndAssert (ndAbs (normal.DotProduct(normal).GetScalar() - ndFloat32 (1.0f)) < ndFloat32 (1.0e-6f));
					#endif
				}
			}
		}
	}
}

ndIntersectStatus ndAabbPolygonSoup::CalculateAllFaceEdgeNormals(void* const context, const ndFloat32* const, ndInt32, const ndInt32* const indexArray, ndInt32 indexCount, ndFloat32)
{
	ndInt32 face[256];
	ndInt64 data[256];
	ndPolyhedra& adjacency = *((ndPolyhedra*)context);

	ndIntPtr userData;
	userData.m_ptr = (void*)indexArray;
	for (ndInt32 i = 0; i < indexCount; ++i)
	{
		face[i] = indexArray[i];
		data[i] = userData.m_int;
	}
	adjacency.AddFace(indexCount, face, data);
	return m_continueSearh;
}

ndAabbPolygonSoup::ndNodeBuilder* ndAabbPolygonSoup::BuildTopDown (ndNodeBuilder* const leafArray, ndInt32 firstBox, ndInt32 lastBox, ndNodeBuilder** const allocator) const
{
	ndAssert (firstBox >= 0);
	ndAssert (lastBox >= 0);

	if (lastBox == firstBox) 
	{
		return &leafArray[firstBox];
	} 
	else 
	{
		ndSplitInfo info (&leafArray[firstBox], lastBox - firstBox + 1);

		ndNodeBuilder* const parent = new (*allocator) ndNodeBuilder (info.m_p0, info.m_p1);
		*allocator = *allocator + 1;

		ndAssert (parent);
		parent->m_right = BuildTopDown (leafArray, firstBox + info.m_axis, lastBox, allocator);
		parent->m_right->m_parent = parent;

		parent->m_left = BuildTopDown (leafArray, firstBox, firstBox + info.m_axis - 1, allocator);
		parent->m_left->m_parent = parent;
		return parent;
	}
}

void ndAabbPolygonSoup::Create (const ndPolygonSoupBuilder& builder)
{
	if (builder.m_faceVertexCount.GetCount() == 0) 
	{
		return;
	}
	ndAssert (builder.m_faceVertexCount.GetCount() >= 1);
	m_strideInBytes = sizeof (ndTriplex);
	m_nodesCount = ((builder.m_faceVertexCount.GetCount() - 1) < 1) ? 1 : ndInt32(builder.m_faceVertexCount.GetCount()) - 1;
	m_aabb = (ndNode*) ndMemory::Malloc (sizeof (ndNode) * m_nodesCount);
	m_indexCount = ndInt32(builder.m_vertexIndex.GetCount() * 2 + builder.m_faceVertexCount.GetCount());

	if (builder.m_faceVertexCount.GetCount() == 1) 
	{
		m_indexCount *= 2;
	}

	m_indices = (ndInt32*) ndMemory::Malloc (sizeof (ndInt32) * m_indexCount);
	ndStack<ndVector> tmpVertexArrayCount(ndInt32(builder.m_vertexPoints.GetCount() + builder.m_normalPoints.GetCount() + builder.m_faceVertexCount.GetCount() * 2 + 4));

	ndVector* const tmpVertexArray = &tmpVertexArrayCount[0];
	for (ndInt32 i = 0; i < builder.m_vertexPoints.GetCount(); ++i) 
	{
		tmpVertexArray[i] = builder.m_vertexPoints[i];
	}

	for (ndInt32 i = 0; i < builder.m_normalPoints.GetCount(); ++i) 
	{
		tmpVertexArray[i + builder.m_vertexPoints.GetCount()] = builder.m_normalPoints[i];
	}

	const ndInt32* const indices = &builder.m_vertexIndex[0];
	ndStack<ndNodeBuilder> constructor (ndInt32(builder.m_faceVertexCount.GetCount() * 2 + 16));

	ndInt32 polygonIndex = 0;
	ndInt32 allocatorIndex = 0;
	if (builder.m_faceVertexCount.GetCount() == 1) 
	{
		ndInt32 indexCount = builder.m_faceVertexCount[0] - 1;
		new (&constructor[allocatorIndex]) ndNodeBuilder (&tmpVertexArray[0], 0, indexCount, &indices[0]);
		allocatorIndex ++;
	}
	for (ndInt32 i = 0; i < builder.m_faceVertexCount.GetCount(); ++i) 
	{
		ndInt32 indexCount = builder.m_faceVertexCount[i] - 1;
		new (&constructor[allocatorIndex]) ndNodeBuilder (&tmpVertexArray[0], i, indexCount, &indices[polygonIndex]);
		allocatorIndex ++;
		polygonIndex += (indexCount + 1);
	}

	ndNodeBuilder* constructorAllocator = &constructor[allocatorIndex];
	ndNodeBuilder* const root = BuildTopDown (&constructor[0], 0, allocatorIndex - 1, &constructorAllocator);

	ndAssert (root);
	ndList<ndNodeBuilder*> list;
	list.Append(root);
	ndInt32 nodeIndex = 0;
	while (list.GetCount())
	{
		ndNodeBuilder* const node = list.GetFirst()->GetInfo();
		list.Remove(list.GetFirst());

		if (node->m_left) 
		{
			node->m_enumeration = nodeIndex;
			nodeIndex ++;
			ndAssert (node->m_right);
			list.Append(node->m_left);
			list.Append(node->m_right);
		}
	}
	ndAssert(!list.GetCount());

	ndInt32 aabbBase = ndInt32(builder.m_vertexPoints.GetCount() + builder.m_normalPoints.GetCount());

	ndVector* const aabbPoints = &tmpVertexArray[aabbBase];

	ndInt32 vertexIndex = 0;
	ndInt32 aabbNodeIndex = 0;
	list.Append(root);
	ndInt32 indexMap = 0;
	while (list.GetCount())
	{
		ndNodeBuilder* const node = list.GetFirst()->GetInfo();
		list.Remove(list.GetFirst());

		if (node->m_enumeration >= 0)
		{
			ndAssert (node->m_left);
			ndAssert (node->m_right);
			ndNode& aabbNode = m_aabb[aabbNodeIndex];
			aabbNodeIndex ++;
			ndAssert (aabbNodeIndex <= m_nodesCount);

			if (node->m_parent)
			{
				if (node->m_parent->m_left == node)
				{
					m_aabb[node->m_parent->m_enumeration].m_left = ndNode::ndLeafNodePtr (ndUnsigned32 (&m_aabb[node->m_enumeration] - m_aabb));
				}
				else 
				{
					ndAssert (node->m_parent->m_right == node);
					m_aabb[node->m_parent->m_enumeration].m_right = ndNode::ndLeafNodePtr (ndUnsigned32 (&m_aabb[node->m_enumeration] - m_aabb));
				}
			}

			aabbPoints[vertexIndex + 0] = node->m_p0;
			aabbPoints[vertexIndex + 1] = node->m_p1;

			aabbNode.m_indexBox0 = aabbBase + vertexIndex;
			aabbNode.m_indexBox1 = aabbBase + vertexIndex + 1;

			vertexIndex += 2;
		}
		else
		{
			ndAssert (!node->m_left);
			ndAssert (!node->m_right);

			if (node->m_parent)
			{
				if (node->m_parent->m_left == node)
				{
					m_aabb[node->m_parent->m_enumeration].m_left = ndNode::ndLeafNodePtr (ndUnsigned32(node->m_indexCount), ndUnsigned32(indexMap));
				}
				else 
				{
					ndAssert (node->m_parent->m_right == node);
					m_aabb[node->m_parent->m_enumeration].m_right = ndNode::ndLeafNodePtr (ndUnsigned32(node->m_indexCount), ndUnsigned32(indexMap));
				}
			}

			// index format i0, i1, i2, ... , id, normal, e0Normal, e1Normal, e2Normal, ..., faceSize
			for (ndInt32 j = 0; j < node->m_indexCount; ++j) 
			{
				m_indices[indexMap + j] = node->m_faceIndices[j];
				m_indices[indexMap + j + node->m_indexCount + 2] = D_CONCAVE_EDGE_MASK | 0xffffffff;
			}

			// face attribute
			m_indices[indexMap + node->m_indexCount] = node->m_faceIndices[node->m_indexCount];
			// face normal
			m_indices[indexMap + node->m_indexCount + 1] = ndInt32(builder.m_vertexPoints.GetCount()) + builder.m_normalIndex[node->m_faceIndex];
			// face size
			ndFloat32 faceMaxDiag = CalculateFaceMaxDiagonal(&tmpVertexArray[0], node->m_indexCount, node->m_faceIndices);
			ndInt32 quantizedDiagSize = ndInt32(ndFloor(faceMaxDiag / D_FACE_CLIP_DIAGONAL_SCALE + ndFloat32(1.0f)));
			m_indices[indexMap + node->m_indexCount * 2 + 2] = quantizedDiagSize;

			indexMap += node->m_indexCount * 2 + 3;
		}

		if (node->m_left) 
		{
			ndAssert (node->m_right);
			list.Append(node->m_left);
			list.Append(node->m_right);
		}
	}

	ndStack<ndInt32> indexArray (vertexIndex);
	ndInt32 aabbPointCount = ndVertexListToIndexList (&aabbPoints[0].m_x, sizeof (ndVector), 3, vertexIndex, &indexArray[0], ndFloat32 (1.0e-6f));

	m_vertexCount = aabbBase + aabbPointCount;
	m_localVertex = (ndFloat32*) ndMemory::Malloc (sizeof (ndTriplex) * m_vertexCount);

	ndTriplex* const dstPoints = (ndTriplex*)m_localVertex;
	for (ndInt32 i = 0; i < m_vertexCount; ++i) 
	{
		dstPoints[i].m_x = tmpVertexArray[i].m_x;
		dstPoints[i].m_y = tmpVertexArray[i].m_y;
		dstPoints[i].m_z = tmpVertexArray[i].m_z;
	}

	for (ndInt32 i = 0; i < m_nodesCount; ++i) 
	{
		ndNode& box = m_aabb[i];

		ndInt32 j = box.m_indexBox0 - aabbBase;
		box.m_indexBox0 = indexArray[j] + aabbBase;

		j = box.m_indexBox1 - aabbBase;
		box.m_indexBox1 = indexArray[j] + aabbBase;
	}

	if (builder.m_faceVertexCount.GetCount() == 1) 
	{
		m_aabb[0].m_right = ndNode::ndLeafNodePtr (0, 0);
	}
}

void ndAabbPolygonSoup::Serialize (const char* const path) const
{
	FILE* const file = fopen(path, "wb");
	if (file)
	{
		fwrite(&m_vertexCount, sizeof(ndInt32), 1, file);
		fwrite(&m_indexCount, sizeof(ndInt32), 1, file);
		fwrite(&m_nodesCount, sizeof(ndInt32), 1, file);
		if (m_aabb)
		{
			fwrite(m_localVertex, sizeof(ndTriplex) * m_vertexCount, 1, file);
			fwrite(m_indices, sizeof(ndInt32) * m_indexCount, 1, file);
			fwrite(m_aabb, sizeof(ndNode) * m_nodesCount, 1, file);
		}
		fclose(file);
	}
}

void ndAabbPolygonSoup::Deserialize (const char* const path)
{
	FILE* const file = fopen(path, "rb");
	if (file)
	{
		size_t readValues = 0; 
		readValues++;
		m_strideInBytes = sizeof(ndTriplex);
		readValues = fread(&m_vertexCount, sizeof(ndInt32), 1, file);
		readValues = fread(&m_indexCount, sizeof(ndInt32), 1, file);
		readValues = fread(&m_nodesCount, sizeof(ndInt32), 1, file);

		if (m_vertexCount) 
		{
			m_localVertex = (ndFloat32*)ndMemory::Malloc(sizeof(ndTriplex) * m_vertexCount);
			m_indices = (ndInt32*)ndMemory::Malloc(sizeof(ndInt32) * m_indexCount);
			m_aabb = (ndNode*)ndMemory::Malloc(sizeof(ndNode) * m_nodesCount);

			readValues = fread(m_localVertex, sizeof(ndTriplex) * m_vertexCount, 1, file);
			readValues = fread(m_indices, sizeof(ndInt32) * m_indexCount, 1, file);
			readValues = fread(m_aabb, sizeof(ndNode) * m_nodesCount, 1, file);
		}
		else 
		{
			m_localVertex = nullptr;
			m_indices = nullptr;
			m_aabb = nullptr;
		}

		fclose(file);
	}
}

ndVector ndAabbPolygonSoup::ForAllSectorsSupportVertex (const ndVector& dir) const
{
	ndVector supportVertex (ndFloat32 (0.0f));
	if (m_aabb) 
	{
		ndFloat32 aabbProjection[DG_STACK_DEPTH];
		const ndNode *stackPool[DG_STACK_DEPTH];

		ndInt32 stack = 1;
		stackPool[0] = m_aabb;
		aabbProjection[0] = ndFloat32 (1.0e10f);
		const ndTriplex* const boxArray = (ndTriplex*)m_localVertex;

		ndFloat32 maxProj = ndFloat32 (-1.0e20f); 
		ndInt32 ix = (dir[0] > ndFloat32 (0.0f)) ? 1 : 0;
		ndInt32 iy = (dir[1] > ndFloat32 (0.0f)) ? 1 : 0;
		ndInt32 iz = (dir[2] > ndFloat32 (0.0f)) ? 1 : 0;

		while (stack) 
		{
			ndFloat32 boxSupportValue;

			stack--;
			boxSupportValue = aabbProjection[stack];
			if (boxSupportValue > maxProj) {
				ndFloat32 backSupportDist = ndFloat32 (0.0f);
				ndFloat32 frontSupportDist = ndFloat32 (0.0f);
				const ndNode* const me = stackPool[stack];
				if (me->m_left.IsLeaf()) 
				{
					backSupportDist = ndFloat32 (-1.0e20f);
					ndInt32 index = ndInt32 (me->m_left.GetIndex());
					ndInt32 vCount = ndInt32 (me->m_left.GetCount());
					ndVector vertex (ndFloat32 (0.0f));
					for (ndInt32 j = 0; j < vCount; ++j) 
					{
						ndInt32 i0 = m_indices[index + j] * ndInt32 (sizeof (ndTriplex) / sizeof (ndFloat32));
						ndVector p (&boxArray[i0].m_x);
						p = p & ndVector::m_triplexMask;
						ndFloat32 dist = p.DotProduct(dir).GetScalar();
						if (dist > backSupportDist) 
						{
							backSupportDist = dist;
							vertex = p;
						}
					}

					if (backSupportDist > maxProj) 
					{
						maxProj = backSupportDist;
						supportVertex = vertex; 
					}

				} 
				else 
				{
					ndVector box[2];
					const ndNode* const node = me->m_left.GetNode(m_aabb);
					box[0].m_x = boxArray[node->m_indexBox0].m_x;
					box[0].m_y = boxArray[node->m_indexBox0].m_y;
					box[0].m_z = boxArray[node->m_indexBox0].m_z;
					box[1].m_x = boxArray[node->m_indexBox1].m_x;
					box[1].m_y = boxArray[node->m_indexBox1].m_y;
					box[1].m_z = boxArray[node->m_indexBox1].m_z;

					ndVector supportPoint (box[ix].m_x, box[iy].m_y, box[iz].m_z, ndFloat32 (0.0));
					backSupportDist = supportPoint.DotProduct(dir).GetScalar();
				}

				if (me->m_right.IsLeaf()) 
				{
					frontSupportDist = ndFloat32 (-1.0e20f);
					ndInt32 index = ndInt32 (me->m_right.GetIndex());
					ndInt32 vCount = ndInt32 (me->m_right.GetCount());
					ndVector vertex (ndFloat32 (0.0f));
					for (ndInt32 j = 0; j < vCount; ++j) 
					{
						ndInt32 i0 = m_indices[index + j] * ndInt32 (sizeof (ndTriplex) / sizeof (ndFloat32));
						ndVector p (&boxArray[i0].m_x);
						p = p & ndVector::m_triplexMask;
						ndFloat32 dist = p.DotProduct(dir).GetScalar();
						if (dist > frontSupportDist) 
						{
							frontSupportDist = dist;
							vertex = p;
						}
					}
					if (frontSupportDist > maxProj) 
					{
						maxProj = frontSupportDist;
						supportVertex = vertex; 
					}

				} 
				else 
				{
					ndVector box[2];
					const ndNode* const node = me->m_right.GetNode(m_aabb);
					box[0].m_x = boxArray[node->m_indexBox0].m_x;
					box[0].m_y = boxArray[node->m_indexBox0].m_y;
					box[0].m_z = boxArray[node->m_indexBox0].m_z;
					box[1].m_x = boxArray[node->m_indexBox1].m_x;
					box[1].m_y = boxArray[node->m_indexBox1].m_y;
					box[1].m_z = boxArray[node->m_indexBox1].m_z;

					ndVector supportPoint (box[ix].m_x, box[iy].m_y, box[iz].m_z, ndFloat32 (0.0f));
					frontSupportDist = supportPoint.DotProduct(dir).GetScalar();
				}

				if (frontSupportDist >= backSupportDist) 
				{
					if (!me->m_left.IsLeaf()) 
					{
						aabbProjection[stack] = backSupportDist;
						stackPool[stack] = me->m_left.GetNode(m_aabb);
						stack++;
					}

					if (!me->m_right.IsLeaf()) 
					{
						aabbProjection[stack] = frontSupportDist;
						stackPool[stack] = me->m_right.GetNode(m_aabb);
						stack++;
					}

				} 
				else 
				{
					if (!me->m_right.IsLeaf()) 
					{
						aabbProjection[stack] = frontSupportDist;
						stackPool[stack] = me->m_right.GetNode(m_aabb);
						stack++;
					}

					if (!me->m_left.IsLeaf()) 
					{
						aabbProjection[stack] = backSupportDist;
						stackPool[stack] = me->m_left.GetNode(m_aabb);
						stack++;
					}
				}
			}
		}
	}
	return supportVertex;
}

void ndAabbPolygonSoup::ForAllSectorsRayHit (const ndFastRay& raySrc, ndFloat32 maxParam, ndRayIntersectCallback callback, void* const context) const
{
	const ndNode *stackPool[DG_STACK_DEPTH];
	ndFloat32 distance[DG_STACK_DEPTH];
	ndFastRay ray (raySrc);

	ndInt32 stack = 1;
	const ndTriplex* const vertexArray = (ndTriplex*) m_localVertex;

	stackPool[0] = m_aabb;
	distance[0] = m_aabb->RayDistance(ray, vertexArray);
	while (stack) 
	{
		stack --;
		ndFloat32 dist = distance[stack];
		if (dist > maxParam) 
		{
			break;
		} 
		else 
		{
			const ndNode *const me = stackPool[stack];
			if (me->m_left.IsLeaf()) 
			{
				ndInt32 vCount = ndInt32 (me->m_left.GetCount());
				if (vCount > 0) 
				{
					ndInt32 index = ndInt32 (me->m_left.GetIndex());
					ndFloat32 param = callback(context, &vertexArray[0].m_x, sizeof (ndTriplex), &m_indices[index], vCount);
					ndAssert (param >= ndFloat32 (0.0f));
					if (param < maxParam) 
					{
						maxParam = param;
						if (maxParam == ndFloat32 (0.0f)) 
						{
							break;
						}
					}
				}

			} 
			else 
			{
				const ndNode* const node = me->m_left.GetNode(m_aabb);
				ndFloat32 dist1 = node->RayDistance(ray, vertexArray);
				if (dist1 < maxParam) 
				{
					ndInt32 j = stack;
					for ( ; j && (dist1 > distance[j - 1]); j --) 
					{
						stackPool[j] = stackPool[j - 1];
						distance[j] = distance[j - 1];
					}
					ndAssert (stack < DG_STACK_DEPTH);
					stackPool[j] = node;
					distance[j] = dist1;
					stack++;
				}
			}

			if (me->m_right.IsLeaf()) 
			{
				ndInt32 vCount = ndInt32 (me->m_right.GetCount());
				if (vCount > 0) 
				{
					ndInt32 index = ndInt32 (me->m_right.GetIndex());
					ndFloat32 param = callback(context, &vertexArray[0].m_x, sizeof (ndTriplex), &m_indices[index], vCount);
					ndAssert (param >= ndFloat32 (0.0f));
					if (param < maxParam) 
					{
						maxParam = param;
						if (maxParam == ndFloat32 (0.0f)) 
						{
							break;
						}
					}
				}
			} 
			else 
			{
				const ndNode* const node = me->m_right.GetNode(m_aabb);
				ndFloat32 dist1 = node->RayDistance(ray, vertexArray);
				if (dist1 < maxParam) 
				{
					ndInt32 j = stack;
					for ( ; j && (dist1 > distance[j - 1]); j --) 
					{
						stackPool[j] = stackPool[j - 1];
						distance[j] = distance[j - 1];
					}
					ndAssert (stack < DG_STACK_DEPTH);
					stackPool[j] = node;
					distance[j] = dist1;
					stack++;
				}
			}
		}
	}
}

void ndAabbPolygonSoup::ForAllSectors (const ndFastAabb& obbAabbInfo, const ndVector& boxDistanceTravel, ndFloat32, ndAaabbIntersectCallback callback, void* const context) const
{
	ndAssert (ndAbs(ndAbs(obbAabbInfo[0][0]) - obbAabbInfo.m_absDir[0][0]) < ndFloat32 (1.0e-4f));
	ndAssert (ndAbs(ndAbs(obbAabbInfo[1][1]) - obbAabbInfo.m_absDir[1][1]) < ndFloat32 (1.0e-4f));
	ndAssert (ndAbs(ndAbs(obbAabbInfo[2][2]) - obbAabbInfo.m_absDir[2][2]) < ndFloat32 (1.0e-4f));

	ndAssert (ndAbs(ndAbs(obbAabbInfo[0][1]) - obbAabbInfo.m_absDir[1][0]) < ndFloat32 (1.0e-4f));
	ndAssert (ndAbs(ndAbs(obbAabbInfo[0][2]) - obbAabbInfo.m_absDir[2][0]) < ndFloat32 (1.0e-4f));
	ndAssert (ndAbs(ndAbs(obbAabbInfo[1][2]) - obbAabbInfo.m_absDir[2][1]) < ndFloat32 (1.0e-4f));

	if (m_aabb) 
	{
		ndFloat32 distance[DG_STACK_DEPTH];
		const ndNode* stackPool[DG_STACK_DEPTH];

		const ndInt32 stride = sizeof (ndTriplex) / sizeof (ndFloat32);
		const ndTriplex* const vertexArray = (ndTriplex*) m_localVertex;

		ndAssert (boxDistanceTravel.m_w == ndFloat32 (0.0f));
		if (boxDistanceTravel.DotProduct(boxDistanceTravel).GetScalar() < ndFloat32 (1.0e-8f)) 
		{
			ndInt32 stack = 1;
			stackPool[0] = m_aabb;
			distance[0] = m_aabb->BoxPenetration(obbAabbInfo, vertexArray);
			if (distance[0] <= ndFloat32(0.0f)) 
			{
				obbAabbInfo.m_separationDistance = ndMin(obbAabbInfo.m_separationDistance[0], -distance[0]);
			}
			while (stack) 
			{
				stack --;
				ndFloat32 dist = distance[stack];
				if (dist > ndFloat32 (0.0f)) 
				{
					const ndNode* const me = stackPool[stack];
					if (me->m_left.IsLeaf()) 
					{
						ndInt32 index = ndInt32 (me->m_left.GetIndex());
						ndInt32 vCount = ndInt32 (me->m_left.GetCount());
						if (vCount > 0) 
						{
							const ndInt32* const indices = &m_indices[index];
							ndInt32 normalIndex = indices[vCount + 1];
							ndVector faceNormal (&vertexArray[normalIndex].m_x);
							faceNormal = faceNormal & ndVector::m_triplexMask;
							ndFloat32 dist1 = obbAabbInfo.PolygonBoxDistance (faceNormal, vCount, indices, stride, &vertexArray[0].m_x);
							if (dist1 > ndFloat32 (0.0f)) 
							{
								obbAabbInfo.m_separationDistance = ndFloat32(0.0f);
								ndAssert (vCount >= 3);
								if (callback(context, &vertexArray[0].m_x, sizeof (ndTriplex), indices, vCount, dist1) == m_stopSearch) 
								{
									return;
								}
							} 
							else 
							{
								obbAabbInfo.m_separationDistance = ndMin(obbAabbInfo.m_separationDistance[0], -dist1);
							}
						}
					} 
					else 
					{
						const ndNode* const node = me->m_left.GetNode(m_aabb);
						ndFloat32 dist1 = node->BoxPenetration(obbAabbInfo, vertexArray);
						if (dist1 > ndFloat32 (0.0f)) 
						{
							ndInt32 j = stack;
							for ( ; j && (dist1 > distance[j - 1]); j --) 
							{
								stackPool[j] = stackPool[j - 1];
								distance[j] = distance[j - 1];
							}
							ndAssert (stack < DG_STACK_DEPTH);
							stackPool[j] = node;
							distance[j] = dist1;
							stack++;
						} 
						else 
						{
							obbAabbInfo.m_separationDistance = ndMin(obbAabbInfo.m_separationDistance[0], -dist1);
						}
					}

					if (me->m_right.IsLeaf()) 
					{
						ndInt32 index = ndInt32 (me->m_right.GetIndex());
						ndInt32 vCount = ndInt32 (me->m_right.GetCount());
						if (vCount > 0) 
						{
							const ndInt32* const indices = &m_indices[index];
							ndInt32 normalIndex = indices[vCount + 1];
							ndVector faceNormal (&vertexArray[normalIndex].m_x);
							faceNormal = faceNormal & ndVector::m_triplexMask;
							ndFloat32 dist1 = obbAabbInfo.PolygonBoxDistance (faceNormal, vCount, indices, stride, &vertexArray[0].m_x);
							if (dist1 > ndFloat32 (0.0f)) 
							{
								ndAssert (vCount >= 3);
								obbAabbInfo.m_separationDistance = ndFloat32(0.0f);
								if (callback(context, &vertexArray[0].m_x, sizeof (ndTriplex), indices, vCount, dist1) == m_stopSearch) 
								{
									return;
								}
							} 
							else 
							{
								obbAabbInfo.m_separationDistance = ndMin(obbAabbInfo.m_separationDistance[0], -dist1);
							}
						}
					} 
					else 
					{
						const ndNode* const node = me->m_right.GetNode(m_aabb);
						ndFloat32 dist1 = node->BoxPenetration(obbAabbInfo, vertexArray);
						if (dist1 > ndFloat32 (0.0f)) 
						{
							ndInt32 j = stack;
							for ( ; j && (dist1 > distance[j - 1]); j --) 
							{
								stackPool[j] = stackPool[j - 1];
								distance[j] = distance[j - 1];
							}
							ndAssert (stack < DG_STACK_DEPTH);
							stackPool[j] = node;
							distance[j] = dist1;
							stack++;
						} 
						else 
						{
							obbAabbInfo.m_separationDistance = ndMin(obbAabbInfo.m_separationDistance[0], -dist1);
						}
					}
				}
			}
		} 
		else 
		{
			ndFastRay ray (ndVector::m_zero, boxDistanceTravel);
			ndFastRay obbRay (ndVector::m_zero, obbAabbInfo.UnrotateVector(boxDistanceTravel));
			ndInt32 stack = 1;
			stackPool[0] = m_aabb;
			distance [0] = m_aabb->BoxIntersect (ray, obbRay, obbAabbInfo, vertexArray);

			while (stack) 
			{
				stack --;
				const ndFloat32 dist = distance[stack];
				const ndNode* const me = stackPool[stack];
				if (dist < ndFloat32 (1.0f)) 
				{
					if (me->m_left.IsLeaf()) 
					{
						ndInt32 index = ndInt32 (me->m_left.GetIndex());
						ndInt32 vCount = ndInt32 (me->m_left.GetCount());
						if (vCount > 0) 
						{
							const ndInt32* const indices = &m_indices[index];
							ndInt32 normalIndex = indices[vCount + 1];
							ndVector faceNormal (&vertexArray[normalIndex].m_x);
							faceNormal = faceNormal & ndVector::m_triplexMask;
							ndFloat32 hitDistance = obbAabbInfo.PolygonBoxRayDistance (faceNormal, vCount, indices, stride, &vertexArray[0].m_x, ray);
							if (hitDistance < ndFloat32 (1.0f)) 
							{
								ndAssert (vCount >= 3);
								if (callback(context, &vertexArray[0].m_x, sizeof (ndTriplex), indices, vCount, hitDistance) == m_stopSearch) 
								{
									return;
								}
							}
						}
					} 
					else 
					{
						const ndNode* const node = me->m_left.GetNode(m_aabb);
						ndFloat32 dist1 = node->BoxIntersect (ray, obbRay, obbAabbInfo, vertexArray);
						if (dist1 < ndFloat32 (1.0f)) 
						{
							ndInt32 j = stack;
							for ( ; j && (dist1 > distance[j - 1]); j --) 
							{
								stackPool[j] = stackPool[j - 1];
								distance[j] = distance[j - 1];
							}
							ndAssert (stack < DG_STACK_DEPTH);
							stackPool[j] = node;
							distance[j] = dist1;
							stack++;
						}
					}

					if (me->m_right.IsLeaf()) 
					{
						ndInt32 index = ndInt32 (me->m_right.GetIndex());
						ndInt32 vCount = ndInt32 (me->m_right.GetCount());
						if (vCount > 0) 
						{
							const ndInt32* const indices = &m_indices[index];
							ndInt32 normalIndex = indices[vCount + 1];
							ndVector faceNormal (&vertexArray[normalIndex].m_x);
							faceNormal = faceNormal & ndVector::m_triplexMask;
							ndFloat32 hitDistance = obbAabbInfo.PolygonBoxRayDistance (faceNormal, vCount, indices, stride, &vertexArray[0].m_x, ray);
							if (hitDistance < ndFloat32 (1.0f)) 
							{
								ndAssert (vCount >= 3);
								if (callback(context, &vertexArray[0].m_x, sizeof (ndTriplex), indices, vCount, hitDistance) == m_stopSearch) {
									return;
								}
							}
						}
					} 
					else 
					{
						const ndNode* const node = me->m_right.GetNode(m_aabb);
						ndFloat32 dist1 = node->BoxIntersect (ray, obbRay, obbAabbInfo, vertexArray);
						if (dist1 < ndFloat32 (1.0f)) 
						{
							ndInt32 j = stack;
							for ( ; j && (dist1 > distance[j - 1]); j --) 
							{
								stackPool[j] = stackPool[j - 1];
								distance[j] = distance[j - 1];
							}
							ndAssert (stack < DG_STACK_DEPTH);
							stackPool[j] = node;
							distance[j] = dist1;
							stack ++;
						}
					}
				}
			}
		}
	}
}

void ndAabbPolygonSoup::ForThisSector(const ndAabbPolygonSoup::ndNode* const node, const ndFastAabb& obbAabbInfo, const ndVector& boxDistanceTravel, ndFloat32, ndAaabbIntersectCallback callback, void* const context) const
{
	ndAssert(ndAbs(ndAbs(obbAabbInfo[0][0]) - obbAabbInfo.m_absDir[0][0]) < ndFloat32(1.0e-4f));
	ndAssert(ndAbs(ndAbs(obbAabbInfo[1][1]) - obbAabbInfo.m_absDir[1][1]) < ndFloat32(1.0e-4f));
	ndAssert(ndAbs(ndAbs(obbAabbInfo[2][2]) - obbAabbInfo.m_absDir[2][2]) < ndFloat32(1.0e-4f));

	ndAssert(ndAbs(ndAbs(obbAabbInfo[0][1]) - obbAabbInfo.m_absDir[1][0]) < ndFloat32(1.0e-4f));
	ndAssert(ndAbs(ndAbs(obbAabbInfo[0][2]) - obbAabbInfo.m_absDir[2][0]) < ndFloat32(1.0e-4f));
	ndAssert(ndAbs(ndAbs(obbAabbInfo[1][2]) - obbAabbInfo.m_absDir[2][1]) < ndFloat32(1.0e-4f));

	if (m_aabb)
	{
		const ndInt32 stride = sizeof(ndTriplex) / sizeof(ndFloat32);
		const ndTriplex* const vertexArray = (ndTriplex*)m_localVertex;

		ndAssert(boxDistanceTravel.m_w == ndFloat32(0.0f));
		if (boxDistanceTravel.DotProduct(boxDistanceTravel).GetScalar() < ndFloat32(1.0e-8f))
		{
			ndFloat32 dist = node->BoxPenetration(obbAabbInfo, vertexArray);
			if (dist <= ndFloat32(0.0f))
			{
				obbAabbInfo.m_separationDistance = ndMin(obbAabbInfo.m_separationDistance[0], dist);
			}
			if (dist > ndFloat32(0.0f))
			{
				if (node->m_left.IsLeaf())
				{
					ndInt32 index = ndInt32(node->m_left.GetIndex());
					ndInt32 vCount = ndInt32(node->m_left.GetCount());
					if (vCount > 0)
					{
						const ndInt32* const indices = &m_indices[index];
						ndInt32 normalIndex = indices[vCount + 1];
						ndVector faceNormal(&vertexArray[normalIndex].m_x);
						faceNormal = faceNormal & ndVector::m_triplexMask;
						ndFloat32 dist1 = obbAabbInfo.PolygonBoxDistance(faceNormal, vCount, indices, stride, &vertexArray[0].m_x);
						if (dist1 > ndFloat32(0.0f))
						{
							obbAabbInfo.m_separationDistance = ndFloat32(0.0f);
							ndAssert(vCount >= 3);
							callback(context, &vertexArray[0].m_x, sizeof(ndTriplex), indices, vCount, dist1);
						}
					}
				}
			
				if (node->m_right.IsLeaf())
				{
					ndInt32 index = ndInt32(node->m_right.GetIndex());
					ndInt32 vCount = ndInt32(node->m_right.GetCount());
					if (vCount > 0)
					{
						const ndInt32* const indices = &m_indices[index];
						ndInt32 normalIndex = indices[vCount + 1];
						ndVector faceNormal(&vertexArray[normalIndex].m_x);
						faceNormal = faceNormal & ndVector::m_triplexMask;
						ndFloat32 dist1 = obbAabbInfo.PolygonBoxDistance(faceNormal, vCount, indices, stride, &vertexArray[0].m_x);
						if (dist1 > ndFloat32(0.0f))
						{
							ndAssert(vCount >= 3);
							obbAabbInfo.m_separationDistance = ndFloat32(0.0f);
							callback(context, &vertexArray[0].m_x, sizeof(ndTriplex), indices, vCount, dist1);
						}
					}
				}
			}
		}
		else
		{
			ndAssert(0);
			//ndFastRay ray(ndVector(ndFloat32(0.0f)), boxDistanceTravel);
			//ndFastRay obbRay(ndVector(ndFloat32(0.0f)), obbAabbInfo.UnrotateVector(boxDistanceTravel));
			//ndInt32 stack = 1;
			//stackPool[0] = m_aabb;
			//distance[0] = m_aabb->BoxIntersect(ray, obbRay, obbAabbInfo, vertexArray);
			//
			//while (stack)
			//{
			//	stack--;
			//	const ndFloat32 dist = distance[stack];
			//	const ndNode* const me = stackPool[stack];
			//	if (dist < ndFloat32(1.0f))
			//	{
			//		if (me->m_left.IsLeaf())
			//		{
			//			ndInt32 index = ndInt32(me->m_left.GetIndex());
			//			ndInt32 vCount = ndInt32(me->m_left.GetCount());
			//			if (vCount > 0)
			//			{
			//				const ndInt32* const indices = &m_indices[index];
			//				ndInt32 normalIndex = indices[vCount + 1];
			//				ndVector faceNormal(&vertexArray[normalIndex].m_x);
			//				faceNormal = faceNormal & ndVector::m_triplexMask;
			//				ndFloat32 hitDistance = obbAabbInfo.PolygonBoxRayDistance(faceNormal, vCount, indices, stride, &vertexArray[0].m_x, ray);
			//				if (hitDistance < ndFloat32(1.0f))
			//				{
			//					ndAssert(vCount >= 3);
			//					if (callback(context, &vertexArray[0].m_x, sizeof(ndTriplex), indices, vCount, hitDistance) == m_stopSearch)
			//					{
			//						return;
			//					}
			//				}
			//			}
			//		}
			//		else
			//		{
			//			const ndNode* const node = me->m_left.GetNode(m_aabb);
			//			ndFloat32 dist1 = node->BoxIntersect(ray, obbRay, obbAabbInfo, vertexArray);
			//			if (dist1 < ndFloat32(1.0f))
			//			{
			//				ndInt32 j = stack;
			//				for (; j && (dist1 > distance[j - 1]); j--)
			//				{
			//					stackPool[j] = stackPool[j - 1];
			//					distance[j] = distance[j - 1];
			//				}
			//				ndAssert(stack < DG_STACK_DEPTH);
			//				stackPool[j] = node;
			//				distance[j] = dist1;
			//				stack++;
			//			}
			//		}
			//
			//		if (me->m_right.IsLeaf())
			//		{
			//			ndInt32 index = ndInt32(me->m_right.GetIndex());
			//			ndInt32 vCount = ndInt32(me->m_right.GetCount());
			//			if (vCount > 0)
			//			{
			//				const ndInt32* const indices = &m_indices[index];
			//				ndInt32 normalIndex = indices[vCount + 1];
			//				ndVector faceNormal(&vertexArray[normalIndex].m_x);
			//				faceNormal = faceNormal & ndVector::m_triplexMask;
			//				ndFloat32 hitDistance = obbAabbInfo.PolygonBoxRayDistance(faceNormal, vCount, indices, stride, &vertexArray[0].m_x, ray);
			//				if (hitDistance < ndFloat32(1.0f))
			//				{
			//					ndAssert(vCount >= 3);
			//					if (callback(context, &vertexArray[0].m_x, sizeof(ndTriplex), indices, vCount, hitDistance) == m_stopSearch) {
			//						return;
			//					}
			//				}
			//			}
			//		}
			//		else
			//		{
			//			const ndNode* const node = me->m_right.GetNode(m_aabb);
			//			ndFloat32 dist1 = node->BoxIntersect(ray, obbRay, obbAabbInfo, vertexArray);
			//			if (dist1 < ndFloat32(1.0f))
			//			{
			//				ndInt32 j = stack;
			//				for (; j && (dist1 > distance[j - 1]); j--)
			//				{
			//					stackPool[j] = stackPool[j - 1];
			//					distance[j] = distance[j - 1];
			//				}
			//				ndAssert(stack < DG_STACK_DEPTH);
			//				stackPool[j] = node;
			//				distance[j] = dist1;
			//				stack++;
			//			}
			//		}
			//	}
			//}
		}
	}
}
