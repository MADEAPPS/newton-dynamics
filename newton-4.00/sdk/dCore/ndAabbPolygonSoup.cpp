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

	ndNodeBuilder (const ndVector* const vertexArray, dInt32 faceIndex, dInt32 indexCount, const dInt32* const indexArray)
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
		ndVector minP ( dFloat32 (1.0e15f)); 
		ndVector maxP (-dFloat32 (1.0e15f)); 
		for (dInt32 i = 0; i < indexCount; i ++) 
		{
			dInt32 index = indexArray[i];
			const ndVector& p (vertexArray[index]);
			minP = p.GetMin(minP); 
			maxP = p.GetMax(maxP); 
		}
		minP -= ndVector (dFloat32 (1.0e-3f));
		maxP += ndVector (dFloat32 (1.0e-3f));
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
		m_size = m_p1 - m_p0;
		m_origin = (m_p1 + m_p0) * ndVector::m_half;
		m_area = m_size.DotProduct(m_size.ShiftTripleRight()).m_x;
	}

	inline static dFloat32 CalculateSurfaceArea (ndNodeBuilder* const node0, ndNodeBuilder* const node1, ndVector& minBox, ndVector& maxBox)
	{
		minBox = node0->m_p0.GetMin(node1->m_p0);
		maxBox = node0->m_p1.GetMax(node1->m_p1);

		ndVector side0 ((maxBox - minBox) * ndVector::m_half);
		ndVector side1 (side0.ShiftTripleLeft());
		return side0.DotProduct(side1).GetScalar();
	}

	ndVector m_p0;
	ndVector m_p1;
	ndVector m_size;
	ndVector m_origin;
	dFloat32 m_area;
	
	ndNodeBuilder* m_left;
	ndNodeBuilder* m_right;
	ndNodeBuilder* m_parent;
	dInt32 m_indexBox0;
	dInt32 m_indexBox1;
	dInt32 m_enumeration;
	dInt32 m_faceIndex;
	dInt32 m_indexCount;
	const dInt32* m_faceIndices;
} D_GCC_NEWTON_ALIGN_32;

class ndAabbPolygonSoup::ndSpliteInfo
{
	public:
	ndSpliteInfo (ndNodeBuilder* const boxArray, dInt32 boxCount)
	{
		ndVector minP ( dFloat32 (1.0e15f)); 
		ndVector maxP (-dFloat32 (1.0e15f)); 

		if (boxCount == 2) 
		{
			m_axis = 1;
			for (dInt32 i = 0; i < boxCount; i ++) 
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
			for (dInt32 i = 0; i < boxCount; i ++) 
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

			varian = varian.Scale (dFloat32 (boxCount)) - median * median;

			dInt32 index = 0;
			dFloat32 maxVarian = dFloat32 (-1.0e10f);
			for (dInt32 i = 0; i < 3; i ++) 
			{
				if (varian[i] > maxVarian) 
				{
					index = i;
					maxVarian = varian[i];
				}
			}

			ndVector center = median.Scale (dFloat32 (1.0f) / dFloat32 (boxCount));
			dFloat32 test = center[index];
			dInt32 i0 = 0;
			dInt32 i1 = boxCount - 1;
			do 
			{    
				for (; i0 <= i1; i0 ++) 
				{
					const ndNodeBuilder& box = boxArray[i0];
					dFloat32 val = (box.m_p0[index] + box.m_p1[index]) * dFloat32 (0.5f);
					if (val > test) 
					{
						break;
					}
				}

				for (; i1 >= i0; i1 --) 
				{
					const ndNodeBuilder& box = boxArray[i1];
					dFloat32 val = (box.m_p0[index] + box.m_p1[index]) * dFloat32 (0.5f);
					if (val < test) 
					{
						break;
					}
				}

				if (i0 < i1)
				{
					dSwap(boxArray[i0], boxArray[i1]);
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

		dAssert (maxP.m_x - minP.m_x >= dFloat32 (0.0f));
		dAssert (maxP.m_y - minP.m_y >= dFloat32 (0.0f));
		dAssert (maxP.m_z - minP.m_z >= dFloat32 (0.0f));
		m_p0 = minP;
		m_p1 = maxP;
	}

	dInt32 m_axis;
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

void ndAabbPolygonSoup::ImproveNodeFitness (ndNodeBuilder* const node) const
{
	dAssert (node->m_left);
	dAssert (node->m_right);

	if (node->m_parent)	
	{
		dAssert(node->m_parent->m_p0.m_w == dFloat32(0.0f));
		dAssert(node->m_parent->m_p1.m_w == dFloat32(0.0f));

		if (node->m_parent->m_left == node) 
		{
			dFloat32 cost0 = node->m_area;

			ndVector cost1P0;
			ndVector cost1P1;		
			dFloat32 cost1 = ndNodeBuilder::CalculateSurfaceArea (node->m_right, node->m_parent->m_right, cost1P0, cost1P1);

			ndVector cost2P0;
			ndVector cost2P1;		
			dFloat32 cost2 = ndNodeBuilder::CalculateSurfaceArea (node->m_left, node->m_parent->m_right, cost2P0, cost2P1);

			if ((cost1 <= cost0) && (cost1 <= cost2)) 
			{
				ndNodeBuilder* const parent = node->m_parent;
				node->m_p0 = parent->m_p0;
				node->m_p1 = parent->m_p1;
				node->m_area = parent->m_area; 
				node->m_size = parent->m_size;
				node->m_origin = parent->m_origin;

				if (parent->m_parent) 
				{
					if (parent->m_parent->m_left == parent) 
					{
						parent->m_parent->m_left = node;
					} 
					else 
					{
						dAssert (parent->m_parent->m_right == parent);
						parent->m_parent->m_right = node;
					}
				}
				node->m_parent = parent->m_parent;
				parent->m_parent = node;
				node->m_right->m_parent = parent;
				parent->m_left = node->m_right;
				node->m_right = parent;
				parent->m_p0 = cost1P0;
				parent->m_p1 = cost1P1;		
				parent->m_area = cost1;
				parent->m_size = (parent->m_p1 - parent->m_p0) * ndVector::m_half;
				parent->m_origin = (parent->m_p1 + parent->m_p0) * ndVector::m_half;

			} 
			else if ((cost2 <= cost0) && (cost2 <= cost1)) 
			{
				ndNodeBuilder* const parent = node->m_parent;
				node->m_p0 = parent->m_p0;
				node->m_p1 = parent->m_p1;
				node->m_area = parent->m_area; 
				node->m_size = parent->m_size;
				node->m_origin = parent->m_origin;

				if (parent->m_parent) 
				{
					if (parent->m_parent->m_left == parent) 
					{
						parent->m_parent->m_left = node;
					} 
					else 
					{
						dAssert (parent->m_parent->m_right == parent);
						parent->m_parent->m_right = node;
					}
				}
				node->m_parent = parent->m_parent;
				parent->m_parent = node;
				node->m_left->m_parent = parent;
				parent->m_left = node->m_left;
				node->m_left = parent;

				parent->m_p0 = cost2P0;
				parent->m_p1 = cost2P1;		
				parent->m_area = cost2;
				parent->m_size = (parent->m_p1 - parent->m_p0) * ndVector::m_half;
				parent->m_origin = (parent->m_p1 + parent->m_p0) * ndVector::m_half;
			}
		} 
		else 
		{
			dFloat32 cost0 = node->m_area;

			ndVector cost1P0;
			ndVector cost1P1;		
			dFloat32 cost1 = ndNodeBuilder::CalculateSurfaceArea (node->m_left, node->m_parent->m_left, cost1P0, cost1P1);

			ndVector cost2P0;
			ndVector cost2P1;		
			dFloat32 cost2 = ndNodeBuilder::CalculateSurfaceArea (node->m_right, node->m_parent->m_left, cost2P0, cost2P1);

			if ((cost1 <= cost0) && (cost1 <= cost2)) 
			{
				ndNodeBuilder* const parent = node->m_parent;
				node->m_p0 = parent->m_p0;
				node->m_p1 = parent->m_p1;
				node->m_area = parent->m_area; 
				node->m_size = parent->m_size;
				node->m_origin = parent->m_origin;

				if (parent->m_parent) 
				{
					if (parent->m_parent->m_left == parent) 
					{
						parent->m_parent->m_left = node;
					} 
					else 
					{
						dAssert (parent->m_parent->m_right == parent);
						parent->m_parent->m_right = node;
					}
				}
				node->m_parent = parent->m_parent;
				parent->m_parent = node;
				node->m_left->m_parent = parent;
				parent->m_right = node->m_left;
				node->m_left = parent;

				parent->m_p0 = cost1P0;
				parent->m_p1 = cost1P1;		
				parent->m_area = cost1;
				parent->m_size = (parent->m_p1 - parent->m_p0) * ndVector::m_half;
				parent->m_origin = (parent->m_p1 + parent->m_p0) * ndVector::m_half;

			} 
			else if ((cost2 <= cost0) && (cost2 <= cost1)) 
			{
				ndNodeBuilder* const parent = node->m_parent;
				node->m_p0 = parent->m_p0;
				node->m_p1 = parent->m_p1;
				node->m_area = parent->m_area; 
				node->m_size = parent->m_size;
				node->m_origin = parent->m_origin;

				if (parent->m_parent) 
				{
					if (parent->m_parent->m_left == parent) 
					{
						parent->m_parent->m_left = node;
					} 
					else 
					{
						dAssert (parent->m_parent->m_right == parent);
						parent->m_parent->m_right = node;
					}
				}
				node->m_parent = parent->m_parent;
				parent->m_parent = node;
				node->m_right->m_parent = parent;
				parent->m_right = node->m_right;
				node->m_right = parent;

				parent->m_p0 = cost2P0;
				parent->m_p1 = cost2P1;		
				parent->m_area = cost2;
				parent->m_size = (parent->m_p1 - parent->m_p0) * ndVector::m_half;
				parent->m_origin = (parent->m_p1 + parent->m_p0) * ndVector::m_half;
			}
		}
	} 
	else 
	{
		// in the future I can handle this but it is too much work for little payoff
	}
}

dFloat32 ndAabbPolygonSoup::CalculateFaceMaxSize (const ndVector* const vertex, dInt32 indexCount, const dInt32* const indexArray) const
{
	dFloat32 maxSize = dFloat32 (0.0f);
	dInt32 index = indexArray[indexCount - 1];
	ndVector p0 (vertex[index]);
	for (dInt32 i = 0; i < indexCount; i ++) 
	{
		dInt32 index1 = indexArray[i];
		ndVector p1 (vertex[index1]);

		ndVector dir (p1 - p0);
		dAssert (dir.m_w == dFloat32 (0.0f));
		dir = dir.Normalize();

		dFloat32 maxVal = dFloat32 (-1.0e10f);
		dFloat32 minVal = dFloat32 ( 1.0e10f);
		for (dInt32 j = 0; j < indexCount; j ++) 
		{
			dInt32 index2 = indexArray[j];
			ndVector q (vertex[index2]);
			dFloat32 val = dir.DotProduct(q).GetScalar();
			minVal = dMin(minVal, val);
			maxVal = dMax(maxVal, val);
		}

		dFloat32 size = maxVal - minVal;
		maxSize = dMax(maxSize, size);
		p0 = p1;
	}

	return dFloor (maxSize + dFloat32 (1.0f));
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

void ndAabbPolygonSoup::CalculateAdjacendy ()
{
	ndVector p0;
	ndVector p1;
	GetAABB (p0, p1);
	ndFastAabb box (p0, p1);

	ndPolyhedra adjacenyMesh;
	adjacenyMesh.BeginFace();
	ForAllSectors(box, ndVector::m_zero, dFloat32(1.0f), CalculateAllFaceEdgeNormals, &adjacenyMesh);
	adjacenyMesh.EndFace();

	dInt32 mark = adjacenyMesh.IncLRU();
	ndPolyhedra::Iterator iter(adjacenyMesh);
	const ndTriplex* const vertexArray = (ndTriplex*)GetLocalVertexPool();
	for (iter.Begin(); iter; iter++)
	{
		ndEdge* const edge = &(*iter);
		if ((edge->m_mark != mark) && (edge->m_incidentFace >= 0) && (edge->m_twin->m_incidentFace >= 0))
		{
			#if 0
			{
				dInt32 indexCount0 = 0;
				dEdge* ptr = edge;
				do
				{
					indexCount0++;
					ptr = ptr->m_next;
				} while (ptr != edge);

				dInt32* const indexArray0 = (dInt32*)edge->m_userData;
				dVector n0(&vertexArray[indexArray0[indexCount0 + 1]].m_x);
				dVector q0(&vertexArray[indexArray0[0]].m_x);
				n0 = n0 & dVector::m_triplexMask;
				q0 = q0 & dVector::m_triplexMask;

				dInt32 indexCount1 = 0;
				ptr = edge->m_twin;
				do
				{
					indexCount1++;
					ptr = ptr->m_next;
				} while (ptr != edge->m_twin);

				dInt32* const indexArray1 = (dInt32*)edge->m_twin->m_userData;
				dVector n1(&vertexArray[indexArray1[indexCount1 + 1]].m_x);
				dVector q1(&vertexArray[indexArray1[0]].m_x);
				n1 = n1 & dVector::m_triplexMask;
				q1 = q1 & dVector::m_triplexMask;

				dPlane plane0(n0, -n0.DotProduct(q0).GetScalar());
				dFloat32 maxDist0 = dFloat32(-1.0f);
				dInt32 offsetIndex1 = -1;
				for (dInt32 i = 0; i < indexCount1; i++)
				{
					if (edge->m_twin->m_incidentVertex == indexArray1[i])
					{
						offsetIndex1 = i;
					}
					dVector point(&vertexArray[indexArray1[i]].m_x);
					dFloat32 dist(plane0.Evalue(point & dVector::m_triplexMask));
					if (dist > maxDist0)
					{
						maxDist0 = dist;
					}
				}
				if (maxDist0 < dFloat32(1.0e-3f))
				{
					dPlane plane1(n1, -n1.DotProduct(q1).GetScalar());
					dFloat32 maxDist1 = dFloat32(-1.0f);
					dInt32 offsetIndex0 = -1;
					for (dInt32 i = 0; i < indexCount0; i++)
					{
						if (edge->m_incidentVertex == indexArray0[i])
						{
							offsetIndex0 = i;
						}

						dVector point(&vertexArray[indexArray0[i]].m_x);
						dFloat32 dist(plane1.Evalue(point & dVector::m_triplexMask));
						if (dist > maxDist1)
						{
							maxDist1 = dist;
						}
					}
					if (maxDist1 < dFloat32(1.0e-3f))
					{
						dAssert(offsetIndex0 >= 0);
						dAssert(offsetIndex1 >= 0);
						indexArray0[indexCount0 + 2 + offsetIndex0] = indexArray1[indexCount1 + 1];
						indexArray1[indexCount1 + 2 + offsetIndex1] = indexArray0[indexCount0 + 1];
					}
				}
			}
			#endif

			dInt32 indexCount0 = 0;
			dInt32 indexCount1 = 0;
			dInt32 offsetIndex0 = -1;
			dInt32 offsetIndex1 = -1;
			dInt32* const indexArray0 = (dInt32*)edge->m_userData;
			dInt32* const indexArray1 = (dInt32*)edge->m_twin->m_userData;
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

			dFloat32 maxDist0 = dFloat32(-1.0f);
			for (dInt32 i = 0; i < indexCount1; i++)
			{
				if (edge->m_twin->m_incidentVertex == indexArray1[i])
				{
					offsetIndex1 = i;
				}
				ndVector point(&vertexArray[indexArray1[i]].m_x);
				dFloat32 dist(plane0.Evalue(point & ndVector::m_triplexMask));
				maxDist0 = dMax(maxDist0, dist);
			}

			dFloat32 maxDist1 = dFloat32(-1.0f);
			for (dInt32 i = 0; i < indexCount0; i++)
			{
				if (edge->m_incidentVertex == indexArray0[i])
				{
					offsetIndex0 = i;
				}
				ndVector point(&vertexArray[indexArray0[i]].m_x);
				dFloat32 dist(plane1.Evalue(point & ndVector::m_triplexMask));
				maxDist1 = dMax(maxDist1, dist);
			}

			bool edgeIsConvex = (maxDist0 <= dFloat32(1.0e-3f));
			edgeIsConvex = edgeIsConvex && (maxDist1 <= dFloat32(1.0e-3f));
			edgeIsConvex = edgeIsConvex || (n0.DotProduct(n1).GetScalar() > dFloat32(0.9991f));

			//hacks for testing adjacency
			//edgeIsConvex = edgeIsConvex || (n0.DotProduct(n1).GetScalar() > dFloat32(0.5f));
			//edgeIsConvex = true;
			if (edgeIsConvex)
			{
				dAssert(offsetIndex0 >= 0);
				dAssert(offsetIndex1 >= 0);
				indexArray0[indexCount0 + 2 + offsetIndex0] = indexArray1[indexCount1 + 1];
				indexArray1[indexCount1 + 2 + offsetIndex1] = indexArray0[indexCount0 + 1];
			}
		}
		edge->m_mark = mark;
		edge->m_twin->m_mark = mark;
	}

	ndStack<ndTriplex> pool ((m_indexCount / 2) - 1);
	dInt32 normalCount = 0;
	for (dInt32 i = 0; i < m_nodesCount; i ++) 
	{
		const ndNode* const node = &m_aabb[i];
		if (node->m_left.IsLeaf()) 
		{
			dInt32 vCount = dInt32 (node->m_left.GetCount());
			if (vCount) 
			{
				dInt32 index = dInt32 (node->m_left.GetIndex());
				dInt32* const face = &m_indices[index];
	
				dInt32 j0 = 2 * (vCount + 1) - 1;
				ndVector normal (&vertexArray[face[vCount + 1]].m_x);
				normal = normal & ndVector::m_triplexMask;
				dAssert (dAbs (normal.DotProduct(normal).GetScalar() - dFloat32 (1.0f)) < dFloat32 (1.0e-6f));
				ndVector q0 (&vertexArray[face[vCount - 1]].m_x);
				q0 = q0 & ndVector::m_triplexMask;
				for (dInt32 j = 0; j < vCount; j ++) 
				{
					dInt32 j1 = vCount + 2 + j;
					ndVector q1 (&vertexArray[face[j]].m_x);
					q1 = q1 & ndVector::m_triplexMask;
					if (face[j0] & D_CONCAVE_EDGE_MASK)
					{
						ndVector e (q1 - q0);
						ndVector n (e.CrossProduct(normal).Normalize());
						dAssert (dAbs (n.DotProduct(n).GetScalar() - dFloat32 (1.0f)) < dFloat32 (1.0e-6f));
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
			dInt32 vCount = dInt32 (node->m_right.GetCount());
			if (vCount) 
			{
				dInt32 index = dInt32 (node->m_right.GetIndex());
				dInt32* const face = &m_indices[index];
	
				dInt32 j0 = 2 * (vCount + 1) - 1;
				ndVector normal (&vertexArray[face[vCount + 1]].m_x);
				normal = normal & ndVector::m_triplexMask;
				dAssert (dAbs (normal.DotProduct(normal).GetScalar() - dFloat32 (1.0f)) < dFloat32 (1.0e-6f));
				ndVector q0 (&vertexArray[face[vCount - 1]].m_x);
				q0 = q0 & ndVector::m_triplexMask;
				for (dInt32 j = 0; j < vCount; j ++) 
				{
					dInt32 j1 = vCount + 2 + j;
					ndVector q1 (&vertexArray[face[j]].m_x);
					q1 = q1 & ndVector::m_triplexMask;
					if (face[j0] & D_CONCAVE_EDGE_MASK)
					{
						ndVector e (q1 - q0);
						ndVector n (e.CrossProduct(normal).Normalize());
						dAssert (dAbs (n.DotProduct(n).GetScalar() - dFloat32 (1.0f)) < dFloat32 (1.0e-6f));
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
		ndStack<dInt32> indexArray (normalCount);
		dInt32 newNormalCount = dVertexListToIndexList (&pool[0].m_x, sizeof (ndTriplex), 3, normalCount, &indexArray[0], dFloat32 (1.0e-6f));
	
		dInt32 oldCount = GetVertexCount();
		ndTriplex* const vertexArray1 = (ndTriplex*)ndMemory::Malloc (sizeof (ndTriplex) * (oldCount + newNormalCount));
		memcpy (vertexArray1, GetLocalVertexPool(), sizeof (ndTriplex) * oldCount);
		memcpy (&vertexArray1[oldCount], &pool[0].m_x, sizeof (ndTriplex) * newNormalCount);
		ndMemory::Free(GetLocalVertexPool());
	
		m_localVertex = &vertexArray1[0].m_x;
		m_vertexCount = oldCount + newNormalCount;
	
		for (dInt32 i = 0; i < m_nodesCount; i ++) 
		{
			const ndNode* const node = &m_aabb[i];
			if (node->m_left.IsLeaf()) 
			{
				dInt32 vCount = dInt32 (node->m_left.GetCount());
				dInt32 index = dInt32 (node->m_left.GetIndex());
				dInt32* const face = &m_indices[index];
				for (dInt32 j = 0; j < vCount; j ++) 
				{
					dInt32 edgeIndexNormal = face[vCount + 2 + j];
					//if (face[vCount + 2 + j] < 0) 
					if (edgeIndexNormal & D_CONCAVE_EDGE_MASK)
					{
						//dInt32 k = -1 - face[vCount + 2 + j];
						//face[vCount + 2 + j] = indexArray[k] + oldCount;
						dInt32 k = edgeIndexNormal & (~D_CONCAVE_EDGE_MASK);
						face[vCount + 2 + j] = (indexArray[k] + oldCount) | D_CONCAVE_EDGE_MASK;
					}
					#ifdef _DEBUG	
						//ndVector normal (&vertexArray1[face[vCount + 2 + j]].m_x);
						ndVector normal (&vertexArray1[face[vCount + 2 + j] & (~D_CONCAVE_EDGE_MASK)].m_x);
						normal = normal & ndVector::m_triplexMask;
						dAssert (dAbs (normal.DotProduct(normal).GetScalar() - dFloat32 (1.0f)) < dFloat32 (1.0e-6f));
					#endif
				}
			}
	
			if (node->m_right.IsLeaf()) 
			{
				dInt32 vCount = dInt32 (node->m_right.GetCount());
				dInt32 index = dInt32 (node->m_right.GetIndex());
				dInt32* const face = &m_indices[index];
				for (dInt32 j = 0; j < vCount; j ++) 
				{
					dInt32 edgeIndexNormal = face[vCount + 2 + j];
					//if (face[vCount + 2 + j] < 0) 
					if (edgeIndexNormal & D_CONCAVE_EDGE_MASK)
					{
						//dInt32 k = -1 - face[vCount + 2 + j];
						//face[vCount + 2 + j] = indexArray[k] + oldCount;
						dInt32 k = edgeIndexNormal & (~D_CONCAVE_EDGE_MASK);
						face[vCount + 2 + j] = (indexArray[k] + oldCount) | D_CONCAVE_EDGE_MASK;
					}
	
					#ifdef _DEBUG	
						//ndVector normal (&vertexArray1[face[vCount + 2 + j]].m_x);
						ndVector normal(&vertexArray1[face[vCount + 2 + j] & (~D_CONCAVE_EDGE_MASK)].m_x);
						normal = normal & ndVector::m_triplexMask;
						dAssert (dAbs (normal.DotProduct(normal).GetScalar() - dFloat32 (1.0f)) < dFloat32 (1.0e-6f));
					#endif
				}
			}
		}
	}
}

dIntersectStatus ndAabbPolygonSoup::CalculateAllFaceEdgeNormals(void* const context, const dFloat32* const, dInt32, const dInt32* const indexArray, dInt32 indexCount, dFloat32)
{
	dInt32 face[256];
	dInt64 data[256];
	ndPolyhedra& adjacency = *((ndPolyhedra*)context);
	for (dInt32 i = 0; i < indexCount; i++)
	{
		face[i] = indexArray[i];
		data[i] = dInt64(indexArray);
	}
	adjacency.AddFace(indexCount, face, data);
	return t_ContinueSearh;
}

dIntersectStatus ndAabbPolygonSoup::CalculateDisjointedFaceEdgeNormals (void* const context, const dFloat32* const polygon, dInt32 strideInBytes, const dInt32* const indexArray, dInt32 indexCount, dFloat32)
{
	#define DG_WELDING_TOL (1.0e-2f)
	#define DG_WELDING_TOL2 (DG_WELDING_TOL * DG_WELDING_TOL)

	const ndAdjacentdFace& adjacentFace = *((ndAdjacentdFace*)context);

	if (adjacentFace.m_index != indexArray) 
	{	
		dInt32 adjacentCount = adjacentFace.m_count;
		dInt32 stride = dInt32 (strideInBytes / sizeof (dFloat32));

		dInt32 j0 = adjacentCount - 1;
		dInt32 indexJ0 = adjacentFace.m_index[adjacentCount - 1];
		for (dInt32 j = 0; j < adjacentCount; j ++) {
			dInt32 indexJ1 = adjacentFace.m_index[j];
			ndBigVector q0 (ndVector(&polygon[indexJ1 * stride]) & ndVector::m_triplexMask);
			ndBigVector q1 (ndVector(&polygon[indexJ0 * stride]) & ndVector::m_triplexMask);
			ndBigVector q1q0 (q1 - q0);
			dFloat64 q1q0Mag2 = q1q0.DotProduct(q1q0).GetScalar();

			dInt32 indexI0 = indexArray[indexCount - 1];
			for (dInt32 i = 0; i < indexCount; i ++) 
			{
				dInt32 indexI1 = indexArray[i];
				ndBigVector p0 (ndVector(&polygon[indexI0 * stride]) & ndVector::m_triplexMask);
				ndBigVector p1 (ndVector(&polygon[indexI1 * stride]) & ndVector::m_triplexMask);
				ndBigVector p1p0 (p1 - p0);
				dFloat64 dot = p1p0.DotProduct(q1q0).GetScalar();
				if (dot > 0.0f) 
				{
					dFloat64 q1p0Mag2 = p1p0.DotProduct(p1p0).GetScalar();
					if ((dot * dot) >= (q1p0Mag2 * q1q0Mag2 * dFloat64(0.99995f))) 
					{
						dFloat64 x0 = q0.DotProduct(q1q0).GetScalar();
						dFloat64 x1 = q1.DotProduct(q1q0).GetScalar();
						dFloat64 y0 = p0.DotProduct(q1q0).GetScalar();
						dFloat64 y1 = p1.DotProduct(q1q0).GetScalar();
						dAssert (x1 > x0);
						dAssert (y1 > y0);
						if (!((y0 >= x1) || (y1 <= x0))) 
						{
							dFloat64 t = q1q0.DotProduct(p0 - q0).GetScalar() / q1q0Mag2;
							dAssert (q1q0.m_w == dFloat32 (0.0f));
							ndBigVector q (q0 + q1q0.Scale(t));
							ndBigVector dist (p0 - q);
							dAssert (dist.m_w == dFloat32 (0.0f));
							dFloat64 err2 = dist.DotProduct(dist).GetScalar();
							if (err2 < DG_WELDING_TOL2) 
							{
								dFloat32 maxDist = dFloat32 (0.0f);
								for (dInt32 k = 0; k < indexCount; k ++) 
								{
									ndVector r (&polygon[indexArray[k] * stride]);
									r = r & ndVector::m_triplexMask;
									dFloat32 dist1 = adjacentFace.m_normal.Evalue(r);
									if (dAbs (dist1) > dAbs (maxDist)) 
									{
										maxDist = dist1;
									}
								}

								if (adjacentFace.m_index[j0 + adjacentCount + 2] == -1) 
								{
									if (maxDist < -dFloat32 (1.0e-3f)) 
									{
										adjacentFace.m_index[j0 + adjacentCount + 2] = indexArray[indexCount + 1];
									} 
									else 
									{
										adjacentFace.m_index[j0 + adjacentCount + 2] = adjacentFace.m_index[adjacentCount + 1];
									}
								} 
								else 
								{
									if (maxDist < -dFloat32 (1.0e-3f)) 
									{
										ndBigVector n0 (adjacentFace.m_normal[0], adjacentFace.m_normal[1], adjacentFace.m_normal[2], dFloat64(0.0f));
										ndBigVector n1 (ndVector(&polygon[adjacentFace.m_index[j0 + adjacentCount + 2] * stride]) & ndVector::m_triplexMask);
										ndBigVector n2 (ndVector(&polygon[indexArray[indexCount + 1] * stride]) & ndVector::m_triplexMask);

										ndBigVector tilt0 (n0.CrossProduct(n1)); 
										ndBigVector tilt1 (n0.CrossProduct(n2)); 
										dFloat64 dist0 (q1q0.DotProduct(tilt0).GetScalar());
										dFloat64 dist1 (q1q0.DotProduct(tilt1).GetScalar());
										if (dist0 < dist1) 
										{
											adjacentFace.m_index[j0 + adjacentCount + 2] = indexArray[indexCount + 1];
										}
									} 
									else 
									{
										adjacentFace.m_index[j0 + adjacentCount + 2] = adjacentFace.m_index[adjacentCount + 1];
									}
								}
								break;
							}
						}
					}
				}
				indexI0 = indexI1;
			}
			j0 = j;
			indexJ0 = indexJ1;
		}
	}
	return t_ContinueSearh;
}

ndAabbPolygonSoup::ndNodeBuilder* ndAabbPolygonSoup::BuildTopDown (ndNodeBuilder* const leafArray, dInt32 firstBox, dInt32 lastBox, ndNodeBuilder** const allocator) const
{
	dAssert (firstBox >= 0);
	dAssert (lastBox >= 0);

	if (lastBox == firstBox) 
	{
		return &leafArray[firstBox];
	} 
	else 
	{
		ndSpliteInfo info (&leafArray[firstBox], lastBox - firstBox + 1);

		ndNodeBuilder* const parent = new (*allocator) ndNodeBuilder (info.m_p0, info.m_p1);
		*allocator = *allocator + 1;

		dAssert (parent);
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
	dAssert (builder.m_faceVertexCount.GetCount() >= 1);
	m_strideInBytes = sizeof (ndTriplex);
	m_nodesCount = ((builder.m_faceVertexCount.GetCount() - 1) < 1) ? 1 : builder.m_faceVertexCount.GetCount() - 1;
	m_aabb = (ndNode*) ndMemory::Malloc (sizeof (ndNode) * m_nodesCount);
	m_indexCount = builder.m_vertexIndex.GetCount() * 2 + builder.m_faceVertexCount.GetCount();

	if (builder.m_faceVertexCount.GetCount() == 1) 
	{
		m_indexCount *= 2;
	}

	m_indices = (dInt32*) ndMemory::Malloc (sizeof (dInt32) * m_indexCount);
	ndStack<ndVector> tmpVertexArrayCount(builder.m_vertexPoints.GetCount() + builder.m_normalPoints.GetCount() + builder.m_faceVertexCount.GetCount() * 2 + 4);

	ndVector* const tmpVertexArray = &tmpVertexArrayCount[0];
	for (dInt32 i = 0; i < builder.m_vertexPoints.GetCount(); i ++) 
	{
		tmpVertexArray[i] = builder.m_vertexPoints[i];
	}

	for (dInt32 i = 0; i < builder.m_normalPoints.GetCount(); i ++) 
	{
		tmpVertexArray[i + builder.m_vertexPoints.GetCount()] = builder.m_normalPoints[i];
	}

	const dInt32* const indices = &builder.m_vertexIndex[0];
	ndStack<ndNodeBuilder> constructor (builder.m_faceVertexCount.GetCount() * 2 + 16); 

	dInt32 polygonIndex = 0;
	dInt32 allocatorIndex = 0;
	if (builder.m_faceVertexCount.GetCount() == 1) 
	{
		dInt32 indexCount = builder.m_faceVertexCount[0] - 1;
		new (&constructor[allocatorIndex]) ndNodeBuilder (&tmpVertexArray[0], 0, indexCount, &indices[0]);
		allocatorIndex ++;
	}
	for (dInt32 i = 0; i < builder.m_faceVertexCount.GetCount(); i ++) 
	{
		dInt32 indexCount = builder.m_faceVertexCount[i] - 1;
		new (&constructor[allocatorIndex]) ndNodeBuilder (&tmpVertexArray[0], i, indexCount, &indices[polygonIndex]);
		allocatorIndex ++;
		polygonIndex += (indexCount + 1);
	}

	ndNodeBuilder* contructorAllocator = &constructor[allocatorIndex];
	ndNodeBuilder* root = BuildTopDown (&constructor[0], 0, allocatorIndex - 1, &contructorAllocator);

	dAssert (root);
	dTrace(("*****->this is broken\n"));
	//if (root->m_left) 
	//{
	//	dAssert (root->m_right);
	//	ndList<ndNodeBuilder*> list;
	//	ndList<ndNodeBuilder*> stack;
	//	stack.Append(root);
	//	while (stack.GetCount()) 
	//	{
	//		ndList<ndNodeBuilder*>::ndNode* const stackNode = stack.GetLast();
	//		ndNodeBuilder* const node = stackNode->GetInfo();
	//		stack.Remove(stackNode);
	//
	//		if (node->m_left) 
	//		{
	//			dAssert (node->m_right);
	//			list.Append(node);
	//			stack.Append(node->m_right);
	//			stack.Append(node->m_left);
	//		} 
	//	}
	//
	//	dFloat64 newCost = dFloat32 (1.0e20f);
	//	dFloat64 prevCost = newCost;
	//	do 
	//	{
	//		prevCost = newCost;
	//		for (ndList<ndNodeBuilder*>::ndNode* listNode = list.GetFirst(); listNode; listNode = listNode->GetNext()) 
	//		{
	//			ndNodeBuilder* const node = listNode->GetInfo();
	//			ImproveNodeFitness (node);
	//		}
	//
	//		newCost = dFloat32 (0.0f);
	//		for (ndList<ndNodeBuilder*>::ndNode* listNode = list.GetFirst(); listNode; listNode = listNode->GetNext()) 
	//		{
	//			ndNodeBuilder* const node = listNode->GetInfo();
	//			newCost += node->m_area;
	//		}
	//	} while (newCost < (prevCost * dFloat32 (0.9999f)));
	//
	//	root = list.GetLast()->GetInfo();
	//	while (root->m_parent) 
	//	{
	//		root = root->m_parent;
	//	}
	//}

	ndList<ndNodeBuilder*> list;
	list.Append(root);
	dInt32 nodeIndex = 0;
	while (list.GetCount())
	{
		ndNodeBuilder* const node = list.GetFirst()->GetInfo();
		list.Remove(list.GetFirst());

		if (node->m_left) 
		{
			node->m_enumeration = nodeIndex;
			nodeIndex ++;
			dAssert (node->m_right);
			list.Append(node->m_left);
			list.Append(node->m_right);
		}
	}
	dAssert(!list.GetCount());

	dInt32 aabbBase = builder.m_vertexPoints.GetCount() + builder.m_normalPoints.GetCount();

	ndVector* const aabbPoints = &tmpVertexArray[aabbBase];

	dInt32 vertexIndex = 0;
	dInt32 aabbNodeIndex = 0;
	list.Append(root);
	dInt32 indexMap = 0;
	while (list.GetCount())
	{
		ndNodeBuilder* const node = list.GetFirst()->GetInfo();
		list.Remove(list.GetFirst());

		if (node->m_enumeration >= 0)
		{
			dAssert (node->m_left);
			dAssert (node->m_right);
			ndNode& aabbNode = m_aabb[aabbNodeIndex];
			aabbNodeIndex ++;
			dAssert (aabbNodeIndex <= m_nodesCount);

			if (node->m_parent)
			{
				if (node->m_parent->m_left == node)
				{
					m_aabb[node->m_parent->m_enumeration].m_left = ndNode::ndLeafNodePtr (dUnsigned32 (&m_aabb[node->m_enumeration] - m_aabb));
				}
				else 
				{
					dAssert (node->m_parent->m_right == node);
					m_aabb[node->m_parent->m_enumeration].m_right = ndNode::ndLeafNodePtr (dUnsigned32 (&m_aabb[node->m_enumeration] - m_aabb));
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
			dAssert (!node->m_left);
			dAssert (!node->m_right);

			if (node->m_parent)
			{
				if (node->m_parent->m_left == node)
				{
					m_aabb[node->m_parent->m_enumeration].m_left = ndNode::ndLeafNodePtr (dUnsigned32(node->m_indexCount), dUnsigned32(indexMap));
				}
				else 
				{
					dAssert (node->m_parent->m_right == node);
					m_aabb[node->m_parent->m_enumeration].m_right = ndNode::ndLeafNodePtr (dUnsigned32(node->m_indexCount), dUnsigned32(indexMap));
				}
			}

			// index format i0, i1, i2, ... , id, normal, e0Normal, e1Normal, e2Normal, ..., faceSize
			for (dInt32 j = 0; j < node->m_indexCount; j ++) 
			{
				m_indices[indexMap + j] = node->m_faceIndices[j];
				m_indices[indexMap + j + node->m_indexCount + 2] = D_CONCAVE_EDGE_MASK | 0xffffffff;
			}

			// face attribute
			m_indices[indexMap + node->m_indexCount] = node->m_faceIndices[node->m_indexCount];
			// face normal
			m_indices[indexMap + node->m_indexCount + 1] = builder.m_vertexPoints.GetCount() + builder.m_normalIndex[node->m_faceIndex];
			// face size
			m_indices[indexMap + node->m_indexCount * 2 + 2] = dInt32 (CalculateFaceMaxSize (&tmpVertexArray[0], node->m_indexCount, node->m_faceIndices));

			indexMap += node->m_indexCount * 2 + 3;
		}

		if (node->m_left) 
		{
			dAssert (node->m_right);
			list.Append(node->m_left);
			list.Append(node->m_right);
		}
	}

	ndStack<dInt32> indexArray (vertexIndex);
	dInt32 aabbPointCount = dVertexListToIndexList (&aabbPoints[0].m_x, sizeof (ndVector), 3, vertexIndex, &indexArray[0], dFloat32 (1.0e-6f));

	m_vertexCount = aabbBase + aabbPointCount;
	m_localVertex = (dFloat32*) ndMemory::Malloc (sizeof (ndTriplex) * m_vertexCount);

	ndTriplex* const dstPoints = (ndTriplex*)m_localVertex;
	for (dInt32 i = 0; i < m_vertexCount; i ++) 
	{
		dstPoints[i].m_x = tmpVertexArray[i].m_x;
		dstPoints[i].m_y = tmpVertexArray[i].m_y;
		dstPoints[i].m_z = tmpVertexArray[i].m_z;
	}

	for (dInt32 i = 0; i < m_nodesCount; i ++) 
	{
		ndNode& box = m_aabb[i];

		dInt32 j = box.m_indexBox0 - aabbBase;
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
		fwrite(&m_vertexCount, sizeof(dInt32), 1, file);
		fwrite(&m_indexCount, sizeof(dInt32), 1, file);
		fwrite(&m_nodesCount, sizeof(dInt32), 1, file);
		if (m_aabb)
		{
			fwrite(m_localVertex, sizeof(ndTriplex) * m_vertexCount, 1, file);
			fwrite(m_indices, sizeof(dInt32) * m_indexCount, 1, file);
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
		m_strideInBytes = sizeof(ndTriplex);
		readValues = fread(&m_vertexCount, sizeof(dInt32), 1, file);
		readValues = fread(&m_indexCount, sizeof(dInt32), 1, file);
		readValues = fread(&m_nodesCount, sizeof(dInt32), 1, file);

		if (m_vertexCount) 
		{
			m_localVertex = (dFloat32*)ndMemory::Malloc(sizeof(ndTriplex) * m_vertexCount);
			m_indices = (dInt32*)ndMemory::Malloc(sizeof(dInt32) * m_indexCount);
			m_aabb = (ndNode*)ndMemory::Malloc(sizeof(ndNode) * m_nodesCount);

			readValues = fread(m_localVertex, sizeof(ndTriplex) * m_vertexCount, 1, file);
			readValues = fread(m_indices, sizeof(dInt32) * m_indexCount, 1, file);
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

ndVector ndAabbPolygonSoup::ForAllSectorsSupportVectex (const ndVector& dir) const
{
	ndVector supportVertex (dFloat32 (0.0f));
	if (m_aabb) 
	{
		dFloat32 aabbProjection[DG_STACK_DEPTH];
		const ndNode *stackPool[DG_STACK_DEPTH];

		dInt32 stack = 1;
		stackPool[0] = m_aabb;
		aabbProjection[0] = dFloat32 (1.0e10f);
		const ndTriplex* const boxArray = (ndTriplex*)m_localVertex;

		dFloat32 maxProj = dFloat32 (-1.0e20f); 
		dInt32 ix = (dir[0] > dFloat32 (0.0f)) ? 1 : 0;
		dInt32 iy = (dir[1] > dFloat32 (0.0f)) ? 1 : 0;
		dInt32 iz = (dir[2] > dFloat32 (0.0f)) ? 1 : 0;

		while (stack) 
		{
			dFloat32 boxSupportValue;

			stack--;
			boxSupportValue = aabbProjection[stack];
			if (boxSupportValue > maxProj) {
				dFloat32 backSupportDist = dFloat32 (0.0f);
				dFloat32 frontSupportDist = dFloat32 (0.0f);
				const ndNode* const me = stackPool[stack];
				if (me->m_left.IsLeaf()) 
				{
					backSupportDist = dFloat32 (-1.0e20f);
					dInt32 index = dInt32 (me->m_left.GetIndex());
					dInt32 vCount = dInt32 (me->m_left.GetCount());
					ndVector vertex (dFloat32 (0.0f));
					for (dInt32 j = 0; j < vCount; j ++) 
					{
						dInt32 i0 = m_indices[index + j] * dInt32 (sizeof (ndTriplex) / sizeof (dFloat32));
						ndVector p (&boxArray[i0].m_x);
						p = p & ndVector::m_triplexMask;
						dFloat32 dist = p.DotProduct(dir).GetScalar();
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

					ndVector supportPoint (box[ix].m_x, box[iy].m_y, box[iz].m_z, dFloat32 (0.0));
					backSupportDist = supportPoint.DotProduct(dir).GetScalar();
				}

				if (me->m_right.IsLeaf()) 
				{
					frontSupportDist = dFloat32 (-1.0e20f);
					dInt32 index = dInt32 (me->m_right.GetIndex());
					dInt32 vCount = dInt32 (me->m_right.GetCount());
					ndVector vertex (dFloat32 (0.0f));
					for (dInt32 j = 0; j < vCount; j ++) 
					{
						dInt32 i0 = m_indices[index + j] * dInt32 (sizeof (ndTriplex) / sizeof (dFloat32));
						ndVector p (&boxArray[i0].m_x);
						p = p & ndVector::m_triplexMask;
						dFloat32 dist = p.DotProduct(dir).GetScalar();
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

					ndVector supportPoint (box[ix].m_x, box[iy].m_y, box[iz].m_z, dFloat32 (0.0f));
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

void ndAabbPolygonSoup::ForAllSectorsRayHit (const ndFastRay& raySrc, dFloat32 maxParam, dRayIntersectCallback callback, void* const context) const
{
	const ndNode *stackPool[DG_STACK_DEPTH];
	dFloat32 distance[DG_STACK_DEPTH];
	ndFastRay ray (raySrc);

	dInt32 stack = 1;
	const ndTriplex* const vertexArray = (ndTriplex*) m_localVertex;

	stackPool[0] = m_aabb;
	distance[0] = m_aabb->RayDistance(ray, vertexArray);
	while (stack) 
	{
		stack --;
		dFloat32 dist = distance[stack];
		if (dist > maxParam) 
		{
			break;
		} 
		else 
		{
			const ndNode *const me = stackPool[stack];
			if (me->m_left.IsLeaf()) 
			{
				dInt32 vCount = dInt32 (me->m_left.GetCount());
				if (vCount > 0) 
				{
					dInt32 index = dInt32 (me->m_left.GetIndex());
					dFloat32 param = callback(context, &vertexArray[0].m_x, sizeof (ndTriplex), &m_indices[index], vCount);
					dAssert (param >= dFloat32 (0.0f));
					if (param < maxParam) 
					{
						maxParam = param;
						if (maxParam == dFloat32 (0.0f)) 
						{
							break;
						}
					}
				}

			} 
			else 
			{
				const ndNode* const node = me->m_left.GetNode(m_aabb);
				dFloat32 dist1 = node->RayDistance(ray, vertexArray);
				if (dist1 < maxParam) 
				{
					dInt32 j = stack;
					for ( ; j && (dist1 > distance[j - 1]); j --) 
					{
						stackPool[j] = stackPool[j - 1];
						distance[j] = distance[j - 1];
					}
					dAssert (stack < DG_STACK_DEPTH);
					stackPool[j] = node;
					distance[j] = dist1;
					stack++;
				}
			}

			if (me->m_right.IsLeaf()) 
			{
				dInt32 vCount = dInt32 (me->m_right.GetCount());
				if (vCount > 0) 
				{
					dInt32 index = dInt32 (me->m_right.GetIndex());
					dFloat32 param = callback(context, &vertexArray[0].m_x, sizeof (ndTriplex), &m_indices[index], vCount);
					dAssert (param >= dFloat32 (0.0f));
					if (param < maxParam) 
					{
						maxParam = param;
						if (maxParam == dFloat32 (0.0f)) 
						{
							break;
						}
					}
				}
			} 
			else 
			{
				const ndNode* const node = me->m_right.GetNode(m_aabb);
				dFloat32 dist1 = node->RayDistance(ray, vertexArray);
				if (dist1 < maxParam) 
				{
					dInt32 j = stack;
					for ( ; j && (dist1 > distance[j - 1]); j --) 
					{
						stackPool[j] = stackPool[j - 1];
						distance[j] = distance[j - 1];
					}
					dAssert (stack < DG_STACK_DEPTH);
					stackPool[j] = node;
					distance[j] = dist1;
					stack++;
				}
			}
		}
	}
}

void ndAabbPolygonSoup::ForAllSectors (const ndFastAabb& obbAabbInfo, const ndVector& boxDistanceTravel, dFloat32, dAaabbIntersectCallback callback, void* const context) const
{
	dAssert (dAbs(dAbs(obbAabbInfo[0][0]) - obbAabbInfo.m_absDir[0][0]) < dFloat32 (1.0e-4f));
	dAssert (dAbs(dAbs(obbAabbInfo[1][1]) - obbAabbInfo.m_absDir[1][1]) < dFloat32 (1.0e-4f));
	dAssert (dAbs(dAbs(obbAabbInfo[2][2]) - obbAabbInfo.m_absDir[2][2]) < dFloat32 (1.0e-4f));

	dAssert (dAbs(dAbs(obbAabbInfo[0][1]) - obbAabbInfo.m_absDir[1][0]) < dFloat32 (1.0e-4f));
	dAssert (dAbs(dAbs(obbAabbInfo[0][2]) - obbAabbInfo.m_absDir[2][0]) < dFloat32 (1.0e-4f));
	dAssert (dAbs(dAbs(obbAabbInfo[1][2]) - obbAabbInfo.m_absDir[2][1]) < dFloat32 (1.0e-4f));

	if (m_aabb) 
	{
		dFloat32 distance[DG_STACK_DEPTH];
		const ndNode* stackPool[DG_STACK_DEPTH];

		const dInt32 stride = sizeof (ndTriplex) / sizeof (dFloat32);
		const ndTriplex* const vertexArray = (ndTriplex*) m_localVertex;

		dAssert (boxDistanceTravel.m_w == dFloat32 (0.0f));
		if (boxDistanceTravel.DotProduct(boxDistanceTravel).GetScalar() < dFloat32 (1.0e-8f)) 
		{
			dInt32 stack = 1;
			stackPool[0] = m_aabb;
			distance[0] = m_aabb->BoxPenetration(obbAabbInfo, vertexArray);
			if (distance[0] <= dFloat32(0.0f)) 
			{
				obbAabbInfo.m_separationDistance = dMin(obbAabbInfo.m_separationDistance[0], -distance[0]);
			}
			while (stack) 
			{
				stack --;
				dFloat32 dist = distance[stack];
				if (dist > dFloat32 (0.0f)) 
				{
					const ndNode* const me = stackPool[stack];
					if (me->m_left.IsLeaf()) 
					{
						dInt32 index = dInt32 (me->m_left.GetIndex());
						dInt32 vCount = dInt32 (me->m_left.GetCount());
						if (vCount > 0) 
						{
							const dInt32* const indices = &m_indices[index];
							dInt32 normalIndex = indices[vCount + 1];
							ndVector faceNormal (&vertexArray[normalIndex].m_x);
							faceNormal = faceNormal & ndVector::m_triplexMask;
							dFloat32 dist1 = obbAabbInfo.PolygonBoxDistance (faceNormal, vCount, indices, stride, &vertexArray[0].m_x);
							if (dist1 > dFloat32 (0.0f)) 
							{
								obbAabbInfo.m_separationDistance = dFloat32(0.0f);
								dAssert (vCount >= 3);
								if (callback(context, &vertexArray[0].m_x, sizeof (ndTriplex), indices, vCount, dist1) == t_StopSearh) 
								{
									return;
								}
							} 
							else 
							{
								obbAabbInfo.m_separationDistance = dMin(obbAabbInfo.m_separationDistance[0], -dist1);
							}
						}
					} 
					else 
					{
						const ndNode* const node = me->m_left.GetNode(m_aabb);
						dFloat32 dist1 = node->BoxPenetration(obbAabbInfo, vertexArray);
						if (dist1 > dFloat32 (0.0f)) 
						{
							dInt32 j = stack;
							for ( ; j && (dist1 > distance[j - 1]); j --) 
							{
								stackPool[j] = stackPool[j - 1];
								distance[j] = distance[j - 1];
							}
							dAssert (stack < DG_STACK_DEPTH);
							stackPool[j] = node;
							distance[j] = dist1;
							stack++;
						} 
						else 
						{
							obbAabbInfo.m_separationDistance = dMin(obbAabbInfo.m_separationDistance[0], -dist1);
						}
					}

					if (me->m_right.IsLeaf()) 
					{
						dInt32 index = dInt32 (me->m_right.GetIndex());
						dInt32 vCount = dInt32 (me->m_right.GetCount());
						if (vCount > 0) 
						{
							const dInt32* const indices = &m_indices[index];
							dInt32 normalIndex = indices[vCount + 1];
							ndVector faceNormal (&vertexArray[normalIndex].m_x);
							faceNormal = faceNormal & ndVector::m_triplexMask;
							dFloat32 dist1 = obbAabbInfo.PolygonBoxDistance (faceNormal, vCount, indices, stride, &vertexArray[0].m_x);
							if (dist1 > dFloat32 (0.0f)) 
							{
								dAssert (vCount >= 3);
								obbAabbInfo.m_separationDistance = dFloat32(0.0f);
								if (callback(context, &vertexArray[0].m_x, sizeof (ndTriplex), indices, vCount, dist1) == t_StopSearh) 
								{
									return;
								}
							} 
							else 
							{
								obbAabbInfo.m_separationDistance = dMin(obbAabbInfo.m_separationDistance[0], -dist1);
							}
						}
					} 
					else 
					{
						const ndNode* const node = me->m_right.GetNode(m_aabb);
						dFloat32 dist1 = node->BoxPenetration(obbAabbInfo, vertexArray);
						if (dist1 > dFloat32 (0.0f)) 
						{
							dInt32 j = stack;
							for ( ; j && (dist1 > distance[j - 1]); j --) 
							{
								stackPool[j] = stackPool[j - 1];
								distance[j] = distance[j - 1];
							}
							dAssert (stack < DG_STACK_DEPTH);
							stackPool[j] = node;
							distance[j] = dist1;
							stack++;
						} 
						else 
						{
							obbAabbInfo.m_separationDistance = dMin(obbAabbInfo.m_separationDistance[0], -dist1);
						}
					}
				}
			}
		} 
		else 
		{
			ndFastRay ray (ndVector::m_zero, boxDistanceTravel);
			ndFastRay obbRay (ndVector::m_zero, obbAabbInfo.UnrotateVector(boxDistanceTravel));
			dInt32 stack = 1;
			stackPool[0] = m_aabb;
			distance [0] = m_aabb->BoxIntersect (ray, obbRay, obbAabbInfo, vertexArray);

			while (stack) 
			{
				stack --;
				const dFloat32 dist = distance[stack];
				const ndNode* const me = stackPool[stack];
				if (dist < dFloat32 (1.0f)) 
				{
					if (me->m_left.IsLeaf()) 
					{
						dInt32 index = dInt32 (me->m_left.GetIndex());
						dInt32 vCount = dInt32 (me->m_left.GetCount());
						if (vCount > 0) 
						{
							const dInt32* const indices = &m_indices[index];
							dInt32 normalIndex = indices[vCount + 1];
							ndVector faceNormal (&vertexArray[normalIndex].m_x);
							faceNormal = faceNormal & ndVector::m_triplexMask;
							dFloat32 hitDistance = obbAabbInfo.PolygonBoxRayDistance (faceNormal, vCount, indices, stride, &vertexArray[0].m_x, ray);
							if (hitDistance < dFloat32 (1.0f)) 
							{
								dAssert (vCount >= 3);
								if (callback(context, &vertexArray[0].m_x, sizeof (ndTriplex), indices, vCount, hitDistance) == t_StopSearh) 
								{
									return;
								}
							}
						}
					} 
					else 
					{
						const ndNode* const node = me->m_left.GetNode(m_aabb);
						dFloat32 dist1 = node->BoxIntersect (ray, obbRay, obbAabbInfo, vertexArray);
						if (dist1 < dFloat32 (1.0f)) 
						{
							dInt32 j = stack;
							for ( ; j && (dist1 > distance[j - 1]); j --) 
							{
								stackPool[j] = stackPool[j - 1];
								distance[j] = distance[j - 1];
							}
							dAssert (stack < DG_STACK_DEPTH);
							stackPool[j] = node;
							distance[j] = dist1;
							stack++;
						}
					}

					if (me->m_right.IsLeaf()) 
					{
						dInt32 index = dInt32 (me->m_right.GetIndex());
						dInt32 vCount = dInt32 (me->m_right.GetCount());
						if (vCount > 0) 
						{
							const dInt32* const indices = &m_indices[index];
							dInt32 normalIndex = indices[vCount + 1];
							ndVector faceNormal (&vertexArray[normalIndex].m_x);
							faceNormal = faceNormal & ndVector::m_triplexMask;
							dFloat32 hitDistance = obbAabbInfo.PolygonBoxRayDistance (faceNormal, vCount, indices, stride, &vertexArray[0].m_x, ray);
							if (hitDistance < dFloat32 (1.0f)) 
							{
								dAssert (vCount >= 3);
								if (callback(context, &vertexArray[0].m_x, sizeof (ndTriplex), indices, vCount, hitDistance) == t_StopSearh) {
									return;
								}
							}
						}
					} 
					else 
					{
						const ndNode* const node = me->m_right.GetNode(m_aabb);
						dFloat32 dist1 = node->BoxIntersect (ray, obbRay, obbAabbInfo, vertexArray);
						if (dist1 < dFloat32 (1.0f)) 
						{
							dInt32 j = stack;
							for ( ; j && (dist1 > distance[j - 1]); j --) 
							{
								stackPool[j] = stackPool[j - 1];
								distance[j] = distance[j - 1];
							}
							dAssert (stack < DG_STACK_DEPTH);
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

void ndAabbPolygonSoup::ForThisSector(const ndAabbPolygonSoup::ndNode* const node, const ndFastAabb& obbAabbInfo, const ndVector& boxDistanceTravel, dFloat32, dAaabbIntersectCallback callback, void* const context) const
{
	dAssert(dAbs(dAbs(obbAabbInfo[0][0]) - obbAabbInfo.m_absDir[0][0]) < dFloat32(1.0e-4f));
	dAssert(dAbs(dAbs(obbAabbInfo[1][1]) - obbAabbInfo.m_absDir[1][1]) < dFloat32(1.0e-4f));
	dAssert(dAbs(dAbs(obbAabbInfo[2][2]) - obbAabbInfo.m_absDir[2][2]) < dFloat32(1.0e-4f));

	dAssert(dAbs(dAbs(obbAabbInfo[0][1]) - obbAabbInfo.m_absDir[1][0]) < dFloat32(1.0e-4f));
	dAssert(dAbs(dAbs(obbAabbInfo[0][2]) - obbAabbInfo.m_absDir[2][0]) < dFloat32(1.0e-4f));
	dAssert(dAbs(dAbs(obbAabbInfo[1][2]) - obbAabbInfo.m_absDir[2][1]) < dFloat32(1.0e-4f));

	if (m_aabb)
	{
		const dInt32 stride = sizeof(ndTriplex) / sizeof(dFloat32);
		const ndTriplex* const vertexArray = (ndTriplex*)m_localVertex;

		dAssert(boxDistanceTravel.m_w == dFloat32(0.0f));
		if (boxDistanceTravel.DotProduct(boxDistanceTravel).GetScalar() < dFloat32(1.0e-8f))
		{
			dFloat32 dist = node->BoxPenetration(obbAabbInfo, vertexArray);
			if (dist <= dFloat32(0.0f))
			{
				obbAabbInfo.m_separationDistance = dMin(obbAabbInfo.m_separationDistance[0], dist);
			}
			if (dist > dFloat32(0.0f))
			{
				if (node->m_left.IsLeaf())
				{
					dInt32 index = dInt32(node->m_left.GetIndex());
					dInt32 vCount = dInt32(node->m_left.GetCount());
					if (vCount > 0)
					{
						const dInt32* const indices = &m_indices[index];
						dInt32 normalIndex = indices[vCount + 1];
						ndVector faceNormal(&vertexArray[normalIndex].m_x);
						faceNormal = faceNormal & ndVector::m_triplexMask;
						dFloat32 dist1 = obbAabbInfo.PolygonBoxDistance(faceNormal, vCount, indices, stride, &vertexArray[0].m_x);
						if (dist1 > dFloat32(0.0f))
						{
							obbAabbInfo.m_separationDistance = dFloat32(0.0f);
							dAssert(vCount >= 3);
							callback(context, &vertexArray[0].m_x, sizeof(ndTriplex), indices, vCount, dist1);
						}
					}
				}
			
				if (node->m_right.IsLeaf())
				{
					dInt32 index = dInt32(node->m_right.GetIndex());
					dInt32 vCount = dInt32(node->m_right.GetCount());
					if (vCount > 0)
					{
						const dInt32* const indices = &m_indices[index];
						dInt32 normalIndex = indices[vCount + 1];
						ndVector faceNormal(&vertexArray[normalIndex].m_x);
						faceNormal = faceNormal & ndVector::m_triplexMask;
						dFloat32 dist1 = obbAabbInfo.PolygonBoxDistance(faceNormal, vCount, indices, stride, &vertexArray[0].m_x);
						if (dist1 > dFloat32(0.0f))
						{
							dAssert(vCount >= 3);
							obbAabbInfo.m_separationDistance = dFloat32(0.0f);
							callback(context, &vertexArray[0].m_x, sizeof(ndTriplex), indices, vCount, dist1);
						}
					}
				}
			}
		}
		else
		{
			dAssert(0);
			//ndFastRay ray(ndVector(dFloat32(0.0f)), boxDistanceTravel);
			//ndFastRay obbRay(ndVector(dFloat32(0.0f)), obbAabbInfo.UnrotateVector(boxDistanceTravel));
			//dInt32 stack = 1;
			//stackPool[0] = m_aabb;
			//distance[0] = m_aabb->BoxIntersect(ray, obbRay, obbAabbInfo, vertexArray);
			//
			//while (stack)
			//{
			//	stack--;
			//	const dFloat32 dist = distance[stack];
			//	const ndNode* const me = stackPool[stack];
			//	if (dist < dFloat32(1.0f))
			//	{
			//		if (me->m_left.IsLeaf())
			//		{
			//			dInt32 index = dInt32(me->m_left.GetIndex());
			//			dInt32 vCount = dInt32(me->m_left.GetCount());
			//			if (vCount > 0)
			//			{
			//				const dInt32* const indices = &m_indices[index];
			//				dInt32 normalIndex = indices[vCount + 1];
			//				ndVector faceNormal(&vertexArray[normalIndex].m_x);
			//				faceNormal = faceNormal & ndVector::m_triplexMask;
			//				dFloat32 hitDistance = obbAabbInfo.PolygonBoxRayDistance(faceNormal, vCount, indices, stride, &vertexArray[0].m_x, ray);
			//				if (hitDistance < dFloat32(1.0f))
			//				{
			//					dAssert(vCount >= 3);
			//					if (callback(context, &vertexArray[0].m_x, sizeof(ndTriplex), indices, vCount, hitDistance) == t_StopSearh)
			//					{
			//						return;
			//					}
			//				}
			//			}
			//		}
			//		else
			//		{
			//			const ndNode* const node = me->m_left.GetNode(m_aabb);
			//			dFloat32 dist1 = node->BoxIntersect(ray, obbRay, obbAabbInfo, vertexArray);
			//			if (dist1 < dFloat32(1.0f))
			//			{
			//				dInt32 j = stack;
			//				for (; j && (dist1 > distance[j - 1]); j--)
			//				{
			//					stackPool[j] = stackPool[j - 1];
			//					distance[j] = distance[j - 1];
			//				}
			//				dAssert(stack < DG_STACK_DEPTH);
			//				stackPool[j] = node;
			//				distance[j] = dist1;
			//				stack++;
			//			}
			//		}
			//
			//		if (me->m_right.IsLeaf())
			//		{
			//			dInt32 index = dInt32(me->m_right.GetIndex());
			//			dInt32 vCount = dInt32(me->m_right.GetCount());
			//			if (vCount > 0)
			//			{
			//				const dInt32* const indices = &m_indices[index];
			//				dInt32 normalIndex = indices[vCount + 1];
			//				ndVector faceNormal(&vertexArray[normalIndex].m_x);
			//				faceNormal = faceNormal & ndVector::m_triplexMask;
			//				dFloat32 hitDistance = obbAabbInfo.PolygonBoxRayDistance(faceNormal, vCount, indices, stride, &vertexArray[0].m_x, ray);
			//				if (hitDistance < dFloat32(1.0f))
			//				{
			//					dAssert(vCount >= 3);
			//					if (callback(context, &vertexArray[0].m_x, sizeof(ndTriplex), indices, vCount, hitDistance) == t_StopSearh) {
			//						return;
			//					}
			//				}
			//			}
			//		}
			//		else
			//		{
			//			const ndNode* const node = me->m_right.GetNode(m_aabb);
			//			dFloat32 dist1 = node->BoxIntersect(ray, obbRay, obbAabbInfo, vertexArray);
			//			if (dist1 < dFloat32(1.0f))
			//			{
			//				dInt32 j = stack;
			//				for (; j && (dist1 > distance[j - 1]); j--)
			//				{
			//					stackPool[j] = stackPool[j - 1];
			//					distance[j] = distance[j - 1];
			//				}
			//				dAssert(stack < DG_STACK_DEPTH);
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
