/* Copyright (c) <2003-2019> <Julio Jerez, Newton Game Dynamics>
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

#include "dCoreStdafx.h"
#include "dTypes.h"
#include "dHeap.h"
#include "dStack.h"
#include "dList.h"
#include "dMatrix.h"
#include "dPolyhedra.h"
#include "dAabbPolygonSoup.h"
#include "dPolygonSoupBuilder.h"

#define DG_STACK_DEPTH 512

D_MSV_NEWTON_ALIGN_32
class dAabbPolygonSoup::dgNodeBuilder: public dAabbPolygonSoup::dNode
{
	public:
	dgNodeBuilder (const dVector& p0, const dVector& p1)
		:dNode()
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

	dgNodeBuilder (const dVector* const vertexArray, dInt32 faceIndex, dInt32 indexCount, const dInt32* const indexArray)
		:dNode()
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
		dVector minP ( dFloat32 (1.0e15f)); 
		dVector maxP (-dFloat32 (1.0e15f)); 
		for (dInt32 i = 0; i < indexCount; i ++) 
		{
			dInt32 index = indexArray[i];
			const dVector& p (vertexArray[index]);
			minP = p.GetMin(minP); 
			maxP = p.GetMax(maxP); 
		}
		minP -= dVector (dFloat32 (1.0e-3f));
		maxP += dVector (dFloat32 (1.0e-3f));
		minP = minP & dVector::m_triplexMask;
		maxP = maxP & dVector::m_triplexMask;
		SetBox (minP, maxP);
	}

	dgNodeBuilder (dgNodeBuilder* const left, dgNodeBuilder* const right)
		:dNode()
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

		dVector p0 (left->m_p0.GetMin(right->m_p0));
		dVector p1 (left->m_p1.GetMax(right->m_p1));
		SetBox(p0, p1);
	}

	void SetBox (const dVector& p0, const dVector& p1)
	{
		m_p0 = p0;
		m_p1 = p1;
		m_size = m_p1 - m_p0;
		m_origin = (m_p1 + m_p0) * dVector::m_half;
		m_area = m_size.DotProduct(m_size.ShiftTripleRight()).m_x;
	}

	inline static dFloat32 CalculateSurfaceArea (dgNodeBuilder* const node0, dgNodeBuilder* const node1, dVector& minBox, dVector& maxBox)
	{
		minBox = node0->m_p0.GetMin(node1->m_p0);
		maxBox = node0->m_p1.GetMax(node1->m_p1);

		dVector side0 ((maxBox - minBox) * dVector::m_half);
		//dVector side1 (side0.m_y, side0.m_z, side0.m_x, dFloat32 (0.0f));
		dVector side1 (side0.ShiftTripleLeft());
		return side0.DotProduct(side1).GetScalar();
	}

	dVector m_p0;
	dVector m_p1;
	dVector m_size;
	dVector m_origin;
	dFloat32 m_area;
	
	dgNodeBuilder* m_left;
	dgNodeBuilder* m_right;
	dgNodeBuilder* m_parent;
	dInt32 m_indexBox0;
	dInt32 m_indexBox1;
	dInt32 m_enumeration;
	dInt32 m_faceIndex;
	dInt32 m_indexCount;
	const dInt32* m_faceIndices;
} D_GCC_NEWTON_ALIGN_32;

class dAabbPolygonSoup::dgSpliteInfo
{
	public:
	dgSpliteInfo (dgNodeBuilder* const boxArray, dInt32 boxCount)
	{
		dVector minP ( dFloat32 (1.0e15f)); 
		dVector maxP (-dFloat32 (1.0e15f)); 

		if (boxCount == 2) 
		{
			m_axis = 1;
			for (dInt32 i = 0; i < boxCount; i ++) 
			{
				const dgNodeBuilder& box = boxArray[i];
				const dVector& p0 = box.m_p0;
				const dVector& p1 = box.m_p1;
				minP = minP.GetMin (p0); 
				maxP = maxP.GetMax (p1); 
			}

		} 
		else 
		{
			dVector median (dVector::m_zero);
			dVector varian (dVector::m_zero);
			for (dInt32 i = 0; i < boxCount; i ++) 
			{
				const dgNodeBuilder& box = boxArray[i];

				const dVector& p0 = box.m_p0;
				const dVector& p1 = box.m_p1;

				minP = minP.GetMin (p0); 
				maxP = maxP.GetMax (p1); 
				dVector p (dVector::m_half * (p0 + p1));

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

			dVector center = median.Scale (dFloat32 (1.0f) / dFloat32 (boxCount));
			dFloat32 test = center[index];
			dInt32 i0 = 0;
			dInt32 i1 = boxCount - 1;
			do 
			{    
				for (; i0 <= i1; i0 ++) 
				{
					const dgNodeBuilder& box = boxArray[i0];
					dFloat32 val = (box.m_p0[index] + box.m_p1[index]) * dFloat32 (0.5f);
					if (val > test) 
					{
						break;
					}
				}

				for (; i1 >= i0; i1 --) 
				{
					const dgNodeBuilder& box = boxArray[i1];
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
	dVector m_p0;
	dVector m_p1;
};

dAabbPolygonSoup::dAabbPolygonSoup ()
	:dPolygonSoupDatabase()
	,m_nodesCount(0)
	,m_indexCount(0)
	,m_aabb(nullptr)
	,m_indices(nullptr)
{
}

dAabbPolygonSoup::~dAabbPolygonSoup ()
{
	if (m_aabb) 
	{
		dMemory::Free(m_aabb);
		dMemory::Free(m_indices);
	}
}

void dAabbPolygonSoup::ImproveNodeFitness (dgNodeBuilder* const node) const
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

			dVector cost1P0;
			dVector cost1P1;		
			dFloat32 cost1 = dgNodeBuilder::CalculateSurfaceArea (node->m_right, node->m_parent->m_right, cost1P0, cost1P1);

			dVector cost2P0;
			dVector cost2P1;		
			dFloat32 cost2 = dgNodeBuilder::CalculateSurfaceArea (node->m_left, node->m_parent->m_right, cost2P0, cost2P1);

			if ((cost1 <= cost0) && (cost1 <= cost2)) 
			{
				dgNodeBuilder* const parent = node->m_parent;
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
				parent->m_size = (parent->m_p1 - parent->m_p0) * dVector::m_half;
				parent->m_origin = (parent->m_p1 + parent->m_p0) * dVector::m_half;

			} 
			else if ((cost2 <= cost0) && (cost2 <= cost1)) 
			{
				dgNodeBuilder* const parent = node->m_parent;
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
				parent->m_size = (parent->m_p1 - parent->m_p0) * dVector::m_half;
				parent->m_origin = (parent->m_p1 + parent->m_p0) * dVector::m_half;
			}
		} 
		else 
		{
			dFloat32 cost0 = node->m_area;

			dVector cost1P0;
			dVector cost1P1;		
			dFloat32 cost1 = dgNodeBuilder::CalculateSurfaceArea (node->m_left, node->m_parent->m_left, cost1P0, cost1P1);

			dVector cost2P0;
			dVector cost2P1;		
			dFloat32 cost2 = dgNodeBuilder::CalculateSurfaceArea (node->m_right, node->m_parent->m_left, cost2P0, cost2P1);

			if ((cost1 <= cost0) && (cost1 <= cost2)) 
			{
				dgNodeBuilder* const parent = node->m_parent;
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
				parent->m_size = (parent->m_p1 - parent->m_p0) * dVector::m_half;
				parent->m_origin = (parent->m_p1 + parent->m_p0) * dVector::m_half;

			} 
			else if ((cost2 <= cost0) && (cost2 <= cost1)) 
			{
				dgNodeBuilder* const parent = node->m_parent;
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
				parent->m_size = (parent->m_p1 - parent->m_p0) * dVector::m_half;
				parent->m_origin = (parent->m_p1 + parent->m_p0) * dVector::m_half;
			}
		}
	} 
	else 
	{
		// in the future I can handle this but it is too much work for little payoff
	}
}

dFloat32 dAabbPolygonSoup::CalculateFaceMaxSize (const dVector* const vertex, dInt32 indexCount, const dInt32* const indexArray) const
{
	dFloat32 maxSize = dFloat32 (0.0f);
	dInt32 index = indexArray[indexCount - 1];
	dVector p0 (vertex[index]);
	for (dInt32 i = 0; i < indexCount; i ++) 
	{
		dInt32 index1 = indexArray[i];
		dVector p1 (vertex[index1]);

		dVector dir (p1 - p0);
		dAssert (dir.m_w == dFloat32 (0.0f));
		dir = dir.Normalize();

		dFloat32 maxVal = dFloat32 (-1.0e10f);
		dFloat32 minVal = dFloat32 ( 1.0e10f);
		for (dInt32 j = 0; j < indexCount; j ++) 
		{
			dInt32 index2 = indexArray[j];
			dVector q (vertex[index2]);
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

void dAabbPolygonSoup::GetAABB (dVector& p0, dVector& p1) const
{
	if (m_aabb) 
	{ 
		GetNodeAABB (m_aabb, p0, p1);
	} 
	else 
	{
		p0 = dVector::m_zero;
		p1 = dVector::m_zero;
	}
}

void dAabbPolygonSoup::CalculateAdjacendy ()
{
	dVector p0;
	dVector p1;
	GetAABB (p0, p1);
	dFastAabbInfo box (p0, p1);

//	ForAllSectors(box, dVector::m_zero, dFloat32(1.0f), CalculateAllFaceEdgeNormalsOld, this);

	dPolyhedra adjacenyMesh;
	adjacenyMesh.BeginFace();
	ForAllSectors(box, dVector::m_zero, dFloat32(1.0f), CalculateAllFaceEdgeNormals, &adjacenyMesh);
	adjacenyMesh.EndFace();

	dInt32 mark = adjacenyMesh.IncLRU();
	dPolyhedra::Iterator iter(adjacenyMesh);
	const dTriplex* const vertexArray = (dTriplex*)GetLocalVertexPool();
	for (iter.Begin(); iter; iter++)
	{
		dEdge* const edge = &(*iter);
		if ((edge->m_mark != mark) && (edge->m_incidentFace >= 0) && (edge->m_twin->m_incidentFace >= 0))
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
			dFloat32 maxDist0 = dFloat32 (-1.0f);
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
			if (maxDist0 < dFloat32 (1.0e-3f))
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
					dAssert(offsetIndex0 != -1);
					dAssert(offsetIndex1 != -1);
					indexArray0[indexCount0 + 2 + offsetIndex0] = indexArray1[indexCount1 + 1];
					indexArray1[indexCount1 + 2 + offsetIndex1] = indexArray0[indexCount0 + 1];
					maxDist1 *= 1;
				}
			}
		}
		edge->m_mark = mark;
		edge->m_twin->m_mark = mark;
	}

	dStack<dTriplex> pool ((m_indexCount / 2) - 1);
	dInt32 normalCount = 0;
	for (dInt32 i = 0; i < m_nodesCount; i ++) 
	{
		const dNode* const node = &m_aabb[i];
		if (node->m_left.IsLeaf()) 
		{
			dInt32 vCount = dInt32 (node->m_left.GetCount());
			if (vCount) 
			{
				dInt32 index = dInt32 (node->m_left.GetIndex());
				dInt32* const face = &m_indices[index];
	
				dInt32 j0 = 2 * (vCount + 1) - 1;
				dVector normal (&vertexArray[face[vCount + 1]].m_x);
				normal = normal & dVector::m_triplexMask;
				dAssert (dAbs (normal.DotProduct(normal).GetScalar() - dFloat32 (1.0f)) < dFloat32 (1.0e-6f));
				dVector q0 (&vertexArray[face[vCount - 1]].m_x);
				q0 = q0 & dVector::m_triplexMask;
				for (dInt32 j = 0; j < vCount; j ++) 
				{
					dInt32 j1 = vCount + 2 + j;
					dVector q1 (&vertexArray[face[j]].m_x);
					q1 = q1 & dVector::m_triplexMask;
					if (face[j0] == -1) 
					{
						dVector e (q1 - q0);
						dVector n (e.CrossProduct(normal).Normalize());
						dAssert (dAbs (n.DotProduct(n).GetScalar() - dFloat32 (1.0f)) < dFloat32 (1.0e-6f));
						pool[normalCount].m_x = n.m_x;
						pool[normalCount].m_y = n.m_y;
						pool[normalCount].m_z = n.m_z;
						face[j0] = -normalCount - 1;
						normalCount ++;
					}
					q0 = q1;
					j0 = j1;
				}
			}
		}
	
		if (node->m_right.IsLeaf()) {
			dInt32 vCount = dInt32 (node->m_right.GetCount());
			if (vCount) {
				dInt32 index = dInt32 (node->m_right.GetIndex());
				dInt32* const face = &m_indices[index];
	
				dInt32 j0 = 2 * (vCount + 1) - 1;
				dVector normal (&vertexArray[face[vCount + 1]].m_x);
				normal = normal & dVector::m_triplexMask;
				dAssert (dAbs (normal.DotProduct(normal).GetScalar() - dFloat32 (1.0f)) < dFloat32 (1.0e-6f));
				dVector q0 (&vertexArray[face[vCount - 1]].m_x);
				q0 = q0 & dVector::m_triplexMask;
				for (dInt32 j = 0; j < vCount; j ++) {
					dInt32 j1 = vCount + 2 + j;
					dVector q1 (&vertexArray[face[j]].m_x);
					q1 = q1 & dVector::m_triplexMask;
					if (face[j0] == -1) {
						dVector e (q1 - q0);
						dVector n (e.CrossProduct(normal).Normalize());
						dAssert (dAbs (n.DotProduct(n).GetScalar() - dFloat32 (1.0f)) < dFloat32 (1.0e-6f));
						pool[normalCount].m_x = n.m_x;
						pool[normalCount].m_y = n.m_y;
						pool[normalCount].m_z = n.m_z;
						face[j0] = -normalCount - 1;
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
		dStack<dInt32> indexArray (normalCount);
		dInt32 newNormalCount = dVertexListToIndexList (&pool[0].m_x, sizeof (dTriplex), sizeof (dTriplex), 0, normalCount, &indexArray[0], dFloat32 (1.0e-6f));
	
		dInt32 oldCount = GetVertexCount();
		dTriplex* const vertexArray1 = (dTriplex*)dMemory::Malloc (sizeof (dTriplex) * (oldCount + newNormalCount));
		memcpy (vertexArray1, GetLocalVertexPool(), sizeof (dTriplex) * oldCount);
		memcpy (&vertexArray1[oldCount], &pool[0].m_x, sizeof (dTriplex) * newNormalCount);
		dMemory::Free(GetLocalVertexPool());
	
		m_localVertex = &vertexArray1[0].m_x;
		m_vertexCount = oldCount + newNormalCount;
	
		for (dInt32 i = 0; i < m_nodesCount; i ++) 
		{
			const dNode* const node = &m_aabb[i];
			if (node->m_left.IsLeaf()) 
			{
				dInt32 vCount = dInt32 (node->m_left.GetCount());
				dInt32 index = dInt32 (node->m_left.GetIndex());
				dInt32* const face = &m_indices[index];
				for (dInt32 j = 0; j < vCount; j ++) 
				{
					if (face[vCount + 2 + j] < 0) 
					{
						dInt32 k = -1 - face[vCount + 2 + j];
						face[vCount + 2 + j] = indexArray[k] + oldCount;
					}
					#ifdef _DEBUG	
						dVector normal (&vertexArray1[face[vCount + 2 + j]].m_x);
						normal = normal & dVector::m_triplexMask;
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
					if (face[vCount + 2 + j] < 0) 
					{
						dInt32 k = -1 - face[vCount + 2 + j];
						face[vCount + 2 + j] = indexArray[k] + oldCount;
					}
	
					#ifdef _DEBUG	
						dVector normal (&vertexArray1[face[vCount + 2 + j]].m_x);
						normal = normal & dVector::m_triplexMask;
						dAssert (dAbs (normal.DotProduct(normal).GetScalar() - dFloat32 (1.0f)) < dFloat32 (1.0e-6f));
					#endif
				}
			}
		}
	}
}


dIntersectStatus dAabbPolygonSoup::CalculateAllFaceEdgeNormalsOld (void* const context, const dFloat32* const polygon, dInt32 strideInBytes, const dInt32* const indexArray, dInt32 indexCount, dFloat32)
{
	dInt32 stride = dInt32 (strideInBytes / sizeof (dFloat32));

	AdjacentdFace adjacentFaces;
	adjacentFaces.m_count = indexCount;
	adjacentFaces.m_index = (dInt32*) indexArray;

	dVector n (&polygon[indexArray[indexCount + 1] * stride]);
	dVector p (&polygon[indexArray[0] * stride]);
	n = n & dVector::m_triplexMask;
	p = p & dVector::m_triplexMask;
	adjacentFaces.m_normal = dPlane (n, - n.DotProduct(p).GetScalar());

	dAssert (indexCount < dInt32 (sizeof (adjacentFaces.m_edgeMap) / sizeof (adjacentFaces.m_edgeMap[0])));

	dInt32 edgeIndex = indexCount - 1;
	dInt32 i0 = indexArray[indexCount - 1];
	dVector p0 ( dFloat32 (1.0e15f),  dFloat32 (1.0e15f),  dFloat32 (1.0e15f), dFloat32 (0.0f)); 
	dVector p1 (-dFloat32 (1.0e15f), -dFloat32 (1.0e15f), -dFloat32 (1.0e15f), dFloat32 (0.0f)); 
	for (dInt32 i = 0; i < indexCount; i ++) 
	{
		dInt32 i1 = indexArray[i];
		dInt32 index = i1 * stride;
		dVector point (&polygon[index]);
		point = point & dVector::m_triplexMask;
		p0 = p0.GetMin(point);
		p1 = p1.GetMax(point);
		adjacentFaces.m_edgeMap[edgeIndex] = (dInt64 (i1) << 32) + i0;
		edgeIndex = i;
		i0 = i1;
	}

	dFloat32 padding = dFloat32 (1.0f/16.0f);
	p0.m_x -= padding;
	p0.m_y -= padding;
	p0.m_z -= padding;
	p1.m_x += padding;
	p1.m_y += padding;
	p1.m_z += padding;

	dAabbPolygonSoup* const me = (dAabbPolygonSoup*) context;
	dFastAabbInfo box (p0, p1);
	me->ForAllSectors (box, dVector::m_zero, dFloat32 (1.0f), CalculateDisjointedFaceEdgeNormals, &adjacentFaces);
	return t_ContinueSearh;
}

dIntersectStatus dAabbPolygonSoup::CalculateAllFaceEdgeNormals(void* const context, const dFloat32* const, dInt32, const dInt32* const indexArray, dInt32 indexCount, dFloat32)
{
	//dInt32 stride = dInt32(strideInBytes / sizeof(dFloat32));
	//
	//AdjacentdFace adjacentFaces;
	//adjacentFaces.m_count = indexCount;
	//adjacentFaces.m_index = (dInt32*)indexArray;
	//
	//dVector n(&polygon[indexArray[indexCount + 1] * stride]);
	//dVector p(&polygon[indexArray[0] * stride]);
	//n = n & dVector::m_triplexMask;
	//p = p & dVector::m_triplexMask;
	//adjacentFaces.m_normal = dPlane(n, -n.DotProduct(p).GetScalar());
	//
	//dAssert(indexCount < dInt32(sizeof(adjacentFaces.m_edgeMap) / sizeof(adjacentFaces.m_edgeMap[0])));
	//
	//dInt32 edgeIndex = indexCount - 1;
	//dInt32 i0 = indexArray[indexCount - 1];
	//dVector p0(dFloat32(1.0e15f), dFloat32(1.0e15f), dFloat32(1.0e15f), dFloat32(0.0f));
	//dVector p1(-dFloat32(1.0e15f), -dFloat32(1.0e15f), -dFloat32(1.0e15f), dFloat32(0.0f));
	//for (dInt32 i = 0; i < indexCount; i++)
	//{
	//	dInt32 i1 = indexArray[i];
	//	dInt32 index = i1 * stride;
	//	dVector point(&polygon[index]);
	//	point = point & dVector::m_triplexMask;
	//	p0 = p0.GetMin(point);
	//	p1 = p1.GetMax(point);
	//	adjacentFaces.m_edgeMap[edgeIndex] = (dInt64(i1) << 32) + i0;
	//	edgeIndex = i;
	//	i0 = i1;
	//}
	//
	//dFloat32 padding = dFloat32(1.0f / 16.0f);
	//p0.m_x -= padding;
	//p0.m_y -= padding;
	//p0.m_z -= padding;
	//p1.m_x += padding;
	//p1.m_y += padding;
	//p1.m_z += padding;
	//
	//dAabbPolygonSoup* const me = (dAabbPolygonSoup*)context;
	//dFastAabbInfo box(p0, p1);
	//me->ForAllSectors(box, dVector::m_zero, dFloat32(1.0f), CalculateDisjointedFaceEdgeNormals, &adjacentFaces);

	dInt32 face[256];
	dInt64 data[256];
	dPolyhedra& adjacency = *((dPolyhedra*)context);
	for (dInt32 i = 0; i < indexCount; i++)
	{
		face[i] = indexArray[i];
		data[i] = dInt64(indexArray);
	}
	adjacency.AddFace(indexCount, face, data);
	return t_ContinueSearh;
}

dIntersectStatus dAabbPolygonSoup::CalculateDisjointedFaceEdgeNormals (void* const context, const dFloat32* const polygon, dInt32 strideInBytes, const dInt32* const indexArray, dInt32 indexCount, dFloat32)
{
	#define DG_WELDING_TOL (1.0e-2f)
	#define DG_WELDING_TOL2 (DG_WELDING_TOL * DG_WELDING_TOL)

	const AdjacentdFace& adjacentFace = *((AdjacentdFace*)context);

	if (adjacentFace.m_index != indexArray) {	
		dInt32 adjacentCount = adjacentFace.m_count;
		dInt32 stride = dInt32 (strideInBytes / sizeof (dFloat32));

		dInt32 j0 = adjacentCount - 1;
		dInt32 indexJ0 = adjacentFace.m_index[adjacentCount - 1];
		for (dInt32 j = 0; j < adjacentCount; j ++) {
			dInt32 indexJ1 = adjacentFace.m_index[j];
			dBigVector q0 (dVector(&polygon[indexJ1 * stride]) & dVector::m_triplexMask);
			dBigVector q1 (dVector(&polygon[indexJ0 * stride]) & dVector::m_triplexMask);
			dBigVector q1q0 (q1 - q0);
			dFloat64 q1q0Mag2 = q1q0.DotProduct(q1q0).GetScalar();

			dInt32 indexI0 = indexArray[indexCount - 1];
			for (dInt32 i = 0; i < indexCount; i ++) 
			{
				dInt32 indexI1 = indexArray[i];
				dBigVector p0 (dVector(&polygon[indexI0 * stride]) & dVector::m_triplexMask);
				dBigVector p1 (dVector(&polygon[indexI1 * stride]) & dVector::m_triplexMask);
				dBigVector p1p0 (p1 - p0);
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
							dBigVector q (q0 + q1q0.Scale(t));
							dBigVector dist (p0 - q);
							dAssert (dist.m_w == dFloat32 (0.0f));
							dFloat64 err2 = dist.DotProduct(dist).GetScalar();
							if (err2 < DG_WELDING_TOL2) 
							{
								dFloat32 maxDist = dFloat32 (0.0f);
								for (dInt32 k = 0; k < indexCount; k ++) 
								{
									dVector r (&polygon[indexArray[k] * stride]);
									r = r & dVector::m_triplexMask;
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
										dBigVector n0 (adjacentFace.m_normal[0], adjacentFace.m_normal[1], adjacentFace.m_normal[2], dFloat64(0.0f));
										dBigVector n1 (dVector(&polygon[adjacentFace.m_index[j0 + adjacentCount + 2] * stride]) & dVector::m_triplexMask);
										dBigVector n2 (dVector(&polygon[indexArray[indexCount + 1] * stride]) & dVector::m_triplexMask);

										dBigVector tilt0 (n0.CrossProduct(n1)); 
										dBigVector tilt1 (n0.CrossProduct(n2)); 
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

dAabbPolygonSoup::dgNodeBuilder* dAabbPolygonSoup::BuildTopDown (dgNodeBuilder* const leafArray, dInt32 firstBox, dInt32 lastBox, dgNodeBuilder** const allocator) const
{
	dAssert (firstBox >= 0);
	dAssert (lastBox >= 0);

	if (lastBox == firstBox) 
	{
		return &leafArray[firstBox];
	} 
	else 
	{
		dgSpliteInfo info (&leafArray[firstBox], lastBox - firstBox + 1);

		dgNodeBuilder* const parent = new (*allocator) dgNodeBuilder (info.m_p0, info.m_p1);
		*allocator = *allocator + 1;

		dAssert (parent);
		parent->m_right = BuildTopDown (leafArray, firstBox + info.m_axis, lastBox, allocator);
		parent->m_right->m_parent = parent;

		parent->m_left = BuildTopDown (leafArray, firstBox, firstBox + info.m_axis - 1, allocator);
		parent->m_left->m_parent = parent;
		return parent;
	}
}

void dAabbPolygonSoup::Create (const dPolygonSoupBuilder& builder)
{
	if (builder.m_faceVertexCount.GetCount() == 0) 
	{
		return;
	}
	dAssert (builder.m_faceVertexCount.GetCount() >= 1);
	m_strideInBytes = sizeof (dTriplex);
	m_nodesCount = ((builder.m_faceVertexCount.GetCount() - 1) < 1) ? 1 : builder.m_faceVertexCount.GetCount() - 1;
	m_aabb = (dNode*) dMemory::Malloc (sizeof (dNode) * m_nodesCount);
	m_indexCount = builder.m_vertexIndex.GetCount() * 2 + builder.m_faceVertexCount.GetCount();

	if (builder.m_faceVertexCount.GetCount() == 1) 
	{
		m_indexCount *= 2;
	}
	m_indices = (dInt32*) dMemory::Malloc (sizeof (dInt32) * m_indexCount);
	dStack<dVector> tmpVertexArrayCont(builder.m_vertexPoints.GetCount() + builder.m_normalPoints.GetCount() + builder.m_faceVertexCount.GetCount() * 2 + 4);

	dVector* const tmpVertexArray = &tmpVertexArrayCont[0];
	for (dInt32 i = 0; i < builder.m_vertexPoints.GetCount(); i ++) 
	{
		tmpVertexArray[i] = builder.m_vertexPoints[i];
	}

	for (dInt32 i = 0; i < builder.m_normalPoints.GetCount(); i ++) 
	{
		tmpVertexArray[i + builder.m_vertexPoints.GetCount()] = builder.m_normalPoints[i];
	}

	const dInt32* const indices = &builder.m_vertexIndex[0];
	dStack<dgNodeBuilder> constructor (builder.m_faceVertexCount.GetCount() * 2 + 16); 

	dInt32 polygonIndex = 0;
	dInt32 allocatorIndex = 0;
	if (builder.m_faceVertexCount.GetCount() == 1) 
	{
		dInt32 indexCount = builder.m_faceVertexCount[0] - 1;
		new (&constructor[allocatorIndex]) dgNodeBuilder (&tmpVertexArray[0], 0, indexCount, &indices[0]);
		allocatorIndex ++;
	}
	for (dInt32 i = 0; i < builder.m_faceVertexCount.GetCount(); i ++) 
	{
		dInt32 indexCount = builder.m_faceVertexCount[i] - 1;
		new (&constructor[allocatorIndex]) dgNodeBuilder (&tmpVertexArray[0], i, indexCount, &indices[polygonIndex]);
		allocatorIndex ++;
		polygonIndex += (indexCount + 1);
	}

	dgNodeBuilder* contructorAllocator = &constructor[allocatorIndex];
	dgNodeBuilder* root = BuildTopDown (&constructor[0], 0, allocatorIndex - 1, &contructorAllocator);

	dAssert (root);
	if (root->m_left) 
	{
		dAssert (root->m_right);
		dList<dgNodeBuilder*> list;
		dList<dgNodeBuilder*> stack;
		stack.Append(root);
		while (stack.GetCount()) 
		{
			dList<dgNodeBuilder*>::dListNode* const stackNode = stack.GetLast();
			dgNodeBuilder* const node = stackNode->GetInfo();
			stack.Remove(stackNode);

			if (node->m_left) 
			{
				dAssert (node->m_right);
				list.Append(node);
				stack.Append(node->m_right);
				stack.Append(node->m_left);
			} 
		}

		dFloat64 newCost = dFloat32 (1.0e20f);
		dFloat64 prevCost = newCost;
		do 
		{
			prevCost = newCost;
			for (dList<dgNodeBuilder*>::dListNode* listNode = list.GetFirst(); listNode; listNode = listNode->GetNext()) 
			{
				dgNodeBuilder* const node = listNode->GetInfo();
				ImproveNodeFitness (node);
			}

			newCost = dFloat32 (0.0f);
			for (dList<dgNodeBuilder*>::dListNode* listNode = list.GetFirst(); listNode; listNode = listNode->GetNext()) 
			{
				dgNodeBuilder* const node = listNode->GetInfo();
				newCost += node->m_area;
			}
		} while (newCost < (prevCost * dFloat32 (0.9999f)));

		root = list.GetLast()->GetInfo();
		while (root->m_parent) 
		{
			root = root->m_parent;
		}
	}

	dList<dgNodeBuilder*> list;
	list.Append(root);
	dInt32 nodeIndex = 0;
	while (list.GetCount())
	{
		dgNodeBuilder* const node = list.GetFirst()->GetInfo();
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

	dVector* const aabbPoints = &tmpVertexArray[aabbBase];

	dInt32 vertexIndex = 0;
	dInt32 aabbNodeIndex = 0;
	list.Append(root);
	dInt32 indexMap = 0;
	while (list.GetCount())
	{
		dgNodeBuilder* const node = list.GetFirst()->GetInfo();
		list.Remove(list.GetFirst());

		if (node->m_enumeration >= 0)
		{
			dAssert (node->m_left);
			dAssert (node->m_right);
			dNode& aabbNode = m_aabb[aabbNodeIndex];
			aabbNodeIndex ++;
			dAssert (aabbNodeIndex <= m_nodesCount);

			if (node->m_parent)
			{
				if (node->m_parent->m_left == node)
				{
					m_aabb[node->m_parent->m_enumeration].m_left = dNode::dgLeafNodePtr (dUnsigned32 (&m_aabb[node->m_enumeration] - m_aabb));
				}
				else 
				{
					dAssert (node->m_parent->m_right == node);
					m_aabb[node->m_parent->m_enumeration].m_right = dNode::dgLeafNodePtr (dUnsigned32 (&m_aabb[node->m_enumeration] - m_aabb));
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
					m_aabb[node->m_parent->m_enumeration].m_left = dNode::dgLeafNodePtr (dUnsigned32(node->m_indexCount), dUnsigned32(indexMap));
				}
				else 
				{
					dAssert (node->m_parent->m_right == node);
					m_aabb[node->m_parent->m_enumeration].m_right = dNode::dgLeafNodePtr (dUnsigned32(node->m_indexCount), dUnsigned32(indexMap));
				}
			}

			// index format i0, i1, i2, ... , id, normal, e0Normal, e1Normal, e2Normal, ..., faceSize
			for (dInt32 j = 0; j < node->m_indexCount; j ++) 
			{
				m_indices[indexMap + j] = node->m_faceIndices[j];
				m_indices[indexMap + j + node->m_indexCount + 2] = -1;
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

	dStack<dInt32> indexArray (vertexIndex);
	dInt32 aabbPointCount = dVertexListToIndexList (&aabbPoints[0].m_x, sizeof (dVector), sizeof (dTriplex), 0, vertexIndex, &indexArray[0], dFloat32 (1.0e-6f));

	m_vertexCount = aabbBase + aabbPointCount;
	m_localVertex = (dFloat32*) dMemory::Malloc (sizeof (dTriplex) * m_vertexCount);

	dTriplex* const dstPoints = (dTriplex*)m_localVertex;
	for (dInt32 i = 0; i < m_vertexCount; i ++) 
	{
		dstPoints[i].m_x = tmpVertexArray[i].m_x;
		dstPoints[i].m_y = tmpVertexArray[i].m_y;
		dstPoints[i].m_z = tmpVertexArray[i].m_z;
	}

	for (dInt32 i = 0; i < m_nodesCount; i ++) 
	{
		dNode& box = m_aabb[i];

		dInt32 j = box.m_indexBox0 - aabbBase;
		box.m_indexBox0 = indexArray[j] + aabbBase;

		j = box.m_indexBox1 - aabbBase;
		box.m_indexBox1 = indexArray[j] + aabbBase;
	}

	if (builder.m_faceVertexCount.GetCount() == 1) 
	{
		m_aabb[0].m_right = dNode::dgLeafNodePtr (0, 0);
	}
}

void dAabbPolygonSoup::Serialize (const char* const path) const
{
	FILE* const file = fopen(path, "wb");
	if (file)
	{
		fwrite(&m_vertexCount, sizeof(dInt32), 1, file);
		fwrite(&m_indexCount, sizeof(dInt32), 1, file);
		fwrite(&m_nodesCount, sizeof(dInt32), 1, file);
		if (m_aabb)
		{
			fwrite(m_localVertex, sizeof(dTriplex) * m_vertexCount, 1, file);
			fwrite(m_indices, sizeof(dInt32) * m_indexCount, 1, file);
			fwrite(m_aabb, sizeof(dNode) * m_nodesCount, 1, file);
		}
		fclose(file);
	}
}

void dAabbPolygonSoup::Deserialize (const char* const path)
{
	FILE* const file = fopen(path, "rb");
	if (file)
	{
		m_strideInBytes = sizeof(dTriplex);
		//callback(userData, &m_vertexCount, sizeof(dInt32));
		//callback(userData, &m_indexCount, sizeof(dInt32));
		//callback(userData, &m_nodesCount, sizeof(dInt32));
		fread(&m_vertexCount, sizeof(dInt32), 1, file);
		fread(&m_indexCount, sizeof(dInt32), 1, file);
		fread(&m_nodesCount, sizeof(dInt32), 1, file);

		if (m_vertexCount) 
		{
			m_localVertex = (dFloat32*)dMemory::Malloc(sizeof(dTriplex) * m_vertexCount);
			m_indices = (dInt32*)dMemory::Malloc(sizeof(dInt32) * m_indexCount);
			m_aabb = (dNode*)dMemory::Malloc(sizeof(dNode) * m_nodesCount);

			//callback(userData, m_localVertex, dInt32(sizeof(dTriplex) * m_vertexCount));
			//callback(userData, m_indices, dInt32(sizeof(dInt32) * m_indexCount));
			//callback(userData, m_aabb, dInt32(sizeof(dNode) * m_nodesCount));
			fread(m_localVertex, sizeof(dTriplex) * m_vertexCount, 1, file);
			fread(m_indices, sizeof(dInt32) * m_indexCount, 1, file);
			fread(m_aabb, sizeof(dNode) * m_nodesCount, 1, file);
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

dVector dAabbPolygonSoup::ForAllSectorsSupportVectex (const dVector& dir) const
{
	dVector supportVertex (dFloat32 (0.0f));
	if (m_aabb) 
	{
		dFloat32 aabbProjection[DG_STACK_DEPTH];
		const dNode *stackPool[DG_STACK_DEPTH];

		dInt32 stack = 1;
		stackPool[0] = m_aabb;
		aabbProjection[0] = dFloat32 (1.0e10f);
		const dTriplex* const boxArray = (dTriplex*)m_localVertex;

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
				const dNode* const me = stackPool[stack];
				if (me->m_left.IsLeaf()) 
				{
					backSupportDist = dFloat32 (-1.0e20f);
					dInt32 index = dInt32 (me->m_left.GetIndex());
					dInt32 vCount = dInt32 (me->m_left.GetCount());
					dVector vertex (dFloat32 (0.0f));
					for (dInt32 j = 0; j < vCount; j ++) 
					{
						dInt32 i0 = m_indices[index + j] * dInt32 (sizeof (dTriplex) / sizeof (dFloat32));
						dVector p (&boxArray[i0].m_x);
						p = p & dVector::m_triplexMask;
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
					dVector box[2];
					const dNode* const node = me->m_left.GetNode(m_aabb);
					box[0].m_x = boxArray[node->m_indexBox0].m_x;
					box[0].m_y = boxArray[node->m_indexBox0].m_y;
					box[0].m_z = boxArray[node->m_indexBox0].m_z;
					box[1].m_x = boxArray[node->m_indexBox1].m_x;
					box[1].m_y = boxArray[node->m_indexBox1].m_y;
					box[1].m_z = boxArray[node->m_indexBox1].m_z;

					dVector supportPoint (box[ix].m_x, box[iy].m_y, box[iz].m_z, dFloat32 (0.0));
					backSupportDist = supportPoint.DotProduct(dir).GetScalar();
				}

				if (me->m_right.IsLeaf()) 
				{
					frontSupportDist = dFloat32 (-1.0e20f);
					dInt32 index = dInt32 (me->m_right.GetIndex());
					dInt32 vCount = dInt32 (me->m_right.GetCount());
					dVector vertex (dFloat32 (0.0f));
					for (dInt32 j = 0; j < vCount; j ++) 
					{
						dInt32 i0 = m_indices[index + j] * dInt32 (sizeof (dTriplex) / sizeof (dFloat32));
						dVector p (&boxArray[i0].m_x);
						p = p & dVector::m_triplexMask;
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
					dVector box[2];
					const dNode* const node = me->m_right.GetNode(m_aabb);
					box[0].m_x = boxArray[node->m_indexBox0].m_x;
					box[0].m_y = boxArray[node->m_indexBox0].m_y;
					box[0].m_z = boxArray[node->m_indexBox0].m_z;
					box[1].m_x = boxArray[node->m_indexBox1].m_x;
					box[1].m_y = boxArray[node->m_indexBox1].m_y;
					box[1].m_z = boxArray[node->m_indexBox1].m_z;

					dVector supportPoint (box[ix].m_x, box[iy].m_y, box[iz].m_z, dFloat32 (0.0f));
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

void dAabbPolygonSoup::ForAllSectorsRayHit (const dFastRayTest& raySrc, dFloat32 maxParam, dRayIntersectCallback callback, void* const context) const
{
	const dNode *stackPool[DG_STACK_DEPTH];
	dFloat32 distance[DG_STACK_DEPTH];
	dFastRayTest ray (raySrc);

	dInt32 stack = 1;
	const dTriplex* const vertexArray = (dTriplex*) m_localVertex;

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
			const dNode *const me = stackPool[stack];
			if (me->m_left.IsLeaf()) 
			{
				dInt32 vCount = dInt32 (me->m_left.GetCount());
				if (vCount > 0) 
				{
					dInt32 index = dInt32 (me->m_left.GetIndex());
					dFloat32 param = callback(context, &vertexArray[0].m_x, sizeof (dTriplex), &m_indices[index], vCount);
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
				const dNode* const node = me->m_left.GetNode(m_aabb);
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
					dFloat32 param = callback(context, &vertexArray[0].m_x, sizeof (dTriplex), &m_indices[index], vCount);
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
				const dNode* const node = me->m_right.GetNode(m_aabb);
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

void dAabbPolygonSoup::ForAllSectors (const dFastAabbInfo& obbAabbInfo, const dVector& boxDistanceTravel, dFloat32, dAaabbIntersectCallback callback, void* const context) const
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
		const dNode* stackPool[DG_STACK_DEPTH];

		const dInt32 stride = sizeof (dTriplex) / sizeof (dFloat32);
		const dTriplex* const vertexArray = (dTriplex*) m_localVertex;

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
					const dNode* const me = stackPool[stack];
					if (me->m_left.IsLeaf()) 
					{
						dInt32 index = dInt32 (me->m_left.GetIndex());
						dInt32 vCount = dInt32 (me->m_left.GetCount());
						if (vCount > 0) 
						{
							const dInt32* const indices = &m_indices[index];
							dInt32 normalIndex = indices[vCount + 1];
							dVector faceNormal (&vertexArray[normalIndex].m_x);
							faceNormal = faceNormal & dVector::m_triplexMask;
							dFloat32 dist1 = obbAabbInfo.PolygonBoxDistance (faceNormal, vCount, indices, stride, &vertexArray[0].m_x);
							if (dist1 > dFloat32 (0.0f)) 
							{
								obbAabbInfo.m_separationDistance = dFloat32(0.0f);
								dAssert (vCount >= 3);
								if (callback(context, &vertexArray[0].m_x, sizeof (dTriplex), indices, vCount, dist1) == t_StopSearh) 
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
						const dNode* const node = me->m_left.GetNode(m_aabb);
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
							dVector faceNormal (&vertexArray[normalIndex].m_x);
							faceNormal = faceNormal & dVector::m_triplexMask;
							dFloat32 dist1 = obbAabbInfo.PolygonBoxDistance (faceNormal, vCount, indices, stride, &vertexArray[0].m_x);
							if (dist1 > dFloat32 (0.0f)) 
							{
								dAssert (vCount >= 3);
								obbAabbInfo.m_separationDistance = dFloat32(0.0f);
								if (callback(context, &vertexArray[0].m_x, sizeof (dTriplex), indices, vCount, dist1) == t_StopSearh) 
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
						const dNode* const node = me->m_right.GetNode(m_aabb);
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
			dFastRayTest ray (dVector (dFloat32 (0.0f)), boxDistanceTravel);
			dFastRayTest obbRay (dVector (dFloat32 (0.0f)), obbAabbInfo.UnrotateVector(boxDistanceTravel));
			dInt32 stack = 1;
			stackPool[0] = m_aabb;
			distance [0] = m_aabb->BoxIntersect (ray, obbRay, obbAabbInfo, vertexArray);

			while (stack) 
			{
				stack --;
				const dFloat32 dist = distance[stack];
				const dNode* const me = stackPool[stack];
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
							dVector faceNormal (&vertexArray[normalIndex].m_x);
							faceNormal = faceNormal & dVector::m_triplexMask;
							dFloat32 hitDistance = obbAabbInfo.PolygonBoxRayDistance (faceNormal, vCount, indices, stride, &vertexArray[0].m_x, ray);
							if (hitDistance < dFloat32 (1.0f)) 
							{
								dAssert (vCount >= 3);
								if (callback(context, &vertexArray[0].m_x, sizeof (dTriplex), indices, vCount, hitDistance) == t_StopSearh) 
								{
									return;
								}
							}
						}
					} 
					else 
					{
						const dNode* const node = me->m_left.GetNode(m_aabb);
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
							dVector faceNormal (&vertexArray[normalIndex].m_x);
							faceNormal = faceNormal & dVector::m_triplexMask;
							dFloat32 hitDistance = obbAabbInfo.PolygonBoxRayDistance (faceNormal, vCount, indices, stride, &vertexArray[0].m_x, ray);
							if (hitDistance < dFloat32 (1.0f)) 
							{
								dAssert (vCount >= 3);
								if (callback(context, &vertexArray[0].m_x, sizeof (dTriplex), indices, vCount, hitDistance) == t_StopSearh) {
									return;
								}
							}
						}
					} 
					else 
					{
						const dNode* const node = me->m_right.GetNode(m_aabb);
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


