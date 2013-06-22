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

#include "dgStdafx.h"
#include "dgHeap.h"
#include "dgStack.h"
#include "dgList.h"
#include "dgMatrix.h"
#include "dgAABBPolygonSoup.h"
#include "dgPolygonSoupBuilder.h"


#define DG_STACK_DEPTH 256

class dgAABBPolygonSoup::dgConstructionTree
{
	public:
	DG_CLASS_ALLOCATOR(allocator)

	dgConstructionTree (dgMemoryAllocator* const allocator, dgInt32 firstBox, dgInt32 lastBox, dgNode* const boxArray, const dgVector* const vertexArray, dgConstructionTree* const parent)
		:m_parent(parent)
	{
		dgAssert (firstBox >= 0);
		dgAssert (lastBox >= 0);

		if (lastBox == firstBox) {
			dgInt32 j0 = boxArray[firstBox].m_minIndex;
			dgInt32 j1 = boxArray[firstBox].m_maxIndex;
			m_p0 = vertexArray[j0];
			m_p1 = vertexArray[j1];

			m_boxIndex = firstBox;
			m_back = NULL;
			m_front = NULL;
		} else {

			struct dgSpliteInfo
			{
				dgSpliteInfo (dgNode* const boxArray, dgInt32 boxCount, const dgVector* const vertexArray, const dgConstructionTree* const tree)
				{

					dgVector minP ( dgFloat32 (1.0e15f)); 
					dgVector maxP (-dgFloat32 (1.0e15f)); 

					if (boxCount == 2) {
						m_axis = 1;

						for (dgInt32 i = 0; i < boxCount; i ++) {
							dgInt32 j0 = boxArray[i].m_minIndex;
							dgInt32 j1 = boxArray[i].m_maxIndex;

							const dgVector& p0 = vertexArray[j0];
							const dgVector& p1 = vertexArray[j1];
							minP = minP.GetMin (p0); 
							maxP = maxP.GetMax (p1); 
						}

					} else {
						dgVector median (dgFloat32 (0.0f));
						dgVector varian (dgFloat32 (0.0f));
						for (dgInt32 i = 0; i < boxCount; i ++) {

							dgInt32 j0 = boxArray[i].m_minIndex;
							dgInt32 j1 = boxArray[i].m_maxIndex;

							const dgVector& p0 = vertexArray[j0];
							const dgVector& p1 = vertexArray[j1];

							minP = minP.GetMin (p0); 
							maxP = maxP.GetMax (p1); 
							dgVector p ((p0 + p1).CompProduct4(dgVector::m_half));

							median += p;
							varian += p.CompProduct4(p);
						}

						varian = varian.Scale4 (dgFloat32 (boxCount)) - median.CompProduct4(median);


						dgInt32 index = 0;
						dgFloat32 maxVarian = dgFloat32 (-1.0e10f);
						for (dgInt32 i = 0; i < 3; i ++) {
							if (varian[i] > maxVarian) {
								index = i;
								maxVarian = varian[i];
							}
						}

						dgVector center = median.Scale4 (dgFloat32 (1.0f) / dgFloat32 (boxCount));

						dgFloat32 test = center[index];

						dgInt32 i0 = 0;
						dgInt32 i1 = boxCount - 1;
						do {    
							for (; i0 <= i1; i0 ++) {
								dgInt32 j0 = boxArray[i0].m_minIndex;
								dgInt32 j1 = boxArray[i0].m_maxIndex;

								dgFloat32 val = (vertexArray[j0][index] + vertexArray[j1][index]) * dgFloat32 (0.5f);
								if (val > test) {
									break;
								}
							}

							for (; i1 >= i0; i1 --) {
								dgInt32 j0 = boxArray[i1].m_minIndex;
								dgInt32 j1 = boxArray[i1].m_maxIndex;

								dgFloat32 val = (vertexArray[j0][index] + vertexArray[j1][index]) * dgFloat32 (0.5f);
								if (val < test) {
									break;
								}
							}

							if (i0 < i1)	{
								dgSwap(boxArray[i0], boxArray[i1]);
								i0++; 
								i1--;
							}
						} while (i0 <= i1);

						if (i0 > 0){
							i0 --;
						}
						if ((i0 + 1) >= boxCount) {
							i0 = boxCount - 2;
						}

						m_axis = i0 + 1;
					}

					dgAssert (maxP.m_x - minP.m_x >= dgFloat32 (0.0f));
					dgAssert (maxP.m_y - minP.m_y >= dgFloat32 (0.0f));
					dgAssert (maxP.m_z - minP.m_z >= dgFloat32 (0.0f));
					m_p0 = minP;
					m_p1 = maxP;
				}

				dgInt32 m_axis;
				dgVector m_p0;
				dgVector m_p1;
			};

			dgSpliteInfo info (&boxArray[firstBox], lastBox - firstBox + 1, vertexArray, this);

			m_boxIndex = -1;
			m_p0 = info.m_p0;
			m_p1 = info.m_p1;

			m_front = new (allocator) dgConstructionTree (allocator, firstBox + info.m_axis, lastBox, boxArray, vertexArray, this);
			m_back = new (allocator) dgConstructionTree (allocator, firstBox, firstBox + info.m_axis - 1, boxArray, vertexArray, this);
		}

		dgVector side0 (m_p1 - m_p0);
		m_surfaceArea = side0.CompProduct4(side0.ShiftTripleRight()).m_x; 
	}

	~dgConstructionTree ()
	{
		if (m_back) {
			delete m_back;
		}
		if (m_front) {
			delete m_front;
		}
	}

	dgInt32 GetAxis (dgConstructionTree** const boxArray, dgInt32 boxCount) const
	{
		dgVector median (dgFloat32 (0.0f));
		dgVector varian (dgFloat32 (0.0f));
		for (dgInt32 i = 0; i < boxCount; i ++) {

			median += boxArray[i]->m_p0;
			median += boxArray[i]->m_p1;

			varian += boxArray[i]->m_p0.CompProduct3(boxArray[i]->m_p0);
			varian += boxArray[i]->m_p1.CompProduct3(boxArray[i]->m_p1);
		}

		boxCount *= 2;
		varian.m_x = boxCount * varian.m_x - median.m_x * median.m_x;
		varian.m_y = boxCount * varian.m_y - median.m_y * median.m_y;
		varian.m_z = boxCount * varian.m_z - median.m_z * median.m_z;

		dgInt32 axis = 0;
		dgFloat32 maxVal = varian[0];
		for (dgInt32 i = 1; i < 3; i ++) {
			if (varian[i] > maxVal) {
				axis = i;
				maxVal = varian[i];
			}
		}

		return axis;
	}

	dgInt32 CompareBox (const dgConstructionTree* const boxA, const dgConstructionTree* const boxB, void* const context)
	{
		dgInt32 axis = *((dgInt32*) context);

		if (boxA->m_p0[axis] < boxB->m_p0[axis]) {
			return -1;
		} else if (boxA->m_p0[axis] > boxB->m_p0[axis]) {
			return 1;
		}
		return 0;
	}


	dgInt32 BuildTree (dgNode* const boxArray, dgNode* const boxCopy, dgVector* const vertexArrayOut, dgInt32 &treeVCount) const
	{
		dgNode::dgLeafNodePtr* parent[128];
		const dgConstructionTree* pool[128];

		dgInt32 index = 0;
		dgInt32 stack = 1;
		parent[0] = NULL;
		pool[0] = this;
		while (stack) {
			stack --;

			const dgConstructionTree* const node = pool[stack];
			dgNode::dgLeafNodePtr* const parentNode = parent[stack];
			if (node->m_boxIndex != -1) {
				if (parentNode) {
					*parentNode = boxCopy[node->m_boxIndex].m_back;
				} else {

					//dgAssert (boxCount == 1);
					dgNode* const newNode = &boxArray[index];
					*newNode = boxCopy[node->m_boxIndex];

					newNode->m_minIndex = treeVCount;
					vertexArrayOut[treeVCount] = node->m_p0;

					newNode->m_maxIndex = treeVCount + 1;
					vertexArrayOut[treeVCount + 1] = node->m_p1;
					treeVCount += 2;

					index ++;
				}

			} else {
				dgNode* const newNode = &boxArray[index];

				newNode->m_minIndex = treeVCount;
				vertexArrayOut[treeVCount] = node->m_p0;

				newNode->m_maxIndex = treeVCount + 1;
				vertexArrayOut[treeVCount + 1] = node->m_p1;
				treeVCount += 2;

				if (parentNode) {
					*parentNode = dgNode::dgLeafNodePtr (dgUnsigned32(index));
				}
				index ++;

				pool[stack] = node->m_front;
				parent[stack] = &newNode->m_front;
				stack ++;
				pool[stack] = node->m_back;
				parent[stack] = &newNode->m_back;
				stack ++;
			}
		}

		return index;
	}

	void PushNodes (dgList<dgConstructionTree*>& list)
	{
		if (m_back) {
			m_back->PushNodes (list);
		}
		if (m_front) {
			m_front->PushNodes (list);
		}
		if (m_boxIndex == -1) {
			list.Append(this);
		}
	}

	dgInt32 CalculateMaximunDepth () const 
	{
		dgInt32 depthPool[128];
		const dgConstructionTree* pool[128];

		depthPool[0] = 0;
		pool[0] = this;
		dgInt32 stack = 1;

		dgInt32 maxDepth = -1;
		while (stack) {
			stack --;

			dgInt32 depth = depthPool[stack];
			const dgConstructionTree* const node = pool[stack];
			maxDepth = dgMax(maxDepth, depth);

			if (node->m_boxIndex == -1) {
				dgAssert (node->m_back);
				dgAssert (node->m_front);

				depth ++;
				depthPool[stack] = depth;
				pool[stack] = node->m_back;
				stack ++;

				depthPool[stack] = depth;
				pool[stack] = node->m_front;
				stack ++;
			}
		}

		return maxDepth + 1;
	}

	dgFloat32 CalculateArea (dgConstructionTree* const node0, dgConstructionTree* const node1) const
	{
		dgVector p0 (node0->m_p0.GetMin(node1->m_p0));
		dgVector p1 (node0->m_p1.GetMax(node1->m_p1));

		dgVector size (p1 - p0);
		return size.DotProduct4(size.ShiftTripleRight()).m_x;
	}

	void SetBox ()
	{
		m_p0 = m_back->m_p0.GetMin(m_front->m_p0);
		m_p1 = m_back->m_p1.GetMax(m_front->m_p1);
		dgVector size (m_p1 - m_p0);
		m_surfaceArea = size.DotProduct4(size.ShiftTripleRight()).m_x;
	}

	void ImproveNodeFitness ()
	{
		dgConstructionTree* const node = this;

		dgAssert (node->m_back);
		dgAssert (node->m_front);

		if (node->m_parent)	{
			if (node->m_parent->m_back == node) {
				dgFloat32 cost0 = node->m_surfaceArea;
				dgFloat32 cost1 = CalculateArea (node->m_front, node->m_parent->m_front);
				dgFloat32 cost2 = CalculateArea (node->m_back, node->m_parent->m_front);
				if ((cost1 <= cost0) && (cost1 <= cost2)) {
					dgConstructionTree* const parent = node->m_parent;
					node->m_p0 = parent->m_p0;
					node->m_p1 = parent->m_p1;
					node->m_surfaceArea = parent->m_surfaceArea; 
					if (parent->m_parent) {
						if (parent->m_parent->m_back == parent) {
							parent->m_parent->m_back = node;
						} else {
							dgAssert (parent->m_parent->m_front == parent);
							parent->m_parent->m_front = node;
						}
					}
					node->m_parent = parent->m_parent;
					parent->m_parent = node;
					node->m_front->m_parent = parent;
					parent->m_back = node->m_front;
					node->m_front = parent;
					parent->SetBox ();
				} else if ((cost2 <= cost0) && (cost2 <= cost1)) {
					dgConstructionTree* const parent = node->m_parent;
					node->m_p0 = parent->m_p0;
					node->m_p1 = parent->m_p1;
					node->m_surfaceArea = parent->m_surfaceArea; 
					if (parent->m_parent) {
						if (parent->m_parent->m_back == parent) {
							parent->m_parent->m_back = node;
						} else {
							dgAssert (parent->m_parent->m_front == parent);
							parent->m_parent->m_front = node;
						}
					}
					node->m_parent = parent->m_parent;
					parent->m_parent = node;
					node->m_back->m_parent = parent;
					parent->m_back = node->m_back;
					node->m_back = parent;
					parent->SetBox ();
				}

			} else {

				dgFloat32 cost0 = node->m_surfaceArea;
				dgFloat32 cost1 = CalculateArea (node->m_back, node->m_parent->m_back);
				dgFloat32 cost2 = CalculateArea (node->m_front, node->m_parent->m_back);
				if ((cost1 <= cost0) && (cost1 <= cost2)) {
					dgConstructionTree* const parent = node->m_parent;
					node->m_p0 = parent->m_p0;
					node->m_p1 = parent->m_p1;
					node->m_surfaceArea = parent->m_surfaceArea; 
					if (parent->m_parent) {
						if (parent->m_parent->m_back == parent) {
							parent->m_parent->m_back = node;
						} else {
							dgAssert (parent->m_parent->m_front == parent);
							parent->m_parent->m_front = node;
						}
					}
					node->m_parent = parent->m_parent;
					parent->m_parent = node;
					node->m_back->m_parent = parent;
					parent->m_front = node->m_back;
					node->m_back = parent;
					parent->SetBox ();
				} else if ((cost2 <= cost0) && (cost2 <= cost1)) {
					dgConstructionTree* const parent = node->m_parent;
					node->m_p0 = parent->m_p0;
					node->m_p1 = parent->m_p1;
					node->m_surfaceArea = parent->m_surfaceArea; 
					if (parent->m_parent) {
						if (parent->m_parent->m_back == parent) {
							parent->m_parent->m_back = node;
						} else {
							dgAssert (parent->m_parent->m_front == parent);
							parent->m_parent->m_front = node;
						}
					}
					node->m_parent = parent->m_parent;
					parent->m_parent = node;
					node->m_front->m_parent = parent;
					parent->m_front = node->m_front;
					node->m_front = parent;
					parent->SetBox ();
				}
			}
		}
	}

	dgFloat64 TotalFitness (dgList<dgConstructionTree*>& nodeList) const
	{
		dgFloat64 cost = dgFloat32 (0.0f);
		for (dgList<dgConstructionTree*>::dgListNode* node = nodeList.GetFirst(); node; node = node->GetNext()) {
			dgConstructionTree* const box = node->GetInfo();
			cost += box->m_surfaceArea;
		}
		return cost;
	}

	dgAABBPolygonSoup::dgConstructionTree* ImproveTotalFitness (dgNode* const boxArray, dgMemoryAllocator* const allocator)
	{
		dgList<dgConstructionTree*> nodesList(allocator);

		dgConstructionTree* newRoot = this;
		PushNodes (nodesList);
		if (nodesList.GetCount()) {
			dgInt32 maxPasses = CalculateMaximunDepth () * 2;
			dgFloat64 newCost = TotalFitness (nodesList);
			dgFloat64 prevCost = newCost;
			do {
				prevCost = newCost;
				for (dgList<dgConstructionTree*>::dgListNode* node = nodesList.GetFirst(); node; node = node->GetNext()) {
					dgConstructionTree* const box = node->GetInfo();
					box->ImproveNodeFitness ();
				}

				newCost = TotalFitness (nodesList);
				maxPasses --;
			} while (maxPasses && (newCost < (prevCost * dgFloat32 (0.999f))));

			newRoot = nodesList.GetLast()->GetInfo();
			while (newRoot->m_parent) {
				newRoot = newRoot->m_parent;
			}
		}

		return newRoot;
	}

	dgVector m_p0;
	dgVector m_p1;
	dgInt32 m_boxIndex;
	dgFloat32 m_surfaceArea;
	dgConstructionTree* m_back;
	dgConstructionTree* m_front;
	dgConstructionTree* m_parent;
};





dgAABBPolygonSoup::dgAABBPolygonSoup ()
	:dgPolygonSoupDatabase()
{
	m_aabb = NULL;
	m_indices = NULL;
	m_indexCount = 0;
	m_nodesCount = 0;
}

dgAABBPolygonSoup::~dgAABBPolygonSoup ()
{
	if (m_aabb) {
		dgFreeStack (m_aabb);
		dgFreeStack (m_indices);
	}
}


void* dgAABBPolygonSoup::GetRootNode() const
{
	return m_aabb;
}

void* dgAABBPolygonSoup::GetBackNode(const void* const root) const
{
	dgNode* const node = (dgNode*) root;
	return node->m_back.IsLeaf() ? NULL : node->m_back.GetNode(m_aabb);
}

void* dgAABBPolygonSoup::GetFrontNode(const void* const root) const
{
	dgNode* const node = (dgNode*) root;
	return node->m_front.IsLeaf() ? NULL : node->m_front.GetNode(m_aabb);
}


void dgAABBPolygonSoup::GetNodeAABB(const void* const root, dgVector& p0, dgVector& p1) const
{
	dgNode* const node = (dgNode*) root;
	const dgTriplex* const vertex = (dgTriplex*) m_localVertex;

	p0 = dgVector (vertex[node->m_minIndex].m_x, vertex[node->m_minIndex].m_y, vertex[node->m_minIndex].m_z, dgFloat32 (0.0f));
	p1 = dgVector (vertex[node->m_maxIndex].m_x, vertex[node->m_maxIndex].m_y, vertex[node->m_maxIndex].m_z, dgFloat32 (0.0f));
}



void dgAABBPolygonSoup::GetAABB (dgVector& p0, dgVector& p1) const
{
	if (m_aabb) { 
		dgNode* const tree = (dgNode*) m_aabb;
		const dgTriplex* const localVertex = (dgTriplex*) m_localVertex;
		p0 = dgVector (&localVertex[tree->m_minIndex].m_x);
		p1 = dgVector (&localVertex[tree->m_maxIndex].m_x);
	} else {
		p0 = dgVector (dgFloat32 (0.0f));
		p1 = dgVector (dgFloat32 (0.0f));
	}
}




void dgAABBPolygonSoup::Serialize (dgSerialize callback, void* const userData) const
{
	dgNode* const tree = (dgNode*) m_aabb;
	callback (userData, &m_vertexCount, sizeof (dgInt32));
	callback (userData, &m_indexCount, sizeof (dgInt32));
	callback (userData, &m_nodesCount, sizeof (dgInt32));
	callback (userData, &m_nodesCount, sizeof (dgInt32));
	if (tree) {
		callback (userData,  m_localVertex, sizeof (dgTriplex) * m_vertexCount);
		callback (userData,  m_indices, sizeof (dgInt32) * m_indexCount);
		callback (userData, tree, sizeof (dgNode) * m_nodesCount);
	}
}


void dgAABBPolygonSoup::Deserialize (dgDeserialize callback, void* const userData)
{
//	dgInt32 nodes;
	dgNode* tree;;

	m_strideInBytes = sizeof (dgTriplex);
	callback (userData, &m_vertexCount, sizeof (dgInt32));
	callback (userData, &m_indexCount, sizeof (dgInt32));
	callback (userData, &m_nodesCount, sizeof (dgInt32));
	callback (userData, &m_nodesCount, sizeof (dgInt32));

	if (m_vertexCount) {
		m_localVertex = (dgFloat32*) dgMallocStack (sizeof (dgTriplex) * m_vertexCount);
		m_indices = (dgInt32*) dgMallocStack (sizeof (dgInt32) * m_indexCount);
		tree = (dgNode*) dgMallocStack (sizeof (dgNode) * m_nodesCount);

		callback (userData, m_localVertex, sizeof (dgTriplex) * m_vertexCount);
		callback (userData, m_indices, sizeof (dgInt32) * m_indexCount);
		callback (userData, tree, sizeof (dgNode) * m_nodesCount);
	} else {
		m_localVertex = NULL;
		m_indices = NULL;
		tree = NULL;
	}
	m_aabb = tree;
}


dgIntersectStatus dgAABBPolygonSoup::CalculateManifoldFaceEdgeNormals (void* const context, const dgFloat32* const polygon, dgInt32 strideInBytes, const dgInt32* const indexArray, dgInt32 indexCount)
{
	AdjacentdFaces& adjacentFaces = *((AdjacentdFaces*)context);

	dgInt32 count = adjacentFaces.m_count;
	dgInt32 stride = dgInt32 (strideInBytes / sizeof (dgFloat32));

	dgInt32 j0 = indexArray[indexCount - 1];
	for (dgInt32 j = 0; j < indexCount; j ++) {
		dgInt32 j1 = indexArray[j];
		dgInt64 key = (dgInt64 (j0) << 32) + j1;
		for (dgInt32 i = 0; i < count; i ++) {
			if (adjacentFaces.m_edgeMap[i] == key) {
				dgFloat32 maxDist = dgFloat32 (0.0f);
				for (dgInt32 k = 0; k < indexCount; k ++) {
					dgVector r (&polygon[indexArray[k] * stride]);
					dgFloat32 dist = adjacentFaces.m_normal.Evalue(r);
					if (dgAbsf (dist) > dgAbsf (maxDist)) {
						maxDist = dist;
					}
				}
				if (maxDist < -dgFloat32 (1.0e-3f)) {
					adjacentFaces.m_index[i + count + 2] = indexArray[indexCount + 1];
				} else {
					adjacentFaces.m_index[i + count + 2] = adjacentFaces.m_index[count + 1];
				}
				break;
			}
		}

		j0 = j1;
	}

	return t_ContinueSearh;
}


dgIntersectStatus dgAABBPolygonSoup::CalculateDisjointedFaceEdgeNormals (void* const context, const dgFloat32* const polygon, dgInt32 strideInBytes, const dgInt32* const indexArray, dgInt32 indexCount, dgFloat32 hitDistance)
{
	#define DG_WELDING_TOL (1.0e-2f)
	#define DG_WELDING_TOL2 (DG_WELDING_TOL * DG_WELDING_TOL)

	AdjacentdFaces& adjacentFaces = *((AdjacentdFaces*)context);

	if (adjacentFaces.m_index != indexArray) {	
		dgInt32 adjacentCount = adjacentFaces.m_count;
		dgInt32 stride = dgInt32 (strideInBytes / sizeof (dgFloat32));

		dgInt32 j0 = adjacentCount - 1;
		dgInt32 indexJ0 = adjacentFaces.m_index[adjacentCount - 1];
		for (dgInt32 j = 0; j < adjacentCount; j ++) {
			dgInt32 indexJ1 = adjacentFaces.m_index[j];
			dgBigVector q0 (&polygon[indexJ1 * stride]);
			dgBigVector q1 (&polygon[indexJ0 * stride]);
			dgBigVector q1q0 (q1 - q0);
			dgFloat64 q1q0Mag2 = q1q0 % q1q0;

			dgInt32 indexI0 = indexArray[indexCount - 1];
			for (dgInt32 i = 0; i < indexCount; i ++) {
				dgInt32 indexI1 = indexArray[i];
				dgBigVector p0 (&polygon[indexI0 * stride]);
				dgBigVector p1 (&polygon[indexI1 * stride]);
				dgBigVector p1p0 (p1 - p0);
				dgFloat64 dot = p1p0 % q1q0;
				if (dot > 0.0f) {
					dgFloat64 q1p0Mag2 = p1p0 % p1p0;
					if ((dot * dot) >= (q1p0Mag2 * q1q0Mag2 * dgFloat64(0.99995f))) {
						dgFloat64 x0 = q0 % q1q0;
						dgFloat64 x1 = q1 % q1q0;
						dgFloat64 y0 = p0 % q1q0;
						dgFloat64 y1 = p1 % q1q0;
						dgAssert (x1 > x0);
						dgAssert (y1 > y0);
						if (!((y0 >= x1) || (y1 <= x0))) {
							dgFloat64 t = ((p0 - q0) % q1q0) / q1q0Mag2;
							dgBigVector q (q0 + q1q0.Scale3(t));
							dgBigVector dist (p0 - q);
							dgFloat64 err2 = dist % dist;
							if (err2 < DG_WELDING_TOL2) {
								dgFloat32 maxDist = dgFloat32 (0.0f);
								for (dgInt32 k = 0; k < indexCount; k ++) {
									dgVector r (&polygon[indexArray[k] * stride]);
									dgFloat32 dist = adjacentFaces.m_normal.Evalue(r);
									if (dgAbsf (dist) > dgAbsf (maxDist)) {
										maxDist = dist;
									}
								}

								if (adjacentFaces.m_index[j0 + adjacentCount + 2] == -1) {
									if (maxDist < -dgFloat32 (1.0e-3f)) {
										adjacentFaces.m_index[j0 + adjacentCount + 2] = indexArray[indexCount + 1];
									} else {
										adjacentFaces.m_index[j0 + adjacentCount + 2] = adjacentFaces.m_index[adjacentCount + 1];
									}
								} else {
									if (maxDist < -dgFloat32 (1.0e-3f)) {
										dgBigVector n0 (adjacentFaces.m_normal[0], adjacentFaces.m_normal[1], adjacentFaces.m_normal[2], dgFloat64(0.0f));
										dgBigVector n1 (&polygon[adjacentFaces.m_index[j0 + adjacentCount + 2] * stride]);
										dgBigVector n2 (&polygon[indexArray[indexCount + 1] * stride]);

										dgBigVector tilt0 (n0 * n1); 
										dgBigVector tilt1 (n0 * n2); 
										dgFloat64 dist0 (q1q0 % tilt0);
										dgFloat64 dist1 (q1q0 % tilt1);
										if (dist0 < dist1) {
											adjacentFaces.m_index[j0 + adjacentCount + 2] = indexArray[indexCount + 1];
										}
									} else {
										adjacentFaces.m_index[j0 + adjacentCount + 2] = adjacentFaces.m_index[adjacentCount + 1];
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



dgIntersectStatus dgAABBPolygonSoup::CalculateAllFaceEdgeNormals (void* const context, const dgFloat32* const polygon, dgInt32 strideInBytes, const dgInt32* const indexArray, dgInt32 indexCount, dgFloat32 hitDistance)
{
	dgAABBPolygonSoup* const me = (dgAABBPolygonSoup*) context;

	dgInt32 stride = dgInt32 (strideInBytes / sizeof (dgFloat32));

	AdjacentdFaces adjacentFaces;
	adjacentFaces.m_count = indexCount;
	adjacentFaces.m_index = (dgInt32*) indexArray;


	dgVector n (&polygon[indexArray[indexCount + 1] * stride]);
	dgVector p (&polygon[indexArray[0] * stride]);
	adjacentFaces.m_normal = dgPlane (n, - (n % p));

	dgAssert (indexCount < dgInt32 (sizeof (adjacentFaces.m_edgeMap) / sizeof (adjacentFaces.m_edgeMap[0])));

	dgInt32 edgeIndex = indexCount - 1;
	dgInt32 i0 = indexArray[indexCount - 1];
	dgVector p0 ( dgFloat32 (1.0e15f)); 
	dgVector p1 (-dgFloat32 (1.0e15f)); 
	for (dgInt32 i = 0; i < indexCount; i ++) {
		dgInt32 i1 = indexArray[i];
		dgInt32 index = i1 * stride;
		dgVector p (&polygon[index]);

		p0 = p0.GetMin (p); 
		p1 = p1.GetMax (p); 

		adjacentFaces.m_edgeMap[edgeIndex] = (dgInt64 (i1) << 32) + i0;
		edgeIndex = i;
		i0 = i1;
	}

	p0 = (p0 + dgVector (dgFloat32 (0.25f)) & dgVector::m_triplexMask);
	p1 = (p1 + dgVector (dgFloat32 (0.25f)) & dgVector::m_triplexMask);

	//me->ForAllSectors (p0, p1, dgVector (dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f)), CalculateManifoldFaceEdgeNormals, &adjacentFaces);
	me->ForAllSectors (p0, p1, dgVector (dgFloat32 (0.0f)), dgFloat32 (1.0f), CalculateDisjointedFaceEdgeNormals, &adjacentFaces);

	return t_ContinueSearh;
}

dgFloat32 dgAABBPolygonSoup::CalculateFaceMaxSize (const dgVector* const vertex, dgInt32 indexCount, const dgInt32* const indexArray) const
{
	dgFloat32 maxSize = dgFloat32 (0.0f);
	dgInt32 index = indexArray[indexCount - 1];
	dgVector p0 (vertex[index]);
	for (dgInt32 i = 0; i < indexCount; i ++) {
		dgInt32 index = indexArray[i];
		dgVector p1 (vertex[index]);

		dgVector dir (p1 - p0);
		dir = dir.Scale4(dgRsqrt (dir % dir));

		dgFloat32 maxVal = dgFloat32 (-1.0e10f);
		dgFloat32 minVal = dgFloat32 ( 1.0e10f);
		for (dgInt32 j = 0; j < indexCount; j ++) {
			dgInt32 index = indexArray[j];
			const dgVector& q = vertex[index];
			dgFloat32 val = dir.DotProduct4(q).m_x;
			minVal = dgMin(minVal, val);
			maxVal = dgMax(maxVal, val);
		}

		dgFloat32 size = maxVal - minVal;
		maxSize = dgMax(maxSize, size);
		p0 = p1;
	}

	return dgFloor (maxSize + dgFloat32 (1.0f));
}

void dgAABBPolygonSoup::CalculateAdjacendy ()
{
	dgVector p0;
	dgVector p1;
	GetAABB (p0, p1);
	ForAllSectors (p0, p1, dgVector (dgFloat32 (0.0f)), dgFloat32 (1.0f), CalculateAllFaceEdgeNormals, this);

	dgNode* const tree = (dgNode*) m_aabb;
	for (dgInt32 i = 0; i < m_nodesCount; i ++) {
		dgNode* const node = &tree[i];
		if (node->m_back.IsLeaf()) {
			dgInt32 vCount = node->m_back.GetCount();
			dgInt32 index = dgInt32 (node->m_back.GetIndex());
			dgInt32* const face = &m_indices[index];
			for (dgInt32 j = 0; j < vCount; j ++) {
				if (face[vCount + 2 + j] == -1) {
					face[vCount + 2 + j] = face[vCount + 1];
				}
			}
		}

		if (node->m_front.IsLeaf()) {
			dgInt32 vCount = node->m_front.GetCount();
			dgInt32 index = dgInt32 (node->m_front.GetIndex());
			dgInt32* const face = &m_indices[index];
			for (dgInt32 j = 0; j < vCount; j ++) {
				if (face[vCount + 2 + j] == -1) {
					face[vCount + 2 + j] = face[vCount + 1];
				}
			}
		}
	}
}

dgInt32 dgAABBPolygonSoup::BuildTopDown (dgMemoryAllocator* const allocator, dgInt32 boxCount, dgNode* const boxArray, dgVector* const vertexArrayOut, dgInt32 &treeVCount, bool optimizedBuild)
{
	dgStack <dgNode> boxCopy (boxCount);
	memcpy (&boxCopy[0], boxArray, boxCount * sizeof (dgNode));

	dgConstructionTree* root = new (allocator) dgConstructionTree (allocator, 0, boxCount - 1, &boxCopy[0], vertexArrayOut, NULL);
	
	optimizedBuild = true;
	if (optimizedBuild) {
		root = root->ImproveTotalFitness (&boxCopy[0], allocator);
	}

	dgInt32 count = root->BuildTree (boxArray, &boxCopy[0], vertexArrayOut, treeVCount);

	delete root;
	return count;
}


void dgAABBPolygonSoup::Create (const dgPolygonSoupDatabaseBuilder& builder, bool optimizedBuild)
{
	if (builder.m_faceCount == 0) {
		return;
	}

	m_strideInBytes = sizeof (dgTriplex);
	m_indexCount = builder.m_indexCount * 2 + builder.m_faceCount;
	m_indices = (dgInt32*) dgMallocStack (sizeof (dgInt32) * m_indexCount);
	m_aabb = (dgNode*) dgMallocStack (sizeof (dgNode) * builder.m_faceCount);

	dgStack<dgVector> tmpVertexArray(builder.m_vertexCount + builder.m_normalCount + builder.m_faceCount * 4);

	for (dgInt32 i = 0; i < builder.m_vertexCount; i ++) {
		tmpVertexArray[i] = builder.m_vertexPoints[i];
	}

	for (dgInt32 i = 0; i < builder.m_normalCount; i ++) {
		tmpVertexArray[i + builder.m_vertexCount] = builder.m_normalPoints[i]; 
	}

	dgInt32 polygonIndex = 0;
	dgInt32* indexMap = m_indices;
	const dgInt32* const indices = &builder.m_vertexIndex[0];

	// index format i0, i1, i2, ... , id, normal, e0Normal, e1Normal, e2Normal, ..., faceSize
	for (dgInt32 i = 0; i < builder.m_faceCount; i ++) {

		dgInt32 indexCount = builder.m_faceVertexCount[i] - 1;
		dgNode& box = m_aabb[i];

		box.m_minIndex = builder.m_normalCount + builder.m_vertexCount + i * 2;
		box.m_maxIndex = builder.m_normalCount + builder.m_vertexCount + i * 2 + 1;

		box.m_front = dgNode::dgLeafNodePtr(0, 0);
		box.m_back = dgNode::dgLeafNodePtr (indexCount, indexMap - m_indices);
		box.CalcExtends (&tmpVertexArray[0], indexCount, &indices[polygonIndex]);

		dgInt32 normalIndex = builder.m_vertexCount + builder.m_normalIndex[i];;
		for (dgInt32 j = 0; j < indexCount; j ++) {
			// face vertex
			indexMap[j] = indices[polygonIndex + j];
			// face edge normal 
			indexMap[j + indexCount + 2] = -1;
		}
		// face attribute
		indexMap[indexCount] = indices[polygonIndex + indexCount];
		// face normal
		indexMap[indexCount + 1] = normalIndex;
		// face size
		indexMap[indexCount * 2 + 2] = dgInt32 (CalculateFaceMaxSize (&tmpVertexArray[0], indexCount, &indices[polygonIndex]));

		indexMap += indexCount * 2 + 3;
		polygonIndex += indexCount + 1;
	}

	dgInt32 baseVertexCount = builder.m_normalCount + builder.m_vertexCount + builder.m_faceCount * 2;
	dgInt32 extraVertexCount = baseVertexCount;
	m_nodesCount = BuildTopDown (builder.m_allocator, builder.m_faceCount, m_aabb, &tmpVertexArray[0], extraVertexCount, optimizedBuild);

	tmpVertexArray[m_aabb->m_minIndex] -= dgVector (dgFloat32 (0.1f));
	tmpVertexArray[m_aabb->m_maxIndex] += dgVector (dgFloat32 (0.1f));

	dgInt32 aabbVeterxCount = extraVertexCount - baseVertexCount;
	dgStack<dgInt32> indexArray (aabbVeterxCount);
	aabbVeterxCount = dgVertexListToIndexList (&tmpVertexArray[baseVertexCount].m_x, sizeof (dgVector), sizeof (dgTriplex), 0, aabbVeterxCount, &indexArray[0], dgFloat32 (1.0e-6f));

	m_vertexCount = builder.m_normalCount + builder.m_vertexCount + aabbVeterxCount;
	m_localVertex = (dgFloat32*) dgMallocStack (sizeof (dgTriplex) * m_vertexCount);
	dgTriplex* const dstPoints = (dgTriplex*)m_localVertex;
	for (dgInt32 i = 0; i < (builder.m_normalCount + builder.m_vertexCount); i ++) {
		dstPoints[i].m_x = tmpVertexArray[i].m_x;
		dstPoints[i].m_y = tmpVertexArray[i].m_y;
		dstPoints[i].m_z = tmpVertexArray[i].m_z;
	}

	for (dgInt32 i = 0; i < aabbVeterxCount; i ++) {
		dstPoints[builder.m_normalCount + builder.m_vertexCount + i].m_x = tmpVertexArray[baseVertexCount + i].m_x;
		dstPoints[builder.m_normalCount + builder.m_vertexCount + i].m_y = tmpVertexArray[baseVertexCount + i].m_y;
		dstPoints[builder.m_normalCount + builder.m_vertexCount + i].m_z = tmpVertexArray[baseVertexCount + i].m_z;
	}

	for (dgInt32 i = 0; i < m_nodesCount; i ++) {
		dgNode& box = m_aabb[i];

		dgInt32 j = box.m_minIndex - baseVertexCount;
		box.m_minIndex = indexArray[j] + builder.m_normalCount + builder.m_vertexCount;

		j = box.m_maxIndex - baseVertexCount;
		box.m_maxIndex = indexArray[j] + builder.m_normalCount + builder.m_vertexCount;
	}
	CalculateAdjacendy();
}


dgVector dgAABBPolygonSoup::ForAllSectorsSupportVectex (const dgVector& dir) const
{
	if (!m_aabb) {
		return dgVector (dgFloat32 (0.0f));
	}

	dgFloat32 aabbProjection[DG_STACK_DEPTH];
	const dgNode *stackPool[DG_STACK_DEPTH];

	dgInt32 stack = 1;
	stackPool[0] = m_aabb;
	aabbProjection[0] = dgFloat32 (1.0e10f);
	const dgTriplex* const boxArray = (dgTriplex*)m_localVertex;
	dgVector supportVertex (dgFloat32 (0.0f));

	dgFloat32 maxProj = dgFloat32 (-1.0e20f); 
	dgInt32 ix = (dir[0] > dgFloat32 (0.0f)) ? 1 : 0;
	dgInt32 iy = (dir[1] > dgFloat32 (0.0f)) ? 1 : 0;
	dgInt32 iz = (dir[2] > dgFloat32 (0.0f)) ? 1 : 0;

	while (stack) {
		dgFloat32 boxSupportValue;

		stack--;
		boxSupportValue = aabbProjection[stack];
		if (boxSupportValue > maxProj) {
			dgFloat32 backSupportDist = dgFloat32 (0.0f);
			dgFloat32 frontSupportDist = dgFloat32 (0.0f);
			const dgNode* const me = stackPool[stack];
			if (me->m_back.IsLeaf()) {
				backSupportDist = dgFloat32 (-1.0e20f);
				dgInt32 index = dgInt32 (me->m_back.GetIndex());
				dgInt32 vCount = me->m_back.GetCount();
				dgVector vertex (dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f));
				for (dgInt32 j = 0; j < vCount; j ++) {
					dgInt32 i0 = m_indices[index + j] * dgInt32 (sizeof (dgTriplex) / sizeof (dgFloat32));
					dgVector p (&boxArray[i0].m_x);
					dgFloat32 dist = p % dir;
					if (dist > backSupportDist) {
						backSupportDist = dist;
						vertex = p;
					}
				}

				if (backSupportDist > maxProj) {
					maxProj = backSupportDist;
					supportVertex = vertex; 
				}

			} else {
				dgVector box[2];
				const dgNode* const node = me->m_back.GetNode(this);
				box[0].m_x = boxArray[node->m_minIndex].m_x;
				box[0].m_y = boxArray[node->m_minIndex].m_y;
				box[0].m_z = boxArray[node->m_minIndex].m_z;
				box[1].m_x = boxArray[node->m_maxIndex].m_x;
				box[1].m_y = boxArray[node->m_maxIndex].m_y;
				box[1].m_z = boxArray[node->m_maxIndex].m_z;

				dgVector supportPoint (box[ix].m_x, box[iy].m_y, box[iz].m_z, dgFloat32 (0.0));
				backSupportDist = supportPoint % dir;
			}

			if (me->m_front.IsLeaf()) {
				frontSupportDist = dgFloat32 (-1.0e20f);
				dgInt32 index = dgInt32 (me->m_front.GetIndex());
				dgInt32 vCount = me->m_front.GetCount();
				dgVector vertex (dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f));
				for (dgInt32 j = 0; j < vCount; j ++) {
					dgInt32 i0 = m_indices[index + j] * dgInt32 (sizeof (dgTriplex) / sizeof (dgFloat32));
					dgVector p (&boxArray[i0].m_x);
					dgFloat32 dist = p % dir;
					if (dist > frontSupportDist) {
						frontSupportDist = dist;
						vertex = p;
					}
				}
				if (frontSupportDist > maxProj) {
					maxProj = frontSupportDist;
					supportVertex = vertex; 
				}

			} else {
				dgVector box[2];
				const dgNode* const node = me->m_front.GetNode(this);
				box[0].m_x = boxArray[node->m_minIndex].m_x;
				box[0].m_y = boxArray[node->m_minIndex].m_y;
				box[0].m_z = boxArray[node->m_minIndex].m_z;
				box[1].m_x = boxArray[node->m_maxIndex].m_x;
				box[1].m_y = boxArray[node->m_maxIndex].m_y;
				box[1].m_z = boxArray[node->m_maxIndex].m_z;

				dgVector supportPoint (box[ix].m_x, box[iy].m_y, box[iz].m_z, dgFloat32 (0.0));
				frontSupportDist = supportPoint % dir;
			}

			if (frontSupportDist >= backSupportDist) {
				if (!me->m_back.IsLeaf()) {
					aabbProjection[stack] = backSupportDist;
					stackPool[stack] = me->m_back.GetNode(m_aabb);
					stack++;
				}

				if (!me->m_front.IsLeaf()) {
					aabbProjection[stack] = frontSupportDist;
					stackPool[stack] = me->m_front.GetNode(m_aabb);
					stack++;
				}

			} else {

				if (!me->m_front.IsLeaf()) {
					aabbProjection[stack] = frontSupportDist;
					stackPool[stack] = me->m_front.GetNode(m_aabb);
					stack++;
				}

				if (!me->m_back.IsLeaf()) {
					aabbProjection[stack] = backSupportDist;
					stackPool[stack] = me->m_back.GetNode(m_aabb);
					stack++;
				}
			}
		}
	}

	return supportVertex;
}


void dgAABBPolygonSoup::ForAllSectors (const dgVector& minBox, const dgVector& maxBox, const dgVector& boxDistanceTravel, dgFloat32 m_maxT, dgAABBIntersectCallback callback, void* const context) const
{
	if (!m_aabb) {
		return;
	}
	dgFloat32 distance[DG_STACK_DEPTH];
	const dgNode* stackPool[DG_STACK_DEPTH];

	const dgInt32 stride = sizeof (dgTriplex) / sizeof (dgFloat32);
	dgFastAABBInfo aabbIndo (minBox, maxBox);
	const dgTriplex* const vertexArray = (dgTriplex*) m_localVertex;

	if ((boxDistanceTravel % boxDistanceTravel) < dgFloat32 (1.0e-8f)) {
		dgInt32 stack = 1;
		stackPool[0] = m_aabb;
		distance[0] = m_aabb->BoxPenetration(vertexArray, minBox, maxBox);

		while (stack) {
			stack --;
			dgFloat32 dist = distance[stack];
			if (dist > dgFloat32 (0.0f)) {
				const dgNode* const me = stackPool[stack];
				if (me->m_back.IsLeaf()) {
					dgInt32 index = dgInt32 (me->m_back.GetIndex());
					dgInt32 vCount = me->m_back.GetCount();
					if (vCount > 0) {
						const dgInt32* const indices = &m_indices[index];
						dgInt32 normalIndex = indices[vCount + 1];
						dgVector faceNormal (&vertexArray[normalIndex].m_x);
						dgFloat32 dist1 = aabbIndo.PolygonBoxDistance (faceNormal, vCount, indices, stride, &vertexArray[0].m_x);
						if (dist1 > dgFloat32 (0.0f)) {
							if (callback(context, &vertexArray[0].m_x, sizeof (dgTriplex), indices, vCount, dist1) == t_StopSearh) {
								return;
							}
						}
					}

				} else {
					const dgNode* const node = me->m_back.GetNode(m_aabb);
					dgFloat32 dist = node->BoxPenetration(vertexArray, minBox, maxBox);
					if (dist > dgFloat32 (0.0f)) {
						dgInt32 j = stack;
						for ( ; j && (dist > distance[j - 1]); j --) {
							stackPool[j] = stackPool[j - 1];
							distance[j] = distance[j - 1];
						}
						dgAssert (stack < DG_STACK_DEPTH);
						stackPool[j] = node;
						distance[j] = dist;
						stack++;
					}
				}

				if (me->m_front.IsLeaf()) {
					dgInt32 index = dgInt32 (me->m_front.GetIndex());
					dgInt32 vCount = me->m_front.GetCount();
					if (vCount > 0) {
						const dgInt32* const indices = &m_indices[index];
						dgInt32 normalIndex = indices[vCount + 1];
						dgVector faceNormal (&vertexArray[normalIndex].m_x);
						dgFloat32 dist1 = aabbIndo.PolygonBoxDistance (faceNormal, vCount, indices, stride, &vertexArray[0].m_x);
						if (dist1 > dgFloat32 (0.0f)) {
							if (callback(context, &vertexArray[0].m_x, sizeof (dgTriplex), indices, vCount, dist1) == t_StopSearh) {
								return;
							}
						}
					}

				} else {
					const dgNode* const node = me->m_front.GetNode(m_aabb);
					dgFloat32 dist = node->BoxPenetration(vertexArray, minBox, maxBox);
					if (dist > dgFloat32 (0.0f)) {
						dgInt32 j = stack;
						for ( ; j && (dist > distance[j - 1]); j --) {
							stackPool[j] = stackPool[j - 1];
							distance[j] = distance[j - 1];
						}
						dgAssert (stack < DG_STACK_DEPTH);
						stackPool[j] = node;
						distance[j] = dist;
						stack++;
					}
				}
			}
		}

	} else {
		dgFastRayTest ray (dgVector (dgFloat32 (0.0f)), boxDistanceTravel);
		dgInt32 stack = 1;
		stackPool[0] = m_aabb;
		distance [0] = m_aabb->BoxIntersect (ray, vertexArray, minBox, maxBox);

		while (stack) {
			stack --;
			const dgFloat32 dist = distance[stack];
			const dgNode* const me = stackPool[stack];
			if (dist < dgFloat32 (1.0f)) {

				if (me->m_back.IsLeaf()) {
					dgInt32 index = dgInt32 (me->m_back.GetIndex());
					dgInt32 vCount = me->m_back.GetCount();
					if (vCount > 0) {
						const dgInt32* const indices = &m_indices[index];
						dgInt32 normalIndex = indices[vCount + 1];
						dgVector faceNormal (&vertexArray[normalIndex].m_x);
						dgFloat32 hitDistance = aabbIndo.PolygonBoxRayDistance (faceNormal, vCount, indices, stride, &vertexArray[0].m_x, ray);
						if (hitDistance < dgFloat32 (1.0f)) {
							if (callback(context, &vertexArray[0].m_x, sizeof (dgTriplex), indices, vCount, hitDistance) == t_StopSearh) {
								return;
							}
						}
					}

				} else {
					const dgNode* const node = me->m_back.GetNode(m_aabb);
					dgFloat32 dist = node->BoxIntersect (ray, vertexArray, minBox, maxBox);
					if (dist < dgFloat32 (1.0f)) {
						dgInt32 j = stack;
						for ( ; j && (dist > distance[j - 1]); j --) {
							stackPool[j] = stackPool[j - 1];
							distance[j] = distance[j - 1];
						}
						dgAssert (stack < DG_STACK_DEPTH);
						stackPool[j] = node;
						distance[j] = dist;
						stack++;
					}
				}

				if (me->m_front.IsLeaf()) {
					dgInt32 index = dgInt32 (me->m_front.GetIndex());
					dgInt32 vCount = me->m_front.GetCount();
					if (vCount > 0) {
						const dgInt32* const indices = &m_indices[index];
						dgInt32 normalIndex = indices[vCount + 1];
						dgVector faceNormal (&vertexArray[normalIndex].m_x);
						dgFloat32 hitDistance = aabbIndo.PolygonBoxRayDistance (faceNormal, vCount, indices, stride, &vertexArray[0].m_x, ray);
						if (hitDistance < dgFloat32 (1.0f)) {
							if (callback(context, &vertexArray[0].m_x, sizeof (dgTriplex), indices, vCount, hitDistance) == t_StopSearh) {
								return;
							}
						}
					}

				} else {
					const dgNode* const node = me->m_front.GetNode(m_aabb);
					dgFloat32 dist = node->BoxIntersect (ray, vertexArray, minBox, maxBox);
					if (dist < dgFloat32 (1.0f)) {
						dgInt32 j = stack;
						for ( ; j && (dist > distance[j - 1]); j --) {
							stackPool[j] = stackPool[j - 1];
							distance[j] = distance[j - 1];
						}
						dgAssert (stack < DG_STACK_DEPTH);
						stackPool[j] = node;
						distance[j] = dist;
						stack ++;
					}
				}
			}
		}
	}
}




void dgAABBPolygonSoup::ForAllSectorsRayHit (const dgFastRayTest& raySrc, dgRayIntersectCallback callback, void* const context) const
{
	const dgNode *stackPool[DG_STACK_DEPTH];
	dgFloat32 distance[DG_STACK_DEPTH];
	dgFastRayTest ray (raySrc);

	dgInt32 stack = 1;
	dgFloat32 maxParam = dgFloat32 (1.0f);
	const dgTriplex* const vertexArray = (dgTriplex*) m_localVertex;

	stackPool[0] = m_aabb;
	distance[0] = m_aabb->RayDistance(ray, vertexArray);
	while (stack) {
		stack --;
		dgFloat32 dist = distance[stack];
		if (dist > maxParam) {
			break;
		} else {
			const dgNode *const me = stackPool[stack];
			if (me->m_back.IsLeaf()) {
				dgInt32 vCount = me->m_back.GetCount();
				if (vCount > 0) {
					dgInt32 index = dgInt32 (me->m_back.GetIndex());
					dgFloat32 param = callback(context, &vertexArray[0].m_x, sizeof (dgTriplex), &m_indices[index], vCount);
					dgAssert (param >= dgFloat32 (0.0f));
					if (param < maxParam) {
						maxParam = param;
						if (maxParam == dgFloat32 (0.0f)) {
							break;
						}
						ray.Reset (maxParam) ;
					}
				}

			} else {
				const dgNode* const node = me->m_back.GetNode(m_aabb);
				dgFloat32 dist = node->RayDistance(ray, vertexArray);
				if (dist < maxParam) {
					dgInt32 j = stack;
					for ( ; j && (dist > distance[j - 1]); j --) {
						stackPool[j] = stackPool[j - 1];
						distance[j] = distance[j - 1];
					}
					dgAssert (stack < DG_STACK_DEPTH);
					stackPool[j] = node;
					distance[j] = dist;
					stack++;
				}
			}

			if (me->m_front.IsLeaf()) {
				dgInt32 vCount = me->m_front.GetCount();
				if (vCount > 0) {
					dgInt32 index = dgInt32 (me->m_front.GetIndex());
					dgFloat32 param = callback(context, &vertexArray[0].m_x, sizeof (dgTriplex), &m_indices[index], vCount);
					dgAssert (param >= dgFloat32 (0.0f));
					if (param < maxParam) {
						maxParam = param;
						if (maxParam == dgFloat32 (0.0f)) {
							break;
						}
						ray.Reset (maxParam);
					}
				}

			} else {
				const dgNode* const node = me->m_front.GetNode(m_aabb);
				dgFloat32 dist = node->RayDistance(ray, vertexArray);
				if (dist < maxParam) {
					dgInt32 j = stack;
					for ( ; j && (dist > distance[j - 1]); j --) {
						stackPool[j] = stackPool[j - 1];
						distance[j] = distance[j - 1];
					}
					dgAssert (stack < DG_STACK_DEPTH);
					stackPool[j] = node;
					distance[j] = dist;
					stack++;
				}
			}
		}
	}
}
