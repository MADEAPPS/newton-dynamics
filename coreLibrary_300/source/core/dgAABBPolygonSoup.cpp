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

#ifndef DG_NEW_AABB_TREE

class dgAABBTree
{
	class TreeNode
	{
		#define DG_INDEX_COUNT_BITS 6

		public:
		inline TreeNode ()
		{
			dgAssert (0);
		}

		inline TreeNode (dgUnsigned32 node)
		{
			m_node = node;
			dgAssert (!IsLeaf());
		}

		inline dgUnsigned32 IsLeaf () const 
		{
			return m_node & 0x80000000;
		}

		inline dgUnsigned32 GetCount() const 
		{
			dgAssert (IsLeaf());
			return (m_node & (~0x80000000)) >> (32 - DG_INDEX_COUNT_BITS - 1);
		}

		inline dgUnsigned32 GetIndex() const 
		{
			dgAssert (IsLeaf());
			return m_node & (~(-(1 << (32 - DG_INDEX_COUNT_BITS - 1))));
		}


		inline TreeNode (dgUnsigned32 faceIndexCount, dgUnsigned32 faceIndexStart)
		{
			dgAssert (faceIndexCount < (1<<DG_INDEX_COUNT_BITS));
			m_node = 0x80000000 | (faceIndexCount << (32 - DG_INDEX_COUNT_BITS - 1)) | faceIndexStart;
		}

		inline dgAABBTree* GetNode (const void* root) const
		{
			return ((dgAABBTree*) root) + m_node;
		}

		union {
			dgUnsigned32 m_node;
		};
	};

	class dgConstructionTree
	{
		public:
		DG_CLASS_ALLOCATOR(allocator)

		dgConstructionTree ()
		{
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

		dgVector m_p0;
		dgVector m_p1;
		dgInt32 m_boxIndex;
		dgFloat32 m_surfaceArea;
		dgConstructionTree* m_back;
		dgConstructionTree* m_front;
		dgConstructionTree* m_parent;
	};



	
	public: 
	void CalcExtends (dgTriplex* const vertex, dgInt32 indexCount, const dgInt32* const indexArray) 
	{
		dgVector minP ( dgFloat32 (1.0e15f),  dgFloat32 (1.0e15f),  dgFloat32 (1.0e15f), dgFloat32 (0.0f)); 
		dgVector maxP (-dgFloat32 (1.0e15f), -dgFloat32 (1.0e15f), -dgFloat32 (1.0e15f), dgFloat32 (0.0f)); 
		for (dgInt32 i = 0; i < indexCount; i ++) {
			dgInt32 index = indexArray[i];
			dgVector p (&vertex[index].m_x);

			minP.m_x = dgMin (p.m_x, minP.m_x); 
			minP.m_y = dgMin (p.m_y, minP.m_y); 
			minP.m_z = dgMin (p.m_z, minP.m_z); 

			maxP.m_x = dgMax (p.m_x, maxP.m_x); 
			maxP.m_y = dgMax (p.m_y, maxP.m_y); 
			maxP.m_z = dgMax (p.m_z, maxP.m_z); 
		}

		vertex[m_minIndex].m_x = minP.m_x - dgFloat32 (1.0e-3f);
		vertex[m_minIndex].m_y = minP.m_y - dgFloat32 (1.0e-3f);
		vertex[m_minIndex].m_z = minP.m_z - dgFloat32 (1.0e-3f);
		vertex[m_maxIndex].m_x = maxP.m_x + dgFloat32 (1.0e-3f);
		vertex[m_maxIndex].m_y = maxP.m_y + dgFloat32 (1.0e-3f);
		vertex[m_maxIndex].m_z = maxP.m_z + dgFloat32 (1.0e-3f);
	}

	dgInt32 BuildTree (dgConstructionTree* const root, dgAABBTree* const boxArray, dgAABBTree* const boxCopy, dgTriplex* const vertexArrayOut, dgInt32 &treeVCount)
	{
		TreeNode* parent[128];
		dgConstructionTree* pool[128];

		dgInt32 index = 0;
		dgInt32 stack = 1;
		parent[0] = NULL;
		pool[0] = root;
		while (stack) {
			stack --;

			dgConstructionTree* const node = pool[stack];
			TreeNode* const parentNode = parent[stack];
			if (node->m_boxIndex != -1) {
				if (parentNode) {
					*parentNode = boxCopy[node->m_boxIndex].m_back;
				} else {

					//dgAssert (boxCount == 1);
					dgAABBTree* const newNode = &boxArray[index];
					*newNode = boxCopy[node->m_boxIndex];
					index ++;
				}

			} else {
				dgAABBTree* const newNode = &boxArray[index];

				newNode->m_minIndex = treeVCount;
				vertexArrayOut[treeVCount].m_x = node->m_p0.m_x;
				vertexArrayOut[treeVCount].m_y = node->m_p0.m_y;
				vertexArrayOut[treeVCount].m_z = node->m_p0.m_z;

				newNode->m_maxIndex = treeVCount + 1;
				vertexArrayOut[treeVCount + 1].m_x = node->m_p1.m_x;
				vertexArrayOut[treeVCount + 1].m_y = node->m_p1.m_y;
				vertexArrayOut[treeVCount + 1].m_z = node->m_p1.m_z;
				treeVCount += 2;

				if (parentNode) {
					*parentNode = TreeNode (dgUnsigned32(index));
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

		// this object is not to be deleted when using stack allocation
		//delete root;
		return index;
	}

	void PushNodes (dgConstructionTree* const root, dgList<dgConstructionTree*>& list) const
	{
		if (root->m_back) {
			PushNodes (root->m_back, list);
		}
		if (root->m_front) {
			PushNodes (root->m_front, list);
		}
		if (root->m_boxIndex == -1) {
			list.Append(root);
		}
	}

	dgInt32 CalculateMaximunDepth (dgConstructionTree* tree) const 
	{
		dgInt32 depthPool[128];
		dgConstructionTree* pool[128];

		depthPool[0] = 0;
		pool[0] = tree;
		dgInt32 stack = 1;

		dgInt32 maxDepth = -1;
		while (stack) {
			stack --;

			dgInt32 depth = depthPool[stack];
			dgConstructionTree* const node = pool[stack];
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

	void SetBox (dgConstructionTree* const node) const
	{
		node->m_p0 = node->m_back->m_p0.GetMin(node->m_front->m_p0);
		node->m_p1 = node->m_back->m_p1.GetMin(node->m_front->m_p1);
		dgVector size (node->m_p1 - node->m_p0);
		node->m_surfaceArea = size.DotProduct4(size.ShiftTripleRight()).m_x;
	}

	void ImproveNodeFitness (dgConstructionTree* const node) const
	{
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
					SetBox (parent);
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
					SetBox (parent);
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
					SetBox (parent);
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
					SetBox (parent);
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

	dgConstructionTree* ImproveTotalFitness (dgConstructionTree* const root, dgAABBTree* const boxArray, dgMemoryAllocator* const allocator)
	{
		dgList<dgConstructionTree*> nodesList(allocator);

		dgConstructionTree* newRoot = root;
		PushNodes (root, nodesList);
		if (nodesList.GetCount()) {
			dgInt32 maxPasses = CalculateMaximunDepth (root) * 2;
			dgFloat64 newCost = TotalFitness (nodesList);
			dgFloat64 prevCost = newCost;
			do {
				prevCost = newCost;
				for (dgList<dgConstructionTree*>::dgListNode* node = nodesList.GetFirst(); node; node = node->GetNext()) {
					dgConstructionTree* const box = node->GetInfo();
					ImproveNodeFitness (box);
				}

				newCost = TotalFitness (nodesList);
				maxPasses --;
			} while (maxPasses && (newCost < prevCost));

			newRoot = nodesList.GetLast()->GetInfo();
			while (newRoot->m_parent) {
				newRoot = newRoot->m_parent;
			}
		}

		return newRoot;
	}

	dgConstructionTree* BuildTree (dgMemoryAllocator* const allocator, dgInt32 firstBox, dgInt32 lastBox, dgAABBTree* const boxArray, const dgTriplex* const vertexArray, dgConstructionTree* parent)
	{
		dgConstructionTree* const tree = new (allocator) dgConstructionTree();
		dgAssert (firstBox >= 0);
		dgAssert (lastBox >= 0);

		tree->m_parent = parent;

		if (lastBox == firstBox) {
			dgInt32 j0 = boxArray[firstBox].m_minIndex;
			dgInt32 j1 = boxArray[firstBox].m_maxIndex;
			tree->m_p0 = dgVector (vertexArray[j0].m_x, vertexArray[j0].m_y, vertexArray[j0].m_z, dgFloat32 (0.0f));
			tree->m_p1 = dgVector (vertexArray[j1].m_x, vertexArray[j1].m_y, vertexArray[j1].m_z, dgFloat32 (0.0f));
			
			tree->m_boxIndex = firstBox;
			tree->m_back = NULL;
			tree->m_front = NULL;
		} else {

			struct dgSpliteInfo
			{
				dgSpliteInfo (dgAABBTree* const boxArray, dgInt32 boxCount, const dgTriplex* const vertexArray, const dgConstructionTree* const tree)
				{

					dgVector minP ( dgFloat32 (1.0e15f)); 
					dgVector maxP (-dgFloat32 (1.0e15f)); 

					if (boxCount == 2) {
						m_axis = 1;

						for (dgInt32 i = 0; i < boxCount; i ++) {
							dgInt32 j0 = boxArray[i].m_minIndex;
							dgInt32 j1 = boxArray[i].m_maxIndex;

							dgVector p0 (&vertexArray[j0].m_x);
							dgVector p1 (&vertexArray[j1].m_x);

							minP.m_x = dgMin (p0.m_x, minP.m_x); 
							minP.m_y = dgMin (p0.m_y, minP.m_y); 
							minP.m_z = dgMin (p0.m_z, minP.m_z); 
							
							maxP.m_x = dgMax (p1.m_x, maxP.m_x); 
							maxP.m_y = dgMax (p1.m_y, maxP.m_y); 
							maxP.m_z = dgMax (p1.m_z, maxP.m_z); 
						}

					} else {
						dgVector median (dgFloat32 (0.0f));
						dgVector varian (dgFloat32 (0.0f));
						for (dgInt32 i = 0; i < boxCount; i ++) {

							dgInt32 j0 = boxArray[i].m_minIndex;
							dgInt32 j1 = boxArray[i].m_maxIndex;

							dgVector p0 (vertexArray[j0].m_x, vertexArray[j0].m_y, vertexArray[j0].m_z, dgFloat32 (0.0f));
							dgVector p1 (vertexArray[j1].m_x, vertexArray[j1].m_y, vertexArray[j1].m_z, dgFloat32 (0.0f));

							minP.m_x = dgMin (p0.m_x, minP.m_x); 
							minP.m_y = dgMin (p0.m_y, minP.m_y); 
							minP.m_z = dgMin (p0.m_z, minP.m_z); 

							maxP.m_x = dgMax (p1.m_x, maxP.m_x); 
							maxP.m_y = dgMax (p1.m_y, maxP.m_y); 
							maxP.m_z = dgMax (p1.m_z, maxP.m_z); 

							dgVector p ((p0 + p1).Scale3 (0.5f));

							median += p;
							varian += p.CompProduct3 (p);
						}

						varian = varian.Scale3 (dgFloat32 (boxCount)) - median.CompProduct3(median);


						dgInt32 index = 0;
						dgFloat32 maxVarian = dgFloat32 (-1.0e10f);
						for (dgInt32 i = 0; i < 3; i ++) {
							if (varian[i] > maxVarian) {
								index = i;
								maxVarian = varian[i];
							}
						}
	
						dgVector center = median.Scale3 (dgFloat32 (1.0f) / dgFloat32 (boxCount));

						dgFloat32 test = center[index];

						dgInt32 i0 = 0;
						dgInt32 i1 = boxCount - 1;
						dgInt32 step = sizeof (dgTriplex) / sizeof (dgFloat32);
						const dgFloat32* const points = &vertexArray[0].m_x;
						do {    
							for (; i0 <= i1; i0 ++) {
								dgInt32 j0 = boxArray[i0].m_minIndex;
								dgInt32 j1 = boxArray[i0].m_maxIndex;

								dgFloat32 val = (points[j0 * step + index] + points[j1 * step + index]) * dgFloat32 (0.5f);
								if (val > test) {
									break;
								}
							}

							for (; i1 >= i0; i1 --) {
								dgInt32 j0 = boxArray[i1].m_minIndex;
								dgInt32 j1 = boxArray[i1].m_maxIndex;

								dgFloat32 val = (points[j0 * step + index] + points[j1 * step + index]) * dgFloat32 (0.5f);
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

			dgSpliteInfo info (&boxArray[firstBox], lastBox - firstBox + 1, vertexArray, tree);

			tree->m_boxIndex = -1;
			tree->m_p0 = info.m_p0;
			tree->m_p1 = info.m_p1;

			tree->m_front = BuildTree (allocator, firstBox + info.m_axis, lastBox, boxArray, vertexArray, tree);
			tree->m_back = BuildTree (allocator, firstBox, firstBox + info.m_axis - 1, boxArray, vertexArray, tree);
		}

		dgVector side0 (tree->m_p1 - tree->m_p0);
		dgVector side1 (side0.m_y, side0.m_z, side0.m_x, dgFloat32 (0.0f));
		tree->m_surfaceArea = side0 % side1; 
	
		return tree;
	}


	dgInt32 BuildTopDown (dgMemoryAllocator* const allocator, dgInt32 boxCount, dgAABBTree* const boxArray, dgTriplex* const vertexArrayOut, dgInt32 &treeVCount, bool optimizedBuild)
	{
		dgStack <dgAABBTree> boxCopy (boxCount);
		memcpy (&boxCopy[0], boxArray, boxCount * sizeof (dgAABBTree));

		dgConstructionTree* tree = BuildTree (allocator, 0, boxCount - 1, &boxCopy[0], vertexArrayOut, NULL);

		optimizedBuild = true;
		if (optimizedBuild) {
			tree = ImproveTotalFitness (tree, &boxCopy[0], allocator);
		}

		dgInt32 count = BuildTree (tree, boxArray, &boxCopy[0], vertexArrayOut, treeVCount);
		delete tree;
		return count;
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
		dgVector mask ((minBox.CompProduct4(maxBox)) < dgVector (dgFloat32 (0.0f)));
		mask = mask & mask.ShiftTripleRight();
		mask = mask & mask.ShiftTripleRight();
		dgVector dist (maxBox.GetMax (minBox.Abs()) & mask);
		dist = dist.GetMax(dist.ShiftTripleRight());
		dist = dist.GetMax(dist.ShiftTripleRight());
		return dist.m_x;
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

	void ForAllSectors (const dgInt32* const indexArray, const dgFloat32* const vertexArray, const dgVector& minBox, const dgVector& maxBox, const dgVector& boxDistanceTravel, dgFloat32 m_maxT, dgAABBIntersectCallback callback, void* const context) const
	{
		const dgAABBTree* stackPool[DG_STACK_DEPTH];

		const dgInt32 stride = sizeof (dgTriplex) / sizeof (dgFloat32);
		dgVector origin ((maxBox + minBox).Scale4 (dgFloat32 (0.5f)));
		dgVector size ((maxBox - minBox).Scale4 (dgFloat32 (0.5f)));
		if ((boxDistanceTravel % boxDistanceTravel) < dgFloat32 (1.0e-8f)) {
			dgInt32 stack = 1;
			stackPool[0] = this;

			while (stack) {
				stack --;
				const dgAABBTree* const me = stackPool[stack];
				if (me->BoxTest ((dgTriplex*) vertexArray, minBox, maxBox)) {

					if (me->m_back.IsLeaf()) {
						dgInt32 index = dgInt32 (me->m_back.GetIndex());
						dgInt32 vCount = me->m_back.GetCount();
						if (vCount > 0) {
							const dgInt32* const indices = &indexArray[index];
							dgInt32 normalIndex = indices[vCount + 1] * stride;
							dgVector faceNormal (&vertexArray[normalIndex]);
							if (PolygonBoxOBBTest (faceNormal, vCount, indices, stride, vertexArray, origin, size)) {
								if (callback(context, vertexArray, sizeof (dgTriplex), indices, vCount, dgFloat32 (0.0f)) == t_StopSearh) {
									return;
								}
							}
						}

					} else {
						dgAssert (stack < DG_STACK_DEPTH);
						stackPool[stack] = me->m_back.GetNode(this);
						stack++;
					}

					if (me->m_front.IsLeaf()) {
						dgInt32 index = dgInt32 (me->m_front.GetIndex());
						dgInt32 vCount = me->m_front.GetCount();
						if (vCount > 0) {
							const dgInt32* const indices = &indexArray[index];
							dgInt32 normalIndex = indices[vCount + 1] * stride;
							dgVector faceNormal (&vertexArray[normalIndex]);
							if (PolygonBoxOBBTest (faceNormal, vCount, indices, stride, vertexArray, origin, size)) {
								if (callback(context, vertexArray, sizeof (dgTriplex), indices, vCount, dgFloat32 (0.0f)) == t_StopSearh) {
									return;
								}
							}
						}

					} else {
						dgAssert (stack < DG_STACK_DEPTH);
						stackPool[stack] = me->m_front.GetNode(this);
						stack ++;
					}
				}
			}
		} else {

			dgFloat32 distance[DG_STACK_DEPTH];
			dgFastRayTest ray (dgVector (dgFloat32 (0.0f)), boxDistanceTravel);

			dgInt32 stack = 1;
			stackPool[0] = this;
			distance [0] = BoxIntersect (ray, (dgTriplex*) vertexArray, minBox, maxBox);

			while (stack) {
				stack --;
				const dgFloat32 dist = distance[stack];
				const dgAABBTree* const me = stackPool[stack];
				if (dist < dgFloat32 (1.0f)) {

					if (me->m_back.IsLeaf()) {
						dgInt32 index = dgInt32 (me->m_back.GetIndex());
						dgInt32 vCount = me->m_back.GetCount();
						if (vCount > 0) {
							const dgInt32* const indices = &indexArray[index];
							dgInt32 normalIndex = indices[vCount + 1] * stride;
							dgVector faceNormal (&vertexArray[normalIndex]);
							dgFloat32 hitDistance = PolygonBoxOBBRayTest (faceNormal, vCount, indices, stride, vertexArray, origin, size, boxDistanceTravel);
							if (hitDistance < dgFloat32 (1.0f)) {
								if (callback(context, vertexArray, sizeof (dgTriplex), indices, vCount, dgMax (hitDistance, dist)) == t_StopSearh) {
									return;
								}
							}
						}

					} else {
						dgAssert (stack < DG_STACK_DEPTH);

						const dgAABBTree* const node = me->m_back.GetNode(this);
						dgFloat32 dist = node->BoxIntersect (ray, (dgTriplex*) vertexArray, minBox, maxBox);
						if (dist < dgFloat32 (1.0f)) {
							dgInt32 j = stack;
							for ( ; j && (dist > distance[j - 1]); j --) {
								stackPool[j] = stackPool[j - 1];
								distance[j] = distance[j - 1];
							}
							stackPool[j] = node;
							distance[j] = dist;
							stack++;
						}
					}

					if (me->m_front.IsLeaf()) {
						dgInt32 index = dgInt32 (me->m_front.GetIndex());
						dgInt32 vCount = me->m_front.GetCount();
						if (vCount > 0) {
							const dgInt32* const indices = &indexArray[index];
							dgInt32 normalIndex = indices[vCount + 1] * stride;
							dgVector faceNormal (&vertexArray[normalIndex]);
							dgFloat32 hitDistance = PolygonBoxOBBRayTest (faceNormal, vCount, indices, stride, vertexArray, origin, size, boxDistanceTravel);
							if (hitDistance < dgFloat32 (1.0f)) {
								if (callback(context, vertexArray, sizeof (dgTriplex), indices, vCount, dgMax (hitDistance, dist)) == t_StopSearh) {
									return;
								}
							}
						}

					} else {
						dgAssert (stack < DG_STACK_DEPTH);
						const dgAABBTree* const node = me->m_front.GetNode(this);
						dgFloat32 dist = node->BoxIntersect (ray, (dgTriplex*) vertexArray, minBox, maxBox);
						if (dist < dgFloat32 (1.0f)) {
							dgInt32 j = stack;
							for ( ; j && (dist > distance[j - 1]); j --) {
								stackPool[j] = stackPool[j - 1];
								distance[j] = distance[j - 1];
							}
							stackPool[j] = node;
							distance[j] = dist;
							stack ++;
						}
					}
				}
			}
		}
	}

	
	void ForAllSectorsRayHit (const dgFastRayTest& raySrc, const dgInt32* const indexArray, const dgFloat32* const vertexArray, dgRayIntersectCallback callback, void* const context) const
	{
		const dgAABBTree *stackPool[DG_STACK_DEPTH];
		dgFastRayTest ray (raySrc);
		
		dgInt32 stack = 1;
		dgFloat32 maxParam = dgFloat32 (1.0f);

		stackPool[0] = this;
		while (stack) {
			stack --;
			const dgAABBTree *const me = stackPool[stack];
			if (me->RayTest (ray, (dgTriplex*) vertexArray)) {
				
				if (me->m_back.IsLeaf()) {
					dgInt32 vCount = me->m_back.GetCount();
					if (vCount > 0) {
						dgInt32 index = dgInt32 (me->m_back.GetIndex());
						dgFloat32 param = callback(context, vertexArray, sizeof (dgTriplex), &indexArray[index], vCount);
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
					dgAssert (stack < DG_STACK_DEPTH);
					stackPool[stack] = me->m_back.GetNode(this);
					stack++;
				}

				if (me->m_front.IsLeaf()) {
					dgInt32 vCount = me->m_front.GetCount();
					if (vCount > 0) {
						dgInt32 index = dgInt32 (me->m_front.GetIndex());
						dgFloat32 param = callback(context, vertexArray, sizeof (dgTriplex), &indexArray[index], vCount);
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
					dgAssert (stack < DG_STACK_DEPTH);
					stackPool[stack] = me->m_front.GetNode(this);
					stack ++;
				}
			}
		}
	}


	dgVector ForAllSectorsSupportVertex (const dgVector& dir, const dgInt32* const indexArray, const dgFloat32* const vertexArray) const
	{
		dgFloat32 aabbProjection[DG_STACK_DEPTH];
		const dgAABBTree *stackPool[DG_STACK_DEPTH];

		dgInt32 stack = 1;
		stackPool[0] = this;
		aabbProjection[0] = dgFloat32 (1.0e10f);
		const dgTriplex* const boxArray = (dgTriplex*)vertexArray;
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
				const dgAABBTree* const me = stackPool[stack];
				if (me->m_back.IsLeaf()) {
					backSupportDist = dgFloat32 (-1.0e20f);
					dgInt32 index = dgInt32 (me->m_back.GetIndex());
					dgInt32 vCount = me->m_back.GetCount();
					dgVector vertex (dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f));
					for (dgInt32 j = 0; j < vCount; j ++) {
						dgInt32 i0 = indexArray[index + j] * dgInt32 (sizeof (dgTriplex) / sizeof (dgFloat32));
						dgVector p (&vertexArray[i0]);
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
					const dgAABBTree* const node = me->m_back.GetNode(this);
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
						dgInt32 i0 = indexArray[index + j] * dgInt32 (sizeof (dgTriplex) / sizeof (dgFloat32));
						dgVector p (&vertexArray[i0]);
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
					const dgAABBTree* const node = me->m_front.GetNode(this);
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
						stackPool[stack] = me->m_back.GetNode(this);
						stack++;
					}

					if (!me->m_front.IsLeaf()) {
						aabbProjection[stack] = frontSupportDist;
						stackPool[stack] = me->m_front.GetNode(this);
						stack++;
					}

				} else {

					if (!me->m_front.IsLeaf()) {
						aabbProjection[stack] = frontSupportDist;
						stackPool[stack] = me->m_front.GetNode(this);
						stack++;
					}

					if (!me->m_back.IsLeaf()) {
						aabbProjection[stack] = backSupportDist;
						stackPool[stack] = me->m_back.GetNode(this);
						stack++;
					}
				}
			}
		}

		return supportVertex;
	}


	private:

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

	static inline dgInt32 CompareBox (const dgConstructionTree* const boxA, const dgConstructionTree* const boxB, void* const context)
	{
		dgInt32 axis = *((dgInt32*) context);

		if (boxA->m_p0[axis] < boxB->m_p0[axis]) {
			return -1;
		} else if (boxA->m_p0[axis] > boxB->m_p0[axis]) {
			return 1;
		}
		return 0;
	}

	dgInt32 m_minIndex;
	dgInt32 m_maxIndex;
	TreeNode m_back;
	TreeNode m_front;
	friend class dgAABBPolygonSoup;
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
	dgAABBTree* const node = (dgAABBTree*) root;
	return node->m_back.IsLeaf() ? NULL : node->m_back.GetNode(m_aabb);
}

void* dgAABBPolygonSoup::GetFrontNode(const void* const root) const
{
	dgAABBTree* const node = (dgAABBTree*) root;
	return node->m_front.IsLeaf() ? NULL : node->m_front.GetNode(m_aabb);
}


void dgAABBPolygonSoup::GetNodeAABB(const void* const root, dgVector& p0, dgVector& p1) const
{
	dgAABBTree* const node = (dgAABBTree*) root;
	const dgTriplex* const vertex = (dgTriplex*) m_localVertex;

	p0 = dgVector (vertex[node->m_minIndex].m_x, vertex[node->m_minIndex].m_y, vertex[node->m_minIndex].m_z, dgFloat32 (0.0f));
	p1 = dgVector (vertex[node->m_maxIndex].m_x, vertex[node->m_maxIndex].m_y, vertex[node->m_maxIndex].m_z, dgFloat32 (0.0f));
}


void dgAABBPolygonSoup::ForAllSectors (const dgVector& minBox, const dgVector& maxBox, const dgVector& boxDistanceTravel, dgFloat32 m_maxT, dgAABBIntersectCallback callback, void* const context) const
{
	if (m_aabb) {
		dgAABBTree* const tree = (dgAABBTree*) m_aabb;
		tree->ForAllSectors (m_indices, m_localVertex, minBox, maxBox, boxDistanceTravel, m_maxT, callback, context);
	}
}

dgVector dgAABBPolygonSoup::ForAllSectorsSupportVectex (const dgVector& dir) const
{
	if (m_aabb) {
		dgAABBTree* const tree = (dgAABBTree*) m_aabb;
		return tree->ForAllSectorsSupportVertex (dir, m_indices, m_localVertex);
	} else {
		return dgVector (0, 0, 0, 0);
	}
}


void dgAABBPolygonSoup::GetAABB (dgVector& p0, dgVector& p1) const
{
	if (m_aabb) { 
		dgAABBTree* const tree = (dgAABBTree*) m_aabb;
		p0 = dgVector (&m_localVertex[tree->m_minIndex * 3]);
		p1 = dgVector (&m_localVertex[tree->m_maxIndex * 3]);
	} else {
		p0 = dgVector (dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f));
		p1 = dgVector (dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f));
	}
}


void dgAABBPolygonSoup::ForAllSectorsRayHit (const dgFastRayTest& ray, dgRayIntersectCallback callback, void* const context) const
{
	if (m_aabb) {
		dgAABBTree* const tree = (dgAABBTree*) m_aabb;
		tree->ForAllSectorsRayHit (ray, m_indices, m_localVertex, callback, context);
	}
}


void dgAABBPolygonSoup::Serialize (dgSerialize callback, void* const userData) const
{
	dgAABBTree* const tree = (dgAABBTree*) m_aabb;
	callback (userData, &m_vertexCount, sizeof (dgInt32));
	callback (userData, &m_indexCount, sizeof (dgInt32));
	callback (userData, &m_nodesCount, sizeof (dgInt32));
	callback (userData, &m_nodesCount, sizeof (dgInt32));
	if (tree) {
		callback (userData,  m_localVertex, sizeof (dgTriplex) * m_vertexCount);
		callback (userData,  m_indices, sizeof (dgInt32) * m_indexCount);
		callback (userData, tree, sizeof (dgAABBTree) * m_nodesCount);
	}
}


void dgAABBPolygonSoup::Deserialize (dgDeserialize callback, void* const userData)
{
//	dgInt32 nodes;
	dgAABBTree* tree;;

	m_strideInBytes = sizeof (dgTriplex);
	callback (userData, &m_vertexCount, sizeof (dgInt32));
	callback (userData, &m_indexCount, sizeof (dgInt32));
	callback (userData, &m_nodesCount, sizeof (dgInt32));
	callback (userData, &m_nodesCount, sizeof (dgInt32));

	if (m_vertexCount) {
		m_localVertex = (dgFloat32*) dgMallocStack (sizeof (dgTriplex) * m_vertexCount);
		m_indices = (dgInt32*) dgMallocStack (sizeof (dgInt32) * m_indexCount);
		tree = (dgAABBTree*) dgMallocStack (sizeof (dgAABBTree) * m_nodesCount);

		callback (userData, m_localVertex, sizeof (dgTriplex) * m_vertexCount);
		callback (userData, m_indices, sizeof (dgInt32) * m_indexCount);
		callback (userData, tree, sizeof (dgAABBTree) * m_nodesCount);
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
	dgVector p0 ( dgFloat32 (1.0e15f),  dgFloat32 (1.0e15f),  dgFloat32 (1.0e15f), dgFloat32 (0.0f)); 
	dgVector p1 (-dgFloat32 (1.0e15f), -dgFloat32 (1.0e15f), -dgFloat32 (1.0e15f), dgFloat32 (0.0f)); 
	for (dgInt32 i = 0; i < indexCount; i ++) {
		dgInt32 i1 = indexArray[i];
		dgInt32 index = i1 * stride;
		dgVector p (&polygon[index]);

		p0.m_x = dgMin (p.m_x, p0.m_x); 
		p0.m_y = dgMin (p.m_y, p0.m_y); 
		p0.m_z = dgMin (p.m_z, p0.m_z); 
								
		p1.m_x = dgMax (p.m_x, p1.m_x); 
		p1.m_y = dgMax (p.m_y, p1.m_y); 
		p1.m_z = dgMax (p.m_z, p1.m_z); 

		adjacentFaces.m_edgeMap[edgeIndex] = (dgInt64 (i1) << 32) + i0;
		edgeIndex = i;
		i0 = i1;
	}

	p0.m_x -= dgFloat32 (0.25f);
	p0.m_y -= dgFloat32 (0.25f);
	p0.m_z -= dgFloat32 (0.25f);
	p1.m_x += dgFloat32 (0.25f);
	p1.m_y += dgFloat32 (0.25f);
	p1.m_z += dgFloat32 (0.25f);

	//me->ForAllSectors (p0, p1, dgVector (dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f)), CalculateManifoldFaceEdgeNormals, &adjacentFaces);
	me->ForAllSectors (p0, p1, dgVector (dgFloat32 (0.0f)), dgFloat32 (1.0f), CalculateDisjointedFaceEdgeNormals, &adjacentFaces);

	return t_ContinueSearh;
}

dgFloat32 dgAABBPolygonSoup::CalculateFaceMaxSize (dgTriplex* const vertex, dgInt32 indexCount, const dgInt32* const indexArray) const
{
	dgFloat32 maxSize = dgFloat32 (0.0f);
	dgInt32 index = indexArray[indexCount - 1];
	dgVector p0 (vertex[index].m_x, vertex[index].m_y, vertex[index].m_z, dgFloat32 (0.0f));
	for (dgInt32 i = 0; i < indexCount; i ++) {
		dgInt32 index = indexArray[i];
		dgVector p1 (vertex[index].m_x, vertex[index].m_y, vertex[index].m_z, dgFloat32 (0.0f));

		dgVector dir (p1 - p0);
		dir = dir.Scale3 (dgRsqrt (dir % dir));

		dgFloat32 maxVal = dgFloat32 (-1.0e10f);
		dgFloat32 minVal = dgFloat32 ( 1.0e10f);
		for (dgInt32 j = 0; j < indexCount; j ++) {
			dgInt32 index = indexArray[j];
			dgVector q (vertex[index].m_x, vertex[index].m_y, vertex[index].m_z, dgFloat32 (0.0f));
			dgFloat32 val = dir % q;
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

	dgAABBTree* const tree = (dgAABBTree*) m_aabb;
	for (dgInt32 i = 0; i < m_nodesCount; i ++) {
		dgAABBTree* const node = &tree[i];
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

void dgAABBPolygonSoup::Create (dgMemoryAllocator* const allocator, const dgPolygonSoupDatabaseBuilder& builder, bool optimizedBuild)
{
	if (builder.m_faceCount == 0) {
		return;
	}

	m_strideInBytes = sizeof (dgTriplex);
	m_indexCount = builder.m_indexCount * 2 + builder.m_faceCount;
	m_indices = (dgInt32*) dgMallocStack (sizeof (dgInt32) * m_indexCount);
	m_aabb = (dgAABBTree*) dgMallocStack (sizeof (dgAABBTree) * builder.m_faceCount);
	m_localVertex = (dgFloat32*) dgMallocStack (sizeof (dgTriplex) * (builder.m_vertexCount + builder.m_normalCount + builder.m_faceCount * 4));

	dgAABBTree* const tree = (dgAABBTree*) m_aabb;
	dgTriplex* const tmpVertexArray = (dgTriplex*)m_localVertex;
	
	for (dgInt32 i = 0; i < builder.m_vertexCount; i ++) {
		tmpVertexArray[i].m_x = dgFloat32 (builder.m_vertexPoints[i].m_x);
		tmpVertexArray[i].m_y = dgFloat32 (builder.m_vertexPoints[i].m_y);
		tmpVertexArray[i].m_z = dgFloat32 (builder.m_vertexPoints[i].m_z);
	}

	for (dgInt32 i = 0; i < builder.m_normalCount; i ++) {
		tmpVertexArray[i + builder.m_vertexCount].m_x = dgFloat32 (builder.m_normalPoints[i].m_x); 
		tmpVertexArray[i + builder.m_vertexCount].m_y = dgFloat32 (builder.m_normalPoints[i].m_y); 
		tmpVertexArray[i + builder.m_vertexCount].m_z = dgFloat32 (builder.m_normalPoints[i].m_z); 
	}

	dgInt32 polygonIndex = 0;
	dgInt32* indexMap = m_indices;
	const dgInt32* const indices = &builder.m_vertexIndex[0];

	// index format i0, i1, i2, ... , id, normal, e0Normal, e1Normal, e2Normal, ..., faceSize
	for (dgInt32 i = 0; i < builder.m_faceCount; i ++) {

		dgInt32 indexCount = builder.m_faceVertexCount[i] - 1;
		dgAABBTree& box = tree[i];

		box.m_minIndex = builder.m_normalCount + builder.m_vertexCount + i * 2;
		box.m_maxIndex = builder.m_normalCount + builder.m_vertexCount + i * 2 + 1;

		box.m_front = dgAABBTree::TreeNode(0, 0);
		//box.m_back = dgAABBTree::TreeNode (dgUnsigned32 (indexCount * 2), dgUnsigned32 (indexMap - m_indices));
		box.m_back = dgAABBTree::TreeNode (dgUnsigned32 (indexCount), dgUnsigned32 (indexMap - m_indices));
		box.CalcExtends (&tmpVertexArray[0], indexCount, &indices[polygonIndex]);

		dgInt32 normalIndex = builder.m_vertexCount + builder.m_normalIndex[i];;
		for (dgInt32 j = 0; j < indexCount; j ++) {
			// face vertex
			indexMap[j] = indices[polygonIndex + j];
			// face edge normal 
			//indexMap[j + indexCount + 2] = normalIndex;
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

	dgInt32 extraVertexCount = builder.m_normalCount + builder.m_vertexCount + builder.m_faceCount * 2;

	m_nodesCount = tree->BuildTopDown (builder.m_allocator, builder.m_faceCount, tree, &tmpVertexArray[0], extraVertexCount, optimizedBuild);

	m_localVertex[tree->m_minIndex * 3 + 0] -= dgFloat32 (0.1f);
	m_localVertex[tree->m_minIndex * 3 + 1] -= dgFloat32 (0.1f);
	m_localVertex[tree->m_minIndex * 3 + 2] -= dgFloat32 (0.1f);
	m_localVertex[tree->m_maxIndex * 3 + 0] += dgFloat32 (0.1f);
	m_localVertex[tree->m_maxIndex * 3 + 1] += dgFloat32 (0.1f);
	m_localVertex[tree->m_maxIndex * 3 + 2] += dgFloat32 (0.1f);


	extraVertexCount -= (builder.m_normalCount + builder.m_vertexCount);

	dgStack<dgInt32> indexArray (extraVertexCount);
	extraVertexCount = dgVertexListToIndexList (&tmpVertexArray[builder.m_normalCount + builder.m_vertexCount].m_x, sizeof (dgTriplex), sizeof (dgTriplex), 0, extraVertexCount, &indexArray[0], dgFloat32 (1.0e-6f));

	for (dgInt32 i = 0; i < m_nodesCount; i ++) {
		dgAABBTree& box = tree[i];

		dgInt32 j = box.m_minIndex - builder.m_normalCount - builder.m_vertexCount;
		box.m_minIndex = indexArray[j] + builder.m_normalCount + builder.m_vertexCount;
		j = box.m_maxIndex - builder.m_normalCount - builder.m_vertexCount;
		box.m_maxIndex = indexArray[j] + builder.m_normalCount + builder.m_vertexCount;
	}
	m_vertexCount = extraVertexCount + builder.m_normalCount + builder.m_vertexCount;

	CalculateAdjacendy();
}

#else


DG_MSC_VECTOR_AVX_ALIGMENT
class dgAABBPolygonSoup::dgNodeBuilder: public dgAABBPolygonSoup::dgNode
{
	public:
	dgNodeBuilder (const dgVector* const vertexArray, dgInt32 faceIndex, dgInt32 indexCount, const dgInt32* const indexArray)
		:dgNode()
		,m_parent (NULL)
		,m_right (NULL)
		,m_left (NULL)
		,m_indexBox0(0)
		,m_indexBox1(0)
		,m_enumeration(-1)
		,m_faceIndex(faceIndex)
		,m_indexCount(indexCount)
		,m_faceIndices(indexArray)
	{
		dgVector minP ( dgFloat32 (1.0e15f)); 
		dgVector maxP (-dgFloat32 (1.0e15f)); 
		for (dgInt32 i = 0; i < indexCount; i ++) {
			dgInt32 index = indexArray[i];
			const dgVector& p (vertexArray[index]);
			minP = p.GetMin(minP); 
			maxP = p.GetMax(maxP); 
		}
		minP -= dgVector (dgFloat32 (1.0e-3f));
		maxP += dgVector (dgFloat32 (1.0e-3f));
		minP = minP & dgVector::m_triplexMask;
		maxP = maxP & dgVector::m_triplexMask;
		SetBox (minP, maxP);
	}

	dgNodeBuilder (dgNodeBuilder* const left, dgNodeBuilder* const right)
		:dgNode()
		,m_parent(NULL)
		,m_left(left)
		,m_right(right)
		,m_indexBox0(0)
		,m_indexBox1(0)
		,m_enumeration(-1)
		,m_faceIndex(0)
		,m_indexCount(0)
		,m_faceIndices(NULL)
	{
		m_left->m_parent = this;
		m_right->m_parent = this;

		dgVector p0 (left->m_p0.GetMin(right->m_p0));
		dgVector p1 (left->m_p1.GetMax(right->m_p1));
		SetBox(p0, p1);
	}

	void SetBox (const dgVector& p0, const dgVector& p1)
	{
		m_p0 = p0;
		m_p1 = p1;
		m_size = m_p1 - m_p0;
		m_area = m_size.DotProduct4(m_size.ShiftTripleRight()).m_x;
	}

	static dgFloat32 CalculateSurfaceArea (dgNodeBuilder* const node0, dgNodeBuilder* const node1, dgVector& minBox, dgVector& maxBox)
	{
		minBox = node0->m_p0.GetMin(node1->m_p0);
		maxBox = node0->m_p1.GetMax(node1->m_p1);

		dgVector side0 ((maxBox - minBox).Scale4 (dgFloat32 (0.5f)));
		dgVector side1 (side0.m_y, side0.m_z, side0.m_x, dgFloat32 (0.0f));
		return side0.DotProduct4(side1).m_x;
	}


	dgVector m_p0;
	dgVector m_p1;
	dgVector m_size;
	dgVector m_origin;
	dgFloat32 m_area;
	dgNodeBuilder* m_parent;
	dgNodeBuilder* m_right;
	dgNodeBuilder* m_left;
	dgInt32 m_indexBox0;
	dgInt32 m_indexBox1;
	dgInt32 m_enumeration;
	dgInt32 m_faceIndex;
	dgInt32 m_indexCount;
	const dgInt32* m_faceIndices;
} DG_GCC_VECTOR_AVX_ALIGMENT;



dgAABBPolygonSoup::dgAABBPolygonSoup ()
	:dgPolygonSoupDatabase()
	,m_nodesCount(0)
	,m_indexCount(0)
	,m_aabb(NULL)
	,m_indices(NULL)
{
}

dgAABBPolygonSoup::~dgAABBPolygonSoup ()
{
	if (m_aabb) {
		dgFreeStack (m_aabb);
		dgFreeStack (m_indices);
	}
}


void dgAABBPolygonSoup::ImproveNodeFitness (dgNodeBuilder* const node) const
{
	dgAssert (node->m_left);
	dgAssert (node->m_right);

	if (node->m_parent)	{
		if (node->m_parent->m_left == node) {
			dgFloat32 cost0 = node->m_area;

			dgVector cost1P0;
			dgVector cost1P1;		
			dgFloat32 cost1 = dgNodeBuilder::CalculateSurfaceArea (node->m_right, node->m_parent->m_right, cost1P0, cost1P1);

			dgVector cost2P0;
			dgVector cost2P1;		
			dgFloat32 cost2 = dgNodeBuilder::CalculateSurfaceArea (node->m_left, node->m_parent->m_right, cost2P0, cost2P1);

			if ((cost1 <= cost0) && (cost1 <= cost2)) {
				dgNodeBuilder* const parent = node->m_parent;
				node->m_p0 = parent->m_p0;
				node->m_p1 = parent->m_p1;
				node->m_area = parent->m_area; 
				node->m_size = parent->m_size;
				node->m_origin = parent->m_origin;

				if (parent->m_parent) {
					if (parent->m_parent->m_left == parent) {
						parent->m_parent->m_left = node;
					} else {
						dgAssert (parent->m_parent->m_right == parent);
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
				parent->m_size = (parent->m_p1 - parent->m_p0).Scale3(dgFloat32 (0.5f));
				parent->m_origin = (parent->m_p1 + parent->m_p0).Scale3(dgFloat32 (0.5f));

			} else if ((cost2 <= cost0) && (cost2 <= cost1)) {
				dgNodeBuilder* const parent = node->m_parent;
				node->m_p0 = parent->m_p0;
				node->m_p1 = parent->m_p1;
				node->m_area = parent->m_area; 
				node->m_size = parent->m_size;
				node->m_origin = parent->m_origin;

				if (parent->m_parent) {
					if (parent->m_parent->m_left == parent) {
						parent->m_parent->m_left = node;
					} else {
						dgAssert (parent->m_parent->m_right == parent);
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
				parent->m_size = (parent->m_p1 - parent->m_p0).Scale3(dgFloat32 (0.5f));
				parent->m_origin = (parent->m_p1 + parent->m_p0).Scale3(dgFloat32 (0.5f));
			}
		} else {
			dgFloat32 cost0 = node->m_area;

			dgVector cost1P0;
			dgVector cost1P1;		
			dgFloat32 cost1 = dgNodeBuilder::CalculateSurfaceArea (node->m_left, node->m_parent->m_left, cost1P0, cost1P1);

			dgVector cost2P0;
			dgVector cost2P1;		
			dgFloat32 cost2 = dgNodeBuilder::CalculateSurfaceArea (node->m_right, node->m_parent->m_left, cost2P0, cost2P1);

			if ((cost1 <= cost0) && (cost1 <= cost2)) {
				dgNodeBuilder* const parent = node->m_parent;
				node->m_p0 = parent->m_p0;
				node->m_p1 = parent->m_p1;
				node->m_area = parent->m_area; 
				node->m_size = parent->m_size;
				node->m_origin = parent->m_origin;

				if (parent->m_parent) {
					if (parent->m_parent->m_left == parent) {
						parent->m_parent->m_left = node;
					} else {
						dgAssert (parent->m_parent->m_right == parent);
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
				parent->m_size = (parent->m_p1 - parent->m_p0).Scale3(dgFloat32 (0.5f));
				parent->m_origin = (parent->m_p1 + parent->m_p0).Scale3(dgFloat32 (0.5f));

			} else if ((cost2 <= cost0) && (cost2 <= cost1)) {
				dgNodeBuilder* const parent = node->m_parent;
				node->m_p0 = parent->m_p0;
				node->m_p1 = parent->m_p1;
				node->m_area = parent->m_area; 
				node->m_size = parent->m_size;
				node->m_origin = parent->m_origin;

				if (parent->m_parent) {
					if (parent->m_parent->m_left == parent) {
						parent->m_parent->m_left = node;
					} else {
						dgAssert (parent->m_parent->m_right == parent);
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
				parent->m_size = (parent->m_p1 - parent->m_p0).Scale3(dgFloat32 (0.5f));
				parent->m_origin = (parent->m_p1 + parent->m_p0).Scale3(dgFloat32 (0.5f));
			}
		}
	} else {
		// in the future I can handle this but it is too much work for little payoff
	}
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
		dir = dir.Scale3 (dgRsqrt (dir % dir));

		dgFloat32 maxVal = dgFloat32 (-1.0e10f);
		dgFloat32 minVal = dgFloat32 ( 1.0e10f);
		for (dgInt32 j = 0; j < indexCount; j ++) {
			dgInt32 index = indexArray[j];
			dgVector q (vertex[index]);
			dgFloat32 val = dir % q;
			minVal = dgMin(minVal, val);
			maxVal = dgMax(maxVal, val);
		}

		dgFloat32 size = maxVal - minVal;
		maxSize = dgMax(maxSize, size);
		p0 = p1;
	}

	return dgFloor (maxSize + dgFloat32 (1.0f));
}




void dgAABBPolygonSoup::Create (dgMemoryAllocator* const allocator, const dgPolygonSoupDatabaseBuilder& builder, bool optimizedBuild)
{
	if (builder.m_faceCount == 0) {
		return;
	}
	dgAssert (builder.m_faceCount >= 2);
	m_strideInBytes = sizeof (dgTriplex);
	m_nodesCount = builder.m_faceCount - 1;
	m_indexCount = builder.m_indexCount * 2 + builder.m_faceCount;
//	m_vertexCount = builder.m_normalCount + builder.m_vertexCount;
	m_aabb = (dgNode*) dgMallocStack (sizeof (dgNode) * m_nodesCount);
	m_indices = (dgInt32*) dgMallocStack (sizeof (dgInt32) * m_indexCount);
//	m_localVertex = (dgFloat32*) dgMallocStack (sizeof (dgTriplex) * m_vertexCount);
//	dgTriplex* const tmpVertexArray = (dgTriplex*) m_localVertex;
	dgStack<dgVector> tmpVertexArray(builder.m_vertexCount + builder.m_normalCount + builder.m_faceCount * 2);

	for (dgInt32 i = 0; i < builder.m_vertexCount; i ++) {
		tmpVertexArray[i] = builder.m_vertexPoints[i];
	}

	for (dgInt32 i = 0; i < builder.m_normalCount; i ++) {
		tmpVertexArray[i + builder.m_vertexCount] = builder.m_normalPoints[i];
	}

	dgInt32 polygonIndex = 0;
	dgInt32 constructorNodesCount = builder.m_faceCount * 2 - 1;
	const dgInt32* const indices = &builder.m_vertexIndex[0];
	dgStack<dgNodeBuilder> constructor (constructorNodesCount); 

	dgNodeBuilder* root = NULL;
	dgInt32 allocatorIndex = 0;

	for (dgInt32 i = 0; i < builder.m_faceCount; i ++) {
		dgInt32 indexCount = builder.m_faceVertexCount[i] - 1;
		dgNodeBuilder* const newNode = new (&constructor[allocatorIndex]) dgNodeBuilder (&tmpVertexArray[0], i, indexCount, &indices[polygonIndex]);
		allocatorIndex ++;

		if (!root) {
			root = newNode;
		} else {
			dgVector p0;
			dgVector p1;		
			dgNodeBuilder* sibling = root;
			dgFloat32 surfaceArea = dgNodeBuilder::CalculateSurfaceArea (newNode, sibling, p0, p1);

			while(sibling->m_left && sibling->m_right) {

				if (surfaceArea > sibling->m_area) {
					break;
				} 

				sibling->SetBox (p0, p1);

				dgVector leftP0;
				dgVector leftP1;		
				dgFloat32 leftSurfaceArea = dgNodeBuilder::CalculateSurfaceArea (newNode, sibling->m_left, leftP0, leftP1);

				dgVector rightP0;
				dgVector rightP1;		
				dgFloat32 rightSurfaceArea = dgNodeBuilder::CalculateSurfaceArea (newNode, sibling->m_right, rightP0, rightP1);

				if (leftSurfaceArea < rightSurfaceArea) {
					sibling = sibling->m_left;
					p0 = leftP0;
					p1 = leftP1;
					surfaceArea = leftSurfaceArea;
				} else {
					sibling = sibling->m_right;
					p0 = rightP0;
					p1 = rightP1;
					surfaceArea = rightSurfaceArea;
				}
			} 

			if (!sibling->m_parent) {
				root = new (&constructor[allocatorIndex]) dgNodeBuilder (sibling, newNode);
				allocatorIndex ++;
				dgAssert (allocatorIndex <= constructorNodesCount);
			} else {

				dgNodeBuilder* const parent = sibling->m_parent;
				if (parent->m_left == sibling) {
					dgNodeBuilder* const node = new (&constructor[allocatorIndex]) dgNodeBuilder (sibling, newNode);
					allocatorIndex ++;
					dgAssert (allocatorIndex <= constructorNodesCount);
					parent->m_left = node;
					node->m_parent = parent;
				} else {
					dgAssert (parent->m_right == sibling); 
					dgNodeBuilder* const node = new (&constructor[allocatorIndex]) dgNodeBuilder (sibling, newNode);
					allocatorIndex ++;
					dgAssert (allocatorIndex <= constructorNodesCount);
					parent->m_right = node;
					node->m_parent = parent;
				}
			}
		}

		polygonIndex += (indexCount + 1);
	}

	dgAssert (root);
	if (root->m_left) {

		dgAssert (root->m_right);
		dgList<dgNodeBuilder*> list (allocator);
		dgList<dgNodeBuilder*> stack (allocator);
		stack.Append(root);
		while (stack.GetCount()) {
			dgList<dgNodeBuilder*>::dgListNode* const stackNode = stack.GetLast();
			dgNodeBuilder* const node = stackNode->GetInfo();
			stack.Remove(stackNode);

			if (node->m_left) {
				dgAssert (node->m_right);
				list.Append(node);
				stack.Append(node->m_right);
				stack.Append(node->m_left);
			} 
		}

		dgFloat64 newCost = dgFloat32 (1.0e20f);
		dgFloat64 prevCost = newCost;
		do {
			prevCost = newCost;
			for (dgList<dgNodeBuilder*>::dgListNode* listNode = list.GetFirst(); listNode; listNode = listNode->GetNext()) {
				dgNodeBuilder* const node = listNode->GetInfo();
				ImproveNodeFitness (node);
			}

			newCost = dgFloat32 (0.0f);
			for (dgList<dgNodeBuilder*>::dgListNode* listNode = list.GetFirst(); listNode; listNode = listNode->GetNext()) {
				dgNodeBuilder* const node = listNode->GetInfo();
				newCost += node->m_area;
			}
		} while (newCost < (prevCost * dgFloat32 (0.9999f)));

		root = list.GetLast()->GetInfo();
		while (root->m_parent) {
			root = root->m_parent;
		}
	}

	dgList<dgNodeBuilder*> list (allocator);

	list.Append(root);
	dgInt32 nodeIndex = 0;
	while (list.GetCount())  {
		dgNodeBuilder* const node = list.GetFirst()->GetInfo();
		list.Remove(list.GetFirst());

		if (node->m_left) {
			node->m_enumeration = nodeIndex;
			nodeIndex ++;
			dgAssert (node->m_right);
			list.Append(node->m_left);
			list.Append(node->m_right);
		}
	}
	dgAssert(!list.GetCount());


//	dgStack<dgVector> aabbMem (2 * m_nodesCount);
	dgInt32 aabbBase = builder.m_vertexCount + builder.m_normalCount;
	dgVector* const aabbPoints = &tmpVertexArray[aabbBase];

	dgInt32 vertexIndex = 0;
	dgInt32 aabbNodeIndex = 0;
	list.Append(root);
	dgInt32 indexMap = 0;
	while (list.GetCount())  {

		dgNodeBuilder* const node = list.GetFirst()->GetInfo();
		list.Remove(list.GetFirst());

		if (node->m_enumeration >= 0) {
			dgAssert (node->m_left);
			dgAssert (node->m_right);
			dgNode& aabbNode = m_aabb[aabbNodeIndex];
			aabbNodeIndex ++;
			dgAssert (aabbNodeIndex <= m_nodesCount);

			if (node->m_parent) {
				if (node->m_parent->m_left == node) {
					m_aabb[node->m_parent->m_enumeration].m_left = dgNode::dgLeafNodePtr (&m_aabb[node->m_parent->m_enumeration] - m_aabb);
				} else {
					dgAssert (node->m_parent->m_right == node);
					m_aabb[node->m_parent->m_enumeration].m_right = dgNode::dgLeafNodePtr (&m_aabb[node->m_parent->m_enumeration] - m_aabb);
				}
			}

			aabbPoints[vertexIndex + 0] = node->m_p0;
			aabbPoints[vertexIndex + 1] = node->m_p1;

			aabbNode.m_indexBox0 = aabbBase + vertexIndex;
			aabbNode.m_indexBox1 = aabbBase + vertexIndex + 1;

			vertexIndex += 2;

		} else {
			dgAssert (!node->m_left);
			dgAssert (!node->m_right);

			if (node->m_parent) {
				if (node->m_parent->m_left == node) {
					m_aabb[node->m_parent->m_enumeration].m_left = dgNode::dgLeafNodePtr (node->m_indexCount, indexMap);
				} else {
					dgAssert (node->m_parent->m_right == node);
					m_aabb[node->m_parent->m_enumeration].m_right = dgNode::dgLeafNodePtr (node->m_indexCount, indexMap);
				}
			}

			// index format i0, i1, i2, ... , id, normal, e0Normal, e1Normal, e2Normal, ..., faceSize
			for (dgInt32 j = 0; j < node->m_indexCount; j ++) {
				m_indices[indexMap + j] = node->m_faceIndices[j];
				m_indices[indexMap + j + node->m_indexCount + 2] = -1;
			}

			// face attribute
			m_indices[indexMap + node->m_indexCount] = node->m_faceIndices[node->m_indexCount];
			// face normal
			m_indices[indexMap + node->m_indexCount + 1] = builder.m_vertexCount + builder.m_normalIndex[node->m_faceIndex];
			// face size
			m_indices[indexMap + node->m_indexCount * 2 + 2] = dgInt32 (CalculateFaceMaxSize (&tmpVertexArray[0], node->m_indexCount, node->m_faceIndices));

			indexMap += node->m_indexCount * 2 + 3;
		}

		if (node->m_left) {
			dgAssert (node->m_right);
			list.Append(node->m_left);
			list.Append(node->m_right);
		}
	}

	dgAssert (vertexIndex == 2 * m_nodesCount);
	dgStack<dgInt32> indexArray (vertexIndex);
	dgInt32 aabbPointCount = dgVertexListToIndexList (&aabbPoints[0].m_x, sizeof (dgVector), sizeof (dgTriplex), 0, vertexIndex, &indexArray[0], dgFloat32 (1.0e-6f));

	m_vertexCount = aabbBase + aabbPointCount;
	m_localVertex = (dgFloat32*) dgMallocStack (sizeof (dgTriplex) * m_vertexCount);

	dgTriplex* const dstPoints = (dgTriplex*)m_localVertex;
	for (dgInt32 i = 0; i < m_vertexCount; i ++) {
		dstPoints[i].m_x = tmpVertexArray[i].m_x;
		dstPoints[i].m_y = tmpVertexArray[i].m_y;
		dstPoints[i].m_z = tmpVertexArray[i].m_z;
	}

	for (dgInt32 i = 0; i < m_nodesCount; i ++) {
		dgNode& box = m_aabb[i];

		dgInt32 j = box.m_indexBox0 - aabbBase;
		box.m_indexBox0 = indexArray[j] + aabbBase;

		j = box.m_indexBox1 - aabbBase;
		box.m_indexBox1 = indexArray[j] + aabbBase;
	}
	CalculateAdjacendy();
}


void dgAABBPolygonSoup::GetAABB (dgVector& p0, dgVector& p1) const
{
	if (m_aabb) { 
		GetNodeAABB (m_aabb, p0, p1);
	} else {
		p0 = dgVector (dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f));
		p1 = dgVector (dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f));
	}
}



void dgAABBPolygonSoup::CalculateAdjacendy ()
{
	dgVector p0;
	dgVector p1;
	GetAABB (p0, p1);
	ForAllSectors (p0, p1, dgVector (dgFloat32 (0.0f)), dgFloat32 (1.0f), CalculateAllFaceEdgeNormals, this);

	for (dgInt32 i = 0; i < m_nodesCount; i ++) {
		const dgNode* const node = &m_aabb[i];
		if (node->m_left.IsLeaf()) {
			dgInt32 vCount = node->m_left.GetCount();
			dgInt32 index = dgInt32 (node->m_left.GetIndex());
			dgInt32* const face = &m_indices[index];
			for (dgInt32 j = 0; j < vCount; j ++) {
				if (face[vCount + 2 + j] == -1) {
					face[vCount + 2 + j] = face[vCount + 1];
				}
			}
		}

		if (node->m_right.IsLeaf()) {
			dgInt32 vCount = node->m_right.GetCount();
			dgInt32 index = dgInt32 (node->m_right.GetIndex());
			dgInt32* const face = &m_indices[index];
			for (dgInt32 j = 0; j < vCount; j ++) {
				if (face[vCount + 2 + j] == -1) {
					face[vCount + 2 + j] = face[vCount + 1];
				}
			}
		}
	}

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
	dgVector p0 ( dgFloat32 (1.0e15f),  dgFloat32 (1.0e15f),  dgFloat32 (1.0e15f), dgFloat32 (0.0f)); 
	dgVector p1 (-dgFloat32 (1.0e15f), -dgFloat32 (1.0e15f), -dgFloat32 (1.0e15f), dgFloat32 (0.0f)); 
	for (dgInt32 i = 0; i < indexCount; i ++) {
		dgInt32 i1 = indexArray[i];
		dgInt32 index = i1 * stride;
		dgVector p (&polygon[index]);

		p0.m_x = dgMin (p.m_x, p0.m_x); 
		p0.m_y = dgMin (p.m_y, p0.m_y); 
		p0.m_z = dgMin (p.m_z, p0.m_z); 

		p1.m_x = dgMax (p.m_x, p1.m_x); 
		p1.m_y = dgMax (p.m_y, p1.m_y); 
		p1.m_z = dgMax (p.m_z, p1.m_z); 

		adjacentFaces.m_edgeMap[edgeIndex] = (dgInt64 (i1) << 32) + i0;
		edgeIndex = i;
		i0 = i1;
	}

	p0.m_x -= dgFloat32 (0.25f);
	p0.m_y -= dgFloat32 (0.25f);
	p0.m_z -= dgFloat32 (0.25f);
	p1.m_x += dgFloat32 (0.25f);
	p1.m_y += dgFloat32 (0.25f);
	p1.m_z += dgFloat32 (0.25f);

	me->ForAllSectors (p0, p1, dgVector (dgFloat32 (0.0f)), dgFloat32 (1.0f), CalculateDisjointedFaceEdgeNormals, &adjacentFaces);

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


void dgAABBPolygonSoup::ForAllSectorsRayHit (const dgFastRayTest& raySrc, dgRayIntersectCallback callback, void* const context) const
{
	const dgNode *stackPool[DG_STACK_DEPTH];
	dgFastRayTest ray (raySrc);

	dgInt32 stack = 1;
	dgFloat32 maxParam = dgFloat32 (1.0f);
	const dgTriplex* const vertexArray = (dgTriplex*) m_localVertex;

	stackPool[0] = m_aabb;
	while (stack) {
		stack --;
		const dgNode *const me = stackPool[stack];
		if (me->RayTest (ray, vertexArray)) {
			if (me->m_left.IsLeaf()) {
				dgInt32 vCount = me->m_left.GetCount();
				if (vCount > 0) {
					dgInt32 index = dgInt32 (me->m_left.GetIndex());
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
				dgAssert (stack < DG_STACK_DEPTH);
				stackPool[stack] = me->m_left.GetNode(this);
				stack++;
			}

			if (me->m_right.IsLeaf()) {
				dgInt32 vCount = me->m_right.GetCount();
				if (vCount > 0) {
					dgInt32 index = dgInt32 (me->m_right.GetIndex());
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
				dgAssert (stack < DG_STACK_DEPTH);
				stackPool[stack] = me->m_right.GetNode(this);
				stack ++;
			}
		}
	}
}


void dgAABBPolygonSoup::ForAllSectors (const dgVector& minBox, const dgVector& maxBox, const dgVector& boxDistanceTravel, dgFloat32 m_maxT, dgAABBIntersectCallback callback, void* const context) const 
{

	dgFloat32 distance[DG_STACK_DEPTH];
	const dgNode* stackPool[DG_STACK_DEPTH];

	dgInt32 stack = 1;
	const dgInt32 stride = sizeof (dgTriplex) / sizeof (dgFloat32);
	dgVector size ((maxBox - minBox).CompProduct4(dgVector::m_half));
	dgVector origin ((maxBox + minBox).CompProduct4(dgVector::m_half));

	const dgTriplex* const vertexArray = (dgTriplex*) m_localVertex;

	if ((boxDistanceTravel % boxDistanceTravel) < dgFloat32 (1.0e-8f)) {
		stackPool[0] = m_aabb;
		distance[0] = m_aabb->BoxPenetration (vertexArray, minBox, maxBox);

		while (stack) {
			stack --;
			dgFloat32 dist =  distance[stack];

			if (dist > dgFloat32 (0.0f)) {
				const dgNode* const me = stackPool[stack];
				if (me->m_left.IsLeaf()) {
					dgInt32 vCount = me->m_left.GetCount();
					if (vCount > 0) {
						dgInt32 index = dgInt32 (me->m_left.GetIndex());
						const dgInt32* const indices = &m_indices[index];
						dgInt32 normalIndex = indices[vCount + 1] * stride;
						dgVector faceNormal (&vertexArray[normalIndex].m_x);
						if (PolygonBoxOBBTest (faceNormal, vCount, indices, stride, &vertexArray[0].m_x, origin, size)) {
							if (callback(context, &vertexArray[0].m_x, sizeof (dgTriplex), indices, vCount, dgFloat32 (0.0f)) == t_StopSearh) {
								return;
							}
						}
					}
				} else {
					dgAssert (stack < DG_STACK_DEPTH);
					stackPool[stack] = me->m_left.GetNode(this);
					stack++;
				}

				if (me->m_right.IsLeaf()) {
					dgInt32 vCount = me->m_right.GetCount();
					if (vCount > 0) {
						dgInt32 index = dgInt32 (me->m_right.GetIndex());
						const dgInt32* const indices = &m_indices[index];
						dgInt32 normalIndex = indices[vCount + 1] * stride;
						dgVector faceNormal (&vertexArray[normalIndex].m_x);
						if (PolygonBoxOBBTest (faceNormal, vCount, indices, stride, &vertexArray[0].m_x, origin, size)) {
							if (callback(context, &vertexArray[0].m_x, sizeof (dgTriplex), indices, vCount, dgFloat32 (0.0f)) == t_StopSearh) {
								return;
							}
						}
					}

				} else {
					dgAssert (stack < DG_STACK_DEPTH);
					stackPool[stack] = me->m_right.GetNode(this);
					stack ++;
				}
			}
		}
	} else {
		dgAssert(0);
/*
		dgFastRayTest ray (dgVector (dgFloat32 (0.0f)), boxDistanceTravel);

		const dgInt32 stride = sizeof (dgTriplex) / sizeof (dgFloat32);
		stackPool[0] = m_aabb;
		distance [0] = m_aabb->BoxIntersect (ray, m_aabbPoints, minBox, maxBox);

		while (stack) {

			stack --;
			const dgFloat32 dist = distance[stack];
			if (dist < dgFloat32 (1.0f)) {
				const dgNode* const me = stackPool[stack];
				if (me->m_nodeType & dgNode::m_leaf) {
					dgInt32 vCount = dgInt32 (me->m_left);
					dgAssert (vCount);
					const dgInt32* const indices = (dgInt32*) me->m_right;
					dgInt32 normalIndex = indices[vCount + 1];
					dgVector faceNormal (&vertexArray[normalIndex].m_x);
					dgFloat32 hitDistance = PolygonBoxOBBRayTest (faceNormal, vCount, indices, stride, &vertexArray[0].m_x, origin, size, boxDistanceTravel);
					if (hitDistance < dgFloat32 (1.0f)) {
						if (callback(context, &vertexArray[0].m_x, sizeof (dgTriplex), indices, vCount, dgMax (hitDistance, dist)) == t_StopSearh) {
							return;

						}
					}
				} else {
					dgAssert (stack < DG_STACK_DEPTH);
					const dgNode* const left = me->m_left;
					dgFloat32 dist = left->BoxIntersect (ray, m_aabbPoints, minBox, maxBox);
					if (dist < dgFloat32 (1.0f)) {
						dgInt32 j = stack;
						for ( ; j && (dist > distance[j - 1]); j --) {
							stackPool[j] = stackPool[j - 1];
							distance[j] = distance[j - 1];
						}
						stackPool[j] = left;
						distance[j] = dist;
						stack ++;
					}

					dgAssert (stack < DG_STACK_DEPTH);
					const dgNode* const right = me->m_right;
					dist = right->BoxIntersect (ray, m_aabbPoints, minBox, maxBox);
					if (dist < dgFloat32 (1.0f)) {
						dgInt32 j = stack;
						for ( ; j && (dist > distance[j - 1]); j --) {
							stackPool[j] = stackPool[j - 1];
							distance[j] = distance[j - 1];
						}
						stackPool[j] = right;
						distance[j] = dist;
						stack ++;
					}

				}
			}
		}
*/
	}

}


#endif


