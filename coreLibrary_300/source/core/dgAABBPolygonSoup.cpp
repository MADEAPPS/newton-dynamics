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
#include "dgStdafx.h"
#include "dgHeap.h"
#include "dgStack.h"
#include "dgList.h"
#include "dgMatrix.h"
#include "dgPolygonSoupBuilder.h"
#include "dgSimd.h"
#include "dgAABBPolygonSoup.h"


#define DG_STACK_DEPTH 63

#ifdef _WIN_32_VER
#pragma warning (disable: 4201)//nonstandard extension used : nameless struct/union
#endif


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

	class dgHeapNodePair
	{
		public:
		dgInt32 m_nodeA;
		dgInt32 m_nodeB;
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
		dgVector p0 (dgMin (node0->m_p0.m_x, node1->m_p0.m_x), dgMin (node0->m_p0.m_y, node1->m_p0.m_y), dgMin (node0->m_p0.m_z, node1->m_p0.m_z), dgFloat32 (0.0f));
		dgVector p1 (dgMax (node0->m_p1.m_x, node1->m_p1.m_x), dgMax (node0->m_p1.m_y, node1->m_p1.m_y), dgMax (node0->m_p1.m_z, node1->m_p1.m_z), dgFloat32 (0.0f));		
		dgVector side0 (p1 - p0);
		dgVector side1 (side0.m_y, side0.m_z, side0.m_x, dgFloat32 (0.0f));
		return side0 % side1;
	}

	void SetBox (dgConstructionTree* const node) const
	{
		node->m_p0.m_x = dgMin (node->m_back->m_p0.m_x, node->m_front->m_p0.m_x);
		node->m_p0.m_y = dgMin (node->m_back->m_p0.m_y, node->m_front->m_p0.m_y);
		node->m_p0.m_z = dgMin (node->m_back->m_p0.m_z, node->m_front->m_p0.m_z);
		node->m_p1.m_x = dgMax (node->m_back->m_p1.m_x, node->m_front->m_p1.m_x);
		node->m_p1.m_y = dgMax (node->m_back->m_p1.m_y, node->m_front->m_p1.m_y);
		node->m_p1.m_z = dgMax (node->m_back->m_p1.m_z, node->m_front->m_p1.m_z);
		dgVector side0 (node->m_p1 - node->m_p0);
		dgVector side1 (side0.m_y, side0.m_z, side0.m_x, dgFloat32 (0.0f));
		node->m_surfaceArea = side0 % side1;
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

					dgVector minP ( dgFloat32 (1.0e15f),  dgFloat32 (1.0e15f),  dgFloat32 (1.0e15f), dgFloat32 (0.0f)); 
					dgVector maxP (-dgFloat32 (1.0e15f), -dgFloat32 (1.0e15f), -dgFloat32 (1.0e15f), dgFloat32 (0.0f)); 

					if (boxCount == 2) {
						m_axis = 1;

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
						}

					} else {
						dgVector median (dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f));
						dgVector varian (dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f));
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
							varian += p.CompProduct (p);
						}

						varian = varian.Scale3 (dgFloat32 (boxCount)) - median.CompProduct(median);


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

	DG_INLINE dgInt32 RayTestSimd (const dgFastRayTest& ray, const dgVector& minBox, const dgVector& maxBox) const
	{
//		dgSimd minBox (&vertexArray[m_minIndex].m_x);
//		dgSimd maxBox (&vertexArray[m_maxIndex].m_x);

		dgSimd paralletTest ((((dgSimd&)ray.m_p0 < (dgSimd&)minBox) | ((dgSimd&)ray.m_p0 > (dgSimd&)maxBox)) & (dgSimd&)ray.m_isParallel);
		if (paralletTest.GetSignMask()) {
			return 0;
		}

		dgSimd tt0 (((dgSimd&)minBox - (dgSimd&)ray.m_p0) * (dgSimd&)ray.m_dpInv);
		dgSimd tt1 (((dgSimd&)maxBox - (dgSimd&)ray.m_p0) * (dgSimd&)ray.m_dpInv);
		dgSimd test (tt0 < tt1);

		dgSimd t0 (((tt0 & test) | tt1.AndNot(test)).GetMax((dgSimd&)ray.m_minT));
		t0 = t0.GetMax(t0.ShiftTripleRight());
		t0 = t0.GetMax(t0.ShiftTripleRight());

		dgSimd t1 (((tt1 & test) | tt0.AndNot(test)).GetMax((dgSimd&)ray.m_maxT));
		t1 = t1.GetMin(t1.ShiftTripleRight());
		t1 = t1.GetMin(t1.ShiftTripleRight());
		return (t0 < t1).GetInt();
	}


	DG_INLINE dgInt32 RayTest (const dgFastRayTest& ray, const dgVector& minBox, const dgVector& maxBox) const
	{
		dgFloat32 tmin = 0.0f;          
		dgFloat32 tmax = 1.0f;

		for (dgInt32 i = 0; i < 3; i++) {
			if (ray.m_isParallel[i]) {
				if (ray.m_p0[i] < minBox[i] || ray.m_p0[i] > maxBox[i]) {
					return 0;
				}
			} else {
				dgFloat32 t1 = (minBox[i] - ray.m_p0[i]) * ray.m_dpInv[i];
				dgFloat32 t2 = (maxBox[i] - ray.m_p0[i]) * ray.m_dpInv[i];

				if (t1 > t2) {
					dgSwap(t1, t2);
				}
				if (t1 > tmin) {
					tmin = t1;
				}
				if (t2 < tmax) {
					tmax = t2;
				}
				if (tmin > tmax) {
					return 0;
				}
			}
		}

		return 0xffffffff;
	}


	DG_INLINE dgInt32 BoxIntersectSimd (const dgTriplex* const vertexArray, const dgVector& boxP0, const dgVector& boxP1) const
	{
		dgSimd minBox(&vertexArray[m_minIndex].m_x);
		dgSimd maxBox(&vertexArray[m_maxIndex].m_x);
		return dgOverlapTestSimd (minBox, maxBox, boxP0, boxP1) - 1;
	}


	DG_INLINE dgInt32 BoxIntersect (const dgTriplex* const vertexArray, const dgVector& boxP0, const dgVector& boxP1) const
	{
		dgVector minBox (vertexArray[m_minIndex].m_x, vertexArray[m_minIndex].m_y, vertexArray[m_minIndex].m_z, dgFloat32 (0.0f));
		dgVector maxBox (vertexArray[m_maxIndex].m_x, vertexArray[m_maxIndex].m_y, vertexArray[m_maxIndex].m_z, dgFloat32 (0.0f));
		return dgOverlapTest (minBox, maxBox, boxP0, boxP1) - 1;
	}

	DG_INLINE dgInt32 BoxIntersectSimd (const dgFastRayTest& ray, const dgTriplex* const vertexArray, const dgVector& boxP0, const dgVector& boxP1) const
	{
		dgVector p0 (vertexArray[m_minIndex].m_x, vertexArray[m_minIndex].m_y, vertexArray[m_minIndex].m_z, dgFloat32 (0.0f));
		dgVector p1 (vertexArray[m_maxIndex].m_x, vertexArray[m_maxIndex].m_y, vertexArray[m_maxIndex].m_z, dgFloat32 (0.0f));

		dgVector minBox (p0 - boxP1);
		dgVector maxBox (p1 - boxP0);
		return RayTestSimd (ray, minBox, maxBox);;
	}


	DG_INLINE dgInt32 BoxIntersect (const dgFastRayTest& ray, const dgTriplex* const vertexArray, const dgVector& boxP0, const dgVector& boxP1) const
	{
		dgVector p0 (vertexArray[m_minIndex].m_x, vertexArray[m_minIndex].m_y, vertexArray[m_minIndex].m_z, dgFloat32 (0.0f));
		dgVector p1 (vertexArray[m_maxIndex].m_x, vertexArray[m_maxIndex].m_y, vertexArray[m_maxIndex].m_z, dgFloat32 (0.0f));

		dgVector minBox (p0 - boxP1);
		dgVector maxBox (p1 - boxP0);
		return RayTest (ray, minBox, maxBox);;
	}


	DG_INLINE dgInt32 RayTestSimd (const dgFastRayTest& ray, const dgTriplex* const vertexArray) const
	{
		dgVector minBox (&vertexArray[m_minIndex].m_x);
		dgVector maxBox (&vertexArray[m_maxIndex].m_x);
		return RayTestSimd (ray, minBox, maxBox);
	}

	DG_INLINE dgInt32 RayTest (const dgFastRayTest& ray, const dgTriplex* const vertexArray) const
	{
		dgVector minBox (&vertexArray[m_minIndex].m_x);
		dgVector maxBox (&vertexArray[m_maxIndex].m_x);
		return RayTest (ray, minBox, maxBox);
	}


	void ForAllSectorsSimd (const dgInt32* const indexArray, const dgFloat32* const vertexArray, const dgVector& minBox, const dgVector& maxBox, const dgVector& BoxDistanceTravel, dgAABBIntersectCallback callback, void* const context) const
	{
dgAssert (0);
		const dgAABBTree *stackPool[DG_STACK_DEPTH];

		dgInt32 stack = 1;
		stackPool[0] = this;
		if ((BoxDistanceTravel % BoxDistanceTravel) < dgFloat32 (1.0e-4f)) {
			while (stack) {
				stack --;
				const dgAABBTree* const me = stackPool[stack];
				if (me->BoxIntersectSimd ((dgTriplex*) vertexArray, minBox, maxBox) >= 0) {

					if (me->m_back.IsLeaf()) {
						dgInt32 index = dgInt32 (me->m_back.GetIndex());
						dgInt32 vCount = me->m_back.GetCount();
						if ((vCount > 0) && callback(context, vertexArray, sizeof (dgTriplex), &indexArray[index], vCount) == t_StopSearh) {
							return;
						}

					} else {
						dgAssert (stack < DG_STACK_DEPTH);
						stackPool[stack] = me->m_back.GetNode(this);
						stack++;
					}

					if (me->m_front.IsLeaf()) {
						dgInt32 index = dgInt32 (me->m_front.GetIndex());
						dgInt32 vCount = me->m_front.GetCount();
						if ((vCount > 0) && callback(context, vertexArray, sizeof (dgTriplex), &indexArray[index], vCount) == t_StopSearh) {
							return;
						}
					} else {
						dgAssert (stack < DG_STACK_DEPTH);
						stackPool[stack] = me->m_front.GetNode(this);
						stack ++;
					}
				}
			}
		} else {
			dgFastRayTest ray (dgVector (dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f)), BoxDistanceTravel);			

			while (stack) {
				stack --;
				const dgAABBTree* const me = stackPool[stack];
				if (me->BoxIntersectSimd (ray, (dgTriplex*) vertexArray, minBox, maxBox)) {

					if (me->m_back.IsLeaf()) {
						dgInt32 index = dgInt32 (me->m_back.GetIndex());
						dgInt32 vCount = me->m_back.GetCount();
						if ((vCount > 0) && callback(context, vertexArray, sizeof (dgTriplex), &indexArray[index], vCount) == t_StopSearh) {
							return;
						}

					} else {
						dgAssert (stack < DG_STACK_DEPTH);
						stackPool[stack] = me->m_back.GetNode(this);
						stack++;
					}

					if (me->m_front.IsLeaf()) {
						dgInt32 index = dgInt32 (me->m_front.GetIndex());
						dgInt32 vCount = me->m_front.GetCount();
						if ((vCount > 0) && callback(context, vertexArray, sizeof (dgTriplex), &indexArray[index], vCount) == t_StopSearh) {
							return;
						}
					} else {
						dgAssert (stack < DG_STACK_DEPTH);
						stackPool[stack] = me->m_front.GetNode(this);
						stack ++;
					}
				}
			}
		}
	}


	void ForAllSectors (const dgInt32* const indexArray, const dgFloat32* const vertexArray, const dgVector& minBox, const dgVector& maxBox, const dgVector& boxDistanceTravel, dgAABBIntersectCallback callback, void* const context) const
	{
		const dgAABBTree *stackPool[DG_STACK_DEPTH];

		dgInt32 stack = 1;
		stackPool[0] = this;
		if ((boxDistanceTravel % boxDistanceTravel) < dgFloat32 (1.0e-8f)) {
			while (stack) {
				stack --;
				const dgAABBTree* const me = stackPool[stack];
				if (me->BoxIntersect ((dgTriplex*) vertexArray, minBox, maxBox) >= 0) {

					if (me->m_back.IsLeaf()) {
						dgInt32 index = dgInt32 (me->m_back.GetIndex());
						dgInt32 vCount = me->m_back.GetCount();
						if ((vCount > 0) && callback(context, vertexArray, sizeof (dgTriplex), &indexArray[index], vCount) == t_StopSearh) {
							return;
						}

					} else {
						dgAssert (stack < DG_STACK_DEPTH);
						stackPool[stack] = me->m_back.GetNode(this);
						stack++;
					}

					if (me->m_front.IsLeaf()) {
						dgInt32 index = dgInt32 (me->m_front.GetIndex());
						dgInt32 vCount = me->m_front.GetCount();
						if ((vCount > 0) && callback(context, vertexArray, sizeof (dgTriplex), &indexArray[index], vCount) == t_StopSearh) {
							return;
						}

					} else {
						dgAssert (stack < DG_STACK_DEPTH);
						stackPool[stack] = me->m_front.GetNode(this);
						stack ++;
					}
				}
			}
		} else {
			dgFastRayTest ray (dgVector (dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f)), boxDistanceTravel);

			while (stack) {
				stack --;
				const dgAABBTree* const me = stackPool[stack];
				if (me->BoxIntersect (ray, (dgTriplex*) vertexArray, minBox, maxBox)) {

					if (me->m_back.IsLeaf()) {
						dgInt32 index = dgInt32 (me->m_back.GetIndex());
						dgInt32 vCount = me->m_back.GetCount();
						if ((vCount > 0) && callback(context, vertexArray, sizeof (dgTriplex), &indexArray[index], vCount) == t_StopSearh) {
							return;
						}

					} else {
						dgAssert (stack < DG_STACK_DEPTH);
						stackPool[stack] = me->m_back.GetNode(this);
						stack++;
					}

					if (me->m_front.IsLeaf()) {
						dgInt32 index = dgInt32 (me->m_front.GetIndex());
						dgInt32 vCount = me->m_front.GetCount();
						if ((vCount > 0) && callback(context, vertexArray, sizeof (dgTriplex), &indexArray[index], vCount) == t_StopSearh) {
							return;
						}

					} else {
						dgAssert (stack < DG_STACK_DEPTH);
						stackPool[stack] = me->m_front.GetNode(this);
						stack ++;
					}
				}
			}
		}
	}


	void ForAllSectorsRayHitSimd (const dgFastRayTest& raySrc, const dgInt32* const indexArray, const dgFloat32* const vertexArray, dgRayIntersectCallback callback, void* const context) const
	{
		const dgAABBTree *stackPool[DG_STACK_DEPTH];

		dgFastRayTest ray (raySrc);
		dgInt32 stack = 1;
		dgFloat32 maxParam = dgFloat32 (1.0f);

		stackPool[0] = this;
		while (stack) {
			stack --;

			const dgAABBTree* const me = stackPool[stack];
			if (me->RayTestSimd (ray, (dgTriplex*) vertexArray)) {
				
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
		dgVector supportVertex (dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f));

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
		dgInt32 axis;
		dgFloat32 maxVal;
		dgVector median (dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f));
		dgVector varian (dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f));
		for (dgInt32 i = 0; i < boxCount; i ++) {

			median += boxArray[i]->m_p0;
			median += boxArray[i]->m_p1;

			varian += boxArray[i]->m_p0.CompProduct(boxArray[i]->m_p0);
			varian += boxArray[i]->m_p1.CompProduct(boxArray[i]->m_p1);
		}

		boxCount *= 2;
		varian.m_x = boxCount * varian.m_x - median.m_x * median.m_x;
		varian.m_y = boxCount * varian.m_y - median.m_y * median.m_y;
		varian.m_z = boxCount * varian.m_z - median.m_z * median.m_z;

		axis = 0;
		maxVal = varian[0];
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
		dgInt32 axis;

		axis = *((dgInt32*) context);

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


void dgAABBPolygonSoup::ForAllSectorsSimd (const dgVector& minBox, const dgVector& maxBox, const dgVector& boxDistanceTravel, dgAABBIntersectCallback callback, void* const context) const
{
	if (m_aabb) {
		dgAABBTree* const tree = (dgAABBTree*) m_aabb;
		tree->ForAllSectorsSimd (m_indices, m_localVertex, minBox, maxBox, boxDistanceTravel, callback, context);
	}
}


void dgAABBPolygonSoup::ForAllSectors (const dgVector& minBox, const dgVector& maxBox, const dgVector& boxDistanceTravel, dgAABBIntersectCallback callback, void* const context) const
{
	if (m_aabb) {
		dgAABBTree* const tree = (dgAABBTree*) m_aabb;
		tree->ForAllSectors (m_indices, m_localVertex, minBox, maxBox, boxDistanceTravel, callback, context);
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

void dgAABBPolygonSoup::ForAllSectorsRayHitSimd (const dgFastRayTest& ray, dgRayIntersectCallback callback, void* const context) const
{
	if (m_aabb) {
		dgAABBTree* const tree = (dgAABBTree*) m_aabb;
		tree->ForAllSectorsRayHitSimd (ray, m_indices, m_localVertex, callback, context);
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


dgIntersectStatus dgAABBPolygonSoup::CalculateDisjointedFaceEdgeNormals (void* const context, const dgFloat32* const polygon, dgInt32 strideInBytes, const dgInt32* const indexArray, dgInt32 indexCount)
{
	#define DG_WELDING_TOL (1.0e-2f)
	#define DG_WELDING_TOL2 (DG_WELDING_TOL * DG_WELDING_TOL)

	AdjacentdFaces& adjacentFaces = *((AdjacentdFaces*)context);


//if ((adjacentFaces.m_index[0] == 29404) && (adjacentFaces.m_index[1] == 23332) && (adjacentFaces.m_index[2] == 18503) && (adjacentFaces.m_index[3] == 1560))
//{
//	if ((indexArray[0] == 29636) && (indexArray[1] == 31197) && (indexArray[2] == 0) && (indexArray[3] == 1175))
//		indexCount *=1;
//	if ((indexArray[0] == 1560) && (indexArray[1] == 2585) && (indexArray[2] == 28583) && (indexArray[3] == 29404))
//		indexCount *=1;
//}


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



dgIntersectStatus dgAABBPolygonSoup::CalculateAllFaceEdgeNormals (void* const context, const dgFloat32* const polygon, dgInt32 strideInBytes, const dgInt32* const indexArray, dgInt32 indexCount)
{
	dgAABBPolygonSoup* const me = (dgAABBPolygonSoup*) context;

	dgInt32 stride = dgInt32 (strideInBytes / sizeof (dgFloat32));

	AdjacentdFaces adjacentFaces;
	adjacentFaces.m_count = indexCount;
	adjacentFaces.m_index = (dgInt32*) indexArray;


//if ((indexArray[0] == 1560) && (indexArray[1] == 2585) && (indexArray[2] == 28583))
//if ((indexArray[0] == 1560) && (indexArray[1] == 2585) && (indexArray[2] == 28583) && (indexArray[3] == 29404))
//stride *=1;


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
	me->ForAllSectors (p0, p1, dgVector (dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f)), CalculateDisjointedFaceEdgeNormals, &adjacentFaces);

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
	ForAllSectors (p0, p1, dgVector (dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f)), CalculateAllFaceEdgeNormals, this);

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

void dgAABBPolygonSoup::Create (const dgPolygonSoupDatabaseBuilder& builder, bool optimizedBuild)
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

