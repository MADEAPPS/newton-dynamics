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

#include "ntStdafx.h"
#include "ntWorld.h"
#include "ntBody.h"
#include "ntBroadPhase.h"
#include "ntDynamicBody.h"

D_MSC_VECTOR_ALIGNMENT
class ntBroadPhase::ntSpliteInfo
{
	public:
	ntSpliteInfo(ntBroadPhaseNode** const boxArray, dInt32 boxCount)
	{
		dVector minP(dFloat32(1.0e15f));
		dVector maxP(-dFloat32(1.0e15f));

		if (boxCount == 2)
		{
			m_axis = 1;
			for (dInt32 i = 0; i < boxCount; i++)
			{
				ntBroadPhaseNode* const node = boxArray[i];
				dAssert(node->GetAsBroadPhaseBodyNode());
				minP = minP.GetMin(node->m_minBox);
				maxP = maxP.GetMax(node->m_maxBox);
			}
		}
		else
		{
			dVector median(dVector::m_zero);
			dVector varian(dVector::m_zero);
			for (dInt32 i = 0; i < boxCount; i++)
			{
				ntBroadPhaseNode* const node = boxArray[i];
				dAssert(node->GetAsBroadPhaseBodyNode());
				minP = minP.GetMin(node->m_minBox);
				maxP = maxP.GetMax(node->m_maxBox);
				dVector p(dVector::m_half * (node->m_minBox + node->m_maxBox));
				median += p;
				varian += p * p;
			}

			varian = varian.Scale(dFloat32(boxCount)) - median * median;

			dInt32 index = 0;
			dFloat32 maxVarian = dFloat32(-1.0e15f);
			for (dInt32 i = 0; i < 3; i++)
			{
				if (varian[i] > maxVarian)
				{
					index = i;
					maxVarian = varian[i];
				}
			}

			dVector center = median.Scale(dFloat32(1.0f) / dFloat32(boxCount));

			dFloat32 test = center[index];

			dInt32 i0 = 0;
			dInt32 i1 = boxCount - 1;
			do
			{
				for (; i0 <= i1; i0++)
				{
					ntBroadPhaseNode* const node = boxArray[i0];
					dFloat32 val = (node->m_minBox[index] + node->m_maxBox[index]) * dFloat32(0.5f);
					if (val > test)
					{
						break;
					}
				}

				for (; i1 >= i0; i1--)
				{
					ntBroadPhaseNode* const node = boxArray[i1];
					dFloat32 val = (node->m_minBox[index] + node->m_maxBox[index]) * dFloat32(0.5f);
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
				i0--;
			}
			if ((i0 + 1) >= boxCount)
			{
				i0 = boxCount - 2;
			}
			m_axis = i0 + 1;
		}

		dAssert(maxP.m_x - minP.m_x >= dFloat32(0.0f));
		dAssert(maxP.m_y - minP.m_y >= dFloat32(0.0f));
		dAssert(maxP.m_z - minP.m_z >= dFloat32(0.0f));
		m_p0 = minP;
		m_p1 = maxP;
	}

	dVector m_p0;
	dVector m_p1;
	dInt32 m_axis;
} D_GCC_VECTOR_ALIGNMENT;

ntBroadPhase::ntFitnessList::ntFitnessList()
	:dList <ntBroadPhaseTreeNode*>()
	,m_prevCost(dFloat32(0.0f))
	,m_index(0)
{
}

dFloat64 ntBroadPhase::ntFitnessList::TotalCost() const
{
	dFloat64 cost = dFloat32(0.0f);
	for (dListNode* node = GetFirst(); node; node = node->GetNext()) {
		ntBroadPhaseNode* const box = node->GetInfo();
		cost += box->m_surfaceArea;
	}
	return cost;
}
	
ntBroadPhase::ntBroadPhase(ntWorld* const world)
	:dClassAlloc()
	,m_newton(world)
	,m_rootNode(nullptr)
{
}

ntBroadPhase::~ntBroadPhase()
{
}

ntBroadPhaseTreeNode* ntBroadPhase::InsertNode(ntBroadPhaseNode* const root, ntBroadPhaseNode* const node)
{
	dVector p0;
	dVector p1;

	ntBroadPhaseNode* sibling = root;
	dFloat32 surfaceArea = CalculateSurfaceArea(node, sibling, p0, p1);
	while (!sibling->GetAsBroadPhaseBodyNode() && (surfaceArea >= sibling->m_surfaceArea))
	{
		sibling->m_minBox = p0;
		sibling->m_maxBox = p1;
		sibling->m_surfaceArea = surfaceArea;
	
		dVector leftP0;
		dVector leftP1;
		dFloat32 leftSurfaceArea = CalculateSurfaceArea(node, sibling->GetLeft(), leftP0, leftP1);
		
		dVector rightP0;
		dVector rightP1;
		dFloat32 rightSurfaceArea = CalculateSurfaceArea(node, sibling->GetRight(), rightP0, rightP1);
	
		if (leftSurfaceArea < rightSurfaceArea) 
		{
			p0 = leftP0;
			p1 = leftP1;
			sibling = sibling->GetLeft();
			surfaceArea = leftSurfaceArea;
		}
		else 
		{
			p0 = rightP0;
			p1 = rightP1;
			sibling = sibling->GetRight();
			surfaceArea = rightSurfaceArea;
		}
	}
	
	ntBroadPhaseTreeNode* const parent = new ntBroadPhaseTreeNode(sibling, node);
	return parent;
}

void ntBroadPhase::Update(dFloat32 timestep)
{
	D_TRACKTIME();

	UpdateAabb(timestep);
	BalanceBroadPhase();
}

void ntBroadPhase::UpdateAabb(dFloat32 timestep)
{
	class dUpdateAabbJob: public ntWorld::dNewtonBaseJob
	{
		public:
		virtual void Execute()
		{
			D_TRACKTIME();
			const dInt32 threadIndex = GetThredID();
			const dInt32 threadsCount = m_newton->GetThreadCount();
			const dInt32 count = m_newton->m_dynamicBodyArray.GetCount();
			ntDynamicBody** const bodies = &m_newton->m_dynamicBodyArray[0];
			ntBroadPhase* const broadPhase = m_newton->m_broadPhase;
			for (dInt32 i = m_it->fetch_add(1); i < count; i = m_it->fetch_add(1))
			{
				broadPhase->UpdateAabb(threadIndex, m_timestep, bodies[i]);
			}
		}
	};
	m_newton->SubmitJobs<dUpdateAabbJob>(timestep);
}


void ntBroadPhase::RotateLeft(ntBroadPhaseTreeNode* const node, ntBroadPhaseNode** const root)
{
	dVector cost1P0;
	dVector cost1P1;

	ntBroadPhaseTreeNode* const parent = (ntBroadPhaseTreeNode*)node->m_parent;
	dAssert(parent && !parent->GetAsBroadPhaseBodyNode());
	dFloat32 cost1 = CalculateSurfaceArea(node->m_left, parent->m_left, cost1P0, cost1P1);

	dVector cost2P0;
	dVector cost2P1;
	dFloat32 cost2 = CalculateSurfaceArea(node->m_right, parent->m_left, cost2P0, cost2P1);

	dFloat32 cost0 = node->m_surfaceArea;
	if ((cost1 <= cost0) && (cost1 <= cost2)) 
	{
		node->m_minBox = parent->m_minBox;
		node->m_maxBox = parent->m_maxBox;
		node->m_surfaceArea = parent->m_surfaceArea;

		ntBroadPhaseTreeNode* const grandParent = (ntBroadPhaseTreeNode*)parent->m_parent;
		if (grandParent) {
			if (grandParent->m_left == parent) 
			{
				grandParent->m_left = node;
			}
			else 
			{
				dAssert(grandParent->m_right == parent);
				grandParent->m_right = node;
			}
		}
		else 
		{
			(*root) = node;
		}

		node->m_parent = parent->m_parent;
		parent->m_parent = node;
		node->m_left->m_parent = parent;
		parent->m_right = node->m_left;
		node->m_left = parent;

		parent->m_minBox = cost1P0;
		parent->m_maxBox = cost1P1;
		parent->m_surfaceArea = cost1;
	}
	else if ((cost2 <= cost0) && (cost2 <= cost1)) 
	{
		node->m_minBox = parent->m_minBox;
		node->m_maxBox = parent->m_maxBox;
		node->m_surfaceArea = parent->m_surfaceArea;

		ntBroadPhaseTreeNode* const grandParent = (ntBroadPhaseTreeNode*)parent->m_parent;
		if (grandParent) 
		{
			if (grandParent->m_left == parent) 
			{
				grandParent->m_left = node;
			}
			else 
			{
				dAssert(grandParent->m_right == parent);
				grandParent->m_right = node;
			}
		}
		else 
		{
			(*root) = node;
		}

		node->m_parent = parent->m_parent;
		parent->m_parent = node;
		node->m_right->m_parent = parent;
		parent->m_right = node->m_right;
		node->m_right = parent;

		parent->m_minBox = cost2P0;
		parent->m_maxBox = cost2P1;
		parent->m_surfaceArea = cost2;
	}
}

void ntBroadPhase::RotateRight(ntBroadPhaseTreeNode* const node, ntBroadPhaseNode** const root)
{
	dVector cost1P0;
	dVector cost1P1;

	ntBroadPhaseTreeNode* const parent = (ntBroadPhaseTreeNode*)node->m_parent;
	dAssert(parent && !parent->GetAsBroadPhaseBodyNode());

	dFloat32 cost1 = CalculateSurfaceArea(node->m_right, parent->m_right, cost1P0, cost1P1);

	dVector cost2P0;
	dVector cost2P1;
	dFloat32 cost2 = CalculateSurfaceArea(node->m_left, parent->m_right, cost2P0, cost2P1);

	dFloat32 cost0 = node->m_surfaceArea;
	if ((cost1 <= cost0) && (cost1 <= cost2)) 
	{
		node->m_minBox = parent->m_minBox;
		node->m_maxBox = parent->m_maxBox;
		node->m_surfaceArea = parent->m_surfaceArea;

		ntBroadPhaseTreeNode* const grandParent = (ntBroadPhaseTreeNode*)parent->m_parent;
		if (grandParent) 
		{
			dAssert(!grandParent->GetAsBroadPhaseBodyNode());
			if (grandParent->m_left == parent) 
			{
				grandParent->m_left = node;
			}
			else 
			{
				dAssert(grandParent->m_right == parent);
				grandParent->m_right = node;
			}
		}
		else 
		{
			(*root) = node;
		}

		node->m_parent = parent->m_parent;
		parent->m_parent = node;
		node->m_right->m_parent = parent;
		parent->m_left = node->m_right;
		node->m_right = parent;
		parent->m_minBox = cost1P0;
		parent->m_maxBox = cost1P1;
		parent->m_surfaceArea = cost1;

	}
	else if ((cost2 <= cost0) && (cost2 <= cost1)) 
	{
		node->m_minBox = parent->m_minBox;
		node->m_maxBox = parent->m_maxBox;
		node->m_surfaceArea = parent->m_surfaceArea;

		ntBroadPhaseTreeNode* const grandParent = (ntBroadPhaseTreeNode*)parent->m_parent;
		if (parent->m_parent) 
		{
			if (grandParent->m_left == parent) 
			{
				grandParent->m_left = node;
			}
			else 
			{
				dAssert(grandParent->m_right == parent);
				grandParent->m_right = node;
			}
		}
		else 
		{
			(*root) = node;
		}

		node->m_parent = parent->m_parent;
		parent->m_parent = node;
		node->m_left->m_parent = parent;
		parent->m_left = node->m_left;
		node->m_left = parent;

		parent->m_minBox = cost2P0;
		parent->m_maxBox = cost2P1;
		parent->m_surfaceArea = cost2;
	}
}

void ntBroadPhase::ImproveNodeFitness(ntBroadPhaseTreeNode* const node, ntBroadPhaseNode** const root)
{
	dAssert(node->GetLeft());
	dAssert(node->GetRight());

	ntBroadPhaseNode* const parent = node->m_parent;
	if (parent && parent->m_parent) 
	{
		dAssert(!parent->GetAsBroadPhaseBodyNode());
		if (parent->GetLeft() == node) 
		{
			RotateRight(node, root);
		}
		else 
		{
			RotateLeft(node, root);
		}
	}
	dAssert(!m_rootNode->m_parent);
}

dFloat64 ntBroadPhase::ReduceEntropy(ntFitnessList& fitness, ntBroadPhaseNode** const root)
{
	dFloat64 cost = dFloat32(0.0f);
	if (fitness.GetCount() < 32) 
	{
		for (ntFitnessList::dListNode* node = fitness.GetFirst(); node; node = node->GetNext()) 
		{
			ImproveNodeFitness(node->GetInfo(), root);
		}
		cost = fitness.TotalCost();
		fitness.m_prevCost = cost;
	}
	else 
	{
		const dInt32 mod = 16;
		cost = fitness.m_prevCost;
		ntFitnessList::dListNode* node = fitness.GetFirst();
		for (dInt32 i = 0; i < fitness.m_index; i++) 
		{
			node = node->GetNext();
		}

		do 
		{
			ImproveNodeFitness(node->GetInfo(), root);
			for (dInt32 i = 0; i < mod; i++) 
			{
				node = node ? node->GetNext() : nullptr;
			}
		} while (node);

		if (!fitness.m_index) 
		{
			cost = fitness.TotalCost();
			fitness.m_prevCost = cost;
		}
		fitness.m_index = (fitness.m_index + 1) % mod;
	}
	return cost;
}

dInt32 ntBroadPhase::CompareNodes(const ntBroadPhaseNode* const nodeA, const ntBroadPhaseNode* const nodeB, void* const)
{
	dFloat32 areaA = nodeA->m_surfaceArea;
	dFloat32 areaB = nodeB->m_surfaceArea;
	if (areaA < areaB) 
	{
		return 1;
	}
	if (areaA > areaB) 
	{
		return -1;
	}
	return 0;
}

void ntBroadPhase::UpdateFitness(ntFitnessList& fitness, dFloat64& oldEntropy, ntBroadPhaseNode** const root)
{
	if (*root) 
	{
		D_TRACKTIME();
		ntBroadPhaseNode* const parent = (*root)->m_parent;

		(*root)->m_parent = nullptr;
		dFloat64 entropy = ReduceEntropy(fitness, root);

		if ((entropy > (oldEntropy * dFloat32(1.5f))) || (entropy < (oldEntropy * dFloat32(0.75f)))) 
		{
			if (fitness.GetFirst()) 
			{
				//m_world->m_solverJacobiansMemory.ResizeIfNecessary((fitness.GetCount() * 2 + 16) * sizeof(dBroadPhaseNode*));
				//dBroadPhaseNode** const leafArray = (dBroadPhaseNode**)&m_world->m_solverJacobiansMemory[0];
				ntBroadPhaseNode** const leafArray = dAlloca(ntBroadPhaseNode*, fitness.GetCount() * 2 + 16);

				dInt32 leafNodesCount = 0;
				for (ntFitnessList::dListNode* nodePtr = fitness.GetFirst(); nodePtr; nodePtr = nodePtr->GetNext()) 
				{
					ntBroadPhaseNode* const node = nodePtr->GetInfo();
					ntBroadPhaseNode* const leftNode = node->GetLeft();

					ntBody* const leftBody = leftNode->GetBody();
					if (leftBody) 
					{
						node->SetAABB(leftBody->m_minAABB, leftBody->m_maxAABB);
						leafArray[leafNodesCount] = leftNode;
						leafNodesCount++;
					}
					else if (leftNode->GetAsBroadPhaseAggregate()) 
					{
						dAssert(0);
						leafArray[leafNodesCount] = leftNode;
						leafNodesCount++;
					}

					ntBroadPhaseNode* const rightNode = node->GetRight();
					ntBody* const rightBody = rightNode->GetBody();
					if (rightBody) 
					{
						rightNode->SetAABB(rightBody->m_minAABB, rightBody->m_maxAABB);
						leafArray[leafNodesCount] = rightNode;
						leafNodesCount++;
					}
					else if (rightNode->GetAsBroadPhaseAggregate()) 
					{
						dAssert(0);
						leafArray[leafNodesCount] = rightNode;
						leafNodesCount++;
					}
				}
				
				ntFitnessList::dListNode* nodePtr = fitness.GetFirst();
				dSortIndirect(leafArray, leafNodesCount, CompareNodes);
				
				*root = BuildTopDownBig(leafArray, 0, leafNodesCount - 1, &nodePtr);
				dAssert(!(*root)->m_parent);
				entropy = fitness.TotalCost();
				fitness.m_prevCost = entropy;
			}
			oldEntropy = entropy;
		}
		(*root)->m_parent = parent;
	}
}

ntBroadPhaseNode* ntBroadPhase::BuildTopDown(ntBroadPhaseNode** const leafArray, dInt32 firstBox, dInt32 lastBox, ntFitnessList::dListNode** const nextNode)
{
	dAssert(firstBox >= 0);
	dAssert(lastBox >= 0);

	if (lastBox == firstBox) 
	{
		return leafArray[firstBox];
	}
	else 
	{
		ntSpliteInfo info(&leafArray[firstBox], lastBox - firstBox + 1);

		ntBroadPhaseTreeNode* const parent = (*nextNode)->GetInfo();
		parent->m_parent = nullptr;
		*nextNode = (*nextNode)->GetNext();

		parent->SetAABB(info.m_p0, info.m_p1);

		parent->m_left = BuildTopDown(leafArray, firstBox, firstBox + info.m_axis - 1, nextNode);
		parent->m_left->m_parent = parent;

		parent->m_right = BuildTopDown(leafArray, firstBox + info.m_axis, lastBox, nextNode);
		parent->m_right->m_parent = parent;
		return parent;
	}
}

ntBroadPhaseNode* ntBroadPhase::BuildTopDownBig(ntBroadPhaseNode** const leafArray, dInt32 firstBox, dInt32 lastBox, ntFitnessList::dListNode** const nextNode)
{
	if (lastBox == firstBox) 
	{
		return BuildTopDown(leafArray, firstBox, lastBox, nextNode);
	}

	dInt32 midPoint = -1;
	const dFloat32 scale = dFloat32(1.0f / 64.0f);
	const ntBroadPhaseNode* const node0 = leafArray[firstBox];
	const dInt32 count = lastBox - firstBox;
	dFloat32 area0 = scale * node0->m_surfaceArea;
	for (dInt32 i = 1; i <= count; i++) 
	{
		const ntBroadPhaseNode* const node1 = leafArray[firstBox + i];
		dFloat32 area1 = node1->m_surfaceArea;
		if (area0 > area1) 
		{
			midPoint = i - 1;
			break;
		}
	}

	if (midPoint == -1) 
	{
		return BuildTopDown(leafArray, firstBox, lastBox, nextNode);
	}
	else 
	{
		ntBroadPhaseTreeNode* const parent = (*nextNode)->GetInfo();

		parent->m_parent = nullptr;
		*nextNode = (*nextNode)->GetNext();

		parent->m_right = BuildTopDown(leafArray, firstBox, firstBox + midPoint, nextNode);
		parent->m_right->m_parent = parent;

		parent->m_left = BuildTopDownBig(leafArray, firstBox + midPoint + 1, lastBox, nextNode);
		parent->m_left->m_parent = parent;

		dVector minP(parent->m_left->m_minBox.GetMin(parent->m_right->m_minBox));
		dVector maxP(parent->m_left->m_maxBox.GetMax(parent->m_right->m_maxBox));
		parent->SetAABB(minP, maxP);

		return parent;
	}
}

void ntBroadPhase::UpdateAabb(dInt32 threadIndex, dFloat32 timestep, ntBody* const body)
{
	if (!body->m_equilibrium)
	{
		ntBroadPhaseBodyNode* const node = body->GetBroadPhaseNode();
		body->UpdateCollisionMatrix();
		
		dAssert(!node->GetLeft());
		dAssert(!node->GetRight());
		dAssert(!((ntShape*)body->GetCollisionShape().GetShape())->GetAsShapeNull());

		if (body->GetBroadPhaseAggregate()) 
		{
			dAssert(0);
			//dBroadPhaseAggregate* const aggregate = body1->GetBroadPhaseAggregate();
			//dgScopeSpinPause lock(&aggregate->m_criticalSectionLock);
			//aggregate->m_isInEquilibrium = body1->m_equilibrium;
		}

		dAssert(0);
//		if (!dgBoxInclusionTest(body1->m_minAABB, body1->m_maxAABB, node->m_minBox, node->m_maxBox)) 
		if (1)
		{
			dAssert(!node->GetAsBroadPhaseAggregate());
			node->SetAABB(body->m_minAABB, body->m_maxAABB);

			if (!m_rootNode->GetAsBroadPhaseBodyNode()) 
			{
				const ntBroadPhaseNode* const root = (m_rootNode->GetLeft() && m_rootNode->GetRight()) ? NULL : m_rootNode;
				for (ntBroadPhaseNode* parent = node->m_parent; parent != root; parent = parent->m_parent) 
				{
					dAssert(0);
					//dgScopeSpinPause lock(&parent->m_criticalSectionLock);
					if (!parent->GetAsBroadPhaseAggregate()) 
					{
						dVector minBox;
						dVector maxBox;
						dFloat32 area = CalculateSurfaceArea(parent->GetLeft(), parent->GetRight(), minBox, maxBox);
						dAssert(0);
						//if (dgBoxInclusionTest(minBox, maxBox, parent->m_minBox, parent->m_maxBox)) 
						//{
						//	break;
						//}
						parent->m_minBox = minBox;
						parent->m_maxBox = maxBox;
						parent->m_surfaceArea = area;
					}
					else 
					{
						dAssert(0);
						//dBroadPhaseAggregate* const aggregate = parent->GetAsBroadPhaseAggregate();
						//aggregate->m_minBox = aggregate->m_root->m_minBox;
						//aggregate->m_maxBox = aggregate->m_root->m_maxBox;
						//aggregate->m_surfaceArea = aggregate->m_root->m_surfaceArea;
					}
				}
			}
		}
	}
}
