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

#include "dgPhysicsStdafx.h"
#include "dgBody.h"
#include "dgWorld.h"
#include "dgContact.h"
#include "dgBroadPhase.h"
#include "dgDynamicBody.h"
#include "dgCollisionConvex.h"
#include "dgCollisionInstance.h"
#include "dgDeformableContact.h"
#include "dgWorldDynamicUpdate.h"
#include "dgCollisionDeformableMesh.h"

#define DG_CONVEX_CAST_POOLSIZE			32
#define DG_BROADPHASE_MAX_STACK_DEPTH	256
#define DG_BROADPHASE_AABB_SCALE		dgFloat32 (8.0f)
#define DG_BROADPHASE_AABB_INV_SCALE	(dgFloat32 (1.0f) / DG_BROADPHASE_AABB_SCALE)

dgVector dgBroadPhase::m_conservativeRotAngle (45.0f * 3.14159f / 180.0f);


DG_MSC_VECTOR_ALIGMENT
class dgBroadPhase::dgNode
{
	public: 
	DG_CLASS_ALLOCATOR(allocator)
	
	dgNode (dgBody* const body)
		:m_minBox (body->m_minAABB)
		,m_maxBox (body->m_maxAABB)
		,m_body(body)
		,m_left(NULL)
		,m_right(NULL)
		,m_parent(NULL)
		,m_fitnessNode(NULL) 
	{
		SetAABB(body->m_minAABB, body->m_maxAABB);
		m_body->m_collisionCell = this;
	}

	dgNode (dgNode* const sibling, dgNode* const myNode)
		:m_body(NULL)
		,m_left(sibling)
		,m_right(myNode)
		,m_parent(sibling->m_parent)
		,m_fitnessNode(NULL)  
	{
		if (m_parent) {
			if (m_parent->m_left == sibling) {
				m_parent->m_left = this;
			} else {
				dgAssert (m_parent->m_right == sibling);
				m_parent->m_right = this;
			}
		}
		sibling->m_parent = this;
		myNode->m_parent = this;

		dgNode* const left = m_left;
		dgNode* const right = m_right;

		m_minBox = left->m_minBox.GetMin(right->m_minBox);
		m_maxBox = left->m_maxBox.GetMax(right->m_maxBox);
		dgVector side0 (m_maxBox - m_minBox);
		m_surfaceArea = side0.DotProduct4(side0.ShiftTripleRight()).m_x;
	}

	dgNode (dgNode* const parent, const dgVector& minBox, const dgVector& maxBox)
		:m_minBox (minBox)
		,m_maxBox(maxBox)
		,m_body(NULL)
		,m_left(NULL)
		,m_right(NULL)
		,m_parent(parent)
		,m_fitnessNode(NULL) 
	{
	}


	~dgNode ()
	{
		if (m_body) {
			dgAssert (!m_left);
			dgAssert (!m_right);
			dgAssert (m_body->m_collisionCell == this);
			m_body->m_collisionCell = NULL;
		} else {
			if (m_left) {
				delete m_left;
			}
			if (m_right) {
				delete m_right;
			}
		}
	}

	void SetAABB (const dgVector& minBox, const dgVector& maxBox)
	{
		dgAssert (minBox.m_x <= maxBox.m_x);
		dgAssert (minBox.m_y <= maxBox.m_y);
		dgAssert (minBox.m_z <= maxBox.m_z);

		dgVector p0 (minBox.CompProduct4(m_broadPhaseScale));
		dgVector p1 (maxBox.CompProduct4(m_broadPhaseScale) + dgVector::m_one);

		m_minBox = p0.Floor().CompProduct4(m_broadInvPhaseScale);
		m_maxBox = p1.Floor().CompProduct4(m_broadInvPhaseScale);

		dgAssert (m_minBox.m_w == dgFloat32 (0.0f));
		dgAssert (m_maxBox.m_w == dgFloat32 (0.0f));

		dgVector side0 (m_maxBox - m_minBox);
		m_surfaceArea = side0.DotProduct4(side0.ShiftTripleRight()).m_x;
	}


	dgVector m_minBox;
	dgVector m_maxBox;
	dgFloat32 m_surfaceArea;
	dgBody* m_body;	
	dgNode* m_left;
	dgNode* m_right;
	dgNode* m_parent;
	dgList<dgNode*>::dgListNode* m_fitnessNode;
	static dgVector m_broadPhaseScale;
	static dgVector m_broadInvPhaseScale;

	friend class dgBody;
	friend class dgBroadPhase;
	friend class dgBroadphaseSyncDescriptor;
	friend class dgFitnessList;
} DG_GCC_VECTOR_ALIGMENT;

dgVector dgBroadPhase::dgNode::m_broadPhaseScale (DG_BROADPHASE_AABB_SCALE, DG_BROADPHASE_AABB_SCALE, DG_BROADPHASE_AABB_SCALE, dgFloat32 (0.0f));
dgVector dgBroadPhase::dgNode::m_broadInvPhaseScale (DG_BROADPHASE_AABB_INV_SCALE, DG_BROADPHASE_AABB_INV_SCALE, DG_BROADPHASE_AABB_INV_SCALE, dgFloat32 (0.0f));



class dgBroadphaseSyncDescriptor
{
	public:
		dgBroadphaseSyncDescriptor(dgBroadPhase::dgType broadPhaseType, dgThread::dgCriticalSection* const lock)
		:m_lock(lock)
		,m_pairsCount(0)
		,m_pairsAtomicCounter(0)
		,m_jointsAtomicCounter(0)
		,m_timestep(dgFloat32 (0.0f))
		,m_collindPairBodyNode(NULL)
		,m_forceAndTorqueBodyNode(NULL)
		,m_newBodiesNodes(NULL)
		,m_broadPhaseType(broadPhaseType)
	{
	}

	void CreatePairsJobs (dgBroadPhase::dgNode* const rootNode)
	{
		dgBroadPhase::dgNode* pool[ DG_BROADPHASE_MAX_STACK_DEPTH];		
		
		pool[0] = rootNode;
		pool[1] = rootNode;

		dgInt32 stack = 2; 
		dgInt32 stackDpeth = 4 * 2;
		m_pairsCount = 0;
		while (stack && (m_pairsCount < dgInt32 (sizeof (m_pairs) / (2 * sizeof (m_pairs[0]))))) {

			while (stack >= stackDpeth) {
				stack -= 2;
				m_pairs[m_pairsCount * 2] = pool[stack];
				m_pairs[m_pairsCount * 2 + 1] = pool[stack + 1];
				m_pairsCount ++;
				dgAssert (m_pairsCount < (sizeof (m_pairs) / (2 * sizeof (m_pairs[0]))));
			}

			stack -= 2;
			dgBroadPhase::dgNode* const left = pool[stack];
			dgBroadPhase::dgNode* const right = pool[stack + 1];

			if (left == right) {
				if (!left->m_body) {
					pool[stack] = left->m_left;	
					pool[stack + 1] = left->m_left;	
					stack += 2;

					pool[stack] = left->m_right;	
					pool[stack + 1] = left->m_right;	
					stack += 2; 

					if (dgOverlapTest (left->m_left->m_minBox, left->m_left->m_maxBox, left->m_right->m_minBox, left->m_right->m_maxBox)) {
						pool[stack] = left->m_left;	
						pool[stack + 1] = left->m_right;	
						stack += 2;
					}
				}
			} else if (left->m_body || right->m_body) {
				if (!left->m_body) {
					dgAssert (right->m_body);

					pool[stack] = left->m_left;	
					pool[stack + 1] = right;	
					stack += 2;

					pool[stack] = left->m_right;	
					pool[stack + 1] = right;	
					stack += 2; 

				} else if (!right->m_body) {
					dgAssert (left->m_body);

					pool[stack] = left;	
					pool[stack + 1] = right->m_left;	
					stack += 2;

					pool[stack] = left;	
					pool[stack + 1] = right->m_right;	
					stack += 2; 

				} else {
					dgAssert (left->m_body);
					dgAssert (right->m_body);
					m_pairs[m_pairsCount * 2] = left;
					m_pairs[m_pairsCount * 2 + 1] = right;
					m_pairsCount ++;
				}

			} else if (dgOverlapTest (left->m_minBox, left->m_maxBox, right->m_minBox, right->m_maxBox)) { 
				dgAssert (!left->m_body);
				dgAssert (!right->m_body);

				pool[stack] = left->m_left;	
				pool[stack + 1] = right->m_left;	
				stack += 2;

				pool[stack] = left->m_left;	
				pool[stack + 1] = right->m_right;	
				stack += 2;

				pool[stack] = left->m_right;	
				pool[stack + 1] = right->m_left;	
				stack += 2;

				pool[stack] = left->m_right;	
				pool[stack + 1] = right->m_right;	
				stack += 2;
			}
		}

		dgAssert (!stack);
	}

	
	dgThread::dgCriticalSection* m_lock;
	dgInt32 m_pairsCount;
	dgInt32 m_pairsAtomicCounter;
	dgInt32 m_jointsAtomicCounter;
	dgFloat32 m_timestep;
	dgBodyMasterList::dgListNode* m_collindPairBodyNode;
	dgBodyMasterList::dgListNode* m_forceAndTorqueBodyNode;
	dgList<dgBody*>::dgListNode* m_newBodiesNodes;
	dgBroadPhase::dgType m_broadPhaseType;
	dgBroadPhase::dgNode* m_pairs[1024 * 4];	
};


class dgBroadPhase::dgSpliteInfo
{
	public:
	dgSpliteInfo (dgNode** const boxArray, dgInt32 boxCount)
	{
		dgVector minP ( dgFloat32 (1.0e15f)); 
		dgVector maxP (-dgFloat32 (1.0e15f)); 

		if (boxCount == 2) {
			m_axis = 1;
			for (dgInt32 i = 0; i < boxCount; i ++) {
				dgNode* const node = boxArray[i];
				dgAssert (node->m_body);
				minP = minP.GetMin (node->m_minBox); 
				maxP = maxP.GetMax (node->m_maxBox); 
			}
		} else {
			dgVector median (dgFloat32 (0.0f));
			dgVector varian (dgFloat32 (0.0f));

			for (dgInt32 i = 0; i < boxCount; i ++) {
				dgNode* const node = boxArray[i];
				dgAssert (node->m_body);
				minP = minP.GetMin (node->m_minBox); 
				maxP = maxP.GetMax (node->m_maxBox); 
				dgVector p ((node->m_minBox + node->m_maxBox).CompProduct4(dgVector::m_half));

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
					dgNode* const node = boxArray[i0];
					dgFloat32 val = (node->m_minBox[index] + node->m_maxBox[index]) * dgFloat32 (0.5f);
					if (val > test) {
						break;
					}
				}

				for (; i1 >= i0; i1 --) {
					dgNode* const node = boxArray[i1];
					dgFloat32 val = (node->m_minBox[index] + node->m_maxBox[index]) * dgFloat32 (0.5f);
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


dgBroadPhase::dgBroadPhase(dgWorld* const world)
	:m_world(world)
	,m_rootNode (NULL)
	,m_treeEntropy(dgFloat32 (0.0f))
	,m_lru(0)
	,m_fitness(world->GetAllocator())
	,m_generatedBodies(world->GetAllocator())
	,m_broadPhaseType(m_generic)
	,m_contacJointLock()
	,m_criticalSectionLock()
	,m_recursiveChunks(false)
{
}

dgBroadPhase::~dgBroadPhase()
{
	if (m_rootNode) {
		delete m_rootNode;
	}
}


dgBroadPhase::dgFitnessList::dgFitnessList (dgMemoryAllocator* const allocator)
	:dgList <dgNode*>(allocator)
{
}

dgFloat64 dgBroadPhase::dgFitnessList::TotalCost () const
{
	dgFloat64 cost = dgFloat32 (0.0f);
	for (dgListNode* node = GetFirst(); node; node = node->GetNext()) {
		dgNode* const box = node->GetInfo();
		cost += box->m_surfaceArea;
	}
	return cost;
}

dgFloat64 dgBroadPhase::CalculateEmptropy ()
{
	dgFloat64 cost0 = m_fitness.TotalCost ();
	dgFloat64 cost1 = cost0;
	do {
		cost0 = cost1;
		for (dgFitnessList::dgListNode* node = m_fitness.GetFirst(); node; node = node->GetNext()) {
			ImproveNodeFitness (node->GetInfo());
		}
		cost1 = m_fitness.TotalCost ();
	} while (cost1 < (dgFloat32 (0.99f)) * cost0);
	return cost1;
}



void dgBroadPhase::InvalidateCache ()
{
	ResetEntropy ();
	ImproveFitness();
}

void dgBroadPhase::GetWorldSize (dgVector& p0, dgVector& p1) const
{
	dgAssert (0);
}


void dgBroadPhase::ForEachBodyInAABB (const dgVector& minBox, const dgVector& maxBox, OnBodiesInAABB callback, void* const userData) const
{
	if (m_rootNode) {
		const dgNode* stackPool[DG_BROADPHASE_MAX_STACK_DEPTH];
		dgInt32 stack = 1;
		stackPool[0] = m_rootNode;

		dgBody* const sentinel = m_world->GetSentinelBody();
		while (stack) {
			stack --;
			const dgNode* const rootNode = stackPool[stack];
			if (dgOverlapTest (rootNode->m_minBox, rootNode->m_maxBox, minBox, maxBox)) {

				if (rootNode->m_body) {
					dgAssert (!rootNode->m_left);
					dgAssert (!rootNode->m_right);
					dgBody* const body = rootNode->m_body;
					if (dgOverlapTest (body->m_minAABB, body->m_maxAABB, minBox, maxBox)) {
						if (body != sentinel) {
							if (!callback (body, userData)) {
								break;
							}
						}
					}
				} else {
					stackPool[stack] = rootNode->m_left;
					stack ++;
					dgAssert (stack < dgInt32 (sizeof (stackPool) / sizeof (stackPool[0])));

					stackPool[stack] = rootNode->m_right;
					stack ++;
					dgAssert (stack < dgInt32 (sizeof (stackPool) / sizeof (stackPool[0])));
				}
			}
		}
	}
}


dgInt32 dgBroadPhase::GetBroadPhaseType () const
{
	return m_broadPhaseType;
}

void dgBroadPhase::SelectBroadPhaseType (dgInt32 algorthmType)
{
	if (algorthmType == 0) {
		m_broadPhaseType = m_generic;
	} else {
		m_broadPhaseType = m_persistent;
	}
}


DG_INLINE dgFloat32 dgBroadPhase::CalculateSurfaceArea (const dgNode* const node0, const dgNode* const node1, dgVector& minBox, dgVector& maxBox) const
{
	minBox = node0->m_minBox.GetMin(node1->m_minBox);
	maxBox = node0->m_maxBox.GetMax(node1->m_maxBox);
	dgVector side0 (maxBox - minBox);
	return side0.DotProduct4(side0.ShiftTripleRight()).m_x;
}



dgBroadPhase::dgNode* dgBroadPhase::InsertNode (dgNode* const node)
{
	dgVector p0;
	dgVector p1;

	dgNode* sibling = m_rootNode;
	dgFloat32 surfaceArea = CalculateSurfaceArea (node, sibling, p0, p1);
	while(sibling->m_left && sibling->m_right) {
		if (surfaceArea > sibling->m_surfaceArea) {
			break;
		} 

		sibling->m_minBox = p0;
		sibling->m_maxBox = p1;
		sibling->m_surfaceArea = surfaceArea;

		dgVector leftP0;
		dgVector leftP1;		
		dgFloat32 leftSurfaceArea = CalculateSurfaceArea (node, sibling->m_left, leftP0, leftP1);

		dgVector rightP0;
		dgVector rightP1;		
		dgFloat32 rightSurfaceArea = CalculateSurfaceArea (node, sibling->m_right, rightP0, rightP1);

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

	dgNode* const parent = new (m_world->GetAllocator()) dgNode (sibling, node);
	parent->m_fitnessNode = m_fitness.Append (parent);

	return parent;
}

void dgBroadPhase::Add (dgBody* const body)
{
	// create a new leaf node;
	dgNode* const newNode = new (m_world->GetAllocator()) dgNode (body);

	if (!m_rootNode) {
		m_rootNode = newNode;
	} else {
		dgNode* const node = InsertNode(newNode);

		if (!node->m_parent) {
			m_rootNode = node;
		}
	}
}


void dgBroadPhase::Remove (dgBody* const body)
{
	dgNode* const node = body->m_collisionCell;

	dgAssert (!node->m_fitnessNode);

	if (node->m_parent) {
		dgNode* const grandParent = node->m_parent->m_parent;
		if (grandParent) {
			if (grandParent->m_left == node->m_parent) {
				if (node->m_parent->m_right == node) {
					grandParent->m_left = node->m_parent->m_left;
					node->m_parent->m_left->m_parent = grandParent;
					node->m_parent->m_left = NULL;
					node->m_parent->m_parent = NULL;
				} else {
					grandParent->m_left = node->m_parent->m_right;
					node->m_parent->m_right->m_parent = grandParent;
					node->m_parent->m_right = NULL;
					node->m_parent->m_parent = NULL;
				}
			} else {
				if (node->m_parent->m_right == node) {
					grandParent->m_right = node->m_parent->m_left;
					node->m_parent->m_left->m_parent = grandParent;
					node->m_parent->m_left = NULL;
					node->m_parent->m_parent = NULL;
				} else {
					grandParent->m_right = node->m_parent->m_right;
					node->m_parent->m_right->m_parent = grandParent;
					node->m_parent->m_right = NULL;
					node->m_parent->m_parent = NULL;
				}
			}
		} else {
			if (node->m_parent->m_right == node) {
				m_rootNode = node->m_parent->m_left;
				m_rootNode->m_parent = NULL;
				node->m_parent->m_left = NULL;
			} else {
				m_rootNode = node->m_parent->m_right;
				m_rootNode->m_parent = NULL;
				node->m_parent->m_right = NULL;
			}
		}

		dgAssert (node->m_parent->m_fitnessNode);
		m_fitness.Remove(node->m_parent->m_fitnessNode);
		delete node->m_parent;
	} else {
		delete node;
		m_rootNode = NULL;
	}
}



void dgBroadPhase::ImproveNodeFitness (dgNode* const node)
{
	dgAssert (node->m_left);
	dgAssert (node->m_right);

	if (node->m_parent)	{
		if (node->m_parent->m_left == node) {
			dgFloat32 cost0 = node->m_surfaceArea;

			dgVector cost1P0;
			dgVector cost1P1;		
			dgFloat32 cost1 = CalculateSurfaceArea (node->m_right, node->m_parent->m_right, cost1P0, cost1P1);

			dgVector cost2P0;
			dgVector cost2P1;		
			dgFloat32 cost2 = CalculateSurfaceArea (node->m_left, node->m_parent->m_right, cost2P0, cost2P1);

			if ((cost1 <= cost0) && (cost1 <= cost2)) {
				dgNode* const parent = node->m_parent;
				node->m_minBox = parent->m_minBox;
				node->m_maxBox = parent->m_maxBox;
				node->m_surfaceArea = parent->m_surfaceArea; 
				if (parent->m_parent) {
					if (parent->m_parent->m_left == parent) {
						parent->m_parent->m_left = node;
					} else {
						dgAssert (parent->m_parent->m_right == parent);
						parent->m_parent->m_right = node;
					}
				} else {
					m_rootNode = node;
				}
				node->m_parent = parent->m_parent;
				parent->m_parent = node;
				node->m_right->m_parent = parent;
				parent->m_left = node->m_right;
				node->m_right = parent;
				parent->m_minBox = cost1P0;
				parent->m_maxBox = cost1P1;		
				parent->m_surfaceArea = cost1;


			} else if ((cost2 <= cost0) && (cost2 <= cost1)) {
				dgNode* const parent = node->m_parent;
				node->m_minBox = parent->m_minBox;
				node->m_maxBox = parent->m_maxBox;
				node->m_surfaceArea = parent->m_surfaceArea; 

				if (parent->m_parent) {
					if (parent->m_parent->m_left == parent) {
						parent->m_parent->m_left = node;
					} else {
						dgAssert (parent->m_parent->m_right == parent);
						parent->m_parent->m_right = node;
					}
				} else {
					m_rootNode = node;
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
		} else {
			dgFloat32 cost0 = node->m_surfaceArea;

			dgVector cost1P0;
			dgVector cost1P1;		
			dgFloat32 cost1 = CalculateSurfaceArea (node->m_left, node->m_parent->m_left, cost1P0, cost1P1);

			dgVector cost2P0;
			dgVector cost2P1;		
			dgFloat32 cost2 = CalculateSurfaceArea (node->m_right, node->m_parent->m_left, cost2P0, cost2P1);


			if ((cost1 <= cost0) && (cost1 <= cost2)) {

				dgNode* const parent = node->m_parent;
				node->m_minBox = parent->m_minBox;
				node->m_maxBox = parent->m_maxBox;
				node->m_surfaceArea = parent->m_surfaceArea; 
				if (parent->m_parent) {
					if (parent->m_parent->m_left == parent) {
						parent->m_parent->m_left = node;
					} else {
						dgAssert (parent->m_parent->m_right == parent);
						parent->m_parent->m_right = node;
					}
				} else {
					m_rootNode = node;
				}
				node->m_parent = parent->m_parent;
				parent->m_parent = node;
				node->m_left->m_parent = parent;
				parent->m_right = node->m_left;
				node->m_left = parent;

				parent->m_minBox = cost1P0;
				parent->m_maxBox = cost1P1;		
				parent->m_surfaceArea = cost1;

			} else if ((cost2 <= cost0) && (cost2 <= cost1)) {
				dgNode* const parent = node->m_parent;
				node->m_minBox = parent->m_minBox;
				node->m_maxBox = parent->m_maxBox;
				node->m_surfaceArea = parent->m_surfaceArea; 
				if (parent->m_parent) {
					if (parent->m_parent->m_left == parent) {
						parent->m_parent->m_left = node;
					} else {
						dgAssert (parent->m_parent->m_right == parent);
						parent->m_parent->m_right = node;
					}
				} else {
					m_rootNode = node;
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
	}
	dgAssert (!m_rootNode->m_parent);
}


dgBroadPhase::dgNode* dgBroadPhase::BuildTopDown (dgNode** const leafArray, dgInt32 firstBox, dgInt32 lastBox, dgFitnessList::dgListNode** const nextNode)
{
	dgAssert (firstBox >= 0);
	dgAssert (lastBox >= 0);

	if (lastBox == firstBox) {
		return leafArray[firstBox];
	} else {
		dgSpliteInfo info (&leafArray[firstBox], lastBox - firstBox + 1);

		dgNode* const parent = (*nextNode)->GetInfo();
		parent->m_parent = NULL;
		*nextNode = (*nextNode)->GetNext();

		parent->SetAABB (info.m_p0, info.m_p1);
		parent->m_right = BuildTopDown (leafArray, firstBox + info.m_axis, lastBox, nextNode);
		parent->m_right->m_parent = parent;

		parent->m_left = BuildTopDown (leafArray, firstBox, firstBox + info.m_axis - 1, nextNode);
		parent->m_left->m_parent = parent;
		return parent;
	}
}

void dgBroadPhase::ResetEntropy ()
{
	m_treeEntropy = dgFloat32 (0.0f);
}

void dgBroadPhase::ImproveFitness()
{
	dgFloat64 entropy = CalculateEmptropy();
	if ((entropy > m_treeEntropy * dgFloat32 (2.0f)) || (entropy < m_treeEntropy * dgFloat32 (0.5f))) {
		if (m_fitness.GetFirst()) {
			dgWorld* const world = m_world;
			dgInt32 count = m_fitness.GetCount() * 2 + 12;
			world->m_pairMemoryBuffer.ExpandCapacityIfNeessesary (count, sizeof (dgNode*));

			dgInt32 leafNodesCount = 0;
			dgNode** const leafArray = (dgNode**)&world->m_pairMemoryBuffer[0];
			for (dgFitnessList::dgListNode* nodePtr = m_fitness.GetFirst(); nodePtr; nodePtr = nodePtr->GetNext()) {
				dgNode* const node = nodePtr->GetInfo();
				dgNode* const leftNode = node->m_left;
				if (leftNode->m_body) {
					node->SetAABB(leftNode->m_body->m_minAABB, leftNode->m_body->m_maxAABB);
					leafArray[leafNodesCount] = leftNode;
					leafNodesCount ++;
				}
				dgNode* const rightNode = node->m_right;
				if (rightNode->m_body) {
					rightNode->SetAABB (rightNode->m_body->m_minAABB, rightNode->m_body->m_maxAABB);
					leafArray[leafNodesCount] = rightNode;
					leafNodesCount ++;
				}
			}

			dgFitnessList::dgListNode* nodePtr = m_fitness.GetFirst();
			m_rootNode = BuildTopDown (leafArray, 0, leafNodesCount - 1, &nodePtr);
			m_treeEntropy = CalculateEmptropy();
		} else {
			m_treeEntropy = entropy;
		}
	}
}

void dgBroadPhase::AddPair (dgBody* const body0, dgBody* const body1, const dgVector& timestep2, dgInt32 threadID)
{
	dgAssert ((body0->GetInvMass().m_w != dgFloat32 (0.0f)) || (body1->GetInvMass().m_w != dgFloat32 (0.0f)) || (body0->IsRTTIType(dgBody::m_kinematicBodyRTTI | dgBody::m_deformableBodyRTTI)) || (body1->IsRTTIType(dgBody::m_kinematicBodyRTTI | dgBody::m_deformableBodyRTTI)));

	// add all pairs 
	const dgBodyMaterialList* const materialList = m_world;  
	dgCollidingPairCollector* const contactPairs = m_world;

	bool isCollidable = true;
	dgContact* contact = NULL;
	if ((body0->IsRTTIType(dgBody::m_kinematicBodyRTTI | dgBody::m_deformableBodyRTTI)) || (body0->GetInvMass().m_w != dgFloat32 (0.0f))) {
		dgThreadHiveScopeLock lock (m_world, &m_contacJointLock);
		for (dgBodyMasterListRow::dgListNode* link = body0->m_masterNode->GetInfo().GetFirst(); link; link = link->GetNext()) {
			dgConstraint* const constraint = link->GetInfo().m_joint;
			if (constraint->GetId() != dgConstraint::m_contactConstraint) {
				break;
			}
			if (link->GetInfo().m_bodyNode == body1) {
				contact = (dgContact*)constraint;
				isCollidable = true;
				break;
			}
		}

		for (dgBodyMasterListRow::dgListNode* link = body0->m_masterNode->GetInfo().GetLast(); link; link = link->GetPrev()) {
			dgConstraint* const constraint = link->GetInfo().m_joint;
			if (constraint->GetId() == dgConstraint::m_contactConstraint) {
				break;
			}
			if (link->GetInfo().m_bodyNode == body1) {
				isCollidable = constraint->IsCollidable();
				break;
			}
		}
	} else {
		dgThreadHiveScopeLock lock (m_world, &m_contacJointLock);
		dgAssert ((body1->GetInvMass().m_w != dgFloat32 (0.0f)) || (body1->IsRTTIType(dgBody::m_kinematicBodyRTTI | dgBody::m_deformableBodyRTTI)));
		for (dgBodyMasterListRow::dgListNode* link = body1->m_masterNode->GetInfo().GetFirst(); link; link = link->GetNext()) {
			dgConstraint* const constraint = link->GetInfo().m_joint;
			if (constraint->GetId() != dgConstraint::m_contactConstraint) {
				break;
			}
			if (link->GetInfo().m_bodyNode == body0) {
				contact = (dgContact*)constraint;
				isCollidable = true;
				break;
			}
		}

		for (dgBodyMasterListRow::dgListNode* link = body1->m_masterNode->GetInfo().GetLast(); link; link = link->GetPrev()) {
			dgConstraint* const constraint = link->GetInfo().m_joint;
			if (constraint->GetId() == dgConstraint::m_contactConstraint) {
				break;
			}
			if (link->GetInfo().m_bodyNode == body0) {
				isCollidable = constraint->IsCollidable();
				break;
			}
		}
	}

	if (isCollidable) {
		bool newContact = false;
		if (!contact) {
			dgUnsigned32 group0_ID = dgUnsigned32 (body0->m_bodyGroupId);
			dgUnsigned32 group1_ID = dgUnsigned32 (body1->m_bodyGroupId);
			if (group1_ID < group0_ID) {
				dgSwap (group0_ID, group1_ID);
			}

			dgUnsigned32 key = (group1_ID << 16) + group0_ID;
			const dgContactMaterial* const material = &materialList->Find (key)->GetInfo();

			if (material->m_flags & dgContactMaterial::m_collisionEnable) {
				newContact = true;
				dgThreadHiveScopeLock lock (m_world, &m_contacJointLock);
				if (body0->IsRTTIType (dgBody::m_deformableBodyRTTI) || body1->IsRTTIType (dgBody::m_deformableBodyRTTI)) {
					contact = new (m_world->m_allocator) dgDeformableContact (m_world, material);
				} else {
					contact = new (m_world->m_allocator) dgContact (m_world, material);
				}
				contact->AppendToActiveList();
				m_world->AttachConstraint (contact, body0, body1);
			}
		}

		if (newContact) {
			contact->m_contactActive = 0;
			contact->m_broadphaseLru = m_lru;
			contact->m_timeOfImpact = dgFloat32 (1.0e10f);
			contactPairs->AddPair(contact, threadID);
		} else if (contact) {
			dgAssert (contact);
			bool kinematicBodyEquilibrium = (((body0->IsRTTIType(dgBody::m_kinematicBodyRTTI) ? true : false) & body0->IsCollidable()) | ((body1->IsRTTIType(dgBody::m_kinematicBodyRTTI) ? true : false) & body1->IsCollidable())) ? false : true;
			if (!(body0->m_equilibrium & body1->m_equilibrium & kinematicBodyEquilibrium & (contact->m_closestDistance > (DG_CACHE_DIST_TOL * dgFloat32 (4.0f))))) {
				contact->m_contactActive = 0;
				contact->m_broadphaseLru = m_lru;
				contact->m_timeOfImpact = dgFloat32 (1.0e10f);
				contactPairs->AddPair(contact, threadID);
			}
		}
	}
}


void dgBroadPhase::ApplyForceAndtorque (dgBroadphaseSyncDescriptor* const descriptor, dgInt32 threadID)
{
	dgFloat32 timestep = descriptor->m_timestep; 

	dgBodyMasterList::dgListNode* node = NULL;
	{
		dgThreadHiveScopeLock lock (m_world, descriptor->m_lock);
		node = descriptor->m_forceAndTorqueBodyNode;
		if (node) {
			descriptor->m_forceAndTorqueBodyNode = node->GetNext();
		}
	}

	for ( ; node; ) {
		dgBody* const body = node->GetInfo().GetBody();

		if (body->IsRTTIType(dgBody::m_dynamicBodyRTTI)) {
			dgDynamicBody* const dymamicBody = (dgDynamicBody*) body;

			dymamicBody->ApplyExtenalForces (timestep, threadID);
			if (!dymamicBody->IsInEquilibrium ()) {
				dymamicBody->m_sleeping = false;
				dymamicBody->m_equilibrium = false;
				// update collision matrix only
				dymamicBody->UpdateCollisionMatrix (timestep, threadID);
			}
			if (dymamicBody->GetInvMass().m_w == dgFloat32(0.0f) || body->m_collision->IsType(dgCollision::dgCollisionMesh_RTTI)) {
				dymamicBody->m_sleeping = true;	
				dymamicBody->m_autoSleep = true;
				dymamicBody->m_equilibrium = true;
			}
			
			dymamicBody->m_prevExternalForce = dymamicBody->m_accel;
			dymamicBody->m_prevExternalTorque = dymamicBody->m_alpha;
		} else {
			dgAssert (body->IsRTTIType(dgBody::m_kinematicBodyRTTI | dgBody::m_deformableBodyRTTI));

			if (body->IsRTTIType(dgBody::m_deformableBodyRTTI)) {
				body->ApplyExtenalForces (timestep, threadID);
			}

			// kinematic bodies are always sleeping (skip collision with kinematic bodies)
			if (body->IsCollidable()) {
				body->m_sleeping = false;	
				body->m_autoSleep = false;
			} else {
				body->m_sleeping = true;	
				body->m_autoSleep = true;
			}
			body->m_equilibrium = true;

			// update collision matrix by calling the transform callback for all kinematic bodies
			body->UpdateMatrix (timestep, threadID);
		}

		dgThreadHiveScopeLock lock (m_world, descriptor->m_lock);
		node = descriptor->m_forceAndTorqueBodyNode;
		if (node) {
			descriptor->m_forceAndTorqueBodyNode = node->GetNext();
		}
	}
}


void dgBroadPhase::KinematicBodyActivation (dgContact* const contatJoint) const
{
	dgBody* const body0 = contatJoint->GetBody0();
	dgBody* const body1 = contatJoint->GetBody1();
	if (body0->IsCollidable() | body1->IsCollidable()) {
		if (body0->IsRTTIType(dgBody::m_kinematicBodyRTTI)) {
			if (body1->IsRTTIType(dgBody::m_dynamicBodyRTTI) && (body1->GetInvMass().m_w > dgFloat32 (0.0f))) {
				if (body1->m_equilibrium) {
					dgVector relVeloc (body0->m_veloc - body1->m_veloc);
					dgVector relOmega (body0->m_omega - body1->m_omega);
					dgVector mask2 ((relVeloc.DotProduct4(relVeloc) < dgDynamicBody::m_equilibriumError2) & (relOmega.DotProduct4(relOmega) < dgDynamicBody::m_equilibriumError2));

					dgThreadHiveScopeLock lock (m_world, &body1->m_criticalSectionLock);
					body1->m_sleeping = false;
					body1->m_equilibrium = mask2.GetSignMask() ? true : false;
				}
			}
		} else if (body1->IsRTTIType(dgBody::m_kinematicBodyRTTI)) {
			if (body0->IsRTTIType(dgBody::m_dynamicBodyRTTI) && (body0->GetInvMass().m_w > dgFloat32 (0.0f))) {
				if (body0->m_equilibrium) {
					dgVector relVeloc (body0->m_veloc - body1->m_veloc);
					dgVector relOmega (body0->m_omega - body1->m_omega);
					dgVector mask2 ((relVeloc.DotProduct4(relVeloc) < dgDynamicBody::m_equilibriumError2) & (relOmega.DotProduct4(relOmega) < dgDynamicBody::m_equilibriumError2));

					dgThreadHiveScopeLock lock (m_world, &body0->m_criticalSectionLock);
					body0->m_sleeping = false;
					body0->m_equilibrium = mask2.GetSignMask() ? true : false;
				}
			}
		}
	}
}

void dgBroadPhase::CalculatePairContacts (dgBroadphaseSyncDescriptor* const descriptor, dgInt32 threadID)
{
	dgContactPoint contacts[DG_MAX_CONTATCS];
	
	dgFloat32 timestep = descriptor->m_timestep;
	dgCollidingPairCollector* const pairCollector = m_world;
	const dgInt32 count = pairCollector->m_count;
	dgCollidingPairCollector::dgPair* const pairs = (dgCollidingPairCollector::dgPair*) &m_world->m_pairMemoryBuffer[0];

	for (dgInt32 i = dgAtomicExchangeAndAdd(&descriptor->m_pairsAtomicCounter, 1); i < count; i = dgAtomicExchangeAndAdd(&descriptor->m_pairsAtomicCounter, 1)) {
		dgCollidingPairCollector::dgPair* const pair = &pairs[i];
		pair->m_cacheIsValid = false;
		pair->m_contactBuffer = contacts;
		m_world->CalculateContacts (pair, timestep, threadID, false, false);

		if (pair->m_contactCount) {
			dgAssert (pair->m_contactCount <= (DG_CONSTRAINT_MAX_ROWS / 3));
			if (pair->m_isDeformable) {
				m_world->ProcessDeformableContacts (pair, timestep, threadID);
			} else {
				m_world->ProcessContacts (pair, timestep, threadID);
				KinematicBodyActivation (pair->m_contact);
			}
		} else {
			if (pair->m_cacheIsValid) {
				//m_world->ProcessCachedContacts (pair->m_contact, timestep, threadID);
				KinematicBodyActivation (pair->m_contact);
			} else {
				pair->m_contact->m_maxDOF = 0;
			}
		}
	}
}





void dgBroadPhase::UpdateBodyBroadphase(dgBody* const body, dgInt32 threadIndex)
{
	if (m_rootNode) {

		dgNode* const node = body->m_collisionCell;
		dgAssert (!node->m_left);
		dgAssert (!node->m_right);

		if (!dgBoxInclusionTest (body->m_minAABB, body->m_maxAABB, node->m_minBox, node->m_maxBox)) {
			node->SetAABB(body->m_minAABB, body->m_maxAABB);
			for (dgNode* parent = node->m_parent; parent; parent = parent->m_parent) {
				dgVector minBox;
				dgVector maxBox;
				dgFloat32 area = CalculateSurfaceArea (parent->m_left, parent->m_right, minBox, maxBox);
				if (dgBoxInclusionTest (minBox, maxBox, parent->m_minBox, parent->m_maxBox)) {
					break;
				}

				dgThreadHiveScopeLock lock (m_world, &m_criticalSectionLock);
				parent->m_minBox = minBox;
				parent->m_maxBox = maxBox;
				parent->m_surfaceArea = area;
			}
		}
	}
}


void dgBroadPhase::ForceAndToqueKernel (void* const context, void* const worldContext, dgInt32 threadID)
{
	dgBroadphaseSyncDescriptor* const descriptor = (dgBroadphaseSyncDescriptor*) context;
	dgWorld* const world = (dgWorld*) worldContext;
	dgBroadPhase* const broadPhase = world->GetBroadPhase();

	if (!threadID) {
		dgUnsigned32 ticks0 = world->m_getPerformanceCount();
		broadPhase->ApplyForceAndtorque (descriptor, threadID);
		world->m_perfomanceCounters[m_forceCallbackTicks] = world->m_getPerformanceCount() - ticks0;
	} else {
		broadPhase->ApplyForceAndtorque (descriptor, threadID);
	}
}

void dgBroadPhase::CollidingPairsKernel (void* const context, void* const worldContext, dgInt32 threadID)
{
	dgBroadphaseSyncDescriptor* const descriptor = (dgBroadphaseSyncDescriptor*) context;
	dgWorld* const world = (dgWorld*) worldContext;
	dgBroadPhase* const broadPhase = world->GetBroadPhase();

	if (!threadID) {
		dgUnsigned32 ticks0 = world->m_getPerformanceCount();
		if (descriptor->m_broadPhaseType == dgBroadPhase::m_generic) {
			broadPhase->FindCollidingPairsGeneric (descriptor, threadID);
		} else {
			broadPhase->FindCollidingPairsPersistent (descriptor, threadID);
		}
		world->m_perfomanceCounters[m_broadPhaceTicks] = world->m_getPerformanceCount() - ticks0;
	} else {
		if (descriptor->m_broadPhaseType == dgBroadPhase::m_generic) {
			broadPhase->FindCollidingPairsGeneric (descriptor, threadID);
		} else {
			broadPhase->FindCollidingPairsPersistent (descriptor, threadID);
		}
	}
}

void dgBroadPhase::UpdateContactsKernel (void* const context, void* const worldContext, dgInt32 threadID)
{
	dgBroadphaseSyncDescriptor* const descriptor = (dgBroadphaseSyncDescriptor*) context;
	dgWorld* const world = (dgWorld*) worldContext;
	dgBroadPhase* const broadPhase = world->GetBroadPhase();

	if (!threadID) {
		dgUnsigned32 ticks0 = world->m_getPerformanceCount();
		broadPhase->CalculatePairContacts (descriptor, threadID);
		world->m_perfomanceCounters[m_narrowPhaseTicks] = world->m_getPerformanceCount() - ticks0;
	} else {
		broadPhase->CalculatePairContacts (descriptor, threadID);
	}
}


void dgBroadPhase::AddGeneratedBodyesContactsKernel (void* const context, void* const worldContext, dgInt32 threadID)
{
	dgBroadphaseSyncDescriptor* const descriptor = (dgBroadphaseSyncDescriptor*) context;
	dgWorld* const world = (dgWorld*) worldContext;
	dgBroadPhase* const broadPhase = world->GetBroadPhase();

	if (!threadID) {
		dgUnsigned32 ticks0 = world->m_getPerformanceCount();
		broadPhase->FindGeneratedBodiesCollidingPairs (descriptor, threadID);
		world->m_perfomanceCounters[m_broadPhaceTicks] += (world->m_getPerformanceCount() - ticks0);
	} else {
		broadPhase->FindGeneratedBodiesCollidingPairs (descriptor, threadID);
	}
}


bool dgBroadPhase::TestOverlaping (const dgBody* const body0, const dgBody* const body1) const
{
	bool mass0 = (body0->m_invMass.m_w != dgFloat32 (0.0f)); 
	bool mass1 = (body1->m_invMass.m_w != dgFloat32 (0.0f)); 
	bool isDynamic0 = body0->IsRTTIType (dgBody::m_dynamicBodyRTTI) != 0;
	bool isDynamic1 = body1->IsRTTIType (dgBody::m_dynamicBodyRTTI) != 0;
	bool isKinematic0 = body0->IsRTTIType (dgBody::m_kinematicBodyRTTI) != 0;
	bool isKinematic1 = body1->IsRTTIType (dgBody::m_kinematicBodyRTTI) != 0;

	bool tier0 = ((-dgOverlapTest (body0->m_minAABB, body0->m_maxAABB, body1->m_minAABB, body1->m_maxAABB)) >> 4) != 0;
	bool tier1 = !(body1->m_collision->IsType (dgCollision::dgCollisionNull_RTTI) | body0->m_collision->IsType (dgCollision::dgCollisionNull_RTTI));
	bool tier2 = !(body0->m_sleeping & body1->m_sleeping);
	bool tier3 = isDynamic0 & mass0; 
	bool tier4 = isDynamic1 & mass1; 
	bool tier5 = isKinematic0 & mass1; 
	bool tier6 = isKinematic1 & mass0; 
	//bool tier7 = body0->m_collision->GetCollisionMode() & body1->m_collision->GetCollisionMode(); 
	bool tier7 = true;
	bool ret = tier0 & tier1 & tier2 & tier7 & (tier3 | tier4 | tier5 | tier6);

	if (ret) {
		const dgCollisionInstance* const instance0 = body0->GetCollision();
		const dgCollisionInstance* const instance1 = body1->GetCollision();

		if (body0->m_continueCollisionMode | body1->m_continueCollisionMode) {
			dgVector box0_p0;
			dgVector box0_p1;
			dgVector box1_p0;
			dgVector box1_p1;
			
			instance0->CalcAABB(instance0->GetGlobalMatrix(), box0_p0, box0_p1);
			instance1->CalcAABB(instance1->GetGlobalMatrix(), box1_p0, box1_p1);

			dgVector boxp0 (box0_p0 - box1_p1);
			dgVector boxp1 (box0_p1 - box1_p0);

			dgVector velRelative (body1->GetVelocity() - body0->GetVelocity());
			dgFastRayTest ray (dgVector (dgFloat32 (0.0f)), velRelative);
			dgFloat32 distance = ray.BoxIntersect(boxp0, boxp1);
			ret = (distance < dgFloat32 (1.0f));

		} else {
#if 0
			dgVector size0;
			dgVector size1;
			dgVector origin0;
			dgVector origin1;

			instance0->CalcObb (origin0, size0);
			instance1->CalcObb (origin1, size1);
			dgMatrix matrix (instance1->GetGlobalMatrix() * instance0->GetGlobalMatrix().Inverse());
			dgMatrix matrixAbs;
			matrixAbs[0] = matrix[0].Abs();
			matrixAbs[1] = matrix[1].Abs();
			matrixAbs[2] = matrix[2].Abs();


			dgVector q0 (origin1 - size1);
			dgVector q1 (origin1 + size1);
			dgVector size (matrixAbs.UnrotateVector(size0));
			dgVector origin = matrix.UntransformVector(origin0);
			dgVector p0 (origin - size);
			dgVector p1 (origin + size);
			dgVector box0 (p0 - q1);
			dgVector box1 (p1 - q0);
			dgVector test (box0.CompProduct4((box1)));
			ret = (test.GetSignMask() & 0x07) == 0x07;
			if (ret) {
				dgVector p0 (origin0 - size0);
				dgVector p1 (origin0 + size0);
				dgVector size (matrixAbs.RotateVector(size1));
				origin1 = matrix.TransformVector(origin1);
				dgVector q0 (origin1 - size);
				dgVector q1 (origin1 + size);
				dgVector box0 (p0 - q1);
				dgVector box1 (p1 - q0);
				dgVector test (box0.CompProduct4((box1)));
				ret = (test.GetSignMask() & 0x07) == 0x07;

				for (dgInt32 i = 0; (i < 3) && ret; i ++) {
					dgVector dir(dgFloat32 (0.0f));
					dir[i] = dgFloat32 (1.0f);
					for (dgInt32 j = 0; (j < 3) && ret; j ++) {
						dgVector crossDir (dir * matrix[j]);
						if (crossDir.DotProduct4(crossDir).m_x > dgFloat32 (1.0e-7f)) {
							dgVector size2 (size0.DotProduct4(crossDir.Abs()));
							dgVector origin2 (origin0.DotProduct4(crossDir));
							dgVector p0 (origin2 - size2);
							dgVector p1 (origin2 + size2);
						
							dgVector origin3 (origin1.DotProduct4(crossDir));
							dgVector crossDir3 (matrix[0].DotProduct4(crossDir).m_x, matrix[1].DotProduct4(crossDir).m_x, matrix[2].DotProduct4(crossDir).m_x, dgFloat32 (0.0f));
							dgVector size3 (size1.DotProduct4(crossDir3.Abs()));
							dgVector q0 (origin3 - size3);
							dgVector q1 (origin3 + size3);

							dgVector box0 (p0 - q1);
							dgVector box1 (p1 - q0);
							dgVector test (box0.CompProduct4((box1)));
							ret = (test.GetSignMask() & 0x01) == 0x01;
						}
					}
				}
			}
#else

			dgVector size0;
			dgVector size1;
			dgVector origin0;
			dgVector origin1;
			instance0->CalcObb (origin0, size0);
			instance1->CalcObb (origin1, size1);
			const dgMatrix& matrix0 = instance0->GetGlobalMatrix();
			const dgMatrix& matrix1 = instance1->GetGlobalMatrix();
			origin0 = matrix0.TransformVector(origin0);
			origin1 = matrix1.TransformVector(origin1);
			const dgVector base (origin1 - origin0);
			dgVector dir0 (dgFloat32 (1.0f), dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f)); 
			for (dgInt32 i = 0; i < 4; i ++) {
				dgVector negDir (dir0.CompProduct4(dgVector::m_negOne));
				dgVector tmpDir (matrix1.UnrotateVector(matrix0.RotateVector(dir0)));

				dgVector mask0 (negDir < dgVector (dgFloat32 (0.0f)));
				dgVector mask1 (tmpDir < dgVector (dgFloat32 (0.0f)));

				dgVector q0(size0.AndNot(mask0) - (size0 & mask0));
				dgVector p0(matrix0.UnrotateVector(matrix1.RotateVector (size1.AndNot(mask1) - (size1 & mask1)) + base));
				
				dgVector q0p0 (q0 - p0);
				dgVector dot (q0p0.DotProduct4 (dir0));
				ret = (dot.GetSignMask() & 0x01) == 0x01;
				if (!ret) { 
					break;
				}
				dgAssert ((q0p0 % q0p0) > dgFloat32 (0.0f));
				q0p0 = q0p0.CompProduct4 (q0p0.InvMagSqrt());
				dot = q0p0.DotProduct4 (dir0);
				dir0 -= q0p0.CompProduct4 (dot.CompProduct4 (dgVector::m_two)); 
				dgAssert (dgAbsf(dir0 % dir0 - dgFloat32 (1.0f)) < dgFloat32 (1.0e-3f));
			}
#endif
		}
	}
	return ret;
}

void dgBroadPhase::SubmitPairsPersistent (dgNode* const bodyNode, dgNode* const node, const dgVector& timeStepBound, dgInt32 threadID)
{
	dgNode* pool[DG_BROADPHASE_MAX_STACK_DEPTH];
	pool[0] = node;
	dgInt32 stack = 1;

	dgBody* const body0 = bodyNode->m_body;
	dgAssert (!body0->m_collision->IsType (dgCollision::dgCollisionNull_RTTI));
	
	while (stack) {
		stack --;
		dgNode* const rootNode = pool[stack];
		if (dgOverlapTest (rootNode->m_minBox, rootNode->m_maxBox, body0->m_minAABB, body0->m_maxAABB)) {
			if (!rootNode->m_left) {
				dgAssert (!rootNode->m_right);
				dgBody* const body1 = rootNode->m_body;
				if (TestOverlaping (body0, body1)) {
					AddPair (body0, body1, timeStepBound, threadID);
				}

			} else {
				pool[stack] = rootNode->m_left;
				stack ++;
				dgAssert (stack < dgInt32 (sizeof (pool) / sizeof (pool[0])));

				pool[stack] = rootNode->m_right;
				stack ++;
				dgAssert (stack < dgInt32 (sizeof (pool) / sizeof (pool[0])));
			}
		}
	}
}


void dgBroadPhase::FindCollidingPairsPersistent (dgBroadphaseSyncDescriptor* const descriptor, dgInt32 threadID)
{
	dgVector timestep2 (descriptor->m_timestep * descriptor->m_timestep * dgFloat32 (4.0f));
	dgBodyMasterList::dgListNode* node = NULL;
	{
		dgThreadHiveScopeLock lock (m_world, descriptor->m_lock);
		node = descriptor->m_collindPairBodyNode;
		if (node) {
			descriptor->m_collindPairBodyNode = node->GetNext();
		}
	}
	
	for ( ;node; ) {
		dgBody* const body = node->GetInfo().GetBody();
		if (body->m_collisionCell) {
			if (!body->m_collision->IsType (dgCollision::dgCollisionNull_RTTI)) {
				dgNode* const bodyNode = body->m_collisionCell;
				for (dgNode* ptr = bodyNode; ptr->m_parent; ptr = ptr->m_parent) {
					dgNode* const sibling = ptr->m_parent->m_right;
					if (sibling != ptr) {
						SubmitPairsPersistent (bodyNode, sibling, timestep2, threadID);
					}
				}
			}
		}

		dgThreadHiveScopeLock lock (m_world, descriptor->m_lock);
		node = descriptor->m_collindPairBodyNode;
		if (node) {
			descriptor->m_collindPairBodyNode = node->GetNext();
		}
	}
}

void dgBroadPhase::FindGeneratedBodiesCollidingPairs (dgBroadphaseSyncDescriptor* const descriptor, dgInt32 threadID)
{
	dgList<dgBody*>::dgListNode* node = NULL;
	{
		dgThreadHiveScopeLock lock (m_world, descriptor->m_lock);
		node = descriptor->m_newBodiesNodes;
		if (node) {
			descriptor->m_newBodiesNodes = node->GetNext();
		}
	}

	dgVector timestep2 (descriptor->m_timestep * descriptor->m_timestep * dgFloat32 (4.0f));
	for ( ;node; ) {
		dgBody* const body = node->GetInfo();
		if (body->m_collisionCell) {
			if (!body->m_collision->IsType (dgCollision::dgCollisionNull_RTTI)) {
				dgNode* const bodyNode = body->m_collisionCell;
				for (dgNode* ptr = bodyNode; ptr->m_parent; ptr = ptr->m_parent) {
					dgNode* const sibling = ptr->m_parent->m_right;
					if (sibling != ptr) {
						SubmitPairsPersistent (bodyNode, sibling, timestep2, threadID);
					} else {
						dgNode* const sibling = ptr->m_parent->m_left;
						dgAssert (sibling);
						dgAssert (sibling != ptr);
						SubmitPairsPersistent (bodyNode, sibling, timestep2, threadID);
					}
				}
			}
		}

		dgThreadHiveScopeLock lock (m_world, descriptor->m_lock);
		node = descriptor->m_newBodiesNodes;
		if (node) {
			descriptor->m_newBodiesNodes = node->GetNext();
		}
	}
}

void dgBroadPhase::FindCollidingPairsGeneric (dgBroadphaseSyncDescriptor* const descriptor, dgInt32 threadID)
{
	dgNode* pool[2 * DG_BROADPHASE_MAX_STACK_DEPTH];		
	dgInt32 index;

	dgVector timestep2 (descriptor->m_timestep * descriptor->m_timestep * dgFloat32 (4.0f));
	while ((index = dgAtomicExchangeAndAdd(&descriptor->m_pairsCount, -1)) > 0) {
		index --;
		pool[0] = descriptor->m_pairs[index * 2];
		pool[1] = descriptor->m_pairs[index * 2 + 1];

		dgInt32 stack = 2;
		while (stack) {
			stack -=2;
			dgNode* const left = pool[stack];
			dgNode* const right = pool[stack + 1];

			if (left == right) {
				if (!left->m_body) {

					dgAssert (stack+1 < (sizeof (pool) / sizeof (pool[0]))) ;
					pool[stack] = left->m_left;	
					pool[stack + 1] = left->m_right;	
					stack += (-dgOverlapTest (left->m_left->m_minBox, left->m_left->m_maxBox, left->m_right->m_minBox, left->m_right->m_maxBox) >> 4) & 2;

					dgAssert (stack + 1 < (sizeof(pool) / sizeof(pool[0])));
					pool[stack] = left->m_left;	
					pool[stack + 1] = left->m_left;	
					stack += 2;

					dgAssert (stack + 1 < (sizeof(pool) / sizeof(pool[0])));
					pool[stack] = left->m_right;	
					pool[stack + 1] = left->m_right;	
					stack += 2;
				}
			} else {
				if (left->m_body && right->m_body) {
					dgBody* const body0 = left->m_body;
					dgBody* const body1 = right->m_body;

					if (TestOverlaping (body0, body1)) {
						AddPair (body0, body1, timestep2, threadID);
					}

				} else if (!(left->m_body || right->m_body)) {

					dgAssert (stack + 1 < (sizeof(pool) / sizeof(pool[0])));
					pool[stack] = left->m_left;	
					pool[stack + 1] = right->m_left;	
					stack += (-dgOverlapTest (left->m_left->m_minBox, left->m_left->m_maxBox, right->m_left->m_minBox, right->m_left->m_maxBox) >> 4) & 2;

					dgAssert (stack + 1 < (sizeof(pool) / sizeof(pool[0])));
					pool[stack] = left->m_left;	
					pool[stack + 1] = right->m_right;	
					stack += (-dgOverlapTest (left->m_left->m_minBox, left->m_left->m_maxBox, right->m_right->m_minBox, right->m_right->m_maxBox) >> 4) & 2;

					dgAssert (stack + 1 < (sizeof(pool) / sizeof(pool[0])));
					pool[stack] = left->m_right;	
					pool[stack + 1] = right->m_left;	
					stack += (-dgOverlapTest (left->m_right->m_minBox, left->m_right->m_maxBox, right->m_left->m_minBox, right->m_left->m_maxBox) >> 4) & 2;

					dgAssert (stack + 1 < (sizeof(pool) / sizeof(pool[0])));
					pool[stack] = left->m_right;	
					pool[stack + 1] = right->m_right;	
					stack += (-dgOverlapTest (left->m_right->m_minBox, left->m_right->m_maxBox, right->m_right->m_minBox, right->m_right->m_maxBox) >> 4) & 2;


				} else if (left->m_body) {

					dgAssert (!right->m_body);
					dgAssert (stack + 1 < (sizeof(pool) / sizeof(pool[0])));
					pool[stack] = right->m_left;	
					pool[stack + 1] = left;	
					stack += (-dgOverlapTest (left->m_minBox, left->m_maxBox, right->m_left->m_minBox, right->m_left->m_maxBox) >> 4) & 2;

					dgAssert (stack + 1 < (sizeof(pool) / sizeof(pool[0])));
					pool[stack] = right->m_right;	
					pool[stack + 1] = left;	
					stack += (-dgOverlapTest (left->m_minBox, left->m_maxBox, right->m_right->m_minBox, right->m_right->m_maxBox) >> 4) & 2;

				} else {

					dgAssert (right->m_body);
					dgAssert (stack + 1 < (sizeof(pool) / sizeof(pool[0])));
					pool[stack] = left->m_left;	
					pool[stack + 1] = right;	
					stack += (-dgOverlapTest (right->m_minBox, right->m_maxBox, left->m_left->m_minBox, left->m_left->m_maxBox) >> 4) & 2;

					dgAssert (stack+1 < (sizeof (pool) / sizeof (pool[0]))) ;
					pool[stack] = left->m_right;	
					pool[stack + 1] = right;	
					stack += (-dgOverlapTest (right->m_minBox, right->m_maxBox, left->m_right->m_minBox, left->m_right->m_maxBox) >> 4) & 2;
				}
			}
		}
	}
}

void dgBroadPhase::UpdateContactsBroadPhaseEnd ()
{
	// delete all non used contacts
	dgInt32 count = 0;
	dgUnsigned32 lru = m_lru;

	dgActiveContacts* const contactList = m_world;
	dgContact** const deadContacs = (dgContact**) &m_world->m_pairMemoryBuffer[0];

	for (dgActiveContacts::dgListNode* contactNode = contactList->GetFirst(); contactNode; contactNode = contactNode->GetNext()) {
		dgContact* const contact = contactNode->GetInfo();
		if (contact->m_broadphaseLru != lru) {
			const dgBody* const body0 = contact->m_body0;
			const dgBody* const body1 = contact->m_body1;
			if (! ((body0->m_sleeping | body0->m_equilibrium) & (body1->m_sleeping | body1->m_equilibrium)) ) {
				deadContacs[count] = contact;
				count ++;
			} else if ((lru -  contact->m_broadphaseLru) > 200) {
				dgVector minBox (body0->m_minAABB - body1->m_maxAABB);
				dgVector maxBox (body0->m_maxAABB - body1->m_minAABB);
				dgVector mask ((minBox.CompProduct4(maxBox)) > dgVector (dgFloat32 (0.0f)));
				dgVector dist ((maxBox.Abs()).GetMin (minBox.Abs()) & mask);
				dist = dist.GetMax(dist.ShiftTripleRight());
				dist = dist.GetMax(dist.ShiftTripleRight());
				if (dist.GetScalar() < dgFloat32(2.0f)) {
					contact->m_broadphaseLru = lru - 1;
				} else {
					deadContacs[count] = contact;
					count ++;
				}
			}
		}
	}

	for (dgInt32 i = 0; i < count; i ++) {
		dgContact* const contact = deadContacs[i];
		m_world->DestroyConstraint (contact);
	}
}


void dgBroadPhase::RayCast (const dgVector& l0, const dgVector& l1, OnRayCastAction filter, OnRayPrecastAction prefilter, void* const userData) const
{
	if (filter && m_rootNode) {
		dgVector segment (l1 - l0);
		dgFloat32 dist2 = segment % segment;
		if (dist2 > dgFloat32 (1.0e-8f)) {

			dgFloat32 distance[DG_COMPOUND_STACK_DEPTH];
			const dgNode* stackPool[DG_BROADPHASE_MAX_STACK_DEPTH];

			dgFastRayTest ray (l0, l1);

			dgInt32 stack = 1;
			stackPool[0] = m_rootNode;
			distance[0] = ray.BoxIntersect(m_rootNode->m_minBox, m_rootNode->m_maxBox);
			
			dgFloat32 maxParam = dgFloat32 (1.2f);

			dgLineBox line;	
			line.m_l0 = l0;
			line.m_l1 = l1;

			dgVector test (line.m_l0 <= line.m_l1);
			line.m_boxL0 = (line.m_l0 & test) | line.m_l1.AndNot(test);
			line.m_boxL1 = (line.m_l1 & test) | line.m_l0.AndNot(test);

			const dgBody* const sentinel = m_world->GetSentinelBody();
			while (stack) {
				stack --;
				dgFloat32 dist = distance[stack];
				if (dist > maxParam) {
					break;
				} else {
					const dgNode* const me = stackPool[stack];
					dgAssert (me);
					if (me->m_body) {
						if (me->m_body != sentinel) {
							dgAssert (!me->m_left);
							dgAssert (!me->m_right);
							dgFloat32 param = me->m_body->RayCast (line, filter, prefilter, userData, maxParam);
							if (param < maxParam) {
								maxParam = param;
								if (maxParam < dgFloat32 (1.0e-8f)) {
									break;
								}
							}
						}
					} else {
						const dgNode* const left = me->m_left;
						dgAssert (left);
						dgFloat32 dist = ray.BoxIntersect(left->m_minBox, left->m_maxBox);
						if (dist < maxParam) {
							dgInt32 j = stack;
							for ( ; j && (dist > distance[j - 1]); j --) {
								stackPool[j] = stackPool[j - 1];
								distance[j] = distance[j - 1];
							}
							stackPool[j] = left;
							distance[j] = dist;
							stack++;
							dgAssert (stack < dgInt32 (sizeof (stackPool) / sizeof (dgNode*)));
						}

						const dgNode* const right = me->m_right;
						dgAssert (right);
						dist = ray.BoxIntersect(right->m_minBox, right->m_maxBox);
						if (dist < maxParam) {
							dgInt32 j = stack;
							for ( ; j && (dist > distance[j - 1]); j --) {
								stackPool[j] = stackPool[j - 1];
								distance[j] = distance[j - 1];
							}
							stackPool[j] = right;
							distance[j] = dist;
							stack++;
							dgAssert (stack < dgInt32 (sizeof (stackPool) / sizeof (dgNode*)));
						}
					}
				}
			}
		}
	}
}



void dgBroadPhase::ConvexRayCast (dgCollisionInstance* const shape, const dgMatrix& matrix, const dgVector& target, OnRayCastAction filter, OnRayPrecastAction prefilter, void* const userData, dgInt32 threadId) const
{
	if (filter && m_rootNode && shape->IsType(dgCollision::dgCollisionConvexShape_RTTI)) {

		dgVector boxP0;
		dgVector boxP1;
		shape->CalcAABB(shape->GetLocalMatrix() * matrix, boxP0, boxP1);

		dgInt32 stack = 1;
		dgFloat32 distance[DG_COMPOUND_STACK_DEPTH];
		const dgNode* stackPool[DG_BROADPHASE_MAX_STACK_DEPTH];		

		dgVector velocA((target - matrix.m_posit) & dgVector::m_triplexMask);
		dgFloat32 maxParam = dgFloat32 (1.02f);
		dgFastRayTest ray (dgVector (dgFloat32 (0.0f)), velocA);
		dgFloat32 quantizeStep = dgMax (dgFloat32 (1.0f) / velocA.DotProduct4(velocA).m_x, dgFloat32 (0.001f));


		dgVector minBox (m_rootNode->m_minBox - boxP1);
		dgVector maxBox (m_rootNode->m_maxBox - boxP0);
		stackPool[0] = m_rootNode;
		distance[0] = ray.BoxIntersect(minBox, maxBox);

		const dgBody* const sentinel = m_world->GetSentinelBody();
		while (stack) {
			stack --;
			dgFloat32 dist = distance[stack];
			if (dist > maxParam) {
				break;
			} else {
				const dgNode* const me = stackPool[stack];
				dgAssert (me);
				if (me->m_body) {
					if (me->m_body != sentinel) {
						dgAssert (!me->m_left);
						dgAssert (!me->m_right);
						dgBody* const body = me->m_body;
						if (!PREFILTER_RAYCAST (prefilter, body, shape, userData)) {
							dgFloat32 param = body->ConvexRayCast (ray, shape,boxP0, boxP1, matrix, velocA, filter, prefilter, userData, maxParam, threadId);
							if (param < maxParam) {
								param = dgMin (param + quantizeStep, dgFloat32 (1.0f));
								maxParam = param;
							}
						}
					}
				} else {
					const dgNode* const left = me->m_left;
					dgAssert (left);
					dgVector minBox (left->m_minBox - boxP1);
					dgVector maxBox (left->m_maxBox - boxP0);
					dgFloat32 dist = ray.BoxIntersect(minBox, maxBox);
					if (dist < maxParam) {
						dgInt32 j = stack;
						for ( ; j && (dist > distance[j - 1]); j --) {
							stackPool[j] = stackPool[j - 1];
							distance[j] = distance[j - 1];
						}
						stackPool[j] = left;
						distance[j] = dist;
						stack++;
						dgAssert (stack < dgInt32 (sizeof (stackPool) / sizeof (dgNode*)));
					}

					const dgNode* const right = me->m_right;
					dgAssert (right);
					minBox = right->m_minBox - boxP1;
					maxBox = right->m_maxBox - boxP0;
					dist = ray.BoxIntersect(minBox, maxBox);
					if (dist < maxParam) {
						dgInt32 j = stack;
						for ( ; j && (dist > distance[j - 1]); j --) {
							stackPool[j] = stackPool[j - 1];
							distance[j] = distance[j - 1];
						}
						stackPool[j] = right;
						distance[j] = dist;
						stack++;
						dgAssert (stack < dgInt32 (sizeof (stackPool) / sizeof (dgNode*)));
					}
				}
			}
		}
	}
}


dgInt32 dgBroadPhase::ConvexCast (dgCollisionInstance* const shape, const dgMatrix& matrix, const dgVector& target, dgFloat32& timeToImpact, OnRayPrecastAction prefilter, void* const userData, dgConvexCastReturnInfo* const info, dgInt32 maxContacts, dgInt32 threadIndex) const
{
	dgInt32 totalCount = 0;
	if (m_rootNode) {
		dgVector boxP0;
		dgVector boxP1;
		dgAssert (matrix.TestOrthogonal());
		shape->CalcAABB(matrix, boxP0, boxP1);

		dgInt32 stack = 1;
		dgTriplex points[DG_CONVEX_CAST_POOLSIZE];
		dgTriplex normals[DG_CONVEX_CAST_POOLSIZE];
		dgFloat32 penetration[DG_CONVEX_CAST_POOLSIZE];
		dgInt64 attributeA[DG_CONVEX_CAST_POOLSIZE];
		dgInt64 attributeB[DG_CONVEX_CAST_POOLSIZE];

		dgFloat32 distance[DG_COMPOUND_STACK_DEPTH];
		const dgNode* stackPool[DG_BROADPHASE_MAX_STACK_DEPTH];		
		
		dgVector velocA((target - matrix.m_posit) & dgVector::m_triplexMask);
		dgVector velocB(dgFloat32(0.0f));

		dgFloat32 time = dgFloat32 (1.0f);
		dgFloat32 maxParam = dgFloat32 (1.2f);
		dgFastRayTest ray (dgVector (dgFloat32 (0.0f)), velocA);

		dgVector minBox (m_rootNode->m_minBox - boxP1);
		dgVector maxBox (m_rootNode->m_maxBox - boxP0);
		stackPool[0] = m_rootNode;
		distance[0] = ray.BoxIntersect(minBox, maxBox);

		const dgBody* const sentinel = m_world->GetSentinelBody();
		while (stack) {
			stack --;

			dgFloat32 dist = distance[stack];

			if (dist > maxParam) {
				break;
			} else {
				const dgNode* const me = stackPool[stack];
				if (me->m_body) {
					if (me->m_body != sentinel) {
						dgAssert (!me->m_left);
						dgAssert (!me->m_right);
						dgBody* const body = me->m_body;
						if (!PREFILTER_RAYCAST (prefilter, body, shape, userData)) {
							dgInt32 count = m_world->CollideContinue(shape, matrix, velocA, velocB, body->m_collision, body->m_matrix, velocB, velocB, time, points, normals, penetration, attributeA, attributeB, DG_CONVEX_CAST_POOLSIZE, threadIndex);

							if (count) {
								if (time < maxParam) {
									if ((time - maxParam) < dgFloat32(-1.0e-3f)) {
										totalCount = 0;
									}
									maxParam = time;
									if (count >= (maxContacts - totalCount)) {
										count = maxContacts - totalCount;
									}

									for (dgInt32 i = 0; i < count; i++) {
										info[totalCount].m_point[0] = points[i].m_x;
										info[totalCount].m_point[1] = points[i].m_y;
										info[totalCount].m_point[2] = points[i].m_z;
                                        info[totalCount].m_point[3] = dgFloat32 (0.0f);
										info[totalCount].m_normal[0] = normals[i].m_x;
										info[totalCount].m_normal[1] = normals[i].m_y;
										info[totalCount].m_normal[2] = normals[i].m_z;
                                        info[totalCount].m_normal[3] = dgFloat32 (0.0f);
										info[totalCount].m_penetration = penetration[i];
										info[totalCount].m_contaID = attributeB[i];
                                        info[totalCount].m_hitBody = body;
										totalCount++;
									}
								}
								if (maxParam < 1.0e-8f) {
									break;
								}
							}
						}
					}

				} else {
					const dgNode* const left = me->m_left;
					dgAssert (left);
					dgVector minBox (left->m_minBox - boxP1);
					dgVector maxBox (left->m_maxBox - boxP0);
					dgFloat32 dist = ray.BoxIntersect(minBox, maxBox);
					if (dist < maxParam) {
						dgInt32 j = stack;
						for ( ; j && (dist > distance[j - 1]); j --) {
							stackPool[j] = stackPool[j - 1];
							distance[j] = distance[j - 1];
						}
						stackPool[j] = left;
						distance[j] = dist;
						stack++;
						dgAssert (stack < dgInt32 (sizeof (stackPool) / sizeof (dgNode*)));
					}

					const dgNode* const right = me->m_right;
					dgAssert (right);
					minBox = right->m_minBox - boxP1;
					maxBox = right->m_maxBox - boxP0;
					dist = ray.BoxIntersect(minBox, maxBox);
					if (dist < maxParam) {
						dgInt32 j = stack;
						for ( ; j && (dist > distance[j - 1]); j --) {
							stackPool[j] = stackPool[j - 1];
							distance[j] = distance[j - 1];
						}
						stackPool[j] = right;
						distance[j] = dist;
						stack++;
						dgAssert (stack < dgInt32 (sizeof (stackPool) / sizeof (dgNode*)));
					}
				}
			}
		}

		timeToImpact = maxParam;
	}

	return totalCount;
}

void dgBroadPhase::AddInternallyGeneratedBody(dgBody* const body)
{
	m_generatedBodies.Append(body);
}

dgUnsigned32 dgBroadPhase::GetLRU () const
{
    return m_lru;
}


void dgBroadPhase::UpdateContacts (dgFloat32 timestep)
{
	dgUnsigned32 ticks = m_world->m_getPerformanceCount();

	dgCollidingPairCollector* const contactPairs = m_world;
	contactPairs->Init();
	m_lru = m_lru + 1;

	m_recursiveChunks = true;
	dgInt32 threadsCount = m_world->GetThreadCount();	

	dgBroadphaseSyncDescriptor syncPoints(m_broadPhaseType, &m_criticalSectionLock);
	syncPoints.m_timestep = timestep;
	
	const dgBodyMasterList* const masterList = m_world;
	dgBodyMasterList::dgListNode* const firstBodyNode = masterList->GetFirst()->GetNext();
	syncPoints.m_collindPairBodyNode = firstBodyNode;
	syncPoints.m_forceAndTorqueBodyNode = firstBodyNode;

	for (dgInt32 i = 0; i < threadsCount; i ++) {
		m_world->QueueJob (ForceAndToqueKernel, &syncPoints, m_world);
	}
	m_world->SynchronizationBarrier();

	// update pre-listeners after the force and true are applied
	if (m_world->m_preListener.GetCount()) {
		dgUnsigned32 ticks = m_world->m_getPerformanceCount();
		for (dgWorld::dgListenerList::dgListNode* node = m_world->m_preListener.GetFirst(); node; node = node->GetNext()) {
			dgWorld::dgListener& listener = node->GetInfo();
			listener.m_onListenerUpdate (m_world, listener.m_userData, timestep);
		}
		m_world->m_perfomanceCounters[m_preUpdataListerTicks] = m_world->m_getPerformanceCount() - ticks;
	}

	ImproveFitness();
	if (m_broadPhaseType == m_generic) {
		syncPoints.CreatePairsJobs (m_rootNode);
	}

	for (dgInt32 i = 0; i < threadsCount; i ++) {
		m_world->QueueJob (CollidingPairsKernel, &syncPoints, m_world);
	}
	m_world->SynchronizationBarrier();

	for (dgInt32 i = 0; i < threadsCount; i ++) {
		m_world->QueueJob (UpdateContactsKernel, &syncPoints, m_world);
	}
	m_world->SynchronizationBarrier();

	m_recursiveChunks = false;
	if (m_generatedBodies.GetCount()) {
		syncPoints.m_newBodiesNodes = m_generatedBodies.GetFirst();
		for (dgInt32 i = 0; i < threadsCount; i ++) {
			m_world->QueueJob (AddGeneratedBodyesContactsKernel, &syncPoints, m_world);
		}
		m_world->SynchronizationBarrier();

		for (dgInt32 i = 0; i < threadsCount; i ++) {
			m_world->QueueJob (UpdateContactsKernel, &syncPoints, m_world);
		}
		m_world->SynchronizationBarrier();

		m_generatedBodies.RemoveAll();
	}


	UpdateContactsBroadPhaseEnd();

	dgUnsigned32 endTicks = m_world->m_getPerformanceCount();
	m_world->m_perfomanceCounters[m_collisionTicks] = endTicks - ticks - m_world->m_perfomanceCounters[m_forceCallbackTicks];

	// update soft body dynamics phase 1
    dgDeformableBodiesUpdate* const softBodyList = m_world;
    softBodyList->ApplyExternaForces(timestep);
	m_world->m_perfomanceCounters[m_softBodyTicks] = m_world->m_getPerformanceCount() - endTicks;
}
