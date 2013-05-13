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
#define DG_BROADPHASE_AABB_SCALE		dgFloat32 (4.0f)
#define DG_BROADPHASE_AABB_INV_SCALE	(dgFloat32 (1.0f) / DG_BROADPHASE_AABB_SCALE)


DG_MSC_VECTOR_AVX_ALIGMENT
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

		m_minBox = dgVector (dgMin (left->m_minBox.m_x, right->m_minBox.m_x), dgMin (left->m_minBox.m_y, right->m_minBox.m_y), dgMin (left->m_minBox.m_z, right->m_minBox.m_z), dgFloat32 (0.0f));
		m_maxBox = dgVector (dgMax (left->m_maxBox.m_x, right->m_maxBox.m_x), dgMax (left->m_maxBox.m_y, right->m_maxBox.m_y), dgMax (left->m_maxBox.m_z, right->m_maxBox.m_z), dgFloat32 (0.0f));

		dgVector side0 (m_maxBox - m_minBox);
		dgVector side1 (side0.m_y, side0.m_z, side0.m_x, dgFloat32 (0.0f));
		m_surfaceArea = side0 % side1;
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

		dgVector p0 (minBox.CompProduct(dgVector (DG_BROADPHASE_AABB_SCALE, DG_BROADPHASE_AABB_SCALE, DG_BROADPHASE_AABB_SCALE, dgFloat32 (0.0f))));
		dgVector p1 (maxBox.CompProduct(dgVector (DG_BROADPHASE_AABB_SCALE, DG_BROADPHASE_AABB_SCALE, DG_BROADPHASE_AABB_SCALE, dgFloat32 (0.0f))));

		p0.m_x = dgFloor (p0.m_x) * DG_BROADPHASE_AABB_INV_SCALE; 
		p0.m_y = dgFloor (p0.m_y) * DG_BROADPHASE_AABB_INV_SCALE;  
		p0.m_z = dgFloor (p0.m_z) * DG_BROADPHASE_AABB_INV_SCALE;  

		p1.m_x = dgFloor (p1.m_x + dgFloat32 (1.0f)) * DG_BROADPHASE_AABB_INV_SCALE;  
		p1.m_y = dgFloor (p1.m_y + dgFloat32 (1.0f)) * DG_BROADPHASE_AABB_INV_SCALE;  
		p1.m_z = dgFloor (p1.m_z + dgFloat32 (1.0f)) * DG_BROADPHASE_AABB_INV_SCALE;  

		m_minBox = p0;
		m_maxBox = p1;
		m_minBox.m_w = dgFloat32 (0.0f);
		m_maxBox.m_w = dgFloat32 (0.0f);

		dgVector side0 (p1 - p0);
		dgVector side1 (side0.m_y, side0.m_z, side0.m_x, dgFloat32 (0.0f));
		m_surfaceArea = side0 % side1;
	}

	void SetAABBSimd (const dgVector& minBox, const dgVector& maxBox)
	{
		dgAssert (minBox.m_x <= maxBox.m_x);
		dgAssert (minBox.m_y <= maxBox.m_y);
		dgAssert (minBox.m_z <= maxBox.m_z);
		dgAssert (minBox.m_w == dgFloat32 (0.0f));
		dgAssert (maxBox.m_w == dgFloat32 (0.0f));

		dgSimd scale (DG_BROADPHASE_AABB_SCALE);
		dgSimd invScale (DG_BROADPHASE_AABB_INV_SCALE);

		dgSimd p0 ((dgSimd&)minBox * scale);
		dgSimd p1 ((dgSimd&)maxBox * scale);

		p0 = p0.Floor() * invScale; 
		p1 = ((p1 + dgSimd::m_one).Floor() * invScale) & dgSimd::m_triplexMask; 

		(dgSimd&) m_minBox = p0;
		(dgSimd&) m_maxBox = p1;
		//m_minBox.m_w = dgFloat32 (0.0f);
		//m_maxBox.m_w = dgFloat32 (0.0f);

		dgSimd size (p1 - p0);
		size = size.DotProduct(size.ShiftTripleLeft());
		size.StoreScalar(&m_surfaceArea);
	}



	dgVector m_minBox;
	dgVector m_maxBox;
	dgFloat32 m_surfaceArea;
	dgBody* m_body;	
	dgNode* m_left;
	dgNode* m_right;
	dgNode* m_parent;
	dgList<dgNode*>::dgListNode* m_fitnessNode;

	friend class dgBody;
	friend class dgBroadPhase;
	friend class dgBroadphaseSyncDescriptor;
	friend class dgFitnessList;
} DG_GCC_VECTOR_AVX_ALIGMENT;



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
		,m_sofBodyNode(NULL)
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
	dgCollisionDeformableMeshList::dgListNode* m_sofBodyNode;
	dgBroadPhase::dgType m_broadPhaseType;
	dgBroadPhase::dgNode* m_pairs[1024 * 4];	
};



dgBroadPhase::dgBroadPhase(dgWorld* const world)
	:m_world(world)
	,m_rootNode (NULL)
	,m_lru(0)
	,m_fitness(world->GetAllocator())
	,m_broadPhaseType(m_dynamic)
	,m_contacJointLock()
	,m_criticalSectionLock()
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




void dgBroadPhase::InvalidateCache ()
{
	dgTrace (("Invalidate cache\n"));
}

void dgBroadPhase::GetWorldSize (dgVector& p0, dgVector& p1) const
{
	dgAssert (0);
}


void dgBroadPhase::ForEachBodyInAABB (const dgVector& q0, const dgVector& q1, OnBodiesInAABB callback, void* const userData) const
{
	if (m_rootNode) {
		const dgNode* stackPool[DG_BROADPHASE_MAX_STACK_DEPTH];
		dgInt32 stack = 1;
		stackPool[0] = m_rootNode;

		dgBody* const sentinel = m_world->GetSentinelBody();
		while (stack) {
			stack --;
			const dgNode* const rootNode = stackPool[stack];
			if (dgOverlapTest (rootNode->m_minBox, rootNode->m_maxBox, q0, q1)) {

				if (rootNode->m_body) {
					dgAssert (!rootNode->m_left);
					dgAssert (!rootNode->m_right);
					dgBody* const body = rootNode->m_body;
					if (dgOverlapTest (body->m_minAABB, body->m_maxAABB, q0, q1)) {
						if (body != sentinel) {
							callback (body, userData);
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
	dgInt32 algorthmType = m_dynamic;
	if (m_broadPhaseType == m_static) {
		algorthmType = 1;
	} else if (m_broadPhaseType == m_hybrid) {
		algorthmType = 2;
	}
	return algorthmType;
}

void dgBroadPhase::SelectBroadPhaseType (dgInt32 algorthmType)
{
	switch (algorthmType) 
	{
		case 1:
			m_broadPhaseType = m_static;
			break;

		case 2:
			m_broadPhaseType = m_hybrid;
			break;

		default:
			m_broadPhaseType = m_dynamic;
			break;
	}
}


DG_INLINE dgFloat32 dgBroadPhase::CalculateSurfaceArea (const dgNode* const node0, const dgNode* const node1, dgVector& minBox, dgVector& maxBox) const
{
	minBox = dgVector (dgMin (node0->m_minBox.m_x, node1->m_minBox.m_x), dgMin (node0->m_minBox.m_y, node1->m_minBox.m_y), dgMin (node0->m_minBox.m_z, node1->m_minBox.m_z), dgFloat32 (0.0f));
	maxBox = dgVector (dgMax (node0->m_maxBox.m_x, node1->m_maxBox.m_x), dgMax (node0->m_maxBox.m_y, node1->m_maxBox.m_y), dgMax (node0->m_maxBox.m_z, node1->m_maxBox.m_z), dgFloat32 (0.0f));		
	dgVector side0 (maxBox - minBox);
	dgVector side1 (side0.m_y, side0.m_z, side0.m_x, dgFloat32 (0.0f));
	return side0 % side1;
}

DG_INLINE dgFloat32 dgBroadPhase::CalculateSurfaceAreaSimd (const dgNode* const node0, const dgNode* const node1, dgSimd& minBox, dgSimd& maxBox) const
{
	minBox = ((dgSimd&)node0->m_minBox).GetMin((dgSimd&)node1->m_minBox);
	maxBox = ((dgSimd&)node0->m_maxBox).GetMax((dgSimd&)node1->m_maxBox);

	dgSimd size (maxBox - minBox);
	size = size.DotProduct(size.ShiftTripleLeft());
	dgFloat32 dot;
	size.StoreScalar(&dot);
	return dot;
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


void dgBroadPhase::ImproveNodeFitnessSimd (dgNode* const node)
{
	dgAssert (node->m_left);
	dgAssert (node->m_right);

	if (node->m_parent)	{
		if (node->m_parent->m_left == node) {
			dgFloat32 cost0 = node->m_surfaceArea;

			dgSimd cost1P0;
			dgSimd cost1P1;		
			dgFloat32 cost1 = CalculateSurfaceAreaSimd (node->m_right, node->m_parent->m_right, cost1P0, cost1P1);

			dgSimd cost2P0;
			dgSimd cost2P1;		
			dgFloat32 cost2 = CalculateSurfaceAreaSimd (node->m_left, node->m_parent->m_right, cost2P0, cost2P1);

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
				(dgSimd&)parent->m_minBox = cost1P0;
				(dgSimd&)parent->m_maxBox = cost1P1;		
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

				(dgSimd&)parent->m_minBox = cost2P0;
				(dgSimd&)parent->m_maxBox = cost2P1;		
				parent->m_surfaceArea = cost2;
			}
		} else {
			dgFloat32 cost0 = node->m_surfaceArea;

			dgSimd cost1P0;
			dgSimd cost1P1;		
			dgFloat32 cost1 = CalculateSurfaceAreaSimd (node->m_left, node->m_parent->m_left, cost1P0, cost1P1);

			dgSimd cost2P0;
			dgSimd cost2P1;		
			dgFloat32 cost2 = CalculateSurfaceAreaSimd (node->m_right, node->m_parent->m_left, cost2P0, cost2P1);


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


void dgBroadPhase::ImproveFitness()
{
	dgCpuClass cpu = m_world->m_cpu;

	dgFloat64 cost0 = m_fitness.TotalCost ();
	dgFloat64 cost1 = cost0;
	do {
		cost0 = cost1;
		if (cpu == dgSimdPresent) {
			for (dgFitnessList::dgListNode* node = m_fitness.GetFirst(); node; node = node->GetNext()) {
				ImproveNodeFitnessSimd (node->GetInfo());
			}
		} else {
			for (dgFitnessList::dgListNode* node = m_fitness.GetFirst(); node; node = node->GetNext()) {
				ImproveNodeFitness (node->GetInfo());
			}
		}
		cost1 = m_fitness.TotalCost ();
	} while (cost1 < (dgFloat32 (0.95f)) * cost0);
}

void dgBroadPhase::AddPair (dgBody* const body0, dgBody* const body1, dgInt32 threadID)
{
	dgAssert ((body0->GetInvMass().m_w != dgFloat32 (0.0f)) || (body1->GetInvMass().m_w != dgFloat32 (0.0f)) || (body0->IsRTTIType(dgBody::m_kinematicBodyRTTI | dgBody::m_deformableBodyRTTI)) || (body1->IsRTTIType(dgBody::m_kinematicBodyRTTI | dgBody::m_deformableBodyRTTI)));

	// add all pairs 
//	if (body0->GetCollision()->GetCollisionMode() & body1->GetCollision()->GetCollisionMode()) {
		const dgBodyMaterialList* const materialList = m_world;  
		dgCollidingPairCollector* const contactPairs = m_world;

		bool isCollidable = true;
		dgContact* contact = NULL;
		if ((body0->IsRTTIType(dgBody::m_kinematicBodyRTTI | dgBody::m_deformableBodyRTTI)) || (body0->GetInvMass().m_w != dgFloat32 (0.0f))) {
			for (dgBodyMasterListRow::dgListNode* link = m_world->FindConstraintLink (body0, body1); isCollidable && link; link = m_world->FindConstraintLinkNext (link, body1)) {
				dgConstraint* const constraint = link->GetInfo().m_joint;
				if (constraint->GetId() == dgConstraint::m_contactConstraint) {
					contact = (dgContact*)constraint;
				}
				isCollidable &= constraint->IsCollidable();
			}
		} else {
			dgAssert ((body1->GetInvMass().m_w != dgFloat32 (0.0f)) || (body1->IsRTTIType(dgBody::m_kinematicBodyRTTI | dgBody::m_deformableBodyRTTI)));
			for (dgBodyMasterListRow::dgListNode* link = m_world->FindConstraintLink (body1, body0); isCollidable && link; link = m_world->FindConstraintLinkNext (link, body0)) {
				dgConstraint* const constraint = link->GetInfo().m_joint;
				if (constraint->GetId() == dgConstraint::m_contactConstraint) {
					contact = (dgContact*)constraint;
					break;
				} 
				isCollidable &= constraint->IsCollidable();
			}
		}

		if (isCollidable) {
			if (!contact) {
				dgUnsigned32 group0_ID = dgUnsigned32 (body0->m_bodyGroupId);
				dgUnsigned32 group1_ID = dgUnsigned32 (body1->m_bodyGroupId);
				if (group1_ID < group0_ID) {
					dgSwap (group0_ID, group1_ID);
				}

				dgUnsigned32 key = (group1_ID << 16) + group0_ID;
				const dgContactMaterial* const material = &materialList->Find (key)->GetInfo();

				if (material->m_flags & dgContactMaterial::m_collisionEnable) {
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

			if (contact) {
				dgAssert (contact);
				contact->m_broadphaseLru = m_lru;
				contact->m_timeOfImpact = dgFloat32 (1.0e10f);
				contactPairs->AddPair(contact, threadID);
			}
		}
//	}
}


void dgBroadPhase::ApplyForceAndtorque (dgBroadphaseSyncDescriptor* const descriptor, dgInt32 threadID)
{
	dgFloat32 timestep = descriptor->m_timestep; 
	bool simd = (m_world->m_cpu == dgSimdPresent);

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
				if (simd) {
					dymamicBody->UpdateCollisionMatrixSimd (timestep, threadID);
				} else {
					dymamicBody->UpdateCollisionMatrix (timestep, threadID);
				}
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

void dgBroadPhase::UpdateSoftBodyForcesKernel (dgBroadphaseSyncDescriptor* const descriptor, dgInt32 threadID)
{
//	dgCollisionDeformableMeshList& softBodyList = *m_world;

	dgFloat32 timestep = descriptor->m_timestep; 
	dgCollisionDeformableMeshList::dgListNode* node = NULL;
	{
		dgThreadHiveScopeLock lock (m_world, descriptor->m_lock);
		node = descriptor->m_sofBodyNode;
		if (node) {
			descriptor->m_sofBodyNode = node->GetNext();
		}
	}
	for ( ; node; ) {
		dgCollisionDeformableMesh* const softShape = node->GetInfo();

		softShape->CalculateInternalForces (timestep);

		dgThreadHiveScopeLock lock (m_world, descriptor->m_lock);
		node = descriptor->m_sofBodyNode;
		if (node) {
			descriptor->m_sofBodyNode = node->GetNext();
		}
	}
}

void dgBroadPhase::CheckKenamaticBodyActivation (dgContact* const contatJoint) const
{
	dgBody* const body0 = contatJoint->GetBody0();
	dgBody* const body1 = contatJoint->GetBody1();
	if (body0->IsRTTIType(dgBody::m_kinematicBodyRTTI)) {
		if (body0->IsCollidable() && (body1->GetInvMass().m_w > dgFloat32 (0.0f))) {
			dgAssert (!body1->IsRTTIType(dgBody::m_kinematicBodyRTTI));
			dgThreadHiveScopeLock lock (m_world, &body1->m_criticalSectionLock);
			body1->m_sleeping = false;
		}
	} else if (body1->IsRTTIType(dgBody::m_kinematicBodyRTTI)) {
		if (body1->IsCollidable() && (body0->GetInvMass().m_w > dgFloat32 (0.0f))) {
			dgAssert (!body0->IsRTTIType(dgBody::m_kinematicBodyRTTI));
			dgThreadHiveScopeLock lock (m_world, &body0->m_criticalSectionLock);
			body0->m_sleeping = false;
		}
	}
}

void dgBroadPhase::CalculatePairContacts (dgBroadphaseSyncDescriptor* const descriptor, dgInt32 threadID)
{
	dgContactPoint contacts[DG_MAX_CONTATCS];
	
	dgFloat32 timestep = descriptor->m_timestep;
	dgCollidingPairCollector* const pairCollector = m_world;
	dgCollidingPairCollector::dgPair* const pairs = pairCollector->m_pairs;
	const dgInt32 count = pairCollector->m_count;

	if (m_world->m_cpu == dgSimdPresent) {
		dgAssert (0);
	} else {

		for (dgInt32 i = dgAtomicExchangeAndAdd(&descriptor->m_pairsAtomicCounter, 1); i < count; i = dgAtomicExchangeAndAdd(&descriptor->m_pairsAtomicCounter, 1)) {
			dgCollidingPairCollector::dgPair* const pair = &pairs[i];
			pair->m_cacheIsValid = false;
			pair->m_contactBuffer = contacts;
			m_world->CalculateContacts (pair, timestep, false, threadID);

			if (pair->m_contactCount) {
				dgAssert (pair->m_contactCount <= (DG_CONSTRAINT_MAX_ROWS / 3));
				if (pair->m_isDeformable) {
					m_world->ProcessDeformableContacts (pair, timestep, threadID);
				} else {
					m_world->ProcessContacts (pair, timestep, threadID);
					CheckKenamaticBodyActivation (pair->m_contact);
				}
			} else {
				if (pair->m_cacheIsValid) {
					m_world->ProcessCachedContacts (pair->m_contact, timestep, threadID);
					CheckKenamaticBodyActivation (pair->m_contact);
				} else {
					pair->m_contact->m_maxDOF = 0;
				}
			}
		}
	}
}




void dgBroadPhase::UpdateBodyBroadphaseSimd(dgBody* const body, dgInt32 threadIndex)
{
	if (m_rootNode) {

		// if the body is move inside the world, then active it
		// update bodies only if they are in the world
		dgNode* const node = body->m_collisionCell;
		dgAssert (!node->m_left);
		dgAssert (!node->m_right);

		if (!dgBoxInclusionTestSimd (body->m_minAABB, body->m_maxAABB, node->m_minBox, node->m_maxBox)) {
			node->SetAABBSimd(body->m_minAABB, body->m_maxAABB);
			for (dgNode* parent = node->m_parent; parent; parent = parent->m_parent) {
				//dgVector minBox;
				//dgVector maxBox;
				dgSimd minBox;
				dgSimd maxBox;		

				dgFloat32 area = CalculateSurfaceAreaSimd (parent->m_left, parent->m_right, minBox, maxBox);
				if (dgBoxInclusionTestSimd(minBox, maxBox, parent->m_minBox, parent->m_maxBox)) {
					break;
				}

				dgThreadHiveScopeLock lock (m_world, &m_criticalSectionLock);
				(dgSimd&)parent->m_minBox = minBox;
				(dgSimd&)parent->m_maxBox = maxBox;
				parent->m_surfaceArea = area;
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
		switch (descriptor->m_broadPhaseType) 
		{
			case dgBroadPhase::m_dynamic:
				broadPhase->FindCollidingPairsDynamics (descriptor, threadID);
				break;
			case dgBroadPhase::m_static:
				broadPhase->FindCollidingPairsStatic (descriptor, threadID);
				break;
			case dgBroadPhase::m_hybrid:
				broadPhase->FindCollidingPairsHybrid (descriptor, threadID);
				break;
		}
		world->m_perfomanceCounters[m_broadPhaceTicks] = world->m_getPerformanceCount() - ticks0;
	} else {
		switch (descriptor->m_broadPhaseType) 
		{
			case dgBroadPhase::m_dynamic:
				broadPhase->FindCollidingPairsDynamics (descriptor, threadID);
				break;
			case dgBroadPhase::m_static:
				broadPhase->FindCollidingPairsStatic (descriptor, threadID);
				break;
			case dgBroadPhase::m_hybrid:
				broadPhase->FindCollidingPairsHybrid (descriptor, threadID);
				break;
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


void dgBroadPhase::UpdateSoftBodyForcesKernel (void* const context, void* const worldContext, dgInt32 threadID)
{
	dgBroadphaseSyncDescriptor* const descriptor = (dgBroadphaseSyncDescriptor*) context;
	dgWorld* const world = (dgWorld*) worldContext;
	dgBroadPhase* const broadPhase = world->GetBroadPhase();
	broadPhase->UpdateSoftBodyForcesKernel (descriptor, threadID);
}


void dgBroadPhase::SubmitPairsStatic (dgNode* const bodyNode, dgNode* const node, dgInt32 threadID)
{
	dgNode* pool[DG_BROADPHASE_MAX_STACK_DEPTH];
	pool[0] = node;
	dgInt32 stack = 1;

	dgBody* const body0 = bodyNode->m_body;
	dgAssert (!body0->m_collision->IsType (dgCollision::dgCollisionNull_RTTI));

	if (m_world->m_cpu == dgSimdPresent) {
		dgAssert (0);
	} else {
		while (stack) {
			stack --;
			dgNode* const rootNode = pool[stack];
			if (dgOverlapTest (rootNode->m_minBox, rootNode->m_maxBox, body0->m_minAABB, body0->m_maxAABB)) {
				if (!rootNode->m_left) {
					dgAssert (!rootNode->m_right);
					dgBody* const body1 = rootNode->m_body;
					if (dgOverlapTest(body1->m_minAABB, body1->m_maxAABB, body0->m_minAABB, body0->m_maxAABB)) {
						if (!body1->m_collision->IsType (dgCollision::dgCollisionNull_RTTI)) {
							if (!(body0->GetSleepState() & body1->GetSleepState())) { 
								AddPair (body0, body1, threadID);
							}
						}
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
}


void dgBroadPhase::FindCollidingPairsHybrid (dgBroadphaseSyncDescriptor* const descriptor, dgInt32 threadID)
{

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
						SubmitPairsStatic (bodyNode, sibling, threadID);
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

void dgBroadPhase::FindCollidingPairsStatic (dgBroadphaseSyncDescriptor* const descriptor, dgInt32 threadID)
{
	FindCollidingPairsHybrid (descriptor, threadID);
}


void dgBroadPhase::FindCollidingPairsDynamics (dgBroadphaseSyncDescriptor* const descriptor, dgInt32 threadID)
{
	dgNode* pool[2 * DG_BROADPHASE_MAX_STACK_DEPTH];		

	dgInt32 index;
	while ((index = dgAtomicExchangeAndAdd(&descriptor->m_pairsCount, -1)) > 0) {
		index --;
		pool[0] = descriptor->m_pairs[index * 2];
		pool[1] = descriptor->m_pairs[index * 2 + 1];

		dgInt32 stack = 2;
		if (m_world->m_cpu != dgNoSimdPresent) {
			while (stack) {
				stack -=2;
				dgNode* const left = pool[stack];
				dgNode* const right = pool[stack + 1];

				if (left == right) {
					if (!left->m_body) {
						dgAssert (stack < (sizeof (pool) / sizeof (pool[0]))) ;
						pool[stack] = left->m_left;	
						pool[stack + 1] = left->m_right;	
						stack += 2 & (-dgOverlapTestSimd (left->m_left->m_minBox, left->m_left->m_maxBox, left->m_right->m_minBox, left->m_right->m_maxBox) >> 4);

						dgAssert (stack < (sizeof (pool) / sizeof (pool[0]))) ;
						pool[stack] = left->m_left;	
						pool[stack + 1] = left->m_left;	
						stack += 2;

						dgAssert (stack < (sizeof (pool) / sizeof (pool[0]))) ;
						pool[stack] = left->m_right;	
						pool[stack + 1] = left->m_right;	
						stack += 2;
					}
				} else {
					if (left->m_body && right->m_body) {
						dgBody* const body0 = left->m_body;
						dgBody* const body1 = right->m_body;
						dgInt32 test = (-dgOverlapTestSimd (body0->m_minAABB, body0->m_maxAABB, body1->m_minAABB, body1->m_maxAABB) >> 4) &
									   !(body1->m_collision->IsType (dgCollision::dgCollisionNull_RTTI) | body0->m_collision->IsType (dgCollision::dgCollisionNull_RTTI)) &
									   !(body0->GetSleepState() & body1->GetSleepState());
						if (test & 1) {
							AddPair (body0, body1, threadID);
						}


					} else if (!(left->m_body || right->m_body)) {

						dgAssert (stack < (sizeof (pool) / sizeof (pool[0]))) ;
						pool[stack] = left->m_left;	
						pool[stack + 1] = right->m_left;	
						stack += 2 & (-dgOverlapTestSimd (left->m_left->m_minBox, left->m_left->m_maxBox, right->m_left->m_minBox, right->m_left->m_maxBox) >> 4);

						dgAssert (stack < (sizeof (pool) / sizeof (pool[0]))) ;
						pool[stack] = left->m_left;	
						pool[stack + 1] = right->m_right;	
						stack += 2 & (-dgOverlapTestSimd (left->m_left->m_minBox, left->m_left->m_maxBox, right->m_right->m_minBox, right->m_right->m_maxBox) >> 4);

						dgAssert (stack < (sizeof (pool) / sizeof (pool[0]))) ;
						pool[stack] = left->m_right;	
						pool[stack + 1] = right->m_left;	
						stack += 2 & (-dgOverlapTestSimd (left->m_right->m_minBox, left->m_right->m_maxBox, right->m_left->m_minBox, right->m_left->m_maxBox) >> 4);

						dgAssert (stack < (sizeof (pool) / sizeof (pool[0]))) ;
						pool[stack] = left->m_right;	
						pool[stack + 1] = right->m_right;	
						stack += 2 & (-dgOverlapTestSimd (left->m_right->m_minBox, left->m_right->m_maxBox, right->m_right->m_minBox, right->m_right->m_maxBox) >> 4);

					} else if (left->m_body) {
						dgAssert (!right->m_body);
						dgAssert (stack < (sizeof (pool) / sizeof (pool[0]))) ;
						pool[stack] = right->m_left;	
						pool[stack + 1] = left;	
						stack += 2 & (-dgOverlapTestSimd (left->m_minBox, left->m_maxBox, right->m_left->m_minBox, right->m_left->m_maxBox) >> 4);

						dgAssert (stack < (sizeof (pool) / sizeof (pool[0]))) ;
						pool[stack] = right->m_right;	
						pool[stack + 1] = left;	
						stack += 2 & (-dgOverlapTestSimd (left->m_minBox, left->m_maxBox, right->m_right->m_minBox, right->m_right->m_maxBox) >> 4);

					} else {
						dgAssert (right->m_body);
						dgAssert (stack < (sizeof (pool) / sizeof (pool[0]))) ;
						pool[stack] = left->m_left;	
						pool[stack + 1] = right;	
						stack += 2 & (-dgOverlapTestSimd (right->m_minBox, right->m_maxBox, left->m_left->m_minBox, left->m_left->m_maxBox) >> 4);

						dgAssert (stack < (sizeof (pool) / sizeof (pool[0]))) ;
						pool[stack] = left->m_right;	
						pool[stack + 1] = right;	
						stack += 2 & (-dgOverlapTestSimd (right->m_minBox, right->m_maxBox, left->m_right->m_minBox, left->m_right->m_maxBox) >> 4);
					}
				}
			}

		} else {
			while (stack) {
				stack -=2;
				dgNode* const left = pool[stack];
				dgNode* const right = pool[stack + 1];

				if (left == right) {
					if (!left->m_body) {
						if (dgOverlapTest (left->m_left->m_minBox, left->m_left->m_maxBox, left->m_right->m_minBox, left->m_right->m_maxBox)) {
							dgAssert (stack < (sizeof (pool) / sizeof (pool[0]))) ;
							pool[stack] = left->m_left;	
							pool[stack + 1] = left->m_right;	
							stack += 2;
						}

						dgAssert (stack < (sizeof (pool) / sizeof (pool[0]))) ;
						pool[stack] = left->m_left;	
						pool[stack + 1] = left->m_left;	
						stack += 2;

						dgAssert (stack < (sizeof (pool) / sizeof (pool[0]))) ;
						pool[stack] = left->m_right;	
						pool[stack + 1] = left->m_right;	
						stack += 2;
					}
				} else {
					if (left->m_body && right->m_body) {
						dgBody* const body0 = left->m_body;
						dgBody* const body1 = right->m_body;
//						if (dgOverlapTest(body0->m_minAABB, body0->m_maxAABB, body1->m_minAABB, body1->m_maxAABB)) {
//							if (!body1->m_collision->IsType (dgCollision::dgCollisionNull_RTTI)) {
//								if (!body0->m_collision->IsType (dgCollision::dgCollisionNull_RTTI)) {
//									if (!(body0->GetSleepState() & body1->GetSleepState())) { 
//										AddPair (body0, body1, threadID);
//									}
//								}
//							}
//						}
						dgInt32 test = (-dgOverlapTest(body0->m_minAABB, body0->m_maxAABB, body1->m_minAABB, body1->m_maxAABB) >> 4) &
										!(body1->m_collision->IsType (dgCollision::dgCollisionNull_RTTI) | body0->m_collision->IsType (dgCollision::dgCollisionNull_RTTI)) &
										!(body0->GetSleepState() & body1->GetSleepState());
						if (test & 1) {
							AddPair (body0, body1, threadID);
						}
					} else if (!(left->m_body || right->m_body)) {

						if (dgOverlapTest (left->m_left->m_minBox, left->m_left->m_maxBox, right->m_left->m_minBox, right->m_left->m_maxBox)) { 
							dgAssert (stack < (sizeof (pool) / sizeof (pool[0]))) ;
							pool[stack] = left->m_left;	
							pool[stack + 1] = right->m_left;	
							stack += 2;
						}

						if (dgOverlapTest (left->m_left->m_minBox, left->m_left->m_maxBox, right->m_right->m_minBox, right->m_right->m_maxBox)) { 
							dgAssert (stack < (sizeof (pool) / sizeof (pool[0]))) ;
							pool[stack] = left->m_left;	
							pool[stack + 1] = right->m_right;	
							stack += 2;
						}

						if (dgOverlapTest (left->m_right->m_minBox, left->m_right->m_maxBox, right->m_left->m_minBox, right->m_left->m_maxBox)) { 
							dgAssert (stack < (sizeof (pool) / sizeof (pool[0]))) ;
							pool[stack] = left->m_right;	
							pool[stack + 1] = right->m_left;	
							stack += 2;
						}

						if (dgOverlapTest (left->m_right->m_minBox, left->m_right->m_maxBox, right->m_right->m_minBox, right->m_right->m_maxBox)) { 
							pool[stack] = left->m_right;	
							pool[stack + 1] = right->m_right;	
							stack += 2;
						}

					} else if (left->m_body) {
						dgAssert (!right->m_body);
						if (dgOverlapTest (left->m_minBox, left->m_maxBox, right->m_left->m_minBox, right->m_left->m_maxBox)) { 
							pool[stack] = right->m_left;	
							pool[stack + 1] = left;	
							stack += 2;
						}

						if (dgOverlapTest (left->m_minBox, left->m_maxBox, right->m_right->m_minBox, right->m_right->m_maxBox)) { 
							pool[stack] = right->m_right;	
							pool[stack + 1] = left;	
							stack += 2;
						}

					} else {
						dgAssert (right->m_body);
						if (dgOverlapTest (right->m_minBox, right->m_maxBox, left->m_left->m_minBox, left->m_left->m_maxBox)) { 
							dgAssert (stack < (sizeof (pool) / sizeof (pool[0]))) ;
							pool[stack] = left->m_left;	
							pool[stack + 1] = right;	
							stack += 2;
						}

						if (dgOverlapTest (right->m_minBox, right->m_maxBox, left->m_right->m_minBox, left->m_right->m_maxBox)) { 
							dgAssert (stack < (sizeof (pool) / sizeof (pool[0]))) ;
							pool[stack] = left->m_right;	
							pool[stack + 1] = right;	
							stack += 2;
						}
					}
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
	dgCollidingPairCollector& contactPair = *m_world;
	dgContact** const deadContacs = (dgContact**) contactPair.m_pairs;

	for (dgActiveContacts::dgListNode* contactNode = contactList->GetFirst(); contactNode; contactNode = contactNode->GetNext()) {
		dgContact* const contact = contactNode->GetInfo();
		if (contact->m_broadphaseLru != lru) {
			const dgBody* const body0 = contact->m_body0;
			const dgBody* const body1 = contact->m_body1;
			if (! ((body0->m_sleeping | body0->m_equilibrium) & (body1->m_sleeping | body1->m_equilibrium)) ) {
				deadContacs[count] = contact;
				count ++;
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
			const dgNode* stackPool[DG_BROADPHASE_MAX_STACK_DEPTH];
			dgInt32 stack = 1;
			stackPool[0] = m_rootNode;
			dgFastRayTest ray (l0, l1);
			dgFloat32 maxParam = dgFloat32 (1.2f);
			const dgBody* const sentinel = m_world->GetSentinelBody();

			if (m_world->m_cpu == dgSimdPresent) {
				dgLineBox line;	

				line.m_l0 = l0;
				line.m_l1 = l1;
				line.m_boxL0 = dgVector (((dgSimd&)l0).GetMin((dgSimd&)l1));
				line.m_boxL1 = dgVector (((dgSimd&)l0).GetMax((dgSimd&)l1));

				while (stack) {
					stack --;
					const dgNode* const node = stackPool[stack];

					if (ray.BoxTestSimd(node->m_minBox, node->m_maxBox)) {

						//if (!node->m_left) {
						if (node->m_body) {
							dgAssert (!node->m_left);
							dgAssert (!node->m_right);
							if (node->m_body != sentinel) {
								dgFloat32 param = node->m_body->RayCastSimd(line, filter, prefilter, userData, maxParam);
								if (param < maxParam) {
									maxParam = param;
									ray.Reset (maxParam);
								}
							}
						} else {
							dgAssert (node->m_left);
							dgAssert (stack < dgInt32 (sizeof (stackPool) / sizeof (dgNode*)));
							stackPool[stack] = node->m_left;
							stack++;

							dgAssert (node->m_right);
							dgAssert (stack < dgInt32 (sizeof (stackPool) / sizeof (dgNode*)));
							stackPool[stack] = node->m_right;
							stack++;
						}
					}
				}

			} else {
				dgLineBox line;	

				line.m_l0 = l0;
				line.m_l1 = l1;
				if (line.m_l0.m_x <= line.m_l1.m_x) {
					line.m_boxL0.m_x = line.m_l0.m_x;
					line.m_boxL1.m_x = line.m_l1.m_x;
				} else {
					line.m_boxL0.m_x = line.m_l1.m_x;
					line.m_boxL1.m_x = line.m_l0.m_x;
				}

				if (line.m_l0.m_y <= line.m_l1.m_y) {
					line.m_boxL0.m_y = line.m_l0.m_y;
					line.m_boxL1.m_y = line.m_l1.m_y;
				} else {
					line.m_boxL0.m_y = line.m_l1.m_y;
					line.m_boxL1.m_y = line.m_l0.m_y;
				}

				if (line.m_l0.m_z <= line.m_l1.m_z) {
					line.m_boxL0.m_z = line.m_l0.m_z;
					line.m_boxL1.m_z = line.m_l1.m_z;
				} else {
					line.m_boxL0.m_z = line.m_l1.m_z;
					line.m_boxL1.m_z = line.m_l0.m_z;
				}

				while (stack) {

					stack --;
					const dgNode* const node = stackPool[stack];

					if (ray.BoxTest (node->m_minBox, node->m_maxBox)) {
						if (node->m_body) {
							dgAssert (!node->m_left);
							dgAssert (!node->m_right);
							if (node->m_body != sentinel) {
								dgFloat32 param = node->m_body->RayCast (line, filter, prefilter, userData, maxParam);
								if (param < maxParam) {
									maxParam = param;
									ray.Reset (maxParam);
								}
							}
						} else {
							dgAssert (node->m_left);
							dgAssert (stack < dgInt32 (sizeof (stackPool) / sizeof (dgNode*)));
							stackPool[stack] = node->m_left;
							stack++;

							dgAssert (node->m_right);
							dgAssert (stack < dgInt32 (sizeof (stackPool) / sizeof (dgNode*)));
							stackPool[stack] = node->m_right;
							stack++;
						}
					}
				}
			}
		}
	}
}


dgInt32 dgBroadPhase::ConvexCast (dgCollisionInstance* const shape, const dgMatrix& matrix, const dgVector& target, dgFloat32& timeToImpact, OnRayPrecastAction prefilter, void* const userData, dgConvexCastReturnInfo* const info, dgInt32 maxContacts, dgInt32 threadIndex) const
{
	dgVector boxP0;
	dgVector boxP1;
	shape->CalcAABB(matrix, boxP0, boxP1);

	dgInt32 stack = 1;
	dgTriplex points[DG_CONVEX_CAST_POOLSIZE];
	dgTriplex normals[DG_CONVEX_CAST_POOLSIZE];
	dgFloat32 penetration[DG_CONVEX_CAST_POOLSIZE];
	dgInt64 attributeA[DG_CONVEX_CAST_POOLSIZE];
	dgInt64 attributeB[DG_CONVEX_CAST_POOLSIZE];
	dgNode* stackPool[DG_BROADPHASE_MAX_STACK_DEPTH];		

	stackPool[0] = m_rootNode;

	dgInt32 cpu = m_world->m_cpu;
	const dgBody* const sentinel = m_world->GetSentinelBody();

	dgVector velocA(target - matrix.m_posit);
	dgVector velocB(dgFloat32(0.0f), dgFloat32(0.0f), dgFloat32(0.0f), dgFloat32(0.0f));

	dgInt32 totalCount = 0;
	dgFloat32 time = dgFloat32 (1.0f);
	dgFloat32 maxParam = dgFloat32 (1.2f);
	dgFastRayTest ray (dgVector (dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f)), velocA);
	while (stack) {
		stack --;
		const dgNode* const node = stackPool[stack];

		dgVector minBox (node->m_minBox - boxP1);
		dgVector maxBox (node->m_maxBox - boxP0);

		if (ray.BoxTest (minBox, maxBox)) {
			if (node->m_body) {
				dgAssert (!node->m_left);
				dgAssert (!node->m_right);
				if (node->m_body != sentinel) {
					dgBody* const body = node->m_body;
					if (!PREFILTER_RAYCAST (prefilter, body, shape, userData)) {
						dgInt32 count;
						if (cpu == dgSimdPresent) {
							count = m_world->CollideContinueSimd(shape, matrix, velocA, velocB, body->m_collision, body->m_matrix, velocB, velocB, time, points, normals, penetration, attributeA, attributeB, DG_CONVEX_CAST_POOLSIZE, threadIndex);
						} else {
							count = m_world->CollideContinue(shape, matrix, velocA, velocB, body->m_collision, body->m_matrix, velocB, velocB, time, points, normals, penetration, attributeA, attributeB, DG_CONVEX_CAST_POOLSIZE, threadIndex);
						}

						if (count) {
							if (time < maxParam) {
								ray.Reset (time);

								if ((time - maxParam) < dgFloat32(-1.0e-3f)) {
									totalCount = 0;
								}
								maxParam = time;
								if (count >= (maxContacts - totalCount)) {
									count = maxContacts - totalCount;
								}

								for (dgInt32 i = 0; i < count; i++) {
									info[totalCount].m_hitBody = body;
									info[totalCount].m_point[0] = points[i].m_x;
									info[totalCount].m_point[1] = points[i].m_y;
									info[totalCount].m_point[2] = points[i].m_z;
									info[totalCount].m_normal[0] = normals[i].m_x;
									info[totalCount].m_normal[1] = normals[i].m_y;
									info[totalCount].m_normal[2] = normals[i].m_z;
									info[totalCount].m_penetration = penetration[i];
									info[totalCount].m_contaID = attributeB[i];
									totalCount++;
								}
							}
						}
					}
				}

			} else {
				dgAssert (node->m_left);
				dgAssert (stack < dgInt32 (sizeof (stackPool) / sizeof (dgNode*)));
				stackPool[stack] = node->m_left;
				stack++;

				dgAssert (node->m_right);
				dgAssert (stack < dgInt32 (sizeof (stackPool) / sizeof (dgNode*)));
				stackPool[stack] = node->m_right;
				stack++;
			}
		}
	}

	timeToImpact = maxParam;

	return totalCount;
}



void dgBroadPhase::UpdateContacts( dgFloat32 timestep)
{
	dgUnsigned32 ticks = m_world->m_getPerformanceCount();

	dgCollidingPairCollector* const contactPairs = m_world;
	contactPairs->Init();
	m_lru = m_lru + 1;

	dgInt32 threadsCount = m_world->GetThreadCount();	

	dgBroadphaseSyncDescriptor syncPoints(m_broadPhaseType, &m_criticalSectionLock);
//	syncPoints.m_world = m_world;
	syncPoints.m_timestep = timestep;

	
	const dgBodyMasterList* const masterList = m_world;
	const dgCollisionDeformableMeshList* const softBodyList = m_world;

	dgBodyMasterList::dgListNode* const firstBodyNode = masterList->GetFirst()->GetNext();
	syncPoints.m_collindPairBodyNode = firstBodyNode;
	syncPoints.m_forceAndTorqueBodyNode = firstBodyNode;
	syncPoints.m_sofBodyNode = softBodyList->GetFirst();

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
	if (m_broadPhaseType == m_dynamic) {
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

	UpdateContactsBroadPhaseEnd();

	dgUnsigned32 endTicks = m_world->m_getPerformanceCount();
	m_world->m_perfomanceCounters[m_collisionTicks] = endTicks - ticks - m_world->m_perfomanceCounters[m_forceCallbackTicks];

	// update sofbody dynamics phase 1
	for (dgInt32 i = 0; i < threadsCount; i ++) {
		m_world->QueueJob (UpdateSoftBodyForcesKernel, &syncPoints, m_world);
	}
	m_world->SynchronizationBarrier();
	m_world->m_perfomanceCounters[m_softBodyTicks] = m_world->m_getPerformanceCount() - endTicks;
}
