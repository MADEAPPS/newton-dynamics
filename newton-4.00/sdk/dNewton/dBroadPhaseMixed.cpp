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

#include "dNewtonStdafx.h"
#include "dBody.h"
#include "dNewton.h"
#include "dBroadPhaseNode.h"
#include "dBroadPhaseMixed.h"

//#include "dBody.h"
//#include "dgWorld.h"
//#include "dgCollisionInstance.h"
//#include "dBroadPhaseMixed.h"
//#include "dBroadPhaseAggregate.h"

#if 0
dBroadPhaseMixed::dBroadPhaseMixed(dgWorld* const world)
	:dgBroadPhase(world)
	,m_treeEntropy(dgFloat32(0.0f))
	,m_fitness(world->GetAllocator())
{
}

dBroadPhaseMixed::~dBroadPhaseMixed()
{
	if (m_rootNode) {
		delete m_rootNode;
	}
	m_rootNode = nullptr;
}

dgInt32 dBroadPhaseMixed::GetType() const
{
	return dgWorld::m_broadphaseMixed;
}

void dBroadPhaseMixed::ResetEntropy()
{
	m_treeEntropy = dgFloat32(0.0f);
}

void dBroadPhaseMixed::UpdateFitness()
{
	ImproveFitness(m_fitness, m_treeEntropy, &m_rootNode);
}

void dBroadPhaseMixed::InvalidateCache()
{
	ResetEntropy();
	ImproveFitness(m_fitness, m_treeEntropy, &m_rootNode);
	m_contactCache.Flush();
}

void dBroadPhaseMixed::ForEachBodyInAABB(const dgVector& minBox, const dgVector& maxBox, OnBodiesInAABB callback, void* const userData) const
{
	if (m_rootNode) {
		const dBroadPhaseNode* stackPool[DG_BROADPHASE_MAX_STACK_DEPTH];
		stackPool[0] = m_rootNode;
		dgBroadPhase::ForEachBodyInAABB(stackPool, 1, minBox, maxBox, callback, userData);
	}
}


void dBroadPhaseMixed::RayCast(const dgVector& l0, const dgVector& l1, OnRayCastAction filter, OnRayPrecastAction prefilter, void* const userData) const
{
	if (filter && m_rootNode) {
		dgVector segment(l1 - l0);
		dAssert (segment.m_w == dgFloat32 (0.0f));
		dgFloat32 dist2 = segment.DotProduct(segment).GetScalar();
		if (dist2 > dgFloat32(1.0e-8f)) {

			dgFloat32 distance[DG_BROADPHASE_MAX_STACK_DEPTH];
			const dBroadPhaseNode* stackPool[DG_BROADPHASE_MAX_STACK_DEPTH];

			dgFastRayTest ray(l0, l1);

			stackPool[0] = m_rootNode;
			distance[0] = ray.BoxIntersect(m_rootNode->m_minBox, m_rootNode->m_maxBox);
			dgBroadPhase::RayCast(stackPool, distance, 1, l0, l1, ray, filter, prefilter, userData);
		}
	}
}

dgInt32 dBroadPhaseMixed::ConvexCast(dgCollisionInstance* const shape, const dgMatrix& matrix, const dgVector& target, dgFloat32* const param, OnRayPrecastAction prefilter, void* const userData, dgConvexCastReturnInfo* const info, dgInt32 maxContacts, dgInt32 threadIndex) const
{
	dgInt32 totalCount = 0;
	if (m_rootNode) {
		dgVector boxP0;
		dgVector boxP1;
		dAssert(matrix.TestOrthogonal());
		shape->CalcAABB(matrix, boxP0, boxP1);

		dgFloat32 distance[DG_BROADPHASE_MAX_STACK_DEPTH];
		const dBroadPhaseNode* stackPool[DG_BROADPHASE_MAX_STACK_DEPTH];

		dgVector velocA((target - matrix.m_posit) & dgVector::m_triplexMask);
		dgVector velocB(dgFloat32(0.0f));
		dgFastRayTest ray(dgVector(dgFloat32(0.0f)), velocA);

		dgVector minBox(m_rootNode->m_minBox - boxP1);
		dgVector maxBox(m_rootNode->m_maxBox - boxP0);
		stackPool[0] = m_rootNode;
		distance[0] = ray.BoxIntersect(minBox, maxBox);

		*param = dgFloat32 (1.0f);
		totalCount = dgBroadPhase::ConvexCast(stackPool, distance, 1, velocA, velocB, ray, shape, matrix, target, param, prefilter, userData, info, maxContacts, threadIndex);
	}

	return totalCount;
}

dgInt32 dBroadPhaseMixed::Collide(dgCollisionInstance* const shape, const dgMatrix& matrix, OnRayPrecastAction prefilter, void* const userData, dgConvexCastReturnInfo* const info, dgInt32 maxContacts, dgInt32 threadIndex) const
{
	dgInt32 totalCount = 0;
	if (m_rootNode) {
		dgVector boxP0;
		dgVector boxP1;
		dAssert(matrix.TestOrthogonal());
		shape->CalcAABB(shape->GetLocalMatrix() * matrix, boxP0, boxP1);

		dgInt32 overlaped[DG_BROADPHASE_MAX_STACK_DEPTH];
		const dBroadPhaseNode* stackPool[DG_BROADPHASE_MAX_STACK_DEPTH];

		stackPool[0] = m_rootNode;
		overlaped[0] = dgOverlapTest(m_rootNode->m_minBox, m_rootNode->m_maxBox, boxP0, boxP1);

		totalCount = dgBroadPhase::Collide(stackPool, overlaped, 1, boxP0, boxP1, shape, matrix, prefilter, userData, info, maxContacts, threadIndex);
	}

	return totalCount;
}

void dBroadPhaseMixed::AddNode(dBroadPhaseNode* const newNode)
{
	if (!m_rootNode) {
		m_rootNode = newNode;
	} else {
		dBroadPhaseTreeNode* const node = InsertNode(m_rootNode, newNode);
		node->m_fitnessNode = m_fitness.Append(node);
		if (!node->m_parent) {
			m_rootNode = node;
		}
	}
}

void dBroadPhaseMixed::Add(dBody* const body)
{
	// create a new leaf node;
	dAssert (!body->GetCollision()->IsType (dgCollision::dgCollisionNull_RTTI));
	dBroadPhaseBodyNode* const bodyNode = new (m_world->GetAllocator()) dBroadPhaseBodyNode(body);
	bodyNode->m_updateNode = m_updateList.Append(bodyNode);
	AddNode(bodyNode);
}

dBroadPhaseAggregate* dBroadPhaseMixed::CreateAggregate()
{
	dBroadPhaseAggregate* const aggregate = new (m_world->GetAllocator()) dBroadPhaseAggregate(m_world->GetBroadPhase());
	LinkAggregate (aggregate);
	return aggregate;
}

void dBroadPhaseMixed::LinkAggregate(dBroadPhaseAggregate* const aggregate)
{
	AddNode(aggregate);
	aggregate->m_broadPhase = this;
	aggregate->m_updateNode = m_updateList.Append(aggregate);
	aggregate->m_myAggregateNode = m_aggregateList.Append(aggregate);
}


void dBroadPhaseMixed::UnlinkAggregate(dBroadPhaseAggregate* const aggregate)
{
	dAssert (m_rootNode);
	if (m_rootNode == aggregate) {
		m_rootNode = nullptr;
	} else if (aggregate->m_parent == m_rootNode) {
		dBroadPhaseTreeNode* const parent = (dBroadPhaseTreeNode*)aggregate->m_parent;
		if (parent->m_left == aggregate) {
			m_rootNode = parent->m_right;
		} else {
			dAssert(parent->m_right == aggregate);
			m_rootNode = parent->m_left;
		}
		m_rootNode->m_parent = nullptr;

		parent->m_left = nullptr;
		parent->m_right = nullptr;
		parent->m_parent = nullptr;
		delete parent;
	} else {
		dBroadPhaseTreeNode* const parent = (dBroadPhaseTreeNode*)aggregate->m_parent;
		dBroadPhaseTreeNode* const grandParent = (dBroadPhaseTreeNode*)parent->m_parent;
		if (grandParent->m_left == parent) {
			if (parent->m_left == aggregate) {
				grandParent->m_left = parent->m_right;
				parent->m_right->m_parent = grandParent;
			} else {
				dAssert (parent->m_right == aggregate);
				grandParent->m_left = parent->m_left;
				parent->m_left->m_parent = grandParent;
			}
		} else {
			dAssert (grandParent->m_right == parent);
			if (parent->m_left == aggregate) {
				grandParent->m_right = parent->m_right;
				parent->m_right->m_parent = grandParent;
			} else {
				dAssert(parent->m_right == aggregate);
				grandParent->m_right = parent->m_left;
				parent->m_left->m_parent = grandParent;
			}
		}
		parent->m_left = nullptr;
		parent->m_right = nullptr;
		parent->m_parent = nullptr;
		delete parent;
	}
	aggregate->m_parent = nullptr;
}

void dBroadPhaseMixed::DestroyAggregate(dBroadPhaseAggregate* const aggregate)
{
	m_updateList.Remove(aggregate->m_updateNode);
	m_aggregateList.Remove(aggregate->m_myAggregateNode);
	RemoveNode(aggregate);
}

void dBroadPhaseMixed::FindCollidingPairs(dgBroadphaseSyncDescriptor* const descriptor, dgList<dBroadPhaseNode*>::dgListNode* const nodePtr, dgInt32 threadID)
{
	DG_TRACKTIME();
	const dgFloat32 timestep = descriptor->m_timestep;

	dgList<dBroadPhaseNode*>::dgListNode* node = nodePtr;
	const dgInt32 threadCount = descriptor->m_world->GetThreadCount();

	if (descriptor->m_fullScan) {
		while (node) {
			dBroadPhaseNode* const broadPhaseNode = node->GetInfo();
			dAssert(broadPhaseNode->IsLeafNode());

			dBody* const body = broadPhaseNode->GetBody();
			dAssert(!body || (body->GetBroadPhase() == broadPhaseNode));
			
			if (!(body && body->m_isdead)) {
				if (broadPhaseNode->IsAggregate()) {
					((dBroadPhaseAggregate*)broadPhaseNode)->SubmitSelfPairs(timestep, threadID);
				}

				for (dBroadPhaseNode* ptr = broadPhaseNode; ptr->m_parent; ptr = ptr->m_parent) {
					dBroadPhaseTreeNode* const parent = (dBroadPhaseTreeNode*)ptr->m_parent;
					dAssert(!parent->IsLeafNode());
					dBroadPhaseNode* const sibling = parent->m_right;
					if (sibling != ptr) {
						SubmitPairs(broadPhaseNode, sibling, timestep, 0, threadID);
					}
				}
			}
	
			for (dgInt32 i = 0; i < threadCount; i++) {
				node = node ? node->GetNext() : nullptr;
			}
		}

	} else {
		const dBodyInfo* const bodyArray = &m_world->m_bodiesMemory[0];
		const dgInt32 bodyCount = descriptor->m_atomicPendingBodiesCount;
		dgInt32* const atomicIndex = &descriptor->m_atomicIndex;

		for (dgInt32 i = dgAtomicExchangeAndAdd(atomicIndex, 1); i < bodyCount; i = dgAtomicExchangeAndAdd(atomicIndex, 1)) {
			dBroadPhaseNode* const broadPhaseNode = bodyArray[i].m_body->GetBroadPhase();
			dAssert(broadPhaseNode->IsLeafNode());
			dAssert(!broadPhaseNode->GetBody() || (broadPhaseNode->GetBody()->GetBroadPhase() == broadPhaseNode));

			for (dBroadPhaseNode* ptr = broadPhaseNode; ptr->m_parent; ptr = ptr->m_parent) {
				dBroadPhaseTreeNode* const parent = (dBroadPhaseTreeNode*)ptr->m_parent;
				if (!parent->IsAggregate()) {
					dAssert(!parent->IsLeafNode());
					dBroadPhaseNode* const rightSibling = parent->m_right;
					if (rightSibling != ptr) {
						SubmitPairs(broadPhaseNode, rightSibling, timestep, threadCount, threadID);
					} else {
						SubmitPairs(broadPhaseNode, parent->m_left, timestep, threadCount, threadID);
					}
				}
			}
		}
	}
}
#endif

dBroadPhaseMixed::dBroadPhaseMixed(dNewton* const world)
	:dBroadPhase(world)
{
}

dBroadPhaseMixed::~dBroadPhaseMixed()
{
}

void dBroadPhaseMixed::AddNode(dBroadPhaseNode* const newNode)
{
	if (m_rootNode) 
	{
		dBroadPhaseTreeNode* const node = InsertNode(m_rootNode, newNode);
	//	node->m_fitnessNode = m_fitness.Append(node);
		if (!node->m_parent) 
		{
			m_rootNode = node;
		}
	}
	else
	{
		m_rootNode = newNode;
	}
}

void dBroadPhaseMixed::RemoveNode(dBroadPhaseNode* const node)
{
	if (node->m_parent) 
	{
		if (!node->m_parent->GetAsBroadPhaseAggregate()) 
		{
			dBroadPhaseTreeNode* const parent = (dBroadPhaseTreeNode*)node->m_parent;
			if (parent->m_parent) 
			{
				dBroadPhaseAggregate* const aggregate = parent->m_parent->GetAsBroadPhaseAggregate();
				if (aggregate)
				{
					dAssert(0);
#if 0
					if (parent->m_left == node) 
					{
						dAssert(parent->m_right);
						aggregate->m_root = parent->m_right;
						parent->m_right->m_parent = aggregate;
						parent->m_right = nullptr;
					}
					else 
					{
						dAssert(parent->m_right == node);
						aggregate->m_root = parent->m_left;
						parent->m_left->m_parent = aggregate;
						parent->m_left = nullptr;
					}
					parent->m_parent = nullptr;
#endif
				}
				else 
				{
					dBroadPhaseTreeNode* const grandParent = (dBroadPhaseTreeNode*)parent->m_parent;
					if (grandParent->m_left == parent) 
					{
						if (parent->m_right == node) 
						{
							grandParent->m_left = parent->m_left;
							parent->m_left->m_parent = grandParent;
							parent->m_left = nullptr;
							parent->m_parent = nullptr;
						}
						else 
						{
							grandParent->m_left = parent->m_right;
							parent->m_right->m_parent = grandParent;
							parent->m_right = nullptr;
							parent->m_parent = nullptr;
						}
					}
					else 
					{
						if (parent->m_right == node) 
						{
							grandParent->m_right = parent->m_left;
							parent->m_left->m_parent = grandParent;
							parent->m_left = nullptr;
							parent->m_parent = nullptr;
						}
						else 
						{
							grandParent->m_right = parent->m_right;
							parent->m_right->m_parent = grandParent;
							parent->m_right = nullptr;
							parent->m_parent = nullptr;
						}
					}
				}
			}
			else 
			{
				dAssert(!node->m_parent->GetAsBroadPhaseBodyNode());
				dBroadPhaseTreeNode* const parent1 = node->m_parent->GetAsBroadPhaseTreeNode();
				if (parent1->m_right == node) 
				{
					m_rootNode = parent1->m_left;
					m_rootNode->m_parent = nullptr;
					parent1->m_left = nullptr;
				}
				else 
				{
					m_rootNode = parent1->m_right;
					m_rootNode->m_parent = nullptr;
					parent1->m_right = nullptr;
				}
			}

			//if (parent->m_fitnessNode) 
			//{
			//	dBody* const body = node->GetBody();
			//	if (body && body->GetBroadPhaseAggregate()) 
			//	{
			//		body->GetBroadPhaseAggregate()->m_fitnessList.Remove(parent->m_fitnessNode);
			//		body->SetBroadPhaseAggregate(nullptr);
			//	}
			//	else 
			//	{
			//		m_fitness.Remove(parent->m_fitnessNode);
			//	}
			//}
			delete parent;
		}
		else 
		{
			dAssert(0);
#if 0
			dBroadPhaseAggregate* const aggregate = (dBroadPhaseAggregate*)node->m_parent;
			dBody* const body = node->GetBody();
			dAssert(body);
			dAssert(body->GetBroadPhaseAggregate() == aggregate);
			body->SetBroadPhaseAggregate(nullptr);
			aggregate->m_root = nullptr;
			node->m_parent = nullptr;
			delete node;
#endif
		}
	}
	else 
	{
		delete node;
		m_rootNode = nullptr;
	}
}


void dBroadPhaseMixed::AddBody(dBody* const body)
{
	body->UpdateCollisionMatrix();
	dBroadPhaseBodyNode* const bodyNode = new dBroadPhaseBodyNode(body);
//	bodyNode->m_updateNode = m_updateList.Append(bodyNode);
	AddNode(bodyNode);
}

void dBroadPhaseMixed::RemoveBody(dBody* const body)
{
	dBroadPhaseBodyNode* const node = body->GetBroadPhaseNode();
	if (node)
	{
		//if (node->m_updateNode) 
		//{
		//	m_updateList.Remove(node->m_updateNode);
		//}
		RemoveNode(node);
	}
}
