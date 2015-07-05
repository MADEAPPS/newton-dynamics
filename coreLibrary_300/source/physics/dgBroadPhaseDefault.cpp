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
#include "dgCollisionInstance.h"
#include "dgBroadPhaseDefault.h"
#include "dgBroadPhaseAggregate.h"


dgBroadPhaseDefault::dgBroadPhaseDefault(dgWorld* const world)
	:dgBroadPhase(world)
	,m_treeEntropy(dgFloat32(0.0f))
	,m_fitness(world->GetAllocator())
{
}

dgBroadPhaseDefault::~dgBroadPhaseDefault()
{
	if (m_rootNode) {
		delete m_rootNode;
	}
	m_rootNode = NULL;
}

dgInt32 dgBroadPhaseDefault::GetType() const
{
	return dgWorld::m_defaultBroadphase;
}

void dgBroadPhaseDefault::ResetEntropy()
{
	m_treeEntropy = dgFloat32(0.0f);
}

void dgBroadPhaseDefault::UpdateFitness()
{
	ImproveFitness(m_fitness, m_treeEntropy, &m_rootNode);
}

void dgBroadPhaseDefault::InvalidateCache()
{
	ResetEntropy();
	ImproveFitness(m_fitness, m_treeEntropy, &m_rootNode);
}

void dgBroadPhaseDefault::ForEachBodyInAABB(const dgVector& minBox, const dgVector& maxBox, OnBodiesInAABB callback, void* const userData) const
{
	if (m_rootNode) {
		const dgBroadPhaseNode* stackPool[DG_BROADPHASE_MAX_STACK_DEPTH];
		stackPool[0] = m_rootNode;
		dgBroadPhase::ForEachBodyInAABB(stackPool, 1, minBox, maxBox, callback, userData);
	}
}


void dgBroadPhaseDefault::RayCast(const dgVector& l0, const dgVector& l1, OnRayCastAction filter, OnRayPrecastAction prefilter, void* const userData) const
{
	if (filter && m_rootNode) {
		dgVector segment(l1 - l0);
		dgFloat32 dist2 = segment % segment;
		if (dist2 > dgFloat32(1.0e-8f)) {

			dgFloat32 distance[DG_BROADPHASE_MAX_STACK_DEPTH];
			const dgBroadPhaseNode* stackPool[DG_BROADPHASE_MAX_STACK_DEPTH];

			dgFastRayTest ray(l0, l1);

			stackPool[0] = m_rootNode;
			distance[0] = ray.BoxIntersect(m_rootNode->m_minBox, m_rootNode->m_maxBox);
			dgBroadPhase::RayCast(stackPool, distance, 1, l0, l1, ray, filter, prefilter, userData);
		}
	}
}

void dgBroadPhaseDefault::ConvexRayCast(dgCollisionInstance* const shape, const dgMatrix& matrix, const dgVector& target, OnRayCastAction filter, OnRayPrecastAction prefilter, void* const userData, dgInt32 threadId) const
{
	if (filter && m_rootNode && shape->IsType(dgCollision::dgCollisionConvexShape_RTTI)) {
		dgVector boxP0;
		dgVector boxP1;
		shape->CalcAABB(shape->GetLocalMatrix() * matrix, boxP0, boxP1);

		//dgInt32 stack = 1;
		dgFloat32 distance[DG_COMPOUND_STACK_DEPTH];
		const dgBroadPhaseNode* stackPool[DG_BROADPHASE_MAX_STACK_DEPTH];

		dgVector velocA((target - matrix.m_posit) & dgVector::m_triplexMask);
		dgFastRayTest ray(dgVector(dgFloat32(0.0f)), velocA);

		dgVector minBox(m_rootNode->m_minBox - boxP1);
		dgVector maxBox(m_rootNode->m_maxBox - boxP0);
		stackPool[0] = m_rootNode;
		distance[0] = ray.BoxIntersect(minBox, maxBox);

		dgBroadPhase::ConvexRayCast(stackPool, distance, 1, velocA, ray, shape, matrix, target, filter, prefilter, userData, threadId);
	}
}


dgInt32 dgBroadPhaseDefault::ConvexCast(dgCollisionInstance* const shape, const dgMatrix& matrix, const dgVector& target, dgFloat32& timeToImpact, OnRayPrecastAction prefilter, void* const userData, dgConvexCastReturnInfo* const info, dgInt32 maxContacts, dgInt32 threadIndex) const
{
	dgInt32 totalCount = 0;
	if (m_rootNode) {
		dgVector boxP0;
		dgVector boxP1;
		dgAssert(matrix.TestOrthogonal());
		shape->CalcAABB(matrix, boxP0, boxP1);

		dgFloat32 distance[DG_BROADPHASE_MAX_STACK_DEPTH];
		const dgBroadPhaseNode* stackPool[DG_BROADPHASE_MAX_STACK_DEPTH];

		dgVector velocA((target - matrix.m_posit) & dgVector::m_triplexMask);
		dgVector velocB(dgFloat32(0.0f));
		dgFastRayTest ray(dgVector(dgFloat32(0.0f)), velocA);

		dgVector minBox(m_rootNode->m_minBox - boxP1);
		dgVector maxBox(m_rootNode->m_maxBox - boxP0);
		stackPool[0] = m_rootNode;
		distance[0] = ray.BoxIntersect(minBox, maxBox);

		totalCount = dgBroadPhase::ConvexCast(stackPool, distance, 1, velocA, velocB, ray, shape, matrix, target, timeToImpact, prefilter, userData, info, maxContacts, threadIndex);
	}

	return totalCount;
}


void dgBroadPhaseDefault::AddNode(dgBroadPhaseNode* const newNode)
{
	if (!m_rootNode) {
		m_rootNode = newNode;
	} else {
		dgBroadPhaseTreeNode* const node = InsertNode(m_rootNode, newNode);
		node->m_fitnessNode = m_fitness.Append(node);
		if (!node->m_parent) {
			m_rootNode = node;
		}
	}
}

void dgBroadPhaseDefault::Add(dgBody* const body)
{
	// create a new leaf node;
	dgAssert (!body->GetCollision()->IsType (dgCollision::dgCollisionNull_RTTI));
	dgBroadPhaseBodyNode* const newNode = new (m_world->GetAllocator()) dgBroadPhaseBodyNode(body);
	newNode->m_updateNode = m_updateList.Append(newNode);
	AddNode(newNode);
}

dgBroadPhaseAggregate* dgBroadPhaseDefault::CreateAggregate()
{
	dgBroadPhaseAggregate* const aggregate = new (m_world->GetAllocator()) dgBroadPhaseAggregate(m_world->GetBroadPhase());
	LinkAggregate (aggregate);
	return aggregate;
}

void dgBroadPhaseDefault::LinkAggregate(dgBroadPhaseAggregate* const aggregate)
{
	AddNode(aggregate);
	aggregate->m_broadPhase = this;
	aggregate->m_updateNode = m_updateList.Append(aggregate);
	aggregate->m_myAggregateNode = m_aggregateList.Append(aggregate);
}

void dgBroadPhaseDefault::RemoveNode(dgBroadPhaseNode* const node)
{
	if (node->m_parent) {
		if (!node->m_parent->IsAggregate()) {
			dgBroadPhaseTreeNode* const parent = (dgBroadPhaseTreeNode*)node->m_parent;
			if (parent->m_parent) {
				if (parent->m_parent->IsAggregate()) {
					dgBroadPhaseAggregate* const aggregate = (dgBroadPhaseAggregate*)parent->m_parent;
					if (parent->m_left == node) {
						dgAssert(parent->m_right);
						aggregate->m_root = parent->m_right;
						parent->m_right->m_parent = aggregate;
						parent->m_right = NULL;
					} else {
						dgAssert(parent->m_right == node);
						aggregate->m_root = parent->m_left;
						parent->m_left->m_parent = aggregate;
						parent->m_left = NULL;
					}
					parent->m_parent = NULL;
				} else {
					dgBroadPhaseTreeNode* const grandParent = (dgBroadPhaseTreeNode*)parent->m_parent;
					if (grandParent->m_left == parent) {
						if (parent->m_right == node) {
							grandParent->m_left = parent->m_left;
							parent->m_left->m_parent = grandParent;
							parent->m_left = NULL;
							parent->m_parent = NULL;
						} else {
							grandParent->m_left = parent->m_right;
							parent->m_right->m_parent = grandParent;
							parent->m_right = NULL;
							parent->m_parent = NULL;
						}
					} else {
						if (parent->m_right == node) {
							grandParent->m_right = parent->m_left;
							parent->m_left->m_parent = grandParent;
							parent->m_left = NULL;
							parent->m_parent = NULL;
						} else {
							grandParent->m_right = parent->m_right;
							parent->m_right->m_parent = grandParent;
							parent->m_right = NULL;
							parent->m_parent = NULL;
						}
					}
				}
			} else {
				dgAssert(!node->m_parent->IsLeafNode());
				dgBroadPhaseTreeNode* const parent = (dgBroadPhaseTreeNode*)node->m_parent;
				if (parent->m_right == node) {
					m_rootNode = parent->m_left;
					m_rootNode->m_parent = NULL;
					parent->m_left = NULL;
				} else {
					m_rootNode = parent->m_right;
					m_rootNode->m_parent = NULL;
					parent->m_right = NULL;
				}
			}

			if (parent->m_fitnessNode) {
				dgBody* const body = node->GetBody();
				if (body && body->GetBroadPhaseAggregate()) {
					body->GetBroadPhaseAggregate()->m_fitnessList.Remove(parent->m_fitnessNode);
					body->SetBroadPhaseAggregate(NULL);
				} else {
					m_fitness.Remove(parent->m_fitnessNode);
				}
			}
			delete parent;
		} else {
			dgBroadPhaseAggregate* const aggregate = (dgBroadPhaseAggregate*)node->m_parent;
			dgBody* const body = node->GetBody();
			dgAssert (body);
			dgAssert(body->GetBroadPhaseAggregate() == aggregate);
			body->SetBroadPhaseAggregate(NULL);
			aggregate->m_root = NULL;
			node->m_parent = NULL;
			delete node;
		}
	} else {
		delete node;
		m_rootNode = NULL;
	}
}

void dgBroadPhaseDefault::UnlinkAggregate(dgBroadPhaseAggregate* const aggregate)
{
	dgAssert (m_rootNode);
	if (m_rootNode == aggregate) {
		m_rootNode = NULL;
	} else if (aggregate->m_parent == m_rootNode) {
		dgBroadPhaseTreeNode* const parent = (dgBroadPhaseTreeNode*)aggregate->m_parent;
		if (parent->m_left == aggregate) {
			m_rootNode = parent->m_right;
		} else {
			dgAssert(parent->m_right == aggregate);
			m_rootNode = parent->m_left;
		}
		m_rootNode->m_parent = NULL;

		parent->m_left = NULL;
		parent->m_right = NULL;
		parent->m_parent = NULL;
		delete parent;
	} else {
		dgBroadPhaseTreeNode* const parent = (dgBroadPhaseTreeNode*)aggregate->m_parent;
		dgBroadPhaseTreeNode* const grandParent = (dgBroadPhaseTreeNode*)parent->m_parent;
		if (grandParent->m_left == parent) {
			if (parent->m_left == aggregate) {
				grandParent->m_left = parent->m_right;
				parent->m_right->m_parent = grandParent;
			} else {
				dgAssert (parent->m_right == aggregate);
				grandParent->m_left = parent->m_left;
				parent->m_left->m_parent = grandParent;
			}
		} else {
			dgAssert (grandParent->m_right == parent);
			if (parent->m_left == aggregate) {
				grandParent->m_right = parent->m_right;
				parent->m_right->m_parent = grandParent;
			} else {
				dgAssert(parent->m_right == aggregate);
				grandParent->m_right = parent->m_left;
				parent->m_left->m_parent = grandParent;
			}
		}
		parent->m_left = NULL;
		parent->m_right = NULL;
		parent->m_parent = NULL;
		delete parent;
	}
	aggregate->m_parent = NULL;
}


void dgBroadPhaseDefault::Remove(dgBody* const body)
{
	if (body->GetBroadPhase()) {
		dgBroadPhaseBodyNode* const node = (dgBroadPhaseBodyNode*)body->GetBroadPhase();
		if (node->m_updateNode) {
			m_updateList.Remove(node->m_updateNode);
		}
		RemoveNode(node);
	}
}


void dgBroadPhaseDefault::DestroyAggregate(dgBroadPhaseAggregate* const aggregate)
{
	m_updateList.Remove(aggregate->m_updateNode);
	m_aggregateList.Remove(aggregate->m_myAggregateNode);
	RemoveNode(aggregate);
}

