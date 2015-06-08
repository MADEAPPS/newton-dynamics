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


void dgBroadPhaseDefault::Add(dgBody* const body)
{
	dgAssert (!body->GetCollision()->IsType (dgCollision::dgCollisionNull_RTTI));
	// create a new leaf node;
	dgBroadPhaseNode* const newNode = new (m_world->GetAllocator()) dgBroadPhaseNode(body);

	if (!m_rootNode) {
		m_rootNode = newNode;
	} else {
		dgBroadPhaseNode* const node = InsertNode(m_rootNode, newNode);
		node->m_fitnessNode = m_fitness.Append(node);
		if (!node->m_parent) {
			m_rootNode = node;
		}
	}
}

void dgBroadPhaseDefault::Remove(dgBody* const body)
{
	if (body->GetBroadPhase()) {
		dgBroadPhaseNode* const node = body->GetBroadPhase();

		dgAssert(!node->m_fitnessNode);

		if (node->m_parent) {
			dgBroadPhaseNode* const grandParent = node->m_parent->m_parent;
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

			dgAssert(node->m_parent->m_fitnessNode);
			m_fitness.Remove(node->m_parent->m_fitnessNode);
			delete node->m_parent;
		} else {
			delete node;
			m_rootNode = NULL;
		}
	}
}

void dgBroadPhaseDefault::FindCollidingPairs (dgBroadphaseSyncDescriptor* const descriptor, dgBodyMasterList::dgListNode* node, dgInt32 threadID)
{
	const dgFloat32 timestep = descriptor->m_timestep;
	
	const dgInt32 threadCount = descriptor->m_world->GetThreadCount();
	while (node) {
		dgBody* const body = node->GetInfo().GetBody();
		dgBroadPhaseNode* const broadPhaseNode = body->GetBroadPhase();
		if (broadPhaseNode) {
			for (dgBroadPhaseNode* ptr = broadPhaseNode; ptr->m_parent; ptr = ptr->m_parent) {
				dgBroadPhaseNode* const sibling = ptr->m_parent->m_right;
				if (sibling != ptr) {
					SubmitPairs (broadPhaseNode, sibling, timestep, threadID);
				}
			}
		}
		for (dgInt32 i = 0; i < threadCount; i++) {
			node = node ? node->GetNext() : NULL;
		}
	}
}

void dgBroadPhaseDefault::ScanForContactJoints(dgBroadphaseSyncDescriptor& syncPoints)
{
	dgInt32 threadsCount = m_world->GetThreadCount();
	const dgBodyMasterList* const masterList = m_world;
	dgBodyMasterList::dgListNode* node = masterList->GetFirst()->GetNext();
	for (dgInt32 i = 0; i < threadsCount; i++) {
		m_world->QueueJob(CollidingPairsKernel, &syncPoints, node);
		node = node ? node->GetNext() : NULL;
	}
	m_world->SynchronizationBarrier();
}