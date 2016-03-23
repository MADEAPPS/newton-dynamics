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
#include "dgBroadPhaseAggregate.h"
#include "dgBroadPhasePersistent.h"


class dgBroadPhasePesistanceRootNode: public dgBroadPhaseTreeNode
{
	public:
	dgBroadPhasePesistanceRootNode()
		:dgBroadPhaseTreeNode()
	{
	}

	virtual bool IsPersistentRoot() const
	{
		return true;
	}

	void SetBox ()
	{
		if (m_right && m_left) {
			dgVector minBox (m_right->m_minBox.GetMin(m_left->m_minBox));
			dgVector maxBox (m_right->m_maxBox.GetMin(m_left->m_maxBox));
			SetAABB(minBox, maxBox);
		} else if (m_right) {
			SetAABB(m_right->m_minBox, m_right->m_maxBox);
		} else if (m_left) {
			SetAABB(m_left->m_minBox, m_left->m_maxBox);
		}
	}
};

dgBroadPhasePersistent::dgBroadPhasePersistent(dgWorld* const world)
	:dgBroadPhase(world)
	,m_staticEntropy(dgFloat32 (0.0f))
	,m_dynamicsEntropy(dgFloat32 (0.0f))
	,m_staticFitness(world->GetAllocator())
	,m_dynamicsFitness(world->GetAllocator())
	,m_staticNeedsUpdate(true)
{
	m_rootNode = new (world->GetAllocator()) dgBroadPhasePesistanceRootNode();
}

dgBroadPhasePersistent::~dgBroadPhasePersistent()
{
	delete m_rootNode;
}

dgInt32 dgBroadPhasePersistent::GetType() const
{
	return dgWorld::m_persistentBroadphase;
}

void dgBroadPhasePersistent::CheckStaticDynamic(dgBody* const body, dgFloat32 mass)
{
	dgBroadPhaseNode* const node = body->GetBroadPhase();
	if (node) {
		dgVector temp (body->GetInvMass());
		if (((mass < (DG_INFINITE_MASS * dgFloat32 (0.9f))) && (temp.m_w == dgFloat32 (0.0f))) || ((mass > (DG_INFINITE_MASS * dgFloat32 (0.5f))) && (temp.m_w != dgFloat32 (0.0f)))) {
			Remove(body);
			body->SetInvMass (dgVector (dgFloat32 (1.0f)));			
			Add(body);
			body->SetInvMass (temp);
		}
	}
}

void dgBroadPhasePersistent::Add(dgBody* const body)
{
	dgAssert (!body->GetCollision()->IsType (dgCollision::dgCollisionNull_RTTI));
	dgBroadPhasePesistanceRootNode* const root = (dgBroadPhasePesistanceRootNode*)m_rootNode;
	dgAssert (m_rootNode->IsPersistentRoot());

	if (body->GetInvMass().m_w == dgFloat32(0.0f)) {
		m_staticNeedsUpdate = true;
		if (root->m_right) {
			dgBroadPhaseTreeNode* const node = InsertNode(root->m_right, new (m_world->GetAllocator()) dgBroadPhaseBodyNode(body));
			node->m_fitnessNode = m_staticFitness.Append(node);
		} else {
			root->m_right = new (m_world->GetAllocator()) dgBroadPhaseBodyNode(body);
			root->m_right->m_parent = m_rootNode;
		}
	} else {
		dgBroadPhaseBodyNode* const newNode = new (m_world->GetAllocator()) dgBroadPhaseBodyNode(body);
		if (root->m_left) {
			dgBroadPhaseTreeNode* const node = InsertNode(root->m_left, newNode);
			node->m_fitnessNode = m_dynamicsFitness.Append(node);
		} else {
			root->m_left = newNode;
			root->m_left->m_parent = m_rootNode;
		}
		newNode->m_updateNode = m_updateList.Append(newNode);
	}
}

dgBroadPhaseAggregate* dgBroadPhasePersistent::CreateAggregate()
{
	dgBroadPhaseAggregate* const aggregate = new (m_world->GetAllocator()) dgBroadPhaseAggregate(m_world->GetBroadPhase());
	LinkAggregate(aggregate);
	return aggregate;
}

void dgBroadPhasePersistent::LinkAggregate(dgBroadPhaseAggregate* const aggregate)
{
	dgAssert(m_rootNode->IsPersistentRoot());
	dgBroadPhasePesistanceRootNode* const root = (dgBroadPhasePesistanceRootNode*)m_rootNode;

	aggregate->m_broadPhase = this;
	if (root->m_left) {
		dgBroadPhaseTreeNode* const node = InsertNode(root->m_left, aggregate);
		node->m_fitnessNode = m_dynamicsFitness.Append(node);
	} else {
		root->m_left = aggregate;
		root->m_left->m_parent = m_rootNode;
	}
	aggregate->m_updateNode = m_updateList.Append(aggregate);
	aggregate->m_myAggregateNode = m_aggregateList.Append(aggregate);
}

void dgBroadPhasePersistent::DestroyAggregate(dgBroadPhaseAggregate* const aggregate)
{
	m_updateList.Remove(aggregate->m_updateNode);
	m_aggregateList.Remove(aggregate->m_myAggregateNode);
	RemoveNode(aggregate);
}

void dgBroadPhasePersistent::RemoveNode(dgBroadPhaseNode* const node)
{
	dgAssert (node->m_parent);

	if (node->m_parent->IsPersistentRoot()) {
		dgBroadPhasePesistanceRootNode* const parent = (dgBroadPhasePesistanceRootNode*)m_rootNode;
		dgAssert(parent == node->m_parent);

		if (parent->m_right == node) {
			m_staticNeedsUpdate = true;
			parent->m_right = NULL;
		} else {
			dgAssert(parent->m_left == node);
			parent->m_left = NULL;
		}
		node->m_parent = NULL;
		delete node;
	} else if (node->m_parent->IsAggregate()) {
		dgBroadPhaseAggregate* const aggregate = (dgBroadPhaseAggregate*)node->m_parent;
		dgBody* const body = node->GetBody();
		dgAssert(body);
		dgAssert(body->GetBroadPhaseAggregate() == aggregate);
		body->SetBroadPhaseAggregate(NULL);
		aggregate->m_root = NULL;
		node->m_parent = NULL;
		delete node;
	} else {
		dgBroadPhaseTreeNode* const parent = (dgBroadPhaseTreeNode*)node->m_parent;
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

			if (parent->m_fitnessNode) {
				dgBody* const body = node->GetBody();
				if (body && body->GetBroadPhaseAggregate()) {
					body->GetBroadPhaseAggregate()->m_fitnessList.Remove(parent->m_fitnessNode);
					body->SetBroadPhaseAggregate(NULL);
				} else {
					m_dynamicsFitness.Remove(parent->m_fitnessNode);
				}
			}

			delete parent;

		} else if (parent->m_parent->IsPersistentRoot()) {
			dgBroadPhasePesistanceRootNode* const grandParent = (dgBroadPhasePesistanceRootNode*) parent->m_parent;
			if (grandParent->m_right == parent) {
				m_staticNeedsUpdate = true;
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
				m_staticFitness.Remove(parent->m_fitnessNode);
				delete parent;
			} else {
				dgAssert (grandParent->m_left == parent);
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
				m_dynamicsFitness.Remove(parent->m_fitnessNode);
				delete parent;
			}
		} else {
			dgBroadPhaseTreeNode* const grandParent = (dgBroadPhaseTreeNode*)parent->m_parent;
			dgAssert (grandParent->GetLeft());
			dgAssert (grandParent->GetRight());
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

			dgBody* const body = node->GetBody();
			if (body) {
				if (body->GetInvMass().m_w == dgFloat32(0.0f)) {
					m_staticNeedsUpdate = true;
					m_staticFitness.Remove(parent->m_fitnessNode);
				} else if (body->GetBroadPhaseAggregate()) {
					body->GetBroadPhaseAggregate()->m_fitnessList.Remove(parent->m_fitnessNode);
					body->SetBroadPhaseAggregate(NULL);
				} else {
					m_dynamicsFitness.Remove(parent->m_fitnessNode);
				}
			} else {
				dgAssert (node->IsAggregate());
				m_dynamicsFitness.Remove(parent->m_fitnessNode);
			}

			delete parent;
		}
	}
}

void dgBroadPhasePersistent::UnlinkAggregate (dgBroadPhaseAggregate* const aggregate)
{
	dgBroadPhasePesistanceRootNode* const root = (dgBroadPhasePesistanceRootNode*)m_rootNode;
	dgAssert (root && root->m_left);
	if (aggregate->m_parent == root) {
		root->m_left = NULL;
	} else {
		dgBroadPhaseTreeNode* const parent = (dgBroadPhaseTreeNode*)aggregate->m_parent;
		dgBroadPhaseTreeNode* const grandParent = (dgBroadPhaseTreeNode*)parent->m_parent;
		if (grandParent->m_left == parent) {
			if (parent->m_left == aggregate) {
				grandParent->m_left = parent->m_right;
				parent->m_right->m_parent = grandParent;
			} else {
				dgAssert(parent->m_right == aggregate);
				grandParent->m_left = parent->m_left;
				parent->m_left->m_parent = grandParent;
			}
		} else {
			dgAssert(grandParent->m_right == parent);
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

void dgBroadPhasePersistent::Remove(dgBody* const body)
{
	if (body->GetBroadPhase()) {
		dgBroadPhaseBodyNode* const node = (dgBroadPhaseBodyNode*)body->GetBroadPhase();
		if (node->m_updateNode) {
			m_updateList.Remove(node->m_updateNode);
		}
		RemoveNode(node);
	}
}

void dgBroadPhasePersistent::ResetEntropy()
{
	m_staticNeedsUpdate = true;
	m_staticEntropy = dgFloat32(0.0f);
	m_dynamicsEntropy = dgFloat32(0.0f);
}


void dgBroadPhasePersistent::InvalidateCache()
{
	ResetEntropy();
	m_staticNeedsUpdate = false;
	dgAssert (m_rootNode->IsPersistentRoot());
	dgBroadPhasePesistanceRootNode* const root = (dgBroadPhasePesistanceRootNode*)m_rootNode;
	ImproveFitness(m_staticFitness, m_staticEntropy, &root->m_right);
	ImproveFitness(m_dynamicsFitness, m_dynamicsEntropy, &root->m_left);
	root->SetBox ();
}

void dgBroadPhasePersistent::UpdateFitness()
{
	dgBroadPhasePesistanceRootNode* const root = (dgBroadPhasePesistanceRootNode*)m_rootNode;
	if (m_staticNeedsUpdate) {
		m_staticNeedsUpdate = false;
		ImproveFitness(m_staticFitness, m_staticEntropy, &root->m_right);
	}
	ImproveFitness(m_dynamicsFitness, m_dynamicsEntropy, &root->m_left);
	root->SetBox ();
}

void dgBroadPhasePersistent::ForEachBodyInAABB(const dgVector& minBox, const dgVector& maxBox, OnBodiesInAABB callback, void* const userData) const
{
	dgBroadPhasePesistanceRootNode* const root = (dgBroadPhasePesistanceRootNode*)m_rootNode;
	const dgBroadPhaseNode* stackPool[DG_BROADPHASE_MAX_STACK_DEPTH];

	dgInt32 stack = 0;
	if (root->m_left) {
		stackPool[stack] = root->m_left;
		stack++;
	}

	if (root->m_right) {
		stackPool[stack] = root->m_right;
		stack++;
	}
	dgBroadPhase::ForEachBodyInAABB(stackPool, stack, minBox, maxBox, callback, userData);
}

void dgBroadPhasePersistent::RayCast(const dgVector& l0, const dgVector& l1, OnRayCastAction filter, OnRayPrecastAction prefilter, void* const userData) const
{
	dgBroadPhasePesistanceRootNode* const root = (dgBroadPhasePesistanceRootNode*)m_rootNode;
	if (filter && (root->m_left || root->m_right)) {
		dgVector segment(l1 - l0);
		dgFloat32 dist2 = segment % segment;
		if (dist2 > dgFloat32(1.0e-8f)) {

			dgFloat32 distance[DG_BROADPHASE_MAX_STACK_DEPTH];
			const dgBroadPhaseNode* stackPool[DG_BROADPHASE_MAX_STACK_DEPTH];

			dgFastRayTest ray(l0, l1);

			dgInt32 stack = 0;
			if (root->m_left) {
				stackPool[stack] = root->m_left;
				distance[stack] = ray.BoxIntersect(root->m_left->m_minBox, root->m_left->m_maxBox);
				stack++;
			}
			if (root->m_right) {
				stackPool[stack] = root->m_right;
				distance[stack] = ray.BoxIntersect(root->m_right->m_minBox, root->m_right->m_maxBox);
				stack++;
			}
			if (stack == 2) {
				if (distance[0] < distance[1]) {
					dgSwap(distance[0], distance[1]);
					dgSwap(stackPool[0], stackPool[1]);
				}
			}

			dgBroadPhase::RayCast(stackPool, distance, stack, l0, l1, ray, filter, prefilter, userData);
		}
	}
}

dgInt32 dgBroadPhasePersistent::ConvexCast(dgCollisionInstance* const shape, const dgMatrix& matrix, const dgVector& target, dgFloat32* const param, OnRayPrecastAction prefilter, void* const userData, dgConvexCastReturnInfo* const info, dgInt32 maxContacts, dgInt32 threadIndex) const
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

		dgInt32 stack = 0;
		dgBroadPhasePesistanceRootNode* const root = (dgBroadPhasePesistanceRootNode*)m_rootNode;
		if (root->m_left) {
			dgVector minBox(root->m_left->m_minBox - boxP1);
			dgVector maxBox(root->m_left->m_maxBox - boxP0);
			stackPool[stack] = root->m_left;
			distance[stack] = ray.BoxIntersect(minBox, maxBox);
			stack++;
		}
		if (root->m_right) {
			dgVector minBox(root->m_right->m_minBox - boxP1);
			dgVector maxBox(root->m_right->m_maxBox - boxP0);

			stackPool[stack] = root->m_right;
			distance[stack] = ray.BoxIntersect(minBox, maxBox);
			stack++;
		}
		if (stack == 2) {
			if (distance[0] < distance[1]) {
				dgSwap(distance[0], distance[1]);
				dgSwap(stackPool[0], stackPool[1]);
			}
		}

		totalCount = dgBroadPhase::ConvexCast(stackPool, distance, 2, velocA, velocB, ray, shape, matrix, target, param, prefilter, userData, info, maxContacts, threadIndex);
	}
	return totalCount;
}

dgInt32 dgBroadPhasePersistent::Collide(dgCollisionInstance* const shape, const dgMatrix& matrix, OnRayPrecastAction prefilter, void* const userData, dgConvexCastReturnInfo* const info, dgInt32 maxContacts, dgInt32 threadIndex) const
{
	dgInt32 totalCount = 0;
	if (m_rootNode) {
		dgVector boxP0;
		dgVector boxP1;
		dgAssert(matrix.TestOrthogonal());
		shape->CalcAABB(matrix, boxP0, boxP1);

		dgInt32 overlaped[DG_BROADPHASE_MAX_STACK_DEPTH];
		const dgBroadPhaseNode* stackPool[DG_BROADPHASE_MAX_STACK_DEPTH];

		dgInt32 stack = 0;
		dgBroadPhasePesistanceRootNode* const root = (dgBroadPhasePesistanceRootNode*)m_rootNode;
		if (dgOverlapTest(m_rootNode->m_minBox, m_rootNode->m_maxBox, boxP0, boxP1)) {
			if (root->m_left) {
				stackPool[stack] = root->m_left;
				overlaped[stack] = dgOverlapTest(root->m_left->m_minBox, root->m_left->m_maxBox, boxP0, boxP1);
				stack ++;
			}
			if (root->m_left) {
				stackPool[stack] = root->m_right;
				overlaped[stack] = dgOverlapTest(root->m_right->m_minBox, root->m_right->m_maxBox, boxP0, boxP1);
				stack++;
			}
			totalCount = dgBroadPhase::Collide(stackPool, overlaped, 1, boxP0, boxP1, shape, matrix, prefilter, userData, info, maxContacts, threadIndex);
		}
	}

	return totalCount;
}