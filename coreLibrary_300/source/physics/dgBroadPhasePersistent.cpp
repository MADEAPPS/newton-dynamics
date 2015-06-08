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
#include "dgBroadPhasePersistent.h"

#if 0
/*

void dgBroadPhasePersistent::ConvexRayCast (dgCollisionInstance* const shape, const dgMatrix& matrix, const dgVector& target, OnRayCastAction filter, OnRayPrecastAction prefilter, void* const userData, dgInt32 threadId) const
{
	if (filter && shape->IsType(dgCollision::dgCollisionConvexShape_RTTI) && (m_rootNode->m_left || m_rootNode->m_right)) {

		dgVector boxP0;
		dgVector boxP1;
		shape->CalcAABB(shape->GetLocalMatrix() * matrix, boxP0, boxP1);

		
		dgFloat32 distance[DG_BROADPHASE_MAX_STACK_DEPTH];
		const dgNode* stackPool[DG_BROADPHASE_MAX_STACK_DEPTH];		

		dgVector velocA((target - matrix.m_posit) & dgVector::m_triplexMask);
		dgFloat32 maxParam = dgFloat32 (1.02f);
		dgFastRayTest ray (dgVector (dgFloat32 (0.0f)), velocA);
		dgFloat32 quantizeStep = dgMax (dgFloat32 (1.0f) / velocA.DotProduct4(velocA).m_x, dgFloat32 (0.001f));

		//dgVector minBox (m_rootNode->m_minBox - boxP1);
		//dgVector maxBox (m_rootNode->m_maxBox - boxP0);
		//stackPool[0] = m_rootNode;
		//distance[0] = ray.BoxIntersect(minBox, maxBox);
		//dgInt32 stack = 1; 
		 
		dgInt32 stack = 0;
		if (m_rootNode->m_left) {
			dgVector minBox(m_rootNode->m_left->m_minBox - boxP1);
			dgVector maxBox(m_rootNode->m_left->m_maxBox - boxP0);

			stackPool[stack] = m_rootNode->m_left;
			distance[stack] = ray.BoxIntersect(minBox, maxBox);
			stack++;
		}
		if (m_rootNode->m_right) {
			dgVector minBox(m_rootNode->m_right->m_minBox - boxP1);
			dgVector maxBox(m_rootNode->m_right->m_maxBox - boxP0);

			stackPool[stack] = m_rootNode->m_right;
			distance[stack] = ray.BoxIntersect(minBox, maxBox);
			stack++;
		}


		while (stack) {
			stack --;
			dgFloat32 dist = distance[stack];
			if (dist > maxParam) {
				break;
			} else {
				const dgNode* const me = stackPool[stack];
				dgAssert (me);
				if (me->m_body) {
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


dgInt32 dgBroadPhasePersistent::ConvexCast (dgCollisionInstance* const shape, const dgMatrix& matrix, const dgVector& target, dgFloat32& timeToImpact, OnRayPrecastAction prefilter, void* const userData, dgConvexCastReturnInfo* const info, dgInt32 maxContacts, dgInt32 threadIndex) const
{
	dgInt32 totalCount = 0;
	if (m_rootNode->m_left || m_rootNode->m_right) {
		dgVector boxP0;
		dgVector boxP1;
		dgAssert (matrix.TestOrthogonal());
		shape->CalcAABB(matrix, boxP0, boxP1);

		
		dgTriplex points[DG_CONVEX_CAST_POOLSIZE];
		dgTriplex normals[DG_CONVEX_CAST_POOLSIZE];
		dgFloat32 penetration[DG_CONVEX_CAST_POOLSIZE];
		dgInt64 attributeA[DG_CONVEX_CAST_POOLSIZE];
		dgInt64 attributeB[DG_CONVEX_CAST_POOLSIZE];

		dgFloat32 distance[DG_BROADPHASE_MAX_STACK_DEPTH];
		const dgNode* stackPool[DG_BROADPHASE_MAX_STACK_DEPTH];		
		
		dgVector velocA((target - matrix.m_posit) & dgVector::m_triplexMask);
		dgVector velocB(dgFloat32(0.0f));

		dgFloat32 time = dgFloat32 (1.0f);
		dgFloat32 maxParam = dgFloat32 (1.2f);
		dgFastRayTest ray (dgVector (dgFloat32 (0.0f)), velocA);

		dgInt32 stack = 0;
		if (m_rootNode->m_left) {
			dgVector minBox (m_rootNode->m_left->m_minBox - boxP1);
			dgVector maxBox (m_rootNode->m_left->m_maxBox - boxP0);

			stackPool[stack] = m_rootNode->m_left;
			distance[stack] = ray.BoxIntersect(minBox, maxBox);
			stack++;
		}
		if (m_rootNode->m_right) {
			dgVector minBox(m_rootNode->m_right->m_minBox - boxP1);
			dgVector maxBox(m_rootNode->m_right->m_maxBox - boxP0);

			stackPool[stack] = m_rootNode->m_right;
			distance[stack] = ray.BoxIntersect(minBox, maxBox);
			stack++;
		}

		if (stack == 2) {
			if (distance[0] < distance[1]) {
				dgSwap(distance[0], distance[1]);
				dgSwap(stackPool[0], stackPool[1]);
			}
		}

		while (stack) {
			stack --;

			dgFloat32 dist = distance[stack];

			if (dist > maxParam) {
				break;
			} else {
				const dgNode* const me = stackPool[stack];
				if (me->m_body) {
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
*/

#endif


dgBroadPhasePersistent::dgBroadPhasePersistent(dgWorld* const world)
	:dgBroadPhase(world)
	,m_staticEntropy(dgFloat32 (0.0f))
	,m_dynamicsEntropy(dgFloat32 (0.0f))
	,m_staticFitness(world->GetAllocator())
	,m_dynamicsFitness(world->GetAllocator())
	,m_staticNeedsUpdate(true)
{
	m_rootNode = new (world->GetAllocator()) dgBroadPhaseNode;
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
		if (((mass != dgFloat32 (0.0f)) && (temp.m_w == dgFloat32 (0.0f))) || ((mass == dgFloat32 (0.0f)) && (temp.m_w != dgFloat32 (0.0f)))) {
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
	if (body->GetInvMass().m_w == dgFloat32(0.0f)) {
		m_staticNeedsUpdate = true;
		if (m_rootNode->m_right) {
			dgBroadPhaseNode* const node = InsertNode(m_rootNode->m_right, new (m_world->GetAllocator()) dgBroadPhaseNode(body));
			node->m_fitnessNode = m_staticFitness.Append(node);
		} else {
			m_rootNode->m_right = new (m_world->GetAllocator()) dgBroadPhaseNode(body);
			m_rootNode->m_right->m_parent = m_rootNode;
		}
	} else {
		if (m_rootNode->m_left) {
			dgBroadPhaseNode* const node = InsertNode(m_rootNode->m_left, new (m_world->GetAllocator()) dgBroadPhaseNode(body));
			node->m_fitnessNode = m_dynamicsFitness.Append(node);
		} else {
			m_rootNode->m_left = new (m_world->GetAllocator()) dgBroadPhaseNode(body);
			m_rootNode->m_left->m_parent = m_rootNode;
		}
	}
}


void dgBroadPhasePersistent::Remove(dgBody* const body)
{
	dgBroadPhaseNode* const node = body->GetBroadPhase();
	if (node) {
		dgAssert(node->m_parent);
		dgAssert(!node->m_fitnessNode);

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

			dgAssert(node->m_parent->m_fitnessNode);
			if (body->GetInvMass().m_w == dgFloat32(0.0f)) {
				m_staticNeedsUpdate = true;
				m_staticFitness.Remove(node->m_parent->m_fitnessNode);
			} else {
				m_dynamicsFitness.Remove(node->m_parent->m_fitnessNode);
			}
			delete node->m_parent;
		} else {
			if (node->m_parent->m_right == node) {
				m_rootNode->m_right = NULL;
			} else {
				m_rootNode->m_left = NULL;
			}
			node->m_parent = NULL;
			delete node;
		}
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
	ImproveFitness(m_staticFitness, m_staticEntropy, &m_rootNode->m_right);
	ImproveFitness(m_dynamicsFitness, m_dynamicsEntropy, &m_rootNode->m_left);
}

void dgBroadPhasePersistent::UpdateFitness()
{
	if (m_staticNeedsUpdate) {
		m_staticNeedsUpdate = false;
		ImproveFitness(m_staticFitness, m_staticEntropy, &m_rootNode->m_right);
	}
	ImproveFitness(m_dynamicsFitness, m_dynamicsEntropy, &m_rootNode->m_left);
}

void dgBroadPhasePersistent::ForEachBodyInAABB(const dgVector& minBox, const dgVector& maxBox, OnBodiesInAABB callback, void* const userData) const
{
	const dgBroadPhaseNode* stackPool[DG_BROADPHASE_MAX_STACK_DEPTH];

	dgInt32 stack = 0;
	if (m_rootNode->m_left) {
		stackPool[stack] = m_rootNode->m_left;
		stack++;
	}

	if (m_rootNode->m_right) {
		stackPool[stack] = m_rootNode->m_right;
		stack++;
	}
	dgBroadPhase::ForEachBodyInAABB(stackPool, stack, minBox, maxBox, callback, userData);
}

void dgBroadPhasePersistent::RayCast(const dgVector& l0, const dgVector& l1, OnRayCastAction filter, OnRayPrecastAction prefilter, void* const userData) const
{
	if (filter && (m_rootNode->m_left || m_rootNode->m_right)) {
		dgVector segment(l1 - l0);
		dgFloat32 dist2 = segment % segment;
		if (dist2 > dgFloat32(1.0e-8f)) {

			dgFloat32 distance[DG_BROADPHASE_MAX_STACK_DEPTH];
			const dgBroadPhaseNode* stackPool[DG_BROADPHASE_MAX_STACK_DEPTH];

			dgFastRayTest ray(l0, l1);

			dgInt32 stack = 0;
			if (m_rootNode->m_left) {
				stackPool[stack] = m_rootNode->m_left;
				distance[stack] = ray.BoxIntersect(m_rootNode->m_left->m_minBox, m_rootNode->m_left->m_maxBox);
				stack++;
			}
			if (m_rootNode->m_right) {
				stackPool[stack] = m_rootNode->m_right;
				distance[stack] = ray.BoxIntersect(m_rootNode->m_right->m_minBox, m_rootNode->m_right->m_maxBox);
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

void dgBroadPhasePersistent::ConvexRayCast(dgCollisionInstance* const shape, const dgMatrix& matrix, const dgVector& target, OnRayCastAction filter, OnRayPrecastAction prefilter, void* const userData, dgInt32 threadId) const
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

		//dgVector minBox(m_rootNode->m_minBox - boxP1);
		//dgVector maxBox(m_rootNode->m_maxBox - boxP0);
		//stackPool[0] = m_rootNode;
		//distance[0] = ray.BoxIntersect(minBox, maxBox);

		dgInt32 stack = 0;
		if (m_rootNode->m_left) {
			dgVector minBox(m_rootNode->m_left->m_minBox - boxP1);
			dgVector maxBox(m_rootNode->m_left->m_maxBox - boxP0);
			stackPool[stack] = m_rootNode->m_left;
			distance[stack] = ray.BoxIntersect(minBox, maxBox);
			stack++;
		}
		if (m_rootNode->m_right) {
			dgVector minBox(m_rootNode->m_right->m_minBox - boxP1);
			dgVector maxBox(m_rootNode->m_right->m_maxBox - boxP0);

			stackPool[stack] = m_rootNode->m_right;
			distance[stack] = ray.BoxIntersect(minBox, maxBox);
			stack++;
		}
		if (stack == 2) {
			if (distance[0] < distance[1]) {
				dgSwap(distance[0], distance[1]);
				dgSwap(stackPool[0], stackPool[1]);
			}
		}

		dgBroadPhase::ConvexRayCast(stackPool, distance, stack, velocA, ray, shape, matrix, target, filter, prefilter, userData, threadId);
	}
}

dgInt32 dgBroadPhasePersistent::ConvexCast(dgCollisionInstance* const shape, const dgMatrix& matrix, const dgVector& target, dgFloat32& timeToImpact, OnRayPrecastAction prefilter, void* const userData, dgConvexCastReturnInfo* const info, dgInt32 maxContacts, dgInt32 threadIndex) const
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

		//dgVector minBox(m_rootNode->m_minBox - boxP1);
		//dgVector maxBox(m_rootNode->m_maxBox - boxP0);
		//stackPool[0] = m_rootNode;
		//distance[0] = ray.BoxIntersect(minBox, maxBox);

		dgInt32 stack = 0;
		if (m_rootNode->m_left) {
			dgVector minBox(m_rootNode->m_left->m_minBox - boxP1);
			dgVector maxBox(m_rootNode->m_left->m_maxBox - boxP0);
			stackPool[stack] = m_rootNode->m_left;
			distance[stack] = ray.BoxIntersect(minBox, maxBox);
			stack++;
		}
		if (m_rootNode->m_right) {
			dgVector minBox(m_rootNode->m_right->m_minBox - boxP1);
			dgVector maxBox(m_rootNode->m_right->m_maxBox - boxP0);

			stackPool[stack] = m_rootNode->m_right;
			distance[stack] = ray.BoxIntersect(minBox, maxBox);
			stack++;
		}
		if (stack == 2) {
			if (distance[0] < distance[1]) {
				dgSwap(distance[0], distance[1]);
				dgSwap(stackPool[0], stackPool[1]);
			}
		}

		totalCount = dgBroadPhase::ConvexCast(stackPool, distance, 2, velocA, velocB, ray, shape, matrix, target, timeToImpact, prefilter, userData, info, maxContacts, threadIndex);
	}

	return totalCount;
}

void dgBroadPhasePersistent::FindCollidingPairs(dgBroadphaseSyncDescriptor* const descriptor, dgBodyMasterList::dgListNode* node, dgInt32 threadID)
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
					SubmitPairs(broadPhaseNode, sibling, timestep, threadID);
				}
			}
		}
		for (dgInt32 i = 0; i < threadCount; i++) {
			node = (node && (node->GetPrev()->GetInfo().GetBody()->GetInvMass().m_w != dgFloat32(0.0f))) ? node->GetPrev() : NULL;
		}
	}
}


void dgBroadPhasePersistent::ScanForContactJoints(dgBroadphaseSyncDescriptor& syncPoints)
{
	dgInt32 threadsCount = m_world->GetThreadCount();
	const dgBodyMasterList* const masterList = m_world;
	dgBodyMasterList::dgListNode* node = (masterList->GetLast()->GetInfo().GetBody()->GetInvMass().m_w != dgFloat32(0.0f)) ? masterList->GetLast() : NULL;
	for (dgInt32 i = 0; i < threadsCount; i++) {
		m_world->QueueJob(CollidingPairsKernel, &syncPoints, node);
		node = (node && (node->GetPrev()->GetInfo().GetBody()->GetInvMass().m_w != NULL)) ? node->GetPrev() : NULL;
	}
	m_world->SynchronizationBarrier();
}