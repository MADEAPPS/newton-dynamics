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
#include "dNewton.h"
#include "dBroadPhase.h"

//#include "dgBody.h"
//#include "dgWorld.h"
//#include "dgContact.h"
//#include "dBroadPhase.h"
//#include "dgDynamicBody.h"
//#include "dgCollisionConvex.h"
//#include "dgCollisionInstance.h"
//#include "dgWorldDynamicUpdate.h"
//#include "dgBilateralConstraint.h"
//#include "dBroadPhaseAggregate.h"
//#include "dgCollisionLumpedMassParticles.h"
//#include "dgCollisionLumpedMassParticles.h"

//#define DG_CONVEX_CAST_POOLSIZE			32
//#define DG_BROADPHASE_AABB_SCALE		dFloat32 (8.0f)
//#define DG_BROADPHASE_AABB_INV_SCALE	(dFloat32 (1.0f) / DG_BROADPHASE_AABB_SCALE)
//#define DG_CONTACT_TRANSLATION_ERROR	dFloat32 (1.0e-3f)
//#define DG_CONTACT_ANGULAR_ERROR		(dFloat32 (0.25f * dgDegreeToRad))
//#define DG_NARROW_PHASE_DIST			dFloat32 (0.2f)
//#define DG_CONTACT_DELAY_FRAMES			4

//#define DG_USE_OLD_SCANNER

//dVector dBroadPhase::m_velocTol(dFloat32(1.0e-16f)); 
//dVector dBroadPhase::m_angularContactError2(DG_CONTACT_ANGULAR_ERROR * DG_CONTACT_ANGULAR_ERROR);
//dVector dBroadPhase::m_linearContactError2(DG_CONTACT_TRANSLATION_ERROR * DG_CONTACT_TRANSLATION_ERROR);
// 
//dVector dBroadPhaseNode::m_broadPhaseScale (DG_BROADPHASE_AABB_SCALE, DG_BROADPHASE_AABB_SCALE, DG_BROADPHASE_AABB_SCALE, dFloat32 (0.0f));
//dVector dBroadPhaseNode::m_broadInvPhaseScale (DG_BROADPHASE_AABB_INV_SCALE, DG_BROADPHASE_AABB_INV_SCALE, DG_BROADPHASE_AABB_INV_SCALE, dFloat32 (0.0f));

//class dBroadPhase::dgSpliteInfo
//{
//	public:
//	dgSpliteInfo (dBroadPhaseNode** const boxArray, dgInt32 boxCount)
//	{
//		dVector minP ( dFloat32 (1.0e15f)); 
//		dVector maxP (-dFloat32 (1.0e15f)); 
//
//		if (boxCount == 2) {
//			m_axis = 1;
//			for (dgInt32 i = 0; i < boxCount; i ++) {
//				dBroadPhaseNode* const node = boxArray[i];
//				dgAssert (node->IsLeafNode());
//				minP = minP.GetMin (node->m_minBox); 
//				maxP = maxP.GetMax (node->m_maxBox); 
//			}
//		} else {
//			dVector median (dFloat32 (0.0f));
//			dVector varian (dFloat32 (0.0f));
//			for (dgInt32 i = 0; i < boxCount; i ++) {
//				dBroadPhaseNode* const node = boxArray[i];
//				dgAssert (node->IsLeafNode());
//				minP = minP.GetMin (node->m_minBox); 
//				maxP = maxP.GetMax (node->m_maxBox); 
//				dVector p (dVector::m_half * (node->m_minBox + node->m_maxBox));
//				median += p;
//				varian += p * p;
//			}
//
//			varian = varian.Scale (dFloat32 (boxCount)) - median * median;
//
//			dgInt32 index = 0;
//			dFloat32 maxVarian = dFloat32 (-1.0e10f);
//			for (dgInt32 i = 0; i < 3; i ++) {
//				if (varian[i] > maxVarian) {
//					index = i;
//					maxVarian = varian[i];
//				}
//			}
//
//			dVector center = median.Scale (dFloat32 (1.0f) / dFloat32 (boxCount));
//
//			dFloat32 test = center[index];
//
//			dgInt32 i0 = 0;
//			dgInt32 i1 = boxCount - 1;
//			do {    
//				for (; i0 <= i1; i0 ++) {
//					dBroadPhaseNode* const node = boxArray[i0];
//					dFloat32 val = (node->m_minBox[index] + node->m_maxBox[index]) * dFloat32 (0.5f);
//					if (val > test) {
//						break;
//					}
//				}
//
//				for (; i1 >= i0; i1 --) {
//					dBroadPhaseNode* const node = boxArray[i1];
//					dFloat32 val = (node->m_minBox[index] + node->m_maxBox[index]) * dFloat32 (0.5f);
//					if (val < test) {
//						break;
//					}
//				}
//
//				if (i0 < i1)	{
//					dgSwap(boxArray[i0], boxArray[i1]);
//					i0++; 
//					i1--;
//				}
//
//			} while (i0 <= i1);
//
//			if (i0 > 0){
//				i0 --;
//			}
//			if ((i0 + 1) >= boxCount) {
//				i0 = boxCount - 2;
//			}
//			m_axis = i0 + 1;
//		}
//
//		dgAssert (maxP.m_x - minP.m_x >= dFloat32 (0.0f));
//		dgAssert (maxP.m_y - minP.m_y >= dFloat32 (0.0f));
//		dgAssert (maxP.m_z - minP.m_z >= dFloat32 (0.0f));
//		m_p0 = minP;
//		m_p1 = maxP;
//	}
//
//	dgInt32 m_axis;
//	dVector m_p0;
//	dVector m_p1;
//};

#if 0
dBroadPhase::dBroadPhase(dgWorld* const world)
	:m_world(world)
	,m_rootNode(NULL)
	,m_generatedBodies(world->GetAllocator())
	,m_updateList(world->GetAllocator())
	,m_aggregateList(world->GetAllocator())
	,m_lru(DG_CONTACT_DELAY_FRAMES)
	,m_contactCache(world->GetAllocator())
	,m_pendingSoftBodyCollisions(world->GetAllocator(), 64)
	,m_pendingSoftBodyPairsCount(0)
	,m_criticalSectionLock(0)
{
}

dBroadPhase::~dBroadPhase()
{
}


void dBroadPhase::MoveNodes (dBroadPhase* const dst)
{
	const dgBodyMasterList* const masterList = m_world;
	for (dgBodyMasterList::dgListNode* node = masterList->GetFirst(); node; node = node->GetNext()) {
		dgBody* const body = node->GetInfo().GetBody();
		if (body->GetBroadPhase() && !body->GetBroadPhaseAggregate()) {
			Remove(body);
			dst->Add(body);
		}
	}

	dgList<dBroadPhaseAggregate*>::dgListNode* next;
	for (dgList<dBroadPhaseAggregate*>::dgListNode* ptr = m_aggregateList.GetFirst(); ptr; ptr = next) {
		next = ptr->GetNext();
		dBroadPhaseAggregate* const aggregate = ptr->GetInfo();
		m_aggregateList.Remove (aggregate->m_myAggregateNode);
		m_updateList.Remove(aggregate->m_updateNode);
		aggregate->m_updateNode = NULL;
		aggregate->m_myAggregateNode = NULL;
		UnlinkAggregate(aggregate);
		dst->LinkAggregate(aggregate);
	}
}

dBroadPhaseTreeNode* dBroadPhase::InsertNode(dBroadPhaseNode* const root, dBroadPhaseNode* const node)
{
	dVector p0;
	dVector p1;

	dBroadPhaseNode* sibling = root;
	dFloat32 surfaceArea = CalculateSurfaceArea(node, sibling, p0, p1);
	while (!sibling->IsLeafNode()) {

		if (surfaceArea > sibling->m_surfaceArea) {
			break;
		}

		sibling->m_minBox = p0;
		sibling->m_maxBox = p1;
		sibling->m_surfaceArea = surfaceArea;

		dVector leftP0;
		dVector leftP1;
		dFloat32 leftSurfaceArea = CalculateSurfaceArea(node, sibling->GetLeft(), leftP0, leftP1);

		dVector rightP0;
		dVector rightP1;
		dFloat32 rightSurfaceArea = CalculateSurfaceArea(node, sibling->GetRight(), rightP0, rightP1);

		if (leftSurfaceArea < rightSurfaceArea) {
			sibling = sibling->GetLeft();
			p0 = leftP0;
			p1 = leftP1;
			surfaceArea = leftSurfaceArea;
		} else {
			sibling = sibling->GetRight();
			p0 = rightP0;
			p1 = rightP1;
			surfaceArea = rightSurfaceArea;
		}
	}

	dBroadPhaseTreeNode* const parent = new (m_world->GetAllocator()) dBroadPhaseTreeNode(sibling, node);
	return parent;
}

void dBroadPhase::UpdateAggregateEntropyKernel(void* const context, void* const node, dgInt32 threadID)
{
	D_TRACKTIME();
	dgBroadphaseSyncDescriptor* const descriptor = (dgBroadphaseSyncDescriptor*)context;
	dgWorld* const world = descriptor->m_world;
	dBroadPhase* const broadPhase = world->GetBroadPhase();
	broadPhase->UpdateAggregateEntropy(descriptor, (dgList<dBroadPhaseAggregate*>::dgListNode*) node, threadID);
}

void dBroadPhase::ForceAndToqueKernel(void* const context, void* const node, dgInt32 threadID)
{
	D_TRACKTIME();
	dgBroadphaseSyncDescriptor* const descriptor = (dgBroadphaseSyncDescriptor*)context;
	dgWorld* const world = descriptor->m_world;
	dBroadPhase* const broadPhase = world->GetBroadPhase();
	broadPhase->ApplyForceAndtorque(descriptor, (dgBodyMasterList::dgListNode*) node, threadID);
}

void dBroadPhase::SleepingStateKernel(void* const context, void* const node, dgInt32 threadID)
{
	D_TRACKTIME();
	dgBroadphaseSyncDescriptor* const descriptor = (dgBroadphaseSyncDescriptor*)context;
	dgWorld* const world = descriptor->m_world;
	dBroadPhase* const broadPhase = world->GetBroadPhase();
	broadPhase->SleepingState(descriptor, (dgBodyMasterList::dgListNode*) node, threadID);
}

bool dBroadPhase::DoNeedUpdate(dgBodyMasterList::dgListNode* const node) const
{
	dgBody* const body = node->GetInfo().GetBody();

	bool state = body->GetInvMass().m_w != dFloat32 (0.0f);
	state = state || !body->m_equilibrium || (body->GetExtForceAndTorqueCallback() != NULL);
	return state;
}

void dBroadPhase::UpdateAggregateEntropy (dgBroadphaseSyncDescriptor* const descriptor, dgList<dBroadPhaseAggregate*>::dgListNode* node, dgInt32 threadID)
{
	DG_TRACKTIME();
	const dgInt32 threadCount = m_world->GetThreadCount();
	while (node) {
		node->GetInfo()->ImproveEntropy();
		for (dgInt32 i = 0; i < threadCount; i++) {
			node = node ? node->GetNext() : NULL;
		}
	}
}

void dBroadPhase::ApplyForceAndtorque(dgBroadphaseSyncDescriptor* const descriptor, dgBodyMasterList::dgListNode* node, dgInt32 threadID)
{
	dFloat32 timestep = descriptor->m_timestep;

	const dgInt32 threadCount = m_world->GetThreadCount();
	while (node) {
		dgBody* const body = node->GetInfo().GetBody();
		body->InitJointSet();
		if (DoNeedUpdate(node)) {
			if (body->IsRTTIType(dgBody::m_dynamicBodyRTTI)) {
				dgDynamicBody* const dynamicBody = (dgDynamicBody*)body;
				dynamicBody->ApplyExtenalForces(timestep, threadID);
			}
		}

		for (dgInt32 i = 0; i < threadCount; i++) {
			node = node ? node->GetNext() : NULL;
		}
	}
}

void dBroadPhase::SleepingState(dgBroadphaseSyncDescriptor* const descriptor, dgBodyMasterList::dgListNode* node, dgInt32 threadID)
{
	DG_TRACKTIME();
	dFloat32 timestep = descriptor->m_timestep;

	const dgInt32 threadCount = m_world->GetThreadCount();
	dgBodyInfo* const pendingBodies = &m_world->m_bodiesMemory[0];

	dgInt32* const atomicBodiesCount = &descriptor->m_atomicDynamicsCount;
	dgInt32* const atomicPendingBodiesCount = &descriptor->m_atomicPendingBodiesCount;

	while (node) {
		if (DoNeedUpdate(node)) {
			dgBody* const body = node->GetInfo().GetBody();

			if (body->IsRTTIType(dgBody::m_dynamicBodyRTTI)) {
				dgDynamicBody* const dynamicBody = (dgDynamicBody*)body;

				if (!dynamicBody->m_equilibrium && (dynamicBody->GetInvMass().m_w == dFloat32(0.0f))) {
					descriptor->m_fullScan = true;
				}
				if (dynamicBody->GetInvMass().m_w) {
					dgAtomicExchangeAndAdd(atomicBodiesCount, 1);
				}

				if (dynamicBody->GetInvMass().m_w == dFloat32(0.0f) || body->m_collision->IsType(dgCollision::dgCollisionMesh_RTTI)) {
					dynamicBody->m_sleeping = true;
					dynamicBody->m_autoSleep = true;
					dynamicBody->m_equilibrium = true;
				}

				if (dynamicBody->IsInEquilibrium()) {
					dynamicBody->m_equilibrium = true;
					dynamicBody->m_sleeping = dynamicBody->m_autoSleep;
				} else {
					dynamicBody->m_sleeping = false;
					dynamicBody->m_equilibrium = false;
					if (dynamicBody->GetBroadPhase()) {
						dynamicBody->UpdateCollisionMatrix(timestep, threadID);
						dgInt32 pendingBodyIndex = dgAtomicExchangeAndAdd(atomicPendingBodiesCount, 1);
						pendingBodies[pendingBodyIndex].m_body = dynamicBody;
					}
				}

				dynamicBody->m_savedExternalForce = dynamicBody->m_externalForce;
				dynamicBody->m_savedExternalTorque = dynamicBody->m_externalTorque;
			} else {
				dgAssert(body->IsRTTIType(dgBody::m_kinematicBodyRTTI));

				// kinematic bodies are always sleeping (skip collision with kinematic bodies)
				bool isResting = (body->m_omega.DotProduct(body->m_omega).GetScalar() < dFloat32 (1.0e-6f)) && (body->m_veloc.DotProduct(body->m_veloc).GetScalar() < dFloat32(1.0e-4f));
				if (body->IsCollidable()) {
					body->m_sleeping = false;
					body->m_autoSleep = false;
				} else {
					body->m_autoSleep = true;
					body->m_sleeping = isResting;
					descriptor->m_fullScan = !isResting;
				}
				body->m_equilibrium = isResting;

				// update collision matrix by calling the transform callback for all kinematic bodies
				if (body->GetBroadPhase()) {
					body->UpdateCollisionMatrix(timestep, threadID);
				}
			}
		}

		for (dgInt32 i = 0; i < threadCount; i++) {
			node = node ? node->GetNext() : NULL;
		}
	}
}


void dBroadPhase::ForEachBodyInAABB(const dBroadPhaseNode** stackPool, dgInt32 stack, const dVector& minBox, const dVector& maxBox, OnBodiesInAABB callback, void* const userData) const
{
	while (stack) {
		stack--;
		const dBroadPhaseNode* const rootNode = stackPool[stack];
		if (dgOverlapTest(rootNode->m_minBox, rootNode->m_maxBox, minBox, maxBox)) {

			dgBody* const body = rootNode->GetBody();
			if (body) {
				if (!body->m_isdead && dgOverlapTest(body->m_minAABB, body->m_maxAABB, minBox, maxBox)) {
					if (!callback(body, userData)) {
						break;
					}
				}
			} else if (rootNode->IsAggregate()) {
				dBroadPhaseAggregate* const aggregate = (dBroadPhaseAggregate*)rootNode;
				if (aggregate->m_root) {
					stackPool[stack] = aggregate->m_root;
					stack++;
					dgAssert(stack < DG_BROADPHASE_MAX_STACK_DEPTH);
				}

			} else {
				dgAssert (!rootNode->IsLeafNode());
				dBroadPhaseTreeNode* const node = (dBroadPhaseTreeNode*)rootNode;
				stackPool[stack] = node->m_left;
				stack++;
				dgAssert(stack < DG_BROADPHASE_MAX_STACK_DEPTH);

				stackPool[stack] = node->m_right;
				stack++;
				dgAssert(stack < DG_BROADPHASE_MAX_STACK_DEPTH);
			}
		}
	}
}

dgInt32 dBroadPhase::ConvexCast(const dBroadPhaseNode** stackPool, dFloat32* const distance, dgInt32 stack, const dVector& velocA, const dVector& velocB, dgFastRayTest& ray,
								 dgCollisionInstance* const shape, const dgMatrix& matrix, const dVector& target, dFloat32* const param, OnRayPrecastAction prefilter, void* const userData, dgConvexCastReturnInfo* const info, dgInt32 maxContacts, dgInt32 threadIndex) const
{
	dVector boxP0;
	dVector boxP1;
	dgTriplex points[DG_CONVEX_CAST_POOLSIZE];
	dgTriplex normals[DG_CONVEX_CAST_POOLSIZE];
	dFloat32 penetration[DG_CONVEX_CAST_POOLSIZE];
	dgInt64 attributeA[DG_CONVEX_CAST_POOLSIZE];
	dgInt64 attributeB[DG_CONVEX_CAST_POOLSIZE];
	dgInt32 totalCount = 0;

	dgAssert(matrix.TestOrthogonal());
	shape->CalcAABB(matrix, boxP0, boxP1);

	maxContacts = dgMin (maxContacts, DG_CONVEX_CAST_POOLSIZE);
	dgAssert (!maxContacts || (maxContacts && info));
	dFloat32 maxParam = *param;
	dFloat32 timeToImpact = *param;
	while (stack) {
		stack--;

		dFloat32 dist = distance[stack];

		if (dist > maxParam) {
			break;
		} else {
			const dBroadPhaseNode* const me = stackPool[stack];

			dgBody* const body = me->GetBody();
			if (body) {
				if (!body->m_isdead && !PREFILTER_RAYCAST(prefilter, body, body->m_collision, userData)) {
					dgInt32 count = m_world->CollideContinue(shape, matrix, velocA, velocB, body->m_collision, body->m_matrix, velocB, velocB, timeToImpact, points, normals, penetration, attributeA, attributeB, maxContacts, threadIndex);

					if (timeToImpact < maxParam) {
						if ((timeToImpact - maxParam) < dFloat32(-1.0e-3f)) {
							totalCount = 0;
						}
						maxParam = timeToImpact;
						if (count >= (maxContacts - totalCount)) {
							count = maxContacts - totalCount;
						}

						for (dgInt32 i = 0; i < count; i++) {
							info[totalCount].m_point[0] = points[i].m_x;
							info[totalCount].m_point[1] = points[i].m_y;
							info[totalCount].m_point[2] = points[i].m_z;
							info[totalCount].m_point[3] = dFloat32(0.0f);
							info[totalCount].m_normal[0] = normals[i].m_x;
							info[totalCount].m_normal[1] = normals[i].m_y;
							info[totalCount].m_normal[2] = normals[i].m_z;
							info[totalCount].m_normal[3] = dFloat32(0.0f);
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
			} else if (me->IsAggregate()) {
				dBroadPhaseAggregate* const aggregate = (dBroadPhaseAggregate*)me;
				const dBroadPhaseNode* const node = aggregate->m_root;
				if (node) {
					dVector minBox(node->m_minBox - boxP1);
					dVector maxBox(node->m_maxBox - boxP0);
					dFloat32 dist1 = ray.BoxIntersect(minBox, maxBox);
					if (dist1 < maxParam) {
						dgInt32 j = stack;
						for (; j && (dist1 > distance[j - 1]); j--) {
							stackPool[j] = stackPool[j - 1];
							distance[j] = distance[j - 1];
						}
						stackPool[j] = node;
						distance[j] = dist1;
						stack++;
						dgAssert(stack < DG_BROADPHASE_MAX_STACK_DEPTH);
					}
				}

			} else {
				dBroadPhaseTreeNode* const node = (dBroadPhaseTreeNode*)me;
				const dBroadPhaseNode* const left = node->m_left;
				dgAssert(left);
				dVector minBox(left->m_minBox - boxP1);
				dVector maxBox(left->m_maxBox - boxP0);
				dFloat32 dist1 = ray.BoxIntersect(minBox, maxBox);
				if (dist1 < maxParam) {
					dgInt32 j = stack;
					for (; j && (dist1 > distance[j - 1]); j--) {
						stackPool[j] = stackPool[j - 1];
						distance[j] = distance[j - 1];
					}
					stackPool[j] = left;
					distance[j] = dist1;
					stack++;
					dgAssert(stack < DG_BROADPHASE_MAX_STACK_DEPTH);
				}

				const dBroadPhaseNode* const right = node->m_right;
				dgAssert(right);
				minBox = right->m_minBox - boxP1;
				maxBox = right->m_maxBox - boxP0;
				dist1 = ray.BoxIntersect(minBox, maxBox);
				if (dist1 < maxParam) {
					dgInt32 j = stack;
					for (; j && (dist1 > distance[j - 1]); j--) {
						stackPool[j] = stackPool[j - 1];
						distance[j] = distance[j - 1];
					}
					stackPool[j] = right;
					distance[j] = dist1;
					stack++;
					dgAssert(stack < DG_BROADPHASE_MAX_STACK_DEPTH);
				}
			}
		}
	}
	*param = maxParam;
	return totalCount;
}

dgInt32 dBroadPhase::Collide(const dBroadPhaseNode** stackPool, dgInt32* const ovelapStack, dgInt32 stack, const dVector& boxP0, const dVector& boxP1, dgCollisionInstance* const shape, const dgMatrix& matrix, OnRayPrecastAction prefilter, void* const userData, dgConvexCastReturnInfo* const info, dgInt32 maxContacts, dgInt32 threadIndex) const
{
	dgTriplex points[DG_CONVEX_CAST_POOLSIZE];
	dgTriplex normals[DG_CONVEX_CAST_POOLSIZE];
	dFloat32 penetration[DG_CONVEX_CAST_POOLSIZE];
	dgInt64 attributeA[DG_CONVEX_CAST_POOLSIZE];
	dgInt64 attributeB[DG_CONVEX_CAST_POOLSIZE];

	dgInt32 totalCount = 0;
	while (stack) {
		stack--;

		dgInt32 test = ovelapStack[stack];
		if (test) {
			const dBroadPhaseNode* const me = stackPool[stack];

			dgBody* const body = me->GetBody();
			if (body) {
				if (!body->m_isdead && !PREFILTER_RAYCAST(prefilter, body, body->m_collision, userData)) {
					dgInt32 count = m_world->Collide(shape, matrix, body->m_collision, body->m_matrix, points, normals, penetration, attributeA, attributeB, DG_CONVEX_CAST_POOLSIZE, threadIndex);

					if (count) {
						bool teminate = false;
						if (count >= (maxContacts - totalCount)) {
							count = maxContacts - totalCount;
							teminate = true;
						}

						for (dgInt32 i = 0; i < count; i++) {
							info[totalCount].m_point[0] = points[i].m_x;
							info[totalCount].m_point[1] = points[i].m_y;
							info[totalCount].m_point[2] = points[i].m_z;
							info[totalCount].m_point[3] = dFloat32(0.0f);
							info[totalCount].m_normal[0] = normals[i].m_x;
							info[totalCount].m_normal[1] = normals[i].m_y;
							info[totalCount].m_normal[2] = normals[i].m_z;
							info[totalCount].m_normal[3] = dFloat32(0.0f);
							info[totalCount].m_penetration = penetration[i];
							info[totalCount].m_contaID = attributeB[i];
							info[totalCount].m_hitBody = body;
							totalCount++;
						}

						if (teminate) {
							break;
						}
					}
				}
			} else if (me->IsAggregate()) {
				dBroadPhaseAggregate* const aggregate = (dBroadPhaseAggregate*)me;
				const dBroadPhaseNode* const node = aggregate->m_root;
				if (node) {
					stackPool[stack] = node;
					ovelapStack[stack] = dgOverlapTest(node->m_minBox, node->m_maxBox, boxP0, boxP1);
					stack++;
					dgAssert(stack < DG_BROADPHASE_MAX_STACK_DEPTH);
				}

			} else {
				dBroadPhaseTreeNode* const node = (dBroadPhaseTreeNode*)me;
				const dBroadPhaseNode* const left = node->m_left;
				stackPool[stack] = left;
				ovelapStack[stack] = dgOverlapTest(left->m_minBox, left->m_maxBox, boxP0, boxP1);
				stack ++;
				dgAssert(stack < DG_BROADPHASE_MAX_STACK_DEPTH);

				const dBroadPhaseNode* const right = node->m_right;
				stackPool[stack] = right;
				ovelapStack[stack] = dgOverlapTest(right->m_minBox, right->m_maxBox, boxP0, boxP1);
				stack++;
				dgAssert(stack < DG_BROADPHASE_MAX_STACK_DEPTH);
			}
		}
	}
	return totalCount;
}

void dBroadPhase::RayCast(const dBroadPhaseNode** stackPool, dFloat32* const distance, dgInt32 stack, const dVector& l0, const dVector& l1, dgFastRayTest& ray, OnRayCastAction filter, OnRayPrecastAction prefilter, void* const userData) const
{
	dgLineBox line;
	line.m_l0 = l0;
	line.m_l1 = l1;
	dVector test(line.m_l0 <= line.m_l1);

	dFloat32 maxParam = dFloat32 (1.2f);

	//line.m_boxL0 = (line.m_l0 & test) | line.m_l1.AndNot(test);
	//line.m_boxL1 = (line.m_l1 & test) | line.m_l0.AndNot(test);
	line.m_boxL0 = line.m_l1.Select(line.m_l0, test);
	line.m_boxL1 = line.m_l0.Select(line.m_l1, test);

	while (stack) {
		stack--;
		dFloat32 dist = distance[stack];
		if (dist > maxParam) {
			break;
		} else {
			const dBroadPhaseNode* const me = stackPool[stack];
			dgAssert(me);
			dgBody* const body = me->GetBody();
			if (body) {
				if (!body->m_isdead) {
					dgAssert(!me->GetLeft());
					dgAssert(!me->GetRight());
					dFloat32 param = body->RayCast(line, filter, prefilter, userData, maxParam);
					if (param < maxParam) {
						maxParam = param;
						if (maxParam < dFloat32(1.0e-8f)) {
							break;
						}
					}
				}
			} else if (me->IsAggregate()) {
				dBroadPhaseAggregate* const aggregate = (dBroadPhaseAggregate*) me;
				if (aggregate->m_root) {
					const dBroadPhaseNode* const child = aggregate->m_root;
					dgAssert(child);
					dFloat32 dist1 = ray.BoxIntersect(child->m_minBox, child->m_maxBox);
					if (dist1 < maxParam) {
						dgInt32 j = stack;
						for (; j && (dist1 > distance[j - 1]); j--) {
							stackPool[j] = stackPool[j - 1];
							distance[j] = distance[j - 1];
						}
						stackPool[j] = child;
						distance[j] = dist1;
						stack++;
						dgAssert(stack < DG_BROADPHASE_MAX_STACK_DEPTH);
					}
				}
			} else {
				const dBroadPhaseNode* const left = me->GetLeft();
				dgAssert(left);
				dFloat32 dist1 = ray.BoxIntersect(left->m_minBox, left->m_maxBox);
				if (dist1 < maxParam) {
					dgInt32 j = stack;
					for (; j && (dist1 > distance[j - 1]); j--) {
						stackPool[j] = stackPool[j - 1];
						distance[j] = distance[j - 1];
					}
					stackPool[j] = left;
					distance[j] = dist1;
					stack++;
					dgAssert(stack < DG_BROADPHASE_MAX_STACK_DEPTH);
				}

				const dBroadPhaseNode* const right = me->GetRight();
				dgAssert(right);
				dist1 = ray.BoxIntersect(right->m_minBox, right->m_maxBox);
				if (dist1 < maxParam) {
					dgInt32 j = stack;
					for (; j && (dist1 > distance[j - 1]); j--) {
						stackPool[j] = stackPool[j - 1];
						distance[j] = distance[j - 1];
					}
					stackPool[j] = right;
					distance[j] = dist1;
					stack++;
					dgAssert(stack < DG_BROADPHASE_MAX_STACK_DEPTH);
				}
			}
		}
	}
}

void dBroadPhase::CollisionChange (dgBody* const body, dgCollisionInstance* const collision)
{
	dgCollisionInstance* const bodyCollision = body->GetCollision();
	if (bodyCollision) {
		if (bodyCollision->IsType (dgCollision::dgCollisionNull_RTTI) && !collision->IsType (dgCollision::dgCollisionNull_RTTI)) {
			dgAssert (!body->GetBroadPhase());
			body->m_collision = m_world->m_pointCollision;
			Add (body);
			body->m_collision = bodyCollision;
		} else if (!bodyCollision->IsType (dgCollision::dgCollisionNull_RTTI) && collision->IsType (dgCollision::dgCollisionNull_RTTI)) {
			Remove(body);
		}
	}
}

void dBroadPhase::UpdateBody(dgBody* const body, dgInt32 threadIndex)
{
	if (m_rootNode && body->m_masterNode) {
		dBroadPhaseBodyNode* const node = body->GetBroadPhase();
		dgBody* const body1 = node->GetBody();
		dgAssert(body1 == body);
		dgAssert(!body1->m_equilibrium);
		dgAssert(!node->GetLeft());
		dgAssert(!node->GetRight());
		dgAssert(!body1->GetCollision()->IsType(dgCollision::dgCollisionNull_RTTI));

		if (body1->GetBroadPhaseAggregate()) {
			dBroadPhaseAggregate* const aggregate = body1->GetBroadPhaseAggregate();
			dgScopeSpinPause lock(&aggregate->m_criticalSectionLock);
			aggregate->m_isInEquilibrium = body1->m_equilibrium;
		}
		
		if (!dgBoxInclusionTest(body1->m_minAABB, body1->m_maxAABB, node->m_minBox, node->m_maxBox)) {
			dgAssert(!node->IsAggregate());
			node->SetAABB(body1->m_minAABB, body1->m_maxAABB);

			if (!m_rootNode->IsLeafNode()) {
				const dBroadPhaseNode* const root = (m_rootNode->GetLeft() && m_rootNode->GetRight()) ? NULL : m_rootNode;
				for (dBroadPhaseNode* parent = node->m_parent; parent != root; parent = parent->m_parent) {
					dgScopeSpinPause lock(&parent->m_criticalSectionLock);
					if (!parent->IsAggregate()) {
						dVector minBox;
						dVector maxBox;
						dFloat32 area = CalculateSurfaceArea(parent->GetLeft(), parent->GetRight(), minBox, maxBox);
						if (dgBoxInclusionTest(minBox, maxBox, parent->m_minBox, parent->m_maxBox)) {
							break;
						}
						parent->m_minBox = minBox;
						parent->m_maxBox = maxBox;
						parent->m_surfaceArea = area;
					} else {
						dBroadPhaseAggregate* const aggregate = (dBroadPhaseAggregate*)parent;
						aggregate->m_minBox = aggregate->m_root->m_minBox;
						aggregate->m_maxBox = aggregate->m_root->m_maxBox;
						aggregate->m_surfaceArea = aggregate->m_root->m_surfaceArea;
					}
				}
			}
		}
	}
}

dBroadPhaseNode* dBroadPhase::BuildTopDown(dBroadPhaseNode** const leafArray, dgInt32 firstBox, dgInt32 lastBox, dgFitnessList::dgListNode** const nextNode)
{
	dgAssert(firstBox >= 0);
	dgAssert(lastBox >= 0);

	if (lastBox == firstBox) {
		return leafArray[firstBox];
	} else {
		dgSpliteInfo info(&leafArray[firstBox], lastBox - firstBox + 1);

		dBroadPhaseTreeNode* const parent = (*nextNode)->GetInfo();
		parent->m_parent = NULL;
		*nextNode = (*nextNode)->GetNext();

		parent->SetAABB(info.m_p0, info.m_p1);

		parent->m_left = BuildTopDown(leafArray, firstBox, firstBox + info.m_axis - 1, nextNode);
		parent->m_left->m_parent = parent;

		parent->m_right = BuildTopDown(leafArray, firstBox + info.m_axis, lastBox, nextNode);
		parent->m_right->m_parent = parent;
		return parent;
	}
}


dBroadPhaseNode* dBroadPhase::BuildTopDownBig(dBroadPhaseNode** const leafArray, dgInt32 firstBox, dgInt32 lastBox, dgFitnessList::dgListNode** const nextNode)
{
	if (lastBox == firstBox) {
		return BuildTopDown(leafArray, firstBox, lastBox, nextNode);
	}

	dgInt32 midPoint = -1;
	const dFloat32 scale = dFloat32 (1.0f / 64.0f);
	const dBroadPhaseNode* const node0 = leafArray[firstBox];
	const dgInt32 count = lastBox - firstBox;
	dFloat32 area0 = scale * node0->m_surfaceArea;
	for (dgInt32 i = 1; i <= count; i++) {
		const dBroadPhaseNode* const node1 = leafArray[firstBox + i];
		dFloat32 area1 = node1->m_surfaceArea;
		if (area0 > area1) {
			midPoint = i - 1;
			break;
		}
	}

	if (midPoint == -1) {
		return BuildTopDown(leafArray, firstBox, lastBox, nextNode);
	} else {
		dBroadPhaseTreeNode* const parent = (*nextNode)->GetInfo();

		parent->m_parent = NULL;
		*nextNode = (*nextNode)->GetNext();

		parent->m_right = BuildTopDown(leafArray, firstBox, firstBox + midPoint, nextNode);
		parent->m_right->m_parent = parent;

		parent->m_left = BuildTopDownBig(leafArray, firstBox + midPoint + 1, lastBox, nextNode);
		parent->m_left->m_parent = parent;

		dVector minP (parent->m_left->m_minBox.GetMin(parent->m_right->m_minBox));
		dVector maxP (parent->m_left->m_maxBox.GetMax(parent->m_right->m_maxBox));
		parent->SetAABB(minP, maxP);

		return parent;
	}
}


dgInt32 dBroadPhase::CompareNodes(const dBroadPhaseNode* const nodeA, const dBroadPhaseNode* const nodeB, void* const)
{
	dFloat32 areaA = nodeA->m_surfaceArea;
	dFloat32 areaB = nodeB->m_surfaceArea;
	if (areaA < areaB) {
		return 1;
	}
	if (areaA > areaB) {
		return -1;
	}
	return 0;
}


void dBroadPhase::ImproveFitness(dgFitnessList& fitness, dgFloat64& oldEntropy, dBroadPhaseNode** const root)
{
	if (*root) {
		DG_TRACKTIME();
		dBroadPhaseNode* const parent = (*root)->m_parent;
		(*root)->m_parent = NULL;
		dgFloat64 entropy = CalculateEntropy(fitness, root);

		if ((entropy > oldEntropy * dFloat32(1.5f)) || (entropy < oldEntropy * dFloat32(0.75f))) {
			if (fitness.GetFirst()) {
				m_world->m_solverJacobiansMemory.ResizeIfNecessary ((fitness.GetCount() * 2 + 16) * sizeof (dBroadPhaseNode*));
				dBroadPhaseNode** const leafArray = (dBroadPhaseNode**)&m_world->m_solverJacobiansMemory[0];

				dgInt32 leafNodesCount = 0;
				for (dgFitnessList::dgListNode* nodePtr = fitness.GetFirst(); nodePtr; nodePtr = nodePtr->GetNext()) {
					dBroadPhaseNode* const node = nodePtr->GetInfo();
					dBroadPhaseNode* const leftNode = node->GetLeft();
					dgBody* const leftBody = leftNode->GetBody();
					if (leftBody) {
						node->SetAABB(leftBody->m_minAABB, leftBody->m_maxAABB);
						leafArray[leafNodesCount] = leftNode;
						leafNodesCount++;
					} else if (leftNode->IsAggregate()) {
						leafArray[leafNodesCount] = leftNode;
						leafNodesCount++;
					}
					dBroadPhaseNode* const rightNode = node->GetRight();
					dgBody* const rightBody = rightNode->GetBody();
					if (rightBody) {
						rightNode->SetAABB(rightBody->m_minAABB, rightBody->m_maxAABB);
						leafArray[leafNodesCount] = rightNode;
						leafNodesCount++;
					} else if (rightNode->IsAggregate()) {
						leafArray[leafNodesCount] = rightNode;
						leafNodesCount++;
					}
				}

				dgFitnessList::dgListNode* nodePtr = fitness.GetFirst();

				dgSortIndirect(leafArray, leafNodesCount, CompareNodes);
				*root = BuildTopDownBig(leafArray, 0, leafNodesCount - 1, &nodePtr);
				dgAssert(!(*root)->m_parent);
				//entropy = CalculateEntropy(fitness, root);
				entropy = fitness.TotalCost();
				fitness.m_prevCost = entropy;
			}
			oldEntropy = entropy;
		}
		(*root)->m_parent = parent;
	}
}


void dBroadPhase::RotateLeft (dBroadPhaseTreeNode* const node, dBroadPhaseNode** const root)
{
	dVector cost1P0;
	dVector cost1P1;

	dBroadPhaseTreeNode* const parent = (dBroadPhaseTreeNode*)node->m_parent;
	dgAssert(parent && !parent->IsLeafNode());
	dFloat32 cost1 = CalculateSurfaceArea(node->m_left, parent->m_left, cost1P0, cost1P1);

	dVector cost2P0;
	dVector cost2P1;
	dFloat32 cost2 = CalculateSurfaceArea(node->m_right, parent->m_left, cost2P0, cost2P1);

	dFloat32 cost0 = node->m_surfaceArea;
	if ((cost1 <= cost0) && (cost1 <= cost2)) {
		//dBroadPhaseNode* const parent = node->m_parent;
		node->m_minBox = parent->m_minBox;
		node->m_maxBox = parent->m_maxBox;
		node->m_surfaceArea = parent->m_surfaceArea;

		dBroadPhaseTreeNode* const grandParent = (dBroadPhaseTreeNode*) parent->m_parent;
		if (grandParent) {
			if (grandParent->m_left == parent) {
				grandParent->m_left = node;
			} else {
				dgAssert(grandParent->m_right == parent);
				grandParent->m_right = node;
			}
		} else {
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

	} else if ((cost2 <= cost0) && (cost2 <= cost1)) {
		//dBroadPhaseNode* const parent = node->m_parent;
		node->m_minBox = parent->m_minBox;
		node->m_maxBox = parent->m_maxBox;
		node->m_surfaceArea = parent->m_surfaceArea;

		dBroadPhaseTreeNode* const grandParent = (dBroadPhaseTreeNode*) parent->m_parent;
		if (grandParent) {
			if (grandParent->m_left == parent) {
				grandParent->m_left = node;
			} else {
				dgAssert(grandParent->m_right == parent);
				grandParent->m_right = node;
			}
		} else {
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

void dBroadPhase::RotateRight (dBroadPhaseTreeNode* const node, dBroadPhaseNode** const root)
{
	dVector cost1P0;
	dVector cost1P1;

	dBroadPhaseTreeNode* const parent = (dBroadPhaseTreeNode*) node->m_parent;
	dgAssert (parent && !parent->IsLeafNode());

	dFloat32 cost1 = CalculateSurfaceArea(node->m_right, parent->m_right, cost1P0, cost1P1);

	dVector cost2P0;
	dVector cost2P1;
	dFloat32 cost2 = CalculateSurfaceArea(node->m_left, parent->m_right, cost2P0, cost2P1);

	dFloat32 cost0 = node->m_surfaceArea;
	if ((cost1 <= cost0) && (cost1 <= cost2)) {
		//dBroadPhaseNode* const parent = node->m_parent;
		node->m_minBox = parent->m_minBox;
		node->m_maxBox = parent->m_maxBox;
		node->m_surfaceArea = parent->m_surfaceArea;

		dBroadPhaseTreeNode* const grandParent = (dBroadPhaseTreeNode*) parent->m_parent;
		if (grandParent) {
			dgAssert (!grandParent->IsLeafNode());
			if (grandParent->m_left == parent) {
				grandParent->m_left = node;
			} else {
				dgAssert(grandParent->m_right == parent);
				grandParent->m_right = node;
			}
		} else {
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

	} else if ((cost2 <= cost0) && (cost2 <= cost1)) {
		//dBroadPhaseNode* const parent = node->m_parent;
		node->m_minBox = parent->m_minBox;
		node->m_maxBox = parent->m_maxBox;
		node->m_surfaceArea = parent->m_surfaceArea;

		dBroadPhaseTreeNode* const grandParent = (dBroadPhaseTreeNode*) parent->m_parent;
		if (parent->m_parent) {
			if (grandParent->m_left == parent) {
				grandParent->m_left = node;
			} else {
				dgAssert(grandParent->m_right == parent);
				grandParent->m_right = node;
			}
		} else {
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

DG_INLINE bool dBroadPhase::ValidateContactCache(dgContact* const contact, const dVector& timestep) const
{
	dgAssert(contact && (contact->GetId() == dgConstraint::m_contactConstraint));

	dgBody* const body0 = contact->GetBody0();
	dgBody* const body1 = contact->GetBody1();
	if (!contact->m_material->m_contactGeneration) {
		dVector positStep(timestep * (body0->m_veloc - body1->m_veloc));
		positStep = ((positStep.DotProduct(positStep)) > m_velocTol) & positStep;
		contact->m_positAcc += positStep;

		dVector positError2(contact->m_positAcc.DotProduct(contact->m_positAcc));
		if ((positError2 < m_linearContactError2).GetSignMask()) {
			dVector rotationStep(timestep * (body0->m_omega - body1->m_omega));
			rotationStep = ((rotationStep.DotProduct(rotationStep)) > m_velocTol) & rotationStep;
			contact->m_rotationAcc = contact->m_rotationAcc * dgQuaternion(dFloat32(1.0f), rotationStep.m_x, rotationStep.m_y, rotationStep.m_z);

			dVector angle(contact->m_rotationAcc.m_x, contact->m_rotationAcc.m_y, contact->m_rotationAcc.m_z, dFloat32(0.0f));
			dVector rotatError2(angle.DotProduct(angle));
			if ((rotatError2 < m_angularContactError2).GetSignMask()) {
				return true;
			}
		}
	}
	return false;
}

void dBroadPhase::CalculatePairContacts (dgPair* const pair, dgInt32 threadID)
{
    dgContactPoint contacts[DG_MAX_CONTATCS];

	pair->m_cacheIsValid = false;
	pair->m_contactBuffer = contacts;
	m_world->CalculateContacts(pair, threadID, false, false);

	if (pair->m_contactCount) {
		dgAssert(pair->m_contactCount <= (DG_CONSTRAINT_MAX_ROWS / 3));
		m_world->ProcessContacts(pair, threadID);
		KinematicBodyActivation(pair->m_contact);
	} else {
		if (pair->m_cacheIsValid) {
			KinematicBodyActivation(pair->m_contact);
		} else {
			pair->m_contact->m_maxDOF = 0;
		}
	}
}

void dBroadPhase::AddPair (dgContact* const contact, dFloat32 timestep, dgInt32 threadIndex)
{
	//DG_TRACKTIME();
	dgWorld* const world = (dgWorld*) m_world;
	dgBody* const body0 = contact->m_body0;
	dgBody* const body1 = contact->m_body1;

	dgAssert (body0 != m_world->m_sentinelBody);
	dgAssert (body1 != m_world->m_sentinelBody);
	dgAssert (contact->GetId() == dgConstraint::m_contactConstraint);
	dgAssert (body0->GetWorld());
	dgAssert (body1->GetWorld());
	dgAssert (body0->GetWorld() == world);
	dgAssert (body1->GetWorld() == world);
	if (!(body0->m_collideWithLinkedBodies & body1->m_collideWithLinkedBodies)) {
		if (world->AreBodyConnectedByJoints (body0, body1)) {
			return;
		}
	}

	const dgContactMaterial* const material = contact->m_material;
	if (material->m_flags & dgContactMaterial::m_collisionEnable) {
		dgInt32 processContacts = 1;
		if (material->m_aabbOverlap) {
			processContacts = material->m_aabbOverlap(*contact, timestep, threadIndex);
		}
		if (processContacts) {
            dgPair pair;
			dgAssert (!body0->m_collision->IsType (dgCollision::dgCollisionNull_RTTI));
			dgAssert (!body1->m_collision->IsType (dgCollision::dgCollisionNull_RTTI));

			pair.m_contact = contact;
			pair.m_timestep = timestep;
            CalculatePairContacts (&pair, threadIndex);
		}
	}
}

bool dBroadPhase::TestOverlaping(const dgBody* const body0, const dgBody* const body1, dFloat32 timestep) const
{
	bool mass0 = (body0->m_invMass.m_w != dFloat32(0.0f));
	bool mass1 = (body1->m_invMass.m_w != dFloat32(0.0f));
	bool isDynamic0 = body0->IsRTTIType(dgBody::m_dynamicBodyRTTI) != 0;
	bool isDynamic1 = body1->IsRTTIType(dgBody::m_dynamicBodyRTTI) != 0;
	bool isKinematic0 = body0->IsRTTIType(dgBody::m_kinematicBodyRTTI) != 0;
	bool isKinematic1 = body1->IsRTTIType(dgBody::m_kinematicBodyRTTI) != 0;

	dgAssert(!body0->GetCollision()->IsType(dgCollision::dgCollisionNull_RTTI));
	dgAssert(!body1->GetCollision()->IsType(dgCollision::dgCollisionNull_RTTI));

	const dBroadPhaseAggregate* const agreggate0 = body0->GetBroadPhaseAggregate();
	const dBroadPhaseAggregate* const agreggate1 = body1->GetBroadPhaseAggregate();

	bool tier1 = true;
	bool tier2 = !(body0->m_sleeping & body1->m_sleeping);
	bool tier3 = (agreggate0 != agreggate1) || !agreggate0 || (agreggate0 && agreggate0->GetSelfCollision());
	bool tier4 = isDynamic0 & mass0;
	bool tier5 = isDynamic1 & mass1;
	bool tier6 = isKinematic0 & mass1;
	bool tier7 = isKinematic1 & mass0;
	bool ret = tier1 & tier2  & tier3 & (tier4 | tier5 | tier6 | tier7);

	if (ret) {
		const dgCollisionInstance* const instance0 = body0->GetCollision();
		const dgCollisionInstance* const instance1 = body1->GetCollision();

		if (body0->m_continueCollisionMode | body1->m_continueCollisionMode) {
			dVector velRelative(body1->GetVelocity() - body0->GetVelocity());
			if (velRelative.DotProduct(velRelative).GetScalar() > dFloat32(0.25f)) {
				dVector box0_p0;
				dVector box0_p1;
				dVector box1_p0;
				dVector box1_p1;

				instance0->CalcAABB(instance0->GetGlobalMatrix(), box0_p0, box0_p1);
				instance1->CalcAABB(instance1->GetGlobalMatrix(), box1_p0, box1_p1);

				dVector boxp0(box0_p0 - box1_p1);
				dVector boxp1(box0_p1 - box1_p0);
				dgFastRayTest ray(dVector::m_zero, velRelative.Scale(timestep * dFloat32(4.0f)));
				dFloat32 distance = ray.BoxIntersect(boxp0, boxp1);
				ret = (distance < dFloat32(1.0f));
			} else {
				ret = dgOverlapTest(body0->m_minAABB, body0->m_maxAABB, body1->m_minAABB, body1->m_maxAABB) ? 1 : 0;
			}
		} else {
			ret = dgOverlapTest(body0->m_minAABB, body0->m_maxAABB, body1->m_minAABB, body1->m_maxAABB) ? 1 : 0;
		}
	}
	return ret;
}

void dBroadPhase::AddPair (dgBody* const body0, dgBody* const body1, const dFloat32 timestep, dgInt32 threadID)
{
	dgAssert(body0);
	dgAssert(body1);
	const bool test = TestOverlaping (body0, body1, timestep);
	if (test) {
		dgContact* contact = m_contactCache.FindContactJoint(body0, body1);
		if (!contact) {
			const dgBilateralConstraint* const bilateral = m_world->FindBilateralJoint (body0, body1);
			const bool isCollidable = bilateral ? bilateral->IsCollidable() : true;

			if (isCollidable) {
				dgUnsigned32 group0_ID = dgUnsigned32 (body0->m_bodyGroupId);
				dgUnsigned32 group1_ID = dgUnsigned32 (body1->m_bodyGroupId);

				if (group1_ID < group0_ID) {
					dgSwap (group0_ID, group1_ID);
				}

				dgUnsigned32 key = (group1_ID << 16) + group0_ID;
				const dgBodyMaterialList* const materialList = m_world;  
				dgAssert (materialList->Find (key));
				const dgContactMaterial* const material = &materialList->Find (key)->GetInfo();

				if (material->m_flags & dgContactMaterial::m_collisionEnable) {

					dgInt32 isBody0Kinematic = body0->IsRTTIType(dgBody::m_kinematicBodyRTTI);
					dgInt32 isBody1Kinematic = body1->IsRTTIType(dgBody::m_kinematicBodyRTTI);

					const dgInt32 kinematicTest = !((isBody0Kinematic && isBody1Kinematic) || ((isBody0Kinematic && body0->IsCollidable()) || (isBody1Kinematic && body1->IsCollidable())));
					const dgInt32 collisionTest = kinematicTest && !(body0->m_isdead | body1->m_isdead) && !(body0->m_equilibrium & body1->m_equilibrium);
					if (collisionTest) {
						const dgInt32 isSofBody0 = body0->m_collision->IsType(dgCollision::dgCollisionLumpedMass_RTTI);
						const dgInt32 isSofBody1 = body1->m_collision->IsType(dgCollision::dgCollisionLumpedMass_RTTI);

						if (isSofBody0 || isSofBody1) {
							m_pendingSoftBodyCollisions[m_pendingSoftBodyPairsCount].m_body0 = body0;
							m_pendingSoftBodyCollisions[m_pendingSoftBodyPairsCount].m_body1 = body1;
							m_pendingSoftBodyPairsCount++;
						} else {
							dgContactList& contactList = *m_world;
							dgAtomicExchangeAndAdd(&contactList.m_contactCountReset, 1);
							if (contactList.m_contactCount < contactList.GetElementsCapacity()) {
								contact = new (m_world->m_allocator) dgContact(m_world, material, body0, body1);
								dgAssert(contact);
								contactList.Push(contact);
							}
						}
					}
				}
			}
		}
	}
}

void dBroadPhase::FindGeneratedBodiesCollidingPairs(dgBroadphaseSyncDescriptor* const descriptor, dgInt32 threadID)
{
dgAssert (0);
/*
	dgList<dgBody*>::dgListNode* node = NULL;
	{
		dgThreadHiveScopeLock lock(m_world, &m_criticalSectionLock, false);
		node = descriptor->m_newBodiesNodes;
		if (node) {
			descriptor->m_newBodiesNodes = node->GetNext();
		}
	}

	dVector timestep2(descriptor->m_timestep * descriptor->m_timestep * dFloat32(4.0f));
	while (node) {
		dgBody* const body = node->GetInfo();
		dBroadPhaseNode* const breadPhaseNode = body->GetBroadPhase();
		if (breadPhaseNode) {
			if (!body->m_collision->IsType(dgCollision::dgCollisionNull_RTTI)) {
				for (dBroadPhaseNode* ptr = breadPhaseNode; ptr->m_parent; ptr = ptr->m_parent) {
					dBroadPhaseNode* const sibling = ptr->m_parent->m_right;
					if (sibling != ptr) {
						dgAssert(0);
						//SubmitPairs (bodyNode, sibling, timestep2, threadID);
					}
					else {
						dgAssert(0);
						//dgNode* const sibling = ptr->m_parent->m_left;
						//dgAssert (sibling);
						//dgAssert (sibling != ptr);
						//dgAssert (0);
						//SubmitPairs (bodyNode, sibling, timestep2, threadID);
					}
				}
			}
		}

		dgThreadHiveScopeLock lock(m_world, &m_criticalSectionLock, false);
		node = descriptor->m_newBodiesNodes;
		if (node) {
			descriptor->m_newBodiesNodes = node->GetNext();
		}
	}
*/
}


void dBroadPhase::SubmitPairs(dBroadPhaseNode* const leafNode, dBroadPhaseNode* const node, dFloat32 timestep, dgInt32 threadCount, dgInt32 threadID)
{
	dBroadPhaseNode* pool[DG_BROADPHASE_MAX_STACK_DEPTH];
	pool[0] = node;
	dgInt32 stack = 1;

	dgAssert (leafNode->IsLeafNode());
	dgBody* const body0 = leafNode->GetBody();

	const dVector boxP0 (body0 ? body0->m_minAABB : leafNode->m_minBox);
	const dVector boxP1 (body0 ? body0->m_maxAABB : leafNode->m_maxBox);

	const bool test0 = body0 ? (body0->GetInvMass().m_w != dFloat32(0.0f)) : true;

	while (stack) {
		stack--;
		dBroadPhaseNode* const rootNode = pool[stack];
		if (dgOverlapTest(rootNode->m_minBox, rootNode->m_maxBox, boxP0, boxP1)) {
			if (rootNode->IsLeafNode()) {
				dgAssert(!rootNode->GetRight());
				dgAssert(!rootNode->GetLeft());
				dgBody* const body1 = rootNode->GetBody();
				if (body0) {
					if (body1) {
						if (test0 || (body1->GetInvMass().m_w != dFloat32(0.0f))) {
							AddPair(body0, body1, timestep, threadID);
						}
					} else {
						dgAssert (rootNode->IsAggregate());
						dBroadPhaseAggregate* const aggregate = (dBroadPhaseAggregate*) rootNode;
						aggregate->SummitPairs(body0, timestep, threadID);
					}
				} else {
					dgAssert (leafNode->IsAggregate());
					dBroadPhaseAggregate* const aggregate = (dBroadPhaseAggregate*) leafNode;
					if (body1) {
						aggregate->SummitPairs(body1, timestep, threadID);
					} else {
						dgAssert (rootNode->IsAggregate());
						aggregate->SummitPairs((dBroadPhaseAggregate*) rootNode, timestep, threadID);
					}
				}
			} else {
				dBroadPhaseTreeNode* const tmpNode = (dBroadPhaseTreeNode*) rootNode;
				dgAssert (tmpNode->m_left);
				dgAssert (tmpNode->m_right);

				pool[stack] = tmpNode->m_left;
				stack++;
				dgAssert(stack < dgInt32(sizeof (pool) / sizeof (pool[0])));

				pool[stack] = tmpNode->m_right;
				stack++;
				dgAssert(stack < dgInt32(sizeof (pool) / sizeof (pool[0])));
			}
		}
	}
}

void dBroadPhase::ImproveNodeFitness(dBroadPhaseTreeNode* const node, dBroadPhaseNode** const root)
{
	dgAssert(node->GetLeft());
	dgAssert(node->GetRight());

	dBroadPhaseNode* const parent = node->m_parent;
	if (parent && parent->m_parent) {
		dgAssert (!parent->IsLeafNode());
		if (parent->GetLeft() == node) {
			RotateRight(node, root);
		} else {
			RotateLeft(node, root);
		}
	}
	dgAssert(!m_rootNode->m_parent);
}

dgFloat64 dBroadPhase::CalculateEntropy (dgFitnessList& fitness, dBroadPhaseNode** const root)
{
	DG_TRACKTIME();
#if 0
	dgFloat64 cost0 = fitness.TotalCost();
	dgFloat64 cost1 = cost0;
	do {
		cost0 = cost1;
		for (dgFitnessList::dgListNode* node = fitness.GetFirst(); node; node = node->GetNext()) {
			ImproveNodeFitness(node->GetInfo(), root);
		}
		cost1 = fitness.TotalCost();
	} while (cost1 < (dFloat32(0.99f)) * cost0);
	return cost1;
#else
	dgFloat64 cost = dFloat32 (0.0f);
	if (fitness.GetCount() < 32) {
		for (dgFitnessList::dgListNode* node = fitness.GetFirst(); node; node = node->GetNext()) {
			ImproveNodeFitness(node->GetInfo(), root);
		}
		cost = fitness.TotalCost();
		fitness.m_prevCost = cost;
	} else {
		const dgInt32 mod = 16;
		cost = fitness.m_prevCost;
		dgFitnessList::dgListNode* node = fitness.GetFirst();
		for (dgInt32 i = 0; i < fitness.m_index; i++) {
			node = node->GetNext();
		}

		do {
			ImproveNodeFitness(node->GetInfo(), root);
			for (dgInt32 i = 0; i < mod; i++) {
				node = node ? node->GetNext() : NULL;
			}
		} while (node);
	
		if (!fitness.m_index) {
			cost = fitness.TotalCost();
			fitness.m_prevCost = cost;
		}
		fitness.m_index = (fitness.m_index + 1) % mod;
	}
	return cost;
#endif
}

void dBroadPhase::KinematicBodyActivation (dgContact* const contatJoint) const
{
	dgBody* const body0 = contatJoint->GetBody0();
	dgBody* const body1 = contatJoint->GetBody1();
	if (body0->IsCollidable() | body1->IsCollidable()) {
		if (body0->IsRTTIType(dgBody::m_kinematicBodyRTTI)) {
			if (body1->IsRTTIType(dgBody::m_dynamicBodyRTTI) && (body1->GetInvMass().m_w > dFloat32 (0.0f))) {
				if (body1->m_equilibrium) {
					dVector relVeloc (body0->m_veloc - body1->m_veloc);
					dVector relOmega (body0->m_omega - body1->m_omega);
					dVector mask2 ((relVeloc.DotProduct(relVeloc) < dgDynamicBody::m_equilibriumError2) & (relOmega.DotProduct(relOmega) < dgDynamicBody::m_equilibriumError2));

					dgScopeSpinPause lock(&body1->m_criticalSectionLock);
					body1->m_sleeping = false;
					body1->m_equilibrium = mask2.GetSignMask() ? true : false;
				}
			}
		} else if (body1->IsRTTIType(dgBody::m_kinematicBodyRTTI)) {
			if (body0->IsRTTIType(dgBody::m_dynamicBodyRTTI) && (body0->GetInvMass().m_w > dFloat32 (0.0f))) {
				if (body0->m_equilibrium) {
					dVector relVeloc (body0->m_veloc - body1->m_veloc);
					dVector relOmega (body0->m_omega - body1->m_omega);
					dVector mask2 ((relVeloc.DotProduct(relVeloc) < dgDynamicBody::m_equilibriumError2) & (relOmega.DotProduct(relOmega) < dgDynamicBody::m_equilibriumError2));

					dgScopeSpinPause lock(&body1->m_criticalSectionLock);
					body0->m_sleeping = false;
					body0->m_equilibrium = mask2.GetSignMask() ? true : false;
				}
			}
		}
	}
}

void dBroadPhase::CollidingPairsKernel(void* const context, void* const node, dgInt32 threadID)
{
	D_TRACKTIME();
	dgBroadphaseSyncDescriptor* const descriptor = (dgBroadphaseSyncDescriptor*)context;
	dgWorld* const world = descriptor->m_world;
	dBroadPhase* const broadPhase = world->GetBroadPhase();
	broadPhase->FindCollidingPairs(descriptor, (dgList<dBroadPhaseNode*>::dgListNode*) node, threadID);
}

void dBroadPhase::AddGeneratedBodiesContactsKernel (void* const context, void* const worldContext, dgInt32 threadID)
{
	D_TRACKTIME();
	dgBroadphaseSyncDescriptor* const descriptor = (dgBroadphaseSyncDescriptor*) context;
	dgWorld* const world = (dgWorld*) worldContext;
	dBroadPhase* const broadPhase = world->GetBroadPhase();
	broadPhase->FindGeneratedBodiesCollidingPairs (descriptor, threadID);
}

void dBroadPhase::UpdateSoftBodyContactKernel(void* const context, void* const worldContext, dgInt32 threadID)
{
	D_TRACKTIME();
	dgBroadphaseSyncDescriptor* const descriptor = (dgBroadphaseSyncDescriptor*)context;
	dgWorld* const world = descriptor->m_world;
	dBroadPhase* const broadPhase = world->GetBroadPhase();
	broadPhase->UpdateSoftBodyContacts(descriptor, descriptor->m_timestep, threadID);
}

void dBroadPhase::UpdateRigidBodyContactKernel(void* const context, void* const , dgInt32 threadID)
{
	D_TRACKTIME();
	dgBroadphaseSyncDescriptor* const descriptor = (dgBroadphaseSyncDescriptor*)context;
	dgWorld* const world = descriptor->m_world;
	dBroadPhase* const broadPhase = world->GetBroadPhase();
	broadPhase->UpdateRigidBodyContacts(descriptor, descriptor->m_timestep, threadID);
}

void dBroadPhase::UpdateSoftBodyContacts(dgBroadphaseSyncDescriptor* const descriptor, dFloat32 timeStep, dgInt32 threadID)
{
	dgAssert(0);
/*
	const dgInt32 count = m_pendingSoftBodyPairsCount;
	for (dgInt32 i = dgAtomicExchangeAndAdd(&descriptor->m_pairsAtomicCounter, 1); i < count; i = dgAtomicExchangeAndAdd(&descriptor->m_pairsAtomicCounter, 1)) {
		dgPendingCollisionSoftBodies& pair = m_pendingSoftBodyCollisions[i];
		if (pair.m_body0->m_collision->IsType(dgCollision::dgCollisionLumpedMass_RTTI)) {
			dgCollisionLumpedMassParticles* const lumpedMassShape = (dgCollisionLumpedMassParticles*)pair.m_body0->m_collision->GetChildShape();
			dgAssert(pair.m_body0->IsRTTIType(dgBody::m_dynamicBodyRTTI));
			dgAssert (pair.m_body0 == lumpedMassShape->GetOwner ());
			lumpedMassShape->RegisterCollision(pair.m_body1);
		} else if (pair.m_body1->m_collision->IsType(dgCollision::dgCollisionLumpedMass_RTTI)) {
			dgCollisionLumpedMassParticles* const lumpedMassShape = (dgCollisionLumpedMassParticles*)pair.m_body1->m_collision->GetChildShape();
			dgAssert(pair.m_body1->IsRTTIType(dgBody::m_dynamicBodyRTTI));
			dgAssert (pair.m_body1 == lumpedMassShape->GetOwner ());
			lumpedMassShape->RegisterCollision(pair.m_body0);
		}
	}
*/
}

void dBroadPhase::UpdateRigidBodyContacts(dgBroadphaseSyncDescriptor* const descriptor, dFloat32 timeStep, dgInt32 threadID)
{
	DG_TRACKTIME();

	dgContactList& contactList = *m_world;
	const dFloat32 timestep = descriptor->m_timestep;
	const dgInt32 threadCount = m_world->GetThreadCount();
	const dgUnsigned32 lru = m_lru - DG_CONTACT_DELAY_FRAMES;

	const dgInt32 contactCount = contactList.m_contactCount;
	dgContact** const contactArray = &contactList[0];

	dVector deltaTime(timestep);
	for (dgInt32 i = threadID; i < contactCount; i += threadCount) {
		dgContact* const contact = contactArray[i];
		dgAssert (contact);

		dgBody* const body0 = contact->GetBody0();
		dgBody* const body1 = contact->GetBody1();

		if (!(contact->m_killContact | (body0->m_equilibrium & body1->m_equilibrium))) {
			dgAssert(!contact->m_killContact);

			bool isActive = contact->m_isActive;
			if (ValidateContactCache(contact, deltaTime)) {
				contact->m_broadphaseLru = m_lru;
				contact->m_timeOfImpact = dFloat32(1.0e10f);
			} else {
				contact->m_isActive = 0;
				contact->m_positAcc = dVector::m_zero;
				contact->m_rotationAcc = dgQuaternion();

				dFloat32 distance = contact->m_separationDistance;
				if (distance >= DG_NARROW_PHASE_DIST) {
					const dVector veloc0 (body0->GetVelocity());
					const dVector veloc1 (body1->GetVelocity());

					const dVector veloc(veloc1 - veloc0);
					const dVector omega0 (body0->GetOmega());
					const dVector omega1 (body1->GetOmega());
					const dgCollisionInstance* const collision0 = body0->GetCollision();
					const dgCollisionInstance* const collision1 = body1->GetCollision();
					const dVector scale(dFloat32(1.0f), dFloat32(3.5f) * collision0->GetBoxMaxRadius(), dFloat32(3.5f) * collision1->GetBoxMaxRadius(), dFloat32(0.0f));
					const dVector velocMag2(veloc.DotProduct(veloc).GetScalar(), omega0.DotProduct(omega0).GetScalar(), omega1.DotProduct(omega1).GetScalar(), dFloat32(0.0f));
					const dVector velocMag(velocMag2.GetMax(dVector::m_epsilon).InvSqrt() * velocMag2 * scale);
					const dFloat32 speed = velocMag.AddHorizontal().GetScalar() + dFloat32(0.5f);

					distance -= speed * timestep;
					contact->m_separationDistance = distance;
				}
				if (distance < DG_NARROW_PHASE_DIST) {
					AddPair(contact, timestep, threadID);
					if (contact->m_maxDOF) {
						contact->m_timeOfImpact = dFloat32(1.0e10f);
					}
					contact->m_broadphaseLru = m_lru;
				} else {
					dgAssert (contact->m_maxDOF == 0);
					const dBroadPhaseNode* const bodyNode0 = contact->GetBody0()->m_broadPhaseNode;
					const dBroadPhaseNode* const bodyNode1 = contact->GetBody1()->m_broadPhaseNode;
					if (dgOverlapTest(bodyNode0->m_minBox, bodyNode0->m_maxBox, bodyNode1->m_minBox, bodyNode1->m_maxBox)) {
						contact->m_broadphaseLru = m_lru;
					} else if (contact->m_broadphaseLru < lru) {
						contact->m_killContact = 1;
					}
				}
			}

			if (isActive ^ contact->m_isActive) {
				if (body0->GetInvMass().m_w) {
					body0->m_equilibrium = false;
				}
				if (body1->GetInvMass().m_w) {
					body1->m_equilibrium = false;
				}
			}

		} else {
			contact->m_broadphaseLru = m_lru;
		}

		contact->m_killContact = contact->m_killContact | (body0->m_equilibrium & body1->m_equilibrium & !contact->m_isActive);
	}
}

bool dBroadPhase::SanityCheck() const
{
#ifdef _DEBUG
	class dgKey
	{
		public:
		dgKey (dgContact* const contact)
			:m_low(dgMin (contact->GetBody0()->m_uniqueID, contact->GetBody1()->m_uniqueID))
			,m_high(dgMax (contact->GetBody0()->m_uniqueID, contact->GetBody1()->m_uniqueID))
		{
		}

		bool operator> (const dgKey& key) const
		{
			return m_key > key.m_key;
		}

		bool operator< (const dgKey& key) const
		{
			return m_key < key.m_key;
		}

		union 
		{
			dgUnsigned64 m_key;
			struct 
			{
				dgInt32 m_low;
				dgInt32 m_high;
			};
		};
	};

	dgTree<dgInt32, dgKey> filter (m_world->GetAllocator());

	const dgContactList& contactList = *m_world;
	for (dgInt32 i = contactList.m_contactCount - 1; i >= 0; i--) {
		dgContact* const contact = contactList[i];
		dgAssert (!contact->m_killContact);
		dgAssert (filter.Insert(0, dgKey(contact)));
	}
#endif
	return true;
}

void dBroadPhase::AttachNewContact(dgInt32 startCount)
{
	DG_TRACKTIME();
	dgContactList& contactList = *m_world;
	if (contactList.m_contactCountReset > contactList.m_contactCount) {
		contactList.Resize(contactList.GetElementsCapacity() * 2);
	}

	dgContact** const contactArray = &contactList[0];
	for (dgInt32 i = contactList.m_contactCount - 1; i >= startCount; i--) {
		dgContact* const contact = contactArray[i];
		if (m_contactCache.AddContactJoint(contact)) {
			m_world->AttachContact(contact);
		} else {
			contactList.m_contactCount--;
			contactArray[i] = contactList[contactList.m_contactCount];
			delete contact;
		}
	}
}

void dBroadPhase::DeleteDeadContact(dFloat32 timestep)
{
	DG_TRACKTIME();
	dgInt32 activeCount = 0;
	dgContactList& contactList = *m_world;
	dgContact** const contactArray = &contactList[0];
	dgArray<dgJointInfo>& constraintArray = m_world->m_jointsMemory;
	for (dgInt32 i = contactList.m_contactCount - 1; i >= 0; i--) {
		dgContact* const contact = contactArray[i];
		if (contact->m_killContact) {
			m_contactCache.RemoveContactJoint(contact);
			m_world->RemoveContact(contact);
			contactList.m_contactCount--;
			contactArray[i] = contactList[contactList.m_contactCount];
			delete contact;
		} else if (contact->m_isActive && contact->m_maxDOF){
			constraintArray[activeCount].m_joint = contact;
			activeCount++;
		} else if (contact->m_body0->m_continueCollisionMode | contact->m_body1->m_continueCollisionMode){
			if (contact->EstimateCCD(timestep)) {
				constraintArray[activeCount].m_joint = contact;
				activeCount++;
			}
		}
	}
	dgAssert(SanityCheck());
	contactList.m_activeContactCount = activeCount;
	//dgTrace (("%d %d\n", contactList.m_activeContactCount, contactList.m_contactCount));
}

void dBroadPhase::UpdateContacts(dFloat32 timestep)
{
	D_TRACKTIME();
    m_lru = m_lru + 1;
	m_pendingSoftBodyPairsCount = 0;

	const dgInt32 threadsCount = m_world->GetThreadCount();

	const dgBodyMasterList* const masterList = m_world;

	m_world->m_bodiesMemory.ResizeIfNecessary(masterList->GetCount());
	dgBroadphaseSyncDescriptor syncPoints(timestep, m_world);

	dgBodyMasterList::dgListNode* node = masterList->GetFirst()->GetNext();
	for (dgInt32 i = 0; i < threadsCount; i++) {
		m_world->QueueJob(ForceAndToqueKernel, &syncPoints, node, "dBroadPhase::ForceAndToque");
		node = node ? node->GetNext() : NULL;
	}
	m_world->SynchronizationBarrier();

	// update pre-listeners after the force and torque are applied
	if (m_world->m_listeners.GetCount()) {
		for (dgWorld::dgListenerList::dgListNode* node1 = m_world->m_listeners.GetFirst(); node1; node1 = node1->GetNext()) {
			dgWorld::dgListener& listener = node1->GetInfo();
			if (listener.m_onPreUpdate) {
				listener.m_onPreUpdate(m_world, listener.m_userData, timestep);
			}
		}
	}

	// check for sleeping bodies states
	node = masterList->GetFirst()->GetNext();
	for (dgInt32 i = 0; i < threadsCount; i++) {
		m_world->QueueJob(SleepingStateKernel, &syncPoints, node, "dBroadPhase::SleepingState");
		node = node ? node->GetNext() : NULL;
	}
	m_world->SynchronizationBarrier();

	// this will move to an asynchronous thread 
	dgList<dBroadPhaseAggregate*>::dgListNode* aggregateNode = m_aggregateList.GetFirst();
	for (dgInt32 i = 0; i < threadsCount; i++) {
		m_world->QueueJob(UpdateAggregateEntropyKernel, &syncPoints, aggregateNode, "dBroadPhase::UpdateAggregateEntropy");
		aggregateNode = aggregateNode ? aggregateNode->GetNext() : NULL;
	}
	m_world->SynchronizationBarrier();

	UpdateFitness();

	dgContactList& contactList = *m_world;
	contactList.m_contactCountReset = contactList.m_contactCount;
	syncPoints.m_contactStart = contactList.m_contactCount;

	syncPoints.m_fullScan = syncPoints.m_fullScan || (syncPoints.m_atomicPendingBodiesCount >= (syncPoints.m_atomicDynamicsCount / 2));
	dgList<dBroadPhaseNode*>::dgListNode* broadPhaseNode = m_updateList.GetFirst();
	for (dgInt32 i = 0; i < threadsCount; i++) {
		m_world->QueueJob(CollidingPairsKernel, &syncPoints, broadPhaseNode, "dBroadPhase::CollidingPairs");
		broadPhaseNode = broadPhaseNode ? broadPhaseNode->GetNext() : NULL;
	}
	m_world->SynchronizationBarrier();

	AttachNewContact(syncPoints.m_contactStart);
	for (dgInt32 i = 0; i < threadsCount; i++) {
		m_world->QueueJob(UpdateRigidBodyContactKernel, &syncPoints, NULL, "dBroadPhase::UpdateRigidBodyContact");
	}
	m_world->SynchronizationBarrier();

	if (m_pendingSoftBodyPairsCount) {
		dgAssert (0);
		//for (dgInt32 i = 0; i < threadsCount; i++) {
		//	m_world->QueueJob(UpdateSoftBodyContactKernel, &syncPoints, contactListNode, "dBroadPhase::UpdateSoftBodyContact");
		//}
		//m_world->SynchronizationBarrier();
	}

	//	m_recursiveChunks = false;
	if (m_generatedBodies.GetCount()) {
		dgAssert(0);
		//syncPoints.m_newBodiesNodes = m_generatedBodies.GetFirst();
		//for (dgInt32 i = 0; i < threadsCount; i++) {
		//	m_world->QueueJob(AddGeneratedBodiesContactsKernel, &syncPoints, m_world);
		//}
		//m_world->SynchronizationBarrier();
		//
		//for (dgInt32 i = 0; i < threadsCount; i++) {
		//	m_world->QueueJob(UpdateContactsKernel, &syncPoints, m_world);
		//}
		//m_world->SynchronizationBarrier();
		//
		//m_generatedBodies.RemoveAll();
	}

	DeleteDeadContact(timestep);
}
#endif


dBroadPhase::dBroadPhase(dNewton* const world)
	:dClassAlloc()
	,m_newton(world)
{
}

dBroadPhase::~dBroadPhase()
{
}

void dBroadPhase::RemoveBody(dBody* const body)
{

}

void dBroadPhase::Update(dFloat32 timestep)
{
	D_TRACKTIME();
}