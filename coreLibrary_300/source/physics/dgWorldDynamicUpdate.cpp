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
#include "dgBroadPhase.h"
#include "dgConstraint.h"
#include "dgDynamicBody.h"
#include "dgCollisionInstance.h"
#include "dgBilateralConstraint.h"
#include "dgWorldDynamicUpdate.h"
#include "dgCollisionDeformableMesh.h"


#ifdef _NEWTON_AMP
#include "dgAmpInstance.h"
#endif



#define DG_CCD_EXTRA_CONTACT_COUNT			(8 * 3)
#define DG_PARALLEL_JOINT_COUNT_CUT_OFF		(256)


dgVector dgWorldDynamicUpdate::m_velocTol (dgFloat32 (1.0e-18f));
dgVector dgWorldDynamicUpdate::m_eulerTaylorCorrection (dgFloat32 (1.0f / 12.0f));

class dgWorldDynamicUpdateSyncDescriptor
{
	public:
	dgWorldDynamicUpdateSyncDescriptor()
	{
		memset (this, 0, sizeof (dgWorldDynamicUpdateSyncDescriptor));
	}

	dgFloat32 m_timestep;
	dgInt32 m_atomicCounter;
	
	dgInt32 m_islandCount;
	dgInt32 m_firstIsland;
	dgThread::dgCriticalSection* m_criticalSection;
};


//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

dgWorldDynamicUpdate::dgWorldDynamicUpdate()
	:m_bodies(0)
	,m_joints(0)
	,m_islands(0)
	,m_markLru(0)
	,m_softBodyCriticalSectionLock()
	,m_islandMemory(NULL)
{
}


void dgWorldDynamicUpdate::UpdateDynamics(dgFloat32 timestep)
{
	dTimeTrackerEvent(__FUNCTION__);
	dgWorld* const world = (dgWorld*) this;
	dgBodyMasterList& masterList = *world;

	m_bodies = 0;
	m_joints = 0;
	m_islands = 0;
	m_solverConvergeQuality = world->m_solverConvergeQuality;
	world->m_dynamicsLru = world->m_dynamicsLru + DG_BODY_LRU_STEP;
	m_markLru = world->m_dynamicsLru;

	dgBody* const sentinelBody = world->m_sentinelBody;
	sentinelBody->m_index = 0; 
	sentinelBody->m_dynamicsLru = m_markLru;

	m_islandMemory = dgAlloca (dgIsland, masterList.GetCount() + 256);
	BuildIslands(timestep);

	dgInt32 maxRowCount = 0;
	for (dgInt32 i = 0; i < m_islands; i ++) {
		m_islandMemory[i].m_rowsStart = maxRowCount;
		maxRowCount += m_islandMemory[i].m_rowsCount;
	}
	m_solverMemory.Init (world, maxRowCount, m_bodies);

	dgInt32 threadCount = world->GetThreadCount();	

	dgWorldDynamicUpdateSyncDescriptor descriptor;
	descriptor.m_timestep = timestep;
	descriptor.m_firstIsland = 0;
	descriptor.m_islandCount = m_islands;
	descriptor.m_atomicCounter = 0;

	sentinelBody->m_resting = true;
	sentinelBody->m_active = false;
	sentinelBody->m_equilibrium = true;
	for (dgInt32 i = 0; i < threadCount; i ++) {
		world->QueueJob (FindActiveJointAndBodiesKernel, &descriptor, world);
	}
	world->SynchronizationBarrier();
	{
		dTimeTrackerEvent("SortIsland");
		dgSort (m_islandMemory, m_islands, CompareIslands); 
	}
	

	if (!(world->m_amp && (world->m_hardwaredIndex > 0))) {
		dgInt32 index = 0;
		dgInt32 useParallel = world->m_useParallelSolver && (threadCount > 1);
//useParallel = 1;
		if (useParallel) {
			dgInt32 sum = m_joints;
			useParallel = useParallel && m_joints && m_islands;
			useParallel = useParallel && ((threadCount * m_islandMemory[0].m_jointCount) >= sum);
			useParallel = useParallel && (m_islandMemory[0].m_jointCount > DG_PARALLEL_JOINT_COUNT_CUT_OFF);
//useParallel = 1;
			while (useParallel) {
				CalculateReactionForcesParallel(&m_islandMemory[index], timestep);
				index ++;
				sum -= m_islandMemory[index].m_jointCount;
				useParallel = useParallel && (index < m_islands);
				useParallel = useParallel && ((threadCount * m_islandMemory[index].m_jointCount) >= m_joints);
				useParallel = useParallel && (m_islandMemory[index].m_jointCount > DG_PARALLEL_JOINT_COUNT_CUT_OFF);
			}
		}

		if (index < m_islands) {
			descriptor.m_firstIsland = index;
			descriptor.m_islandCount = m_islands - index;
			descriptor.m_atomicCounter = 0;
			for (dgInt32 i = 0; i < threadCount; i ++) {
				world->QueueJob (CalculateIslandReactionForcesKernel, &descriptor, world);
			}
			world->SynchronizationBarrier();
		}
	} else {
		#ifdef _NEWTON_AMP 
			
			world->m_amp->ConstraintSolver (m_islands, m_islandMemory, timestep);
		#endif
	}

	m_islandMemory = NULL;
	// integrate soft body dynamics phase 2
	dgDeformableBodiesUpdate* const softBodyList = world;
    softBodyList->SolveConstraintsAndIntegrate (timestep);
}

void dgWorldDynamicUpdate::BuildIslands(dgFloat32 timestep)
{
	dTimeTrackerEvent(__FUNCTION__);

	dgWorld* const world = (dgWorld*) this;
	dgUnsigned32 lru = m_markLru - 1;

	dgBodyMasterList& masterList = *world;

	dgAssert (masterList.GetFirst()->GetInfo().GetBody() == world->m_sentinelBody);
	dgDynamicBody** const stackPoolBuffer = dgAlloca (dgDynamicBody*, 2 * (masterList.m_constraintCount + 1024));

	for (dgBodyMasterList::dgListNode* node = masterList.GetLast(); node; node = node->GetPrev()) {
		const dgBodyMasterListRow& graphNode = node->GetInfo();
		dgBody* const body = graphNode.GetBody();

		if (body->GetInvMass().m_w == dgFloat32(0.0f)) {
#ifdef _DEBUG
			for (; node; node = node->GetPrev()) {
				//dgAssert ((body->GetType() == dgBody::m_kinamticBody) ||(node->GetInfo().GetBody()->GetInvMass().m_w == dgFloat32(0.0f)));
				dgAssert(node->GetInfo().GetBody()->GetInvMass().m_w == dgFloat32(0.0f));
			}
#endif
			break;
		}

		if (body->IsRTTIType(dgBody::m_dynamicBodyRTTI)) {
			dgDynamicBody* const dynamicBody = (dgDynamicBody*)body;
			if (dynamicBody->m_dynamicsLru < lru) {
				if (!(dynamicBody->m_freeze | dynamicBody->m_spawnnedFromCallback | dynamicBody->m_sleeping)) {
					SpanningTree(dynamicBody, stackPoolBuffer, timestep);
				}
			}
			dynamicBody->m_spawnnedFromCallback = false;
		}
	}
}


void dgWorldDynamicUpdate::SpanningTree (dgDynamicBody* const body, dgDynamicBody** const queueBuffer, dgFloat32 timestep)
{
	dgInt32 bodyCount = 0;
	dgInt32 jointCount = 0;
	dgInt32 staticCount = 0;
	dgUnsigned32 lruMark = m_markLru - 1;
	dgInt32 isInEquilibrium = 1;
	
	dgFloat32 haviestMass = dgFloat32 (0.0f);

	dgDynamicBody* heaviestBody = NULL;
	dgWorld* const world = (dgWorld*) this;
	dgBodyMasterList& masterList = *world;

	dgQueue<dgDynamicBody*> queue (queueBuffer, masterList.m_constraintCount + 1024); 
	
	dgDynamicBody** const staticPool = &queue.m_pool[queue.m_mod];

	body->m_dynamicsLru = lruMark;
	
	bool hasLinksToStatic = false;
	queue.Insert (body);
	while (!queue.IsEmpty()) {
		dgInt32 count = queue.m_firstIndex - queue.m_lastIndex;
		if (count < 0) {
			dgAssert (0);
			count += queue.m_mod;
		}

		dgInt32 index = queue.m_lastIndex;
		queue.Reset ();

		for (dgInt32 j = 0; j < count; j ++) {
			dgDynamicBody* const srcBody = queue.m_pool[index];
			dgAssert (srcBody);
			dgAssert (srcBody->GetInvMass().m_w > dgFloat32 (0.0f));
			dgAssert (srcBody->m_dynamicsLru == lruMark);
			dgAssert (srcBody->m_masterNode);

			dgInt32 bodyIndex = m_bodies + bodyCount;
			world->m_bodiesMemory.ExpandCapacityIfNeessesary(bodyIndex, sizeof (dgBodyInfo));
			dgBodyInfo* const bodyArray = (dgBodyInfo*) &world->m_bodiesMemory[0]; 
			bodyArray[bodyIndex].m_body = srcBody;

			srcBody->m_sleeping = false;

			if (srcBody->m_mass.m_w > haviestMass) {
				haviestMass = srcBody->m_mass.m_w;
				heaviestBody = srcBody;
			}

			bodyCount ++;

			bool isLinkedToStatic = false;
			for (dgBodyMasterListRow::dgListNode* jointNode = srcBody->m_masterNode->GetInfo().GetFirst(); jointNode; jointNode = jointNode->GetNext()) {
				dgBodyMasterListCell* const cell = &jointNode->GetInfo();
				dgConstraint* const constraint = cell->m_joint;
				dgAssert (constraint);
				dgBody* const linkBody = cell->m_bodyNode;
				dgAssert ((constraint->m_body0 == srcBody) || (constraint->m_body1 == srcBody));
				dgAssert ((constraint->m_body0 == linkBody) || (constraint->m_body1 == linkBody));
				const dgContact* const contact = (constraint->GetId() == dgConstraint::m_contactConstraint) ? (dgContact*)constraint : NULL;
				if (linkBody->IsCollidable() && (!contact || contact->m_maxDOF || (srcBody->m_continueCollisionMode | linkBody->m_continueCollisionMode))) { 
					dgDynamicBody* const body = (dgDynamicBody*)linkBody;

					isInEquilibrium &= srcBody->m_equilibrium;
					isInEquilibrium &= srcBody->m_autoSleep;

					isInEquilibrium &= body->m_equilibrium;
					isInEquilibrium &= body->m_autoSleep;

					if (body->m_dynamicsLru < lruMark) {
						body->m_dynamicsLru = lruMark;
						if (body->m_invMass.m_w > dgFloat32 (0.0f)) { 
							queue.Insert (body);
						} else {
							hasLinksToStatic = true;
							if (!isLinkedToStatic) {
								isLinkedToStatic = true;
								staticPool[staticCount] = srcBody;
								staticCount ++;
							}
							
							dgAssert (dgInt32 (constraint->m_dynamicsLru) != m_markLru);

							dgInt32 jointIndex = m_joints + jointCount; 

							world->m_jointsMemory.ExpandCapacityIfNeessesary(jointIndex, sizeof (dgJointInfo));

							dgJointInfo* const constraintArray = (dgJointInfo*) &world->m_jointsMemory[0];
							constraintArray[jointIndex].m_joint = constraint;
							jointCount ++;

							dgAssert (constraint->m_body0);
							dgAssert (constraint->m_body1);
							if (body->GetSkeleton()) {
								for (dgBodyMasterListRow::dgListNode* jointNode = body->m_masterNode->GetInfo().GetFirst(); jointNode; jointNode = jointNode->GetNext()) {
									dgBodyMasterListCell* const cell = &jointNode->GetInfo();
									dgDynamicBody* const otherBody = (dgDynamicBody*)cell->m_bodyNode;
									if ((otherBody->GetSkeleton() == body->GetSkeleton()) && (otherBody->m_dynamicsLru < lruMark)) {
										queue.Insert (otherBody);
										otherBody->m_dynamicsLru = lruMark;
									}
								}
							}
						}

					} else if (body->m_invMass.m_w == dgFloat32 (0.0f)) { 
						hasLinksToStatic = true;
						if (!isLinkedToStatic) {
							isLinkedToStatic = true;
							staticPool[staticCount] = srcBody;
							staticCount++;
						}
					
						dgAssert (dgInt32 (constraint->m_dynamicsLru) != m_markLru);

						dgInt32 jointIndex = m_joints + jointCount; 
						world->m_jointsMemory.ExpandCapacityIfNeessesary(jointIndex, sizeof (dgJointInfo));

						dgJointInfo* const constraintArray = (dgJointInfo*) &world->m_jointsMemory[0];
						constraintArray[jointIndex].m_joint = constraint;
						jointCount ++;

						dgAssert (constraint->m_body0);
						dgAssert (constraint->m_body1);

						if (body->GetSkeleton()) {
							for (dgBodyMasterListRow::dgListNode* jointNode = body->m_masterNode->GetInfo().GetFirst(); jointNode; jointNode = jointNode->GetNext()) {
								dgBodyMasterListCell* const cell = &jointNode->GetInfo();
								dgDynamicBody* const otherBody = (dgDynamicBody*)cell->m_bodyNode;
								if ((otherBody->GetSkeleton() == body->GetSkeleton()) && (otherBody->m_dynamicsLru < lruMark)) {
									queue.Insert(otherBody);
									otherBody->m_dynamicsLru = lruMark;
								}
							}
						}
					}
				}
			}

			index ++;
			if (index >= queue.m_mod) {
				dgAssert (0);
				index = 0;
			}
		}
	}

	if (!jointCount) {
		//dgAssert (bodyCount == 1);
		if (bodyCount == 1) {
			isInEquilibrium &= body->m_equilibrium;
			isInEquilibrium &= body->m_autoSleep;
		}
	}


	dgBodyInfo* const bodyArray = (dgBodyInfo*) &world->m_bodiesMemory[0]; 
	dgJointInfo* const constraintArray = (dgJointInfo*) &world->m_jointsMemory[0];

	if (isInEquilibrium) {
		for (dgInt32 i = 0; i < bodyCount; i ++) {
			dgBody* const body = bodyArray[m_bodies + i].m_body;
			body->m_dynamicsLru = m_markLru;
			body->m_sleeping = true;
		}
	} else {
		if (world->m_islandUpdate) {
			dgIslandCallbackStruct record;
			record.m_world = world;
			record.m_count = bodyCount;
			record.m_strideInByte = sizeof (dgBodyInfo);
			record.m_bodyArray = &bodyArray[m_bodies].m_body;
			if (!world->m_islandUpdate (world, &record, bodyCount)) {
				for (dgInt32 i = 0; i < bodyCount; i ++) {
					dgBody* const body = bodyArray[m_bodies + i].m_body;
					body->m_dynamicsLru = m_markLru;
				}
				return;
			}
		}

		if (staticCount) {
			queue.Reset ();
			for (dgInt32 i = 0; i < staticCount; i ++) {
				dgDynamicBody* const body = staticPool[i];
				body->m_dynamicsLru = m_markLru;
				queue.Insert (body);
				dgAssert (dgInt32 (body->m_dynamicsLru) == m_markLru);
			}

			const dgInt32 vectorStride = dgInt32 (sizeof (dgVector) / sizeof (dgFloat32));
			for (dgInt32 i = 0; i < jointCount; i ++) {
				dgConstraint* const constraint = constraintArray[m_joints + i].m_joint;
				constraint->m_dynamicsLru = m_markLru;
				dgInt32 rows = (constraint->m_maxDOF + vectorStride - 1) & (-vectorStride);
				constraintArray[m_joints + i].m_pairCount = dgInt16 (rows);
			}
		} else {
			dgAssert (heaviestBody);
			queue.Insert (heaviestBody);
			heaviestBody->m_dynamicsLru = m_markLru;
		}

		BuildIsland (queue, timestep, jointCount);
	}
}

void dgWorldDynamicUpdate::BuildIsland (dgQueue<dgDynamicBody*>& queue, dgFloat32 timestep, dgInt32 jointCount)
{
	dgInt32 bodyCount = 1;
	dgUnsigned32 lruMark = m_markLru;

	dgWorld* const world = (dgWorld*) this;
	world->m_bodiesMemory.ExpandCapacityIfNeessesary(m_bodies, sizeof (dgBodyInfo));

	dgBodyInfo* const bodyArray0 = (dgBodyInfo*) &world->m_bodiesMemory[0]; 

	const dgInt32 vectorStride = dgInt32 (sizeof (dgVector) / sizeof (dgFloat32));

	bodyArray0[m_bodies].m_body = world->m_sentinelBody;
	dgAssert (world->m_sentinelBody->m_index == 0); 
	dgAssert (dgInt32 (world->m_sentinelBody->m_dynamicsLru) == m_markLru); 

	while (!queue.IsEmpty()) {

		dgInt32 count = queue.m_firstIndex - queue.m_lastIndex;
		if (count < 0) {
			dgAssert (0);
			count += queue.m_mod;
		}

		dgInt32 index = queue.m_lastIndex;
		queue.Reset ();

		for (dgInt32 j = 0; j < count; j ++) {

			dgDynamicBody* const body = queue.m_pool[index];
			dgAssert (body);
			dgAssert (body->m_dynamicsLru == lruMark);
			dgAssert (body->m_masterNode);

			if (body->m_invMass.m_w > dgFloat32 (0.0f)) { 
				dgInt32 bodyIndex = m_bodies + bodyCount;
				world->m_bodiesMemory.ExpandCapacityIfNeessesary(bodyIndex, sizeof (dgBodyInfo));
				dgBodyInfo* const bodyArray1 = (dgBodyInfo*) &world->m_bodiesMemory[0]; 

				body->m_index = bodyCount; 
				body->m_active = false;
				body->m_resting = true;
				bodyArray1[bodyIndex].m_body = body;
				bodyCount ++;
			}

			for (dgBodyMasterListRow::dgListNode* jointNode = body->m_masterNode->GetInfo().GetFirst(); jointNode; jointNode = jointNode->GetNext()) {
				dgBodyMasterListCell* const cell = &jointNode->GetInfo();
				dgConstraint* const constraint = cell->m_joint;
				dgAssert (constraint);
				dgBody* const linkBody = cell->m_bodyNode;
				dgAssert ((constraint->m_body0 == body) || (constraint->m_body1 == body));
				dgAssert ((constraint->m_body0 == linkBody) || (constraint->m_body1 == linkBody));
				const dgContact* const contact = (constraint->GetId() == dgConstraint::m_contactConstraint) ? (dgContact*)constraint : NULL;
				dgInt32 ccdMode = contact ? (body->m_continueCollisionMode | linkBody->m_continueCollisionMode) : 0;
				if (linkBody->IsCollidable() && (!contact || contact->m_maxDOF || ccdMode)) { 
					dgDynamicBody* const body = (dgDynamicBody*)linkBody;

					if (constraint->m_dynamicsLru != lruMark) {
						constraint->m_dynamicsLru = lruMark;

						dgInt32 jointIndex = m_joints + jointCount; 
						world->m_jointsMemory.ExpandCapacityIfNeessesary(jointIndex, sizeof (dgJointInfo));

						dgJointInfo* const constraintArray = (dgJointInfo*) &world->m_jointsMemory[0];
						constraintArray[jointIndex].m_joint = constraint;
						dgInt32 rows = (constraint->m_maxDOF + vectorStride - 1) & (-vectorStride);
 						constraintArray[jointIndex].m_pairCount = dgInt16 (rows);
						jointCount ++;

						dgAssert (constraint->m_body0);
						dgAssert (constraint->m_body1);
					}

					if (body->m_dynamicsLru != lruMark) {
						if (body->m_invMass.m_w > dgFloat32 (0.0f)) { 
							queue.Insert (body);
							body->m_dynamicsLru = lruMark;
						}
					}
				}
			}

			index ++;
			if (index >= queue.m_mod) {
				dgAssert (0);
				index = 0;
			}
		}
	}

	if (bodyCount > 1) {
		m_islandMemory[m_islands].m_bodyStart = m_bodies;
		m_islandMemory[m_islands].m_jointStart = m_joints;
		m_islandMemory[m_islands].m_bodyCount = bodyCount;
		m_islandMemory[m_islands].m_jointCount = jointCount;
		
		m_islandMemory[m_islands].m_rowsStart = 0;
		m_islandMemory[m_islands].m_isContinueCollision = false;

		dgJointInfo* const constraintArrayPtr = (dgJointInfo*) &world->m_jointsMemory[0];
		dgJointInfo* const constraintArray = &constraintArrayPtr[m_joints];

		dgInt32 rowsCount = 0;
		dgInt32 isContinueCollisionIsland = 0;
		for (dgInt32 i = 0; i < jointCount; i ++) {
			dgConstraint* const joint = constraintArray[i].m_joint;
			rowsCount += constraintArray[i].m_pairCount;
			if (joint->GetId() == dgConstraint::m_contactConstraint) {
				const dgBody* const body0 = joint->m_body0;
				const dgBody* const body1 = joint->m_body1;
				if (body0->m_continueCollisionMode | body1->m_continueCollisionMode) {
					dgInt32 ccdJoint = false;
					const dgVector& veloc0 = body0->m_veloc;
					const dgVector& veloc1 = body1->m_veloc;

					const dgVector& omega0 = body0->m_omega;
					const dgVector& omega1 = body1->m_omega;

					const dgVector& com0 = body0->m_globalCentreOfMass;
					const dgVector& com1 = body1->m_globalCentreOfMass;

					const dgCollisionInstance* const collision0 = body0->m_collision;
					const dgCollisionInstance* const collision1 = body1->m_collision;
					dgFloat32 dist = dgMax (body0->m_collision->GetBoxMinRadius(), body1->m_collision->GetBoxMinRadius()) * dgFloat32 (0.25f);

					dgVector relVeloc (veloc1 - veloc0);
					dgVector relOmega (omega1 - omega0);
					dgVector relVelocMag2 (relVeloc.DotProduct4 (relVeloc));
					dgVector relOmegaMag2 (relOmega.DotProduct4 (relOmega));

					if ((relOmegaMag2.m_w > dgFloat32 (1.0f)) || ((relVelocMag2.m_w * timestep * timestep) > (dist * dist))) {
						dgTriplex normals[16];
						dgTriplex points[16];
						dgInt64 attrib0[16];
						dgInt64 attrib1[16];
						dgFloat32 penetrations[16];
						dgFloat32 timeToImpact = timestep;
						const dgInt32 ccdContactCount = world->CollideContinue (collision0, body0->m_matrix, veloc0, omega0, collision1, body1->m_matrix, veloc1, omega1, 
																				timeToImpact, points, normals, penetrations, attrib0, attrib1, 6, 0);

						for (dgInt32 j = 0; j < ccdContactCount; j ++) {
							dgVector point (&points[j].m_x);
							dgVector normal (&normals[j].m_x);
							dgVector vel0 (veloc0 + omega0 * (point - com0));
							dgVector vel1 (veloc1 + omega1 * (point - com1));
							dgVector vRel (vel1 - vel0);
							dgFloat32 contactDistTravel = vRel.DotProduct4(normal).m_w * timestep;
							ccdJoint |= (contactDistTravel > dist);
						}
					}
					//ccdJoint = body0->m_continueCollisionMode | body1->m_continueCollisionMode;
					isContinueCollisionIsland |= ccdJoint;
					rowsCount += DG_CCD_EXTRA_CONTACT_COUNT;
				}
			}
		}
		if (isContinueCollisionIsland) {
			rowsCount = dgMax(rowsCount, 64);
		}
		m_islandMemory[m_islands].m_rowsCount = rowsCount;
		m_islandMemory[m_islands].m_isContinueCollision = isContinueCollisionIsland;

		m_islands ++;
		m_bodies += bodyCount;
		m_joints += jointCount;
	}
}


void dgWorldDynamicUpdate::FindActiveJointAndBodies (dgIsland* const island)
{
	dgWorld* const world = (dgWorld*) this;
	dgBodyInfo* const bodyArrayPtr = (dgBodyInfo*) &world->m_bodiesMemory[0]; 
	dgBodyInfo* const bodyArray = &bodyArrayPtr[island->m_bodyStart];

	dgJointInfo* const constraintArrayPtr = (dgJointInfo*) &world->m_jointsMemory[0];
	dgJointInfo* const constraintArray = &constraintArrayPtr[island->m_jointStart];


	dgInt32 jointCount = island->m_jointCount;
//	if (jointCount > 100000000) {
	if (jointCount > DG_SMALL_ISLAND_COUNT) {
		for (dgInt32 i = 0; i < jointCount; i ++) {
			dgJointInfo* const jointInfo = &constraintArray[i];
			dgConstraint* const joint = jointInfo->m_joint;

			dgInt32 m0 = (joint->m_body0->GetInvMass().m_w != dgFloat32(0.0f)) ? joint->m_body0->m_index: 0;
			dgInt32 m1 = (joint->m_body1->GetInvMass().m_w != dgFloat32(0.0f)) ? joint->m_body1->m_index: 0;

			jointInfo->m_m0 = m0;
			jointInfo->m_m1 = m1;

			dgBody* const body0 = bodyArray[m0].m_body;
			dgBody* const body1 = bodyArray[m1].m_body;

			bool resting = body0->m_equilibrium & body1->m_equilibrium & ((joint->GetId() == dgConstraint::m_contactConstraint) ? true : false);
			body0->m_resting &= (resting | !m0);
			body1->m_resting &= (resting | !m1);
			body0->m_active |= !(!m0);
			body1->m_active |= !(!m1);
			joint->m_solverActive = true;
		}
	} else if (jointCount){
		for (dgInt32 i = 0; i < jointCount; i ++) {
			dgJointInfo* const jointInfo = &constraintArray[i];
			dgConstraint* const joint = jointInfo->m_joint;
			dgInt32 m0 = (joint->m_body0->GetInvMass().m_w != dgFloat32(0.0f)) ? joint->m_body0->m_index: 0;
			dgInt32 m1 = (joint->m_body1->GetInvMass().m_w != dgFloat32(0.0f)) ? joint->m_body1->m_index: 0;
			jointInfo->m_m0 = m0;
			jointInfo->m_m1 = m1;
			joint->m_solverActive = true;
		}

		dgInt32 bodyCount = island->m_bodyCount;
		for (dgInt32 i = 1; i < bodyCount; i ++) {
			dgBody* const body = bodyArray[i].m_body;
			body->m_active = true;
			//body->m_resting = body->m_equilibrium;
			body->m_resting = false;
		}

	} else {
		dgAssert (island->m_bodyCount == 2);
		dgBody* const body = bodyArray[1].m_body;
		body->m_active = true;
		//body->m_resting = body->m_equilibrium;
		body->m_resting = false;
	}
}

dgBody* dgWorldDynamicUpdate::GetIslandBody(const void* const islandPtr, dgInt32 index) const
{
	const dgIslandCallbackStruct* const island = (dgIslandCallbackStruct*)islandPtr;

	char* const ptr = &((char*)island->m_bodyArray)[island->m_strideInByte * index];
	dgBody** const bodyPtr = (dgBody**)ptr;
	return (index < island->m_count) ? ((index >= 0) ? *bodyPtr : NULL) : NULL;
}


// sort from high to low
dgInt32 dgWorldDynamicUpdate::CompareIslands(const dgIsland* const islandA, const dgIsland* const islandB, void* notUsed)
{
	//dgInt32 countA = islandA->m_jointCount + (islandA->m_hasExactSolverJoints << 28);
	//dgInt32 countB = islandB->m_jointCount + (islandB->m_hasExactSolverJoints << 28);
	dgInt32 countA = islandA->m_jointCount;
	dgInt32 countB = islandB->m_jointCount;

	if (countA < countB) {
		return 1;
	}
	if (countA > countB) {
		return -1;
	}
	return 0;
}


void dgWorldDynamicUpdate::FindActiveJointAndBodiesKernel (void* const context, void* const worldContext, dgInt32 threadID)
{
	dTimeTrackerEvent(__FUNCTION__);
	dgWorldDynamicUpdateSyncDescriptor* const descriptor = (dgWorldDynamicUpdateSyncDescriptor*) context;

	dgWorld* const world = (dgWorld*) worldContext;
	dgInt32 count = descriptor->m_islandCount;
	dgIsland* const islands = &((dgIsland*)&world->m_islandMemory[0])[descriptor->m_firstIsland];

	for (dgInt32 i = dgAtomicExchangeAndAdd(&descriptor->m_atomicCounter, 1); i < count; i = dgAtomicExchangeAndAdd(&descriptor->m_atomicCounter, 1)) {
		dgIsland* const island = &islands[i]; 
		world->FindActiveJointAndBodies (island); 
	}
}


void dgWorldDynamicUpdate::CalculateIslandReactionForcesKernel (void* const context, void* const worldContext, dgInt32 threadID)
{
	dTimeTrackerEvent(__FUNCTION__);
	dgWorldDynamicUpdateSyncDescriptor* const descriptor = (dgWorldDynamicUpdateSyncDescriptor*) context;

	dgFloat32 timestep = descriptor->m_timestep;
	dgWorld* const world = (dgWorld*) worldContext;
	dgInt32 count = descriptor->m_islandCount;
	dgIsland* const islands = &((dgIsland*)&world->m_islandMemory[0])[descriptor->m_firstIsland];

	for (dgInt32 i = dgAtomicExchangeAndAdd(&descriptor->m_atomicCounter, 1); i < count; i = dgAtomicExchangeAndAdd(&descriptor->m_atomicCounter, 1)) {
		dgIsland* const island = &islands[i]; 
		world->CalculateIslandReactionForces (island, timestep, threadID);
	}
}

dgInt32 dgWorldDynamicUpdate::GetJacobianDerivatives (dgContraintDescritor& constraintParamOut, dgJointInfo* const jointInfo, dgConstraint* const constraint, dgJacobianMatrixElement* const matrixRow, dgInt32 rowCount) const
{
	if (constraint->m_solverActive) {
		dgInt32 dof = dgInt32(constraint->m_maxDOF);
		dgAssert(dof <= DG_CONSTRAINT_MAX_ROWS);
		for (dgInt32 i = 0; i < dof; i++) {
			constraintParamOut.m_forceBounds[i].m_low = DG_MIN_BOUND;
			constraintParamOut.m_forceBounds[i].m_upper = DG_MAX_BOUND;
			constraintParamOut.m_forceBounds[i].m_jointForce = NULL;
			constraintParamOut.m_forceBounds[i].m_normalIndex = DG_BILATERAL_CONSTRAINT;
		}

		dgAssert(constraint->m_body0);
		dgAssert(constraint->m_body1);

		constraint->m_body0->m_inCallback = true;
		constraint->m_body1->m_inCallback = true;

		dof = dgInt32(constraint->JacobianDerivative(constraintParamOut));

		constraint->m_body0->m_inCallback = false;
		constraint->m_body1->m_inCallback = false;

		//dgAssert(jointInfo->m_m0 < island->m_bodyCount);
		//dgAssert(jointInfo->m_m1 < island->m_bodyCount);
		//dgAssert(constraint->m_index == dgUnsigned32(j));

		jointInfo->m_pairStart = rowCount;
		jointInfo->m_pairCount = dgInt16 (dof);
		jointInfo->m_pairActiveCount = dgInt16 (dof);

		for (dgInt32 i = 0; i < dof; i++) {
			dgJacobianMatrixElement* const row = &matrixRow[rowCount];
			dgAssert(constraintParamOut.m_forceBounds[i].m_jointForce);
			row->m_Jt = constraintParamOut.m_jacobian[i];

			//dgAssert(constraintParamOut.m_jointStiffness[i] >= dgFloat32(0.0f));
			//dgAssert(constraintParamOut.m_jointStiffness[i] <= dgFloat32(1.0f));

			//row->m_stiffness = constraintParamOut.m_jointStiffness[i];
			row->m_diagDamp = dgFloat32 (0.0f);
			row->m_stiffness = DG_PSD_DAMP_TOL * (dgFloat32 (1.0f) - constraintParamOut.m_jointStiffness[i]) + dgFloat32 (1.0e-6f);
			dgAssert(row->m_stiffness >= dgFloat32(0.0f));
			row->m_coordenateAccel = constraintParamOut.m_jointAccel[i];
			row->m_accelIsMotor = constraintParamOut.m_flags[i].m_isMotor ? true : false;
			row->m_applyCorrection = constraintParamOut.m_flags[i].m_applyCorrection ? true : false;
			row->m_restitution = constraintParamOut.m_restitution[i];
			row->m_penetration = constraintParamOut.m_penetration[i];
			row->m_penetrationStiffness = constraintParamOut.m_penetrationStiffness[i];
			row->m_lowerBoundFrictionCoefficent = constraintParamOut.m_forceBounds[i].m_low;
			row->m_upperBoundFrictionCoefficent = constraintParamOut.m_forceBounds[i].m_upper;
			row->m_jointFeebackForce = constraintParamOut.m_forceBounds[i].m_jointForce;

			row->m_normalForceIndex = constraintParamOut.m_forceBounds[i].m_normalIndex;
			rowCount++;
		}
		rowCount = (rowCount & (dgInt32(sizeof (dgVector) / sizeof (dgFloat32)) - 1)) ? ((rowCount & (-dgInt32(sizeof (dgVector) / sizeof (dgFloat32)))) + dgInt32(sizeof (dgVector) / sizeof (dgFloat32))) : rowCount;
		dgAssert((rowCount & (dgInt32(sizeof (dgVector) / sizeof (dgFloat32)) - 1)) == 0);
	}
	return rowCount;
}

void dgWorldDynamicUpdate::GetJacobianDerivatives (const dgIsland* const island, dgInt32 threadIndex, dgInt32 rowCount, dgFloat32 timestep) const
{
	dgContraintDescritor constraintParams;
	dgWorld* const world = (dgWorld*) this;

	constraintParams.m_world = world; 
	constraintParams.m_threadIndex = threadIndex;
	constraintParams.m_timestep = timestep;
	constraintParams.m_invTimestep = (timestep > dgFloat32 (1.0e-5f)) ? dgFloat32 (1.0f / timestep) : dgFloat32 (0.0f);

	dgJointInfo* const constraintArrayPtr = (dgJointInfo*) &world->m_jointsMemory[0];
	dgJointInfo* const constraintArray = &constraintArrayPtr[island->m_jointStart];

	dgJacobianMatrixElement* const matrixRow = &m_solverMemory.m_memory[island->m_rowsStart];
	dgInt32 jointCount = island->m_jointCount;
	for (dgInt32 j = 0; j < jointCount; j ++) {
		dgJointInfo* const jointInfo = &constraintArray[j];
		dgConstraint* const constraint = jointInfo->m_joint;

		dgAssert(jointInfo->m_m0 < island->m_bodyCount);
		dgAssert(jointInfo->m_m1 < island->m_bodyCount);
		//dgAssert (constraint->m_index == dgUnsigned32(j));

		rowCount = GetJacobianDerivatives (constraintParams, jointInfo, constraint, matrixRow, rowCount);
		dgAssert (rowCount <= island->m_rowsCount);
	}
}

void dgWorldDynamicUpdate::IntegrateArray (const dgIsland* const island, dgFloat32 accelTolerance, dgFloat32 timestep, dgInt32 threadIndex) const
{
	bool isAutoSleep = true;
	bool stackSleeping = true;
	dgInt32 sleepCounter = 10000;

	dgWorld* const world = (dgWorld*) this;

	dgFloat32 forceDamp = DG_FREEZZING_VELOCITY_DRAG;
	dgBodyInfo* const bodyArrayPtr = (dgBodyInfo*) &world->m_bodiesMemory[0]; 
	dgBodyInfo* const bodyArray = &bodyArrayPtr[island->m_bodyStart + 1]; 
	dgInt32 count = island->m_bodyCount - 1;
	if (count <= 2) {
		bool autosleep = bodyArray[0].m_body->m_autoSleep;
		if (count == 2) {
			autosleep &= bodyArray[1].m_body->m_autoSleep;
		}
		if (!autosleep) {
			forceDamp = dgFloat32 (0.9999f);
		}
	}

	dgFloat32 maxAccel = dgFloat32 (0.0f);
	dgFloat32 maxAlpha = dgFloat32 (0.0f);
	dgFloat32 maxSpeed = dgFloat32 (0.0f);
	dgFloat32 maxOmega = dgFloat32 (0.0f);

	dgFloat32 speedFreeze = world->m_freezeSpeed2;
	dgFloat32 accelFreeze = world->m_freezeAccel2 * ((island->m_jointCount <= DG_SMALL_ISLAND_COUNT) ? dgFloat32 (0.05f) : dgFloat32 (1.0f));
	dgVector forceDampVect (forceDamp, forceDamp, forceDamp, dgFloat32 (0.0f));
	for (dgInt32 i = 0; i < count; i ++) {
		dgDynamicBody* const body = (dgDynamicBody*) bodyArray[i].m_body;

		dgVector isMovingMask = (body->m_veloc | body->m_omega | body->m_accel | body->m_alpha) & dgVector::m_signMask;
		//if (((isMovingMask.TestZero().GetSignMask() & 7) != 7) && body->IsRTTIType (dgBody::m_dynamicBodyRTTI)) {
		if (body->IsRTTIType (dgBody::m_dynamicBodyRTTI) && (!body->m_equilibrium || ((isMovingMask.TestZero().GetSignMask() & 7) != 7))) {
			dgAssert (body->m_invMass.m_w);
			body->IntegrateVelocity(timestep);

			dgFloat32 accel2 = body->m_accel % body->m_accel;
			dgFloat32 alpha2 = body->m_alpha % body->m_alpha;
			dgFloat32 speed2 = body->m_veloc % body->m_veloc;
			dgFloat32 omega2 = body->m_omega % body->m_omega;

			maxAccel = dgMax (maxAccel, accel2);
			maxAlpha = dgMax (maxAlpha, alpha2);
			maxSpeed = dgMax (maxSpeed, speed2);
			maxOmega = dgMax (maxOmega, omega2);

			bool equilibrium = (accel2 < accelFreeze) && (alpha2 < accelFreeze) && (speed2 < speedFreeze) && (omega2 < speedFreeze);

			if (equilibrium) {
				dgVector veloc (body->m_veloc.CompProduct4(forceDampVect));
				dgVector omega = body->m_omega.CompProduct4 (forceDampVect);
				body->m_veloc = (dgVector (veloc.DotProduct4(veloc)) > m_velocTol) & veloc;
				body->m_omega = (dgVector (omega.DotProduct4(omega)) > m_velocTol) & omega;
			}

			body->m_equilibrium = dgUnsigned32 (equilibrium);
			stackSleeping &= equilibrium;
			isAutoSleep &= body->m_autoSleep;

			sleepCounter = dgMin (sleepCounter, body->m_sleepingCounter);

			body->UpdateMatrix (timestep, threadIndex);
		}
	}

	if (isAutoSleep && (island->m_jointCount > DG_SMALL_ISLAND_COUNT)) {
		if (stackSleeping) {
			for (dgInt32 i = 0; i < count; i ++) {
				dgBody* const body = (dgDynamicBody*) bodyArray[i].m_body;
				if (body->m_active && body->IsRTTIType (dgBody::m_dynamicBodyRTTI)) {
					body->m_netForce = dgVector::m_zero;
					body->m_netTorque = dgVector::m_zero;
					body->m_veloc = dgVector::m_zero;
					body->m_omega = dgVector::m_zero;
				}
			}
		} else {
			if ((maxAccel > world->m_sleepTable[DG_SLEEP_ENTRIES - 1].m_maxAccel) ||
				(maxAlpha > world->m_sleepTable[DG_SLEEP_ENTRIES - 1].m_maxAlpha) ||
				(maxSpeed > world->m_sleepTable[DG_SLEEP_ENTRIES - 1].m_maxVeloc) ||
				(maxOmega > world->m_sleepTable[DG_SLEEP_ENTRIES - 1].m_maxOmega)) {
					for (dgInt32 i = 0; i < count; i ++) {
						dgDynamicBody* const body = (dgDynamicBody*) bodyArray[i].m_body;
						if (body->m_active && body->IsRTTIType (dgBody::m_dynamicBodyRTTI)) {
							body->m_sleepingCounter = 0;
						}
					}
			} else {
				dgInt32 index = 0;
				for (dgInt32 i = 0; i < DG_SLEEP_ENTRIES; i ++) {
					if ((maxAccel <= world->m_sleepTable[i].m_maxAccel) &&
						(maxAlpha <= world->m_sleepTable[i].m_maxAlpha) &&
						(maxSpeed <= world->m_sleepTable[i].m_maxVeloc) &&
						(maxOmega <= world->m_sleepTable[i].m_maxOmega)) {
							index = i;
							break;
					}
				}

				dgInt32 timeScaleSleepCount = dgInt32 (dgFloat32 (60.0f) * sleepCounter * timestep);
				if (timeScaleSleepCount > world->m_sleepTable[index].m_steps) {
					for (dgInt32 i = 0; i < count; i ++) {
						dgBody* const body = (dgDynamicBody*) bodyArray[i].m_body;
						if (body->m_active && body->IsRTTIType (dgBody::m_dynamicBodyRTTI)) {
							body->m_netForce = dgVector::m_zero;
							body->m_netTorque = dgVector::m_zero;
							body->m_veloc = dgVector::m_zero;
							body->m_omega = dgVector::m_zero;
							body->m_equilibrium = true;
						}
					}
				} else {
					sleepCounter ++;
					for (dgInt32 i = 0; i < count; i ++) {
						dgDynamicBody* const body = (dgDynamicBody*) bodyArray[i].m_body;
						if (body->m_active && body->IsRTTIType (dgBody::m_dynamicBodyRTTI)) {
							body->m_sleepingCounter = sleepCounter;
						}
					}
				}
			}
		}
	}
}

void dgJacobianMemory::Init(dgWorld* const world, dgInt32 rowsCount, dgInt32 bodyCount)
{
	world->m_solverMatrixMemory.ExpandCapacityIfNeessesary(rowsCount, sizeof (dgJacobianMatrixElement));
	m_memory = (dgJacobianMatrixElement*)&world->m_solverMatrixMemory[0];

	world->m_solverRightSideMemory.ExpandCapacityIfNeessesary(bodyCount + 8, sizeof (dgJacobian));
	m_internalForces = (dgJacobian*)&world->m_solverRightSideMemory[0];
	dgAssert(bodyCount <= (((world->m_solverRightSideMemory.GetBytesCapacity() - 16) / dgInt32(sizeof (dgJacobian))) & (-8)));

	dgAssert((dgUnsigned64(m_memory) & 0x01f) == 0);
	dgAssert((dgUnsigned64(m_internalForces) & 0x01f) == 0);
}


