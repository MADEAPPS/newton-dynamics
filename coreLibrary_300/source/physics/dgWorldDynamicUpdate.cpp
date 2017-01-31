/* Copyright (c) <2003-2016> <Julio Jerez, Newton Game Dynamics>
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
#include "dgSkeletonContainer.h"
#include "dgWorldDynamicUpdate.h"
#include "dgBilateralConstraint.h"
#include "dgCollisionDeformableMesh.h"

#define DG_CCD_EXTRA_CONTACT_COUNT			(8 * 3)
#define DG_PARALLEL_JOINT_COUNT_CUT_OFF		(256)
#define DG_HEAVY_MASS_SCALE_FACTOR			(25.0f)
#define DG_LARGE_STACK_DAMP_FACTOR			(0.25f)

dgVector dgWorldDynamicUpdate::m_velocTol (dgFloat32 (1.0e-8f));


class dgWorldDynamicUpdateSyncDescriptor
{
	public:
	dgWorldDynamicUpdateSyncDescriptor()
	{
		memset (this, 0, sizeof (dgWorldDynamicUpdateSyncDescriptor));
	}

	dgFloat32 m_timestep;
	dgInt32 m_atomicCounter;
	
	dgInt32 m_clusterCount;
	dgInt32 m_firstCluster;
	dgThread::dgCriticalSection* m_criticalSection;
};


//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

dgWorldDynamicUpdate::dgWorldDynamicUpdate()
	:m_bodies(0)
	,m_joints(0)
	,m_clusters(0)
	,m_markLru(0)
	,m_softBodyCriticalSectionLock()
	,m_clusterMemory(NULL)
{
}

void dgWorldDynamicUpdate::UpdateDynamics(dgFloat32 timestep)
{
	dTimeTrackerEvent(__FUNCTION__);
	dgWorld* const world = (dgWorld*) this;
	
	UpdateSkeletons();
	
	m_bodies = 0;
	m_joints = 0;
	m_clusters = 0;
	m_solverConvergeQuality = world->m_solverConvergeQuality;
	world->m_dynamicsLru = world->m_dynamicsLru + DG_BODY_LRU_STEP;
	m_markLru = world->m_dynamicsLru;

	dgDynamicBody* const sentinelBody = world->m_sentinelBody;
	sentinelBody->m_index = 0; 
	sentinelBody->m_dynamicsLru = m_markLru;

	BuildClusters(timestep);
	SortClustersByCount();

	dgInt32 maxRowCount = 0;
	dgInt32 blockMatrixSize = 0;
	dgInt32 softBodiesCount = 0;
	for (dgInt32 i = 0; i < m_clusters; i ++) {
		dgBodyCluster& cluster = m_clusterMemory[i];
		cluster.m_rowsStart = maxRowCount;
		maxRowCount += cluster.m_rowsCount;
		softBodiesCount += cluster.m_hasSoftBodies;
	}
	m_solverMemory.Init (world, maxRowCount, m_bodies, blockMatrixSize);

	dgInt32 threadCount = world->GetThreadCount();	

	dgWorldDynamicUpdateSyncDescriptor descriptor;
	descriptor.m_timestep = timestep;

	dgInt32 index = softBodiesCount;

	descriptor.m_atomicCounter = 0;
	descriptor.m_firstCluster = index;
	descriptor.m_clusterCount = m_clusters - index;
	sentinelBody->m_sleeping = true;
	sentinelBody->m_equilibrium = true;

	dgInt32 useParallel = world->m_useParallelSolver && (threadCount > 1);
	//useParallel = 1;
	if (useParallel) {
		dgInt32 sum = m_joints;
		useParallel = useParallel && m_joints && m_clusters;
		useParallel = useParallel && ((threadCount * m_clusterMemory[0].m_jointCount) >= sum);
		useParallel = useParallel && (m_clusterMemory[0].m_jointCount > DG_PARALLEL_JOINT_COUNT_CUT_OFF);
		//useParallel = 1;
		while (useParallel) {
			CalculateReactionForcesParallel(&m_clusterMemory[index], timestep);
			index ++;
			sum -= m_clusterMemory[index].m_jointCount;
			useParallel = useParallel && (index < m_clusters);
			useParallel = useParallel && ((threadCount * m_clusterMemory[index].m_jointCount) >= m_joints);
			useParallel = useParallel && (m_clusterMemory[index].m_jointCount > DG_PARALLEL_JOINT_COUNT_CUT_OFF);
		}
	}

	if (index < m_clusters) {
		descriptor.m_atomicCounter = 0;
		descriptor.m_firstCluster = index;
		descriptor.m_clusterCount = m_clusters - index;
		for (dgInt32 i = 0; i < threadCount; i ++) {
			world->QueueJob (CalculateClusterReactionForcesKernel, &descriptor, world);
		}
		world->SynchronizationBarrier();
	}

	dgBodyInfo* const bodyArrayPtr = (dgBodyInfo*)&world->m_bodiesMemory[0];
	for (dgInt32 i = 0; i < softBodiesCount; i++) {
		dgBodyCluster* const cluster = &m_clusterMemory[i];
//		IntegrateExternalForce(cluster, timestep, 0);
		dgBodyInfo* const bodyArray = &bodyArrayPtr[cluster->m_bodyStart];
		dgAssert (cluster->m_bodyCount == 2);
		dgDynamicBody* const body = (dgDynamicBody*)bodyArray[1].m_body;
		dgAssert (body->m_collision->IsType(dgCollision::dgCollisionLumpedMass_RTTI));
		body->IntegrateOpenLoopExternalForce(timestep);
		IntegrateVelocity(cluster, DG_SOLVER_MAX_ERROR, timestep, 0);
	}

	m_clusterMemory = NULL;
}

void dgWorldDynamicUpdate::SortClustersByCount ()
{
	dTimeTrackerEvent(__FUNCTION__);
	dgSort(m_clusterMemory, m_clusters, CompareClusters);
}

void dgWorldDynamicUpdate::BuildClusters(dgFloat32 timestep)
{
	dTimeTrackerEvent(__FUNCTION__);

	dgWorld* const world = (dgWorld*) this;
	dgUnsigned32 lru = m_markLru - 1;

	dgBodyMasterList& masterList = *world;

	dgAssert (masterList.GetFirst()->GetInfo().GetBody() == world->m_sentinelBody);
	world->m_solverJacobiansMemory.ResizeIfNecessary ((2 * (masterList.m_constraintCount + 1024)) * sizeof (dgDynamicBody*));
	dgDynamicBody** const stackPoolBuffer = (dgDynamicBody**)&world->m_solverJacobiansMemory[0];

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
	dgInt32 stack = 1;
	dgInt32 bodyCount = 1;
	dgInt32 jointCount = 0;
	dgInt32 isInEquilibrium = 1;
	dgInt32 hasSoftBodies = 0;

	dgWorld* const world = (dgWorld*) this;
	const dgInt32 clusterLRU = world->m_clusterLRU;
	const dgUnsigned32 lruMark = m_markLru - 1;

	world->m_clusterLRU ++;

	queueBuffer[0] = body;
	world->m_bodiesMemory.ResizeIfNecessary ((m_bodies + 1) * sizeof (dgBodyInfo));
	dgBodyInfo* const bodyArray0 = (dgBodyInfo*)&world->m_bodiesMemory[0];

	bodyArray0[m_bodies].m_body = world->m_sentinelBody;
	dgAssert(world->m_sentinelBody->m_index == 0);
	dgAssert(dgInt32(world->m_sentinelBody->m_dynamicsLru) == m_markLru);
	const dgInt32 vectorStride = dgInt32 (sizeof (dgVector) / sizeof (dgFloat32));

	dgInt32 activeJointCount = 0;
	bool globalAutoSleep = true;
	while (stack) {
		stack --;
		dgDynamicBody* const srcBody = queueBuffer[stack];

		if (srcBody->m_dynamicsLru < lruMark) {
			dgAssert(srcBody->GetInvMass().m_w > dgFloat32(0.0f));
			dgAssert(srcBody->m_masterNode);

			dgInt32 bodyIndex = m_bodies + bodyCount;
			world->m_bodiesMemory.ResizeIfNecessary ((bodyIndex + 1) * sizeof (dgBodyInfo));
			dgBodyInfo* const bodyArray1 = (dgBodyInfo*)&world->m_bodiesMemory[0];
			bodyArray1[bodyIndex].m_body = srcBody;
			isInEquilibrium &= srcBody->m_equilibrium;
			globalAutoSleep &= (srcBody->m_autoSleep & srcBody->m_equilibrium); 
			
			srcBody->m_index = bodyCount;
			srcBody->m_dynamicsLru = lruMark;
			hasSoftBodies |= (srcBody->m_collision->IsType(dgCollision::dgCollisionDeformableMesh_RTTI) ? 1 : 0);

			srcBody->m_sleeping = false;
			const bool equilibrium0 = srcBody->m_equilibrium & !srcBody->GetSkeleton();

			bodyCount++;
			for (dgBodyMasterListRow::dgListNode* jointNode = srcBody->m_masterNode->GetInfo().GetFirst(); jointNode; jointNode = jointNode->GetNext()) {

				dgBodyMasterListCell* const cell = &jointNode->GetInfo();
				dgConstraint* const constraint = cell->m_joint;
				dgAssert(constraint);
				dgBody* const linkBody = cell->m_bodyNode;
				dgAssert((constraint->m_body0 == srcBody) || (constraint->m_body1 == srcBody));
				dgAssert((constraint->m_body0 == linkBody) || (constraint->m_body1 == linkBody));
				const dgContact* const contact = (constraint->GetId() == dgConstraint::m_contactConstraint) ? (dgContact*)constraint : NULL;

				bool check0 = linkBody->IsCollidable();
				check0 = check0 && (!contact || (contact->m_contactActive && contact->m_maxDOF) || (srcBody->m_continueCollisionMode | linkBody->m_continueCollisionMode));
				if (check0) {
					bool check1 = constraint->m_dynamicsLru != lruMark;
					if (check1) {
						const dgInt32 jointIndex = m_joints + jointCount;
						world->m_jointsMemory.ResizeIfNecessary ((jointIndex + 1) * sizeof (dgJointInfo));
						dgJointInfo* const constraintArray = (dgJointInfo*)&world->m_jointsMemory[0];

						constraint->m_index = jointCount;
						constraint->m_clusterLRU = clusterLRU;
						constraint->m_dynamicsLru = lruMark;

						constraintArray[jointIndex].m_joint = constraint;
						const dgInt32 rows = (constraint->m_maxDOF + vectorStride - 1) & (-vectorStride);
						constraintArray[jointIndex].m_pairCount = dgInt16(rows);

						//constraintArray[jointIndex].m_isSkeleton = sourceSkel && constraint->IsBilateral() && (sourceSkel == linkBody->GetSkeleton());
						constraintArray[jointIndex].m_isSkeleton = constraint->IsSkeleton();
						constraintArray[jointIndex].m_isFrontier = equilibrium0 ^ linkBody->m_equilibrium;

						const bool isEquilibrium = equilibrium0 & linkBody->m_equilibrium & !linkBody->GetSkeleton();
						if (!isEquilibrium) {
							if (jointIndex != m_joints + activeJointCount) {
								dgSwap(constraintArray[jointIndex], constraintArray[m_joints + activeJointCount]);
								dgSwap(constraintArray[jointIndex].m_joint->m_index, constraintArray[m_joints + activeJointCount].m_joint->m_index);
							}
							activeJointCount++;	
						}
						jointCount++;

						dgAssert(constraint->m_body0);
						dgAssert(constraint->m_body1);
					}

					dgDynamicBody* const adjacentBody = (dgDynamicBody*)linkBody;
	
					if (adjacentBody->GetSkeleton() && (adjacentBody->GetInvMass().m_w == dgFloat32(0.0f))) {
						for (dgBodyMasterListRow::dgListNode* jointNode = adjacentBody->m_masterNode->GetInfo().GetFirst(); jointNode; jointNode = jointNode->GetNext()) {
							dgBodyMasterListCell* const cell = &jointNode->GetInfo();
							dgDynamicBody* const otherBody = (dgDynamicBody*)cell->m_bodyNode;
							if ((otherBody->GetSkeleton() == body->GetSkeleton()) && (otherBody->m_dynamicsLru != lruMark)) {
								queueBuffer[stack] = otherBody;
								stack ++;
							}
						}
					}

					if ((adjacentBody->m_dynamicsLru != lruMark) && (adjacentBody->GetInvMass().m_w > dgFloat32(0.0f))) {
						queueBuffer[stack] = adjacentBody;
						stack ++;
					}
				}
			}
		}
	}


	dgBodyInfo* const bodyArray = (dgBodyInfo*) &world->m_bodiesMemory[0]; 
	if (globalAutoSleep) {
		for (dgInt32 i = 1; i < bodyCount; i++) {
			dgBody* const body = bodyArray[m_bodies + i].m_body;
			body->m_dynamicsLru = m_markLru;
			body->m_sleeping = globalAutoSleep;
		}
	} else {
		if (world->m_clusterUpdate) {
			dgClusterCallbackStruct record;
			record.m_world = world;
			record.m_count = bodyCount;
			record.m_strideInByte = sizeof (dgBodyInfo);
			record.m_bodyArray = &bodyArray[m_bodies].m_body;
			if (!world->m_clusterUpdate(world, &record, bodyCount)) {
				for (dgInt32 i = 0; i < bodyCount; i++) {
					dgBody* const body = bodyArray[m_bodies + i].m_body;
					body->m_dynamicsLru = m_markLru;
				}
				return;
			}
		}

		world->m_clusterMemory.ResizeIfNecessary  ((m_clusters + 1) * sizeof (dgBodyCluster));
		m_clusterMemory = (dgBodyCluster*) &world->m_clusterMemory[0];
		dgBodyCluster& cluster = m_clusterMemory[m_clusters];

		cluster.m_bodyStart = m_bodies;
		cluster.m_jointStart = m_joints;
		cluster.m_bodyCount = bodyCount;
		cluster.m_clusterLRU = clusterLRU;
		cluster.m_jointCount = jointCount;
		cluster.m_activeJointCount = activeJointCount;
		
		cluster.m_rowsStart = 0;
		cluster.m_isContinueCollision = 0;
		cluster.m_hasSoftBodies = dgInt16 (hasSoftBodies);

		dgJointInfo* const constraintArrayPtr = (dgJointInfo*)&world->m_jointsMemory[0];
		dgJointInfo* const constraintArray = &constraintArrayPtr[m_joints];

		dgInt32 rowsCount = 0;
		dgInt32 isContinueCollisionCluster = 0;
		for (dgInt32 i = 0; i < jointCount; i++) {
			dgJointInfo* const jointInfo = &constraintArray[i];
			dgConstraint* const joint = jointInfo->m_joint;

			dgInt32 m0 = (joint->m_body0->GetInvMass().m_w != dgFloat32(0.0f)) ? joint->m_body0->m_index : 0;
			dgInt32 m1 = (joint->m_body1->GetInvMass().m_w != dgFloat32(0.0f)) ? joint->m_body1->m_index : 0;
			jointInfo->m_m0 = m0;
			jointInfo->m_m1 = m1;
			jointInfo->m_isInQueueFrontier = 0;

			dgBody* const body0 = joint->m_body0;
			dgBody* const body1 = joint->m_body1;
			body0->m_dynamicsLru = m_markLru;
			body1->m_dynamicsLru = m_markLru;

			dgAssert (constraintArray[i].m_pairCount >= 0);
			dgAssert (constraintArray[i].m_pairCount < 64);
			rowsCount += constraintArray[i].m_pairCount;
			if (joint->GetId() == dgConstraint::m_contactConstraint) {
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
					dgFloat32 dist = dgMax(body0->m_collision->GetBoxMinRadius(), body1->m_collision->GetBoxMinRadius()) * dgFloat32(0.25f);

					dgVector relVeloc(veloc1 - veloc0);
					dgVector relOmega(omega1 - omega0);
					dgVector relVelocMag2(relVeloc.DotProduct4(relVeloc));
					dgVector relOmegaMag2(relOmega.DotProduct4(relOmega));

					if ((relOmegaMag2.m_w > dgFloat32(1.0f)) || ((relVelocMag2.m_w * timestep * timestep) > (dist * dist))) {
						dgTriplex normals[16];
						dgTriplex points[16];
						dgInt64 attrib0[16];
						dgInt64 attrib1[16];
						dgFloat32 penetrations[16];
						dgFloat32 timeToImpact = timestep;
						const dgInt32 ccdContactCount = world->CollideContinue(collision0, body0->m_matrix, veloc0, omega0, collision1, body1->m_matrix, veloc1, omega1,
																			   timeToImpact, points, normals, penetrations, attrib0, attrib1, 6, 0);

						for (dgInt32 j = 0; j < ccdContactCount; j++) {
							dgVector point(&points[j].m_x);
							dgVector normal(&normals[j].m_x);
							dgVector vel0(veloc0 + omega0.CrossProduct3(point - com0));
							dgVector vel1(veloc1 + omega1.CrossProduct3(point - com1));
							dgVector vRel(vel1 - vel0);
							dgFloat32 contactDistTravel = vRel.DotProduct4(normal).m_w * timestep;
							ccdJoint |= (contactDistTravel > dist);
						}
					}
					//ccdJoint = body0->m_continueCollisionMode | body1->m_continueCollisionMode;
					isContinueCollisionCluster |= ccdJoint;
					rowsCount += DG_CCD_EXTRA_CONTACT_COUNT;
				}
			}
		}

		if (isContinueCollisionCluster) {
			rowsCount = dgMax(rowsCount, 64);
		}
		cluster.m_rowsCount = rowsCount;
		cluster.m_isContinueCollision = dgInt16 (isContinueCollisionCluster);

		m_clusters++;
		m_bodies += bodyCount;
		m_joints += jointCount;
	}
}

void dgWorldDynamicUpdate::UpdateSkeletons()
{
	dgWorld* const world = (dgWorld*) this;
	dgSkeletonList& skelManager = *world;
	if (skelManager.m_skelListIsDirty) {
		dTimeTrackerEvent(__FUNCTION__);
		skelManager.m_skelListIsDirty = false;
		dgSkeletonList::Iterator iter(skelManager);
		for (iter.Begin(); iter; iter++) {
			dgSkeletonContainer* const skeleton = iter.GetNode()->GetInfo();
			delete skeleton;
		}
		skelManager.RemoveAll();

		world->m_dynamicsLru = world->m_dynamicsLru + 1;
		dgUnsigned32 lru = world->m_dynamicsLru;

		dgBodyMasterList& masterList = *world;
		world->m_solverJacobiansMemory.ResizeIfNecessary((2 * (masterList.m_constraintCount + 1024)) * sizeof (dgBilateralConstraint*));
		dgBilateralConstraint** const jointList = (dgBilateralConstraint**)&world->m_solverJacobiansMemory[0];

		dgInt32 jointCount = 0;
		for (dgBodyMasterList::dgListNode* node = masterList.GetFirst(); node; node = node->GetNext()) {
			const dgBodyMasterListRow& graphNode = node->GetInfo();
			dgBody* const srcBody = graphNode.GetBody();

			for (dgBodyMasterListRow::dgListNode* jointNode = srcBody->m_masterNode->GetInfo().GetLast(); jointNode; jointNode = jointNode->GetPrev()) {
				dgBodyMasterListCell* const cell = &jointNode->GetInfo();
				dgConstraint* const constraint = cell->m_joint;
				dgAssert(constraint);
				dgAssert((constraint->m_body0 == srcBody) || (constraint->m_body1 == srcBody));
				dgAssert((constraint->m_body0 == cell->m_bodyNode) || (constraint->m_body1 == cell->m_bodyNode));
				if (constraint->IsBilateral() && constraint->m_canBeSkeleton && (constraint->m_dynamicsLru != lru)) {
					constraint->m_dynamicsLru = lru;
					jointList[jointCount] = (dgBilateralConstraint*)constraint;
					jointCount++;
				}
			}
		}

		dgSortIndirect(jointList, jointCount, CompareJointByInvMass);

		const dgInt32 poolSize = 1024 * 4;
		dgSkeletonContainer::dgGraph* pool [poolSize];
		dgBilateralConstraint* loopJoints[16];
		world->m_dynamicsLru = world->m_dynamicsLru + 1;
		lru = world->m_dynamicsLru;
		for (dgInt32 i = 0; i < jointCount; i++) {
			dgBilateralConstraint* const constraint = jointList[i];
			if (constraint->m_dynamicsLru != lru) {
				dgInt32 loopCount = 0;
				dgDynamicBody* const rootBody = (dgDynamicBody*) ((constraint->GetBody0()->GetInvMass().m_w < constraint->GetBody1()->GetInvMass().m_w) ? constraint->GetBody0() : constraint->GetBody1());
				dgSkeletonContainer* const skeleton = world->CreateNewtonSkeletonContainer (rootBody);
				dgQueue<dgSkeletonContainer::dgGraph*> queue(pool, poolSize); 
				queue.Insert (skeleton->GetRoot());
				rootBody->m_dynamicsLru = lru;

				while (!queue.IsEmpty()) {
					dgInt32 count = queue.m_firstIndex - queue.m_lastIndex;
					if (count < 0) {
						count += queue.m_mod;
					}

					dgInt32 index = queue.m_lastIndex;
					queue.Reset();

					for (dgInt32 j = 0; j < count; j++) {
						dgSkeletonContainer::dgGraph* const parentNode (queue.m_pool[index]);
						dgDynamicBody* const parentBody = skeleton->GetBody(parentNode);

						for (dgBodyMasterListRow::dgListNode* jointNode = parentBody->m_masterNode->GetInfo().GetFirst(); jointNode; jointNode = jointNode->GetNext()) {
							dgBodyMasterListCell* const cell = &jointNode->GetInfo();
							dgConstraint* const constraint = cell->m_joint;
							if (constraint->IsBilateral() && (constraint->m_dynamicsLru != lru)) {
								constraint->m_dynamicsLru = lru;

								if (constraint->m_canBeSkeleton) {
									dgDynamicBody* const childBody = (dgDynamicBody*) ((constraint->GetBody0() == parentBody) ? constraint->GetBody1() : constraint->GetBody0());

									if ((childBody->m_dynamicsLru != lru) && (childBody->GetInvMass().m_w != dgFloat32 (0.0f))) {
										childBody->m_dynamicsLru = lru;
										dgSkeletonContainer::dgGraph* const childNode = skeleton->AddChild((dgBilateralConstraint*)constraint, parentNode);
										queue.Insert (childNode);
										
									} else if (loopCount < (sizeof (loopJoints)/sizeof(loopJoints[0]))) {
										loopJoints[loopCount] = (dgBilateralConstraint*)constraint;
										loopCount ++;
									}

								} else if (loopCount < (sizeof (loopJoints)/sizeof(loopJoints))) {

									loopJoints[loopCount] = (dgBilateralConstraint*)constraint;
									loopCount ++;
									dgAssert(loopCount < (sizeof (loopJoints) / sizeof (loopJoints[0])));
								}
							}
						}
						index++;
						if (index >= queue.m_mod) {
							index = 0;
						}
					}
				}
				skeleton->Finalize();

				if (loopCount < (sizeof (loopJoints)/sizeof(loopJoints[0]))) {
					for (dgInt32 i = 0; i < loopCount; i ++) {
						skeleton->AttachCyclingJoint(loopJoints[i]); 
					}
				}
			}
		}
	}
}


dgInt32 dgWorldDynamicUpdate::SortClusters(const dgBodyCluster* const cluster, dgFloat32 timestep, dgInt32 threadID) const
{
	dgWorld* const world = (dgWorld*) this;
	dgJointInfo* const constraintArrayPtr = (dgJointInfo*)&world->m_jointsMemory[0];
	dgJointInfo* const constraintArray = &constraintArrayPtr[cluster->m_jointStart];

	dgJointInfo** queueBuffer = dgAlloca(dgJointInfo*, cluster->m_jointCount * 2 + 1024);
	dgJointInfo* const tmpInfoList = dgAlloca(dgJointInfo, cluster->m_jointCount);
	dgQueue<dgJointInfo*> queue(queueBuffer, cluster->m_jointCount * 2 + 1024);
	dgFloat32 heaviestMass = dgFloat32(1.0e20f);
	dgJointInfo* heaviestBody = NULL;

//	for (dgInt32 i = 0; i < island->m_jointCount; i++) {
	for (dgInt32 i = 0; i < cluster->m_activeJointCount; i++) {
		dgJointInfo& jointInfo = constraintArray[i];
		tmpInfoList[i] = jointInfo;
		if (jointInfo.m_isFrontier) {
			queue.Insert(&tmpInfoList[i]);
			tmpInfoList[i].m_isInQueueFrontier = -1;
		} else {
			if (jointInfo.m_joint->m_body0->GetInvMass().m_w && (heaviestMass > jointInfo.m_joint->m_body0->GetInvMass().m_w)) {
				heaviestMass = jointInfo.m_joint->m_body0->GetInvMass().m_w;
				heaviestBody = &tmpInfoList[i];
			}
			if (jointInfo.m_joint->m_body1->GetInvMass().m_w && (heaviestMass > jointInfo.m_joint->m_body1->GetInvMass().m_w)) {
				heaviestMass = jointInfo.m_joint->m_body1->GetInvMass().m_w;
				heaviestBody = &tmpInfoList[i];
			}
		}
	}

	if (queue.IsEmpty()) {
		dgAssert(heaviestBody);
		queue.Insert(heaviestBody);
		heaviestBody->m_isInQueueFrontier = -1;
	}

	dgInt32 infoIndex = 0;
	const dgInt32 lru = cluster->m_clusterLRU;
	dgBodyInfo* const bodyArrayPtr = (dgBodyInfo*)&world->m_bodiesMemory[0];
	dgBodyInfo* const bodyArray = &bodyArrayPtr[cluster->m_bodyStart];

	bool hasSkel = false;
	while (!queue.IsEmpty()) {
		dgInt32 count = queue.m_firstIndex - queue.m_lastIndex;
		if (count < 0) {
			count += queue.m_mod;
		}

		dgInt32 index = queue.m_lastIndex;
		queue.Reset();

		for (dgInt32 j = 0; j < count; j++) {
			dgJointInfo* const jointInfo = queue.m_pool[index];
			dgConstraint* const constraint = jointInfo->m_joint;
			if (constraint->m_clusterLRU == lru) {
				dgAssert (dgInt32 (constraint->m_index) < cluster->m_activeJointCount);
				constraint->m_index = infoIndex;
				constraintArray[infoIndex] = *jointInfo;
				constraint->m_clusterLRU--;
				infoIndex++;
				dgAssert(infoIndex <= cluster->m_jointCount);

				const dgInt32 m0 = jointInfo->m_m0;
				const dgBody* const body0 = bodyArray[m0].m_body;
				hasSkel = hasSkel || body0->GetSkeleton();

				const dgInt32 m1 = jointInfo->m_m1;
				const dgBody* const body1 = bodyArray[m1].m_body;
				hasSkel = hasSkel || body1->GetSkeleton();

				//if (infoIndex == island->m_jointCount) {
				if (infoIndex == cluster->m_activeJointCount) {
					queue.Reset();
					break;
				}
				
				if (body0->GetInvMass().m_w > dgFloat32(0.0f)) {
					for (dgBodyMasterListRow::dgListNode* jointNode = body0->m_masterNode->GetInfo().GetFirst(); jointNode; jointNode = jointNode->GetNext()) {
						dgBodyMasterListCell* const cell = &jointNode->GetInfo();
						dgConstraint* const constraint = cell->m_joint;
						if ((constraint->m_clusterLRU == lru) && (dgInt32 (constraint->m_index) < cluster->m_activeJointCount)){
							dgJointInfo* const nextInfo = &tmpInfoList[constraint->m_index];
							//dgAssert (!nextInfo->m_isInQueueFrontier);
							if (!nextInfo->m_isInQueueFrontier) {
								queue.Insert(nextInfo);
								nextInfo->m_isInQueueFrontier = -1;
							}
						}
					}
				}

				if (body1->GetInvMass().m_w > dgFloat32(0.0f)) {
					for (dgBodyMasterListRow::dgListNode* jointNode = body1->m_masterNode->GetInfo().GetFirst(); jointNode; jointNode = jointNode->GetNext()) {
						dgBodyMasterListCell* const cell = &jointNode->GetInfo();
						dgConstraint* const constraint = cell->m_joint;
						if ((constraint->m_clusterLRU == lru) && (dgInt32 (constraint->m_index) < cluster->m_activeJointCount)){
							dgJointInfo* const nextInfo = &tmpInfoList[constraint->m_index];
//							dgAssert (!nextInfo->m_isInQueueFrontier);
							if (!nextInfo->m_isInQueueFrontier) {
								queue.Insert(nextInfo);
								nextInfo->m_isInQueueFrontier = -1;
							}
						}
					}
				}
			}
			index++;
			if (index >= queue.m_mod) {
				index = 0;
			}
		}
	}

	if (hasSkel) {
		dgInt32 j = 0;
		for (dgInt32 i = 0; (i < cluster->m_activeJointCount) && (j < cluster->m_activeJointCount); i ++) {
			if (constraintArray[i].m_isSkeleton) {
				for (j = dgMax (j, i + 1); j < cluster->m_activeJointCount; j ++) {
					if (!constraintArray[j].m_isSkeleton) {
						dgConstraint* const constraint0 = constraintArray[i].m_joint;
						dgConstraint* const constraint1 = constraintArray[j].m_joint;
						dgSwap(constraint0->m_index, constraint1->m_index); 
						dgSwap(constraintArray[j], constraintArray[i]);
						j ++;
						break;
					}
				}
			}
		}
	}
	dgAssert(infoIndex == cluster->m_activeJointCount);

	return 0;
}


dgBody* dgWorldDynamicUpdate::GetClusterBody(const void* const clusterPtr, dgInt32 index) const
{
	const dgClusterCallbackStruct* const cluster = (dgClusterCallbackStruct*)clusterPtr;

	char* const ptr = &((char*)cluster->m_bodyArray)[cluster->m_strideInByte * index];
	dgBody** const bodyPtr = (dgBody**)ptr;
	return (index < cluster->m_count) ? ((index >= 0) ? *bodyPtr : NULL) : NULL;
}


// sort from high to low
dgInt32 dgWorldDynamicUpdate::CompareClusters(const dgBodyCluster* const clusterA, const dgBodyCluster* const clusterB, void* notUsed)
{
	dgInt32 countA = clusterA->m_activeJointCount + (clusterA->m_hasSoftBodies << 30);
	dgInt32 countB = clusterB->m_activeJointCount + (clusterB->m_hasSoftBodies << 30);

	if (countA < countB) {
		return 1;
	}
	if (countA > countB) {
		return -1;
	}
	return 0;
}


dgInt32 dgWorldDynamicUpdate::CompareJointByInvMass (const dgBilateralConstraint* const jointA, const dgBilateralConstraint* const jointB, void* notUsed)
{
	dgFloat32 invMass0 = dgMin (jointA->GetBody0()->m_invMass.m_w, jointA->GetBody1()->m_invMass.m_w);
	dgFloat32 invMass1 = dgMin (jointB->GetBody0()->m_invMass.m_w, jointB->GetBody1()->m_invMass.m_w);
	if (invMass0 < invMass1) {
		return -1;
	} else if (invMass0 > invMass1) {
		return 1;
	}
	return 0;
}


void dgWorldDynamicUpdate::CalculateClusterReactionForcesKernel (void* const context, void* const worldContext, dgInt32 threadID)
{
	dTimeTrackerEvent(__FUNCTION__);
	dgWorldDynamicUpdateSyncDescriptor* const descriptor = (dgWorldDynamicUpdateSyncDescriptor*) context;

	dgFloat32 timestep = descriptor->m_timestep;
	dgWorld* const world = (dgWorld*) worldContext;
	dgInt32 count = descriptor->m_clusterCount;
	dgBodyCluster* const clusters = &((dgBodyCluster*)&world->m_clusterMemory[0])[descriptor->m_firstCluster];

	for (dgInt32 i = dgAtomicExchangeAndAdd(&descriptor->m_atomicCounter, 1); i < count; i = dgAtomicExchangeAndAdd(&descriptor->m_atomicCounter, 1)) {
		dgBodyCluster* const cluster = &clusters[i]; 
		world->ResolveClusterForces (cluster, threadID, timestep);
	}
}

dgInt32 dgWorldDynamicUpdate::GetJacobianDerivatives (dgContraintDescritor& constraintParamOut, dgJointInfo* const jointInfo, dgConstraint* const constraint, dgJacobianMatrixElement* const matrixRow, dgInt32 rowCount) const
{
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

	dgDynamicBody* const body0 = (dgDynamicBody*)constraint->m_body0;
	dgDynamicBody* const body1 = (dgDynamicBody*)constraint->m_body1;
	dgAssert(body0->IsRTTIType(dgBody::m_dynamicBodyRTTI));
	dgAssert(body1->IsRTTIType(dgBody::m_dynamicBodyRTTI));

	body0->m_inCallback = true;
	body1->m_inCallback = true;
	dof = dgInt32(constraint->JacobianDerivative(constraintParamOut));
	body0->m_inCallback = false;
	body1->m_inCallback = false;
	
	jointInfo->m_scale0 = dgFloat32(1.0f);
	jointInfo->m_scale1 = dgFloat32(1.0f);
	const dgFloat32 invMass0 = body0->GetInvMass().m_w;
	const dgFloat32 invMass1 = body1->GetInvMass().m_w;
	bool isScaled = false;
	if ((invMass0 > dgFloat32(0.0f)) && (invMass1 > dgFloat32(0.0f)) && !body0->GetSkeleton() && !body1->GetSkeleton()) {
		const dgFloat32 mass0 = body0->GetMass().m_w;
		const dgFloat32 mass1 = body1->GetMass().m_w;
		const dgFloat32 scaleFactor = dgFloat32 (DG_HEAVY_MASS_SCALE_FACTOR);
			
		if (mass0 > scaleFactor * mass1) {
			isScaled = true;
			jointInfo->m_scale0 = invMass1 * mass0 / scaleFactor;
		} else if (mass1 > scaleFactor * mass0) {
			jointInfo->m_scale1 = invMass0 * mass1 / scaleFactor;
			isScaled = true;
		}
	}

	if (!isScaled) {
		if (body0->m_equilibrium) {
			if ((invMass0 > dgFloat32(0.0f)) && !body0->GetSkeleton() && !body1->GetSkeleton()) {
				dgAssert (!body1->m_equilibrium);
				dgVector isMovingMask(body0->m_veloc + body0->m_omega + body0->m_accel + body0->m_alpha);
				dgAssert(dgCheckVector(isMovingMask));
				if ((isMovingMask.TestZero().GetSignMask() & 7) == 7) {
					jointInfo->m_scale0 *= DG_LARGE_STACK_DAMP_FACTOR;
				}
			}
		} else if (body1->m_equilibrium) {
			if ((invMass1 > dgFloat32(0.0f)) && !body0->GetSkeleton() && !body1->GetSkeleton()) {
				dgVector isMovingMask(body1->m_veloc + body1->m_omega + body1->m_accel + body1->m_alpha);
				dgAssert(dgCheckVector(isMovingMask));
				if ((isMovingMask.TestZero().GetSignMask() & 7) == 7) {
					jointInfo->m_scale1 *= DG_LARGE_STACK_DAMP_FACTOR;
				}
			}
		}
	}

	jointInfo->m_pairCount = dof;
	jointInfo->m_pairStart = rowCount;
	for (dgInt32 i = 0; i < dof; i++) {
		dgJacobianMatrixElement* const row = &matrixRow[rowCount];
		dgAssert(constraintParamOut.m_forceBounds[i].m_jointForce);
		row->m_Jt = constraintParamOut.m_jacobian[i];

		row->m_diagDamp = dgFloat32 (0.0f);
		row->m_stiffness = DG_PSD_DAMP_TOL * (dgFloat32 (1.0f) - constraintParamOut.m_jointStiffness[i]) + dgFloat32 (1.0e-6f);
		dgAssert(row->m_stiffness >= dgFloat32(0.0f));
		dgAssert(constraintParamOut.m_jointStiffness[i] <= dgFloat32(1.0f));
		dgAssert ((dgFloat32 (1.0f) - constraintParamOut.m_jointStiffness[i]) >= dgFloat32 (0.0f));
		row->m_coordenateAccel = constraintParamOut.m_jointAccel[i];
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

	return rowCount;
}


void dgWorldDynamicUpdate::IntegrateVelocity(const dgBodyCluster* const cluster, dgFloat32 accelTolerance, dgFloat32 timestep, dgInt32 threadID) const
{
	bool isAutoSleep = true;
	bool stackSleeping = true;
	dgInt32 sleepCounter = 10000;

	dgWorld* const world = (dgWorld*) this;

	dgFloat32 velocityDragCoeff = DG_FREEZZING_VELOCITY_DRAG;
	dgBodyInfo* const bodyArrayPtr = (dgBodyInfo*) &world->m_bodiesMemory[0]; 
	dgBodyInfo* const bodyArray = &bodyArrayPtr[cluster->m_bodyStart + 1]; 
	dgInt32 count = cluster->m_bodyCount - 1;
	if (count <= 2) {
		bool autosleep = bodyArray[0].m_body->m_autoSleep;
		if (count == 2) {
			autosleep &= bodyArray[1].m_body->m_autoSleep;
		}
		if (!autosleep) {
			velocityDragCoeff *= dgFloat32 (0.99f);
		}
	}

	dgFloat32 maxAccel = dgFloat32 (0.0f);
	dgFloat32 maxAlpha = dgFloat32 (0.0f);
	dgFloat32 maxSpeed = dgFloat32 (0.0f);
	dgFloat32 maxOmega = dgFloat32 (0.0f);

	const dgFloat32 smallClusterCutoff = (cluster->m_jointCount ? dgFloat32(0.01f) : dgFloat32(1.0f));
	const dgFloat32 speedFreeze = world->m_freezeSpeed2 * smallClusterCutoff;
	const dgFloat32 accelFreeze = world->m_freezeAccel2 * smallClusterCutoff;
	dgVector velocDragVect (velocityDragCoeff, velocityDragCoeff, velocityDragCoeff, dgFloat32 (0.0f));
	for (dgInt32 i = 0; i < count; i ++) {
		dgDynamicBody* const body = (dgDynamicBody*) bodyArray[i].m_body;
		dgAssert(body->IsRTTIType(dgBody::m_dynamicBodyRTTI));

		dgVector isMovingMask (body->m_veloc + body->m_omega + body->m_accel + body->m_alpha);
		dgAssert (dgCheckVector(isMovingMask));
		if (!body->m_equilibrium || ((isMovingMask.TestZero().GetSignMask() & 7) != 7)) {
			dgAssert (body->m_invMass.m_w);
			body->IntegrateVelocity(timestep);

			dgAssert (body->m_accel.m_w == dgFloat32 (0.0f));
			dgAssert (body->m_alpha.m_w == dgFloat32 (0.0f));
			dgAssert (body->m_veloc.m_w == dgFloat32 (0.0f));
			dgAssert (body->m_omega.m_w == dgFloat32 (0.0f));
			dgFloat32 accel2 = body->m_accel.DotProduct4(body->m_accel).GetScalar();
			dgFloat32 alpha2 = body->m_alpha.DotProduct4(body->m_alpha).GetScalar();
			dgFloat32 speed2 = body->m_veloc.DotProduct4(body->m_veloc).GetScalar();
			dgFloat32 omega2 = body->m_omega.DotProduct4(body->m_omega).GetScalar();

			maxAccel = dgMax (maxAccel, accel2);
			maxAlpha = dgMax (maxAlpha, alpha2);
			maxSpeed = dgMax (maxSpeed, speed2);
			maxOmega = dgMax (maxOmega, omega2);

			bool equilibrium = (accel2 < accelFreeze) && (alpha2 < accelFreeze) && (speed2 < speedFreeze) && (omega2 < speedFreeze);

			if (equilibrium) {
				dgVector veloc (body->m_veloc.CompProduct4(velocDragVect));
				dgVector omega = body->m_omega.CompProduct4 (velocDragVect);
				body->m_veloc = (dgVector (veloc.CompProduct4(veloc)) > m_velocTol) & veloc;
				body->m_omega = (dgVector (omega.CompProduct4(omega)) > m_velocTol) & omega;
			}

			body->m_equilibrium = dgUnsigned32 (equilibrium);
//body->m_equilibrium &= body->m_autoSleep;

			stackSleeping &= equilibrium;
			isAutoSleep &= body->m_autoSleep;

			sleepCounter = dgMin (sleepCounter, body->m_sleepingCounter);

			body->UpdateMatrix (timestep, threadID);
		}
	}


	if (isAutoSleep && cluster->m_jointCount) {
		if (stackSleeping) {
			for (dgInt32 i = 0; i < count; i ++) {
				dgDynamicBody* const body = (dgDynamicBody*) bodyArray[i].m_body;
				dgAssert(body->IsRTTIType(dgBody::m_dynamicBodyRTTI));
				body->m_accel = dgVector::m_zero;
				body->m_alpha = dgVector::m_zero;
				body->m_veloc = dgVector::m_zero;
				body->m_omega = dgVector::m_zero;
			}
		} else {
			const bool state = (maxAccel > world->m_sleepTable[DG_SLEEP_ENTRIES - 1].m_maxAccel) || (maxAlpha > world->m_sleepTable[DG_SLEEP_ENTRIES - 1].m_maxAlpha) ||
							   (maxSpeed > world->m_sleepTable[DG_SLEEP_ENTRIES - 1].m_maxVeloc) || (maxOmega > world->m_sleepTable[DG_SLEEP_ENTRIES - 1].m_maxOmega);
			if (state) { 
					for (dgInt32 i = 0; i < count; i ++) {
						dgDynamicBody* const body = (dgDynamicBody*) bodyArray[i].m_body;
						dgAssert(body->IsRTTIType(dgBody::m_dynamicBodyRTTI));
						body->m_sleepingCounter = 0;
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
						dgDynamicBody* const body = (dgDynamicBody*) bodyArray[i].m_body;
						dgAssert(body->IsRTTIType(dgBody::m_dynamicBodyRTTI));
						body->m_accel = dgVector::m_zero;
						body->m_alpha = dgVector::m_zero;
						body->m_veloc = dgVector::m_zero;
						body->m_omega = dgVector::m_zero;
						body->m_equilibrium = true;
					}
				} else {
					sleepCounter ++;
					for (dgInt32 i = 0; i < count; i ++) {
						dgDynamicBody* const body = (dgDynamicBody*) bodyArray[i].m_body;
						dgAssert(body->IsRTTIType(dgBody::m_dynamicBodyRTTI));
						body->m_sleepingCounter = sleepCounter;
					}
				}
			}
		}
	}
}

void dgJacobianMemory::Init(dgWorld* const world, dgInt32 rowsCount, dgInt32 bodyCount, dgInt32 blockMatrixSizeInBytes)
{
	world->m_solverJacobiansMemory.ResizeIfNecessary ((rowsCount + 1) * sizeof (dgJacobianMatrixElement));
	m_jacobianBuffer = (dgJacobianMatrixElement*)&world->m_solverJacobiansMemory[0];

	world->m_solverForceAccumulatorMemory.ResizeIfNecessary ((bodyCount + 8) * sizeof (dgJacobian));
	m_internalForcesBuffer = (dgJacobian*)&world->m_solverForceAccumulatorMemory[0];
	dgAssert(bodyCount <= (((world->m_solverForceAccumulatorMemory.GetBytesCapacity() - 16) / dgInt32(sizeof (dgJacobian))) & (-8)));

	dgAssert((dgUnsigned64(m_jacobianBuffer) & 0x01f) == 0);
	dgAssert((dgUnsigned64(m_internalForcesBuffer) & 0x01f) == 0);
}


