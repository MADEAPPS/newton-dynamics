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
#include "dgBilateralConstraint.h"
#include "dgWorldDynamicUpdate.h"
#include "dgCollisionDeformableMesh.h"


#define DG_PARALLEL_JOINT_COUNT_CUT_OFF		(4 * DG_MAX_THREADS_HIVE_COUNT)
#define DG_CCD_EXTRA_CONTACT_COUNT			(4 * 4)


dgVector dgWorldDynamicUpdate::m_velocTol (dgFloat32 (1.0e-18f));

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
	dgThread::dgCriticalSection* m_lock;
	dgCollisionDeformableMeshList::dgListNode* m_sofBodyNode;
};


//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

dgBody* dgWorld::GetIslandBody (const void* const islandPtr, dgInt32 index) const
{
	const dgIslandCallbackStruct* const island = (dgIslandCallbackStruct*)islandPtr;

	char* const ptr = &((char*)island->m_bodyArray)[island->m_strideInByte * index];
	dgBody** const bodyPtr = (dgBody**)ptr;
	return (index < island->m_count) ? ((index >= 0) ? *bodyPtr : NULL) : NULL;
}


dgWorldDynamicUpdate::dgWorldDynamicUpdate()
	:m_bodies(0)
	,m_joints(0)
	,m_islands(0)
	,m_markLru(0)
	,m_islandColor(0)
	,m_equilibriumMark(0)
	,m_rowCountAtomicIndex(0)
	,m_softBodyCriticalSectionLock()
{
}


void dgWorldDynamicUpdate::UpdateDynamics(dgFloat32 timestep)
{
	dgWorld* const world = (dgWorld*) this;
	dgUnsigned32 updateTime = world->m_getPerformanceCount();

	m_bodies = 0;
	m_joints = 0;
	m_islands = 0;
	m_markLru = 0;

	world->m_dynamicsLru = world->m_dynamicsLru + DG_BODY_LRU_STEP;
	m_markLru = world->m_dynamicsLru;
	dgUnsigned32 lru = m_markLru - 1;

	dgBodyMasterList& me = *world;

	world->m_pairMemoryBuffer.ExpandCapacityIfNeessesary (4 * me.GetCount(), sizeof (dgBody*));

	dgAssert (me.GetFirst()->GetInfo().GetBody() == world->m_sentionelBody);

	world->m_sentionelBody->m_index = 0; 
	world->m_sentionelBody->m_dynamicsLru = m_markLru;

	for (dgBodyMasterList::dgListNode* node = me.GetLast(); node; node = node->GetPrev()) {
		const dgBodyMasterListRow& graphNode = node->GetInfo();
		dgBody* const body = graphNode.GetBody();	

		//if ((body->GetType() == dgBody::m_kinamticBody) || (body->GetInvMass().m_w == dgFloat32(0.0f))) {
		if (body->GetInvMass().m_w == dgFloat32(0.0f)) {
#ifdef _DEBUG
			for (; node; node = node->GetPrev()) {
				//dgAssert ((body->GetType() == dgBody::m_kinamticBody) ||(node->GetInfo().GetBody()->GetInvMass().m_w == dgFloat32(0.0f)));
				dgAssert (node->GetInfo().GetBody()->GetInvMass().m_w == dgFloat32(0.0f));
			}
#endif
			break;
		}

		if (body->IsRTTIType(dgBody::m_dynamicBodyRTTI)) {
			dgDynamicBody* const dynamicBody = (dgDynamicBody*) body;
			if (dynamicBody->m_dynamicsLru < lru) {
				if (!(dynamicBody->m_freeze | dynamicBody->m_spawnnedFromCallback | dynamicBody->m_sleeping)) {
					SpanningTree (dynamicBody);
				}
			}
			dynamicBody->m_spawnnedFromCallback = false;
		}
	}

	dgIsland* const islands = (dgIsland*) &world->m_islandMemory[0];
	dgSort (islands, m_islands, CompareIslands); 

	dgInt32 maxRowCount = 0;
	for (dgInt32 i = 0; i < m_islands; i ++) {
		maxRowCount += islands[i].m_rowsCount;
		islands[i].m_rowsCountBaseBlock = -1;
	}
	m_solverMemory.Init (world, maxRowCount, m_bodies);

	dgUnsigned32 dynamicsTime = world->m_getPerformanceCount();
	world->m_perfomanceCounters[m_dynamicsBuildSpanningTreeTicks] = dynamicsTime - updateTime;

	m_rowCountAtomicIndex = 0;

	dgInt32 threadCount = world->GetThreadCount();	
	dgWorldDynamicUpdateSyncDescriptor descriptor;
	descriptor.m_timestep = timestep;

	dgInt32 index = 0;
	if (world->m_useParallelSolver && (threadCount > 1)) {
		for ( ; (index < m_islands) && (islands[index].m_jointCount >= DG_PARALLEL_JOINT_COUNT_CUT_OFF); index ++) {
			int i = index + 1;
			if ((i < m_islands) && (islands[index].m_jointCount < (2 * islands[i].m_jointCount))) { 
				break;
			}
			CalculateReactionForcesParallel (&islands[index], timestep);
		}
	}

	descriptor.m_firstIsland = index;
	descriptor.m_islandCount = m_islands - index;
	for (dgInt32 i = 0; i < threadCount; i ++) {
		world->QueueJob (CalculateIslandReactionForcesKernel, &descriptor, world);
	}
	world->SynchronizationBarrier();


	dgUnsigned32 ticks = world->m_getPerformanceCount();
	world->m_perfomanceCounters[m_dynamicsSolveSpanningTreeTicks] = ticks - dynamicsTime;
	world->m_perfomanceCounters[m_dynamicsTicks] = ticks - updateTime;

	// integrate soft body dynamics phase 2
	const dgCollisionDeformableMeshList* const softBodyList = world;
	descriptor.m_sofBodyNode = softBodyList->GetFirst();
	descriptor.m_lock = &m_softBodyCriticalSectionLock;
	for (dgInt32 i = 0; i < threadCount; i ++) {
		world->QueueJob (IntegrateSoftBodyForcesKernel, &descriptor, world);
	}
	world->SynchronizationBarrier();
	world->m_perfomanceCounters[m_softBodyTicks] += (world->m_getPerformanceCount() - ticks);
}



void dgJacobianMemory::Init (dgWorld* const world, dgInt32 rowsCount, dgInt32 bodyCount)
{
	world->m_solverMatrixMemory.ExpandCapacityIfNeessesary (rowsCount, sizeof (dgJacobianMatrixElement));
	m_memory = (dgJacobianMatrixElement*) &world->m_solverMatrixMemory[0];


	dgInt32 stride = CalculateIntenalMemorySize();
	if (world->m_internalForcesMemory.ExpandCapacityIfNeessesary (bodyCount + 8, stride)) {
		//dgInt32 newCount = ((world->m_internalForcesMemory.GetBytesCapacity() - 16)/ stride) & (-8);
		dgAssert (bodyCount <= (((world->m_internalForcesMemory.GetBytesCapacity() - 16)/ stride) & (-8)));
		dgInt8* const memory = (dgInt8*) &world->m_internalForcesMemory[0];

		m_internalForces = (dgJacobian*) memory;
		//m_internalVeloc = (dgJacobian*) &m_internalForces[newCount];

		dgAssert ((dgUnsigned64(m_internalForces) & 0x01f) == 0);
		//dgAssert ((dgUnsigned64(m_internalVeloc) & 0x01f) == 0);
	}
}

// sort from high to low
dgInt32 dgWorldDynamicUpdate::CompareIslands (const dgIsland* const islandA, const dgIsland* const islandB, void* notUsed)
{
	dgInt32 countA = islandA->m_jointCount + (islandA->m_hasExactSolverJoints << 28);
	dgInt32 countB = islandB->m_jointCount + (islandB->m_hasExactSolverJoints << 28);

	if (countA < countB) {
		return 1;
	}
	if (countA > countB) {
		return -1;
	}
	return 0;
}



void dgWorldDynamicUpdate::SpanningTree (dgDynamicBody* const body)
{
	dgInt32 bodyCount = 0;
	dgInt32 jointCount = 0;
	dgInt32 staticCount = 0;
	dgUnsigned32 lruMark = m_markLru - 1;
	dgInt32 isInEquilibrium = 1;
	dgInt32 hasExactSolverJoints = 0;
	dgInt32 isContinueCollisionIsland = 0;
	dgFloat32 haviestMass = dgFloat32 (0.0f);

	dgDynamicBody* heaviestBody = NULL;
	dgWorld* const world = (dgWorld*) this;
	dgQueue<dgDynamicBody*> queue ((dgDynamicBody**) &world->m_pairMemoryBuffer[0], dgInt32 ((world->m_pairMemoryBuffer.GetBytesCapacity()>>1)/sizeof (void*)));
	
	dgDynamicBody** const staticPool = &queue.m_pool[queue.m_mod];

	body->m_dynamicsLru = lruMark;

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

// hack to test stability
//isInEquilibrium = 0;

			srcBody->m_sleeping = false;
			isContinueCollisionIsland |= srcBody->m_continueCollisionMode;

			if (srcBody->m_mass.m_w > haviestMass) {
				haviestMass = srcBody->m_mass.m_w;
				heaviestBody = srcBody;
			}

			bodyCount ++;

			for (dgBodyMasterListRow::dgListNode* jointNode = srcBody->m_masterNode->GetInfo().GetFirst(); jointNode; jointNode = jointNode->GetNext()) {
				dgBodyMasterListCell* const cell = &jointNode->GetInfo();
				dgConstraint* const constraint = cell->m_joint;

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
							dgInt32 duplicateBody = 0;
							for (; duplicateBody < staticCount; duplicateBody ++) {
								if (staticPool[duplicateBody] == srcBody) {
									break;
								}
							}
							if (duplicateBody == staticCount) {
								staticPool[staticCount] = srcBody;
								staticCount ++;
								dgAssert (srcBody->m_invMass.m_w > dgFloat32 (0.0f));
							}

							
							dgAssert (dgInt32 (constraint->m_dynamicsLru) != m_markLru);

							dgInt32 jointIndex = m_joints + jointCount; 

							world->m_jointsMemory.ExpandCapacityIfNeessesary(jointIndex, sizeof (dgJointInfo));

							hasExactSolverJoints |= constraint->m_useExactSolver;
							
							constraint->m_index = dgUnsigned32 (jointCount);
							dgJointInfo* const constraintArray = (dgJointInfo*) &world->m_jointsMemory[0];
							constraintArray[jointIndex].m_joint = constraint;
							jointCount ++;

							dgAssert (constraint->m_body0);
							dgAssert (constraint->m_body1);
						}

					} else if (body->m_invMass.m_w == dgFloat32 (0.0f)) { 
						dgInt32 duplicateBody = 0;
						for (; duplicateBody < staticCount; duplicateBody ++) {
							if (staticPool[duplicateBody] == srcBody) {
								break;
							}
						}
						if (duplicateBody == staticCount) {
							staticPool[staticCount] = srcBody;
							staticCount ++;
							dgAssert (srcBody->m_invMass.m_w > dgFloat32 (0.0f));
						}
					
						dgAssert (dgInt32 (constraint->m_dynamicsLru) != m_markLru);

						dgInt32 jointIndex = m_joints + jointCount; 
						world->m_jointsMemory.ExpandCapacityIfNeessesary(jointIndex, sizeof (dgJointInfo));

						hasExactSolverJoints |= constraint->m_useExactSolver;
						
						constraint->m_index = dgUnsigned32 (jointCount);
						dgJointInfo* const constraintArray = (dgJointInfo*) &world->m_jointsMemory[0];
						constraintArray[jointIndex].m_joint = constraint;
						jointCount ++;

						dgAssert (constraint->m_body0);
						dgAssert (constraint->m_body1);
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

		dgInt32 rowsCount = 0;
		if (staticCount) {
			queue.Reset ();
			for (dgInt32 i = 0; i < staticCount; i ++) {
				dgDynamicBody* const body = staticPool[i];
				body->m_dynamicsLru = m_markLru;
				queue.Insert (body);
				dgAssert (dgInt32 (body->m_dynamicsLru) == m_markLru);
			}

			if (isContinueCollisionIsland) {
				for (dgInt32 i = 0; i < jointCount; i ++) {
					dgConstraint* const constraint = constraintArray[m_joints + i].m_joint;
					constraint->m_dynamicsLru = m_markLru;
					dgInt32 rows = dgInt32 ((constraint->m_maxDOF & (dgInt32 (sizeof (dgVector) / sizeof (dgFloat32)) - 1)) ? ((constraint->m_maxDOF & (-dgInt32 (sizeof (dgVector) / sizeof (dgFloat32)))) + dgInt32 (sizeof (dgVector) / sizeof (dgFloat32))) : constraint->m_maxDOF);
					rowsCount += rows;
					constraintArray[m_joints + i].m_autoPaircount = rows;
					if (constraint->GetId() == dgConstraint::m_contactConstraint) {
						// add some padding for extra contacts in a continue  
						// 4 extra contacts three rows each
						rowsCount += DG_CCD_EXTRA_CONTACT_COUNT;
					}
				}
			} else {
				for (dgInt32 i = 0; i < jointCount; i ++) {
					dgConstraint* const constraint = constraintArray[m_joints + i].m_joint;
					constraint->m_dynamicsLru = m_markLru;
					dgInt32 rows = dgInt32 ((constraint->m_maxDOF & (dgInt32 (sizeof (dgVector) / sizeof (dgFloat32)) - 1)) ? ((constraint->m_maxDOF & (-dgInt32 (sizeof (dgVector) / sizeof (dgFloat32)))) + dgInt32 (sizeof (dgVector) / sizeof (dgFloat32))) : constraint->m_maxDOF);
					rowsCount += rows;
					constraintArray[m_joints + i].m_autoPaircount = rows;
				}
			}
		} else {
			dgAssert (heaviestBody);
			queue.Insert (heaviestBody);
			heaviestBody->m_dynamicsLru = m_markLru;
		}

		BuildIsland (queue, jointCount, rowsCount, isContinueCollisionIsland, hasExactSolverJoints);
	}
}

void dgWorldDynamicUpdate::BuildIsland (dgQueue<dgDynamicBody*>& queue, dgInt32 jointCount, dgInt32 rowsCount, dgInt32 isContinueCollisionIsland, dgInt32 hasExactSolverJoints)
{
	dgInt32 bodyCount = 1;
	dgUnsigned32 lruMark = m_markLru;

	dgWorld* const world = (dgWorld*) this;
	world->m_bodiesMemory.ExpandCapacityIfNeessesary(m_bodies, sizeof (dgBodyInfo));

	dgBodyInfo* const bodyArray0 = (dgBodyInfo*) &world->m_bodiesMemory[0]; 

	bodyArray0[m_bodies].m_body = world->m_sentionelBody;
	dgAssert (world->m_sentionelBody->m_index == 0); 
	dgAssert (dgInt32 (world->m_sentionelBody->m_dynamicsLru) == m_markLru); 

	m_islandColor ++;
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
				body->m_alived0 = false;
				body->m_alived1 = false;
				body->m_resting = true;
				body->m_islandColor = m_islandColor;
				bodyArray1[bodyIndex].m_body = body;
				bodyCount ++;
			}


			for (dgBodyMasterListRow::dgListNode* jointNode = body->m_masterNode->GetInfo().GetFirst(); jointNode; jointNode = jointNode->GetNext()) {
				dgBodyMasterListCell* const cell = &jointNode->GetInfo();
				dgConstraint* const constraint = cell->m_joint;

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

						hasExactSolverJoints |= constraint->m_useExactSolver;
						
						constraint->m_index = dgUnsigned32 (jointCount);
						dgJointInfo* const constraintArray = (dgJointInfo*) &world->m_jointsMemory[0];
						constraintArray[jointIndex].m_joint = constraint;

						dgInt32 rows = dgInt32 ((constraint->m_maxDOF & (dgInt32 (sizeof (dgVector) / sizeof (dgFloat32)) - 1)) ? ((constraint->m_maxDOF & (-dgInt32 (sizeof (dgVector) / sizeof (dgFloat32)))) + dgInt32 (sizeof (dgVector) / sizeof (dgFloat32))) : constraint->m_maxDOF);
						rowsCount += (rows + (ccdMode ? DG_CCD_EXTRA_CONTACT_COUNT : 0));
 						constraintArray[jointIndex].m_autoPaircount = rows;

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
		dgBodyInfo* const bodyArray = (dgBodyInfo*) &world->m_bodiesMemory[0]; 
		if (isContinueCollisionIsland && jointCount && (rowsCount < 32)) {
			rowsCount = 32;
		}

		world->m_islandMemory.ExpandCapacityIfNeessesary (m_islands, sizeof (dgIsland));
		dgJointInfo* const constraintArray = (dgJointInfo*) &world->m_jointsMemory[0];

		if (jointCount) {
			for (dgInt32 i = 0; i < jointCount; i ++) {
				dgConstraint* const joint = constraintArray[m_joints + i].m_joint;
				constraintArray[m_joints + i].m_color = 0;
				dgBody* const body0 = joint->m_body0;
				dgBody* const body1 = joint->m_body1;
				bool resting = body0->m_equilibrium & body1->m_equilibrium;
				body0->m_resting &= resting;
				body1->m_resting &= resting;

	body0->m_alived1 = true;
	body1->m_alived1 = true;
			}
			bodyArray[m_bodies].m_body->m_islandColor = 0;
			bodyArray[m_bodies].m_body->m_alived0 = false;
			bodyArray[m_bodies].m_body->m_alived1 = false;
			bodyArray[m_bodies].m_body->m_resting = true;
/*
		for (dgInt32 i = 1; i < bodyCount; i ++) {
			dgBody* const body = bodyArray[i].m_body;
			if (!body->m_resting) {
				body->m_alived0 = true;
				for (dgBodyMasterListRow::dgListNode* jointNode = body->m_masterNode->GetInfo().GetFirst(); jointNode; jointNode = jointNode->GetNext()) {
					dgBodyMasterListCell* const cell = &jointNode->GetInfo();
					dgBody* const linkBody = cell->m_bodyNode;

					if (linkBody->m_islandColor == m_islandColor) {
						linkBody->m_alived0 = true;
					}
				}
			}
		}

		for (dgInt32 i = 1; i < bodyCount; i ++) {
			dgBody* const body = bodyArray[i].m_body;
			if (body->m_alived0) {
				body->m_alived1 = true;
				for (dgBodyMasterListRow::dgListNode* jointNode = body->m_masterNode->GetInfo().GetFirst(); jointNode; jointNode = jointNode->GetNext()) {
					dgBodyMasterListCell* const cell = &jointNode->GetInfo();
					dgBody* const linkBody = cell->m_bodyNode;

					if (linkBody->m_islandColor == m_islandColor) {
						linkBody->m_alived1 = true;
					}
				}
			}
		}
*/
		} else {
			bodyArray[m_bodies].m_body->m_islandColor = 0;
			bodyArray[m_bodies].m_body->m_alived0 = false;
			bodyArray[m_bodies].m_body->m_alived1 = false;
			bodyArray[m_bodies].m_body->m_resting = true;
			bodyArray[m_bodies + 1].m_body->m_alived0 = true;
			bodyArray[m_bodies + 1].m_body->m_alived1 = true;
		}
		

		dgIsland* const islandArray = (dgIsland*) &world->m_islandMemory[0];

		islandArray[m_islands].m_bodyStart = m_bodies;
		islandArray[m_islands].m_jointStart = m_joints;
		islandArray[m_islands].m_bodyCount = bodyCount;
		islandArray[m_islands].m_jointCount = jointCount;
		islandArray[m_islands].m_rowsCount = rowsCount;
		islandArray[m_islands].m_rowsCountBaseBlock = -1;

		islandArray[m_islands].m_hasExactSolverJoints = hasExactSolverJoints;
		islandArray[m_islands].m_isContinueCollision = isContinueCollisionIsland;

		if (hasExactSolverJoints) {
			dgInt32 contactsCount = 0;
			for (dgInt32 i = 0; i < jointCount; i ++) {
				dgConstraint* const joint = constraintArray[m_joints + i].m_joint;
				contactsCount += (joint->GetId() == dgConstraint::m_contactConstraint); 
			}

			for (dgInt32 i = 0; i < jointCount; i ++) {
				dgConstraint* const joint = constraintArray[m_joints + i].m_joint;
				if (joint->m_useExactSolver) {
					dgAssert (joint->IsBilateral());
					dgBilateralConstraint* const bilateralJoint = (dgBilateralConstraint*) joint;
					if (bilateralJoint->m_useExactSolver && (bilateralJoint->m_useExactSolverContactLimit < contactsCount)) {
						islandArray[m_islands].m_hasExactSolverJoints = 0;
						break;
					}
				}
			}
		}

		m_islands ++;
		m_bodies += bodyCount;
		m_joints += jointCount;
	}
}


void dgWorldDynamicUpdate::IntegrateSoftBody (dgWorldDynamicUpdateSyncDescriptor* const descriptor, dgInt32 threadID)
{
	dgFloat32 timestep = descriptor->m_timestep; 
	dgWorld* const world = (dgWorld*) this;

	dgCollisionDeformableMeshList::dgListNode* node = NULL;
	{
		dgThreadHiveScopeLock lock (world, descriptor->m_lock);
		node = descriptor->m_sofBodyNode;
		if (node) {
			descriptor->m_sofBodyNode = node->GetNext();
		}
	}

	for ( ; node; ) {
		dgCollisionDeformableMesh* const softShape = node->GetInfo();

		softShape->ResolvePositionsConstraints (timestep);

		dgThreadHiveScopeLock lock (world, descriptor->m_lock);
		node = descriptor->m_sofBodyNode;
		if (node) {
			descriptor->m_sofBodyNode = node->GetNext();
		}
	}
}

void dgWorldDynamicUpdate::IntegrateSoftBodyForcesKernel (void* const context, void* const worldContext, dgInt32 threadID)
{
	dgWorldDynamicUpdateSyncDescriptor* const descriptor = (dgWorldDynamicUpdateSyncDescriptor*) context;
	dgWorld* const world = (dgWorld*) worldContext;
	world->IntegrateSoftBody (descriptor, threadID);
}

void dgWorldDynamicUpdate::CalculateIslandReactionForcesKernel (void* const context, void* const worldContext, dgInt32 threadID)
{
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


//dgInt32 dgWorldDynamicUpdate::GetJacobianDerivatives (const dgIsland* const island, dgInt32 threadIndex, bool bitMode, dgInt32 rowBase, dgInt32 rowCount, dgFloat32 timestep) const
dgInt32 dgWorldDynamicUpdate::GetJacobianDerivatives (const dgIsland* const island, dgInt32 threadIndex, dgInt32 rowBase, dgInt32 rowCount, dgFloat32 timestep) const
{
	dgContraintDescritor constraintParams;
	dgWorld* const world = (dgWorld*) this;

	constraintParams.m_world = world; 
	constraintParams.m_threadIndex = threadIndex;
	constraintParams.m_timestep = timestep;
	constraintParams.m_invTimestep = (timestep > dgFloat32 (1.0e-5f)) ? dgFloat32 (1.0f / timestep) : dgFloat32 (0.0f);

	dgJointInfo* const constraintArrayPtr = (dgJointInfo*) &world->m_jointsMemory[0];
	dgJointInfo* const constraintArray = &constraintArrayPtr[island->m_jointStart];

	dgJacobianMatrixElement* const matrixRow = &m_solverMemory.m_memory[rowBase];
	dgInt32 jointCount = island->m_jointCount;
	for (dgInt32 j = 0; j < jointCount; j ++) {
		dgJointInfo* const jointInfo = &constraintArray[j];
		dgConstraint* const constraint = jointInfo->m_joint;

		dgAssert (constraint->m_body0);
		dgAssert (constraint->m_body1);

		dgBody* const body0 = constraint->m_body0;
		dgBody* const body1 = constraint->m_body1;
		if (body0->m_alived1 | body1->m_alived1) {

			dgInt32 dof = dgInt32 (constraint->m_maxDOF);
			dgAssert (dof <= DG_CONSTRAINT_MAX_ROWS);
			for (dgInt32 i = 0; i < dof; i ++) {
				constraintParams.m_forceBounds[i].m_low = DG_MIN_BOUND;
				constraintParams.m_forceBounds[i].m_upper = DG_MAX_BOUND;
				constraintParams.m_forceBounds[i].m_jointForce = NULL;
				constraintParams.m_forceBounds[i].m_normalIndex = DG_BILATERAL_CONSTRAINT;
			}

			body0->m_inCallback = true;
			body1->m_inCallback = true;

			dof = dgInt32 (constraint->JacobianDerivative (constraintParams)); 

			body0->m_inCallback = false;
			body1->m_inCallback = false;

			dgInt32 m0 = (body0->GetInvMass().m_w != dgFloat32(0.0f)) ? body0->m_index: 0;
			dgInt32 m1 = (body1->GetInvMass().m_w != dgFloat32(0.0f)) ? body1->m_index: 0;

			dgAssert (m0 < island->m_bodyCount);
			dgAssert (m1 < island->m_bodyCount);

			dgAssert (constraint->m_index == dgUnsigned32(j));
			jointInfo->m_autoPairstart = rowCount;
			jointInfo->m_autoPaircount = dof;
			jointInfo->m_autoPairActiveCount = dof;
			jointInfo->m_m0 = m0;
			jointInfo->m_m1 = m1;

			for (dgInt32 i = 0; i < dof; i ++) {

				dgJacobianMatrixElement* const row = &matrixRow[rowCount];
				dgAssert (constraintParams.m_forceBounds[i].m_jointForce);
				row->m_Jt = constraintParams.m_jacobian[i]; 

				dgAssert (constraintParams.m_jointStiffness[i] >= dgFloat32(0.1f));
				dgAssert (constraintParams.m_jointStiffness[i] <= dgFloat32(100.0f));

				row->m_diagDamp = constraintParams.m_jointStiffness[i];
				row->m_coordenateAccel = constraintParams.m_jointAccel[i];
				row->m_accelIsMotor = constraintParams.m_isMotor[i];
				row->m_restitution = constraintParams.m_restitution[i];
				row->m_penetration = constraintParams.m_penetration[i];
				row->m_penetrationStiffness = constraintParams.m_penetrationStiffness[i];
				row->m_lowerBoundFrictionCoefficent = constraintParams.m_forceBounds[i].m_low;
				row->m_upperBoundFrictionCoefficent = constraintParams.m_forceBounds[i].m_upper;
				row->m_jointFeebackForce = constraintParams.m_forceBounds[i].m_jointForce;

				row->m_normalForceIndex = constraintParams.m_forceBounds[i].m_normalIndex; 
				rowCount ++;
			}

			rowCount = (rowCount & (dgInt32 (sizeof (dgVector) / sizeof (dgFloat32)) - 1)) ? ((rowCount & (-dgInt32 (sizeof (dgVector) / sizeof (dgFloat32)))) + dgInt32 (sizeof (dgVector) / sizeof (dgFloat32))) : rowCount;
			dgAssert ((rowCount & (dgInt32 (sizeof (dgVector) / sizeof (dgFloat32)) - 1)) == 0);
		}
	}
	return rowCount;
}


void dgWorldDynamicUpdate::GetJacobianDerivativesParallel (dgJointInfo* const jointInfo, dgInt32 threadIndex, dgInt32 rowBase, dgFloat32 timestep) const
{
	dgWorld* const world = (dgWorld*) this;

	dgContraintDescritor constraintParams;
	constraintParams.m_world = world; 
	constraintParams.m_threadIndex = threadIndex;
	constraintParams.m_timestep = timestep;
	constraintParams.m_invTimestep = dgFloat32 (1.0f / timestep);

	dgJacobianMatrixElement* const matrixRow = &m_solverMemory.m_memory[rowBase];
	dgConstraint* const constraint = jointInfo->m_joint;

	dgInt32 dof = dgInt32 (constraint->m_maxDOF);
	dgAssert (dof <= DG_CONSTRAINT_MAX_ROWS);
	for (dgInt32 i = 0; i < dof; i ++) {
		constraintParams.m_forceBounds[i].m_low = DG_MIN_BOUND;
		constraintParams.m_forceBounds[i].m_upper = DG_MAX_BOUND;
		constraintParams.m_forceBounds[i].m_jointForce = NULL;
		constraintParams.m_forceBounds[i].m_normalIndex = DG_BILATERAL_CONSTRAINT;
	}

	dgAssert (constraint->m_body0);
	dgAssert (constraint->m_body1);

	constraint->m_body0->m_inCallback = true;
	constraint->m_body1->m_inCallback = true;

	dof = dgInt32 (constraint->JacobianDerivative (constraintParams)); 

	constraint->m_body0->m_inCallback = false;
	constraint->m_body1->m_inCallback = false;

	dgInt32 m0 = (constraint->m_body0->GetInvMass().m_w != dgFloat32(0.0f)) ? constraint->m_body0->m_index: 0;
	dgInt32 m1 = (constraint->m_body1->GetInvMass().m_w != dgFloat32(0.0f)) ? constraint->m_body1->m_index: 0;

	jointInfo->m_autoPairstart = rowBase;
	jointInfo->m_autoPaircount = dof;
	jointInfo->m_autoPairActiveCount = dof;
	jointInfo->m_m0 = m0;
	jointInfo->m_m1 = m1;

	for (dgInt32 i = 0; i < dof; i ++) {
		dgJacobianMatrixElement* const row = &matrixRow[i];
		dgAssert (constraintParams.m_forceBounds[i].m_jointForce);
		row->m_Jt = constraintParams.m_jacobian[i]; 

		dgAssert (constraintParams.m_jointStiffness[i] >= dgFloat32(0.1f));
		dgAssert (constraintParams.m_jointStiffness[i] <= dgFloat32(100.0f));

		row->m_diagDamp = constraintParams.m_jointStiffness[i];
		row->m_coordenateAccel = constraintParams.m_jointAccel[i];
		row->m_accelIsMotor = constraintParams.m_isMotor[i];
		row->m_restitution = constraintParams.m_restitution[i];
		row->m_penetration = constraintParams.m_penetration[i];
		row->m_penetrationStiffness = constraintParams.m_penetrationStiffness[i];
		row->m_lowerBoundFrictionCoefficent = constraintParams.m_forceBounds[i].m_low;
		row->m_upperBoundFrictionCoefficent = constraintParams.m_forceBounds[i].m_upper;
		row->m_jointFeebackForce = constraintParams.m_forceBounds[i].m_jointForce;
		row->m_normalForceIndex = constraintParams.m_forceBounds[i].m_normalIndex; 
	}
	if (dof & 1) {
		matrixRow[dof] = matrixRow[0];
	}
}


void dgWorldDynamicUpdate::IntegrateArray (const dgIsland* const island, dgFloat32 accelTolerance, dgFloat32 timestep, dgInt32 threadIndex) const
{
	bool isAutoSleep = true;
	bool stackSleeping = true;
	dgInt32 sleepCounter = 10000;

	dgWorld* const world = (dgWorld*) this;


	dgVector zero (dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f));

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
	dgFloat32 accelFreeze = world->m_freezeAccel2;
	dgVector forceDampVect (forceDamp, forceDamp, forceDamp, dgFloat32 (0.0f));
	for (dgInt32 i = 0; i < count; i ++) {
		dgDynamicBody* const body = (dgDynamicBody*) bodyArray[i].m_body;
		if (body->m_alived1) {
			if (body->m_invMass.m_w && body->IsRTTIType (dgBody::m_dynamicBodyRTTI)) {
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
			}
		}
	}

	for (dgInt32 i = 0; i < count; i ++) {
		dgBody* const body = bodyArray[i].m_body;
		if (body->m_alived1) {
			if (body->m_invMass.m_w && body->IsRTTIType (dgBody::m_dynamicBodyRTTI)) {
				body->UpdateMatrix (timestep, threadIndex);
			}
		}
	}

	if (isAutoSleep) {
		if (stackSleeping) {
			for (dgInt32 i = 0; i < count; i ++) {
				dgBody* const body = (dgDynamicBody*) bodyArray[i].m_body;
				if (body->m_alived1) {
					if (body->IsRTTIType (dgBody::m_dynamicBodyRTTI)) {
						body->m_netForce = zero;
						body->m_netTorque = zero;
						body->m_veloc = zero;
						body->m_omega = zero;
					}
				}
			}
		} else {
			if ((maxAccel > world->m_sleepTable[DG_SLEEP_ENTRIES - 1].m_maxAccel) ||
				(maxAlpha > world->m_sleepTable[DG_SLEEP_ENTRIES - 1].m_maxAlpha) ||
				(maxSpeed > world->m_sleepTable[DG_SLEEP_ENTRIES - 1].m_maxVeloc) ||
				(maxOmega > world->m_sleepTable[DG_SLEEP_ENTRIES - 1].m_maxOmega)) {
					for (dgInt32 i = 0; i < count; i ++) {
						dgDynamicBody* const body = (dgDynamicBody*) bodyArray[i].m_body;
						if (body->m_alived1) {
							if (body->IsRTTIType (dgBody::m_dynamicBodyRTTI)) {
								body->m_sleepingCounter = 0;
							}
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
						if (body->m_alived1) {
							if (body->IsRTTIType (dgBody::m_dynamicBodyRTTI)) {
								body->m_netForce = zero;
								body->m_netTorque = zero;
								body->m_veloc = zero;
								body->m_omega = zero;
								body->m_equilibrium = true;
							}
						}
					}
				} else {
					sleepCounter ++;
					for (dgInt32 i = 0; i < count; i ++) {
						dgDynamicBody* const body = (dgDynamicBody*) bodyArray[i].m_body;
						if (body->m_alived1) {
							if (body->IsRTTIType (dgBody::m_dynamicBodyRTTI)) {
								body->m_sleepingCounter = sleepCounter;
							}
						}
					}
				}
			}
		}
	}
}



dgInt32 dgWorldDynamicUpdate::SortJointInfoByColor (const dgParallelJointMap* const indirectIndexA, const dgParallelJointMap* const indirectIndexB, void* const context)
{
	if (indirectIndexA->m_bashIndex < indirectIndexB->m_bashIndex) {
		return -1;
	}
	if (indirectIndexA->m_bashIndex > indirectIndexB->m_bashIndex) {
		return 1;
	}
	return 0;
}


//dgInt32 dgWorldDynamicUpdate::LinearizeJointParallelArray(dgInt32 islandsCount, dgParallelSolverSyncData* const solverSyncData, dgJointInfo* const constraintArray, const dgIsland* const islandArray) const
void dgWorldDynamicUpdate::LinearizeJointParallelArray(dgParallelSolverSyncData* const solverSyncData, dgJointInfo* const constraintArray, const dgIsland* const island) const
{
	dgParallelJointMap* const jointInfoMap = solverSyncData->m_jointInfoMap;
	dgInt32 count = island->m_jointCount;
	dgInt32 index = island->m_jointStart;
	for (dgInt32 j = 0; j < count; j ++) {
		dgConstraint* const joint = constraintArray[index].m_joint;
//		constraintArray[index].m_color = 0;
		jointInfoMap[j].m_jointIndex = index;
		joint->m_index = index;
		index ++;
	}

	jointInfoMap[count].m_bashIndex = 0x7fffffff;
	jointInfoMap[count].m_jointIndex= -1;

	for (dgInt32 i = 0; i < count; i ++) {
		dgJointInfo& jointInfo = constraintArray[jointInfoMap[i].m_jointIndex];
		
		dgInt32 index = 0; 
		dgInt32 color = jointInfo.m_color;
		for (dgInt32 n = 1; n & color; n <<= 1) {
			index ++;
			dgAssert (index < 32);
		}
		jointInfoMap[i].m_bashIndex = index;

		color = 1 << index;
		dgConstraint* const constraint = jointInfo.m_joint;
		dgDynamicBody* const body0 = (dgDynamicBody*) constraint->m_body0;
		dgAssert (body0->IsRTTIType(dgBody::m_dynamicBodyRTTI));

		if (body0->m_invMass.m_w > dgFloat32 (0.0f)) {
			for (dgBodyMasterListRow::dgListNode* jointNode = body0->m_masterNode->GetInfo().GetFirst(); jointNode; jointNode = jointNode->GetNext()) {
				dgBodyMasterListCell& cell = jointNode->GetInfo();

				dgConstraint* const neiborgLink = cell.m_joint;
				if ((neiborgLink != constraint) && (neiborgLink->m_maxDOF)) {
					dgJointInfo& info = constraintArray[neiborgLink->m_index];
					info.m_color |= color;
				}
			}
		}

		dgDynamicBody* const body1 = (dgDynamicBody*)constraint->m_body1;
		dgAssert (body1->IsRTTIType(dgBody::m_dynamicBodyRTTI));
		if (body1->m_invMass.m_w > dgFloat32 (0.0f)) {
			for (dgBodyMasterListRow::dgListNode* jointNode = body1->m_masterNode->GetInfo().GetFirst(); jointNode; jointNode = jointNode->GetNext()) {
				dgBodyMasterListCell& cell = jointNode->GetInfo();

				dgConstraint* const neiborgLink = cell.m_joint;
				if ((neiborgLink != constraint) && (neiborgLink->m_maxDOF)) {
					dgJointInfo& info = constraintArray[neiborgLink->m_index];
					info.m_color |= color;
				}
			}
		}
	}

	dgSort (jointInfoMap, count, SortJointInfoByColor, constraintArray);

	dgInt32 bash = 0;
	for (int index = 0; index < count; index ++) {
		dgInt32 count = 0; 
		solverSyncData->m_jointBatches[bash].m_start = index;
		while (jointInfoMap[index].m_bashIndex == bash) {
			count ++;
			index ++;
		}
		solverSyncData->m_jointBatches[bash].m_count = count;
		bash ++;
		index --;
	}
	solverSyncData->m_batchesCount = bash;
}

