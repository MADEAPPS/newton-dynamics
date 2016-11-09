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
#include "dgConstraint.h"
#include "dgDynamicBody.h"
#include "dgWorldDynamicUpdate.h"





void dgWorldDynamicUpdate::CalculateReactionForcesParallel (const dgIsland* const island, dgFloat32 timestep) const
{
	dgParallelSolverSyncData syncData;

	dgWorld* const world = (dgWorld*) this;

	syncData.m_bodyLocks = dgAlloca (dgInt32, island->m_bodyCount + 1024);
	syncData.m_jointConflicts = dgAlloca (dgParallelSolverSyncData::dgParallelJointMap, island->m_jointCount + 1024);
	memset (syncData.m_bodyLocks, 0, island->m_bodyCount * sizeof (dgInt32));

	dgInt32 bodyCount = island->m_bodyCount;
	dgInt32 jointsCount = island->m_jointCount;
	dgJointInfo* const constraintArrayPtr = (dgJointInfo*) &world->m_jointsMemory[0];
	dgJointInfo* const constraintArray = &constraintArrayPtr[island->m_jointStart];

	LinearizeJointParallelArray (&syncData, constraintArray, island);
	
	dgJacobian* const internalForces = &world->m_solverMemory.m_internalForcesBuffer[0];
	internalForces[0].m_linear = dgVector::m_zero;
	internalForces[0].m_angular = dgVector::m_zero;

	const dgInt32 maxPasses = 4;
	syncData.m_timestep = timestep;
	syncData.m_invTimestep = (timestep > dgFloat32 (0.0f)) ? dgFloat32 (1.0f) / timestep : dgFloat32 (0.0f);
	syncData.m_invStepRK = (dgFloat32 (1.0f) / dgFloat32 (maxPasses));
	syncData.m_timestepRK = syncData.m_timestep * syncData.m_invStepRK;
	syncData.m_invTimestepRK = syncData.m_invTimestep * dgFloat32 (maxPasses);
	syncData.m_maxPasses = maxPasses;
	syncData.m_passes = world->m_solverMode;

	syncData.m_bodyCount = bodyCount;
	syncData.m_jointCount = jointsCount;
	syncData.m_atomicIndex = 0;
	syncData.m_island = island;

	InitilizeBodyArrayParallel (&syncData);
	BuildJacobianMatrixParallel (&syncData);
	SolverInitInternalForcesParallel (&syncData);
	CalculateForcesGameModeParallel (&syncData);
	IntegrateIslandParallel(&syncData); 
}


dgInt32 dgWorldDynamicUpdate::SortJointInfoByColor(const dgParallelSolverSyncData::dgParallelJointMap* const indirectIndexA, const dgParallelSolverSyncData::dgParallelJointMap* const indirectIndexB, void* const )
{
	dgInt64 keyA = (((dgInt64)indirectIndexA->m_bashCount) << 32) + indirectIndexA->m_color;
	dgInt64 keyB = (((dgInt64)indirectIndexB->m_bashCount) << 32) + indirectIndexB->m_color;
	if (keyA < keyB) {
		return -1;
	}
	if (keyA > keyB) {
		return 1;
	}
	return 0;
}


void dgWorldDynamicUpdate::LinearizeJointParallelArray(dgParallelSolverSyncData* const solverSyncData, dgJointInfo* const constraintArray, const dgIsland* const island) const
{
	dgParallelSolverSyncData::dgParallelJointMap* const jointInfoMap = solverSyncData->m_jointConflicts;
	dgInt32 count = island->m_jointCount;
	for (dgInt32 i = 0; i < count; i++) {
		dgConstraint* const joint = constraintArray[i].m_joint;
		joint->m_index = i;
		jointInfoMap[i].m_jointIndex = i;
		jointInfoMap[i].m_color = 0;
	}
	jointInfoMap[count].m_color = 0x7fffffff;
	jointInfoMap[count].m_jointIndex = -1;

	for (dgInt32 i = 0; i < count; i++) {
		dgInt32 index = 0;
		dgInt32 color = jointInfoMap[i].m_color;
		for (dgInt32 n = 1; n & color; n <<= 1) {
			index++;
			dgAssert(index < 32);
		}
		jointInfoMap[i].m_bashCount = index;
		color = 1 << index;
		dgAssert(jointInfoMap[i].m_jointIndex == i);
		dgJointInfo& jointInfo = constraintArray[i];

		dgConstraint* const constraint = jointInfo.m_joint;
		dgDynamicBody* const body0 = (dgDynamicBody*)constraint->m_body0;
		dgAssert(body0->IsRTTIType(dgBody::m_dynamicBodyRTTI));

		if (body0->m_invMass.m_w > dgFloat32(0.0f)) {
			for (dgBodyMasterListRow::dgListNode* jointNode = body0->m_masterNode->GetInfo().GetFirst(); jointNode; jointNode = jointNode->GetNext()) {
				dgBodyMasterListCell& cell = jointNode->GetInfo();

				dgConstraint* const neiborgLink = cell.m_joint;
				if ((neiborgLink != constraint) && (neiborgLink->m_maxDOF)) {
					dgParallelSolverSyncData::dgParallelJointMap& info = jointInfoMap[neiborgLink->m_index];
					info.m_color |= color;
				}
			}
		}

		dgDynamicBody* const body1 = (dgDynamicBody*)constraint->m_body1;
		dgAssert(body1->IsRTTIType(dgBody::m_dynamicBodyRTTI));
		if (body1->m_invMass.m_w > dgFloat32(0.0f)) {
			for (dgBodyMasterListRow::dgListNode* jointNode = body1->m_masterNode->GetInfo().GetFirst(); jointNode; jointNode = jointNode->GetNext()) {
				dgBodyMasterListCell& cell = jointNode->GetInfo();

				dgConstraint* const neiborgLink = cell.m_joint;
				if ((neiborgLink != constraint) && (neiborgLink->m_maxDOF)) {
					dgParallelSolverSyncData::dgParallelJointMap& info = jointInfoMap[neiborgLink->m_index];
					info.m_color |= color;
				}
			}
		}
	}

	dgSort(jointInfoMap, count, SortJointInfoByColor);

	dgInt32 acc = 0;
	dgInt32 bash = 0;
	dgInt32 bachCount = 0;
	for (int i = 0; i < count; i++) {
		if (jointInfoMap[i].m_bashCount > bash) {
			bash = jointInfoMap[i].m_bashCount;
			solverSyncData->m_jointBatches[bachCount + 1] = acc;
			bachCount++;
			dgAssert(bachCount < (dgInt32(sizeof (solverSyncData->m_jointBatches) / sizeof (solverSyncData->m_jointBatches[0])) - 1));
		}
		acc++;
	}
	bachCount++;
	solverSyncData->m_bachCount = bachCount;
	solverSyncData->m_jointBatches[bachCount] = acc;
	dgAssert(bachCount < (dgInt32(sizeof (solverSyncData->m_jointBatches) / sizeof (solverSyncData->m_jointBatches[0])) - 1));
}


void dgWorldDynamicUpdate::InitilizeBodyArrayParallel (dgParallelSolverSyncData* const syncData) const
{
	dgWorld* const world = (dgWorld*) this;
	const dgInt32 threadCounts = world->GetThreadCount();	

	dgJacobian* const internalForces = &world->m_solverMemory.m_internalForcesBuffer[0];
	internalForces[0].m_linear = dgVector(dgFloat32(0.0f));
	internalForces[0].m_angular = dgVector(dgFloat32(0.0f));

	syncData->m_atomicIndex = 1;
	for (dgInt32 i = 0; i < threadCounts; i ++) {
		world->QueueJob (InitializeBodyArrayParallelKernel, syncData, world);
	}
	world->SynchronizationBarrier();
}

void dgWorldDynamicUpdate::InitializeBodyArrayParallelKernel (void* const context, void* const worldContext, dgInt32 threadID)
{
	dgParallelSolverSyncData* const syncData = (dgParallelSolverSyncData*) context;
	dgWorld* const world = (dgWorld*) worldContext;
	dgInt32* const atomicIndex = &syncData->m_atomicIndex; 

	const dgIsland* const island = syncData->m_island;
	dgBodyInfo* const bodyArrayPtr = (dgBodyInfo*) &world->m_bodiesMemory[0]; 
	dgBodyInfo* const bodyArray = &bodyArrayPtr[island->m_bodyStart];
	dgJacobian* const internalForces = &world->m_solverMemory.m_internalForcesBuffer[0];

	if (syncData->m_timestep != dgFloat32 (0.0f)) {
		for (dgInt32 i = dgAtomicExchangeAndAdd(atomicIndex, 1); i < syncData->m_bodyCount; i = dgAtomicExchangeAndAdd(atomicIndex, 1)) {
			dgAssert (bodyArray[0].m_body->IsRTTIType (dgBody::m_dynamicBodyRTTI) || (((dgDynamicBody*)bodyArray[0].m_body)->m_accel.DotProduct3(((dgDynamicBody*)bodyArray[0].m_body)->m_accel)) == dgFloat32 (0.0f));
			dgAssert (bodyArray[0].m_body->IsRTTIType (dgBody::m_dynamicBodyRTTI) || (((dgDynamicBody*)bodyArray[0].m_body)->m_alpha.DotProduct3(((dgDynamicBody*)bodyArray[0].m_body)->m_alpha)) == dgFloat32 (0.0f));

			dgBody* const body = bodyArray[i].m_body;
			dgAssert (body->m_index == i);
			if (!body->m_equilibrium) {
				dgAssert (body->m_invMass.m_w > dgFloat32 (0.0f));
				body->AddDampingAcceleration(syncData->m_timestep);
				body->CalcInvInertiaMatrix ();
			}

			if (body->m_active) {
				// re use these variables for temp storage 
				body->m_netForce = body->m_veloc;
				body->m_netTorque = body->m_omega;
				internalForces[i].m_linear = dgVector::m_zero;
				internalForces[i].m_angular = dgVector::m_zero;
			}
		}
	} else {
		for (dgInt32 i = dgAtomicExchangeAndAdd(atomicIndex, 1); i < syncData->m_bodyCount; i = dgAtomicExchangeAndAdd(atomicIndex, 1)) {
			dgAssert(bodyArray[0].m_body->IsRTTIType(dgBody::m_dynamicBodyRTTI) || (((dgDynamicBody*)bodyArray[0].m_body)->m_accel.DotProduct3(((dgDynamicBody*)bodyArray[0].m_body)->m_accel)) == dgFloat32(0.0f));
			dgAssert(bodyArray[0].m_body->IsRTTIType(dgBody::m_dynamicBodyRTTI) || (((dgDynamicBody*)bodyArray[0].m_body)->m_alpha.DotProduct3(((dgDynamicBody*)bodyArray[0].m_body)->m_alpha)) == dgFloat32(0.0f));

			dgBody* const body = bodyArray[i].m_body;
			if (!body->m_equilibrium) {
				dgAssert(body->m_invMass.m_w > dgFloat32(0.0f));
				body->CalcInvInertiaMatrix();
			}

			if (body->m_active) {
				 //re use these variables for temp storage 
				body->m_netForce = body->m_veloc;
				body->m_netTorque = body->m_omega;
				internalForces[i].m_linear = dgVector::m_zero;
				internalForces[i].m_angular = dgVector::m_zero;
			}
		}
	}
}


void dgWorldDynamicUpdate::BuildJacobianMatrixParallel (dgParallelSolverSyncData* const syncData) const
{
	dgWorld* const world = (dgWorld*) this;
	const dgInt32 threadCounts = world->GetThreadCount();	

	syncData->m_atomicIndex = 0;
	for (dgInt32 i = 0; i < threadCounts; i ++) {
		world->QueueJob (BuildJacobianMatrixParallelKernel, syncData, world);
	}
	world->SynchronizationBarrier();
}


void dgWorldDynamicUpdate::BuildJacobianMatrixParallelKernel (void* const context, void* const worldContext, dgInt32 threadID)
{
	dgParallelSolverSyncData* const syncData = (dgParallelSolverSyncData*) context;
	dgWorld* const world = (dgWorld*) worldContext;
	dgInt32* const atomicIndex = &syncData->m_atomicIndex; 
	const dgIsland* const island = syncData->m_island;
	dgBodyInfo* const bodyArrayPtr = (dgBodyInfo*) &world->m_bodiesMemory[0]; 
	dgBodyInfo* const bodyArray = &bodyArrayPtr[island->m_bodyStart];
	dgJointInfo* const constraintArrayPtr = (dgJointInfo*) &world->m_jointsMemory[0];
	dgJointInfo* const constraintArray = &constraintArrayPtr[island->m_jointStart];
	dgJacobianMatrixElement* const matrixRow = &world->m_solverMemory.m_jacobianBuffer[0];
	dgAssert (syncData->m_jointCount);

	dgContraintDescritor constraintParams;
	constraintParams.m_world = world;
	constraintParams.m_threadIndex = threadID;
	constraintParams.m_timestep = syncData->m_timestep;
	constraintParams.m_invTimestep = syncData->m_invTimestep;

	dgFloat32 forceOrImpulseScale = (syncData->m_timestep > dgFloat32 (0.0f)) ? dgFloat32 (1.0f) : dgFloat32 (0.0f);
	for (dgInt32 jointIndex = dgAtomicExchangeAndAdd(atomicIndex, 1); jointIndex < syncData->m_jointCount;  jointIndex = dgAtomicExchangeAndAdd(atomicIndex, 1)) {
		dgJointInfo* const jointInfo = &constraintArray[jointIndex];
		dgConstraint* const constraint = jointInfo->m_joint;

		dgInt32 rowBase = dgAtomicExchangeAndAdd(&syncData->m_jacobianMatrixRowAtomicIndex, jointInfo->m_pairCount);

		world->GetJacobianDerivatives(constraintParams, jointInfo, constraint, matrixRow, rowBase);

		dgAssert (jointInfo->m_m0 >= 0);
		dgAssert (jointInfo->m_m1 >= 0);
		dgAssert (jointInfo->m_m0 != jointInfo->m_m1);
		dgAssert (jointInfo->m_m0 < island->m_bodyCount);
		dgAssert (jointInfo->m_m1 < island->m_bodyCount);
		world->BuildJacobianMatrix (bodyArray, jointInfo, matrixRow, forceOrImpulseScale);
	}
}


void dgWorldDynamicUpdate::SolverInitInternalForcesParallel (dgParallelSolverSyncData* const syncData) const
{
	dgWorld* const world = (dgWorld*) this;
	const dgInt32 threadCounts = world->GetThreadCount();	

	syncData->m_atomicIndex = 0;
	for (dgInt32 i = 0; i < threadCounts; i ++) {
		world->QueueJob (SolverInitInternalForcesParallelKernel, syncData, world);
	}
	world->SynchronizationBarrier();
}


void dgWorldDynamicUpdate::SolverInitInternalForcesParallelKernel (void* const context, void* const worldContext, dgInt32 threadID)
{
	dgParallelSolverSyncData* const syncData = (dgParallelSolverSyncData*) context;
	dgWorld* const world = (dgWorld*) worldContext;
	const dgIsland* const island = syncData->m_island;
	dgJacobian* const internalForces = &world->m_solverMemory.m_internalForcesBuffer[0];
	dgJacobianMatrixElement* const matrixRow = &world->m_solverMemory.m_jacobianBuffer[0];
	dgInt32* const atomicIndex = &syncData->m_atomicIndex; 
	dgJointInfo* const constraintArrayPtr = (dgJointInfo*) &world->m_jointsMemory[0];
	dgJointInfo* const constraintArray = &constraintArrayPtr[island->m_jointStart];
	dgInt32* const bodyLocks = syncData->m_bodyLocks;

	for (dgInt32 i = dgAtomicExchangeAndAdd(atomicIndex, 1); i < syncData->m_jointCount;  i = dgAtomicExchangeAndAdd(atomicIndex, 1)) {
		dgJointInfo* const jointInfo = &constraintArray[i];
		if (jointInfo->m_joint->m_solverActive) {
			dgJacobian y0;
			dgJacobian y1;
			world->InitJointForce (jointInfo, matrixRow, y0, y1);
			const dgInt32 m0 = jointInfo->m_m0;
			const dgInt32 m1 = jointInfo->m_m1;
			dgAssert (m0 != m1);

			if (m0) {
				dgSpinLock(&bodyLocks[m0], false);
				internalForces[m0].m_linear += y0.m_linear;
				internalForces[m0].m_angular += y0.m_angular;
				dgSpinUnlock(&bodyLocks[m0]);
			}
			if (m1) {
				dgSpinLock(&bodyLocks[m1], false);
				internalForces[m1].m_linear += y1.m_linear;
				internalForces[m1].m_angular += y1.m_angular;
				dgSpinUnlock(&bodyLocks[m1]);
			}
		}
	}
}



void dgWorldDynamicUpdate::CalculateJointsAccelParallelKernel (void* const context, void* const worldContext, dgInt32 threadID)
{
	dgParallelSolverSyncData* const syncData = (dgParallelSolverSyncData*) context;
	dgWorld* const world = (dgWorld*) worldContext;
	const dgIsland* const island = syncData->m_island;
	dgJointInfo* const constraintArrayPtr = (dgJointInfo*) &world->m_jointsMemory[0];
	dgJointInfo* const constraintArray = &constraintArrayPtr[island->m_jointStart];
	dgJacobianMatrixElement* const matrixRow = &world->m_solverMemory.m_jacobianBuffer[0];

	dgJointAccelerationDecriptor joindDesc;
	joindDesc.m_timeStep = syncData->m_timestepRK;
	joindDesc.m_invTimeStep = syncData->m_invTimestepRK;
	joindDesc.m_firstPassCoefFlag = syncData->m_firstPassCoef;

	dgInt32* const atomicIndex = &syncData->m_atomicIndex; 
	if (joindDesc.m_firstPassCoefFlag == dgFloat32 (0.0f)) {
		for (dgInt32 curJoint = dgAtomicExchangeAndAdd(atomicIndex, 1); curJoint < syncData->m_jointCount;  curJoint = dgAtomicExchangeAndAdd(atomicIndex, 1)) {
			dgJointInfo* const jointInfo = &constraintArray[curJoint];
			dgConstraint* const constraint = jointInfo->m_joint;
			if (constraint->m_solverActive) {
				joindDesc.m_rowsCount = jointInfo->m_pairCount;
				joindDesc.m_rowMatrix = &matrixRow[jointInfo->m_pairStart];
				constraint->JointAccelerations(&joindDesc);
			}
		}
	} else {
		const dgIsland* const island = syncData->m_island;
		dgBodyInfo* const bodyArrayPtr = (dgBodyInfo*) &world->m_bodiesMemory[0]; 
		const dgBodyInfo* const bodyArray = &bodyArrayPtr[island->m_bodyStart];

		for (dgInt32 curJoint = dgAtomicExchangeAndAdd(atomicIndex, 1); curJoint < syncData->m_jointCount;  curJoint = dgAtomicExchangeAndAdd(atomicIndex, 1)) {
			dgJointInfo* const jointInfo = &constraintArray[curJoint];
			dgConstraint* const constraint = jointInfo->m_joint;
			if (constraint->m_solverActive) {
				const dgInt32 m0 = jointInfo->m_m0;
				const dgInt32 m1 = jointInfo->m_m1;
				const dgBody* const body0 = bodyArray[m0].m_body;
				const dgBody* const body1 = bodyArray[m1].m_body;
				if (!(body0->m_resting & body1->m_resting)) {
					joindDesc.m_rowsCount = jointInfo->m_pairCount;
					joindDesc.m_rowMatrix = &matrixRow[jointInfo->m_pairStart];
					constraint->JointAccelerations(&joindDesc);
				}
			}
		}
	}
}


void dgWorldDynamicUpdate::CalculateJointsVelocParallelKernel (void* const context, void* const worldContext, dgInt32 threadID)
{
	dgParallelSolverSyncData* const syncData = (dgParallelSolverSyncData*) context;
	dgWorld* const world = (dgWorld*) worldContext;

	const dgIsland* const island = syncData->m_island;
	dgBodyInfo* const bodyArrayPtr = (dgBodyInfo*) &world->m_bodiesMemory[0]; 
	dgBodyInfo* const bodyArray = &bodyArrayPtr[island->m_bodyStart];

	dgJacobian* const internalForces = &world->m_solverMemory.m_internalForcesBuffer[0];

	dgVector speedFreeze2 (world->m_freezeSpeed2 * dgFloat32 (0.1f));
	dgVector freezeOmega2 (world->m_freezeOmega2 * dgFloat32 (0.1f));
	//dgVector forceActiveMask ((syncData->m_jointCount <= DG_SMALL_ISLAND_COUNT) ?  dgFloat32 (-1.0f): dgFloat32 (0.0f));
	dgVector forceActiveMask ((syncData->m_jointCount <= DG_SMALL_ISLAND_COUNT) ?  dgVector (-1, -1, -1, -1) : dgFloat32 (0.0f));
	dgInt32* const atomicIndex = &syncData->m_atomicIndex;

	if (syncData->m_timestepRK != dgFloat32 (0.0f)) {
		dgVector timestep4 (syncData->m_timestepRK);
		for (dgInt32 i = dgAtomicExchangeAndAdd(atomicIndex, 1); i < syncData->m_bodyCount;  i = dgAtomicExchangeAndAdd(atomicIndex, 1)) {
			dgDynamicBody* const body = (dgDynamicBody*) bodyArray[i].m_body;
			dgAssert (body->m_index == i);
			world->ApplyNetVelcAndOmega (body, internalForces[i], timestep4, speedFreeze2, forceActiveMask);
		}
	} else {
		for (dgInt32 i = dgAtomicExchangeAndAdd(atomicIndex, 1); i < syncData->m_bodyCount;  i = dgAtomicExchangeAndAdd(atomicIndex, 1)) {
			dgBody* const body = bodyArray[i].m_body;
			if (body->m_active) {
				const dgVector& linearMomentum = internalForces[i].m_linear;
				const dgVector& angularMomentum = internalForces[i].m_angular;

				body->m_veloc += linearMomentum.Scale4(body->m_invMass.m_w);
				body->m_omega += body->m_invWorldInertiaMatrix.RotateVector(angularMomentum);
			}
		}
	}
}

/*
void dgWorldDynamicUpdate::CalculateJointsImpulseVelocParallelKernel (void* const context, void* const worldContext, dgInt32 threadID)
{
	dgParallelSolverSyncData* const syncData = (dgParallelSolverSyncData*) context;
	dgWorld* const world = (dgWorld*) worldContext;

	const dgIsland* const island = syncData->m_island;
	dgBodyInfo* const bodyArrayPtr = (dgBodyInfo*) &world->m_bodiesMemory[0]; 
	dgBodyInfo* const bodyArray = &bodyArrayPtr[island->m_bodyStart];

	dgJacobian* const internalForces = &world->m_solverMemory.m_internalForces[0];
	//dgJacobian* const internalVeloc = &world->m_solverMemory.m_internalVeloc[0];

	dgInt32* const atomicIndex = &syncData->m_atomicIndex;
	for (dgInt32 i = dgAtomicExchangeAndAdd(atomicIndex, 1); i < syncData->m_bodyCount;  i = dgAtomicExchangeAndAdd(atomicIndex, 1)) {
		dgInt32 index = i + 1;
		dgAssert (index);
		dgDynamicBody* const body = (dgDynamicBody*) bodyArray[index].m_body;
		dgAssert (body->m_index == index);

		const dgVector& linearMomentum = internalForces[index].m_linear;
		const dgVector& angularMomentum = internalForces[index].m_angular;

		body->m_veloc += linearMomentum.Scale4(body->m_invMass.m_w);
		body->m_omega += body->m_invWorldInertiaMatrix.RotateVector (angularMomentum);

		//internalVeloc[index].m_linear += body->m_veloc;
		//internalVeloc[index].m_angular += body->m_omega;
	}
}
*/

void dgWorldDynamicUpdate::UpdateFeedbackForcesParallelKernel (void* const context, void* const worldContext, dgInt32 threadID)
{
	dgParallelSolverSyncData* const syncData = (dgParallelSolverSyncData*) context;
	dgWorld* const world = (dgWorld*) worldContext;
	const dgIsland* const island = syncData->m_island;
	dgJointInfo* const constraintArrayPtr = (dgJointInfo*) &world->m_jointsMemory[0];
	dgJointInfo* const constraintArray = &constraintArrayPtr[island->m_jointStart];
	dgJacobianMatrixElement* const matrixRow = &world->m_solverMemory.m_jacobianBuffer[0];

	dgInt32 hasJointFeeback = 0;
	dgInt32* const atomicIndex = &syncData->m_atomicIndex;
	
	for (dgInt32 curJoint = dgAtomicExchangeAndAdd(atomicIndex, 1); curJoint < syncData->m_jointCount;  curJoint = dgAtomicExchangeAndAdd(atomicIndex, 1)) {
		dgJointInfo* const jointInfo = &constraintArray[curJoint];
		dgConstraint* const constraint = jointInfo->m_joint;
		if (constraint->m_solverActive) {
			const dgInt32 first = jointInfo->m_pairStart;
			const dgInt32 count = jointInfo->m_pairCount;

			for (dgInt32 j = 0; j < count; j++) {
				dgJacobianMatrixElement* const row = &matrixRow[j + first];
				dgFloat32 val = row->m_force;
				dgAssert(dgCheckFloat(val));
				row->m_jointFeebackForce[0].m_force = val;
				row->m_jointFeebackForce[0].m_impact = row->m_maxImpact * syncData->m_timestepRK;
			}
			hasJointFeeback |= (constraint->m_updaFeedbackCallback ? 1 : 0);
		}
	}
	syncData->m_hasJointFeeback[threadID] = hasJointFeeback;
}


void dgWorldDynamicUpdate::UpdateBodyVelocityParallelKernel (void* const context, void* const worldContext, dgInt32 threadID)
{
	dgParallelSolverSyncData* const syncData = (dgParallelSolverSyncData*) context;
	dgWorld* const world = (dgWorld*) worldContext;

	const dgIsland* const island = syncData->m_island;
	dgBodyInfo* const bodyArrayPtr = (dgBodyInfo*) &world->m_bodiesMemory[0]; 
	dgBodyInfo* const bodyArray = &bodyArrayPtr[island->m_bodyStart];

	dgFloat32 maxAccNorm2 = DG_SOLVER_MAX_ERROR * DG_SOLVER_MAX_ERROR;

	//dgFloat32 invTimestepSrc = dgFloat32 (1.0f) / syncData->m_timestep;
	dgFloat32 invTimestepSrc = syncData->m_invTimestep;

	dgVector invTime (invTimestepSrc);
	dgInt32* const atomicIndex = &syncData->m_atomicIndex;
	dgVector forceActiveMask ((syncData->m_jointCount <= DG_SMALL_ISLAND_COUNT) ?  dgVector (-1, -1, -1, -1) : dgFloat32 (0.0f));
	for (dgInt32 i = dgAtomicExchangeAndAdd(atomicIndex, 1); i < syncData->m_bodyCount; i = dgAtomicExchangeAndAdd(atomicIndex, 1)) {
		dgDynamicBody* const body = (dgDynamicBody*) bodyArray[i].m_body;
		world->ApplyNetTorqueAndForce (body, invTime, maxAccNorm2, forceActiveMask);
	}
}


void dgWorldDynamicUpdate::KinematicCallbackUpdateParallelKernel (void* const context, void* const worldContext, dgInt32 threadID)
{
	dgParallelSolverSyncData* const syncData = (dgParallelSolverSyncData*) context;
	dgWorld* const world = (dgWorld*) worldContext;
	const dgIsland* const island = syncData->m_island;
	dgJointInfo* const constraintArrayPtr = (dgJointInfo*) &world->m_jointsMemory[0];
	dgJointInfo* const constraintArray = &constraintArrayPtr[island->m_jointStart];

	dgInt32* const atomicIndex = &syncData->m_atomicIndex;
	for (dgInt32 i = dgAtomicExchangeAndAdd(atomicIndex, 1); i < syncData->m_jointCount;  i = dgAtomicExchangeAndAdd(atomicIndex, 1)) {
		dgInt32 curJoint = i;
		if (constraintArray[curJoint].m_joint->m_updaFeedbackCallback) {
			constraintArray[curJoint].m_joint->m_updaFeedbackCallback (*constraintArray[curJoint].m_joint, syncData->m_timestep, threadID);
		}
	}
}


void dgWorldDynamicUpdate::IntegrateIslandParallel(dgParallelSolverSyncData* const syncData) const
{
	dgWorld* const world = (dgWorld*) this;
//	dgWorldDynamicUpdate::IntegrateIslandParallelKernel (syncData, world, 0);
	world->IntegrateVelocity (syncData->m_island, DG_SOLVER_MAX_ERROR, syncData->m_timestep, 0); 
}


void dgWorldDynamicUpdate::CalculateForcesGameModeParallel (dgParallelSolverSyncData* const syncData) const
{
	dgWorld* const world = (dgWorld*) this;
	const dgInt32 threadCounts = world->GetThreadCount();	

	const dgInt32 passes = syncData->m_passes;
	const dgInt32 maxPasses = syncData->m_maxPasses;
//	const dgInt32 batchCount = syncData->m_bachCount;
	syncData->m_firstPassCoef = dgFloat32 (0.0f);

	for (dgInt32 step = 0; step < maxPasses; step++) {

		syncData->m_atomicIndex = 0;
		for (dgInt32 i = 0; i < threadCounts; i++) {
			world->QueueJob(CalculateJointsAccelParallelKernel, syncData, world);
		}
		world->SynchronizationBarrier();
		syncData->m_firstPassCoef = dgFloat32(1.0f);

		dgFloat32 accNorm = DG_SOLVER_MAX_ERROR * dgFloat32(2.0f);
		for (dgInt32 k = 0; (k < passes) && (accNorm > DG_SOLVER_MAX_ERROR); k++) {

/*
			dgInt32 batchIndex = 0;
			dgInt32 count = syncData->m_jointBatches[batchIndex + 1];
			while ((batchIndex < batchCount) && (count >= threadCounts * 8)) {
				syncData->m_bachIndex = syncData->m_jointBatches[batchIndex + 1];
				syncData->m_atomicIndex = syncData->m_jointBatches[batchIndex];
				for (dgInt32 i = 0; i < threadCounts; i++) {
					world->QueueJob(CalculateJointsForceParallelKernel, syncData, world);
				}
				world->SynchronizationBarrier();
				accNorm = dgFloat32(0.0f);
				for (dgInt32 i = 0; i < threadCounts; i++) {
					accNorm = dgMax(accNorm, syncData->m_accelNorm[i]);
				}

				batchIndex++;
				count = syncData->m_jointBatches[batchIndex + 1] - syncData->m_jointBatches[batchIndex];
			}

			if (batchIndex < batchCount) {
				syncData->m_bachIndex = syncData->m_jointBatches[batchCount];
				syncData->m_atomicIndex = syncData->m_jointBatches[batchIndex];
				CalculateJointsForceParallelKernel(syncData, world, 0);
				accNorm = dgMax(accNorm, syncData->m_accelNorm[0]);
			}
*/			

			syncData->m_atomicIndex = 0;
			for (dgInt32 i = 0; i < threadCounts; i++) {
				world->QueueJob(CalculateJointsForceParallelKernel, syncData, world);
			}
			world->SynchronizationBarrier();
			accNorm = dgFloat32(0.0f);
			for (dgInt32 i = 0; i < threadCounts; i++) {
				accNorm = dgMax(accNorm, syncData->m_accelNorm[i]);
			}
		}


		syncData->m_atomicIndex = 1;
		for (dgInt32 j = 0; j < threadCounts; j++) {
			world->QueueJob(CalculateJointsVelocParallelKernel, syncData, world);
		}
		world->SynchronizationBarrier();
	}

	if (syncData->m_timestepRK != dgFloat32 (0.0f)) {
		syncData->m_atomicIndex = 0;
		for (dgInt32 j = 0; j < threadCounts; j ++) {
			world->QueueJob (UpdateFeedbackForcesParallelKernel, syncData, world);
		}
		world->SynchronizationBarrier();

		dgInt32 hasJointFeeback = 0;
		for (dgInt32 i = 0; i < DG_MAX_THREADS_HIVE_COUNT; i ++) {
			hasJointFeeback |= syncData->m_hasJointFeeback[i];
		}

		syncData->m_atomicIndex = 1;
		for (dgInt32 j = 0; j < threadCounts; j++) {
			world->QueueJob(UpdateBodyVelocityParallelKernel, syncData, world);
		}
		world->SynchronizationBarrier();

		if (hasJointFeeback) {
			syncData->m_atomicIndex = 0;
			for (dgInt32 j = 0; j < threadCounts; j++) {
				world->QueueJob(KinematicCallbackUpdateParallelKernel, syncData, world);
			}
			world->SynchronizationBarrier();
		}

	} else {
		const dgInt32 count = syncData->m_bodyCount;
		const dgIsland* const island = syncData->m_island;
		dgBodyInfo* const bodyArrayPtr = (dgBodyInfo*)&world->m_bodiesMemory[0];
		dgBodyInfo* const bodyArray = &bodyArrayPtr[island->m_bodyStart];
		for (dgInt32 i = 1; i < count; i++) {
			dgBody* const body = bodyArray[i].m_body;
			if (body->m_active) {
				body->m_netForce = dgVector::m_zero;
				body->m_netTorque = dgVector::m_zero;
			}
		}
	}
}


void dgWorldDynamicUpdate::CalculateJointsForceParallelKernel (void* const context, void* const worldContext, dgInt32 threadID)
{
	dgParallelSolverSyncData* const syncData = (dgParallelSolverSyncData*) context;
	dgWorld* const world = (dgWorld*) worldContext;

	const dgIsland* const island = syncData->m_island;
	dgJacobianMatrixElement* const matrixRow = &world->m_solverMemory.m_jacobianBuffer[0];
	dgBodyInfo* const bodyArrayPtr = (dgBodyInfo*) &world->m_bodiesMemory[0]; 
	const dgBodyInfo* const bodyArray = &bodyArrayPtr[island->m_bodyStart];
	dgJointInfo* const constraintArrayPtr = (dgJointInfo*) &world->m_jointsMemory[0];
	dgJointInfo* const constraintArray = &constraintArrayPtr[island->m_jointStart];
	dgJacobian* const internalForces = &world->m_solverMemory.m_internalForcesBuffer[0];
//	const int jointCount = syncData->m_bachIndex;
	const int jointCount = syncData->m_jointCount;

	dgInt32* const bodyLocks = syncData->m_bodyLocks;
	dgParallelSolverSyncData::dgParallelJointMap* const jointInfoMap = syncData->m_jointConflicts;

	dgInt32* const atomicIndex = &syncData->m_atomicIndex;
	dgFloat32 accNorm = dgFloat32(0.0f);
	for (dgInt32 i = dgAtomicExchangeAndAdd(atomicIndex, 1); i < jointCount; i = dgAtomicExchangeAndAdd(atomicIndex, 1)) {
		dgInt32 index = jointInfoMap[i].m_jointIndex;
		dgJointInfo* const jointInfo = &constraintArray[index];

		const dgInt32 m0 = jointInfo->m_m0;
		const dgInt32 m1 = jointInfo->m_m1;
		dgAssert(m0 != m1);

		dgSpinLock(&syncData->m_lock0, false);
			if (m0) {
				dgSpinLock(&bodyLocks[m0], false);
			}
			if (m1) {
				dgSpinLock(&bodyLocks[m1], false);
			}
		dgSpinUnlock(&syncData->m_lock0);

		dgFloat32 accel = world->CalculateJointForce(jointInfo, bodyArray, internalForces, matrixRow);

		dgSpinLock(&syncData->m_lock1, false);
			if (m0) {
				dgSpinUnlock(&bodyLocks[m0]);
			}
			if (m1) {
				dgSpinUnlock(&bodyLocks[m1]);
			}
		dgSpinUnlock(&syncData->m_lock1);

		accNorm = (accel > accNorm) ? accel : accNorm;
	}

	syncData->m_accelNorm[threadID] = accNorm;
}
