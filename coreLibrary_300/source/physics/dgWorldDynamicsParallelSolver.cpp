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
#include "dgConstraint.h"
#include "dgDynamicBody.h"
#include "dgWorldDynamicUpdate.h"

#define dgParalletSolverStackPoolSize (1<<8)

void dgWorldDynamicUpdate::CalculateReactionForcesParallel (const dgIsland* const island, dgFloat32 timestep) const
{
	dgParallelSolverSyncData syncData;

	dgWorld* const world = (dgWorld*) this;

	world->m_pairMemoryBuffer.ExpandCapacityIfNeessesary (island->m_jointCount + 1024, sizeof (dgInt32));
	syncData.m_bodyLocks = (dgInt32*) (&world->m_pairMemoryBuffer[0]);
	memset (syncData.m_bodyLocks, 0, island->m_bodyCount * sizeof (syncData.m_bodyLocks[0]));

	dgInt32 bodyCount = island->m_bodyCount - 1;
	dgInt32 jointsCount = island->m_jointCount;
	dgJointInfo* const constraintArrayPtr = (dgJointInfo*) &world->m_jointsMemory[0];
	dgJointInfo* const constraintArray = &constraintArrayPtr[island->m_jointStart];
	//CreateParallelArrayBatchArrays (&syncData, constraintArray, island);

	dgInt32 index = island->m_jointStart;
	for (dgInt32 i = 0; i < jointsCount; i ++) {
		dgConstraint* const joint = constraintArray[i].m_joint;
		joint->m_index = index;
		index ++;
	}
	
	dgJacobian* const internalForces = &world->m_solverMemory.m_internalForces[0];
	dgVector zero(dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f));
	internalForces[0].m_linear = zero;
	internalForces[0].m_angular = zero;

	dgInt32 maxPasses = dgInt32 (world->m_solverMode + LINEAR_SOLVER_SUB_STEPS);


	syncData.m_timestep = timestep;
	syncData.m_invTimestep = (timestep > dgFloat32 (0.0f)) ? dgFloat32 (1.0f) / timestep : dgFloat32 (0.0f);
	syncData.m_invStepRK = (dgFloat32 (1.0f) / dgFloat32 (maxPasses));
	syncData.m_timestepRK = syncData.m_timestep * syncData.m_invStepRK;
	syncData.m_invTimestepRK = syncData.m_invTimestep * dgFloat32 (maxPasses);
	syncData.m_maxPasses = maxPasses;

	syncData.m_bodyCount = bodyCount;
	syncData.m_jointCount = jointsCount;
	syncData.m_atomicIndex = 0;
	syncData.m_island = island;

	InitilizeBodyArrayParallel (&syncData);
	BuildJacobianMatrixParallel (&syncData);
	SolverInitInternalForcesParallel (&syncData);
	CalculateForcesGameModeParallel (&syncData);
	IntegrateInslandParallel(&syncData); 
}

/*
dgInt32 dgWorldDynamicUpdate::SortJointInfoByBatchIndex (const dgParallelJointMap* const indirectIndexA, const dgParallelJointMap* const indirectIndexB, void* const context)
{
	if (indirectIndexA->m_bashIndex < indirectIndexB->m_bashIndex) {
		return -1;
	}
	if (indirectIndexA->m_bashIndex > indirectIndexB->m_bashIndex) {
		return 1;
	}
	return 0;
}

void dgWorldDynamicUpdate::CreateParallelArrayBatchArrays(dgParallelSolverSyncData* const solverSyncData, dgJointInfo* const constraintArray, const dgIsland* const island) const
{
	dgParallelJointMap* const jointInfoMap = solverSyncData->m_jointInfoMap;
	dgInt32 count = island->m_jointCount;
	dgInt32 index = island->m_jointStart;
	for (dgInt32 j = 0; j < count; j ++) {
		dgConstraint* const joint = constraintArray[index].m_joint;
		constraintArray[index].m_color = 0;
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
				if ((neiborgLink != constraint) && (neiborgLink->m_maxDOF) && neiborgLink->IsActive()) {
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
				if ((neiborgLink != constraint) && (neiborgLink->m_maxDOF) && neiborgLink->IsActive()) {
					dgJointInfo& info = constraintArray[neiborgLink->m_index];
					info.m_color |= color;
				}
			}
		}
	}

	dgSort (jointInfoMap, count, SortJointInfoByBatchIndex, constraintArray);
}
*/

void dgWorldDynamicUpdate::InitilizeBodyArrayParallel (dgParallelSolverSyncData* const syncData) const
{
	dgWorld* const world = (dgWorld*) this;
	dgInt32 threadCounts = world->GetThreadCount();	

	syncData->m_atomicIndex = 0;
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

	dgJacobian* const internalForces = &world->m_solverMemory.m_internalForces[0];
	dgVector zero(dgFloat32 (0.0f));

	for (dgInt32 i = dgAtomicExchangeAndAdd(atomicIndex, 1); i < syncData->m_bodyCount; i = dgAtomicExchangeAndAdd(atomicIndex, 1)) {
		dgAssert (bodyArray[0].m_body->IsRTTIType (dgBody::m_dynamicBodyRTTI) || (((dgDynamicBody*)bodyArray[0].m_body)->m_accel % ((dgDynamicBody*)bodyArray[0].m_body)->m_accel) == dgFloat32 (0.0f));
		dgAssert (bodyArray[0].m_body->IsRTTIType (dgBody::m_dynamicBodyRTTI) || (((dgDynamicBody*)bodyArray[0].m_body)->m_alpha % ((dgDynamicBody*)bodyArray[0].m_body)->m_alpha) == dgFloat32 (0.0f));

		dgInt32 index = i + 1;
		dgBody* const body = bodyArray[index].m_body;
		dgAssert (body->m_invMass.m_w > dgFloat32 (0.0f));
		body->AddDampingAcceleration();
		body->CalcInvInertiaMatrix ();

		// re use these variables for temp storage 
		body->m_netForce = body->m_veloc;
		body->m_netTorque = body->m_omega;

		internalForces[index].m_linear = zero;
		internalForces[index].m_angular = zero;
	}
}


void dgWorldDynamicUpdate::BuildJacobianMatrixParallel (dgParallelSolverSyncData* const syncData) const
{
	dgWorld* const world = (dgWorld*) this;
	dgInt32 threadCounts = world->GetThreadCount();	

	syncData->m_atomicIndex = 0;
	for (dgInt32 i = 0; i < threadCounts; i ++) {
		world->QueueJob (BuildJacobianMatrixParallelKernel, syncData, world);
	}
	world->SynchronizationBarrier();
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
	dgJacobianMatrixElement* const matrixRow = &world->m_solverMemory.m_memory[0];

	dgVector zero (dgFloat32 (0.0f));
	for (dgInt32 i = dgAtomicExchangeAndAdd(atomicIndex, 1); i < syncData->m_jointCount;  i = dgAtomicExchangeAndAdd(atomicIndex, 1)) {
		dgInt32 jointIndex = i;
		dgJointInfo* const jointInfo = &constraintArray[jointIndex];

		dgInt32 rowBase = dgAtomicExchangeAndAdd(&syncData->m_jacobianMatrixRowAtomicIndex, jointInfo->m_autoPaircount);
		world->GetJacobianDerivativesParallel (jointInfo, threadID, rowBase, syncData->m_timestep);

		dgInt32 index = jointInfo->m_autoPairstart;
		dgInt32 count = jointInfo->m_autoPaircount;
		dgInt32 m0 = jointInfo->m_m0;
		dgInt32 m1 = jointInfo->m_m1;
		dgAssert (m0 >= 0);
		dgAssert (m1 >= 0);
		dgAssert (m0 != m1);
		dgAssert (m0 < island->m_bodyCount);
		dgAssert (m1 < island->m_bodyCount);

		const dgBody* const body0 = bodyArray[m0].m_body;
		const dgBody* const body1 = bodyArray[m1].m_body;

		const dgVector invMass0 (body0->m_invMass[3]);
		const dgMatrix& invInertia0 = body0->m_invWorldInertiaMatrix;
		const dgVector invMass1 (body1->m_invMass[3]);
		const dgMatrix& invInertia1 = body1->m_invWorldInertiaMatrix;

		dgVector accel0 (zero); 
		dgVector alpha0 (zero); 
		if (body0->IsRTTIType (dgBody::m_dynamicBodyRTTI)) {
			accel0 = ((dgDynamicBody*)body0)->m_accel;
			alpha0 = ((dgDynamicBody*)body0)->m_alpha;
		}

		dgVector accel1 (zero); 
		dgVector alpha1 (zero); 
		if (body1->IsRTTIType (dgBody::m_dynamicBodyRTTI)) {
			accel1 = ((dgDynamicBody*)body1)->m_accel;
			alpha1 = ((dgDynamicBody*)body1)->m_alpha;
		}

		// the parallel solve will not deal with impulse phase, only for large solution solution
		for (dgInt32 i = 0; i < count; i ++) {
			dgJacobianMatrixElement* const row = &matrixRow[index];

			//dgVector JMinvJacobianLinearM0 (row->m_Jt.m_jacobianM0.m_linear.Scale3 (invMass0));
			//dgVector JMinvJacobianAngularM0 (invInertia0.UnrotateVector (row->m_Jt.m_jacobianM0.m_angular));
			//dgVector JMinvJacobianLinearM1 (row->m_Jt.m_jacobianM1.m_linear.Scale3 (invMass1));
			//dgVector JMinvJacobianAngularM1 (invInertia1.UnrotateVector (row->m_Jt.m_jacobianM1.m_angular));

			dgVector JMinvJacobianLinearM0 (row->m_Jt.m_jacobianM0.m_linear.CompProduct4 (invMass0));
			dgVector JMinvJacobianAngularM0 (invInertia0.RotateVector (row->m_Jt.m_jacobianM0.m_angular));
			dgVector JMinvJacobianLinearM1 (row->m_Jt.m_jacobianM1.m_linear.CompProduct4 (invMass1));
			dgVector JMinvJacobianAngularM1 (invInertia1.RotateVector (row->m_Jt.m_jacobianM1.m_angular));

			dgVector tmpDiag (JMinvJacobianLinearM0.CompProduct4(row->m_Jt.m_jacobianM0.m_linear) + JMinvJacobianAngularM0.CompProduct4(row->m_Jt.m_jacobianM0.m_angular) +
							  JMinvJacobianLinearM1.CompProduct4(row->m_Jt.m_jacobianM1.m_linear) + JMinvJacobianAngularM1.CompProduct4(row->m_Jt.m_jacobianM1.m_angular));

			dgVector tmpAccel (JMinvJacobianLinearM0.CompProduct4(accel0) + JMinvJacobianAngularM0.CompProduct4(alpha0) + JMinvJacobianLinearM1.CompProduct4(accel1) + JMinvJacobianAngularM1.CompProduct4(alpha1));

			dgFloat32 extenalAcceleration = -(tmpAccel.m_x + tmpAccel.m_y + tmpAccel.m_z);
			//row->m_extAccel = extenalAcceleration;
			row->m_deltaAccel = extenalAcceleration;
			row->m_coordenateAccel += extenalAcceleration;
			row->m_force = row->m_jointFeebackForce[0].m_force;
			row->m_maxImpact = dgFloat32 (0.0f);

			dgAssert (row->m_diagDamp >= dgFloat32(0.1f));
			dgAssert (row->m_diagDamp <= dgFloat32(100.0f));
			dgFloat32 stiffness = DG_PSD_DAMP_TOL * row->m_diagDamp;

			dgFloat32 diag = (tmpDiag.m_x + tmpDiag.m_y + tmpDiag.m_z);
			dgAssert (diag > dgFloat32 (0.0f));
			row->m_diagDamp = diag * stiffness;

			diag *= (dgFloat32(1.0f) + stiffness);
			//solverMemory.m_diagJMinvJt[index] = diag;
			row->m_invDJMinvJt = dgFloat32(1.0f) / diag;

			index ++;
		}
	}
}


void dgWorldDynamicUpdate::SolverInitInternalForcesParallel (dgParallelSolverSyncData* const syncData) const
{
	dgWorld* const world = (dgWorld*) this;
	dgInt32 threadCounts = world->GetThreadCount();	

	syncData->m_atomicIndex = 0;
	for (dgInt32 i = 0; i < threadCounts; i ++) {
		world->QueueJob (SolverInitInternalForcesParallelKernel, syncData, world);
	}
	world->SynchronizationBarrier();
	dgJacobian* const internalForces = &world->m_solverMemory.m_internalForces[0];
	internalForces[0].m_linear = dgVector (dgFloat32 (0.0f));
	internalForces[0].m_angular = dgVector (dgFloat32 (0.0f));
}


void dgWorldDynamicUpdate::SolverInitInternalForcesParallelKernel (void* const context, void* const worldContext, dgInt32 threadID)
{
	dgParallelSolverSyncData* const syncData = (dgParallelSolverSyncData*) context;
	dgWorld* const world = (dgWorld*) worldContext;
	const dgIsland* const island = syncData->m_island;
	dgJacobian* const internalForces = &world->m_solverMemory.m_internalForces[0];
	dgJacobianMatrixElement* const matrixRow = &world->m_solverMemory.m_memory[0];
	dgInt32* const atomicIndex = &syncData->m_atomicIndex; 
	dgJointInfo* const constraintArrayPtr = (dgJointInfo*) &world->m_jointsMemory[0];
	dgJointInfo* const constraintArray = &constraintArrayPtr[island->m_jointStart];

	dgVector zero(dgFloat32 (0.0f));
	for (dgInt32 i = dgAtomicExchangeAndAdd(atomicIndex, 1); i < syncData->m_jointCount;  i = dgAtomicExchangeAndAdd(atomicIndex, 1)) {
		dgJacobian y0;
		dgJacobian y1;

		y0.m_linear = zero;
		y0.m_angular = zero;
		y1.m_linear = zero;
		y1.m_angular = zero;

		dgInt32 jointIndex = i;
		dgJointInfo* const jointInfo = &constraintArray[jointIndex];

		dgInt32 first = jointInfo->m_autoPairstart;
		dgInt32 count = jointInfo->m_autoPaircount;
		for (dgInt32 j = 0; j < count; j ++) { 
			dgJacobianMatrixElement* const row = &matrixRow[j + first];

			//dgFloat32 val = row->m_force; 
			dgVector val (row->m_force); 
			dgAssert (dgCheckFloat(row->m_force));
			y0.m_linear += row->m_Jt.m_jacobianM0.m_linear.CompProduct4(val);
			y0.m_angular += row->m_Jt.m_jacobianM0.m_angular.CompProduct4(val);
			y1.m_linear += row->m_Jt.m_jacobianM1.m_linear.CompProduct4(val);
			y1.m_angular += row->m_Jt.m_jacobianM1.m_angular.CompProduct4(val);
		}

		dgInt32 m0 = jointInfo->m_m0;
		dgInt32 m1 = jointInfo->m_m1;
		dgAssert (m0 != m1);

		dgSpinLock(&syncData->m_lock, false);
			internalForces[m0].m_linear += y0.m_linear;
			internalForces[m0].m_angular += y0.m_angular;
			internalForces[m1].m_linear += y1.m_linear;
			internalForces[m1].m_angular += y1.m_angular;
		dgSpinUnlock(&syncData->m_lock);
	}
}



void dgWorldDynamicUpdate::CalculateJointsAccelParallelKernel (void* const context, void* const worldContext, dgInt32 threadID)
{
	dgParallelSolverSyncData* const syncData = (dgParallelSolverSyncData*) context;
	dgWorld* const world = (dgWorld*) worldContext;
	const dgIsland* const island = syncData->m_island;
	dgJointInfo* const constraintArrayPtr = (dgJointInfo*) &world->m_jointsMemory[0];
	dgJointInfo* const constraintArray = &constraintArrayPtr[island->m_jointStart];
	dgJacobianMatrixElement* const matrixRow = &world->m_solverMemory.m_memory[0];

//	dgFloat32 invTimestepSrc = dgFloat32 (1.0f) / syncData->m_timestep;
//	dgFloat32 invStep = (dgFloat32 (1.0f) / dgFloat32 (LINEAR_SOLVER_SUB_STEPS));
//	dgFloat32 timestep =  syncData->m_timestep * invStep;
//	dgFloat32 invTimestep = invTimestepSrc * dgFloat32 (LINEAR_SOLVER_SUB_STEPS);

	dgJointAccelerationDecriptor joindDesc;
	joindDesc.m_timeStep = syncData->m_invStepRK;
	joindDesc.m_invTimeStep = syncData->m_invTimestepRK;
	joindDesc.m_firstPassCoefFlag = syncData->m_firstPassCoef;

	dgInt32* const atomicIndex = &syncData->m_atomicIndex; 
	if (joindDesc.m_firstPassCoefFlag == dgFloat32 (0.0f)) {
		for (dgInt32 curJoint = dgAtomicExchangeAndAdd(atomicIndex, 1); curJoint < syncData->m_jointCount;  curJoint = dgAtomicExchangeAndAdd(atomicIndex, 1)) {
			joindDesc.m_rowsCount = constraintArray[curJoint].m_autoPaircount;
			joindDesc.m_rowMatrix = &matrixRow[constraintArray[curJoint].m_autoPairstart];
			constraintArray[curJoint].m_joint->JointAccelerations (&joindDesc);
		}
	} else {

		const dgIsland* const island = syncData->m_island;
		dgBodyInfo* const bodyArrayPtr = (dgBodyInfo*) &world->m_bodiesMemory[0]; 
		const dgBodyInfo* const bodyArray = &bodyArrayPtr[island->m_bodyStart];

		for (dgInt32 curJoint = dgAtomicExchangeAndAdd(atomicIndex, 1); curJoint < syncData->m_jointCount;  curJoint = dgAtomicExchangeAndAdd(atomicIndex, 1)) {
			dgInt32 m0 = constraintArray[curJoint].m_m0;
			dgInt32 m1 = constraintArray[curJoint].m_m1;
			const dgBody* const body0 = bodyArray[m0].m_body;
			const dgBody* const body1 = bodyArray[m1].m_body;
			if (!(body0->m_resting & body1->m_resting)) {
				joindDesc.m_rowsCount = constraintArray[curJoint].m_autoPaircount;
				joindDesc.m_rowMatrix = &matrixRow[constraintArray[curJoint].m_autoPairstart];
				constraintArray[curJoint].m_joint->JointAccelerations (&joindDesc);
			}
		}
	}
}



void dgWorldDynamicUpdate::CalculateJointsVelocParallelKernel (void* const context, void* const worldContext, dgInt32 threadID)
{
	dgParallelSolverSyncData* const syncData = (dgParallelSolverSyncData*) context;
	dgWorld* const world = (dgWorld*) worldContext;

	//const dgInt32* const bodyInfoIndexArray = syncData->m_bodyInfoMap;
	//dgBodyInfo* const bodyArray = (dgBodyInfo*) &world->m_bodiesMemory[0]; 
	const dgIsland* const island = syncData->m_island;
	dgBodyInfo* const bodyArrayPtr = (dgBodyInfo*) &world->m_bodiesMemory[0]; 
	dgBodyInfo* const bodyArray = &bodyArrayPtr[island->m_bodyStart];

	dgJacobian* const internalForces = &world->m_solverMemory.m_internalForces[0];
	//dgJacobian* const internalVeloc = &world->m_solverMemory.m_internalVeloc[0];
	//dgFloat32 invStep = (dgFloat32 (1.0f) / dgFloat32 (LINEAR_SOLVER_SUB_STEPS));
	//dgFloat32 timestep = syncData->m_timestep * invStep;
	//dgFloat32 invStep = syncData->m_invStepRK;

	dgVector speedFreeze2 (world->m_freezeSpeed2 * dgFloat32 (0.1f));
	dgVector freezeOmega2 (world->m_freezeOmega2 * dgFloat32 (0.1f));
	//dgVector forceActiveMask ((jointCount <= DG_SMALL_ISLAND_COUNT) ?  dgFloat32 (-1.0f): dgFloat32 (0.0f));

	dgVector timestep4 (syncData->m_timestepRK);

	dgInt32* const atomicIndex = &syncData->m_atomicIndex;
	for (dgInt32 i = dgAtomicExchangeAndAdd(atomicIndex, 1); i < syncData->m_bodyCount;  i = dgAtomicExchangeAndAdd(atomicIndex, 1)) {
		dgDynamicBody* const body = (dgDynamicBody*) bodyArray[i].m_body;
		dgAssert (body->m_index == i);

		//dgVector force (body->m_accel + internalForces[index].m_linear);
		//dgVector torque (body->m_alpha + internalForces[index].m_angular);
		dgVector force (internalForces[i].m_linear);
		dgVector torque (internalForces[i].m_angular);
		if (body->IsRTTIType (dgBody::m_dynamicBodyRTTI)) {
			force += body->m_accel;
			torque += body->m_alpha;
		}

		//dgVector accel (force.Scale4 (body->m_invMass.m_w));
		//dgVector alpha (body->m_invWorldInertiaMatrix.RotateVector (torque));
		//body->m_veloc += accel.CompProduct4(timestep4);
		//body->m_omega += alpha.CompProduct4(timestep4);
		dgVector velocStep ((force.Scale4 (body->m_invMass.m_w)).CompProduct4(timestep4));
		dgVector omegaStep ((body->m_invWorldInertiaMatrix.RotateVector (torque)).CompProduct4(timestep4));
		if (!body->m_resting) {
			body->m_veloc += velocStep;
			body->m_omega += omegaStep;
		} else {
			dgVector velocStep2 (velocStep.DotProduct4(velocStep));
			dgVector omegaStep2 (omegaStep.DotProduct4(omegaStep));
			dgVector test ((velocStep2 > speedFreeze2) | (omegaStep2 > speedFreeze2));
			if (test.GetSignMask()) {
				body->m_resting = false;
			}
		}
	}
}


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


void dgWorldDynamicUpdate::UpdateFeedbackForcesParallelKernel (void* const context, void* const worldContext, dgInt32 threadID)
{
	dgParallelSolverSyncData* const syncData = (dgParallelSolverSyncData*) context;
	dgWorld* const world = (dgWorld*) worldContext;
	const dgIsland* const island = syncData->m_island;
	dgJointInfo* const constraintArrayPtr = (dgJointInfo*) &world->m_jointsMemory[0];
	dgJointInfo* const constraintArray = &constraintArrayPtr[island->m_jointStart];
	dgJacobianMatrixElement* const matrixRow = &world->m_solverMemory.m_memory[0];

	dgInt32 hasJointFeeback = 0;
	dgInt32* const atomicIndex = &syncData->m_atomicIndex;
	for (dgInt32 curJoint = dgAtomicExchangeAndAdd(atomicIndex, 1); curJoint < syncData->m_jointCount;  curJoint = dgAtomicExchangeAndAdd(atomicIndex, 1)) {
		dgInt32 first = constraintArray[curJoint].m_autoPairstart;
		dgInt32 count = constraintArray[curJoint].m_autoPaircount;

		for (dgInt32 j = 0; j < count; j ++) { 
			dgJacobianMatrixElement* const row = &matrixRow[j + first];
			dgFloat32 val = row->m_force; 
			dgAssert (dgCheckFloat(val));
			row->m_jointFeebackForce[0].m_force = val;
			row->m_jointFeebackForce[0].m_impact = row->m_maxImpact * syncData->m_timestepRK;
		}
		hasJointFeeback |= (constraintArray[curJoint].m_joint->m_updaFeedbackCallback ? 1 : 0);
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
	dgVector zero(dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f));

	//dgFloat32 invTimestepSrc = dgFloat32 (1.0f) / syncData->m_timestep;
	dgFloat32 invTimestepSrc = syncData->m_invTimestep;

	dgVector invTime (invTimestepSrc);
	dgInt32* const atomicIndex = &syncData->m_atomicIndex;
	for (dgInt32 i = dgAtomicExchangeAndAdd(atomicIndex, 1); i < syncData->m_bodyCount; i = dgAtomicExchangeAndAdd(atomicIndex, 1)) {
		dgDynamicBody* const body = (dgDynamicBody*) bodyArray[i].m_body;

		// the initial velocity and angular velocity were stored in net force and net torque, for memory saving
		dgVector accel = (body->m_veloc - body->m_netForce).CompProduct4 (invTime);
		dgVector alpha = (body->m_omega - body->m_netTorque).CompProduct4 (invTime);

		if ((accel % accel) < maxAccNorm2) {
			accel = zero;
		}

		if ((alpha % alpha) < maxAccNorm2) {
			alpha = zero;
		}

		if (body->IsRTTIType (dgBody::m_dynamicBodyRTTI)) {
			body->m_accel = accel;
			body->m_alpha = alpha;
		}
		body->m_netForce = accel.Scale4 (body->m_mass[3]);

		alpha = body->m_matrix.UnrotateVector(alpha);
		body->m_netTorque = body->m_matrix.RotateVector (alpha.CompProduct4(body->m_mass));
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


void dgWorldDynamicUpdate::IntegrateInslandParallel(dgParallelSolverSyncData* const syncData) const
{
	dgWorld* const world = (dgWorld*) this;
//	dgWorldDynamicUpdate::IntegrateInslandParallelKernel (syncData, world, 0);
	world->IntegrateArray (syncData->m_island, DG_SOLVER_MAX_ERROR, syncData->m_timestep, 0); 
}

/*
void dgWorldDynamicUpdate::IntegrateInslandParallelKernel (void* const context, void* const worldContext, dgInt32 threadID) 
{
	dgParallelSolverSyncData* const syncData = (dgParallelSolverSyncData*) context;
	dgWorld* const world = (dgWorld*) worldContext;
	dgFloat32 timestep = syncData->m_timestep;
	dgInt32* const atomicIndex = &syncData->m_islandCountCounter; 
	const dgIsland* const islandArray = syncData->m_island;
	for (dgInt32 i = dgAtomicExchangeAndAdd(atomicIndex, 1); i < syncData->m_islandCount; i = dgAtomicExchangeAndAdd(atomicIndex, 1)) {
		const dgIsland* const island = &islandArray[i];
		world->IntegrateArray (island, DG_SOLVER_MAX_ERROR, timestep, threadID); 
	}
} 
*/

void dgWorldDynamicUpdate::CalculateForcesGameModeParallel (dgParallelSolverSyncData* const syncData) const
{
	dgWorld* const world = (dgWorld*) this;
	dgInt32 threadCounts = world->GetThreadCount();	

	dgInt32 maxPasses = syncData->m_maxPasses;
	syncData->m_firstPassCoef = dgFloat32 (0.0f);
	for (dgInt32 step = 0; step < maxPasses; step ++) {

		syncData->m_atomicIndex = 0;
		for (dgInt32 i = 0; i < threadCounts; i ++) {
			world->QueueJob (CalculateJointsAccelParallelKernel, syncData, world);
		}
		world->SynchronizationBarrier();
		syncData->m_firstPassCoef = dgFloat32 (1.0f);

		dgFloat32 accNorm = DG_SOLVER_MAX_ERROR * dgFloat32 (2.0f);
		for (dgInt32 passes = 0; (passes < DG_BASE_ITERATION_COUNT) && (accNorm > DG_SOLVER_MAX_ERROR); passes ++) {
			for (dgInt32 i = 0; i < threadCounts; i ++) {
				syncData->m_accelNorm[i] = dgVector (dgFloat32 (0.0f));
			}
			syncData->m_atomicIndex = 0;
			for (dgInt32 i = 0; i < threadCounts; i ++) {
				world->QueueJob (CalculateJointsForceParallelKernel, syncData, world);
			}
			world->SynchronizationBarrier();

			accNorm = dgFloat32 (0.0f);
			for (dgInt32 i = 0; i < threadCounts; i ++) {
				accNorm = dgMax (accNorm, syncData->m_accelNorm[i].m_x);
			}
		}

		syncData->m_atomicIndex = 1;
		if (syncData->m_timestep != dgFloat32 (0.0f)) {
			for (dgInt32 j = 0; j < threadCounts; j ++) {
				world->QueueJob (CalculateJointsVelocParallelKernel, syncData, world);
			}
		} else {
			for (dgInt32 j = 0; j < threadCounts; j ++) {
				world->QueueJob (CalculateJointsImpulseVelocParallelKernel, syncData, world);
			}
		}
		world->SynchronizationBarrier();

	}

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
	for (dgInt32 j = 0; j < threadCounts; j ++) {
		world->QueueJob (UpdateBodyVelocityParallelKernel, syncData, world);
	}
	world->SynchronizationBarrier();

	if (hasJointFeeback) {

		syncData->m_atomicIndex = 0;
		for (dgInt32 j = 0; j < threadCounts; j ++) {
			world->QueueJob (KinematicCallbackUpdateParallelKernel, syncData, world);
		}
		world->SynchronizationBarrier();
	}
}


void dgWorldDynamicUpdate::CalculateJointsForceParallelKernel (void* const context, void* const worldContext, dgInt32 threadID)
{
	dgParallelSolverSyncData* const syncData = (dgParallelSolverSyncData*) context;
	dgWorld* const world = (dgWorld*) worldContext;

	const dgIsland* const island = syncData->m_island;
	dgJacobianMatrixElement* const matrixRow = &world->m_solverMemory.m_memory[0];
	dgBodyInfo* const bodyArrayPtr = (dgBodyInfo*) &world->m_bodiesMemory[0]; 
	const dgBodyInfo* const bodyArray = &bodyArrayPtr[island->m_bodyStart];
	dgJointInfo* const constraintArrayPtr = (dgJointInfo*) &world->m_jointsMemory[0];
	dgJointInfo* const constraintArray = &constraintArrayPtr[island->m_jointStart];
	dgJacobian* const internalForces = &world->m_solverMemory.m_internalForces[0];

	dgInt32 pool [dgParalletSolverStackPoolSize];
	dgQueue<dgInt32> queue(pool, dgParalletSolverStackPoolSize);

	dgFloat32 cacheForce[DG_CONSTRAINT_MAX_ROWS + 4];
	cacheForce[0] = dgFloat32 (1.0f);
	cacheForce[1] = dgFloat32 (1.0f);
	cacheForce[2] = dgFloat32 (1.0f);
	cacheForce[3] = dgFloat32 (1.0f);
	dgFloat32* const normalForce = &cacheForce[4];

	dgInt32* const atomicIndex = &syncData->m_atomicIndex;
	
	dgVector accNorm (syncData->m_accelNorm[threadID]);
	for (dgInt32 jointIndex = dgAtomicExchangeAndAdd(atomicIndex, 1); jointIndex < syncData->m_jointCount;  jointIndex = dgAtomicExchangeAndAdd(atomicIndex, 1)) {

		dgInt32 m0 = constraintArray[jointIndex].m_m0;
		dgInt32 m1 = constraintArray[jointIndex].m_m1;

/*
		do {
			dgSpinLock(&syncData->m_lock, false);
			dgInt32 m0 = constraintArray[jointIndex].m_m0;
			dgInt32 m1 = constraintArray[jointIndex].m_m1;
			dgInt32 test0 = syncData->m_bodyLocks[m0];
			dgInt32 test1 = syncData->m_bodyLocks[m1];
			if (test0 | test1) {
				queue.Insert(jointIndex);
				for (jointIndex = dgAtomicExchangeAndAdd(atomicIndex, 1); jointIndex < syncData->m_jointCount;  jointIndex = dgAtomicExchangeAndAdd(atomicIndex, 1)) {
					m0 = constraintArray[jointIndex].m_m0;
					m1 = constraintArray[jointIndex].m_m1;
					test0 = syncData->m_bodyLocks[m0];
					test1 = syncData->m_bodyLocks[m1];
					if (test0 | test1) {
						queue.Insert(jointIndex);
					} else {
						break;
					}
				}
			}
			if (!(test0 | test1)) {
				syncData->m_bodyLocks[m0] = m0 > 0;
				syncData->m_bodyLocks[m1] = m1 > 0;
			} else {
				jointIndex = -1;
			}
			dgSpinUnlock(&syncData->m_lock);
			if (jointIndex != -1) {
*/
			const dgBody* const body0 = bodyArray[m0].m_body;
			const dgBody* const body1 = bodyArray[m1].m_body;

			if (!(body0->m_resting & body1->m_resting)) {
				dgInt32 index = constraintArray[jointIndex].m_autoPairstart;
				dgInt32 rowsCount = constraintArray[jointIndex].m_autoPaircount;

				dgVector linearM0 (internalForces[m0].m_linear);
				dgVector angularM0 (internalForces[m0].m_angular);
				dgVector linearM1 (internalForces[m1].m_linear);
				dgVector angularM1 (internalForces[m1].m_angular);

				const dgVector invMass0 (body0->m_invMass[3]);
				const dgMatrix& invInertia0 = body0->m_invWorldInertiaMatrix;

				const dgVector invMass1 (body1->m_invMass[3]);
				const dgMatrix& invInertia1 = body1->m_invWorldInertiaMatrix;

				for (dgInt32 k = 0; k < rowsCount; k ++) {
					dgJacobianMatrixElement* const row = &matrixRow[index];

					dgAssert (row->m_Jt.m_jacobianM0.m_linear.m_w == dgFloat32 (0.0f));
					dgAssert (row->m_Jt.m_jacobianM0.m_angular.m_w == dgFloat32 (0.0f));
					dgAssert (row->m_Jt.m_jacobianM1.m_linear.m_w == dgFloat32 (0.0f));
					dgAssert (row->m_Jt.m_jacobianM1.m_angular.m_w == dgFloat32 (0.0f));

					//dgVector JMinvJacobianLinearM0 (row->m_Jt.m_jacobianM0.m_linear.Scale3 (invMass0));
					//dgVector JMinvJacobianAngularM0 (invInertia0.UnrotateVector (row->m_Jt.m_jacobianM0.m_angular));
					//dgVector JMinvJacobianLinearM1 (row->m_Jt.m_jacobianM1.m_linear.Scale3 (invMass1));
					//dgVector JMinvJacobianAngularM1 (invInertia1.UnrotateVector (row->m_Jt.m_jacobianM1.m_angular));

					dgVector JMinvJacobianLinearM0 (row->m_Jt.m_jacobianM0.m_linear.CompProduct4 (invMass0));
					dgVector JMinvJacobianAngularM0 (invInertia0.RotateVector (row->m_Jt.m_jacobianM0.m_angular));
					dgVector JMinvJacobianLinearM1 (row->m_Jt.m_jacobianM1.m_linear.CompProduct4 (invMass1));
					dgVector JMinvJacobianAngularM1 (invInertia1.RotateVector (row->m_Jt.m_jacobianM1.m_angular));

					dgVector a (JMinvJacobianLinearM0.CompProduct4(linearM0) + JMinvJacobianAngularM0.CompProduct4(angularM0) + JMinvJacobianLinearM1.CompProduct4(linearM1) + JMinvJacobianAngularM1.CompProduct4(angularM1));

					//dgFloat32 a = row->m_coordenateAccel - acc.m_x - acc.m_y - acc.m_z - row->m_force * row->m_diagDamp;
					a = dgVector (row->m_coordenateAccel  - row->m_force * row->m_diagDamp) - a.AddHorizontal();

					//dgFloat32 f = row->m_force + row->m_invDJMinvJt * a;
					dgVector f (row->m_force + row->m_invDJMinvJt * a.m_x);

					dgInt32 frictionIndex = row->m_normalForceIndex;
					dgAssert (((frictionIndex < 0) && (normalForce[frictionIndex] == dgFloat32 (1.0f))) || ((frictionIndex >= 0) && (normalForce[frictionIndex] >= dgFloat32 (0.0f))));

					dgFloat32 frictionNormal = normalForce[frictionIndex];
					dgVector lowerFrictionForce = (frictionNormal * row->m_lowerBoundFrictionCoefficent);
					dgVector upperFrictionForce = (frictionNormal * row->m_upperBoundFrictionCoefficent);

					//if (f > upperFrictionForce) {
					//	a = dgFloat32 (0.0f);
					//	f = upperFrictionForce;
					//} else if (f < lowerFrictionForce) {
					//	a = dgFloat32 (0.0f);
					//	f = lowerFrictionForce;
					//}
					a = a.AndNot((f > upperFrictionForce) | (f < lowerFrictionForce));
					f = f.GetMax(lowerFrictionForce).GetMin(upperFrictionForce);

					accNorm = accNorm.GetMax(a.Abs());
					dgAssert (accNorm.m_x >= dgAbsf (a.m_x));

					// no early out for parallel solver
					//accNorm = accNorm.GetMax(a.Abs());
					//dgAssert (accNorm.m_x >= dgAbsf (a.m_x));

					//dgFloat32 prevValue = f - row->m_force;
					dgVector prevValue (f - dgVector (row->m_force));

					row->m_force = f.m_x;
					normalForce[k] = f.m_x;
					row->m_maxImpact = f.Abs().GetMax (row->m_maxImpact).m_x;

					linearM0 += row->m_Jt.m_jacobianM0.m_linear.CompProduct4 (prevValue);
					angularM0 += row->m_Jt.m_jacobianM0.m_angular.CompProduct4 (prevValue);
					linearM1 += row->m_Jt.m_jacobianM1.m_linear.CompProduct4 (prevValue);
					angularM1 += row->m_Jt.m_jacobianM1.m_angular.CompProduct4 (prevValue);
					index ++;
				}

				internalForces[m0].m_linear = linearM0;
				internalForces[m0].m_angular = angularM0;

				internalForces[m1].m_linear = linearM1;
				internalForces[m1].m_angular = angularM1;
			}

/*
			}
			dgSpinLock(&syncData->m_lock, false);
			if (jointIndex != -1) {
				syncData->m_bodyLocks[m0] = 0;
				syncData->m_bodyLocks[m1] = 0;
			}
			jointIndex = !queue.IsEmpty() ? queue.Remove() : -1;
			dgSpinUnlock(&syncData->m_lock);
		} while (jointIndex != -1);
*/		 
	}
	syncData->m_accelNorm[threadID] = accNorm;
}
