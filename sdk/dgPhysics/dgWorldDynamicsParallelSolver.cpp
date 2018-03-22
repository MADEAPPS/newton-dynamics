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


void dgWorldDynamicUpdate::CalculateReactionForcesParallel(const dgBodyCluster* const cluster, dgFloat32 timestep) const
{
	dgParallelSolverSyncData syncData;

	dgWorld* const world = (dgWorld*) this;

	syncData.m_bodyLocks = dgAlloca (dgInt32, cluster->m_bodyCount + 1024);
//	syncData.m_jointConflicts = dgAlloca (dgParallelSolverSyncData::dgParallelJointMap, cluster->m_jointCount + 1024);
	memset (syncData.m_bodyLocks, 0, cluster->m_bodyCount * sizeof (dgInt32));

	dgInt32 bodyCount = cluster->m_bodyCount;
	dgInt32 jointsCount = cluster->m_jointCount;
//	dgJointInfo* const constraintArrayPtr = (dgJointInfo*) &world->m_jointsMemory[0];
//	dgJointInfo* const constraintArray = &constraintArrayPtr[cluster->m_jointStart];
//	LinearizeJointParallelArray (&syncData, constraintArray, cluster);
	
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
	syncData.m_cluster = cluster;

	InitilizeBodyArrayParallel (&syncData);
	BuildJacobianMatrixParallel (&syncData);
	SolverInitInternalForcesParallel (&syncData);
	CalculateForcesParallel (&syncData);
	IntegrateClusterParallel(&syncData); 
}

/*
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


void dgWorldDynamicUpdate::LinearizeJointParallelArray(dgParallelSolverSyncData* const solverSyncData, dgJointInfo* const constraintArray, const dgBodyCluster* const cluster) const
{
	dgParallelSolverSyncData::dgParallelJointMap* const jointInfoMap = solverSyncData->m_jointConflicts;
	dgInt32 count = cluster->m_jointCount;
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
*/

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

	const dgBodyCluster* const cluster = syncData->m_cluster;
	dgBodyInfo* const bodyArrayPtr = (dgBodyInfo*) &world->m_bodiesMemory[0]; 
	dgBodyInfo* const bodyArray = &bodyArrayPtr[cluster->m_bodyStart];
	dgJacobian* const internalForces = &world->m_solverMemory.m_internalForcesBuffer[0];

	if (syncData->m_timestep != dgFloat32 (0.0f)) {
		dgAssert(bodyArray[0].m_body->IsRTTIType(dgBody::m_dynamicBodyRTTI) || (((dgDynamicBody*)bodyArray[0].m_body)->m_accel.DotProduct3(((dgDynamicBody*)bodyArray[0].m_body)->m_accel)) == dgFloat32(0.0f));
		for (dgInt32 i = dgAtomicExchangeAndAdd(atomicIndex, 1); i < syncData->m_bodyCount; i = dgAtomicExchangeAndAdd(atomicIndex, 1)) {
			dgDynamicBody* const body = (dgDynamicBody*)bodyArray[i].m_body;
			
			dgAssert(body->IsRTTIType(dgBody::m_dynamicBodyRTTI) || body->IsRTTIType(dgBody::m_kinematicBodyRTTI));
			if (!body->m_equilibrium) {
				dgAssert(body->m_invMass.m_w > dgFloat32(0.0f));
				body->AddDampingAcceleration(syncData->m_timestep);
				body->CalcInvInertiaMatrix();
			}

			// re use these variables for temp storage 
			body->m_accel = body->m_veloc;
			body->m_alpha = body->m_omega;
			internalForces[i].m_linear = dgVector::m_zero;
			internalForces[i].m_angular = dgVector::m_zero;
		}
	} else {
		dgAssert(bodyArray[0].m_body->IsRTTIType(dgBody::m_dynamicBodyRTTI) || (((dgDynamicBody*)bodyArray[0].m_body)->m_accel.DotProduct3(((dgDynamicBody*)bodyArray[0].m_body)->m_accel)) == dgFloat32(0.0f));
		for (dgInt32 i = dgAtomicExchangeAndAdd(atomicIndex, 1); i < syncData->m_bodyCount; i = dgAtomicExchangeAndAdd(atomicIndex, 1)) {
			dgBody* const body = bodyArray[i].m_body;
			dgAssert(body->IsRTTIType(dgBody::m_dynamicBodyRTTI) || body->IsRTTIType(dgBody::m_kinematicBodyRTTI));
			if (!body->m_equilibrium) {
				dgAssert(body->m_invMass.m_w > dgFloat32(0.0f));
				body->CalcInvInertiaMatrix();
			}

			// re use these variables for temp storage 
			body->m_accel = body->m_veloc;
			body->m_alpha = body->m_omega;

			internalForces[i].m_linear = dgVector::m_zero;
			internalForces[i].m_angular = dgVector::m_zero;
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
	const dgBodyCluster* const cluster = syncData->m_cluster;
	dgBodyInfo* const bodyArrayPtr = (dgBodyInfo*) &world->m_bodiesMemory[0]; 
	dgBodyInfo* const bodyArray = &bodyArrayPtr[cluster->m_bodyStart];
	dgJointInfo* const constraintArrayPtr = (dgJointInfo*) &world->m_jointsMemory[0];
	dgJointInfo* const constraintArray = &constraintArrayPtr[cluster->m_jointStart];
	dgJacobianMatrixElement* const matrixRow = &world->m_solverMemory.m_jacobianBuffer[0];
	dgJacobian* const internalForces = &world->m_solverMemory.m_internalForcesBuffer[0];
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
		dgAssert (jointInfo->m_m0 < cluster->m_bodyCount);
		dgAssert (jointInfo->m_m1 < cluster->m_bodyCount);
		world->BuildJacobianMatrix (bodyArray, jointInfo, internalForces, matrixRow, forceOrImpulseScale);
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
	const dgBodyCluster* const cluster = syncData->m_cluster;
	dgJacobian* const internalForces = &world->m_solverMemory.m_internalForcesBuffer[0];
	dgJacobianMatrixElement* const matrixRow = &world->m_solverMemory.m_jacobianBuffer[0];
	dgInt32* const atomicIndex = &syncData->m_atomicIndex; 
	dgBodyInfo* const bodyInfoArrayPtr = (dgBodyInfo*)&world->m_bodiesMemory[0];
	dgBodyInfo* const bodyInfoArray = &bodyInfoArrayPtr[cluster->m_bodyStart];
	dgJointInfo* const constraintArrayPtr = (dgJointInfo*) &world->m_jointsMemory[0];
	dgJointInfo* const constraintArray = &constraintArrayPtr[cluster->m_jointStart];
	dgInt32* const bodyLocks = syncData->m_bodyLocks;


dgFloat32 forceImpulseScale = dgFloat32(1.0f);

	for (dgInt32 j = dgAtomicExchangeAndAdd(atomicIndex, 1); j < syncData->m_jointCount;  j = dgAtomicExchangeAndAdd(atomicIndex, 1)) {
		dgJointInfo* const jointInfo = &constraintArray[j];
		const dgInt32 index = jointInfo->m_pairStart;
		const dgInt32 count = jointInfo->m_pairCount;
		const dgInt32 m0 = jointInfo->m_m0;
		const dgInt32 m1 = jointInfo->m_m1;
		dgAssert (m0 != m1);

		const dgBody* const body0 = bodyInfoArray[m0].m_body;
		const dgBody* const body1 = bodyInfoArray[m1].m_body;
		const bool isBilateral = jointInfo->m_joint->IsBilateral();

		const dgVector invMass0(body0->m_invMass[3]);
		const dgMatrix& invInertia0 = body0->m_invWorldInertiaMatrix;
		const dgVector invMass1(body1->m_invMass[3]);
		const dgMatrix& invInertia1 = body1->m_invWorldInertiaMatrix;

		dgVector force0(dgVector::m_zero);
		dgVector torque0(dgVector::m_zero);
		if (body0->IsRTTIType(dgBody::m_dynamicBodyRTTI)) {
			force0 = ((dgDynamicBody*)body0)->m_externalForce;
			torque0 = ((dgDynamicBody*)body0)->m_externalTorque;
		}

		dgVector force1(dgVector::m_zero);
		dgVector torque1(dgVector::m_zero);
		if (body1->IsRTTIType(dgBody::m_dynamicBodyRTTI)) {
			force1 = ((dgDynamicBody*)body1)->m_externalForce;
			torque1 = ((dgDynamicBody*)body1)->m_externalTorque;
		}

		jointInfo->m_scale0 = dgFloat32(1.0f);
		jointInfo->m_scale1 = dgFloat32(1.0f);
		if ((invMass0.GetScalar() > dgFloat32(0.0f)) && (invMass1.GetScalar() > dgFloat32(0.0f)) && !(body0->GetSkeleton() && body1->GetSkeleton())) {
			const dgFloat32 mass0 = body0->GetMass().m_w;
			const dgFloat32 mass1 = body1->GetMass().m_w;
			if (mass0 > (DG_HEAVY_MASS_SCALE_FACTOR * mass1)) {
				jointInfo->m_scale0 = invMass1.GetScalar() * mass0 * DG_HEAVY_MASS_INV_SCALE_FACTOR;
			} else if (mass1 > (DG_HEAVY_MASS_SCALE_FACTOR * mass0)) {
				jointInfo->m_scale1 = invMass0.GetScalar() * mass1 * DG_HEAVY_MASS_INV_SCALE_FACTOR;
			}
		}

		dgJacobian forceAcc0;
		dgJacobian forceAcc1;
		const dgVector scale0(jointInfo->m_scale0);
		const dgVector scale1(jointInfo->m_scale1);
		forceAcc0.m_linear = dgVector::m_zero;
		forceAcc0.m_angular = dgVector::m_zero;
		forceAcc1.m_linear = dgVector::m_zero;
		forceAcc1.m_angular = dgVector::m_zero;

		for (dgInt32 i = 0; i < count; i++) {
			dgJacobianMatrixElement* const row = &matrixRow[index + i];
			dgAssert(row->m_Jt.m_jacobianM0.m_linear.m_w == dgFloat32(0.0f));
			dgAssert(row->m_Jt.m_jacobianM0.m_angular.m_w == dgFloat32(0.0f));
			dgAssert(row->m_Jt.m_jacobianM1.m_linear.m_w == dgFloat32(0.0f));
			dgAssert(row->m_Jt.m_jacobianM1.m_angular.m_w == dgFloat32(0.0f));

			row->m_JMinv.m_jacobianM0.m_linear = row->m_Jt.m_jacobianM0.m_linear * invMass0;
			row->m_JMinv.m_jacobianM0.m_angular = invInertia0.RotateVector(row->m_Jt.m_jacobianM0.m_angular);
			row->m_JMinv.m_jacobianM1.m_linear = row->m_Jt.m_jacobianM1.m_linear * invMass1;
			row->m_JMinv.m_jacobianM1.m_angular = invInertia1.RotateVector(row->m_Jt.m_jacobianM1.m_angular);

			dgVector tmpAccel(row->m_JMinv.m_jacobianM0.m_linear * force0 + row->m_JMinv.m_jacobianM0.m_angular * torque0 +
							  row->m_JMinv.m_jacobianM1.m_linear * force1 + row->m_JMinv.m_jacobianM1.m_angular * torque1);

			dgAssert(tmpAccel.m_w == dgFloat32(0.0f));
			dgFloat32 extenalAcceleration = -(tmpAccel.AddHorizontal()).GetScalar();
			row->m_deltaAccel = extenalAcceleration * forceImpulseScale;
			row->m_coordenateAccel += extenalAcceleration * forceImpulseScale;
			dgAssert(row->m_jointFeebackForce);
			const dgFloat32 force = row->m_jointFeebackForce->m_force * forceImpulseScale;
			row->m_force = isBilateral ? dgClamp(force, row->m_lowerBoundFrictionCoefficent, row->m_upperBoundFrictionCoefficent) : force;
			//row->m_force = 0.0f;
			row->m_maxImpact = dgFloat32(0.0f);

			dgVector jMinvM0linear(scale0 * row->m_JMinv.m_jacobianM0.m_linear);
			dgVector jMinvM0angular(scale0 * row->m_JMinv.m_jacobianM0.m_angular);
			dgVector jMinvM1linear(scale1 * row->m_JMinv.m_jacobianM1.m_linear);
			dgVector jMinvM1angular(scale1 * row->m_JMinv.m_jacobianM1.m_angular);

			dgVector tmpDiag(jMinvM0linear * row->m_Jt.m_jacobianM0.m_linear + jMinvM0angular * row->m_Jt.m_jacobianM0.m_angular +
							 jMinvM1linear * row->m_Jt.m_jacobianM1.m_linear + jMinvM1angular * row->m_Jt.m_jacobianM1.m_angular);

			dgAssert(tmpDiag.m_w == dgFloat32(0.0f));
			dgFloat32 diag = (tmpDiag.AddHorizontal()).GetScalar();
			dgAssert(diag > dgFloat32(0.0f));
			row->m_diagDamp = diag * row->m_stiffness;
			diag *= (dgFloat32(1.0f) + row->m_stiffness);
			row->m_jinvMJt = diag;
			row->m_invJinvMJt = dgFloat32(1.0f) / diag;

			dgAssert(dgCheckFloat(row->m_force));
			dgVector val(row->m_force);
			forceAcc0.m_linear += row->m_Jt.m_jacobianM0.m_linear * val;
			forceAcc0.m_angular += row->m_Jt.m_jacobianM0.m_angular * val;
			forceAcc1.m_linear += row->m_Jt.m_jacobianM1.m_linear * val;
			forceAcc1.m_angular += row->m_Jt.m_jacobianM1.m_angular * val;
		}

		forceAcc0.m_linear = forceAcc0.m_linear * scale0;
		forceAcc0.m_angular = forceAcc0.m_angular * scale0;
		forceAcc1.m_linear = forceAcc1.m_linear * scale1;
		forceAcc1.m_angular = forceAcc1.m_angular * scale1;

		if (m0) {
			dgScopeSpinLock lock(&bodyLocks[m0]);
			internalForces[m0].m_linear += forceAcc0.m_linear;
			internalForces[m0].m_angular += forceAcc0.m_angular;
		}

		if (m1) {
			dgScopeSpinLock lock(&bodyLocks[m1]);
			internalForces[m1].m_linear += forceAcc1.m_linear;
			internalForces[m1].m_angular += forceAcc1.m_angular;
		}
	}
}


void dgWorldDynamicUpdate::CalculateJointsAccelParallelKernel (void* const context, void* const worldContext, dgInt32 threadID)
{
	dgParallelSolverSyncData* const syncData = (dgParallelSolverSyncData*) context;
	dgWorld* const world = (dgWorld*) worldContext;
	const dgBodyCluster* const cluster = syncData->m_cluster;
	dgJointInfo* const constraintArrayPtr = (dgJointInfo*) &world->m_jointsMemory[0];
	dgJointInfo* const constraintArray = &constraintArrayPtr[cluster->m_jointStart];
	dgJacobianMatrixElement* const matrixRow = &world->m_solverMemory.m_jacobianBuffer[0];

	dgJointAccelerationDecriptor joindDesc;
	joindDesc.m_timeStep = syncData->m_timestepRK;
	joindDesc.m_invTimeStep = syncData->m_invTimestepRK;
	joindDesc.m_firstPassCoefFlag = syncData->m_firstPassCoef;

	dgInt32* const atomicIndex = &syncData->m_atomicIndex; 
	for (dgInt32 i = dgAtomicExchangeAndAdd(atomicIndex, 1); i < syncData->m_jointCount; i = dgAtomicExchangeAndAdd(atomicIndex, 1)) {
		dgJointInfo* const jointInfo = &constraintArray[i];
		dgConstraint* const constraint = jointInfo->m_joint;
		joindDesc.m_rowsCount = jointInfo->m_pairCount;
		joindDesc.m_rowMatrix = &matrixRow[jointInfo->m_pairStart];
		constraint->JointAccelerations(&joindDesc);
	}
}

void dgWorldDynamicUpdate::KinematicCallbackUpdateParallelKernel (void* const context, void* const worldContext, dgInt32 threadID)
{
	dgParallelSolverSyncData* const syncData = (dgParallelSolverSyncData*) context;
	dgWorld* const world = (dgWorld*) worldContext;
	const dgBodyCluster* const cluster = syncData->m_cluster;
	dgJointInfo* const constraintArrayPtr = (dgJointInfo*) &world->m_jointsMemory[0];
	dgJointInfo* const constraintArray = &constraintArrayPtr[cluster->m_jointStart];

	dgInt32* const atomicIndex = &syncData->m_atomicIndex;
	for (dgInt32 i = dgAtomicExchangeAndAdd(atomicIndex, 1); i < syncData->m_jointCount;  i = dgAtomicExchangeAndAdd(atomicIndex, 1)) {
		dgInt32 curJoint = i;
		if (constraintArray[curJoint].m_joint->m_updaFeedbackCallback) {
			constraintArray[curJoint].m_joint->m_updaFeedbackCallback (*constraintArray[curJoint].m_joint, syncData->m_timestep, threadID);
		}
	}
}


void dgWorldDynamicUpdate::IntegrateClusterParallel(dgParallelSolverSyncData* const syncData) const
{
	dgWorld* const world = (dgWorld*) this;
	world->IntegrateVelocity (syncData->m_cluster, DG_SOLVER_MAX_ERROR, syncData->m_timestep, 0); 
}

void dgWorldDynamicUpdate::CalculateJointsForceParallelKernel (void* const context, void* const worldContext, dgInt32 threadID)
{
	dgParallelSolverSyncData* const syncData = (dgParallelSolverSyncData*) context;
	dgWorld* const world = (dgWorld*) worldContext;

	const dgBodyCluster* const cluster = syncData->m_cluster;
	dgJacobianMatrixElement* const matrixRow = &world->m_solverMemory.m_jacobianBuffer[0];
	dgBodyInfo* const bodyArrayPtr = (dgBodyInfo*) &world->m_bodiesMemory[0]; 
	const dgBodyInfo* const bodyArray = &bodyArrayPtr[cluster->m_bodyStart];
	dgJointInfo* const constraintArrayPtr = (dgJointInfo*) &world->m_jointsMemory[0];
	dgJointInfo* const constraintArray = &constraintArrayPtr[cluster->m_jointStart];
	dgJacobian* const internalForces = &world->m_solverMemory.m_internalForcesBuffer[0];
	const int jointCount = syncData->m_jointCount;

	dgFloat32 accNorm = dgFloat32(0.0f);
	dgInt32* const atomicIndex = &syncData->m_atomicIndex;
	for (dgInt32 i = dgAtomicExchangeAndAdd(atomicIndex, 1); i < jointCount; i = dgAtomicExchangeAndAdd(atomicIndex, 1)) {
		dgJointInfo* const jointInfo = &constraintArray[i];
		dgFloat32 accel2 = world->CalculateJointForce(jointInfo, bodyArray, internalForces, matrixRow, false);
		accNorm += accel2;
	}
	syncData->m_accelNorm[threadID] = accNorm;
}


void dgWorldDynamicUpdate::CalculateBodiesForceParallelKernel(void* const context, void* const worldContext, dgInt32 threadID)
{
	dgParallelSolverSyncData* const syncData = (dgParallelSolverSyncData*)context;
	dgWorld* const world = (dgWorld*)worldContext;

	const dgBodyCluster* const cluster = syncData->m_cluster;
	dgJacobianMatrixElement* const matrixRow = &world->m_solverMemory.m_jacobianBuffer[0];
	dgJointInfo* const constraintArrayPtr = (dgJointInfo*)&world->m_jointsMemory[0];
	dgJointInfo* const constraintArray = &constraintArrayPtr[cluster->m_jointStart];
	dgJacobian* const internalForces = &world->m_solverMemory.m_internalForcesBuffer[0];
	const int jointCount = syncData->m_jointCount;
	dgInt32* const bodyLocks = syncData->m_bodyLocks;

	dgInt32* const atomicIndex = &syncData->m_atomicIndex;
	for (dgInt32 j = dgAtomicExchangeAndAdd(atomicIndex, 1); j < jointCount; j = dgAtomicExchangeAndAdd(atomicIndex, 1)) {
		dgJacobian forceAcc0;
		dgJacobian forceAcc1;
		const dgJointInfo* const jointInfo = &constraintArray[j];
		const dgInt32 index = jointInfo->m_pairStart;
		const dgInt32 count = jointInfo->m_pairCount;
		const dgInt32 m0 = jointInfo->m_m0;
		const dgInt32 m1 = jointInfo->m_m1;

		forceAcc0.m_linear = dgVector::m_zero;
		forceAcc0.m_angular = dgVector::m_zero;
		forceAcc1.m_linear = dgVector::m_zero;
		forceAcc1.m_angular = dgVector::m_zero;
		for (dgInt32 i = 0; i < count; i++) {
			const dgJacobianMatrixElement* const row = &matrixRow[index + i];
			dgAssert(row->m_Jt.m_jacobianM0.m_linear.m_w == dgFloat32(0.0f));
			dgAssert(row->m_Jt.m_jacobianM0.m_angular.m_w == dgFloat32(0.0f));
			dgAssert(row->m_Jt.m_jacobianM1.m_linear.m_w == dgFloat32(0.0f));
			dgAssert(row->m_Jt.m_jacobianM1.m_angular.m_w == dgFloat32(0.0f));

			dgAssert(dgCheckFloat(row->m_force));
			dgVector val(row->m_force);
			forceAcc0.m_linear += row->m_Jt.m_jacobianM0.m_linear * val;
			forceAcc0.m_angular += row->m_Jt.m_jacobianM0.m_angular * val;
			forceAcc1.m_linear += row->m_Jt.m_jacobianM1.m_linear * val;
			forceAcc1.m_angular += row->m_Jt.m_jacobianM1.m_angular * val;
		}

		const dgVector scale0(jointInfo->m_scale0);
		const dgVector scale1(jointInfo->m_scale1);
		forceAcc0.m_linear = forceAcc0.m_linear * scale0;
		forceAcc0.m_angular = forceAcc0.m_angular * scale0;
		forceAcc1.m_linear = forceAcc1.m_linear * scale1;
		forceAcc1.m_angular = forceAcc1.m_angular * scale1;

		if (m0) {
			dgScopeSpinLock lock(&bodyLocks[m0]);
			internalForces[m0].m_linear += forceAcc0.m_linear;
			internalForces[m0].m_angular += forceAcc0.m_angular;
		}

		if (m1) {
			dgScopeSpinLock lock(&bodyLocks[m1]);
			internalForces[m1].m_linear += forceAcc1.m_linear;
			internalForces[m1].m_angular += forceAcc1.m_angular;
		}
	}
}


void dgWorldDynamicUpdate::CalculateJointsVelocParallelKernel(void* const context, void* const worldContext, dgInt32 threadID)
{
	dgWorld* const world = (dgWorld*)worldContext;
	dgParallelSolverSyncData* const syncData = (dgParallelSolverSyncData*)context;

	const dgBodyCluster* const cluster = syncData->m_cluster;
	dgBodyInfo* const bodyArrayPtr = (dgBodyInfo*)&world->m_bodiesMemory[0];
	dgBodyInfo* const bodyArray = &bodyArrayPtr[cluster->m_bodyStart];

	dgJacobian* const internalForces = &world->m_solverMemory.m_internalForcesBuffer[0];

	dgVector speedFreeze2(world->m_freezeSpeed2 * dgFloat32(0.1f));
	dgVector freezeOmega2(world->m_freezeOmega2 * dgFloat32(0.1f));
	dgInt32* const atomicIndex = &syncData->m_atomicIndex;

	dgVector timestep4(syncData->m_timestepRK);
	for (dgInt32 i = dgAtomicExchangeAndAdd(atomicIndex, 1); i < syncData->m_bodyCount; i = dgAtomicExchangeAndAdd(atomicIndex, 1)) {
		dgDynamicBody* const body = (dgDynamicBody*)bodyArray[i].m_body;
		dgAssert(body->m_index == i);
		if (body->IsRTTIType(dgBody::m_dynamicBodyRTTI)) {
			const dgJacobian& forceAndTorque = internalForces[i];
			const dgVector force(body->m_externalForce + forceAndTorque.m_linear);
			const dgVector torque(body->m_externalTorque + forceAndTorque.m_angular);

			const dgVector velocStep((force.Scale4(body->m_invMass.m_w)) * timestep4);
			const dgVector omegaStep((body->m_invWorldInertiaMatrix.RotateVector(torque)) * timestep4);

			if (!body->m_resting) {
				body->m_veloc += velocStep;
				body->m_omega += omegaStep;
			} else {
				const dgVector velocStep2(velocStep.DotProduct4(velocStep));
				const dgVector omegaStep2(omegaStep.DotProduct4(omegaStep));
				const dgVector test(((velocStep2 > speedFreeze2) | (omegaStep2 > speedFreeze2)) & dgVector::m_negOne);
				const dgInt32 equilibrium = test.GetSignMask() ? 0 : 1;
				body->m_resting &= equilibrium;
			}

			dgAssert(body->m_veloc.m_w == dgFloat32(0.0f));
			dgAssert(body->m_omega.m_w == dgFloat32(0.0f));
		}
	}
}


void dgWorldDynamicUpdate::UpdateFeedbackForcesParallelKernel(void* const context, void* const worldContext, dgInt32 threadID)
{
	dgWorld* const world = (dgWorld*)worldContext;
	dgParallelSolverSyncData* const syncData = (dgParallelSolverSyncData*)context;
	const dgBodyCluster* const cluster = syncData->m_cluster;
	dgJointInfo* const constraintArrayPtr = (dgJointInfo*)&world->m_jointsMemory[0];
	dgJointInfo* const constraintArray = &constraintArrayPtr[cluster->m_jointStart];
	dgJacobianMatrixElement* const matrixRow = &world->m_solverMemory.m_jacobianBuffer[0];

	dgInt32 hasJointFeeback = 0;
	dgInt32* const atomicIndex = &syncData->m_atomicIndex;

	for (dgInt32 curJoint = dgAtomicExchangeAndAdd(atomicIndex, 1); curJoint < syncData->m_jointCount; curJoint = dgAtomicExchangeAndAdd(atomicIndex, 1)) {
		dgJointInfo* const jointInfo = &constraintArray[curJoint];
		dgConstraint* const constraint = jointInfo->m_joint;
		const dgInt32 first = jointInfo->m_pairStart;
		const dgInt32 count = jointInfo->m_pairCount;

		for (dgInt32 j = 0; j < count; j++) {
			dgJacobianMatrixElement* const row = &matrixRow[j + first];
			dgAssert(dgCheckFloat(row->m_force));
			row->m_jointFeebackForce->m_force = row->m_force;
			row->m_jointFeebackForce->m_impact = row->m_maxImpact * syncData->m_timestepRK;
		}
		hasJointFeeback |= (constraint->m_updaFeedbackCallback ? 1 : 0);
	}
	syncData->m_hasJointFeeback[threadID] = hasJointFeeback;
}


void dgWorldDynamicUpdate::UpdateBodyVelocityParallelKernel(void* const context, void* const worldContext, dgInt32 threadID)
{
	dgWorld* const world = (dgWorld*)worldContext;
	dgParallelSolverSyncData* const syncData = (dgParallelSolverSyncData*)context;

	const dgBodyCluster* const cluster = syncData->m_cluster;
	dgBodyInfo* const bodyArrayPtr = (dgBodyInfo*)&world->m_bodiesMemory[0];
	dgBodyInfo* const bodyArray = &bodyArrayPtr[cluster->m_bodyStart];

	dgFloat32 maxAccNorm2 = DG_SOLVER_MAX_ERROR * DG_SOLVER_MAX_ERROR;
	dgFloat32 invTimestepSrc = syncData->m_invTimestep;

	dgVector invTime(invTimestepSrc);
	dgInt32* const atomicIndex = &syncData->m_atomicIndex;
	for (dgInt32 i = dgAtomicExchangeAndAdd(atomicIndex, 1); i < syncData->m_bodyCount; i = dgAtomicExchangeAndAdd(atomicIndex, 1)) {
		dgDynamicBody* const body = (dgDynamicBody*)bodyArray[i].m_body;
		world->CalculateNetAcceleration(body, invTime, maxAccNorm2);
	}
}

void dgWorldDynamicUpdate::CalculateForcesParallel(dgParallelSolverSyncData* const syncData) const
{
	dgWorld* const world = (dgWorld*) this;
	const dgInt32 threadCounts = world->GetThreadCount();

	const dgInt32 passes = syncData->m_passes;
	const dgInt32 maxPasses = syncData->m_maxPasses;
	syncData->m_firstPassCoef = dgFloat32(0.0f);

	const dgInt32 bodyCount = syncData->m_bodyCount;
	dgJacobian* const internalForces = &world->m_solverMemory.m_internalForcesBuffer[0];

	for (dgInt32 step = 0; step < maxPasses; step++) {

		syncData->m_atomicIndex = 0;
		for (dgInt32 i = 0; i < threadCounts; i++) {
			world->QueueJob(CalculateJointsAccelParallelKernel, syncData, world);
		}
		world->SynchronizationBarrier();
		syncData->m_firstPassCoef = dgFloat32(1.0f);

		dgFloat32 accNorm = DG_SOLVER_MAX_ERROR * dgFloat32(2.0f);
		for (dgInt32 k = 0; (k < passes) && (accNorm > DG_SOLVER_MAX_ERROR); k++) {

			syncData->m_atomicIndex = 0;
			for (dgInt32 i = 0; i < threadCounts; i++) {
				world->QueueJob(CalculateJointsForceParallelKernel, syncData, world);
			}
			world->SynchronizationBarrier();

			syncData->m_atomicIndex = 0;
			memset(internalForces, 0, bodyCount * sizeof(dgJacobian));
			for (dgInt32 i = 0; i < threadCounts; i++) {
				world->QueueJob(CalculateBodiesForceParallelKernel, syncData, world);
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

	syncData->m_atomicIndex = 0;
	for (dgInt32 j = 0; j < threadCounts; j++) {
		world->QueueJob(UpdateFeedbackForcesParallelKernel, syncData, world);
	}
	world->SynchronizationBarrier();

	dgInt32 hasJointFeeback = 0;
	for (dgInt32 i = 0; i < DG_MAX_THREADS_HIVE_COUNT; i++) {
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
}
