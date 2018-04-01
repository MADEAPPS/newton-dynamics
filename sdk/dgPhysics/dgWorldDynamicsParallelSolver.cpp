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


void dgWorldDynamicUpdate::CalculateReactionForcesParallel(const dgBodyCluster* const clusterArray, dgInt32 clustersCount, dgFloat32 timestep) const
{
	dgParallelSolverSyncData syncData;
	dgWorld* const world = (dgWorld*) this;
	
	dgInt32 bodyCount = 0;
	dgInt32 jointsCount = 0;
	for (dgInt32 i = 0; i < clustersCount; i ++) {
		const dgBodyCluster* const cluster = &clusterArray[i];
		jointsCount += cluster->m_jointCount;
		bodyCount += cluster->m_bodyCount - 1;
	}

	world->m_jointsMemory.ResizeIfNecessary((m_joints + jointsCount) * sizeof(dgJointInfo));
	dgJointInfo* const constraintArray = (dgJointInfo*)&world->m_jointsMemory[0];
	dgJointInfo* const jointArray = &constraintArray[m_joints];

	world->m_bodiesMemory.ResizeIfNecessary((m_bodies + bodyCount + 1) * sizeof(dgBodyInfo));
	dgBodyInfo* const bodyPtr = (dgBodyInfo*)&world->m_bodiesMemory[0];
	dgBodyInfo* const bodyArray = &bodyPtr[m_bodies];

	bodyArray[0].m_body = world->m_sentinelBody;
	dgAssert(world->m_sentinelBody->m_index == 0);

	dgInt32 rowsCount = 0;
	dgInt32 bodyIndex = 1;
	dgInt32 jointIndex = 0;

	dgBodyInfo* const bodyArrayPtr = (dgBodyInfo*)&world->m_bodiesMemory[0];
	dgJointInfo* const constraintArrayPtr = (dgJointInfo*)&world->m_jointsMemory[0];
	for (dgInt32 i = 0; i < clustersCount; i ++) {
		const dgBodyCluster* const cluster = &clusterArray[i];
		rowsCount += cluster->m_rowsCount;

		dgBodyInfo* const srcBodyArray = &bodyArrayPtr[cluster->m_bodyStart];
		const dgInt32 count = cluster->m_bodyCount;
		for (dgInt32 j = 1; j < count; j ++) {
			dgBody* const body = srcBodyArray[j].m_body;
			bodyArray[bodyIndex].m_body = body;
			body->m_index = bodyIndex;
			bodyIndex ++;
		}

		dgJointInfo* const clusterJointArray = &constraintArrayPtr[cluster->m_jointStart];
		const dgInt32 joints = cluster->m_jointCount;
		for (dgInt32 j = 0; j < joints; j ++) {
			jointArray[jointIndex] = clusterJointArray[j];
			dgJointInfo* const jointInfo = &jointArray[jointIndex];
			
			dgConstraint* const constraint = jointInfo->m_joint;
			const dgBody* const body0 = constraint->GetBody0();
			const dgBody* const body1 = constraint->GetBody1();

			dgInt32 m0 = (body0->GetInvMass().m_w != dgFloat32(0.0f)) ? body0->m_index : 0;
			dgInt32 m1 = (body1->GetInvMass().m_w != dgFloat32(0.0f)) ? body1->m_index : 0;
			jointInfo->m_m0 = m0;
			jointInfo->m_m1 = m1;
			jointIndex ++;
		}
	}

	syncData.m_bodyLocks = dgAlloca (dgInt32, bodyIndex);
	memset (syncData.m_bodyLocks, 0, bodyIndex * sizeof (dgInt32));

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
syncData.m_passes = 8;

	syncData.m_bodyCount = bodyIndex;
	syncData.m_jointCount = jointsCount;
	syncData.m_bodyArray = bodyArray;
	syncData.m_jointsArray = jointArray;
	syncData.m_atomicIndex = 0;

dgSwap (jointArray[0], jointArray[1]);

	syncData.m_clusterCount = clustersCount;
	syncData.m_clusterArray = clusterArray;

	dgBodyCluster cluster;
	cluster.m_bodyStart = 0;
	cluster.m_jointStart = 0;
	cluster.m_bodyCount = bodyIndex;
	cluster.m_clusterLRU = clusterArray[0].m_clusterLRU;
	cluster.m_jointCount = jointsCount;
	cluster.m_rowsCount = rowsCount;

	cluster.m_rowsStart = 0;
	cluster.m_isContinueCollision = 0;
	cluster.m_hasSoftBodies = 0;
	syncData.m_cluster = &cluster;
	syncData.m_weight = dgAlloca(dgFloat32, cluster.m_bodyCount * 2);
	memset(syncData.m_weight, 0, sizeof(dgFloat32) * cluster.m_bodyCount);

	for (dgInt32 i = 0; i < jointsCount; i++) {
		dgJointInfo* const jointInfo = &jointArray[i];
		const dgInt32 m0 = jointInfo->m_m0;
		const dgInt32 m1 = jointInfo->m_m1;
		syncData.m_weight[m0] += dgFloat32(1.0f);
		syncData.m_weight[m1] += dgFloat32(1.0f);
	}

	for (dgInt32 i = 0; i < cluster.m_bodyCount; i++) {
		const dgFloat32 weight = syncData.m_weight[i] ? syncData.m_weight[i] : dgFloat32(1.0f);
		syncData.m_weight[i + cluster.m_bodyCount] = weight;
		syncData.m_weight[i] = dgFloat32 (1.0f) / weight;
	}


	InitilizeBodyArrayParallel (&syncData);
	BuildJacobianMatrixParallel (&syncData);
	CalculateForcesParallel (&syncData);
	IntegrateClusterParallel(&syncData); 
}


void dgWorldDynamicUpdate::InitilizeBodyArrayParallel (dgParallelSolverSyncData* const syncData) const
{
	dgWorld* const world = (dgWorld*) this;
	const dgInt32 threadCounts = world->GetThreadCount();	

	dgJacobian* const internalForces = &world->m_solverMemory.m_internalForcesBuffer[0];
	internalForces[0].m_linear = dgVector::m_zero;
	internalForces[0].m_angular = dgVector::m_zero;

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

	dgBodyInfo* const bodyArray = syncData->m_bodyArray;
	dgJacobian* const internalForces = &world->m_solverMemory.m_internalForcesBuffer[0];

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


void dgWorldDynamicUpdate::BuildJacobianMatrixParallel(const dgBodyInfo* const bodyInfoArray, dgJointInfo* const jointInfo, dgJacobian* const internalForces, dgJacobianMatrixElement* const matrixRow, dgInt32* const bodyLocks, const dgFloat32* const weight, const dgFloat32* const invWeight) const
{
	const dgInt32 index = jointInfo->m_pairStart;
	const dgInt32 count = jointInfo->m_pairCount;
	const dgInt32 m0 = jointInfo->m_m0;
	const dgInt32 m1 = jointInfo->m_m1;

	const dgBody* const body0 = bodyInfoArray[m0].m_body;
	const dgBody* const body1 = bodyInfoArray[m1].m_body;
	const bool isBilateral = jointInfo->m_joint->IsBilateral();

	dgMatrix invInertia0 (body0->m_invWorldInertiaMatrix);
	dgMatrix invInertia1 (body1->m_invWorldInertiaMatrix);
	const dgVector invMass0(body0->m_invMass[3] * invWeight[m0]);
	const dgVector invMass1(body1->m_invMass[3] * invWeight[m1]);
	const dgVector w0(invWeight[m0]);
	const dgVector w1(invWeight[m1]);
	invInertia0.m_front *= w0;
	invInertia0.m_up *= w0;
	invInertia0.m_right *= w0;
	invInertia1.m_front *= w1;
	invInertia1.m_up *= w1;
	invInertia1.m_right *= w1;

	dgVector force0(dgVector::m_zero);
	dgVector torque0(dgVector::m_zero);
	if (body0->IsRTTIType(dgBody::m_dynamicBodyRTTI)) {
		dgVector invW0 (weight[m0]);
		force0 = ((dgDynamicBody*)body0)->m_externalForce * invW0;
		torque0 = ((dgDynamicBody*)body0)->m_externalTorque * invW0;
	}

	dgVector force1(dgVector::m_zero);
	dgVector torque1(dgVector::m_zero);
	if (body1->IsRTTIType(dgBody::m_dynamicBodyRTTI)) {
		dgVector invW1 (weight[m1]);
		force1 = ((dgDynamicBody*)body1)->m_externalForce * invW1;
		torque1 = ((dgDynamicBody*)body1)->m_externalTorque * invW1;
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

	const dgFloat32 forceImpulseScale = dgFloat32(1.0f);

	for (dgInt32 i = 0; i < count; i++) {
		dgJacobianMatrixElement* const row = &matrixRow[index + i];
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
		dgScopeSpinLock lock(&bodyLocks[m0]);
		internalForces[m1].m_linear += forceAcc1.m_linear;
		internalForces[m1].m_angular += forceAcc1.m_angular;
	}
}

void dgWorldDynamicUpdate::BuildJacobianMatrixParallelKernel (void* const context, void* const worldContext, dgInt32 threadID)
{
	dgParallelSolverSyncData* const syncData = (dgParallelSolverSyncData*) context;
	dgWorld* const world = (dgWorld*) worldContext;
	dgInt32* const atomicIndex = &syncData->m_atomicIndex; 
	dgBodyInfo* const bodyArray = syncData->m_bodyArray;
	dgJointInfo* const constraintArray = syncData->m_jointsArray;
	dgJacobianMatrixElement* const matrixRow = &world->m_solverMemory.m_jacobianBuffer[0];
	dgJacobian* const internalForces = &world->m_solverMemory.m_internalForcesBuffer[0];
	const dgFloat32* const weight = syncData->m_weight;
	const dgFloat32* const invWeight = syncData->m_weight + syncData->m_bodyCount;
	dgInt32* const bodyLocks = syncData->m_bodyLocks;
	dgAssert (syncData->m_jointCount);

	dgContraintDescritor constraintParams;
	constraintParams.m_world = world;
	constraintParams.m_threadIndex = threadID;
	constraintParams.m_timestep = syncData->m_timestep;
	constraintParams.m_invTimestep = syncData->m_invTimestep;

	for (dgInt32 i = dgAtomicExchangeAndAdd(atomicIndex, 1); i < syncData->m_jointCount;  i = dgAtomicExchangeAndAdd(atomicIndex, 1)) {
		dgJointInfo* const jointInfo = &constraintArray[i];
		dgConstraint* const constraint = jointInfo->m_joint;

		dgInt32 rowBase = dgAtomicExchangeAndAdd(&syncData->m_jacobianMatrixRowAtomicIndex, jointInfo->m_pairCount);

		world->GetJacobianDerivatives(constraintParams, jointInfo, constraint, matrixRow, rowBase);

		dgAssert (jointInfo->m_m0 >= 0);
		dgAssert (jointInfo->m_m1 >= 0);
		dgAssert (jointInfo->m_m0 != jointInfo->m_m1);
		world->BuildJacobianMatrixParallel(bodyArray, jointInfo, internalForces, matrixRow, bodyLocks, weight, invWeight);
	}
}


void dgWorldDynamicUpdate::CalculateJointsAccelParallelKernel (void* const context, void* const worldContext, dgInt32 threadID)
{
	dgParallelSolverSyncData* const syncData = (dgParallelSolverSyncData*) context;
	dgWorld* const world = (dgWorld*) worldContext;
	dgJointInfo* const constraintArray = syncData->m_jointsArray;
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
	dgJointInfo* const constraintArray = syncData->m_jointsArray;

	dgInt32* const atomicIndex = &syncData->m_atomicIndex;
	for (dgInt32 i = dgAtomicExchangeAndAdd(atomicIndex, 1); i < syncData->m_jointCount;  i = dgAtomicExchangeAndAdd(atomicIndex, 1)) {
		dgInt32 curJoint = i;
		if (constraintArray[curJoint].m_joint->m_updaFeedbackCallback) {
			constraintArray[curJoint].m_joint->m_updaFeedbackCallback (*constraintArray[curJoint].m_joint, syncData->m_timestep, threadID);
		}
	}
}

void dgWorldDynamicUpdate::IntegrateClusterParallelParallelKernel(void* const context, void* const worldContext, dgInt32 threadID)
{
	dgParallelSolverSyncData* const syncData = (dgParallelSolverSyncData*) context;
	dgWorld* const world = (dgWorld*) worldContext;

	dgInt32* const atomicIndex = &syncData->m_atomicIndex;
	for (dgInt32 i = dgAtomicExchangeAndAdd(atomicIndex, 1); i < syncData->m_clusterCount; i = dgAtomicExchangeAndAdd(atomicIndex, 1)) {
		world->IntegrateVelocity (&syncData->m_clusterArray[i], DG_SOLVER_MAX_ERROR, syncData->m_timestep, 0); 
	}
}

void dgWorldDynamicUpdate::CalculateJointsForceParallelKernel (void* const context, void* const worldContext, dgInt32 threadID)
{
	dgParallelSolverSyncData* const syncData = (dgParallelSolverSyncData*) context;
	dgWorld* const world = (dgWorld*) worldContext;
	dgJacobianMatrixElement* const matrixRow = &world->m_solverMemory.m_jacobianBuffer[0];
	dgBodyInfo* const bodyArray = syncData->m_bodyArray;
	dgJointInfo* const constraintArray = syncData->m_jointsArray;
	dgJacobian* const internalForces = &world->m_solverMemory.m_internalForcesBuffer[0];
	const int jointCount = syncData->m_jointCount;

	dgFloat32 accNorm = dgFloat32(0.0f);
	dgInt32* const atomicIndex = &syncData->m_atomicIndex;
	for (dgInt32 i = dgAtomicExchangeAndAdd(atomicIndex, 1); i < jointCount; i = dgAtomicExchangeAndAdd(atomicIndex, 1)) {
		dgJointInfo* const jointInfo = &constraintArray[i];
		dgFloat32 accel2 = world->CalculateJointForceParallel(jointInfo, bodyArray, internalForces, matrixRow);
		accNorm += accel2;
	}
	syncData->m_accelNorm[threadID] = accNorm;
}


void dgWorldDynamicUpdate::CalculateBodiesForceParallelKernel(void* const context, void* const worldContext, dgInt32 threadID)
{
	dgParallelSolverSyncData* const syncData = (dgParallelSolverSyncData*)context;
	dgWorld* const world = (dgWorld*)worldContext;
	dgJacobianMatrixElement* const matrixRow = &world->m_solverMemory.m_jacobianBuffer[0];
	dgJointInfo* const constraintArray = syncData->m_jointsArray;
	dgJacobian* const internalForces = &world->m_solverMemory.m_internalForcesBuffer[0];
	const dgFloat32* const weight = syncData->m_weight;
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
			dgAssert(dgCheckFloat(row->m_force));
			dgVector val(row->m_force);
			forceAcc0.m_linear += row->m_Jt.m_jacobianM0.m_linear * val;
			forceAcc0.m_angular += row->m_Jt.m_jacobianM0.m_angular * val;
			forceAcc1.m_linear += row->m_Jt.m_jacobianM1.m_linear * val;
			forceAcc1.m_angular += row->m_Jt.m_jacobianM1.m_angular * val;
		}

		const dgVector scale0(jointInfo->m_scale0 * weight[m0]);
		const dgVector scale1(jointInfo->m_scale1 * weight[m1]);
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
	dgBodyInfo* const bodyArray = syncData->m_bodyArray;
	dgJacobian* const internalForces = &world->m_solverMemory.m_internalForcesBuffer[0];
	const dgInt32 bodyCount = syncData->m_bodyCount;
	const dgFloat32* const weights = syncData->m_weight + bodyCount;
	dgVector speedFreeze2(world->m_freezeSpeed2 * dgFloat32(0.1f));
	dgVector freezeOmega2(world->m_freezeOmega2 * dgFloat32(0.1f));
	dgInt32* const atomicIndex = &syncData->m_atomicIndex;

	dgVector timestep4(syncData->m_timestepRK);
	for (dgInt32 i = dgAtomicExchangeAndAdd(atomicIndex, 1); i < bodyCount; i = dgAtomicExchangeAndAdd(atomicIndex, 1)) {
		dgDynamicBody* const body = (dgDynamicBody*)bodyArray[i].m_body;
		dgAssert(body->m_index == i);
		if (body->IsRTTIType(dgBody::m_dynamicBodyRTTI)) {
			const dgVector weight (weights[i]);
			const dgJacobian& forceAndTorque = internalForces[i];
			const dgVector force(body->m_externalForce + forceAndTorque.m_linear * weight);
			const dgVector torque(body->m_externalTorque + forceAndTorque.m_angular * weight);

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
	dgJointInfo* const constraintArray = syncData->m_jointsArray;
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
	dgBodyInfo* const bodyArray = syncData->m_bodyArray;

	dgFloat32 maxAccNorm2 = DG_SOLVER_MAX_ERROR * DG_SOLVER_MAX_ERROR;
	dgFloat32 invTimestepSrc = syncData->m_invTimestep;

	dgVector invTime(invTimestepSrc);
	dgInt32* const atomicIndex = &syncData->m_atomicIndex;
	for (dgInt32 i = dgAtomicExchangeAndAdd(atomicIndex, 1); i < syncData->m_bodyCount; i = dgAtomicExchangeAndAdd(atomicIndex, 1)) {
		dgDynamicBody* const body = (dgDynamicBody*)bodyArray[i].m_body;
		world->CalculateNetAcceleration(body, invTime, maxAccNorm2);
	}
}

void dgWorldDynamicUpdate::IntegrateClusterParallel(dgParallelSolverSyncData* const syncData) const
{
	dgWorld* const world = (dgWorld*) this;
	const dgInt32 threadCounts = world->GetThreadCount();

	syncData->m_atomicIndex = 0;
	for (dgInt32 i = 0; i < threadCounts; i++) {
		world->QueueJob(IntegrateClusterParallelParallelKernel, syncData, world);
	}
	world->SynchronizationBarrier();
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

/*
			for (dgInt32 i = 0; i < bodyCount; i ++) {
				dgVector scale (weights[i]);
				internalForces[i].m_linear = weigthedForces[i].m_linear * scale;
				internalForces[i].m_angular = weigthedForces[i].m_angular * scale;
				weigthedForces[i].m_linear = dgVector::m_zero;
				weigthedForces[i].m_angular = dgVector::m_zero;
			}
*/

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


dgFloat32 dgWorldDynamicUpdate::CalculateJointForceParallel(const dgJointInfo* const jointInfo, const dgBodyInfo* const bodyArray, const dgJacobian* const internalForces, dgJacobianMatrixElement* const matrixRow) const
{
	dgFloat32 accNorm = dgFloat32(0.0f);
	const dgInt32 m0 = jointInfo->m_m0;
	const dgInt32 m1 = jointInfo->m_m1;
	const dgBody* const body0 = bodyArray[m0].m_body;
	const dgBody* const body1 = bodyArray[m1].m_body;
	if (!(body0->m_resting & body1->m_resting)) {
		dgVector b[DG_CONSTRAINT_MAX_ROWS];
		dgVector x[DG_CONSTRAINT_MAX_ROWS + 1];
		dgVector low[DG_CONSTRAINT_MAX_ROWS];
		dgVector high[DG_CONSTRAINT_MAX_ROWS];
		dgVector delta_x[DG_CONSTRAINT_MAX_ROWS];
		dgVector delta_r[DG_CONSTRAINT_MAX_ROWS];
		dgVector diagDamp[DG_CONSTRAINT_MAX_ROWS];
		dgVector invJinvMJt[DG_CONSTRAINT_MAX_ROWS];
		dgVector x0[DG_CONSTRAINT_MAX_ROWS];
		dgVector mask[DG_CONSTRAINT_MAX_ROWS];
//		dgInt32 activeRows[DG_CONSTRAINT_MAX_ROWS];
		dgInt32 frictionIndex[DG_CONSTRAINT_MAX_ROWS];

		dgVector linearM0(internalForces[m0].m_linear);
		dgVector angularM0(internalForces[m0].m_angular);
		dgVector linearM1(internalForces[m1].m_linear);
		dgVector angularM1(internalForces[m1].m_angular);

		const dgVector scale0(jointInfo->m_scale0);
		const dgVector scale1(jointInfo->m_scale1);

		const dgInt32 index = jointInfo->m_pairStart;
		const dgInt32 rowsCount = jointInfo->m_pairCount;

		x[rowsCount] = dgVector::m_one;
		for (dgInt32 i = 0; i < rowsCount; i++) {
			dgJacobianMatrixElement* const row = &matrixRow[index + i];
			dgVector diag(row->m_JMinv.m_jacobianM0.m_linear * linearM0 + row->m_JMinv.m_jacobianM0.m_angular * angularM0 +
						  row->m_JMinv.m_jacobianM1.m_linear * linearM1 + row->m_JMinv.m_jacobianM1.m_angular * angularM1);

			dgVector accel(row->m_coordenateAccel - row->m_force * row->m_diagDamp - (diag.AddHorizontal()).GetScalar());
			dgVector force(row->m_force + row->m_invJinvMJt * accel.GetScalar());

			dgAssert(row->m_normalForceIndex >= 0);
			dgAssert(row->m_normalForceIndex <= rowsCount);
			frictionIndex[i] = row->m_normalForceIndex;

			dgVector frictionNormal(x[frictionIndex[i]]);
			dgVector lowerFrictionForce(frictionNormal * row->m_lowerBoundFrictionCoefficent);
			dgVector upperFrictionForce(frictionNormal * row->m_upperBoundFrictionCoefficent);

			accel = accel.AndNot((force > upperFrictionForce) | (force < lowerFrictionForce));
			force = force.GetMax(lowerFrictionForce).GetMin(upperFrictionForce);
			accNorm += accel.GetScalar() * accel.GetScalar();

			dgVector deltaForce(force - dgVector(row->m_force));

			x[i] = force;
			b[i] = dgVector(row->m_coordenateAccel);
			low[i] = dgVector(row->m_lowerBoundFrictionCoefficent);
			high[i] = dgVector(row->m_upperBoundFrictionCoefficent);
			diagDamp[i] = dgVector(row->m_diagDamp);
			invJinvMJt[i] = dgVector(row->m_invJinvMJt);

			dgVector deltaforce0(scale0 * deltaForce);
			dgVector deltaforce1(scale1 * deltaForce);
			linearM0 += row->m_Jt.m_jacobianM0.m_linear * deltaforce0;
			angularM0 += row->m_Jt.m_jacobianM0.m_angular * deltaforce0;
			linearM1 += row->m_Jt.m_jacobianM1.m_linear * deltaforce1;
			angularM1 += row->m_Jt.m_jacobianM1.m_angular * deltaforce1;
		}

#if 0
		const dgFloat32 tol2 = dgFloat32(1.0e-4f);
		if (accNorm > tol2) {
			dgVector maxAccel(accNorm);
			for (dgInt32 j = 0; (j < 4) && (maxAccel.GetScalar() > tol2); j++) {
				maxAccel = dgFloat32(0.0f);
				for (dgInt32 i = 0; i < rowsCount; i++) {
					const dgJacobianMatrixElement* const row = &matrixRow[index + i];
					dgVector diag(row->m_JMinv.m_jacobianM0.m_linear * linearM0 + row->m_JMinv.m_jacobianM0.m_angular * angularM0 +
								  row->m_JMinv.m_jacobianM1.m_linear * linearM1 + row->m_JMinv.m_jacobianM1.m_angular * angularM1);

					dgVector accel(b[i] - x[i] * diagDamp[i] - diag.AddHorizontal());
					dgVector force(x[i] + invJinvMJt[i] * accel);

					const dgVector frictionNormal(x[frictionIndex[i]]);
					const dgVector lowerFrictionForce(frictionNormal * low[i]);
					const dgVector upperFrictionForce(frictionNormal * high[i]);
					const dgVector clampedForce(force.GetMax(lowerFrictionForce).GetMin(upperFrictionForce));

					const dgVector deltaForce(clampedForce - x[i]);
					x[i] = clampedForce;

					const dgVector clampedAccel(accel.AndNot((force > upperFrictionForce) | (force < lowerFrictionForce)));
					maxAccel += clampedAccel * clampedAccel;

					const dgVector deltaforce0(scale0 * deltaForce);
					const dgVector deltaforce1(scale1 * deltaForce);
					linearM0 += row->m_Jt.m_jacobianM0.m_linear * deltaforce0;
					angularM0 += row->m_Jt.m_jacobianM0.m_angular * deltaforce0;
					linearM1 += row->m_Jt.m_jacobianM1.m_linear * deltaforce1;
					angularM1 += row->m_Jt.m_jacobianM1.m_angular * deltaforce1;
				}
			}

			if (maxAccel.GetScalar() > tol2) {
				dgInt32 passes = rowsCount;
				dgFloat32 beta = dgFloat32(0.0f);
				dgInt32 activeRowsCount = 0;
				dgInt32 clampedIndex = rowsCount - 1;
				for (dgInt32 i = 0; i < rowsCount; i++) {
					const dgJacobianMatrixElement* const row = &matrixRow[index + i];
					const dgVector frictionNormal = x[frictionIndex[i]];
					low[i] = low[i] * frictionNormal;
					high[i] = high[i] * frictionNormal;

					dgVector diag(row->m_JMinv.m_jacobianM0.m_linear * linearM0 + row->m_JMinv.m_jacobianM0.m_angular * angularM0 +
						row->m_JMinv.m_jacobianM1.m_linear * linearM1 + row->m_JMinv.m_jacobianM1.m_angular * angularM1);

					b[i] -= (x[i] * diagDamp[i] + diag.AddHorizontal());
					x0[i] = x[i];
					const dgVector x1(x[i] + invJinvMJt[i] * b[i]);
					mask[i] = dgVector::m_one.AndNot((x1 < low[i]) | (x1 > high[i]));
					delta_x[i] = b[i] * mask[i];
					beta += (delta_x[i] * delta_x[i]).GetScalar();
					const dgInt32 isClamped = mask[i].GetScalar() ? 0 : 1;
					passes -= mask[i].GetScalar() ? 0 : 1;

					activeRows[clampedIndex] = i;
					activeRows[activeRowsCount] = i;
					activeRowsCount += 1 - isClamped;
					clampedIndex -= isClamped;
				}

				dgInt32 iter = 0;
				for (dgInt32 k = 0; (k < passes) && (beta > tol2); k++) {
					iter++;
					dgJacobian delta_x0;
					dgJacobian delta_x1;
					delta_x0.m_linear = dgVector::m_zero;
					delta_x0.m_angular = dgVector::m_zero;
					delta_x1.m_linear = dgVector::m_zero;
					delta_x1.m_angular = dgVector::m_zero;

					for (dgInt32 i = 0; i < activeRowsCount; i++) {
						const dgInt32 j = activeRows[i];
						const dgJacobianMatrixElement* const row = &matrixRow[index + j];

						dgVector deltaforce0(scale0 * delta_x[j]);
						dgVector deltaforce1(scale1 * delta_x[j]);
						delta_x0.m_linear += row->m_Jt.m_jacobianM0.m_linear * deltaforce0;
						delta_x0.m_angular += row->m_Jt.m_jacobianM0.m_angular * deltaforce0;
						delta_x1.m_linear += row->m_Jt.m_jacobianM1.m_linear * deltaforce1;
						delta_x1.m_angular += row->m_Jt.m_jacobianM1.m_angular * deltaforce1;
					}

					dgFloat32 num = dgFloat32(0.0f);
					dgFloat32 den = dgFloat32(0.0f);
					for (dgInt32 i = 0; i < rowsCount; i++) {
						const dgJacobianMatrixElement* const row = &matrixRow[index + i];
						dgVector diag(row->m_JMinv.m_jacobianM0.m_linear * delta_x0.m_linear + row->m_JMinv.m_jacobianM0.m_angular * delta_x0.m_angular +
							row->m_JMinv.m_jacobianM1.m_linear * delta_x1.m_linear + row->m_JMinv.m_jacobianM1.m_angular * delta_x1.m_angular);
						delta_r[i] = diag.AddHorizontal() + delta_x[i] * diagDamp[i];

						den += (delta_x[i] * delta_r[i]).GetScalar();
						num += (b[i] * b[i] * mask[i]).GetScalar();
					}

					dgInt32 clampIndex = -1;
					dgFloat32 alpha = num / den;
					dgAssert(alpha > dgFloat32(0.0f));
					for (dgInt32 i = 0; (i < activeRowsCount) && (alpha > dgFloat32(0.0f)); i++) {
						const dgInt32 j = activeRows[i];
						dgFloat32 x1 = x[j].GetScalar() + alpha * delta_x[j].GetScalar();
						if (x1 < low[j].GetScalar()) {
							clampIndex = i;
							alpha = (low[j].GetScalar() - x[j].GetScalar()) / delta_x[j].GetScalar();
						}
						else if (x1 > high[j].GetScalar()) {
							clampIndex = i;
							alpha = (high[j].GetScalar() - x[j].GetScalar()) / delta_x[j].GetScalar();
						}
						dgAssert(alpha >= dgFloat32(-1.0e-4f));
						if (alpha < dgFloat32(1.0e-6f)) {
							alpha = dgFloat32(0.0f);
						}
					}

					beta = dgFloat32(0.0f);
					dgVector alphav(alpha);
					for (dgInt32 i = 0; i < rowsCount; i++) {
						x[i] += alphav * delta_x[i];
						b[i] -= alphav * delta_r[i];
						beta += (b[i] * b[i] * mask[i]).GetScalar();
					}

					if (clampIndex >= 0) {
						k = 0;
						passes--;
						beta = dgFloat32(0.0f);
						const dgInt32 n = activeRows[clampIndex];
						mask[n] = dgVector::m_zero;
						delta_x[n] = dgVector::m_zero;
						activeRowsCount--;
						dgSwap(activeRows[clampIndex], activeRows[activeRowsCount]);
						for (dgInt32 i = 0; i < activeRowsCount; i++) {
							const dgInt32 j = activeRows[i];
							delta_x[j] = b[j];
							beta += (b[j] * b[j]).GetScalar();
						}
					} else {
						dgVector gamma(beta / num);
						for (dgInt32 i = 0; i < activeRowsCount; i++) {
							const dgInt32 j = activeRows[i];
							delta_x[j] = b[j] + gamma * delta_x[j];
						}
					}
				}
/*
				for (dgInt32 i = 0; i < rowsCount; i++) {
					const dgJacobianMatrixElement* const row = &matrixRow[index + i];
					const dgVector deltax(x[i] - x0[i]);
					const dgVector deltaforce0(scale0 * deltax);
					const dgVector deltaforce1(scale1 * deltax);
					linearM0 += row->m_Jt.m_jacobianM0.m_linear * deltaforce0;
					angularM0 += row->m_Jt.m_jacobianM0.m_angular * deltaforce0;
					linearM1 += row->m_Jt.m_jacobianM1.m_linear * deltaforce1;
					angularM1 += row->m_Jt.m_jacobianM1.m_angular * deltaforce1;
				}
*/
			}
		}
/*
		//const dgFloat32 weight = dgFloat32 (0.66f);
		const dgFloat32 weight = dgFloat32(0.3f);
		for (dgInt32 i = 0; i < rowsCount; i++) {
			dgJacobianMatrixElement* const row = &matrixRow[index + i];
			row->m_force = row->m_force + (x[i].GetScalar() - row->m_force) * weight;
			row->m_maxImpact = dgMax(dgAbs(row->m_force), row->m_maxImpact);
		}
*/

/*
		if (m0) {
			dgScopeSpinLock lock(&bodyLocks[m0]);
			weightedForces[m0].m_linear += (linearM0 - internalForces[m0].m_linear);
			weightedForces[m0].m_angular += (angularM0 - internalForces[m0].m_angular);
		}
		if (m1) {
			dgScopeSpinLock lock(&bodyLocks[m0]);
			weightedForces[m1].m_linear += linearM1;
			weightedForces[m1].m_angular += angularM1;
		}
*/

		#else

		dgVector maxAccel(accNorm);
		dgVector sor(dgFloat32(1.3f));
		const dgFloat32 tol2 = dgFloat32(1.0e-4f);
		for (dgInt32 j = 0; (j < 8) && (maxAccel.GetScalar() > tol2); j++) {
			maxAccel = dgVector::m_zero;
			for (dgInt32 i = 0; i < rowsCount; i++) {
				const dgJacobianMatrixElement* const row = &matrixRow[index + i];
				dgVector diag(row->m_JMinv.m_jacobianM0.m_linear * linearM0 + row->m_JMinv.m_jacobianM0.m_angular * angularM0 +
					row->m_JMinv.m_jacobianM1.m_linear * linearM1 + row->m_JMinv.m_jacobianM1.m_angular * angularM1);

				dgVector accel(b[i] - x[i] * diagDamp[i] - diag.AddHorizontal());
				//dgVector force(x[i] + invJinvMJt[i] * accel);
				dgVector force(x[i] + invJinvMJt[i] * accel * sor);

				const dgVector frictionNormal(x[frictionIndex[i]]);
				const dgVector lowerFrictionForce(frictionNormal * low[i]);
				const dgVector upperFrictionForce(frictionNormal * high[i]);
				const dgVector clampedForce(force.GetMax(lowerFrictionForce).GetMin(upperFrictionForce));

				const dgVector deltaForce(clampedForce - x[i]);
				x[i] = clampedForce;

				const dgVector clampedAccel(accel.AndNot((force > upperFrictionForce) | (force < lowerFrictionForce)));
				maxAccel += clampedAccel * clampedAccel;

				const dgVector deltaforce0(scale0 * deltaForce);
				const dgVector deltaforce1(scale1 * deltaForce);
				linearM0 += row->m_Jt.m_jacobianM0.m_linear * deltaforce0;
				angularM0 += row->m_Jt.m_jacobianM0.m_angular * deltaforce0;
				linearM1 += row->m_Jt.m_jacobianM1.m_linear * deltaforce1;
				angularM1 += row->m_Jt.m_jacobianM1.m_angular * deltaforce1;
			}
		}
		#endif

		for (dgInt32 i = 0; i < rowsCount; i++) {
			dgJacobianMatrixElement* const row = &matrixRow[index + i];
			row->m_force = x[i].GetScalar();
			row->m_maxImpact = dgMax(dgAbs(row->m_force), row->m_maxImpact);
		}
	}

	return accNorm;
}

