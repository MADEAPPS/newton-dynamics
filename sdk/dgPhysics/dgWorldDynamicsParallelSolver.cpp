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
#include "dgWorldDynamicsParallelSolver.h"


dgWorkGroupFloat dgWorkGroupFloat::m_one(dgVector::m_one, dgVector::m_one);
dgWorkGroupFloat dgWorkGroupFloat::m_zero(dgVector::m_zero, dgVector::m_zero);


void dgWorldDynamicUpdate::CalculateReactionForcesParallel(const dgBodyCluster* const clusterArray, dgInt32 clustersCount, dgFloat32 timestep)
{
	dgWorld* const world = (dgWorld*) this;

	dgBodyCluster cluster(MergeClusters(clusterArray, clustersCount));

	dgBodyInfo* const bodyPtr = (dgBodyInfo*)&world->m_bodiesMemory[0];
	dgJointInfo* const constraintArray = (dgJointInfo*)&world->m_jointsMemory[0];

	dgBodyInfo* const bodyArray = &bodyPtr[m_bodies];
	dgJointInfo* const jointArray = &constraintArray[m_joints];

	if (world->m_currentPlugin) {
		dgWorldPlugin* const plugin = world->m_currentPlugin->GetInfo().m_plugin;
		plugin->CalculateJointForces(cluster, bodyArray, jointArray, timestep);
	} else {
		m_parallelSolver.CalculateJointForces(cluster, bodyArray, jointArray, timestep);
/*
	dgJacobian* const internalForces = &world->m_solverMemory.m_internalForcesBuffer[0];
	internalForces[0].m_linear = dgVector::m_zero;
	internalForces[0].m_angular = dgVector::m_zero;

	dgFloat32* const weights = dgAlloca(dgFloat32, cluster.m_bodyCount * 2);
	memset(weights, 0, sizeof(dgFloat32) * cluster.m_bodyCount);
	for (dgInt32 i = 0; i < cluster.m_jointCount; i++) {
		dgJointInfo* const jointInfo = &jointArray[i];
		const dgInt32 m0 = jointInfo->m_m0;
		const dgInt32 m1 = jointInfo->m_m1;
		weights[m0] += dgFloat32(1.0f);
		weights[m1] += dgFloat32(1.0f);
	}

	for (dgInt32 i = 0; i < cluster.m_bodyCount; i++) {
		//const dgFloat32 weight = syncData.m_weight[i] ? syncData.m_weight[i] : dgFloat32(1.0f);
		const dgFloat32 weight = weights[i] ? weights[i] : dgFloat32(1.0f);
		weights[i + cluster.m_bodyCount] = weight;
		weights[i] = dgFloat32 (1.0f) / weight;
	}


	dgParallelSolverSyncData syncData;

	syncData.m_bodyLocks = dgAlloca(dgInt32, cluster.m_bodyCount);
	memset(syncData.m_bodyLocks, 0, cluster.m_bodyCount * sizeof(dgInt32));

	const dgInt32 rkSubSteps = 4;
	syncData.m_timestep = timestep;
	syncData.m_invTimestep = (timestep > dgFloat32(0.0f)) ? dgFloat32(1.0f) / timestep : dgFloat32(0.0f);
	syncData.m_invStepRK = (dgFloat32(1.0f) / dgFloat32(rkSubSteps));
	syncData.m_timestepRK = syncData.m_timestep * syncData.m_invStepRK;
	syncData.m_invTimestepRK = syncData.m_invTimestep * dgFloat32(rkSubSteps);
	syncData.m_rkSubSteps = rkSubSteps;
	syncData.m_passes = world->m_solverMode;
	syncData.m_passes = 16;

	syncData.m_bodyCount = cluster.m_bodyCount;
	syncData.m_jointCount = cluster.m_jointCount;
	syncData.m_bodyArray = bodyArray;
	syncData.m_jointsArray = jointArray;
	syncData.m_atomicIndex = 0;

	syncData.m_clusterCount = clustersCount;
	syncData.m_clusterArray = clusterArray;

	syncData.m_cluster = &cluster;
	syncData.m_weight = weights;

	InitilizeBodyArrayParallel(&syncData);
	BuildJacobianMatrixParallel(&syncData);
	CalculateForcesParallel(&syncData);
	IntegrateClusterParallel(&syncData);
*/
	}
}

dgInt32  dgWorldDynamicUpdate::CompareJointInfos(const dgJointInfo* const infoA, const dgJointInfo* const infoB, void* notUsed)
{
	dgInt32 countA = infoA->m_pairCount;
	dgInt32 countB = infoB->m_pairCount;

	if (countA < countB) {
		return 1;
	}
	if (countA > countB) {
		return -1;
	}
	return 0;
}

dgBodyCluster dgWorldDynamicUpdate::MergeClusters(const dgBodyCluster* const clusterArray, dgInt32 clustersCount) const
{
	dgBodyCluster cluster;

	dgWorld* const world = (dgWorld*) this;
	dgInt32 bodyCount = 0;
	dgInt32 jointsCount = 0;
	for (dgInt32 i = 0; i < clustersCount; i++) {
		const dgBodyCluster* const srcCluster = &clusterArray[i];
		jointsCount += srcCluster->m_jointCount;
		bodyCount += srcCluster->m_bodyCount - 1;
	}

	world->m_bodiesMemory.ResizeIfNecessary((m_bodies + bodyCount + DG_WORK_GROUP_SIZE + 1) * sizeof(dgBodyInfo));
	world->m_jointsMemory.ResizeIfNecessary((m_joints + jointsCount + 32) * sizeof(dgJointInfo));

	dgBodyInfo* const bodyPtr = (dgBodyInfo*)&world->m_bodiesMemory[0];
	dgJointInfo* const constraintPtr = (dgJointInfo*)&world->m_jointsMemory[0];

	dgBodyInfo* const bodyArray = &bodyPtr[m_bodies];
	dgJointInfo* const jointArray = &constraintPtr[m_joints];

	bodyArray[0].m_body = world->m_sentinelBody;
	dgAssert(world->m_sentinelBody->m_index == 0);

	dgInt32 rowsCount = 0;
	dgInt32 bodyIndex = 1;
	dgInt32 jointIndex = 0;
	for (dgInt32 i = 0; i < clustersCount; i++) {
		const dgBodyCluster* const srcCluster = &clusterArray[i];
		rowsCount += srcCluster->m_rowsCount;

		dgBodyInfo* const srcBodyArray = &bodyPtr[srcCluster->m_bodyStart];
		const dgInt32 count = srcCluster->m_bodyCount;
		for (dgInt32 j = 1; j < count; j++) {
			dgBody* const body = srcBodyArray[j].m_body;
			bodyArray[bodyIndex].m_body = body;
			body->m_index = bodyIndex;
			bodyIndex++;
		}

		dgJointInfo* const clusterJointArray = &constraintPtr[srcCluster->m_jointStart];
		const dgInt32 joints = srcCluster->m_jointCount;
		for (dgInt32 j = 0; j < joints; j++) {
			jointArray[jointIndex] = clusterJointArray[j];
			dgJointInfo* const jointInfo = &jointArray[jointIndex];

			dgConstraint* const constraint = jointInfo->m_joint;
			const dgBody* const body0 = constraint->GetBody0();
			const dgBody* const body1 = constraint->GetBody1();

			dgInt32 m0 = (body0->GetInvMass().m_w != dgFloat32(0.0f)) ? body0->m_index : 0;
			dgInt32 m1 = (body1->GetInvMass().m_w != dgFloat32(0.0f)) ? body1->m_index : 0;
			jointInfo->m_m0 = m0;
			jointInfo->m_m1 = m1;
			jointIndex++;
		}
	}

	dgSort(jointArray, jointIndex, CompareJointInfos);

	cluster.m_bodyStart = 0;
	cluster.m_jointStart = 0;
	cluster.m_bodyCount = bodyIndex;
	cluster.m_clusterLRU = clusterArray[0].m_clusterLRU;
	cluster.m_jointCount = jointsCount;
	cluster.m_rowsCount = rowsCount;

	cluster.m_rowsStart = 0;
	cluster.m_isContinueCollision = 0;
	cluster.m_hasSoftBodies = 0;

	return cluster;
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

void dgWorldDynamicUpdate::BuildJacobianMatrixParallel(const dgBodyInfo* const bodyInfoArray, dgJointInfo* const jointInfo, dgJacobian* const internalForces, dgLeftHandSide* const matrixRow, dgRightHandSide* const rightHandSide, dgInt32* const bodyLocks, const dgFloat32* const weight, const dgFloat32* const invWeight) const
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

	jointInfo->m_preconditioner0 = dgFloat32(1.0f);
	jointInfo->m_preconditioner1 = dgFloat32(1.0f);
//	const dgFloat32 diagonalPreconditioner = jointInfo->m_joint->m_diagonalPreconditioner;
//	if ((invMass0.GetScalar() > dgFloat32(0.0f)) && (invMass1.GetScalar() > dgFloat32(0.0f))) {
	if ((invMass0.GetScalar() > dgFloat32(0.0f)) && (invMass1.GetScalar() > dgFloat32(0.0f)) && !(body0->GetSkeleton() && body1->GetSkeleton())) {
		const dgFloat32 mass0 = body0->GetMass().m_w;
		const dgFloat32 mass1 = body1->GetMass().m_w;
		if (mass0 > (DG_DIAGONAL_PRECONDITIONER * mass1)) {
			jointInfo->m_preconditioner0 = mass0 / (mass1 * DG_DIAGONAL_PRECONDITIONER);
		} else if (mass1 > (DG_DIAGONAL_PRECONDITIONER * mass0)) {
			jointInfo->m_preconditioner1 = mass1 / (mass0 * DG_DIAGONAL_PRECONDITIONER);
		}
	}

	dgJacobian forceAcc0;
	dgJacobian forceAcc1;
	const dgVector preconditioner0(jointInfo->m_preconditioner0);
	const dgVector preconditioner1(jointInfo->m_preconditioner1);
	forceAcc0.m_linear = dgVector::m_zero;
	forceAcc0.m_angular = dgVector::m_zero;
	forceAcc1.m_linear = dgVector::m_zero;
	forceAcc1.m_angular = dgVector::m_zero;

	const dgFloat32 forceImpulseScale = dgFloat32(1.0f);

	for (dgInt32 i = 0; i < count; i++) {
		dgRightHandSide* const rhs = &rightHandSide[index + i];
		dgLeftHandSide* const row = &matrixRow[index + i];
		row->m_JMinv.m_jacobianM0.m_linear = row->m_Jt.m_jacobianM0.m_linear * invMass0;
		row->m_JMinv.m_jacobianM0.m_angular = invInertia0.RotateVector(row->m_Jt.m_jacobianM0.m_angular);
		row->m_JMinv.m_jacobianM1.m_linear = row->m_Jt.m_jacobianM1.m_linear * invMass1;
		row->m_JMinv.m_jacobianM1.m_angular = invInertia1.RotateVector(row->m_Jt.m_jacobianM1.m_angular);

		dgVector tmpAccel(row->m_JMinv.m_jacobianM0.m_linear * force0 + row->m_JMinv.m_jacobianM0.m_angular * torque0 +
						  row->m_JMinv.m_jacobianM1.m_linear * force1 + row->m_JMinv.m_jacobianM1.m_angular * torque1);

		dgAssert(tmpAccel.m_w == dgFloat32(0.0f));
		dgFloat32 extenalAcceleration = -(tmpAccel.AddHorizontal()).GetScalar();
		rhs->m_deltaAccel = extenalAcceleration * forceImpulseScale;
		rhs->m_coordenateAccel += extenalAcceleration * forceImpulseScale;
		dgAssert(rhs->m_jointFeebackForce);
		const dgFloat32 force = rhs->m_jointFeebackForce->m_force * forceImpulseScale;
		rhs->m_force = isBilateral ? dgClamp(force, rhs->m_lowerBoundFrictionCoefficent, rhs->m_upperBoundFrictionCoefficent) : force;
		rhs->m_maxImpact = dgFloat32(0.0f);

		dgVector jMinvM0linear(preconditioner0 * row->m_JMinv.m_jacobianM0.m_linear);
		dgVector jMinvM0angular(preconditioner0 * row->m_JMinv.m_jacobianM0.m_angular);
		dgVector jMinvM1linear(preconditioner1 * row->m_JMinv.m_jacobianM1.m_linear);
		dgVector jMinvM1angular(preconditioner1 * row->m_JMinv.m_jacobianM1.m_angular);

		dgVector tmpDiag(jMinvM0linear * row->m_Jt.m_jacobianM0.m_linear + jMinvM0angular * row->m_Jt.m_jacobianM0.m_angular +
						 jMinvM1linear * row->m_Jt.m_jacobianM1.m_linear + jMinvM1angular * row->m_Jt.m_jacobianM1.m_angular);

		dgAssert(tmpDiag.m_w == dgFloat32(0.0f));
		dgFloat32 diag = (tmpDiag.AddHorizontal()).GetScalar();
		dgAssert(diag > dgFloat32(0.0f));
		rhs->m_diagDamp = diag * rhs->m_stiffness;
		diag *= (dgFloat32(1.0f) + rhs->m_stiffness);
//		rhs->m_jinvMJt = diag;
		rhs->m_invJinvMJt = dgFloat32(1.0f) / diag;

		dgAssert(dgCheckFloat(rhs->m_force));
		dgVector val(rhs->m_force);
		forceAcc0.m_linear += row->m_Jt.m_jacobianM0.m_linear * val;
		forceAcc0.m_angular += row->m_Jt.m_jacobianM0.m_angular * val;
		forceAcc1.m_linear += row->m_Jt.m_jacobianM1.m_linear * val;
		forceAcc1.m_angular += row->m_Jt.m_jacobianM1.m_angular * val;
	}

	forceAcc0.m_linear = forceAcc0.m_linear * preconditioner0;
	forceAcc0.m_angular = forceAcc0.m_angular * preconditioner0;
	forceAcc1.m_linear = forceAcc1.m_linear * preconditioner1;
	forceAcc1.m_angular = forceAcc1.m_angular * preconditioner1;

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
	dgRightHandSide* const righHandSide = &world->m_solverMemory.m_righHandSizeBuffer[0];
	dgLeftHandSide* const matrixRow = &world->m_solverMemory.m_jacobianBuffer[0];
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

		world->GetJacobianDerivatives(constraintParams, jointInfo, constraint, matrixRow, righHandSide, rowBase);

		dgAssert (jointInfo->m_m0 >= 0);
		dgAssert (jointInfo->m_m1 >= 0);
		dgAssert (jointInfo->m_m0 != jointInfo->m_m1);
		world->BuildJacobianMatrixParallel(bodyArray, jointInfo, internalForces, matrixRow, righHandSide, bodyLocks, weight, invWeight);
	}
}

void dgWorldDynamicUpdate::CalculateJointsAccelParallelKernel (void* const context, void* const worldContext, dgInt32 threadID)
{
	dgParallelSolverSyncData* const syncData = (dgParallelSolverSyncData*) context;
	dgWorld* const world = (dgWorld*) worldContext;
	dgJointInfo* const constraintArray = syncData->m_jointsArray;
	dgLeftHandSide* const matrixRow = &world->m_solverMemory.m_jacobianBuffer[0];

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
	dgRightHandSide* const righHandSide = &world->m_solverMemory.m_righHandSizeBuffer[0];
	const dgLeftHandSide* const matrixRow = &world->m_solverMemory.m_jacobianBuffer[0];
	dgBodyInfo* const bodyArray = syncData->m_bodyArray;
	dgJointInfo* const constraintArray = syncData->m_jointsArray;
	dgJacobian* const internalForces = &world->m_solverMemory.m_internalForcesBuffer[0];
	const int jointCount = syncData->m_jointCount;

	dgFloat32 accNorm = dgFloat32(0.0f);
	dgInt32* const atomicIndex = &syncData->m_atomicIndex;
	for (dgInt32 i = dgAtomicExchangeAndAdd(atomicIndex, 1); i < jointCount; i = dgAtomicExchangeAndAdd(atomicIndex, 1)) {
		dgJointInfo* const jointInfo = &constraintArray[i];
		dgFloat32 accel2 = world->CalculateJointForceParallel(jointInfo, bodyArray, internalForces, matrixRow, righHandSide);
		accNorm += accel2;
	}
	syncData->m_accelNorm[threadID] = accNorm;
}


void dgWorldDynamicUpdate::CalculateBodiesForceParallelKernel(void* const context, void* const worldContext, dgInt32 threadID)
{
	dgParallelSolverSyncData* const syncData = (dgParallelSolverSyncData*)context;
	dgWorld* const world = (dgWorld*)worldContext;
	dgRightHandSide* const righHandSide = &world->m_solverMemory.m_righHandSizeBuffer[0];
	const dgLeftHandSide* const matrixRow = &world->m_solverMemory.m_jacobianBuffer[0];
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
			const dgRightHandSide* const rhs = &righHandSide[index + i];
			const dgLeftHandSide* const row = &matrixRow[index + i];
			dgAssert(dgCheckFloat(rhs->m_force));
			dgVector val(rhs->m_force);
			forceAcc0.m_linear += row->m_Jt.m_jacobianM0.m_linear * val;
			forceAcc0.m_angular += row->m_Jt.m_jacobianM0.m_angular * val;
			forceAcc1.m_linear += row->m_Jt.m_jacobianM1.m_linear * val;
			forceAcc1.m_angular += row->m_Jt.m_jacobianM1.m_angular * val;
		}

		const dgVector preconditioner0(jointInfo->m_preconditioner0 * weight[m0]);
		const dgVector preconditioner1(jointInfo->m_preconditioner1 * weight[m1]);
		forceAcc0.m_linear = forceAcc0.m_linear * preconditioner0;
		forceAcc0.m_angular = forceAcc0.m_angular * preconditioner0;
		forceAcc1.m_linear = forceAcc1.m_linear * preconditioner1;
		forceAcc1.m_angular = forceAcc1.m_angular * preconditioner1;

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
	dgRightHandSide* const rightHandSide = &world->m_solverMemory.m_righHandSizeBuffer[0];
	//dgLeftHandSide* const matrixRow = &world->m_solverMemory.m_jacobianBuffer[0];

	dgInt32 hasJointFeeback = 0;
	dgInt32* const atomicIndex = &syncData->m_atomicIndex;

	for (dgInt32 curJoint = dgAtomicExchangeAndAdd(atomicIndex, 1); curJoint < syncData->m_jointCount; curJoint = dgAtomicExchangeAndAdd(atomicIndex, 1)) {
		dgJointInfo* const jointInfo = &constraintArray[curJoint];
		dgConstraint* const constraint = jointInfo->m_joint;
		const dgInt32 first = jointInfo->m_pairStart;
		const dgInt32 count = jointInfo->m_pairCount;

		for (dgInt32 j = 0; j < count; j++) {
			dgRightHandSide* const rhs = &rightHandSide[j + first];
			//dgLeftHandSide* const row = &matrixRow[j + first];
			dgAssert(dgCheckFloat(rhs->m_force));
			rhs->m_jointFeebackForce->m_force = rhs->m_force;
			rhs->m_jointFeebackForce->m_impact = rhs->m_maxImpact * syncData->m_timestepRK;
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


dgFloat32 dgWorldDynamicUpdate::CalculateJointForceParallel(const dgJointInfo* const jointInfo, const dgBodyInfo* const bodyArray, const dgJacobian* const internalForces, const dgLeftHandSide* const matrixRow, dgRightHandSide* const rightHandSide) const
{
	dgVector accNorm(dgVector::m_zero);
	dgFloat32 normalForce[DG_CONSTRAINT_MAX_ROWS + 4];

	const dgInt32 m0 = jointInfo->m_m0;
	const dgInt32 m1 = jointInfo->m_m1;
	const dgBody* const body0 = bodyArray[m0].m_body;
	const dgBody* const body1 = bodyArray[m1].m_body;

	if (!(body0->m_resting & body1->m_resting)) {
		dgInt32 rowsCount = jointInfo->m_pairCount;

		dgVector linearM0(internalForces[m0].m_linear);
		dgVector angularM0(internalForces[m0].m_angular);
		dgVector linearM1(internalForces[m1].m_linear);
		dgVector angularM1(internalForces[m1].m_angular);

		const dgVector preconditioner0(jointInfo->m_preconditioner0);
		const dgVector preconditioner1(jointInfo->m_preconditioner1);

		normalForce[rowsCount] = dgFloat32(1.0f);
		//dgInt32 j = jointInfo->m_pairStart;
		const dgInt32 index = jointInfo->m_pairStart;
		for (dgInt32 k = 0; k < rowsCount; k++) {
			dgRightHandSide* const rhs = &rightHandSide[index + k];
			const dgLeftHandSide* const row = &matrixRow[index + k];

			dgAssert(row->m_Jt.m_jacobianM0.m_linear.m_w == dgFloat32(0.0f));
			dgAssert(row->m_Jt.m_jacobianM0.m_angular.m_w == dgFloat32(0.0f));
			dgAssert(row->m_Jt.m_jacobianM1.m_linear.m_w == dgFloat32(0.0f));
			dgAssert(row->m_Jt.m_jacobianM1.m_angular.m_w == dgFloat32(0.0f));

			dgVector a(row->m_JMinv.m_jacobianM0.m_linear * linearM0 + row->m_JMinv.m_jacobianM0.m_angular * angularM0 +
				row->m_JMinv.m_jacobianM1.m_linear * linearM1 + row->m_JMinv.m_jacobianM1.m_angular * angularM1);
			a = dgVector(rhs->m_coordenateAccel - rhs->m_force * rhs->m_diagDamp) - a.AddHorizontal();

			dgVector f(rhs->m_force + rhs->m_invJinvMJt * a.GetScalar());
			dgAssert(rhs->m_normalForceIndex >= 0);
			dgAssert(rhs->m_normalForceIndex <= rowsCount);
			dgInt32 frictionIndex = rhs->m_normalForceIndex;

			dgFloat32 frictionNormal = normalForce[frictionIndex];
			dgVector lowerFrictionForce(frictionNormal * rhs->m_lowerBoundFrictionCoefficent);
			dgVector upperFrictionForce(frictionNormal * rhs->m_upperBoundFrictionCoefficent);

			a = a.AndNot((f > upperFrictionForce) | (f < lowerFrictionForce));
			f = f.GetMax(lowerFrictionForce).GetMin(upperFrictionForce);

			//accNorm = accNorm.GetMax(a.Abs());
			accNorm += a * a;
			dgVector deltaForce(f - dgVector(rhs->m_force));

			rhs->m_force = f.GetScalar();
			normalForce[k] = f.GetScalar();

			dgVector deltaforce0(preconditioner0 * deltaForce);
			dgVector deltaforce1(preconditioner1 * deltaForce);
			linearM0 += row->m_Jt.m_jacobianM0.m_linear * deltaforce0;
			angularM0 += row->m_Jt.m_jacobianM0.m_angular * deltaforce0;
			linearM1 += row->m_Jt.m_jacobianM1.m_linear * deltaforce1;
			angularM1 += row->m_Jt.m_jacobianM1.m_angular * deltaforce1;
		}

		dgVector maxAccel(accNorm);
		const dgFloat32 tol = dgFloat32(0.5f);
		const dgFloat32 tol2 = tol * tol;
		for (dgInt32 i = 0; (i < 4) && (maxAccel.GetScalar() > tol2); i++) {
			maxAccel = dgVector::m_zero;
			for (dgInt32 k = 0; k < rowsCount; k++) {
				dgRightHandSide* const rhs = &rightHandSide[index + k];
				const dgLeftHandSide* const row = &matrixRow[index + k];
				dgVector a(row->m_JMinv.m_jacobianM0.m_linear * linearM0 + row->m_JMinv.m_jacobianM0.m_angular * angularM0 +
						   row->m_JMinv.m_jacobianM1.m_linear * linearM1 + row->m_JMinv.m_jacobianM1.m_angular * angularM1);
				a = dgVector(rhs->m_coordenateAccel - rhs->m_force * rhs->m_diagDamp) - a.AddHorizontal();

				dgVector f(rhs->m_force + rhs->m_invJinvMJt * a.GetScalar());
				dgAssert(rhs->m_normalForceIndex >= 0);
				dgAssert(rhs->m_normalForceIndex <= rowsCount);
				dgInt32 frictionIndex = rhs->m_normalForceIndex;

				dgFloat32 frictionNormal = normalForce[frictionIndex];
				dgVector lowerFrictionForce(frictionNormal * rhs->m_lowerBoundFrictionCoefficent);
				dgVector upperFrictionForce(frictionNormal * rhs->m_upperBoundFrictionCoefficent);

				a = a.AndNot((f > upperFrictionForce) | (f < lowerFrictionForce));
				f = f.GetMax(lowerFrictionForce).GetMin(upperFrictionForce);
				maxAccel = a * a;

				dgVector deltaForce(f - dgVector(rhs->m_force));

				rhs->m_force = f.GetScalar();
				normalForce[k] = f.GetScalar();

				dgVector deltaforce0(preconditioner0 * deltaForce);
				dgVector deltaforce1(preconditioner1 * deltaForce);
				linearM0 += row->m_Jt.m_jacobianM0.m_linear * deltaforce0;
				angularM0 += row->m_Jt.m_jacobianM0.m_angular * deltaforce0;
				linearM1 += row->m_Jt.m_jacobianM1.m_linear * deltaforce1;
				angularM1 += row->m_Jt.m_jacobianM1.m_angular * deltaforce1;
			}
		}

		for (dgInt32 i = 0; i < rowsCount; i++) {
			dgRightHandSide* const rhs = &rightHandSide[index + i];
			//dgLeftHandSide* const row = &matrixRow[index];
			rhs->m_maxImpact = dgMax(dgAbs(rhs->m_force), rhs->m_maxImpact);
		}
	}
	return accNorm.GetScalar();
}

void dgWorldDynamicUpdate::CalculateForcesParallel(dgParallelSolverSyncData* const syncData) const
{
	dgWorld* const world = (dgWorld*) this;
	const dgInt32 threadCounts = world->GetThreadCount();

	const dgInt32 passes = syncData->m_passes;
	const dgInt32 rkSubSteps = syncData->m_rkSubSteps;
	syncData->m_firstPassCoef = dgFloat32(0.0f);

	const dgInt32 bodyCount = syncData->m_bodyCount;
	dgJacobian* const internalForces = &world->m_solverMemory.m_internalForcesBuffer[0];

	for (dgInt32 step = 0; step < rkSubSteps; step++) {

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


//****************************************************************************************
//
//
//****************************************************************************************

dgParallelBodySolver::dgParallelBodySolver(dgMemoryAllocator* const allocator)
	:m_weigh(allocator)
	,m_invWeigh(allocator)
	,m_veloc(allocator)
	,m_veloc0(allocator)
	,m_rotation(allocator)
//	,m_angularDamp(allocator)
//	,m_linearDamp(allocator)
	,m_internalForces(allocator)
	,m_world(NULL)
	,m_cluster(NULL)
	,m_bodyArray(NULL)
	,m_jointArray(NULL)
	,m_timestep(dgFloat32 (0.0f))
	,m_count(0)
	,m_atomicIndex(0)
{
}

dgParallelBodySolver::~dgParallelBodySolver()
{
}

void dgParallelBodySolver::Reserve (dgInt32 count)
{
	m_count = ((count + DG_WORK_GROUP_SIZE - 1) & -(DG_WORK_GROUP_SIZE - 1)) >> DG_WORK_GROUP_BITS;

	m_weigh.ResizeIfNecessary(m_count);
	m_invWeigh.ResizeIfNecessary(m_count);
	m_veloc.ResizeIfNecessary(m_count);
	m_veloc0.ResizeIfNecessary(m_count);
	m_rotation.ResizeIfNecessary(m_count);
	m_internalForces.ResizeIfNecessary(m_count);

//	m_localInvInertia.Reserve(m_count);
//	m_invMass.Reserve(m_count);
//	m_invInertia.Reserve(m_count);
}

void dgParallelBodySolver::CalculateJointForces(dgBodyCluster& cluster, dgBodyInfo* const bodyArray, dgJointInfo* const jointArray, dgFloat32 timestep)
{
	m_cluster = &cluster;
	m_bodyArray = bodyArray;
	m_jointArray = jointArray;
	m_timestep = timestep;
	Reserve(cluster.m_bodyCount);
	
	const dgInt32 bodyCount = m_count * DG_WORK_GROUP_SIZE;
	for (dgInt32 i = cluster.m_bodyCount; i < bodyCount; i++) {
		bodyArray[i] = bodyArray[0];
	}

	dgFloat32* const weights = &m_weigh[0][0];
	memset(weights, 0, m_count * sizeof (dgWorkGroupFloat));

	m_atomicIndex = 0;
	const dgInt32 threadCounts = m_world->GetThreadCount();
	for (dgInt32 i = 0; i < threadCounts; i++) {
		m_world->QueueJob(InitWeightKernel, this, NULL);
	}
	m_world->SynchronizationBarrier();

	m_atomicIndex = 0;
	for (dgInt32 i = 0; i < threadCounts; i++) {
		m_world->QueueJob(InitInvWeightKernel, this, NULL);
	}
	m_world->SynchronizationBarrier();

	m_atomicIndex = 0;
	for (dgInt32 i = 0; i < threadCounts; i++) {
		m_world->QueueJob(InitBodyArrayKernel, this, NULL);
	}
	m_world->SynchronizationBarrier();
}

void dgParallelBodySolver::InitWeightKernel (void* const context, void* const, dgInt32 threadID)
{
	dgParallelBodySolver* const me = (dgParallelBodySolver*)context;
	me->InitWeights();
}

void dgParallelBodySolver::InitBodyArrayKernel(void* const context, void* const, dgInt32 threadID)
{
	dgParallelBodySolver* const me = (dgParallelBodySolver*)context;
	me->InitInvWeights();
}

void dgParallelBodySolver::InitInvWeightKernel(void* const context, void* const, dgInt32 threadID)
{
	dgParallelBodySolver* const me = (dgParallelBodySolver*)context;
	me->InityBodyArray();
}

void dgParallelBodySolver::InitWeights()
{
	dgFloat32* const weights = &m_weigh[0][0];
	const dgInt32 jointCount = m_cluster->m_jointCount;
	for (dgInt32 i = dgAtomicExchangeAndAdd(&m_atomicIndex, 1); i < jointCount;  i = dgAtomicExchangeAndAdd(&m_atomicIndex, 1)) {
		dgJointInfo* const jointInfo = &m_jointArray[i];
		const dgInt32 m0 = jointInfo->m_m0;
		const dgInt32 m1 = jointInfo->m_m1;
		dgBody* const body0 = jointInfo->m_joint->GetBody0();
		dgBody* const body1 = jointInfo->m_joint->GetBody1();
		{
			dgScopeSpinLock lock(&body0->m_criticalSectionLock);
			weights[m0] += dgFloat32(1.0f);
		}
		{
			dgScopeSpinLock lock(&body1->m_criticalSectionLock);
			weights[m1] += dgFloat32(1.0f);
		}
	}
}

void dgParallelBodySolver::InitInvWeights()
{
	const dgInt32 bodyCount = m_count * DG_WORK_GROUP_SIZE;
	dgFloat32* const weights = &m_weigh[0][0];
	dgFloat32* const invWeights = &m_invWeigh[0][0];
	for (dgInt32 i = dgAtomicExchangeAndAdd(&m_atomicIndex, DG_WORK_GROUP_SIZE); i < bodyCount;  i = dgAtomicExchangeAndAdd(&m_atomicIndex, DG_WORK_GROUP_SIZE)) {
		for (dgInt32 j = 0; j < DG_WORK_GROUP_SIZE; j ++) {
			const dgFloat32 w = weights[j + i] ? weights[j + i] : 1.0f;
			weights[j + i] = w;
			invWeights[j + i] = 1.0f / w;
		}
	}
}


void dgParallelBodySolver::InityBodyArray()
{
/*
//	syncData->m_atomicIndex = 0;
	const dgInt32 threadCounts = m_world->GetThreadCount();	
	for (dgInt32 i = 0; i < threadCounts; i++) {
		m_world->QueueJob(CalculateJointsAccelParallelKernel, this, NULL);
	}
	m_world->SynchronizationBarrier();
*/

	const dgInt32 bodyCount = m_count * DG_WORK_GROUP_SIZE;
	dgWorkGroupVector3 zero(dgWorkGroupFloat::m_zero, dgWorkGroupFloat::m_zero, dgWorkGroupFloat::m_zero);
	for (dgInt32 i = dgAtomicExchangeAndAdd(&m_atomicIndex, DG_WORK_GROUP_SIZE); i < bodyCount; i = dgAtomicExchangeAndAdd(&m_atomicIndex, DG_WORK_GROUP_SIZE)) {
		dgBodyInfo* const bodyArray = &m_bodyArray[i];
		dgDynamicBody* const body0 = (dgDynamicBody*)bodyArray[0].m_body;
		dgDynamicBody* const body1 = (dgDynamicBody*)bodyArray[1].m_body;
		dgDynamicBody* const body2 = (dgDynamicBody*)bodyArray[2].m_body;
		dgDynamicBody* const body3 = (dgDynamicBody*)bodyArray[3].m_body;
		dgDynamicBody* const body4 = (dgDynamicBody*)bodyArray[4].m_body;
		dgDynamicBody* const body5 = (dgDynamicBody*)bodyArray[7].m_body;
		dgDynamicBody* const body6 = (dgDynamicBody*)bodyArray[6].m_body;
		dgDynamicBody* const body7 = (dgDynamicBody*)bodyArray[7].m_body;

		m_rotation[i] = dgWorkGroupMatrix3x3(
			body0->GetMatrix(), body1->GetMatrix(), body2->GetMatrix(), body3->GetMatrix(),
			body4->GetMatrix(), body5->GetMatrix(), body6->GetMatrix(), body7->GetMatrix());

		dgWorkGroupFloat damp_ang_x(body0->GetDampCoeffcient(m_timestep), body4->GetDampCoeffcient(m_timestep));
		dgWorkGroupFloat damp_ang_y(body1->GetDampCoeffcient(m_timestep), body5->GetDampCoeffcient(m_timestep));
		dgWorkGroupFloat damp_ang_z(body2->GetDampCoeffcient(m_timestep), body6->GetDampCoeffcient(m_timestep));
		dgWorkGroupFloat damp_linear(body3->GetDampCoeffcient(m_timestep), body7->GetDampCoeffcient(m_timestep));
		dgWorkGroupFloat::Transpose4x8(damp_ang_x, damp_ang_y, damp_ang_z, damp_linear);

		dgWorkGroupVector3 omega(
			body0->GetOmega(), body1->GetOmega(), body2->GetOmega(), body3->GetOmega(),
			body4->GetOmega(), body5->GetOmega(), body6->GetOmega(), body7->GetOmega());
		m_veloc[i].m_angular = m_rotation[i].RotateVector(m_rotation[i].UnrotateVector(omega) * dgWorkGroupVector3(damp_ang_x, damp_ang_y, damp_ang_z));

		dgWorkGroupVector3 veloc(
			body0->GetVelocity(), body1->GetVelocity(), body2->GetVelocity(), body3->GetVelocity(),
			body4->GetVelocity(), body5->GetVelocity(), body6->GetVelocity(), body7->GetVelocity());
		m_veloc[i].m_linear = veloc.Scale(damp_linear);

		m_veloc0[i] = m_veloc[i];

//		dgMatrix3x3Avx invInertia(m_rotation[index].Transposed());
//		invInertia.m_front = invInertia.m_front * m_localInvInertia[index];
//		invInertia.m_up = invInertia.m_up * m_localInvInertia[index];
//		invInertia.m_right = invInertia.m_right * m_localInvInertia[index];
//		invInertia = invInertia * m_rotation[index];
//		invInertia.Store(&m_invInertia[index]);

		m_internalForces[i].m_linear = zero;
		m_internalForces[i].m_angular = zero;
	}
}