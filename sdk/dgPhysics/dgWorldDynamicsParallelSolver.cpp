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

#if (DG_WORK_GROUP_SIZE > 4)
dgWorkGroupFloat dgWorkGroupFloat::m_one(dgVector::m_one, dgVector::m_one);
dgWorkGroupFloat dgWorkGroupFloat::m_zero(dgVector::m_zero, dgVector::m_zero);
#else
dgWorkGroupFloat dgWorkGroupFloat::m_one(dgVector::m_one);
dgWorkGroupFloat dgWorkGroupFloat::m_zero(dgVector::m_zero);
#endif

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
	}

	dgInt32 atomicIndex = 0;
	for (dgInt32 i = dgAtomicExchangeAndAdd(&atomicIndex, 1); i < clustersCount; i = dgAtomicExchangeAndAdd(&atomicIndex, 1)) {
		world->IntegrateVelocity(&clusterArray[i], DG_SOLVER_MAX_ERROR, timestep, 0);
	}
}

dgBodyCluster dgWorldDynamicUpdate::MergeClusters(const dgBodyCluster* const clusterArray, dgInt32 clustersCount) const
{
	dgBodyCluster cluster;

	dgWorld* const world = (dgWorld*) this;
	dgInt32 bodyCount = 0;
	dgInt32 jointsCount = 0;
	for (dgInt32 i = 0; i < clustersCount; i++) {
		const dgBodyCluster* const srcCluster = &clusterArray[i];
		bodyCount += srcCluster->m_bodyCount - 1;
		jointsCount += srcCluster->m_jointCount;
	}

//	world->m_bodiesMemory.ResizeIfNecessary((m_bodies + bodyCount + DG_WORK_GROUP_SIZE + 1) * sizeof(dgBodyInfo));
	world->m_bodiesMemory.ResizeIfNecessary((m_bodies + bodyCount + 1) * sizeof(dgBodyInfo));
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

dgInt32 dgParallelBodySolver::CompareJointInfos(const dgJointInfo* const infoA, const dgJointInfo* const infoB, void* notUsed)
{
	const dgInt32 restingA = (infoA->m_joint->m_body0->m_resting & infoA->m_joint->m_body1->m_resting) ? 1 : 0;
	const dgInt32 restingB = (infoB->m_joint->m_body0->m_resting & infoB->m_joint->m_body1->m_resting) ? 1 : 0;

	const dgInt32 countA = infoA->m_pairCount * 2 + restingA;
	const dgInt32 countB = infoB->m_pairCount * 2 + restingB;

	if (countA < countB) {
		return 1;
	}
	if (countA > countB) {
		return -1;
	}
	return 0;
}

void dgParallelBodySolver::CalculateJointForces(const dgBodyCluster& cluster, dgBodyInfo* const bodyArray, dgJointInfo* const jointArray, dgFloat32 timestep)
{
	m_cluster = &cluster;
	m_bodyArray = bodyArray;
	m_jointArray = jointArray;
	m_timestep = timestep;
	m_invTimestep = (timestep > dgFloat32(0.0f)) ? dgFloat32(1.0f) / timestep : dgFloat32(0.0f);

	m_invStepRK = dgFloat32 (0.25f);
	m_timestepRK = m_timestep * m_invStepRK;
	m_invTimestepRK = m_invTimestep * dgFloat32 (4.0f);

	m_threadCounts = m_world->GetThreadCount();
	m_solverPasses = m_world->GetSolverMode();

	m_weight = dgAlloca(dgFloat32, cluster.m_bodyCount);
	m_invWeight = dgAlloca(dgFloat32, cluster.m_bodyCount);
#ifdef DG_SOLVER_USES_SOA
	m_soaRowStart = dgAlloca(dgInt32, cluster.m_jointCount / DG_SOLVER_USES_SOA + 1);
#endif

	InitWeights();
	InitBodyArray();
	InitJacobianMatrix();
	CalculateForces();
}

void dgParallelBodySolver::InitWeightKernel(void* const context, void* const, dgInt32 threadID)
{
	dgParallelBodySolver* const me = (dgParallelBodySolver*)context;
	me->InitWeights(threadID);
}

void dgParallelBodySolver::InitBodyArrayKernel(void* const context, void* const, dgInt32 threadID)
{
	dgParallelBodySolver* const me = (dgParallelBodySolver*)context;
	me->InitBodyArray(threadID);
}

void dgParallelBodySolver::InitJacobianMatrixKernel(void* const context, void* const, dgInt32 threadID)
{
	dgParallelBodySolver* const me = (dgParallelBodySolver*)context;
	me->InitJacobianMatrix(threadID);
}

void dgParallelBodySolver::CalculateJointsAccelerationKernel(void* const context, void* const, dgInt32 threadID)
{
	dgParallelBodySolver* const me = (dgParallelBodySolver*)context;
	me->CalculateJointsAcceleration(threadID);
}

void dgParallelBodySolver::CalculateJointsForceKernel(void* const context, void* const, dgInt32 threadID)
{
	dgParallelBodySolver* const me = (dgParallelBodySolver*)context;
	me->CalculateJointsForce(threadID);
}

void dgParallelBodySolver::CalculateBodyForceKernel(void* const context, void* const, dgInt32 threadID)
{
	dgParallelBodySolver* const me = (dgParallelBodySolver*)context;
	me->CalculateBodyForce(threadID);
}

void dgParallelBodySolver::IntegrateBodiesVelocityKernel(void* const context, void* const worldContext, dgInt32 threadID)
{
	dgParallelBodySolver* const me = (dgParallelBodySolver*)context;
	me->IntegrateBodiesVelocity(threadID);
}

void dgParallelBodySolver::CalculateBodiesAccelerationKernel(void* const context, void* const, dgInt32 threadID)
{
	dgParallelBodySolver* const me = (dgParallelBodySolver*)context;
	me->CalculateBodiesAcceleration(threadID);
}

void dgParallelBodySolver::UpdateForceFeedbackKernel(void* const context, void* const, dgInt32 threadID)
{
	dgParallelBodySolver* const me = (dgParallelBodySolver*)context;
	me->UpdateForceFeedback(threadID);
}

void dgParallelBodySolver::UpdateKinematicFeedbackKernel(void* const context, void* const, dgInt32 threadID)
{
	dgParallelBodySolver* const me = (dgParallelBodySolver*)context;
	me->UpdateKinematicFeedback(threadID);
}

void dgParallelBodySolver::TransposeMassMatrixKernel(void* const context, void* const, dgInt32 threadID)
{
	dgParallelBodySolver* const me = (dgParallelBodySolver*)context;
	me->TransposeMassMatrix(threadID);
}

void dgParallelBodySolver::UpdateRowAccelerationKernel(void* const context, void* const, dgInt32 threadID)
{
	dgParallelBodySolver* const me = (dgParallelBodySolver*)context;
	me->UpdateRowAcceleration(threadID);
}

void dgParallelBodySolver::InitWeights()
{
	m_atomicIndex = 0;
	memset(m_weight, 0, m_cluster->m_bodyCount * sizeof(dgFloat32));
	for (dgInt32 i = 0; i < m_threadCounts; i++) {
		m_world->QueueJob(InitWeightKernel, this, NULL);
	}
	m_world->SynchronizationBarrier();
	m_weight[0] = dgFloat32(1.0f);
}

void dgParallelBodySolver::InitBodyArray()
{
	m_atomicIndex = 1;
	dgJacobian* const internalForces = &m_world->m_solverMemory.m_internalForcesBuffer[0];
	memset(internalForces, 0, m_cluster->m_bodyCount * sizeof(dgJacobian));
	for (dgInt32 i = 0; i < m_threadCounts; i++) {
		m_world->QueueJob(InitBodyArrayKernel, this, NULL);
	}
	m_world->SynchronizationBarrier();
}

void dgParallelBodySolver::InitJacobianMatrix()
{
	m_atomicIndex = 0;
	m_jacobianMatrixRowAtomicIndex = 0;
	for (dgInt32 i = 0; i < m_threadCounts; i++) {
		m_world->QueueJob(InitJacobianMatrixKernel, this, NULL);
	}
	m_world->SynchronizationBarrier();

	dgJointInfo* const jointArray = m_jointArray;
	dgSort(jointArray, m_cluster->m_jointCount, CompareJointInfos);

	m_jointCount = ((m_cluster->m_jointCount + DG_WORK_GROUP_SIZE - 1) & -dgInt32(DG_WORK_GROUP_SIZE - 1)) / DG_WORK_GROUP_SIZE;
	const dgInt32 jointCount = m_jointCount * DG_WORK_GROUP_SIZE;
	for (dgInt32 i = m_cluster->m_jointCount; i < jointCount; i++) {
		memset(&jointArray[i], 0, sizeof(dgJointInfo));
	}

int xxx = sizeof (dgSolverSoaElement);
xxx*=1;

#ifdef DG_SOLVER_USES_SOA
	dgInt32 size = 0;
	for (dgInt32 i = 0; i < jointCount; i+= DG_WORK_GROUP_SIZE) {
		size += jointArray[i].m_pairCount;
	}
	m_massMatrix.ResizeIfNecessary(size);

	m_atomicIndex = 0;
	m_soaRowsCount = 0;
	for (dgInt32 i = 0; i < m_threadCounts; i++) {
		m_world->QueueJob(TransposeMassMatrixKernel, this, NULL);
	}
	m_world->SynchronizationBarrier();
#endif
}

void dgParallelBodySolver::CalculateJointsAcceleration()
{
	m_atomicIndex = 0;
	for (dgInt32 i = 0; i < m_threadCounts; i++) {
		m_world->QueueJob(CalculateJointsAccelerationKernel, this, NULL);
	}
	m_world->SynchronizationBarrier();
	m_firstPassCoef = dgFloat32(1.0f);

#ifdef DG_SOLVER_USES_SOA
	m_atomicIndex = 0;
	for (dgInt32 i = 0; i < m_threadCounts; i++) {
		m_world->QueueJob(UpdateRowAccelerationKernel, this, NULL);
	}
	m_world->SynchronizationBarrier();
#endif
}

void dgParallelBodySolver::CalculateBodiesAcceleration()
{
	m_atomicIndex = 1;
	for (dgInt32 i = 0; i < m_threadCounts; i++) {
		m_world->QueueJob(CalculateBodiesAccelerationKernel, this, NULL);
	}
	m_world->SynchronizationBarrier();
}


void dgParallelBodySolver::CalculateJointsForce()
{
	m_atomicIndex = 0;
	for (dgInt32 i = 0; i < m_threadCounts; i++) {
		m_world->QueueJob(CalculateJointsForceKernel, this, NULL);
	}
	m_world->SynchronizationBarrier();
}

void dgParallelBodySolver::CalculateBodyForce()
{
	m_atomicIndex = 0;
	dgJacobian* const internalForces = &m_world->m_solverMemory.m_internalForcesBuffer[0];
	memset(internalForces, 0, m_cluster->m_bodyCount * sizeof(dgJacobian));
	for (dgInt32 i = 0; i < m_threadCounts; i++) {
		m_world->QueueJob(CalculateBodyForceKernel, this, NULL);
	}
	m_world->SynchronizationBarrier();
}

void dgParallelBodySolver::IntegrateBodiesVelocity()
{
	m_atomicIndex = 1;
	for (dgInt32 i = 0; i < m_threadCounts; i++) {
		m_world->QueueJob(IntegrateBodiesVelocityKernel, this, NULL);
	}
	m_world->SynchronizationBarrier();
}


void dgParallelBodySolver::UpdateForceFeedback()
{
	m_atomicIndex = 0;
	for (dgInt32 i = 0; i < m_threadCounts; i++) {
		m_world->QueueJob(UpdateForceFeedbackKernel, this, NULL);
	}
	m_world->SynchronizationBarrier();
}

void dgParallelBodySolver::UpdateKinematicFeedback()
{
	m_atomicIndex = 0;
	for (dgInt32 i = 0; i < m_threadCounts; i++) {
		m_world->QueueJob(UpdateKinematicFeedbackKernel, this, NULL);
	}
	m_world->SynchronizationBarrier();
}

void dgParallelBodySolver::InitWeights(dgInt32 threadID)
{
	dgFloat32* const weight = m_weight;
	const dgJointInfo* const jointArray = m_jointArray;
	const dgInt32 jointCount = m_cluster->m_jointCount;
	for (dgInt32 i = dgAtomicExchangeAndAdd(&m_atomicIndex, 1); i < jointCount; i = dgAtomicExchangeAndAdd(&m_atomicIndex, 1)) {
		const dgJointInfo* const jointInfo = &jointArray[i];
		const dgInt32 m0 = jointInfo->m_m0;
		const dgInt32 m1 = jointInfo->m_m1;
		dgBody* const body0 = jointInfo->m_joint->GetBody0();
		dgBody* const body1 = jointInfo->m_joint->GetBody1();
		if (m0) {
			dgScopeSpinLock lock(&body0->m_criticalSectionLock);
			weight[m0] += dgFloat32(1.0f);
		}
		if (m1) {
			dgScopeSpinLock lock(&body1->m_criticalSectionLock);
			weight[m1] += dgFloat32(1.0f);
		}
	}
}

void dgParallelBodySolver::InitBodyArray(dgInt32 threadID)
{
	dgFloat32* const weight = m_weight;
	dgFloat32* const invWeight = m_invWeight;
	const dgBodyInfo* const bodyArray = m_bodyArray;

	const dgInt32 bodyCount = m_cluster->m_bodyCount;
	for (dgInt32 i = dgAtomicExchangeAndAdd(&m_atomicIndex, 1); i < bodyCount; i = dgAtomicExchangeAndAdd(&m_atomicIndex, 1)) {
		const dgBodyInfo* const bodyInfo = &bodyArray[i];
		dgBody* const body = (dgDynamicBody*)bodyInfo->m_body;
		body->AddDampingAcceleration(m_timestep);
		body->CalcInvInertiaMatrix();

		body->m_accel = body->m_veloc;
		body->m_alpha = body->m_omega;

		const dgFloat32 w = weight[i] ? weight[i] : dgFloat32(1.0f);
		weight[i] = w;
		invWeight[i] = dgFloat32 (1.0f) / w;
	}
}

void dgParallelBodySolver::CalculateJointsAcceleration(dgInt32 threadID)
{
	dgJointAccelerationDecriptor joindDesc;
	joindDesc.m_timeStep = m_timestepRK;
	joindDesc.m_invTimeStep = m_invTimestepRK;
	joindDesc.m_firstPassCoefFlag = m_firstPassCoef;
	const dgInt32 jointCount = m_cluster->m_jointCount;
	dgRightHandSide* const rightHandSide = &m_world->m_solverMemory.m_righHandSizeBuffer[0];
	const dgLeftHandSide* const leftHandSide = &m_world->m_solverMemory.m_leftHandSizeBuffer[0];

	for (dgInt32 i = dgAtomicExchangeAndAdd(&m_atomicIndex, 1); i < jointCount; i = dgAtomicExchangeAndAdd(&m_atomicIndex, 1)) {
		dgJointInfo* const jointInfo = &m_jointArray[i];
		dgConstraint* const constraint = jointInfo->m_joint;
		const dgInt32 pairStart = jointInfo->m_pairStart;
		joindDesc.m_rowsCount = jointInfo->m_pairCount;
		joindDesc.m_leftHandSide = &leftHandSide[pairStart];
		joindDesc.m_rightHandSide = &rightHandSide[pairStart];

		constraint->JointAccelerations(&joindDesc);
	}
}

#ifdef DG_SOLVER_USES_SOA
dgFloat32 dgParallelBodySolver::CalculateJointForce(const dgJointInfo* const jointInfo, dgSolverSoaElement* const massMatrix, const dgJacobian* const internalForces) const
{
	dgWorkGroupVector6 forceM0;
	dgWorkGroupVector6 forceM1;
	dgWorkGroupFloat preconditioner0;
	dgWorkGroupFloat preconditioner1;
	dgWorkGroupFloat accNorm(dgWorkGroupFloat::m_zero);
	dgWorkGroupFloat normalForce[DG_CONSTRAINT_MAX_ROWS + 1];

	bool isSleeping = true;
	const dgBodyInfo* const bodyArray = m_bodyArray;
	for (dgInt32 i = 0; (i < DG_WORK_GROUP_SIZE) && isSleeping; i++) {
		const dgInt32 m0 = jointInfo[i].m_m0;
		const dgInt32 m1 = jointInfo[i].m_m1;
		const dgBody* const body0 = bodyArray[m0].m_body;
		const dgBody* const body1 = bodyArray[m1].m_body;
		isSleeping &= body0->m_resting;
		isSleeping &= body1->m_resting;
	}

	if (!isSleeping) {
		const dgFloat32* const weight = m_weight;
		for (dgInt32 i = 0; i < DG_WORK_GROUP_SIZE; i++) {
			const dgInt32 m0 = jointInfo[i].m_m0;
			const dgInt32 m1 = jointInfo[i].m_m1;

			forceM0.m_linear.m_x[i] = internalForces[m0].m_linear.m_x;
			forceM0.m_linear.m_y[i] = internalForces[m0].m_linear.m_y;
			forceM0.m_linear.m_z[i] = internalForces[m0].m_linear.m_z;
			forceM0.m_angular.m_x[i] = internalForces[m0].m_angular.m_x;
			forceM0.m_angular.m_y[i] = internalForces[m0].m_angular.m_y;
			forceM0.m_angular.m_z[i] = internalForces[m0].m_angular.m_z;

			forceM1.m_linear.m_x[i] = internalForces[m1].m_linear.m_x;
			forceM1.m_linear.m_y[i] = internalForces[m1].m_linear.m_y;
			forceM1.m_linear.m_z[i] = internalForces[m1].m_linear.m_z;
			forceM1.m_angular.m_x[i] = internalForces[m1].m_angular.m_x;
			forceM1.m_angular.m_y[i] = internalForces[m1].m_angular.m_y;
			forceM1.m_angular.m_z[i] = internalForces[m1].m_angular.m_z;

			preconditioner0[i] = jointInfo[i].m_preconditioner0 * weight[m0];
			preconditioner1[i] = jointInfo[i].m_preconditioner1 * weight[m1];
		}
#if 0
		const dgInt32 rowsCount = jointInfo->m_pairCount;
		normalForce[0] = dgWorkGroupFloat::m_one;
		for (dgInt32 j = 0; j < rowsCount; j++) {
			dgSolverSoaElement* const row = &massMatrix[j];
			dgWorkGroupFloat a(
				(row->m_JMinv.m_jacobianM0.m_linear.m_x * forceM0.m_linear.m_x +
				row->m_JMinv.m_jacobianM0.m_linear.m_y * forceM0.m_linear.m_y +
				row->m_JMinv.m_jacobianM0.m_linear.m_z * forceM0.m_linear.m_z +
				row->m_JMinv.m_jacobianM0.m_angular.m_x * forceM0.m_angular.m_x +
				row->m_JMinv.m_jacobianM0.m_angular.m_y * forceM0.m_angular.m_y +
				row->m_JMinv.m_jacobianM0.m_angular.m_z * forceM0.m_angular.m_z) *
				preconditioner0 +
				(row->m_JMinv.m_jacobianM1.m_linear.m_x * forceM1.m_linear.m_x +
				row->m_JMinv.m_jacobianM1.m_linear.m_y * forceM1.m_linear.m_y +
				row->m_JMinv.m_jacobianM1.m_linear.m_z * forceM1.m_linear.m_z +
				row->m_JMinv.m_jacobianM1.m_angular.m_x * forceM1.m_angular.m_x +
				row->m_JMinv.m_jacobianM1.m_angular.m_y * forceM1.m_angular.m_y +
				row->m_JMinv.m_jacobianM1.m_angular.m_z * forceM1.m_angular.m_z) *
				preconditioner1);

			a = row->m_coordenateAccel - row->m_force * row->m_diagDamp - a;
			dgWorkGroupFloat f (row->m_force + row->m_invJinvMJt * a);

			dgWorkGroupFloat frictionNormal;
			for (dgInt32 k = 0; k < DG_WORK_GROUP_SIZE; k++) {
				dgAssert(row->m_normalForceIndex.m_i[k] >= -1);
				dgAssert(row->m_normalForceIndex.m_i[k] <= rowsCount);
				const dgInt32 frictionIndex = dgInt32(row->m_normalForceIndex.m_i[k] + 1);
				frictionNormal[k] = normalForce[frictionIndex][k];
			}

			dgWorkGroupFloat lowerFrictionForce(frictionNormal * row->m_lowerBoundFrictionCoefficent);
			dgWorkGroupFloat upperFrictionForce(frictionNormal * row->m_upperBoundFrictionCoefficent);

			a = a.AndNot((f > upperFrictionForce) | (f < lowerFrictionForce));
			f = f.GetMax(lowerFrictionForce).GetMin(upperFrictionForce);

			accNorm = accNorm + a * a;
			dgWorkGroupFloat deltaForce(f - row->m_force);

			row->m_force = f;
			normalForce[j + 1] = f;

			forceM0.m_linear.m_x = forceM0.m_linear.m_x + row->m_Jt.m_jacobianM0.m_linear.m_x * deltaForce;
			forceM0.m_linear.m_y = forceM0.m_linear.m_y + row->m_Jt.m_jacobianM0.m_linear.m_y * deltaForce;
			forceM0.m_linear.m_z = forceM0.m_linear.m_z + row->m_Jt.m_jacobianM0.m_linear.m_z * deltaForce;
			forceM0.m_angular.m_x = forceM0.m_angular.m_x + row->m_Jt.m_jacobianM0.m_angular.m_x * deltaForce;
			forceM0.m_angular.m_y = forceM0.m_angular.m_y + row->m_Jt.m_jacobianM0.m_angular.m_y * deltaForce;
			forceM0.m_angular.m_z = forceM0.m_angular.m_z + row->m_Jt.m_jacobianM0.m_angular.m_z * deltaForce;

			forceM1.m_linear.m_x = forceM1.m_linear.m_x + row->m_Jt.m_jacobianM1.m_linear.m_x * deltaForce;
			forceM1.m_linear.m_y = forceM1.m_linear.m_y + row->m_Jt.m_jacobianM1.m_linear.m_y * deltaForce;
			forceM1.m_linear.m_z = forceM1.m_linear.m_z + row->m_Jt.m_jacobianM1.m_linear.m_z * deltaForce;
			forceM1.m_angular.m_x = forceM1.m_angular.m_x + row->m_Jt.m_jacobianM1.m_angular.m_x * deltaForce;
			forceM1.m_angular.m_y = forceM1.m_angular.m_y + row->m_Jt.m_jacobianM1.m_angular.m_y * deltaForce;
			forceM1.m_angular.m_z = forceM1.m_angular.m_z + row->m_Jt.m_jacobianM1.m_angular.m_z * deltaForce;
		}

		const dgFloat32 tol = dgFloat32(0.5f);
		const dgFloat32 tol2 = tol * tol;
		dgWorkGroupFloat maxAccel(accNorm);
		for (dgInt32 i = 0; (i < 4) && (maxAccel.GetMax() > tol2); i++) {
			maxAccel = dgWorkGroupFloat::m_zero;
			for (dgInt32 j = 0; j < rowsCount; j++) {
				dgSolverSoaElement* const row = &massMatrix[j];
				dgWorkGroupFloat a(
					(row->m_JMinv.m_jacobianM0.m_linear.m_x * forceM0.m_linear.m_x +
					row->m_JMinv.m_jacobianM0.m_linear.m_y * forceM0.m_linear.m_y +
					row->m_JMinv.m_jacobianM0.m_linear.m_z * forceM0.m_linear.m_z +
					row->m_JMinv.m_jacobianM0.m_angular.m_x * forceM0.m_angular.m_x +
					row->m_JMinv.m_jacobianM0.m_angular.m_y * forceM0.m_angular.m_y +
					row->m_JMinv.m_jacobianM0.m_angular.m_z * forceM0.m_angular.m_z) *
					preconditioner0 +
					(row->m_JMinv.m_jacobianM1.m_linear.m_x * forceM1.m_linear.m_x +
					row->m_JMinv.m_jacobianM1.m_linear.m_y * forceM1.m_linear.m_y +
					row->m_JMinv.m_jacobianM1.m_linear.m_z * forceM1.m_linear.m_z +
					row->m_JMinv.m_jacobianM1.m_angular.m_x * forceM1.m_angular.m_x +
					row->m_JMinv.m_jacobianM1.m_angular.m_y * forceM1.m_angular.m_y +
					row->m_JMinv.m_jacobianM1.m_angular.m_z * forceM1.m_angular.m_z) *
					preconditioner1);

				a = row->m_coordenateAccel - row->m_force * row->m_diagDamp - a;
				dgWorkGroupFloat f (row->m_force + row->m_invJinvMJt * a);

				dgWorkGroupFloat frictionNormal;
				for (dgInt32 k = 0; k < DG_WORK_GROUP_SIZE; k++) {
					dgAssert(row->m_normalForceIndex.m_i[k] >= -1);
					dgAssert(row->m_normalForceIndex.m_i[k] <= rowsCount);
					const dgInt32 frictionIndex = dgInt32 (row->m_normalForceIndex.m_i[k] + 1);
					frictionNormal[k] = normalForce[frictionIndex][k];
				}

				dgWorkGroupFloat lowerFrictionForce(frictionNormal * row->m_lowerBoundFrictionCoefficent);
				dgWorkGroupFloat upperFrictionForce(frictionNormal * row->m_upperBoundFrictionCoefficent);

				a = a.AndNot((f > upperFrictionForce) | (f < lowerFrictionForce));
				f = f.GetMax(lowerFrictionForce).GetMin(upperFrictionForce);
				maxAccel = maxAccel + a * a;

				dgWorkGroupFloat deltaForce(f - row->m_force);

				row->m_force = f;
				normalForce[j + 1] = f;

				forceM0.m_linear.m_x = forceM0.m_linear.m_x + row->m_Jt.m_jacobianM0.m_linear.m_x * deltaForce;
				forceM0.m_linear.m_y = forceM0.m_linear.m_y + row->m_Jt.m_jacobianM0.m_linear.m_y * deltaForce;
				forceM0.m_linear.m_z = forceM0.m_linear.m_z + row->m_Jt.m_jacobianM0.m_linear.m_z * deltaForce;
				forceM0.m_angular.m_x = forceM0.m_angular.m_x + row->m_Jt.m_jacobianM0.m_angular.m_x * deltaForce;
				forceM0.m_angular.m_y = forceM0.m_angular.m_y + row->m_Jt.m_jacobianM0.m_angular.m_y * deltaForce;
				forceM0.m_angular.m_z = forceM0.m_angular.m_z + row->m_Jt.m_jacobianM0.m_angular.m_z * deltaForce;

				forceM1.m_linear.m_x = forceM1.m_linear.m_x + row->m_Jt.m_jacobianM1.m_linear.m_x * deltaForce;
				forceM1.m_linear.m_y = forceM1.m_linear.m_y + row->m_Jt.m_jacobianM1.m_linear.m_y * deltaForce;
				forceM1.m_linear.m_z = forceM1.m_linear.m_z + row->m_Jt.m_jacobianM1.m_linear.m_z * deltaForce;
				forceM1.m_angular.m_x = forceM1.m_angular.m_x + row->m_Jt.m_jacobianM1.m_angular.m_x * deltaForce;
				forceM1.m_angular.m_y = forceM1.m_angular.m_y + row->m_Jt.m_jacobianM1.m_angular.m_y * deltaForce;
				forceM1.m_angular.m_z = forceM1.m_angular.m_z + row->m_Jt.m_jacobianM1.m_angular.m_z * deltaForce;
			}
		}
#else
		forceM0.m_linear.m_x = forceM0.m_linear.m_x * preconditioner0;
		forceM0.m_linear.m_y = forceM0.m_linear.m_y * preconditioner0;
		forceM0.m_linear.m_z = forceM0.m_linear.m_z * preconditioner0;
		forceM0.m_angular.m_x = forceM0.m_angular.m_x * preconditioner0;
		forceM0.m_angular.m_y = forceM0.m_angular.m_y * preconditioner0;
		forceM0.m_angular.m_z = forceM0.m_angular.m_z * preconditioner0;

		forceM1.m_linear.m_x = forceM1.m_linear.m_x * preconditioner1;
		forceM1.m_linear.m_y = forceM1.m_linear.m_y * preconditioner1;
		forceM1.m_linear.m_z = forceM1.m_linear.m_z * preconditioner1;
		forceM1.m_angular.m_x = forceM1.m_angular.m_x * preconditioner1;
		forceM1.m_angular.m_y = forceM1.m_angular.m_y * preconditioner1;
		forceM1.m_angular.m_z = forceM1.m_angular.m_z * preconditioner1;

		const dgInt32 rowsCount = jointInfo->m_pairCount;
		normalForce[0] = dgWorkGroupFloat::m_one;
		for (dgInt32 j = 0; j < rowsCount; j++) {
			dgSolverSoaElement* const row = &massMatrix[j];
			dgWorkGroupFloat a(
				row->m_JMinv.m_jacobianM0.m_linear.m_x * forceM0.m_linear.m_x +
				row->m_JMinv.m_jacobianM0.m_linear.m_y * forceM0.m_linear.m_y +
				row->m_JMinv.m_jacobianM0.m_linear.m_z * forceM0.m_linear.m_z +
				row->m_JMinv.m_jacobianM0.m_angular.m_x * forceM0.m_angular.m_x +
				row->m_JMinv.m_jacobianM0.m_angular.m_y * forceM0.m_angular.m_y +
				row->m_JMinv.m_jacobianM0.m_angular.m_z * forceM0.m_angular.m_z +
				row->m_JMinv.m_jacobianM1.m_linear.m_x * forceM1.m_linear.m_x +
				row->m_JMinv.m_jacobianM1.m_linear.m_y * forceM1.m_linear.m_y +
				row->m_JMinv.m_jacobianM1.m_linear.m_z * forceM1.m_linear.m_z +
				row->m_JMinv.m_jacobianM1.m_angular.m_x * forceM1.m_angular.m_x +
				row->m_JMinv.m_jacobianM1.m_angular.m_y * forceM1.m_angular.m_y +
				row->m_JMinv.m_jacobianM1.m_angular.m_z * forceM1.m_angular.m_z);

			a = row->m_coordenateAccel - row->m_force * row->m_diagDamp - a;
			dgWorkGroupFloat f(row->m_force + row->m_invJinvMJt * a);

			dgWorkGroupFloat frictionNormal;
			for (dgInt32 k = 0; k < DG_WORK_GROUP_SIZE; k++) {
				dgAssert(row->m_normalForceIndex.m_i[k] >= -1);
				dgAssert(row->m_normalForceIndex.m_i[k] <= rowsCount);
				const dgInt32 frictionIndex = dgInt32(row->m_normalForceIndex.m_i[k] + 1);
				frictionNormal[k] = normalForce[frictionIndex][k];
			}

			dgWorkGroupFloat lowerFrictionForce(frictionNormal * row->m_lowerBoundFrictionCoefficent);
			dgWorkGroupFloat upperFrictionForce(frictionNormal * row->m_upperBoundFrictionCoefficent);

			a = a.AndNot((f > upperFrictionForce) | (f < lowerFrictionForce));
			f = f.GetMax(lowerFrictionForce).GetMin(upperFrictionForce);

			accNorm = accNorm + a * a;
			dgWorkGroupFloat deltaForce(f - row->m_force);

			row->m_force = f;
			normalForce[j + 1] = f;

			dgWorkGroupFloat deltaForce0(deltaForce * preconditioner0);
			dgWorkGroupFloat deltaForce1(deltaForce * preconditioner1);

			forceM0.m_linear.m_x = forceM0.m_linear.m_x + row->m_Jt.m_jacobianM0.m_linear.m_x * deltaForce0;
			forceM0.m_linear.m_y = forceM0.m_linear.m_y + row->m_Jt.m_jacobianM0.m_linear.m_y * deltaForce0;
			forceM0.m_linear.m_z = forceM0.m_linear.m_z + row->m_Jt.m_jacobianM0.m_linear.m_z * deltaForce0;
			forceM0.m_angular.m_x = forceM0.m_angular.m_x + row->m_Jt.m_jacobianM0.m_angular.m_x * deltaForce0;
			forceM0.m_angular.m_y = forceM0.m_angular.m_y + row->m_Jt.m_jacobianM0.m_angular.m_y * deltaForce0;
			forceM0.m_angular.m_z = forceM0.m_angular.m_z + row->m_Jt.m_jacobianM0.m_angular.m_z * deltaForce0;

			forceM1.m_linear.m_x = forceM1.m_linear.m_x + row->m_Jt.m_jacobianM1.m_linear.m_x * deltaForce1;
			forceM1.m_linear.m_y = forceM1.m_linear.m_y + row->m_Jt.m_jacobianM1.m_linear.m_y * deltaForce1;
			forceM1.m_linear.m_z = forceM1.m_linear.m_z + row->m_Jt.m_jacobianM1.m_linear.m_z * deltaForce1;
			forceM1.m_angular.m_x = forceM1.m_angular.m_x + row->m_Jt.m_jacobianM1.m_angular.m_x * deltaForce1;
			forceM1.m_angular.m_y = forceM1.m_angular.m_y + row->m_Jt.m_jacobianM1.m_angular.m_y * deltaForce1;
			forceM1.m_angular.m_z = forceM1.m_angular.m_z + row->m_Jt.m_jacobianM1.m_angular.m_z * deltaForce1;
		}

		const dgFloat32 tol = dgFloat32(0.5f);
		const dgFloat32 tol2 = tol * tol;
		dgWorkGroupFloat maxAccel(accNorm);
		for (dgInt32 i = 0; (i < 4) && (maxAccel.GetMax() > tol2); i++) {
			maxAccel = dgWorkGroupFloat::m_zero;
			for (dgInt32 j = 0; j < rowsCount; j++) {
				dgSolverSoaElement* const row = &massMatrix[j];
				dgWorkGroupFloat a(
					row->m_JMinv.m_jacobianM0.m_linear.m_x * forceM0.m_linear.m_x +
					row->m_JMinv.m_jacobianM0.m_linear.m_y * forceM0.m_linear.m_y +
					row->m_JMinv.m_jacobianM0.m_linear.m_z * forceM0.m_linear.m_z +
					row->m_JMinv.m_jacobianM0.m_angular.m_x * forceM0.m_angular.m_x +
					row->m_JMinv.m_jacobianM0.m_angular.m_y * forceM0.m_angular.m_y +
					row->m_JMinv.m_jacobianM0.m_angular.m_z * forceM0.m_angular.m_z +
					row->m_JMinv.m_jacobianM1.m_linear.m_x * forceM1.m_linear.m_x +
					row->m_JMinv.m_jacobianM1.m_linear.m_y * forceM1.m_linear.m_y +
					row->m_JMinv.m_jacobianM1.m_linear.m_z * forceM1.m_linear.m_z +
					row->m_JMinv.m_jacobianM1.m_angular.m_x * forceM1.m_angular.m_x +
					row->m_JMinv.m_jacobianM1.m_angular.m_y * forceM1.m_angular.m_y +
					row->m_JMinv.m_jacobianM1.m_angular.m_z * forceM1.m_angular.m_z);

				a = row->m_coordenateAccel - row->m_force * row->m_diagDamp - a;
				dgWorkGroupFloat f(row->m_force + row->m_invJinvMJt * a);

				dgWorkGroupFloat frictionNormal;
				for (dgInt32 k = 0; k < DG_WORK_GROUP_SIZE; k++) {
					dgAssert(row->m_normalForceIndex.m_i[k] >= -1);
					dgAssert(row->m_normalForceIndex.m_i[k] <= rowsCount);
					const dgInt32 frictionIndex = dgInt32(row->m_normalForceIndex.m_i[k] + 1);
					frictionNormal[k] = normalForce[frictionIndex][k];
				}

				dgWorkGroupFloat lowerFrictionForce(frictionNormal * row->m_lowerBoundFrictionCoefficent);
				dgWorkGroupFloat upperFrictionForce(frictionNormal * row->m_upperBoundFrictionCoefficent);

				a = a.AndNot((f > upperFrictionForce) | (f < lowerFrictionForce));
				f = f.GetMax(lowerFrictionForce).GetMin(upperFrictionForce);
				maxAccel = maxAccel + a * a;

				dgWorkGroupFloat deltaForce(f - row->m_force);

				row->m_force = f;
				normalForce[j + 1] = f;

				dgWorkGroupFloat deltaForce0(deltaForce * preconditioner0);
				dgWorkGroupFloat deltaForce1(deltaForce * preconditioner1);

				forceM0.m_linear.m_x = forceM0.m_linear.m_x + row->m_Jt.m_jacobianM0.m_linear.m_x * deltaForce0;
				forceM0.m_linear.m_y = forceM0.m_linear.m_y + row->m_Jt.m_jacobianM0.m_linear.m_y * deltaForce0;
				forceM0.m_linear.m_z = forceM0.m_linear.m_z + row->m_Jt.m_jacobianM0.m_linear.m_z * deltaForce0;
				forceM0.m_angular.m_x = forceM0.m_angular.m_x + row->m_Jt.m_jacobianM0.m_angular.m_x * deltaForce0;
				forceM0.m_angular.m_y = forceM0.m_angular.m_y + row->m_Jt.m_jacobianM0.m_angular.m_y * deltaForce0;
				forceM0.m_angular.m_z = forceM0.m_angular.m_z + row->m_Jt.m_jacobianM0.m_angular.m_z * deltaForce0;

				forceM1.m_linear.m_x = forceM1.m_linear.m_x + row->m_Jt.m_jacobianM1.m_linear.m_x * deltaForce1;
				forceM1.m_linear.m_y = forceM1.m_linear.m_y + row->m_Jt.m_jacobianM1.m_linear.m_y * deltaForce1;
				forceM1.m_linear.m_z = forceM1.m_linear.m_z + row->m_Jt.m_jacobianM1.m_linear.m_z * deltaForce1;
				forceM1.m_angular.m_x = forceM1.m_angular.m_x + row->m_Jt.m_jacobianM1.m_angular.m_x * deltaForce1;
				forceM1.m_angular.m_y = forceM1.m_angular.m_y + row->m_Jt.m_jacobianM1.m_angular.m_y * deltaForce1;
				forceM1.m_angular.m_z = forceM1.m_angular.m_z + row->m_Jt.m_jacobianM1.m_angular.m_z * deltaForce1;
			}
		}
#endif

/*
		for (dgInt32 i = 0; i < rowsCount; i++) {
			dgRightHandSide* const rhs = &rightHandSide[index + i];
			rhs->m_maxImpact = dgMax(dgAbs(rhs->m_force), rhs->m_maxImpact);
		}
*/
	}
	return accNorm.GetMax();
}

#else

dgFloat32 dgParallelBodySolver::CalculateJointForce(const dgJointInfo* const jointInfo, const dgLeftHandSide* const leftHandSide, dgRightHandSide* const rightHandSide, const dgJacobian* const internalForces) const
{
	dgVector accNorm(dgVector::m_zero);
	dgFloat32 normalForce[DG_CONSTRAINT_MAX_ROWS + 4];

	const dgInt32 m0 = jointInfo->m_m0;
	const dgInt32 m1 = jointInfo->m_m1;
	const dgBody* const body0 = m_bodyArray[m0].m_body;
	const dgBody* const body1 = m_bodyArray[m1].m_body;

	if (!(body0->m_resting & body1->m_resting)) {
		const dgVector preconditioner0(jointInfo->m_preconditioner0 * m_weight[m0]);
		const dgVector preconditioner1(jointInfo->m_preconditioner1 * m_weight[m1]);

#if 0
		dgVector linearM0(internalForces[m0].m_linear);
		dgVector angularM0(internalForces[m0].m_angular);
		dgVector linearM1(internalForces[m1].m_linear);
		dgVector angularM1(internalForces[m1].m_angular);

		normalForce[0] = dgFloat32(1.0f);
		const dgInt32 index = jointInfo->m_pairStart;
		const dgInt32 rowsCount = jointInfo->m_pairCount;
		for (dgInt32 j = 0; j < rowsCount; j++) {
			dgRightHandSide* const rhs = &rightHandSide[index + j];
			const dgLeftHandSide* const row = &leftHandSide[index + j];

			dgAssert(row->m_Jt.m_jacobianM0.m_linear.m_w == dgFloat32(0.0f));
			dgAssert(row->m_Jt.m_jacobianM0.m_angular.m_w == dgFloat32(0.0f));
			dgAssert(row->m_Jt.m_jacobianM1.m_linear.m_w == dgFloat32(0.0f));
			dgAssert(row->m_Jt.m_jacobianM1.m_angular.m_w == dgFloat32(0.0f));

			dgVector a((row->m_JMinv.m_jacobianM0.m_linear * linearM0 + row->m_JMinv.m_jacobianM0.m_angular * angularM0) * preconditioner0 +
					   (row->m_JMinv.m_jacobianM1.m_linear * linearM1 + row->m_JMinv.m_jacobianM1.m_angular * angularM1) * preconditioner1);
			a = dgVector(rhs->m_coordenateAccel - rhs->m_force * rhs->m_diagDamp) - a.AddHorizontal();

			dgVector f(rhs->m_force + rhs->m_invJinvMJt * a.GetScalar());
			dgAssert(rhs->m_normalForceIndex >= -1);
			dgAssert(rhs->m_normalForceIndex <= rowsCount);
			dgInt32 frictionIndex = rhs->m_normalForceIndex + 1;

			dgFloat32 frictionNormal = normalForce[frictionIndex];
			dgVector lowerFrictionForce(frictionNormal * rhs->m_lowerBoundFrictionCoefficent);
			dgVector upperFrictionForce(frictionNormal * rhs->m_upperBoundFrictionCoefficent);

			a = a.AndNot((f > upperFrictionForce) | (f < lowerFrictionForce));
			f = f.GetMax(lowerFrictionForce).GetMin(upperFrictionForce);

			//accNorm = accNorm.GetMax(a.Abs());
			accNorm += a * a;
			dgVector deltaForce(f - dgVector(rhs->m_force));

			rhs->m_force = f.GetScalar();
			normalForce[j + 1] = f.GetScalar();

			//dgVector deltaforce0(preconditioner0 * deltaForce);
			//dgVector deltaforce1(preconditioner1 * deltaForce);
			linearM0 += row->m_Jt.m_jacobianM0.m_linear * deltaForce;
			angularM0 += row->m_Jt.m_jacobianM0.m_angular * deltaForce;
			linearM1 += row->m_Jt.m_jacobianM1.m_linear * deltaForce;
			angularM1 += row->m_Jt.m_jacobianM1.m_angular * deltaForce;
		}

		dgVector maxAccel(accNorm);
		const dgFloat32 tol = dgFloat32(0.5f);
		const dgFloat32 tol2 = tol * tol;
		for (dgInt32 i = 0; (i < 4) && (maxAccel.GetScalar() > tol2); i++) {
			maxAccel = dgVector::m_zero;
			for (dgInt32 j = 0; j < rowsCount; j++) {
				dgRightHandSide* const rhs = &rightHandSide[index + j];
				const dgLeftHandSide* const row = &leftHandSide[index + j];
				dgVector a((row->m_JMinv.m_jacobianM0.m_linear * linearM0 + row->m_JMinv.m_jacobianM0.m_angular * angularM0) * preconditioner0 +
						   (row->m_JMinv.m_jacobianM1.m_linear * linearM1 + row->m_JMinv.m_jacobianM1.m_angular * angularM1) * preconditioner1);
				a = dgVector(rhs->m_coordenateAccel - rhs->m_force * rhs->m_diagDamp) - a.AddHorizontal();

				dgVector f(rhs->m_force + rhs->m_invJinvMJt * a.GetScalar());
				dgAssert(rhs->m_normalForceIndex >= -1);
				dgAssert(rhs->m_normalForceIndex <= rowsCount);
				dgInt32 frictionIndex = rhs->m_normalForceIndex + 1;

				dgFloat32 frictionNormal = normalForce[frictionIndex];
				dgVector lowerFrictionForce(frictionNormal * rhs->m_lowerBoundFrictionCoefficent);
				dgVector upperFrictionForce(frictionNormal * rhs->m_upperBoundFrictionCoefficent);

				a = a.AndNot((f > upperFrictionForce) | (f < lowerFrictionForce));
				f = f.GetMax(lowerFrictionForce).GetMin(upperFrictionForce);
				maxAccel = a * a;

				dgVector deltaForce(f - dgVector(rhs->m_force));

				rhs->m_force = f.GetScalar();
				normalForce[j + 1] = f.GetScalar();

				//dgVector deltaforce0(preconditioner0 * deltaForce);
				//dgVector deltaforce1(weight * deltaForce);
				linearM0 += row->m_Jt.m_jacobianM0.m_linear * deltaForce;
				angularM0 += row->m_Jt.m_jacobianM0.m_angular * deltaForce;
				linearM1 += row->m_Jt.m_jacobianM1.m_linear * deltaForce;
				angularM1 += row->m_Jt.m_jacobianM1.m_angular * deltaForce;
			}
		}
#else
		dgVector linearM0(internalForces[m0].m_linear * preconditioner0);
		dgVector angularM0(internalForces[m0].m_angular * preconditioner0);
		dgVector linearM1(internalForces[m1].m_linear * preconditioner1);
		dgVector angularM1(internalForces[m1].m_angular * preconditioner1);

		normalForce[0] = dgFloat32(1.0f);
		const dgInt32 index = jointInfo->m_pairStart;
		const dgInt32 rowsCount = jointInfo->m_pairCount;
		for (dgInt32 j = 0; j < rowsCount; j++) {
			dgRightHandSide* const rhs = &rightHandSide[index + j];
			const dgLeftHandSide* const row = &leftHandSide[index + j];

			dgVector a(
				row->m_JMinv.m_jacobianM0.m_linear * linearM0 + row->m_JMinv.m_jacobianM0.m_angular * angularM0 +
				row->m_JMinv.m_jacobianM1.m_linear * linearM1 + row->m_JMinv.m_jacobianM1.m_angular * angularM1);
			a = dgVector(rhs->m_coordenateAccel - rhs->m_force * rhs->m_diagDamp) - a.AddHorizontal();

			dgVector f(rhs->m_force + rhs->m_invJinvMJt * a.GetScalar());
			dgAssert(rhs->m_normalForceIndex >= -1);
			dgAssert(rhs->m_normalForceIndex <= rowsCount);
			dgInt32 frictionIndex = rhs->m_normalForceIndex + 1;

			dgFloat32 frictionNormal = normalForce[frictionIndex];
			dgVector lowerFrictionForce(frictionNormal * rhs->m_lowerBoundFrictionCoefficent);
			dgVector upperFrictionForce(frictionNormal * rhs->m_upperBoundFrictionCoefficent);

			a = a.AndNot((f > upperFrictionForce) | (f < lowerFrictionForce));
			f = f.GetMax(lowerFrictionForce).GetMin(upperFrictionForce);

			//accNorm = accNorm.GetMax(a.Abs());
			accNorm += a * a;
			dgVector deltaForce(f - dgVector(rhs->m_force));

			rhs->m_force = f.GetScalar();
			normalForce[j + 1] = f.GetScalar();

			dgVector deltaForce0(preconditioner0 * deltaForce);
			dgVector deltaForce1(preconditioner1 * deltaForce);
			linearM0 += row->m_Jt.m_jacobianM0.m_linear * deltaForce0;
			angularM0 += row->m_Jt.m_jacobianM0.m_angular * deltaForce0;
			linearM1 += row->m_Jt.m_jacobianM1.m_linear * deltaForce1;
			angularM1 += row->m_Jt.m_jacobianM1.m_angular * deltaForce1;
		}

		dgVector maxAccel(accNorm);
		const dgFloat32 tol = dgFloat32(0.5f);
		const dgFloat32 tol2 = tol * tol;
		for (dgInt32 i = 0; (i < 4) && (maxAccel.GetScalar() > tol2); i++) {
			maxAccel = dgVector::m_zero;
			for (dgInt32 j = 0; j < rowsCount; j++) {
				dgRightHandSide* const rhs = &rightHandSide[index + j];
				const dgLeftHandSide* const row = &leftHandSide[index + j];
				dgVector a(
					row->m_JMinv.m_jacobianM0.m_linear * linearM0 + row->m_JMinv.m_jacobianM0.m_angular * angularM0 +
					row->m_JMinv.m_jacobianM1.m_linear * linearM1 + row->m_JMinv.m_jacobianM1.m_angular * angularM1);
				a = dgVector(rhs->m_coordenateAccel - rhs->m_force * rhs->m_diagDamp) - a.AddHorizontal();

				dgVector f(rhs->m_force + rhs->m_invJinvMJt * a.GetScalar());
				dgAssert(rhs->m_normalForceIndex >= -1);
				dgAssert(rhs->m_normalForceIndex <= rowsCount);
				dgInt32 frictionIndex = rhs->m_normalForceIndex + 1;

				dgFloat32 frictionNormal = normalForce[frictionIndex];
				dgVector lowerFrictionForce(frictionNormal * rhs->m_lowerBoundFrictionCoefficent);
				dgVector upperFrictionForce(frictionNormal * rhs->m_upperBoundFrictionCoefficent);

				a = a.AndNot((f > upperFrictionForce) | (f < lowerFrictionForce));
				f = f.GetMax(lowerFrictionForce).GetMin(upperFrictionForce);
				maxAccel = a * a;

				dgVector deltaForce(f - dgVector(rhs->m_force));

				rhs->m_force = f.GetScalar();
				normalForce[j + 1] = f.GetScalar();

				dgVector deltaForce0(preconditioner0 * deltaForce);
				dgVector deltaForce1(preconditioner1 * deltaForce);
				linearM0 += row->m_Jt.m_jacobianM0.m_linear * deltaForce0;
				angularM0 += row->m_Jt.m_jacobianM0.m_angular * deltaForce0;
				linearM1 += row->m_Jt.m_jacobianM1.m_linear * deltaForce1;
				angularM1 += row->m_Jt.m_jacobianM1.m_angular * deltaForce1;
			}
		}
#endif

		for (dgInt32 i = 0; i < rowsCount; i++) {
			dgRightHandSide* const rhs = &rightHandSide[index + i];
			rhs->m_maxImpact = dgMax(dgAbs(rhs->m_force), rhs->m_maxImpact);
		}
	}
	return accNorm.GetScalar();
}
#endif

void dgParallelBodySolver::CalculateJointsForce(dgInt32 threadID)
{
#ifdef DG_SOLVER_USES_SOA
	dgJacobian* const internalForces = &m_world->m_solverMemory.m_internalForcesBuffer[0];
	dgSolverSoaElement* const massMatrix = &m_massMatrix[0];

	dgFloat32 accNorm = dgFloat32(0.0f);
	dgInt32* const atomicIndex = &m_atomicIndex;
	const int jointCount = m_jointCount;
	for (dgInt32 i = dgAtomicExchangeAndAdd(atomicIndex, 1); i < jointCount; i = dgAtomicExchangeAndAdd(atomicIndex, 1)) {
		const dgInt32 rowStart = m_soaRowStart[i];
		dgJointInfo* const jointInfo = &m_jointArray[i * DG_WORK_GROUP_SIZE];
		dgFloat32 accel2 = CalculateJointForce(jointInfo, &massMatrix[rowStart], internalForces);
		accNorm += accel2;
	}
	m_accelNorm[threadID] = accNorm;

#else
	const dgLeftHandSide* const leftHandSide = &m_world->m_solverMemory.m_leftHandSizeBuffer[0];
	dgRightHandSide* const rightHandSide = &m_world->m_solverMemory.m_righHandSizeBuffer[0];
	dgJacobian* const internalForces = &m_world->m_solverMemory.m_internalForcesBuffer[0];

	dgFloat32 accNorm = dgFloat32(0.0f);
	dgInt32* const atomicIndex = &m_atomicIndex;
	const int jointCount = m_cluster->m_jointCount;
	for (dgInt32 i = dgAtomicExchangeAndAdd(atomicIndex, 1); i < jointCount; i = dgAtomicExchangeAndAdd(atomicIndex, 1)) {
		dgJointInfo* const jointInfo = &m_jointArray[i];
		dgFloat32 accel2 = CalculateJointForce(jointInfo, leftHandSide, rightHandSide, internalForces);
		accNorm += accel2;
	}
	m_accelNorm[threadID] = accNorm;
#endif
}

void dgParallelBodySolver::UpdateRowAcceleration(dgInt32 threadID)
{
#ifdef DG_SOLVER_USES_SOA
	dgSolverSoaElement* const massMatrix = &m_massMatrix[0];
	const dgRightHandSide* const rightHandSide = &m_world->m_solverMemory.m_righHandSizeBuffer[0];

	const dgInt32* const soaRowStart = m_soaRowStart;
	const dgJointInfo* const jointInfoArray = m_jointArray;
	const int jointCount = m_jointCount;
	dgInt32* const atomicIndex = &m_atomicIndex;
	for (dgInt32 i = dgAtomicExchangeAndAdd(atomicIndex, 1); i < jointCount; i = dgAtomicExchangeAndAdd(atomicIndex, 1)) {
		const dgJointInfo* const jointInfoBase = &jointInfoArray[i * DG_WORK_GROUP_SIZE];

		const dgInt32 rowStart = soaRowStart[i];
		for (dgInt32 j = 0; j < DG_WORK_GROUP_SIZE; j++) {
			const dgJointInfo* const jointInfo = &jointInfoBase[j];
			if (jointInfo->m_joint) {
				dgInt32 const rowCount = jointInfo->m_pairCount;
				dgInt32 const rowStartBase = jointInfo->m_pairStart;
				for (dgInt32 k = 0; k < rowCount; k++) {
					dgSolverSoaElement* const row = &massMatrix[rowStart + k];
					row->m_coordenateAccel[j] = rightHandSide[k + rowStartBase].m_coordenateAccel;
				}
			}
		}
	}
#endif
}

void dgParallelBodySolver::CalculateBodyForce(dgInt32 threadID)
{
#ifdef DG_SOLVER_USES_SOA
	dgRightHandSide* const rightHandSide = &m_world->m_solverMemory.m_righHandSizeBuffer[0];
	
	dgSolverSoaElement* const massMatrix = &m_massMatrix[0];
	dgJacobian* const internalForces = &m_world->m_solverMemory.m_internalForcesBuffer[0];

	const dgFloat32* const invWeight = m_invWeight;
	const dgInt32* const soaRowStart = m_soaRowStart;
	const dgJointInfo* const jointInfoArray = m_jointArray;

	dgInt32* const atomicIndex = &m_atomicIndex;
	const int jointCount = m_jointCount;
	for (dgInt32 i = dgAtomicExchangeAndAdd(atomicIndex, 1); i < jointCount; i = dgAtomicExchangeAndAdd(atomicIndex, 1)) {
		dgWorkGroupVector6 forceAcc0;
		dgWorkGroupVector6 forceAcc1;

		const dgJointInfo* const jointInfoBase = &jointInfoArray[i * DG_WORK_GROUP_SIZE];
		const dgInt32 count = jointInfoBase->m_pairCount;

		forceAcc0.m_linear.m_x = dgWorkGroupFloat::m_zero;
		forceAcc0.m_linear.m_y = dgWorkGroupFloat::m_zero;
		forceAcc0.m_linear.m_z = dgWorkGroupFloat::m_zero;
		forceAcc0.m_angular.m_x = dgWorkGroupFloat::m_zero;
		forceAcc0.m_angular.m_y = dgWorkGroupFloat::m_zero;
		forceAcc0.m_angular.m_z = dgWorkGroupFloat::m_zero;

		forceAcc1.m_linear.m_x = dgWorkGroupFloat::m_zero;
		forceAcc1.m_linear.m_y = dgWorkGroupFloat::m_zero;
		forceAcc1.m_linear.m_z = dgWorkGroupFloat::m_zero;
		forceAcc1.m_angular.m_x = dgWorkGroupFloat::m_zero;
		forceAcc1.m_angular.m_y = dgWorkGroupFloat::m_zero;
		forceAcc1.m_angular.m_z = dgWorkGroupFloat::m_zero;
		
		const dgInt32 rowStart = soaRowStart[i];
		for (dgInt32 j = 0; j < count; j++) {
			const dgSolverSoaElement* const row = &massMatrix[rowStart + j];

			forceAcc0.m_linear.m_x = forceAcc0.m_linear.m_x + row->m_Jt.m_jacobianM0.m_linear.m_x * row->m_force;
			forceAcc0.m_linear.m_y = forceAcc0.m_linear.m_y + row->m_Jt.m_jacobianM0.m_linear.m_y * row->m_force;
			forceAcc0.m_linear.m_z = forceAcc0.m_linear.m_z + row->m_Jt.m_jacobianM0.m_linear.m_z * row->m_force;
			forceAcc0.m_angular.m_x = forceAcc0.m_angular.m_x + row->m_Jt.m_jacobianM0.m_angular.m_x * row->m_force;
			forceAcc0.m_angular.m_y = forceAcc0.m_angular.m_y + row->m_Jt.m_jacobianM0.m_angular.m_y * row->m_force;
			forceAcc0.m_angular.m_z = forceAcc0.m_angular.m_z + row->m_Jt.m_jacobianM0.m_angular.m_z * row->m_force;

			forceAcc1.m_linear.m_x = forceAcc1.m_linear.m_x + row->m_Jt.m_jacobianM1.m_linear.m_x * row->m_force;
			forceAcc1.m_linear.m_y = forceAcc1.m_linear.m_y + row->m_Jt.m_jacobianM1.m_linear.m_y * row->m_force;
			forceAcc1.m_linear.m_z = forceAcc1.m_linear.m_z + row->m_Jt.m_jacobianM1.m_linear.m_z * row->m_force;
			forceAcc1.m_angular.m_x = forceAcc1.m_angular.m_x + row->m_Jt.m_jacobianM1.m_angular.m_x * row->m_force;
			forceAcc1.m_angular.m_y = forceAcc1.m_angular.m_y + row->m_Jt.m_jacobianM1.m_angular.m_y * row->m_force;
			forceAcc1.m_angular.m_z = forceAcc1.m_angular.m_z + row->m_Jt.m_jacobianM1.m_angular.m_z * row->m_force;
		}

		for (dgInt32 j = 0; j < DG_WORK_GROUP_SIZE; j++) {
			const dgJointInfo* const jointInfo = &jointInfoBase[j];
			if (jointInfo->m_joint) {
				dgInt32 const rowCount = jointInfo->m_pairCount;
				dgInt32 const rowStartBase = jointInfo->m_pairStart;
				for (dgInt32 k = 0; k < rowCount; k++) {
					const dgSolverSoaElement* const row = &massMatrix[rowStart + k];
					rightHandSide[k + rowStartBase].m_force = row->m_force[j];
				}
				
				const dgInt32 m0 = jointInfo->m_m0;
				const dgInt32 m1 = jointInfo->m_m1;
				if (m0)
				{
					const dgVector weight(invWeight[m0]);
					dgBody* const body0 = jointInfo->m_joint->GetBody0();
					dgVector linear (weight * dgVector (forceAcc0.m_linear.m_x[j], forceAcc0.m_linear.m_y[j], forceAcc0.m_linear.m_z[j], dgFloat32 (0.0f)));
					dgVector angular (weight * dgVector (forceAcc0.m_angular.m_x[j], forceAcc0.m_angular.m_y[j], forceAcc0.m_angular.m_z[j], dgFloat32 (0.0f)));
					dgScopeSpinLock lock(&body0->m_criticalSectionLock);
					internalForces[m0].m_linear += linear;
					internalForces[m0].m_angular += angular;
				}
				if (m1) 
				{
					const dgVector weight(invWeight[m1]);
					dgBody* const body1 = jointInfo->m_joint->GetBody1();
					dgVector linear(weight * dgVector(forceAcc1.m_linear.m_x[j], forceAcc1.m_linear.m_y[j], forceAcc1.m_linear.m_z[j], dgFloat32(0.0f)));
					dgVector angular(weight * dgVector(forceAcc1.m_angular.m_x[j], forceAcc1.m_angular.m_y[j], forceAcc1.m_angular.m_z[j], dgFloat32(0.0f)));
					dgScopeSpinLock lock(&body1->m_criticalSectionLock);
					internalForces[m1].m_linear += linear;
					internalForces[m1].m_angular += angular;
				}
			}
		}
	}

#else
	const dgLeftHandSide* const leftHandSide = &m_world->m_solverMemory.m_leftHandSizeBuffer[0];
	const dgRightHandSide* const rightHandSide = &m_world->m_solverMemory.m_righHandSizeBuffer[0];
	dgJacobian* const internalForces = &m_world->m_solverMemory.m_internalForcesBuffer[0];

	const dgFloat32* const invWeight = m_invWeight;
	const dgJointInfo* const jointInfoArray = m_jointArray;

	dgInt32* const atomicIndex = &m_atomicIndex;
	const int jointCount = m_cluster->m_jointCount;
	for (dgInt32 j = dgAtomicExchangeAndAdd(atomicIndex, 1); j < jointCount; j = dgAtomicExchangeAndAdd(atomicIndex, 1)) {

		dgJacobian forceAcc0;
		dgJacobian forceAcc1;
		const dgJointInfo* const jointInfoBase = &jointInfoArray[j];
		const dgInt32 index = jointInfoBase->m_pairStart;
		const dgInt32 count = jointInfoBase->m_pairCount;
		const dgInt32 m0 = jointInfoBase->m_m0;
		const dgInt32 m1 = jointInfoBase->m_m1;

		forceAcc0.m_linear = dgVector::m_zero;
		forceAcc0.m_angular = dgVector::m_zero;
		forceAcc1.m_linear = dgVector::m_zero;
		forceAcc1.m_angular = dgVector::m_zero;
		for (dgInt32 i = 0; i < count; i++) {
			const dgLeftHandSide* const row = &leftHandSide[index + i];
			const dgRightHandSide* const rhs = &rightHandSide[index + i];
			dgAssert(dgCheckFloat(rhs->m_force));
			dgVector val(rhs->m_force);
			forceAcc0.m_linear += row->m_Jt.m_jacobianM0.m_linear * val;
			forceAcc0.m_angular += row->m_Jt.m_jacobianM0.m_angular * val;
			forceAcc1.m_linear += row->m_Jt.m_jacobianM1.m_linear * val;
			forceAcc1.m_angular += row->m_Jt.m_jacobianM1.m_angular * val;
		}

		if (m0)
		{
			const dgVector weight(invWeight[m0]);
			dgBody* const body0 = jointInfoBase->m_joint->GetBody0();
			forceAcc0.m_linear = forceAcc0.m_linear * weight;
			forceAcc0.m_angular = forceAcc0.m_angular * weight;
			dgScopeSpinLock lock(&body0->m_criticalSectionLock);
			internalForces[m0].m_linear += forceAcc0.m_linear;
			internalForces[m0].m_angular += forceAcc0.m_angular;
		}
		if (m1) 
		{
			const dgVector weight(invWeight[m1]);
			dgBody* const body1 = jointInfoBase->m_joint->GetBody1();
			forceAcc1.m_linear = forceAcc1.m_linear * weight;
			forceAcc1.m_angular = forceAcc1.m_angular * weight;

			dgScopeSpinLock lock(&body1->m_criticalSectionLock);
			internalForces[m1].m_linear += forceAcc1.m_linear;
			internalForces[m1].m_angular += forceAcc1.m_angular;
		}
	}
#endif
}

void dgParallelBodySolver::IntegrateBodiesVelocity(dgInt32 threadID)
{
	dgVector speedFreeze2(m_world->m_freezeSpeed2 * dgFloat32(0.1f));
	dgVector freezeOmega2(m_world->m_freezeOmega2 * dgFloat32(0.1f));

	dgVector timestep4(m_timestepRK);
	const dgFloat32* const weight = m_weight;
	dgJacobian* const internalForces = &m_world->m_solverMemory.m_internalForcesBuffer[0];
	dgInt32* const atomicIndex = &m_atomicIndex;
	const dgInt32 bodyCount = m_cluster->m_bodyCount;
	for (dgInt32 i = dgAtomicExchangeAndAdd(atomicIndex, 1); i < bodyCount; i = dgAtomicExchangeAndAdd(atomicIndex, 1)) {
		dgDynamicBody* const body = (dgDynamicBody*)m_bodyArray[i].m_body;
		dgAssert(body->m_index == i);

		if (body->IsRTTIType(dgBody::m_dynamicBodyRTTI)) {
			const dgVector w(weight[i]);
			const dgJacobian& forceAndTorque = internalForces[i];
			const dgVector force(body->m_externalForce + forceAndTorque.m_linear * w);
			const dgVector torque(body->m_externalTorque + forceAndTorque.m_angular * w);

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
			//dgTrace(("%d v(%f %f %f)\n", body->m_uniqueID, body->m_veloc[0], body->m_veloc[1], body->m_veloc[2]));
		}
	}
}

void dgParallelBodySolver::CalculateBodiesAcceleration(dgInt32 threadID)
{
	dgVector invTime(m_invTimestep);
	dgInt32* const atomicIndex = &m_atomicIndex;
	dgFloat32 maxAccNorm2 = DG_SOLVER_MAX_ERROR * DG_SOLVER_MAX_ERROR;
	const dgInt32 bodyCount = m_cluster->m_bodyCount;
	for (dgInt32 i = dgAtomicExchangeAndAdd(atomicIndex, 1); i < bodyCount; i = dgAtomicExchangeAndAdd(atomicIndex, 1)) {
		dgDynamicBody* const body = (dgDynamicBody*)m_bodyArray[i].m_body;
		m_world->CalculateNetAcceleration(body, invTime, maxAccNorm2);
	}
}

void dgParallelBodySolver::UpdateForceFeedback(dgInt32 threadID)
{
	const dgRightHandSide* const rightHandSide = &m_world->m_solverMemory.m_righHandSizeBuffer[0];
	dgInt32 hasJointFeeback = 0;
	dgInt32* const atomicIndex = &m_atomicIndex;
	const int jointCount = m_cluster->m_jointCount;
	for (dgInt32 i = dgAtomicExchangeAndAdd(atomicIndex, 1); i < jointCount; i = dgAtomicExchangeAndAdd(atomicIndex, 1)) {
		dgJointInfo* const jointInfo = &m_jointArray[i];
		dgConstraint* const constraint = jointInfo->m_joint;
		const dgInt32 first = jointInfo->m_pairStart;
		const dgInt32 count = jointInfo->m_pairCount;

		for (dgInt32 j = 0; j < count; j++) {
			const dgRightHandSide* const rhs = &rightHandSide[j + first];
			//dgLeftHandSide* const row = &matrixRow[j + first];
			dgAssert(dgCheckFloat(rhs->m_force));
			rhs->m_jointFeebackForce->m_force = rhs->m_force;
			rhs->m_jointFeebackForce->m_impact = rhs->m_maxImpact * m_timestepRK;
		}
		hasJointFeeback |= (constraint->m_updaFeedbackCallback ? 1 : 0);
	}
	m_hasJointFeeback[threadID] = hasJointFeeback;
}


void dgParallelBodySolver::UpdateKinematicFeedback(dgInt32 threadID)
{
	dgInt32* const atomicIndex = &m_atomicIndex;
	const int jointCount = m_cluster->m_jointCount;
	for (dgInt32 i = dgAtomicExchangeAndAdd(atomicIndex, 1); i < jointCount; i = dgAtomicExchangeAndAdd(atomicIndex, 1)) {
		dgJointInfo* const jointInfo = &m_jointArray[i];
		if (jointInfo->m_joint->m_updaFeedbackCallback) {
			jointInfo->m_joint->m_updaFeedbackCallback(*jointInfo->m_joint, m_timestep, threadID);
		}
	}
}

void dgParallelBodySolver::BuildJacobianMatrix(dgJointInfo* const jointInfo, dgLeftHandSide* const leftHandSide, dgRightHandSide* const rightHandSide, dgJacobian* const internalForces)
{
	const dgInt32 m0 = jointInfo->m_m0;
	const dgInt32 m1 = jointInfo->m_m1;
	const dgInt32 index = jointInfo->m_pairStart;
	const dgInt32 count = jointInfo->m_pairCount;
	const dgDynamicBody* const body0 = (dgDynamicBody*)m_bodyArray[m0].m_body;
	const dgDynamicBody* const body1 = (dgDynamicBody*)m_bodyArray[m1].m_body;
	const bool isBilateral = jointInfo->m_joint->IsBilateral();

	const dgMatrix invInertia0 = body0->m_invWorldInertiaMatrix;
	const dgMatrix invInertia1 = body1->m_invWorldInertiaMatrix;
	const dgVector invMass0(body0->m_invMass[3]);
	const dgVector invMass1(body1->m_invMass[3]);

	dgVector force0(dgVector::m_zero);
	dgVector torque0(dgVector::m_zero);
	if (body0->IsRTTIType(dgBody::m_dynamicBodyRTTI)) {
		force0 = body0->m_externalForce;
		torque0 = body0->m_externalTorque;
	}

	dgVector force1(dgVector::m_zero);
	dgVector torque1(dgVector::m_zero);
	if (body1->IsRTTIType(dgBody::m_dynamicBodyRTTI)) {
		force1 = body1->m_externalForce;
		torque1 = body1->m_externalTorque;
	}

	jointInfo->m_preconditioner0 = dgFloat32(1.0f);
	jointInfo->m_preconditioner1 = dgFloat32(1.0f);
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
	forceAcc0.m_linear = dgVector::m_zero;
	forceAcc0.m_angular = dgVector::m_zero;
	forceAcc1.m_linear = dgVector::m_zero;
	forceAcc1.m_angular = dgVector::m_zero;

	const dgFloat32 forceImpulseScale = dgFloat32(1.0f);

	const dgVector weight0(m_weight[m0] * jointInfo->m_preconditioner0);
	const dgVector weight1(m_weight[m1] * jointInfo->m_preconditioner0);
	for (dgInt32 i = 0; i < count; i++) {
		dgLeftHandSide* const row = &leftHandSide[index + i];
		dgRightHandSide* const rhs = &rightHandSide[index + i];
		
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

		dgVector jMinvM0linear(weight0 * row->m_JMinv.m_jacobianM0.m_linear);
		dgVector jMinvM0angular(weight0 * row->m_JMinv.m_jacobianM0.m_angular);
		dgVector jMinvM1linear(weight1 * row->m_JMinv.m_jacobianM1.m_linear);
		dgVector jMinvM1angular(weight1 * row->m_JMinv.m_jacobianM1.m_angular);
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

	const dgVector preconditioner0(jointInfo->m_preconditioner0);
	const dgVector preconditioner1(jointInfo->m_preconditioner1);

	forceAcc0.m_linear = forceAcc0.m_linear * preconditioner0;
	forceAcc0.m_angular = forceAcc0.m_angular * preconditioner0;
	forceAcc1.m_linear = forceAcc1.m_linear * preconditioner1;
	forceAcc1.m_angular = forceAcc1.m_angular * preconditioner1;

	if (m0) {
		dgScopeSpinLock lock(&body0->m_criticalSectionLock);
		internalForces[m0].m_linear += forceAcc0.m_linear;
		internalForces[m0].m_angular += forceAcc0.m_angular;
	}
	if (m1) {
		dgScopeSpinLock lock(&body1->m_criticalSectionLock);
		internalForces[m1].m_linear += forceAcc1.m_linear;
		internalForces[m1].m_angular += forceAcc1.m_angular;
	}
}

void dgParallelBodySolver::InitJacobianMatrix(dgInt32 threadID)
{
	dgLeftHandSide* const leftHandSide = &m_world->m_solverMemory.m_leftHandSizeBuffer[0];
	dgRightHandSide* const rightHandSide = &m_world->m_solverMemory.m_righHandSizeBuffer[0];
	dgJacobian* const internalForces = &m_world->m_solverMemory.m_internalForcesBuffer[0];

	dgContraintDescritor constraintParams;
	constraintParams.m_world = m_world;
	constraintParams.m_threadIndex = threadID;
	constraintParams.m_timestep = m_timestep;
	constraintParams.m_invTimestep = m_invTimestep;

	const dgInt32 jointCount = m_cluster->m_jointCount;
	for (dgInt32 i = dgAtomicExchangeAndAdd(&m_atomicIndex, 1); i < jointCount; i = dgAtomicExchangeAndAdd(&m_atomicIndex, 1)) {
		dgJointInfo* const jointInfo = &m_jointArray[i];
		dgConstraint* const constraint = jointInfo->m_joint;
		dgAssert(jointInfo->m_m0 >= 0);
		dgAssert(jointInfo->m_m1 >= 0);
		dgAssert(jointInfo->m_m0 != jointInfo->m_m1);
		const dgInt32 rowBase = dgAtomicExchangeAndAdd(&m_jacobianMatrixRowAtomicIndex, jointInfo->m_pairCount);
		m_world->GetJacobianDerivatives(constraintParams, jointInfo, constraint, leftHandSide, rightHandSide, rowBase);
		BuildJacobianMatrix(jointInfo, leftHandSide, rightHandSide, internalForces);
	}
}

void dgParallelBodySolver::TransposeRow (dgSolverSoaElement* const row, const dgJointInfo* const jointInfoArray, dgInt32 index)
{
	const dgLeftHandSide* const leftHandSide = &m_world->m_solverMemory.m_leftHandSizeBuffer[0];
	const dgRightHandSide* const rightHandSide = &m_world->m_solverMemory.m_righHandSizeBuffer[0];
	if (jointInfoArray[0].m_pairCount == jointInfoArray[DG_WORK_GROUP_SIZE - 1].m_pairCount) {
		for (dgInt32 i = 0; i < DG_WORK_GROUP_SIZE; i++) {
			const dgJointInfo* const jointInfo = &jointInfoArray[i];
			const dgLeftHandSide* const lhs = &leftHandSide[jointInfo->m_pairStart + index];
			const dgRightHandSide* const rhs = &rightHandSide[jointInfo->m_pairStart + index];

			row->m_Jt.m_jacobianM0.m_linear.m_x[i] = lhs->m_Jt.m_jacobianM0.m_linear.m_x;
			row->m_Jt.m_jacobianM0.m_linear.m_y[i] = lhs->m_Jt.m_jacobianM0.m_linear.m_y;
			row->m_Jt.m_jacobianM0.m_linear.m_z[i] = lhs->m_Jt.m_jacobianM0.m_linear.m_z;
			row->m_Jt.m_jacobianM0.m_angular.m_x[i] = lhs->m_Jt.m_jacobianM0.m_angular.m_x;
			row->m_Jt.m_jacobianM0.m_angular.m_y[i] = lhs->m_Jt.m_jacobianM0.m_angular.m_y;
			row->m_Jt.m_jacobianM0.m_angular.m_z[i] = lhs->m_Jt.m_jacobianM0.m_angular.m_z;
			row->m_Jt.m_jacobianM1.m_linear.m_x[i] = lhs->m_Jt.m_jacobianM1.m_linear.m_x;
			row->m_Jt.m_jacobianM1.m_linear.m_y[i] = lhs->m_Jt.m_jacobianM1.m_linear.m_y;
			row->m_Jt.m_jacobianM1.m_linear.m_z[i] = lhs->m_Jt.m_jacobianM1.m_linear.m_z;
			row->m_Jt.m_jacobianM1.m_angular.m_x[i] = lhs->m_Jt.m_jacobianM1.m_angular.m_x;
			row->m_Jt.m_jacobianM1.m_angular.m_y[i] = lhs->m_Jt.m_jacobianM1.m_angular.m_y;
			row->m_Jt.m_jacobianM1.m_angular.m_z[i] = lhs->m_Jt.m_jacobianM1.m_angular.m_z;

			row->m_JMinv.m_jacobianM0.m_linear.m_x[i] = lhs->m_JMinv.m_jacobianM0.m_linear.m_x;
			row->m_JMinv.m_jacobianM0.m_linear.m_y[i] = lhs->m_JMinv.m_jacobianM0.m_linear.m_y;
			row->m_JMinv.m_jacobianM0.m_linear.m_z[i] = lhs->m_JMinv.m_jacobianM0.m_linear.m_z;
			row->m_JMinv.m_jacobianM0.m_angular.m_x[i] = lhs->m_JMinv.m_jacobianM0.m_angular.m_x;
			row->m_JMinv.m_jacobianM0.m_angular.m_y[i] = lhs->m_JMinv.m_jacobianM0.m_angular.m_y;
			row->m_JMinv.m_jacobianM0.m_angular.m_z[i] = lhs->m_JMinv.m_jacobianM0.m_angular.m_z;
			row->m_JMinv.m_jacobianM1.m_linear.m_x[i] = lhs->m_JMinv.m_jacobianM1.m_linear.m_x;
			row->m_JMinv.m_jacobianM1.m_linear.m_y[i] = lhs->m_JMinv.m_jacobianM1.m_linear.m_y;
			row->m_JMinv.m_jacobianM1.m_linear.m_z[i] = lhs->m_JMinv.m_jacobianM1.m_linear.m_z;
			row->m_JMinv.m_jacobianM1.m_angular.m_x[i] = lhs->m_JMinv.m_jacobianM1.m_angular.m_x;
			row->m_JMinv.m_jacobianM1.m_angular.m_y[i] = lhs->m_JMinv.m_jacobianM1.m_angular.m_y;
			row->m_JMinv.m_jacobianM1.m_angular.m_z[i] = lhs->m_JMinv.m_jacobianM1.m_angular.m_z;

			row->m_force[i] = rhs->m_force;
			row->m_diagDamp[i] = rhs->m_diagDamp;
			row->m_invJinvMJt[i] = rhs->m_invJinvMJt;
			row->m_coordenateAccel[i] = rhs->m_coordenateAccel;
			row->m_normalForceIndex.m_i[i] = rhs->m_normalForceIndex;
			row->m_lowerBoundFrictionCoefficent[i] = rhs->m_lowerBoundFrictionCoefficent;
			row->m_upperBoundFrictionCoefficent[i] = rhs->m_upperBoundFrictionCoefficent;
		}
	} else {
		memset (row, 0, sizeof (dgSolverSoaElement));
		for (dgInt32 i = 0; i < DG_WORK_GROUP_SIZE; i ++) {
			if (index < jointInfoArray[i].m_pairCount) {
				const dgJointInfo* const jointInfo = &jointInfoArray[i];
				const dgLeftHandSide* const lhs = &leftHandSide[jointInfo->m_pairStart + index];	
				const dgRightHandSide* const rhs = &rightHandSide[jointInfo->m_pairStart + index];

				row->m_Jt.m_jacobianM0.m_linear.m_x[i] = lhs->m_Jt.m_jacobianM0.m_linear.m_x;
				row->m_Jt.m_jacobianM0.m_linear.m_y[i] = lhs->m_Jt.m_jacobianM0.m_linear.m_y;
				row->m_Jt.m_jacobianM0.m_linear.m_z[i] = lhs->m_Jt.m_jacobianM0.m_linear.m_z;
				row->m_Jt.m_jacobianM0.m_angular.m_x[i] = lhs->m_Jt.m_jacobianM0.m_angular.m_x;
				row->m_Jt.m_jacobianM0.m_angular.m_y[i] = lhs->m_Jt.m_jacobianM0.m_angular.m_y;
				row->m_Jt.m_jacobianM0.m_angular.m_z[i] = lhs->m_Jt.m_jacobianM0.m_angular.m_z;
				row->m_Jt.m_jacobianM1.m_linear.m_x[i] = lhs->m_Jt.m_jacobianM1.m_linear.m_x;
				row->m_Jt.m_jacobianM1.m_linear.m_y[i] = lhs->m_Jt.m_jacobianM1.m_linear.m_y;
				row->m_Jt.m_jacobianM1.m_linear.m_z[i] = lhs->m_Jt.m_jacobianM1.m_linear.m_z;
				row->m_Jt.m_jacobianM1.m_angular.m_x[i] = lhs->m_Jt.m_jacobianM1.m_angular.m_x;
				row->m_Jt.m_jacobianM1.m_angular.m_y[i] = lhs->m_Jt.m_jacobianM1.m_angular.m_y;
				row->m_Jt.m_jacobianM1.m_angular.m_z[i] = lhs->m_Jt.m_jacobianM1.m_angular.m_z;

				row->m_JMinv.m_jacobianM0.m_linear.m_x[i]  = lhs->m_JMinv.m_jacobianM0.m_linear.m_x;
				row->m_JMinv.m_jacobianM0.m_linear.m_y[i]  = lhs->m_JMinv.m_jacobianM0.m_linear.m_y;
				row->m_JMinv.m_jacobianM0.m_linear.m_z[i]  = lhs->m_JMinv.m_jacobianM0.m_linear.m_z;
				row->m_JMinv.m_jacobianM0.m_angular.m_x[i] = lhs->m_JMinv.m_jacobianM0.m_angular.m_x;
				row->m_JMinv.m_jacobianM0.m_angular.m_y[i] = lhs->m_JMinv.m_jacobianM0.m_angular.m_y;
				row->m_JMinv.m_jacobianM0.m_angular.m_z[i] = lhs->m_JMinv.m_jacobianM0.m_angular.m_z;
				row->m_JMinv.m_jacobianM1.m_linear.m_x[i]  = lhs->m_JMinv.m_jacobianM1.m_linear.m_x;
				row->m_JMinv.m_jacobianM1.m_linear.m_y[i]  = lhs->m_JMinv.m_jacobianM1.m_linear.m_y;
				row->m_JMinv.m_jacobianM1.m_linear.m_z[i]  = lhs->m_JMinv.m_jacobianM1.m_linear.m_z;
				row->m_JMinv.m_jacobianM1.m_angular.m_x[i] = lhs->m_JMinv.m_jacobianM1.m_angular.m_x;
				row->m_JMinv.m_jacobianM1.m_angular.m_y[i] = lhs->m_JMinv.m_jacobianM1.m_angular.m_y;
				row->m_JMinv.m_jacobianM1.m_angular.m_z[i] = lhs->m_JMinv.m_jacobianM1.m_angular.m_z;

				row->m_force[i] = rhs->m_force;
				row->m_diagDamp[i] = rhs->m_diagDamp;
				row->m_invJinvMJt[i] = rhs->m_invJinvMJt;
				row->m_coordenateAccel[i] = rhs->m_coordenateAccel;
				row->m_normalForceIndex.m_i[i] = rhs->m_normalForceIndex;
				row->m_lowerBoundFrictionCoefficent[i] = rhs->m_lowerBoundFrictionCoefficent;
				row->m_upperBoundFrictionCoefficent[i] = rhs->m_upperBoundFrictionCoefficent;
			} else {
				row->m_normalForceIndex.m_i[i] = DG_INDEPENDENT_ROW;
			}
		}
	}
}

void dgParallelBodySolver::TransposeMassMatrix(dgInt32 threadID)
{
	const dgJointInfo* const jointInfoArray = m_jointArray;
	dgSolverSoaElement* const massMatrixArray = &m_massMatrix[0];
	const dgInt32 jointCount = m_jointCount;

	for (dgInt32 i = dgAtomicExchangeAndAdd(&m_atomicIndex, 1); i < jointCount; i = dgAtomicExchangeAndAdd(&m_atomicIndex, 1)) {
		const dgInt32 index = i * DG_WORK_GROUP_SIZE;
		const dgInt32 rowCount = jointInfoArray[index].m_pairCount;
		const dgInt32 rowSoaStart = dgAtomicExchangeAndAdd(&m_soaRowsCount, rowCount);
		m_soaRowStart[i] = rowSoaStart;
		for (dgInt32 j = 0; j < rowCount; j ++) {
			dgSolverSoaElement* const row = &massMatrixArray[rowSoaStart + j];
			TransposeRow (row, &jointInfoArray[index], j);
		}
	}
}

void dgParallelBodySolver::CalculateForces()
{
	const dgInt32 passes = m_solverPasses + 1;
	m_firstPassCoef = dgFloat32(0.0f);
	const dgInt32 threadCounts = m_world->GetThreadCount();

	for (dgInt32 step = 0; step < 4; step++) {
		CalculateJointsAcceleration();
		dgFloat32 accNorm = DG_SOLVER_MAX_ERROR * dgFloat32(2.0f);
		for (dgInt32 k = 0; (k < passes) && (accNorm > DG_SOLVER_MAX_ERROR); k++) {
			CalculateJointsForce();
			CalculateBodyForce();
			accNorm = dgFloat32(0.0f);
			for (dgInt32 i = 0; i < threadCounts; i++) {
				accNorm = dgMax(accNorm, m_accelNorm[i]);
			}
		}
		IntegrateBodiesVelocity();
	}

	UpdateForceFeedback();

	dgInt32 hasJointFeeback = 0;
	for (dgInt32 i = 0; i < DG_MAX_THREADS_HIVE_COUNT; i++) {
		hasJointFeeback |= m_hasJointFeeback[i];
	}
	CalculateBodiesAcceleration();

	if (hasJointFeeback) {
		UpdateKinematicFeedback();
	}
}


