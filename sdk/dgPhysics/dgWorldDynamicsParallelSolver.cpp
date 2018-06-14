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

//dgWorkGroupFloat dgWorkGroupFloat::m_one(dgVector::m_one, dgVector::m_one);
//dgWorkGroupFloat dgWorkGroupFloat::m_zero(dgVector::m_zero, dgVector::m_zero);

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

dgParallelBodySolver::dgParallelBodySolver(dgMemoryAllocator* const allocator)
	:m_world(NULL)
	,m_cluster(NULL)
	,m_bodyArray(NULL)
	,m_jointArray(NULL)
	,m_weight____(NULL)
	,m_invWeight____(NULL)
	,m_timestep(dgFloat32 (0.0f))
	,m_invTimestep(dgFloat32(0.0f))
	,m_invStepRK(dgFloat32(0.0f))
	,m_timestepRK(dgFloat32(0.0f))
	,m_invTimestepRK(dgFloat32(0.0f))
	,m_firstPassCoef(dgFloat32(0.0f))
//	,m_bodyCount(0)
	,m_jointCount(0)
	,m_atomicIndex(0)
	,m_jacobianMatrixRowAtomicIndex(0)
	,m_solverPasses(0)
	,m_threadCounts(0)
//	:m_weigh(allocator)
//	,m_invWeigh(allocator)
//	,m_veloc(allocator)
//	,m_veloc0(allocator)
//	,m_rotation(allocator)
//	,m_invMass(allocator)
//	,m_localInvInertia(allocator)
//	,m_invInertia(allocator)
//	,m_internalForces(allocator)
{
}

dgParallelBodySolver::~dgParallelBodySolver()
{
}

void dgParallelBodySolver::Reserve (dgInt32 bodyCount, dgInt32 jointCount)
{
/*
	bodyCount = (bodyCount + DG_WORK_GROUP_SIZE - 1) & -dgInt32 (DG_WORK_GROUP_SIZE - 1);
	m_bodyCount = bodyCount / DG_WORK_GROUP_SIZE;

	m_weigh.ResizeIfNecessary(m_bodyCount);
	m_invWeigh.ResizeIfNecessary(m_bodyCount);
	m_veloc.ResizeIfNecessary(m_bodyCount);
	m_veloc0.ResizeIfNecessary(m_bodyCount);
	m_invMass.ResizeIfNecessary(m_bodyCount);
	m_invInertia.ResizeIfNecessary(m_bodyCount);
//	m_rotation.ResizeIfNecessary(m_count);
	m_internalForces.ResizeIfNecessary(m_bodyCount);
	m_localInvInertia.ResizeIfNecessary(m_bodyCount);
*/

	jointCount = (jointCount + DG_WORK_GROUP_SIZE - 1) & -dgInt32(DG_WORK_GROUP_SIZE - 1);
	m_jointCount = jointCount / DG_WORK_GROUP_SIZE;
}

void dgParallelBodySolver::CalculateJointForces(dgBodyCluster& cluster, dgBodyInfo* const bodyArray, dgJointInfo* const jointArray, dgFloat32 timestep)
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
m_solverPasses = 16;

//	m_jacobianMatrixRowAtomicIndex = 0;
	
	Reserve(cluster.m_bodyCount, cluster.m_jointCount);

//	const dgInt32 bodyCount = m_bodyCount * DG_WORK_GROUP_SIZE;
//	for (dgInt32 i = cluster.m_bodyCount; i < bodyCount; i++) {
//		bodyArray[i] = bodyArray[0];
//	}

	const dgInt32 jointCount = m_jointCount * DG_WORK_GROUP_SIZE;
	for (dgInt32 i = cluster.m_jointCount; i < jointCount; i++) {
		memset(&jointArray[i], 0, sizeof(dgBodyInfo));
	}

	m_weight____ = dgAlloca(dgFloat32, cluster.m_bodyCount);
	m_invWeight____ = dgAlloca(dgFloat32, cluster.m_bodyCount);

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

void dgParallelBodySolver::InitWeights()
{
	memset(m_weight____, 0, m_cluster->m_bodyCount * sizeof(dgFloat32));

	m_atomicIndex = 0;
	for (dgInt32 i = 0; i < m_threadCounts; i++) {
		m_world->QueueJob(InitWeightKernel, this, NULL);
	}
	m_world->SynchronizationBarrier();
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
}

void dgParallelBodySolver::CalculateJointsAcceleration()
{
	m_atomicIndex = 0;
	for (dgInt32 i = 0; i < m_threadCounts; i++) {
		m_world->QueueJob(CalculateJointsAccelerationKernel, this, NULL);
	}
	m_world->SynchronizationBarrier();
	m_firstPassCoef = dgFloat32(1.0f);
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
		m_world->QueueJob(InitJacobianMatrixKernel, this, NULL);
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
	dgFloat32* const invWeight = m_invWeight____;
	const dgInt32 jointCount = m_cluster->m_jointCount;
	for (dgInt32 i = dgAtomicExchangeAndAdd(&m_atomicIndex, 1); i < jointCount; i = dgAtomicExchangeAndAdd(&m_atomicIndex, 1)) {
		dgJointInfo* const jointInfo = &m_jointArray[i];
		const dgInt32 m0 = jointInfo->m_m0;
		const dgInt32 m1 = jointInfo->m_m1;
		dgBody* const body0 = jointInfo->m_joint->GetBody0();
		dgBody* const body1 = jointInfo->m_joint->GetBody1();
		{
			dgScopeSpinLock lock(&body0->m_criticalSectionLock);
			invWeight[m0] += dgFloat32(1.0f);
		}
		{
			dgScopeSpinLock lock(&body1->m_criticalSectionLock);
			invWeight[m1] += dgFloat32(1.0f);
		}
	}
}


void dgParallelBodySolver::InitBodyArray(dgInt32 threadID)
{
//	const dgInt32 bodyCount = m_bodyCount * DG_WORK_GROUP_SIZE;
//	dgWorkGroupVector3 zero(dgWorkGroupFloat::m_zero, dgWorkGroupFloat::m_zero, dgWorkGroupFloat::m_zero);

	const dgInt32 bodyCount = m_cluster->m_bodyCount;
//	for (dgInt32 i = dgAtomicExchangeAndAdd(&m_atomicIndex, DG_WORK_GROUP_SIZE); i < bodyCount; i = dgAtomicExchangeAndAdd(&m_atomicIndex, DG_WORK_GROUP_SIZE)) {
	for (dgInt32 i = dgAtomicExchangeAndAdd(&m_atomicIndex, 1); i < bodyCount; i = dgAtomicExchangeAndAdd(&m_atomicIndex, 1)) {
		dgBodyInfo* const bodyInfo = &m_bodyArray[i];

		dgBody* const body = (dgDynamicBody*)bodyInfo->m_body;
//		dgDynamicBody* const body0 = (dgDynamicBody*)bodyArray[0].m_body;
//		dgDynamicBody* const body1 = (dgDynamicBody*)bodyArray[1].m_body;
//		dgDynamicBody* const body2 = (dgDynamicBody*)bodyArray[2].m_body;
//		dgDynamicBody* const body3 = (dgDynamicBody*)bodyArray[3].m_body;
//		dgDynamicBody* const body4 = (dgDynamicBody*)bodyArray[4].m_body;
//		dgDynamicBody* const body5 = (dgDynamicBody*)bodyArray[7].m_body;
//		dgDynamicBody* const body6 = (dgDynamicBody*)bodyArray[6].m_body;
//		dgDynamicBody* const body7 = (dgDynamicBody*)bodyArray[7].m_body;

//		dgWorkGroupFloat damp_ang_x(body0->GetDampCoeffcient(m_timestep), body4->GetDampCoeffcient(m_timestep));
//		dgWorkGroupFloat damp_ang_y(body1->GetDampCoeffcient(m_timestep), body5->GetDampCoeffcient(m_timestep));
//		dgWorkGroupFloat damp_ang_z(body2->GetDampCoeffcient(m_timestep), body6->GetDampCoeffcient(m_timestep));
//		dgWorkGroupFloat damp_linear(body3->GetDampCoeffcient(m_timestep), body7->GetDampCoeffcient(m_timestep));
//		dgWorkGroupFloat::Transpose4x8(damp_ang_x, damp_ang_y, damp_ang_z, damp_linear);

		body->AddDampingAcceleration(m_timestep);
		body->CalcInvInertiaMatrix();

/*
		dgWorkGroupVector3 omega(
			body0->GetOmega(), body1->GetOmega(), body2->GetOmega(), body3->GetOmega(),
			body4->GetOmega(), body5->GetOmega(), body6->GetOmega(), body7->GetOmega());

		dgWorkGroupMatrix3x3 rotation(
			body0->GetMatrix(), body1->GetMatrix(), body2->GetMatrix(), body3->GetMatrix(),
			body4->GetMatrix(), body5->GetMatrix(), body6->GetMatrix(), body7->GetMatrix());
		m_veloc[i].m_angular = rotation.RotateVector(rotation.UnrotateVector(omega) * dgWorkGroupVector3(damp_ang_x, damp_ang_y, damp_ang_z));

		dgWorkGroupVector3 veloc(
			body0->GetVelocity(), body1->GetVelocity(), body2->GetVelocity(), body3->GetVelocity(),
			body4->GetVelocity(), body5->GetVelocity(), body6->GetVelocity(), body7->GetVelocity());
		m_veloc[i].m_linear = veloc.Scale(damp_linear);

		m_veloc0[i] = m_veloc[i];

		dgWorkGroupFloat invIIx(body0->GetInvMass(), body4->GetInvMass());
		dgWorkGroupFloat invIIy(body1->GetInvMass(), body5->GetInvMass());
		dgWorkGroupFloat invIIz(body2->GetInvMass(), body6->GetInvMass());
		dgWorkGroupFloat invMass(body3->GetInvMass(), body7->GetInvMass());
		dgWorkGroupFloat::Transpose4x8(invIIx, invIIy, invIIz, invMass);
		m_invMass[i] = invMass;
		m_localInvInertia[i] = dgWorkGroupVector3(invIIx, invIIy, invIIz);

		dgWorkGroupMatrix3x3 invInertia(rotation.Transposed());
		invInertia.m_front = invInertia.m_front * m_localInvInertia[i];
		invInertia.m_up = invInertia.m_up * m_localInvInertia[i];
		invInertia.m_right = invInertia.m_right * m_localInvInertia[i];
		m_invInertia[i] = invInertia * rotation;

		dgWorkGroupFloat mask(m_weigh[i] > dgWorkGroupFloat::m_zero);
		dgWorkGroupFloat weight(mask.Select(m_weigh[i], dgWorkGroupFloat::m_one));
*/

		body->m_accel = body->m_veloc;
		body->m_alpha = body->m_omega;

		dgAssert (0);
/*
		const dgFloat32 weight = m_weight[i] ? m_weight[i] : dgFloat32(1.0f);
		//m_weight[i] = weight;
		//m_invWeight[i] = dgFloat32 (1.0f) / weight;
		m_weight[i] = dgFloat32 (1.0f) / weight;
*/
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
	const dgLeftHandSide* const leftHandSide = &m_world->m_solverMemory.m_jacobianBuffer[0];

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


dgFloat32 dgParallelBodySolver::CalculateJointForce(const dgJointInfo* const jointInfo, const dgLeftHandSide* const leftHandSide, dgRightHandSide* const rightHandSide, const dgJacobian* const internalForces) const
{
	dgVector accNorm(dgVector::m_zero);
	dgFloat32 normalForce[DG_CONSTRAINT_MAX_ROWS + 4];

	const dgInt32 m0 = jointInfo->m_m0;
	const dgInt32 m1 = jointInfo->m_m1;
	const dgBody* const body0 = m_bodyArray[m0].m_body;
	const dgBody* const body1 = m_bodyArray[m1].m_body;
	dgAssert (0);
/*
	if (!(body0->m_resting & body1->m_resting)) {
		dgVector m_weight0 (m_weight[m0]);
		dgVector m_weight1 (m_weight[m1]);

		dgVector linearM0(internalForces[m0].m_linear * m_weight0);
		dgVector angularM0(internalForces[m0].m_angular * m_weight0);
		dgVector linearM1(internalForces[m1].m_linear * m_weight1);
		dgVector angularM1(internalForces[m1].m_angular * m_weight1);

		const dgVector preconditioner0(jointInfo->m_preconditioner0);
		const dgVector preconditioner1(jointInfo->m_preconditioner1);

		const dgInt32 index = jointInfo->m_pairStart;
		const dgInt32 rowsCount = jointInfo->m_pairCount;

		normalForce[rowsCount] = dgFloat32(1.0f);
		for (dgInt32 k = 0; k < rowsCount; k++) {
			dgRightHandSide* const rhs = &rightHandSide[index + k];
			const dgLeftHandSide* const row = &leftHandSide[index + k];

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
				const dgLeftHandSide* const row = &leftHandSide[index + k];
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
			rhs->m_maxImpact = dgMax(dgAbs(rhs->m_force), rhs->m_maxImpact);
		}
	}
*/
	return accNorm.GetScalar();
}


void dgParallelBodySolver::CalculateJointsForce(dgInt32 threadID)
{
	const dgLeftHandSide* const leftHandSide = &m_world->m_solverMemory.m_jacobianBuffer[0];
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
}

void dgParallelBodySolver::CalculateBodyForce(dgInt32 threadID)
{
	const dgLeftHandSide* const leftHandSide = &m_world->m_solverMemory.m_jacobianBuffer[0];
	const dgRightHandSide* const rightHandSide = &m_world->m_solverMemory.m_righHandSizeBuffer[0];
	dgJacobian* const internalForces = &m_world->m_solverMemory.m_internalForcesBuffer[0];

	dgInt32* const atomicIndex = &m_atomicIndex;
	const int jointCount = m_cluster->m_jointCount;
	for (dgInt32 j = dgAtomicExchangeAndAdd(atomicIndex, 1); j < jointCount; j = dgAtomicExchangeAndAdd(atomicIndex, 1)) {

		dgJacobian forceAcc0;
		dgJacobian forceAcc1;
		const dgJointInfo* const jointInfo = &m_jointArray[j];
		const dgInt32 index = jointInfo->m_pairStart;
		const dgInt32 count = jointInfo->m_pairCount;
		const dgInt32 m0 = jointInfo->m_m0;
		const dgInt32 m1 = jointInfo->m_m1;

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
		dgAssert (0);
/*
		const dgVector preconditioner0(jointInfo->m_preconditioner0 * m_weight[m0]);
		const dgVector preconditioner1(jointInfo->m_preconditioner1 * m_weight[m1]);
		forceAcc0.m_linear = forceAcc0.m_linear * preconditioner0;
		forceAcc0.m_angular = forceAcc0.m_angular * preconditioner0;
		forceAcc1.m_linear = forceAcc1.m_linear * preconditioner1;
		forceAcc1.m_angular = forceAcc1.m_angular * preconditioner1;

		dgBody* const body0 = jointInfo->m_joint->GetBody0();
		dgBody* const body1 = jointInfo->m_joint->GetBody1();
		//if (m0)
		{
			dgScopeSpinLock lock(&body0->m_criticalSectionLock);
			internalForces[m0].m_linear += forceAcc0.m_linear;
			internalForces[m0].m_angular += forceAcc0.m_angular;
		}
		//if (m1) 
		{
			dgScopeSpinLock lock(&body1->m_criticalSectionLock);
			internalForces[m1].m_linear += forceAcc1.m_linear;
			internalForces[m1].m_angular += forceAcc1.m_angular;
		}
*/
	}
}

void dgParallelBodySolver::IntegrateBodiesVelocity(dgInt32 threadID)
{
	dgVector speedFreeze2(m_world->m_freezeSpeed2 * dgFloat32(0.1f));
	dgVector freezeOmega2(m_world->m_freezeOmega2 * dgFloat32(0.1f));

	dgVector timestep4(m_timestepRK);
	dgJacobian* const internalForces = &m_world->m_solverMemory.m_internalForcesBuffer[0];
	dgInt32* const atomicIndex = &m_atomicIndex;
	const dgInt32 bodyCount = m_cluster->m_bodyCount;
	for (dgInt32 i = dgAtomicExchangeAndAdd(atomicIndex, 1); i < bodyCount; i = dgAtomicExchangeAndAdd(atomicIndex, 1)) {
		dgDynamicBody* const body = (dgDynamicBody*)m_bodyArray[i].m_body;
		dgAssert(body->m_index == i);

		if (body->IsRTTIType(dgBody::m_dynamicBodyRTTI)) {
			dgAssert (0);
/*
			const dgVector weight(m_weight[i]);
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
*/
			dgAssert(body->m_veloc.m_w == dgFloat32(0.0f));
			dgAssert(body->m_omega.m_w == dgFloat32(0.0f));
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

	dgMatrix invInertia0(body0->m_invWorldInertiaMatrix);
	dgMatrix invInertia1(body1->m_invWorldInertiaMatrix);

//	const dgVector invWeight0(m_invWeight[m0]);
//	const dgVector invWeight1(m_invWeight[m1]);
	dgAssert (0);
/*
	const dgVector invWeight0(m_weight[m0]);
	const dgVector invWeight1(m_weight[m1]);
	const dgVector invMass0(body0->m_invMass[3] * invWeight0.GetScalar());
	const dgVector invMass1(body1->m_invMass[3] * invWeight1.GetScalar());
	invInertia0.m_front *= invWeight0;
	invInertia0.m_up *= invWeight0;
	invInertia0.m_right *= invWeight0;
	invInertia1.m_front *= invWeight1;
	invInertia1.m_up *= invWeight1;
	invInertia1.m_right *= invWeight1;

	dgVector force0(dgVector::m_zero);
	dgVector torque0(dgVector::m_zero);
	if (body0->IsRTTIType(dgBody::m_dynamicBodyRTTI)) {
		//dgVector weight0(dgFloat32 (1.0f) / m_weight[m0]);
		dgVector weight0(m_weight[m0]);
		force0 = body0->m_externalForce * weight0;
		torque0 = body0->m_externalTorque * weight0;
	}

	dgVector force1(dgVector::m_zero);
	dgVector torque1(dgVector::m_zero);
	if (body1->IsRTTIType(dgBody::m_dynamicBodyRTTI)) {
		//dgVector weight1(dgFloat32 (1.0f) / m_weight[m1]);
		dgVector weight1(m_weight[m1]);
		force1 = body1->m_externalForce * weight1;
		torque1 = body1->m_externalTorque * weight1;
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
	const dgVector preconditioner0(jointInfo->m_preconditioner0);
	const dgVector preconditioner1(jointInfo->m_preconditioner1);
	forceAcc0.m_linear = dgVector::m_zero;
	forceAcc0.m_angular = dgVector::m_zero;
	forceAcc1.m_linear = dgVector::m_zero;
	forceAcc1.m_angular = dgVector::m_zero;

	const dgFloat32 forceImpulseScale = dgFloat32(1.0f);

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
		dgScopeSpinLock lock(&body0->m_criticalSectionLock);
		internalForces[m0].m_linear += forceAcc0.m_linear;
		internalForces[m0].m_angular += forceAcc0.m_angular;
	}
	if (m1) {
		dgScopeSpinLock lock(&body1->m_criticalSectionLock);
		internalForces[m1].m_linear += forceAcc1.m_linear;
		internalForces[m1].m_angular += forceAcc1.m_angular;
	}
*/
}


void dgParallelBodySolver::InitJacobianMatrix(dgInt32 threadID)
{
	dgLeftHandSide* const leftHandSide = &m_world->m_solverMemory.m_jacobianBuffer[0];
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
		//world->BuildJacobianMatrixParallel(bodyArray, jointInfo, internalForces, leftHandSide, rightHandSide, bodyLocks, weight, invWeight);
		BuildJacobianMatrix(jointInfo, leftHandSide, rightHandSide, internalForces);
	}
}


void dgParallelBodySolver::CalculateForces()
{
	const dgInt32 passes = m_solverPasses;
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


