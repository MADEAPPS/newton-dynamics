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



void dgWorldDynamicUpdate::CalculateReactionForcesParallel (const dgIsland* const islandArray, dgInt32 islandsCount, dgFloat32 timestep) const
{
	dgParallelSolverSyncData syncData;

	dgWorld* const world = (dgWorld*) this;
	world->m_pairMemoryBuffer.ExpandCapacityIfNeessesary (m_bodies + m_joints + 1024, sizeof (dgParallelJointMap));

	syncData.m_bodyInfoMap = (dgInt32*) &world->m_pairMemoryBuffer[0];
	syncData.m_jointInfoMap = (dgParallelJointMap*) (&syncData.m_bodyInfoMap[(m_bodies + 15) & (-16)]);
	dgJointInfo* const constraintArray = (dgJointInfo*) &world->m_jointsMemory[0];

	dgInt32 bodyCount = LinearizeBodyParallelArray (islandsCount, &syncData, (dgBodyInfo*) &world->m_bodiesMemory[0], islandArray);
	dgInt32 jointsCount = LinearizeJointParallelArray (islandsCount, &syncData, constraintArray, islandArray);
	
	//dgJacobian* const internalVeloc = &world->m_solverMemory.m_internalVeloc[0];
	dgJacobian* const internalForces = &world->m_solverMemory.m_internalForces[0];
	dgVector zero(dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f));
	//internalVeloc[0].m_linear = zero;
	//internalVeloc[0].m_angular = zero;
	internalForces[0].m_linear = zero;
	internalForces[0].m_angular = zero;


//	syncData.m_world = world;
	syncData.m_timestep = timestep;
	syncData.m_invTimestep = (timestep > dgFloat32 (0.0f)) ? dgFloat32 (1.0f) / timestep : dgFloat32 (0.0f);
	syncData.m_invStepRK = (dgFloat32 (1.0f) / dgFloat32 (LINEAR_SOLVER_SUB_STEPS));
	syncData.m_timestepRK = syncData.m_timestep * syncData.m_invStepRK;
	syncData.m_invTimestepRK = syncData.m_invTimestep * dgFloat32 (LINEAR_SOLVER_SUB_STEPS);

	syncData.m_bodyCount = bodyCount;
	syncData.m_jointCount = jointsCount;
	syncData.m_atomicIndex = 0;
	syncData.m_islandCount = islandsCount;
	syncData.m_islandArray = islandArray;

	InitilizeBodyArrayParallel (&syncData);
	BuildJacobianMatrixParallel (&syncData);
	SolverInitInternalForcesParallel (&syncData);
	CalculateForcesGameModeParallel (&syncData);
	IntegrateInslandParallel(&syncData); 
}


dgInt32 dgWorldDynamicUpdate::LinearizeBodyParallelArray(dgInt32 islandsCount, dgParallelSolverSyncData* const solverSyncData, const dgBodyInfo* const bodyArray, const dgIsland* const islandArray) const
{
dgAssert (0);
return 0;
/*
	dgInt32* const bodyInfoMap = solverSyncData->m_bodyInfoMap;
	dgInt32 bodyCount = 0;
	for (dgInt32 i = 0; i < islandsCount; i ++) {
		dgInt32 count = islandArray[i].m_bodyCount;
		dgInt32 bodyStart = islandArray[i].m_bodyStart;
		for (dgInt32 j = 1; j < count; j ++) {
			dgInt32 index = bodyStart + j;
			bodyInfoMap[bodyCount] = index;
			dgDynamicBody* const body = bodyArray[index].m_body;
			body->m_index = index;
			bodyCount ++;
		}
	}
	return bodyCount;
*/
}




void dgWorldDynamicUpdate::InitilizeBodyArrayParallelKernel (void* const context, void* const worldContext, dgInt32 threadID)
{
dgAssert (0);
/*
	dgParallelSolverSyncData* const syncData = (dgParallelSolverSyncData*) context;
	dgWorld* const world = (dgWorld*) worldContext;
	dgInt32* const atomicIndex = &syncData->m_atomicIndex; 
	const dgInt32* const bodyInfoIndexArray = syncData->m_bodyInfoMap;
	dgBodyInfo* const bodyArray = (dgBodyInfo*) &world->m_bodiesMemory[0]; 
	//dgJacobian* const internalVeloc = &world->m_solverMemory.m_internalVeloc[0];
	dgJacobian* const internalForces = &world->m_solverMemory.m_internalForces[0];
	dgVector zero(dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f));

	for (dgInt32 i = dgAtomicExchangeAndAdd(atomicIndex, 1); i < syncData->m_bodyCount; i = dgAtomicExchangeAndAdd(atomicIndex, 1)) {
		dgAssert ((bodyArray[0].m_body->m_accel % bodyArray[0].m_body->m_accel) == dgFloat32 (0.0f));
		dgAssert ((bodyArray[0].m_body->m_alpha % bodyArray[0].m_body->m_alpha) == dgFloat32 (0.0f));

		dgInt32 index = bodyInfoIndexArray[i];
		dgAssert (index);
		dgDynamicBody* const body = bodyArray[index].m_body;
		dgAssert (body->m_invMass.m_w > dgFloat32 (0.0f));
		body->AddDampingAcceleration();
		body->CalcInvInertiaMatrix ();

		// re use these variables for temp storage 
		body->m_netForce = body->m_veloc;
		body->m_netTorque = body->m_omega;

		//internalVeloc[index].m_linear = zero;
		//internalVeloc[index].m_angular = zero;
		internalForces[index].m_linear = zero;
		internalForces[index].m_angular = zero;
	}
*/
}


void dgWorldDynamicUpdate::InitilizeBodyArrayParallel (dgParallelSolverSyncData* const syncData) const
{
	dgWorld* const world = (dgWorld*) this;
	dgInt32 threadCounts = world->GetThreadCount();	

	syncData->m_atomicIndex = 0;
	for (dgInt32 j = 0; j < threadCounts; j ++) {
		world->QueueJob (InitilizeBodyArrayParallelKernel, syncData, world);
	}
	world->SynchronizationBarrier();
}


void dgWorldDynamicUpdate::BuildJacobianMatrixParallelKernel (void* const context, void* const worldContext, dgInt32 threadID)
{
dgAssert (0);
/*
	dgParallelSolverSyncData* const syncData = (dgParallelSolverSyncData*) context;
	dgWorld* const world = (dgWorld*) worldContext;
	const dgParallelJointMap* const jointInfoIndexArray = syncData->m_jointInfoMap;
	dgInt32* const atomicIndex = &syncData->m_atomicIndex; 
	dgBodyInfo* const bodyArray = (dgBodyInfo*) &world->m_bodiesMemory[0]; 
	dgJointInfo* const constraintArray = (dgJointInfo*) &world->m_jointsMemory[0];
	dgJacobianMatrixElement* const matrixRow = &world->m_solverMemory.m_memory[0];

	for (dgInt32 i = dgAtomicExchangeAndAdd(atomicIndex, 1); i < syncData->m_jointsInBatch;  i = dgAtomicExchangeAndAdd(atomicIndex, 1)) {

		dgInt32 jointIndex = jointInfoIndexArray[i].m_jointIndex;
		dgJointInfo* const jointInfo = &constraintArray[jointIndex];
		//dgAssert (dgInt32 (jointInfo->m_joint->m_index) == i);

		dgInt32 rowBase = dgAtomicExchangeAndAdd(&syncData->m_jacobianMatrixRowAtomicIndex, jointInfo->m_autoPaircount);
		//	if (island->m_hasUnilateralJoints) {
		//		rowCount = GetJacobianDerivatives (island, threadID, false, rowBase, rowCount, timestep);
		//	}
		//	rowCount = GetJacobianDerivatives (island, threadID, true, rowBase, rowCount, timestep);
		world->GetJacobianDerivativesParallel (jointInfo, threadID, false, rowBase, syncData->m_timestep);

		dgInt32 index = jointInfo->m_autoPairstart;
		dgInt32 count = jointInfo->m_autoPaircount;
		dgInt32 m0 = jointInfo->m_m0;
		dgInt32 m1 = jointInfo->m_m1;

		const dgDynamicBody* const body0 = bodyArray[m0].m_body;
		const dgDynamicBody* const body1 = bodyArray[m1].m_body;
		const dgFloat32 invMass0 = body0->m_invMass[3];
		const dgMatrix& invInertia0 = body0->m_invWorldInertiaMatrix;
		const dgFloat32 invMass1 = body1->m_invMass[3];
		const dgMatrix& invInertia1 = body1->m_invWorldInertiaMatrix;

		// the parallel solve will not deal with impulse phase, only for large solution solution
		for (dgInt32 i = 0; i < count; i ++) {
			dgJacobianMatrixElement* const row = &matrixRow[index];

			dgVector JMinvJacobianLinearM0 (row->m_Jt.m_jacobianM0.m_linear.Scale3 (invMass0));
			dgVector JMinvJacobianAngularM0 (invInertia0.UnrotateVector (row->m_Jt.m_jacobianM0.m_angular));
			dgVector JMinvJacobianLinearM1 (row->m_Jt.m_jacobianM1.m_linear.Scale3 (invMass1));
			dgVector JMinvJacobianAngularM1 (invInertia1.UnrotateVector (row->m_Jt.m_jacobianM1.m_angular));

			dgVector tmpDiag (JMinvJacobianLinearM0.CompProduct(row->m_Jt.m_jacobianM0.m_linear) +
							  JMinvJacobianAngularM0.CompProduct(row->m_Jt.m_jacobianM0.m_angular) +
							  JMinvJacobianLinearM1.CompProduct(row->m_Jt.m_jacobianM1.m_linear) +
							  JMinvJacobianAngularM1.CompProduct(row->m_Jt.m_jacobianM1.m_angular));

			dgVector tmpAccel (JMinvJacobianLinearM0.CompProduct(body0->m_accel) +
							   JMinvJacobianAngularM0.CompProduct(body0->m_alpha) +
							   JMinvJacobianLinearM1.CompProduct(body1->m_accel) +
							   JMinvJacobianAngularM1.CompProduct(body1->m_alpha));


			dgFloat32 extenalAcceleration = -(tmpAccel.m_x + tmpAccel.m_y + tmpAccel.m_z);
			//row->m_extAccel = extenalAcceleration;
			row->m_deltaAccel = extenalAcceleration;
			row->m_coordenateAccel += extenalAcceleration;
			row->m_force = row->m_jointFeebackForce[0];

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
*/
}


void dgWorldDynamicUpdate::BuildJacobianMatrixParallel (dgParallelSolverSyncData* const syncData) const
{
	dgWorld* const world = (dgWorld*) this;
	dgInt32 threadCounts = world->GetThreadCount();	

	for (int i = 0; i < syncData->m_batchesCount; i ++) {
		syncData->m_atomicIndex = syncData->m_jointBatches[i].m_start;
		syncData->m_jointsInBatch = syncData->m_jointBatches[i].m_count + syncData->m_atomicIndex;
		for (dgInt32 j = 0; j < threadCounts; j ++) {
			world->QueueJob (BuildJacobianMatrixParallelKernel, syncData, world);
		}
		world->SynchronizationBarrier();
	}
}


void dgWorldDynamicUpdate::SolverInitInternalForcesParallelKernel (void* const context, void* const worldContext, dgInt32 threadID)
{
	dgParallelSolverSyncData* const syncData = (dgParallelSolverSyncData*) context;
	dgWorld* const world = (dgWorld*) worldContext;
	const dgParallelJointMap* const jointInfoIndexArray = syncData->m_jointInfoMap;
	dgJacobian* const internalForces = &world->m_solverMemory.m_internalForces[0];
	dgJacobianMatrixElement* const matrixRow = &world->m_solverMemory.m_memory[0];
	dgInt32* const atomicIndex = &syncData->m_atomicIndex; 
	dgJointInfo* const constraintArray = (dgJointInfo*) &world->m_jointsMemory[0];

	dgVector zero(dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f));
	for (dgInt32 i = dgAtomicExchangeAndAdd(atomicIndex, 1); i < syncData->m_jointsInBatch;  i = dgAtomicExchangeAndAdd(atomicIndex, 1)) {
		dgJacobian y0;
		dgJacobian y1;

		y0.m_linear = zero;
		y0.m_angular = zero;
		y1.m_linear = zero;
		y1.m_angular = zero;

		//dgInt32 jointIndex = jointInfoIndexArray[i];
		dgInt32 jointIndex = jointInfoIndexArray[i].m_jointIndex;
		dgJointInfo* const jointInfo = &constraintArray[jointIndex];

		dgInt32 first = jointInfo->m_autoPairstart;
		dgInt32 count = jointInfo->m_autoPaircount;
		for (dgInt32 j = 0; j < count; j ++) { 
			dgJacobianMatrixElement* const row = &matrixRow[j + first];

			dgFloat32 val = row->m_force; 
			dgAssert (dgCheckFloat(val));
			y0.m_linear += row->m_Jt.m_jacobianM0.m_linear.Scale3 (val);
			y0.m_angular += row->m_Jt.m_jacobianM0.m_angular.Scale3 (val);
			y1.m_linear += row->m_Jt.m_jacobianM1.m_linear.Scale3 (val);
			y1.m_angular += row->m_Jt.m_jacobianM1.m_angular.Scale3 (val);
		}

		dgInt32 m0 = jointInfo->m_m0;
		dgInt32 m1 = jointInfo->m_m1;
		dgAssert (m0 != m1);
		if (m0) {
			internalForces[m0].m_linear += y0.m_linear;
			internalForces[m0].m_angular += y0.m_angular;
		}

		if (m1) {
			internalForces[m1].m_linear += y1.m_linear;
			internalForces[m1].m_angular += y1.m_angular;
		}
	}
}


void dgWorldDynamicUpdate::SolverInitInternalForcesParallel (dgParallelSolverSyncData* const syncData) const
{
	dgWorld* const world = (dgWorld*) this;
	dgInt32 threadCounts = world->GetThreadCount();	

	for (int i = 0; i < syncData->m_batchesCount; i ++) {
		syncData->m_atomicIndex = syncData->m_jointBatches[i].m_start;
		syncData->m_jointsInBatch = syncData->m_jointBatches[i].m_count + syncData->m_atomicIndex;
		for (dgInt32 j = 0; j < threadCounts; j ++) {
			world->QueueJob (SolverInitInternalForcesParallelKernel, syncData, world);
		}
		world->SynchronizationBarrier();
	}
}



void dgWorldDynamicUpdate::CalculateJointsAccelParallelKernel (void* const context, void* const worldContext, dgInt32 threadID)
{
	dgParallelSolverSyncData* const syncData = (dgParallelSolverSyncData*) context;
	dgWorld* const world = (dgWorld*) worldContext;
	const dgParallelJointMap* const jointInfoIndexArray = syncData->m_jointInfoMap;
	dgJointInfo* const constraintArray = (dgJointInfo*) &world->m_jointsMemory[0];
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
	for (dgInt32 i = dgAtomicExchangeAndAdd(atomicIndex, 1); i < syncData->m_jointsInBatch;  i = dgAtomicExchangeAndAdd(atomicIndex, 1)) {	
		dgInt32 curJoint = jointInfoIndexArray[i].m_jointIndex;
		joindDesc.m_rowsCount = constraintArray[curJoint].m_autoPaircount;
		joindDesc.m_rowMatrix = &matrixRow[constraintArray[curJoint].m_autoPairstart];
		constraintArray[curJoint].m_joint->JointAccelerations (&joindDesc);
	}
}


void dgWorldDynamicUpdate::CalculateJointsForceParallelKernel (void* const context, void* const worldContext, dgInt32 threadID)
{
dgAssert (0);
/*
	dgParallelSolverSyncData* const syncData = (dgParallelSolverSyncData*) context;
	dgWorld* const world = (dgWorld*) worldContext;
	
	const dgParallelJointMap* const jointInfoIndexArray = syncData->m_jointInfoMap;
	dgJointInfo* const constraintArray = (dgJointInfo*) &world->m_jointsMemory[0];
	dgJacobian* const internalForces = &world->m_solverMemory.m_internalForces[0];
	dgJacobianMatrixElement* const matrixRow = &world->m_solverMemory.m_memory[0];
	const dgBodyInfo* const bodyArray = (dgBodyInfo*) &world->m_bodiesMemory[0]; 

	dgFloat32 cacheForce[DG_CONSTRAINT_MAX_ROWS + 4];
	cacheForce[0] = dgFloat32 (1.0f);
	cacheForce[1] = dgFloat32 (1.0f);
	cacheForce[2] = dgFloat32 (1.0f);
	cacheForce[3] = dgFloat32 (1.0f);
	dgFloat32* const normalForce = &cacheForce[4];

	dgInt32* const atomicIndex = &syncData->m_atomicIndex;
	for (dgInt32 i = dgAtomicExchangeAndAdd(atomicIndex, 1); i < syncData->m_jointsInBatch; i = dgAtomicExchangeAndAdd(atomicIndex, 1)) {
		dgInt32 curJoint = jointInfoIndexArray[i].m_jointIndex;
		dgInt32 m0 = constraintArray[curJoint].m_m0;
		dgInt32 m1 = constraintArray[curJoint].m_m1;
		dgInt32 index = constraintArray[curJoint].m_autoPairstart;
		dgInt32 rowsCount = constraintArray[curJoint].m_autoPaircount;

		dgVector linearM0 (internalForces[m0].m_linear);
		dgVector angularM0 (internalForces[m0].m_angular);
		dgVector linearM1 (internalForces[m1].m_linear);
		dgVector angularM1 (internalForces[m1].m_angular);

		const dgDynamicBody* const body0 = bodyArray[m0].m_body;
		const dgDynamicBody* const body1 = bodyArray[m1].m_body;
		const dgFloat32 invMass0 = body0->m_invMass[3];
		const dgMatrix& invInertia0 = body0->m_invWorldInertiaMatrix;
		const dgFloat32 invMass1 = body1->m_invMass[3];
		const dgMatrix& invInertia1 = body1->m_invWorldInertiaMatrix;

		for (dgInt32 k = 0; k < rowsCount; k ++) {
			dgJacobianMatrixElement* const row = &matrixRow[index];

			dgVector JMinvJacobianLinearM0 (row->m_Jt.m_jacobianM0.m_linear.Scale3 (invMass0));
			dgVector JMinvJacobianAngularM0 (invInertia0.UnrotateVector (row->m_Jt.m_jacobianM0.m_angular));
			dgVector JMinvJacobianLinearM1 (row->m_Jt.m_jacobianM1.m_linear.Scale3 (invMass1));
			dgVector JMinvJacobianAngularM1 (invInertia1.UnrotateVector (row->m_Jt.m_jacobianM1.m_angular));

			dgVector acc (JMinvJacobianLinearM0.CompProduct(linearM0) + 
						  JMinvJacobianAngularM0.CompProduct(angularM0) + 
						  JMinvJacobianLinearM1.CompProduct(linearM1) + 
						  JMinvJacobianAngularM1.CompProduct(angularM1));

			dgFloat32 a = row->m_coordenateAccel - acc.m_x - acc.m_y - acc.m_z - row->m_force * row->m_diagDamp;
			dgFloat32 f = row->m_force + row->m_invDJMinvJt * a;

			dgInt32 frictionIndex = row->m_normalForceIndex;
			dgAssert (((frictionIndex < 0) && (normalForce[frictionIndex] == dgFloat32 (1.0f))) || ((frictionIndex >= 0) && (normalForce[frictionIndex] >= dgFloat32 (0.0f))));
			dgFloat32 frictionNormal = normalForce[frictionIndex];

			dgFloat32 lowerFrictionForce = frictionNormal * row->m_lowerBoundFrictionCoefficent;
			dgFloat32 upperFrictionForce = frictionNormal * row->m_upperBoundFrictionCoefficent;

			if (f > upperFrictionForce) {
				a = dgFloat32 (0.0f);
				f = upperFrictionForce;
			} else if (f < lowerFrictionForce) {
				a = dgFloat32 (0.0f);
				f = lowerFrictionForce;
			}

			dgFloat32 prevValue = f - row->m_force;
			row->m_force = f;
			normalForce[k] = f;

			linearM0 += row->m_Jt.m_jacobianM0.m_linear.Scale3 (prevValue);
			angularM0 += row->m_Jt.m_jacobianM0.m_angular.Scale3 (prevValue);
			linearM1 += row->m_Jt.m_jacobianM1.m_linear.Scale3 (prevValue);
			angularM1 += row->m_Jt.m_jacobianM1.m_angular.Scale3 (prevValue);
			index ++;
		}
		internalForces[m0].m_linear = linearM0;
		internalForces[m0].m_angular = angularM0;
		internalForces[m1].m_linear = linearM1;
		internalForces[m1].m_angular = angularM1;
	}
*/
}

void dgWorldDynamicUpdate::CalculateJointsVelocParallelKernel (void* const context, void* const worldContext, dgInt32 threadID)
{
dgAssert (0);

/*
	dgParallelSolverSyncData* const syncData = (dgParallelSolverSyncData*) context;
	dgWorld* const world = (dgWorld*) worldContext;

	const dgInt32* const bodyInfoIndexArray = syncData->m_bodyInfoMap;
	dgBodyInfo* const bodyArray = (dgBodyInfo*) &world->m_bodiesMemory[0]; 
	dgJacobian* const internalForces = &world->m_solverMemory.m_internalForces[0];
	//dgJacobian* const internalVeloc = &world->m_solverMemory.m_internalVeloc[0];

	//dgFloat32 invStep = (dgFloat32 (1.0f) / dgFloat32 (LINEAR_SOLVER_SUB_STEPS));
	//dgFloat32 timestep = syncData->m_timestep * invStep;
	//dgFloat32 invStep = syncData->m_invStepRK;
	dgFloat32 timestep = syncData->m_timestepRK;

	dgInt32* const atomicIndex = &syncData->m_atomicIndex;
	for (dgInt32 i = dgAtomicExchangeAndAdd(atomicIndex, 1); i < syncData->m_bodyCount;  i = dgAtomicExchangeAndAdd(atomicIndex, 1)) {
		dgInt32 index = bodyInfoIndexArray[i];
		dgAssert (index);
		dgDynamicBody* const body = bodyArray[index].m_body;
		dgAssert (body->m_index == index);
		dgVector force (body->m_accel + internalForces[index].m_linear);
		dgVector torque (body->m_alpha + internalForces[index].m_angular);

		dgVector accel (force.Scale3 (body->m_invMass.m_w));
		dgVector alpha (body->m_invWorldInertiaMatrix.RotateVector (torque));
		body->m_veloc += accel.Scale3(timestep);
		body->m_omega += alpha.Scale3(timestep);

		//internalVeloc[index].m_linear += body->m_veloc;
		//internalVeloc[index].m_angular += body->m_omega;
	}
*/
}


void dgWorldDynamicUpdate::CalculateJointsImpulseVelocParallelKernel (void* const context, void* const worldContext, dgInt32 threadID)
{
dgAssert (0);
/*

	dgParallelSolverSyncData* const syncData = (dgParallelSolverSyncData*) context;
	dgWorld* const world = (dgWorld*) worldContext;

	const dgInt32* const bodyInfoIndexArray = syncData->m_bodyInfoMap;
	dgBodyInfo* const bodyArray = (dgBodyInfo*) &world->m_bodiesMemory[0]; 
	dgJacobian* const internalForces = &world->m_solverMemory.m_internalForces[0];
	//dgJacobian* const internalVeloc = &world->m_solverMemory.m_internalVeloc[0];

	dgInt32* const atomicIndex = &syncData->m_atomicIndex;
	for (dgInt32 i = dgAtomicExchangeAndAdd(atomicIndex, 1); i < syncData->m_bodyCount;  i = dgAtomicExchangeAndAdd(atomicIndex, 1)) {
		dgInt32 index = bodyInfoIndexArray[i];
		dgAssert (index);
		dgDynamicBody* const body = bodyArray[index].m_body;
		dgAssert (body->m_index == index);

		const dgVector& linearMomentum = internalForces[index].m_linear;
		const dgVector& angularMomentum = internalForces[index].m_angular;

		body->m_veloc += linearMomentum.Scale3(body->m_invMass.m_w);
		body->m_omega += body->m_invWorldInertiaMatrix.RotateVector (angularMomentum);

		//internalVeloc[index].m_linear += body->m_veloc;
		//internalVeloc[index].m_angular += body->m_omega;
	}
*/
}


void dgWorldDynamicUpdate::UpdateFeedbackForcesParallelKernel (void* const context, void* const worldContext, dgInt32 threadID)
{
	dgParallelSolverSyncData* const syncData = (dgParallelSolverSyncData*) context;
	dgWorld* const world = (dgWorld*) worldContext;
	//const dgInt32* const jointInfoIndexArray = syncData->m_jointInfoMap;
	const dgParallelJointMap* const jointInfoIndexArray = syncData->m_jointInfoMap;
	dgJointInfo* const constraintArray = (dgJointInfo*) &world->m_jointsMemory[0];
	dgJacobianMatrixElement* const matrixRow = &world->m_solverMemory.m_memory[0];

	dgInt32 hasJointFeeback = 0;
	dgInt32* const atomicIndex = &syncData->m_atomicIndex;
	for (dgInt32 i = dgAtomicExchangeAndAdd(atomicIndex, 1); i < syncData->m_jointsInBatch; i = dgAtomicExchangeAndAdd(atomicIndex, 1)) {
		dgInt32 curJoint = jointInfoIndexArray[i].m_jointIndex;
		dgInt32 first = constraintArray[curJoint].m_autoPairstart;
		dgInt32 count = constraintArray[curJoint].m_autoPaircount;

		for (dgInt32 j = 0; j < count; j ++) { 
			dgJacobianMatrixElement* const row = &matrixRow[j + first];
			dgFloat32 val = row->m_force; 
			dgAssert (dgCheckFloat(val));
			row->m_jointFeebackForce[0] = val;
		}
		hasJointFeeback |= (constraintArray[i].m_joint->m_updaFeedbackCallback ? 1 : 0);
	}
	syncData->m_hasJointFeeback[threadID] = hasJointFeeback;
}


void dgWorldDynamicUpdate::UpdateBodyVelocityParallelKernel (void* const context, void* const worldContext, dgInt32 threadID)
{
dgAssert (0);
/*
	dgParallelSolverSyncData* const syncData = (dgParallelSolverSyncData*) context;
	dgWorld* const world = (dgWorld*) worldContext;

	const dgInt32* const bodyInfoIndexArray = syncData->m_bodyInfoMap;
	dgBodyInfo* const bodyArray = (dgBodyInfo*) &world->m_bodiesMemory[0]; 

	dgFloat32 maxAccNorm2 = DG_SOLVER_MAX_ERROR * DG_SOLVER_MAX_ERROR;
	dgVector zero(dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f));

	//dgFloat32 invTimestepSrc = dgFloat32 (1.0f) / syncData->m_timestep;
	dgFloat32 invTimestepSrc = syncData->m_invTimestep;

	dgInt32* const atomicIndex = &syncData->m_atomicIndex;
	for (dgInt32 i = dgAtomicExchangeAndAdd(atomicIndex, 1); i < syncData->m_bodyCount; i = dgAtomicExchangeAndAdd(atomicIndex, 1)) {
		dgInt32 index = bodyInfoIndexArray[i];
		dgAssert (index);
		dgDynamicBody* const body = bodyArray[index].m_body;

		// the initial velocity and angular velocity were stored in net force and net torque, for memory saving
		dgVector accel = (body->m_veloc - body->m_netForce).Scale3 (invTimestepSrc);
		dgVector alpha = (body->m_omega - body->m_netTorque).Scale3 (invTimestepSrc);

		if ((accel % accel) < maxAccNorm2) {
			accel = zero;
		}

		if ((alpha % alpha) < maxAccNorm2) {
			alpha = zero;
		}

		body->m_accel = accel;
		body->m_alpha = alpha;
		body->m_netForce = accel.Scale3 (body->m_mass[3]);

		alpha = body->m_matrix.UnrotateVector(alpha);
		body->m_netTorque = body->m_matrix.RotateVector (alpha.CompProduct(body->m_mass));
	}
*/
}

void dgWorldDynamicUpdate::KinematicCallbackUpdateParallelKernel (void* const context, void* const worldContext, dgInt32 threadID)
{
	dgParallelSolverSyncData* const syncData = (dgParallelSolverSyncData*) context;
	dgWorld* const world = (dgWorld*) worldContext;
	const dgParallelJointMap* const jointInfoIndexArray = syncData->m_jointInfoMap;
	dgJointInfo* const constraintArray = (dgJointInfo*) &world->m_jointsMemory[0];

	dgInt32* const atomicIndex = &syncData->m_atomicIndex;
	for (dgInt32 i = dgAtomicExchangeAndAdd(atomicIndex, 1); i < syncData->m_jointsInBatch; i = dgAtomicExchangeAndAdd(atomicIndex, 1)) {
		dgInt32 curJoint = jointInfoIndexArray[i].m_jointIndex;
		if (constraintArray[curJoint].m_joint->m_updaFeedbackCallback) {
			constraintArray[curJoint].m_joint->m_updaFeedbackCallback (*constraintArray[curJoint].m_joint, syncData->m_timestep, threadID);
		}
	}
}


void dgWorldDynamicUpdate::CalculateForcesGameModeParallel (dgParallelSolverSyncData* const syncData) const
{
	dgWorld* const world = (dgWorld*) this;
	dgInt32 threadCounts = world->GetThreadCount();	

	dgInt32 maxPasses = dgInt32 (world->m_solverMode + DG_BASE_ITERATION_COUNT);
	if (maxPasses > DG_MAX_PARALLEL_PASSES) {
		maxPasses = DG_MAX_PARALLEL_PASSES;
	}

	syncData->m_firstPassCoef = dgFloat32 (0.0f);
	for (dgInt32 step = 0; step < LINEAR_SOLVER_SUB_STEPS; step ++) {

		for (int i = 0; i < syncData->m_batchesCount; i ++) {
			syncData->m_atomicIndex = syncData->m_jointBatches[i].m_start;
			syncData->m_jointsInBatch = syncData->m_jointBatches[i].m_count + syncData->m_atomicIndex;
			for (dgInt32 j = 0; j < threadCounts; j ++) {
				world->QueueJob (CalculateJointsAccelParallelKernel, syncData, world);
			}
			world->SynchronizationBarrier();
		}
		syncData->m_firstPassCoef = dgFloat32 (1.0f);

		for (dgInt32 passes = 0; passes < maxPasses; passes ++) {
			for (int i = 0; i < syncData->m_batchesCount; i ++) {
				syncData->m_atomicIndex = syncData->m_jointBatches[i].m_start;
				syncData->m_jointsInBatch = syncData->m_jointBatches[i].m_count + syncData->m_atomicIndex;
				for (dgInt32 j = 0; j < threadCounts; j ++) {
					world->QueueJob (CalculateJointsForceParallelKernel, syncData, world);
				}
				world->SynchronizationBarrier();
			}
		}

		syncData->m_atomicIndex = 0;
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

	for (int i = 0; i < syncData->m_batchesCount; i ++) {
		syncData->m_atomicIndex = syncData->m_jointBatches[i].m_start;
		syncData->m_jointsInBatch = syncData->m_jointBatches[i].m_count + syncData->m_atomicIndex;
		for (dgInt32 j = 0; j < threadCounts; j ++) {
			world->QueueJob (UpdateFeedbackForcesParallelKernel, syncData, world);
		}
		world->SynchronizationBarrier();
	}

	dgInt32 hasJointFeeback = 0;
	for (dgInt32 i = 0; i < DG_MAX_THREADS_HIVE_COUNT; i ++) {
		hasJointFeeback |= syncData->m_hasJointFeeback[i];
	}


	syncData->m_atomicIndex = 0;
	for (dgInt32 j = 0; j < threadCounts; j ++) {
		world->QueueJob (UpdateBodyVelocityParallelKernel, syncData, world);
	}
	world->SynchronizationBarrier();

	if (hasJointFeeback) {
		for (int i = 0; i < syncData->m_batchesCount; i ++) {
			syncData->m_atomicIndex = syncData->m_jointBatches[i].m_start;
			syncData->m_jointsInBatch = syncData->m_jointBatches[i].m_count + syncData->m_atomicIndex;
			for (dgInt32 j = 0; j < threadCounts; j ++) {
				world->QueueJob (KinematicCallbackUpdateParallelKernel, syncData, world);
			}
			world->SynchronizationBarrier();
		}
	}
}


void dgWorldDynamicUpdate::IntegrateInslandParallelKernel (void* const context, void* const worldContext, dgInt32 threadID) 
{
	dgParallelSolverSyncData* const syncData = (dgParallelSolverSyncData*) context;
	dgWorld* const world = (dgWorld*) worldContext;
	dgFloat32 timestep = syncData->m_timestep;
	dgInt32* const atomicIndex = &syncData->m_islandCountCounter; 
	const dgIsland* const islandArray = syncData->m_islandArray;
	for (dgInt32 i = dgAtomicExchangeAndAdd(atomicIndex, 1); i < syncData->m_islandCount; i = dgAtomicExchangeAndAdd(atomicIndex, 1)) {
		const dgIsland* const island = &islandArray[i];
		world->IntegrateArray (island, DG_SOLVER_MAX_ERROR, timestep, threadID); 
	}
} 



void dgWorldDynamicUpdate::IntegrateInslandParallel(dgParallelSolverSyncData* const syncData) const
{
	dgWorld* const world = (dgWorld*) this;
	dgInt32 threadCounts = world->GetThreadCount();	

	syncData->m_atomicIndex = 0;
	for (dgInt32 j = 0; j < threadCounts; j ++) {
		world->QueueJob (IntegrateInslandParallelKernel, syncData, world);
	}
	world->SynchronizationBarrier();
}



