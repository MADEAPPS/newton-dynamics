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


void dgWorldDynamicUpdate::CalculateReactionForcesParallelSimd (const dgIsland* const islandArray, dgInt32 islandsCount, dgFloat32 timestep) const
{
	dgParallelSolverSyncData syncData;

	dgWorld* const world = (dgWorld*) this;
	world->m_pairMemoryBuffer.ExpandCapacityIfNeessesary (m_bodies + m_joints + 1024, sizeof (dgParallelJointMap));

	syncData.m_bodyInfoMap = (dgInt32*) &world->m_pairMemoryBuffer[0];
	syncData.m_jointInfoMap = (dgParallelJointMap*) (&syncData.m_bodyInfoMap[(m_bodies + 15) & (-16)]);
	dgJointInfo* const constraintArray = (dgJointInfo*) &world->m_jointsMemory[0];

	dgInt32 bodyCount = LinearizeBodyParallelArray (islandsCount, &syncData, (dgBodyInfo*) &world->m_bodiesMemory[0], islandArray);
	dgInt32 jointsCount = LinearizeJointParallelArray (islandsCount, &syncData, constraintArray, islandArray);

	dgJacobian* const internalForces = &world->m_solverMemory.m_internalForces[0];
	dgVector zero(dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f));
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

	InitilizeBodyArrayParallelSimd (&syncData);
	BuildJacobianMatrixParallelSimd (&syncData);
	SolverInitInternalForcesParallelSimd (&syncData);
	CalculateForcesGameModeParallelSimd (&syncData);
	IntegrateInslandParallelSimd (&syncData); 
}



void dgWorldDynamicUpdate::InitilizeBodyArrayParallelKernelSimd (void* const context, void* const worldContext, dgInt32 threadID)
{
dgAssert (0);

/*
	dgParallelSolverSyncData* const syncData = (dgParallelSolverSyncData*) context;
	dgWorld* const world = (dgWorld*) worldContext;
	dgInt32* const atomicIndex = &syncData->m_atomicIndex; 
	const dgInt32* const bodyInfoIndexArray = syncData->m_bodyInfoMap;
	dgBodyInfo* const bodyArray = (dgBodyInfo*) &world->m_bodiesMemory[0]; 
//	dgJacobian* const internalVeloc = &world->m_solverMemory.m_internalVeloc[0];
	dgJacobian* const internalForces = &world->m_solverMemory.m_internalForces[0];

	dgSimd zero (dgFloat32 (0.0f));
	for (dgInt32 i = dgAtomicExchangeAndAdd(atomicIndex, 1); i < syncData->m_bodyCount; i = dgAtomicExchangeAndAdd(atomicIndex, 1)) {
		dgAssert ((bodyArray[0].m_body->m_accel % bodyArray[0].m_body->m_accel) == dgFloat32 (0.0f));
		dgAssert ((bodyArray[0].m_body->m_alpha % bodyArray[0].m_body->m_alpha) == dgFloat32 (0.0f));

		dgInt32 index = bodyInfoIndexArray[i];
		dgAssert (index);
		dgDynamicBody* const body = bodyArray[index].m_body;
		dgAssert (body->m_invMass.m_w > dgFloat32 (0.0f));
		body->AddDampingAccelerationSimd();
		body->CalcInvInertiaMatrixSimd();

		// re use these variables for temp storage 
		(dgSimd&)body->m_netForce  = (dgSimd&)body->m_veloc;
		(dgSimd&)body->m_netTorque = (dgSimd&)body->m_omega;

//		(dgSimd&)internalVeloc[index].m_linear = zero;
//		(dgSimd&)internalVeloc[index].m_angular = zero;
		(dgSimd&)internalForces[index].m_linear = zero;
		(dgSimd&)internalForces[index].m_angular = zero;
	}
*/
}


void dgWorldDynamicUpdate::InitilizeBodyArrayParallelSimd (dgParallelSolverSyncData* const syncData) const
{
	dgWorld* const world = (dgWorld*) this;
	dgInt32 threadCounts = world->GetThreadCount();	

	syncData->m_atomicIndex = 0;
	for (dgInt32 j = 0; j < threadCounts; j ++) {
		world->QueueJob (InitilizeBodyArrayParallelKernelSimd, syncData, world);
	}
	world->SynchronizationBarrier();
}


void dgWorldDynamicUpdate::BuildJacobianMatrixParallelKernelSimd (void* const context, void* const worldContext, dgInt32 threadID)
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

	dgSimd one (dgFloat32 (1.0f));
	dgSimd zero (dgFloat32 (0.0f));
	dgSimd diagDampConst (DG_PSD_DAMP_TOL);
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

		dgMatrix invInertiaTrans0;
		dgSimd invMass0 (body0->m_invMass[3]);
		dgSimd::Transpose4x4 ((dgSimd&)invInertiaTrans0[0], (dgSimd&)invInertiaTrans0[1], (dgSimd&)invInertiaTrans0[2], (dgSimd&)invInertiaTrans0[3], 
							  (dgSimd&)body0->m_invWorldInertiaMatrix[0], (dgSimd&)body0->m_invWorldInertiaMatrix[1], 
							  (dgSimd&)body0->m_invWorldInertiaMatrix[2], (dgSimd&)body0->m_invWorldInertiaMatrix[3]);
		
		dgSimd invMass1 (body1->m_invMass[3]);
		dgMatrix invInertiaTrans1;
		dgSimd::Transpose4x4 ((dgSimd&)invInertiaTrans1[0], (dgSimd&)invInertiaTrans1[1], (dgSimd&)invInertiaTrans1[2], (dgSimd&)invInertiaTrans1[3], 
							   (dgSimd&)body1->m_invWorldInertiaMatrix[0], (dgSimd&)body1->m_invWorldInertiaMatrix[1], 
							  (dgSimd&)body1->m_invWorldInertiaMatrix[2], (dgSimd&)body1->m_invWorldInertiaMatrix[3]);

		// the parallel solve will not deal with impulse phase, only for large solution solution
		for (dgInt32 i = 0; i < count; i ++) {
			dgJacobianMatrixElement* const row = &matrixRow[index];

			dgSimd JMinvJacobianLinearM0 ((dgSimd&)row->m_Jt.m_jacobianM0.m_linear * invMass0);
			dgSimd JMinvJacobianAngularM0 (invInertiaTrans0.RotateVectorSimd((dgSimd&)row->m_Jt.m_jacobianM0.m_angular));
			dgSimd JMinvJacobianLinearM1 ((dgSimd&)row->m_Jt.m_jacobianM1.m_linear * invMass1);
			dgSimd JMinvJacobianAngularM1 (invInertiaTrans1.RotateVectorSimd((dgSimd&)row->m_Jt.m_jacobianM1.m_angular));

			dgSimd tmpDiag (JMinvJacobianLinearM0 * (dgSimd&)row->m_Jt.m_jacobianM0.m_linear + 
							JMinvJacobianAngularM0 * (dgSimd&)row->m_Jt.m_jacobianM0.m_angular +
							JMinvJacobianLinearM1 * (dgSimd&)row->m_Jt.m_jacobianM1.m_linear + 
							JMinvJacobianAngularM1 * (dgSimd&)row->m_Jt.m_jacobianM1.m_angular);

			dgSimd tmpAccel (JMinvJacobianLinearM0 * (dgSimd&)body0->m_accel + 
							 JMinvJacobianAngularM0 * (dgSimd&)body0->m_alpha +
							 JMinvJacobianLinearM1 * (dgSimd&)body1->m_accel +
							 JMinvJacobianAngularM1 * (dgSimd&)body1->m_alpha);

			tmpAccel = zero - tmpAccel.AddHorizontal();

			tmpAccel.StoreScalar(&row->m_deltaAccel);
			tmpAccel = tmpAccel + dgSimd (row->m_coordenateAccel);
			tmpAccel.StoreScalar(&row->m_coordenateAccel);
			row->m_force = row->m_jointFeebackForce[0];
			dgAssert (row->m_diagDamp >= dgFloat32(0.1f));
			dgAssert (row->m_diagDamp <= dgFloat32(100.0f));

			dgSimd stiffness (dgSimd(row->m_diagDamp) * diagDampConst);
			tmpDiag = tmpDiag.AddHorizontal();
			dgSimd diagDamp (tmpDiag * stiffness);
			diagDamp.StoreScalar (&row->m_diagDamp);

			dgSimd invDiagDamp (one / (tmpDiag * (one + stiffness)));
			invDiagDamp.StoreScalar(&row->m_invDJMinvJt);

			index ++;
		}
	}
*/
}


void dgWorldDynamicUpdate::BuildJacobianMatrixParallelSimd (dgParallelSolverSyncData* const syncData) const
{
	dgWorld* const world = (dgWorld*) this;
	dgInt32 threadCounts = world->GetThreadCount();	

	for (int i = 0; i < syncData->m_batchesCount; i ++) {
		syncData->m_atomicIndex = syncData->m_jointBatches[i].m_start;
		syncData->m_jointsInBatch = syncData->m_jointBatches[i].m_count + syncData->m_atomicIndex;
		for (dgInt32 j = 0; j < threadCounts; j ++) {
			world->QueueJob (BuildJacobianMatrixParallelKernelSimd, syncData, world);
		}
		world->SynchronizationBarrier();
	}
}



void dgWorldDynamicUpdate::SolverInitInternalForcesParallelKernelSimd (void* const context, void* const worldContext, dgInt32 threadID)
{
	dgParallelSolverSyncData* const syncData = (dgParallelSolverSyncData*) context;
	dgWorld* const world = (dgWorld*) worldContext;
	const dgParallelJointMap* const jointInfoIndexArray = syncData->m_jointInfoMap;
	dgJacobian* const internalForces = &world->m_solverMemory.m_internalForces[0];
	dgJacobianMatrixElement* const matrixRow = &world->m_solverMemory.m_memory[0];
	dgInt32* const atomicIndex = &syncData->m_atomicIndex; 
	dgJointInfo* const constraintArray = (dgJointInfo*) &world->m_jointsMemory[0];

	dgSimd zero (dgFloat32 (0.0f));
	for (dgInt32 i = dgAtomicExchangeAndAdd(atomicIndex, 1); i < syncData->m_jointsInBatch;  i = dgAtomicExchangeAndAdd(atomicIndex, 1)) {
		dgJacobian y0;
		dgJacobian y1;

		(dgSimd&)y0.m_linear = zero;
		(dgSimd&)y0.m_angular = zero;
		(dgSimd&)y1.m_linear = zero;
		(dgSimd&)y1.m_angular = zero;

		//dgInt32 jointIndex = jointInfoIndexArray[i];
		dgInt32 jointIndex = jointInfoIndexArray[i].m_jointIndex;
		dgJointInfo* const jointInfo = &constraintArray[jointIndex];

		dgInt32 first = jointInfo->m_autoPairstart;
		dgInt32 count = jointInfo->m_autoPaircount;
		for (dgInt32 j = 0; j < count; j ++) { 
			dgJacobianMatrixElement* const row = &matrixRow[j + first];
			dgSimd val (row->m_force);
			(dgSimd&)y0.m_linear = (dgSimd&)y0.m_linear + (dgSimd&)row->m_Jt.m_jacobianM0.m_linear * val;
			(dgSimd&)y0.m_angular = (dgSimd&)y0.m_angular + (dgSimd&)row->m_Jt.m_jacobianM0.m_angular * val;
			(dgSimd&)y1.m_linear = (dgSimd&)y1.m_linear + (dgSimd&)row->m_Jt.m_jacobianM1.m_linear * val;
			(dgSimd&)y1.m_angular = (dgSimd&)y1.m_angular + (dgSimd&)row->m_Jt.m_jacobianM1.m_angular * val;
		}

		dgInt32 m0 = jointInfo->m_m0;
		dgInt32 m1 = jointInfo->m_m1;
		dgAssert (m0 != m1);
		if (m0) {
			(dgSimd&)internalForces[m0].m_linear = (dgSimd&)internalForces[m0].m_linear + (dgSimd&)y0.m_linear;
			(dgSimd&)internalForces[m0].m_angular = (dgSimd&)internalForces[m0].m_angular + (dgSimd&)y0.m_angular;
		}

		if (m1) {
			(dgSimd&)internalForces[m1].m_linear = (dgSimd&)internalForces[m1].m_linear + (dgSimd&)y1.m_linear;
			(dgSimd&)internalForces[m1].m_angular = (dgSimd&)internalForces[m1].m_angular + (dgSimd&)y1.m_angular;
		}
	}
}


void dgWorldDynamicUpdate::SolverInitInternalForcesParallelSimd (dgParallelSolverSyncData* const syncData) const
{
	dgWorld* const world = (dgWorld*) this;
	dgInt32 threadCounts = world->GetThreadCount();	

	for (int i = 0; i < syncData->m_batchesCount; i ++) {
		syncData->m_atomicIndex = syncData->m_jointBatches[i].m_start;
		syncData->m_jointsInBatch = syncData->m_jointBatches[i].m_count + syncData->m_atomicIndex;
		for (dgInt32 j = 0; j < threadCounts; j ++) {
			world->QueueJob (SolverInitInternalForcesParallelKernelSimd, syncData, world);
		}
		world->SynchronizationBarrier();
	}
}


void dgWorldDynamicUpdate::CalculateJointsAccelParallelKernelSimd (void* const context, void* const worldContext, dgInt32 threadID)
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
		constraintArray[curJoint].m_joint->JointAccelerationsSimd (&joindDesc);
	}
}


void dgWorldDynamicUpdate::CalculateJointsForceParallelKernelSimd (void* const context, void* const worldContext, dgInt32 threadID)
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

	dgSimd cacheForce[DG_CONSTRAINT_MAX_ROWS * sizeof (dgFloat32) / DG_VECTOR_SIMD_SIZE];
	cacheForce[0] = dgSimd::m_one;
	dgFloat32* const normalForce = (dgFloat32*) &cacheForce[1];

	dgInt32* const atomicIndex = &syncData->m_atomicIndex;
	for (dgInt32 i = dgAtomicExchangeAndAdd(atomicIndex, 1); i < syncData->m_jointsInBatch; i = dgAtomicExchangeAndAdd(atomicIndex, 1)) {
		dgInt32 curJoint = jointInfoIndexArray[i].m_jointIndex;
		dgInt32 m0 = constraintArray[curJoint].m_m0;
		dgInt32 m1 = constraintArray[curJoint].m_m1;
		dgInt32 index = constraintArray[curJoint].m_autoPairstart;
		dgInt32 rowsCount = constraintArray[curJoint].m_autoPaircount;

		dgSimd linearM0  = (dgSimd&)internalForces[m0].m_linear;
		dgSimd angularM0 = (dgSimd&)internalForces[m0].m_angular;
		dgSimd linearM1  = (dgSimd&)internalForces[m1].m_linear;
		dgSimd angularM1 = (dgSimd&)internalForces[m1].m_angular;

		const dgDynamicBody* const body0 = bodyArray[m0].m_body;
		const dgDynamicBody* const body1 = bodyArray[m1].m_body;

		dgMatrix invInertiaTrans0;
		dgSimd invMass0 (body0->m_invMass[3]);
		dgSimd::Transpose4x4 ((dgSimd&)invInertiaTrans0[0], (dgSimd&)invInertiaTrans0[1], (dgSimd&)invInertiaTrans0[2], (dgSimd&)invInertiaTrans0[3], 
							 (dgSimd&)body0->m_invWorldInertiaMatrix[0], (dgSimd&)body0->m_invWorldInertiaMatrix[1], 
							 (dgSimd&)body0->m_invWorldInertiaMatrix[2], (dgSimd&)body0->m_invWorldInertiaMatrix[3]);

		dgSimd invMass1 (body1->m_invMass[3]);
		dgMatrix invInertiaTrans1;
		dgSimd::Transpose4x4 ((dgSimd&)invInertiaTrans1[0], (dgSimd&)invInertiaTrans1[1], (dgSimd&)invInertiaTrans1[2], (dgSimd&)invInertiaTrans1[3], 
							  (dgSimd&)body1->m_invWorldInertiaMatrix[0], (dgSimd&)body1->m_invWorldInertiaMatrix[1], 
							  (dgSimd&)body1->m_invWorldInertiaMatrix[2], (dgSimd&)body1->m_invWorldInertiaMatrix[3]);

		for (dgInt32 k = 0; k < rowsCount; k ++) {
			dgJacobianMatrixElement* const row = &matrixRow[index];

			dgSimd JMinvJacobianLinearM0 ((dgSimd&)row->m_Jt.m_jacobianM0.m_linear * invMass0);
			dgSimd JMinvJacobianAngularM0 (invInertiaTrans0.RotateVectorSimd((dgSimd&)row->m_Jt.m_jacobianM0.m_angular));
			dgSimd JMinvJacobianLinearM1 ((dgSimd&)row->m_Jt.m_jacobianM1.m_linear * invMass1);
			dgSimd JMinvJacobianAngularM1 (invInertiaTrans1.RotateVectorSimd((dgSimd&)row->m_Jt.m_jacobianM1.m_angular));

			dgSimd a (JMinvJacobianLinearM0 * linearM0 + 
					  JMinvJacobianAngularM0 * angularM0 +
					  JMinvJacobianLinearM1 * linearM1 + 
				      JMinvJacobianAngularM1 * angularM1);

			dgSimd force (row->m_force);
			a = dgSimd(row->m_coordenateAccel) - a.AddHorizontal() - force * dgSimd(row->m_diagDamp);
			dgSimd f (force + a * dgSimd(row->m_invDJMinvJt));

			dgInt32 frictionIndex = row->m_normalForceIndex;
			//dgAssert (((frictionIndex < 0) && (matrixRow[frictionIndex].m_force == dgFloat32 (1.0f))) || ((frictionIndex >= 0) && (matrixRow[frictionIndex].m_force >= dgFloat32 (0.0f))));
			dgAssert (((frictionIndex < 0) && (normalForce[frictionIndex] == dgFloat32 (1.0f))) || ((frictionIndex >= 0) && (normalForce[frictionIndex] >= dgFloat32 (0.0f))));
			dgSimd frictionNormal (normalForce[frictionIndex]);
			dgSimd lowerFrictionForce (frictionNormal * dgSimd(row->m_lowerBoundFrictionCoefficent));
			dgSimd upperFrictionForce (frictionNormal * dgSimd(row->m_upperBoundFrictionCoefficent));

			a = a.AndNot((f > upperFrictionForce) | (f < lowerFrictionForce));
			f = f.GetMax(lowerFrictionForce).GetMin(upperFrictionForce);

			dgSimd prevValue (f - force);
			f.StoreScalar(&row->m_force);
			f.StoreScalar(&normalForce[k]);

			linearM0 = linearM0 + (dgSimd&) row->m_Jt.m_jacobianM0.m_linear * prevValue;
			angularM0 = angularM0 + (dgSimd&) row->m_Jt.m_jacobianM0.m_angular * prevValue;
			linearM1 = linearM1 + (dgSimd&) row->m_Jt.m_jacobianM1.m_linear * prevValue;
			angularM1 = angularM1 + (dgSimd&) row->m_Jt.m_jacobianM1.m_angular * prevValue;
			index ++;
		}
		(dgSimd&)internalForces[m0].m_linear = linearM0;
		(dgSimd&)internalForces[m0].m_angular = angularM0;
		(dgSimd&)internalForces[m1].m_linear = linearM1;
		(dgSimd&)internalForces[m1].m_angular = angularM1;
	}
*/
}


void dgWorldDynamicUpdate::CalculateJointsVelocParallelKernelSimd (void* const context, void* const worldContext, dgInt32 threadID)
{
dgAssert (0);
/*
	dgParallelSolverSyncData* const syncData = (dgParallelSolverSyncData*) context;
	dgWorld* const world = (dgWorld*) worldContext;

	const dgInt32* const bodyInfoIndexArray = syncData->m_bodyInfoMap;
	dgBodyInfo* const bodyArray = (dgBodyInfo*) &world->m_bodiesMemory[0]; 
	dgJacobian* const internalForces = &world->m_solverMemory.m_internalForces[0];
//	dgJacobian* const internalVeloc = &world->m_solverMemory.m_internalVeloc[0];

//	dgSimd invStep ((dgFloat32 (1.0f) / dgFloat32 (LINEAR_SOLVER_SUB_STEPS)));
//	dgSimd timeStepSimd (dgSimd(syncData->m_timestep) * invStep);
//	dgFloat32 invStep = syncData->m_invStepRK;
	dgFloat32 timestep = syncData->m_timestepRK;

	dgInt32* const atomicIndex = &syncData->m_atomicIndex;
	for (dgInt32 i = dgAtomicExchangeAndAdd(atomicIndex, 1); i < syncData->m_bodyCount;  i = dgAtomicExchangeAndAdd(atomicIndex, 1)) {
		dgInt32 index = bodyInfoIndexArray[i];
		dgAssert (index);
		dgDynamicBody* const body = bodyArray[index].m_body;
		dgAssert (body->m_index == index);

		//dgVector force (body->m_accel + internalForces[index].m_linear);
		//dgVector torque (body->m_alpha + internalForces[index].m_angular);
		//dgVector accel (force.Scale (body->m_invMass.m_w));
		//dgVector alpha (body->m_invWorldInertiaMatrix.RotateVector (torque));
		//body->m_veloc += accel.Scale(timestep);
		//body->m_omega += alpha.Scale(timestep);
		//internalVeloc[index].m_linear += body->m_veloc;
		//internalVeloc[index].m_angular += body->m_omega;

		dgSimd force ((dgSimd&)body->m_accel + (dgSimd&)internalForces[index].m_linear);
		dgSimd torque((dgSimd&)body->m_alpha + (dgSimd&)internalForces[index].m_angular);
		dgSimd accel (force * dgSimd(body->m_invMass.m_w));
		dgSimd alpha (body->m_invWorldInertiaMatrix.RotateVectorSimd(torque));

		(dgSimd&)body->m_veloc = (dgSimd&)body->m_veloc + accel * timestep;
		(dgSimd&)body->m_omega = (dgSimd&)body->m_omega + alpha * timestep;
//		(dgSimd&)internalVeloc[index].m_linear = (dgSimd&)internalVeloc[index].m_linear + (dgSimd&) body->m_veloc;
//		(dgSimd&)internalVeloc[index].m_angular = (dgSimd&)internalVeloc[index].m_angular + (dgSimd&) body->m_omega;
	}
*/
}


void dgWorldDynamicUpdate::CalculateJointsImpulseVelocParallelKernelSimd (void* const context, void* const worldContext, dgInt32 threadID)
{
dgAssert (0);
/*

	dgParallelSolverSyncData* const syncData = (dgParallelSolverSyncData*) context;
	dgWorld* const world = (dgWorld*) worldContext;

	const dgInt32* const bodyInfoIndexArray = syncData->m_bodyInfoMap;
	dgBodyInfo* const bodyArray = (dgBodyInfo*) &world->m_bodiesMemory[0]; 
	dgJacobian* const internalForces = &world->m_solverMemory.m_internalForces[0];
//	dgJacobian* const internalVeloc = &world->m_solverMemory.m_internalVeloc[0];

	dgInt32* const atomicIndex = &syncData->m_atomicIndex;
	for (dgInt32 i = dgAtomicExchangeAndAdd(atomicIndex, 1); i < syncData->m_bodyCount;  i = dgAtomicExchangeAndAdd(atomicIndex, 1)) {
		dgInt32 index = bodyInfoIndexArray[i];
		dgAssert (index);
		dgDynamicBody* const body = bodyArray[index].m_body;
		dgAssert (body->m_index == index);

		//const dgVector& linearMomentum = internalForces[index].m_linear;
		//const dgVector& angularMomentum = internalForces[index].m_angular;
		dgSimd linearMomentum ((dgSimd&)internalForces[index].m_linear);
		dgSimd angularMomentum((dgSimd&)internalForces[index].m_angular);

		//body->m_veloc += linearMomentum.Scale(body->m_invMass.m_w);
		//body->m_omega += body->m_invWorldInertiaMatrix.RotateVector (angularMomentum);
		(dgSimd&)body->m_veloc = (dgSimd&)body->m_veloc + linearMomentum * dgSimd(body->m_invMass.m_w);
		(dgSimd&)body->m_omega = (dgSimd&)body->m_omega + body->m_invWorldInertiaMatrix.RotateVectorSimd(angularMomentum);

		//internalVeloc[index].m_linear += body->m_veloc;
		//internalVeloc[index].m_angular += body->m_omega;
		//(dgSimd&)internalVeloc[index].m_linear = (dgSimd&)internalVeloc[index].m_linear + (dgSimd&) body->m_veloc;
		//(dgSimd&)internalVeloc[index].m_angular = (dgSimd&)internalVeloc[index].m_angular + (dgSimd&) body->m_omega;
	}
*/
}


void dgWorldDynamicUpdate::UpdateFeedbackForcesParallelKernelSimd (void* const context, void* const worldContext, dgInt32 threadID)
{
	dgParallelSolverSyncData* const syncData = (dgParallelSolverSyncData*) context;
	dgWorld* const world = (dgWorld*) worldContext;
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


void dgWorldDynamicUpdate::UpdateBodyVelocityParallelKernelSimd (void* const context, void* const worldContext, dgInt32 threadID)
{
dgAssert (0);

/*
	dgParallelSolverSyncData* const syncData = (dgParallelSolverSyncData*) context;
	dgWorld* const world = (dgWorld*) worldContext;

	const dgInt32* const bodyInfoIndexArray = syncData->m_bodyInfoMap;
	dgBodyInfo* const bodyArray = (dgBodyInfo*) &world->m_bodiesMemory[0]; 

	dgSimd zero (dgFloat32 (0.0f));
	//dgSimd invTimestepSrc (dgSimd::m_one / dgSimd (syncData->m_timestep));	
	dgSimd invTimestepSrc (syncData->m_invTimestep);	
	dgSimd accelerationTolerance (DG_SOLVER_MAX_ERROR * DG_SOLVER_MAX_ERROR, DG_SOLVER_MAX_ERROR * DG_SOLVER_MAX_ERROR, DG_SOLVER_MAX_ERROR * DG_SOLVER_MAX_ERROR, dgFloat32 (0.0f));	

	dgInt32* const atomicIndex = &syncData->m_atomicIndex;
	for (dgInt32 i = dgAtomicExchangeAndAdd(atomicIndex, 1); i < syncData->m_bodyCount; i = dgAtomicExchangeAndAdd(atomicIndex, 1)) {
		dgInt32 index = bodyInfoIndexArray[i];
		dgAssert (index);
		dgDynamicBody* const body = bodyArray[index].m_body;

		dgSimd accel (((dgSimd&) body->m_veloc - (dgSimd&) body->m_netForce) * invTimestepSrc);
		dgSimd alpha (((dgSimd&) body->m_omega - (dgSimd&) body->m_netTorque) * invTimestepSrc);

		//if ((accel % accel) < maxAccNorm2) {
		//	accel = zero;
		//}
		dgSimd accelTest ((accel * accel) > accelerationTolerance);
		accelTest = accelTest | accelTest.MoveHigh(accelTest);
		accelTest = accelTest | accelTest.PackLow(accelTest);

		//if ((alpha % alpha) < maxAccNorm2) {
		//	alpha = zero;
		//}
		dgSimd alphaTest ((alpha * alpha) > accelerationTolerance);
		alphaTest = alphaTest | alphaTest.MoveHigh(alphaTest);
		alphaTest = alphaTest | alphaTest.PackLow(alphaTest);
		
		//body->m_accel = accel;
		//body->m_alpha = alpha;
		(dgSimd&)body->m_accel = accel & (accelTest | accelTest.MoveLow(accelTest));
		(dgSimd&)body->m_alpha = alpha & (alphaTest | alphaTest.MoveLow(alphaTest));

		//body->m_netForce = accel.Scale (body->m_mass[3]);
		(dgSimd&)body->m_netForce = accel * dgSimd (body->m_mass[3]);

		//alpha = body->m_matrix.UnrotateVector(alpha);
		//body->m_netTorque = body->m_matrix.RotateVector (alpha.CompProduct(body->m_mass));
		alpha = body->m_matrix.UnrotateVectorSimd(alpha);
		(dgSimd&)body->m_netTorque = body->m_matrix.RotateVectorSimd(alpha * (dgSimd&)body->m_mass);

		dgAssert (dgAbsf (body->m_netForce[0]) < dgFloat32 (1.0e10f));
		dgAssert (dgAbsf (body->m_netForce[1]) < dgFloat32 (1.0e10f));
		dgAssert (dgAbsf (body->m_netForce[2]) < dgFloat32 (1.0e10f));
	}
*/
}

void dgWorldDynamicUpdate::KinematicCallbackUpdateParallelKernelSimd (void* const context, void* const worldContext, dgInt32 threadID)
{
	dgParallelSolverSyncData* const syncData = (dgParallelSolverSyncData*) context;
	dgWorld* const world = (dgWorld*) worldContext;
	//const dgInt32* const jointInfoIndexArray = syncData->m_jointInfoMap;
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


void dgWorldDynamicUpdate::CalculateForcesGameModeParallelSimd (dgParallelSolverSyncData* const syncData) const
{
	dgWorld* const world = 	(dgWorld*) this;
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
				world->QueueJob (CalculateJointsAccelParallelKernelSimd, syncData, world);
			}
			world->SynchronizationBarrier();
		}
		syncData->m_firstPassCoef = dgFloat32 (1.0f);

		for (dgInt32 passes = 0; passes < maxPasses; passes ++) {
			for (int i = 0; i < syncData->m_batchesCount; i ++) {
				syncData->m_atomicIndex = syncData->m_jointBatches[i].m_start;
				syncData->m_jointsInBatch = syncData->m_jointBatches[i].m_count + syncData->m_atomicIndex;
				for (dgInt32 j = 0; j < threadCounts; j ++) {
					world->QueueJob (CalculateJointsForceParallelKernelSimd, syncData, world);
				}
				world->SynchronizationBarrier();
			}
		}

		syncData->m_atomicIndex = 0;
		if (syncData->m_timestep != dgFloat32 (0.0f)) {
			for (dgInt32 j = 0; j < threadCounts; j ++) {
				world->QueueJob (CalculateJointsVelocParallelKernelSimd, syncData, world);
			}
		} else {
			for (dgInt32 j = 0; j < threadCounts; j ++) {
				world->QueueJob (CalculateJointsImpulseVelocParallelKernelSimd, syncData, world);
			}
		}
		world->SynchronizationBarrier();
	}


	for (int i = 0; i < syncData->m_batchesCount; i ++) {
		syncData->m_atomicIndex = syncData->m_jointBatches[i].m_start;
		syncData->m_jointsInBatch = syncData->m_jointBatches[i].m_count + syncData->m_atomicIndex;
		for (dgInt32 j = 0; j < threadCounts; j ++) {
			world->QueueJob (UpdateFeedbackForcesParallelKernelSimd, syncData, world);
		}
		world->SynchronizationBarrier();
	}

	dgInt32 hasJointFeeback = 0;
	for (dgInt32 i = 0; i < DG_MAX_THREADS_HIVE_COUNT; i ++) {
		hasJointFeeback |= syncData->m_hasJointFeeback[i];
	}


	syncData->m_atomicIndex = 0;
	for (dgInt32 j = 0; j < threadCounts; j ++) {
		world->QueueJob (UpdateBodyVelocityParallelKernelSimd, syncData, world);
	}
	world->SynchronizationBarrier();

	if (hasJointFeeback) {
		for (int i = 0; i < syncData->m_batchesCount; i ++) {
			syncData->m_atomicIndex = syncData->m_jointBatches[i].m_start;
			syncData->m_jointsInBatch = syncData->m_jointBatches[i].m_count + syncData->m_atomicIndex;
			for (dgInt32 j = 0; j < threadCounts; j ++) {
				world->QueueJob (KinematicCallbackUpdateParallelKernelSimd, syncData, world);
			}
			world->SynchronizationBarrier();
		}
	}
}



void dgWorldDynamicUpdate::IntegrateInslandParallelKernelSimd (void* const context, void* const worldContext, dgInt32 threadID) 
{
	dgParallelSolverSyncData* const syncData = (dgParallelSolverSyncData*) context;
	dgWorld* const world = (dgWorld*) worldContext;
	dgFloat32 timestep = syncData->m_timestep;
	dgInt32* const atomicIndex = &syncData->m_islandCountCounter; 
	const dgIsland* const islandArray = syncData->m_islandArray;
	for (dgInt32 i = dgAtomicExchangeAndAdd(atomicIndex, 1); i < syncData->m_islandCount;  i = dgAtomicExchangeAndAdd(atomicIndex, 1)) {
		const dgIsland* const island = &islandArray[i];
		world->IntegrateArray (island, DG_SOLVER_MAX_ERROR, timestep, threadID); 
	}
} 


void dgWorldDynamicUpdate::IntegrateInslandParallelSimd(dgParallelSolverSyncData* const syncData) const
{
	dgWorld* const world = (dgWorld*) this;
	dgInt32 threadCounts = world->GetThreadCount();	

	syncData->m_atomicIndex = 0;
	for (dgInt32 j = 0; j < threadCounts; j ++) {
		world->QueueJob (IntegrateInslandParallelKernelSimd, syncData, world);
	}
	world->SynchronizationBarrier();
}



