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
#include "dgWorld.h"
#include "dgConstraint.h"
#include "dgDynamicBody.h"
#include "dgDynamicBody.h"
#include "dgCollisionInstance.h"
#include "dgWorldDynamicUpdate.h"



void dgWorldDynamicUpdate::CalculateIslandReactionForcesSimd (dgIsland* const island, dgFloat32 timestep, dgInt32 threadID) const
{
	dgAssert (0);
/*
	if (!(island->m_isContinueCollision && island->m_jointCount)) {
		dgInt32 rowBase = BuildJacobianMatrixSimd(island, threadID, timestep);
		CalculateReactionsForcesSimd (island, rowBase, threadID, timestep, DG_SOLVER_MAX_ERROR);
		IntegrateArray (island, DG_SOLVER_MAX_ERROR, timestep, threadID); 
	} else {

		// calculate reaction force sand new velocities
		dgInt32 rowBase = BuildJacobianMatrixSimd (island, threadID, timestep);
		CalculateReactionsForcesSimd (island, rowBase, threadID, timestep, DG_SOLVER_MAX_ERROR);

		// see if the island goes to sleep
		bool isAutoSleep = true;
		bool stackSleeping = true;
		dgInt32 sleepCounter = 10000;

		dgWorld* const world = (dgWorld*) this;
		dgInt32 bodyCount = island->m_bodyCount;
		dgBodyInfo* const bodyArrayPtr = (dgBodyInfo*) &world->m_bodiesMemory[0]; 
		dgBodyInfo* const bodyArray = &bodyArrayPtr[island->m_bodyStart];

		dgVector zero (dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f));

		dgFloat32 forceDamp = DG_FREEZZING_VELOCITY_DRAG;
		dgFloat32 maxAccel = dgFloat32 (0.0f);
		dgFloat32 maxAlpha = dgFloat32 (0.0f);
		dgFloat32 maxSpeed = dgFloat32 (0.0f);
		dgFloat32 maxOmega = dgFloat32 (0.0f);

		dgFloat32 speedFreeze = world->m_freezeSpeed2;
		dgFloat32 accelFreeze = world->m_freezeAccel2;
		for (dgInt32 i = 1; i < bodyCount; i ++) {
			dgDynamicBody* const body = bodyArray[i].m_body;
			dgAssert (body->m_invMass.m_w);

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
				body->m_veloc = body->m_veloc.Scale (forceDamp);
				body->m_omega = body->m_omega.Scale (forceDamp);
			}
			body->m_equilibrium = dgUnsigned32 (equilibrium);
			stackSleeping &= equilibrium;
			isAutoSleep &= body->m_autoSleep;

			sleepCounter = dgMin (sleepCounter, body->m_sleepingCounter);

			// clear force and torque accumulators
			body->m_accel = zero;
			body->m_alpha = zero;
		}

		if (isAutoSleep) {
			if (stackSleeping) {
				// the island went to sleep mode, 
				for (dgInt32 i = 1; i < bodyCount; i ++) {
					dgDynamicBody* const body = bodyArray[i].m_body;
					body->m_netForce = zero;
					body->m_netTorque = zero;
					body->m_veloc = zero;
					body->m_omega = zero;
				}
			} else {
				// island is not sleeping but may be at rest with small residual velocity for a long time
				// see if we can force to go to sleep
				if ((maxAccel > world->m_sleepTable[DG_SLEEP_ENTRIES - 1].m_maxAccel) ||
					(maxAlpha > world->m_sleepTable[DG_SLEEP_ENTRIES - 1].m_maxAlpha) ||
					(maxSpeed > world->m_sleepTable[DG_SLEEP_ENTRIES - 1].m_maxVeloc) ||
					(maxOmega > world->m_sleepTable[DG_SLEEP_ENTRIES - 1].m_maxOmega)) {
						for (dgInt32 i = 1; i < bodyCount; i ++) {
							dgDynamicBody* const body = bodyArray[i].m_body;
							body->m_sleepingCounter = 0;
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
						// force island to sleep
						stackSleeping = true;
						for (dgInt32 i = 1; i < bodyCount; i ++) {
							dgDynamicBody* const body = bodyArray[i].m_body;
							body->m_netForce = zero;
							body->m_netTorque = zero;
							body->m_veloc = zero;
							body->m_omega = zero;
							body->m_equilibrium = true;
						}
					} else {
						sleepCounter ++;
						for (dgInt32 i = 1; i < bodyCount; i ++) {
							dgDynamicBody* const body = bodyArray[i].m_body;
							body->m_sleepingCounter = sleepCounter;
						}
					}
				}
			}
		} 


		if (!(isAutoSleep & stackSleeping)) {
			// island is not sleeping, need to integrate island velocity

			dgUnsigned32 lru = world->GetBroadPhase()->m_lru;
			dgInt32 jointCount = island->m_jointCount;
			dgJointInfo* const constraintArrayPtr = (dgJointInfo*) &world->m_jointsMemory[0];
			dgJointInfo* const constraintArray = &constraintArrayPtr[island->m_jointStart];

			dgFloat32 timeRemaining = timestep;
			const dgFloat32 timeTol = dgFloat32 (0.01f) * timestep;
			for (dgInt32 i = 0; (i < DG_MAX_CONTINUE_COLLISON_STEPS) && (timeRemaining > timeTol); i ++) {

				// calculate the closest time to impact 
				dgFloat32 timeToImpact = timeRemaining;
				for (dgInt32 j = 0; j < jointCount; j ++) {
					dgContact* const contact = (dgContact*) constraintArray[j].m_joint;
					if (contact->GetId() == dgConstraint::m_contactConstraint) {
						dgDynamicBody* const body0 = (dgDynamicBody*)contact->m_body0;
						dgDynamicBody* const body1 = (dgDynamicBody*)contact->m_body1;
						if (body0->m_continueCollisionMode | body1->m_continueCollisionMode) {
							dgVector p;
							dgVector q;
							dgVector normal;
							dgFloat32 t = world->CalculateTimeToImpact (contact, timeToImpact, threadID, p, q, normal);
							if (t < timeToImpact) {
								if (t == dgFloat32 (0.0f)) {
									// apply resolve penetration resolution
									dgAssert (dgAbsf (normal % normal - dgFloat32 (1.0f)) < dgFloat32 (1.0e-2f));
									// determine if bodies are residing away
									dgVector veloc0 (body0->m_veloc + ((p - body0->m_globalCentreOfMass) * body0->m_omega));
									dgVector veloc1 (body1->m_veloc + ((q - body1->m_globalCentreOfMass) * body1->m_omega));
									dgVector relVeloc (veloc1 - veloc0);
									dgFloat32 speed (relVeloc % normal);
									if (speed > dgFloat32 (0.0f)) {
										// bodies are moving toward each other, check if the speed is too small
										if (speed < dgFloat32 (2.0f)) {
											speed = dgMax (dgFloat32 (1.0e-3f), speed);
											dgFloat32 distance = dgMax (body0->m_collision->GetBoxMinRadius(), body1->m_collision->GetBoxMinRadius()) * dgFloat32 (0.25f);
											dgAssert (distance > dgFloat32 (0.0f));
											dgFloat32 maxtime = distance / speed;
											t = dgMax(timeTol * dgFloat32 (2.0f), dgMin(maxtime, timeRemaining));
										}
									} else {
										// bodies are residing, let the move by a fraction of the tome step
										t = timeRemaining;
									}
								}
								timeToImpact = t;
							}
						}
					}
				}

				if (timeToImpact > timeTol) {
					timeRemaining -= timeToImpact;
					for (dgInt32 j = 1; j < bodyCount; j ++) {
						dgDynamicBody* const body = bodyArray[j].m_body;
						body->IntegrateVelocity(timeToImpact);
						body->UpdateWorlCollisionMatrix();
					}
				} else {
					for (dgInt32 j = 0; j < jointCount; j ++) {
						dgContact* const contact = (dgContact*) constraintArray[j].m_joint;
						if (contact->GetId() == dgConstraint::m_contactConstraint) {
							dgContactPoint contactArray[DG_MAX_CONTATCS];
							dgCollidingPairCollector::dgPair pair;

							contact->m_maxDOF = 0;
							contact->m_broadphaseLru = lru;
							pair.m_contact = contact;
							pair.m_cacheIsValid = false;
							pair.m_contactBuffer = contactArray;
							world->CalculateContacts (&pair, timeRemaining, false, threadID);

							if (pair.m_contactCount) {
								dgAssert (pair.m_contactCount <= (DG_CONSTRAINT_MAX_ROWS / 3));
								world->ProcessContacts (&pair, timeRemaining, threadID);
							}
						}
					}

					dgInt32 rowBase = BuildJacobianMatrixSimd(island, threadID, 0.0f);
					CalculateReactionsForcesSimd (island, rowBase, threadID, 0.0f, DG_SOLVER_MAX_ERROR);
				}
			}

			if (timeRemaining > dgFloat32 (0.0)) {
				for (dgInt32 i = 1; i < bodyCount; i ++) {
					dgDynamicBody* const body = bodyArray[i].m_body;
					body->IntegrateVelocity(timeRemaining);
					body->UpdateMatrix (timeRemaining, threadID);
				}
			} else {
				for (dgInt32 i = 1; i < bodyCount; i ++) {
					dgDynamicBody* const body = bodyArray[i].m_body;
					body->UpdateMatrix (timestep, threadID);
				}
			}
		}
	}	
*/
}

dgInt32 dgWorldDynamicUpdate::BuildJacobianMatrixSimd (dgIsland* const island, dgInt32 threadIndex, dgFloat32 timestep) const
{
dgAssert (0);
return 0;
/*
	dgAssert (island->m_bodyCount >= 2);
	dgWorld* const world = (dgWorld*) this;

	dgInt32 bodyCount = island->m_bodyCount;
	dgBodyInfo* const bodyArrayPtr = (dgBodyInfo*) &world->m_bodiesMemory[0]; 
	dgBodyInfo* const bodyArray = &bodyArrayPtr[island->m_bodyStart];

	dgAssert ((bodyArray[0].m_body->m_accel % bodyArray[0].m_body->m_accel) == dgFloat32 (0.0f));
	dgAssert ((bodyArray[0].m_body->m_alpha % bodyArray[0].m_body->m_alpha) == dgFloat32 (0.0f));

	for (dgInt32 i = 1; i < bodyCount; i ++) {
		dgDynamicBody* const body = bodyArray[i].m_body;
		dgAssert (body->m_invMass.m_w > dgFloat32 (0.0f));
		body->AddDampingAccelerationSimd();
		body->CalcInvInertiaMatrixSimd ();
	}

	dgInt32 jointCount = island->m_jointCount;
	dgInt32 rowBase = island->m_rowsCountBaseBlock;
	dgAssert (rowBase == -1);
	if (rowBase == -1) {
		rowBase = dgAtomicExchangeAndAdd(&m_rowCountAtomicIndex, island->m_rowsCount);
		island->m_rowsCountBaseBlock = rowBase;
	}

	if (jointCount) {
		dgInt32 rowCount = 0;
		dgJointInfo* const constraintArrayPtr = (dgJointInfo*) &world->m_jointsMemory[0];
		dgJointInfo* const constraintArray = &constraintArrayPtr[island->m_jointStart];
	
		if (island->m_hasUnilateralJoints) {
			rowCount = GetJacobianDerivatives (island, threadIndex, false, rowBase, rowCount, timestep);
		}
		rowCount = GetJacobianDerivatives (island, threadIndex, true, rowBase, rowCount, timestep);
		dgAssert (rowCount <= island->m_rowsCount);

		dgJacobianMatrixElement* const matrixRow = &m_solverMemory.m_memory[rowBase];

		dgSimd one (dgFloat32 (1.0f));
		dgSimd zero (dgFloat32 (0.0f));
		dgSimd diagDampConst (DG_PSD_DAMP_TOL);
		for (dgInt32 k = 0; k < jointCount; k ++) {

			const dgJointInfo* const jointInfo = &constraintArray[k];
			dgInt32 index = jointInfo->m_autoPairstart;
			dgInt32 count = jointInfo->m_autoPaircount;
			dgInt32 m0 = jointInfo->m_m0;
			dgInt32 m1 = jointInfo->m_m1;

			dgAssert (m0 >= 0);
			dgAssert (m0 < bodyCount);
			dgAssert (m1 >= 0);
			dgAssert (m1 < bodyCount);

			dgDynamicBody* const body0 = bodyArray[m0].m_body;
			dgDynamicBody* const body1 = bodyArray[m1].m_body;

			dgSimd invMass0 (body0->m_invMass[3]);
			dgMatrix invInertiaTrans0;
			dgSimd::Transpose4x4 ((dgSimd&)invInertiaTrans0[0], (dgSimd&)invInertiaTrans0[1], (dgSimd&)invInertiaTrans0[2], (dgSimd&)invInertiaTrans0[3], 
								  (dgSimd&)body0->m_invWorldInertiaMatrix[0], (dgSimd&)body0->m_invWorldInertiaMatrix[1], 
								  (dgSimd&)body0->m_invWorldInertiaMatrix[2], (dgSimd&)body0->m_invWorldInertiaMatrix[3]);
			
			dgSimd invMass1 (body1->m_invMass[3]);
			dgMatrix invInertiaTrans1;
			dgSimd::Transpose4x4 ((dgSimd&)invInertiaTrans1[0], (dgSimd&)invInertiaTrans1[1], (dgSimd&)invInertiaTrans1[2], (dgSimd&)invInertiaTrans1[3], 
								  (dgSimd&)body1->m_invWorldInertiaMatrix[0], (dgSimd&)body1->m_invWorldInertiaMatrix[1], 
								  (dgSimd&)body1->m_invWorldInertiaMatrix[2], (dgSimd&)body1->m_invWorldInertiaMatrix[3]);

			for (dgInt32 i = 0; i < count; i ++) {
				dgJacobianMatrixElement* const row = &matrixRow[index];
//				row->m_JMinv.m_jacobianM0.m_linear = (dgSimd&)row->m_Jt.m_jacobianM0.m_linear * invMass0;
//				row->m_JMinv.m_jacobianM0.m_angular = invInertiaTrans0.RotateVectorSimd((dgSimd&)row->m_Jt.m_jacobianM0.m_angular);
//				row->m_JMinv.m_jacobianM1.m_linear = (dgSimd&)row->m_Jt.m_jacobianM1.m_linear * invMass1;
//				row->m_JMinv.m_jacobianM1.m_angular = invInertiaTrans1.RotateVectorSimd((dgSimd&)row->m_Jt.m_jacobianM1.m_angular);

				dgSimd JMinvJacobianLinearM0 ((dgSimd&)row->m_Jt.m_jacobianM0.m_linear * invMass0);
				dgSimd JMinvJacobianAngularM0 (invInertiaTrans0.RotateVectorSimd((dgSimd&)row->m_Jt.m_jacobianM0.m_angular));
				dgSimd JMinvJacobianLinearM1 ((dgSimd&)row->m_Jt.m_jacobianM1.m_linear * invMass1);
				dgSimd JMinvJacobianAngularM1 (invInertiaTrans1.RotateVectorSimd((dgSimd&)row->m_Jt.m_jacobianM1.m_angular));

				dgSimd tmpDiag ( JMinvJacobianLinearM0 * (dgSimd&)row->m_Jt.m_jacobianM0.m_linear + 
								 JMinvJacobianAngularM0 * (dgSimd&)row->m_Jt.m_jacobianM0.m_angular +
								 JMinvJacobianLinearM1 * (dgSimd&)row->m_Jt.m_jacobianM1.m_linear + 
								 JMinvJacobianAngularM1 * (dgSimd&)row->m_Jt.m_jacobianM1.m_angular);

				dgSimd tmpAccel ( JMinvJacobianLinearM0 * (dgSimd&)body0->m_accel +
								  JMinvJacobianAngularM0 * (dgSimd&)body0->m_alpha +
								  JMinvJacobianLinearM1 * (dgSimd&)body1->m_accel +
								  JMinvJacobianAngularM1 * (dgSimd&)body1->m_alpha);

				//dgFloat32 extenalAcceleration = -(tmpAccel.m_x + tmpAccel.m_y + tmpAccel.m_z);
				//dgAssert (tmpAccel.m_type.m128_f32[3] == dgFloat32 (0.0f));
				tmpAccel = zero - tmpAccel.AddHorizontal();

				//row->m_deltaAccel = extenalAcceleration;
				//row->m_coordenateAccel += extenalAcceleration;
				tmpAccel.StoreScalar(&row->m_deltaAccel);
				tmpAccel = tmpAccel + dgSimd (row->m_coordenateAccel);
				tmpAccel.StoreScalar(&row->m_coordenateAccel);

				//row->m_force = row->m_jointFeebackForce[0];
				row->m_force = row->m_jointFeebackForce[0];
			
				dgAssert (row->m_diagDamp >= dgFloat32(0.1f));
				dgAssert (row->m_diagDamp <= dgFloat32(100.0f));

				//dgFloat32 stiffness = DG_PSD_DAMP_TOL * row->m_diagDamp;
				//dgFloat32 diag = (tmpDiag.m_x + tmpDiag.m_y + tmpDiag.m_z);
				//row->m_diagDamp = diag * stiffness;
				dgSimd stiffness (dgSimd(row->m_diagDamp) * diagDampConst);
				tmpDiag = tmpDiag.AddHorizontal();
				dgSimd diagDamp (tmpDiag * stiffness);
				diagDamp.StoreScalar (&row->m_diagDamp);

				//diag *= (dgFloat32(1.0f) + stiffness);
				//row->m_invDJMinvJt = dgFloat32(1.0f) / diag;
				dgSimd invDiagDamp (one / (tmpDiag * (one + stiffness)));
				invDiagDamp.StoreScalar(&row->m_invDJMinvJt);

				index ++;
			}
		}
	}
	return rowBase;
*/
}



void dgWorldDynamicUpdate::ApplyExternalForcesAndAccelerationSimd (const dgIsland* const island, dgInt32 rowStart, dgInt32 threadIndex, dgFloat32 timestepSrc, dgFloat32 maxAccNorm) const
{
dgAssert (0);
/*
	dgJacobian* const internalForces = &m_solverMemory.m_internalForces[island->m_bodyStart];

	dgSimd zero (dgFloat32 (0.0f));
	dgInt32 bodyCount = island->m_bodyCount;
	for (dgInt32 i = 0; i < bodyCount; i ++) {
		(dgSimd&) internalForces[i].m_linear = zero;
		(dgSimd&) internalForces[i].m_angular = zero;
	}

	dgInt32 hasJointFeeback = 0;
	dgInt32 jointCount = island->m_jointCount;
	dgWorld* const world = (dgWorld*) this;
	dgJointInfo* const constraintArrayPtr = (dgJointInfo*) &world->m_jointsMemory[0];
	dgJointInfo* const constraintArray = &constraintArrayPtr[island->m_jointStart];
	dgJacobianMatrixElement* const matrixRow = &m_solverMemory.m_memory[rowStart];


	for (dgInt32 i = 0; i < jointCount; i ++) {
		dgInt32 first = constraintArray[i].m_autoPairstart;
		dgInt32 count = constraintArray[i].m_autoPaircount;
		dgInt32 m0 = constraintArray[i].m_m0;
		dgInt32 m1 = constraintArray[i].m_m1;

		dgSimd y0_linear (zero);
		dgSimd y0_angular (zero);
		dgSimd y1_linear (zero);
		dgSimd y1_angular (zero);
		for (dgInt32 j = 0; j < count; j ++) { 
			dgJacobianMatrixElement* const row = &matrixRow[j + first];

			dgAssert (dgCheckFloat(row->m_force));
			dgSimd val (row->m_force);
			val.StoreScalar(&row->m_jointFeebackForce[0]);
			y0_linear = y0_linear + (dgSimd&)row->m_Jt.m_jacobianM0.m_linear * val;
			y0_angular = y0_angular + (dgSimd&)row->m_Jt.m_jacobianM0.m_angular * val;
			y1_linear = y1_linear + (dgSimd&)row->m_Jt.m_jacobianM1.m_linear * val;
			y1_angular = y1_angular + (dgSimd&)row->m_Jt.m_jacobianM1.m_angular * val;
		}
		hasJointFeeback |= (constraintArray[i].m_joint->m_updaFeedbackCallback ? 1 : 0);
		(dgSimd&)internalForces[m0].m_linear = (dgSimd&)internalForces[m0].m_linear + y0_linear;
		(dgSimd&)internalForces[m0].m_angular = (dgSimd&)internalForces[m0].m_angular + y0_angular;
		(dgSimd&)internalForces[m1].m_linear = (dgSimd&)internalForces[m1].m_linear + y1_linear;
		(dgSimd&)internalForces[m1].m_angular = (dgSimd&)internalForces[m1].m_angular + y1_angular;
	}

	dgBodyInfo* const bodyArrayPtr = (dgBodyInfo*) &world->m_bodiesMemory[0]; 
	dgBodyInfo* const bodyArray = &bodyArrayPtr[island->m_bodyStart];
	if (timestepSrc > dgFloat32 (0.0f)) {
		dgSimd timestep (timestepSrc);
		dgSimd toleranceSimd (dgSimd (maxAccNorm) * dgSimd (maxAccNorm));
		for (dgInt32 i = 1; i < bodyCount; i ++) {
			dgDynamicBody* const body = bodyArray[i].m_body;
			(dgSimd&)body->m_accel = (dgSimd&)body->m_accel + (dgSimd&)internalForces[i].m_linear;
			(dgSimd&)body->m_alpha = (dgSimd&)body->m_alpha + (dgSimd&)internalForces[i].m_angular;

			dgSimd accel = (dgSimd&)body->m_accel * dgSimd(body->m_invMass.m_w);
			dgSimd alpha (body->m_invWorldInertiaMatrix.RotateVectorSimd(body->m_alpha));

			dgSimd accelTest ((accel * accel) > toleranceSimd);
			accelTest = accelTest | accelTest.MoveHigh(accelTest);
			accelTest = accelTest | accelTest.PackLow(accelTest);
			(dgSimd&)body->m_accel = accel & (accelTest | accelTest.MoveLow(accelTest));

			dgSimd alphaTest ((alpha * alpha) > toleranceSimd);
			alphaTest = alphaTest | alphaTest.MoveHigh(alphaTest);
			alphaTest = alphaTest | alphaTest.PackLow(alphaTest);
			(dgSimd&)body->m_alpha = alpha & (alphaTest | alphaTest.MoveLow(alphaTest));

			(dgSimd&)body->m_netForce = (dgSimd&)body->m_accel;
			(dgSimd&)body->m_netTorque = (dgSimd&)body->m_alpha;
			(dgSimd&)body->m_veloc = (dgSimd&)body->m_veloc +  accel * timestep;
			(dgSimd&)body->m_omega = (dgSimd&)body->m_omega + alpha * timestep;
		}

		if (hasJointFeeback) {
			for (dgInt32 i = 0; i < jointCount; i ++) {
				if (constraintArray[i].m_joint->m_updaFeedbackCallback) {
					constraintArray[i].m_joint->m_updaFeedbackCallback (*constraintArray[i].m_joint, timestepSrc, threadIndex);
				}
			}
		}
	} else {
		for (dgInt32 i = 1; i < bodyCount; i ++) {
			dgDynamicBody* const body = bodyArray[i].m_body;
			dgSimd linearMomentum ((dgSimd&)internalForces[i].m_linear);
			dgSimd angularMomentum ((dgSimd&)internalForces[i].m_angular);

			(dgSimd&)body->m_veloc = (dgSimd&)body->m_veloc + linearMomentum * (dgSimd (body->m_invMass.m_w));
			(dgSimd&)body->m_omega = (dgSimd&)body->m_omega + body->m_invWorldInertiaMatrix.RotateVectorSimd (angularMomentum);
		}
	}
*/
}


void dgWorldDynamicUpdate::CalculateSimpleBodyReactionsForcesSimd (const dgIsland* const island, dgInt32 rowStart, dgInt32 threadIndex, dgFloat32 timestep, dgFloat32 maxAccNorm) const
{
dgAssert (0);
/*
	dgFloat32 accel[DG_CONSTRAINT_MAX_ROWS];
	dgFloat32 activeRow[DG_CONSTRAINT_MAX_ROWS];
	dgFloat32 lowBound[DG_CONSTRAINT_MAX_ROWS];
	dgFloat32 highBound[DG_CONSTRAINT_MAX_ROWS];
	dgFloat32 deltaForce[DG_CONSTRAINT_MAX_ROWS];
	dgFloat32 deltaAccel[DG_CONSTRAINT_MAX_ROWS];
	dgJacobianPair JMinv[DG_CONSTRAINT_MAX_ROWS];

	dgWorld* const world = (dgWorld*) this;
	dgJointInfo* const constraintArrayPtr = (dgJointInfo*) &world->m_jointsMemory[0];
	dgJointInfo* const constraintArray = &constraintArrayPtr[island->m_jointStart];
	dgInt32 count = constraintArray[0].m_autoPaircount;
	dgAssert (constraintArray[0].m_autoPairstart == 0);

	const dgBodyInfo* const bodyArrayPtr = (dgBodyInfo*) &world->m_bodiesMemory[0]; 
	const dgBodyInfo* const bodyArray = &bodyArrayPtr[island->m_bodyStart];

	dgInt32 m0 = constraintArray[0].m_m0;
	dgInt32 m1 = constraintArray[0].m_m1;
	dgDynamicBody* const body0 = bodyArray[m0].m_body;
	dgDynamicBody* const body1 = bodyArray[m1].m_body;

	dgSimd invMass0 (body0->m_invMass[3]);
	dgMatrix invInertiaTrans0;
	dgSimd::Transpose4x4 ((dgSimd&)invInertiaTrans0[0], (dgSimd&)invInertiaTrans0[1], (dgSimd&)invInertiaTrans0[2], (dgSimd&)invInertiaTrans0[3], 
						  (dgSimd&)body0->m_invWorldInertiaMatrix[0], (dgSimd&)body0->m_invWorldInertiaMatrix[1], 
		                  (dgSimd&)body0->m_invWorldInertiaMatrix[2], (dgSimd&)body0->m_invWorldInertiaMatrix[3]);

	dgSimd invMass1 (body1->m_invMass[3]);
	dgMatrix invInertiaTrans1;
	dgSimd::Transpose4x4 ((dgSimd&)invInertiaTrans1[0], (dgSimd&)invInertiaTrans1[1], (dgSimd&)invInertiaTrans1[2], (dgSimd&)invInertiaTrans1[3], 
						  (dgSimd&)body1->m_invWorldInertiaMatrix[0], (dgSimd&)body1->m_invWorldInertiaMatrix[1], 
						  (dgSimd&)body1->m_invWorldInertiaMatrix[2], (dgSimd&)body1->m_invWorldInertiaMatrix[3]);


	dgSimd one(dgSimd::m_one);
	dgSimd passesCount (dgSimd::m_zero);
	dgJacobianMatrixElement* const matrixRow = &m_solverMemory.m_memory[rowStart];
	for (dgInt32 i = 0; i < count; i ++) {
		dgJacobianMatrixElement* const row = &matrixRow[i];

		dgInt32 frictionIndex = row->m_normalForceIndex;
		dgAssert ((frictionIndex < 0) || ((frictionIndex >= 0) && (matrixRow[frictionIndex].m_force >= dgFloat32 (0.0f))));
		//dgSimd val (matrixRow[frictionIndex].m_force);
		dgSimd val ((frictionIndex < 0) ? one : matrixRow[frictionIndex].m_force);
		dgSimd low (val * dgSimd (row->m_lowerBoundFrictionCoefficent));
		dgSimd high (val * dgSimd (row->m_upperBoundFrictionCoefficent));
		low.StoreScalar(&lowBound[i]);
		high.StoreScalar(&highBound[i]);

		dgSimd force (row->m_force);
		dgSimd test ((force < low) | (force > high));
		dgSimd unit (one.AndNot(test));
		unit.StoreScalar(&activeRow[i]);
		passesCount = passesCount + unit;
		high.GetMin(low.GetMax(force)).StoreScalar(&row->m_force);
	}
	dgInt32 maxPasses = passesCount.GetInt();


	dgSimd zero(dgFloat32 (0.0f));
	dgSimd y0_linear (zero);
	dgSimd y0_angular (zero);
	dgSimd y1_linear (zero);
	dgSimd y1_angular (zero);
	for (dgInt32 i = 0; i < count; i ++) {
		dgJacobianMatrixElement* const row = &matrixRow[i];
		dgSimd val (row->m_force);
		y0_linear = y0_linear + ((dgSimd&)row->m_Jt.m_jacobianM0.m_linear) * val;
		y0_angular = y0_angular + ((dgSimd&)row->m_Jt.m_jacobianM0.m_angular) * val;
		y1_linear = y1_linear + ((dgSimd&)row->m_Jt.m_jacobianM1.m_linear) * val;
		y1_angular = y1_angular + ((dgSimd&)row->m_Jt.m_jacobianM1.m_angular) * val;
	}

	dgSimd akNumSimd (zero);
	dgSimd accNormSimd(zero);
	for (dgInt32 i = 0; i < count; i ++) {
		dgJacobianMatrixElement* const row = &matrixRow[i];
//		dgSimd acc ((dgSimd&)row->m_JMinv.m_jacobianM0.m_linear * y0_linear +
//					(dgSimd&)row->m_JMinv.m_jacobianM0.m_angular * y0_angular +
//					(dgSimd&)row->m_JMinv.m_jacobianM1.m_linear * y1_linear +
//					(dgSimd&)row->m_JMinv.m_jacobianM1.m_angular * y1_angular);

		JMinv[i].m_jacobianM0.m_linear = (dgSimd&)row->m_Jt.m_jacobianM0.m_linear * invMass0;
		JMinv[i].m_jacobianM0.m_angular = invInertiaTrans0.RotateVectorSimd((dgSimd&)row->m_Jt.m_jacobianM0.m_angular);
		JMinv[i].m_jacobianM1.m_linear  = (dgSimd&)row->m_Jt.m_jacobianM1.m_linear * invMass1;
		JMinv[i].m_jacobianM1.m_angular = invInertiaTrans1.RotateVectorSimd((dgSimd&)row->m_Jt.m_jacobianM1.m_angular);

		dgSimd acc ((dgSimd&)JMinv[i].m_jacobianM0.m_linear * y0_linear + 
					(dgSimd&)JMinv[i].m_jacobianM0.m_angular * y0_angular +
					(dgSimd&)JMinv[i].m_jacobianM1.m_linear * y1_linear + 
					(dgSimd&)JMinv[i].m_jacobianM1.m_angular * y1_angular);

		//dgAssert (acc.m_type.m128_f32[3] == dgFloat32 (0.0f));

		dgSimd acceleration (dgSimd (row->m_coordenateAccel) - acc.AddHorizontal() - dgSimd(row->m_force) * dgSimd(row->m_diagDamp));
		acceleration.StoreScalar(&accel[i]);
		dgSimd dForce (acceleration * dgSimd(row->m_invDJMinvJt) * dgSimd (activeRow[i]));
		dForce.StoreScalar(&deltaForce[i]);
		akNumSimd = akNumSimd + acceleration * dForce;
		accNormSimd = accNormSimd.GetMax(acceleration.Abs() * dgSimd (activeRow[i]));
	}

	dgFloat32 akNum;
	dgFloat32 accNorm;
	akNumSimd.StoreScalar(&akNum);
	accNormSimd.StoreScalar(&accNorm);
	
	dgSimd tol_Pos1eNeg8 (dgFloat32 ( 1.0e-8f));
	dgSimd tol_Pos1eNeg16 (dgFloat32 ( 1.0e-16f));
	dgSimd tol_Neg1eNeg16 (dgFloat32 ( 1.0e-16f));
	for (dgInt32 i = 0; (i < maxPasses) && (accNorm > maxAccNorm); i ++) {

		dgSimd y0_linear (zero);
		dgSimd y0_angular (zero);
		dgSimd y1_linear (zero);
		dgSimd y1_angular (zero);
		for (dgInt32 k = 0; k < count; k ++) {
			dgJacobianMatrixElement* const row = &matrixRow[k];
			dgSimd val (deltaForce[k]);
			y0_linear = y0_linear + ((dgSimd&)row->m_Jt.m_jacobianM0.m_linear) * val;
			y0_angular = y0_angular + ((dgSimd&)row->m_Jt.m_jacobianM0.m_angular) * val;
			y1_linear = y1_linear + ((dgSimd&)row->m_Jt.m_jacobianM1.m_linear) * val;
			y1_angular = y1_angular + ((dgSimd&)row->m_Jt.m_jacobianM1.m_angular) * val;
		}


		dgSimd akDen(dgFloat32 (0.0f));
		for (dgInt32 k = 0; k < count; k ++) {
			dgJacobianMatrixElement* const row = &matrixRow[k];
			dgSimd acc ((dgSimd&)JMinv[k].m_jacobianM0.m_linear * y0_linear +
						(dgSimd&)JMinv[k].m_jacobianM0.m_angular * y0_angular +
						(dgSimd&)JMinv[k].m_jacobianM1.m_linear * y1_linear +
						(dgSimd&)JMinv[k].m_jacobianM1.m_angular * y1_angular);

			dgSimd dForce (deltaForce[k]);
			dgSimd dAccel (acc.AddHorizontal() + dForce * dgSimd(row->m_diagDamp));
			dAccel.StoreScalar(&deltaAccel[k]);
			akDen = akDen + dAccel * dForce;
		}

//		dgAssert (akDen.m_type.m128_f32[0] > dgFloat32 (0.0f));
//		dgAssert (akDen.m_type.m128_f32[1] > dgFloat32 (0.0f));
//		dgAssert (akDen.m_type.m128_f32[2] > dgFloat32 (0.0f));
//		dgAssert (akDen.m_type.m128_f32[3] > dgFloat32 (0.0f));
		akDen = akDen.GetMax (tol_Pos1eNeg16);
		dgSimd akSimd (akNumSimd / akDen);

		dgInt32 clampedForceIndex = -1;
		dgSimd clampedForceIndexValueSimd (dgFloat32(0.0f));
		for (dgInt32 k = 0; k < count; k ++) {
			if (activeRow[k]) {
				dgJacobianMatrixElement* const row = &matrixRow[k];
				dgSimd force (row->m_force);
				dgSimd dForce (deltaForce[k]);
				dgSimd val__ (force + akSimd * dForce);
				if ((dForce < tol_Neg1eNeg16).GetInt()) {
					dgSimd low(lowBound[k])	;
					if ((val__ < low).GetInt()) {
						akSimd = zero.GetMax ((low - force) / dForce);
						clampedForceIndex = k;
						clampedForceIndexValueSimd = low;
						dgSimd test (akSimd < tol_Pos1eNeg8);
						akSimd = akSimd.GetMax(zero);
						if (test.GetInt()) {
							break;
						}
					}
				} else if ((dForce > tol_Pos1eNeg16).GetInt()) {
					dgSimd high(highBound[k])	;
					if ((val__ > high).GetInt()) {
						akSimd = zero.GetMax ((high - force) / dForce);
						clampedForceIndex = k;
						clampedForceIndexValueSimd = high;
						dgSimd test (akSimd < tol_Pos1eNeg8);
						akSimd = akSimd.GetMax(zero);
						if (test.GetInt()) {
							break;
						}
					}
				}
			}
		}


		dgFloat32 ak;
		akSimd.StoreScalar(&ak);
		dgFloat32 clampedForceIndexValue;
		clampedForceIndexValueSimd.StoreScalar(&clampedForceIndexValue);

		accNormSimd = zero;
		if (ak == dgFloat32 (0.0f) && (clampedForceIndex != -1)) {
			dgAssert (clampedForceIndex !=-1);
			akNum = dgFloat32 (0.0f);

			activeRow[clampedForceIndex] = dgFloat32 (0.0f);
			deltaForce[clampedForceIndex] = dgFloat32 (0.0f);
			matrixRow[clampedForceIndex].m_force = clampedForceIndexValue;
			for (dgInt32 k = 0; k < count; k ++) {
				if (activeRow[k]) {
					dgJacobianMatrixElement* const row = &matrixRow[k];
					dgFloat32 val = lowBound[k] - row->m_force;
					if ((dgAbsf (val) < dgFloat32 (1.0e-5f)) && (accel[k] < dgFloat32 (0.0f))) {
						row->m_force = lowBound[k];
						activeRow[k] = dgFloat32 (0.0f);
						deltaForce[k] = dgFloat32 (0.0f); 

					} else {
						val = highBound[k] - row->m_force;
						if ((dgAbsf (val) < dgFloat32 (1.0e-5f)) && (accel[k] > dgFloat32 (0.0f))) {
							row->m_force = highBound[k];
							activeRow[k] = dgFloat32 (0.0f);
							deltaForce[k] = dgFloat32 (0.0f); 
						} else {
							dgAssert (activeRow[k] > dgFloat32 (0.0f));
							deltaForce[k] = accel[k] * row->m_invDJMinvJt;
							akNum += accel[k] * deltaForce[k];
							//accNorm = GetMax (dgAbsf (accel[k]), accNorm);
							accNormSimd = accNormSimd.GetMax (dgSimd(accel[k]).Abs());
						}
					}
				}
			}


			i = -1;
			maxPasses = dgMax (maxPasses - 1, 1); 

		} else if (clampedForceIndex >= 0) {
			akNum = dgFloat32(0.0f);
			activeRow[clampedForceIndex] = dgFloat32 (0.0f);
			for (dgInt32 k = 0; k < count; k ++) {
				dgJacobianMatrixElement* const row = &matrixRow[k];
				row->m_force += ak * deltaForce[k];
				accel[k] -= ak * deltaAccel[k];
				//accNorm = GetMax (dgAbsf (accel[k] * activeRow[k]), accNorm);
				accNormSimd = accNormSimd.GetMax(dgSimd(accel[k]).Abs() * dgSimd(activeRow[k]));
				dgAssert (dgCheckFloat(row->m_force));
				dgAssert (dgCheckFloat(accel[k]));

				deltaForce[k] = accel[k] * row->m_invDJMinvJt * activeRow[k];
				akNum += deltaForce[k] * accel[k];
			}
			matrixRow[clampedForceIndex].m_force = clampedForceIndexValue;

			i = -1;
			maxPasses = dgMax (maxPasses - 1, 1); 

		} else {
			//accNorm = dgFloat32(0.0f);
			for (dgInt32 k = 0; k < count; k ++) {
				dgJacobianMatrixElement* const row = &matrixRow[k];
				row->m_force += ak * deltaForce[k];
				accel[k] -= ak * deltaAccel[k];
				//accNorm = GetMax (dgAbsf (accel[k] * activeRow[k]), accNorm);
				accNormSimd = accNormSimd.GetMax(dgSimd(accel[k]).Abs() * dgSimd(activeRow[k]));
				dgAssert (dgCheckFloat(row->m_force));
				dgAssert (dgCheckFloat(accel[k]));
			}

			if (accNorm > maxAccNorm) {
				dgFloat32 akDen = akNum;
				akNum = dgFloat32(0.0f);
				for (dgInt32 k = 0; k < count; k ++) {
					deltaAccel[k] = accel[k] * matrixRow[k].m_invDJMinvJt * activeRow[k];
					akNum += accel[k] * deltaAccel[k];
				}

				dgAssert (akDen > dgFloat32(0.0f));
				akDen = dgMax (akDen, dgFloat32 (1.0e-17f));
				ak = dgFloat32 (akNum / akDen);
				for (dgInt32 k = 0; k < count; k ++) {
					deltaForce[k] = deltaAccel[k] + ak * deltaForce[k];
				}
			}
		}
		akNumSimd = dgSimd (akNum);
		accNormSimd.StoreScalar(&accNorm);
	}
*/
}


void dgWorldDynamicUpdate::CalculateForcesGameModeSimd (const dgIsland* const island, dgInt32 rowStart, dgInt32 threadIndex, dgFloat32 timestepSrc, dgFloat32 maxAccNorm) const
{
dgAssert (0);
/*
	dgJacobian* const internalForces = &m_solverMemory.m_internalForces[island->m_bodyStart];

	dgSimd zero (dgSimd::m_zero);

	dgInt32 bodyCount = island->m_bodyCount;
	dgWorld* const world = (dgWorld*) this;
	dgBodyInfo* const bodyArrayPtr = (dgBodyInfo*) &world->m_bodiesMemory[0]; 
	dgBodyInfo* const bodyArray = &bodyArrayPtr[island->m_bodyStart];
	for (dgInt32 i = 1; i < bodyCount; i ++) {
		dgDynamicBody* const body = bodyArray[i].m_body;
//		(dgSimd&) internalVeloc[i].m_linear = zero;
//		(dgSimd&) internalVeloc[i].m_angular = zero;
		(dgSimd&) internalForces[i].m_linear = zero;
		(dgSimd&) internalForces[i].m_angular = zero;
		// re use these variables for temp storage 
		(dgSimd&) body->m_netForce = (dgSimd&) body->m_veloc;
		(dgSimd&) body->m_netTorque = (dgSimd&) body->m_omega;
	}
//	(dgSimd&) internalVeloc[0].m_linear = zero;
//	(dgSimd&) internalVeloc[0].m_angular = zero;
	(dgSimd&) internalForces[0].m_linear = zero;
	(dgSimd&) internalForces[0].m_angular = zero;

	dgJacobianMatrixElement* const matrixRow = &m_solverMemory.m_memory[rowStart];

	dgInt32 jointCount = island->m_jointCount;
	dgJointInfo* const constraintArrayPtr = (dgJointInfo*) &world->m_jointsMemory[0];
	dgJointInfo* const constraintArray = &constraintArrayPtr[island->m_jointStart];
	for (dgInt32 i = 0; i < jointCount; i ++) {
		dgInt32 first = constraintArray[i].m_autoPairstart;
		dgInt32 count = constraintArray[i].m_autoPaircount;

		dgInt32 m0 = constraintArray[i].m_m0;
		dgInt32 m1 = constraintArray[i].m_m1;

		dgSimd y0_linear = zero;
		dgSimd y0_angular = zero;
		dgSimd y1_linear = zero;
		dgSimd y1_angular = zero;
		for (dgInt32 j = 0; j < count; j ++) { 
			dgJacobianMatrixElement* const row = &matrixRow[j + first];
			dgAssert (dgCheckFloat(row->m_force));
			dgSimd val (row->m_force);
			y0_linear  = y0_linear + (dgSimd&)row->m_Jt.m_jacobianM0.m_linear * val;
			y0_angular = y0_angular + (dgSimd&) row->m_Jt.m_jacobianM0.m_angular * val;
			y1_linear  = y1_linear +  (dgSimd&) row->m_Jt.m_jacobianM1.m_linear * val;
			y1_angular = y1_angular + (dgSimd&) row->m_Jt.m_jacobianM1.m_angular * val;
		}
		(dgSimd&)internalForces[m0].m_linear = (dgSimd&) internalForces[m0].m_linear + y0_linear;
		(dgSimd&)internalForces[m0].m_angular = (dgSimd&) internalForces[m0].m_angular +  y0_angular;
		(dgSimd&)internalForces[m1].m_linear = (dgSimd&) internalForces[m1].m_linear +  y1_linear;
		(dgSimd&)internalForces[m1].m_angular = (dgSimd&)internalForces[m1].m_angular +  y1_angular;
	}

	//dgFloat32 invTimestepSrc = dgFloat32 (1.0f) / timestepSrc;
	dgFloat32 invTimestepSrc = (timestepSrc > dgFloat32 (0.0f)) ? dgFloat32 (1.0f) / timestepSrc : dgFloat32 (0.0f);
	dgFloat32 invStep = (dgFloat32 (1.0f) / dgFloat32 (LINEAR_SOLVER_SUB_STEPS));
	dgFloat32 timestep =  timestepSrc * invStep;
	dgFloat32 invTimestep = invTimestepSrc * dgFloat32 (LINEAR_SOLVER_SUB_STEPS);
	dgAssert (bodyArray[0].m_body == world->m_sentionelBody);

	dgSimd cacheForce[DG_CONSTRAINT_MAX_ROWS * sizeof (dgFloat32) / DG_VECTOR_SIMD_SIZE];
	cacheForce[0] = dgSimd::m_one;
	dgFloat32* const normalForce = (dgFloat32*) &cacheForce[1];


	dgSimd signMask (dgSimd::m_signMask);
	dgSimd timeStepSimd (timestep);
	dgFloat32 firstPassCoef = dgFloat32 (0.0f);
	dgInt32 maxPasses = dgInt32 (world->m_solverMode + DG_BASE_ITERATION_COUNT);

	for (dgInt32 step = 0; step < LINEAR_SOLVER_SUB_STEPS; step ++) {
		dgJointAccelerationDecriptor joindDesc;
		joindDesc.m_timeStep = timestep;
		joindDesc.m_invTimeStep = invTimestep;
		joindDesc.m_firstPassCoefFlag = firstPassCoef;

		for (dgInt32 curJoint = 0; curJoint < jointCount; curJoint ++) {
			joindDesc.m_rowsCount = constraintArray[curJoint].m_autoPaircount;
			joindDesc.m_rowMatrix = &matrixRow[constraintArray[curJoint].m_autoPairstart];
			constraintArray[curJoint].m_joint->JointAccelerationsSimd (&joindDesc);
		}

		firstPassCoef = dgFloat32 (1.0f);
		dgFloat32 accNorm = maxAccNorm * dgFloat32 (2.0f);
		for (dgInt32 passes = 0; (passes < maxPasses) && (accNorm > maxAccNorm); passes ++) {
			dgSimd accNormSimd = zero;

			for (dgInt32 curJoint = 0; curJoint < jointCount; curJoint ++) {
				dgInt32 index = constraintArray[curJoint].m_autoPairstart;
				dgInt32 rowsCount = constraintArray[curJoint].m_autoPaircount;
				dgInt32 m0 = constraintArray[curJoint].m_m0;
				dgInt32 m1 = constraintArray[curJoint].m_m1;

				dgSimd linearM0  = (dgSimd&)internalForces[m0].m_linear;
				dgSimd angularM0 = (dgSimd&)internalForces[m0].m_angular;
				dgSimd linearM1  = (dgSimd&)internalForces[m1].m_linear;
				dgSimd angularM1 = (dgSimd&)internalForces[m1].m_angular;

				dgDynamicBody* const body0 = bodyArray[m0].m_body;
				dgDynamicBody* const body1 = bodyArray[m1].m_body;

				dgSimd invMass0 (body0->m_invMass[3]);
				dgMatrix invInertiaTrans0;
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

//					dgSimd a ((dgSimd&)row->m_JMinv.m_jacobianM0.m_linear * linearM0 +
//							  (dgSimd&)row->m_JMinv.m_jacobianM0.m_angular * angularM0 +
//							  (dgSimd&)row->m_JMinv.m_jacobianM1.m_linear * linearM1 +
//							  (dgSimd&)row->m_JMinv.m_jacobianM1.m_angular * angularM1);

					dgSimd JMinvJacobianLinearM0 ((dgSimd&)row->m_Jt.m_jacobianM0.m_linear * invMass0);
					dgSimd JMinvJacobianAngularM0 (invInertiaTrans0.RotateVectorSimd((dgSimd&)row->m_Jt.m_jacobianM0.m_angular));
					dgSimd JMinvJacobianLinearM1 ((dgSimd&)row->m_Jt.m_jacobianM1.m_linear * invMass1);
					dgSimd JMinvJacobianAngularM1 (invInertiaTrans1.RotateVectorSimd((dgSimd&)row->m_Jt.m_jacobianM1.m_angular));

					dgSimd a (JMinvJacobianLinearM0 * linearM0 + 
							  JMinvJacobianAngularM0 * angularM0 +
						      JMinvJacobianLinearM1 * linearM1 + 
						      JMinvJacobianAngularM1 * angularM1);


					//dgAssert (a.m_type.m128_f32[3] == dgFloat32 (0.0f));
					dgSimd force (row->m_force);
					a = dgSimd(row->m_coordenateAccel) - a.AddHorizontal() - force * dgSimd(row->m_diagDamp);
					dgSimd f (force + a * dgSimd(row->m_invDJMinvJt));

					dgInt32 frictionIndex = row->m_normalForceIndex;
					dgAssert (((frictionIndex < 0) && (normalForce[frictionIndex] == dgFloat32 (1.0f))) || ((frictionIndex >= 0) && (normalForce[frictionIndex] >= dgFloat32 (0.0f))));

					dgSimd frictionNormal (normalForce[frictionIndex]);
					dgSimd lowerFrictionForce (frictionNormal * dgSimd(row->m_lowerBoundFrictionCoefficent));
					dgSimd upperFrictionForce (frictionNormal * dgSimd(row->m_upperBoundFrictionCoefficent));

					a = a.AndNot((f > upperFrictionForce) | (f < lowerFrictionForce));
					f = f.GetMax(lowerFrictionForce).GetMin(upperFrictionForce);

					accNormSimd = accNormSimd.GetMax(a & signMask);

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
			accNormSimd.StoreScalar(&accNorm);
		}

		if (timestep != dgFloat32 (0.0f)) {
			for (dgInt32 i = 1; i < bodyCount; i ++) {
				dgDynamicBody* const body = bodyArray[i].m_body;
				dgSimd force ((dgSimd&)body->m_accel + (dgSimd&)internalForces[i].m_linear);
				dgSimd torque((dgSimd&)body->m_alpha + (dgSimd&)internalForces[i].m_angular);
				dgSimd accel (force * dgSimd(body->m_invMass.m_w));
				dgSimd alpha (body->m_invWorldInertiaMatrix.RotateVectorSimd(torque));
				(dgSimd&)body->m_veloc = (dgSimd&)body->m_veloc + accel * timeStepSimd;
				(dgSimd&)body->m_omega = (dgSimd&)body->m_omega + alpha * timeStepSimd;
//				(dgSimd&)internalVeloc[i].m_linear = (dgSimd&)internalVeloc[i].m_linear + (dgSimd&) body->m_veloc;
//				(dgSimd&)internalVeloc[i].m_angular = (dgSimd&)internalVeloc[i].m_angular + (dgSimd&) body->m_omega;
			}
		} else {
			for (dgInt32 i = 1; i < bodyCount; i ++) {
				dgDynamicBody* const body = bodyArray[i].m_body;
				dgSimd linearMomentum = (dgSimd&)internalForces[i].m_linear;
				dgSimd angularMomentum = (dgSimd&)internalForces[i].m_angular;

				(dgSimd&)body->m_veloc = (dgSimd&)body->m_veloc + linearMomentum * dgSimd (body->m_invMass.m_w);
				(dgSimd&)body->m_omega = body->m_invWorldInertiaMatrix.RotateVectorSimd (angularMomentum);

//				(dgSimd&)internalVeloc[i].m_linear = (dgSimd&)internalVeloc[i].m_linear + (dgSimd&)body->m_veloc;
//				(dgSimd&)internalVeloc[i].m_angular = (dgSimd&)internalVeloc[i].m_angular + (dgSimd&)body->m_omega;
			}
		}
	}

	dgInt32 hasJointFeeback = 0;
	if (timestep != dgFloat32 (0.0f)) {
		for (dgInt32 i = 0; i < jointCount; i ++) {
			dgInt32 first = constraintArray[i].m_autoPairstart;
			dgInt32 count = constraintArray[i].m_autoPaircount;

			for (dgInt32 j = 0; j < count; j ++) { 
				dgJacobianMatrixElement* const row = &matrixRow[j + first];
				dgFloat32 val = row->m_force; 
				dgAssert (dgCheckFloat(val));
				row->m_jointFeebackForce[0] = val;
			}
			hasJointFeeback |= (constraintArray[i].m_joint->m_updaFeedbackCallback ? 1 : 0);
		}

		dgSimd invStepSimd (invStep);	
		dgSimd invTimeStepSimd (invTimestepSrc);	
		dgSimd accelerationTolerance (maxAccNorm, maxAccNorm, maxAccNorm, dgFloat32 (0.0f));	

		for (dgInt32 i = 1; i < bodyCount; i ++) {
			dgDynamicBody* const body = bodyArray[i].m_body;

			// the initial velocity and angular velocity were stored in net force and net torque, for memory saving
			dgSimd accel (((dgSimd&) body->m_veloc - (dgSimd&) body->m_netForce) * invTimeStepSimd);
			dgSimd alpha (((dgSimd&) body->m_omega - (dgSimd&) body->m_netTorque) * invTimeStepSimd);

			dgSimd accelTest ((accel * accel) > accelerationTolerance);
			accelTest = accelTest | accelTest.MoveHigh(accelTest);
			accelTest = accelTest | accelTest.PackLow(accelTest);
			(dgSimd&)body->m_accel = accel & (accelTest | accelTest.MoveLow(accelTest));

			dgSimd alphaTest ((alpha * alpha) > accelerationTolerance);
			alphaTest = alphaTest | alphaTest.MoveHigh(alphaTest);
			alphaTest = alphaTest | alphaTest.PackLow(alphaTest);
			(dgSimd&)body->m_alpha = alpha & (alphaTest | alphaTest.MoveLow(alphaTest));

			(dgSimd&)body->m_netForce = accel * dgSimd (body->m_mass[3]);

			alpha = body->m_matrix.UnrotateVectorSimd(alpha);
			(dgSimd&)body->m_netTorque = body->m_matrix.RotateVectorSimd(alpha * (dgSimd&)body->m_mass);
		}

		if (hasJointFeeback) {
			for (dgInt32 i = 0; i < jointCount; i ++) {
				if (constraintArray[i].m_joint->m_updaFeedbackCallback) {
					constraintArray[i].m_joint->m_updaFeedbackCallback (*constraintArray[i].m_joint, timestepSrc, threadIndex);
				}
			}
		}
	}
*/
}

dgFloat32 dgWorldDynamicUpdate::CalculateJointForcesSimd (const dgIsland* const island, dgInt32 rowStart, dgInt32 joint, dgFloat32* const forceStep, dgFloat32 maxAccNorm, const dgJacobianPair* const JMinv) const
{
	dgFloat32 deltaAccel[DG_CONSTRAINT_MAX_ROWS];
	dgFloat32 deltaForce[DG_CONSTRAINT_MAX_ROWS];
	dgFloat32 activeRow[DG_CONSTRAINT_MAX_ROWS];
	dgFloat32 lowBound[DG_CONSTRAINT_MAX_ROWS];
	dgFloat32 highBound[DG_CONSTRAINT_MAX_ROWS];

	dgWorld* const world = (dgWorld*) this;
	dgJointInfo* const constraintArrayPtr = (dgJointInfo*) &world->m_jointsMemory[0];
	dgJointInfo* const constraintArray = &constraintArrayPtr[island->m_jointStart];

	const dgJointInfo* const jointInfo = &constraintArray[joint];
	dgInt32 first___ = jointInfo->m_autoPairstart;
	dgInt32 count = jointInfo->m_autoPaircount;

	dgInt32 maxPasses = count;
	dgFloat32 akNum = dgFloat32 (0.0f);
	dgFloat32 accNorm = dgFloat32(0.0f);

	dgJacobianMatrixElement* const matrixRow = &m_solverMemory.m_memory[rowStart + first___];
	for (dgInt32 j = 0; j < count; j ++) {
		dgJacobianMatrixElement* const row = &matrixRow[j];

		dgInt32 frictionIndex = row->m_normalForceIndex;
		dgAssert ((frictionIndex < 0) || ((frictionIndex >= 0) && (matrixRow[frictionIndex].m_force >= dgFloat32 (0.0f))));
		dgFloat32 val = (frictionIndex < 0) ? dgFloat32 (1.0f) : matrixRow[frictionIndex].m_force;

		lowBound[j] = val * row->m_lowerBoundFrictionCoefficent;
		highBound[j] = val * row->m_upperBoundFrictionCoefficent;

		activeRow[j] = dgFloat32 (1.0f);
		forceStep[j] = row->m_force;
		if (row->m_force < lowBound[j]) {
			maxPasses --;
			row->m_force = lowBound[j];
			activeRow[j] = dgFloat32 (0.0f);
		} else if (row->m_force > highBound[j]) {
			maxPasses --;
			row->m_force = highBound[j];
			activeRow[j] = dgFloat32 (0.0f);
		}

		deltaForce[j] = row->m_accel * row->m_invDJMinvJt * activeRow[j];
		akNum += row->m_accel * deltaForce[j];
		accNorm = dgMax (dgAbsf (row->m_accel * activeRow[j]), accNorm);
	}

	dgFloat32 retAccel = accNorm;
	dgFloat32 clampedForceIndexValue = dgFloat32(0.0f);
	dgVector zero (dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f));
	for (dgInt32 i = 0; (i < maxPasses) && (accNorm >  maxAccNorm); i ++) {
		dgJacobian y0;
		dgJacobian y1;
		y0.m_linear = zero;
		y0.m_angular = zero;
		y1.m_linear = zero;
		y1.m_angular = zero;

		for (dgInt32 j = 0; j < count; j ++) {
			dgJacobianMatrixElement* const row = &matrixRow[j];
			dgFloat32 val = deltaForce[j]; 
			y0.m_linear += row->m_Jt.m_jacobianM0.m_linear.Scale (val);
			y0.m_angular += row->m_Jt.m_jacobianM0.m_angular.Scale (val);
			y1.m_linear += row->m_Jt.m_jacobianM1.m_linear.Scale (val);
			y1.m_angular += row->m_Jt.m_jacobianM1.m_angular.Scale (val);

		}

		dgFloat32 akDen = dgFloat32 (0.0f);
		for (dgInt32 j = 0; j < count; j ++) {
			dgJacobianMatrixElement* const row = &matrixRow[j];
			dgVector acc (JMinv[j].m_jacobianM0.m_linear.CompProduct(y0.m_linear) +
						  JMinv[j].m_jacobianM0.m_angular.CompProduct(y0.m_angular) +
			              JMinv[j].m_jacobianM1.m_linear.CompProduct(y1.m_linear) +
			              JMinv[j].m_jacobianM1.m_angular.CompProduct(y1.m_angular));

			deltaAccel[j] = acc.m_x + acc.m_y + acc.m_z + deltaForce[j] * row->m_diagDamp;
			akDen += deltaAccel[j] * deltaForce[j];
		}
		dgAssert (akDen > dgFloat32 (0.0f));
		akDen = dgMax (akDen, dgFloat32(1.0e-16f));
		dgAssert (dgAbsf (akDen) >= dgFloat32(1.0e-16f));
		dgFloat32 ak = akNum / akDen;

		dgInt32 clampedForceIndex = -1;
		for (dgInt32 j = 0; j < count; j ++) {
			if (activeRow[j]) {
				dgJacobianMatrixElement* const row = &matrixRow[j];
				if (deltaForce[j] < dgFloat32 (-1.0e-16f)) {
					dgFloat32 val = row->m_force + ak * deltaForce[j];
					if (val < lowBound[j]) {
						ak = dgMax ((lowBound[j] - row->m_force) / deltaForce[j], dgFloat32 (0.0f));
						clampedForceIndex = j;
						clampedForceIndexValue = lowBound[j];
						if (ak < dgFloat32 (1.0e-8f)) {
							ak = dgFloat32 (0.0f);
							break;
						}
					}
				} else if (deltaForce[j] > dgFloat32 (1.0e-16f)) {
					dgFloat32 val = row->m_force + ak * deltaForce[j];
					if (val > highBound[j]) {
						ak = dgMax ((highBound[j] - row->m_force) / deltaForce[j], dgFloat32 (0.0f));
						clampedForceIndex = j;
						clampedForceIndexValue = highBound[j];
						if (ak < dgFloat32 (1.0e-8f)) {
							ak = dgFloat32 (0.0f);
							break;
						}
					}
				}
			}
		}

		if (ak == dgFloat32 (0.0f) && (clampedForceIndex != -1)) {
			dgAssert (clampedForceIndex !=-1);
			akNum = dgFloat32 (0.0f);
			accNorm = dgFloat32(0.0f);

			activeRow[clampedForceIndex] = dgFloat32 (0.0f);
			deltaForce[clampedForceIndex] = dgFloat32 (0.0f);
			matrixRow[clampedForceIndex].m_force = clampedForceIndexValue;
			for (dgInt32 j = 0; j < count; j ++) {
				if (activeRow[j]) {
					dgJacobianMatrixElement* const row = &matrixRow[j];
					dgFloat32 val = lowBound[j] - row->m_force;
					if ((dgAbsf (val) < dgFloat32 (1.0e-5f)) && (row->m_accel < dgFloat32 (0.0f))) {
						row->m_force = lowBound[j];
						activeRow[j] = dgFloat32 (0.0f);
						deltaForce[j] = dgFloat32 (0.0f); 

					} else {
						val = highBound[j] - row->m_force;
						if ((dgAbsf (val) < dgFloat32 (1.0e-5f)) && (row->m_accel > dgFloat32 (0.0f))) {
							row->m_force = highBound[j];
							activeRow[j] = dgFloat32 (0.0f);
							deltaForce[j] = dgFloat32 (0.0f); 
						} else {
							dgAssert (activeRow[j] > dgFloat32 (0.0f));
							deltaForce[j] = row->m_accel * row->m_invDJMinvJt;
							akNum += row->m_accel * deltaForce[j];
							accNorm = dgMax (dgAbsf (row->m_accel), accNorm);
						}
					}
				}
			}

			dgAssert (activeRow[clampedForceIndex] == dgFloat32 (0.0f));
			i = -1;
			maxPasses = dgMax (maxPasses - 1, 1); 

		} else if (clampedForceIndex >= 0) {
			akNum = dgFloat32(0.0f);
			accNorm = dgFloat32(0.0f);
			activeRow[clampedForceIndex] = dgFloat32 (0.0f);
			for (dgInt32 j = 0; j < count; j ++) {
				dgJacobianMatrixElement* const row = &matrixRow[j];
				row->m_force += ak * deltaForce[j];
				row->m_accel -= ak * deltaAccel[j];
				accNorm = dgMax (dgAbsf (row->m_accel * activeRow[j]), accNorm);
				dgAssert (dgCheckFloat(row->m_force));
				dgAssert (dgCheckFloat(row->m_accel));

				deltaForce[j] = row->m_accel * row->m_invDJMinvJt * activeRow[j];
				akNum += deltaForce[j] * row->m_accel;
			}
			//force[clampedForceIndex + first] = clampedForceIndexValue;
			matrixRow[clampedForceIndex].m_force = clampedForceIndexValue;

			i = -1;
			maxPasses = dgMax (maxPasses - 1, 1); 

		} else {
			accNorm = dgFloat32(0.0f);
			for (dgInt32 j = 0; j < count; j ++) {
				dgJacobianMatrixElement* const row = &matrixRow[j];
				row->m_force += ak * deltaForce[j];
				row->m_accel -= ak * deltaAccel[j];
				accNorm = dgMax (dgAbsf (row->m_accel * activeRow[j]), accNorm);
				dgAssert (dgCheckFloat(row->m_force));
				dgAssert (dgCheckFloat(row->m_accel));
			}

			if (accNorm > maxAccNorm) {
				akDen = akNum;
				akNum = dgFloat32(0.0f);
				for (dgInt32 j = 0; j < count; j ++) {
					dgJacobianMatrixElement* const row = &matrixRow[j];
					deltaAccel[j] = row->m_accel * row->m_invDJMinvJt * activeRow[j];
					akNum += row->m_accel * deltaAccel[j];
				}

				dgAssert (akDen > dgFloat32(0.0f));
				akDen = dgMax (akDen, dgFloat32 (1.0e-17f));
				ak = dgFloat32 (akNum / akDen);
				for (dgInt32 j = 0; j < count; j ++) {
					deltaForce[j] = deltaAccel[j] + ak * deltaForce[j];
				}
			}
		}
	}

	for (dgInt32 j = 0; j < count; j ++) {
		dgJacobianMatrixElement* const row = &matrixRow[j];
		forceStep[j] = row->m_force - forceStep[j];
	}
	return retAccel;

}


void dgWorldDynamicUpdate::CalculateForcesSimulationModeSimd (const dgIsland* const island, dgInt32 rowStart, dgInt32 threadIndex, dgFloat32 timestep, dgFloat32 maxAccNorm) const
{
dgAssert (0);
/*
	dgFloat32 forceStep[DG_CONSTRAINT_MAX_ROWS];	
	dgWorld* const world = (dgWorld*) this;
	dgJacobian* const internalForces = &m_solverMemory.m_internalForces[island->m_bodyStart];

	const dgBodyInfo* const bodyArrayPtr = (dgBodyInfo*) &world->m_bodiesMemory[0]; 
	const dgBodyInfo* const bodyArray = &bodyArrayPtr[island->m_bodyStart];

	dgInt32 bodyCount = island->m_bodyCount;
	dgInt32 jointCount = island->m_jointCount;

	// initialize the intermediate force accumulation to zero 
	dgVector zero(dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f));
	for (dgInt32 i = 0; i < bodyCount; i ++) {
		internalForces[i].m_linear = zero;
		internalForces[i].m_angular = zero;
	}

	dgJacobianMatrixElement* const matrixRow = &m_solverMemory.m_memory[rowStart];
	dgJointInfo* const constraintArrayPtr = (dgJointInfo*) &world->m_jointsMemory[0];
	dgJointInfo* const constraintArray = &constraintArrayPtr[island->m_jointStart];
	for (dgInt32 i = 0; i < jointCount; i ++) {
		dgJacobian y0;
		dgJacobian y1;
		y0.m_linear = zero;
		y0.m_angular = zero;
		y1.m_linear = zero;
		y1.m_angular = zero;

		dgInt32 first = constraintArray[i].m_autoPairstart;
		dgInt32 count = constraintArray[i].m_autoPairActiveCount;
		for (dgInt32 j = 0; j < count; j ++) {
			dgJacobianMatrixElement* const row = &matrixRow[j + first];
			dgFloat32 val = row->m_force; 
			dgAssert (dgCheckFloat(val));
			y0.m_linear += row->m_Jt.m_jacobianM0.m_linear.Scale (val);
			y0.m_angular += row->m_Jt.m_jacobianM0.m_angular.Scale (val);
			y1.m_linear += row->m_Jt.m_jacobianM1.m_linear.Scale (val);
			y1.m_angular += row->m_Jt.m_jacobianM1.m_angular.Scale (val);
		}

		dgInt32 m0 = constraintArray[i].m_m0;
		dgInt32 m1 = constraintArray[i].m_m1;
		internalForces[m0].m_linear += y0.m_linear;
		internalForces[m0].m_angular += y0.m_angular;
		internalForces[m1].m_linear += y1.m_linear;
		internalForces[m1].m_angular += y1.m_angular;
	}

	for (dgInt32 i = 0; i < DG_CONSTRAINT_MAX_ROWS; i ++) {
		forceStep[i] = dgFloat32 (0.0f);
	}


	dgInt32 maxPasses = 4;
	dgInt32 prevJoint = 0;
	dgFloat32 accNorm = maxAccNorm * dgFloat32 (2.0f);
	for (dgInt32 passes = 0; (passes < maxPasses) && (accNorm > maxAccNorm); passes ++) {
		accNorm = dgFloat32 (0.0f);
		for (dgInt32 currJoint = 0; currJoint < jointCount; currJoint ++) {
			dgJacobian y0;
			dgJacobian y1;
			y0.m_linear = zero;
			y0.m_angular = zero;
			y1.m_linear = zero;
			y1.m_angular = zero;

			dgInt32 first = constraintArray[prevJoint].m_autoPairstart;
			dgInt32 rowsCount = constraintArray[prevJoint].m_autoPaircount;
			for (dgInt32 i = 0; i < rowsCount; i ++) {
				dgFloat32 deltaForce = forceStep[i]; 
				dgJacobianMatrixElement* const row = &matrixRow[i + first];

				y0.m_linear += row->m_Jt.m_jacobianM0.m_linear.Scale (deltaForce);
				y0.m_angular += row->m_Jt.m_jacobianM0.m_angular.Scale (deltaForce);
				y1.m_linear += row->m_Jt.m_jacobianM1.m_linear.Scale (deltaForce);
				y1.m_angular += row->m_Jt.m_jacobianM1.m_angular.Scale (deltaForce);
			}
			dgInt32 m0 = constraintArray[prevJoint].m_m0;
			dgInt32 m1 = constraintArray[prevJoint].m_m1;
			internalForces[m0].m_linear += y0.m_linear;
			internalForces[m0].m_angular += y0.m_angular;
			internalForces[m1].m_linear += y1.m_linear;
			internalForces[m1].m_angular += y1.m_angular;

			first = constraintArray[currJoint].m_autoPairstart;
			rowsCount = constraintArray[currJoint].m_autoPaircount;
			m0 = constraintArray[currJoint].m_m0;
			m1 = constraintArray[currJoint].m_m1;
			y0 = internalForces[m0];
			y1 = internalForces[m1];

			dgDynamicBody* const body0 = bodyArray[m0].m_body;
			dgDynamicBody* const body1 = bodyArray[m1].m_body;

			const dgFloat32 invMass0 = body0->m_invMass[3];
			const dgMatrix& invInertia0 = body0->m_invWorldInertiaMatrix;

			const dgFloat32 invMass1 = body1->m_invMass[3];
			const dgMatrix& invInertia1 = body1->m_invWorldInertiaMatrix;

			dgJacobianPair JMinv[DG_CONSTRAINT_MAX_ROWS];

			for (dgInt32 i = 0; i < rowsCount; i ++) {
				dgJacobianMatrixElement* const row = &matrixRow[i + first];

				JMinv[i].m_jacobianM0.m_linear = row->m_Jt.m_jacobianM0.m_linear.Scale (invMass0);
				JMinv[i].m_jacobianM0.m_angular = invInertia0.UnrotateVector (row->m_Jt.m_jacobianM0.m_angular);
				JMinv[i].m_jacobianM1.m_linear  = row->m_Jt.m_jacobianM1.m_linear.Scale (invMass1);
				JMinv[i].m_jacobianM1.m_angular = invInertia1.UnrotateVector (row->m_Jt.m_jacobianM1.m_angular);

				dgVector acc (JMinv[i].m_jacobianM0.m_linear.CompProduct(y0.m_linear) + 
							  JMinv[i].m_jacobianM0.m_angular.CompProduct(y0.m_angular) + 
							  JMinv[i].m_jacobianM1.m_linear.CompProduct(y1.m_linear) + 
							  JMinv[i].m_jacobianM1.m_angular.CompProduct(y1.m_angular));

				row->m_accel = row->m_coordenateAccel - acc.m_x - acc.m_y - acc.m_z - row->m_force * row->m_diagDamp;
			}

			dgFloat32 jointAccel = CalculateJointForcesSimd (island, rowStart, currJoint, forceStep, maxAccNorm, JMinv);
			accNorm = dgMax(accNorm, jointAccel);
			prevJoint = currJoint;
		}
	}

	for (dgInt32 i = 0; i < jointCount; i ++) {
		dgInt32 first = constraintArray[i].m_autoPairstart;
		dgInt32 count = constraintArray[i].m_autoPaircount;
		constraintArray[i].m_autoPaircount = count;
		dgJacobianMatrixElement* const rowBase = &matrixRow[first];
		for (dgInt32 k = 0; k < count; k ++) {
			dgJacobianMatrixElement* const row = &rowBase[k];

			dgInt32 frictionIndex = row->m_normalForceIndex;
			//dgAssert (((frictionIndex <0) && (matrixRow[frictionIndex].m_force == dgFloat32 (1.0f))) || ((frictionIndex >= 0) && (matrixRow[frictionIndex].m_force >= dgFloat32 (0.0f))));
			dgAssert ((frictionIndex < 0) || ((frictionIndex >= 0) && (rowBase[frictionIndex].m_force >= dgFloat32 (0.0f))));
			//dgFloat32 val = matrixRow[j].m_force;
			dgFloat32 val = (frictionIndex < 0) ? dgFloat32 (1.0f) : rowBase[frictionIndex].m_force;

			row->m_lowerBoundFrictionCoefficent *= val;
			row->m_upperBoundFrictionCoefficent *= val;
			row->m_force = dgClamp(row->m_force, row->m_lowerBoundFrictionCoefficent, row->m_upperBoundFrictionCoefficent);
		}
	}

	for (dgInt32 i = 0; i < bodyCount; i ++) {
		internalForces[i].m_linear[0] = dgFloat32(0.0f);
		internalForces[i].m_linear[1] = dgFloat32(0.0f);
		internalForces[i].m_linear[2] = dgFloat32(0.0f);
		internalForces[i].m_linear[3] = dgFloat32(0.0f);
		internalForces[i].m_angular[0] = dgFloat32(0.0f);
		internalForces[i].m_angular[1] = dgFloat32(0.0f);
		internalForces[i].m_angular[2] = dgFloat32(0.0f);
		internalForces[i].m_angular[3] = dgFloat32(0.0f);
	}

	for (dgInt32 i = 0; i < jointCount; i ++) {
		dgJacobian y0;
		dgJacobian y1;
		y0.m_linear = zero;
		y0.m_angular = zero;
		y1.m_linear = zero;
		y1.m_angular = zero;
		dgInt32 first = constraintArray[i].m_autoPairstart;
		dgInt32 count = constraintArray[i].m_autoPairActiveCount;
		for (dgInt32 j = 0; j < count; j ++) {
			dgJacobianMatrixElement* const row = &matrixRow[j + first];
			dgFloat32 val = row->m_force; 
			y0.m_linear += row->m_Jt.m_jacobianM0.m_linear.Scale (val);
			y0.m_angular += row->m_Jt.m_jacobianM0.m_angular.Scale (val);
			y1.m_linear += row->m_Jt.m_jacobianM1.m_linear.Scale (val);
			y1.m_angular += row->m_Jt.m_jacobianM1.m_angular.Scale (val);

		}
		dgInt32 m0 = constraintArray[i].m_m0;
		dgInt32 m1 = constraintArray[i].m_m1;
		internalForces[m0].m_linear += y0.m_linear;
		internalForces[m0].m_angular += y0.m_angular;
		internalForces[m1].m_linear += y1.m_linear;
		internalForces[m1].m_angular += y1.m_angular;
	}


	dgInt32 forceRows = 0;
	dgFloat32 akNum = dgFloat32 (0.0f);
	accNorm = dgFloat32(0.0f);
	for (dgInt32 i = 0; i < jointCount; i ++) {
		bool isClamped[DG_CONSTRAINT_MAX_ROWS];
		dgInt32 first = constraintArray[i].m_autoPairstart;
		dgInt32 count = constraintArray[i].m_autoPairActiveCount;
		dgInt32 m0 = constraintArray[i].m_m0;
		dgInt32 m1 = constraintArray[i].m_m1;
		const dgJacobian& y0 = internalForces[m0];
		const dgJacobian& y1 = internalForces[m1];

		dgDynamicBody* const body0 = bodyArray[m0].m_body;
		dgDynamicBody* const body1 = bodyArray[m1].m_body;

		const dgFloat32 invMass0 = body0->m_invMass[3];
		const dgMatrix& invInertia0 = body0->m_invWorldInertiaMatrix;

		const dgFloat32 invMass1 = body1->m_invMass[3];
		const dgMatrix& invInertia1 = body1->m_invWorldInertiaMatrix;

		for (dgInt32 j = 0; j < count; j ++) {
			dgJacobianMatrixElement* const row = &matrixRow[j + first];

			dgVector JMinvJacobianLinearM0 (row->m_Jt.m_jacobianM0.m_linear.Scale (invMass0));
			dgVector JMinvJacobianAngularM0 (invInertia0.UnrotateVector (row->m_Jt.m_jacobianM0.m_angular));
			dgVector JMinvJacobianLinearM1 (row->m_Jt.m_jacobianM1.m_linear.Scale (invMass1));
			dgVector JMinvJacobianAngularM1 (invInertia1.UnrotateVector (row->m_Jt.m_jacobianM1.m_angular));

			dgVector tmpAccel (JMinvJacobianLinearM0.CompProduct(y0.m_linear) +
							   JMinvJacobianAngularM0.CompProduct(y0.m_angular) +
				               JMinvJacobianLinearM1.CompProduct(y1.m_linear) +
				               JMinvJacobianAngularM1.CompProduct(y1.m_angular));
			row->m_accel = row->m_coordenateAccel - tmpAccel.m_x - tmpAccel.m_y - tmpAccel.m_z - row->m_force * row->m_diagDamp;
		}

		dgInt32 activeCount = 0;
		for (dgInt32 j = 0; j < count; j ++) {
			dgJacobianMatrixElement* const row = &matrixRow[j + first];
			dgFloat32 val = row->m_lowerBoundFrictionCoefficent - row->m_force;
			if ((dgAbsf (val) < dgFloat32 (1.0e-5f)) && (row->m_accel < dgFloat32 (0.0f))) {
				row->m_force = row->m_lowerBoundFrictionCoefficent;
				isClamped[j] = true;
			} else {
				val = row->m_upperBoundFrictionCoefficent - row->m_force;
				if ((dgAbsf (val) < dgFloat32 (1.0e-5f)) && (row->m_accel > dgFloat32 (0.0f))) {
					row->m_force = row->m_upperBoundFrictionCoefficent;
					isClamped[j] = true;
				} else {
					forceRows ++;
					activeCount ++;
					row->m_deltaForce = row->m_accel * row->m_invDJMinvJt;
					akNum += row->m_accel * row->m_deltaForce;
					accNorm = dgMax (dgAbsf (row->m_accel), accNorm);
					isClamped[j] = false;
				}
			}
		}

		if (activeCount < count) {
			dgInt32 i0 = 0;
			dgInt32 i1 = count - 1;
			constraintArray[i].m_autoPairActiveCount = activeCount;
			do { 
				while ((i0 <= i1) && !isClamped[i0]) i0 ++;
				while ((i0 <= i1) && isClamped[i1]) i1 --;
				if (i0 < i1) {
					//SwapRows (first + i0, first + i1);
					dgSwap (matrixRow[first + i0], matrixRow[first + i1]);
					i0 ++;
					i1 --;
				}
			} while (i0 < i1); 
		}
	}


	maxPasses = forceRows;
	dgInt32 totalPassesCount = 0;
	for (dgInt32 passes = 0; (passes < maxPasses) && (accNorm > maxAccNorm); passes ++) {
		for (dgInt32 i = 0; i < bodyCount; i ++) {
			internalForces[i].m_linear[0] = dgFloat32(0.0f);
			internalForces[i].m_linear[1] = dgFloat32(0.0f);
			internalForces[i].m_linear[2] = dgFloat32(0.0f);
			internalForces[i].m_linear[3] = dgFloat32(0.0f);
			internalForces[i].m_angular[0] = dgFloat32(0.0f);
			internalForces[i].m_angular[1] = dgFloat32(0.0f);
			internalForces[i].m_angular[2] = dgFloat32(0.0f);
			internalForces[i].m_angular[3] = dgFloat32(0.0f);
		}

		for (dgInt32 i = 0; i < jointCount; i ++) {
			dgJacobian y0;
			dgJacobian y1;

			y0.m_linear = zero;
			y0.m_angular = zero;
			y1.m_linear = zero;
			y1.m_angular = zero;
			dgInt32 first = constraintArray[i].m_autoPairstart;
			dgInt32 count = constraintArray[i].m_autoPairActiveCount;

			for (dgInt32 j = 0; j < count; j ++) {
				dgJacobianMatrixElement* const row = &matrixRow[j + first];
				dgFloat32 val = row->m_deltaForce;
				y0.m_linear += row->m_Jt.m_jacobianM0.m_linear.Scale (val);
				y0.m_angular += row->m_Jt.m_jacobianM0.m_angular.Scale (val);
				y1.m_linear += row->m_Jt.m_jacobianM1.m_linear.Scale (val);
				y1.m_angular += row->m_Jt.m_jacobianM1.m_angular.Scale (val);
			}
			dgInt32 m0 = constraintArray[i].m_m0;
			dgInt32 m1 = constraintArray[i].m_m1;
			internalForces[m0].m_linear += y0.m_linear;
			internalForces[m0].m_angular += y0.m_angular;
			internalForces[m1].m_linear += y1.m_linear;
			internalForces[m1].m_angular += y1.m_angular;
		}


		dgFloat32 akDen = dgFloat32 (0.0f);
		for (dgInt32 i = 0; i < jointCount; i ++) {
			dgInt32 first = constraintArray[i].m_autoPairstart;
			dgInt32 count = constraintArray[i].m_autoPairActiveCount;
			dgInt32 m0 = constraintArray[i].m_m0;
			dgInt32 m1 = constraintArray[i].m_m1;
			const dgJacobian& y0 = internalForces[m0];
			const dgJacobian& y1 = internalForces[m1];

			dgDynamicBody* const body0 = bodyArray[m0].m_body;
			dgDynamicBody* const body1 = bodyArray[m1].m_body;

			const dgFloat32 invMass0 = body0->m_invMass[3];
			const dgMatrix& invInertia0 = body0->m_invWorldInertiaMatrix;

			const dgFloat32 invMass1 = body1->m_invMass[3];
			const dgMatrix& invInertia1 = body1->m_invWorldInertiaMatrix;

			for (dgInt32 j = 0; j < count; j ++) {
				dgJacobianMatrixElement* const row = &matrixRow[j + first];

				dgVector JMinvJacobianLinearM0 (row->m_Jt.m_jacobianM0.m_linear.Scale (invMass0));
				dgVector JMinvJacobianAngularM0 (invInertia0.UnrotateVector (row->m_Jt.m_jacobianM0.m_angular));
				dgVector JMinvJacobianLinearM1 (row->m_Jt.m_jacobianM1.m_linear.Scale (invMass1));
				dgVector JMinvJacobianAngularM1 (invInertia1.UnrotateVector (row->m_Jt.m_jacobianM1.m_angular));

				dgVector tmpAccel (JMinvJacobianLinearM0.CompProduct(y0.m_linear) +
								   JMinvJacobianAngularM0.CompProduct(y0.m_angular) +
					               JMinvJacobianLinearM1.CompProduct(y1.m_linear) +
					               JMinvJacobianAngularM1.CompProduct(y1.m_angular));

				row->m_deltaAccel = tmpAccel.m_x + tmpAccel.m_y + tmpAccel.m_z + row->m_deltaForce * row->m_diagDamp;
				akDen += row->m_deltaAccel * row->m_deltaForce;
			}
		}

		dgAssert (akDen > dgFloat32 (0.0f));
		akDen = dgMax (akDen, dgFloat32(1.0e-16f));
		dgAssert (dgAbsf (akDen) >= dgFloat32(1.0e-16f));
		dgFloat32 ak = akNum / akDen;
		dgInt32 clampedForceIndex = -1;
		dgInt32 clampedForceJoint = -1;
		dgFloat32 clampedForceIndexValue = dgFloat32 (0.0f);

		for (dgInt32 i = 0; i < jointCount; i ++) {
			if (ak > dgFloat32 (1.0e-8f)) {
				dgInt32 first = constraintArray[i].m_autoPairstart;
				dgInt32 count = constraintArray[i].m_autoPairActiveCount;
				for (dgInt32 j = 0; j < count; j ++) {
					dgJacobianMatrixElement* const row = &matrixRow[j + first];
					dgFloat32 val = row->m_force + ak * row->m_deltaForce;
					if (row->m_deltaForce < dgFloat32 (-1.0e-16f)) {
						if (val < row->m_lowerBoundFrictionCoefficent) {
							ak = dgMax ((row->m_lowerBoundFrictionCoefficent - row->m_force) / row->m_deltaForce, dgFloat32 (0.0f));
							dgAssert (ak >= dgFloat32 (0.0f));
							clampedForceIndex = j;
							clampedForceJoint = i;
							clampedForceIndexValue = row->m_lowerBoundFrictionCoefficent;
						}
					} else if (row->m_deltaForce > dgFloat32 (1.0e-16f)) {
						if (val > row->m_upperBoundFrictionCoefficent) {
							ak = dgMax ((row->m_upperBoundFrictionCoefficent - row->m_force) / row->m_deltaForce, dgFloat32 (0.0f));
							dgAssert (ak >= dgFloat32 (0.0f));
							clampedForceIndex = j;
							clampedForceJoint = i;
							clampedForceIndexValue = row->m_upperBoundFrictionCoefficent;
						}
					}
				}
			}
		}

		if (clampedForceIndex >= 0) {
			bool isClamped[DG_CONSTRAINT_MAX_ROWS];
			for (dgInt32 i = 0; i < jointCount; i ++) {
				dgInt32 first = constraintArray[i].m_autoPairstart;
				dgInt32 count = constraintArray[i].m_autoPairActiveCount;
				for (dgInt32 j = 0; j < count; j ++) {
					dgJacobianMatrixElement* const row = &matrixRow[j + first];
					row->m_force += ak * row->m_deltaForce;
					row->m_accel -= ak * row->m_deltaAccel;
				}
			}

			dgInt32 first = constraintArray[clampedForceJoint].m_autoPairstart;
			dgInt32 count = constraintArray[clampedForceJoint].m_autoPairActiveCount;
			count --;
			//force[first + clampedForceIndex] = clampedForceIndexValue;
			matrixRow[first + clampedForceIndex].m_force = clampedForceIndexValue;
			if (clampedForceIndex != count) {
				//SwapRows (first + clampedForceIndex, first + count);
				dgSwap (matrixRow[first + clampedForceIndex], matrixRow[first + count]);
			}

			dgInt32 activeCount = count;
			for (dgInt32 i = 0; i < count; i ++) {
				//dgInt32 index;
				//dgFloat32 val;
				//index = first + i;
				dgJacobianMatrixElement* const row = &matrixRow[first + i];
				isClamped[i] = false;
				dgFloat32 val = row->m_lowerBoundFrictionCoefficent - row->m_force;
				if ((val > dgFloat32 (-1.0e-5f)) && (row->m_accel < dgFloat32 (0.0f))) {
					activeCount --;
					isClamped[i] = true;
				} else {
					val = row->m_upperBoundFrictionCoefficent - row->m_force;
					if ((val < dgFloat32 (1.0e-5f)) && (row->m_accel > dgFloat32 (0.0f))) {
						activeCount --;
						isClamped[i] = true;
					}
				}
			}

			if (activeCount < count) {
				dgInt32 i0 = 0;
				dgInt32 i1 = count - 1;
				do { 
					while ((i0 <= i1) && !isClamped[i0]) i0 ++;
					while ((i0 <= i1) && isClamped[i1]) i1 --;
					if (i0 < i1) {
						//SwapRows (first + i0, first + i1);
						dgSwap (matrixRow[first + i0], matrixRow[first + i1]);
						i0 ++;
						i1 --;
					}
				} while (i0 < i1); 
			}
			constraintArray[clampedForceJoint].m_autoPairActiveCount = activeCount;

			forceRows = 0;
			akNum = dgFloat32 (0.0f);
			accNorm = dgFloat32(0.0f);
			for (dgInt32 i = 0; i < jointCount; i ++) {
				dgInt32 first = constraintArray[i].m_autoPairstart;
				dgInt32 count = constraintArray[i].m_autoPairActiveCount;
				forceRows += count;

				for (dgInt32 j = 0; j < count; j ++) {
					//index = first + j;
					dgJacobianMatrixElement* const row = &matrixRow[first + j];
					dgAssert ((i != clampedForceJoint) || !((dgAbsf (row->m_lowerBoundFrictionCoefficent - row->m_force) < dgFloat32 (1.0e-5f)) && (row->m_accel < dgFloat32 (0.0f))));
					dgAssert ((i != clampedForceJoint) || !((dgAbsf (row->m_upperBoundFrictionCoefficent - row->m_force) < dgFloat32 (1.0e-5f)) && (row->m_accel > dgFloat32 (0.0f))));
					row->m_deltaForce = row->m_accel * row->m_invDJMinvJt;
					akNum += row->m_deltaForce * row->m_accel;
					accNorm = dgMax (dgAbsf (row->m_accel), accNorm);
					dgAssert (dgCheckFloat(row->m_deltaForce));
				}
			}

			dgAssert (akNum >= dgFloat32 (0.0f));
			passes = -1;
			maxPasses = forceRows;

		} else {

			accNorm = dgFloat32(0.0f);
			for (dgInt32 i = 0; i < jointCount; i ++) {
				dgInt32 first = constraintArray[i].m_autoPairstart;
				dgInt32 count = constraintArray[i].m_autoPairActiveCount;
				for (dgInt32 j = 0; j < count; j ++) {
					//index = j + first;
					dgJacobianMatrixElement* const row = &matrixRow[first + j];
					row->m_force += ak * row->m_deltaForce;
					row->m_accel -= ak * row->m_deltaAccel;
					accNorm = dgMax (dgAbsf (row->m_accel), accNorm);
				}
			}

			if (accNorm > maxAccNorm) {
				akDen = akNum;
				akNum = dgFloat32(0.0f);
				for (dgInt32 i = 0; i < jointCount; i ++) {
					dgInt32 first = constraintArray[i].m_autoPairstart;
					dgInt32 count = constraintArray[i].m_autoPairActiveCount;
					for (dgInt32 j = 0; j < count; j ++) {
						//index = j + first;
						dgJacobianMatrixElement* const row = &matrixRow[first + j];
						row->m_deltaAccel = row->m_accel * row->m_invDJMinvJt;
						akNum += row->m_accel * row->m_deltaAccel;
					}
				}

				dgAssert (akNum >= dgFloat32 (0.0f));
				dgAssert (akDen > dgFloat32(0.0f));
				akDen = dgMax (akDen, dgFloat32 (1.0e-17f));
				ak = dgFloat32 (akNum / akDen);
				for (dgInt32 i = 0; i < jointCount; i ++) {
					dgInt32 first = constraintArray[i].m_autoPairstart;
					dgInt32 count = constraintArray[i].m_autoPairActiveCount;
					for (dgInt32 j = 0; j < count; j ++) {
						//index = j + first;
						dgJacobianMatrixElement* const row = &matrixRow[first + j];
						row->m_deltaForce = row->m_deltaAccel + ak * row->m_deltaForce;
					}
				}
			}
		}

		totalPassesCount ++;
	}

	ApplyExternalForcesAndAccelerationSimd (island, rowStart, threadIndex, timestep, maxAccNorm);
*/
}


void dgWorldDynamicUpdate::CalculateReactionsForcesSimd (const dgIsland* const island, dgInt32 rowStart, dgInt32 threadIndex, dgFloat32 timestep, dgFloat32 maxAccNorm) const
{
	if (island->m_jointCount == 0) {
		ApplyExternalForcesAndAccelerationSimd (island, rowStart, threadIndex, timestep, 0.0f);
		return;
	}

	if (island->m_jointCount == 1) {
		CalculateSimpleBodyReactionsForcesSimd (island, rowStart, threadIndex, timestep, maxAccNorm);
		ApplyExternalForcesAndAccelerationSimd (island, rowStart, threadIndex, timestep, maxAccNorm * dgFloat32 (0.001f));
		return;
	}

	dgWorld* const world = (dgWorld*) this;
	if (world->m_solverMode && !island->m_hasExactSolverJoints) {
		CalculateForcesGameModeSimd (island, rowStart, threadIndex, timestep, maxAccNorm);
	} else {
		dgAssert (timestep > dgFloat32 (0.0f));
		// remember to make the change for the impulsive solver for CC
		CalculateForcesSimulationModeSimd (island, rowStart, threadIndex, timestep, maxAccNorm);
	}
}
