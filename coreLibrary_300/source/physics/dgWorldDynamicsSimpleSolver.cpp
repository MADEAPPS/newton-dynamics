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
#include "dgDynamicBody.h"
#include "dgCollisionInstance.h"
#include "dgWorldDynamicUpdate.h"


void dgWorldDynamicUpdate::CalculateIslandReactionForces (dgIsland* const island, dgFloat32 timestep, dgInt32 threadID) const
{
	if (!(island->m_isContinueCollision && island->m_jointCount)) {
		BuildJacobianMatrix (island, threadID, timestep);
		CalculateReactionsForces (island, threadID, timestep, DG_SOLVER_MAX_ERROR);
		IntegrateArray (island, DG_SOLVER_MAX_ERROR, timestep, threadID); 
	} else {
		// calculate reaction force sand new velocities
		BuildJacobianMatrix (island, threadID, timestep);
		CalculateReactionsForces (island, threadID, timestep, DG_SOLVER_MAX_ERROR);

		// see if the island goes to sleep
		bool isAutoSleep = true;
		bool stackSleeping = true;
		dgInt32 sleepCounter = 10000;

		dgWorld* const world = (dgWorld*) this;
		const dgInt32 bodyCount = island->m_bodyCount;
		dgBodyInfo* const bodyArrayPtr = (dgBodyInfo*) &world->m_bodiesMemory[0]; 
		dgBodyInfo* const bodyArray = &bodyArrayPtr[island->m_bodyStart];

		dgVector zero (dgFloat32 (0.0f));

		const dgFloat32 forceDamp = DG_FREEZZING_VELOCITY_DRAG;
		dgFloat32 maxAccel = dgFloat32 (0.0f);
		dgFloat32 maxAlpha = dgFloat32 (0.0f);
		dgFloat32 maxSpeed = dgFloat32 (0.0f);
		dgFloat32 maxOmega = dgFloat32 (0.0f);

		const dgFloat32 speedFreeze = world->m_freezeSpeed2;
		const dgFloat32 accelFreeze = world->m_freezeAccel2;
		const dgVector forceDampVect (forceDamp, forceDamp, forceDamp, dgFloat32 (0.0f));
		for (dgInt32 i = 1; i < bodyCount; i ++) {
			dgDynamicBody* const body = (dgDynamicBody*) bodyArray[i].m_body;
			if (body->IsRTTIType (dgBody::m_dynamicBodyRTTI)) {
				dgAssert (body->m_invMass.m_w);

				const dgFloat32 accel2 = body->m_accel % body->m_accel;
				const dgFloat32 alpha2 = body->m_alpha % body->m_alpha;
				const dgFloat32 speed2 = body->m_veloc % body->m_veloc;
				const dgFloat32 omega2 = body->m_omega % body->m_omega;

				maxAccel = dgMax (maxAccel, accel2);
				maxAlpha = dgMax (maxAlpha, alpha2);
				maxSpeed = dgMax (maxSpeed, speed2);
				maxOmega = dgMax (maxOmega, omega2);

				bool equilibrium = (accel2 < accelFreeze) && (alpha2 < accelFreeze) && (speed2 < speedFreeze) && (omega2 < speedFreeze);
				if (equilibrium) {
					dgVector veloc (body->m_veloc.CompProduct4(forceDampVect));
					dgVector omega = body->m_omega.CompProduct4 (forceDampVect);
					body->m_veloc = (dgVector (veloc.DotProduct4(veloc)) > m_velocTol) & veloc;
					body->m_omega = (dgVector (omega.DotProduct4(omega)) > m_velocTol) & omega;

				}
				body->m_equilibrium = dgUnsigned32 (equilibrium);
				stackSleeping &= equilibrium;
				isAutoSleep &= body->m_autoSleep;

				sleepCounter = dgMin (sleepCounter, body->m_sleepingCounter);

				// clear force and torque accumulators
				body->m_accel = zero;
				body->m_alpha = zero;
			}
		}

		if (isAutoSleep) {
			if (stackSleeping) {
				// the island went to sleep mode, 
				for (dgInt32 i = 1; i < bodyCount; i ++) {
					dgDynamicBody* const body = (dgDynamicBody*) bodyArray[i].m_body;
					if (body->IsRTTIType (dgBody::m_dynamicBodyRTTI)) {
						body->m_netForce = zero;
						body->m_netTorque = zero;
						body->m_veloc = zero;
						body->m_omega = zero;
					}
				}
			} else {
				// island is no sleeping but may be resting with small residual velocity for a long time
				// see if we can force to go to sleep
				if ((maxAccel > world->m_sleepTable[DG_SLEEP_ENTRIES - 1].m_maxAccel) ||
					(maxAlpha > world->m_sleepTable[DG_SLEEP_ENTRIES - 1].m_maxAlpha) ||
					(maxSpeed > world->m_sleepTable[DG_SLEEP_ENTRIES - 1].m_maxVeloc) ||
					(maxOmega > world->m_sleepTable[DG_SLEEP_ENTRIES - 1].m_maxOmega)) {
					for (dgInt32 i = 1; i < bodyCount; i ++) {
						dgDynamicBody* const body = (dgDynamicBody*) bodyArray[i].m_body;
						if (body->IsRTTIType (dgBody::m_dynamicBodyRTTI)) {
							body->m_sleepingCounter = 0;
						}
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
							dgDynamicBody* const body = (dgDynamicBody*) bodyArray[i].m_body;
							if (body->IsRTTIType (dgBody::m_dynamicBodyRTTI)) {
								body->m_netForce = zero;
								body->m_netTorque = zero;
								body->m_veloc = zero;
								body->m_omega = zero;
								body->m_equilibrium = true;
							}
						}
					} else {
						sleepCounter ++;
						for (dgInt32 i = 1; i < bodyCount; i ++) {
							dgDynamicBody* const body = (dgDynamicBody*) bodyArray[i].m_body;
							if (body->IsRTTIType (dgBody::m_dynamicBodyRTTI)) {
								body->m_sleepingCounter = sleepCounter;
							}
						}
					}
				}
			}
		} 


		if (!(isAutoSleep & stackSleeping)) {
			// island is not sleeping, need to integrate island velocity

			const dgUnsigned32 lru = world->GetBroadPhase()->m_lru;
			const dgInt32 jointCount = island->m_jointCount;
			dgJointInfo* const constraintArrayPtr = (dgJointInfo*) &world->m_jointsMemory[0];
			dgJointInfo* const constraintArray = &constraintArrayPtr[island->m_jointStart];

			dgFloat32 timeRemaining = timestep;
			const dgFloat32 timeTol = dgFloat32 (0.01f) * timestep;
			for (dgInt32 i = 0; (i < DG_MAX_CONTINUE_COLLISON_STEPS) && (timeRemaining > timeTol); i ++) {
//				dgAssert((i + 1) < DG_MAX_CONTINUE_COLLISON_STEPS);
				// calculate the closest time to impact 
				dgFloat32 timeToImpact = timeRemaining;
				for (dgInt32 j = 0; (j < jointCount) && (timeToImpact > timeTol); j ++) {
					dgContact* const contact = (dgContact*) constraintArray[j].m_joint;
					if (contact->GetId() == dgConstraint::m_contactConstraint) {
						dgDynamicBody* const body0 = (dgDynamicBody*)contact->m_body0;
						dgDynamicBody* const body1 = (dgDynamicBody*)contact->m_body1;
						if (body0->m_continueCollisionMode | body1->m_continueCollisionMode) {
							dgVector p;
							dgVector q;
							dgVector normal;
							timeToImpact = dgMin (timeToImpact, world->CalculateTimeToImpact (contact, timeToImpact, threadID, p, q, normal));
						}
					}
				}

				if (timeToImpact > timeTol) {
					timeRemaining -= timeToImpact;
					for (dgInt32 j = 1; j < bodyCount; j ++) {
						dgDynamicBody* const body = (dgDynamicBody*) bodyArray[j].m_body;
						if (body->IsRTTIType (dgBody::m_dynamicBodyRTTI)) {
							body->IntegrateVelocity(timeToImpact);
							body->UpdateWorlCollisionMatrix();
						}
					}
				} else {
					
					CalculateIslandContacts (island, timeRemaining, lru, threadID);
					BuildJacobianMatrix (island, threadID, 0.0f);
					CalculateReactionsForces (island, threadID, 0.0f, DG_SOLVER_MAX_ERROR);

					bool islandResinding = true;
					for (dgInt32 k = 0; (k < DG_MAX_CONTINUE_COLLISON_STEPS) && islandResinding; k ++) {
						dgFloat32 smallTimeStep = dgMin (timestep * dgFloat32 (1.0f / 8.0f), timeRemaining);
						timeRemaining -= smallTimeStep;
						for (dgInt32 j = 1; j < bodyCount; j ++) {
							dgDynamicBody* const body = (dgDynamicBody*) bodyArray[j].m_body;
							if (body->IsRTTIType (dgBody::m_dynamicBodyRTTI)) {
								body->IntegrateVelocity (smallTimeStep);
								body->UpdateWorlCollisionMatrix();
							}
						}

						islandResinding = false;
						if (timeRemaining > timeTol) {
							CalculateIslandContacts (island, timeRemaining, lru, threadID);

							bool isColliding = false;
							for (dgInt32 j = 0; (j < jointCount) && !isColliding; j ++) {
								dgContact* const contact = (dgContact*) constraintArray[j].m_joint;
								if (contact->GetId() == dgConstraint::m_contactConstraint) {

									const dgBody* const body0 = contact->m_body0;
									const dgBody* const body1 = contact->m_body1;

									const dgVector& veloc0 = body0->m_veloc;
									const dgVector& veloc1 = body1->m_veloc;

									const dgVector& omega0 = body0->m_omega;
									const dgVector& omega1 = body1->m_omega;

									const dgVector& com0 = body0->m_globalCentreOfMass;
									const dgVector& com1 = body1->m_globalCentreOfMass;
									
									for (dgList<dgContactMaterial>::dgListNode* node = contact->GetFirst(); node; node = node->GetNext()) {
										const dgContactMaterial* const contactMaterial = &node->GetInfo();
										dgVector vel0 (veloc0 + omega0 * (contactMaterial->m_point - com0));
										dgVector vel1 (veloc1 + omega1 * (contactMaterial->m_point - com1));
										dgVector vRel (vel0 - vel1);
										dgAssert (contactMaterial->m_normal.m_w == dgFloat32 (0.0f));
										dgFloat32 speed = vRel.DotProduct4(contactMaterial->m_normal).m_w;
										isColliding |= (speed < dgFloat32 (0.0f));
									}
								}
							}
							islandResinding = !isColliding;
						}
					}
				}
			}

			if (timeRemaining > dgFloat32 (0.0)) {
				for (dgInt32 j = 1; j < bodyCount; j ++) {
					dgDynamicBody* const body = (dgDynamicBody*) bodyArray[j].m_body;
					if (body->IsRTTIType (dgBody::m_dynamicBodyRTTI)) {
						body->IntegrateVelocity(timeRemaining);
						body->UpdateMatrix (timeRemaining, threadID);
					}
				}
			} else {
				for (dgInt32 j = 1; j < bodyCount; j ++) {
					dgDynamicBody* const body = (dgDynamicBody*) bodyArray[j].m_body;
					if (body->IsRTTIType (dgBody::m_dynamicBodyRTTI)) {
						body->UpdateMatrix (timestep, threadID);
					}
				}
			}
		}
	}
}


void dgWorldDynamicUpdate::CalculateIslandContacts (dgIsland* const island, dgFloat32 timestep, dgInt32 currLru, dgInt32 threadID) const
{
	dgWorld* const world = (dgWorld*) this;
	dgInt32 jointCount = island->m_jointCount;
	dgJointInfo* const constraintArrayPtr = (dgJointInfo*) &world->m_jointsMemory[0];
	dgJointInfo* const constraintArray = &constraintArrayPtr[island->m_jointStart];

	for (dgInt32 j = 0; (j < jointCount); j ++) {
		dgContact* const contact = (dgContact*) constraintArray[j].m_joint;
		if (contact->GetId() == dgConstraint::m_contactConstraint) {
			dgContactPoint contactArray[DG_MAX_CONTATCS];
			dgCollidingPairCollector::dgPair pair;

			contact->m_maxDOF = 0;
			contact->m_broadphaseLru = currLru;
			pair.m_contact = contact;
			pair.m_cacheIsValid = false;
			pair.m_contactBuffer = contactArray;
			world->CalculateContacts (&pair, timestep, threadID, false, false);
			if (pair.m_contactCount) {
				dgAssert (pair.m_contactCount <= (DG_CONSTRAINT_MAX_ROWS / 3));
				world->ProcessContacts (&pair, timestep, threadID);
			}
		}
	}
}


void dgWorldDynamicUpdate::BuildJacobianMatrix (dgIsland* const island, dgInt32 threadIndex, dgFloat32 timestep) const 
{
	dgAssert (island->m_bodyCount >= 2);

	dgWorld* const world = (dgWorld*) this;

	dgInt32 bodyCount = island->m_bodyCount;
	dgBodyInfo* const bodyArrayPtr = (dgBodyInfo*) &world->m_bodiesMemory[0]; 
	dgBodyInfo* const bodyArray = &bodyArrayPtr[island->m_bodyStart];

	dgAssert (((dgDynamicBody*) bodyArray[0].m_body)->IsRTTIType (dgBody::m_dynamicBodyRTTI));
	dgAssert ((((dgDynamicBody*)bodyArray[0].m_body)->m_accel % ((dgDynamicBody*)bodyArray[0].m_body)->m_accel) == dgFloat32 (0.0f));
	dgAssert ((((dgDynamicBody*)bodyArray[0].m_body)->m_alpha % ((dgDynamicBody*)bodyArray[0].m_body)->m_alpha) == dgFloat32 (0.0f));

	if (timestep != dgFloat32 (0.0f)) {
		for (dgInt32 i = 1; i < bodyCount; i ++) {
			dgBody* const body = bodyArray[i].m_body;
			//if (body->m_active) {
			if (!body->m_equilibrium) {
				dgAssert (body->m_invMass.m_w > dgFloat32 (0.0f));
				body->AddDampingAcceleration();
				body->CalcInvInertiaMatrix ();
			}
		}

	} else {
		for (dgInt32 i = 1; i < bodyCount; i ++) {
			dgBody* const body = bodyArray[i].m_body;
	//		if (body->m_active) {
			if (!body->m_equilibrium) {
				dgAssert (body->m_invMass.m_w > dgFloat32 (0.0f));
				body->CalcInvInertiaMatrix ();
			}
		}
	}
	dgInt32 jointCount = island->m_jointCount;
	if (jointCount) {
		dgInt32 rowCount = 0;
		dgJointInfo* const constraintArrayPtr = (dgJointInfo*) &world->m_jointsMemory[0];
		dgJointInfo* const constraintArray = &constraintArrayPtr[island->m_jointStart];

		GetJacobianDerivatives (island, threadIndex, rowCount, timestep);

		dgVector zero (dgFloat32 (0.0f));
		dgFloat32 forceOrImpulseScale = (timestep > dgFloat32 (0.0f)) ? dgFloat32 (1.0f) : dgFloat32 (0.0f);

		dgJacobianMatrixElement* const matrixRow = &m_solverMemory.m_memory[island->m_rowsStart];
		for (dgInt32 k = 0; k < jointCount; k ++) {
			const dgJointInfo* const jointInfo = &constraintArray[k];
			dgConstraint* const constraint = jointInfo->m_joint;
			if (constraint->m_solverActive) {
				dgInt32 index = jointInfo->m_autoPairstart;
				dgInt32 count = jointInfo->m_autoPaircount;
				dgInt32 m0 = jointInfo->m_m0;
				dgInt32 m1 = jointInfo->m_m1;

				dgAssert (m0 >= 0);
				dgAssert (m0 < bodyCount);
				dgAssert (m1 >= 0);
				dgAssert (m1 < bodyCount);

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

				for (dgInt32 i = 0; i < count; i ++) {
					dgJacobianMatrixElement* const row = &matrixRow[index];
					dgAssert (row->m_Jt.m_jacobianM0.m_linear.m_w == dgFloat32 (0.0f));
					dgAssert (row->m_Jt.m_jacobianM0.m_angular.m_w == dgFloat32 (0.0f));
					dgAssert (row->m_Jt.m_jacobianM1.m_linear.m_w == dgFloat32 (0.0f));
					dgAssert (row->m_Jt.m_jacobianM1.m_angular.m_w == dgFloat32 (0.0f));

					dgVector JMinvJacobianLinearM0 (row->m_Jt.m_jacobianM0.m_linear.CompProduct4 (invMass0));
					dgVector JMinvJacobianAngularM0 (invInertia0.RotateVector (row->m_Jt.m_jacobianM0.m_angular));
					dgVector JMinvJacobianLinearM1 (row->m_Jt.m_jacobianM1.m_linear.CompProduct4 (invMass1));
					dgVector JMinvJacobianAngularM1 (invInertia1.RotateVector (row->m_Jt.m_jacobianM1.m_angular));

					dgVector tmpDiag (JMinvJacobianLinearM0.CompProduct4(row->m_Jt.m_jacobianM0.m_linear) + JMinvJacobianAngularM0.CompProduct4(row->m_Jt.m_jacobianM0.m_angular) +
									  JMinvJacobianLinearM1.CompProduct4(row->m_Jt.m_jacobianM1.m_linear) + JMinvJacobianAngularM1.CompProduct4(row->m_Jt.m_jacobianM1.m_angular));

					dgVector tmpAccel (JMinvJacobianLinearM0.CompProduct4(accel0) + JMinvJacobianAngularM0.CompProduct4(alpha0) + JMinvJacobianLinearM1.CompProduct4(accel1) + JMinvJacobianAngularM1.CompProduct4(alpha1));

					dgFloat32 extenalAcceleration = -(tmpAccel.m_x + tmpAccel.m_y + tmpAccel.m_z);
					row->m_deltaAccel = extenalAcceleration * forceOrImpulseScale;
					row->m_coordenateAccel += extenalAcceleration * forceOrImpulseScale;
					dgAssert (row->m_jointFeebackForce);
					row->m_force = row->m_jointFeebackForce[0].m_force * forceOrImpulseScale;

					row->m_maxImpact = dgFloat32 (0.0f);

					//force[index] = 0.0f;
					dgAssert (row->m_diagDamp >= dgFloat32(0.1f));
					dgAssert (row->m_diagDamp <= dgFloat32(100.0f));
					dgFloat32 stiffness = DG_PSD_DAMP_TOL * row->m_diagDamp;

					dgFloat32 diag = (tmpDiag.m_x + tmpDiag.m_y + tmpDiag.m_z);
					dgAssert (diag > dgFloat32 (0.0f));
					row->m_diagDamp = diag * stiffness;

					diag *= (dgFloat32(1.0f) + stiffness);
					row->m_invDJMinvJt = dgFloat32(1.0f) / diag;
					index ++;
				}
			}
		}
	}
}


void dgWorldDynamicUpdate::ApplyExternalForcesAndAcceleration(const dgIsland* const island, dgInt32 threadIndex, dgFloat32 timestep, dgFloat32 maxAccNorm) const
{
	dgJacobian* const internalForces = &m_solverMemory.m_internalForces[island->m_bodyStart];

	dgVector zero (dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f));
	dgInt32 bodyCount = island->m_bodyCount;
	for (dgInt32 i = 0; i < bodyCount; i ++) {
		internalForces[i].m_linear = zero;
		internalForces[i].m_angular = zero;
	}

	dgInt32 hasJointFeeback = 0;
	dgInt32 jointCount = island->m_jointCount;
	dgWorld* const world = (dgWorld*) this;
	dgJointInfo* const constraintArrayPtr = (dgJointInfo*) &world->m_jointsMemory[0];
	dgJointInfo* const constraintArray = &constraintArrayPtr[island->m_jointStart];

	dgJacobianMatrixElement* const matrixRow = &m_solverMemory.m_memory[island->m_rowsStart];
	for (dgInt32 i = 0; i < jointCount; i ++) {
		dgInt32 first = constraintArray[i].m_autoPairstart;
		dgInt32 count = constraintArray[i].m_autoPaircount;

		dgInt32 m0 = constraintArray[i].m_m0;
		dgInt32 m1 = constraintArray[i].m_m1;

		dgJacobian y0;
		dgJacobian y1;
		y0.m_linear = zero;
		y0.m_angular = zero;
		y1.m_linear = zero;
		y1.m_angular = zero;

		for (dgInt32 j = 0; j < count; j ++) { 
			dgJacobianMatrixElement* const row = &matrixRow[j + first];
			dgFloat32 val = row->m_force; 

			dgAssert (dgCheckFloat(val));
			row->m_jointFeebackForce[0].m_force = val;

			dgVector force (val);
			y0.m_linear += row->m_Jt.m_jacobianM0.m_linear.CompProduct4 (force);
			y0.m_angular += row->m_Jt.m_jacobianM0.m_angular.CompProduct4 (force);
			y1.m_linear += row->m_Jt.m_jacobianM1.m_linear.CompProduct4 (force);
			y1.m_angular += row->m_Jt.m_jacobianM1.m_angular.CompProduct4 (force);
		}

		hasJointFeeback |= (constraintArray[i].m_joint->m_updaFeedbackCallback ? 1 : 0);

		internalForces[m0].m_linear += y0.m_linear;
		internalForces[m0].m_angular += y0.m_angular;
		internalForces[m1].m_linear += y1.m_linear;
		internalForces[m1].m_angular += y1.m_angular;
	}


	dgBodyInfo* const bodyArrayPtr = (dgBodyInfo*) &world->m_bodiesMemory[0]; 
	dgBodyInfo* const bodyArray = &bodyArrayPtr[island->m_bodyStart];

	dgVector timeStepVect (timestep, timestep, timestep, dgFloat32 (0.0));
	if (timestep > dgFloat32 (0.0f)) {
		// apply force
		dgFloat32 accelTol2 = maxAccNorm * maxAccNorm;
		for (dgInt32 i = 1; i < bodyCount; i ++) {
			//dgDynamicBody* const body = bodyArray[i].m_body;
			dgDynamicBody* const body = (dgDynamicBody*) bodyArray[i].m_body;
			if (body->IsRTTIType (dgBody::m_dynamicBodyRTTI)) {
				body->m_accel += internalForces[i].m_linear;
				body->m_alpha += internalForces[i].m_angular;
			}

			dgVector accel (body->m_accel.Scale3 (body->m_invMass.m_w));
			dgFloat32 error = accel % accel;
			if (error < accelTol2) {
				accel = zero;
				body->m_accel = zero;
			}

			dgVector alpha (body->m_invWorldInertiaMatrix.RotateVector (body->m_alpha));
			error = alpha % alpha;
			if (error < accelTol2) {
				alpha = zero;
				body->m_alpha = zero;
			}

			body->m_netForce = body->m_accel;
			body->m_netTorque = body->m_alpha;

			body->m_veloc += accel.CompProduct4(timeStepVect);
			body->m_omega += alpha.CompProduct4(timeStepVect);
		}

		if (hasJointFeeback) {
			for (dgInt32 i = 0; i < jointCount; i ++) {
				if (constraintArray[i].m_joint->m_updaFeedbackCallback) {
					constraintArray[i].m_joint->m_updaFeedbackCallback (*constraintArray[i].m_joint, timestep, threadIndex);
				}
			}
		}
	} else {
		// apply impulse
		for (dgInt32 i = 1; i < bodyCount; i ++) {
			dgBody* const body = bodyArray[i].m_body;

			const dgVector& linearMomentum = internalForces[i].m_linear;
			const dgVector& angularMomentum = internalForces[i].m_angular;

			body->m_netForce = zero;
			body->m_netTorque = zero;
			body->m_veloc += linearMomentum.Scale3(body->m_invMass.m_w);
			body->m_omega += body->m_invWorldInertiaMatrix.RotateVector (angularMomentum);
		}
	}
}


void dgWorldDynamicUpdate::CalculateSimpleBodyReactionsForces (const dgIsland* const island, dgInt32 rowStart, dgInt32 threadIndex, dgFloat32 timestep, dgFloat32 maxAccNorm) const
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

	const dgBodyInfo* const bodyArrayPtr = (dgBodyInfo*) &world->m_bodiesMemory[0]; 
	const dgBodyInfo* const bodyArray = &bodyArrayPtr[island->m_bodyStart];

	dgJointInfo* const constraintArrayPtr = (dgJointInfo*) &world->m_jointsMemory[0];
	dgJointInfo* const constraintArray = &constraintArrayPtr[island->m_jointStart];
	dgInt32 count = constraintArray[0].m_autoPaircount;
	dgAssert (constraintArray[0].m_autoPairstart == 0);

	dgInt32 m0 = constraintArray[0].m_m0;
	dgInt32 m1 = constraintArray[0].m_m1;
	dgDynamicBody* const body0 = bodyArray[m0].m_body;
	dgDynamicBody* const body1 = bodyArray[m1].m_body;

	const dgFloat32 invMass0 = body0->m_invMass[3];
	const dgMatrix& invInertia0 = body0->m_invWorldInertiaMatrix;
	
	const dgFloat32 invMass1 = body1->m_invMass[3];
	const dgMatrix& invInertia1 = body1->m_invWorldInertiaMatrix;

	dgInt32 maxPasses = count;
	dgJacobianMatrixElement* const matrixRow = &m_solverMemory.m_memory[rowStart];
	for (dgInt32 i = 0; i < count; i ++) {
		dgJacobianMatrixElement* const row = &matrixRow[i];

		dgInt32 frictionIndex = row->m_normalForceIndex;
		//dgAssert (((k <0) && (matrixRow[k].m_force == dgFloat32 (1.0f))) || ((k >= 0) && (matrixRow[k].m_force >= dgFloat32 (0.0f))));
		dgAssert ((frictionIndex < 0) || ((frictionIndex >= 0) && (matrixRow[frictionIndex].m_force >= dgFloat32 (0.0f))));
		//dgFloat32 val = matrixRow[k].m_force;
		dgFloat32 val = (frictionIndex < 0) ? 1.0f : matrixRow[frictionIndex].m_force;
		lowBound[i] = val * row->m_lowerBoundFrictionCoefficent;
		highBound[i] = val * row->m_upperBoundFrictionCoefficent;

		activeRow[i] = dgFloat32 (1.0f);
		if (row->m_force < lowBound[i]) {
			maxPasses --;
			row->m_force = lowBound[i];
			activeRow[i] = dgFloat32 (0.0f);
		} else if (row->m_force > highBound[i]) {
			maxPasses --;
			row->m_force = highBound[i];
			activeRow[i] = dgFloat32 (0.0f);
		}
	}

	dgJacobian y0;
	dgJacobian y1;
	dgVector zero (dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f));
	y0.m_linear = zero;
	y0.m_angular = zero;
	y1.m_linear = zero;
	y1.m_angular = zero;
	for (dgInt32 i = 0; i < count; i ++) {
		dgJacobianMatrixElement* const row = &matrixRow[i];
		dgFloat32 val = row->m_force; 
		y0.m_linear += row->m_Jt.m_jacobianM0.m_linear.Scale3 (val);
		y0.m_angular += row->m_Jt.m_jacobianM0.m_angular.Scale3 (val);
		y1.m_linear += row->m_Jt.m_jacobianM1.m_linear.Scale3 (val);
		y1.m_angular += row->m_Jt.m_jacobianM1.m_angular.Scale3 (val);
	}



	dgFloat32 akNum = dgFloat32 (0.0f);
	dgFloat32 accNorm = dgFloat32(0.0f);
	for (dgInt32 i = 0; i < count; i ++) {
		dgJacobianMatrixElement* const row = &matrixRow[i];

		JMinv[i].m_jacobianM0.m_linear = row->m_Jt.m_jacobianM0.m_linear.Scale3 (invMass0);
		JMinv[i].m_jacobianM0.m_angular = invInertia0.UnrotateVector (row->m_Jt.m_jacobianM0.m_angular);
		JMinv[i].m_jacobianM1.m_linear  = row->m_Jt.m_jacobianM1.m_linear.Scale3 (invMass1);
		JMinv[i].m_jacobianM1.m_angular = invInertia1.UnrotateVector (row->m_Jt.m_jacobianM1.m_angular);

		dgVector acc (JMinv[i].m_jacobianM0.m_linear.CompProduct3(y0.m_linear) + 
					  JMinv[i].m_jacobianM0.m_angular.CompProduct3(y0.m_angular) + 
					  JMinv[i].m_jacobianM1.m_linear.CompProduct3(y1.m_linear) + 
					  JMinv[i].m_jacobianM1.m_angular.CompProduct3(y1.m_angular));

		accel[i] = row->m_coordenateAccel - acc.m_x - acc.m_y - acc.m_z - row->m_force * row->m_diagDamp;

		deltaForce[i] = accel[i] * row->m_invDJMinvJt * activeRow[i];
		akNum += accel[i] * deltaForce[i];
		accNorm = dgMax (dgAbsf (accel[i] * activeRow[i]), accNorm);
	}

	
	for (dgInt32 i = 0; (i < maxPasses) && (accNorm > maxAccNorm); i ++) {
		y0.m_linear = zero;
		y0.m_angular = zero;
		y1.m_linear = zero;
		y1.m_angular = zero;
		for (dgInt32 k = 0; k < count; k ++) {
			dgJacobianMatrixElement* const row = &matrixRow[k];
			dgFloat32 val = deltaForce[k]; 
			y0.m_linear += row->m_Jt.m_jacobianM0.m_linear.Scale3 (val);
			y0.m_angular += row->m_Jt.m_jacobianM0.m_angular.Scale3 (val);
			y1.m_linear += row->m_Jt.m_jacobianM1.m_linear.Scale3 (val);
			y1.m_angular += row->m_Jt.m_jacobianM1.m_angular.Scale3 (val);
		}

		dgFloat32 akDen = dgFloat32 (0.0f);
		for (dgInt32 k = 0; k < count; k ++) {
			dgJacobianMatrixElement* const row = &matrixRow[k];
			dgVector acc (JMinv[k].m_jacobianM0.m_linear.CompProduct3(y0.m_linear) +
						  JMinv[k].m_jacobianM0.m_angular.CompProduct3(y0.m_angular) +
			              JMinv[k].m_jacobianM1.m_linear.CompProduct3(y1.m_linear) +
			              JMinv[k].m_jacobianM1.m_angular.CompProduct3(y1.m_angular));
			deltaAccel[k] = acc.m_x + acc.m_y + acc.m_z + deltaForce[k] * row->m_diagDamp;
			akDen += deltaAccel[k] * deltaForce[k];
		}

		dgAssert (akDen > dgFloat32 (0.0f));
		akDen = dgMax (akDen, dgFloat32(1.0e-16f));
		dgAssert (dgAbsf (akDen) >= dgFloat32(1.0e-16f));
		dgFloat32 ak = akNum / akDen;

		dgInt32 clampedForceIndex = -1;
		dgFloat32 clampedForceIndexValue = dgFloat32(0.0f);
		for (dgInt32 k = 0; k < count; k ++) {
			if (activeRow[k]) {
				dgJacobianMatrixElement* const row = &matrixRow[k];
				dgFloat32 val = row->m_force + ak * deltaForce[k];
				if (deltaForce[k] < dgFloat32 (-1.0e-16f)) {
					if (val < lowBound[k]) {
						ak = dgMax ((lowBound[k] - row->m_force) / deltaForce[k], dgFloat32 (0.0f));
						clampedForceIndex = k;
						clampedForceIndexValue = lowBound[k];
						if (ak < dgFloat32 (1.0e-8f)) {
							ak = dgFloat32 (0.0f);
							break;
						}
					}
				} else if (deltaForce[k] > dgFloat32 (1.0e-16f)) {
					if (val >= highBound[k]) {
						ak = dgMax ((highBound[k] - row->m_force) / deltaForce[k], dgFloat32 (0.0f));;
						clampedForceIndex = k;
						clampedForceIndexValue = highBound[k];
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
							accNorm = dgMax (dgAbsf (accel[k]), accNorm);
						}
					}
				}
			}


			i = -1;
			maxPasses = dgMax (maxPasses - 1, 1); 

		} else if (clampedForceIndex >= 0) {
			akNum = dgFloat32(0.0f);
			accNorm = dgFloat32(0.0f);
			activeRow[clampedForceIndex] = dgFloat32 (0.0f);
			for (dgInt32 k = 0; k < count; k ++) {
				dgJacobianMatrixElement* const row = &matrixRow[k];
				row->m_force += ak * deltaForce[k];
				accel[k] -= ak * deltaAccel[k];
				accNorm = dgMax (dgAbsf (accel[k] * activeRow[k]), accNorm);
				dgAssert (dgCheckFloat(row->m_force));
				dgAssert (dgCheckFloat(accel[k]));

				deltaForce[k] = accel[k] * row->m_invDJMinvJt * activeRow[k];
				akNum += deltaForce[k] * accel[k];
			}
			matrixRow[clampedForceIndex].m_force = clampedForceIndexValue;

			i = -1;
			maxPasses = dgMax (maxPasses - 1, 1); 

		} else {
			accNorm = dgFloat32(0.0f);
			for (dgInt32 k = 0; k < count; k ++) {
				dgJacobianMatrixElement* const row = &matrixRow[k];
				row->m_force += ak * deltaForce[k];
				accel[k] -= ak * deltaAccel[k];
				accNorm = dgMax (dgAbsf (accel[k] * activeRow[k]), accNorm);
				dgAssert (dgCheckFloat(row->m_force));
				dgAssert (dgCheckFloat(accel[k]));
			}

			if (accNorm > maxAccNorm) {

				akDen = akNum;
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
	}
*/
}


void dgWorldDynamicUpdate::CalculateForcesGameMode (const dgIsland* const island, dgInt32 threadIndex, dgFloat32 timestep, dgFloat32 maxAccNorm) const
{
	dgJacobian* const internalForces = &m_solverMemory.m_internalForces[island->m_bodyStart];
	dgVector zero(dgFloat32 (0.0f));

	dgWorld* const world = (dgWorld*) this;
	dgInt32 bodyCount = island->m_bodyCount;
	dgBodyInfo* const bodyArrayPtr = (dgBodyInfo*) &world->m_bodiesMemory[0]; 
	dgBodyInfo* const bodyArray = &bodyArrayPtr[island->m_bodyStart];
	for (dgInt32 i = 1; i < bodyCount; i ++) {
		dgBody* const body = bodyArray[i].m_body;
		if (body->m_active) {
			// re use these variables for temp storage 
			body->m_netForce = body->m_veloc;
			body->m_netTorque = body->m_omega;

			internalForces[i].m_linear = zero;
			internalForces[i].m_angular = zero;
		}
	}

	dgAssert (bodyArray[0].m_body->m_resting);
	internalForces[0].m_linear = zero;
	internalForces[0].m_angular = zero;

	dgJacobianMatrixElement* const matrixRow = &m_solverMemory.m_memory[island->m_rowsStart];

	dgInt32 jointCount = island->m_jointCount;
	dgJointInfo* const constraintArrayPtr = (dgJointInfo*) &world->m_jointsMemory[0];
	dgJointInfo* const constraintArray = &constraintArrayPtr[island->m_jointStart];
	for (dgInt32 i = 0; i < jointCount; i ++) {
		dgJointInfo* const jointInfo = &constraintArray[i];
		dgConstraint* const constraint = jointInfo->m_joint;
		if (constraint->m_solverActive) {
			dgJacobian y0;
			dgJacobian y1;
			y0.m_linear = zero;
			y0.m_angular = zero;
			y1.m_linear = zero;
			y1.m_angular = zero;
			dgInt32 first = constraintArray[i].m_autoPairstart;
			dgInt32 count = constraintArray[i].m_autoPaircount;
			for (dgInt32 j = 0; j < count; j ++) { 
				dgJacobianMatrixElement* const row = &matrixRow[j + first];
				dgAssert (dgCheckFloat(row->m_force));
				dgVector val (row->m_force); 
				y0.m_linear += row->m_Jt.m_jacobianM0.m_linear.CompProduct4 (val);
				y0.m_angular += row->m_Jt.m_jacobianM0.m_angular.CompProduct4 (val);
				y1.m_linear += row->m_Jt.m_jacobianM1.m_linear.CompProduct4 (val);
				y1.m_angular += row->m_Jt.m_jacobianM1.m_angular.CompProduct4 (val);
			}
			dgInt32 m0 = constraintArray[i].m_m0;
			dgInt32 m1 = constraintArray[i].m_m1;
			internalForces[m0].m_linear += y0.m_linear;
			internalForces[m0].m_angular += y0.m_angular;
			internalForces[m1].m_linear += y1.m_linear;
			internalForces[m1].m_angular += y1.m_angular;
		}
	}

	dgInt32 maxPasses = dgInt32 (world->m_solverMode + LINEAR_SOLVER_SUB_STEPS);

	dgFloat32 invTimestep = (timestep > dgFloat32 (0.0f)) ? dgFloat32 (1.0f) / timestep : dgFloat32 (0.0f);
	dgFloat32 invStepRK = (dgFloat32 (1.0f) / dgFloat32 (maxPasses));
	dgFloat32 timestepRK =  timestep * invStepRK;
	dgFloat32 invTimestepRK = invTimestep * dgFloat32 (maxPasses);
	dgAssert (bodyArray[0].m_body == world->m_sentinelBody);

	dgFloat32 cacheForce[DG_CONSTRAINT_MAX_ROWS + 4];
	cacheForce[0] = dgFloat32 (1.0f);
	cacheForce[1] = dgFloat32 (1.0f);
	cacheForce[2] = dgFloat32 (1.0f);
	cacheForce[3] = dgFloat32 (1.0f);
	dgFloat32* const normalForce = &cacheForce[4];

	dgVector speedFreeze2 (world->m_freezeSpeed2 * dgFloat32 (0.1f));
	dgVector freezeOmega2 (world->m_freezeOmega2 * dgFloat32 (0.1f));
	dgVector forceActiveMask ((jointCount <= DG_SMALL_ISLAND_COUNT) ?  dgVector (-1, -1, -1, -1): dgFloat32 (0.0f));

	dgFloat32 firstPassCoef = dgFloat32 (0.0f);
	for (dgInt32 step = 0; step < maxPasses; step ++) {
		dgJointAccelerationDecriptor joindDesc;
		joindDesc.m_timeStep = timestepRK;
		joindDesc.m_invTimeStep = invTimestepRK;
		joindDesc.m_firstPassCoefFlag = firstPassCoef;
		if (firstPassCoef == dgFloat32 (0.0f)) {
			for (dgInt32 curJoint = 0; curJoint < jointCount; curJoint ++) {
				dgJointInfo* const jointInfo = &constraintArray[curJoint];
				dgConstraint* const constraint = jointInfo->m_joint;
				if (constraint->m_solverActive) {
					joindDesc.m_rowsCount = constraintArray[curJoint].m_autoPaircount;
					joindDesc.m_rowMatrix = &matrixRow[constraintArray[curJoint].m_autoPairstart];
					constraint->JointAccelerations (&joindDesc);
				}
			}
			firstPassCoef = dgFloat32 (1.0f);
		} else {
			for (dgInt32 curJoint = 0; curJoint < jointCount; curJoint ++) {
				dgJointInfo* const jointInfo = &constraintArray[curJoint];
				dgConstraint* const constraint = jointInfo->m_joint;
				if (constraint->m_solverActive) {
					dgInt32 m0 = constraintArray[curJoint].m_m0;
					dgInt32 m1 = constraintArray[curJoint].m_m1;
					const dgBody* const body0 = bodyArray[m0].m_body;
					const dgBody* const body1 = bodyArray[m1].m_body;
					if (!(body0->m_resting & body1->m_resting)) {
						joindDesc.m_rowsCount = constraintArray[curJoint].m_autoPaircount;
						joindDesc.m_rowMatrix = &matrixRow[constraintArray[curJoint].m_autoPairstart];
						constraint->JointAccelerations (&joindDesc);
					}
				}
			}
		}

		dgVector accNorm (maxAccNorm * dgFloat32 (2.0f));
		for (dgInt32 passes = 0; (passes < DG_BASE_ITERATION_COUNT) && (accNorm.m_x > maxAccNorm); passes ++) {

			accNorm = dgVector (dgFloat32 (0.0f));
			for (dgInt32 curJoint = 0; curJoint < jointCount; curJoint ++) {
				dgJointInfo* const jointInfo = &constraintArray[curJoint];
				dgConstraint* const constraint = jointInfo->m_joint;
				if(constraint->m_solverActive) {
					dgInt32 m0 = constraintArray[curJoint].m_m0;
					dgInt32 m1 = constraintArray[curJoint].m_m1;
					const dgBody* const body0 = bodyArray[m0].m_body;
					const dgBody* const body1 = bodyArray[m1].m_body;

					if (!(body0->m_resting & body1->m_resting)) {

						dgInt32 index = constraintArray[curJoint].m_autoPairstart;
						dgInt32 rowsCount = constraintArray[curJoint].m_autoPaircount;

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
							dgVector lowerFrictionForce (frictionNormal * row->m_lowerBoundFrictionCoefficent);
							dgVector upperFrictionForce (frictionNormal * row->m_upperBoundFrictionCoefficent);

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

							dgVector prevValue (f - dgVector (row->m_force));

							//row->m_force = f.m_x;
							//normalForce[k] = f.m_x;
							//f.StoreScalar(&row->m_force);
							//f.StoreScalar(&normalForce[k]);
							row->m_force = f.GetScalar();
							normalForce[k] = f.GetScalar();

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
				}
			}
		}

		if (timestepRK != dgFloat32 (0.0f)) {
			dgVector timestep4 (timestepRK);
			for (dgInt32 i = 1; i < bodyCount; i ++) {
				dgDynamicBody* const body = (dgDynamicBody*) bodyArray[i].m_body;
				if (body->m_active) {
					dgVector force (internalForces[i].m_linear);
					dgVector torque (internalForces[i].m_angular);
					if (body->IsRTTIType (dgBody::m_dynamicBodyRTTI)) {
						force += body->m_accel;
						torque += body->m_alpha;
					}

					dgVector velocStep ((force.Scale4 (body->m_invMass.m_w)).CompProduct4(timestep4));
					dgVector omegaStep ((body->m_invWorldInertiaMatrix.RotateVector (torque)).CompProduct4(timestep4));
					if (!body->m_resting) {
						body->m_veloc += velocStep;
						body->m_omega += omegaStep;
					} else {
						dgVector velocStep2 (velocStep.DotProduct4(velocStep));
						dgVector omegaStep2 (omegaStep.DotProduct4(omegaStep));
						dgVector test ((velocStep2 > speedFreeze2) | (omegaStep2 > speedFreeze2) | forceActiveMask);
						if (test.GetSignMask()) {
							body->m_resting = false;
						}
					}
				}
			}
		} else {
			for (dgInt32 i = 1; i < bodyCount; i ++) {
				dgBody* const body = bodyArray[i].m_body;
				if (body->m_active) {
					const dgVector& linearMomentum = internalForces[i].m_linear;
					const dgVector& angularMomentum = internalForces[i].m_angular;

					body->m_veloc += linearMomentum.Scale4(body->m_invMass.m_w);
					body->m_omega += body->m_invWorldInertiaMatrix.RotateVector (angularMomentum);
				}
			}
		}
	}

	dgInt32 hasJointFeeback = 0;
	if (timestepRK != dgFloat32 (0.0f)) {
		for (dgInt32 i = 0; i < jointCount; i ++) {
			dgJointInfo* const jointInfo = &constraintArray[i];
			dgConstraint* const constraint = jointInfo->m_joint;
			if (constraint->m_solverActive) {
				dgInt32 first = constraintArray[i].m_autoPairstart;
				dgInt32 count = constraintArray[i].m_autoPaircount;

				for (dgInt32 j = 0; j < count; j ++) { 
					dgJacobianMatrixElement* const row = &matrixRow[j + first];
					dgFloat32 val = row->m_force; 
					dgAssert (dgCheckFloat(val));
					row->m_jointFeebackForce[0].m_force = val;
					row->m_jointFeebackForce[0].m_impact = row->m_maxImpact * timestepRK;
				}
				hasJointFeeback |= (constraint->m_updaFeedbackCallback ? 1 : 0);
			}
		}


		dgVector invTime (invTimestep);
		//dgFloat32 maxAccNorm2 = maxAccNorm * maxAccNorm;
		dgVector maxAccNorm2 (maxAccNorm * maxAccNorm);
		for (dgInt32 i = 1; i < bodyCount; i ++) {
			dgDynamicBody* const body = (dgDynamicBody*) bodyArray[i].m_body;
			if (body->m_active) {
				// the initial velocity and angular velocity were stored in net force and net torque, for memory saving
				dgVector accel = (body->m_veloc - body->m_netForce).CompProduct4 (invTime);
				dgVector alpha = (body->m_omega - body->m_netTorque).CompProduct4 (invTime);
				dgVector accelTest ((accel.DotProduct4(accel) > maxAccNorm2) | (alpha.DotProduct4(alpha) > maxAccNorm2) | forceActiveMask);
//				if ((accel % accel) < maxAccNorm2) {
//					accel = zero;
//				}
//				if ((alpha % alpha) < maxAccNorm2) {
//					alpha = zero;
//				}
				accel = accel & accelTest;
				alpha = alpha & accelTest;

				if (body->IsRTTIType (dgBody::m_dynamicBodyRTTI)) {
					body->m_accel = accel;
					body->m_alpha = alpha;
				}
				body->m_netForce = accel.Scale4 (body->m_mass[3]);

				alpha = body->m_matrix.UnrotateVector(alpha);
				body->m_netTorque = body->m_matrix.RotateVector (alpha.CompProduct4(body->m_mass));
			}
		}
		if (hasJointFeeback) {
			for (dgInt32 i = 0; i < jointCount; i ++) {
				if (constraintArray[i].m_joint->m_updaFeedbackCallback) {
					constraintArray[i].m_joint->m_updaFeedbackCallback (*constraintArray[i].m_joint, timestep, threadIndex);
				}
			}
		}
	} else {
		for (dgInt32 i = 1; i < bodyCount; i ++) {
			dgBody* const body = bodyArray[i].m_body;
			if (body->m_active) {
				body->m_netForce = zero;
				body->m_netTorque = zero;
			}
		}
	}
}


dgFloat32 dgWorldDynamicUpdate::CalculateJointForces (const dgIsland* const island, dgInt32 rowStart, dgInt32 joint, dgFloat32* const forceStep, dgFloat32 maxAccNorm, const dgJacobianPair* const JMinv) const
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
	dgInt32 first = jointInfo->m_autoPairstart;
	dgInt32 count = jointInfo->m_autoPaircount;

	dgInt32 maxPasses = count;
	dgFloat32 akNum = dgFloat32 (0.0f);
	dgFloat32 accNorm = dgFloat32(0.0f);

	dgJacobianMatrixElement* const matrixRow = &m_solverMemory.m_memory[rowStart + first];
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
			dgVector val (deltaForce[j]); 

			y0.m_linear += row->m_Jt.m_jacobianM0.m_linear.CompProduct4(val);
			y0.m_angular += row->m_Jt.m_jacobianM0.m_angular.CompProduct4 (val);
			y1.m_linear += row->m_Jt.m_jacobianM1.m_linear.CompProduct4 (val);
			y1.m_angular += row->m_Jt.m_jacobianM1.m_angular.CompProduct4 (val);
		}

		dgFloat32 akDen = dgFloat32 (0.0f);
		for (dgInt32 j = 0; j < count; j ++) {
			dgJacobianMatrixElement* const row = &matrixRow[j];
			dgVector acc (JMinv[j].m_jacobianM0.m_linear.CompProduct4(y0.m_linear) + JMinv[j].m_jacobianM0.m_angular.CompProduct4(y0.m_angular) + JMinv[j].m_jacobianM1.m_linear.CompProduct4(y1.m_linear) + JMinv[j].m_jacobianM1.m_angular.CompProduct4(y1.m_angular));

			//deltaAccel[j] = acc.m_x + acc.m_y + acc.m_z + deltaForce[j] * row->m_diagDamp;
			acc = dgVector (deltaForce[j] * row->m_diagDamp) + acc.AddHorizontal();
			//acc.StoreScalar(&deltaAccel[j]);
			deltaAccel[j] = acc.GetScalar();

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


void dgWorldDynamicUpdate::CalculateForcesSimulationMode (const dgIsland* const island, dgInt32 threadIndex, dgFloat32 timestep, dgFloat32 maxAccNorm) const
{
	dgVector forceStepBuffer[DG_CONSTRAINT_MAX_ROWS / 4];	
	dgFloat32* const forceStep = &forceStepBuffer[0].m_x;

	dgWorld* const world = (dgWorld*) this;
	dgJacobian* const internalForces = &m_solverMemory.m_internalForces[island->m_bodyStart];
	dgInt32 bodyCount = island->m_bodyCount;
	dgInt32 jointCount = island->m_jointCount;

	// initialize the intermediate force accumulation to zero 
	dgVector zero(dgFloat32 (0.0f));
	for (dgInt32 i = 0; i < bodyCount; i ++) {
		internalForces[i].m_linear = zero;
		internalForces[i].m_angular = zero;
	}

	const dgBodyInfo* const bodyArrayPtr = (dgBodyInfo*) &world->m_bodiesMemory[0]; 
	const dgBodyInfo* const bodyArray = &bodyArrayPtr[island->m_bodyStart];

	dgJacobianMatrixElement* const matrixRow = &m_solverMemory.m_memory[island->m_rowsStart];
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
			dgVector val (row->m_force); 
			dgAssert (dgCheckFloat(row->m_force));
			y0.m_linear += row->m_Jt.m_jacobianM0.m_linear.CompProduct4 (val);
			y0.m_angular += row->m_Jt.m_jacobianM0.m_angular.CompProduct4 (val);
			y1.m_linear += row->m_Jt.m_jacobianM1.m_linear.CompProduct4 (val);
			y1.m_angular += row->m_Jt.m_jacobianM1.m_angular.CompProduct4 (val);
		}

		dgInt32 m0 = constraintArray[i].m_m0;
		dgInt32 m1 = constraintArray[i].m_m1;
		internalForces[m0].m_linear += y0.m_linear;
		internalForces[m0].m_angular += y0.m_angular;
		internalForces[m1].m_linear += y1.m_linear;
		internalForces[m1].m_angular += y1.m_angular;
	}

	for (dgInt32 i = 0; i < dgInt32 (sizeof (forceStepBuffer) / sizeof (forceStepBuffer[0])); i ++) {
		forceStepBuffer[i] = zero;
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
				//dgFloat32 deltaForce = forceStep[i]; 
				dgVector deltaForce (forceStep[i]); 
				dgJacobianMatrixElement* const row = &matrixRow[i + first];

				y0.m_linear += row->m_Jt.m_jacobianM0.m_linear.CompProduct4 (deltaForce);
				y0.m_angular += row->m_Jt.m_jacobianM0.m_angular.CompProduct4 (deltaForce);
				y1.m_linear += row->m_Jt.m_jacobianM1.m_linear.CompProduct4 (deltaForce);
				y1.m_angular += row->m_Jt.m_jacobianM1.m_angular.CompProduct4 (deltaForce);
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

			const dgBody* const body0 = bodyArray[m0].m_body;
			const dgBody* const body1 = bodyArray[m1].m_body;

			const dgVector invMass0 (body0->m_invMass[3]);
			const dgMatrix& invInertia0 = body0->m_invWorldInertiaMatrix;
			const dgVector invMass1 (body1->m_invMass[3]);
			const dgMatrix& invInertia1 = body1->m_invWorldInertiaMatrix;

			dgJacobianPair JMinv[DG_CONSTRAINT_MAX_ROWS];
			for (dgInt32 i = 0; i < rowsCount; i ++) {
				dgJacobianMatrixElement* const row = &matrixRow[i + first];

				dgAssert (row->m_Jt.m_jacobianM0.m_linear.m_w == dgFloat32 (0.0f));
				dgAssert (row->m_Jt.m_jacobianM0.m_angular.m_w == dgFloat32 (0.0f));
				dgAssert (row->m_Jt.m_jacobianM1.m_linear.m_w == dgFloat32 (0.0f));
				dgAssert (row->m_Jt.m_jacobianM1.m_angular.m_w == dgFloat32 (0.0f));


				JMinv[i].m_jacobianM0.m_linear = row->m_Jt.m_jacobianM0.m_linear.CompProduct4 (invMass0);
				JMinv[i].m_jacobianM0.m_angular = invInertia0.UnrotateVector (row->m_Jt.m_jacobianM0.m_angular);
				JMinv[i].m_jacobianM1.m_linear  = row->m_Jt.m_jacobianM1.m_linear.CompProduct4 (invMass1);
				JMinv[i].m_jacobianM1.m_angular = invInertia1.UnrotateVector (row->m_Jt.m_jacobianM1.m_angular);

				dgVector acc (JMinv[i].m_jacobianM0.m_linear.CompProduct4(y0.m_linear) + 
							  JMinv[i].m_jacobianM0.m_angular.CompProduct4(y0.m_angular) + 
							  JMinv[i].m_jacobianM1.m_linear.CompProduct4(y1.m_linear) + 
							  JMinv[i].m_jacobianM1.m_angular.CompProduct4(y1.m_angular));


				//row->m_accel = row->m_coordenateAccel - acc.m_x - acc.m_y - acc.m_z - row->m_force * row->m_diagDamp;
				acc = dgVector (row->m_coordenateAccel - row->m_force * row->m_diagDamp) - acc.AddHorizontal();
				//acc.StoreScalar(&row->m_accel);
				row->m_accel = acc.GetScalar();
			}

			dgFloat32 jointAccel = CalculateJointForces (island, island->m_rowsStart, currJoint, forceStep, maxAccNorm, JMinv);
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
			dgAssert ((frictionIndex < 0) || ((frictionIndex >= 0) && (rowBase[frictionIndex].m_force >= dgFloat32 (0.0f))));
			dgFloat32 val = (frictionIndex < 0) ? dgFloat32 (1.0f) : rowBase[frictionIndex].m_force;
			row->m_lowerBoundFrictionCoefficent *= val;
			row->m_upperBoundFrictionCoefficent *= val;
			row->m_force = dgClamp(row->m_force, row->m_lowerBoundFrictionCoefficent, row->m_upperBoundFrictionCoefficent);
		}
	}

	for (dgInt32 i = 0; i < bodyCount; i ++) {
		internalForces[i].m_linear = zero;
		internalForces[i].m_angular = zero;
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
			dgVector val (row->m_force); 
			y0.m_linear += row->m_Jt.m_jacobianM0.m_linear.CompProduct4 (val);
			y0.m_angular += row->m_Jt.m_jacobianM0.m_angular.CompProduct4 (val);
			y1.m_linear += row->m_Jt.m_jacobianM1.m_linear.CompProduct4 (val);
			y1.m_angular += row->m_Jt.m_jacobianM1.m_angular.CompProduct4 (val);
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

		const dgBody* const body0 = bodyArray[m0].m_body;
		const dgBody* const body1 = bodyArray[m1].m_body;
		const dgVector invMass0 (body0->m_invMass[3]);
		const dgMatrix& invInertia0 = body0->m_invWorldInertiaMatrix;
		const dgVector invMass1 (body1->m_invMass[3]);
		const dgMatrix& invInertia1 = body1->m_invWorldInertiaMatrix;

		for (dgInt32 j = 0; j < count; j ++) {
			dgJacobianMatrixElement* const row = &matrixRow[j + first];

			dgVector JMinvJacobianLinearM0 (row->m_Jt.m_jacobianM0.m_linear.CompProduct4 (invMass0));
			dgVector JMinvJacobianAngularM0 (invInertia0.UnrotateVector (row->m_Jt.m_jacobianM0.m_angular));
			dgVector JMinvJacobianLinearM1 (row->m_Jt.m_jacobianM1.m_linear.CompProduct4(invMass1));
			dgVector JMinvJacobianAngularM1 (invInertia1.UnrotateVector (row->m_Jt.m_jacobianM1.m_angular));

			dgVector acc (JMinvJacobianLinearM0.CompProduct4(y0.m_linear) + JMinvJacobianAngularM0.CompProduct4(y0.m_angular) + JMinvJacobianLinearM1.CompProduct4(y1.m_linear) + JMinvJacobianAngularM1.CompProduct4(y1.m_angular));
			//row->m_accel = row->m_coordenateAccel - acc.m_x - acc.m_y - acc.m_z - row->m_force * row->m_diagDamp;
			acc = dgVector (row->m_coordenateAccel - row->m_force * row->m_diagDamp) - acc.AddHorizontal();
			//acc.StoreScalar(&row->m_accel);
			row->m_accel = acc.GetScalar();
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
					dgSwap (matrixRow[first + i0], matrixRow[first + i1]);
					i0 ++;
					i1 --;
				}
			} while (i0 < i1); 
		}
	}


	maxPasses = forceRows;
	dgInt32 totalPassesCount = 0;
	dgVector zeroDivide(dgFloat32 (1.0e-16f));
	for (dgInt32 passes = 0; (passes < maxPasses) && (accNorm > maxAccNorm); passes ++) {
		for (dgInt32 i = 0; i < bodyCount; i ++) {
			internalForces[i].m_linear = zero;
			internalForces[i].m_angular = zero;
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
				dgVector val (row->m_deltaForce);
				y0.m_linear += row->m_Jt.m_jacobianM0.m_linear.CompProduct4(val);
				y0.m_angular += row->m_Jt.m_jacobianM0.m_angular.CompProduct4 (val);
				y1.m_linear += row->m_Jt.m_jacobianM1.m_linear.CompProduct4 (val);
				y1.m_angular += row->m_Jt.m_jacobianM1.m_angular.CompProduct4 (val);
			}
			dgInt32 m0 = constraintArray[i].m_m0;
			dgInt32 m1 = constraintArray[i].m_m1;
			internalForces[m0].m_linear += y0.m_linear;
			internalForces[m0].m_angular += y0.m_angular;
			internalForces[m1].m_linear += y1.m_linear;
			internalForces[m1].m_angular += y1.m_angular;
		}
		dgVector tmpDen(zero);
		for (dgInt32 i = 0; i < jointCount; i ++) {
			dgInt32 first = constraintArray[i].m_autoPairstart;
			dgInt32 count = constraintArray[i].m_autoPairActiveCount;
			dgInt32 m0 = constraintArray[i].m_m0;
			dgInt32 m1 = constraintArray[i].m_m1;
			const dgJacobian& y0 = internalForces[m0];
			const dgJacobian& y1 = internalForces[m1];

			const dgBody* const body0 = bodyArray[m0].m_body;
			const dgBody* const body1 = bodyArray[m1].m_body;
			const dgVector invMass0 (body0->m_invMass[3]);
			const dgMatrix& invInertia0 = body0->m_invWorldInertiaMatrix;
			const dgVector invMass1 (body1->m_invMass[3]);
			const dgMatrix& invInertia1 = body1->m_invWorldInertiaMatrix;

			
			for (dgInt32 j = 0; j < count; j ++) {
				dgJacobianMatrixElement* const row = &matrixRow[j + first];

				dgVector JMinvJacobianLinearM0 (row->m_Jt.m_jacobianM0.m_linear.CompProduct4(invMass0));
				dgVector JMinvJacobianAngularM0 (invInertia0.UnrotateVector (row->m_Jt.m_jacobianM0.m_angular));
				dgVector JMinvJacobianLinearM1 (row->m_Jt.m_jacobianM1.m_linear.CompProduct4 (invMass1));
				dgVector JMinvJacobianAngularM1 (invInertia1.UnrotateVector (row->m_Jt.m_jacobianM1.m_angular));

				dgVector tmpAccel (JMinvJacobianLinearM0.CompProduct4(y0.m_linear) + JMinvJacobianAngularM0.CompProduct4(y0.m_angular) + JMinvJacobianLinearM1.CompProduct4(y1.m_linear) + JMinvJacobianAngularM1.CompProduct4(y1.m_angular));

				//row->m_deltaAccel = tmpAccel.m_x + tmpAccel.m_y + tmpAccel.m_z + row->m_deltaForce * row->m_diagDamp;
				tmpAccel = tmpAccel.AddHorizontal() + dgVector (row->m_deltaForce * row->m_diagDamp);
				//tmpAccel.StoreScalar(&row->m_deltaAccel);
				row->m_deltaAccel = tmpAccel.GetScalar();

				//akDen += row->m_deltaAccel * row->m_deltaForce;
				tmpDen = tmpDen + dgVector (row->m_deltaAccel * row->m_deltaForce);
			}
		}
		tmpDen = tmpDen.GetMax(zeroDivide);
		
		//dgFloat32 akDen;
		//tmpDen.StoreScalar(&akDen);
		dgFloat32 akDen = tmpDen.GetScalar();
		dgAssert (akDen > dgFloat32 (0.0f));

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
			matrixRow[first + clampedForceIndex].m_force = clampedForceIndexValue;
			if (clampedForceIndex != count) {
				dgSwap (matrixRow[first + clampedForceIndex], matrixRow[first + count]);
			}

			dgInt32 activeCount = count;
			for (dgInt32 i = 0; i < count; i ++) {
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
						dgJacobianMatrixElement* const row = &matrixRow[first + j];
						row->m_deltaForce = row->m_deltaAccel + ak * row->m_deltaForce;
					}
				}
			}
		}
		totalPassesCount ++;
	}
	
//	ApplyExternalForcesAndAcceleration (island, island->m_rowsStart, threadIndex, timestep, maxAccNorm);
	ApplyExternalForcesAndAcceleration (island, threadIndex, timestep, maxAccNorm);
}


void dgWorldDynamicUpdate::CalculateReactionsForces(const dgIsland* const island, dgInt32 threadIndex, dgFloat32 timestep, dgFloat32 maxAccNorm) const
{
	if (island->m_jointCount == 0) {
		ApplyExternalForcesAndAcceleration (island, threadIndex, timestep, 0.0f);

//	} else if (island->m_jointCount == 1) {
//		CalculateSimpleBodyReactionsForces (island, rowStart, threadIndex, timestep, maxAccNorm);
//		ApplyExternalForcesAndAcceleration (island, rowStart, threadIndex, timestep, maxAccNorm * dgFloat32 (0.001f));
	} else {
		dgWorld* const world = (dgWorld*) this;
		if (world->m_solverMode && !island->m_hasExactSolverJoints) {
			CalculateForcesGameMode (island, threadIndex, timestep, maxAccNorm);
		} else {
			dgAssert (timestep > dgFloat32 (0.0f));
			// remember to make the change for the impulsive solver for CC
			CalculateForcesSimulationMode (island, threadIndex, timestep, maxAccNorm);
		}
	}
}
