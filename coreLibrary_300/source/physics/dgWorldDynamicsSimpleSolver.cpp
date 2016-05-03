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
#include "dgSkeletonContainer.h"
#include "dgCollisionInstance.h"
#include "dgWorldDynamicUpdate.h"
#include "dgBilateralConstraint.h"


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
				body->m_accel = dgVector::m_zero;
				body->m_alpha = dgVector::m_zero;
			}
		}

		if (isAutoSleep) {
			if (stackSleeping) {
				// the island went to sleep mode, 
				for (dgInt32 i = 1; i < bodyCount; i ++) {
					dgDynamicBody* const body = (dgDynamicBody*) bodyArray[i].m_body;
					if (body->IsRTTIType (dgBody::m_dynamicBodyRTTI)) {
						body->m_netForce = dgVector::m_zero;
						body->m_netTorque = dgVector::m_zero;
						body->m_veloc = dgVector::m_zero;
						body->m_omega = dgVector::m_zero;
					}
				}
			} else {
				// island is not sleeping but may be resting with small residual velocity for a long time
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
								body->m_netForce = dgVector::m_zero;
								body->m_netTorque = dgVector::m_zero;
								body->m_veloc = dgVector::m_zero;
								body->m_omega = dgVector::m_zero;
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
			const dgContactMaterial* const material = contact->m_material;
			if (material->m_flags & dgContactMaterial::m_collisionEnable) {
				dgInt32 processContacts = 1;
				if (material->m_aabbOverlap) {
					processContacts = material->m_aabbOverlap (*material, *contact->GetBody0(), *contact->GetBody1(), threadID);
				}

				if (processContacts) {
					dgContactPoint contactArray[DG_MAX_CONTATCS];
					dgBroadPhase::dgPair pair;

					contact->m_maxDOF = 0;
					contact->m_broadphaseLru = currLru;
					pair.m_contact = contact;
					pair.m_cacheIsValid = false;
					pair.m_timestep = timestep;
					pair.m_contactBuffer = contactArray;
					world->CalculateContacts (&pair, threadID, false, false);
					if (pair.m_contactCount) {
						dgAssert (pair.m_contactCount <= (DG_CONSTRAINT_MAX_ROWS / 3));
						world->ProcessContacts (&pair, threadID);
					}
				}
			}
		}
	}
}


void dgWorldDynamicUpdate::BuildJacobianMatrix (const dgBodyInfo* const bodyInfoArray, const dgJointInfo* const jointInfo, dgJacobianMatrixElement* const matrixRow, dgFloat32 forceImpulseScale) const 
{
	if (jointInfo->m_joint->m_solverActive) {
		dgInt32 index = jointInfo->m_pairStart;
		dgInt32 count = jointInfo->m_pairCount;
		dgInt32 m0 = jointInfo->m_m0;
		dgInt32 m1 = jointInfo->m_m1;

	//	dgAssert(m0 >= 0);
	//	dgAssert(m0 < bodyCount);
	//	dgAssert(m1 >= 0);
	//	dgAssert(m1 < bodyCount);

		const dgBody* const body0 = bodyInfoArray[m0].m_body;
		const dgBody* const body1 = bodyInfoArray[m1].m_body;

		const dgVector invMass0(body0->m_invMass[3]);
		const dgMatrix& invInertia0 = body0->m_invWorldInertiaMatrix;
		const dgVector invMass1(body1->m_invMass[3]);
		const dgMatrix& invInertia1 = body1->m_invWorldInertiaMatrix;

		dgVector accel0(dgVector::m_zero);
		dgVector alpha0(dgVector::m_zero);
		if (body0->IsRTTIType(dgBody::m_dynamicBodyRTTI)) {
			accel0 = ((dgDynamicBody*)body0)->m_accel;
			alpha0 = ((dgDynamicBody*)body0)->m_alpha;
		}

		dgVector accel1(dgVector::m_zero);
		dgVector alpha1(dgVector::m_zero);
		if (body1->IsRTTIType(dgBody::m_dynamicBodyRTTI)) {
			accel1 = ((dgDynamicBody*)body1)->m_accel;
			alpha1 = ((dgDynamicBody*)body1)->m_alpha;
		}

		for (dgInt32 i = 0; i < count; i++) {
			dgJacobianMatrixElement* const row = &matrixRow[index];
			dgAssert(row->m_Jt.m_jacobianM0.m_linear.m_w == dgFloat32(0.0f));
			dgAssert(row->m_Jt.m_jacobianM0.m_angular.m_w == dgFloat32(0.0f));
			dgAssert(row->m_Jt.m_jacobianM1.m_linear.m_w == dgFloat32(0.0f));
			dgAssert(row->m_Jt.m_jacobianM1.m_angular.m_w == dgFloat32(0.0f));

			//dgVector JMinvJacobianLinearM0(row->m_Jt.m_jacobianM0.m_linear.CompProduct4(invMass0));
			//dgVector JMinvJacobianAngularM0(invInertia0.RotateVector(row->m_Jt.m_jacobianM0.m_angular));
			//dgVector JMinvJacobianLinearM1(row->m_Jt.m_jacobianM1.m_linear.CompProduct4(invMass1));
			//dgVector JMinvJacobianAngularM1(invInertia1.RotateVector(row->m_Jt.m_jacobianM1.m_angular));

			row->m_JMinv.m_jacobianM0.m_linear = row->m_Jt.m_jacobianM0.m_linear.CompProduct4(invMass0);
			row->m_JMinv.m_jacobianM0.m_angular = invInertia0.RotateVector(row->m_Jt.m_jacobianM0.m_angular);
			row->m_JMinv.m_jacobianM1.m_linear = row->m_Jt.m_jacobianM1.m_linear.CompProduct4(invMass1);
			row->m_JMinv.m_jacobianM1.m_angular = invInertia1.RotateVector(row->m_Jt.m_jacobianM1.m_angular);

			dgVector tmpDiag(row->m_JMinv.m_jacobianM0.m_linear.CompProduct4(row->m_Jt.m_jacobianM0.m_linear) + row->m_JMinv.m_jacobianM0.m_angular.CompProduct4(row->m_Jt.m_jacobianM0.m_angular) +
							 row->m_JMinv.m_jacobianM1.m_linear.CompProduct4(row->m_Jt.m_jacobianM1.m_linear) + row->m_JMinv.m_jacobianM1.m_angular.CompProduct4(row->m_Jt.m_jacobianM1.m_angular));

			dgVector tmpAccel(row->m_JMinv.m_jacobianM0.m_linear.CompProduct4(accel0) + row->m_JMinv.m_jacobianM0.m_angular.CompProduct4(alpha0)+ 
							  row->m_JMinv.m_jacobianM1.m_linear.CompProduct4(accel1) + row->m_JMinv.m_jacobianM1.m_angular.CompProduct4(alpha1));

			dgFloat32 extenalAcceleration = -(tmpAccel.m_x + tmpAccel.m_y + tmpAccel.m_z);
			row->m_deltaAccel = extenalAcceleration * forceImpulseScale;
			row->m_coordenateAccel += extenalAcceleration * forceImpulseScale;
			dgAssert(row->m_jointFeebackForce);
			row->m_force = row->m_jointFeebackForce[0].m_force * forceImpulseScale;

			row->m_maxImpact = dgFloat32(0.0f);

			//force[index] = 0.0f;
			dgAssert (tmpDiag.m_w == dgFloat32 (0.0f));
			dgFloat32 diag = (tmpDiag.AddHorizontal()).GetScalar();
			dgAssert(diag > dgFloat32(0.0f));
			row->m_diagDamp = diag * row->m_stiffness;

			//row->m_JMinvJt = diag;
			diag *= (dgFloat32(1.0f) + row->m_stiffness);
			row->m_invJMinvJt = dgFloat32(1.0f) / diag;
			index++;
		}
	}
}


void dgWorldDynamicUpdate::BuildJacobianMatrix (dgIsland* const island, dgInt32 threadIndex, dgFloat32 timestep) const 
{
	dTimeTrackerEvent(__FUNCTION__);
	dgAssert (island->m_bodyCount >= 2);

	dgWorld* const world = (dgWorld*) this;
	const dgInt32 bodyCount = island->m_bodyCount;
	const dgInt32 jointCount = island->m_jointCount;

	dgBodyInfo* const bodyArrayPtr = (dgBodyInfo*) &world->m_bodiesMemory[0]; 
	dgBodyInfo* const bodyArray = &bodyArrayPtr[island->m_bodyStart];
	dgJacobian* const internalForces = &m_solverMemory.m_internalForces[island->m_bodyStart];

	dgAssert (((dgDynamicBody*) bodyArray[0].m_body)->IsRTTIType (dgBody::m_dynamicBodyRTTI));
	dgAssert ((((dgDynamicBody*)bodyArray[0].m_body)->m_accel % ((dgDynamicBody*)bodyArray[0].m_body)->m_accel) == dgFloat32 (0.0f));
	dgAssert ((((dgDynamicBody*)bodyArray[0].m_body)->m_alpha % ((dgDynamicBody*)bodyArray[0].m_body)->m_alpha) == dgFloat32 (0.0f));

	dgAssert(bodyArray[0].m_body->m_resting);
	internalForces[0].m_linear = dgVector::m_zero;
	internalForces[0].m_angular = dgVector::m_zero;

	if (timestep != dgFloat32 (0.0f)) {
		for (dgInt32 i = 1; i < bodyCount; i ++) {
			dgBody* const body = bodyArray[i].m_body;
			if (!body->m_equilibrium) {
				dgAssert (body->m_invMass.m_w > dgFloat32 (0.0f));
				body->AddDampingAcceleration(timestep);
				body->CalcInvInertiaMatrix ();
			}

			if (body->m_active) {
				// re use these variables for temp storage 
				body->m_netForce = body->m_veloc;
				body->m_netTorque = body->m_omega;

				internalForces[i].m_linear = dgVector::m_zero;
				internalForces[i].m_angular = dgVector::m_zero;
			}
		}

	} else {
		for (dgInt32 i = 1; i < bodyCount; i ++) {
			dgBody* const body = bodyArray[i].m_body;
			if (!body->m_equilibrium) {
				dgAssert (body->m_invMass.m_w > dgFloat32 (0.0f));
				body->CalcInvInertiaMatrix ();
			}
			if (body->m_active) {
				// re use these variables for temp storage 
				body->m_netForce = body->m_veloc;
				body->m_netTorque = body->m_omega;

				internalForces[i].m_linear = dgVector::m_zero;
				internalForces[i].m_angular = dgVector::m_zero;
			}
		}
	}
	
	if (jointCount) {
		dgInt32 rowCount = 0;
		dgJointInfo* const constraintArrayPtr = (dgJointInfo*) &world->m_jointsMemory[0];
		dgJointInfo* const constraintArray = &constraintArrayPtr[island->m_jointStart];

		GetJacobianDerivatives (island, threadIndex, rowCount, timestep);

		dgFloat32 forceOrImpulseScale = (timestep > dgFloat32 (0.0f)) ? dgFloat32 (1.0f) : dgFloat32 (0.0f);

		dgJacobianMatrixElement* const matrixRow = &m_solverMemory.m_memory[island->m_rowsStart];
		for (dgInt32 k = 0; k < jointCount; k ++) {
			const dgJointInfo* const jointInfo = &constraintArray[k];

			dgAssert(jointInfo->m_m0 >= 0);
			dgAssert(jointInfo->m_m0 < bodyCount);
			dgAssert(jointInfo->m_m1 >= 0);
			dgAssert(jointInfo->m_m1 < bodyCount);
			BuildJacobianMatrix (bodyArray, jointInfo, matrixRow, forceOrImpulseScale);
		}
	}
}

void dgWorldDynamicUpdate::ApplyExternalForcesAndAcceleration(const dgIsland* const island, dgInt32 threadIndex, dgFloat32 timestep, dgFloat32 maxAccNorm) const
{
	dgJacobian* const internalForces = &m_solverMemory.m_internalForces[island->m_bodyStart];

	dgInt32 bodyCount = island->m_bodyCount;
	for (dgInt32 i = 0; i < bodyCount; i ++) {
		internalForces[i].m_linear = dgVector::m_zero;
		internalForces[i].m_angular = dgVector::m_zero;
	}

	dgInt32 hasJointFeeback = 0;
	dgInt32 jointCount = island->m_jointCount;
	dgWorld* const world = (dgWorld*) this;
	dgJointInfo* const constraintArrayPtr = (dgJointInfo*) &world->m_jointsMemory[0];
	dgJointInfo* const constraintArray = &constraintArrayPtr[island->m_jointStart];

	dgJacobianMatrixElement* const matrixRow = &m_solverMemory.m_memory[island->m_rowsStart];
	for (dgInt32 i = 0; i < jointCount; i ++) {
		dgInt32 first = constraintArray[i].m_pairStart;
		dgInt32 count = constraintArray[i].m_pairCount;

		dgInt32 m0 = constraintArray[i].m_m0;
		dgInt32 m1 = constraintArray[i].m_m1;

		dgJacobian y0;
		dgJacobian y1;
		y0.m_linear = dgVector::m_zero;
		y0.m_angular = dgVector::m_zero;
		y1.m_linear = dgVector::m_zero;
		y1.m_angular = dgVector::m_zero;

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

	dgVector timeStepVect (timestep, timestep, timestep, dgFloat32 (0.0f));
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
				accel = dgVector::m_zero;
				body->m_accel = dgVector::m_zero;
			}

			dgVector alpha (body->m_invWorldInertiaMatrix.RotateVector (body->m_alpha));
			error = alpha % alpha;
			if (error < accelTol2) {
				alpha = dgVector::m_zero;
				body->m_alpha = dgVector::m_zero;
			}

			body->m_netForce = body->m_accel;
			body->m_netTorque = body->m_alpha;

			body->m_veloc += accel.CompProduct4(timeStepVect);
			dgVector correction (alpha * body->m_omega);
			body->m_omega += alpha.CompProduct4(timeStepVect.CompProduct4 (dgVector::m_half)) + correction.CompProduct4(timeStepVect.CompProduct4(timeStepVect.CompProduct4 (m_eulerTaylorCorrection)));
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

			body->m_netForce = dgVector::m_zero;
			body->m_netTorque = dgVector::m_zero;
			body->m_veloc += linearMomentum.Scale3(body->m_invMass.m_w);
			body->m_omega += body->m_invWorldInertiaMatrix.RotateVector (angularMomentum);
		}
	}
}



void dgWorldDynamicUpdate::InitJointForce(dgJointInfo* const jointInfo, dgJacobianMatrixElement* const matrixRow, dgJacobian& force0, dgJacobian& force1) const
{
	force0.m_linear = dgVector::m_zero;
	force0.m_angular = dgVector::m_zero;
	force1.m_linear = dgVector::m_zero;
	force1.m_angular = dgVector::m_zero;
	const dgInt32 first = jointInfo->m_pairStart;
	const dgInt32 count = jointInfo->m_pairCount;
	for (dgInt32 j = 0; j < count; j++) {
		dgJacobianMatrixElement* const row = &matrixRow[j + first];
		dgAssert(dgCheckFloat(row->m_force));
		dgVector val(row->m_force);
		force0.m_linear += row->m_Jt.m_jacobianM0.m_linear.CompProduct4(val);
		force0.m_angular += row->m_Jt.m_jacobianM0.m_angular.CompProduct4(val);
		force1.m_linear += row->m_Jt.m_jacobianM1.m_linear.CompProduct4(val);
		force1.m_angular += row->m_Jt.m_jacobianM1.m_angular.CompProduct4(val);
	}
}



dgFloat32 dgWorldDynamicUpdate::CalculateJointForce(dgJointInfo* const jointInfo, const dgBodyInfo* const bodyArray, dgJacobian* const internalForces, dgJacobianMatrixElement* const matrixRow) const
{
	dgVector accNorm(dgVector::m_zero);

	dgFloat32 cacheForce[DG_CONSTRAINT_MAX_ROWS + 4];
	cacheForce[0] = dgFloat32(1.0f);
	cacheForce[1] = dgFloat32(1.0f);
	cacheForce[2] = dgFloat32(1.0f);
	cacheForce[3] = dgFloat32(1.0f);
	dgFloat32* const normalForce = &cacheForce[4];

	dgConstraint* const constraint = jointInfo->m_joint;
	if (constraint->m_solverActive) {
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

			dgInt32 index = jointInfo->m_pairStart;
			for (dgInt32 i = 0; i < rowsCount; i++) {
				dgJacobianMatrixElement* const row = &matrixRow[index];

				dgAssert(row->m_Jt.m_jacobianM0.m_linear.m_w == dgFloat32(0.0f));
				dgAssert(row->m_Jt.m_jacobianM0.m_angular.m_w == dgFloat32(0.0f));
				dgAssert(row->m_Jt.m_jacobianM1.m_linear.m_w == dgFloat32(0.0f));
				dgAssert(row->m_Jt.m_jacobianM1.m_angular.m_w == dgFloat32(0.0f));

				dgVector a(row->m_JMinv.m_jacobianM0.m_linear.CompProduct4(linearM0) + row->m_JMinv.m_jacobianM0.m_angular.CompProduct4(angularM0) +
						   row->m_JMinv.m_jacobianM1.m_linear.CompProduct4(linearM1) + row->m_JMinv.m_jacobianM1.m_angular.CompProduct4(angularM1));

				//dgFloat32 a = row->m_coordenateAccel - acc.m_x - acc.m_y - acc.m_z - row->m_force * row->m_diagDamp;
				a = dgVector(row->m_coordenateAccel - row->m_force * row->m_diagDamp) - a.AddHorizontal();

				//dgFloat32 f = row->m_force + row->m_invDJMinvJt * a;
				dgVector f(row->m_force + row->m_invJMinvJt * a.GetScalar());

				dgInt32 frictionIndex = row->m_normalForceIndex;
				dgAssert(((frictionIndex < 0) && (normalForce[frictionIndex] == dgFloat32(1.0f))) || ((frictionIndex >= 0) && (normalForce[frictionIndex] >= dgFloat32(0.0f))));

				dgFloat32 frictionNormal = normalForce[frictionIndex];
				dgVector lowerFrictionForce(frictionNormal * row->m_lowerBoundFrictionCoefficent);
				dgVector upperFrictionForce(frictionNormal * row->m_upperBoundFrictionCoefficent);

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
				dgAssert(accNorm.m_x >= dgAbsf(a.m_x));

				dgVector prevValue(f - dgVector(row->m_force));

				row->m_force = f.GetScalar();
				normalForce[i] = f.GetScalar();

				row->m_maxImpact = f.Abs().GetMax(row->m_maxImpact).GetScalar();

				linearM0 += row->m_Jt.m_jacobianM0.m_linear.CompProduct4(prevValue);
				angularM0 += row->m_Jt.m_jacobianM0.m_angular.CompProduct4(prevValue);
				linearM1 += row->m_Jt.m_jacobianM1.m_linear.CompProduct4(prevValue);
				angularM1 += row->m_Jt.m_jacobianM1.m_angular.CompProduct4(prevValue);
				index++;
			}

			dgVector maxAccel(accNorm);
			for (dgInt32 i = 0; (i < 3) && (maxAccel.GetScalar() > m_solverConvergeQuality); i++) {
				maxAccel = dgFloat32(0.0f);
				dgInt32 index = jointInfo->m_pairStart;
				for (dgInt32 i = 0; i < rowsCount; i++) {
					dgJacobianMatrixElement* const row = &matrixRow[index];

					dgAssert(row->m_Jt.m_jacobianM0.m_linear.m_w == dgFloat32(0.0f));
					dgAssert(row->m_Jt.m_jacobianM0.m_angular.m_w == dgFloat32(0.0f));
					dgAssert(row->m_Jt.m_jacobianM1.m_linear.m_w == dgFloat32(0.0f));
					dgAssert(row->m_Jt.m_jacobianM1.m_angular.m_w == dgFloat32(0.0f));

					dgVector a(row->m_JMinv.m_jacobianM0.m_linear.CompProduct4(linearM0) + row->m_JMinv.m_jacobianM0.m_angular.CompProduct4(angularM0) +
							   row->m_JMinv.m_jacobianM1.m_linear.CompProduct4(linearM1) + row->m_JMinv.m_jacobianM1.m_angular.CompProduct4(angularM1));

					//dgFloat32 a = row->m_coordenateAccel - acc.m_x - acc.m_y - acc.m_z - row->m_force * row->m_diagDamp;
					a = dgVector(row->m_coordenateAccel - row->m_force * row->m_diagDamp) - a.AddHorizontal();

					//dgFloat32 f = row->m_force + row->m_invDJMinvJt * a;
					dgVector f(row->m_force + row->m_invJMinvJt * a.GetScalar());

					dgInt32 frictionIndex = row->m_normalForceIndex;
					dgAssert(((frictionIndex < 0) && (normalForce[frictionIndex] == dgFloat32(1.0f))) || ((frictionIndex >= 0) && (normalForce[frictionIndex] >= dgFloat32(0.0f))));

					dgFloat32 frictionNormal = normalForce[frictionIndex];
					dgVector lowerFrictionForce(frictionNormal * row->m_lowerBoundFrictionCoefficent);
					dgVector upperFrictionForce(frictionNormal * row->m_upperBoundFrictionCoefficent);

					//if (f > upperFrictionForce) {
					//	a = dgFloat32 (0.0f);
					//	f = upperFrictionForce;
					//} else if (f < lowerFrictionForce) {
					//	a = dgFloat32 (0.0f);
					//	f = lowerFrictionForce;
					//}
					a = a.AndNot((f > upperFrictionForce) | (f < lowerFrictionForce));
					f = f.GetMax(lowerFrictionForce).GetMin(upperFrictionForce);

					maxAccel = maxAccel.GetMax(a.Abs());
					dgAssert(maxAccel.m_x >= dgAbsf(a.m_x));

					dgVector prevValue(f - dgVector(row->m_force));

					row->m_force = f.GetScalar();
					normalForce[i] = f.GetScalar();

					row->m_maxImpact = f.Abs().GetMax(row->m_maxImpact).GetScalar();

					linearM0 += row->m_Jt.m_jacobianM0.m_linear.CompProduct4(prevValue);
					angularM0 += row->m_Jt.m_jacobianM0.m_angular.CompProduct4(prevValue);
					linearM1 += row->m_Jt.m_jacobianM1.m_linear.CompProduct4(prevValue);
					angularM1 += row->m_Jt.m_jacobianM1.m_angular.CompProduct4(prevValue);
					index++;
				}
			}

			internalForces[m0].m_linear = linearM0;
			internalForces[m0].m_angular = angularM0;
			internalForces[m1].m_linear = linearM1;
			internalForces[m1].m_angular = angularM1;
		}
	}
	return accNorm.GetScalar();
}



void dgWorldDynamicUpdate::CalculateForcesGameMode (const dgIsland* const island, dgInt32 threadIndex, dgFloat32 timestep, dgFloat32 maxAccNorm) const
{
	dTimeTrackerEvent(__FUNCTION__);
	dgWorld* const world = (dgWorld*) this;
	const dgInt32 bodyCount = island->m_bodyCount;
	const dgInt32 jointCount = island->m_jointCount;

	dgJacobian* const internalForces = &m_solverMemory.m_internalForces[island->m_bodyStart];
	dgBodyInfo* const bodyArrayPtr = (dgBodyInfo*) &world->m_bodiesMemory[0]; 
	dgJointInfo* const constraintArrayPtr = (dgJointInfo*) &world->m_jointsMemory[0];

	dgBodyInfo* const bodyArray = &bodyArrayPtr[island->m_bodyStart];
	dgJointInfo* const constraintArray = &constraintArrayPtr[island->m_jointStart];
	dgJacobianMatrixElement* const matrixRow = &m_solverMemory.m_memory[island->m_rowsStart];

	for (dgInt32 i = 0; i < jointCount; i ++) {
		dgJointInfo* const jointInfo = &constraintArray[i];
		dgConstraint* const constraint = jointInfo->m_joint;
		constraint->m_index = i;
		if (constraint->m_solverActive) {
			dgJacobian y0;
			dgJacobian y1;
			InitJointForce (jointInfo,  matrixRow, y0, y1);
			const dgInt32 m0 = jointInfo->m_m0;
			const dgInt32 m1 = jointInfo->m_m1;
			dgAssert (m0 != m1);
			internalForces[m0].m_linear += y0.m_linear;
			internalForces[m0].m_angular += y0.m_angular;
			internalForces[m1].m_linear += y1.m_linear;
			internalForces[m1].m_angular += y1.m_angular;
		}
	}

	const dgInt32 maxPasses = 4;

	dgFloat32 invTimestep = (timestep > dgFloat32 (0.0f)) ? dgFloat32 (1.0f) / timestep : dgFloat32 (0.0f);
	dgFloat32 invStepRK = (dgFloat32 (1.0f) / dgFloat32 (maxPasses));
	dgFloat32 timestepRK =  timestep * invStepRK;
	dgFloat32 invTimestepRK = invTimestep * dgFloat32 (maxPasses);
	dgAssert (bodyArray[0].m_body == world->m_sentinelBody);

	dgVector speedFreeze2 (world->m_freezeSpeed2 * dgFloat32 (0.1f));
	dgVector freezeOmega2 (world->m_freezeOmega2 * dgFloat32 (0.1f));
	dgVector forceActiveMask ((jointCount <= DG_SMALL_ISLAND_COUNT) ?  dgVector (-1, -1, -1, -1): dgFloat32 (0.0f));

	dgJointAccelerationDecriptor joindDesc;
	joindDesc.m_timeStep = timestepRK;
	joindDesc.m_invTimeStep = invTimestepRK;
	joindDesc.m_firstPassCoefFlag = dgFloat32 (0.0f);

	dgInt32 skeletonCount = 0;
	dgInt32 lru = dgAtomicExchangeAndAdd (&dgSkeletonContainer::m_lruMarker, 1);
	dgSkeletonContainer* skeletonArray[DG_MAX_SKELETON_JOINT_COUNT];
	for (dgInt32 i = 1; i < bodyCount; i ++) {
		dgDynamicBody* const body = (dgDynamicBody*) bodyArray[i].m_body;
		dgSkeletonContainer* const container = body->GetSkeleton();
		if (container && (container->m_lru != lru)) {
			container->m_lru = lru;
			skeletonArray[skeletonCount] = container;
			skeletonCount ++;
			dgAssert (skeletonCount < dgInt32 (sizeof (skeletonArray) / sizeof (skeletonArray[0])));
		}
	}

	for (dgInt32 i = 0; i < skeletonCount; i ++) {
		dgSkeletonContainer* const container = skeletonArray[i];
		if (!container->m_skeletonHardMotors) {
			container->InitMassMatrix (constraintArray, matrixRow);
		}
	}

	const dgInt32 passes = world->m_solverMode;
	for (dgInt32 step = 0; step < maxPasses; step ++) {
		if (joindDesc.m_firstPassCoefFlag == dgFloat32 (0.0f)) {
			for (dgInt32 curJoint = 0; curJoint < jointCount; curJoint ++) {
				dgJointInfo* const jointInfo = &constraintArray[curJoint];
				dgConstraint* const constraint = jointInfo->m_joint;
				if (constraint->m_solverActive) {
					joindDesc.m_rowsCount = jointInfo->m_pairCount;
					joindDesc.m_rowMatrix = &matrixRow[jointInfo->m_pairStart];
					constraint->JointAccelerations(&joindDesc);
				}
			}
			joindDesc.m_firstPassCoefFlag = dgFloat32 (1.0f);
		} else {
			for (dgInt32 curJoint = 0; curJoint < jointCount; curJoint ++) {
				dgJointInfo* const jointInfo = &constraintArray[curJoint];
				dgConstraint* const constraint = jointInfo->m_joint;
				if (constraint->m_solverActive) {
					const dgInt32 m0 = jointInfo->m_m0;
					const dgInt32 m1 = jointInfo->m_m1;
					const dgBody* const body0 = bodyArray[m0].m_body;
					const dgBody* const body1 = bodyArray[m1].m_body;
					if (!(body0->m_resting & body1->m_resting)) {
						joindDesc.m_rowsCount = jointInfo->m_pairCount;
						joindDesc.m_rowMatrix = &matrixRow[jointInfo->m_pairStart];
						constraint->JointAccelerations(&joindDesc);
					}
				}
			}
		}

		dgFloat32 accNorm(maxAccNorm * dgFloat32(2.0f));
		for (dgInt32 k = 0; (k < passes) && (accNorm > maxAccNorm); k++) {
			accNorm = dgFloat32(0.0f);
			for (dgInt32 i = 0; i < jointCount; i ++) {
				dgJointInfo* const jointInfo = &constraintArray[i];
				//const dgConstraint* const constraint = jointInfo->m_joint;
				dgFloat32 accel = CalculateJointForce(jointInfo, bodyArray, internalForces, matrixRow);
				accNorm = (accel > accNorm) ? accel : accNorm;
			}

			for (dgInt32 i = 0; i < skeletonCount; i++) {
				dgSkeletonContainer* const container = skeletonArray[i];
				//dgFloat32 accel = container->CalculateJointForce (constraintArray, bodyArray, internalForces, matrixRow);
				//accNorm = (accel > accNorm) ? accel : accNorm;
				container->CalculateJointForce (constraintArray, bodyArray, internalForces, matrixRow); 
			}
		}

		if (timestepRK != dgFloat32 (0.0f)) {
			dgVector timestep4 (timestepRK);
			for (dgInt32 i = 1; i < bodyCount; i ++) {
				dgDynamicBody* const body = (dgDynamicBody*) bodyArray[i].m_body;
				dgAssert (body->m_index == i);
				ApplyNetVelcAndOmega (body, internalForces[i], timestep4, speedFreeze2, forceActiveMask);
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
				const dgInt32 first = jointInfo->m_pairStart;
				const dgInt32 count = jointInfo->m_pairCount;

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
			ApplyNetTorqueAndForce (body, invTime, maxAccNorm2, forceActiveMask);
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
				body->m_netForce = dgVector::m_zero;
				body->m_netTorque = dgVector::m_zero;
			}
		}
	}
}

void dgWorldDynamicUpdate::ApplyNetVelcAndOmega (dgDynamicBody* const body, const dgJacobian& forceAndTorque, const dgVector& timestep4, const dgVector& speedFreeze2, const dgVector& forceActiveMask) const
{
	if (body->m_active) {
		dgVector force(forceAndTorque.m_linear);
		dgVector torque(forceAndTorque.m_angular);
		if (body->IsRTTIType(dgBody::m_dynamicBodyRTTI)) {
			force += body->m_accel;
			torque += body->m_alpha;
		}

#if 1
		// this method is more accurate numerically 
		dgVector velocStep((force.Scale4(body->m_invMass.m_w)).CompProduct4(timestep4));
		dgVector omegaStep((body->m_invWorldInertiaMatrix.RotateVector(torque)).CompProduct4(timestep4));
		if (!body->m_resting) {
			body->m_veloc += velocStep;
			body->m_omega += omegaStep;
		} else {
			dgVector velocStep2(velocStep.DotProduct4(velocStep));
			dgVector omegaStep2(omegaStep.DotProduct4(omegaStep));
			dgVector test((velocStep2 > speedFreeze2) | (omegaStep2 > speedFreeze2) | forceActiveMask);
			if (test.GetSignMask()) {
				body->m_resting = false;
			}
		}

#else
		// this method is Lagrange-Euler by is more expensive and accumulate more numerical error. 
		dgVector linearMomentum(force.CompProduct4(timestep4) + body->m_veloc.Scale4 (body->m_mass.m_w));
		dgVector angularMomentum(torque.CompProduct4(timestep4) + body->m_matrix.RotateVector(body->m_mass.CompProduct4(body->m_matrix.UnrotateVector(body->m_omega))));
		if (!body->m_resting) {
			body->m_veloc = linearMomentum.Scale4 (body->m_invMass.m_w);
			body->m_omega = body->m_invWorldInertiaMatrix.RotateVector(angularMomentum);
		} else {
			dgVector velocStep (linearMomentum.Scale4(body->m_invMass.m_w));
			dgVector omegaStep (body->m_invWorldInertiaMatrix.RotateVector(angularMomentum));
			dgVector velocStep2(velocStep.DotProduct4(velocStep));
			dgVector omegaStep2(omegaStep.DotProduct4(omegaStep));
			dgVector test((velocStep2 > speedFreeze2) | (omegaStep2 > speedFreeze2) | forceActiveMask);
			if (test.GetSignMask()) {
				body->m_resting = false;
			}
		}
#endif
	}

	dgAssert(body->m_veloc.m_w == dgFloat32(0.0f));
	dgAssert(body->m_omega.m_w == dgFloat32(0.0f));
}

void dgWorldDynamicUpdate::ApplyNetTorqueAndForce (dgDynamicBody* const body, const dgVector& invTimeStep, const dgVector& maxAccNorm2, const dgVector& forceActiveMask) const
{
	if (body->m_active) {
		// the initial velocity and angular velocity were stored in net force and net torque, for memory saving
		dgVector accel = (body->m_veloc - body->m_netForce).CompProduct4(invTimeStep);
		dgVector alpha = (body->m_omega - body->m_netTorque).CompProduct4(invTimeStep);
		dgVector accelTest((accel.DotProduct4(accel) > maxAccNorm2) | (alpha.DotProduct4(alpha) > maxAccNorm2) | forceActiveMask);
		//if ((accel % accel) < maxAccNorm2) {
		//	accel = dgVector::m_zero;
		//}
		//if ((alpha % alpha) < maxAccNorm2) {
		//	alpha = dgVector::m_zero;
		//}
		accel = accel & accelTest;
		alpha = alpha & accelTest;

		if (body->IsRTTIType(dgBody::m_dynamicBodyRTTI)) {
			body->m_accel = accel;
			body->m_alpha = alpha;
		}
		body->m_netForce = accel.Scale4(body->m_mass[3]);

		alpha = body->m_matrix.UnrotateVector(alpha);
		body->m_netTorque = body->m_matrix.RotateVector(alpha.CompProduct4(body->m_mass));
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
	dgInt32 first = jointInfo->m_pairStart;
	dgInt32 count = jointInfo->m_pairCount;

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

		deltaForce[j] = row->m_accel * row->m_invJMinvJt * activeRow[j];
		akNum += row->m_accel * deltaForce[j];
		accNorm = dgMax (dgAbsf (row->m_accel * activeRow[j]), accNorm);
	}

	dgFloat32 retAccel = accNorm;
	dgFloat32 clampedForceIndexValue = dgFloat32(0.0f);
	for (dgInt32 i = 0; (i < maxPasses) && (accNorm >  maxAccNorm); i ++) {
		dgJacobian y0;
		dgJacobian y1;
		y0.m_linear = dgVector::m_zero;
		y0.m_angular = dgVector::m_zero;
		y1.m_linear = dgVector::m_zero;
		y1.m_angular = dgVector::m_zero;

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
							deltaForce[j] = row->m_accel * row->m_invJMinvJt;
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

				deltaForce[j] = row->m_accel * row->m_invJMinvJt * activeRow[j];
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
					deltaAccel[j] = row->m_accel * row->m_invJMinvJt * activeRow[j];
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
	for (dgInt32 i = 0; i < bodyCount; i ++) {
		internalForces[i].m_linear = dgVector::m_zero;
		internalForces[i].m_angular = dgVector::m_zero;
	}

	const dgBodyInfo* const bodyArrayPtr = (dgBodyInfo*) &world->m_bodiesMemory[0]; 
	const dgBodyInfo* const bodyArray = &bodyArrayPtr[island->m_bodyStart];

	dgJacobianMatrixElement* const matrixRow = &m_solverMemory.m_memory[island->m_rowsStart];
	dgJointInfo* const constraintArrayPtr = (dgJointInfo*) &world->m_jointsMemory[0];
	dgJointInfo* const constraintArray = &constraintArrayPtr[island->m_jointStart];
	for (dgInt32 i = 0; i < jointCount; i ++) {
		dgJacobian y0;
		dgJacobian y1;
		y0.m_linear = dgVector::m_zero;
		y0.m_angular = dgVector::m_zero;
		y1.m_linear = dgVector::m_zero;
		y1.m_angular = dgVector::m_zero;

		dgInt32 first = constraintArray[i].m_pairStart;
		dgInt32 count = constraintArray[i].m_pairActiveCount;
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
		forceStepBuffer[i] = dgVector::m_zero;
	}

	dgInt32 maxPasses = 4;
	dgInt32 prevJoint = 0;
	dgFloat32 accNorm = maxAccNorm * dgFloat32 (2.0f);
	for (dgInt32 passes = 0; (passes < maxPasses) && (accNorm > maxAccNorm); passes ++) {

		accNorm = dgFloat32 (0.0f);
		for (dgInt32 currJoint = 0; currJoint < jointCount; currJoint ++) {
			dgJacobian y0;
			dgJacobian y1;
			y0.m_linear = dgVector::m_zero;
			y0.m_angular = dgVector::m_zero;
			y1.m_linear = dgVector::m_zero;
			y1.m_angular = dgVector::m_zero;

			dgInt32 first = constraintArray[prevJoint].m_pairStart;
			dgInt32 rowsCount = constraintArray[prevJoint].m_pairCount;
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

			first = constraintArray[currJoint].m_pairStart;
			rowsCount = constraintArray[currJoint].m_pairCount;
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
		dgInt32 first = constraintArray[i].m_pairStart;
		dgInt32 count = constraintArray[i].m_pairCount;
		constraintArray[i].m_pairCount = dgInt16 (count);
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
		internalForces[i].m_linear = dgVector::m_zero;
		internalForces[i].m_angular = dgVector::m_zero;
	}

	for (dgInt32 i = 0; i < jointCount; i ++) {
		dgJacobian y0;
		dgJacobian y1;
		y0.m_linear = dgVector::m_zero;
		y0.m_angular = dgVector::m_zero;
		y1.m_linear = dgVector::m_zero;
		y1.m_angular = dgVector::m_zero;
		dgInt32 first = constraintArray[i].m_pairStart;
		dgInt32 count = constraintArray[i].m_pairActiveCount;
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
		dgInt32 first = constraintArray[i].m_pairStart;
		dgInt32 count = constraintArray[i].m_pairActiveCount;
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
					row->m_deltaForce = row->m_accel * row->m_invJMinvJt;
					akNum += row->m_accel * row->m_deltaForce;
					accNorm = dgMax (dgAbsf (row->m_accel), accNorm);
					isClamped[j] = false;
				}
			}
		}

		if (activeCount < count) {
			dgInt32 i0 = 0;
			dgInt32 i1 = count - 1;
			constraintArray[i].m_pairActiveCount = dgInt16 (activeCount);
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
			internalForces[i].m_linear = dgVector::m_zero;
			internalForces[i].m_angular = dgVector::m_zero;
		}

		for (dgInt32 i = 0; i < jointCount; i ++) {
			dgJacobian y0;
			dgJacobian y1;

			y0.m_linear = dgVector::m_zero;
			y0.m_angular = dgVector::m_zero;
			y1.m_linear = dgVector::m_zero;
			y1.m_angular = dgVector::m_zero;
			dgInt32 first = constraintArray[i].m_pairStart;
			dgInt32 count = constraintArray[i].m_pairActiveCount;

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
		dgVector tmpDen(dgVector::m_zero);
		for (dgInt32 i = 0; i < jointCount; i ++) {
			dgInt32 first = constraintArray[i].m_pairStart;
			dgInt32 count = constraintArray[i].m_pairActiveCount;
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
				dgInt32 first = constraintArray[i].m_pairStart;
				dgInt32 count = constraintArray[i].m_pairActiveCount;
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
				dgInt32 first = constraintArray[i].m_pairStart;
				dgInt32 count = constraintArray[i].m_pairActiveCount;
				for (dgInt32 j = 0; j < count; j ++) {
					dgJacobianMatrixElement* const row = &matrixRow[j + first];
					row->m_force += ak * row->m_deltaForce;
					row->m_accel -= ak * row->m_deltaAccel;
				}
			}

			dgInt32 first = constraintArray[clampedForceJoint].m_pairStart;
			dgInt32 count = constraintArray[clampedForceJoint].m_pairActiveCount;
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
			constraintArray[clampedForceJoint].m_pairActiveCount = dgInt16 (activeCount);

			forceRows = 0;
			akNum = dgFloat32 (0.0f);
			accNorm = dgFloat32(0.0f);
			for (dgInt32 i = 0; i < jointCount; i ++) {
				dgInt32 first = constraintArray[i].m_pairStart;
				dgInt32 count = constraintArray[i].m_pairActiveCount;
				forceRows += count;

				for (dgInt32 j = 0; j < count; j ++) {
					dgJacobianMatrixElement* const row = &matrixRow[first + j];
					dgAssert ((i != clampedForceJoint) || !((dgAbsf (row->m_lowerBoundFrictionCoefficent - row->m_force) < dgFloat32 (1.0e-5f)) && (row->m_accel < dgFloat32 (0.0f))));
					dgAssert ((i != clampedForceJoint) || !((dgAbsf (row->m_upperBoundFrictionCoefficent - row->m_force) < dgFloat32 (1.0e-5f)) && (row->m_accel > dgFloat32 (0.0f))));
					row->m_deltaForce = row->m_accel * row->m_invJMinvJt;
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
				dgInt32 first = constraintArray[i].m_pairStart;
				dgInt32 count = constraintArray[i].m_pairActiveCount;
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
					dgInt32 first = constraintArray[i].m_pairStart;
					dgInt32 count = constraintArray[i].m_pairActiveCount;
					for (dgInt32 j = 0; j < count; j ++) {
						dgJacobianMatrixElement* const row = &matrixRow[first + j];
						row->m_deltaAccel = row->m_accel * row->m_invJMinvJt;
						akNum += row->m_accel * row->m_deltaAccel;
					}
				}

				dgAssert (akNum >= dgFloat32 (0.0f));
				dgAssert (akDen > dgFloat32(0.0f));
				akDen = dgMax (akDen, dgFloat32 (1.0e-17f));
				ak = dgFloat32 (akNum / akDen);
				for (dgInt32 i = 0; i < jointCount; i ++) {
					dgInt32 first = constraintArray[i].m_pairStart;
					dgInt32 count = constraintArray[i].m_pairActiveCount;
					for (dgInt32 j = 0; j < count; j ++) {
						dgJacobianMatrixElement* const row = &matrixRow[first + j];
						row->m_deltaForce = row->m_deltaAccel + ak * row->m_deltaForce;
					}
				}
			}
		}
		totalPassesCount ++;
	}
	
	ApplyExternalForcesAndAcceleration (island, threadIndex, timestep, maxAccNorm);
}


void dgWorldDynamicUpdate::CalculateReactionsForces(const dgIsland* const island, dgInt32 threadIndex, dgFloat32 timestep, dgFloat32 maxAccNorm) const
{
	if (island->m_jointCount == 0) {
		ApplyExternalForcesAndAcceleration (island, threadIndex, timestep, dgFloat32 (0.0f));
	} else {
		dgWorld* const world = (dgWorld*) this;
		if (world->m_solverMode) {
			CalculateForcesGameMode (island, threadIndex, timestep, maxAccNorm);
		} else {
			dgAssert (timestep > dgFloat32 (0.0f));
			// remember to make the change for the impulsive solver for CC
			CalculateForcesSimulationMode (island, threadIndex, timestep, maxAccNorm);
		}
	}
}
