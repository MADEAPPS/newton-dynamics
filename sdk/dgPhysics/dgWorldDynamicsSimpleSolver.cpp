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
#include "dgDynamicBody.h"
#include "dgSkeletonContainer.h"
#include "dgCollisionInstance.h"
#include "dgWorldDynamicUpdate.h"
#include "dgBilateralConstraint.h"


void dgWorldDynamicUpdate::ResolveClusterForces(dgBodyCluster* const cluster, dgInt32 threadID, dgFloat32 timestep) const
{
	dgInt32 activeJoint = cluster->m_jointCount;
	if (activeJoint > 0) {
		activeJoint = SortClusters(cluster, timestep, threadID);
	}

	dgWorld* const world = (dgWorld*) this;
	dgJointInfo* const constraintArrayPtr = (dgJointInfo*)&world->m_jointsMemory[0];
	dgJointInfo* const constraintArray = &constraintArrayPtr[cluster->m_jointStart];

	if (!cluster->m_isContinueCollision) {
		//if ((activeJoint == 1) && (cluster->m_jointCount == 1)) {
		if ((activeJoint == 1) && (cluster->m_jointCount == 1) && (constraintArray[0].m_joint->GetId() == dgConstraint::m_contactConstraint)) {
			BuildJacobianMatrix(cluster, threadID, timestep);
			//CalculateClusterReactionForces(cluster, threadID, timestep);
			CalculateSingleContactReactionForces(cluster, threadID, timestep);
		} else if (activeJoint >= 1) {
			BuildJacobianMatrix(cluster, threadID, timestep);
			CalculateClusterReactionForces(cluster, threadID, timestep);
		} else if (cluster->m_jointCount == 0) {
			IntegrateExternalForce(cluster, timestep, threadID);
		} else {
			dgAssert((activeJoint == 0) && cluster->m_jointCount);
			dgBodyInfo* const bodyArrayPtr = (dgBodyInfo*)&world->m_bodiesMemory[0];
			dgBodyInfo* const bodyArray = &bodyArrayPtr[cluster->m_bodyStart];
			dgVector zero(dgVector::m_zero);
			for (dgInt32 i = 1; i < cluster->m_bodyCount; i++) {
				dgDynamicBody* const body = (dgDynamicBody*)bodyArray[i].m_body;
				body->m_accel = zero;
				body->m_alpha = zero;
			}
		}

		IntegrateVelocity (cluster, DG_SOLVER_MAX_ERROR, timestep, threadID); 
	} else {
		// calculate reaction forces and new velocities
		BuildJacobianMatrix (cluster, threadID, timestep);
		IntegrateReactionsForces (cluster, threadID, timestep);

		// see if the island goes to sleep
		bool isAutoSleep = true;
		bool stackSleeping = true;
		dgInt32 sleepCounter = 10000;

		const dgInt32 bodyCount = cluster->m_bodyCount;
		dgBodyInfo* const bodyArrayPtr = (dgBodyInfo*) &world->m_bodiesMemory[0]; 
		dgBodyInfo* const bodyArray = &bodyArrayPtr[cluster->m_bodyStart];

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

				const dgFloat32 accel2 = body->m_accel.DotProduct3(body->m_accel);
				const dgFloat32 alpha2 = body->m_alpha.DotProduct3(body->m_alpha);
				const dgFloat32 speed2 = body->m_veloc.DotProduct3(body->m_veloc);
				const dgFloat32 omega2 = body->m_omega.DotProduct3(body->m_omega);

				maxAccel = dgMax (maxAccel, accel2);
				maxAlpha = dgMax (maxAlpha, alpha2);
				maxSpeed = dgMax (maxSpeed, speed2);
				maxOmega = dgMax (maxOmega, omega2);

				bool equilibrium = (accel2 < accelFreeze) && (alpha2 < accelFreeze) && (speed2 < speedFreeze) && (omega2 < speedFreeze);
				if (equilibrium) {
					dgVector veloc (body->m_veloc * forceDampVect);
					dgVector omega = body->m_omega * forceDampVect;
					body->m_veloc = (veloc.DotProduct4(veloc) > m_velocTol) & veloc;
					body->m_omega = (omega.DotProduct4(omega) > m_velocTol) & omega;

				}
				body->m_equilibrium = equilibrium ? 1 : 0;
				stackSleeping &= equilibrium;
				isAutoSleep &= body->m_autoSleep;

				sleepCounter = dgMin (sleepCounter, body->m_sleepingCounter);
			}
			// clear accel and angular acceleration
			body->m_accel = dgVector::m_zero;
			body->m_alpha = dgVector::m_zero;
		}

		if (isAutoSleep) {
			if (stackSleeping) {
				// the island went to sleep mode, 
				for (dgInt32 i = 1; i < bodyCount; i ++) {
					dgBody* const body = bodyArray[i].m_body;
					dgAssert (body->IsRTTIType (dgBody::m_dynamicBodyRTTI) || body->IsRTTIType (dgBody::m_kinematicBodyRTTI));
					body->m_accel = dgVector::m_zero;
					body->m_alpha = dgVector::m_zero;
					body->m_veloc = dgVector::m_zero;
					body->m_omega = dgVector::m_zero;
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
							dgBody* const body = bodyArray[i].m_body;
							dgAssert (body->IsRTTIType (dgBody::m_dynamicBodyRTTI) || body->IsRTTIType (dgBody::m_kinematicBodyRTTI));
							body->m_accel = dgVector::m_zero;
							body->m_alpha = dgVector::m_zero;
							body->m_veloc = dgVector::m_zero;
							body->m_omega = dgVector::m_zero;
							body->m_equilibrium = 1;
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
			const dgInt32 jointCount = cluster->m_jointCount;
			//dgJointInfo* const constraintArrayPtr = (dgJointInfo*) &world->m_jointsMemory[0];
			//dgJointInfo* const constraintArray = &constraintArrayPtr[cluster->m_jointStart];

			dgFloat32 timeRemaining = timestep;
			const dgFloat32 timeTol = dgFloat32 (0.01f) * timestep;
			for (dgInt32 i = 0; (i < DG_MAX_CONTINUE_COLLISON_STEPS) && (timeRemaining > timeTol); i ++) {
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
							timeToImpact = dgMin (timeToImpact, world->CalculateTimeToImpact (contact, timeToImpact, threadID, p, q, normal, dgFloat32 (-1.0f / 256.0f)));
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
					if (timeToImpact >= dgFloat32 (-1.0e-5f)) {
						for (dgInt32 j = 1; j < bodyCount; j++) {
							dgDynamicBody* const body = (dgDynamicBody*)bodyArray[j].m_body;
							if (body->IsRTTIType(dgBody::m_dynamicBodyRTTI)) {
								body->IntegrateVelocity(timeToImpact);
								body->UpdateWorlCollisionMatrix();
							}
						}
					}

					CalculateClusterContacts (cluster, timeRemaining, lru, threadID);
					BuildJacobianMatrix (cluster, threadID, 0.0f);
					IntegrateReactionsForces (cluster, threadID, 0.0f);

					bool clusterReceding = true;
					const dgFloat32 step = timestep * dgFloat32 (1.0f / DG_MAX_CONTINUE_COLLISON_STEPS); 
					for (dgInt32 k = 0; (k < DG_MAX_CONTINUE_COLLISON_STEPS) && clusterReceding; k ++) {
						dgFloat32 smallTimeStep = dgMin (step, timeRemaining);
						timeRemaining -= smallTimeStep;
						for (dgInt32 j = 1; j < bodyCount; j ++) {
							dgDynamicBody* const body = (dgDynamicBody*) bodyArray[j].m_body;
							if (body->IsRTTIType (dgBody::m_dynamicBodyRTTI)) {
								body->IntegrateVelocity (smallTimeStep);
								body->UpdateWorlCollisionMatrix();
							}
						}

						clusterReceding = false;
						if (timeRemaining > timeTol) {
							CalculateClusterContacts (cluster, timeRemaining, lru, threadID);

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
										dgVector vel0 (veloc0 + omega0.CrossProduct3(contactMaterial->m_point - com0));
										dgVector vel1 (veloc1 + omega1.CrossProduct3(contactMaterial->m_point - com1));
										dgVector vRel (vel0 - vel1);
										dgAssert (contactMaterial->m_normal.m_w == dgFloat32 (0.0f));
										dgFloat32 speed = vRel.DotProduct4(contactMaterial->m_normal).m_w;
										isColliding |= (speed < dgFloat32 (0.0f));
									}
								}
							}
							clusterReceding = !isColliding;
						}
					}
				}
			}

			if (timeRemaining > dgFloat32 (0.0)) {
				for (dgInt32 j = 1; j < bodyCount; j ++) {
					dgDynamicBody* const body = (dgDynamicBody*) bodyArray[j].m_body;
					if (body->IsRTTIType (dgBody::m_dynamicBodyRTTI)) {
						body->IntegrateVelocity(timeRemaining);
						body->UpdateCollisionMatrix (timeRemaining, threadID);
					}
				}
			} else {
				for (dgInt32 j = 1; j < bodyCount; j ++) {
					dgDynamicBody* const body = (dgDynamicBody*) bodyArray[j].m_body;
					if (body->IsRTTIType (dgBody::m_dynamicBodyRTTI)) {
						body->UpdateCollisionMatrix (timestep, threadID);
					}
				}
			}
		}
	}
}


void dgWorldDynamicUpdate::CalculateClusterContacts(dgBodyCluster* const cluster, dgFloat32 timestep, dgInt32 currLru, dgInt32 threadID) const
{
	dgWorld* const world = (dgWorld*) this;
	dgInt32 jointCount = cluster->m_jointCount;
	dgJointInfo* const constraintArrayPtr = (dgJointInfo*) &world->m_jointsMemory[0];
	dgJointInfo* const constraintArray = &constraintArrayPtr[cluster->m_jointStart];

	dgBroadPhase::dgPair pair;
	dgContactPoint contactArray[DG_MAX_CONTATCS];
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



void dgWorldDynamicUpdate::IntegrateExternalForce(const dgBodyCluster* const cluster, dgFloat32 timestep, dgInt32 threadID) const
{
	dgWorld* const world = (dgWorld*) this;
	dgBodyInfo* const bodyArrayPtr = (dgBodyInfo*) &world->m_bodiesMemory[0]; 
	dgBodyInfo* const bodyArray = &bodyArrayPtr[cluster->m_bodyStart];

	dgAssert (timestep > dgFloat32 (0.0f));
	const dgInt32 bodyCount = cluster->m_bodyCount;
	for (dgInt32 i = 1; i < bodyCount; i ++) {
		dgDynamicBody* const body = (dgDynamicBody*) bodyArray[i].m_body;
		body->AddDampingAcceleration(timestep);
		body->IntegrateOpenLoopExternalForce(timestep);
	}
}


void dgWorldDynamicUpdate::CalculateNetAcceleration(dgBody* const body, const dgVector& invTimeStep, const dgVector& maxAccNorm2) const
{
	dgAssert(body->IsRTTIType(dgBody::m_dynamicBodyRTTI) || body->IsRTTIType(dgBody::m_kinematicBodyRTTI));
	// the initial velocity and angular velocity were stored in m_accel and body->m_alpha for memory saving
	dgVector accel (invTimeStep * (body->m_veloc - body->m_accel));
	dgVector alpha (invTimeStep * (body->m_omega - body->m_alpha));
	dgVector accelTest((accel.DotProduct4(accel) > maxAccNorm2) | (alpha.DotProduct4(alpha) > maxAccNorm2));
	accel = accel & accelTest;
	alpha = alpha & accelTest;

	body->m_accel = accel;
	body->m_alpha = alpha;
}


void dgWorldDynamicUpdate::IntegrateReactionsForces(const dgBodyCluster* const cluster, dgInt32 threadID, dgFloat32 timestep) const
{
	if (cluster->m_jointCount == 0) {
		IntegrateExternalForce(cluster, timestep, threadID);
	} else {
		CalculateClusterReactionForces(cluster, threadID, timestep);
	}
}


dgFloat32 dgWorldDynamicUpdate::CalculateJointForce_3_13(const dgJointInfo* const jointInfo, const dgBodyInfo* const bodyArray, dgJacobian* const internalForces, dgJacobianMatrixElement* const matrixRow) const
{
	dgVector accNorm(dgVector::m_zero);
	const dgInt32 m0 = jointInfo->m_m0;
	const dgInt32 m1 = jointInfo->m_m1;
	const dgBody* const body0 = bodyArray[m0].m_body;
	const dgBody* const body1 = bodyArray[m1].m_body;

	if (!(body0->m_resting & body1->m_resting)) {
		dgFloat32 normalForce[DG_CONSTRAINT_MAX_ROWS + 4];
		dgVector linearM0(internalForces[m0].m_linear);
		dgVector angularM0(internalForces[m0].m_angular);
		dgVector linearM1(internalForces[m1].m_linear);
		dgVector angularM1(internalForces[m1].m_angular);

		const dgVector scale0(jointInfo->m_scale0);
		const dgVector scale1(jointInfo->m_scale1);

		const dgInt32 index = jointInfo->m_pairStart;
		const dgInt32 rowsCount = jointInfo->m_pairCount;

		normalForce[rowsCount] = dgFloat32 (1.0f);

		
		dgVector firstPass(dgVector::m_one);
		dgVector maxAccel(dgVector::m_three);
		const dgFloat32 restAcceleration = DG_SOLVER_MAX_ERROR * DG_SOLVER_MAX_ERROR * dgFloat32(4.0f);
		for (dgInt32 i = 0; (i < 4) && (maxAccel.GetScalar() > restAcceleration); i++) {
			maxAccel = dgFloat32(0.0f);
			for (dgInt32 k = 0; k < rowsCount; k++) {
				dgJacobianMatrixElement* const row = &matrixRow[index + k];

				dgAssert(row->m_Jt.m_jacobianM0.m_linear.m_w == dgFloat32(0.0f));
				dgAssert(row->m_Jt.m_jacobianM0.m_angular.m_w == dgFloat32(0.0f));
				dgAssert(row->m_Jt.m_jacobianM1.m_linear.m_w == dgFloat32(0.0f));
				dgAssert(row->m_Jt.m_jacobianM1.m_angular.m_w == dgFloat32(0.0f));

				dgVector diag(row->m_JMinv.m_jacobianM0.m_linear * linearM0 + row->m_JMinv.m_jacobianM0.m_angular * angularM0 +
							  row->m_JMinv.m_jacobianM1.m_linear * linearM1 + row->m_JMinv.m_jacobianM1.m_angular * angularM1);

				dgVector accel(row->m_coordenateAccel - row->m_force * row->m_diagDamp - (diag.AddHorizontal()).GetScalar());
				dgVector force(row->m_force + row->m_invJinvMJt * accel.GetScalar());

				dgInt32 frictionIndex = row->m_normalForceIndex;
				dgAssert (frictionIndex >= 0);
				dgAssert (frictionIndex <= rowsCount);

				dgFloat32 frictionNormal = normalForce[frictionIndex];
				dgVector lowerFrictionForce(frictionNormal * row->m_lowerBoundFrictionCoefficent);
				dgVector upperFrictionForce(frictionNormal * row->m_upperBoundFrictionCoefficent);

				accel = accel.AndNot((force > upperFrictionForce) | (force < lowerFrictionForce));
				force = force.GetMax(lowerFrictionForce).GetMin(upperFrictionForce);

				maxAccel = maxAccel.GetMax(accel.Abs());
				dgAssert(maxAccel.m_x >= dgAbs(accel.m_x));

				accNorm = accNorm.GetMax(maxAccel * firstPass);

				dgVector deltaForce(force - dgVector(row->m_force));
				row->m_force = force.GetScalar();
				normalForce[k] = force.GetScalar();

				dgVector deltaforce0(scale0 * deltaForce);
				dgVector deltaforce1(scale1 * deltaForce);

				linearM0 += row->m_Jt.m_jacobianM0.m_linear * deltaforce0;
				angularM0 += row->m_Jt.m_jacobianM0.m_angular * deltaforce0;
				linearM1 += row->m_Jt.m_jacobianM1.m_linear * deltaforce1;
				angularM1 += row->m_Jt.m_jacobianM1.m_angular * deltaforce1;
			}
			firstPass = dgVector::m_zero;
		}

		for (dgInt32 i = 0; i < rowsCount; i++) {
			dgJacobianMatrixElement* const row = &matrixRow[index + i];
			row->m_maxImpact = dgMax(dgAbs(row->m_force), row->m_maxImpact);
		}

		internalForces[m0].m_linear = linearM0;
		internalForces[m0].m_angular = angularM0;
		internalForces[m1].m_linear = linearM1;
		internalForces[m1].m_angular = angularM1;
	}

	return accNorm.GetScalar() * accNorm.GetScalar();
}

dgFloat32 dgWorldDynamicUpdate::CalculateJointForce_1_50(const dgJointInfo* const jointInfo, const dgBodyInfo* const bodyArray, dgJacobian* const internalForces, dgJacobianMatrixElement* const matrixRow) const
{
	dgFloat32 accNorm = dgFloat32(0.0f);
	const dgInt32 m0 = jointInfo->m_m0;
	const dgInt32 m1 = jointInfo->m_m1;
	const dgBody* const body0 = bodyArray[m0].m_body;
	const dgBody* const body1 = bodyArray[m1].m_body;
	if (!(body0->m_resting & body1->m_resting)) {
		const dgInt32 index = jointInfo->m_pairStart;
		const dgInt32 rowsCount = jointInfo->m_pairCount;
		dgAssert(rowsCount <= DG_CONSTRAINT_MAX_ROWS);

		dgVector linearM0(internalForces[m0].m_linear);
		dgVector angularM0(internalForces[m0].m_angular);
		dgVector linearM1(internalForces[m1].m_linear);
		dgVector angularM1(internalForces[m1].m_angular);

		dgOldSolverNetwon_1_5<DG_CONSTRAINT_MAX_ROWS> solver;
		solver.SetSize(rowsCount);

		dgFloat32* const b = solver.GetB();
		dgFloat32* const x = solver.GetX();
		dgFloat32* const low = solver.GetLow();
		dgFloat32* const high = solver.GetHigh();
		dgFloat32* const invDiag = solver.GetInvDiag();
		dgFloat32* const massMatrix = solver.GetMatrixRow(0);
		dgInt32* const frictionIndex = solver.GetFrictionIndex();

		for (dgInt32 i = 0; i < rowsCount; i++) {
			const dgJacobianMatrixElement* const row_i = &matrixRow[index + i];
			dgFloat32* const massMatrixRow = solver.GetMatrixRow(i);
			const dgJacobian JMinvM0(row_i->m_JMinv.m_jacobianM0);
			const dgJacobian JMinvM1(row_i->m_JMinv.m_jacobianM1);
			invDiag[i] = row_i->m_invJinvMJt;
			massMatrixRow[i] = row_i->m_jinvMJt;
			for (dgInt32 j = i + 1; j < rowsCount; j++) {
				const dgJacobianMatrixElement* const row_j = &matrixRow[index + j];
				dgVector offDiag(JMinvM0.m_linear * row_j->m_Jt.m_jacobianM0.m_linear + JMinvM0.m_angular * row_j->m_Jt.m_jacobianM0.m_angular +
					JMinvM1.m_linear * row_j->m_Jt.m_jacobianM1.m_linear + JMinvM1.m_angular * row_j->m_Jt.m_jacobianM1.m_angular);
				dgFloat32 val1 = offDiag.AddHorizontal().GetScalar();
				massMatrixRow[j] = val1;
				massMatrix[j * rowsCount + i] = val1;
			}

			x[i] = row_i->m_force;
			b[i] = row_i->m_coordenateAccel;
			low[i] = row_i->m_lowerBoundFrictionCoefficent;
			high[i] = row_i->m_upperBoundFrictionCoefficent;
			frictionIndex[i] = row_i->m_normalForceIndex;
		}

		accNorm = solver.Solve();

		const dgVector scale0(jointInfo->m_scale0);
		const dgVector scale1(jointInfo->m_scale1);
		for (dgInt32 i = 0; i < rowsCount; i++) {
			dgJacobianMatrixElement* const row = &matrixRow[index + i];

			dgVector deltaForce(x[i] - row->m_force);
			row->m_maxImpact = dgMax(dgAbs(row->m_force), row->m_maxImpact);

			row->m_force = x[i];
			dgVector deltaforce0(scale0 * deltaForce);
			dgVector deltaforce1(scale1 * deltaForce);
			linearM0 += row->m_Jt.m_jacobianM0.m_linear * deltaforce0;
			angularM0 += row->m_Jt.m_jacobianM0.m_angular * deltaforce0;
			linearM1 += row->m_Jt.m_jacobianM1.m_linear * deltaforce1;
			angularM1 += row->m_Jt.m_jacobianM1.m_angular * deltaforce1;
		}

		internalForces[m0].m_linear = linearM0;
		internalForces[m0].m_angular = angularM0;
		internalForces[m1].m_linear = linearM1;
		internalForces[m1].m_angular = angularM1;
	}
	return accNorm;
}


dgFloat32 dgWorldDynamicUpdate::CalculateJointForce(const dgJointInfo* const jointInfo, const dgBodyInfo* const bodyArray, dgJacobian* const internalForces, dgJacobianMatrixElement* const matrixRow) const
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

			dgAssert(row->m_Jt.m_jacobianM0.m_linear.m_w == dgFloat32(0.0f));
			dgAssert(row->m_Jt.m_jacobianM0.m_angular.m_w == dgFloat32(0.0f));
			dgAssert(row->m_Jt.m_jacobianM1.m_linear.m_w == dgFloat32(0.0f));
			dgAssert(row->m_Jt.m_jacobianM1.m_angular.m_w == dgFloat32(0.0f));

			dgVector diag(row->m_JMinv.m_jacobianM0.m_linear * linearM0 + row->m_JMinv.m_jacobianM0.m_angular * angularM0 +
						  row->m_JMinv.m_jacobianM1.m_linear * linearM1 + row->m_JMinv.m_jacobianM1.m_angular * angularM1);

			dgVector accel(row->m_coordenateAccel - row->m_force * row->m_diagDamp - (diag.AddHorizontal()).GetScalar());
			dgVector force(row->m_force + row->m_invJinvMJt * accel.GetScalar());

			dgAssert(row->m_normalForceIndex >= 0);
			dgAssert(row->m_normalForceIndex <= rowsCount);
			frictionIndex[i] = row->m_normalForceIndex;

			dgVector frictionNormal (x[frictionIndex[i]]);
			dgVector lowerFrictionForce(frictionNormal * row->m_lowerBoundFrictionCoefficent);
			dgVector upperFrictionForce(frictionNormal * row->m_upperBoundFrictionCoefficent);

			accel = accel.AndNot((force > upperFrictionForce) | (force < lowerFrictionForce));
			force = force.GetMax(lowerFrictionForce).GetMin(upperFrictionForce);
			accNorm += accel.GetScalar() * accel.GetScalar();

			dgVector deltaForce(force - dgVector (row->m_force));

			x[i] = force;
			b[i] = dgVector (row->m_coordenateAccel);
			low[i] = dgVector (row->m_lowerBoundFrictionCoefficent);
			high[i] = dgVector (row->m_upperBoundFrictionCoefficent);
			diagDamp[i] = dgVector (row->m_diagDamp);
			invJinvMJt[i] = dgVector (row->m_invJinvMJt);

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
			dgVector maxAccel (accNorm);
			for (dgInt32 j = 0; (j < 4) && (maxAccel.GetScalar() > tol2); j++) {
				maxAccel = dgFloat32(0.0f);
				for (dgInt32 i = 0; i < rowsCount; i++) {
					const dgJacobianMatrixElement* const row = &matrixRow[index + i];

					dgAssert(row->m_Jt.m_jacobianM0.m_linear.m_w == dgFloat32(0.0f));
					dgAssert(row->m_Jt.m_jacobianM0.m_angular.m_w == dgFloat32(0.0f));
					dgAssert(row->m_Jt.m_jacobianM1.m_linear.m_w == dgFloat32(0.0f));
					dgAssert(row->m_Jt.m_jacobianM1.m_angular.m_w == dgFloat32(0.0f));

					dgVector diag(row->m_JMinv.m_jacobianM0.m_linear * linearM0 + row->m_JMinv.m_jacobianM0.m_angular * angularM0 +
								  row->m_JMinv.m_jacobianM1.m_linear * linearM1 + row->m_JMinv.m_jacobianM1.m_angular * angularM1);

					dgVector accel(b[i] - x[i] * diagDamp[i] - diag.AddHorizontal());
					dgVector force(x[i] + invJinvMJt[i] * accel);

					const dgVector frictionNormal (x[frictionIndex[i]]);
					const dgVector lowerFrictionForce(frictionNormal * low[i]);
					const dgVector upperFrictionForce(frictionNormal * high[i]);
					const dgVector clampedForce (force.GetMax(lowerFrictionForce).GetMin(upperFrictionForce));

					const dgVector deltaForce(clampedForce - x[i]);
					x[i] = clampedForce;

					const dgVector clampedAccel (accel.AndNot((force > upperFrictionForce) | (force < lowerFrictionForce)));
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
				dgInt32 activeRows[DG_CONSTRAINT_MAX_ROWS];
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
					const dgVector x1 (x[i] + invJinvMJt[i] * b[i]);
					mask[i] = dgVector::m_one.AndNot ((x1 < low[i]) | (x1 > high[i])); 
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
						} else if (x1 > high[j].GetScalar()) {
							clampIndex = i;
							alpha = (high[j].GetScalar() - x[j].GetScalar()) / delta_x[j].GetScalar();
						}
						dgAssert(alpha >= dgFloat32(-1.0e-4f));
						if (alpha < dgFloat32(1.0e-6f)) {
							alpha = dgFloat32(0.0f);
						}
					}

					beta = dgFloat32(0.0f);
					dgVector alphav (alpha);
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
						activeRowsCount --;
						dgSwap (activeRows[clampIndex], activeRows[activeRowsCount]);
						for (dgInt32 i = 0; i < activeRowsCount; i++) {
							const dgInt32 j = activeRows[i];
							delta_x[j] = b[j];
							beta += (b[j] * b[j]).GetScalar();
						}
					} else {
						dgVector gamma (beta / num);
						for (dgInt32 i = 0; i < activeRowsCount; i++) {
							const dgInt32 j = activeRows[i];
							delta_x[j] = b[j] + gamma * delta_x[j];
						}
					}
				}

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
			}
		}
#else

		dgVector maxAccel(accNorm);
		dgVector sor(dgFloat32 (1.3f));
		const dgFloat32 tol2 = dgFloat32(1.0e-4f);
		for (dgInt32 j = 0; (j < 4) && (maxAccel.GetScalar() > tol2); j++) {
			maxAccel = dgVector::m_zero;
			for (dgInt32 i = 0; i < rowsCount; i++) {
				const dgJacobianMatrixElement* const row = &matrixRow[index + i];
				dgVector diag(row->m_JMinv.m_jacobianM0.m_linear * linearM0 + row->m_JMinv.m_jacobianM0.m_angular * angularM0 +
							  row->m_JMinv.m_jacobianM1.m_linear * linearM1 + row->m_JMinv.m_jacobianM1.m_angular * angularM1);

				dgVector accel(b[i] - x[i] * diagDamp[i] - diag.AddHorizontal());
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

		internalForces[m0].m_linear = linearM0;
		internalForces[m0].m_angular = angularM0;
		internalForces[m1].m_linear = linearM1;
		internalForces[m1].m_angular = angularM1;
	}

	return accNorm;
}


void dgWorldDynamicUpdate::CalculateSingleContactReactionForces(const dgBodyCluster* const cluster, dgInt32 threadID, dgFloat32 timestep) const
{
	dgWorld* const world = (dgWorld*) this;
	const dgInt32 bodyCount = cluster->m_bodyCount;
	const dgInt32 jointCount = cluster->m_jointCount;

	dgJacobian* const internalForces = &m_solverMemory.m_internalForcesBuffer[cluster->m_bodyStart];
	dgBodyInfo* const bodyArrayPtr = (dgBodyInfo*)&world->m_bodiesMemory[0];
	dgJointInfo* const constraintArrayPtr = (dgJointInfo*)&world->m_jointsMemory[0];

	dgBodyInfo* const bodyArray = &bodyArrayPtr[cluster->m_bodyStart];
	dgJointInfo* const constraintArray = &constraintArrayPtr[cluster->m_jointStart];
	dgJacobianMatrixElement* const matrixRow = &m_solverMemory.m_jacobianBuffer[cluster->m_rowsStart];

	dgFloat32 invTimestep = (timestep > dgFloat32(0.0f)) ? dgFloat32(1.0f) / timestep : dgFloat32(0.0f);
	dgAssert(bodyArray[0].m_body == world->m_sentinelBody);

	dgVector speedFreeze2(world->m_freezeSpeed2 * dgFloat32(0.1f));
	dgVector freezeOmega2(world->m_freezeOmega2 * dgFloat32(0.1f));

	dgJointAccelerationDecriptor joindDesc;
	joindDesc.m_timeStep = timestep;
	joindDesc.m_invTimeStep = invTimestep;
	joindDesc.m_firstPassCoefFlag = dgFloat32(0.0f);

	dgJointInfo* const jointInfo = &constraintArray[0];
	dgConstraint* const constraint = jointInfo->m_joint;
	joindDesc.m_rowsCount = jointInfo->m_pairCount;
	joindDesc.m_rowMatrix = &matrixRow[jointInfo->m_pairStart];
	constraint->JointAccelerations(&joindDesc);
//	CalculateJointForce_3_13(jointInfo, bodyArray, internalForces, matrixRow);
	CalculateJointForce(jointInfo, bodyArray, internalForces, matrixRow);

	if (timestep != dgFloat32(0.0f)) {
		dgVector timestep4(timestep);
		for (dgInt32 i = 1; i < bodyCount; i++) {
			dgDynamicBody* const body = (dgDynamicBody*)bodyArray[i].m_body;
			dgAssert(body->m_index == i);
			if (body->IsRTTIType(dgBody::m_dynamicBodyRTTI)) {
				const dgJacobian& forceAndTorque = internalForces[i];
				if (!body->m_resting) {
					body->m_externalForce += forceAndTorque.m_linear;
					body->m_externalTorque += forceAndTorque.m_angular;
					body->IntegrateOpenLoopExternalForce(timestep);

				} else {
					const dgVector force(body->m_externalForce + forceAndTorque.m_linear);
					const dgVector torque(body->m_externalTorque + forceAndTorque.m_angular);

					const dgVector velocStep((force.Scale4(body->m_invMass.m_w)) * timestep4);
					const dgVector omegaStep((body->m_invWorldInertiaMatrix.RotateVector(torque)) * timestep4);

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
		
	} else {
		for (dgInt32 i = 1; i < bodyCount; i++) {
			dgDynamicBody* const body = (dgDynamicBody*)bodyArray[i].m_body;
			const dgVector& linearMomentum = internalForces[1].m_linear;
			const dgVector& angularMomentum = internalForces[1].m_angular;
			body->m_veloc += linearMomentum.Scale4(body->m_invMass.m_w);
			body->m_omega += body->m_invWorldInertiaMatrix.RotateVector(angularMomentum);
		}
	}

	dgInt32 hasJointFeeback = 0;
	if (timestep != dgFloat32(0.0f)) {

		dgJointInfo* const jointInfo1 = &constraintArray[0];
		dgConstraint* const constraint1 = jointInfo1->m_joint;

		const dgInt32 first = jointInfo1->m_pairStart;
		const dgInt32 count = jointInfo1->m_pairCount;

		for (dgInt32 j = 0; j < count; j++) {
			dgJacobianMatrixElement* const row = &matrixRow[j + first];
			dgAssert(dgCheckFloat(row->m_force));
			row->m_jointFeebackForce->m_force = row->m_force;
			row->m_jointFeebackForce->m_impact = row->m_maxImpact * timestep;
		}
		hasJointFeeback |= (constraint1->m_updaFeedbackCallback ? 1 : 0);

		if (hasJointFeeback) {
			for (dgInt32 i = 0; i < jointCount; i++) {
				if (constraintArray[i].m_joint->m_updaFeedbackCallback) {
					constraintArray[i].m_joint->m_updaFeedbackCallback(*constraintArray[i].m_joint, timestep, threadID);
				}
			}
		}
	} else {
		for (dgInt32 i = 1; i < bodyCount; i++) {
			dgBody* const body = bodyArray[i].m_body;
			dgAssert(body->IsRTTIType(dgBody::m_dynamicBodyRTTI) || body->IsRTTIType(dgBody::m_kinematicBodyRTTI));
			body->m_accel = dgVector::m_zero;
			body->m_alpha = dgVector::m_zero;
		}
	}
}


void dgWorldDynamicUpdate::CalculateClusterReactionForces(const dgBodyCluster* const cluster, dgInt32 threadID, dgFloat32 timestep) const
{
	dgWorld* const world = (dgWorld*) this;
	const dgInt32 bodyCount = cluster->m_bodyCount;
	const dgInt32 jointCount = cluster->m_jointCount;

	dgJacobian* const internalForces = &m_solverMemory.m_internalForcesBuffer[cluster->m_bodyStart];
	dgBodyInfo* const bodyArrayPtr = (dgBodyInfo*)&world->m_bodiesMemory[0];
	dgJointInfo* const constraintArrayPtr = (dgJointInfo*)&world->m_jointsMemory[0];

	dgBodyInfo* const bodyArray = &bodyArrayPtr[cluster->m_bodyStart];
	dgJointInfo* const constraintArray = &constraintArrayPtr[cluster->m_jointStart];
	dgJacobianMatrixElement* const matrixRow = &m_solverMemory.m_jacobianBuffer[cluster->m_rowsStart];

	const dgInt32 derivativesEvaluationsRK4 = 4;
	dgFloat32 invTimestep = (timestep > dgFloat32(0.0f)) ? dgFloat32(1.0f) / timestep : dgFloat32(0.0f);
	dgFloat32 invStepRK = (dgFloat32(1.0f) / dgFloat32(derivativesEvaluationsRK4));
	dgFloat32 timestepRK = timestep * invStepRK;
	dgFloat32 invTimestepRK = invTimestep * dgFloat32(derivativesEvaluationsRK4);
	dgAssert(bodyArray[0].m_body == world->m_sentinelBody);

	dgVector speedFreeze2(world->m_freezeSpeed2 * dgFloat32(0.1f));
	dgVector freezeOmega2(world->m_freezeOmega2 * dgFloat32(0.1f));

	dgJointAccelerationDecriptor joindDesc;
	joindDesc.m_timeStep = timestepRK;
	joindDesc.m_invTimeStep = invTimestepRK;
	joindDesc.m_firstPassCoefFlag = dgFloat32(0.0f);

	dgInt32 skeletonCount = 0;
	dgInt32 lru = dgAtomicExchangeAndAdd(&dgSkeletonContainer::m_lruMarker, 1);
	dgSkeletonContainer* skeletonArray[DG_MAX_SKELETON_JOINT_COUNT];

	for (dgInt32 i = 1; i < bodyCount; i++) {
		dgDynamicBody* const body = (dgDynamicBody*)bodyArray[i].m_body;
		dgSkeletonContainer* const container = body->GetSkeleton();
		if (container && (container->m_lru != lru)) {
			container->m_lru = lru;
			skeletonArray[skeletonCount] = container;
			container->InitMassMatrix(constraintArray, matrixRow);
			skeletonCount++;
			dgAssert(skeletonCount < dgInt32(sizeof(skeletonArray) / sizeof(skeletonArray[0])));
		}
	}

	const dgInt32 passes = world->m_solverMode;
	for (dgInt32 step = 0; step < derivativesEvaluationsRK4; step++) {

		for (dgInt32 i = 0; i < jointCount; i++) {
			dgJointInfo* const jointInfo = &constraintArray[i];
			dgConstraint* const constraint = jointInfo->m_joint;
			joindDesc.m_rowsCount = jointInfo->m_pairCount;
			joindDesc.m_rowMatrix = &matrixRow[jointInfo->m_pairStart];
			constraint->JointAccelerations(&joindDesc);
		}
		joindDesc.m_firstPassCoefFlag = dgFloat32(1.0f);

		dgFloat32 maxAccNorm = DG_SOLVER_MAX_ERROR * DG_SOLVER_MAX_ERROR;
		dgFloat32 accNorm = maxAccNorm * dgFloat32(2.0f);

		for (dgInt32 i = 0; (i < passes) && (accNorm > maxAccNorm); i++) {
			accNorm = dgFloat32(0.0f);
			for (dgInt32 j = 0; j < jointCount; j++) {
				dgJointInfo* const jointInfo = &constraintArray[j];
				if (!jointInfo->m_joint->IsSkeleton()) {
					//dgFloat32 accel2 = CalculateJointForce_3_13(jointInfo, bodyArray, internalForces, matrixRow);
					dgFloat32 accel2 = CalculateJointForce(jointInfo, bodyArray, internalForces, matrixRow);
					accNorm += accel2;
				}
			}
			for (dgInt32 j = 0; j < skeletonCount; j++) {
				skeletonArray[j]->CalculateJointForce(constraintArray, bodyArray, internalForces, matrixRow);
			}
		}


		if (timestepRK != dgFloat32(0.0f)) {
			dgVector timestep4(timestepRK);
			for (dgInt32 i = 1; i < bodyCount; i++) {
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
		} else {
			for (dgInt32 i = 1; i < bodyCount; i++) {
				dgDynamicBody* const body = (dgDynamicBody*)bodyArray[i].m_body;
				const dgVector& linearMomentum = internalForces[i].m_linear;
				const dgVector& angularMomentum = internalForces[i].m_angular;

				body->m_veloc += linearMomentum.Scale4(body->m_invMass.m_w);
				body->m_omega += body->m_invWorldInertiaMatrix.RotateVector(angularMomentum);
			}
		}
	}

	dgInt32 hasJointFeeback = 0;
	if (timestepRK != dgFloat32(0.0f)) {
		for (dgInt32 i = 0; i < jointCount; i++) {
			dgJointInfo* const jointInfo = &constraintArray[i];
			dgConstraint* const constraint = jointInfo->m_joint;

			const dgInt32 first = jointInfo->m_pairStart;
			const dgInt32 count = jointInfo->m_pairCount;

			for (dgInt32 j = 0; j < count; j++) {
				dgJacobianMatrixElement* const row = &matrixRow[j + first];
				dgAssert(dgCheckFloat(row->m_force));
				row->m_jointFeebackForce->m_force = row->m_force;
				row->m_jointFeebackForce->m_impact = row->m_maxImpact * timestepRK;
			}
			hasJointFeeback |= (constraint->m_updaFeedbackCallback ? 1 : 0);
		}

		const dgVector invTime(invTimestep);
		const dgVector maxAccNorm2(DG_SOLVER_MAX_ERROR * DG_SOLVER_MAX_ERROR);

		for (dgInt32 i = 1; i < bodyCount; i++) {
			dgBody* const body = bodyArray[i].m_body;
			CalculateNetAcceleration(body, invTime, maxAccNorm2);
		}

		if (hasJointFeeback) {
			for (dgInt32 i = 0; i < jointCount; i++) {
				if (constraintArray[i].m_joint->m_updaFeedbackCallback) {
					constraintArray[i].m_joint->m_updaFeedbackCallback(*constraintArray[i].m_joint, timestep, threadID);
				}
			}
		}
	} else {
		for (dgInt32 i = 1; i < bodyCount; i++) {
			dgBody* const body = bodyArray[i].m_body;
			dgAssert(body->IsRTTIType(dgBody::m_dynamicBodyRTTI) || body->IsRTTIType(dgBody::m_kinematicBodyRTTI));
			body->m_accel = dgVector::m_zero;
			body->m_alpha = dgVector::m_zero;
		}
	}
}



