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

#define DG_IMPULSE_COUNT		 (DG_PARALLEL_JOINT_COUNT_CUT_OFF + 32)
#define DG_IMPULSE_CONTACT_SPEED dgFloat32(0.8f)

void dgWorldDynamicUpdate::BuildJacobianMatrix(const dgBodyInfo* const bodyInfoArray, dgJointInfo* const jointInfo, dgJacobian* const internalForces, dgLeftHandSide* const leftHandSide, dgRightHandSide* const rightHandSide, dgFloat32 forceImpulseScale) const
{
	const dgInt32 index = jointInfo->m_pairStart;
	const dgInt32 count = jointInfo->m_pairCount;
	const dgInt32 m0 = jointInfo->m_m0;
	const dgInt32 m1 = jointInfo->m_m1;

	const dgConstraint* const joint = jointInfo->m_joint;
	const dgBody* const body0 = bodyInfoArray[m0].m_body;
	const dgBody* const body1 = bodyInfoArray[m1].m_body;
	const bool isBilateral = joint->IsBilateral();

	const dgVector invMass0(body0->m_invMass[3]);
	const dgMatrix& invInertia0 = body0->m_invWorldInertiaMatrix;
	const dgVector invMass1(body1->m_invMass[3]);
	const dgMatrix& invInertia1 = body1->m_invWorldInertiaMatrix;

	dgVector force0(dgVector::m_zero);
	dgVector torque0(dgVector::m_zero);
	if (body0->IsRTTIType(dgBody::m_dynamicBodyRTTI)) {
		force0 = ((dgDynamicBody*)body0)->m_externalForce;
		torque0 = ((dgDynamicBody*)body0)->m_externalTorque;
	}

	dgVector force1(dgVector::m_zero);
	dgVector torque1(dgVector::m_zero);
	if (body1->IsRTTIType(dgBody::m_dynamicBodyRTTI)) {
		force1 = ((dgDynamicBody*)body1)->m_externalForce;
		torque1 = ((dgDynamicBody*)body1)->m_externalTorque;
	}

	jointInfo->m_preconditioner0 = dgFloat32(1.0f);
	jointInfo->m_preconditioner1 = dgFloat32(1.0f);

	if ((invMass0.GetScalar() > dgFloat32(0.0f)) && (invMass1.GetScalar() > dgFloat32(0.0f))) {
		const dgFloat32 mass0 = body0->GetMass().m_w;
		const dgFloat32 mass1 = body1->GetMass().m_w;
		if (mass0 > (DG_DIAGONAL_PRECONDITIONER * mass1)) {
			jointInfo->m_preconditioner0 = mass0 / (mass1 * DG_DIAGONAL_PRECONDITIONER);
		}
		else if (mass1 > (DG_DIAGONAL_PRECONDITIONER * mass0)) {
			jointInfo->m_preconditioner1 = mass1 / (mass0 * DG_DIAGONAL_PRECONDITIONER);
		}
	}

	if (joint->IsSkeletonLoop() || joint->IsSkeleton() || (body0->GetSkeleton() && body1->GetSkeleton())) {
		jointInfo->m_preconditioner0 = dgFloat32(1.0f);
		jointInfo->m_preconditioner1 = dgFloat32(1.0f);
	}

	dgJacobian forceAcc0;
	dgJacobian forceAcc1;
	const dgVector preconditioner0(jointInfo->m_preconditioner0);
	const dgVector preconditioner1(jointInfo->m_preconditioner1);
	forceAcc0.m_linear = dgVector::m_zero;
	forceAcc0.m_angular = dgVector::m_zero;
	forceAcc1.m_linear = dgVector::m_zero;
	forceAcc1.m_angular = dgVector::m_zero;

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
		//rhs->m_force = 0.0f;
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

	internalForces[m0].m_linear += forceAcc0.m_linear;
	internalForces[m0].m_angular += forceAcc0.m_angular;
	internalForces[m1].m_linear += forceAcc1.m_linear;
	internalForces[m1].m_angular += forceAcc1.m_angular;
}

void dgWorldDynamicUpdate::BuildJacobianMatrix(dgBodyCluster* const cluster, dgInt32 threadID, dgFloat32 timestep) const
{
	D_TRACKTIME();
	dgAssert(cluster->m_bodyCount >= 2);

	dgWorld* const world = (dgWorld*) this;
	const dgInt32 bodyCount = cluster->m_bodyCount;

	dgBodyInfo* const bodyArray = &world->m_bodiesMemory[cluster->m_bodyStart];
	dgJacobian* const internalForces = &m_solverMemory.m_internalForcesBuffer[cluster->m_bodyStart];

	dgAssert(((dgDynamicBody*)bodyArray[0].m_body)->IsRTTIType(dgBody::m_dynamicBodyRTTI));
	dgAssert((((dgDynamicBody*)bodyArray[0].m_body)->m_accel.DotProduct(((dgDynamicBody*)bodyArray[0].m_body)->m_accel)).GetScalar() == dgFloat32(0.0f));
	dgAssert((((dgDynamicBody*)bodyArray[0].m_body)->m_alpha.DotProduct(((dgDynamicBody*)bodyArray[0].m_body)->m_alpha)).GetScalar() == dgFloat32(0.0f));
	dgAssert((((dgDynamicBody*)bodyArray[0].m_body)->m_externalForce.DotProduct(((dgDynamicBody*)bodyArray[0].m_body)->m_externalForce)).GetScalar() == dgFloat32(0.0f));
	dgAssert((((dgDynamicBody*)bodyArray[0].m_body)->m_externalTorque.DotProduct(((dgDynamicBody*)bodyArray[0].m_body)->m_externalTorque)).GetScalar() == dgFloat32(0.0f));

	internalForces[0].m_linear = dgVector::m_zero;
	internalForces[0].m_angular = dgVector::m_zero;

	if (timestep != dgFloat32(0.0f)) {
		for (dgInt32 i = 1; i < bodyCount; i++) {
			dgBody* const body = (dgDynamicBody*)bodyArray[i].m_body;
			dgAssert(body->IsRTTIType(dgBody::m_dynamicBodyRTTI) || body->IsRTTIType(dgBody::m_kinematicBodyRTTI));
			if (!body->m_equilibrium) {
				dgAssert(body->m_invMass.m_w > dgFloat32(0.0f));
				body->AddDampingAcceleration(timestep);
				body->CalcInvInertiaMatrix();
			}

			// re use these variables for temp storage 
			body->m_accel = body->m_veloc;
			body->m_alpha = body->m_omega;

			internalForces[i].m_linear = dgVector::m_zero;
			internalForces[i].m_angular = dgVector::m_zero;
		}
	} else {
		for (dgInt32 i = 1; i < bodyCount; i++) {
			dgBody* const body = bodyArray[i].m_body;
			dgAssert(body->IsRTTIType(dgBody::m_dynamicBodyRTTI) || body->IsRTTIType(dgBody::m_kinematicBodyRTTI));
			if (!body->m_equilibrium) {
				dgAssert(body->m_invMass.m_w > dgFloat32(0.0f));
				body->CalcInvInertiaMatrix();
			}

			// re use these variables for temp storage 
			body->m_accel = body->m_veloc;
			body->m_alpha = body->m_omega;

			internalForces[i].m_linear = dgVector::m_zero;
			internalForces[i].m_angular = dgVector::m_zero;
		}
	}

	dgContraintDescritor constraintParams;

	constraintParams.m_world = world;
	constraintParams.m_threadIndex = threadID;
	constraintParams.m_timestep = timestep;
	constraintParams.m_invTimestep = (timestep > dgFloat32(1.0e-5f)) ? dgFloat32(1.0f / timestep) : dgFloat32(0.0f);
	const dgFloat32 forceOrImpulseScale = (timestep > dgFloat32(0.0f)) ? dgFloat32(1.0f) : dgFloat32(0.0f);

	dgJointInfo* const constraintArrayPtr = &world->m_jointsMemory[0];
	dgJointInfo* const constraintArray = &constraintArrayPtr[cluster->m_jointStart];
	dgLeftHandSide* const leftHandSide = &m_solverMemory.m_leftHandSizeBuffer[cluster->m_rowStart];
	dgRightHandSide* const rightHandSide = &m_solverMemory.m_righHandSizeBuffer[cluster->m_rowStart];

	dgInt32 rowCount = 0;
	const dgInt32 jointCount = cluster->m_jointCount;
	
	dgUnsigned8 impactBuffer[256 * (sizeof(dgContact*) + sizeof(dgFloat32))];
	dgDownHeap<dgContact*, dgFloat32> impactJoints(impactBuffer, sizeof(impactBuffer));

	for (dgInt32 i = 0; i < jointCount; i++) {
		dgJointInfo* const jointInfo = &constraintArray[i];
		dgConstraint* const constraint = jointInfo->m_joint;

		dgAssert(dgInt32(constraint->m_index) == i);
		dgAssert(jointInfo->m_m0 < cluster->m_bodyCount);
		dgAssert(jointInfo->m_m1 < cluster->m_bodyCount);
		//dgAssert (constraint->m_index == dgUnsigned32(i));

		rowCount = GetJacobianDerivatives(constraintParams, jointInfo, constraint, leftHandSide, rightHandSide, rowCount);
		dgAssert(rowCount <= cluster->m_rowCount);

		dgAssert(jointInfo->m_m0 >= 0);
		dgAssert(jointInfo->m_m0 < bodyCount);
		dgAssert(jointInfo->m_m1 >= 0);
		dgAssert(jointInfo->m_m1 < bodyCount);
		BuildJacobianMatrix(bodyArray, jointInfo, internalForces, leftHandSide, rightHandSide, forceOrImpulseScale);

		dgFloat32 impulseSpeed = constraint->GetImpulseContactSpeed();
		if (impulseSpeed > DG_IMPULSE_CONTACT_SPEED) {
			dgContact* contact = (dgContact*)constraint;
			impactJoints.Push(contact, impulseSpeed);
		}
	}

	if (impactJoints.GetCount()) {
		//ResolveImpulse(constraintArrayPtr, leftHandSide, rightHandSide, impactJoints);
	}
}

void dgWorldDynamicUpdate::ResolveImpulse(const dgJointInfo* const constraintArray, const dgLeftHandSide* const leftHandSide, dgRightHandSide* const rightHandSide, dgDownHeap<dgContact*, dgFloat32>& impactJoints) const
{
	dgJacobian impulse[DG_IMPULSE_COUNT];
	dgBodyInfo bodyArray[DG_IMPULSE_COUNT];
	dgJointImpulseInfo contactArray[DG_IMPULSE_COUNT];
	dgFloat32 relVeloc[DG_IMPULSE_COUNT];
	dgFloat32 outImpulse[DG_IMPULSE_COUNT];

	while (impactJoints.GetCount()) {
		dgContact* const contact = impactJoints[0];
		impactJoints.Pop();
		if (contact->GetImpulseContactSpeed() > DG_IMPULSE_CONTACT_SPEED) {
			dgBody* body = contact->GetBody0();
			dgBody* const otherBody = contact->GetBody1();
			bool test = otherBody->GetInvMass().m_w >= dgFloat32(0.0f);
			test = test && (otherBody->m_veloc.DotProduct(otherBody->m_veloc).GetScalar() > body->m_veloc.DotProduct(body->m_veloc).GetScalar());
			if (test) {
				body = otherBody;
			}

			dgInt32 bodyCount = 1;
			dgInt32 rowsCount = 0;
			dgInt32 contactCount = 0;
		
			bodyArray[0].m_body = body;
			impulse[0].m_linear = dgVector::m_zero;
			impulse[0].m_angular = dgVector::m_zero;
			for (dgBodyMasterListRow::dgListNode* jointNode = body->m_masterNode->GetInfo().GetFirst(); jointNode; jointNode = jointNode->GetNext()) {
				dgBodyMasterListCell* const cell = &jointNode->GetInfo();
				dgConstraint* const constraint = cell->m_joint;

				if (constraint->IsActive() && (constraint->GetId() == dgConstraint::m_contactConstraint)) {
					dgContact* const contactJoint = (dgContact*)constraint;
					if (contactJoint->GetImpulseContactSpeed() > DG_IMPULSE_CONTACT_SPEED) {
						dgBody* const body0 = contactJoint->GetBody0();
						dgBody* const body1 = contactJoint->GetBody1();
						dgAssert ((body0 == body) || (body1 == body));

						const dgJointInfo& jointInfo = constraintArray[contactJoint->m_index];
						dgJointImpulseInfo& contactInfo = contactArray[contactCount];
						contactInfo.m_joint = contactJoint;
						dgAssert(jointInfo.m_joint == contactJoint);

						contactInfo.m_pairStart = jointInfo.m_pairStart;
						contactInfo.m_pairCount = jointInfo.m_pairCount;
						contactInfo.m_rhsStart = rowsCount;
						
						bodyArray[bodyCount].m_body = (body1 != body) ? body1 : body0;
						contactArray[contactCount].m_m0 = (body1 != body) ? 0 : bodyCount;
						contactArray[contactCount].m_m1 = (body1 != body) ? bodyCount : 0;

						impulse[bodyCount].m_linear = dgVector::m_zero;
						impulse[bodyCount].m_angular = dgVector::m_zero;
		
						dgAssert (bodyArray[contactArray[contactCount].m_m0].m_body == contactJoint->GetBody0());
						dgAssert (bodyArray[contactArray[contactCount].m_m1].m_body == contactJoint->GetBody1());

						bodyCount++;
						contactCount++;
						rowsCount += contactInfo.m_pairCount;
						dgAssert(bodyCount < sizeof(bodyArray) / sizeof(bodyArray[0]));
					}
				}
			}

			memset(outImpulse, 0, rowsCount * sizeof(dgFloat32));
			for (dgInt32 i = 0; i < contactCount; i++) {
				dgJointImpulseInfo* const jointInfo = &contactArray[i];
				const dgInt32 index = jointInfo->m_pairStart;
				CalculateImpulseVeloc(jointInfo, &leftHandSide[index], &rightHandSide[index], &relVeloc[jointInfo->m_rhsStart]);
			}

			dgFloat32 impulseNorm = dgFloat32 (10.0f);
			dgFloat32 maxImpulseNorm = DG_IMPULSE_CONTACT_SPEED;
			for (dgInt32 j = 0; (j < 4) && (impulseNorm > maxImpulseNorm); j++) {
				impulseNorm = dgFloat32 (0.0f);
				for (dgInt32 i = 0; i < contactCount; i++) {
					dgJointImpulseInfo* const jointInfo = &contactArray[i];
					const dgInt32 index = jointInfo->m_pairStart;
					dgFloat32 impulse2 = CalculateJointImpulse(jointInfo, bodyArray, impulse, &leftHandSide[index], &rightHandSide[index], &relVeloc[jointInfo->m_rhsStart], &outImpulse[i]);
					impulseNorm += impulse2;
				}
			}

			for (dgInt32 i = 0; i < bodyCount; i++) {
				dgDynamicBody* const impulseBody = (dgDynamicBody*)bodyArray[i].m_body;
				const dgVector velocStep(impulse[i].m_linear.Scale(impulseBody->m_invMass.m_w));
				const dgVector omegaStep(impulseBody->m_invWorldInertiaMatrix.RotateVector(impulse[i].m_angular));
				impulseBody->m_veloc += velocStep;
				impulseBody->m_omega += omegaStep;
			}
		}
	}
}

dgFloat32 dgWorldDynamicUpdate::CalculateJointImpulse(const dgJointImpulseInfo* const jointInfo, const dgBodyInfo* const bodyArray, dgJacobian* const internalForces, const dgLeftHandSide* const matrixRow, const dgRightHandSide* const rightHandSide, dgFloat32* const relVel, dgFloat32* const out) const
{
	dgVector accNorm(dgVector::m_zero);

	dgFloat32 normalForce[DG_CONSTRAINT_MAX_ROWS + 4];

	const dgInt32 m0 = jointInfo->m_m0;
	const dgInt32 m1 = jointInfo->m_m1;
	dgInt32 rowsCount = jointInfo->m_pairCount;

	dgVector linearM0(internalForces[m0].m_linear);
	dgVector angularM0(internalForces[m0].m_angular);
	dgVector linearM1(internalForces[m1].m_linear);
	dgVector angularM1(internalForces[m1].m_angular);

	normalForce[0] = dgFloat32(1.0f);
//	const dgInt32 rowStart = jointInfo->m_pairStart;

	for (dgInt32 j = 0; j < rowsCount; j++) {
		const dgLeftHandSide* const row = &matrixRow[j];
		const dgRightHandSide* const rhs____ = &rightHandSide[j];
		dgVector a(row->m_JMinv.m_jacobianM0.m_linear * linearM0);
		a = a.MulAdd(row->m_JMinv.m_jacobianM0.m_angular, angularM0);
		a = a.MulAdd(row->m_JMinv.m_jacobianM1.m_linear, linearM1);
		a = a.MulAdd(row->m_JMinv.m_jacobianM1.m_angular, angularM1);
		//a = dgVector(rhs->m_coordenateAccel - rhs->m_force * rhs->m_diagDamp) - a.AddHorizontal();
		a = dgVector(relVel[j] - out[j] * rhs____->m_diagDamp) - a.AddHorizontal();

		//dgVector f(rhs->m_force + rhs->m_invJinvMJt * a.GetScalar());
		dgVector f(out[j] + rhs____->m_invJinvMJt * a.GetScalar());
		dgAssert(rhs____->m_normalForceIndex >= -1);
		dgAssert(rhs____->m_normalForceIndex <= rowsCount);

		dgInt32 frictionIndex = rhs____->m_normalForceIndex + 1;
		dgFloat32 frictionNormal = normalForce[frictionIndex];
		dgVector lowerFrictionForce(frictionNormal * rhs____->m_lowerBoundFrictionCoefficent);
		dgVector upperFrictionForce(frictionNormal * rhs____->m_upperBoundFrictionCoefficent);

		a = a & (f < upperFrictionForce) & (f > lowerFrictionForce);
		f = f.GetMax(lowerFrictionForce).GetMin(upperFrictionForce);
		accNorm = accNorm.MulAdd(a, a);

		//dgVector deltaForce(f - dgVector(rhs->m_force));
		dgVector deltaForce(f - dgVector(out[j]));

		//rhs->m_force = f.GetScalar();
		out[j] = f.GetScalar();
		normalForce[j + 1] = f.GetScalar();

		linearM0 = linearM0.MulAdd(row->m_Jt.m_jacobianM0.m_linear, deltaForce);
		angularM0 = angularM0.MulAdd(row->m_Jt.m_jacobianM0.m_angular, deltaForce);
		linearM1 = linearM1.MulAdd(row->m_Jt.m_jacobianM1.m_linear, deltaForce);
		angularM1 = angularM1.MulAdd(row->m_Jt.m_jacobianM1.m_angular, deltaForce);
	}


	dgVector maxAccel(accNorm);
	const dgFloat32 tol = dgFloat32(0.5f);
	const dgFloat32 tol2 = tol * tol;
	for (dgInt32 i = 0; (i < 4) && (maxAccel.GetScalar() > tol2); i++) {
		maxAccel = dgVector::m_zero;
		for (dgInt32 j = 0; j < rowsCount; j++) {
			const dgLeftHandSide* const row = &matrixRow[j];
			const dgRightHandSide* const rhs____ = &rightHandSide[j];
			dgVector a(row->m_JMinv.m_jacobianM0.m_linear * linearM0);
			a = a.MulAdd(row->m_JMinv.m_jacobianM0.m_angular, angularM0);
			a = a.MulAdd(row->m_JMinv.m_jacobianM1.m_linear, linearM1);
			a = a.MulAdd(row->m_JMinv.m_jacobianM1.m_angular, angularM1);
			//a = dgVector(rhs->m_coordenateAccel - rhs->m_force * rhs->m_diagDamp) - a.AddHorizontal();
			a = dgVector(relVel[j] - out[j] * rhs____->m_diagDamp) - a.AddHorizontal();

			//dgVector f(rhs->m_force + rhs->m_invJinvMJt * a.GetScalar());
			dgVector f(out[j] + rhs____->m_invJinvMJt * a.GetScalar());
			dgAssert(rhs____->m_normalForceIndex >= -1);
			dgAssert(rhs____->m_normalForceIndex <= rowsCount);

			dgInt32 frictionIndex = rhs____->m_normalForceIndex + 1;
			dgFloat32 frictionNormal = normalForce[frictionIndex];
			dgVector lowerFrictionForce(frictionNormal * rhs____->m_lowerBoundFrictionCoefficent);
			dgVector upperFrictionForce(frictionNormal * rhs____->m_upperBoundFrictionCoefficent);

			a = a & (f < upperFrictionForce) & (f > lowerFrictionForce);
			f = f.GetMax(lowerFrictionForce).GetMin(upperFrictionForce);
			maxAccel = maxAccel.MulAdd(a, a);

			//dgVector deltaForce(f - dgVector(rhs->m_force));
			dgVector deltaForce(f - dgVector(out[j]));

			//rhs->m_force = f.GetScalar();
			out[j] = f.GetScalar();
			normalForce[j + 1] = f.GetScalar();

			linearM0 = linearM0.MulAdd(row->m_Jt.m_jacobianM0.m_linear, deltaForce);
			angularM0 = angularM0.MulAdd(row->m_Jt.m_jacobianM0.m_angular, deltaForce);
			linearM1 = linearM1.MulAdd(row->m_Jt.m_jacobianM1.m_linear, deltaForce);
			angularM1 = angularM1.MulAdd(row->m_Jt.m_jacobianM1.m_angular, deltaForce);
		}
	}


	internalForces[m0].m_linear = linearM0;
	internalForces[m0].m_angular = angularM0;
	internalForces[m1].m_linear = linearM1;
	internalForces[m1].m_angular = angularM1;

	return accNorm.GetScalar();
}


void dgWorldDynamicUpdate::CalculateImpulseVeloc(dgJointImpulseInfo* const jointInfo, const dgLeftHandSide* const leftHandSide, const dgRightHandSide* const rightHandSide, dgFloat32* const contactVeloc) const
{
	dgContact* const contact = (dgContact*)jointInfo->m_joint;
	const dgVector& bodyVeloc0 = contact->GetBody0()->m_veloc;
	const dgVector& bodyOmega0 = contact->GetBody0()->m_omega;
	const dgVector& bodyVeloc1 = contact->GetBody1()->m_veloc;
	const dgVector& bodyOmega1 = contact->GetBody1()->m_omega;

	const dgInt32 count = jointInfo->m_pairCount;
	for (dgInt32 k = 0; k < count; k++) {
		//dgAssert(rightHandSide[k].m_restitution >= dgFloat32(0.0f));
		//if (rightHandSide[k].m_restitution >= dgFloat32(0.0f)) 
		{
			const dgRightHandSide* const rhs = &rightHandSide[k];
			const dgLeftHandSide* const row = &leftHandSide[k];
			const dgJacobian &jacobian0 = row->m_Jt.m_jacobianM0;
			const dgJacobian &jacobian1 = row->m_Jt.m_jacobianM1;

			dgVector relVeloc(jacobian0.m_linear * bodyVeloc0 + jacobian0.m_angular * bodyOmega0 + jacobian1.m_linear * bodyVeloc1 + jacobian1.m_angular * bodyOmega1);
			dgFloat32 vRel = relVeloc.AddHorizontal().GetScalar();
			if (rhs->m_normalForceIndex == DG_INDEPENDENT_ROW) {
				dgAssert(rhs->m_restitution >= 0.0f);
				dgAssert(rhs->m_restitution <= 2.0f);
				dgFloat32 restitution = (vRel <= dgFloat32(0.0f)) ? (dgFloat32(1.0f) + rhs->m_restitution) : dgFloat32(1.0f);
				vRel = vRel * restitution;
			}
			contactVeloc[k] = - vRel;
		}
	}
}

dgInt32 dgWorldDynamicUpdate::SortClusters(const dgBodyCluster* const cluster, dgFloat32 timestep, dgInt32 threadID) const
{
	DG_TRACKTIME();
	dgWorld* const world = (dgWorld*) this;
	dgBodyInfo* const bodyArray = &world->m_bodiesMemory[cluster->m_bodyStart];
	dgJointInfo* const constraintArray = &world->m_jointsMemory[cluster->m_jointStart];

	const dgInt32 bodyCount = cluster->m_bodyCount;
	const dgInt32 jointCount = cluster->m_jointCount;

	dgJointInfo* const tmpInfoList = dgAlloca(dgJointInfo, cluster->m_jointCount);
	dgJointInfo** queueBuffer = dgAlloca(dgJointInfo*, cluster->m_jointCount * 2 + 1024 * 8);
	dgBodyJacobianPair* const bodyJoint = dgAlloca(dgBodyJacobianPair, cluster->m_jointCount * 2);
	dgInt32* const bodyJointList = dgAlloca(dgInt32, bodyCount + 1);

	dgQueue<dgJointInfo*> queue(queueBuffer, cluster->m_jointCount * 2 + 1024 * 8);
	dgFloat32 heaviestMass = dgFloat32(1.0e20f);
	dgInt32 infoIndex = 0;
	dgInt32 activeJoints = 0;
	dgJointInfo* heaviestBody = NULL;

	for (dgInt32 i = 0; i < jointCount; i++) {
		dgJointInfo& jointInfo = constraintArray[i];
		tmpInfoList[i] = jointInfo;
		tmpInfoList[i].m_preconditioner0 = dgFloat32(0.0f);

		jointInfo.m_joint->m_graphTagged = 0;
		const dgInt32 m0 = jointInfo.m_m0;
		const dgInt32 m1 = jointInfo.m_m1;
		dgBody* const body0 = bodyArray[m0].m_body;
		dgBody* const body1 = bodyArray[m1].m_body;

		const dgFloat32 invMass0 = body0->GetInvMass().m_w;
		const dgFloat32 invMass1 = body1->GetInvMass().m_w;

		if ((invMass0 == dgFloat32(0.0f)) || (invMass1 == dgFloat32(0.0f))) {
			queue.Insert(&tmpInfoList[i]);
			tmpInfoList[i].m_preconditioner0 = dgFloat32(1.0f);
		} else if (invMass0 && (heaviestMass > invMass0)) {
			heaviestMass = invMass0;
			heaviestBody = &tmpInfoList[i];
		} else if (invMass1 && (heaviestMass > invMass1)) {
			heaviestMass = invMass1;
			heaviestBody = &tmpInfoList[i];
		}

		bodyJoint[i * 2 + 0].m_bodyIndex = m0;
		bodyJoint[i * 2 + 0].m_JointIndex = i;
		bodyJoint[i * 2 + 1].m_bodyIndex = m1;
		bodyJoint[i * 2 + 1].m_JointIndex = i;
	}

	if (queue.IsEmpty()) {
		dgAssert(heaviestBody);
		queue.Insert(heaviestBody);
		heaviestBody->m_preconditioner0 = dgFloat32(1.0f);
	}

	dgSort(bodyJoint, jointCount * 2, CompareBodyJacobianPair);
	memset(bodyJointList, 0, sizeof(dgInt32) * (cluster->m_bodyCount + 1));
	for (dgInt32 i = 0; i < jointCount * 2; i++) {
		dgInt32 index = bodyJoint[i].m_bodyIndex;
		bodyJointList[index] ++;
	}

	dgInt32 startIndex = 0;
	for (dgInt32 i = 0; i <= bodyCount; i++) {
		dgInt32 count = bodyJointList[i];
		bodyJointList[i] = startIndex;
		startIndex += count;
	}

	while (!queue.IsEmpty()) {
		dgInt32 count = queue.m_firstIndex - queue.m_lastIndex;
		if (count < 0) {
			count += queue.m_mod;
		}

		dgInt32 index = queue.m_lastIndex;
		queue.Reset();

		for (dgInt32 i = 0; i < count; i++) {
			dgJointInfo* const jointInfo = queue.m_pool[index];
			dgConstraint* const constraint = jointInfo->m_joint;
			if (!constraint->m_graphTagged) {
				constraint->m_index = infoIndex;
				constraintArray[infoIndex] = *jointInfo;
				constraint->m_graphTagged = 1;
				infoIndex++;
				dgAssert(infoIndex <= cluster->m_jointCount);

				const dgInt32 m0 = jointInfo->m_m0;
				const dgInt32 m1 = jointInfo->m_m1;
				const dgBody* const body0 = bodyArray[m0].m_body;
				const dgBody* const body1 = bodyArray[m1].m_body;

				activeJoints += !(body0->m_resting & body1->m_resting);

				if (body0->GetInvMass().m_w > dgFloat32(0.0f)) {
					const dgInt32 endJoint = bodyJointList[m0 + 1];
					for (dgInt32 j = bodyJointList[m0]; j < endJoint; j++) {
						dgJointInfo* const info = &tmpInfoList[bodyJoint[j].m_JointIndex];
						dgConstraint* const nextConstraint = info->m_joint;
						if (!nextConstraint->m_graphTagged) {
							if (!info->m_preconditioner0) {
								queue.Insert(info);
								info->m_preconditioner0 = dgFloat32(1.0f);
							}
						}
					}
				}

				if (body1->GetInvMass().m_w > dgFloat32(0.0f)) {
					const dgInt32 endJoint = bodyJointList[m1 + 1];
					for (dgInt32 j = bodyJointList[m1]; j < endJoint; j++) {
						dgJointInfo* const info = &tmpInfoList[bodyJoint[j].m_JointIndex];
						dgConstraint* const nextConstraint = info->m_joint;
						if (!nextConstraint->m_graphTagged) {
							if (!info->m_preconditioner0) {
								queue.Insert(info);
								info->m_preconditioner0 = dgFloat32(1.0f);
							}
						}
					}
				}

				if (infoIndex == cluster->m_jointCount) {
					queue.Reset();
					break;
				}
			}
			index++;
			if (index >= queue.m_mod) {
				index = 0;
			}
		}
	}

	dgAssert(infoIndex == cluster->m_jointCount);
	return activeJoints;
}

void dgWorldDynamicUpdate::ResolveClusterForces(dgBodyCluster* const cluster, dgInt32 threadID, dgFloat32 timestep) const
{
	dgInt32 activeJoint = cluster->m_jointCount;
	if (activeJoint > 0) {
		activeJoint = SortClusters(cluster, timestep, threadID);
	}

	dgWorld* const world = (dgWorld*) this;
	dgJointInfo* const constraintArray = &world->m_jointsMemory[cluster->m_jointStart];

	if (!cluster->m_isContinueCollision) {
		if (activeJoint >= 1) {
			BuildJacobianMatrix(cluster, threadID, timestep);
			CalculateClusterReactionForces(cluster, threadID, timestep);
		} else if (cluster->m_jointCount == 0) {
			IntegrateExternalForce(cluster, timestep, threadID);
		} else {
			dgAssert((activeJoint == 0) && cluster->m_jointCount);
			dgBodyInfo* const bodyArray = &world->m_bodiesMemory[cluster->m_bodyStart];
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

		// see if the cluster goes to sleep
		bool isAutoSleep = true;
		bool stackSleeping = true;
		dgInt32 sleepCounter = 10000;

		const dgInt32 bodyCount = cluster->m_bodyCount;
		dgBodyInfo* const bodyArray = &world->m_bodiesMemory[cluster->m_bodyStart];

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

				const dgFloat32 accel2 = body->m_accel.DotProduct(body->m_accel).GetScalar();
				const dgFloat32 alpha2 = body->m_alpha.DotProduct(body->m_alpha).GetScalar();
				const dgFloat32 speed2 = body->m_veloc.DotProduct(body->m_veloc).GetScalar();
				const dgFloat32 omega2 = body->m_omega.DotProduct(body->m_omega).GetScalar();

				maxAccel = dgMax (maxAccel, accel2);
				maxAlpha = dgMax (maxAlpha, alpha2);
				maxSpeed = dgMax (maxSpeed, speed2);
				maxOmega = dgMax (maxOmega, omega2);

				bool equilibrium = (accel2 < accelFreeze) && (alpha2 < accelFreeze) && (speed2 < speedFreeze) && (omega2 < speedFreeze);
				if (equilibrium) {
					dgVector veloc (body->m_veloc * forceDampVect);
					dgVector omega = body->m_omega * forceDampVect;
					body->m_veloc = (veloc.DotProduct(veloc) > m_velocTol) & veloc;
					body->m_omega = (omega.DotProduct(omega) > m_velocTol) & omega;

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
				// the cluster went to sleep mode, 
				for (dgInt32 i = 1; i < bodyCount; i ++) {
					dgBody* const body = bodyArray[i].m_body;
					dgAssert (body->IsRTTIType (dgBody::m_dynamicBodyRTTI) || body->IsRTTIType (dgBody::m_kinematicBodyRTTI));
					body->m_accel = dgVector::m_zero;
					body->m_alpha = dgVector::m_zero;
					body->m_veloc = dgVector::m_zero;
					body->m_omega = dgVector::m_zero;
				}
			} else {
				// cluster is not sleeping but may be resting with small residual velocity for a long time
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
						// force cluster to sleep
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
			// cluster is not sleeping, need to integrate cluster velocity
			const dgUnsigned32 lru = world->GetBroadPhase()->m_lru;
			const dgInt32 jointCount = cluster->m_jointCount;

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
										dgVector vel0 (veloc0 + omega0.CrossProduct(contactMaterial->m_point - com0));
										dgVector vel1 (veloc1 + omega1.CrossProduct(contactMaterial->m_point - com1));
										dgVector vRel (vel0 - vel1);
										dgAssert (contactMaterial->m_normal.m_w == dgFloat32 (0.0f));
										dgFloat32 speed = vRel.DotProduct(contactMaterial->m_normal).m_w;
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
	dgJointInfo* const constraintArrayPtr = &world->m_jointsMemory[0];
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
					//processContacts = material->m_aabbOverlap (*material, *contact->GetBody0(), *contact->GetBody1(), threadID);
					processContacts = material->m_aabbOverlap(*contact, timestep, threadID);
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
	D_TRACKTIME();
	dgWorld* const world = (dgWorld*) this;
	dgBodyInfo* const bodyArray = &world->m_bodiesMemory[cluster->m_bodyStart];

	dgAssert (timestep > dgFloat32 (0.0f));
	const dgInt32 bodyCount = cluster->m_bodyCount;
	for (dgInt32 i = 1; i < bodyCount; i ++) {
		dgDynamicBody* const body = (dgDynamicBody*) bodyArray[i].m_body;
		body->UpdateGyroData();
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
	dgVector accelTest((accel.DotProduct(accel) > maxAccNorm2) | (alpha.DotProduct(alpha) > maxAccNorm2));
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

dgFloat32 dgWorldDynamicUpdate::CalculateJointForce_3_13(const dgJointInfo* const jointInfo, const dgBodyInfo* const bodyArray, dgJacobian* const internalForces, const dgLeftHandSide* const matrixRow, dgRightHandSide* const rightHandSide) const
{
	dgVector accNorm(dgVector::m_zero);
	const dgInt32 m0 = jointInfo->m_m0;
	const dgInt32 m1 = jointInfo->m_m1;
	const dgBody* const body0 = bodyArray[m0].m_body;
	const dgBody* const body1 = bodyArray[m1].m_body;

	if (!(body0->m_resting & body1->m_resting)) {
		dgFloat32 normalForce[DG_CONSTRAINT_MAX_ROWS + 1];
		dgVector linearM0(internalForces[m0].m_linear);
		dgVector angularM0(internalForces[m0].m_angular);
		dgVector linearM1(internalForces[m1].m_linear);
		dgVector angularM1(internalForces[m1].m_angular);

		const dgVector preconditioner0(jointInfo->m_preconditioner0);
		const dgVector preconditioner1(jointInfo->m_preconditioner1);

		const dgInt32 index = jointInfo->m_pairStart;
		const dgInt32 rowsCount = jointInfo->m_pairCount;

		normalForce[0] = dgFloat32 (1.0f);
		dgVector firstPass(dgVector::m_one);
		dgVector maxAccel(dgVector::m_three);
		const dgFloat32 restAcceleration = DG_SOLVER_MAX_ERROR * DG_SOLVER_MAX_ERROR * dgFloat32(4.0f);
		for (dgInt32 i = 0; (i < 4) && (maxAccel.GetScalar() > restAcceleration); i++) {
			maxAccel = dgFloat32(0.0f);
			for (dgInt32 j = 0; j < rowsCount; j++) {
				dgRightHandSide* const rhs = &rightHandSide[index + j];
				const dgLeftHandSide* const row = &matrixRow[index + j];

				dgAssert(row->m_Jt.m_jacobianM0.m_linear.m_w == dgFloat32(0.0f));
				dgAssert(row->m_Jt.m_jacobianM0.m_angular.m_w == dgFloat32(0.0f));
				dgAssert(row->m_Jt.m_jacobianM1.m_linear.m_w == dgFloat32(0.0f));
				dgAssert(row->m_Jt.m_jacobianM1.m_angular.m_w == dgFloat32(0.0f));

				dgVector diag(row->m_JMinv.m_jacobianM0.m_linear * linearM0 + row->m_JMinv.m_jacobianM0.m_angular * angularM0 +
							  row->m_JMinv.m_jacobianM1.m_linear * linearM1 + row->m_JMinv.m_jacobianM1.m_angular * angularM1);

				dgVector accel(rhs->m_coordenateAccel - rhs->m_force * rhs->m_diagDamp - (diag.AddHorizontal()).GetScalar());
				dgVector force(rhs->m_force + rhs->m_invJinvMJt * accel.GetScalar());

				dgAssert (rhs->m_normalForceIndex >= -1);
				dgAssert (rhs->m_normalForceIndex <= rowsCount);
				dgInt32 frictionIndex = rhs->m_normalForceIndex + 1;

				dgFloat32 frictionNormal = normalForce[frictionIndex];
				dgVector lowerFrictionForce(frictionNormal * rhs->m_lowerBoundFrictionCoefficent);
				dgVector upperFrictionForce(frictionNormal * rhs->m_upperBoundFrictionCoefficent);

				accel = accel & (force < upperFrictionForce) & (force > lowerFrictionForce);
				force = force.GetMax(lowerFrictionForce).GetMin(upperFrictionForce);
				maxAccel = maxAccel.GetMax(accel.Abs());

				dgAssert(maxAccel.m_x >= dgAbs(accel.m_x));

				accNorm = accNorm.GetMax(maxAccel * firstPass);

				dgVector deltaForce(force - dgVector(rhs->m_force));
				rhs->m_force = force.GetScalar();
				normalForce[j + 1] = force.GetScalar();

				dgVector deltaforce0(preconditioner0 * deltaForce);
				dgVector deltaforce1(preconditioner1 * deltaForce);

				linearM0 += row->m_Jt.m_jacobianM0.m_linear * deltaforce0;
				angularM0 += row->m_Jt.m_jacobianM0.m_angular * deltaforce0;
				linearM1 += row->m_Jt.m_jacobianM1.m_linear * deltaforce1;
				angularM1 += row->m_Jt.m_jacobianM1.m_angular * deltaforce1;
			}
			firstPass = dgVector::m_zero;
		}

		for (dgInt32 i = 0; i < rowsCount; i++) {
			//dgLeftHandSide* const row = &matrixRow[index + i];
			dgRightHandSide* const rhs = &rightHandSide[index + i];
			rhs->m_maxImpact = dgMax(dgAbs(rhs->m_force), rhs->m_maxImpact);
		}

		internalForces[m0].m_linear = linearM0;
		internalForces[m0].m_angular = angularM0;
		internalForces[m1].m_linear = linearM1;
		internalForces[m1].m_angular = angularM1;
	}

	return accNorm.GetScalar() * accNorm.GetScalar();
}

dgFloat32 dgWorldDynamicUpdate::CalculateJointForce(const dgJointInfo* const jointInfo, const dgBodyInfo* const bodyArray, dgJacobian* const internalForces, const dgLeftHandSide* const matrixRow, dgRightHandSide* const rightHandSide) const
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

		normalForce[0] = dgFloat32(1.0f);
		const dgInt32 rowStart = jointInfo->m_pairStart;
		for (dgInt32 j = 0; j < rowsCount; j++) {
			dgRightHandSide* const rhs = &rightHandSide[rowStart + j];
			const dgLeftHandSide* const row = &matrixRow[rowStart + j];
			dgVector a (row->m_JMinv.m_jacobianM0.m_linear * linearM0);
			a = a.MulAdd(row->m_JMinv.m_jacobianM0.m_angular, angularM0);
			a = a.MulAdd(row->m_JMinv.m_jacobianM1.m_linear, linearM1);
			a = a.MulAdd(row->m_JMinv.m_jacobianM1.m_angular, angularM1);
			a = dgVector(rhs->m_coordenateAccel - rhs->m_force * rhs->m_diagDamp) - a.AddHorizontal();

			dgVector f(rhs->m_force + rhs->m_invJinvMJt * a.GetScalar());
			dgAssert(rhs->m_normalForceIndex >= -1);
			dgAssert(rhs->m_normalForceIndex <= rowsCount);
			dgInt32 frictionIndex = rhs->m_normalForceIndex + 1;

			dgFloat32 frictionNormal = normalForce[frictionIndex];
			dgVector lowerFrictionForce(frictionNormal * rhs->m_lowerBoundFrictionCoefficent);
			dgVector upperFrictionForce(frictionNormal * rhs->m_upperBoundFrictionCoefficent);

			a = a & (f < upperFrictionForce) & (f > lowerFrictionForce);
			f = f.GetMax(lowerFrictionForce).GetMin(upperFrictionForce);
			accNorm = accNorm.MulAdd(a, a);

			dgVector deltaForce(f - dgVector(rhs->m_force));

			rhs->m_force = f.GetScalar();
			normalForce[j + 1] = f.GetScalar();

			dgVector deltaforce0(preconditioner0 * deltaForce);
			dgVector deltaforce1(preconditioner1 * deltaForce);
			linearM0 = linearM0.MulAdd(row->m_Jt.m_jacobianM0.m_linear, deltaforce0);
			angularM0 = angularM0.MulAdd(row->m_Jt.m_jacobianM0.m_angular, deltaforce0);
			linearM1 = linearM1.MulAdd(row->m_Jt.m_jacobianM1.m_linear, deltaforce1);
			angularM1 = angularM1.MulAdd(row->m_Jt.m_jacobianM1.m_angular, deltaforce1);
		}

		dgVector maxAccel(accNorm);
		const dgFloat32 tol = dgFloat32(0.5f);
		const dgFloat32 tol2 = tol * tol;
		for (dgInt32 i = 0; (i < 4) && (maxAccel.GetScalar() > tol2); i++) {
			maxAccel = dgVector::m_zero;
			for (dgInt32 j = 0; j < rowsCount; j++) {
				dgRightHandSide* const rhs = &rightHandSide[rowStart + j];
				const dgLeftHandSide* const row = &matrixRow[rowStart + j];
				dgVector a(row->m_JMinv.m_jacobianM0.m_linear * linearM0);
				a = a.MulAdd(row->m_JMinv.m_jacobianM0.m_angular, angularM0);
				a = a.MulAdd(row->m_JMinv.m_jacobianM1.m_linear, linearM1);
				a = a.MulAdd(row->m_JMinv.m_jacobianM1.m_angular, angularM1);
				a = dgVector(rhs->m_coordenateAccel - rhs->m_force * rhs->m_diagDamp) - a.AddHorizontal();

				dgVector f(rhs->m_force + rhs->m_invJinvMJt * a.GetScalar());
				dgAssert(rhs->m_normalForceIndex >= -1);
				dgAssert(rhs->m_normalForceIndex <= rowsCount);
				dgInt32 frictionIndex = rhs->m_normalForceIndex + 1;

				dgFloat32 frictionNormal = normalForce[frictionIndex];
				dgVector lowerFrictionForce(frictionNormal * rhs->m_lowerBoundFrictionCoefficent);
				dgVector upperFrictionForce(frictionNormal * rhs->m_upperBoundFrictionCoefficent);

				a = a & (f < upperFrictionForce) & (f > lowerFrictionForce);
				f = f.GetMax(lowerFrictionForce).GetMin(upperFrictionForce);
				maxAccel = maxAccel.MulAdd (a, a);

				dgVector deltaForce(f - dgVector(rhs->m_force));

				rhs->m_force = f.GetScalar();
				normalForce[j + 1] = f.GetScalar();

				dgVector deltaforce0(preconditioner0 * deltaForce);
				dgVector deltaforce1(preconditioner1 * deltaForce);
				linearM0 = linearM0.MulAdd(row->m_Jt.m_jacobianM0.m_linear, deltaforce0);
				angularM0 = angularM0.MulAdd(row->m_Jt.m_jacobianM0.m_angular, deltaforce0);
				linearM1 = linearM1.MulAdd(row->m_Jt.m_jacobianM1.m_linear, deltaforce1);
				angularM1 = angularM1.MulAdd(row->m_Jt.m_jacobianM1.m_angular, deltaforce1);
			}
		}

		for (dgInt32 i = 0; i < rowsCount; i++) {
			dgRightHandSide* const rhs = &rightHandSide[rowStart + i];
			rhs->m_maxImpact = dgMax(dgAbs(rhs->m_force), rhs->m_maxImpact);
		}

		internalForces[m0].m_linear = linearM0;
		internalForces[m0].m_angular = angularM0;
		internalForces[m1].m_linear = linearM1;
		internalForces[m1].m_angular = angularM1;
	}
	return accNorm.GetScalar();
}


dgJacobian dgWorldDynamicUpdate::IntegrateForceAndToque(dgDynamicBody* const body, const dgVector& force, const dgVector& torque, const dgVector& timestep) const
{
	dgJacobian velocStep;
	if (body->m_gyroTorqueOn) {
		dgVector dtHalf(timestep * dgVector::m_half);
		dgMatrix matrix(body->m_gyroRotation, dgVector::m_wOne);

		dgVector localOmega(matrix.UnrotateVector(body->m_omega));
		dgVector localTorque(matrix.UnrotateVector(torque));

		// derivative at half time step. (similar to midpoint Euler so that it does not loses too much energy)
		dgVector dw(localOmega * dtHalf);
		dgVector inertia(body->m_mass);

		dgMatrix jacobianMatrix(
			dgVector(inertia[0], (inertia[2] - inertia[1]) * dw[2], (inertia[2] - inertia[1]) * dw[1], dgFloat32(0.0f)),
			dgVector((inertia[0] - inertia[2]) * dw[2], inertia[1], (inertia[0] - inertia[2]) * dw[0], dgFloat32(1.0f)),
			dgVector((inertia[1] - inertia[0]) * dw[1], (inertia[1] - inertia[0]) * dw[0], inertia[2], dgFloat32(1.0f)),
			dgVector::m_wOne);

		// and solving for alpha we get the angular acceleration at t + dt
		// calculate gradient at a full time step
		//dgVector gradientStep(localTorque * timestep);
		dgVector gradientStep (jacobianMatrix.SolveByGaussianElimination(localTorque * timestep));

		dgVector omega(matrix.RotateVector(localOmega + gradientStep));
		dgAssert(omega.m_w == dgFloat32(0.0f));

		// integrate rotation here
		dgFloat32 omegaMag2 = omega.DotProduct(omega).GetScalar() + dgFloat32(1.0e-12f);
		dgFloat32 invOmegaMag = dgRsqrt(omegaMag2);
		dgVector omegaAxis(omega.Scale(invOmegaMag));
		dgFloat32 omegaAngle = invOmegaMag * omegaMag2 * timestep.GetScalar();
		dgQuaternion deltaRotation(omegaAxis, omegaAngle);
		body->m_gyroRotation = body->m_gyroRotation * deltaRotation;
		dgAssert((body->m_gyroRotation.DotProduct(body->m_gyroRotation) - dgFloat32(1.0f)) < dgFloat32(1.0e-5f));

		matrix = dgMatrix(body->m_gyroRotation, dgVector::m_wOne);
		localOmega = matrix.UnrotateVector(omega);
		//dgVector angularMomentum(inertia * localOmega);
		//body->m_gyroTorque = matrix.RotateVector(localOmega.CrossProduct(angularMomentum));
		//body->m_gyroAlpha = body->m_invWorldInertiaMatrix.RotateVector(body->m_gyroTorque);
		dgVector localGyroTorque(localOmega.CrossProduct(inertia * localOmega));
		body->m_gyroTorque = matrix.RotateVector(localGyroTorque);
		body->m_gyroAlpha = matrix.RotateVector(localGyroTorque * body->m_invMass);

		velocStep.m_angular = matrix.RotateVector(gradientStep);
	} else {
		velocStep.m_angular = body->m_invWorldInertiaMatrix.RotateVector(torque - body->m_gyroTorque) * timestep;
//		velocStep.m_angular = velocStep.m_angular * dgVector::m_half;
	}

	velocStep.m_linear = force.Scale(body->m_invMass.m_w) * timestep;
	return velocStep;
}


void dgWorldDynamicUpdate::CalculateClusterReactionForces(const dgBodyCluster* const cluster, dgInt32 threadID, dgFloat32 timestep) const
{
	D_TRACKTIME();
	dgWorld* const world = (dgWorld*) this;
	const dgInt32 bodyCount = cluster->m_bodyCount;
	const dgInt32 jointCount = cluster->m_jointCount;

	dgJacobian* const internalForces = &m_solverMemory.m_internalForcesBuffer[cluster->m_bodyStart];
	dgBodyInfo* const bodyArray = &world->m_bodiesMemory[cluster->m_bodyStart];
	dgJointInfo* const constraintArray = &world->m_jointsMemory[cluster->m_jointStart];

	dgRightHandSide* const rightHandSide = &m_solverMemory.m_righHandSizeBuffer[cluster->m_rowStart];
	const dgLeftHandSide* const leftHandSide = &m_solverMemory.m_leftHandSizeBuffer[cluster->m_rowStart];

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
	dgSkeletonList& skeletonList = *world;
	dgSkeletonContainer* skeletonArray[DG_MAX_SKELETON_JOINT_COUNT];
	dgInt32 lru = dgAtomicExchangeAndAdd(&skeletonList.m_lruMarker, 1);
	for (dgInt32 i = 1; i < bodyCount; i++) {
		dgDynamicBody* const body = (dgDynamicBody*)bodyArray[i].m_body;
		dgSkeletonContainer* const container = body->GetSkeleton();
		if (container && (container->m_lru != lru)) {
			container->m_lru = lru;
			skeletonArray[skeletonCount] = container;
			container->InitMassMatrix(constraintArray, leftHandSide, rightHandSide);
			skeletonCount++;
			dgAssert(skeletonCount < dgInt32(sizeof(skeletonArray) / sizeof(skeletonArray[0])));
		}
	}

	const dgInt32 passes = world->m_solverIterations;
	const dgFloat32 maxAccNorm = DG_SOLVER_MAX_ERROR * DG_SOLVER_MAX_ERROR;
	for (dgInt32 step = 0; step < derivativesEvaluationsRK4; step++) {

		for (dgInt32 i = 0; i < jointCount; i++) {
			dgJointInfo* const jointInfo = &constraintArray[i];
			dgConstraint* const constraint = jointInfo->m_joint;
			const dgInt32 pairStart = jointInfo->m_pairStart;

			joindDesc.m_rowsCount = jointInfo->m_pairCount;
			joindDesc.m_leftHandSide = &leftHandSide[pairStart];
			joindDesc.m_rightHandSide = &rightHandSide[pairStart];
			constraint->JointAccelerations(&joindDesc);
		}
		joindDesc.m_firstPassCoefFlag = dgFloat32(1.0f);
	
		dgFloat32 accNorm = maxAccNorm * dgFloat32(2.0f);
		for (dgInt32 i = 0; (i < passes) && (accNorm > maxAccNorm); i++) {
			accNorm = dgFloat32(0.0f);
			for (dgInt32 j = 0; j < jointCount; j++) {
				dgJointInfo* const jointInfo = &constraintArray[j];
				//if (!jointInfo->m_joint->IsSkeleton()) 
				{
					//dgFloat32 accel2 = CalculateJointForce_3_13(jointInfo, bodyArray, internalForces, leftHandSide);
					dgFloat32 accel2 = CalculateJointForce(jointInfo, bodyArray, internalForces, leftHandSide, rightHandSide);
					accNorm += accel2;
				}
			}
		}
		for (dgInt32 j = 0; j < skeletonCount; j++) {
			skeletonArray[j]->CalculateJointForce(constraintArray, bodyArray, internalForces);
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

					dgJacobian velocStep(body->IntegrateForceAndToque(force, torque, timestep4));
					if (!body->m_resting) {
						body->m_veloc += velocStep.m_linear;
						body->m_omega += velocStep.m_angular;
					} else {
						const dgVector velocStep2(velocStep.m_linear.DotProduct(velocStep.m_linear));
						const dgVector omegaStep2(velocStep.m_angular.DotProduct(velocStep.m_angular));
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

				body->m_veloc += linearMomentum.Scale(body->m_invMass.m_w);
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
				dgRightHandSide* const rhs = &rightHandSide[j + first];
				dgAssert(dgCheckFloat(rhs->m_force));
				rhs->m_jointFeebackForce->m_force = rhs->m_force;
				rhs->m_jointFeebackForce->m_impact = rhs->m_maxImpact * timestepRK;
			}
			hasJointFeeback |= (constraint->m_updaFeedbackCallback ? 1 : 0);
		}

		const dgFloat32 zeroAceel = (jointCount >= 16) ? DG_SOLVER_MAX_ERROR : DG_SOLVER_MAX_ERROR * dgFloat32 (0.25f);
		const dgVector invTime(invTimestep);
		const dgVector maxAccNorm2(zeroAceel * zeroAceel);

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

