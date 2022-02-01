/* Copyright (c) <2003-2021> <Julio Jerez, Newton Game Dynamics>
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

#include "ndCoreStdafx.h"
#include "ndNewtonStdafx.h"
#include "ndWorld.h"
#include "ndBodyDynamic.h"
#include "ndDynamicsUpdate.h"
#include "ndSkeletonContainer.h"
#include "ndSkeletonImmediateSolver.h"
#include "ndJointBilateralConstraint.h"

void ndSkeletonImmediateSolver::GetJacobianDerivatives(ndConstraint* const joint)
{
	ndConstraintDescritor constraintParam;
	dAssert(joint->GetRowsCount() <= D_CONSTRAINT_MAX_ROWS);
	for (ndInt32 i = joint->GetRowsCount() - 1; i >= 0; i--)
	{
		constraintParam.m_forceBounds[i].m_low = D_MIN_BOUND;
		constraintParam.m_forceBounds[i].m_upper = D_MAX_BOUND;
		constraintParam.m_forceBounds[i].m_jointForce = nullptr;
		constraintParam.m_forceBounds[i].m_normalIndex = D_INDEPENDENT_ROW;
	}
	joint->m_rowCount = joint->GetRowsCount();

	constraintParam.m_rowsCount = 0;
	constraintParam.m_timestep = m_timestep;
	constraintParam.m_invTimestep = m_invTimestep;
	joint->JacobianDerivative(constraintParam);
	const ndInt32 dof = constraintParam.m_rowsCount;
	dAssert(dof <= joint->m_rowCount);

	if (joint->GetAsContact())
	{
		ndContact* const contactJoint = joint->GetAsContact();
		contactJoint->m_isInSkeletonLoop = 0;
		ndSkeletonContainer* const skeleton0 = contactJoint->GetBody0()->GetSkeleton();
		ndSkeletonContainer* const skeleton1 = contactJoint->GetBody1()->GetSkeleton();
		if (skeleton0 && (skeleton0 == skeleton1))
		{
			if (contactJoint->IsSkeletonSelftCollision())
			{
				contactJoint->m_isInSkeletonLoop = 1;
				skeleton0->AddSelfCollisionJoint(contactJoint);
			}
		}
		else if (contactJoint->IsSkeletonIntraCollision())
		{
			if (skeleton0 && !skeleton1)
			{
				contactJoint->m_isInSkeletonLoop = 1;
				skeleton0->AddSelfCollisionJoint(contactJoint);
			}
			else if (skeleton1 && !skeleton0)
			{
				contactJoint->m_isInSkeletonLoop = 1;
				skeleton1->AddSelfCollisionJoint(contactJoint);
			}
		}
	}
	else
	{
		ndJointBilateralConstraint* const bilareral = joint->GetAsBilateral();
		dAssert(bilareral);
		if (!bilareral->GetSkeletonFlag() && (bilareral->GetSolverModel() == m_jointkinematicAttachment))
		{
			ndSkeletonContainer* const skeleton0 = bilareral->GetBody0()->GetSkeleton();
			ndSkeletonContainer* const skeleton1 = bilareral->GetBody1()->GetSkeleton();
			if (skeleton0 || skeleton1)
			{
				if (skeleton0 && !skeleton1)
				{
					bilareral->m_isInSkeletonLoop = 1;
					skeleton0->AddSelfCollisionJoint(bilareral);
				}
				else if (skeleton1 && !skeleton0)
				{
					bilareral->m_isInSkeletonLoop = 1;
					skeleton1->AddSelfCollisionJoint(bilareral);
				}
			}
		}
	}

	joint->m_rowCount = dof;
	joint->m_rowStart = m_leftHandSide.GetCount();
	const ndInt32 baseIndex = joint->m_rowStart;
	for (ndInt32 i = 0; i < dof; ++i)
	{
		dAssert(constraintParam.m_forceBounds[i].m_jointForce);

		m_leftHandSide.PushBack(ndLeftHandSide());
		m_rightHandSide.PushBack(ndRightHandSide());
		ndLeftHandSide* const row = &m_leftHandSide[m_leftHandSide.GetCount()-1];
		ndRightHandSide* const rhs = &m_rightHandSide[m_leftHandSide.GetCount() - 1];

		row->m_Jt = constraintParam.m_jacobian[i];
		rhs->m_diagDamp = ndFloat32(0.0f);
		rhs->m_diagonalRegularizer = dMax(constraintParam.m_diagonalRegularizer[i], ndFloat32(1.0e-5f));

		rhs->m_coordenateAccel = constraintParam.m_jointAccel[i];
		rhs->m_restitution = constraintParam.m_restitution[i];
		rhs->m_penetration = constraintParam.m_penetration[i];
		rhs->m_penetrationStiffness = constraintParam.m_penetrationStiffness[i];
		rhs->m_lowerBoundFrictionCoefficent = constraintParam.m_forceBounds[i].m_low;
		rhs->m_upperBoundFrictionCoefficent = constraintParam.m_forceBounds[i].m_upper;
		rhs->m_jointFeebackForce = constraintParam.m_forceBounds[i].m_jointForce;

		dAssert(constraintParam.m_forceBounds[i].m_normalIndex >= -1);
		const ndInt32 frictionIndex = constraintParam.m_forceBounds[i].m_normalIndex;
		const ndInt32 mask = frictionIndex >> 31;
		rhs->m_normalForceIndex = frictionIndex;
		rhs->m_normalForceIndexFlat = ~mask & (frictionIndex + baseIndex);
	}
}

void ndSkeletonImmediateSolver::BuildJacobianMatrix (ndConstraint* const joint)
{
	dAssert(joint->GetBody0());
	dAssert(joint->GetBody1());
	const ndBodyKinematic* const body0 = joint->GetBody0();
	const ndBodyKinematic* const body1 = joint->GetBody1();

	const ndVector force0(body0->GetForce());
	const ndVector torque0(body0->GetTorque());
	const ndVector force1(body1->GetForce());
	const ndVector torque1(body1->GetTorque());

	const ndInt32 index = joint->m_rowStart;
	const ndInt32 count = joint->m_rowCount;
	const ndMatrix& invInertia0 = body0->m_invWorldInertiaMatrix;
	const ndMatrix& invInertia1 = body1->m_invWorldInertiaMatrix;
	const ndVector invMass0(body0->m_invMass[3]);
	const ndVector invMass1(body1->m_invMass[3]);

	joint->m_preconditioner0 = ndFloat32(1.0f);
	joint->m_preconditioner1 = ndFloat32(1.0f);

	//const bool test = !((body0->m_isStatic | body1->m_isStatic) || (body0->GetSkeleton() && body1->GetSkeleton()));
	//dAssert(test == ((invMass0.GetScalar() > ndFloat32(0.0f)) && (invMass1.GetScalar() > ndFloat32(0.0f)) && !(body0->GetSkeleton() && body1->GetSkeleton())));
	//if (test)
	//{
	//	const ndFloat32 mass0 = body0->GetMassMatrix().m_w;
	//	const ndFloat32 mass1 = body1->GetMassMatrix().m_w;
	//	if (mass0 > (D_DIAGONAL_PRECONDITIONER * mass1))
	//	{
	//		joint->m_preconditioner0 = mass0 / (mass1 * D_DIAGONAL_PRECONDITIONER);
	//	}
	//	else if (mass1 > (D_DIAGONAL_PRECONDITIONER * mass0))
	//	{
	//		joint->m_preconditioner1 = mass1 / (mass0 * D_DIAGONAL_PRECONDITIONER);
	//	}
	//}

	//const ndVector zero(ndVector::m_zero);
	//ndVector forceAcc0(zero);
	//ndVector torqueAcc0(zero);
	//ndVector forceAcc1(zero);
	//ndVector torqueAcc1(zero);

	#ifdef D_PROGRESSIVE_SLEEP_EXPERIMENT
	//const ndVector progressiveSleepWeigh(ndFloat32(0.01f));
	//if (body0->m_isJointFence1 & !body0->m_isStatic)
	//{
	//	for (ndInt32 i = 0; i < count; ++i)
	//	{
	//		ndLeftHandSide* const row = &m_leftHandSide[index + i];
	//		row->m_Jt.m_jacobianM0.m_linear = row->m_Jt.m_jacobianM0.m_linear * progressiveSleepWeigh;
	//		row->m_Jt.m_jacobianM0.m_angular = row->m_Jt.m_jacobianM0.m_linear * progressiveSleepWeigh;
	//	}
	//}
	//
	//if (body1->m_isJointFence1 & !body1->m_isStatic)
	//{
	//	for (ndInt32 i = 0; i < count; ++i)
	//	{
	//		ndLeftHandSide* const row = &m_leftHandSide[index + i];
	//		row->m_Jt.m_jacobianM1.m_linear = row->m_Jt.m_jacobianM1.m_linear * progressiveSleepWeigh;
	//		row->m_Jt.m_jacobianM1.m_angular = row->m_Jt.m_jacobianM1.m_linear * progressiveSleepWeigh;
	//	}
	//}
	#endif

	//const ndVector weigh0(body0->m_weigh * joint->m_preconditioner0);
	//const ndVector weigh1(body1->m_weigh * joint->m_preconditioner1);
	//const ndFloat32 preconditioner0 = joint->m_preconditioner0;
	//const ndFloat32 preconditioner1 = joint->m_preconditioner1;

	const ndVector weigh0(ndVector::m_one);
	const ndVector weigh1(ndVector::m_one);
	//const ndFloat32 preconditioner0 = ndFloat32(1.0f);
	//const ndFloat32 preconditioner1 = ndFloat32(1.0f);

	//const bool isBilateral = joint->IsBilateral();
	for (ndInt32 i = 0; i < count; ++i)
	{
		ndLeftHandSide* const row = &m_leftHandSide[index + i];
		ndRightHandSide* const rhs = &m_rightHandSide[index + i];

		row->m_JMinv.m_jacobianM0.m_linear = row->m_Jt.m_jacobianM0.m_linear * invMass0;
		row->m_JMinv.m_jacobianM0.m_angular = invInertia0.RotateVector(row->m_Jt.m_jacobianM0.m_angular);
		row->m_JMinv.m_jacobianM1.m_linear = row->m_Jt.m_jacobianM1.m_linear * invMass1;
		row->m_JMinv.m_jacobianM1.m_angular = invInertia1.RotateVector(row->m_Jt.m_jacobianM1.m_angular);

		const ndJacobian& JMinvM0 = row->m_JMinv.m_jacobianM0;
		const ndJacobian& JMinvM1 = row->m_JMinv.m_jacobianM1;
		const ndVector tmpAccel(
			JMinvM0.m_linear * force0 + JMinvM0.m_angular * torque0 +
			JMinvM1.m_linear * force1 + JMinvM1.m_angular * torque1);
		
		const ndFloat32 extenalAcceleration = -tmpAccel.AddHorizontal().GetScalar();
		rhs->m_deltaAccel = extenalAcceleration;
		rhs->m_coordenateAccel += extenalAcceleration;
		dAssert(rhs->m_jointFeebackForce);

		//const ndFloat32 force = rhs->m_jointFeebackForce->GetInitialGuess();
		//rhs->m_force = isBilateral ? dClamp(force, rhs->m_lowerBoundFrictionCoefficent, rhs->m_upperBoundFrictionCoefficent) : force;
		//rhs->m_maxImpact = ndFloat32(0.0f);
		rhs->m_force = ndFloat32(0.0f);
		
		const ndJacobian& JtM0 = row->m_Jt.m_jacobianM0;
		const ndJacobian& JtM1 = row->m_Jt.m_jacobianM1;
		const ndVector tmpDiag(
			weigh0 * (JMinvM0.m_linear * JtM0.m_linear + JMinvM0.m_angular * JtM0.m_angular) +
			weigh1 * (JMinvM1.m_linear * JtM1.m_linear + JMinvM1.m_angular * JtM1.m_angular));
		
		ndFloat32 diag = tmpDiag.AddHorizontal().GetScalar();
		dAssert(diag > ndFloat32(0.0f));
		rhs->m_diagDamp = diag * rhs->m_diagonalRegularizer;
		
		diag *= (ndFloat32(1.0f) + rhs->m_diagonalRegularizer);
		rhs->m_invJinvMJt = ndFloat32(1.0f) / diag;
		
		//ndVector f0(rhs->m_force * preconditioner0);
		//ndVector f1(rhs->m_force * preconditioner1);
		//forceAcc0 = forceAcc0 + JtM0.m_linear * f0;
		//torqueAcc0 = torqueAcc0 + JtM0.m_angular * f0;
		//forceAcc1 = forceAcc1 + JtM1.m_linear * f1;
		//torqueAcc1 = torqueAcc1 + JtM1.m_angular * f1;
	}

	//ndInt32 index0 = body0->m_index;
	//ndJacobian& outBody0 = m_internalForces[index0];
	//outBody0.m_linear += forceAcc0;
	//outBody0.m_angular += torqueAcc0;
	//
	//ndInt32 index1 = body0->m_index;
	//ndJacobian& outBody1 = m_internalForces[index1];
	//outBody1.m_linear += forceAcc1;
	//outBody1.m_angular += torqueAcc1;
}

void ndSkeletonImmediateSolver::Solve(ndSkeletonContainer* const skeleton, ndWorld* const world, ndFloat32 timestep)
{
	if (!skeleton->m_isResting)
	{
		m_world = world;
		m_skeleton = skeleton;
		m_timestep = timestep;
		m_invTimestep = ndFloat32(1.0f) / timestep;

		// initialize bodies velocity base forces
		ndVector zero(ndVector::m_zero);
		m_bodyArray.SetCount(m_skeleton->m_nodeList.GetCount());
		m_internalForces.SetCount(m_skeleton->m_nodeList.GetCount());
		for (ndInt32 i = 0; i < m_skeleton->m_nodeList.GetCount(); ++i)
		{
			ndSkeletonContainer::ndNode* const node = m_skeleton->m_nodesOrder[i];
			ndBodyKinematic* const body = node->m_body;
			body->UpdateInvInertiaMatrix();

			const ndVector angularMomentum(body->CalculateAngularMomentum());
			body->m_gyroTorque = body->m_omega.CrossProduct(angularMomentum);
			body->m_gyroAlpha = body->m_invWorldInertiaMatrix.RotateVector(body->m_gyroTorque);

			body->m_accel = body->m_veloc;
			body->m_alpha = body->m_omega;
			body->m_gyroRotation = body->m_rotation;

			body->m_rank = body->m_index;
			body->m_index = i;

			m_bodyArray[i] = body;
			m_internalForces[i].m_linear = zero;
			m_internalForces[i].m_angular = zero;
		}

		m_leftHandSide.SetCount(0);
		m_rightHandSide.SetCount(0);

		for (ndInt32 i = m_skeleton->m_nodeList.GetCount() - 2; i >= 0; --i)
		{
			ndSkeletonContainer::ndNode* const node = m_skeleton->m_nodesOrder[i];
			ndJointBilateralConstraint* const joint = node->m_joint;
			GetJacobianDerivatives(joint);
			BuildJacobianMatrix(joint);
		}

		m_skeleton->InitMassMatrix(&m_leftHandSide[0], &m_rightHandSide[0]);
		m_skeleton->CalculateJointForceImmediate(&m_internalForces[0]);

		// restore body info
		for (ndInt32 i = 0; i < m_skeleton->m_nodeList.GetCount(); ++i)
		{
			ndSkeletonContainer::ndNode* const node = m_skeleton->m_nodesOrder[i];
			ndBodyKinematic* const body = node->m_body;
			body->m_index = body->m_rank;
			body->m_rank = 0;
		}
	}
}