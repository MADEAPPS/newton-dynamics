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
#include "ndIkSolver.h"
#include "ndBodyDynamic.h"
#include "ndDynamicsUpdate.h"
#include "ndSkeletonContainer.h"
#include "ndJointBilateralConstraint.h"

ndIkSolver::ndIkSolver()
	:ndClassAlloc()
	,m_sentinelBody()
	,m_bodies(32)
	,m_internalForces(32)
	,m_leftHandSide(128)
	,m_rightHandSide(128)
	,m_world(nullptr)
	,m_skeleton(nullptr)
	,m_timestep(ndFloat32(0.0f))
	,m_invTimestep(ndFloat32(0.0f))
	,m_maxIterations(4)
{
}

ndIkSolver::~ndIkSolver()
{
}

void ndIkSolver::SetMaxIterations(ndInt32 iterCount)
{
	m_maxIterations = dClamp(iterCount, 1, 16);
}

void ndIkSolver::GetJacobianDerivatives(ndConstraint* const joint)
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
				skeleton0->AddCloseLoopJoint(contactJoint);
			}
		}
		else if (contactJoint->IsSkeletonIntraCollision())
		{
			if (skeleton0 && !skeleton1)
			{
				contactJoint->m_isInSkeletonLoop = 1;
				skeleton0->AddCloseLoopJoint(contactJoint);
			}
			else if (skeleton1 && !skeleton0)
			{
				contactJoint->m_isInSkeletonLoop = 1;
				skeleton1->AddCloseLoopJoint(contactJoint);
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
					skeleton0->AddCloseLoopJoint(bilareral);
				}
				else if (skeleton1 && !skeleton0)
				{
					bilareral->m_isInSkeletonLoop = 1;
					skeleton1->AddCloseLoopJoint(bilareral);
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

void ndIkSolver::BuildJacobianMatrix (ndConstraint* const joint)
{
	dAssert(joint->GetBody0());
	dAssert(joint->GetBody1());
	const ndBodyKinematic* const body0 = joint->GetBody0();
	const ndBodyKinematic* const body1 = joint->GetBody1();
	
	const ndInt32 index = joint->m_rowStart;
	const ndInt32 count = joint->m_rowCount;
	const ndMatrix& invInertia0 = body0->m_invWorldInertiaMatrix;
	const ndMatrix& invInertia1 = body1->m_invWorldInertiaMatrix;
	const ndVector invMass0(body0->m_invMass[3]);
	const ndVector invMass1(body1->m_invMass[3]);

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
		rhs->m_force = ndFloat32(0.0f);
		
		const ndJacobian& JtM0 = row->m_Jt.m_jacobianM0;
		const ndJacobian& JtM1 = row->m_Jt.m_jacobianM1;
		const ndVector tmpDiag(
			JMinvM0.m_linear * JtM0.m_linear + JMinvM0.m_angular * JtM0.m_angular +
			JMinvM1.m_linear * JtM1.m_linear + JMinvM1.m_angular * JtM1.m_angular);
		
		ndFloat32 diag = tmpDiag.AddHorizontal().GetScalar();
		dAssert(diag > ndFloat32(0.0f));
		rhs->m_diagDamp = diag * rhs->m_diagonalRegularizer;
	}
}

void ndIkSolver::AddEffector(ndSkeletonContainer* const skeleton, ndConstraint* const joint)
{
	skeleton->AddCloseLoopJoint(joint);
}

bool ndIkSolver::IsSleeping(ndSkeletonContainer* const skeleton) const
{
	return skeleton->m_isResting ? true : false;
}

ndVector ndIkSolver::GetBodyForce(const ndBodyKinematic* const body) const
{
	return body->m_accel;
}

ndVector ndIkSolver::GetBodyTorque(const ndBodyKinematic* const body) const
{
	return body->m_alpha;
}

void ndIkSolver::BuildMassMatrix()
{
	m_bodies.SetCount(0);
	m_leftHandSide.SetCount(0);
	m_rightHandSide.SetCount(0);
	ndFixSizeArray<ndContact*, 256> contacts;
	
	const ndVector zero(ndVector::m_zero);
	m_bodies.PushBack(&m_sentinelBody);
	m_sentinelBody.m_index = 0;
	m_sentinelBody.m_accel = zero;
	m_sentinelBody.m_alpha = zero;
	
	// add open loop bodies
	for (ndInt32 i = 0; i < m_skeleton->m_nodeList.GetCount(); ++i)
	{
		ndSkeletonContainer::ndNode* const node = m_skeleton->m_nodesOrder[i];
		ndBodyKinematic* const body = node->m_body;
		if (body->GetInvMass() > ndFloat32(0.0f))
		{
			m_bodies.PushBack(body);
			body->m_rank = -1;
		}
	}
	
	// add close loop 
	const ndInt32 loopCount = m_skeleton->m_dynamicsLoopCount + m_skeleton->m_loopCount;
	for (ndInt32 i = 0; i < loopCount; i++)
	{
		ndConstraint* const joint = m_skeleton->m_loopingJoints[i];
		ndBodyKinematic* const body0 = joint->GetBody0();
		ndBodyKinematic* const body1 = joint->GetBody1();
		dAssert(body0->GetInvMass() > ndFloat32(0.0f));
		if (body0->m_rank == 0)
		{
			m_bodies.PushBack(body0);
			body0->m_rank = -1;
		}
		if ((body1->m_rank == 0) && (body1->GetInvMass() > ndFloat32(0.0f)))
		{
			m_bodies.PushBack(body0);
			body0->m_rank = -1;
		}
	}
	
	// add contacts loop bodies and joints
	for (ndInt32 i = 0; i < m_skeleton->m_nodeList.GetCount(); ++i)
	{
		ndSkeletonContainer::ndNode* const node = m_skeleton->m_nodesOrder[i];
		ndBodyKinematic* const body = node->m_body;
	
		ndBodyKinematic::ndContactMap& contactMap = body->GetContactMap();
		ndBodyKinematic::ndContactMap::Iterator it(contactMap);
		for (it.Begin(); it; it++)
		{
			ndContact* const contact = it.GetNode()->GetInfo();
			if (contact->IsActive())
			{
				bool duplicate = false;
				const ndInt32 loops = m_skeleton->m_dynamicsLoopCount + m_skeleton->m_loopCount;
				for (ndInt32 j = 0; j < loops; ++j)
				{
					duplicate = duplicate | (m_skeleton->m_loopingJoints[j] == contact);
				}
				if (!duplicate)
				{
					contacts.PushBack(contact);
					ndBodyKinematic* const body0 = contact->GetBody0();
					ndBodyKinematic* const body1 = contact->GetBody1();
					dAssert(body0->GetInvMass() > ndFloat32(0.0f));
					if (body0->m_rank == 0)
					{
						m_bodies.PushBack(body0);
						body0->m_rank = -1;
					}
					if ((body1->m_rank == 0) && (body1->GetInvMass() > ndFloat32(0.0f)))
					{
						m_bodies.PushBack(body1);
						body1->m_rank = -1;
					}
				}
			}
		}
	}
	
	m_internalForces.SetCount(m_bodies.GetCount());
	for (ndInt32 i = 0; i < m_bodies.GetCount(); ++i)
	{
		ndBodyKinematic* const body = m_bodies[i];
		body->m_rank = body->m_index;
		body->m_index = i;
	
		body->UpdateInvInertiaMatrix();
		const ndVector gyroTorque(body->m_omega.CrossProduct(body->CalculateAngularMomentum()));
	
		body->m_accel = zero;
		body->m_alpha = zero;
		m_internalForces[i].m_linear = body->GetForce();
		m_internalForces[i].m_angular = body->GetTorque() - gyroTorque;
	}
	
	for (ndInt32 i = m_skeleton->m_nodeList.GetCount() - 2; i >= 0; --i)
	{
		ndSkeletonContainer::ndNode* const node = m_skeleton->m_nodesOrder[i];
		ndJointBilateralConstraint* const joint = node->m_joint;
		GetJacobianDerivatives(joint);
		BuildJacobianMatrix(joint);
	}
	
	const ndInt32 loops = m_skeleton->m_dynamicsLoopCount + m_skeleton->m_loopCount;
	for (ndInt32 i = 0; i < loops; i++)
	{
		ndConstraint* const joint = m_skeleton->m_loopingJoints[i];
		GetJacobianDerivatives(joint);
		BuildJacobianMatrix(joint);
	}
	
	for (ndInt32 i = 0; i < contacts.GetCount(); i++)
	{
		ndContact* const contact = contacts[i];
		GetJacobianDerivatives(contact);
		BuildJacobianMatrix(contact);
	}
	m_skeleton->InitMassMatrix(&m_leftHandSide[0], &m_rightHandSide[0]);
}

void ndIkSolver::Solve(ndSkeletonContainer* const skeleton, ndWorld* const world, ndFloat32 timestep)
{
	m_world = world;
	m_skeleton = skeleton;
	m_timestep = timestep;
	m_invTimestep = ndFloat32(1.0f) / timestep;

	for (ndInt32 i = m_skeleton->m_nodeList.GetCount() - 2; i >= 0; --i)
	{
		ndSkeletonContainer::ndNode* const node = m_skeleton->m_nodesOrder[i];
		ndJointBilateralConstraint* const joint = node->m_joint;
		//joint->SetIkSolver();
		joint->SetIkMode(true);
	}
	BuildMassMatrix();

	m_skeleton->SolveImmediate(*this);

	//ndFixSizeArray<bool, 256> mask;
	//ndFixSizeArray<ndFloat32, 256> accelerations;
	ndFixSizeArray<ndJacobian, 256> accelerations;
	//mask.SetCount(m_skeleton->m_nodeList.GetCount());
	accelerations.SetCount(m_skeleton->m_nodeList.GetCount());
	//for (ndInt32 i = m_skeleton->m_nodeList.GetCount() - 2; i >= 0; --i)
	//{
	//	mask[i] = true;
	//}

	accelerations[0].m_linear = ndVector::m_zero;
	accelerations[0].m_angular = ndVector::m_zero;
	for (ndInt32 i = 1; i < m_bodies.GetCount(); ++i)
	{
		ndBodyKinematic* const body = m_bodies[i];
		const ndInt32 index = body->m_index;

		const ndVector invMass(body->GetInvMass());
		const ndMatrix& invInertia = body->GetInvInertiaMatrix();

		accelerations[i].m_linear = invMass * (body->m_accel + m_internalForces[index].m_linear);
		accelerations[i].m_angular = invInertia.RotateVector (body->m_alpha + m_internalForces[index].m_angular);
	}

	//bool accelerationsAreValid = false;
	//ndInt32 maxPasses = m_maxIterations;
	//const ndVector zero(ndVector::m_zero);
	//while (!accelerationsAreValid && maxPasses)
	//{
	//	maxPasses--;
	//	accelerationsAreValid = true;
	//	for (ndInt32 i = m_skeleton->m_nodeList.GetCount() - 2; i >= 0; --i)
	//	{
	//		if (mask[i])
	//		{
	//			ndJacobian forceBody0;
	//			ndJacobian forceBody1;
	//			ndSkeletonContainer::ndNode* const node = m_skeleton->m_nodesOrder[i];
	//			ndJointBilateralConstraint* const joint = node->m_joint;
	//
	//			const ndInt32 index0 = (joint->GetBody0()->GetInvMass() > ndFloat32(0.0f)) ? joint->GetBody0()->m_index : 0;
	//			const ndInt32 index1 = (joint->GetBody1()->GetInvMass() > ndFloat32(0.0f)) ? joint->GetBody1()->m_index : 0;
	//			ndBodyKinematic* const body0 = m_bodies[index0];
	//			ndBodyKinematic* const body1 = m_bodies[index1];
	//
	//			forceBody0.m_linear = body0->m_accel + m_internalForces[index0].m_linear;
	//			forceBody0.m_angular = body0->m_alpha + m_internalForces[index0].m_angular;
	//			forceBody1.m_linear = body1->m_accel + m_internalForces[index1].m_linear;
	//			forceBody1.m_angular = body1->m_alpha + m_internalForces[index1].m_angular;
	//
	//			dAssert(0);
	//			//mask[i] = joint->SetIkMotor(timestep, forceBody0, forceBody1, &m_leftHandSide[joint->m_rowStart]);
	//			//accelerationsAreValid = accelerationsAreValid & mask[i];
	//		}
	//	}
	//
	//	if (!accelerationsAreValid)
	//	{
	//		if (!maxPasses)
	//		{
	//			for (ndInt32 i = m_skeleton->m_nodeList.GetCount() - 2; i >= 0; --i)
	//			{
	//				if (mask[i])
	//				{
	//					ndSkeletonContainer::ndNode* const node = m_skeleton->m_nodesOrder[i];
	//					ndJointBilateralConstraint* const joint = node->m_joint;
	//					dAssert(0);
	//					//joint->StopIkMotor(timestep);
	//				}
	//			}
	//			break;
	//		}
	//		else
	//		{
	//			for (ndInt32 i = 0; i < m_bodies.GetCount(); ++i)
	//			{
	//				ndBodyKinematic* const body = m_bodies[i];
	//				body->m_accel = zero;
	//				body->m_alpha = zero;
	//			}
	//
	//			for (ndInt32 j = m_skeleton->m_nodeList.GetCount() - 2; j >= 0; --j)
	//			{
	//				if (!mask[j])
	//				{
	//					ndSkeletonContainer::ndNode* const node = m_skeleton->m_nodesOrder[j];
	//					ndJointBilateralConstraint* const joint = node->m_joint;
	//
	//					ndConstraintDescritor constraintParam;
	//					dAssert(joint->GetRowsCount() <= D_CONSTRAINT_MAX_ROWS);
	//					for (ndInt32 i = joint->GetRowsCount() - 1; i >= 0; i--)
	//					{
	//						constraintParam.m_forceBounds[i].m_low = D_MIN_BOUND;
	//						constraintParam.m_forceBounds[i].m_upper = D_MAX_BOUND;
	//						constraintParam.m_forceBounds[i].m_jointForce = nullptr;
	//						constraintParam.m_forceBounds[i].m_normalIndex = D_INDEPENDENT_ROW;
	//					}
	//					joint->m_rowCount = joint->GetRowsCount();
	//
	//					constraintParam.m_rowsCount = 0;
	//					constraintParam.m_timestep = m_timestep;
	//					constraintParam.m_invTimestep = m_invTimestep;
	//					joint->JacobianDerivative(constraintParam);
	//					const ndInt32 dof = constraintParam.m_rowsCount;
	//					//dAssert(dof == joint->m_rowCount);
	//					const ndInt32 baseIndex = joint->m_rowStart;
	//					for (ndInt32 i = 0; i < dof; ++i)
	//					{
	//						ndRightHandSide* const rhs = &m_rightHandSide[baseIndex + i];
	//						rhs->m_coordenateAccel = constraintParam.m_jointAccel[i];
	//						rhs->m_lowerBoundFrictionCoefficent = constraintParam.m_forceBounds[i].m_low;
	//						rhs->m_upperBoundFrictionCoefficent = constraintParam.m_forceBounds[i].m_upper;
	//					}
	//				}
	//			}
	//			m_skeleton->SolveImmediate(*this);
	//		}
	//	}
	//}
	
	for (ndInt32 i = m_skeleton->m_nodeList.GetCount() - 2; i >= 0; --i)
	{
		ndSkeletonContainer::ndNode* const node = m_skeleton->m_nodesOrder[i];
		ndJointBilateralConstraint* const joint = node->m_joint;
		const ndInt32 index0 = (joint->GetBody0()->GetInvMass() > ndFloat32(0.0f)) ? joint->GetBody0()->m_index : 0;
		const ndInt32 index1 = (joint->GetBody1()->GetInvMass() > ndFloat32(0.0f)) ? joint->GetBody1()->m_index : 0;
		joint->SetIkSetAccel(accelerations[index0], accelerations[index1]);
		joint->SetIkMode(false);
	}

	for (ndInt32 i = 1; i < m_bodies.GetCount(); ++i)
	{
		ndBodyKinematic* const body = m_bodies[i];
		const ndInt32 index = body->m_index;
		body->m_accel += m_internalForces[index].m_linear;
		body->m_alpha += m_internalForces[index].m_angular;
	
		body->m_index = body->m_rank;
		body->m_rank = 0;
	}
	
	m_skeleton->ClearCloseLoopJoints();
}