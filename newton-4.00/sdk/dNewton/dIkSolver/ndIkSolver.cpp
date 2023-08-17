/* Copyright (c) <2003-2022> <Julio Jerez, Newton Game Dynamics>
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
	,m_contacts(32)
	,m_bodies(32)
	,m_savedBodiesIndex(32)
	,m_leftHandSide(128)
	,m_rightHandSide(128)
	,m_world(nullptr)
	,m_skeleton(nullptr)
	,m_timestep(ndFloat32(0.0f))
	,m_invTimestep(ndFloat32(0.0f))
	,m_maxAccel(ndFloat32(1.0e3f))
	,m_maxAlpha(ndFloat32(1.0e4f))
{
}

ndIkSolver::~ndIkSolver()
{
}

void ndIkSolver::SetMaxAccel(ndFloat32 maxAccel, ndFloat32 maxAlpha)
{
	m_maxAlpha = ndAbs(maxAlpha);
	m_maxAccel = ndAbs(maxAccel);
}

void ndIkSolver::GetJacobianDerivatives(ndConstraint* const joint)
{
	ndConstraintDescritor constraintParam;
	ndAssert(joint->GetRowsCount() <= D_CONSTRAINT_MAX_ROWS);
	for (ndInt32 i = ndInt32(joint->GetRowsCount() - 1); i >= 0; --i)
	{
		constraintParam.m_forceBounds[i].m_low = D_MIN_BOUND;
		constraintParam.m_forceBounds[i].m_upper = D_MAX_BOUND;
		constraintParam.m_forceBounds[i].m_jointForce = nullptr;
		constraintParam.m_forceBounds[i].m_normalIndex = D_INDEPENDENT_ROW;
	}
	joint->m_rowCount = ndInt32(joint->GetRowsCount());

	constraintParam.m_rowsCount = 0;
	constraintParam.m_timestep = m_timestep;
	constraintParam.m_invTimestep = m_invTimestep;
	joint->JacobianDerivative(constraintParam);
	const ndInt32 dof = constraintParam.m_rowsCount;
	ndAssert(dof <= joint->m_rowCount);

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
		else
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
		ndAssert(bilareral);
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
	ndAssert(baseIndex == m_leftHandSide.GetCount());
	ndAssert(baseIndex == m_leftHandSide.GetCount());
	for (ndInt32 i = 0; i < dof; ++i)
	{
		ndAssert(constraintParam.m_forceBounds[i].m_jointForce);

		m_leftHandSide.PushBack(ndLeftHandSide());
		m_rightHandSide.PushBack(ndRightHandSide());
		ndLeftHandSide* const row = &m_leftHandSide[baseIndex + i];
		ndRightHandSide* const rhs = &m_rightHandSide[baseIndex + i];

		row->m_Jt = constraintParam.m_jacobian[i];
		rhs->m_diagDamp = ndFloat32(0.0f);
		rhs->m_diagonalRegularizer = ndMax(constraintParam.m_diagonalRegularizer[i], ndFloat32(1.0e-5f));

		rhs->m_coordenateAccel = constraintParam.m_jointAccel[i];
		rhs->m_restitution = constraintParam.m_restitution[i];
		rhs->m_penetration = constraintParam.m_penetration[i];
		rhs->m_penetrationStiffness = constraintParam.m_penetrationStiffness[i];
		rhs->m_lowerBoundFrictionCoefficent = constraintParam.m_forceBounds[i].m_low;
		rhs->m_upperBoundFrictionCoefficent = constraintParam.m_forceBounds[i].m_upper;
		rhs->m_jointFeebackForce = constraintParam.m_forceBounds[i].m_jointForce;

		ndAssert(constraintParam.m_forceBounds[i].m_normalIndex >= -1);
		const ndInt32 frictionIndex = constraintParam.m_forceBounds[i].m_normalIndex;
		const ndInt32 mask = frictionIndex >> 31;
		rhs->m_normalForceIndex = frictionIndex;
		rhs->m_normalForceIndexFlat = ~mask & (frictionIndex + baseIndex);
	}
}

void ndIkSolver::UpdateJointAcceleration(ndConstraint* const joint)
{
	ndConstraintDescritor constraintParam;
	ndAssert(joint->GetRowsCount() <= D_CONSTRAINT_MAX_ROWS);
	for (ndInt32 i = ndInt32(joint->GetRowsCount() - 1); i >= 0; --i)
	{
		constraintParam.m_forceBounds[i].m_low = D_MIN_BOUND;
		constraintParam.m_forceBounds[i].m_upper = D_MAX_BOUND;
		constraintParam.m_forceBounds[i].m_jointForce = nullptr;
		constraintParam.m_forceBounds[i].m_normalIndex = D_INDEPENDENT_ROW;
	}
	ndAssert(!joint->GetAsContact());
	const ndInt32 dof = joint->m_rowCount;
	joint->m_rowCount = ndInt32(joint->GetRowsCount());
	constraintParam.m_rowsCount = 0;
	constraintParam.m_timestep = m_timestep;
	constraintParam.m_invTimestep = m_invTimestep;
	joint->JacobianDerivative(constraintParam);

	ndAssert(dof <= joint->m_rowCount);
	ndAssert(dof == constraintParam.m_rowsCount);
	joint->m_rowCount = dof;
	
	ndAssert(joint->GetAsBilateral());
	ndAssert(joint->GetAsBilateral()->GetSolverModel() != m_jointkinematicAttachment);
	ndAssert(joint->GetAsBilateral()->GetBody0()->GetSkeleton() == m_skeleton);
	ndAssert(joint->GetAsBilateral()->GetBody1()->GetSkeleton() == m_skeleton);
	
	const ndInt32 baseIndex = joint->m_rowStart;
	for (ndInt32 i = 0; i < dof; ++i)
	{
		ndAssert(constraintParam.m_forceBounds[i].m_jointForce);
		ndRightHandSide* const rhs = &m_rightHandSide[baseIndex + i];
		rhs->m_coordenateAccel = constraintParam.m_jointAccel[i];
	}
}

void ndIkSolver::BuildJacobianMatrix (ndConstraint* const joint)
{
	ndAssert(joint->GetBody0());
	ndAssert(joint->GetBody1());
	const ndBodyKinematic* const body0 = joint->GetBody0();
	const ndBodyKinematic* const body1 = joint->GetBody1();
	
	const ndInt32 index = joint->m_rowStart;
	const ndInt32 count = joint->m_rowCount;
	const ndMatrix& invInertia0 = body0->m_invWorldInertiaMatrix;
	const ndMatrix& invInertia1 = body1->m_invWorldInertiaMatrix;
	const ndVector invMass0(body0->m_invMass[3]);
	const ndVector invMass1(body1->m_invMass[3]);
	const ndFloat32 diagDampScale = joint->GetAsContact() ? ndFloat32(0.1f) : ndFloat32(1.0f);

	const ndVector force0(body0->GetForce());
	const ndVector torque0(body0->GetTorque());
	const ndVector force1(body1->GetForce());
	const ndVector torque1(body1->GetTorque());

	for (ndInt32 i = 0; i < count; ++i)
	{
		ndLeftHandSide* const row = &m_leftHandSide[index + i];
		ndRightHandSide* const rhs = &m_rightHandSide[index + i];

		row->m_JMinv.m_jacobianM0.m_linear = row->m_Jt.m_jacobianM0.m_linear * invMass0;
		row->m_JMinv.m_jacobianM0.m_angular = invInertia0.RotateVector(row->m_Jt.m_jacobianM0.m_angular);
		row->m_JMinv.m_jacobianM1.m_linear = row->m_Jt.m_jacobianM1.m_linear * invMass1;
		row->m_JMinv.m_jacobianM1.m_angular = invInertia1.RotateVector(row->m_Jt.m_jacobianM1.m_angular);

		const ndJacobian& JtM0 = row->m_Jt.m_jacobianM0;
		const ndJacobian& JtM1 = row->m_Jt.m_jacobianM1;
		const ndJacobian& JMinvM0 = row->m_JMinv.m_jacobianM0;
		const ndJacobian& JMinvM1 = row->m_JMinv.m_jacobianM1;

		rhs->m_force = ndFloat32(0.0f);
		const ndVector tmpDiag(
			JMinvM0.m_linear * JtM0.m_linear + JMinvM0.m_angular * JtM0.m_angular +
			JMinvM1.m_linear * JtM1.m_linear + JMinvM1.m_angular * JtM1.m_angular);
		
		ndAssert(tmpDiag.AddHorizontal().GetScalar() * diagDampScale > ndFloat32(0.0f));
		rhs->m_diagDamp = tmpDiag.AddHorizontal().GetScalar() * diagDampScale * rhs->m_diagonalRegularizer;
	}
}

bool ndIkSolver::IsSleeping(ndSkeletonContainer* const skeleton) const
{
	return skeleton->m_isResting ? true : false;
}

ndVector ndIkSolver::GetBodyForce(const ndBodyKinematic* const body) const
{
	ndAssert(0);
	return body->m_accel;
}

ndVector ndIkSolver::GetBodyTorque(const ndBodyKinematic* const body) const
{
	ndAssert(0);
	return body->m_alpha;
}

void ndIkSolver::BuildMassMatrix()
{
	m_bodies.SetCount(0);
	m_contacts.SetCount(0);
	m_leftHandSide.SetCount(0);
	m_rightHandSide.SetCount(0);
	//ndFixSizeArray<ndContact*, 256> contacts;
	
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
		ndAssert(body->m_buildSkelIndex == 0);
		if (body->GetInvMass() > ndFloat32(0.0f))
		{
			m_bodies.PushBack(body);
			body->m_buildSkelIndex = -1;
		}
	}

	// add close loop 
	const ndInt32 loopCount = m_skeleton->m_dynamicsLoopCount + m_skeleton->m_loopCount;
	for (ndInt32 i = 0; i < loopCount; ++i)
	{
		ndConstraint* const joint = m_skeleton->m_loopingJoints[i];
		ndBodyKinematic* const body0 = joint->GetBody0();
		ndBodyKinematic* const body1 = joint->GetBody1();

		ndAssert(body0->GetInvMass() > ndFloat32(0.0f));
		if (body0->m_buildSkelIndex == 0)
		{
			m_bodies.PushBack(body0);
			body0->m_buildSkelIndex = -1;
		}

		if ((body1->m_buildSkelIndex == 0) && (body1->GetInvMass() > ndFloat32(0.0f)))
		{
			m_bodies.PushBack(body1);
			body1->m_buildSkelIndex = -1;
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
				for (ndInt32 j = m_contacts.GetCount() - 1; j >= 0; --j)
				{
					duplicate = duplicate || (m_contacts[j] == contact);
				}
				if (!duplicate)
				{
					m_contacts.PushBack(contact);
					ndBodyKinematic* const body0 = contact->GetBody0();
					ndBodyKinematic* const body1 = contact->GetBody1();
					ndAssert(body0->GetInvMass() > ndFloat32(0.0f));

					if (body0->m_buildSkelIndex == 0)
					{
						m_bodies.PushBack(body0);
						body0->m_buildSkelIndex = -1;
					}

					if ((body1->m_buildSkelIndex == 0) && (body1->GetInvMass() > ndFloat32(0.0f)))
					{
						m_bodies.PushBack(body1);
						body1->m_buildSkelIndex = -1;
					}
				}
			}
		}
	}
	
	m_savedBodiesIndex.SetCount(m_bodies.GetCount());
	for (ndInt32 i = m_bodies.GetCount() - 1; i >= 0 ; --i)
	{
		ndBodyKinematic* const body = m_bodies[i];
		m_savedBodiesIndex[i] = body->m_index;
		body->m_index = i;
	
		body->UpdateInvInertiaMatrix();
		const ndVector gyroTorque(body->m_omega.CrossProduct(body->CalculateAngularMomentum()));
		body->m_gyroTorque = gyroTorque;
	}
	
	for (ndInt32 i = m_skeleton->m_nodeList.GetCount() - 2; i >= 0; --i)
	{
		ndSkeletonContainer::ndNode* const node = m_skeleton->m_nodesOrder[i];
		ndJointBilateralConstraint* const joint = node->m_joint;
		GetJacobianDerivatives(joint);
		BuildJacobianMatrix(joint);
	}
	
	const ndInt32 loops = m_skeleton->m_dynamicsLoopCount + m_skeleton->m_loopCount;
	for (ndInt32 i = 0; i < loops; ++i)
	{
		ndConstraint* const joint = m_skeleton->m_loopingJoints[i];
		GetJacobianDerivatives(joint);
		BuildJacobianMatrix(joint);
	}
	
	for (ndInt32 i = 0; i < m_contacts.GetCount(); ++i)
	{
		ndContact* const contact = m_contacts[i];
		GetJacobianDerivatives(contact);
		BuildJacobianMatrix(contact);
	}
	m_skeleton->InitMassMatrix(&m_leftHandSide[0], &m_rightHandSide[0]);
}

void ndIkSolver::SolverBegin(ndSkeletonContainer* const skeleton, ndJointBilateralConstraint* const* joints, ndInt32 jointCount, ndWorld* const world, ndFloat32 timestep)
{
	m_world = world;
	m_skeleton = skeleton;
	m_timestep = timestep;
	m_invTimestep = ndFloat32(1.0f) / m_timestep;

	m_skeleton->ClearCloseLoopJoints();
	for (ndInt32 i = jointCount - 1; i >= 0; --i)
	{
		m_skeleton->AddCloseLoopJoint((ndConstraint*)joints[i]);
	}
	
	for (ndInt32 i = m_skeleton->m_nodeList.GetCount() - 2; i >= 0; --i)
	{
		ndSkeletonContainer::ndNode* const node = m_skeleton->m_nodesOrder[i];
		ndJointBilateralConstraint* const joint = node->m_joint;
		joint->SetIkMode(true);
	}
	BuildMassMatrix();
}

void ndIkSolver::SolverEnd()
{
	if (m_skeleton)
	{
		for (ndInt32 i = m_bodies.GetCount()-1; i >= 1; --i)
		{
			ndBodyKinematic* const body = m_bodies[i];
			body->m_buildSkelIndex = 0;
			body->m_index = m_savedBodiesIndex[i];
		}
		
		m_skeleton->ClearCloseLoopJoints();
	}
}

void ndIkSolver::Solve()
{
	if (m_skeleton)
	{
		const ndVector zero(ndVector::m_zero);
		for (ndInt32 i = m_bodies.GetCount() - 1; i >= 0; --i)
		{
			ndBodyKinematic* const body = m_bodies[i];
			body->m_accel = body->GetForce();
			body->m_alpha = body->GetTorque() - body->GetGyroTorque();
		}
		for (ndInt32 i = m_contacts.GetCount() - 1; i >= 0; --i)
		{
			ndContact* const contact = m_contacts[i];
			ndBodyKinematic* const body1 = contact->GetBody1();
			if (body1->GetInvMass() > ndFloat32 (0.0f)) 
			{
				ndBodyKinematic* const body0 = contact->GetBody0();
				ndAssert(((body0->GetSkeleton() == m_skeleton) && (body1->GetSkeleton() != m_skeleton)) ||
						 ((body0->GetSkeleton() != m_skeleton) && (body1->GetSkeleton() == m_skeleton)));
				ndBodyKinematic* const body = (body0->GetSkeleton() != m_skeleton) ? body0 : body1;

				//auto AddContactForce = [contact](ndBodyKinematic* const body)
				//{
				//	ndBodyKinematic::ndContactMap& contactMap = body->GetContactMap();
				//	ndBodyKinematic::ndContactMap::Iterator it(contactMap);
				//	for (it.Begin(); it; it++)
				//	{
				//		ndContact* const fronterContact = it.GetNode()->GetInfo();
				//		if (fronterContact->IsActive() && (fronterContact != contact))
				//		{
				//
				//		}
				//	}
				//};

				ndBodyKinematic::ndContactMap& contactMap = body->GetContactMap();
				ndBodyKinematic::ndContactMap::Iterator it(contactMap);
				for (it.Begin(); it; it++)
				{
					ndContact* const fronterContact = it.GetNode()->GetInfo();
					if (fronterContact->IsActive() && (fronterContact != contact))
					{
						if (body == fronterContact->GetBody0())
						{
							body->m_accel += fronterContact->GetForceBody0();
							body->m_alpha += fronterContact->GetTorqueBody0();
						}
						else
						{
							ndAssert(body == fronterContact->GetBody1());
							body->m_accel += fronterContact->GetForceBody1();
							body->m_alpha += fronterContact->GetTorqueBody1();
						}
					}
				}
			}
		}

		m_skeleton->SolveImmediate(*this);

		for (ndInt32 i = m_skeleton->m_nodeList.GetCount() - 2; i >= 0; --i)
		{
			ndSkeletonContainer::ndNode* const node = m_skeleton->m_nodesOrder[i];
			ndJointBilateralConstraint* const joint = node->m_joint;
			ndJacobian accel0;
			ndJacobian accel1;
			accel0.m_linear = joint->GetBody0()->m_accel;
			accel0.m_angular = joint->GetBody0()->m_alpha;
			accel1.m_linear = joint->GetBody1()->m_accel;
			accel1.m_angular = joint->GetBody1()->m_alpha;
			joint->SetIkSetAccel(accel0, accel1);
			joint->SetIkMode(false);
		}
	}
}
