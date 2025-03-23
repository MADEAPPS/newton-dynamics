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
	,m_contacts(32)
	,m_bodies(32)
	,m_savedBodiesIndex(32)
	,m_leftHandSide(128)
	,m_rightHandSide(128)
	,m_surrogateContact(32)
	,m_surrogateBodies(32)
	,m_world(nullptr)
	,m_skeleton(nullptr)
	,m_timestep(ndFloat32(0.0f))
	,m_invTimestep(ndFloat32(0.0f))
	,m_maxAccel(ndFloat32(1.0e3f))
	,m_maxAlpha(ndFloat32(1.0e4f))
{
	ndBodyDynamic* const sentinelBody = new ndBodyDynamic;
	m_surrogateBodies.PushBack(sentinelBody);
	sentinelBody->m_uniqueId = 0;
}

ndIkSolver::~ndIkSolver()
{
	for (ndInt32 i = ndInt32(m_surrogateBodies.GetCount()) - 1; i >= 0; --i)
	{
		delete m_surrogateBodies[i];
	}

	for (ndInt32 i = ndInt32(m_surrogateContact.GetCount()) - 1; i >= 0; --i)
	{
		delete m_surrogateContact[i];
	}
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
	joint->m_rowStart = ndInt32(m_leftHandSide.GetCount());
	const ndInt32 baseIndex = joint->m_rowStart;
	ndAssert(baseIndex == ndInt32(m_leftHandSide.GetCount()));
	ndAssert(baseIndex == ndInt32(m_leftHandSide.GetCount()));
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

		rhs->SetSanityCheck(joint);
		ndAssert(rhs->SanityCheck());
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

	//const ndVector force0(body0->GetForce());
	//const ndVector torque0(body0->GetTorque());
	//const ndVector force1(body1->GetForce());
	//const ndVector torque1(body1->GetTorque());

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

void ndIkSolver::BuildMassMatrix()
{
	m_bodies.SetCount(0);
	m_contacts.SetCount(0);
	m_leftHandSide.SetCount(0);
	m_rightHandSide.SetCount(0);
	
	const ndVector zero(ndVector::m_zero);
	m_bodies.PushBack(m_surrogateBodies[0]);
	m_surrogateBodies[0]->m_index = 0;
	m_surrogateBodies[0]->m_accel = zero;
	m_surrogateBodies[0]->m_alpha = zero;
	
	auto CopyOpenLoopBody = [this](ndBodyKinematic* const body)
	{
		if (body->GetInvMass() > ndFloat32(0.0f))
		{
			ndAssert(body->GetId() > 0);
			ndAssert(body->GetInvMass() > ndFloat32(0.0f));
			ndUnsigned32 id = body->GetId();

			m_bodies.PushBack(body);
			for (ndInt32 i = ndInt32(m_bodies.GetCount()) - 1; i >= 1; --i)
			{
				if (id > m_bodies[i - 1]->GetId())
				{
					m_bodies[i] = body;
					break;
				}
				m_bodies[i] = m_bodies[i - 1];
			}
		}
	};

	// add open loop bodies
	for (ndInt32 i = 0; i < m_skeleton->m_nodeList.GetCount(); ++i)
	{
		ndSkeletonContainer::ndNode* const node = m_skeleton->m_nodesOrder[i];
		ndBodyKinematic* const body = node->m_body;
		CopyOpenLoopBody(body);
	}

	auto CopyCloseLoopBody = [this](ndBodyKinematic* const body)
	{
		ndAssert(body->GetId() > 0);
		if (body->GetInvMass() == ndFloat32(0.0f))
		{
			return;
		}

		for (ndInt32 i = ndInt32(m_bodies.GetCount()) - 1; i >= 1; --i)
		{
			if (body == m_bodies[i])
			{
				return;
			}
		}

		ndUnsigned32 id = body->GetId();
		m_bodies.PushBack(body);
		for (ndInt32 i = ndInt32(m_bodies.GetCount()) - 1; i >= 1; --i)
		{
			if (id > m_bodies[i - 1]->GetId())
			{
				m_bodies[i] = body;
				break;
			}
			m_bodies[i] = m_bodies[i - 1];
		}
	};

	// add closed loop 
	for (ndInt32 i = ndInt32 (m_skeleton->m_permanentLoopingJoints.GetCount() - 1); i >= 0; --i)
	{
		ndConstraint* const joint = m_skeleton->m_permanentLoopingJoints[i];
		ndBodyKinematic* const body0 = joint->GetBody0();
		ndBodyKinematic* const body1 = joint->GetBody1();

		ndAssert(body0->GetInvMass() > ndFloat32(0.0f));
		CopyCloseLoopBody(body0);
		CopyCloseLoopBody(body1);
	}

	// add contacts loop bodies and joints
	class ndBodyPair
	{
		public:
		ndBodyKinematic* m_body;
		ndBodyKinematic* m_surrogateBody;
	};
	ndFixSizeArray<ndBodyPair, 256> m_surrogateArray;

	ndInt32 surrogateBodiesIndex = 1;
	ndInt32 surrogateContactIndex = 0;
	for (ndInt32 i = m_skeleton->m_nodeList.GetCount() - 1; i >= 0 ; --i)
	{
		ndSkeletonContainer::ndNode* const node = m_skeleton->m_nodesOrder[i];
		ndBodyKinematic* const body = node->m_body;

		if (body->GetInvMass() > ndFloat32(0.0f))
		{ 
			ndBodyKinematic::ndContactMap& contactMap = body->GetContactMap();
			ndBodyKinematic::ndContactMap::Iterator it(contactMap);

			for (it.Begin(); it; it++)
			{
				ndContact* const contact = it.GetNode()->GetInfo();
				if (contact->IsActive())
				{
					bool duplicate = false;
					for (ndInt32 j = ndInt32(m_contacts.GetCount()) - 1; j >= 0; --j)
					{
						duplicate = duplicate || (m_contacts[j] == contact);
					}
					if (!duplicate)
					{
						auto GetSurrogateBody = [this, &m_surrogateArray, &surrogateBodiesIndex](ndBodyKinematic* const body)
						{
							for (ndInt32 i = m_surrogateArray.GetCount() - 1; i >= 0; --i)
							{
								if (body == m_surrogateArray[i].m_body)
								{
									return m_surrogateArray[i].m_surrogateBody;
								}
							}
							if (surrogateBodiesIndex == ndInt32(m_surrogateBodies.GetCount()))
							{
								ndBodyDynamic* const surrogateBody = new ndBodyDynamic();
								m_surrogateBodies.PushBack(surrogateBody);
							}
							ndBodyKinematic* const surrogateBody = m_surrogateBodies[surrogateBodiesIndex];

							ndBodyPair pair;
							pair.m_body = body;
							pair.m_surrogateBody = surrogateBody;
							m_surrogateArray.PushBack(pair);

							m_bodies.PushBack(surrogateBody);
							surrogateBodiesIndex++;
							body->InitSurrogateBody(surrogateBody);
							return surrogateBody;
						};

						ndBodyKinematic* const body0 = contact->GetBody0()->GetAsBodyDynamic();
						ndBodyKinematic* const body1 = contact->GetBody1()->GetAsBodyDynamic();
						if (body0 == body)
						{
							if (body1->GetInvMass() == ndFloat32(0.0f))
							{
								m_contacts.PushBack(contact);
							}
							else
							{
								ndBodyKinematic* const surrogateBody = GetSurrogateBody(body1);

								if (surrogateContactIndex == ndInt32(m_surrogateContact.GetCount()))
								{
									m_surrogateContact.PushBack(new ndContact);
								}
								ndContact* const surrogateContact = m_surrogateContact[surrogateContactIndex];
								surrogateContactIndex++;
								contact->InitSurrogateContact(surrogateContact, body0, surrogateBody);
								m_contacts.PushBack(surrogateContact);
							}
						}
						else
						{
							ndAssert(body1 == body);
							ndAssert(body1->GetInvMass() > ndFloat32(0.0f));
							ndBodyKinematic* const surrogateBody = GetSurrogateBody(body0);

							if (surrogateContactIndex == ndInt32(m_surrogateContact.GetCount()))
							{
								m_surrogateContact.PushBack(new ndContact);
							}
							ndContact* const surrogateContact = m_surrogateContact[surrogateContactIndex];
							surrogateContactIndex++;
							contact->InitSurrogateContact(surrogateContact, surrogateBody, body1);
							m_contacts.PushBack(surrogateContact);
						}
					}
				}
			}
		}
	}

	m_savedBodiesIndex.SetCount(m_bodies.GetCount());
	for (ndInt32 i = ndInt32(m_bodies.GetCount()) - 1; i >= 1 ; --i)
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
	
	for (ndInt32 i = ndInt32 (m_skeleton->m_permanentLoopingJoints.GetCount() - 1); i >= 0; --i)
	{
		ndConstraint* const joint = m_skeleton->m_permanentLoopingJoints[i];
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
	if (m_skeleton)
	{
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
}

void ndIkSolver::SolverEnd()
{
	if (m_skeleton)
	{
		for (ndInt32 i = ndInt32(m_bodies.GetCount())-1; i >= 1; --i)
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
	ndAssert(m_skeleton);
	if (m_skeleton)
	{
		const ndVector zero(ndVector::m_zero);
		for (ndInt32 i = ndInt32(m_bodies.GetCount()) - 1; i >= 0; --i)
		{
			ndBodyKinematic* const body = m_bodies[i];
			body->m_accel = body->GetForce();
			body->m_alpha = body->GetTorque() - body->GetGyroTorque();
		}
		for (ndInt32 i = ndInt32(m_contacts.GetCount()) - 1; i >= 0; --i)
		{
			ndContact* const contact = m_contacts[i];
			ndBodyKinematic* const body1 = contact->GetBody1();
			if (body1->GetInvMass() > ndFloat32 (0.0f)) 
			{
				ndBodyKinematic* const body0 = contact->GetBody0();
				ndAssert((body0->GetSkeleton() == m_skeleton) || (body1->GetSkeleton() == m_skeleton));

				ndBodyKinematic* const body = (body0->GetSkeleton() != m_skeleton) ? body0 : body1;
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
			ndJacobian accel0;
			ndJacobian accel1;
			const ndSkeletonContainer::ndNode* const node = m_skeleton->m_nodesOrder[i];
			ndJointBilateralConstraint* const joint = node->m_joint;
			const ndBodyKinematic* const body0 = joint->GetBody0();
			const ndBodyKinematic* const body1 = joint->GetBody1();
			accel0.m_linear = body0->m_accel;
			accel0.m_angular = body0->m_alpha;
			accel1.m_linear = body1->m_accel;
			accel1.m_angular = body1->m_alpha;

			#if 0
			ndFloat32 xxx0 = body0->m_omega.DotProduct(body0->m_omega).GetScalar();
			ndFloat32 xxx1 = body1->m_omega.DotProduct(body1->m_omega).GetScalar();
			ndFloat32 xxx2 = body0->m_accel.DotProduct(body0->m_accel).GetScalar();
			ndFloat32 xxx3 = body1->m_alpha.DotProduct(body1->m_alpha).GetScalar();
			if ((xxx0 > 50000.0f) || (xxx1 > 50000.0f) || (xxx2 > 1.0e8f) || (xxx3 > 1.0e8f))
			{
				for (ndInt32 j = ndInt32(m_bodies.GetCount()) - 1; j >= 0; --j)
				{
					ndBodyKinematic* const body = m_bodies[j];
					body->m_accel = body->GetForce();
					body->m_alpha = body->GetTorque() - body->GetGyroTorque();
				}
				m_skeleton->SolveImmediate(*this);
			}
			#endif
			joint->SetIkSetAccel(accel0, accel1);
			joint->SetIkMode(false);
		}
	}
}
