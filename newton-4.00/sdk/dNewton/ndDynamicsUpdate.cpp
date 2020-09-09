/* Copyright (c) <2003-2019> <Julio Jerez, Newton Game Dynamics>
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

#include "ndNewtonStdafx.h"
#include "ndWorld.h"
#include "ndBodyDynamic.h"
#include "ndDynamicsUpdate.h"


ndDynamicsUpdate::ndDynamicsUpdate()
	:m_internalForces()
	,m_jointArray()
	,m_bodyProxyArray()
	,m_leftHandSide()
	,m_rightHandSide()
	,m_timestep(dFloat32 (0.0f))
	,m_invTimestep(dFloat32(0.0f))
	,m_solverPasses(0)
	,m_maxRowsCount(0)
	,m_rowsCount(0)
{
}

ndDynamicsUpdate::~ndDynamicsUpdate()
{
}

void ndDynamicsUpdate::DynamicsUpdate()
{
	DefaultUpdate();
}

void ndDynamicsUpdate::DefaultUpdate()
{
	D_TRACKTIME();
	InitWeights();
	InitBodyArray();
	InitJacobianMatrix();
}

void ndDynamicsUpdate::InitWeights()
{
	D_TRACKTIME();
	const ndWorld* const world = (ndWorld*)this;
	const ndScene* const scene = world->GetScene();

	m_timestep = world->m_timestep;
	m_invTimestep = dFloat32 (1.0f) / m_timestep;
	const dArray<ndContact*>& contactArray = scene->GetActiveContacts();
	const dArray<ndBodyKinematic*>& bodyArray = scene->GetWorkingBodyArray();

	if (m_bodyProxyArray.GetCapacity() < 256)
	{
		m_jointArray.Resize(256);
		m_leftHandSide.Resize(256);
		m_internalForces.Resize(256);
		m_rightHandSide.Resize(256);
		m_bodyProxyArray.Resize(256);
	}

	const dInt32 bodyCount = bodyArray.GetCount();
	const dInt32 jointCount = contactArray.GetCount();

	m_bodyProxyArray.SetCount(bodyCount);
	m_internalForces.SetCount(bodyCount);
	m_jointArray.SetCount(jointCount);

	memset(&m_internalForces[0], 0, bodyArray.GetCount() * sizeof(ndJacobian));
	memset(&m_bodyProxyArray[0], 0, bodyArray.GetCount() * sizeof(ndBodyProxy));

	dUnsigned32 maxRowCount = 0;
	for (dInt32 i = contactArray.GetCount() - 1; i >= 0; i--) 
	{
		const ndContact* const contact = contactArray[i];
		m_jointArray[i] = (ndConstraint*)contact;
		const ndBodyKinematic* const body0 = contact->GetBody0();
		const ndBodyKinematic* const body1 = contact->GetBody1();
		const dInt32 m0 = body0->GetIndex();
		const dInt32 m1 = body1->GetIndex();
		maxRowCount += contact->GetRowsCount();
		if (body0->GetInvMass() != dFloat32(0.0f))
		{
			m_bodyProxyArray[m0].m_weight = dFloat32(1.0f);
		}
		else
		{
			m_bodyProxyArray[m0].m_weight += dFloat32(1.0f);
		}
		
		if (body1->GetInvMass() != dFloat32(0.0f))
		{
			m_bodyProxyArray[m1].m_weight = dFloat32(1.0f);
		}
		else
		{
			m_bodyProxyArray[m1].m_weight += dFloat32(1.0f);
		}
	}
	m_maxRowsCount = maxRowCount;
	m_leftHandSide.SetCount(maxRowCount);
	m_rightHandSide.SetCount(maxRowCount);
	
	dFloat32 extraPasses = dFloat32(0.0f);

	//dgSkeletonList& skeletonList = *m_world;
	//const dInt32 lru = skeletonList.m_lruMarker;
	//skeletonList.m_lruMarker += 1;
	//m_skeletonCount = 0;

	for (dInt32 i = contactArray.GetCount() - 1; i >= 0; i--) 
	{
		const ndBodyKinematic* const body = bodyArray[i];
		if (body->GetInvMass() != dFloat32(0.0f))
		{
			extraPasses = dMax(m_bodyProxyArray[i].m_weight, extraPasses);
		}
	
	//	dgSkeletonContainer* const container = body->GetSkeleton();
	//	if (container && (container->m_lru != lru)) {
	//		container->m_lru = lru;
	//		m_skeletonArray[m_skeletonCount] = container;
	//		m_skeletonCount++;
	//	}
	}
	const dInt32 conectivity = 7;
	m_solverPasses = world->GetSolverIterations() + 2 * dInt32(extraPasses) / conectivity + 1;
}

void ndDynamicsUpdate::InitBodyArray()
{
	D_TRACKTIME();
	class ndInitBodyArray: public ndScene::ndBaseJob
	{
		public:
		virtual void Execute()
		{
			D_TRACKTIME();

			const dInt32 threadIndex = GetThredID();
			const dArray<ndBodyKinematic*>& bodyArray = m_owner->GetWorkingBodyArray();
			const dInt32 count = bodyArray.GetCount();

			ndWorld* const world = (ndWorld*)m_owner->GetWorld();
			dArray<ndBodyProxy>& bodyProxyArray = world->m_bodyProxyArray;
			
			for (dInt32 i = m_it->fetch_add(1); i < count; i = m_it->fetch_add(1))
			{
				ndBodyDynamic* const body = bodyArray[i]->GetAsBodyDynamic();
				if (body)
				{
					body->AddDampingAcceleration(m_timestep);
					body->UpdateInvInertiaMatrix();
					
					body->SetAccel(body->GetVelocity());
					body->SetAlpha(body->GetOmega());

					const dFloat32 w = bodyProxyArray[i].m_weight;
					bodyProxyArray[i].m_invWeight = dFloat32(1.0f) / w;
				}
			}
		}
	};

	const ndWorld* const world = (ndWorld*)this;
	ndScene* const scene = world->GetScene();
	scene->SubmitJobs<ndInitBodyArray>();
}

dInt32 ndDynamicsUpdate::GetJacobianDerivatives(dInt32 baseIndex, ndConstraint* const joint)
{
	ndConstraintDescritor constraintParam;
	dInt32 dof = joint->GetRowsCount();
	dAssert(dof <= D_CONSTRAINT_MAX_ROWS);
	for (dInt32 i = 0; i < dof; i++) 
	{
		constraintParam.m_forceBounds[i].m_low = D_MIN_BOUND;
		constraintParam.m_forceBounds[i].m_upper = D_MAX_BOUND;
		constraintParam.m_forceBounds[i].m_jointForce = nullptr;
		constraintParam.m_forceBounds[i].m_normalIndex = D_INDEPENDENT_ROW;
	}
	
	constraintParam.m_timestep = m_timestep;
	constraintParam.m_invTimestep = m_invTimestep;
	dof = joint->JacobianDerivative(constraintParam);
	
	//if (constraint->GetId() == dgConstraint::m_contactConstraint) 
	//{
	//	dgContact* const contactJoint = (dgContact*)constraint;
	//	contactJoint->m_isInSkeletonLoop = false;
	//	dgSkeletonContainer* const skeleton0 = body0->GetSkeleton();
	//	dgSkeletonContainer* const skeleton1 = body1->GetSkeleton();
	//	if (skeleton0 && (skeleton0 == skeleton1)) 
	//	{
	//		if (contactJoint->IsSkeletonSelftCollision()) 
	//		{
	//			contactJoint->m_isInSkeletonLoop = true;
	//			skeleton0->AddSelfCollisionJoint(contactJoint);
	//		}
	//	}
	//	else if (contactJoint->IsSkeletonIntraCollision()) 
	//	{
	//		if (skeleton0 && !skeleton1) 
	//		{
	//			contactJoint->m_isInSkeletonLoop = true;
	//			skeleton0->AddSelfCollisionJoint(contactJoint);
	//		}
	//		else if (skeleton1 && !skeleton0) 
	//		{
	//			contactJoint->m_isInSkeletonLoop = true;
	//			skeleton1->AddSelfCollisionJoint(contactJoint);
	//		}
	//	}
	//}
	//else if (constraint->IsBilateral() && !constraint->m_isInSkeleton && (constraint->m_solverModel == 3)) 
	//{
	//	dgSkeletonContainer* const skeleton0 = body0->GetSkeleton();
	//	dgSkeletonContainer* const skeleton1 = body1->GetSkeleton();
	//	if (skeleton0 || skeleton1) 
	//	{
	//		if (skeleton0 && !skeleton1) 
	//		{
	//			constraint->m_isInSkeletonLoop = true;
	//			skeleton0->AddSelfCollisionJoint(constraint);
	//		}
	//		else if (skeleton1 && !skeleton0) 
	//		{
	//			constraint->m_isInSkeletonLoop = true;
	//			skeleton1->AddSelfCollisionJoint(constraint);
	//		}
	//	}
	//}
	
	//jointInfo->m_pairCount = dof;
	//jointInfo->m_pairStart = rowCount;
	joint->m_rowCount = dof;
	joint->m_rowStart = baseIndex;
	for (dInt32 i = 0; i < dof; i++) 
	{
		dAssert(constraintParam.m_forceBounds[i].m_jointForce);
	
		ndLeftHandSide* const row = &m_leftHandSide[baseIndex];
		ndRightHandSide* const rhs = &m_rightHandSide[baseIndex];
	
		row->m_Jt = constraintParam.m_jacobian[i];
		rhs->m_diagDamp = dFloat32(0.0f);
		rhs->m_diagonalRegularizer = dClamp(constraintParam.m_diagonalRegularizer[i], dFloat32(1.0e-5f), dFloat32(1.0f));
		rhs->m_coordenateAccel = constraintParam.m_jointAccel[i];
		rhs->m_restitution = constraintParam.m_restitution[i];
		rhs->m_penetration = constraintParam.m_penetration[i];
		rhs->m_penetrationStiffness = constraintParam.m_penetrationStiffness[i];
		rhs->m_lowerBoundFrictionCoefficent = constraintParam.m_forceBounds[i].m_low;
		rhs->m_upperBoundFrictionCoefficent = constraintParam.m_forceBounds[i].m_upper;
		rhs->m_jointFeebackForce = constraintParam.m_forceBounds[i].m_jointForce;
	
		dAssert(constraintParam.m_forceBounds[i].m_normalIndex >= -1);
		rhs->m_normalForceIndex = constraintParam.m_forceBounds[i].m_normalIndex;
		baseIndex++;
	}
	return baseIndex;
}

//void ndDynamicsUpdate::BuildJacobianMatrix(dgJointInfo* const jointInfo, ndLeftHandSide* const leftHandSide, ndRightHandSide* const rightHandSide, ndJacobian* const internalForces)
void ndDynamicsUpdate::BuildJacobianMatrix(ndConstraint* const joint)
{
	ndBodyKinematic* const body0 = joint->GetKinematicBody0();
	ndBodyKinematic* const body1 = joint->GetKinematicBody1();
	dAssert(body0);
	dAssert(body1);
	const ndBodyDynamic* const dynBody0 = body0->GetAsBodyDynamic();
	const ndBodyDynamic* const dynBody1 = body1->GetAsBodyDynamic();

	const dInt32 m0 = body0->m_index;
	const dInt32 m1 = body1->m_index;
	const dInt32 index = joint->m_rowStart;
	const dInt32 count = joint->m_rowCount;
	const bool isBilateral = joint->IsBilateral();
	
	const dMatrix invInertia0 (body0->m_invWorldInertiaMatrix);
	const dMatrix invInertia1 (body1->m_invWorldInertiaMatrix);
	const dVector invMass0(body0->m_invMass[3]);
	const dVector invMass1(body1->m_invMass[3]);
	
	dVector force0(dVector::m_zero);
	dVector torque0(dVector::m_zero);
	if (dynBody0) 
	{
		force0 = dynBody0->m_externalForce;
		torque0 = dynBody0->m_externalTorque;
	}
	
	dVector force1(dVector::m_zero);
	dVector torque1(dVector::m_zero);
	if (dynBody1)
	{
		force1 = dynBody1->m_externalForce;
		torque1 = dynBody1->m_externalTorque;
	}
	
	joint->m_preconditioner0 = dFloat32(1.0f);
	joint->m_preconditioner1 = dFloat32(1.0f);
	if ((invMass0.GetScalar() > dFloat32(0.0f)) && (invMass1.GetScalar() > dFloat32(0.0f)) && !(body0->GetSkeleton() && body1->GetSkeleton())) 
	{
		const dFloat32 mass0 = body0->GetMassMatrix().m_w;
		const dFloat32 mass1 = body1->GetMassMatrix().m_w;
		if (mass0 > (D_DIAGONAL_PRECONDITIONER * mass1)) 
		{
			joint->m_preconditioner0 = mass0 / (mass1 * D_DIAGONAL_PRECONDITIONER);
		}
		else if (mass1 > (D_DIAGONAL_PRECONDITIONER * mass0)) 
		{
			joint->m_preconditioner1 = mass1 / (mass0 * D_DIAGONAL_PRECONDITIONER);
		}
	}
	
	dVector forceAcc0(dVector::m_zero);
	dVector torqueAcc0(dVector::m_zero);
	dVector forceAcc1(dVector::m_zero);
	dVector torqueAcc1(dVector::m_zero);
	
	const dVector weight0(m_bodyProxyArray[m0].m_weight * joint->m_preconditioner0);
	const dVector weight1(m_bodyProxyArray[m1].m_weight * joint->m_preconditioner0);
	
	const dFloat32 forceImpulseScale = dFloat32(1.0f);
	const dFloat32 preconditioner0 = joint->m_preconditioner0;
	const dFloat32 preconditioner1 = joint->m_preconditioner1;
	
	for (dInt32 i = 0; i < count; i++) 
	{
		ndLeftHandSide* const row = &m_leftHandSide[index + i];
		ndRightHandSide* const rhs = &m_rightHandSide[index + i];
	
		row->m_JMinv.m_jacobianM0.m_linear = row->m_Jt.m_jacobianM0.m_linear * invMass0;
		row->m_JMinv.m_jacobianM0.m_angular = invInertia0.RotateVector(row->m_Jt.m_jacobianM0.m_angular);
		row->m_JMinv.m_jacobianM1.m_linear = row->m_Jt.m_jacobianM1.m_linear * invMass1;
		row->m_JMinv.m_jacobianM1.m_angular = invInertia1.RotateVector(row->m_Jt.m_jacobianM1.m_angular);
	
		const ndJacobian& JMinvM0 = row->m_JMinv.m_jacobianM0;
		const ndJacobian& JMinvM1 = row->m_JMinv.m_jacobianM1;
		const dVector tmpAccel(
			JMinvM0.m_linear * force0 + JMinvM0.m_angular * torque0 +
			JMinvM1.m_linear * force1 + JMinvM1.m_angular * torque1);
	
		dFloat32 extenalAcceleration = -tmpAccel.AddHorizontal().GetScalar();
		rhs->m_deltaAccel = extenalAcceleration * forceImpulseScale;
		rhs->m_coordenateAccel += extenalAcceleration * forceImpulseScale;
		dAssert(rhs->m_jointFeebackForce);
		const dFloat32 force = rhs->m_jointFeebackForce->GetInitiailGuess() * forceImpulseScale;
		//const dFloat32 force = rhs->m_jointFeebackForce->m_force * forceImpulseScale;
	
		rhs->m_force = isBilateral ? dClamp(force, rhs->m_lowerBoundFrictionCoefficent, rhs->m_upperBoundFrictionCoefficent) : force;
		rhs->m_maxImpact = dFloat32(0.0f);
	
		//const dgSoaFloat& JtM0 = (dgSoaFloat&)row->m_Jt.m_jacobianM0;
		//const dgSoaFloat& JtM1 = (dgSoaFloat&)row->m_Jt.m_jacobianM1;
		//const dgSoaFloat tmpDiag((weight0 * JMinvM0 * JtM0).MulAdd(weight1, JMinvM1 * JtM1));
		const ndJacobian& JtM0 = row->m_Jt.m_jacobianM0;
		const ndJacobian& JtM1 = row->m_Jt.m_jacobianM1;
		const dVector tmpDiag(
			weight0 * (JMinvM0.m_linear * JtM0.m_linear + JMinvM0.m_angular * JtM0.m_angular) +
			weight1 * (JMinvM1.m_linear * JtM1.m_linear + JMinvM1.m_angular * JtM1.m_angular));
	
		dFloat32 diag = tmpDiag.AddHorizontal().GetScalar();
		dAssert(diag > dFloat32(0.0f));
		rhs->m_diagDamp = diag * rhs->m_diagonalRegularizer;
		diag *= (dFloat32(1.0f) + rhs->m_diagonalRegularizer);
		rhs->m_invJinvMJt = dFloat32(1.0f) / diag;
	
		dVector f0(rhs->m_force * preconditioner0);
		dVector f1(rhs->m_force * preconditioner1);
		//forceAcc0 = forceAcc0.MulAdd(JtM0, f0);
		//forceAcc1 = forceAcc1.MulAdd(JtM1, f1);
		forceAcc0 = forceAcc0 + JtM0.m_linear * f0;
		torqueAcc0 = torqueAcc0 + JtM0.m_angular * f0;
		forceAcc1 = forceAcc1 + JtM1.m_linear * f1;
		torqueAcc1 = torqueAcc1 + JtM1.m_angular * f1;
	}
	
	if (body0->GetInvMass() > dFloat32 (0.0f)) 
	{
		//dgSoaFloat& out = (dgSoaFloat&)internalForces[m0];
		//dgScopeSpinPause lock(&m_bodyProxyArray[m0].m_lock);
		//out = out + forceAcc0;
		ndJacobian& out = m_internalForces[m0];
		dScopeSpinLock lock(body0->m_lock);
		out.m_linear += forceAcc0;
		out.m_angular += torqueAcc0;
	}

	dAssert(body1->GetInvMass() > dFloat32(0.0f));
	//if (m1) 
	{
		//dgSoaFloat& out = (dgSoaFloat&)internalForces[m1];
		//dgScopeSpinPause lock(&m_bodyProxyArray[m1].m_lock);
		//out = out + forceAcc1;
		ndJacobian& out = m_internalForces[m1];
		dScopeSpinLock lock(body1->m_lock);
		out.m_linear += forceAcc1;
		out.m_angular += torqueAcc1;
	}
}

void ndDynamicsUpdate::InitJacobianMatrix()
{
	D_TRACKTIME();
	class ndInitJacobianMatrix : public ndScene::ndBaseJob
	{
		public:
		virtual void Execute()
		{
			D_TRACKTIME();

			const dInt32 threadIndex = GetThredID();
			ndWorld* const world = m_owner->GetWorld();
			const dArray<ndConstraint*>& jointArray = world->m_jointArray;

			dAtomic<dUnsigned32>& rowCount = world->m_rowsCount;
			for (dInt32 i = m_it->fetch_add(1); i < jointArray.GetCount(); i = m_it->fetch_add(1))
			{
				ndConstraint* const joint = jointArray[i];
				const dUnsigned32 rowBase = rowCount.fetch_add(joint->GetRowsCount());
				dAssert(rowCount.load() <= world->m_maxRowsCount);
				world->GetJacobianDerivatives(rowBase, joint);
				world->BuildJacobianMatrix(joint);
			}
		}
	};

	m_rowsCount.store(0);
	const ndWorld* const world = (ndWorld*)this;
	ndScene* const scene = world->GetScene();
	scene->SubmitJobs<ndInitJacobianMatrix>();
}