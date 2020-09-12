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
	:m_islands()
	,m_bodyIslands()
	,m_internalForces()
	,m_internalForcesBack()
	,m_jointArray()
	,m_bodyProxyArray()
	,m_leftHandSide()
	,m_rightHandSide()
	,m_timestep(dFloat32 (0.0f))
	,m_invTimestep(dFloat32(0.0f))
	,m_firstPassCoef(dFloat32(0.0f))
	,m_invStepRK(dFloat32(0.0f))
	,m_timestepRK(dFloat32(0.0f))
	,m_invTimestepRK(dFloat32(0.0f))
	,m_solverPasses(0)
	,m_maxRowsCount(0)
	,m_rowsCount(0)
{
	memset(m_accelNorm, 0, sizeof (m_accelNorm));
	memset(m_hasJointFeeback, 0, sizeof(m_hasJointFeeback));
}

ndDynamicsUpdate::~ndDynamicsUpdate()
{
	Clear();
}

void ndDynamicsUpdate::Clear()
{
	m_islands.Resize(0);
	m_bodyIslands.Resize(0);
	m_jointArray.Resize(0);
	m_leftHandSide.Resize(0);
	m_internalForces.Resize(0);
	m_rightHandSide.Resize(0);
	m_bodyProxyArray.Resize(0);
	m_internalForcesBack.Resize(0);
}

inline ndBodyKinematic* ndDynamicsUpdate::FindRootAndSplit(ndBodyKinematic* const body)
{
	ndBodyKinematic* node = body;
	while (node->m_islandParent != node)
	{
		ndBodyKinematic* const prev = node;
		node = node->m_islandParent;
		prev->m_islandParent = node->m_islandParent;
	}
	return node;
}

dInt32 ndDynamicsUpdate::CompareIslandBodies(const ndBodyIndexPair* const  pairA, const ndBodyIndexPair* const pairB, void* const context)
{
	if (pairA->m_root->m_uniqueID > pairB->m_root->m_uniqueID)
	{
		return 1;
	}
	else if (pairA->m_root->m_uniqueID < pairB->m_root->m_uniqueID)
	{
		return -1;
	}
	return 0;
}

dInt32 ndDynamicsUpdate::CompareIslands(const ndIsland* const islandA, const ndIsland* const islandB, void* const context)
{
	if (islandA->m_count < islandB->m_count)
	{
		return 1;
	}
	else if (islandA->m_count > islandB->m_count)
	{
		return -1;
	}
	return 0;
}

void ndDynamicsUpdate::BuildIsland()
{
	D_TRACKTIME();
	const ndScene* const scene = m_world->GetScene();
	const dArray<ndContact*>& contactArray = scene->GetActiveContacts();
	for (dInt32 i = contactArray.GetCount() - 1; i >= 0; i--)
	{
		ndConstraint* const joint = contactArray[i];
		ndBodyKinematic* const body0 = joint->GetKinematicBody0();
		if (body0->GetInvMass() > dFloat32(0.0f))
		{
			ndBodyKinematic* const body1 = joint->GetKinematicBody1();
			ndBodyKinematic* root0 = FindRootAndSplit(body0);
			ndBodyKinematic* root1 = FindRootAndSplit(body1);
			if (root0 != root1)
			{
				if (root0->m_rank < root1->m_rank)
				{
					dSwap(root0, root1);
				}
				root1->m_islandParent = root0;
				if (root0->m_rank == root1->m_rank)
				{
					root0->m_rank += 1;
					dAssert(root0->m_rank <= 6);
				}
			}
		}
	}

	// re use body array and working buffer 
	const dArray<ndBodyKinematic*>& bodyArray = scene->GetWorkingBodyArray();
	m_internalForces.SetCount(bodyArray.GetCount());

	dUnsigned32 count = 0;
	dUnsigned32 clusterCount = 0;
	ndBodyIndexPair* const buffer = (ndBodyIndexPair*)&m_internalForces[0];

	for (dInt32 i = bodyArray.GetCount() - 1; i >= 0; i--)
	{
		ndBodyKinematic* const body = bodyArray[i];
		if (body->GetInvMass() > dFloat32(0.0f))
		{
			ndBodyKinematic* root = body->m_islandParent;
			while (root != root->m_islandParent)
			{
				root = root->m_islandParent;
			}
			buffer[count].m_body = body;
			buffer[count].m_root = root;
			count++;
			if (root->m_rank != -1)
			{
				clusterCount++;
				root->m_rank = -1;
			}
		}
	}

	dSort(buffer, count, CompareIslandBodies);
	if (m_bodyIslands.GetCapacity() < 256)
	{
		m_islands.Resize(256);
		m_bodyIslands.Resize(256);
	}

	m_islands.SetCount(0);
	m_bodyIslands.SetCount(count);
	for (dUnsigned32 i = 0; i < count; i++)
	{
		m_bodyIslands[i] = buffer[i].m_body;
		if (buffer[i].m_root->m_rank == -1)
		{
			buffer[i].m_root->m_rank = 0;
			ndIsland island(buffer[i].m_root);
			m_islands.PushBack(island);
		}
		buffer[i].m_root->m_rank += 1;
	}

	dInt32 start = 0;
	for (dInt32 i = 0; i < m_islands.GetCount(); i++)
	{
		m_islands[i].m_start = start;
		m_islands[i].m_count = m_islands[i].m_root->m_rank;
		start += m_islands[i].m_count;
	}

	dSort(&m_islands[0], m_islands.GetCount(), CompareIslands);
}

void ndDynamicsUpdate::DynamicsUpdate()
{
	m_world = (ndWorld*)this;
	BuildIsland();
	DefaultUpdate();
	DetermineSleepStates();
}

void ndDynamicsUpdate::DefaultUpdate()
{
	D_TRACKTIME();
	InitWeights();
	InitBodyArray();
	InitJacobianMatrix();
	CalculateForces();
}

void ndDynamicsUpdate::InitWeights()
{
	D_TRACKTIME();
	const ndScene* const scene = m_world->GetScene();

	m_timestep = m_world->m_timestep;
	m_invTimestep = dFloat32 (1.0f) / m_timestep;

	m_invStepRK = dFloat32(0.25f);
	m_timestepRK = m_timestep * m_invStepRK;
	m_invTimestepRK = m_invTimestep * dFloat32(4.0f);

	const dArray<ndContact*>& contactArray = scene->GetActiveContacts();
	const dArray<ndBodyKinematic*>& bodyArray = scene->GetWorkingBodyArray();

	if (m_bodyProxyArray.GetCapacity() < 256)
	{
		m_jointArray.Resize(256);
		m_leftHandSide.Resize(256);
		m_internalForces.Resize(256);
		m_rightHandSide.Resize(256);
		m_bodyProxyArray.Resize(256);
		m_internalForcesBack.Resize(256);
	}

	const dInt32 bodyCount = bodyArray.GetCount();
	const dInt32 jointCount = contactArray.GetCount();

	m_jointArray.SetCount(jointCount);
	m_bodyProxyArray.SetCount(bodyCount);
	m_internalForces.SetCount(bodyCount);
	m_internalForcesBack.SetCount(bodyCount);

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
	m_solverPasses = m_world->GetSolverIterations() + 2 * dInt32(extraPasses) / conectivity + 1;
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

	ndScene* const scene = m_world->GetScene();
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
		ndJacobian& out = m_internalForces[m0];
		dScopeSpinLock lock(body0->m_lock);
		out.m_linear += forceAcc0;
		out.m_angular += torqueAcc0;
	}

	dAssert(body1->GetInvMass() > dFloat32(0.0f));
	//if (m1) 
	{
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
	ndScene* const scene = m_world->GetScene();
	scene->SubmitJobs<ndInitJacobianMatrix>();
}

void ndDynamicsUpdate::CalculateForces()
{
	D_TRACKTIME();
	m_firstPassCoef = dFloat32(0.0f);
	//if (m_skeletonCount) 
	//{
	//	InitSkeletons();
	//}

	for (dInt32 step = 0; step < 4; step++) 
	{
		CalculateJointsAcceleration();
		CalculateJointsForce();
		//if (m_skeletonCount) 
		//{
		//	UpdateSkeletons();
		//}
		IntegrateBodiesVelocity();
	}
	
	dInt32 hasJointFeeback = 0;
	UpdateForceFeedback();
	for (dInt32 i = 0; i < m_world->GetThreadCount(); i++)
	{
		hasJointFeeback |= m_hasJointFeeback[i];
	}
	IntegrateBodies();

	if (hasJointFeeback) 
	{
		dAssert(0);
	//	UpdateKinematicFeedback();
	}
}

void ndDynamicsUpdate::CalculateJointsAcceleration()
{
	D_TRACKTIME();
	class ndCalculateJointsAcceleration: public ndScene::ndBaseJob
	{
		public:
		virtual void Execute()
		{
			D_TRACKTIME();

			ndWorld* const world = m_owner->GetWorld();
			const dArray<ndConstraint*>& jointArray = world->m_jointArray;

			ndJointAccelerationDecriptor joindDesc;
			joindDesc.m_timeStep = world->m_timestepRK;
			joindDesc.m_invTimeStep = world->m_invTimestepRK;
			joindDesc.m_firstPassCoefFlag = world->m_firstPassCoef;
			dArray<ndLeftHandSide>& leftHandSide = world->m_leftHandSide;
			dArray<ndRightHandSide>& rightHandSide = world->m_rightHandSide;
			
			for (dInt32 i = m_it->fetch_add(1); i < jointArray.GetCount(); i = m_it->fetch_add(1))
			{
				ndConstraint* const joint = jointArray[i];
				const dInt32 pairStart = joint->m_rowStart;
				joindDesc.m_rowsCount = joint->m_rowCount;
				joindDesc.m_leftHandSide = &leftHandSide[pairStart];
				joindDesc.m_rightHandSide = &rightHandSide[pairStart];
				joint->JointAccelerations(&joindDesc);
			}
		}
	};

	ndScene* const scene = m_world->GetScene();
	scene->SubmitJobs<ndCalculateJointsAcceleration>();
	m_firstPassCoef = dFloat32(1.0f);
}

void ndDynamicsUpdate::CalculateJointsForce()
{
	D_TRACKTIME();
	class ndCalculateJointsForce : public ndScene::ndBaseJob
	{
		public:
		virtual void Execute()
		{
			D_TRACKTIME();

			ndWorld* const world = m_owner->GetWorld();
			dArray<ndConstraint*>& jointArray = world->m_jointArray;
			
			dFloat32 accNorm = dFloat32 (0.0f);
			for (dInt32 i = m_it->fetch_add(1); i < jointArray.GetCount(); i = m_it->fetch_add(1))
			{
				ndConstraint* const joint = jointArray[i];
				accNorm += world->CalculateJointsForce(joint);
			}
			const dInt32 threadIndex = GetThredID();
			world->m_accelNorm[threadIndex] = accNorm;
		}
	};

	ndScene* const scene = m_world->GetScene();
	const dInt32 bodyCount = m_internalForces.GetCount();
	const dInt32 passes = m_solverPasses;
	const dInt32 threadCounts = scene->GetThreadCount();
	dAssert(bodyCount == scene->GetWorkingBodyArray().GetCount());

	dFloat32 accNorm = D_SOLVER_MAX_ERROR * dFloat32(2.0f);
	for (dInt32 i = 0; (i < passes) && (accNorm > D_SOLVER_MAX_ERROR); i++) 
	{
		memset(&m_internalForcesBack[0], 0, bodyCount * sizeof(ndJacobian));
		scene->SubmitJobs<ndCalculateJointsForce>();
		memcpy(&m_internalForces[0], &m_internalForcesBack[0], bodyCount * sizeof(ndJacobian));

		for (dInt32 j = 0; j < threadCounts; j++) 
		{
			accNorm = dMax(accNorm, m_accelNorm[j]);
		}
	}
}

dFloat32 ndDynamicsUpdate::CalculateJointsForce(ndConstraint* const joint)
{
	dVector accNorm(dVector::m_zero);
	dFloat32 normalForce[D_CONSTRAINT_MAX_ROWS + 1];

	ndBodyKinematic* const body0 = joint->GetKinematicBody0();
	ndBodyKinematic* const body1 = joint->GetKinematicBody1();
	dAssert(body0);
	dAssert(body1);

	const dInt32 m0 = body0->m_index;
	const dInt32 m1 = body1->m_index;
	const dInt32 rowsCount = joint->m_rowCount;
	const dInt32 rowStart = joint->m_rowStart;

	dInt32 isSleeping = body0->m_resting & body1->m_resting;
	if (!isSleeping) 
	{
		dVector preconditioner0(joint->m_preconditioner0);
		dVector preconditioner1(joint->m_preconditioner1);
		
		dVector forceM0(m_internalForces[m0].m_linear * preconditioner0);
		dVector torqueM0(m_internalForces[m0].m_angular * preconditioner0);
		dVector forceM1(m_internalForces[m1].m_linear * preconditioner1);
		dVector torqueM1(m_internalForces[m1].m_angular * preconditioner1);
		
		preconditioner0 = preconditioner0.Scale(m_bodyProxyArray[m0].m_weight);
		preconditioner1 = preconditioner1.Scale(m_bodyProxyArray[m1].m_weight);
		
		normalForce[0] = dFloat32(1.0f);
		for (dInt32 j = 0; j < rowsCount; j++) 
		{
			ndRightHandSide* const rhs = &m_rightHandSide[rowStart + j];
			const ndLeftHandSide* const lhs = &m_leftHandSide[rowStart + j];
			dVector a(lhs->m_JMinv.m_jacobianM0.m_linear * forceM0);
			a = a.MulAdd(lhs->m_JMinv.m_jacobianM0.m_angular, torqueM0);
			a = a.MulAdd(lhs->m_JMinv.m_jacobianM1.m_linear, forceM1);
			a = a.MulAdd(lhs->m_JMinv.m_jacobianM1.m_angular, torqueM1);
			//a = dVector(rhs->m_coordenateAccel + rhs->m_gyroAccel - rhs->m_force * rhs->m_diagDamp) - a.AddHorizontal();
			a = dVector(rhs->m_coordenateAccel - rhs->m_force * rhs->m_diagDamp) - a.AddHorizontal();
			dVector f(rhs->m_force + rhs->m_invJinvMJt * a.GetScalar());
		
			dAssert(rhs->m_normalForceIndex >= -1);
			dAssert(rhs->m_normalForceIndex <= rowsCount);
		
			const dInt32 frictionIndex = rhs->m_normalForceIndex + 1;
			const dFloat32 frictionNormal = normalForce[frictionIndex];
			const dVector lowerFrictionForce(frictionNormal * rhs->m_lowerBoundFrictionCoefficent);
			const dVector upperFrictionForce(frictionNormal * rhs->m_upperBoundFrictionCoefficent);
		
			a = a & (f < upperFrictionForce) & (f > lowerFrictionForce);
			f = f.GetMax(lowerFrictionForce).GetMin(upperFrictionForce);
			accNorm = accNorm.MulAdd(a, a);
		
			dVector deltaForce(f - dVector(rhs->m_force));
		
			rhs->m_force = f.GetScalar();
			normalForce[j + 1] = f.GetScalar();
		
			dVector deltaForce0(deltaForce * preconditioner0);
			dVector deltaForce1(deltaForce * preconditioner1);
		
			forceM0 = forceM0.MulAdd(lhs->m_Jt.m_jacobianM0.m_linear, deltaForce0);
			torqueM0 = torqueM0.MulAdd(lhs->m_Jt.m_jacobianM0.m_angular, deltaForce0);
			forceM1 = forceM1.MulAdd(lhs->m_Jt.m_jacobianM1.m_linear, deltaForce1);
			torqueM1 = torqueM1.MulAdd(lhs->m_Jt.m_jacobianM1.m_angular, deltaForce1);
		}
		
		const dFloat32 tol = dFloat32(0.5f);
		const dFloat32 tol2 = tol * tol;

		dVector maxAccel(accNorm);
		for (dInt32 k = 0; (k < 4) && (maxAccel.GetScalar() > tol2); k++) 
		{
			maxAccel = dVector::m_zero;
			for (dInt32 j = 0; j < rowsCount; j++) 
			{
				ndRightHandSide* const rhs = &m_rightHandSide[rowStart + j];
				const ndLeftHandSide* const lhs = &m_leftHandSide[rowStart + j];
		
				dVector a(lhs->m_JMinv.m_jacobianM0.m_linear * forceM0);
				a = a.MulAdd(lhs->m_JMinv.m_jacobianM0.m_angular, torqueM0);
				a = a.MulAdd(lhs->m_JMinv.m_jacobianM1.m_linear, forceM1);
				a = a.MulAdd(lhs->m_JMinv.m_jacobianM1.m_angular, torqueM1);
				//a = dVector(rhs->m_coordenateAccel + rhs->m_gyroAccel - rhs->m_force * rhs->m_diagDamp) - a.AddHorizontal();
				a = dVector(rhs->m_coordenateAccel - rhs->m_force * rhs->m_diagDamp) - a.AddHorizontal();
				dVector f(rhs->m_force + rhs->m_invJinvMJt * a.GetScalar());
		
				dAssert(rhs->m_normalForceIndex >= -1);
				dAssert(rhs->m_normalForceIndex <= rowsCount);
		
				const dInt32 frictionIndex = rhs->m_normalForceIndex + 1;
				const dFloat32 frictionNormal = normalForce[frictionIndex];
				const dVector lowerFrictionForce(frictionNormal * rhs->m_lowerBoundFrictionCoefficent);
				const dVector upperFrictionForce(frictionNormal * rhs->m_upperBoundFrictionCoefficent);
		
				a = a & (f < upperFrictionForce) & (f > lowerFrictionForce);
				f = f.GetMax(lowerFrictionForce).GetMin(upperFrictionForce);
				maxAccel = maxAccel.MulAdd(a, a);
		
				dVector deltaForce(f - rhs->m_force);
		
				rhs->m_force = f.GetScalar();
				normalForce[j + 1] = f.GetScalar();
		
				dVector deltaForce0(deltaForce * preconditioner0);
				dVector deltaForce1(deltaForce * preconditioner1);
				forceM0 = forceM0.MulAdd(lhs->m_Jt.m_jacobianM0.m_linear, deltaForce0);
				torqueM0 = torqueM0.MulAdd(lhs->m_Jt.m_jacobianM0.m_angular, deltaForce0);
				forceM1 = forceM1.MulAdd(lhs->m_Jt.m_jacobianM1.m_linear, deltaForce1);
				torqueM1 = torqueM1.MulAdd(lhs->m_Jt.m_jacobianM1.m_angular, deltaForce1);
			}
		}
	}
		
	dVector forceM0(dVector::m_zero);
	dVector torqueM0(dVector::m_zero);
	dVector forceM1(dVector::m_zero);
	dVector torqueM1(dVector::m_zero);
		
	for (dInt32 j = 0; j < rowsCount; j++)
	{
		const ndRightHandSide* const rhs = &m_rightHandSide[rowStart + j];
		const ndLeftHandSide* const lhs = &m_leftHandSide[rowStart + j];
		
		dVector f(rhs->m_force);
		forceM0 = forceM0.MulAdd(lhs->m_Jt.m_jacobianM0.m_linear, f);
		torqueM0 = torqueM0.MulAdd(lhs->m_Jt.m_jacobianM0.m_angular, f);
		forceM1 = forceM1.MulAdd(lhs->m_Jt.m_jacobianM1.m_linear, f);
		torqueM1 = torqueM1.MulAdd(lhs->m_Jt.m_jacobianM1.m_angular, f);
	}
		
	//if (m0) 
	if (body0->GetInvMass() > dFloat32(0.0f))
	{
		dScopeSpinLock lock(body0->m_lock);
		m_internalForcesBack[m0].m_linear += forceM0;
		m_internalForcesBack[m0].m_angular += torqueM0;
	}

	//if (m1) 
	{
		dScopeSpinLock lock(body1->m_lock);
		m_internalForcesBack[m1].m_linear += forceM1;
		m_internalForcesBack[m1].m_angular += torqueM1;
	}
	return accNorm.GetScalar();
}

void ndDynamicsUpdate::IntegrateBodiesVelocity()
{
	D_TRACKTIME();
	class ndIntegrateBodiesVelocity : public ndScene::ndBaseJob
	{
		public:
		virtual void Execute()
		{
			D_TRACKTIME();

			const dArray<ndBodyKinematic*>& bodyArray = m_owner->GetWorkingBodyArray();
			const dInt32 bodyCount = bodyArray.GetCount();

			ndWorld* const world = (ndWorld*)m_owner->GetWorld();
			const dArray<ndJacobian>& internalForces = world->m_internalForces;

			dVector speedFreeze2(world->m_freezeSpeed2 * dFloat32(0.1f));

			dVector timestep4(world->m_timestepRK);
			for (dInt32 i = m_it->fetch_add(1); i < bodyCount; i = m_it->fetch_add(1))
			{
				ndBodyKinematic* const body = bodyArray[i];
				ndBodyDynamic* const dynBody = body->GetAsBodyDynamic();
				
				if (dynBody && dynBody->GetInvMass() > dFloat32 (0.0f))
				{
					const ndJacobian& forceAndTorque = internalForces[i];
					const dVector force(dynBody->GetForce() + forceAndTorque.m_linear);
					const dVector torque(dynBody->GetToque() + forceAndTorque.m_angular);
					const ndJacobian velocStep(dynBody->IntegrateForceAndToque(force, torque, timestep4));
				
					if (!body->m_resting) 
					{
						body->m_veloc += velocStep.m_linear;
						body->m_omega += velocStep.m_angular;
					}
					else 
					{
						dAssert(0);
						const dVector velocStep2(velocStep.m_linear.DotProduct(velocStep.m_linear));
						const dVector omegaStep2(velocStep.m_angular.DotProduct(velocStep.m_angular));
						const dVector test(((velocStep2 > speedFreeze2) | (omegaStep2 > speedFreeze2)) & dVector::m_negOne);
						const dInt32 equilibrium = test.GetSignMask() ? 0 : 1;
						body->m_resting &= equilibrium;
					}
					dAssert(body->m_veloc.m_w == dFloat32(0.0f));
					dAssert(body->m_omega.m_w == dFloat32(0.0f));
				}
			}
		}
	};

	ndScene* const scene = m_world->GetScene();
	scene->SubmitJobs<ndIntegrateBodiesVelocity>();
}

void ndDynamicsUpdate::UpdateForceFeedback()
{
	D_TRACKTIME();
	class ndUpdateForceFeedback : public ndScene::ndBaseJob
	{
		public:
		virtual void Execute()
		{
			D_TRACKTIME();

			ndWorld* const world = m_owner->GetWorld();
			dArray<ndConstraint*>& jointArray = world->m_jointArray;
			ndRightHandSide* const rightHandSide = &world->m_rightHandSide[0];

			bool hasJointFeeback = false;
			dFloat32 timestepRK = world->m_timestepRK;
			for (dInt32 i = m_it->fetch_add(1); i < jointArray.GetCount(); i = m_it->fetch_add(1))
			{
				ndConstraint* const joint = jointArray[i];
				const dInt32 first = joint->m_rowStart;
				const dInt32 count = joint->m_rowCount;

				for (dInt32 j = 0; j < count; j++) 
				{
					const ndRightHandSide* const rhs = &rightHandSide[j + first];
					dAssert(dCheckFloat(rhs->m_force));
					rhs->m_jointFeebackForce->Push(rhs->m_force);
					rhs->m_jointFeebackForce->m_force = rhs->m_force;
					rhs->m_jointFeebackForce->m_impact = rhs->m_maxImpact * timestepRK;
				}
				//hasJointFeeback |= (joint->m_updaFeedbackCallback ? 1 : 0);
				hasJointFeeback |= joint->m_jointFeebackForce;
			}
			const dInt32 threadIndex = GetThredID();
			world->m_hasJointFeeback[threadIndex] = hasJointFeeback ? 1 : 0;
		}
	};
	
	ndScene* const scene = m_world->GetScene();
	scene->SubmitJobs<ndUpdateForceFeedback>();
}

void ndDynamicsUpdate::IntegrateBodies()
{
	D_TRACKTIME();
	class ndIntegrateBodies: public ndScene::ndBaseJob
	{
		public:
		virtual void Execute()
		{
			D_TRACKTIME();

			const dArray<ndBodyKinematic*>& bodyArray = m_owner->GetWorkingBodyArray();
			const dInt32 bodyCount = bodyArray.GetCount();

			ndWorld* const world = (ndWorld*)m_owner->GetWorld();
			dVector invTime(world->m_invTimestep);
			dFloat32 maxAccNorm2 = D_SOLVER_MAX_ERROR * D_SOLVER_MAX_ERROR;
			
			dFloat32 timestep = world->m_timestep;
			for (dInt32 i = m_it->fetch_add(1); i < bodyCount; i = m_it->fetch_add(1))
			{
				ndBodyKinematic* const body = bodyArray[i];
				ndBodyDynamic* const dynBody = body->GetAsBodyDynamic();

				if (dynBody && dynBody->GetInvMass() > dFloat32(0.0f))
				{
					// the initial velocity and angular velocity were stored in m_accel and body->m_alpha for memory saving
					dVector accel(invTime * (dynBody->m_veloc - dynBody->m_accel));
					dVector alpha(invTime * (dynBody->m_omega - dynBody->m_alpha));
					dVector accelTest((accel.DotProduct(accel) > maxAccNorm2) | (alpha.DotProduct(alpha) > maxAccNorm2));
					dynBody->m_accel = accel & accelTest;
					dynBody->m_alpha = alpha & accelTest;

					dVector isMovingMask(dynBody->m_veloc + dynBody->m_omega + dynBody->m_accel + dynBody->m_alpha);
					dVector mask (isMovingMask.TestZero());
					dInt32 test = mask.GetSignMask() & 0x07;
					if (test != 7)
					{
						body->IntegrateVelocity(timestep);
					}
				}
			}
		}
	};

	ndScene* const scene = m_world->GetScene();
	scene->SubmitJobs<ndIntegrateBodies>();
}

void ndDynamicsUpdate::DetermineSleepStates()
{
	D_TRACKTIME();
	class ndDetermineSleepStates : public ndScene::ndBaseJob
	{
		public:
		virtual void Execute()
		{
			D_TRACKTIME();
			ndWorld* const world = (ndWorld*)m_owner->GetWorld();
			const dArray<ndIsland>& islands = world->m_islands;
			const dInt32 islandsCount = islands.GetCount();
			for (dInt32 i = m_it->fetch_add(1); i < islandsCount; i = m_it->fetch_add(1))
			{
				const ndIsland& island = islands[i];
				world->UpdateIslandState(island);
			}
		}
	};

	ndScene* const scene = m_world->GetScene();
	scene->SubmitJobs<ndDetermineSleepStates>();
}

void ndDynamicsUpdate::UpdateIslandState(const ndIsland& island)
{
	for (dInt32 i = 0; i < island.m_count; i++)
	{

	}
}