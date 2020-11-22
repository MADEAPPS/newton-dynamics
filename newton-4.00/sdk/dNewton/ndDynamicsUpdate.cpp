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

#include "dCoreStdafx.h"
#include "ndNewtonStdafx.h"
#include "ndWorld.h"
#include "ndBodyDynamic.h"
#include "ndSkeletonList.h"
#include "ndDynamicsUpdate.h"
#include "ndJointBilateralConstraint.h"

#define D_BASH_SIZE	64

//#define D_LOCK_FREE_SOLVER

ndDynamicsUpdate::ndDynamicsUpdate()
	:m_velocTol(dFloat32(1.0e-8f))
	,m_islands(1024)
	,m_bodyIslandOrder(1024)
	,m_internalForces(1024)
	,m_jointArray()
	,m_leftHandSide(1024 * 4)
	,m_rightHandSide(1024)
	,m_timestep(dFloat32 (0.0f))
	,m_invTimestep(dFloat32(0.0f))
	,m_firstPassCoef(dFloat32(0.0f))
	,m_invStepRK(dFloat32(0.0f))
	,m_timestepRK(dFloat32(0.0f))
	,m_invTimestepRK(dFloat32(0.0f))
	,m_solverPasses(0)
	,m_maxRowsCount(0)
	,m_unConstrainedBodyCount(0)
	,m_rowsCount(0)
{
	//memset(m_accelNorm, 0, sizeof (m_accelNorm));
	memset(m_hasJointFeeback, 0, sizeof(m_hasJointFeeback));
}

ndDynamicsUpdate::~ndDynamicsUpdate()
{
	Clear();
}

void ndDynamicsUpdate::Clear()
{
	m_islands.Resize(0);
	m_jointArray.Resize(0);
	m_leftHandSide.Resize(0);
	m_rightHandSide.Resize(0);
	m_internalForces.Resize(0);
	m_bodyIslandOrder.Resize(0);
}

void ndDynamicsUpdate::Update()
{
	m_world = (ndWorld*)this;
	m_timestep = m_world->GetScene()->GetTimestep();

	BuildIsland();
	if (m_islands.GetCount())
	{
		IntegrateUnconstrainedBodies();
		DefaultUpdate();
		DetermineSleepStates();
	}
}

void ndDynamicsUpdate::DefaultUpdate()
{
	D_TRACKTIME();
	InitWeights();
	InitBodyArray();
	InitJacobianMatrix();
	CalculateForces();
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

dInt32 ndDynamicsUpdate::CompareIslands(const ndIsland* const islandA, const ndIsland* const islandB, void* const context)
{
	dUnsigned32 keyA = islandA->m_count * 2 + islandA->m_root->m_bodyIsConstrained;
	dUnsigned32 keyB = islandB->m_count * 2 + islandB->m_root->m_bodyIsConstrained;;
	if (keyA < keyB)
	{
		return 1;
	}
	else if (keyA > keyB)
	{
		return -1;
	}
	return 0;
}

#ifdef D_USE_STANDARD_SORT
dInt32 ndDynamicsUpdate::CompareIslandBodies(const ndBodyIndexPair* const  pairA, const ndBodyIndexPair* const pairB, void* const context)
{
	union dKey
	{
		dKey(dUnsigned32 low, dUnsigned32 high)
			:m_val((high<<31) + low)
		{
		}
		dUnsigned32 m_val;
	};
	dKey keyA(pairA->m_root->m_uniqueID, pairA->m_root->m_bodyIsConstrained);
	dKey keyB(pairB->m_root->m_uniqueID, pairB->m_root->m_bodyIsConstrained);

	if (keyA.m_val < keyB.m_val)
	{
		return 1;
	}
	else if (keyA.m_val > keyB.m_val)
	{
		return -1;
	}
	return 0;
}
#endif

void ndDynamicsUpdate::BuildIsland()
{
	ndScene* const scene = m_world->GetScene();
	const dArray<ndBodyKinematic*>& bodyArray = scene->GetActiveBodyArray();
	dAssert(bodyArray.GetCount() >= 1);
	if (bodyArray.GetCount() - 1)
	{
		D_TRACKTIME();
		const ndJointList& jointList = m_world->GetJointList();
		ndConstraintArray& jointArray = scene->GetActiveContactArray();

		dInt32 index = jointArray.GetCount();
		jointArray.SetCount(index + jointList.GetCount());
		for (ndJointList::dListNode* node = jointList.GetFirst(); node; node = node->GetNext())
		{
			jointArray[index] = node->GetInfo();
			index++;
		}

		dInt32 rowCount = 0;
		for (dInt32 i = 0; i < jointArray.GetCount(); i ++)
		{
			ndConstraint* const joint = jointArray[i];
			const dInt32 rows = joint->GetRowsCount();
			joint->m_rowCount = rows;
			joint->m_rowStart = rowCount;
			rowCount += rows;

			ndBodyKinematic* const body1 = joint->GetBody1();
			if (body1->GetInvMass() > dFloat32(0.0f))
			{
				ndBodyKinematic* const body0 = joint->GetBody0();

				body0->m_bodyIsConstrained = 1;
				body1->m_bodyIsConstrained = 1;
				const dInt32 resting = body0->m_equilibrium & body1->m_equilibrium;
				body0->m_resting = body0->m_resting & resting;
				body1->m_resting = body1->m_resting & resting;

				ndBodyKinematic* root0 = FindRootAndSplit(body0);
				ndBodyKinematic* root1 = FindRootAndSplit(body1);
				if (root0 != root1)
				{
					if (root0->m_rank > root1->m_rank)
					{
						dSwap(root0, root1);
					}
					root0->m_islandParent = root1;
					if (root0->m_rank == root1->m_rank)
					{
						root1->m_rank += 1;
						dAssert(root1->m_rank <= 6);
					}
				}

				const dInt32 sleep = body0->m_islandSleep & body1->m_islandSleep;
				if (!sleep)
				{
					dAssert(root1->m_islandParent == root1);
					root1->m_islandSleep = 0;
				}
			}
		}

		for (dInt32 i = 0; i < jointArray.GetCount(); i++)
		{
			ndConstraint* const joint = jointArray[i];
			ndBodyKinematic* const body1 = joint->GetBody1();
			if (body1->GetInvMass() == dFloat32(0.0f))
			{
				ndBodyKinematic* const body0 = joint->GetBody0();
				const dInt32 resting = body0->m_equilibrium & body1->m_equilibrium;
				body0->m_bodyIsConstrained = 1;
				body0->m_resting = body0->m_resting & resting;
				if (!body0->m_islandSleep)
				{
					ndBodyKinematic* const root = FindRootAndSplit(body0);
					root->m_islandSleep = 0;
				}
			}
		}


		// re use body array and working buffer 
		m_internalForces.SetCount(bodyArray.GetCount());

		dInt32 count = 0;
		ndBodyIndexPair* const buffer0 = (ndBodyIndexPair*)&m_internalForces[0];
		for (dInt32 i = bodyArray.GetCount() - 2; i >= 0; i--)
		{
			ndBodyKinematic* const body = bodyArray[i];
			if (!(body->m_resting & body->m_islandSleep))
			{
				buffer0[count].m_body = body;
				if (body->GetInvMass() > dFloat32(0.0f))
				{
					ndBodyKinematic* root = body->m_islandParent;
					while (root != root->m_islandParent)
					{
						root = root->m_islandParent;
					}

					buffer0[count].m_root = root;
					if (root->m_rank != -1)
					{
						root->m_rank = -1;
					}
				}
				else
				{
					buffer0[count].m_root = body;
					body->m_rank = -1;
				}
				count++;
			}
		}

		m_islands.SetCount(0);
		m_bodyIslandOrder.SetCount(count);
		m_unConstrainedBodyCount = 0;
		if (count)
		{
			#ifdef D_USE_STANDARD_SORT
			// sort using quick sort o(n * log(n))
			dSort(buffer0, count, CompareIslandBodies);
			const ndBodyIndexPair* const buffer1 = buffer0;
			#else
			// sort using counting sort o(n)
			dInt32 scans[2];
			scans[0] = 0;
			scans[1] = 0;
			for (dInt32 i = 0; i < count; i++)
			{
				dInt32 j = 1 - buffer0[i].m_root->m_bodyIsConstrained;
				scans[j] ++;
			}
			scans[1] = scans[0];
			scans[0] = 0;
			ndBodyIndexPair* const buffer1 = buffer0 + count;
			for (dInt32 i = 0; i < count; i++)
			{
				const dInt32 key = 1 - buffer0[i].m_root->m_bodyIsConstrained;
				const dInt32 j = scans[key];
				buffer1[j] = buffer0[i];
				scans[key] = j + 1;
			}
			#endif

			for (dInt32 i = 0; i < count; i++)
			{
				m_bodyIslandOrder[i] = buffer1[i].m_body;
				if (buffer1[i].m_root->m_rank == -1)
				{
					buffer1[i].m_root->m_rank = 0;
					ndIsland island(buffer1[i].m_root);
					m_islands.PushBack(island);
				}
				buffer1[i].m_root->m_rank += 1;
			}

			dInt32 start = 0;
			dInt32 unConstrainedCount = 0;
			for (dInt32 i = 0; i < m_islands.GetCount(); i++)
			{
				ndIsland& island = m_islands[i];
				island.m_start = start;
				island.m_count = island.m_root->m_rank;
				start += island.m_count;
				unConstrainedCount += island.m_root->m_bodyIsConstrained ? 0 : 1;
			}

			m_unConstrainedBodyCount = unConstrainedCount;
			dSort(&m_islands[0], m_islands.GetCount(), CompareIslands);
		}
	}
}

void ndDynamicsUpdate::IntegrateUnconstrainedBodies()
{
	class ndIntegrateUnconstrainedBodies : public ndScene::ndBaseJob
	{
		public:

		virtual void Execute()
		{
			D_TRACKTIME();
			ndWorld* const world = m_owner->GetWorld();
			dArray<ndBodyKinematic*>& bodyArray = world->m_bodyIslandOrder;

			const dInt32 threadIndex = GetThredID();
			const dInt32 threadCount = m_owner->GetThreadCount();
			const dInt32 bodyCount = world->m_unConstrainedBodyCount;
			const dInt32 base = bodyArray.GetCount() - bodyCount;
			const dInt32 step = bodyCount / threadCount;
			const dInt32 start = threadIndex * step;
			const dInt32 count = ((threadIndex + 1) < threadCount) ? step : bodyCount - start;
			const dFloat32 timestep = m_timestep;

			for (dInt32 i = 0; i < count; i++)
			{
				ndBodyKinematic* const body = bodyArray[base + start + i]->GetAsBodyKinematic();
				dAssert(body);
				body->UpdateInvInertiaMatrix();
				body->AddDampingAcceleration(m_timestep);
				body->IntegrateExternalForce(timestep);
			}
		}
	};

	if (m_unConstrainedBodyCount)
	{
		D_TRACKTIME();
		ndScene* const scene = m_world->GetScene();
		scene->SubmitJobs<ndIntegrateUnconstrainedBodies>();
	}
}

void ndDynamicsUpdate::InitWeights()
{
	D_TRACKTIME();
	const ndScene* const scene = m_world->GetScene();

	m_invTimestep = dFloat32 (1.0f) / m_timestep;
	m_invStepRK = dFloat32(0.25f);
	m_timestepRK = m_timestep * m_invStepRK;
	m_invTimestepRK = m_invTimestep * dFloat32(4.0f);

	const dArray<ndBodyKinematic*>& bodyArray = scene->GetActiveBodyArray();
	const ndConstraintArray& constraintArray = scene->GetActiveContactArray();

	const dInt32 bodyCount = bodyArray.GetCount();
	const dInt32 jointCount = constraintArray.GetCount();

	m_jointArray.SetCount(jointCount);
	#ifdef D_LOCK_FREE_SOLVER
		const dInt32 threadCount = dMax(scene->GetThreadCount(), 2);
		m_internalForces.SetCount(bodyCount * threadCount);
	#else
		m_internalForces.SetCount(bodyCount * 2);
	#endif	

	dUnsigned32 maxRowCount = 0;
	dFloat32 extraPasses = dFloat32(1.0f);
	for (dInt32 i = constraintArray.GetCount() - 1; i >= 0; i--) 
	{
		ndConstraint* const constraint = constraintArray[i];
		m_jointArray[i] = constraint;
		ndBodyKinematic* const body0 = constraint->GetBody0();
		ndBodyKinematic* const body1 = constraint->GetBody1();
		maxRowCount += constraint->GetRowsCount();
		
		if (body1->GetInvMass() == dFloat32(0.0f))
		{
			body1->m_weigh = dFloat32(1.0f);
		}
		else
		{
			body1->m_weigh += dFloat32(1.0f);
			extraPasses = dMax(body1->m_weigh, extraPasses);
		}

		body0->m_weigh += dFloat32(1.0f);
		dAssert(body0->GetInvMass() != dFloat32(0.0f));
		extraPasses = dMax(body0->m_weigh, extraPasses);
	}

	m_maxRowsCount = maxRowCount;
	m_leftHandSide.SetCount(maxRowCount);
	m_rightHandSide.SetCount(maxRowCount);

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
			ndWorld* const world = m_owner->GetWorld();
			dArray<ndBodyKinematic*>& bodyArray = world->m_bodyIslandOrder;

			const dInt32 threadIndex = GetThredID();
			const dInt32 threadCount = m_owner->GetThreadCount();
			const dInt32 bodyCount = bodyArray.GetCount() - world->m_unConstrainedBodyCount;
			const dInt32 step = bodyCount / threadCount;
			const dInt32 start = threadIndex * step;
			const dInt32 count = ((threadIndex + 1) < threadCount) ? step : bodyCount - start;
			const dFloat32 timestep = m_timestep;

			for (dInt32 i = 0; i < count; i++)
			{
				ndBodyKinematic* const body = bodyArray[start + i]->GetAsBodyDynamic();
				ndBodyDynamic* const kinBody = body->GetAsBodyDynamic();
				if (kinBody)
				{
					dAssert(kinBody->m_bodyIsConstrained);
					kinBody->UpdateInvInertiaMatrix();
					kinBody->AddDampingAcceleration(m_timestep);
					kinBody->m_accel = kinBody->m_veloc;
					kinBody->m_alpha = kinBody->m_omega;
				}
			}
		}
	};

	ndScene* const scene = m_world->GetScene();
	scene->SubmitJobs<ndInitBodyArray>();
}

void ndDynamicsUpdate::GetJacobianDerivatives(ndConstraint* const joint)
{
	ndConstraintDescritor constraintParam;
	dAssert(joint->GetRowsCount() <= D_CONSTRAINT_MAX_ROWS);
	for (dInt32 i = joint->GetRowsCount() - 1; i >= 0; i--)
	{
		constraintParam.m_forceBounds[i].m_low = D_MIN_BOUND;
		constraintParam.m_forceBounds[i].m_upper = D_MAX_BOUND;
		constraintParam.m_forceBounds[i].m_jointForce = nullptr;
		constraintParam.m_forceBounds[i].m_normalIndex = D_INDEPENDENT_ROW;
	}
	
	constraintParam.m_rowsCount = 0;
	constraintParam.m_timestep = m_timestep;
	constraintParam.m_invTimestep = m_invTimestep;
	joint->JacobianDerivative(constraintParam);
	const dInt32 dof = constraintParam.m_rowsCount;
	dAssert(dof <= joint->m_rowCount);
	
	//if (constraint->GetId() == dgConstraint::m_contactConstraint) 
	if (joint->GetAsContact())
	{
		ndContact* const contactJoint = joint->GetAsContact();
		contactJoint->m_isInSkeletonLoop = false;
		ndSkeletonContainer* const skeleton0 = contactJoint->GetBody0()->GetSkeleton();
		ndSkeletonContainer* const skeleton1 = contactJoint->GetBody1()->GetSkeleton();
		if (skeleton0 && (skeleton0 == skeleton1)) 
		{
			if (contactJoint->IsSkeletonSelftCollision()) 
			{
				contactJoint->m_isInSkeletonLoop = true;
				skeleton0->AddSelfCollisionJoint(contactJoint);
			}
		}
		else if (contactJoint->IsSkeletonIntraCollision()) 
		{
			if (skeleton0 && !skeleton1) 
			{
				contactJoint->m_isInSkeletonLoop = true;
				skeleton0->AddSelfCollisionJoint(contactJoint);
			}
			else if (skeleton1 && !skeleton0) 
			{
				contactJoint->m_isInSkeletonLoop = true;
				skeleton1->AddSelfCollisionJoint(contactJoint);
			}
		}
	}
	else
	{ 
		ndJointBilateralConstraint* const bilareral = joint->GetAsBilateral();
		dAssert(bilareral);
		if (!bilareral->m_isInSkeleton && (bilareral->m_solverModel == 3))
		{
			ndSkeletonContainer* const skeleton0 = bilareral->m_body0->GetSkeleton();
			ndSkeletonContainer* const skeleton1 = bilareral->m_body1->GetSkeleton();
			if (skeleton0 || skeleton1) 
			{
				if (skeleton0 && !skeleton1) 
				{
					bilareral->m_isInSkeletonLoop = true;
					skeleton0->AddSelfCollisionJoint(bilareral);
				}
				else if (skeleton1 && !skeleton0) 
				{
					bilareral->m_isInSkeletonLoop = true;
					skeleton1->AddSelfCollisionJoint(bilareral);
				}
			}
		}
	}
	
	joint->m_rowCount = dof;
	const dInt32 baseIndex = joint->m_rowStart;
	for (dInt32 i = 0; i < dof; i++) 
	{
		dAssert(constraintParam.m_forceBounds[i].m_jointForce);
	
		ndLeftHandSide* const row = &m_leftHandSide[baseIndex + i];
		ndRightHandSide* const rhs = &m_rightHandSide[baseIndex + i];
	
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
	}
}

void ndDynamicsUpdate::BuildJacobianMatrix(ndConstraint* const joint, ndJacobian* const internalForces, bool singleThreaded)
{
	dAssert(joint->GetBody0());
	dAssert(joint->GetBody1());
	ndBodyKinematic* const body0 = joint->GetBody0();
	ndBodyKinematic* const body1 = joint->GetBody1();
	const ndBodyDynamic* const dynBody0 = body0->GetAsBodyDynamic();
	const ndBodyDynamic* const dynBody1 = body1->GetAsBodyDynamic();

	const dInt32 m0 = body0->m_index;
	const dInt32 m1 = body1->m_index;
	const dInt32 index = joint->m_rowStart;
	const dInt32 count = joint->m_rowCount;

	const bool isBilateral = joint->IsBilateral();

	const dMatrix& invInertia0 = body0->m_invWorldInertiaMatrix;
	const dMatrix& invInertia1 = body1->m_invWorldInertiaMatrix;
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

	const dVector weigh0(body0->m_weigh * joint->m_preconditioner0);
	const dVector weigh1(body1->m_weigh * joint->m_preconditioner0);

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

		const ndJacobian& JtM0 = row->m_Jt.m_jacobianM0;
		const ndJacobian& JtM1 = row->m_Jt.m_jacobianM1;
		const dVector tmpDiag(
			weigh0 * (JMinvM0.m_linear * JtM0.m_linear + JMinvM0.m_angular * JtM0.m_angular) +
			weigh1 * (JMinvM1.m_linear * JtM1.m_linear + JMinvM1.m_angular * JtM1.m_angular));

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

	#ifdef D_LOCK_FREE_SOLVER
		ndJacobian& outBody0 = internalForces[m0];
		outBody0.m_linear += forceAcc0;
		outBody0.m_angular += torqueAcc0;

		ndJacobian& outBody1 = internalForces[m1];
		outBody1.m_linear += forceAcc1;
		outBody1.m_angular += torqueAcc1;
	#else
		if (singleThreaded)
		{
			ndJacobian& outBody0 = internalForces[m0];
			outBody0.m_linear += forceAcc0;
			outBody0.m_angular += torqueAcc0;

			ndJacobian& outBody1 = internalForces[m1];
			outBody1.m_linear += forceAcc1;
			outBody1.m_angular += torqueAcc1;
		}
		else
		{
			dAssert(body0->GetInvMass() > dFloat32(0.0f));
			{
				ndJacobian& outBody0 = internalForces[m0];
				dScopeSpinLock lock(body0->m_lock);
				outBody0.m_linear += forceAcc0;
				outBody0.m_angular += torqueAcc0;
			}

			if (body1->GetInvMass() > dFloat32(0.0f))
			{
				ndJacobian& outBody1 = internalForces[m1];
				dScopeSpinLock lock(body1->m_lock);
				outBody1.m_linear += forceAcc1;
				outBody1.m_angular += torqueAcc1;
			}
		}
	#endif
}

void ndDynamicsUpdate::InitJacobianMatrix()
{
	class ndInitJacobianMatrix : public ndScene::ndBaseJob
	{
		public:

		virtual void Execute()
		{
			D_TRACKTIME();

			ndWorld* const world = m_owner->GetWorld();
			ndConstraint** const jointArray = &world->m_jointArray[0];
			const dInt32 jointCount = world->m_jointArray.GetCount();
			const dInt32 threadCount = dMax (m_owner->GetThreadCount(), 1);

			if (threadCount == 1)
			{
				ndJacobian* const internalForces = &world->m_internalForces[0];
				for (dInt32 i = 0; i < jointCount; i++)
				{
					ndConstraint* const joint = jointArray[i];
					world->GetJacobianDerivatives(joint);
					world->BuildJacobianMatrix(joint, internalForces, true);
				}
			}
			else 
			{
				#ifdef D_LOCK_FREE_SOLVER
					const dInt32 threadIndex = GetThredID();
					const dInt32 step = jointCount / threadCount;
					const dInt32 start = threadIndex * step;
					const dInt32 count = ((threadIndex + 1) < threadCount) ? step : jointCount - start;

					const dInt32 bodyCount = m_owner->GetActiveBodyArray().GetCount();
					ndJacobian* const internalForces = &world->m_internalForces[threadIndex * bodyCount];
					memset(internalForces, 0, bodyCount * sizeof(ndJacobian));
					for (dInt32 i = 0; i < count; i++)
					{
						ndConstraint* const joint = jointArray[i + start];
						world->GetJacobianDerivatives(joint);
						world->BuildJacobianMatrix(joint, internalForces, true);
					}

				#else

					const dInt32 threadIndex = GetThredID();
					const dInt32 step = jointCount / threadCount;
					const dInt32 start = threadIndex * step;
					const dInt32 count = ((threadIndex + 1) < threadCount) ? step : jointCount - start;

					ndJacobian* const internalForces = &world->m_internalForces[0];
					for (dInt32 i = 0; i < count; i++)
					{
						ndConstraint* const joint = jointArray[i + start];
						world->GetJacobianDerivatives(joint);
						world->BuildJacobianMatrix(joint, internalForces, false);
					}
				#endif
			}
		}
	};

	class ndInitJacobianAccumulatePartialForces : public ndScene::ndBaseJob
	{
		public:

		virtual void Execute()
		{
			D_TRACKTIME();
			ndWorld* const world = m_owner->GetWorld();
			const dInt32 threadIndex = GetThredID();
			const dInt32 threadCount = m_owner->GetThreadCount();
			const dInt32 bodyCount = m_owner->GetActiveBodyArray().GetCount();
			const dInt32 step = bodyCount / threadCount;
			const dInt32 start = threadIndex * step;
			const dInt32 count = ((threadIndex + 1) < threadCount) ? step : bodyCount - start;

			ndJacobian* const internalForces = &world->m_internalForces[0];
			for (dInt32 i = 0; i < count; i++)
			{
				dVector force(dVector::m_zero);
				dVector torque(dVector::m_zero);
				const dInt32 base = i + start;
				for (dInt32 j = 1; j < threadCount; j++)
				{
					force += internalForces[bodyCount * j + base].m_linear;
					torque += internalForces[bodyCount * j + base].m_angular;
				}
				internalForces[base].m_linear += force;
				internalForces[base].m_angular += torque;
			}
		}
	};

	if (m_jointArray.GetCount())
	{
		D_TRACKTIME();
		m_rowsCount.store(0);
		ndScene* const scene = m_world->GetScene();
		const dArray<ndBodyKinematic*>& bodyArray = scene->GetActiveBodyArray();

		if (scene->GetThreadCount() <= 1)
		{
			memset(&m_internalForces[0], 0, bodyArray.GetCount() * sizeof(ndJacobian));
			scene->SubmitJobs<ndInitJacobianMatrix>();
		}
		else
		{
			#ifdef D_LOCK_FREE_SOLVER
				scene->SubmitJobs<ndInitJacobianMatrix>();
				scene->SubmitJobs<ndInitJacobianAccumulatePartialForces>();
			#else
				memset(&m_internalForces[0], 0, bodyArray.GetCount() * sizeof(ndJacobian));
				scene->SubmitJobs<ndInitJacobianMatrix>();
			#endif
		}
	}
}

void ndDynamicsUpdate::CalculateForces()
{
	D_TRACKTIME();
	dInt32 hasJointFeeback = 0;
	if (m_jointArray.GetCount())
	{
		m_firstPassCoef = dFloat32(0.0f);
		if (m_world->m_skeletonList.GetCount()) 
		{
			InitSkeletons();
		}

		for (dInt32 step = 0; step < 4; step++)
		{
			CalculateJointsAcceleration();
			CalculateJointsForce();
			if (m_world->m_skeletonList.GetCount())
			{
				UpdateSkeletons();
			}
			IntegrateBodiesVelocity();
		}
		
		UpdateForceFeedback();
		for (dInt32 i = 0; i < m_world->GetThreadCount(); i++)
		{
			hasJointFeeback |= m_hasJointFeeback[i];
		}
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
			const ndConstraintArray& jointArray = world->m_jointArray;

			ndJointAccelerationDecriptor joindDesc;
			joindDesc.m_timestep = world->m_timestepRK;
			joindDesc.m_invTimeStep = world->m_invTimestepRK;
			joindDesc.m_firstPassCoefFlag = world->m_firstPassCoef;
			dArray<ndLeftHandSide>& leftHandSide = world->m_leftHandSide;
			dArray<ndRightHandSide>& rightHandSide = world->m_rightHandSide;
			
			const dInt32 threadIndex = GetThredID();
			const dInt32 threadCount = m_owner->GetThreadCount();
			const dInt32 jointCount = jointArray.GetCount();
			const dInt32 step = jointCount / threadCount;
			const dInt32 start = threadIndex * step;
			const dInt32 count = ((threadIndex + 1) < threadCount) ? step : jointCount - start;

			for (dInt32 i = 0; i < count; i++)
			{
				ndConstraint* const joint = jointArray[i + start];
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
		dFloat32 ExecuteBatch(const dInt32 start, const dInt32 count, ndConstraintArray& jointArray, ndJacobian* const internalForces)
		{
			//D_TRACKTIME();
			ndWorld* const world = m_owner->GetWorld();
			dFloat32 accNorm = dFloat32(0.0f);

			for (dInt32 i = 0; i < count; i++)
			{
				ndConstraint* const joint = jointArray[i + start];
				accNorm += world->CalculateJointsForce(joint, internalForces, false);
			}
			return accNorm;
		}

		virtual void Execute()
		{
			D_TRACKTIME();
			ndWorld* const world = m_owner->GetWorld();
			ndConstraintArray& jointArray = world->m_jointArray;
			dFloat32 accNorm = dFloat32 (0.0f);
			const dInt32 jointCount = jointArray.GetCount();
			const dInt32 bodyCount = m_owner->GetActiveBodyArray().GetCount();
			const dInt32 threadCount = dMax(m_owner->GetThreadCount(), 1);
			if (threadCount == 1)
			{
				ndJacobian* const internalForces = &world->m_internalForces[bodyCount];
				for (dInt32 i = 0; i < jointCount; i++)
				{
					ndConstraint* const joint = jointArray[i];
					accNorm += world->CalculateJointsForce(joint, internalForces, true);
				}
				//const dInt32 threadIndex = GetThredID();
				//world->m_accelNorm[threadIndex] = accNorm;
				dFloat32* const accelNorm = (dFloat32*)m_context;
				accelNorm[0] = accNorm;
			}
			else
			{
				#ifdef D_LOCK_FREE_SOLVER___
					dAssert(0);
				#else
					const dInt32 stepSize = D_BASH_SIZE;
					const dInt32 jointCountBatches = jointCount & -stepSize;

					ndJacobian* const internalForces = &world->m_internalForces[bodyCount];
					dInt32 index = m_it->fetch_add(stepSize);
					for (; index < jointCountBatches; index = m_it->fetch_add(stepSize))
					{
						accNorm += ExecuteBatch(index, stepSize, jointArray, internalForces);
					}
					if (index < jointCount)
					{
						accNorm += ExecuteBatch(index, jointCount - index, jointArray, internalForces);
					}

					const dInt32 threadIndex = GetThredID();
					//world->m_accelNorm[threadIndex] = accNorm;
					//const dInt32 threadIndex = GetThredID();
					dFloat32* const accelNorm = (dFloat32*)m_context;
					accelNorm[threadIndex] = accNorm;
				#endif
			}

		}
	};

	ndScene* const scene = m_world->GetScene();
	const dInt32 passes = m_solverPasses;
	const dInt32 bodyCount = scene->GetActiveBodyArray().GetCount();
	const dInt32 threadsCount = dMax (scene->GetThreadCount(), 1);

	dFloat32 m_accelNorm[D_MAX_THREADS_COUNT][D_MAX_THREADS_COUNT];
	dFloat32 accNorm = D_SOLVER_MAX_ERROR * dFloat32(2.0f);
	for (dInt32 i = 0; (i < passes) && (accNorm > D_SOLVER_MAX_ERROR); i++) 
	{
		#ifdef D_LOCK_FREE_SOLVER___
			dAssert(0);
			//if (threadsCount == 1)
			//{
			//	memcpy(&m_internalForces[0], &m_internalForces[bodyCount], bodyCount * sizeof(ndJacobian));
			//}
			//else
			//{
			//	dAssert(0);
			//}

		#else
			memset(&m_internalForces[bodyCount], 0, bodyCount * sizeof(ndJacobian));
			scene->SubmitJobs<ndCalculateJointsForce>(m_accelNorm);
			memcpy(&m_internalForces[0], &m_internalForces[bodyCount], bodyCount * sizeof(ndJacobian));
		#endif

		accNorm = dFloat32(0.0f);
		for (dInt32 j = 0; j < threadsCount; j++) 
		{
			accNorm = dMax(accNorm, m_accelNorm[0][j]);
		}
	}
}

dFloat32 ndDynamicsUpdate::CalculateJointsForce(ndConstraint* const joint, ndJacobian* const internalForces, bool singleThreaded)
{
	dVector accNorm(dVector::m_zero);
	dFloat32 normalForce[D_CONSTRAINT_MAX_ROWS + 1];

	ndBodyKinematic* const body0 = joint->GetBody0();
	ndBodyKinematic* const body1 = joint->GetBody1();
	dAssert(body0);
	dAssert(body1);

	const dInt32 m0 = body0->m_index;
	const dInt32 m1 = body1->m_index;
	const dInt32 rowStart = joint->m_rowStart;
	const dInt32 rowsCount = joint->m_rowCount;

	dInt32 isSleeping = body0->m_resting & body1->m_resting;
	if (!isSleeping) 
	{
		dVector preconditioner0(joint->m_preconditioner0);
		dVector preconditioner1(joint->m_preconditioner1);
		
		dVector forceM0(m_internalForces[m0].m_linear * preconditioner0);
		dVector torqueM0(m_internalForces[m0].m_angular * preconditioner0);
		dVector forceM1(m_internalForces[m1].m_linear * preconditioner1);
		dVector torqueM1(m_internalForces[m1].m_angular * preconditioner1);
		
		preconditioner0 = preconditioner0.Scale(body0->m_weigh);
		preconditioner1 = preconditioner1.Scale(body1->m_weigh);
		
		normalForce[0] = dFloat32(1.0f);
		for (dInt32 j = 0; j < rowsCount; j++) 
		{
			ndRightHandSide* const rhs = &m_rightHandSide[rowStart + j];
			const ndLeftHandSide* const lhs = &m_leftHandSide[rowStart + j];
			dVector a(lhs->m_JMinv.m_jacobianM0.m_linear * forceM0);
			a = a.MulAdd(lhs->m_JMinv.m_jacobianM0.m_angular, torqueM0);
			a = a.MulAdd(lhs->m_JMinv.m_jacobianM1.m_linear, forceM1);
			a = a.MulAdd(lhs->m_JMinv.m_jacobianM1.m_angular, torqueM1);
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

	#ifdef D_LOCK_FREE_SOLVER___
		dAssert(0);
		ndJacobian& outBody0 = internalForces[m0];
		outBody0.m_linear += forceM0;
		outBody0.m_angular += torqueM0;

		ndJacobian& outBody1 = internalForces[m1];
		outBody1.m_linear += forceM1;
		outBody1.m_angular += torqueM1;

	#else
		if (singleThreaded)
		{
			ndJacobian& outBody0 = internalForces[m0];
			outBody0.m_linear += forceM0;
			outBody0.m_angular += torqueM0;

			ndJacobian& outBody1 = internalForces[m1];
			outBody1.m_linear += forceM1;
			outBody1.m_angular += torqueM1;
		}
		else
		{
			dAssert(body0->GetInvMass() > dFloat32(0.0f));
			{
				ndJacobian& outBody0 = internalForces[m0];
				dScopeSpinLock lock(body0->m_lock);
				outBody0.m_linear += forceM0;
				outBody0.m_angular += torqueM0;
			}

			if (body1->GetInvMass() > dFloat32(0.0f))
			{
				ndJacobian& outBody1 = internalForces[m1];
				dScopeSpinLock lock(body1->m_lock);
				outBody1.m_linear += forceM1;
				outBody1.m_angular += torqueM1;
			}
		}
	#endif

	return accNorm.GetScalar();
}

void ndDynamicsUpdate::IntegrateBodiesVelocity()
{
	D_TRACKTIME();
	class ndIntegrateBodiesVelocity : public ndScene::ndBaseJob
	{
		public:
		void ExecuteBatch(const dInt32 start, const dInt32 count, dArray<ndBodyKinematic*>& bodyArray, const dArray<ndJacobian>& internalForces)
		{
			//D_TRACKTIME();
			ndWorld* const world = m_owner->GetWorld();
			const dVector timestep4(world->m_timestepRK);
			const dVector speedFreeze2(world->m_freezeSpeed2 * dFloat32(0.1f));

			for (dInt32 i = 0; i < count; i++)
			{
				ndBodyKinematic* const body = bodyArray[i + start];
				ndBodyDynamic* const dynBody = body->GetAsBodyDynamic();
				if (dynBody)
				{
					dAssert(dynBody->m_bodyIsConstrained);
					const dInt32 index = dynBody->m_index;
					const ndJacobian& forceAndTorque = internalForces[index];
					const dVector force(dynBody->GetForce() + forceAndTorque.m_linear);
					const dVector torque(dynBody->GetTorque() + forceAndTorque.m_angular);

					const ndJacobian velocStep(dynBody->IntegrateForceAndToque(force, torque, timestep4));
					if (!body->m_resting)
					{
						body->m_veloc += velocStep.m_linear;
						body->m_omega += velocStep.m_angular;
					}
					else
					{
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

		virtual void Execute()
		{
			D_TRACKTIME();
			ndWorld* const world = m_owner->GetWorld();
			dArray<ndBodyKinematic*>& bodyArray = world->m_bodyIslandOrder;
			const dArray<ndJacobian>& internalForces = world->m_internalForces;
			const dInt32 bodyCount = bodyArray.GetCount() - world->m_unConstrainedBodyCount;

			const dInt32 stepSize = D_BASH_SIZE;
			const dInt32 bodyCountBatches = bodyCount & -stepSize;
			dInt32 index = m_it->fetch_add(stepSize);
			for (; index < bodyCountBatches; index = m_it->fetch_add(stepSize))
			{
				ExecuteBatch(index, stepSize, bodyArray, internalForces);
			}
			if (index < bodyCount)
			{
				ExecuteBatch(index, bodyCount - index, bodyArray, internalForces);
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
		bool ExecuteBatch(const dInt32 start, const dInt32 count, dFloat32 timestepRK,
			const ndConstraintArray& jointArray, const ndRightHandSide* const rightHandSide)
		{
			//D_TRACKTIME();
			bool hasJointFeeback = false;
			for (dInt32 i = 0; i < count; i++)
			{
				ndConstraint* const joint = jointArray[i + start];
				const dInt32 rows = joint->m_rowCount;
				const dInt32 first = joint->m_rowStart;
				
				for (dInt32 j = 0; j < rows; j++) 
				{
					const ndRightHandSide* const rhs = &rightHandSide[j + first];
					dAssert(dCheckFloat(rhs->m_force));
					rhs->m_jointFeebackForce->Push(rhs->m_force);
					rhs->m_jointFeebackForce->m_force = rhs->m_force;
					rhs->m_jointFeebackForce->m_impact = rhs->m_maxImpact * timestepRK;
				}
				hasJointFeeback |= joint->m_jointFeebackForce;
			}
			return hasJointFeeback;
		}

		virtual void Execute()
		{
			D_TRACKTIME();
			ndWorld* const world = m_owner->GetWorld();
			ndConstraintArray& jointArray = world->m_jointArray;
			const ndRightHandSide* const rightHandSide = &world->m_rightHandSide[0];

			bool hasJointFeeback = false;
			const dFloat32 timestepRK = world->m_timestepRK;

			const dInt32 stepSize = D_BASH_SIZE;
			const dInt32 jointCount = jointArray.GetCount();
			const dInt32 jointCountBatches = jointCount & -stepSize;
			dInt32 index = m_it->fetch_add(stepSize);
			for (; index < jointCountBatches; index = m_it->fetch_add(stepSize))
			{
				hasJointFeeback |= ExecuteBatch(index, stepSize, timestepRK, jointArray, rightHandSide);
			}
			if (index < jointCount)
			{
				hasJointFeeback |= ExecuteBatch(index, jointCount - index, timestepRK, jointArray, rightHandSide);
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

		void ExecuteBatch(const dInt32 start, const dInt32 count, dArray<ndBodyKinematic*>& bodyArray)
		{
			//D_TRACKTIME();
			ndWorld* const world = m_owner->GetWorld();
			const dVector invTime(world->m_invTimestep);
			const dFloat32 timestep = m_timestep;

			const dFloat32 maxAccNorm2 = D_SOLVER_MAX_ERROR * D_SOLVER_MAX_ERROR;
			for (dInt32 i = 0; i < count; i++)
			{
				ndBodyDynamic* const dynBody = bodyArray[start + i]->GetAsBodyDynamic();
			
				// the initial velocity and angular velocity were stored in m_accel and dynBody->m_alpha for memory saving
				if (dynBody)
				{
					if (!dynBody->m_equilibrium)
					{
						dVector accel(invTime * (dynBody->m_veloc - dynBody->m_accel));
						dVector alpha(invTime * (dynBody->m_omega - dynBody->m_alpha));
						dVector accelTest((accel.DotProduct(accel) > maxAccNorm2) | (alpha.DotProduct(alpha) > maxAccNorm2));
						dynBody->m_accel = accel & accelTest;
						dynBody->m_alpha = alpha & accelTest;
						dynBody->IntegrateVelocity(timestep);
					}
				}
				else
				{
					ndBodyKinematic* const kinBody = bodyArray[start + i]->GetAsBodyKinematic();
					dAssert(kinBody);
					if (!kinBody->m_equilibrium)
					{
						kinBody->IntegrateVelocity(timestep);
					}
				}
			}
		}

		virtual void Execute()
		{
			D_TRACKTIME();

			ndWorld* const world = m_owner->GetWorld();
			//dArray<ndBodyKinematic*>& bodyArray = m_owner->GetWorkingBodyArray();
			dArray<ndBodyKinematic*>& bodyArray = world->m_bodyIslandOrder;

			const dVector invTime(world->m_invTimestep);
			const dFloat32 timestep = m_timestep;
			
			const dInt32 bodyCount = bodyArray.GetCount();
			const dInt32 stepSize = D_BASH_SIZE;
			const dInt32 bodyCountBatches = bodyCount & -stepSize;
			dInt32 index = m_it->fetch_add(stepSize);
			for (; index < bodyCountBatches; index = m_it->fetch_add(stepSize))
			{
				ExecuteBatch(index, stepSize, bodyArray);
			}
			if (index < bodyCount)
			{
				ExecuteBatch(index, bodyCount - index, bodyArray);
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

		void ExecuteBatch(const dInt32 start, const dInt32 count, const dArray<ndIsland>& islandArray)
		{
			//D_TRACKTIME();
			ndWorld* const world = m_owner->GetWorld();
			for (dInt32 i = 0; i < count; i++)
			{
				const ndIsland& island = islandArray[start + i];
				world->UpdateIslandState(island);
			}
		}

		virtual void Execute()
		{
			D_TRACKTIME();
			ndWorld* const world = m_owner->GetWorld();
			const dArray<ndIsland>& islandArray = world->m_islands;
			const dInt32 islandCount = islandArray.GetCount();

			const dInt32 stepSize = D_BASH_SIZE;
			const dInt32 islandCountBatches = islandCount & -stepSize;

			dInt32 index = m_it->fetch_add(stepSize);
			for (; index < islandCountBatches; index = m_it->fetch_add(stepSize))
			{
				ExecuteBatch(index, stepSize, islandArray);
			}
			if (index < islandCount)
			{
				ExecuteBatch(index, islandCount - index, islandArray);
			}
		}
	};

	ndScene* const scene = m_world->GetScene();
	scene->SubmitJobs<ndDetermineSleepStates>();
}

void ndDynamicsUpdate::UpdateIslandState(const ndIsland& island)
{
	dFloat32 velocityDragCoeff = D_FREEZZING_VELOCITY_DRAG;
	
	const dInt32 count = island.m_count;
	if (count <= D_SMALL_ISLAND_COUNT) 
	{
		velocityDragCoeff = dFloat32(0.9999f);
	}
	
	dFloat32 maxAccel = dFloat32(0.0f);
	dFloat32 maxAlpha = dFloat32(0.0f);
	dFloat32 maxSpeed = dFloat32(0.0f);
	dFloat32 maxOmega = dFloat32(0.0f);
	
	const dFloat32 speedFreeze = m_world->m_freezeSpeed2;
	const dFloat32 accelFreeze = m_world->m_freezeAccel2 * ((count <= D_SMALL_ISLAND_COUNT) ? dFloat32(0.01f) : dFloat32(1.0f));
	//const dFloat32 accelFreeze = world->m_freezeAccel2 * ((count <= DG_SMALL_ISLAND_COUNT) ? dFloat32(0.0025f) : dFloat32(1.0f));
	dVector velocDragVect(velocityDragCoeff, velocityDragCoeff, velocityDragCoeff, dFloat32(0.0f));
	
	dInt32 stackSleeping = 1;
	dInt32 sleepCounter = 10000;
	
	ndBodyKinematic** const bodyIslands = &m_world->m_bodyIslandOrder[island.m_start];
	for (dInt32 i = 0; i < count; i++)
	{
		ndBodyDynamic* const dynBody = bodyIslands[i]->GetAsBodyDynamic();
		if (dynBody)
		{
			dAssert(dynBody->m_accel.m_w == dFloat32(0.0f));
			dAssert(dynBody->m_alpha.m_w == dFloat32(0.0f));
			dAssert(dynBody->m_veloc.m_w == dFloat32(0.0f));
			dAssert(dynBody->m_omega.m_w == dFloat32(0.0f));

			dUnsigned32 equilibrium = (dynBody->GetInvMass() == dFloat32(0.0f)) ? 1 : dynBody->m_autoSleep;
			const dVector isMovingMask(dynBody->m_veloc + dynBody->m_omega + dynBody->m_accel + dynBody->m_alpha);
			const dVector mask(isMovingMask.TestZero());
			const dInt32 test = mask.GetSignMask() & 7;
			if (test != 7)
			{
				const dFloat32 accel2 = dynBody->m_accel.DotProduct(dynBody->m_accel).GetScalar();
				const dFloat32 alpha2 = dynBody->m_alpha.DotProduct(dynBody->m_alpha).GetScalar();
				const dFloat32 speed2 = dynBody->m_veloc.DotProduct(dynBody->m_veloc).GetScalar();
				const dFloat32 omega2 = dynBody->m_omega.DotProduct(dynBody->m_omega).GetScalar();

				maxAccel = dMax(maxAccel, accel2);
				maxAlpha = dMax(maxAlpha, alpha2);
				maxSpeed = dMax(maxSpeed, speed2);
				maxOmega = dMax(maxOmega, omega2);
				dUnsigned32 equilibriumTest = (accel2 < accelFreeze) && (alpha2 < accelFreeze) && (speed2 < speedFreeze) && (omega2 < speedFreeze);

				if (equilibriumTest)
				{
					const dVector veloc(dynBody->m_veloc * velocDragVect);
					const dVector omega(dynBody->m_omega * velocDragVect);
					const dVector velocMask(veloc.DotProduct(veloc) > m_velocTol);
					const dVector omegaMask(omega.DotProduct(omega) > m_velocTol);
					dynBody->m_veloc = velocMask & veloc;
					dynBody->m_omega = omegaMask & omega;
				}

				equilibrium &= equilibriumTest;
				//stackSleeping &= equilibriumTest;
				stackSleeping &= equilibrium;
				sleepCounter = dMin(sleepCounter, dynBody->m_sleepingCounter);
				dynBody->m_sleepingCounter++;
			}
			if (dynBody->m_equilibrium != equilibrium)
			{
				dynBody->m_equilibrium = equilibrium;
			}
		}
		else 
		{
			ndBodyKinematic* const kinBody = bodyIslands[i]->GetAsBodyKinematic();
			dAssert(kinBody);
			dUnsigned32 equilibrium = (kinBody->GetInvMass() == dFloat32(0.0f)) ? 1 : (kinBody->m_autoSleep & ~kinBody->m_equilibriumOverride);
			const dVector isMovingMask(kinBody->m_veloc + kinBody->m_omega);
			const dVector mask(isMovingMask.TestZero());
			const dInt32 test = mask.GetSignMask() & 7;
			if (test != 7)
			{
				const dFloat32 speed2 = kinBody->m_veloc.DotProduct(kinBody->m_veloc).GetScalar();
				const dFloat32 omega2 = kinBody->m_omega.DotProduct(kinBody->m_omega).GetScalar();

				maxSpeed = dMax(maxSpeed, speed2);
				maxOmega = dMax(maxOmega, omega2);
				dUnsigned32 equilibriumTest = (speed2 < speedFreeze) && (omega2 < speedFreeze);

				if (equilibriumTest)
				{
					const dVector veloc(kinBody->m_veloc * velocDragVect);
					const dVector omega(kinBody->m_omega * velocDragVect);
					const dVector velocMask(veloc.DotProduct(veloc) > m_velocTol);
					const dVector omegaMask(omega.DotProduct(omega) > m_velocTol);
					kinBody->m_veloc = velocMask & veloc;
					kinBody->m_omega = omegaMask & omega;
				}

				equilibrium &= equilibriumTest;
				//stackSleeping &= equilibriumTest;
				stackSleeping &= equilibrium;
				sleepCounter = dMin(sleepCounter, kinBody->m_sleepingCounter);
				//kinBody->m_sleepingCounter++;
			}
			if (kinBody->m_equilibrium != equilibrium)
			{
				kinBody->m_equilibrium = equilibrium;
			}
		}
	}
	
	if (stackSleeping) 
	{
		for (dInt32 i = 0; i < count; i++) 
		{
			// force entire island to equilibriumTest
			ndBodyDynamic* const body = bodyIslands[i]->GetAsBodyDynamic();
			if (body)
			{
				body->m_accel = dVector::m_zero;
				body->m_alpha = dVector::m_zero;
				body->m_veloc = dVector::m_zero;
				body->m_omega = dVector::m_zero;
				body->m_equilibrium = (body->GetInvMass() == dFloat32(0.0f)) ? 1 : body->m_autoSleep;
			}
			else 
			{
				ndBodyKinematic* const kinBody = bodyIslands[i]->GetAsBodyKinematic();
				dAssert(kinBody);
				kinBody->m_veloc = dVector::m_zero;
				kinBody->m_omega = dVector::m_zero;
				kinBody->m_equilibrium = (kinBody->GetInvMass() == dFloat32(0.0f)) ? 1 : kinBody->m_autoSleep;
			}
		}
	}
	else if ((count > 1) || bodyIslands[0]->m_bodyIsConstrained)
	{
		const bool state = 
			(maxAccel > m_world->m_sleepTable[D_SLEEP_ENTRIES - 1].m_maxAccel) ||
			(maxAlpha > m_world->m_sleepTable[D_SLEEP_ENTRIES - 1].m_maxAlpha) ||
			(maxSpeed > m_world->m_sleepTable[D_SLEEP_ENTRIES - 1].m_maxVeloc) ||
			(maxOmega > m_world->m_sleepTable[D_SLEEP_ENTRIES - 1].m_maxOmega);

		if (state) 
		{
			for (dInt32 i = 0; i < count; i++) 
			{
				ndBodyDynamic* const body = bodyIslands[i]->GetAsBodyDynamic();
				dAssert(body);
				if (body) 
				{
					body->m_sleepingCounter = 0;
				}
			}
		}
		else 
		{
			if (count < D_SMALL_ISLAND_COUNT) 
			{
				// delay small islandArray for about 10 seconds
				sleepCounter >>= 8;
				for (dInt32 i = 0; i < count; i++) 
				{
					ndBodyKinematic* const body = bodyIslands[i];
					body->m_equilibrium = 0;
				}
			}
			dInt32 timeScaleSleepCount = dInt32(dFloat32(60.0f) * sleepCounter * m_timestep);

			dInt32 index = D_SLEEP_ENTRIES;
			for (dInt32 i = 1; i < D_SLEEP_ENTRIES; i++) 
			{
				if (m_world->m_sleepTable[i].m_steps > timeScaleSleepCount) 
				{
					index = i;
					break;
				}
			}
			index--;

			bool state1 = 
				(maxAccel < m_world->m_sleepTable[index].m_maxAccel) &&
				(maxAlpha < m_world->m_sleepTable[index].m_maxAlpha) &&
				(maxSpeed < m_world->m_sleepTable[index].m_maxVeloc) &&
				(maxOmega < m_world->m_sleepTable[index].m_maxOmega);
			if (state1) 
			{
				for (dInt32 i = 0; i < count; i++) 
				{
					ndBodyKinematic* const body = bodyIslands[i];
					body->m_veloc = dVector::m_zero;
					body->m_omega = dVector::m_zero;
					body->m_equilibrium = body->m_autoSleep;
					ndBodyDynamic* const dynBody = body->GetAsBodyDynamic();
					if (dynBody)
					{
						dynBody->m_accel = dVector::m_zero;
						dynBody->m_alpha = dVector::m_zero;
						dynBody->m_sleepingCounter = 0;
					}
				}
			}
		}
	}
}

void ndDynamicsUpdate::InitSkeletons()
{
	D_TRACKTIME();

	class ndInitSkeletons : public ndScene::ndBaseJob
	{
		public:
		virtual void Execute()
		{
			D_TRACKTIME();
			const dInt32 threadIndex = GetThredID();
			ndWorld* const world = m_owner->GetWorld();
			ndSkeletonList::dListNode* node = world->GetSkeletonList().GetFirst();
			for (dInt32 i = 0; i < threadIndex; i++)
			{
				node = node ? node->GetNext() : nullptr;
			}

			dArray<ndRightHandSide>& rightHandSide = world->m_rightHandSide;
			const dArray<ndLeftHandSide>& leftHandSide = world->m_leftHandSide;
			
			const dInt32 threadCount = m_owner->GetThreadCount();
			while (node)
			{
				ndSkeletonContainer* const skeleton = &node->GetInfo();
				skeleton->InitMassMatrix(&leftHandSide[0], &rightHandSide[0]);

				for (dInt32 i = 0; i < threadCount; i++)
				{
					node = node ? node->GetNext() : nullptr;
				}
			}
		}
	};

	ndScene* const scene = m_world->GetScene();
	scene->SubmitJobs<ndInitSkeletons>();
}

void ndDynamicsUpdate::UpdateSkeletons()
{
	D_TRACKTIME();
	class ndUpdateSkeletons : public ndScene::ndBaseJob
	{
		public:
		virtual void Execute()
		{
			D_TRACKTIME();
			const dInt32 threadIndex = GetThredID();
			ndWorld* const world = m_owner->GetWorld();
			ndSkeletonList::dListNode* node = world->GetSkeletonList().GetFirst();
			for (dInt32 i = 0; i < threadIndex; i++)
			{
				node = node ? node->GetNext() : nullptr;
			}

			ndJacobian* const internalForces = &world->m_internalForces[0];
			const dArray<ndBodyKinematic*>& ativeBodies = m_owner->ndScene::GetActiveBodyArray();
			const ndBodyKinematic** const bodyArray = (const ndBodyKinematic**)&ativeBodies[0];

			const dInt32 threadCount = m_owner->GetThreadCount();
			while (node)
			{
				ndSkeletonContainer* const skeleton = &node->GetInfo();
				skeleton->CalculateJointForce(bodyArray, internalForces);

				for (dInt32 i = 0; i < threadCount; i++)
				{
					node = node ? node->GetNext() : nullptr;
				}
			}
		}
	};

	ndScene* const scene = m_world->GetScene();
	scene->SubmitJobs<ndUpdateSkeletons>();
}
