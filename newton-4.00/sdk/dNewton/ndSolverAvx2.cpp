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


#define D_SOA_WORD_GROUP_SIZE	8 

#define D_RADIX_BITS	5
#define D_RADIX_DIGIT	(1<<(D_RADIX_BITS + 1))

inline dInt32 ndDynamicsUpdate::GetSortKeyAvx2(const ndConstraint* const joint)
{
	const ndBodyKinematic* const body0 = joint->GetBody0();
	const ndBodyKinematic* const body1 = joint->GetBody1();

	const dInt32 rows = joint->GetRowsCount();
	dAssert(rows < D_RADIX_DIGIT / 2);

	//const dInt32 isResting = !(body0->m_resting & body1->m_resting) << D_RADIX_BITS;
	const dInt32 isResting = 0;
	const dInt32 key = D_RADIX_DIGIT/2 - rows + isResting;
	dAssert(key >= 0 && key < D_RADIX_DIGIT);
	return key;
}

void ndDynamicsUpdate::BuildIslandAvx2()
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

		if (jointArray.GetCount())
		{
			dInt32 rowCount = 0;
			for (dInt32 i = 0; i < jointArray.GetCount(); i++)
			{
				ndConstraint* const joint = jointArray[i];
				const dInt32 rows = joint->GetRowsCount();
				joint->m_rowCount = rows;
				joint->m_rowStart = rowCount;
				rowCount += rows;

				ndBodyKinematic* const body0 = joint->GetBody0();
				ndBodyKinematic* const body1 = joint->GetBody1();

				const dInt32 resting = body0->m_equilibrium & body1->m_equilibrium;
				body0->m_bodyIsConstrained = 1;
				body0->m_resting = body0->m_resting & resting;

				if (body1->GetInvMass() > dFloat32(0.0f))
				{
					body1->m_bodyIsConstrained = 1;
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
				else
				{
					if (!body0->m_islandSleep)
					{
						ndBodyKinematic* const root = FindRootAndSplit(body0);
						root->m_islandSleep = 0;
					}
				}
			}

			const dInt32 savedCount = jointArray.GetCount();
			jointArray.SetCount(2 * savedCount + D_SOA_WORD_GROUP_SIZE);

			dInt32 jointCountSpans[D_RADIX_DIGIT];
			memset(jointCountSpans, 0, sizeof(jointCountSpans));
			ndConstraint** const JointTmpBuffer = &jointArray[savedCount];

			for (dInt32 i = 0; i < savedCount; i++)
			{
				ndConstraint* const joint = jointArray[i];
				JointTmpBuffer[i] = joint;
				const dInt32 key = GetSortKeyAvx2(joint);
				jointCountSpans[key] ++;
			}

			dInt32 acc = 0;
			for (dInt32 i = 0; i < sizeof(jointCountSpans) / sizeof(jointCountSpans[0]); i++)
			{
				const dInt32 val = jointCountSpans[i];
				jointCountSpans[i] = acc;
				acc += val;
			}

			for (dInt32 i = 0; i < savedCount; i++)
			{
				ndConstraint* const joint = JointTmpBuffer[i];
				const dInt32 key = GetSortKeyAvx2(joint);
				const dInt32 entry = jointCountSpans[key];
				jointArray[entry] = joint;
				jointCountSpans[key] = entry + 1;
			}

			const dInt32 mask = -dInt32(D_SOA_WORD_GROUP_SIZE);
			const dInt32 jointCountSoa = (savedCount + D_SOA_WORD_GROUP_SIZE - 1) & mask;
			for (dInt32 i = savedCount; i < jointCountSoa; i++)
			{
				jointArray[i] = nullptr;
			}

			jointArray.SetCount(savedCount);

			#ifdef _DEBUG
			for (dInt32 i = 0; i < savedCount - 1; i++)
			{
				const ndConstraint* const joint0 = jointArray[i + 0];
				const ndConstraint* const joint1 = jointArray[i + 1];

				const dInt32 key0 = GetSortKeyAvx2(joint0);
				const dInt32 key1 = GetSortKeyAvx2(joint1);
				dAssert(key0 <= key1);
			}
			#endif
		}

		dInt32 count = 0;
		m_internalForces.SetCount(bodyArray.GetCount());
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
			ndBodyIndexPair* const buffer2 = buffer0 + count;
			for (dInt32 i = 0; i < count; i++)
			{
				const dInt32 key = 1 - buffer0[i].m_root->m_bodyIsConstrained;
				const dInt32 slot = scans[key];
				buffer2[slot] = buffer0[i];
				scans[key] = slot + 1;
			}

			const ndBodyIndexPair* const buffer1 = buffer0 + count;
			for (dInt32 i = 0; i < count; i++)
			{
				dAssert((i == count - 1) || (buffer1[i].m_root->m_bodyIsConstrained >= buffer1[i + 1].m_root->m_bodyIsConstrained));

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

void ndDynamicsUpdate::BuildJacobianMatrixAvx2(ndConstraint* const joint, ndJacobian* const internalForces)
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

	ndJacobian& outBody0 = internalForces[m0];
	outBody0.m_linear += forceAcc0;
	outBody0.m_angular += torqueAcc0;

	ndJacobian& outBody1 = internalForces[m1];
	outBody1.m_linear += forceAcc1;
	outBody1.m_angular += torqueAcc1;
}

void ndDynamicsUpdate::InitJacobianMatrixAvx2()
{
	class ndInitJacobianMatrixAvx2 : public ndScene::ndBaseJob
	{
		public:
		virtual void Execute()
		{
			D_TRACKTIME();

			ndWorld* const world = m_owner->GetWorld();
			ndConstraint** const jointArray = &world->m_jointArray[0];
			const dInt32 jointCount = world->m_jointArray.GetCount();
			const dInt32 threadCount = dMax(m_owner->GetThreadCount(), 1);

			if (threadCount == 1)
			{
				ndJacobian* const internalForces = &world->m_internalForces[0];
				for (dInt32 i = 0; i < jointCount; i++)
				{
					ndConstraint* const joint = jointArray[i];
					world->GetJacobianDerivatives(joint);
					world->BuildJacobianMatrixAvx2(joint, internalForces);
				}
			}
			else
			{
				const dInt32 threadIndex = GetThredId();
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
					world->BuildJacobianMatrixAvx2(joint, internalForces);
				}
			}
		}
	};

	class ndInitJacobianAccumulatePartialForcesAvx2 : public ndScene::ndBaseJob
	{
		public:
		virtual void Execute()
		{
			D_TRACKTIME();
			ndWorld* const world = m_owner->GetWorld();
			const dInt32 threadIndex = GetThredId();
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
			scene->SubmitJobs<ndInitJacobianMatrixAvx2>();
		}
		else
		{
			scene->SubmitJobs<ndInitJacobianMatrixAvx2>();
			scene->SubmitJobs<ndInitJacobianAccumulatePartialForcesAvx2>();
		}
	}
}

void ndDynamicsUpdate::CalculateJointsAccelerationAvx2()
{
	D_TRACKTIME();
	class ndCalculateJointsAccelerationAvx2 : public ndScene::ndBaseJob
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
	
			const dInt32 threadIndex = GetThredId();
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
	scene->SubmitJobs<ndCalculateJointsAccelerationAvx2>();
	m_firstPassCoef = dFloat32(1.0f);
}

dFloat32 ndDynamicsUpdate::CalculateJointsForceAvx2(ndConstraint* const joint, ndJacobian* const internalForces)
{
	//D_TRACKTIME();
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


	ndJacobian& outBody0 = internalForces[m0];
	outBody0.m_linear += forceM0;
	outBody0.m_angular += torqueM0;

	ndJacobian& outBody1 = internalForces[m1];
	outBody1.m_linear += forceM1;
	outBody1.m_angular += torqueM1;

	return accNorm.GetScalar();
}

void ndDynamicsUpdate::CalculateJointsForceAvx2()
{
	D_TRACKTIME();
	class ndCalculateJointsForceAvx2 : public ndScene::ndBaseJob
	{
		public:
		virtual void Execute()
		{
			D_TRACKTIME();
			ndWorld* const world = m_owner->GetWorld();
			ndConstraintArray& jointArray = world->m_jointArray;
			dFloat32 accNorm = dFloat32(0.0f);
			const dInt32 jointCount = jointArray.GetCount();
			const dInt32 bodyCount = m_owner->GetActiveBodyArray().GetCount();
			const dInt32 threadCount = dMax(m_owner->GetThreadCount(), 1);
			if (threadCount == 1)
			{
				ndJacobian* const internalForces = &world->m_internalForces[bodyCount];
				for (dInt32 i = 0; i < jointCount; i++)
				{
					ndConstraint* const joint = jointArray[i];
					accNorm += world->CalculateJointsForceAvx2(joint, internalForces);
				}
				dFloat32* const accelNorm = (dFloat32*)m_context;
				accelNorm[0] = accNorm;
			}
			else
			{
				const dInt32 threadIndex = GetThredId();
				ndJacobian* const internalForces = &world->m_internalForces[bodyCount * (threadIndex + 1)];
				memset(internalForces, 0, bodyCount * sizeof(ndJacobian));
				for (dInt32 i = threadIndex; i < jointCount; i += threadCount)
				{
					ndConstraint* const joint = jointArray[i];
					accNorm += world->CalculateJointsForceAvx2(joint, internalForces);
				}
				dFloat32* const accelNorm = (dFloat32*)m_context;
				accelNorm[threadIndex] = accNorm;
			}
		}
	};

	class ndInitJacobianAccumulatePartialForcesAvx2 : public ndScene::ndBaseJob
	{
		public:
		virtual void Execute()
		{
			D_TRACKTIME();
			ndWorld* const world = m_owner->GetWorld();
			const dInt32 threadIndex = GetThredId();
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
				for (dInt32 j = 1; j <= threadCount; j++)
				{
					force += internalForces[bodyCount * j + base].m_linear;
					torque += internalForces[bodyCount * j + base].m_angular;
				}
				internalForces[base].m_linear = force;
				internalForces[base].m_angular = torque;
			}
		}
	};

	ndScene* const scene = m_world->GetScene();
	const dInt32 passes = m_solverPasses;
	const dInt32 bodyCount = scene->GetActiveBodyArray().GetCount();
	const dInt32 threadsCount = dMax(scene->GetThreadCount(), 1);

	dFloat32 m_accelNorm[D_MAX_THREADS_COUNT];
	dFloat32 accNorm = D_SOLVER_MAX_ERROR * dFloat32(2.0f);

	for (dInt32 i = 0; (i < passes) && (accNorm > D_SOLVER_MAX_ERROR); i++)
	{
#ifdef D_PROFILE_JOINTS
		dUnsigned64 cpuClock = dGetCpuClock();
#endif
		if (threadsCount == 1)
		{
			memset(&m_internalForces[bodyCount], 0, bodyCount * sizeof(ndJacobian));
			scene->SubmitJobs<ndCalculateJointsForceAvx2>(m_accelNorm);
			memcpy(&m_internalForces[0], &m_internalForces[bodyCount], bodyCount * sizeof(ndJacobian));
		}
		else
		{
			scene->SubmitJobs<ndCalculateJointsForceAvx2>(m_accelNorm);
			scene->SubmitJobs<ndInitJacobianAccumulatePartialForcesAvx2>();
		}

#ifdef D_PROFILE_JOINTS
		static dUnsigned64 ticks = 0;
		static dUnsigned64 joints = 0;
		static dInt32 averageCount = 0;
		cpuClock = dGetCpuClock() - cpuClock;
		ticks += cpuClock;
		joints += m_world->m_jointArray.GetCount();
		averageCount++;
		if (averageCount > 10000)
		{
			dgExpandTraceMessage("ticks per joints: %d\n", ticks / joints);
			joints = 0;
			ticks = 0;
			averageCount = 0;
		}
#endif

		accNorm = dFloat32(0.0f);
		for (dInt32 j = 0; j < threadsCount; j++)
		{
			accNorm = dMax(accNorm, m_accelNorm[j]);
		}
	}
}

void ndDynamicsUpdate::IntegrateBodiesVelocityAvx2()
{
	D_TRACKTIME();
	class ndIntegrateBodiesVelocityAvx2 : public ndScene::ndBaseJob
	{
		public:
		virtual void Execute()
		{
			D_TRACKTIME();
			ndWorld* const world = m_owner->GetWorld();
			dArray<ndBodyKinematic*>& bodyArray = world->m_bodyIslandOrder;

			const dInt32 threadIndex = GetThredId();
			const dInt32 threadCount = m_owner->GetThreadCount();
			const dInt32 bodyCount = bodyArray.GetCount() - world->m_unConstrainedBodyCount;
			const dInt32 step = bodyCount / threadCount;
			const dInt32 start = threadIndex * step;
			const dInt32 count = ((threadIndex + 1) < threadCount) ? step : bodyCount - start;
			const dFloat32 timestep = m_timestep;

			const dVector timestep4(world->m_timestepRK);
			const dVector speedFreeze2(world->m_freezeSpeed2 * dFloat32(0.1f));

			const dArray<ndJacobian>& internalForces = world->m_internalForces;
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
	};

	ndScene* const scene = m_world->GetScene();
	scene->SubmitJobs<ndIntegrateBodiesVelocityAvx2>();
}

void ndDynamicsUpdate::InitSkeletonsAvx2()
{
	D_TRACKTIME();

	class ndInitSkeletonsAvx2 : public ndScene::ndBaseJob
	{
		public:
		virtual void Execute()
		{
			D_TRACKTIME();
			const dInt32 threadIndex = GetThredId();
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
	scene->SubmitJobs<ndInitSkeletonsAvx2>();
}

void ndDynamicsUpdate::UpdateSkeletonsAvx2()
{
	D_TRACKTIME();
	class ndUpdateSkeletonsAvx2 : public ndScene::ndBaseJob
	{
		public:
		virtual void Execute()
		{
			D_TRACKTIME();
			const dInt32 threadIndex = GetThredId();
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
	scene->SubmitJobs<ndUpdateSkeletonsAvx2>();
}

void ndDynamicsUpdate::CalculateForcesAvx2()
{
	D_TRACKTIME();
	dInt32 hasJointFeeback = 0;
	if (m_jointArray.GetCount())
	{
		m_firstPassCoef = dFloat32(0.0f);
		if (m_world->m_skeletonList.GetCount())
		{
			InitSkeletonsAvx2();
		}

		for (dInt32 step = 0; step < 4; step++)
		{
			CalculateJointsAccelerationAvx2();
			CalculateJointsForceAvx2();
			if (m_world->m_skeletonList.GetCount())
			{
				UpdateSkeletonsAvx2();
			}
			IntegrateBodiesVelocityAvx2();
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

void ndDynamicsUpdate::UpdateAvx2()
{
	m_world = (ndWorld*)this;
	m_timestep = m_world->GetScene()->GetTimestep();

	BuildIslandAvx2();
	if (m_islands.GetCount())
	{
		IntegrateUnconstrainedBodies();

		InitWeights();
		InitBodyArray();
		InitJacobianMatrixAvx2();
		CalculateForcesAvx2();
		DetermineSleepStates();
	}
}
