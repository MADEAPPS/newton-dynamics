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

using namespace ndAvx2;

ndSoaFloat ndDynamicsUpdate::m_one(dFloat32 (1.0f));
ndSoaFloat ndDynamicsUpdate::m_zero(dFloat32(0.0f));

inline dInt32 ndDynamicsUpdate::GetSortKeyAvx2(const ndConstraint* const joint)
{
	const ndBodyKinematic* const body0 = joint->GetBody0();
	const ndBodyKinematic* const body1 = joint->GetBody1();

	const dInt32 rows = joint->GetRowsCount();
	dAssert(rows < D_RADIX_DIGIT / 2);

	const dInt32 isResting = !(body0->m_resting & body1->m_resting) << D_RADIX_BITS;
	const dInt32 key = D_RADIX_DIGIT / 2 - rows + isResting;
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

	class ndSortJointGroupAvx2 : public ndScene::ndBaseJob
	{
		public:
		virtual void Execute()
		{
			D_TRACKTIME();

			ndWorld* const world = m_owner->GetWorld();
			ndConstraint** const jointArray = &world->m_soaJointArray[0];
			const dInt32 jointCount = world->m_soaJointArray.GetCount();

			const dInt32 threadIndex = GetThredId();
			const dInt32 threadCount = dMax(m_owner->GetThreadCount(), 1);
			dAssert(jointCount % D_SOA_WORD_GROUP_SIZE == 0);
			const dInt32 soaJointCount = jointCount / D_SOA_WORD_GROUP_SIZE;
			for (dInt32 i = threadIndex; i < soaJointCount; i += threadCount)
			{
				const dInt32 index = i * D_SOA_WORD_GROUP_SIZE;
				ndConstraint** const array = &jointArray[index];
				const ndConstraint* const lastJoint = array[D_SOA_WORD_GROUP_SIZE - 1];
				if (lastJoint)
				{
					for (dInt32 j = 1; j < D_SOA_WORD_GROUP_SIZE; j++) 
					{
						dInt32 slot = j;
						ndConstraint* const joint = array[slot];
						for (; (slot > 0) && (array[slot - 1]->m_rowCount < joint->m_rowCount); slot--)
						{
							array[slot] = array[slot - 1];
						}
						array[slot] = joint;
					}
				}
				else
				{
					for (dInt32 count = D_SOA_WORD_GROUP_SIZE - 1; count >= 0; count--)
					{
						if (array[count])
						{
							for (dInt32 j = 1; j <= count; j++)
							{
								dInt32 slot = j;
								ndConstraint* const joint = array[slot];
								for (; (slot > 0) && (array[slot - 1]->m_rowCount < joint->m_rowCount); slot--)
								{
									array[slot] = array[slot - 1];
								}
								array[slot] = joint;
							}
							break;
						}
					}
				}
			}
		}
	};

	class ndTransposeMassMatrixAvx2 : public ndScene::ndBaseJob
	{
		public:
		virtual void Execute()
		{
			D_TRACKTIME();

			ndWorld* const world = m_owner->GetWorld();
			ndConstraint** const jointArray = &world->m_soaJointArray[0];
			const dInt32 jointCount = world->m_soaJointArray.GetCount();

			const dInt32 threadIndex = GetThredId();
			const dInt32 threadCount = dMax(m_owner->GetThreadCount(), 1);
			const ndLeftHandSide* const leftHandSide = &world->m_leftHandSide[0];
			const ndRightHandSide* const rightHandSide = &world->m_rightHandSide[0];

			dArray<ndAvx2::ndSoaMatrixElement>& massMatrix = world->m_soaMassMatrix;

			
			dAssert(jointCount % D_SOA_WORD_GROUP_SIZE == 0);
			const dInt32 soaJointCount = jointCount / D_SOA_WORD_GROUP_SIZE;
			const dInt32* const soaJointRows = &world->m_soaJointRows[0];
			for (dInt32 i = threadIndex; i < soaJointCount; i += threadCount)
			{
				const dInt32 index = i * D_SOA_WORD_GROUP_SIZE;
				const dInt32 soaRowBase = soaJointRows[i];
				const ndConstraint* const lastJoint = jointArray[index + D_SOA_WORD_GROUP_SIZE - 1];
				if (lastJoint && (lastJoint->m_rowCount == jointArray[index]->m_rowCount))
				{
					const ndConstraint* const joint0 = jointArray[index + 0];
					const ndConstraint* const joint1 = jointArray[index + 1];
					const ndConstraint* const joint2 = jointArray[index + 2];
					const ndConstraint* const joint3 = jointArray[index + 3];
					const ndConstraint* const joint4 = jointArray[index + 4];
					const ndConstraint* const joint5 = jointArray[index + 5];
					const ndConstraint* const joint6 = jointArray[index + 6];
					const ndConstraint* const joint7 = jointArray[index + 7];

					const ndConstraint* const joint = jointArray[index];
					const dInt32 rowCount = joint->m_rowCount;
					
					for (dInt32 j = 0; j < rowCount; j++)
					{
						dVector tmp[8];
						const ndLeftHandSide* const row0 = &leftHandSide[joint0->m_rowStart + j];
						const ndLeftHandSide* const row1 = &leftHandSide[joint1->m_rowStart + j];
						const ndLeftHandSide* const row2 = &leftHandSide[joint2->m_rowStart + j];
						const ndLeftHandSide* const row3 = &leftHandSide[joint3->m_rowStart + j];
						const ndLeftHandSide* const row4 = &leftHandSide[joint4->m_rowStart + j];
						const ndLeftHandSide* const row5 = &leftHandSide[joint5->m_rowStart + j];
						const ndLeftHandSide* const row6 = &leftHandSide[joint6->m_rowStart + j];
						const ndLeftHandSide* const row7 = &leftHandSide[joint7->m_rowStart + j];
						ndAvx2::ndSoaMatrixElement& row = massMatrix[soaRowBase + j];

						dVector::Transpose4x4(tmp[0], tmp[1], tmp[2], tmp[3],
							row0->m_Jt.m_jacobianM0.m_linear,
							row1->m_Jt.m_jacobianM0.m_linear,
							row2->m_Jt.m_jacobianM0.m_linear,
							row3->m_Jt.m_jacobianM0.m_linear);
						dVector::Transpose4x4(tmp[4], tmp[5], tmp[6], tmp[7],
							row4->m_Jt.m_jacobianM0.m_linear,
							row5->m_Jt.m_jacobianM0.m_linear,
							row6->m_Jt.m_jacobianM0.m_linear,
							row7->m_Jt.m_jacobianM0.m_linear);
						row.m_Jt.m_jacobianM0.m_linear.m_x = ndSoaFloat(tmp[0], tmp[4]);
						row.m_Jt.m_jacobianM0.m_linear.m_y = ndSoaFloat(tmp[1], tmp[5]);;
						row.m_Jt.m_jacobianM0.m_linear.m_z = ndSoaFloat(tmp[2], tmp[6]);
						dVector::Transpose4x4(tmp[0], tmp[1], tmp[2], tmp[3],
							row0->m_Jt.m_jacobianM0.m_angular,
							row1->m_Jt.m_jacobianM0.m_angular,
							row2->m_Jt.m_jacobianM0.m_angular,
							row3->m_Jt.m_jacobianM0.m_angular);
						dVector::Transpose4x4(tmp[4], tmp[5], tmp[6], tmp[7],
							row4->m_Jt.m_jacobianM0.m_angular,
							row5->m_Jt.m_jacobianM0.m_angular,
							row6->m_Jt.m_jacobianM0.m_angular,
							row7->m_Jt.m_jacobianM0.m_angular);
						row.m_Jt.m_jacobianM0.m_angular.m_x = ndSoaFloat(tmp[0], tmp[4]);
						row.m_Jt.m_jacobianM0.m_angular.m_y = ndSoaFloat(tmp[1], tmp[5]);;
						row.m_Jt.m_jacobianM0.m_angular.m_z = ndSoaFloat(tmp[2], tmp[6]);

						dVector::Transpose4x4(tmp[0], tmp[1], tmp[2], tmp[3],
							row0->m_Jt.m_jacobianM1.m_linear,
							row1->m_Jt.m_jacobianM1.m_linear,
							row2->m_Jt.m_jacobianM1.m_linear,
							row3->m_Jt.m_jacobianM1.m_linear);
						dVector::Transpose4x4(tmp[4], tmp[5], tmp[6], tmp[7],
							row4->m_Jt.m_jacobianM1.m_linear,
							row5->m_Jt.m_jacobianM1.m_linear,
							row6->m_Jt.m_jacobianM1.m_linear,
							row7->m_Jt.m_jacobianM1.m_linear);
						row.m_Jt.m_jacobianM1.m_linear.m_x = ndSoaFloat(tmp[0], tmp[4]);
						row.m_Jt.m_jacobianM1.m_linear.m_y = ndSoaFloat(tmp[1], tmp[5]);;
						row.m_Jt.m_jacobianM1.m_linear.m_z = ndSoaFloat(tmp[2], tmp[6]);
						dVector::Transpose4x4(tmp[0], tmp[1], tmp[2], tmp[3],
							row0->m_Jt.m_jacobianM1.m_angular,
							row1->m_Jt.m_jacobianM1.m_angular,
							row2->m_Jt.m_jacobianM1.m_angular,
							row3->m_Jt.m_jacobianM1.m_angular);
						dVector::Transpose4x4(tmp[4], tmp[5], tmp[6], tmp[7],
							row4->m_Jt.m_jacobianM1.m_angular,
							row5->m_Jt.m_jacobianM1.m_angular,
							row6->m_Jt.m_jacobianM1.m_angular,
							row7->m_Jt.m_jacobianM1.m_angular);
						row.m_Jt.m_jacobianM1.m_angular.m_x = ndSoaFloat(tmp[0], tmp[4]);
						row.m_Jt.m_jacobianM1.m_angular.m_y = ndSoaFloat(tmp[1], tmp[5]);;
						row.m_Jt.m_jacobianM1.m_angular.m_z = ndSoaFloat(tmp[2], tmp[6]);


						dVector::Transpose4x4(tmp[0], tmp[1], tmp[2], tmp[3],
							row0->m_JMinv.m_jacobianM0.m_linear,
							row1->m_JMinv.m_jacobianM0.m_linear,
							row2->m_JMinv.m_jacobianM0.m_linear,
							row3->m_JMinv.m_jacobianM0.m_linear);
						dVector::Transpose4x4(tmp[4], tmp[5], tmp[6], tmp[7],
							row4->m_JMinv.m_jacobianM0.m_linear,
							row5->m_JMinv.m_jacobianM0.m_linear,
							row6->m_JMinv.m_jacobianM0.m_linear,
							row7->m_JMinv.m_jacobianM0.m_linear);
						row.m_JMinv.m_jacobianM0.m_linear.m_x = ndSoaFloat(tmp[0], tmp[4]);
						row.m_JMinv.m_jacobianM0.m_linear.m_y = ndSoaFloat(tmp[1], tmp[5]);;
						row.m_JMinv.m_jacobianM0.m_linear.m_z = ndSoaFloat(tmp[2], tmp[6]);
						dVector::Transpose4x4(tmp[0], tmp[1], tmp[2], tmp[3],
							row0->m_JMinv.m_jacobianM0.m_angular,
							row1->m_JMinv.m_jacobianM0.m_angular,
							row2->m_JMinv.m_jacobianM0.m_angular,
							row3->m_JMinv.m_jacobianM0.m_angular);
						dVector::Transpose4x4(tmp[4], tmp[5], tmp[6], tmp[7],
							row4->m_JMinv.m_jacobianM0.m_angular,
							row5->m_JMinv.m_jacobianM0.m_angular,
							row6->m_JMinv.m_jacobianM0.m_angular,
							row7->m_JMinv.m_jacobianM0.m_angular);
						row.m_JMinv.m_jacobianM0.m_angular.m_x = ndSoaFloat(tmp[0], tmp[4]);
						row.m_JMinv.m_jacobianM0.m_angular.m_y = ndSoaFloat(tmp[1], tmp[5]);;
						row.m_JMinv.m_jacobianM0.m_angular.m_z = ndSoaFloat(tmp[2], tmp[6]);

						dVector::Transpose4x4(tmp[0], tmp[1], tmp[2], tmp[3],
							row0->m_JMinv.m_jacobianM1.m_linear,
							row1->m_JMinv.m_jacobianM1.m_linear,
							row2->m_JMinv.m_jacobianM1.m_linear,
							row3->m_JMinv.m_jacobianM1.m_linear);
						dVector::Transpose4x4(tmp[4], tmp[5], tmp[6], tmp[7],
							row4->m_JMinv.m_jacobianM1.m_linear,
							row5->m_JMinv.m_jacobianM1.m_linear,
							row6->m_JMinv.m_jacobianM1.m_linear,
							row7->m_JMinv.m_jacobianM1.m_linear);
						row.m_JMinv.m_jacobianM1.m_linear.m_x = ndSoaFloat(tmp[0], tmp[4]);
						row.m_JMinv.m_jacobianM1.m_linear.m_y = ndSoaFloat(tmp[1], tmp[5]);;
						row.m_JMinv.m_jacobianM1.m_linear.m_z = ndSoaFloat(tmp[2], tmp[6]);
						dVector::Transpose4x4(tmp[0], tmp[1], tmp[2], tmp[3],
							row0->m_JMinv.m_jacobianM1.m_angular,
							row1->m_JMinv.m_jacobianM1.m_angular,
							row2->m_JMinv.m_jacobianM1.m_angular,
							row3->m_JMinv.m_jacobianM1.m_angular);
						dVector::Transpose4x4(tmp[4], tmp[5], tmp[6], tmp[7],
							row4->m_JMinv.m_jacobianM1.m_angular,
							row5->m_JMinv.m_jacobianM1.m_angular,
							row6->m_JMinv.m_jacobianM1.m_angular,
							row7->m_JMinv.m_jacobianM1.m_angular);
						row.m_JMinv.m_jacobianM1.m_angular.m_x = ndSoaFloat(tmp[0], tmp[4]);
						row.m_JMinv.m_jacobianM1.m_angular.m_y = ndSoaFloat(tmp[1], tmp[5]);;
						row.m_JMinv.m_jacobianM1.m_angular.m_z = ndSoaFloat(tmp[2], tmp[6]);


						dInt32* const normalIndex = (dInt32*)&row.m_normalForceIndex[0];
						for (dInt32 k = 0; k < D_SOA_WORD_GROUP_SIZE; k++)
						{
							const ndConstraint* const soaJoint = jointArray[index + k];
							const ndRightHandSide* const rhs = &rightHandSide[soaJoint->m_rowStart + j];
							row.m_force[k] = rhs->m_force;
							row.m_diagDamp[k] = rhs->m_diagDamp;
							row.m_invJinvMJt[k] = rhs->m_invJinvMJt;
							row.m_coordenateAccel[k] = rhs->m_coordenateAccel;
							row.m_lowerBoundFrictionCoefficent[k] = rhs->m_lowerBoundFrictionCoefficent;
							row.m_upperBoundFrictionCoefficent[k] = rhs->m_upperBoundFrictionCoefficent;
							normalIndex[k] = (rhs->m_normalForceIndex + 1) * D_SOA_WORD_GROUP_SIZE + k;
						}
					}
				}
				else 
				{
					const ndConstraint* const firstJoint = jointArray[index];
					for (dInt32 j = 0; j < firstJoint->m_rowCount; j++)
					{
						ndAvx2::ndSoaMatrixElement& row = massMatrix[soaRowBase + j];
						row.m_Jt.m_jacobianM0.m_linear.m_x = m_zero;
						row.m_Jt.m_jacobianM0.m_linear.m_y = m_zero;
						row.m_Jt.m_jacobianM0.m_linear.m_z = m_zero;
						row.m_Jt.m_jacobianM0.m_angular.m_x = m_zero;
						row.m_Jt.m_jacobianM0.m_angular.m_y = m_zero;
						row.m_Jt.m_jacobianM0.m_angular.m_z = m_zero;
						row.m_Jt.m_jacobianM1.m_linear.m_x = m_zero;
						row.m_Jt.m_jacobianM1.m_linear.m_y = m_zero;
						row.m_Jt.m_jacobianM1.m_linear.m_z = m_zero;
						row.m_Jt.m_jacobianM1.m_angular.m_x = m_zero;
						row.m_Jt.m_jacobianM1.m_angular.m_y = m_zero;
						row.m_Jt.m_jacobianM1.m_angular.m_z = m_zero;

						row.m_JMinv.m_jacobianM0.m_linear.m_x = m_zero;
						row.m_JMinv.m_jacobianM0.m_linear.m_y = m_zero;
						row.m_JMinv.m_jacobianM0.m_linear.m_z = m_zero;
						row.m_JMinv.m_jacobianM0.m_angular.m_x = m_zero;
						row.m_JMinv.m_jacobianM0.m_angular.m_y = m_zero;
						row.m_JMinv.m_jacobianM0.m_angular.m_z = m_zero;
						row.m_JMinv.m_jacobianM1.m_linear.m_x = m_zero;
						row.m_JMinv.m_jacobianM1.m_linear.m_y = m_zero;
						row.m_JMinv.m_jacobianM1.m_linear.m_z = m_zero;
						row.m_JMinv.m_jacobianM1.m_angular.m_x = m_zero;
						row.m_JMinv.m_jacobianM1.m_angular.m_y = m_zero;
						row.m_JMinv.m_jacobianM1.m_angular.m_z = m_zero;

						row.m_force = m_zero;
						row.m_diagDamp = m_zero;
						row.m_invJinvMJt = m_zero;
						row.m_coordenateAccel = m_zero;
						row.m_lowerBoundFrictionCoefficent = m_zero;
						row.m_upperBoundFrictionCoefficent = m_zero;
						dInt32* const normalIndex = (dInt32*)&row.m_normalForceIndex[0];
						for (dInt32 k = 0; k < D_SOA_WORD_GROUP_SIZE; k++)
						{
							normalIndex[k] = k;
						}
					}

					for (dInt32 j = 0; j < D_SOA_WORD_GROUP_SIZE; j++)
					{
						const ndConstraint* const joint = jointArray[index + j];
						if (joint)
						{
							for (dInt32 k = 0; k < joint->m_rowCount; k++)
							{
								ndAvx2::ndSoaMatrixElement& row = massMatrix[soaRowBase + k];
								const ndLeftHandSide* const lhs = &leftHandSide[joint->m_rowStart + k];

								row.m_Jt.m_jacobianM0.m_linear.m_x[j] = lhs->m_Jt.m_jacobianM0.m_linear.m_x;
								row.m_Jt.m_jacobianM0.m_linear.m_y[j] = lhs->m_Jt.m_jacobianM0.m_linear.m_y;
								row.m_Jt.m_jacobianM0.m_linear.m_z[j] = lhs->m_Jt.m_jacobianM0.m_linear.m_z;
								row.m_Jt.m_jacobianM0.m_angular.m_x[j] = lhs->m_Jt.m_jacobianM0.m_angular.m_x;
								row.m_Jt.m_jacobianM0.m_angular.m_y[j] = lhs->m_Jt.m_jacobianM0.m_angular.m_y;
								row.m_Jt.m_jacobianM0.m_angular.m_z[j] = lhs->m_Jt.m_jacobianM0.m_angular.m_z;
								row.m_Jt.m_jacobianM1.m_linear.m_x[j] = lhs->m_Jt.m_jacobianM1.m_linear.m_x;
								row.m_Jt.m_jacobianM1.m_linear.m_y[j] = lhs->m_Jt.m_jacobianM1.m_linear.m_y;
								row.m_Jt.m_jacobianM1.m_linear.m_z[j] = lhs->m_Jt.m_jacobianM1.m_linear.m_z;
								row.m_Jt.m_jacobianM1.m_angular.m_x[j] = lhs->m_Jt.m_jacobianM1.m_angular.m_x;
								row.m_Jt.m_jacobianM1.m_angular.m_y[j] = lhs->m_Jt.m_jacobianM1.m_angular.m_y;
								row.m_Jt.m_jacobianM1.m_angular.m_z[j] = lhs->m_Jt.m_jacobianM1.m_angular.m_z;

								row.m_JMinv.m_jacobianM0.m_linear.m_x[j] = lhs->m_JMinv.m_jacobianM0.m_linear.m_x;
								row.m_JMinv.m_jacobianM0.m_linear.m_y[j] = lhs->m_JMinv.m_jacobianM0.m_linear.m_y;
								row.m_JMinv.m_jacobianM0.m_linear.m_z[j] = lhs->m_JMinv.m_jacobianM0.m_linear.m_z;
								row.m_JMinv.m_jacobianM0.m_angular.m_x[j] = lhs->m_JMinv.m_jacobianM0.m_angular.m_x;
								row.m_JMinv.m_jacobianM0.m_angular.m_y[j] = lhs->m_JMinv.m_jacobianM0.m_angular.m_y;
								row.m_JMinv.m_jacobianM0.m_angular.m_z[j] = lhs->m_JMinv.m_jacobianM0.m_angular.m_z;
								row.m_JMinv.m_jacobianM1.m_linear.m_x[j] = lhs->m_JMinv.m_jacobianM1.m_linear.m_x;
								row.m_JMinv.m_jacobianM1.m_linear.m_y[j] = lhs->m_JMinv.m_jacobianM1.m_linear.m_y;
								row.m_JMinv.m_jacobianM1.m_linear.m_z[j] = lhs->m_JMinv.m_jacobianM1.m_linear.m_z;
								row.m_JMinv.m_jacobianM1.m_angular.m_x[j] = lhs->m_JMinv.m_jacobianM1.m_angular.m_x;
								row.m_JMinv.m_jacobianM1.m_angular.m_y[j] = lhs->m_JMinv.m_jacobianM1.m_angular.m_y;
								row.m_JMinv.m_jacobianM1.m_angular.m_z[j] = lhs->m_JMinv.m_jacobianM1.m_angular.m_z;

								const ndRightHandSide* const rhs = &rightHandSide[joint->m_rowStart + k];
								row.m_force[j] = rhs->m_force;
								row.m_diagDamp[j] = rhs->m_diagDamp;
								row.m_invJinvMJt[j] = rhs->m_invJinvMJt;
								row.m_coordenateAccel[j] = rhs->m_coordenateAccel;
								row.m_lowerBoundFrictionCoefficent[j] = rhs->m_lowerBoundFrictionCoefficent;
								row.m_upperBoundFrictionCoefficent[j] = rhs->m_upperBoundFrictionCoefficent;

								dInt32* const normalIndex = (dInt32*)&row.m_normalForceIndex[0];
								normalIndex[j] = (rhs->m_normalForceIndex + 1) * D_SOA_WORD_GROUP_SIZE + j;
							}
						}
					}
				}
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

		const ndConstraintArray& jointArray = m_world->m_jointArray;

		const dInt32 mask = -dInt32(D_SOA_WORD_GROUP_SIZE);
		const dInt32 soaJointCount = (jointArray.GetCount() + D_SOA_WORD_GROUP_SIZE - 1) & mask;
		m_soaJointArray.SetCount(soaJointCount);

		dInt32 jointCountSpans[D_RADIX_DIGIT];
		memset(jointCountSpans, 0, sizeof(jointCountSpans));
		
		for (dInt32 i = 0; i < jointArray.GetCount(); i++)
		{
			ndConstraint* const joint = jointArray[i];
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
		
		for (dInt32 i = 0; i < jointArray.GetCount(); i++)
		{
			ndConstraint* const joint = jointArray[i];
			const dInt32 key = GetSortKeyAvx2(joint);
			const dInt32 entry = jointCountSpans[key];
			m_soaJointArray[entry] = joint;
			jointCountSpans[key] = entry + 1;
		}
		
		#ifdef _DEBUG
		for (dInt32 i = 0; i < jointArray.GetCount() - 1; i++)
		{
			const ndConstraint* const joint0 = m_soaJointArray[i + 0];
			const ndConstraint* const joint1 = m_soaJointArray[i + 1];
		
			const dInt32 key0 = GetSortKeyAvx2(joint0);
			const dInt32 key1 = GetSortKeyAvx2(joint1);
			dAssert(key0 <= key1);
		}
		#endif

		for (dInt32 i = jointArray.GetCount(); i < m_soaJointArray.GetCount(); i++)
		{
			m_soaJointArray[i] = nullptr;
		}
		scene->SubmitJobs<ndSortJointGroupAvx2>();

		dInt32 soaJointRowCount = 0;
		const dInt32 soaJointCountBatches = soaJointCount / D_SOA_WORD_GROUP_SIZE;
		m_soaJointRows.SetCount(soaJointCountBatches);
		for (dInt32 i = 0; i < soaJointCountBatches; i++)
		{
			const ndConstraint* const joint = m_soaJointArray[i * D_SOA_WORD_GROUP_SIZE];
			m_soaJointRows[i] = soaJointRowCount;
			soaJointRowCount += joint->m_rowCount;
		}
		m_soaMassMatrix.SetCount(soaJointRowCount);
		
		scene->SubmitJobs<ndTransposeMassMatrixAvx2>();
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


	class ndUpdateAccelerationAvx2 : public ndScene::ndBaseJob
	{
		public:
		virtual void Execute()
		{
			D_TRACKTIME();

			ndWorld* const world = m_owner->GetWorld();
			const ndConstraintArray& jointArray = world->m_soaJointArray;
			const dArray<ndRightHandSide>& rightHandSide = world->m_rightHandSide;

			const dInt32 threadIndex = GetThredId();
			const dInt32 threadCount = m_owner->GetThreadCount();
			const dInt32 soaJointCount = jointArray.GetCount();

			const dInt32* const soaJointRows = &world->m_soaJointRows[0];
			ndSoaMatrixElement* const massMatrix = &world->m_soaMassMatrix[0];

			const dInt32 soaJointCountBatches = soaJointCount / D_SOA_WORD_GROUP_SIZE;
			for (dInt32 i = threadIndex; i < soaJointCountBatches; i += threadCount)
			{
				const ndConstraint* const * jointGroup = &jointArray[i * D_SOA_WORD_GROUP_SIZE];
				const ndConstraint* const firstJoint = jointGroup[0];
				const ndConstraint* const lastJoint = jointGroup[D_SOA_WORD_GROUP_SIZE - 1];
				const dInt32 soaRowStartBase = soaJointRows[i];
				if (lastJoint && (firstJoint->m_rowCount == lastJoint->m_rowCount))
				{
					const dInt32 rowCount = firstJoint->m_rowCount;
					for (dInt32 j = 0; j < D_SOA_WORD_GROUP_SIZE; j++)
					{
						const ndConstraint* const Joint = jointGroup[j];
						const dInt32 startBase = Joint->m_rowStart;
						for (dInt32 k = 0; k < rowCount; k++)
						{
							ndSoaMatrixElement* const row = &massMatrix[soaRowStartBase + k];
							row->m_coordenateAccel[j] = rightHandSide[startBase + k].m_coordenateAccel;
						}
					}
				} 
				else
				{
					for (dInt32 j = 0; j < D_SOA_WORD_GROUP_SIZE; j++)
					{
						const ndConstraint* const Joint = jointGroup[j];
						if (Joint)
						{
							const dInt32 rowCount = Joint->m_rowCount;
							const dInt32 startBase = Joint->m_rowStart;
							for (dInt32 k = 0; k < rowCount; k++) 
							{
								ndSoaMatrixElement* const row = &massMatrix[soaRowStartBase + k];
								row->m_coordenateAccel[j] = rightHandSide[startBase + k].m_coordenateAccel;
							}
						}
					}
				}
			}
		}
	};
	
	ndScene* const scene = m_world->GetScene();
	scene->SubmitJobs<ndCalculateJointsAccelerationAvx2>();
	m_firstPassCoef = dFloat32(1.0f);

	scene->SubmitJobs<ndUpdateAccelerationAvx2>();
}

dFloat32 ndDynamicsUpdate::CalculateJointsForceAvx2(ndConstraint** const jointGroup, ndSoaMatrixElement* const massMatrix, ndJacobian* const internalForces)
{
	//D_TRACKTIME();
	ndSoaFloat weight0;
	ndSoaFloat weight1;
	ndSoaVector6 forceM0;
	ndSoaVector6 forceM1;
	ndSoaFloat preconditioner0;
	ndSoaFloat preconditioner1;
	ndSoaFloat normalForce[D_CONSTRAINT_MAX_ROWS + 1];

	if (jointGroup[D_SOA_WORD_GROUP_SIZE - 1])
	{
		for (dInt32 i = 0; i < D_SOA_WORD_GROUP_SIZE; i++)
		{
			const ndConstraint* const joint = jointGroup[i];
			const ndBodyKinematic* const body0 = joint->GetBody0();
			const ndBodyKinematic* const body1 = joint->GetBody1();

			const dInt32 m0 = body0->m_index;
			const dInt32 m1 = body1->m_index;

			weight0[i] = body0->m_weigh;
			weight1[i] = body1->m_weigh;
			preconditioner0[i] = joint->m_preconditioner0;
			preconditioner1[i] = joint->m_preconditioner1;

			forceM0.m_linear.m_x[i] = m_internalForces[m0].m_linear.m_x;
			forceM0.m_linear.m_y[i] = m_internalForces[m0].m_linear.m_y;
			forceM0.m_linear.m_z[i] = m_internalForces[m0].m_linear.m_z;
			forceM0.m_angular.m_x[i] = m_internalForces[m0].m_angular.m_x;
			forceM0.m_angular.m_y[i] = m_internalForces[m0].m_angular.m_y;
			forceM0.m_angular.m_z[i] = m_internalForces[m0].m_angular.m_z;
			
			forceM1.m_linear.m_x[i] = m_internalForces[m1].m_linear.m_x;
			forceM1.m_linear.m_y[i] = m_internalForces[m1].m_linear.m_y;
			forceM1.m_linear.m_z[i] = m_internalForces[m1].m_linear.m_z;
			forceM1.m_angular.m_x[i] = m_internalForces[m1].m_angular.m_x;
			forceM1.m_angular.m_y[i] = m_internalForces[m1].m_angular.m_y;
			forceM1.m_angular.m_z[i] = m_internalForces[m1].m_angular.m_z;
		}
	}
	else
	{
		weight0 = m_zero;
		weight1 = m_zero;
		preconditioner0 = m_zero;
		preconditioner1 = m_zero;

		forceM0.m_linear.m_x = m_zero;
		forceM0.m_linear.m_y = m_zero;
		forceM0.m_linear.m_z = m_zero;
		forceM0.m_angular.m_x = m_zero;
		forceM0.m_angular.m_y = m_zero;
		forceM0.m_angular.m_z = m_zero;

		forceM1.m_linear.m_x = m_zero;
		forceM1.m_linear.m_y = m_zero;
		forceM1.m_linear.m_z = m_zero;
		forceM1.m_angular.m_x = m_zero;
		forceM1.m_angular.m_y = m_zero;
		forceM1.m_angular.m_z = m_zero;
		for (dInt32 i = 0; i < D_SOA_WORD_GROUP_SIZE; i++)
		{
			const ndConstraint* const joint = jointGroup[i];
			if (joint)
			{
				const ndBodyKinematic* const body0 = joint->GetBody0();
				const ndBodyKinematic* const body1 = joint->GetBody1();

				const dInt32 m0 = body0->m_index;
				const dInt32 m1 = body1->m_index;

				preconditioner0[i] = joint->m_preconditioner0;
				preconditioner1[i] = joint->m_preconditioner1;

				forceM0.m_linear.m_x[i] = m_internalForces[m0].m_linear.m_x;
				forceM0.m_linear.m_y[i] = m_internalForces[m0].m_linear.m_y;
				forceM0.m_linear.m_z[i] = m_internalForces[m0].m_linear.m_z;
				forceM0.m_angular.m_x[i] = m_internalForces[m0].m_angular.m_x;
				forceM0.m_angular.m_y[i] = m_internalForces[m0].m_angular.m_y;
				forceM0.m_angular.m_z[i] = m_internalForces[m0].m_angular.m_z;

				forceM1.m_linear.m_x[i] = m_internalForces[m1].m_linear.m_x;
				forceM1.m_linear.m_y[i] = m_internalForces[m1].m_linear.m_y;
				forceM1.m_linear.m_z[i] = m_internalForces[m1].m_linear.m_z;
				forceM1.m_angular.m_x[i] = m_internalForces[m1].m_angular.m_x;
				forceM1.m_angular.m_y[i] = m_internalForces[m1].m_angular.m_y;
				forceM1.m_angular.m_z[i] = m_internalForces[m1].m_angular.m_z;

				weight0[i] = body0->m_weigh;
				weight1[i] = body1->m_weigh;
			}
		}
	}

	forceM0.m_linear.m_x = forceM0.m_linear.m_x * preconditioner0;
	forceM0.m_linear.m_y = forceM0.m_linear.m_y * preconditioner0;
	forceM0.m_linear.m_z = forceM0.m_linear.m_z * preconditioner0;
	forceM0.m_angular.m_x = forceM0.m_angular.m_x * preconditioner0;
	forceM0.m_angular.m_y = forceM0.m_angular.m_y * preconditioner0;
	forceM0.m_angular.m_z = forceM0.m_angular.m_z * preconditioner0;

	forceM1.m_linear.m_x = forceM1.m_linear.m_x * preconditioner1;
	forceM1.m_linear.m_y = forceM1.m_linear.m_y * preconditioner1;
	forceM1.m_linear.m_z = forceM1.m_linear.m_z * preconditioner1;
	forceM1.m_angular.m_x = forceM1.m_angular.m_x * preconditioner1;
	forceM1.m_angular.m_y = forceM1.m_angular.m_y * preconditioner1;
	forceM1.m_angular.m_z = forceM1.m_angular.m_z * preconditioner1;

	preconditioner0 = preconditioner0 * weight0;
	preconditioner1 = preconditioner1 * weight1;
	
//static int xxxxx;
//if (xxxxx >= 64)
//xxxxx *= 1;
//xxxxx++;

	normalForce[0] = m_one;
	ndSoaFloat accNorm(m_zero);
	const dInt32 rowsCount = jointGroup[0]->m_rowCount;
	for (dInt32 j = 0; j < rowsCount; j++)
	{
		ndSoaMatrixElement* const row = &massMatrix[j];

		ndSoaFloat a0(row->m_JMinv.m_jacobianM0.m_linear.m_x * forceM0.m_linear.m_x);
		ndSoaFloat a1(row->m_JMinv.m_jacobianM1.m_linear.m_x * forceM1.m_linear.m_x);
		a0 = a0.MulAdd(row->m_JMinv.m_jacobianM0.m_angular.m_x, forceM0.m_angular.m_x);
		a1 = a1.MulAdd(row->m_JMinv.m_jacobianM1.m_angular.m_z, forceM1.m_angular.m_z);

		a0 = a0.MulAdd(row->m_JMinv.m_jacobianM0.m_linear.m_y, forceM0.m_linear.m_y);
		a1 = a1.MulAdd(row->m_JMinv.m_jacobianM1.m_linear.m_y, forceM1.m_linear.m_y);
		a0 = a0.MulAdd(row->m_JMinv.m_jacobianM0.m_angular.m_y, forceM0.m_angular.m_y);
		a1 = a1.MulAdd(row->m_JMinv.m_jacobianM1.m_angular.m_y, forceM1.m_angular.m_y);

		a0 = a0.MulAdd(row->m_JMinv.m_jacobianM0.m_linear.m_z, forceM0.m_linear.m_z);
		a1 = a1.MulAdd(row->m_JMinv.m_jacobianM1.m_linear.m_z, forceM1.m_linear.m_z);
		a0 = a0.MulAdd(row->m_JMinv.m_jacobianM0.m_angular.m_z, forceM0.m_angular.m_z);
		a1 = a1.MulAdd(row->m_JMinv.m_jacobianM1.m_angular.m_z, forceM1.m_angular.m_z);

		a0 = a0 + a1;
		a0 = a0.MulAdd(row->m_force, row->m_diagDamp);
		a0 = row->m_coordenateAccel - a0;
		
		ndSoaFloat f(row->m_force.MulAdd(row->m_invJinvMJt, a0));
		const ndSoaFloat frictionNormal(normalForce, row->m_normalForceIndex);

		const ndSoaFloat lowerFrictionForce(frictionNormal * row->m_lowerBoundFrictionCoefficent);
		const ndSoaFloat upperFrictionForce(frictionNormal * row->m_upperBoundFrictionCoefficent);

		a0 = a0 & (f < upperFrictionForce) & (f > lowerFrictionForce);
		f = f.GetMax(lowerFrictionForce).GetMin(upperFrictionForce);

		accNorm = accNorm.MulAdd(a0, a0);
		const ndSoaFloat deltaForce(f - row->m_force);
	
		row->m_force = f;
		normalForce[j + 1] = f;
		
		const ndSoaFloat deltaForce0(deltaForce * preconditioner0);
		const ndSoaFloat deltaForce1(deltaForce * preconditioner1);

		forceM0.m_linear.m_x = forceM0.m_linear.m_x.MulAdd(row->m_Jt.m_jacobianM0.m_linear.m_x, deltaForce0);
		forceM0.m_linear.m_y = forceM0.m_linear.m_y.MulAdd(row->m_Jt.m_jacobianM0.m_linear.m_y, deltaForce0);
		forceM0.m_linear.m_z = forceM0.m_linear.m_z.MulAdd(row->m_Jt.m_jacobianM0.m_linear.m_z, deltaForce0);
		forceM0.m_angular.m_x = forceM0.m_angular.m_x.MulAdd(row->m_Jt.m_jacobianM0.m_angular.m_x, deltaForce0);
		forceM0.m_angular.m_y = forceM0.m_angular.m_y.MulAdd(row->m_Jt.m_jacobianM0.m_angular.m_y, deltaForce0);
		forceM0.m_angular.m_z = forceM0.m_angular.m_z.MulAdd(row->m_Jt.m_jacobianM0.m_angular.m_z, deltaForce0);

		forceM1.m_linear.m_x = forceM1.m_linear.m_x.MulAdd(row->m_Jt.m_jacobianM1.m_linear.m_x, deltaForce1);
		forceM1.m_linear.m_y = forceM1.m_linear.m_y.MulAdd(row->m_Jt.m_jacobianM1.m_linear.m_y, deltaForce1);
		forceM1.m_linear.m_z = forceM1.m_linear.m_z.MulAdd(row->m_Jt.m_jacobianM1.m_linear.m_z, deltaForce1);
		forceM1.m_angular.m_x = forceM1.m_angular.m_x.MulAdd(row->m_Jt.m_jacobianM1.m_angular.m_x, deltaForce1);
		forceM1.m_angular.m_y = forceM1.m_angular.m_y.MulAdd(row->m_Jt.m_jacobianM1.m_angular.m_y, deltaForce1);
		forceM1.m_angular.m_z = forceM1.m_angular.m_z.MulAdd(row->m_Jt.m_jacobianM1.m_angular.m_z, deltaForce1);
	}
	
	const dFloat32 tol = dFloat32(0.5f);
	const dFloat32 tol2 = tol * tol;
	
	ndSoaFloat maxAccel(accNorm);
	for (dInt32 k = 0; (k < 4) && (maxAccel.AddHorizontal() > tol2); k++)
	{
		maxAccel = m_zero;
		for (dInt32 j = 0; j < rowsCount; j++)
		{
			ndSoaMatrixElement* const row = &massMatrix[j];
		
			ndSoaFloat a0(row->m_JMinv.m_jacobianM0.m_linear.m_x * forceM0.m_linear.m_x);
			ndSoaFloat a1(row->m_JMinv.m_jacobianM1.m_linear.m_x * forceM1.m_linear.m_x);
			a0 = a0.MulAdd(row->m_JMinv.m_jacobianM0.m_angular.m_x, forceM0.m_angular.m_x);
			a1 = a1.MulAdd(row->m_JMinv.m_jacobianM1.m_angular.m_z, forceM1.m_angular.m_z);

			a0 = a0.MulAdd(row->m_JMinv.m_jacobianM0.m_linear.m_y, forceM0.m_linear.m_y);
			a1 = a1.MulAdd(row->m_JMinv.m_jacobianM1.m_linear.m_y, forceM1.m_linear.m_y);
			a0 = a0.MulAdd(row->m_JMinv.m_jacobianM0.m_angular.m_y, forceM0.m_angular.m_y);
			a1 = a1.MulAdd(row->m_JMinv.m_jacobianM1.m_angular.m_y, forceM1.m_angular.m_y);

			a0 = a0.MulAdd(row->m_JMinv.m_jacobianM0.m_linear.m_z, forceM0.m_linear.m_z);
			a1 = a1.MulAdd(row->m_JMinv.m_jacobianM1.m_linear.m_z, forceM1.m_linear.m_z);
			a0 = a0.MulAdd(row->m_JMinv.m_jacobianM0.m_angular.m_z, forceM0.m_angular.m_z);
			a1 = a1.MulAdd(row->m_JMinv.m_jacobianM1.m_angular.m_z, forceM1.m_angular.m_z);

			a0 = a0 + a1;
			a0 = a0.MulAdd(row->m_force, row->m_diagDamp);
			a0 = row->m_coordenateAccel - a0;

			ndSoaFloat f(row->m_force.MulAdd(row->m_invJinvMJt, a0));

			const ndSoaFloat frictionNormal(normalForce, row->m_normalForceIndex);

			const ndSoaFloat lowerFrictionForce(frictionNormal * row->m_lowerBoundFrictionCoefficent);
			const ndSoaFloat upperFrictionForce(frictionNormal * row->m_upperBoundFrictionCoefficent);

			a0 = a0 & (f < upperFrictionForce) & (f > lowerFrictionForce);
			f = f.GetMax(lowerFrictionForce).GetMin(upperFrictionForce);

			maxAccel = maxAccel.MulAdd(a0, a0);
			
			const ndSoaFloat deltaForce(f - row->m_force);
			
			row->m_force = f;
			normalForce[j + 1] = f;
			
			const ndSoaFloat deltaForce0(deltaForce * preconditioner0);
			const ndSoaFloat deltaForce1(deltaForce * preconditioner1);

			forceM0.m_linear.m_x = forceM0.m_linear.m_x.MulAdd(row->m_Jt.m_jacobianM0.m_linear.m_x, deltaForce0);
			forceM0.m_linear.m_y = forceM0.m_linear.m_y.MulAdd(row->m_Jt.m_jacobianM0.m_linear.m_y, deltaForce0);
			forceM0.m_linear.m_z = forceM0.m_linear.m_z.MulAdd(row->m_Jt.m_jacobianM0.m_linear.m_z, deltaForce0);
			forceM0.m_angular.m_x = forceM0.m_angular.m_x.MulAdd(row->m_Jt.m_jacobianM0.m_angular.m_x, deltaForce0);
			forceM0.m_angular.m_y = forceM0.m_angular.m_y.MulAdd(row->m_Jt.m_jacobianM0.m_angular.m_y, deltaForce0);
			forceM0.m_angular.m_z = forceM0.m_angular.m_z.MulAdd(row->m_Jt.m_jacobianM0.m_angular.m_z, deltaForce0);

			forceM1.m_linear.m_x = forceM1.m_linear.m_x.MulAdd(row->m_Jt.m_jacobianM1.m_linear.m_x, deltaForce1);
			forceM1.m_linear.m_y = forceM1.m_linear.m_y.MulAdd(row->m_Jt.m_jacobianM1.m_linear.m_y, deltaForce1);
			forceM1.m_linear.m_z = forceM1.m_linear.m_z.MulAdd(row->m_Jt.m_jacobianM1.m_linear.m_z, deltaForce1);
			forceM1.m_angular.m_x = forceM1.m_angular.m_x.MulAdd(row->m_Jt.m_jacobianM1.m_angular.m_x, deltaForce1);
			forceM1.m_angular.m_y = forceM1.m_angular.m_y.MulAdd(row->m_Jt.m_jacobianM1.m_angular.m_y, deltaForce1);
			forceM1.m_angular.m_z = forceM1.m_angular.m_z.MulAdd(row->m_Jt.m_jacobianM1.m_angular.m_z, deltaForce1);
		}
	}
	
	forceM0.m_linear.m_x = m_zero;
	forceM0.m_linear.m_y = m_zero;
	forceM0.m_linear.m_z = m_zero;
	forceM0.m_angular.m_x = m_zero;
	forceM0.m_angular.m_y = m_zero;
	forceM0.m_angular.m_z = m_zero;

	forceM1.m_linear.m_x = m_zero;
	forceM1.m_linear.m_y = m_zero;
	forceM1.m_linear.m_z = m_zero;
	forceM1.m_angular.m_x = m_zero;
	forceM1.m_angular.m_y = m_zero;
	forceM1.m_angular.m_z = m_zero;
	
	for (dInt32 j = 0; j < rowsCount; j++)
	{
		const ndSoaMatrixElement* const row = &massMatrix[j];
	
		const ndSoaFloat f(row->m_force);
		forceM0.m_linear.m_x = forceM0.m_linear.m_x.MulAdd(row->m_Jt.m_jacobianM0.m_linear.m_x, f);
		forceM0.m_linear.m_y = forceM0.m_linear.m_y.MulAdd(row->m_Jt.m_jacobianM0.m_linear.m_y, f);
		forceM0.m_linear.m_z = forceM0.m_linear.m_z.MulAdd(row->m_Jt.m_jacobianM0.m_linear.m_z, f);
		forceM0.m_angular.m_x = forceM0.m_angular.m_x.MulAdd(row->m_Jt.m_jacobianM0.m_angular.m_x, f);
		forceM0.m_angular.m_y = forceM0.m_angular.m_y.MulAdd(row->m_Jt.m_jacobianM0.m_angular.m_y, f);
		forceM0.m_angular.m_z = forceM0.m_angular.m_z.MulAdd(row->m_Jt.m_jacobianM0.m_angular.m_z, f);

		forceM1.m_linear.m_x = forceM1.m_linear.m_x.MulAdd(row->m_Jt.m_jacobianM1.m_linear.m_x, f);
		forceM1.m_linear.m_y = forceM1.m_linear.m_y.MulAdd(row->m_Jt.m_jacobianM1.m_linear.m_y, f);
		forceM1.m_linear.m_z = forceM1.m_linear.m_z.MulAdd(row->m_Jt.m_jacobianM1.m_linear.m_z, f);
		forceM1.m_angular.m_x = forceM1.m_angular.m_x.MulAdd(row->m_Jt.m_jacobianM1.m_angular.m_x, f);
		forceM1.m_angular.m_y = forceM1.m_angular.m_y.MulAdd(row->m_Jt.m_jacobianM1.m_angular.m_y, f);
		forceM1.m_angular.m_z = forceM1.m_angular.m_z.MulAdd(row->m_Jt.m_jacobianM1.m_angular.m_z, f);
	}
	
	for (dInt32 j = 0; j < D_SOA_WORD_GROUP_SIZE; j++)
	{
		const ndConstraint* const joint = jointGroup[j];
		if (joint)
		{
			const ndBodyKinematic* const body0 = joint->GetBody0();
			const ndBodyKinematic* const body1 = joint->GetBody1();

			const dInt32 m0 = body0->m_index;
			const dInt32 m1 = body1->m_index;

			ndJacobian m_body0Force;
			ndJacobian m_body1Force;

			m_body0Force.m_linear = dVector(forceM0.m_linear.m_x[j], forceM0.m_linear.m_y[j], forceM0.m_linear.m_z[j], dFloat32(0.0f));
			m_body0Force.m_angular = dVector(forceM0.m_angular.m_x[j], forceM0.m_angular.m_y[j], forceM0.m_angular.m_z[j], dFloat32(0.0f));

			m_body1Force.m_linear = dVector(forceM1.m_linear.m_x[j], forceM1.m_linear.m_y[j], forceM1.m_linear.m_z[j], dFloat32(0.0f));
			m_body1Force.m_angular = dVector(forceM1.m_angular.m_x[j], forceM1.m_angular.m_y[j], forceM1.m_angular.m_z[j], dFloat32(0.0f));

			ndJacobian& outBody0 = internalForces[m0];
			outBody0.m_linear += m_body0Force.m_linear;
			outBody0.m_angular += m_body0Force.m_angular;

			ndJacobian& outBody1 = internalForces[m1];
			outBody1.m_linear += m_body1Force.m_linear;
			outBody1.m_angular += m_body1Force.m_angular;
		}
	}

	return accNorm.AddHorizontal();
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
			ndConstraintArray& jointArray = world->m_soaJointArray;
			ndSoaMatrixElement* const soaMassMatrix = &world->m_soaMassMatrix[0];
			const dInt32* const soaJointRows = &world->m_soaJointRows[0];
			dFloat32 accNorm = dFloat32(0.0f);
			const dInt32 jointCount = jointArray.GetCount() / D_SOA_WORD_GROUP_SIZE;
			const dInt32 bodyCount = m_owner->GetActiveBodyArray().GetCount();
			const dInt32 threadCount = dMax(m_owner->GetThreadCount(), 1);

			if (threadCount == 1)
			{
				ndJacobian* const internalForces = &world->m_internalForces[bodyCount];
				for (dInt32 j = 0; j < jointCount; j++)
				{
					ndConstraint** const jointGroup = &jointArray[j * D_SOA_WORD_GROUP_SIZE];

					bool isSleeping = true;
					for (dInt32 i = 0; (i < D_SOA_WORD_GROUP_SIZE) && isSleeping; i++)
					{
						const ndConstraint* const joint = jointGroup[i];
						if (joint)
						{
							ndBodyKinematic* const body0 = joint->GetBody0();
							ndBodyKinematic* const body1 = joint->GetBody1();
							dAssert(body0);
							dAssert(body1);

							const dInt32 m0 = body0->m_index;
							const dInt32 m1 = body1->m_index;

							isSleeping &= body0->m_resting;
							isSleeping &= body1->m_resting;
						}
					}
					if (!isSleeping)
					{
						accNorm += world->CalculateJointsForceAvx2(jointGroup, &soaMassMatrix[soaJointRows[j]], internalForces);
					}
				}
				dFloat32* const accelNorm = (dFloat32*)m_context;
				accelNorm[0] = accNorm;
			}
			else
			{
				dAssert(0);
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
			//static int xxxx;
			//dTrace(("%d:", xxxx));
			//if (xxxx >= 112)
			//	xxxx *= 1;
			//xxxx++;

			memset(&m_internalForces[bodyCount], 0, bodyCount * sizeof(ndJacobian));
			scene->SubmitJobs<ndCalculateJointsForceAvx2>(m_accelNorm);
			memcpy(&m_internalForces[0], &m_internalForces[bodyCount], bodyCount * sizeof(ndJacobian));

			//for (int i = 0; i < bodyCount; i++)
			//{
			//	dTrace(("(%f %f %f)  ", m_internalForces[i].m_linear.m_x, m_internalForces[i].m_linear.m_y, m_internalForces[i].m_linear.m_z));
			//}
			//dTrace(("\n"));
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
