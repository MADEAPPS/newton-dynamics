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
#define D_RADIX_DIGIT			32

D_MSV_NEWTON_ALIGN_32
class ndSoaFloat
{
	public:
	D_INLINE ndSoaFloat()
	{
	}

	D_INLINE ndSoaFloat(const dFloat32 val)
		:m_type(_mm256_set1_ps(val))
	{
	}

	D_INLINE ndSoaFloat(const __m256 type)
		: m_type(type)
	{
	}

	D_INLINE ndSoaFloat(const ndSoaFloat& copy)
		: m_type(copy.m_type)
	{
	}

	D_INLINE ndSoaFloat(const dVector& low, const dVector& high)
		: m_type(_mm256_set_m128(high.m_type, low.m_type))
	{
	}

	D_INLINE ndSoaFloat(const ndSoaFloat* const baseAddr, const ndSoaFloat& index)
		: m_type(_mm256_i32gather_ps(&(*baseAddr)[0], index.m_typeInt, 4))
	{
	}

	D_INLINE dFloat32& operator[] (dInt32 i)
	{
		dAssert(i < D_SOA_WORD_GROUP_SIZE);
		dAssert(i >= 0);
		//return m_f[i];
		dFloat32* const ptr = (dFloat32*)&m_type;
		return ptr[i];
	}

	D_INLINE const dFloat32& operator[] (dInt32 i) const
	{
		dAssert(i < D_SOA_WORD_GROUP_SIZE);
		dAssert(i >= 0);
		//return m_f[i];
		const dFloat32* const ptr = (dFloat32*)&m_type;
		return ptr[i];
	}

	D_INLINE ndSoaFloat operator+ (const ndSoaFloat& A) const
	{
		return _mm256_add_ps(m_type, A.m_type);
	}

	D_INLINE ndSoaFloat operator- (const ndSoaFloat& A) const
	{
		return _mm256_sub_ps(m_type, A.m_type);
	}

	D_INLINE ndSoaFloat operator* (const ndSoaFloat& A) const
	{
		return _mm256_mul_ps(m_type, A.m_type);
	}

	D_INLINE ndSoaFloat MulAdd(const ndSoaFloat& A, const ndSoaFloat& B) const
	{
		return _mm256_fmadd_ps(A.m_type, B.m_type, m_type);
	}

	D_INLINE ndSoaFloat MulSub(const ndSoaFloat& A, const ndSoaFloat& B) const
	{
		return _mm256_fnmadd_ps(A.m_type, B.m_type, m_type);
	}

	D_INLINE ndSoaFloat operator> (const ndSoaFloat& A) const
	{
		return _mm256_cmp_ps(m_type, A.m_type, _CMP_GT_OQ);
	}

	D_INLINE ndSoaFloat operator< (const ndSoaFloat& A) const
	{
		return _mm256_cmp_ps(m_type, A.m_type, _CMP_LT_OQ);
	}

	D_INLINE ndSoaFloat operator| (const ndSoaFloat& A) const
	{
		return _mm256_or_ps(m_type, A.m_type);
	}

	D_INLINE ndSoaFloat operator& (const ndSoaFloat& A) const
	{
		return _mm256_and_ps(m_type, A.m_type);
	}

	D_INLINE ndSoaFloat GetMin(const ndSoaFloat& A) const
	{
		return _mm256_min_ps(m_type, A.m_type);
	}

	D_INLINE ndSoaFloat GetMax(const ndSoaFloat& A) const
	{
		return _mm256_max_ps(m_type, A.m_type);
	}

	D_INLINE dFloat32 AddHorizontal() const
	{
		__m256 tmp0(_mm256_add_ps(m_type, _mm256_permute2f128_ps(m_type, m_type, 1)));
		__m256 tmp1(_mm256_hadd_ps(tmp0, tmp0));
		__m256 tmp2(_mm256_hadd_ps(tmp1, tmp1));
		//int xxx = _mm256_cvtsi256_si32 (tmp2);
		return *((dFloat32*)&tmp2);
	}

	static D_INLINE void FlushRegisters()
	{
		_mm256_zeroall();
	}

	union
	{
		__m256 m_type;
		__m256i m_typeInt;
	};
} D_GCC_NEWTON_ALIGN_32;

D_MSV_NEWTON_ALIGN_32
class ndSoaVector3
{
	public:
	ndSoaFloat m_x;
	ndSoaFloat m_y;
	ndSoaFloat m_z;
} D_GCC_NEWTON_ALIGN_32;

D_MSV_NEWTON_ALIGN_32
class ndSoaVector6
{
	public:
	ndSoaVector3 m_linear;
	ndSoaVector3 m_angular;
} D_GCC_NEWTON_ALIGN_32;

D_MSV_NEWTON_ALIGN_32
class ndSoaJacobianPair
{
	public:
	ndSoaVector6 m_jacobianM0;
	ndSoaVector6 m_jacobianM1;
} D_GCC_NEWTON_ALIGN_32;

D_MSV_NEWTON_ALIGN_32
class ndSoaMatrixElement
{
	public:
	ndSoaJacobianPair m_Jt;
	ndSoaJacobianPair m_JMinv;

	ndSoaFloat m_force;
	ndSoaFloat m_diagDamp;
	ndSoaFloat m_invJinvMJt;
	ndSoaFloat m_coordenateAccel;
	ndSoaFloat m_normalForceIndex;
	ndSoaFloat m_lowerBoundFrictionCoefficent;
	ndSoaFloat m_upperBoundFrictionCoefficent;
} D_GCC_NEWTON_ALIGN_32;

dInt32 ndDynamicsUpdate::CompareIslandsAvx2(const ndIsland* const islandA, const ndIsland* const islandB, void* const context)
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

		dInt32 rowCount = 0;
		dInt32 jointCountSpans[D_RADIX_DIGIT];
		m_leftHandSide.SetCount(m_leftHandSide.GetCount() + 1);
		ndConstraint** const sortBuffer = (ndConstraint**)&m_leftHandSide[0];
		memset(jointCountSpans, 0, sizeof(jointCountSpans));

		for (dInt32 i = 0; i < jointArray.GetCount(); i++)
		{
			ndConstraint* const joint = jointArray[i];
			const dInt32 rows = joint->GetRowsCount();
			joint->m_rowCount = rows;
			joint->m_rowStart = rowCount;
			rowCount += rows;

			sortBuffer[i] = joint;
			const dInt32 key = D_RADIX_DIGIT - rows;
			dAssert(key > 0);
			jointCountSpans[key] ++;

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

		dInt32 acc = 0;
		for (dInt32 i = 0; i < sizeof(jointCountSpans) / sizeof(jointCountSpans[0]); i++)
		{
			const dInt32 val = jointCountSpans[i];
			jointCountSpans[i] = acc;
			acc += val;
		}
		
		for (dInt32 i = 0; i < jointArray.GetCount(); i++)
		{
			ndConstraint* const joint = sortBuffer[i];
			const dInt32 key = D_RADIX_DIGIT - joint->m_rowCount;
			const dInt32 entry = jointCountSpans[key];
			jointArray[entry] = joint;
			jointCountSpans[key] = entry + 1;
		}

		#ifdef _DEBUG
		for (dInt32 i = 1; i < jointArray.GetCount(); i++)
		{
			ndConstraint* const joint0 = jointArray[i-1];
			ndConstraint* const joint1 = jointArray[i-0];
			dAssert(joint0->m_rowCount >= joint1->m_rowCount);
		}
		#endif

				
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
				const dInt32 j = scans[key];
				buffer2[j] = buffer0[i];
				scans[key] = j + 1;
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
			dSort(&m_islands[0], m_islands.GetCount(), CompareIslandsAvx2);
		}
	}
}

void ndDynamicsUpdate::IntegrateUnconstrainedBodiesAvx2()
{
	class ndIntegrateUnconstrainedBodies : public ndScene::ndBaseJob
	{
		public:
		virtual void Execute()
		{
			//D_TRACKTIME();
			ndWorld* const world = m_owner->GetWorld();
			dArray<ndBodyKinematic*>& bodyArray = world->m_bodyIslandOrder;

			const dInt32 threadIndex = GetThreadId();
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

void ndDynamicsUpdate::InitWeightsAvx2()
{
	D_TRACKTIME();
	const ndScene* const scene = m_world->GetScene();

	m_invTimestep = dFloat32(1.0f) / m_timestep;
	m_invStepRK = dFloat32(0.25f);
	m_timestepRK = m_timestep * m_invStepRK;
	m_invTimestepRK = m_invTimestep * dFloat32(4.0f);

	const dArray<ndBodyKinematic*>& bodyArray = scene->GetActiveBodyArray();
	const ndConstraintArray& constraintArray = scene->GetActiveContactArray();

	const dInt32 bodyCount = bodyArray.GetCount();
	const dInt32 jointCount = constraintArray.GetCount();

	m_jointArray.SetCount(jointCount);
	const dInt32 buffersCount = dMax(scene->GetThreadCount(), 1) + 1;
	m_internalForces.SetCount(bodyCount * buffersCount);

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

void ndDynamicsUpdate::InitBodyArrayAvx2()
{
	D_TRACKTIME();
	class ndInitBodyArray : public ndScene::ndBaseJob
	{
		public:
		virtual void Execute()
		{
			//D_TRACKTIME();
			ndWorld* const world = m_owner->GetWorld();
			dArray<ndBodyKinematic*>& bodyArray = world->m_bodyIslandOrder;

			const dInt32 threadIndex = GetThreadId();
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

void ndDynamicsUpdate::InitJacobianMatrixAvx2()
{
	class ndInitJacobianMatrix : public ndScene::ndBaseJob
	{
		public:
		void BuildJacobianMatrix(ndConstraint* const joint)
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

			const dInt32 isBilateral = joint->IsBilateral() ? 1 : 0;

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
				forceAcc0 = forceAcc0 + JtM0.m_linear * f0;
				torqueAcc0 = torqueAcc0 + JtM0.m_angular * f0;
				forceAcc1 = forceAcc1 + JtM1.m_linear * f1;
				torqueAcc1 = torqueAcc1 + JtM1.m_angular * f1;
			}

			ndJacobian& outBody0 = m_internalForces[m0];
			outBody0.m_linear += forceAcc0;
			outBody0.m_angular += torqueAcc0;

			ndJacobian& outBody1 = m_internalForces[m1];
			outBody1.m_linear += forceAcc1;
			outBody1.m_angular += torqueAcc1;
		}

		virtual void Execute()
		{
			//D_TRACKTIME();
			ndWorld* const world = m_owner->GetWorld();
			m_leftHandSide = &world->m_leftHandSide[0];
			m_rightHandSide = &world->m_rightHandSide[0];

			ndConstraint** const jointArray = &world->m_jointArray[0];
			const dInt32 jointCount = world->m_jointArray.GetCount();
			const dInt32 bodyCount = m_owner->GetActiveBodyArray().GetCount();

			const dInt32 threadIndex = GetThreadId();
			const dInt32 threadCount = dMax(m_owner->GetThreadCount(), 1);

			m_internalForces = &world->m_internalForces[threadIndex * bodyCount];

			world->ClearJacobianBuffer(bodyCount, m_internalForces);
			for (dInt32 i = threadIndex; i < jointCount; i += threadCount)
			{
				ndConstraint* const joint = jointArray[i];
				world->GetJacobianDerivatives(joint);
				BuildJacobianMatrix(joint);
			}
		}

		ndJacobian* m_internalForces;
		ndRightHandSide* m_rightHandSide;
		ndLeftHandSide* m_leftHandSide;
	};

	class ndInitJacobianAccumulatePartialForces : public ndScene::ndBaseJob
	{
		public:
		virtual void Execute()
		{
			//D_TRACKTIME();
			ndWorld* const world = m_owner->GetWorld();
			const dInt32 threadIndex = GetThreadId();
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
		ndScene* const scene = m_world->GetScene();

		scene->SubmitJobs<ndInitJacobianMatrix>();
		if (scene->GetThreadCount() > 1)
		{
			scene->SubmitJobs<ndInitJacobianAccumulatePartialForces>();
		}
	}
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

		UpdateForceFeedbackAvx2();
		for (dInt32 i = 0; i < m_world->GetThreadCount(); i++)
		{
			hasJointFeeback |= m_hasJointFeeback[i];
		}
	}

	IntegrateBodiesAvx2();

	if (hasJointFeeback)
	{
		dAssert(0);
		//	UpdateKinematicFeedback();
	}
}

void ndDynamicsUpdate::DetermineSleepStatesAvx2()
{
	D_TRACKTIME();
	class ndDetermineSleepStates : public ndScene::ndBaseJob
	{
		public:
		virtual void Execute()
		{
			//D_TRACKTIME();
			ndWorld* const world = m_owner->GetWorld();
			const dArray<ndIsland>& islandArray = world->m_islands;

			const dInt32 threadIndex = GetThreadId();
			const dInt32 threadCount = m_owner->GetThreadCount();
			const dInt32 islandCount = islandArray.GetCount();
			const dInt32 step = islandCount / threadCount;
			const dInt32 start = threadIndex * step;
			const dInt32 count = ((threadIndex + 1) < threadCount) ? step : islandCount - start;

			for (dInt32 i = 0; i < count; i++)
			{
				const ndIsland& island = islandArray[start + i];
				world->UpdateIslandStateAvx2(island);
			}
		}
	};

	ndScene* const scene = m_world->GetScene();
	scene->SubmitJobs<ndDetermineSleepStates>();
}

void ndDynamicsUpdate::InitSkeletonsAvx2()
{
	D_TRACKTIME();

	class ndInitSkeletons : public ndScene::ndBaseJob
	{
		public:
		virtual void Execute()
		{
			//D_TRACKTIME();
			const dInt32 threadIndex = GetThreadId();
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

void ndDynamicsUpdate::UpdateSkeletonsAvx2()
{
	D_TRACKTIME();
	class ndUpdateSkeletons : public ndScene::ndBaseJob
	{
		public:
		virtual void Execute()
		{
			//D_TRACKTIME();
			const dInt32 threadIndex = GetThreadId();
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

void ndDynamicsUpdate::UpdateForceFeedbackAvx2()
{
	D_TRACKTIME();
	class ndUpdateForceFeedback : public ndScene::ndBaseJob
	{
		public:
		virtual void Execute()
		{
			//D_TRACKTIME();
			ndWorld* const world = m_owner->GetWorld();
			const ndConstraintArray& jointArray = world->m_jointArray;
			dArray<ndRightHandSide>& rightHandSide = world->m_rightHandSide;

			const dInt32 threadIndex = GetThreadId();
			const dInt32 threadCount = m_owner->GetThreadCount();
			const dInt32 jointCount = jointArray.GetCount();
			const dInt32 step = jointCount / threadCount;
			const dInt32 start = threadIndex * step;
			const dInt32 count = ((threadIndex + 1) < threadCount) ? step : jointCount - start;

			bool hasJointFeeback = false;
			const dFloat32 timestepRK = world->m_timestepRK;
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

			world->m_hasJointFeeback[threadIndex] = hasJointFeeback ? 1 : 0;
		}
	};

	ndScene* const scene = m_world->GetScene();
	scene->SubmitJobs<ndUpdateForceFeedback>();
}

void ndDynamicsUpdate::IntegrateBodiesVelocityAvx2()
{
	D_TRACKTIME();
	class ndIntegrateBodiesVelocity : public ndScene::ndBaseJob
	{
		public:
		virtual void Execute()
		{
			//D_TRACKTIME();
			ndWorld* const world = m_owner->GetWorld();
			dArray<ndBodyKinematic*>& bodyArray = world->m_bodyIslandOrder;

			const dInt32 threadIndex = GetThreadId();
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
	scene->SubmitJobs<ndIntegrateBodiesVelocity>();
}

void ndDynamicsUpdate::CalculateJointsAccelerationAvx2()
{
	D_TRACKTIME();
	class ndCalculateJointsAcceleration : public ndScene::ndBaseJob
	{
		public:
		virtual void Execute()
		{
			//D_TRACKTIME();
			ndWorld* const world = m_owner->GetWorld();
			const ndConstraintArray& jointArray = world->m_jointArray;

			ndJointAccelerationDecriptor joindDesc;
			joindDesc.m_timestep = world->m_timestepRK;
			joindDesc.m_invTimeStep = world->m_invTimestepRK;
			joindDesc.m_firstPassCoefFlag = world->m_firstPassCoef;
			dArray<ndLeftHandSide>& leftHandSide = world->m_leftHandSide;
			dArray<ndRightHandSide>& rightHandSide = world->m_rightHandSide;

			const dInt32 threadIndex = GetThreadId();
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

void ndDynamicsUpdate::CalculateJointsForceAvx2()
{
	D_TRACKTIME();
	class ndCalculateJointsForce : public ndScene::ndBaseJob
	{
		public:
		dFloat32 JointForce(ndConstraint* const joint) const
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

			ndJacobian& outBody0 = m_outputForces[m0];
			outBody0.m_linear += forceM0;
			outBody0.m_angular += torqueM0;

			ndJacobian& outBody1 = m_outputForces[m1];
			outBody1.m_linear += forceM1;
			outBody1.m_angular += torqueM1;

			return accNorm.GetScalar();
		}

		virtual void Execute()
		{
			//D_TRACKTIME();
			ndWorld* const world = m_owner->GetWorld();
			m_leftHandSide = &world->m_leftHandSide[0];
			m_rightHandSide = &world->m_rightHandSide[0];
			m_internalForces = &world->m_internalForces[0];

			ndConstraintArray& jointArray = world->m_jointArray;
			dFloat32 accNorm = dFloat32(0.0f);
			const dInt32 jointCount = jointArray.GetCount();
			const dInt32 bodyCount = m_owner->GetActiveBodyArray().GetCount();

			const dInt32 threadIndex = GetThreadId();
			const dInt32 threadCount = dMax(m_owner->GetThreadCount(), 1);
			m_outputForces = &world->m_internalForces[bodyCount * (threadIndex + 1)];

			world->ClearJacobianBuffer(bodyCount, m_outputForces);
			for (dInt32 i = threadIndex; i < jointCount; i += threadCount)
			{
				ndConstraint* const joint = jointArray[i];
				accNorm += JointForce(joint);
			}
			dFloat32* const accelNorm = (dFloat32*)m_context;
			accelNorm[threadIndex] = accNorm;
		}
		
		ndJacobian* m_outputForces;
		ndJacobian* m_internalForces;
		ndRightHandSide* m_rightHandSide;
		const ndLeftHandSide* m_leftHandSide;
	};

	class ndInitJacobianAccumulatePartialForces : public ndScene::ndBaseJob
	{
		public:
		virtual void Execute()
		{
			//D_TRACKTIME();
			ndWorld* const world = m_owner->GetWorld();
			const dInt32 threadIndex = GetThreadId();
			const dInt32 threadCount = m_owner->GetThreadCount();
			const dInt32 bodyCount = m_owner->GetActiveBodyArray().GetCount();
			ndJacobian* const internalForces = &world->m_internalForces[0];

			if (threadCount > 1)
			{
				const dInt32 step = bodyCount / threadCount;
				const dInt32 start = threadIndex * step;
				const dInt32 count = ((threadIndex + 1) < threadCount) ? step : bodyCount - start;
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
			else
			{
				memcpy(&internalForces[0], &internalForces[bodyCount], bodyCount * sizeof(ndJacobian));
			}
		}
	};

	ndScene* const scene = m_world->GetScene();
	const dInt32 passes = m_solverPasses;
	const dInt32 threadsCount = dMax(scene->GetThreadCount(), 1);

	dFloat32 m_accelNorm[D_MAX_THREADS_COUNT];
	dFloat32 accNorm = D_SOLVER_MAX_ERROR * dFloat32(2.0f);

	for (dInt32 i = 0; (i < passes) && (accNorm > D_SOLVER_MAX_ERROR); i++)
	{
		#ifdef D_PROFILE_JOINTS
		dUnsigned64 cpuClock = dGetCpuClock();
		#endif

		scene->SubmitJobs<ndCalculateJointsForce>(m_accelNorm);
		scene->SubmitJobs<ndInitJacobianAccumulatePartialForces>();

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

void ndDynamicsUpdate::UpdateIslandStateAvx2(const ndIsland& island)
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

void ndDynamicsUpdate::IntegrateBodiesAvx2()
{
	D_TRACKTIME();
	class ndIntegrateBodies : public ndScene::ndBaseJob
	{
		public:
		virtual void Execute()
		{
			//D_TRACKTIME();
			ndWorld* const world = m_owner->GetWorld();
			dArray<ndBodyKinematic*>& bodyArray = world->m_bodyIslandOrder;

			const dInt32 threadIndex = GetThreadId();
			const dInt32 threadCount = m_owner->GetThreadCount();
			const dInt32 bodyCount = bodyArray.GetCount();
			const dInt32 step = bodyCount / threadCount;
			const dInt32 start = threadIndex * step;
			const dInt32 count = ((threadIndex + 1) < threadCount) ? step : bodyCount - start;

			const dFloat32 timestep = m_timestep;
			const dFloat32 maxAccNorm2 = D_SOLVER_MAX_ERROR * D_SOLVER_MAX_ERROR;
			const dVector invTime(world->m_invTimestep);
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
	};

	ndScene* const scene = m_world->GetScene();
	scene->SubmitJobs<ndIntegrateBodies>();
}

void ndDynamicsUpdate::UpdateAvx2()
{
	m_world = (ndWorld*)this;
	m_timestep = m_world->GetScene()->GetTimestep();

	BuildIslandAvx2();
	if (m_islands.GetCount())
	{
		IntegrateUnconstrainedBodiesAvx2();

		InitWeightsAvx2();
		InitBodyArrayAvx2();
		InitJacobianMatrixAvx2();
		CalculateForcesAvx2();
		DetermineSleepStatesAvx2();
	}
}

