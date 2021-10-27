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

#ifndef __D_WORLD_DYNAMICS_UPDATE_H__
#define __D_WORLD_DYNAMICS_UPDATE_H__

#include "ndNewtonStdafx.h"

#define D_SMALL_ISLAND_COUNT		32
#define	D_FREEZZING_VELOCITY_DRAG	dFloat32 (0.9f)
#define	D_SOLVER_MAX_ERROR			(D_FREEZE_MAG * dFloat32 (0.5f))

#define D_DEFAULT_BUFFER_SIZE		1024
#define D_MAX_BODY_RADIX_BIT		9
#define D_MAX_BODY_RADIX_DIGIT		(1<<D_MAX_BODY_RADIX_BIT)
#define D_MAX_BODY_RADIX_MASK		(D_MAX_BODY_RADIX_DIGIT-1)

// the solver is a RK order 4, but instead of weighting the intermediate derivative by the usual 1/6, 1/3, 1/3, 1/6 coefficients
// I am using 1/4, 1/4, 1/4, 1/4.
// This is correct.The weighting coefficients of any RK method comes from an arbitrary criteria
// solving for a set of linear equation on the coefficients. 
// The standard coefficients just happen to lead to an accurate result because they are optimal 
// with respect to a second order solution.For differential equations of higher order, then is not 
// clear if the traditional weigh factors are any better than any other set of weighting factors.
// A different set of coefficient generates results that are not much different than the optimal set, 
// but it allows for simpler calculation of the intermediate derivatives and also for less intermediate memory.
// For more detail on the derivation of the Runge Kutta coefficients you can go to:  
// http://pathfinder.scar.utoronto.ca/~dyer/csca57/book_P/node51.html

class ndWorld;

D_MSV_NEWTON_ALIGN_32
class ndDynamicsUpdate: public dClassAlloc
{
	public:
	class ndJointBodyPairIndex
	{
		public:
		dInt32 m_body;
		dInt32 m_joint;
	};

	class ndJointBodySortKey
	{
		public:
		dInt32 m_lowCount;
		dInt32 m_highCount;
	};

	class ndPartialJointForceCounters
	{
		public:
		dInt32 m_scans[2][D_MAX_THREADS_COUNT][D_MAX_BODY_RADIX_DIGIT];
	};

	class ndSortKey
	{
		public:
		ndSortKey(dInt32 sleep, dInt32 rows)
			:m_value(0)
		{
			dAssert(rows > 0);
			m_upperBit = sleep;
			m_lowerBit = (1 << 6) - rows - 1;
		}

		union 
		{
			dInt32 m_value;
			struct
			{
				dUnsigned32 m_lowerBit : 6;
				dUnsigned32 m_upperBit : 1;
			};
		};
	};

	class ndBodyIndexPair
	{
		public:
		ndBodyKinematic* m_body;
		ndBodyKinematic* m_root;
	};

	class ndIsland
	{
		public:
		ndIsland(ndBodyKinematic* const root)
			:m_start(0)
			,m_count(0)
			,m_root(root)
		{
		}

		dInt32 m_start;
		dInt32 m_count;
		ndBodyKinematic* m_root;
	};

	public:
	ndDynamicsUpdate(ndWorld* const world);
	virtual ~ndDynamicsUpdate();

	void* GetTempBuffer() const;
	virtual const char* GetStringId() const;
	dInt32 GetUnconstrainedBodyCount() const;
	void ClearBuffer(void* const buffer, dInt32 sizeInByte) const;
	void ClearJacobianBuffer(dInt32 count, ndJacobian* const dst) const;

	dArray<ndJacobian>& GetInternalForces();
	dArray<ndLeftHandSide>& GetLeftHandSide();
	dArray<dInt32>& GetJointForceIndexBuffer();
	dArray<ndRightHandSide>& GetRightHandSide();
	dArray<ndJacobian>& GetTempInternalForces();
	dArray<ndBodyKinematic*>& GetBodyIslandOrder();
	dArray<ndJointBodyPairIndex>& GetJointBodyPairIndexBuffer();

	private:
	void RadixSort();
	void SortJoints();
	void SortIslands();
	void BuildIsland();
	void InitWeights();
	void InitBodyArray();
	void InitSkeletons();
	void CalculateForces();
	void IntegrateBodies();
	void UpdateSkeletons();
	void InitJacobianMatrix();
	void UpdateForceFeedback();
	void CalculateJointsForce();
	void IntegrateBodiesVelocity();
	void CalculateJointsAcceleration();
	void IntegrateUnconstrainedBodies();

	void DetermineSleepStates();
	void UpdateIslandState(const ndIsland& island);
	void GetJacobianDerivatives(ndConstraint* const joint);
	static dInt32 CompareIslands(const ndIsland* const  A, const ndIsland* const B, void* const);

	protected:
	void Clear();
	virtual void Update();
	void SortBodyJointScan();
	ndBodyKinematic* FindRootAndSplit(ndBodyKinematic* const body);

	dVector m_velocTol;
	dArray<ndIsland> m_islands;
	dArray<dInt32> m_jointForcesIndex;
	dArray<ndJacobian> m_internalForces;
	dArray<ndLeftHandSide> m_leftHandSide;
	dArray<ndRightHandSide> m_rightHandSide;
	dArray<ndJacobian> m_tempInternalForces;
	dArray<ndBodyKinematic*> m_bodyIslandOrder;
	dArray<ndJointBodyPairIndex> m_jointBodyPairIndexBuffer;

	ndWorld* m_world;
	dFloat32 m_timestep;
	dFloat32 m_invTimestep;
	dFloat32 m_firstPassCoef;
	dFloat32 m_invStepRK;
	dFloat32 m_timestepRK;
	dFloat32 m_invTimestepRK;
	dUnsigned32 m_solverPasses;
	dInt32 m_activeJointCount;
	dInt32 m_unConstrainedBodyCount;

	friend class ndWorld;
} D_GCC_NEWTON_ALIGN_32;

inline void* ndDynamicsUpdate::GetTempBuffer() const
{
	return (void*)&m_leftHandSide[0];
}

inline dArray<ndJacobian>& ndDynamicsUpdate::GetInternalForces()
{
	return m_internalForces;
}

inline dArray<ndJacobian>& ndDynamicsUpdate::GetTempInternalForces()
{
	return m_tempInternalForces;
}

inline  dArray<ndLeftHandSide>& ndDynamicsUpdate::GetLeftHandSide()
{ 
	return m_leftHandSide; 
}

inline  dArray<ndRightHandSide>& ndDynamicsUpdate::GetRightHandSide()
{ 
	return m_rightHandSide; 
}

inline  dArray<ndBodyKinematic*>& ndDynamicsUpdate::GetBodyIslandOrder()
{ 
	return m_bodyIslandOrder; 
}

inline dInt32 ndDynamicsUpdate::GetUnconstrainedBodyCount() const
{
	return m_unConstrainedBodyCount;
}

inline dArray<ndDynamicsUpdate::ndJointBodyPairIndex>& ndDynamicsUpdate::GetJointBodyPairIndexBuffer()
{
	return m_jointBodyPairIndexBuffer;
}

inline dArray<dInt32>& ndDynamicsUpdate::GetJointForceIndexBuffer()
{
	return m_jointForcesIndex;
}


inline void ndDynamicsUpdate::ClearJacobianBuffer(dInt32 count, ndJacobian* const buffer) const
{
	const dVector zero(dVector::m_zero);
	dVector* const dst = &buffer[0].m_linear;
	for (dInt32 i = 0; i < count; i++)
	{
		dst[i * 2 + 0] = zero;
		dst[i * 2 + 1] = zero;
	}
}

inline void ndDynamicsUpdate::ClearBuffer(void* const buffer, dInt32 sizeInByte) const
{
	dInt32 sizeInJacobian = sizeInByte / sizeof(ndJacobian);
	ClearJacobianBuffer(sizeInJacobian, (ndJacobian*)buffer);
	char* const ptr = (char*)buffer;
	for (dInt32 i = sizeInJacobian * sizeof(ndJacobian); i < sizeInByte; i++)
	{
		ptr[i] = 0;
	}
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

inline void ndDynamicsUpdate::SortBodyJointScan()
{
	class ndCountJointBodyPairs : public ndScene::ndBaseJob
	{
		public:
		virtual void Execute()
		{
			D_TRACKTIME();
			ndWorld* const world = m_owner->GetWorld();
			ndScene* const scene = world->GetScene();
			ndDynamicsUpdate* const me = world->m_solver;
			const ndConstraintArray& jointArray = scene->GetActiveContactArray();
			ndJointBodyPairIndex* const jointBodyBuffer = &me->GetJointBodyPairIndexBuffer()[0];

			const dInt32 count = jointArray.GetCount();
			const dInt32 threadIndex = GetThreadId();
			const dInt32 threadCount = m_owner->GetThreadCount();
			const dInt32 stride = count / threadCount;
			const dInt32 start = threadIndex * stride;
			const dInt32 blockSize = (threadIndex != (threadCount - 1)) ? stride : count - start;

			for (dInt32 i = 0; i < blockSize; i++)
			{
				const dInt32 index = i + start;
				const ndConstraint* const joint = jointArray[index];
				const ndBodyKinematic* const body0 = joint->GetBody0();
				const ndBodyKinematic* const body1 = joint->GetBody1();

				const dInt32 m0 = body0->m_index;
				const dInt32 m1 = body1->m_index;
				jointBodyBuffer[index * 2 + 0].m_body = m0;
				jointBodyBuffer[index * 2 + 0].m_joint = index * 2 + 0;
				jointBodyBuffer[index * 2 + 1].m_body = m1;
				jointBodyBuffer[index * 2 + 1].m_joint = index * 2 + 1;
			}
		}
	};

	class ndBodyJointForceScan : public ndScene::ndBaseJob
	{
		public:
		ndBodyJointForceScan()
		{
		}

		virtual void Execute()
		{
			D_TRACKTIME();
			ndWorld* const world = m_owner->GetWorld();
			ndDynamicsUpdate* const me = (ndDynamicsUpdate*)world->m_solver;
			const dArray<ndJointBodyPairIndex>& bodyJointPairs = me->GetJointBodyPairIndexBuffer();
			const dInt32 count = bodyJointPairs.GetCount();
			const dInt32 threadIndex = GetThreadId();
			const dInt32 threadCount = m_owner->GetThreadCount();
			const dInt32 stride = count / threadCount;
			const dInt32 start = threadIndex * stride;
			const dInt32 blockSize = (threadIndex != (threadCount - 1)) ? stride : count - start;

			ndPartialJointForceCounters& counters = *((ndPartialJointForceCounters*)m_context);
			dInt32* const lowDigit = &counters.m_scans[0][threadIndex][0];
			dInt32* const highDigit = &counters.m_scans[1][threadIndex][0];
			memset(lowDigit, 0, D_MAX_BODY_RADIX_DIGIT * sizeof(dInt32));
			memset(highDigit, 0, D_MAX_BODY_RADIX_DIGIT * sizeof(dInt32));

			for (dInt32 i = 0; i < blockSize; i++)
			{
				const dUnsigned32 key = bodyJointPairs[i + start].m_body;
				const dInt32 lowEntry = key & D_MAX_BODY_RADIX_MASK;
				const dInt32 highEntry = key >> D_MAX_BODY_RADIX_BIT;
				lowDigit[lowEntry] ++;
				highDigit[highEntry] ++;
			}
		}
	};

	class ndBodyJointForceSortLowDigit : public ndScene::ndBaseJob
	{
		public:
		ndBodyJointForceSortLowDigit()
		{
		}

		virtual void Execute()
		{
			D_TRACKTIME();
			ndWorld* const world = m_owner->GetWorld();
			ndDynamicsUpdate* const me = (ndDynamicsUpdate*)world->m_solver;
			const dArray<ndJointBodyPairIndex>& bodyJointPairs = me->GetJointBodyPairIndexBuffer();
			const dInt32 count = bodyJointPairs.GetCount();
			const dInt32 threadIndex = GetThreadId();
			const dInt32 threadCount = m_owner->GetThreadCount();
			const dInt32 stride = count / threadCount;
			const dInt32 start = threadIndex * stride;
			const dInt32 blockSize = (threadIndex != (threadCount - 1)) ? stride : count - start;

			ndJointBodyPairIndex* const sortBuffer = (ndJointBodyPairIndex*)me->GetTempBuffer();
			ndPartialJointForceCounters& counters = *((ndPartialJointForceCounters*)m_context);
			dInt32* const scan = &counters.m_scans[0][threadIndex][0];
			for (dInt32 i = 0; i < blockSize; i++)
			{
				const ndJointBodyPairIndex pair(bodyJointPairs[i + start]);
				if (pair.m_body >= 512)
					i *= 1;
				const dUnsigned32 key = pair.m_body & D_MAX_BODY_RADIX_MASK;
				const dInt32 index = scan[key];
				sortBuffer[index] = pair;
				scan[key]++;
			}
		}
	};

	class ndBodyJointForceSortHighDigit : public ndScene::ndBaseJob
	{
		public:
		ndBodyJointForceSortHighDigit()
		{
		}

		virtual void Execute()
		{
			D_TRACKTIME();
			ndWorld* const world = m_owner->GetWorld();
			ndDynamicsUpdate* const me = (ndDynamicsUpdate*)world->m_solver;
			dArray<ndJointBodyPairIndex>& bodyJointPairs = me->GetJointBodyPairIndexBuffer();
			const dInt32 count = bodyJointPairs.GetCount();
			const dInt32 threadIndex = GetThreadId();
			const dInt32 threadCount = m_owner->GetThreadCount();
			const dInt32 stride = count / threadCount;
			const dInt32 start = threadIndex * stride;
			const dInt32 blockSize = (threadIndex != (threadCount - 1)) ? stride : count - start;

			const ndJointBodyPairIndex* const sortBuffer = (ndJointBodyPairIndex*)me->GetTempBuffer();
			ndPartialJointForceCounters& counters = *((ndPartialJointForceCounters*)m_context);
			dInt32* const scan = &counters.m_scans[1][threadIndex][0];

			for (dInt32 i = 0; i < blockSize; i++)
			{
				const ndJointBodyPairIndex pair(sortBuffer[i + start]);

				if (pair.m_body >= 512)
					i *= 1;

				const dUnsigned32 key = pair.m_body >> D_MAX_BODY_RADIX_BIT;
				const dInt32 index = scan[key];
				bodyJointPairs[index] = pair;
				scan[key] ++;
			}
		}
	};

	
	ndPartialJointForceCounters counters;

	ndScene* const scene = m_world->GetScene();
	ndConstraintArray& jointArray = scene->GetActiveContactArray();
	GetTempInternalForces().SetCount(jointArray.GetCount() * 2);
	GetJointBodyPairIndexBuffer().SetCount(jointArray.GetCount() * 2);

	scene->SubmitJobs<ndCountJointBodyPairs>();
	scene->SubmitJobs<ndBodyJointForceScan>(&counters);

	dInt32 lowDigitSum = 0;
	dInt32 highDigitSum = 0;

	const dInt32 threadCount = scene->GetThreadCount();
	for (dInt32 i = 0; i < D_MAX_BODY_RADIX_DIGIT; i++)
	{
		for (dInt32 j = 0; j < threadCount; j++)
		{
			const dInt32 lowDigit = counters.m_scans[0][j][i];
			const dInt32 highDigit = counters.m_scans[1][j][i];
			counters.m_scans[0][j][i] = lowDigitSum;
			counters.m_scans[1][j][i] = highDigitSum;
			lowDigitSum += lowDigit;
			highDigitSum += highDigit;
		}
	}

	scene->SubmitJobs<ndBodyJointForceSortLowDigit>(&counters);
	scene->SubmitJobs<ndBodyJointForceSortHighDigit>(&counters);

	dArray<dInt32>& bodyJointIndex = GetJointForceIndexBuffer();
	const dInt32 bodyJointIndexCount = scene->GetActiveBodyArray().GetCount() + 1;
	bodyJointIndex.SetCount(bodyJointIndexCount);
	ClearBuffer(&bodyJointIndex[0], bodyJointIndexCount * sizeof(dInt32));

	for (dInt32 i = 0; i < jointArray.GetCount(); i++)
	{
		ndConstraint* const joint = jointArray[i];
		const ndBodyKinematic* const body0 = joint->GetBody0();
		const ndBodyKinematic* const body1 = joint->GetBody1();
		dInt32 m0 = body0->m_index;
		dInt32 m1 = body1->m_index;
		bodyJointIndex[m0] ++;
		bodyJointIndex[m1] ++;
	}

	dInt32 bodyJointIndexAcc = 0;
	for (dInt32 i = 0; i < bodyJointIndexCount; i++)
	{
		dInt32 count = bodyJointIndex[i];
		bodyJointIndex[i] = bodyJointIndexAcc;
		bodyJointIndexAcc += count;
	}

	#ifdef _DEBUG
		const dArray<ndJointBodyPairIndex>& jointBodyPairIndexBuffer = GetJointBodyPairIndexBuffer();
		for (dInt32 i = 0; i < scene->GetActiveBodyArray().GetCount(); i++)
		{
			dInt32 startIndex = bodyJointIndex[i];
			dInt32 count = bodyJointIndex[i + 1] - startIndex;
			for (dInt32 j = 0; j < count; j++)
			{
				dInt32 bodyIndex = jointBodyPairIndexBuffer[startIndex + j].m_body;
				dAssert(bodyIndex == i);
			}
		}
	#endif
}

#endif

