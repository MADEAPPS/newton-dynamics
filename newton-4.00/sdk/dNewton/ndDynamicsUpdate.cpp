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
#include "ndBodyDynamic.h"
#include "ndSkeletonList.h"
#include "ndDynamicsUpdate.h"
#include "ndJointBilateralConstraint.h"

#define D_MAX_BODY_RADIX_BIT		9
#define D_DEFAULT_BUFFER_SIZE		1024

ndDynamicsUpdate::ndDynamicsUpdate(ndWorld* const world)
	:m_velocTol(ndFloat32(1.0e-8f))
	,m_islands(D_DEFAULT_BUFFER_SIZE)
	,m_jointForcesIndex(D_DEFAULT_BUFFER_SIZE)
	,m_internalForces(D_DEFAULT_BUFFER_SIZE)
	,m_leftHandSide(D_DEFAULT_BUFFER_SIZE * 4)
	,m_rightHandSide(D_DEFAULT_BUFFER_SIZE)
	,m_tempInternalForces(D_DEFAULT_BUFFER_SIZE)
	,m_bodyIslandOrder(D_DEFAULT_BUFFER_SIZE)
	,m_jointBodyPairIndexBuffer(D_DEFAULT_BUFFER_SIZE)
	,m_world(world)
	,m_timestep(ndFloat32(0.0f))
	,m_invTimestep(ndFloat32(0.0f))
	,m_firstPassCoef(ndFloat32(0.0f))
	,m_invStepRK(ndFloat32(0.0f))
	,m_timestepRK(ndFloat32(0.0f))
	,m_invTimestepRK(ndFloat32(0.0f))
	,m_solverPasses(0)
	,m_activeJointCount(0)
	,m_unConstrainedBodyCount(0)
{
}

ndDynamicsUpdate::~ndDynamicsUpdate()
{
	Clear();
}

const char* ndDynamicsUpdate::GetStringId() const
{
	return "default";
}

void ndDynamicsUpdate::Clear()
{
	m_islands.Resize(D_DEFAULT_BUFFER_SIZE);
	m_rightHandSide.Resize(D_DEFAULT_BUFFER_SIZE);
	m_internalForces.Resize(D_DEFAULT_BUFFER_SIZE);
	m_bodyIslandOrder.Resize(D_DEFAULT_BUFFER_SIZE);
	m_leftHandSide.Resize(D_DEFAULT_BUFFER_SIZE * 4);
	m_tempInternalForces.Resize(D_DEFAULT_BUFFER_SIZE);
	m_jointForcesIndex.Resize(D_DEFAULT_BUFFER_SIZE);
	m_jointBodyPairIndexBuffer.Resize(D_DEFAULT_BUFFER_SIZE);
}

void ndDynamicsUpdate::SortBodyJointScan()
{
	D_TRACKTIME();
	class ndEvaluateKey
	{
		public:
		ndEvaluateKey(void* const context)
		{
			ndInt32 digit = *((ndInt32*)context);
			m_shit = digit * D_MAX_BODY_RADIX_BIT;
		}

		ndUnsigned32 GetKey(const ndDynamicsUpdate::ndJointBodyPairIndex& entry) const
		{
			ndUnsigned32 key = entry.m_body >> m_shit;
			return key & ((1<< D_MAX_BODY_RADIX_BIT)-1);
		}

		ndUnsigned32 m_shit;
	};

	class ndCountJointBodyPairs : public ndThreadPoolJob_old
	{
		public:
		virtual void Execute()
		{
			D_TRACKTIME();
			ndScene* const scene = (ndScene*)GetThreadPool();
			ndWorld* const world = scene->GetWorld();
			ndDynamicsUpdate* const me = world->m_solver;
			const ndArray<ndConstraint*>& jointArray = scene->GetActiveContactArray();
			ndJointBodyPairIndex* const jointBodyBuffer = &me->GetJointBodyPairIndexBuffer()[0];

			const ndStartEnd startEnd(jointArray.GetCount(), GetThreadId(), GetThreadCount());
			for (ndInt32 i = startEnd.m_start; i < startEnd.m_end; ++i)
			{
				const ndInt32 index = i;
				const ndConstraint* const joint = jointArray[index];
				const ndBodyKinematic* const body0 = joint->GetBody0();
				const ndBodyKinematic* const body1 = joint->GetBody1();

				const ndInt32 m0 = body0->m_index;
				const ndInt32 m1 = body1->m_index;
				jointBodyBuffer[index * 2 + 0].m_body = m0;
				jointBodyBuffer[index * 2 + 0].m_joint = index * 2 + 0;
				jointBodyBuffer[index * 2 + 1].m_body = m1;
				jointBodyBuffer[index * 2 + 1].m_joint = index * 2 + 1;
			}
		}
	};

	ndScene* const scene = m_world->GetScene();
	ndArray<ndConstraint*>& jointArray = scene->GetActiveContactArray();
	ndArray<ndJointBodyPairIndex>& bodyJointPairs = GetJointBodyPairIndexBuffer();

	bodyJointPairs.SetCount(jointArray.GetCount() * 2);
	GetTempInternalForces().SetCount(jointArray.GetCount() * 2);

	scene->SubmitJobs<ndCountJointBodyPairs>();

	ndJointBodyPairIndex* const tempBuffer = (ndJointBodyPairIndex*)GetTempBuffer();

	ndInt32 digit = 0;
	ndCountingSort<ndJointBodyPairIndex, ndEvaluateKey, D_MAX_BODY_RADIX_BIT>(*scene, &bodyJointPairs[0], tempBuffer, bodyJointPairs.GetCount(), &digit);

	digit = 1;
	ndCountingSort<ndJointBodyPairIndex, ndEvaluateKey, D_MAX_BODY_RADIX_BIT>(*scene, tempBuffer, &bodyJointPairs[0], bodyJointPairs.GetCount(), &digit);

	bodyJointPairs.SetCount(bodyJointPairs.GetCount() + 1);
	bodyJointPairs[bodyJointPairs.GetCount() - 1] = bodyJointPairs[bodyJointPairs.GetCount() - 2];

	ndArray<ndInt32>& bodyJointIndex = GetJointForceIndexBuffer();
	const ndInt32 bodyJointIndexCount = scene->GetActiveBodyArray().GetCount() + 1;
	bodyJointIndex.SetCount(bodyJointIndexCount);
	ClearBuffer(&bodyJointIndex[0], bodyJointIndexCount * sizeof(ndInt32));

	for (ndInt32 i = 0; i < jointArray.GetCount(); ++i)
	{
		const ndConstraint* const joint = jointArray[i];
		const ndBodyKinematic* const body0 = joint->GetBody0();
		const ndBodyKinematic* const body1 = joint->GetBody1();
		const ndInt32 m0 = body0->m_index;
		const ndInt32 m1 = body1->m_index;
		bodyJointIndex[m0] ++;
		bodyJointIndex[m1] ++;
	}

	ndInt32 bodyJointIndexAcc = 0;
	for (ndInt32 i = 0; i < bodyJointIndexCount; ++i)
	{
		ndInt32 count = bodyJointIndex[i];
		bodyJointIndex[i] = bodyJointIndexAcc;
		bodyJointIndexAcc += count;
	}

#ifdef _DEBUG
	const ndArray<ndJointBodyPairIndex>& jointBodyPairIndexBuffer = GetJointBodyPairIndexBuffer();
	for (ndInt32 i = 0; i < scene->GetActiveBodyArray().GetCount(); ++i)
	{
		ndInt32 startIndex = bodyJointIndex[i];
		ndInt32 count = bodyJointIndex[i + 1] - startIndex;
		for (ndInt32 j = 0; j < count; ++j)
		{
			ndInt32 bodyIndex = jointBodyPairIndexBuffer[startIndex + j].m_body;
			dAssert(bodyIndex == i);
		}
	}
#endif
}

void ndDynamicsUpdate::BuildDisjointSets()
{
	D_TRACKTIME();

	// to be parallelized. but not an eassy task
	ndScene* const scene = m_world->GetScene();

	ndArray<ndConstraint*>& jointArray = scene->GetActiveContactArray();
	ndConstraint** const tempJointBuffer = (ndConstraint**)GetTempBuffer();
	//dAssert(m_activeJointCount <= jointArray.GetCount());
	//jointArray.SetCount(m_activeJointCount);
	
	for (ndInt32 i = jointArray.GetCount() - 1; i >= 0; i--)
	{
		ndConstraint* const joint = tempJointBuffer[i];
		ndBodyKinematic* const body0 = joint->GetBody0();
		ndBodyKinematic* const body1 = joint->GetBody1();
		if (!body1->m_isStatic)
		{
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

			const ndInt32 sleep = body0->m_islandSleep & body1->m_islandSleep;
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

void ndDynamicsUpdate::SortJointsScan()
{
	D_TRACKTIME();
	class ndEvaluateCountRows
	{
		public:
		class ndSortKey
		{
			public:
			ndSortKey(ndInt32 sleep, ndInt32 rows)
				:m_value(0)
			{
				dAssert(rows > 0);
				m_upperBit = sleep;
				m_lowerBit = (1 << 6) - rows - 1;
			}

			union
			{
				ndInt32 m_value;
				struct
				{
					ndUnsigned32 m_lowerBit : 6;
					ndUnsigned32 m_upperBit : 1;
				};
			};
		};

		ndEvaluateCountRows(void* const) {}
		ndUnsigned32 GetKey(const ndConstraint* const joint) const
		{
			const ndSortKey key(joint->m_resting, joint->m_rowCount);
			return key.m_value;
		}
	};

	ndScene* const scene = m_world->GetScene();

	for (ndSkeletonList::ndNode* node = m_world->GetSkeletonList().GetFirst(); node; node = node->GetNext())
	{
		ndSkeletonContainer* const skeleton = &node->GetInfo();
		skeleton->CheckSleepState();
	}

	const ndJointList& jointList = m_world->GetJointList();
	ndArray<ndConstraint*>& jointArray = scene->GetActiveContactArray();

	ndInt32 jointCount = jointArray.GetCount();
	jointArray.SetCount(jointCount + jointList.GetCount());

	for (ndJointList::ndNode* node = jointList.GetFirst(); node; node = node->GetNext())
	{
		ndJointBilateralConstraint* const joint = node->GetInfo();
		if (joint->IsActive())
		{
			jointArray[jointCount] = joint;
			jointCount++;
		}
	}
	jointArray.SetCount(jointCount);

	m_leftHandSide.SetCount(jointArray.GetCount() + 32);

	ndInt32 histogram[D_MAX_THREADS_COUNT][2];
	ndInt32 movingJoints[D_MAX_THREADS_COUNT];
	const ndInt32 threadCount = scene->GetThreadCount();
	
	auto MarkFence0 = ndMakeObject::ndFunction([this, &jointArray](ndInt32 threadIndex, ndInt32 threadCount)
	{
		D_TRACKTIME();
		const ndStartEnd startEnd(jointArray.GetCount(), threadIndex, threadCount);
		for (ndInt32 i = startEnd.m_start; i < startEnd.m_end; ++i)
		{
			ndConstraint* const joint = jointArray[i];
			ndBodyKinematic* const body0 = joint->GetBody0();
			ndBodyKinematic* const body1 = joint->GetBody1();
			const ndInt32 rows = joint->GetRowsCount();
			joint->m_rowCount = rows;

			const ndInt32 equilibrium = body0->m_equilibrium & body1->m_equilibrium;
			if (!equilibrium)
			{
				body0->m_isJointFence0 = 0;
				body1->m_isJointFence0 = body1->m_isStatic;
				dAssert((body1->m_invMass.m_w == ndFloat32(0.0f)) == body1->m_isStatic);
			}

			body0->m_bodyIsConstrained = 1;
			body0->m_equilibrium0 = body0->m_equilibrium0 & equilibrium;
			if (!body1->m_isStatic)
			{
				body1->m_bodyIsConstrained = 1;
				body1->m_equilibrium0 = body1->m_equilibrium0 & equilibrium;
			}
		}
	});

	auto MarkFence1 = ndMakeObject::ndFunction([this, &jointArray, &movingJoints](ndInt32 threadIndex, ndInt32 threadCount)
	{
		D_TRACKTIME();
		ndInt32 activeJointCount = 0;
		const ndStartEnd startEnd(jointArray.GetCount(), threadIndex, threadCount);
		for (ndInt32 i = startEnd.m_start; i < startEnd.m_end; ++i)
		{
			ndConstraint* const joint = jointArray[i];
			ndBodyKinematic* const body0 = joint->GetBody0();
			ndBodyKinematic* const body1 = joint->GetBody1();

			const ndInt8 resting = body0->m_equilibrium0 & body1->m_equilibrium0;
			activeJointCount += (1 - resting);
			joint->m_resting = resting;

			const ndInt32 solverSleep0 = body0->m_isJointFence0 & body1->m_isJointFence0;
			if (!solverSleep0)
			{
				body0->m_isJointFence1 = 0;
				body1->m_isJointFence1 = body1->m_isStatic;
				dAssert((body1->m_invMass.m_w == ndFloat32(0.0f)) == body1->m_isStatic);
			}
		}
		movingJoints[threadIndex] = activeJointCount;
	});

	auto Scan0 = ndMakeObject::ndFunction([this, &jointArray, &histogram](ndInt32 threadIndex, ndInt32 threadCount)
	{
		D_TRACKTIME();
		ndInt32* const hist = &histogram[threadIndex][0];
		ndConstraint** const dstBuffer = (ndConstraint**)GetTempBuffer();

		hist[0] = 0;
		hist[1] = 0;
		const ndStartEnd startEnd(jointArray.GetCount(), threadIndex, threadCount);
		for (ndInt32 i = startEnd.m_start; i < startEnd.m_end; ++i)
		{
			ndConstraint* const joint = jointArray[i];
			ndBodyKinematic* const body0 = joint->GetBody0();
			ndBodyKinematic* const body1 = joint->GetBody1();
			const ndInt32 key = body0->m_isJointFence1 & body1->m_isJointFence1;
			const ndInt32 entry = hist[key];
			dstBuffer[entry] = joint;
			hist[key] = entry + 1;
		}
	});

	auto Sort0 = ndMakeObject::ndFunction([this, &jointArray, &histogram](ndInt32 threadIndex, ndInt32 threadCount)
	{
		D_TRACKTIME();
		ndInt32* const hist = &histogram[threadIndex][0];
		ndConstraint** const dstBuffer = (ndConstraint**)GetTempBuffer();

		const ndStartEnd startEnd(jointArray.GetCount(), threadIndex, threadCount);
		for (ndInt32 i = startEnd.m_start; i < startEnd.m_end; ++i)
		{
			ndConstraint* const joint = jointArray[i];
			ndBodyKinematic* const body0 = joint->GetBody0();
			ndBodyKinematic* const body1 = joint->GetBody1();
			const ndInt32 key = body0->m_isJointFence1 & body1->m_isJointFence1;
			const ndInt32 entry = hist[key];
			dstBuffer[entry] = joint;
			hist[key] = entry + 1;
		}
	});

	scene->Execute(MarkFence0);
	scene->Execute(MarkFence1);
	scene->Execute(Scan0);

	ndInt32 scan[2];
	scan[0] = 0;
	scan[1] = 0;
	ndInt32 movingJointCount = 0;
	for (ndInt32 i = 0; i < threadCount; ++i)
	{
		scan[0] += histogram[i][0];
		scan[1] += histogram[i][1];
		movingJointCount += movingJoints[i];
	}

	m_activeJointCount = scan[0];
	dAssert(m_activeJointCount <= jointArray.GetCount());
	if (!m_activeJointCount)
	{
		jointArray.SetCount(0);
		return;
	}

	ndInt32 sum = 0;
	for (ndInt32 i = 0; i < 2; ++i)
	{
		for (ndInt32 j = 0; j < threadCount; ++j)
		{
			ndInt32 partialSum = histogram[j][i];
			histogram[j][i] = sum;
			sum += partialSum;
		}
	}

	scene->Execute(Sort0);
	ndConstraint** const tempJointBuffer = (ndConstraint**)GetTempBuffer();

#ifdef _DEBUG
	for (ndInt32 i = 0; i < (jointArray.GetCount() - 1); ++i)
	{
		const ndConstraint* const joint0 = tempJointBuffer[i];
		const ndConstraint* const joint1 = tempJointBuffer[i + 1];
		const ndInt32 key0 = (joint0->GetBody0()->m_isJointFence1 & joint0->GetBody1()->m_isJointFence1) ? 1 : 0;
		const ndInt32 key1 = (joint1->GetBody0()->m_isJointFence1 & joint1->GetBody1()->m_isJointFence1) ? 1 : 0;
		dAssert(key0 <= key1);
	}
#endif

	dAssert(m_activeJointCount <= jointArray.GetCount());
	jointArray.SetCount(m_activeJointCount);
	BuildDisjointSets();

	m_activeJointCount = movingJointCount;
	GetTempInternalForces().SetCount(jointArray.GetCount() * 2);
	GetJointBodyPairIndexBuffer().SetCount(jointArray.GetCount() * 2);
	ndCountingSort<ndConstraint*, ndEvaluateCountRows, 7>(*scene, tempJointBuffer, &jointArray[0], jointArray.GetCount());
}

void ndDynamicsUpdate::SortJoints()
{
	D_TRACKTIME();

	SortJointsScan();

	if (!m_activeJointCount)
	{
		return;
	}

	ndScene* const scene = m_world->GetScene();
	ndArray<ndConstraint*>& jointArray = scene->GetActiveContactArray();

	ndInt32 rowCount = 1;
	for (ndInt32 i = 0; i < jointArray.GetCount(); ++i)
	{
		ndConstraint* const joint = jointArray[i];
		joint->m_rowStart = rowCount;
		rowCount += joint->m_rowCount;
	}

	m_leftHandSide.SetCount(rowCount);
	m_rightHandSide.SetCount(rowCount);

#ifdef _DEBUG
	dAssert(m_activeJointCount <= jointArray.GetCount());
	for (ndInt32 i = 0; i < jointArray.GetCount(); ++i)
	{
		ndConstraint* const joint = jointArray[i];
		dAssert(joint->m_rowStart < m_leftHandSide.GetCount());
		dAssert((joint->m_rowStart + joint->m_rowCount) <= rowCount);
	}

	for (ndInt32 i = 1; i < m_activeJointCount; ++i)
	{
		ndConstraint* const joint0 = jointArray[i - 1];
		ndConstraint* const joint1 = jointArray[i - 0];
		dAssert(!joint0->m_resting);
		dAssert(!joint1->m_resting);
		dAssert(joint0->m_rowCount >= joint1->m_rowCount);
		dAssert(!(joint0->GetBody0()->m_equilibrium0 & joint0->GetBody1()->m_equilibrium0));
		dAssert(!(joint1->GetBody0()->m_equilibrium0 & joint1->GetBody1()->m_equilibrium0));
	}

	for (ndInt32 i = m_activeJointCount + 1; i < jointArray.GetCount(); ++i)
	{
		ndConstraint* const joint0 = jointArray[i - 1];
		ndConstraint* const joint1 = jointArray[i - 0];
		dAssert(joint0->m_resting);
		dAssert(joint1->m_resting);
		dAssert(joint0->m_rowCount >= joint1->m_rowCount);
		dAssert(joint0->GetBody0()->m_equilibrium0 & joint0->GetBody1()->m_equilibrium0);
		dAssert(joint1->GetBody0()->m_equilibrium0 & joint1->GetBody1()->m_equilibrium0);
	}
#endif

	SortBodyJointScan();
}

void ndDynamicsUpdate::SortIslands()
{
	D_TRACKTIME();
	class ndIslandKey
	{
		public:
		ndIslandKey(void* const) {}
		ndUnsigned32 GetKey(const ndBodyIndexPair& pair) const
		{
			const ndBodyKinematic* const body = pair.m_root;
			const ndInt32 key = 1 - body->m_bodyIsConstrained;
			return key;
		}
	};

	class ndEvaluateKey
	{
		public:
		ndEvaluateKey(void* const context)
		{
			ndInt32 digitLocation = *((ndInt32*)context);
			m_shift = digitLocation * D_MAX_BODY_RADIX_BIT;
		}

		ndUnsigned32 GetKey(const ndIsland& island) const
		{
			ndUnsigned32 key = island.m_count * 2 + island.m_root->m_bodyIsConstrained;
			const ndUnsigned32 maxVal = 1 << (D_MAX_BODY_RADIX_BIT * 2);
			dAssert(key < maxVal);
			const ndUnsigned32 lowKey = (maxVal - key) >> m_shift;
			return lowKey & ((1<<D_MAX_BODY_RADIX_BIT) - 1);
		}
		ndInt32 m_shift;
	};

	ndScene* const scene = m_world->GetScene();
	const ndArray<ndBodyKinematic*>& bodyArray = scene->GetActiveBodyArray();
	GetInternalForces().SetCount(bodyArray.GetCount());

	ndInt32 bodyCount = 0;
	ndBodyIndexPair* const buffer0 = (ndBodyIndexPair*)&GetInternalForces()[0];
	for (ndInt32 i = 0; i < bodyArray.GetCount(); ++i)
	{
		ndBodyKinematic* const body = bodyArray[i];
		if (!(body->m_equilibrium0 & body->m_islandSleep))
		{
			buffer0[bodyCount].m_body = body;
			if (!body->m_isStatic)
			{
				ndBodyKinematic* root = body->m_islandParent;
				while (root != root->m_islandParent)
				{
					root = root->m_islandParent;
				}

				buffer0[bodyCount].m_root = root;
				if (root->m_rank != -1)
				{
					root->m_rank = -1;
				}
			}
			else
			{
				buffer0[bodyCount].m_root = body;
				body->m_rank = -1;
			}
			bodyCount++;
		}
	}

	ndArray<ndIsland>& islands = GetIslands();
	ndArray<ndBodyKinematic*>& activeBodyArray = GetBodyIslandOrder();

	islands.SetCount(0);
	activeBodyArray.SetCount(bodyCount);

	ndInt32 unConstrainedCount = 0;
	if (bodyCount)
	{
		ndBodyIndexPair* const buffer1 = buffer0 + bodyCount;
		ndCountingSort<ndBodyIndexPair, ndIslandKey, 1>(*scene, buffer0, buffer1, bodyCount);
		for (ndInt32 i = 0; i < bodyCount; ++i)
		{
			dAssert((i == bodyCount - 1) || (buffer1[i].m_root->m_bodyIsConstrained >= buffer1[i + 1].m_root->m_bodyIsConstrained));

			activeBodyArray[i] = buffer1[i].m_body;
			if (buffer1[i].m_root->m_rank == -1)
			{
				buffer1[i].m_root->m_rank = 0;
				ndIsland island(buffer1[i].m_root);
				islands.PushBack(island);
			}
			buffer1[i].m_root->m_rank += 1;
		}

		ndInt32 start = 0;
		ndInt32 islandMaxKeySize = 0;
		for (ndInt32 i = 0; i < islands.GetCount(); ++i)
		{
			ndIsland& island = islands[i];
			island.m_start = start;
			island.m_count = island.m_root->m_rank;
			islandMaxKeySize = dMax(islandMaxKeySize, island.m_count);
			start += island.m_count;
			unConstrainedCount -= island.m_root->m_bodyIsConstrained;
		}
		unConstrainedCount += islands.GetCount();

		ndInt32 context = 0;
		ndIsland* const islandTempBuffer = (ndIsland*)GetTempBuffer();
		ndCountingSort<ndIsland, ndEvaluateKey, D_MAX_BODY_RADIX_BIT>(*scene, &islands[0], islandTempBuffer, islands.GetCount(), &context);
		if (islandMaxKeySize >= (1 << (D_MAX_BODY_RADIX_BIT - 1)))
		{
			context = 1;
			ndCountingSort<ndIsland, ndEvaluateKey, D_MAX_BODY_RADIX_BIT>(*scene, islandTempBuffer, &islands[0], islands.GetCount(), &context);
		}
		else
		{
			for (ndInt32 i = 0; i < islands.GetCount(); ++i)
			{
				islands[i] = islandTempBuffer[i];
			}
		}
	}
	m_unConstrainedBodyCount = unConstrainedCount;
}

void ndDynamicsUpdate::BuildIsland()
{
	ndScene* const scene = m_world->GetScene();
	const ndArray<ndBodyKinematic*>& bodyArray = scene->GetActiveBodyArray();
	dAssert(bodyArray.GetCount() >= 1);
	if (bodyArray.GetCount() - 1)
	{
		D_TRACKTIME();
		SortJoints();
		SortIslands();
	}
}

void ndDynamicsUpdate::IntegrateUnconstrainedBodies()
{
	ndScene* const scene = m_world->GetScene();
	auto IntegrateUnconstrainedBodies = ndMakeObject::ndFunction([this, &scene](ndInt32 threadIndex, ndInt32 threadCount)
	{
		D_TRACKTIME();
		ndArray<ndBodyKinematic*>& bodyArray = GetBodyIslandOrder();

		const ndFloat32 timestep = scene->GetTimestep();
		const ndInt32 base = bodyArray.GetCount() - GetUnconstrainedBodyCount();

		const ndStartEnd startEnd(GetUnconstrainedBodyCount(), threadIndex, threadCount);
		for (ndInt32 i = startEnd.m_start; i < startEnd.m_end; ++i)
		{
			ndBodyKinematic* const body = bodyArray[base + i];
			dAssert(body);
			body->UpdateInvInertiaMatrix();
			body->AddDampingAcceleration(timestep);
			body->IntegrateExternalForce(timestep);
		}
	});

	if (GetUnconstrainedBodyCount())
	{
		D_TRACKTIME();
		scene->Execute(IntegrateUnconstrainedBodies);
	}
}

void ndDynamicsUpdate::InitWeights()
{
	D_TRACKTIME();
	class ndInitWeights : public ndThreadPoolJob_old
	{
		public:
		virtual void Execute()
		{
			D_TRACKTIME();
			ndScene* const scene = (ndScene*)GetThreadPool();
			ndWorld* const world = scene->GetWorld();
			ndDynamicsUpdate* const me = (ndDynamicsUpdate*)world->m_solver;

			ndBodyKinematic** const bodyArray = &scene->GetActiveBodyArray()[0];
			const ndArray<ndInt32>& jointForceIndexBuffer = me->GetJointForceIndexBuffer();
			const ndArray<ndJointBodyPairIndex>& jointBodyPairIndex = me->GetJointBodyPairIndexBuffer();

			ndInt32 maxExtraPasses = 1;

			const ndStartEnd startEnd(jointForceIndexBuffer.GetCount() - 1, GetThreadId(), GetThreadCount());
			for (ndInt32 i = startEnd.m_start; i < startEnd.m_end; ++i)
			{
				const ndInt32 index = jointForceIndexBuffer[i];
				const ndJointBodyPairIndex& scan = jointBodyPairIndex[index];
				ndBodyKinematic* const body = bodyArray[scan.m_body];
				dAssert(body->m_index == scan.m_body);
				dAssert(body->m_bodyIsConstrained <= 1);
				const ndInt32 count = jointForceIndexBuffer[i + 1] - index - 1;
				const ndInt32 mask = -ndInt32(body->m_bodyIsConstrained & ~body->m_isStatic);
				const ndInt32 weigh = 1 + (mask & count);
				dAssert(weigh >= 0);
				if (weigh)
				{
					body->m_weigh = ndFloat32(weigh);
				}
				maxExtraPasses = dMax(weigh, maxExtraPasses);
			}
			ndInt32* const extraPasses = (ndInt32*)GetContext();
			extraPasses[GetThreadId()] = maxExtraPasses;
		}
	};

	ndScene* const scene = m_world->GetScene();
	m_invTimestep = ndFloat32(1.0f) / m_timestep;
	m_invStepRK = ndFloat32(0.25f);
	m_timestepRK = m_timestep * m_invStepRK;
	m_invTimestepRK = m_invTimestep * ndFloat32(4.0f);

	const ndArray<ndBodyKinematic*>& bodyArray = scene->GetActiveBodyArray();
	const ndInt32 bodyCount = bodyArray.GetCount();
	GetInternalForces().SetCount(bodyCount);

	ndInt32 extraPassesArray[D_MAX_THREADS_COUNT];
	scene->SubmitJobs<ndInitWeights>(extraPassesArray);

	ndInt32 extraPasses = 0;
	const ndInt32 threadCount = scene->GetThreadCount();
	for (ndInt32 i = 0; i < threadCount; ++i)
	{
		extraPasses = dMax(extraPasses, extraPassesArray[i]);
	}

	const ndInt32 conectivity = 7;
	m_solverPasses = m_world->GetSolverIterations() + 2 * extraPasses / conectivity + 1;
}

void ndDynamicsUpdate::InitBodyArray()
{
	D_TRACKTIME();
	class ndInitBodyArray : public ndThreadPoolJob_old
	{
		public:
		virtual void Execute()
		{
			D_TRACKTIME();
			ndScene* const scene = (ndScene*)GetThreadPool();
			ndWorld* const world = scene->GetWorld();
			ndDynamicsUpdate* const me = world->m_solver;
			ndArray<ndBodyKinematic*>& bodyArray = me->GetBodyIslandOrder();

			const ndFloat32 timestep = scene->GetTimestep();

			const ndStartEnd startEnd(bodyArray.GetCount() - me->GetUnconstrainedBodyCount(), GetThreadId(), GetThreadCount());
			for (ndInt32 i = startEnd.m_start; i < startEnd.m_end; ++i)
			{
				ndBodyKinematic* const body = bodyArray[i];
				dAssert(body);
				dAssert(body->m_bodyIsConstrained | body->m_isStatic);

				body->UpdateInvInertiaMatrix();
				body->AddDampingAcceleration(timestep);
				const ndVector angularMomentum(body->CalculateAngularMomentum());
				body->m_gyroTorque = body->m_omega.CrossProduct(angularMomentum);
				body->m_gyroAlpha = body->m_invWorldInertiaMatrix.RotateVector(body->m_gyroTorque);

				body->m_accel = body->m_veloc;
				body->m_alpha = body->m_omega;
				body->m_gyroRotation = body->m_rotation;
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
	for (ndInt32 i = joint->GetRowsCount() - 1; i >= 0; i--)
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
				skeleton0->AddSelfCollisionJoint(contactJoint);
			}
		}
		else if (contactJoint->IsSkeletonIntraCollision())
		{
			if (skeleton0 && !skeleton1)
			{
				contactJoint->m_isInSkeletonLoop = 1;
				skeleton0->AddSelfCollisionJoint(contactJoint);
			}
			else if (skeleton1 && !skeleton0)
			{
				contactJoint->m_isInSkeletonLoop = 1;
				skeleton1->AddSelfCollisionJoint(contactJoint);
			}
		}
	}
	else
	{
		ndJointBilateralConstraint* const bilareral = joint->GetAsBilateral();
		dAssert(bilareral);
		if (!bilareral->m_isInSkeleton && (bilareral->GetSolverModel() == m_jointkinematicAttachment))
		{
			ndSkeletonContainer* const skeleton0 = bilareral->m_body0->GetSkeleton();
			ndSkeletonContainer* const skeleton1 = bilareral->m_body1->GetSkeleton();
			if (skeleton0 || skeleton1)
			{
				if (skeleton0 && !skeleton1)
				{
					bilareral->m_isInSkeletonLoop = 1;
					skeleton0->AddSelfCollisionJoint(bilareral);
				}
				else if (skeleton1 && !skeleton0)
				{
					bilareral->m_isInSkeletonLoop = 1;
					skeleton1->AddSelfCollisionJoint(bilareral);
				}
			}
		}
	}

	joint->m_rowCount = dof;
	const ndInt32 baseIndex = joint->m_rowStart;
	for (ndInt32 i = 0; i < dof; ++i)
	{
		dAssert(constraintParam.m_forceBounds[i].m_jointForce);

		ndLeftHandSide* const row = &m_leftHandSide[baseIndex + i];
		ndRightHandSide* const rhs = &m_rightHandSide[baseIndex + i];

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

void ndDynamicsUpdate::InitJacobianMatrix()
{
	class ndInitJacobianMatrix : public ndThreadPoolJob_old
	{
		public:
		ndInitJacobianMatrix()
			:m_zero(ndVector::m_zero)
		{
		}

		void BuildJacobianMatrix(ndConstraint* const joint, ndInt32 jointIndex)
		{
			dAssert(joint->GetBody0());
			dAssert(joint->GetBody1());
			const ndBodyKinematic* const body0 = joint->GetBody0();
			const ndBodyKinematic* const body1 = joint->GetBody1();

			const ndVector force0(body0->GetForce());
			const ndVector torque0(body0->GetTorque());
			const ndVector force1(body1->GetForce());
			const ndVector torque1(body1->GetTorque());

			const ndInt32 index = joint->m_rowStart;
			const ndInt32 count = joint->m_rowCount;
			const ndMatrix& invInertia0 = body0->m_invWorldInertiaMatrix;
			const ndMatrix& invInertia1 = body1->m_invWorldInertiaMatrix;
			const ndVector invMass0(body0->m_invMass[3]);
			const ndVector invMass1(body1->m_invMass[3]);

			joint->m_preconditioner0 = ndFloat32(1.0f);
			joint->m_preconditioner1 = ndFloat32(1.0f);

			const bool test = !((body0->m_isStatic | body1->m_isStatic) || (body0->GetSkeleton() && body1->GetSkeleton()));
			dAssert(test == ((invMass0.GetScalar() > ndFloat32(0.0f)) && (invMass1.GetScalar() > ndFloat32(0.0f)) && !(body0->GetSkeleton() && body1->GetSkeleton())));
			if (test)
			{
				const ndFloat32 mass0 = body0->GetMassMatrix().m_w;
				const ndFloat32 mass1 = body1->GetMassMatrix().m_w;
				if (mass0 > (D_DIAGONAL_PRECONDITIONER * mass1))
				{
					joint->m_preconditioner0 = mass0 / (mass1 * D_DIAGONAL_PRECONDITIONER);
				}
				else if (mass1 > (D_DIAGONAL_PRECONDITIONER * mass0))
				{
					joint->m_preconditioner1 = mass1 / (mass0 * D_DIAGONAL_PRECONDITIONER);
				}
			}

			ndVector forceAcc0(m_zero);
			ndVector torqueAcc0(m_zero);
			ndVector forceAcc1(m_zero);
			ndVector torqueAcc1(m_zero);

#ifdef D_PROGRESSIVE_SLEEP_EXPERIMENT
			const ndVector progressiveSleepWeigh (ndFloat32 (0.01f));
			if (body0->m_isJointFence1 & !body0->m_isStatic)
			{
				for (ndInt32 i = 0; i < count; ++i)
				{
					ndLeftHandSide* const row = &m_leftHandSide[index + i];
					row->m_Jt.m_jacobianM0.m_linear = row->m_Jt.m_jacobianM0.m_linear * progressiveSleepWeigh;
					row->m_Jt.m_jacobianM0.m_angular = row->m_Jt.m_jacobianM0.m_linear * progressiveSleepWeigh;
				}
			}

			if (body1->m_isJointFence1 & !body1->m_isStatic)
			{
				for (ndInt32 i = 0; i < count; ++i)
				{
					ndLeftHandSide* const row = &m_leftHandSide[index + i];
					row->m_Jt.m_jacobianM1.m_linear = row->m_Jt.m_jacobianM1.m_linear * progressiveSleepWeigh;
					row->m_Jt.m_jacobianM1.m_angular = row->m_Jt.m_jacobianM1.m_linear * progressiveSleepWeigh;
				}
			}
#endif

			const ndVector weigh0(body0->m_weigh * joint->m_preconditioner0);
			const ndVector weigh1(body1->m_weigh * joint->m_preconditioner1);

			const ndFloat32 preconditioner0 = joint->m_preconditioner0;
			const ndFloat32 preconditioner1 = joint->m_preconditioner1;

			const bool isBilateral = joint->IsBilateral();
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
				const ndVector tmpAccel(
					JMinvM0.m_linear * force0 + JMinvM0.m_angular * torque0 +
					JMinvM1.m_linear * force1 + JMinvM1.m_angular * torque1);

				const ndFloat32 extenalAcceleration = -tmpAccel.AddHorizontal().GetScalar();
				rhs->m_deltaAccel = extenalAcceleration;
				rhs->m_coordenateAccel += extenalAcceleration;
				dAssert(rhs->m_jointFeebackForce);
				const ndFloat32 force = rhs->m_jointFeebackForce->GetInitialGuess();

				rhs->m_force = isBilateral ? dClamp(force, rhs->m_lowerBoundFrictionCoefficent, rhs->m_upperBoundFrictionCoefficent) : force;
				rhs->m_maxImpact = ndFloat32(0.0f);

				const ndJacobian& JtM0 = row->m_Jt.m_jacobianM0;
				const ndJacobian& JtM1 = row->m_Jt.m_jacobianM1;
				const ndVector tmpDiag(
					weigh0 * (JMinvM0.m_linear * JtM0.m_linear + JMinvM0.m_angular * JtM0.m_angular) +
					weigh1 * (JMinvM1.m_linear * JtM1.m_linear + JMinvM1.m_angular * JtM1.m_angular));

				ndFloat32 diag = tmpDiag.AddHorizontal().GetScalar();
				dAssert(diag > ndFloat32(0.0f));
				rhs->m_diagDamp = diag * rhs->m_diagonalRegularizer;

				diag *= (ndFloat32(1.0f) + rhs->m_diagonalRegularizer);
				rhs->m_invJinvMJt = ndFloat32(1.0f) / diag;

				ndVector f0(rhs->m_force * preconditioner0);
				ndVector f1(rhs->m_force * preconditioner1);
				forceAcc0 = forceAcc0 + JtM0.m_linear * f0;
				torqueAcc0 = torqueAcc0 + JtM0.m_angular * f0;
				forceAcc1 = forceAcc1 + JtM1.m_linear * f1;
				torqueAcc1 = torqueAcc1 + JtM1.m_angular * f1;
			}

			const ndInt32 index0 = jointIndex * 2 + 0;
			ndJacobian& outBody0 = m_internalForces[index0];
			outBody0.m_linear = forceAcc0;
			outBody0.m_angular = torqueAcc0;

			const ndInt32 index1 = jointIndex * 2 + 1;
			ndJacobian& outBody1 = m_internalForces[index1];
			outBody1.m_linear = forceAcc1;
			outBody1.m_angular = torqueAcc1;
		}

		virtual void Execute()
		{
			D_TRACKTIME();
			ndScene* const scene = (ndScene*)GetThreadPool();
			ndWorld* const world = scene->GetWorld();
			ndDynamicsUpdate* const me = world->m_solver;
			m_leftHandSide = &me->GetLeftHandSide()[0];
			m_rightHandSide = &me->GetRightHandSide()[0];
			m_internalForces = &me->GetTempInternalForces()[0];
			m_jointBodyPairIndexBuffer = &me->GetJointBodyPairIndexBuffer()[0];
			ndConstraint** const jointArray = &scene->GetActiveContactArray()[0];

			const ndInt32 threadIndex = GetThreadId();
			const ndInt32 threadCount = GetThreadCount();
			const ndInt32 jointCount = scene->GetActiveContactArray().GetCount();
			for (ndInt32 i = threadIndex; i < jointCount; i += threadCount)
			{
				ndConstraint* const joint = jointArray[i];
				me->GetJacobianDerivatives(joint);
				BuildJacobianMatrix(joint, i);
			}
		}

		ndVector m_zero;
		ndJacobian* m_internalForces;
		ndLeftHandSide* m_leftHandSide;
		ndRightHandSide* m_rightHandSide;
		const ndJointBodyPairIndex* m_jointBodyPairIndexBuffer;
	};

	class ndInitJacobianAccumulatePartialForces : public ndThreadPoolJob_old
	{
		public:
		virtual void Execute()
		{
			D_TRACKTIME();
			const ndVector zero(ndVector::m_zero);
			ndScene* const scene = (ndScene*)GetThreadPool();
			ndWorld* const world = scene->GetWorld();
			ndDynamicsUpdate* const me = (ndDynamicsUpdate*)world->m_solver;

			ndJacobian* const internalForces = &me->GetInternalForces()[0];
			const ndArray<ndInt32>& bodyIndex = me->GetJointForceIndexBuffer();
			ndBodyKinematic** const bodyArray = &scene->GetActiveBodyArray()[0];

			const ndJacobian* const jointInternalForces = &me->GetTempInternalForces()[0];
			const ndJointBodyPairIndex* const jointBodyPairIndexBuffer = &me->GetJointBodyPairIndexBuffer()[0];

			const ndStartEnd startEnd(bodyIndex.GetCount() - 1, GetThreadId(), GetThreadCount());
			for (ndInt32 i = startEnd.m_start; i < startEnd.m_end; ++i)
			{
				ndVector force(zero);
				ndVector torque(zero);

				const ndInt32 index = bodyIndex[i];
				const ndJointBodyPairIndex& scan = jointBodyPairIndexBuffer[index];
				ndBodyKinematic* const body = bodyArray[scan.m_body];

				dAssert(body->m_isStatic <= 1);
				dAssert(body->m_index == scan.m_body);
				const ndInt32 mask = ndInt32(body->m_isStatic) - 1;
				const ndInt32 count = mask & (bodyIndex[i + 1] - index);

				for (ndInt32 j = 0; j < count; ++j)
				{
					const ndInt32 jointIndex = jointBodyPairIndexBuffer[index + j].m_joint;
					force += jointInternalForces[jointIndex].m_linear;
					torque += jointInternalForces[jointIndex].m_angular;
				}
				internalForces[i].m_linear = force;
				internalForces[i].m_angular = torque;
			}
		}
	};

	ndScene* const scene = m_world->GetScene();
	if (scene->GetActiveContactArray().GetCount())
	{
		D_TRACKTIME();
		m_rightHandSide[0].m_force = ndFloat32(1.0f);
		scene->SubmitJobs<ndInitJacobianMatrix>();
		scene->SubmitJobs<ndInitJacobianAccumulatePartialForces>();
	}
}

void ndDynamicsUpdate::CalculateJointsAcceleration()
{
	D_TRACKTIME();
	class ndCalculateJointsAcceleration : public ndThreadPoolJob_old
	{
		public:
		virtual void Execute()
		{
			D_TRACKTIME();
			ndScene* const scene = (ndScene*)GetThreadPool();
			ndWorld* const world = scene->GetWorld();
			ndDynamicsUpdate* const me = world->m_solver;
			const ndArray<ndConstraint*>& jointArray = scene->GetActiveContactArray();

			ndJointAccelerationDecriptor joindDesc;
			joindDesc.m_timestep = me->m_timestepRK;
			joindDesc.m_invTimestep = me->m_invTimestepRK;
			joindDesc.m_firstPassCoefFlag = me->m_firstPassCoef;
			ndArray<ndLeftHandSide>& leftHandSide = me->m_leftHandSide;
			ndArray<ndRightHandSide>& rightHandSide = me->m_rightHandSide;

			const ndStartEnd startEnd(scene->GetActiveContactArray().GetCount(), GetThreadId(), GetThreadCount());
			for (ndInt32 i = startEnd.m_start; i < startEnd.m_end; ++i)
			{
				ndConstraint* const joint = jointArray[i];
				const ndInt32 pairStart = joint->m_rowStart;
				joindDesc.m_rowsCount = joint->m_rowCount;
				joindDesc.m_leftHandSide = &leftHandSide[pairStart];
				joindDesc.m_rightHandSide = &rightHandSide[pairStart];
				joint->JointAccelerations(&joindDesc);
			}
		}
	};

	ndScene* const scene = m_world->GetScene();
	scene->SubmitJobs<ndCalculateJointsAcceleration>();
	m_firstPassCoef = ndFloat32(1.0f);
}

void ndDynamicsUpdate::IntegrateBodiesVelocity()
{
	D_TRACKTIME();
	class ndIntegrateBodiesVelocity : public ndThreadPoolJob_old
	{
		public:
		virtual void Execute()
		{
			D_TRACKTIME();
			ndScene* const scene = (ndScene*)GetThreadPool();
			ndWorld* const world = scene->GetWorld();
			ndDynamicsUpdate* const me = world->m_solver;
			ndArray<ndBodyKinematic*>& bodyArray = me->GetBodyIslandOrder();
			const ndArray<ndJacobian>& internalForces = me->GetInternalForces();

			const ndVector timestep4(me->GetTimestepRK());
			const ndVector speedFreeze2(world->m_freezeSpeed2 * ndFloat32(0.1f));

			const ndStartEnd startEnd(bodyArray.GetCount() - me->GetUnconstrainedBodyCount(), GetThreadId(), GetThreadCount());
			for (ndInt32 i = startEnd.m_start; i < startEnd.m_end; ++i)
			{
				ndBodyKinematic* const body = bodyArray[i];

				dAssert(body);
				dAssert(body->GetAsBodyDynamic());
				dAssert(body->m_bodyIsConstrained);
				const ndInt32 index = body->m_index;
				const ndJacobian& forceAndTorque = internalForces[index];
				const ndVector force(body->GetForce() + forceAndTorque.m_linear);
				const ndVector torque(body->GetTorque() + forceAndTorque.m_angular - body->GetGyroTorque());
				const ndJacobian velocStep(body->IntegrateForceAndToque(force, torque, timestep4));

				if (!body->m_equilibrium0)
				{
					body->m_veloc += velocStep.m_linear;
					body->m_omega += velocStep.m_angular;
					body->IntegrateGyroSubstep(timestep4);
				}
				else
				{
					const ndVector velocStep2(velocStep.m_linear.DotProduct(velocStep.m_linear));
					const ndVector omegaStep2(velocStep.m_angular.DotProduct(velocStep.m_angular));
					const ndVector test(((velocStep2 > speedFreeze2) | (omegaStep2 > speedFreeze2)) & ndVector::m_negOne);
					const ndInt8 equilibrium = test.GetSignMask() ? 0 : 1;
					body->m_equilibrium0 = equilibrium;
				}
				dAssert(body->m_veloc.m_w == ndFloat32(0.0f));
				dAssert(body->m_omega.m_w == ndFloat32(0.0f));
			}
		}
	};

	ndScene* const scene = m_world->GetScene();
	scene->SubmitJobs<ndIntegrateBodiesVelocity>();
}

void ndDynamicsUpdate::UpdateForceFeedback()
{
	D_TRACKTIME();
	class ndUpdateForceFeedback : public ndThreadPoolJob_old
	{
		public:
		virtual void Execute()
		{
			D_TRACKTIME();
			ndScene* const scene = (ndScene*)GetThreadPool();
			ndWorld* const world = scene->GetWorld();
			ndDynamicsUpdate* const me = world->m_solver;
			const ndArray<ndConstraint*>& jointArray = scene->GetActiveContactArray();
			ndArray<ndRightHandSide>& rightHandSide = me->m_rightHandSide;

			const ndFloat32 timestepRK = me->GetTimestepRK();
			const ndStartEnd startEnd(jointArray.GetCount(), GetThreadId(), GetThreadCount());
			for (ndInt32 i = startEnd.m_start; i < startEnd.m_end; ++i)
			{
				ndConstraint* const joint = jointArray[i];
				const ndInt32 rows = joint->m_rowCount;
				const ndInt32 first = joint->m_rowStart;

				for (ndInt32 j = 0; j < rows; ++j)
				{
					const ndRightHandSide* const rhs = &rightHandSide[j + first];
					dAssert(dCheckFloat(rhs->m_force));
					rhs->m_jointFeebackForce->Push(rhs->m_force);
					rhs->m_jointFeebackForce->m_force = rhs->m_force;
					rhs->m_jointFeebackForce->m_impact = rhs->m_maxImpact * timestepRK;
				}

				if (joint->GetAsBilateral())
				{
					const ndArray<ndLeftHandSide>& leftHandSide = me->m_leftHandSide;
					ndVector force0(ndVector::m_zero);
					ndVector force1(ndVector::m_zero);
					ndVector torque0(ndVector::m_zero);
					ndVector torque1(ndVector::m_zero);

					for (ndInt32 j = 0; j < rows; ++j)
					{
						const ndRightHandSide* const rhs = &rightHandSide[j + first];
						const ndLeftHandSide* const lhs = &leftHandSide[j + first];
						const ndVector f(rhs->m_force);
						force0 += lhs->m_Jt.m_jacobianM0.m_linear * f;
						torque0 += lhs->m_Jt.m_jacobianM0.m_angular * f;
						force1 += lhs->m_Jt.m_jacobianM1.m_linear * f;
						torque1 += lhs->m_Jt.m_jacobianM1.m_angular * f;
					}
					ndJointBilateralConstraint* const bilateral = (ndJointBilateralConstraint*)joint;
					bilateral->m_forceBody0 = force0;
					bilateral->m_torqueBody0 = torque0;
					bilateral->m_forceBody1 = force1;
					bilateral->m_torqueBody1 = torque1;
				}
			}
		}
	};

	ndScene* const scene = m_world->GetScene();
	scene->SubmitJobs<ndUpdateForceFeedback>();
}

void ndDynamicsUpdate::IntegrateBodies()
{
	D_TRACKTIME();
	class ndIntegrateBodies : public ndThreadPoolJob_old
	{
		public:
		virtual void Execute()
		{
			D_TRACKTIME();
			ndScene* const scene = (ndScene*)GetThreadPool();
			ndWorld* const world = scene->GetWorld();
			ndDynamicsUpdate* const me = world->m_solver;
			ndArray<ndBodyKinematic*>& bodyArray = me->GetBodyIslandOrder();

			const ndVector invTime(me->m_invTimestep);
			const ndFloat32 timestep = scene->GetTimestep();

			const ndStartEnd startEnd(bodyArray.GetCount(), GetThreadId(), GetThreadCount());
			for (ndInt32 i = startEnd.m_start; i < startEnd.m_end; ++i)
			{
				ndBodyKinematic* const body = bodyArray[i];
				if (!body->m_equilibrium)
				{
					body->m_accel = invTime * (body->m_veloc - body->m_accel);
					body->m_alpha = invTime * (body->m_omega - body->m_alpha);
					body->IntegrateVelocity(timestep);
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
	class ndDetermineSleepStates : public ndThreadPoolJob_old
	{
		public:
		void UpdateIslandState(const ndIsland& island)
		{
			ndScene* const scene = (ndScene*)GetThreadPool();
			ndWorld* const world = scene->GetWorld();
			ndFloat32 velocityDragCoeff = D_FREEZZING_VELOCITY_DRAG;

			const ndInt32 count = island.m_count;
			if (count <= D_SMALL_ISLAND_COUNT)
			{
				velocityDragCoeff = ndFloat32(0.9999f);
			}

			ndFloat32 maxAccel = ndFloat32(0.0f);
			ndFloat32 maxAlpha = ndFloat32(0.0f);
			ndFloat32 maxSpeed = ndFloat32(0.0f);
			ndFloat32 maxOmega = ndFloat32(0.0f);

			const ndFloat32 speedFreeze = world->m_freezeSpeed2;
			const ndFloat32 accelFreeze = world->m_freezeAccel2 * ((count <= D_SMALL_ISLAND_COUNT) ? ndFloat32(0.01f) : ndFloat32(1.0f));
			const ndFloat32 acc2 = D_SOLVER_MAX_ERROR * D_SOLVER_MAX_ERROR;
			const ndFloat32 maxAccNorm2 = (count > 4) ? acc2 : acc2 * ndFloat32(0.0625f);
			const ndVector velocDragVect(velocityDragCoeff, velocityDragCoeff, velocityDragCoeff, ndFloat32(0.0f));

			ndInt32 stackSleeping = 1;
			ndInt32 sleepCounter = 10000;

			ndDynamicsUpdate* const me = world->m_solver;
			ndBodyKinematic** const bodyIslands = &me->GetBodyIslandOrder()[island.m_start];
			for (ndInt32 i = 0; i < count; ++i)
			{
				ndBodyDynamic* const dynBody = bodyIslands[i]->GetAsBodyDynamic();
				if (dynBody)
				{
					dAssert(dynBody->m_accel.m_w == ndFloat32(0.0f));
					dAssert(dynBody->m_alpha.m_w == ndFloat32(0.0f));
					dAssert(dynBody->m_veloc.m_w == ndFloat32(0.0f));
					dAssert(dynBody->m_omega.m_w == ndFloat32(0.0f));

					ndVector accelTest((dynBody->m_accel.DotProduct(dynBody->m_accel) > maxAccNorm2) | (dynBody->m_alpha.DotProduct(dynBody->m_alpha) > maxAccNorm2));
					dynBody->m_accel = dynBody->m_accel & accelTest;
					dynBody->m_alpha = dynBody->m_alpha & accelTest;

					ndUnsigned8 equilibrium = dynBody->m_isStatic | dynBody->m_autoSleep;
					dAssert(equilibrium == ((dynBody->m_invMass.m_w == ndFloat32(0.0f)) ? 1 : dynBody->m_autoSleep));
					const ndVector isMovingMask(dynBody->m_veloc + dynBody->m_omega + dynBody->m_accel + dynBody->m_alpha);
					const ndVector mask(isMovingMask.TestZero());
					const ndInt32 test = mask.GetSignMask() & 7;
					if (test != 7)
					{
						const ndFloat32 accel2 = dynBody->m_accel.DotProduct(dynBody->m_accel).GetScalar();
						const ndFloat32 alpha2 = dynBody->m_alpha.DotProduct(dynBody->m_alpha).GetScalar();
						const ndFloat32 speed2 = dynBody->m_veloc.DotProduct(dynBody->m_veloc).GetScalar();
						const ndFloat32 omega2 = dynBody->m_omega.DotProduct(dynBody->m_omega).GetScalar();

						maxAccel = dMax(maxAccel, accel2);
						maxAlpha = dMax(maxAlpha, alpha2);
						maxSpeed = dMax(maxSpeed, speed2);
						maxOmega = dMax(maxOmega, omega2);
						ndUnsigned32 equilibriumTest = (accel2 < accelFreeze) && (alpha2 < accelFreeze) && (speed2 < speedFreeze) && (omega2 < speedFreeze);

						if (equilibriumTest)
						{
							const ndVector veloc(dynBody->m_veloc * velocDragVect);
							const ndVector omega(dynBody->m_omega * velocDragVect);
							const ndVector velocMask(veloc.DotProduct(veloc) > m_velocTol);
							const ndVector omegaMask(omega.DotProduct(omega) > m_velocTol);
							dynBody->m_veloc = velocMask & veloc;
							dynBody->m_omega = omegaMask & omega;
						}

						equilibrium &= equilibriumTest;
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
					ndUnsigned8 equilibrium = (kinBody->m_invMass.m_w == ndFloat32(0.0f)) ? 1 : (kinBody->m_autoSleep & ~kinBody->m_equilibriumOverride);
					const ndVector isMovingMask(kinBody->m_veloc + kinBody->m_omega);
					const ndVector mask(isMovingMask.TestZero());
					const ndInt32 test = mask.GetSignMask() & 7;
					if (test != 7)
					{
						const ndFloat32 speed2 = kinBody->m_veloc.DotProduct(kinBody->m_veloc).GetScalar();
						const ndFloat32 omega2 = kinBody->m_omega.DotProduct(kinBody->m_omega).GetScalar();

						maxSpeed = dMax(maxSpeed, speed2);
						maxOmega = dMax(maxOmega, omega2);
						ndUnsigned32 equilibriumTest = (speed2 < speedFreeze) && (omega2 < speedFreeze);

						if (equilibriumTest)
						{
							const ndVector veloc(kinBody->m_veloc * velocDragVect);
							const ndVector omega(kinBody->m_omega * velocDragVect);
							const ndVector velocMask(veloc.DotProduct(veloc) > m_velocTol);
							const ndVector omegaMask(omega.DotProduct(omega) > m_velocTol);
							kinBody->m_veloc = velocMask & veloc;
							kinBody->m_omega = omegaMask & omega;
						}

						equilibrium &= equilibriumTest;
						stackSleeping &= equilibrium;
						sleepCounter = dMin(sleepCounter, kinBody->m_sleepingCounter);
					}
					if (kinBody->m_equilibrium != equilibrium)
					{
						kinBody->m_equilibrium = equilibrium;
					}
				}
			}

			if (stackSleeping)
			{
				for (ndInt32 i = 0; i < count; ++i)
				{
					// force entire island to equilibriumTest
					ndBodyDynamic* const body = bodyIslands[i]->GetAsBodyDynamic();
					if (body)
					{
						body->m_accel = ndVector::m_zero;
						body->m_alpha = ndVector::m_zero;
						body->m_veloc = ndVector::m_zero;
						body->m_omega = ndVector::m_zero;
						//body->m_equilibrium = (body->m_invMass.m_w == ndFloat32(0.0f)) ? 1 : body->m_autoSleep;
						body->m_equilibrium = body->m_isStatic | body->m_autoSleep;
					}
					else
					{
						ndBodyKinematic* const kinBody = bodyIslands[i]->GetAsBodyKinematic();
						dAssert(kinBody);
						kinBody->m_veloc = ndVector::m_zero;
						kinBody->m_omega = ndVector::m_zero;
						kinBody->m_equilibrium = kinBody->m_isStatic | kinBody->m_autoSleep;
					}
				}
			}
			else if ((count > 1) || bodyIslands[0]->m_bodyIsConstrained)
			{
				const bool state =
					(maxAccel > world->m_sleepTable[D_SLEEP_ENTRIES - 1].m_maxAccel) ||
					(maxAlpha > world->m_sleepTable[D_SLEEP_ENTRIES - 1].m_maxAlpha) ||
					(maxSpeed > world->m_sleepTable[D_SLEEP_ENTRIES - 1].m_maxVeloc) ||
					(maxOmega > world->m_sleepTable[D_SLEEP_ENTRIES - 1].m_maxOmega);

				if (state)
				{
					for (ndInt32 i = 0; i < count; ++i)
					{
						ndBodyDynamic* const body = bodyIslands[i]->GetAsBodyDynamic();
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
						for (ndInt32 i = 0; i < count; ++i)
						{
							ndBodyKinematic* const body = bodyIslands[i];
							body->m_equilibrium = 0;
						}
					}
					ndInt32 timeScaleSleepCount = ndInt32(ndFloat32(60.0f) * sleepCounter * scene->GetTimestep());

					ndInt32 sleepIndex = D_SLEEP_ENTRIES;
					for (ndInt32 i = 1; i < D_SLEEP_ENTRIES; ++i)
					{
						if (world->m_sleepTable[i].m_steps > timeScaleSleepCount)
						{
							sleepIndex = i;
							break;
						}
					}
					sleepIndex--;

					bool state1 =
						(maxAccel < world->m_sleepTable[sleepIndex].m_maxAccel) &&
						(maxAlpha < world->m_sleepTable[sleepIndex].m_maxAlpha) &&
						(maxSpeed < world->m_sleepTable[sleepIndex].m_maxVeloc) &&
						(maxOmega < world->m_sleepTable[sleepIndex].m_maxOmega);
					if (state1)
					{
						for (ndInt32 i = 0; i < count; ++i)
						{
							ndBodyKinematic* const body = bodyIslands[i];
							body->m_veloc = ndVector::m_zero;
							body->m_omega = ndVector::m_zero;
							body->m_equilibrium = body->m_autoSleep;
							ndBodyDynamic* const dynBody = body->GetAsBodyDynamic();
							if (dynBody)
							{
								dynBody->m_accel = ndVector::m_zero;
								dynBody->m_alpha = ndVector::m_zero;
								dynBody->m_sleepingCounter = 0;
							}
						}
					}
				}
			}
		}

		virtual void Execute()
		{
			D_TRACKTIME();
			ndScene* const scene = (ndScene*)GetThreadPool();
			ndWorld* const world = scene->GetWorld();
			ndDynamicsUpdate* const me = world->m_solver;
			const ndArray<ndIsland>& islandArray = me->GetIslands();

			const ndInt32 threadIndex = GetThreadId();
			const ndInt32 threadCount = GetThreadCount();
			const ndInt32 islandCount = islandArray.GetCount();

			m_zero = ndVector::m_zero;
			m_velocTol = me->GetVelocTol();
			for (ndInt32 i = threadIndex; i < islandCount; i += threadCount)
			{
				const ndIsland& island = islandArray[i];
				UpdateIslandState(island);
			}
		}

		ndVector m_zero;
		ndVector m_velocTol;
	};

	ndScene* const scene = m_world->GetScene();
	scene->SubmitJobs<ndDetermineSleepStates>();
}

void ndDynamicsUpdate::InitSkeletons()
{
	D_TRACKTIME();

	class ndInitSkeletons : public ndThreadPoolJob_old
	{
		public:
		virtual void Execute()
		{
			D_TRACKTIME();
			ndScene* const scene = (ndScene*)GetThreadPool();
			const ndInt32 threadIndex = GetThreadId();
			ndWorld* const world = scene->GetWorld();
			ndDynamicsUpdate* const me = world->m_solver;
			ndSkeletonList::ndNode* node = world->GetSkeletonList().GetFirst();
			for (ndInt32 i = 0; i < threadIndex; ++i)
			{
				node = node ? node->GetNext() : nullptr;
			}

			ndArray<ndRightHandSide>& rightHandSide = me->m_rightHandSide;
			const ndArray<ndLeftHandSide>& leftHandSide = me->m_leftHandSide;

			const ndInt32 threadCount = GetThreadCount();
			while (node)
			{
				ndSkeletonContainer* const skeleton = &node->GetInfo();
				skeleton->InitMassMatrix(&leftHandSide[0], &rightHandSide[0]);

				for (ndInt32 i = 0; i < threadCount; ++i)
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
	class ndUpdateSkeletons : public ndThreadPoolJob_old
	{
		public:
		virtual void Execute()
		{
			D_TRACKTIME();
			ndScene* const scene = (ndScene*)GetThreadPool();
			const ndInt32 threadIndex = GetThreadId();
			ndWorld* const world = scene->GetWorld();
			ndDynamicsUpdate* const me = world->m_solver;
			ndSkeletonList::ndNode* node = world->GetSkeletonList().GetFirst();
			for (ndInt32 i = 0; i < threadIndex; ++i)
			{
				node = node ? node->GetNext() : nullptr;
			}

			ndJacobian* const internalForces = &me->GetInternalForces()[0];
			const ndArray<ndBodyKinematic*>& activeBodies = scene->ndScene::GetActiveBodyArray();
			const ndBodyKinematic** const bodyArray = (const ndBodyKinematic**)&activeBodies[0];

			const ndInt32 threadCount = GetThreadCount();
			while (node)
			{
				ndSkeletonContainer* const skeleton = &node->GetInfo();
				skeleton->CalculateJointForce(bodyArray, internalForces);

				for (ndInt32 i = 0; i < threadCount; ++i)
				{
					node = node ? node->GetNext() : nullptr;
				}
			}
		}
	};

	ndScene* const scene = m_world->GetScene();
	scene->SubmitJobs<ndUpdateSkeletons>();
}

void ndDynamicsUpdate::CalculateJointsForce()
{
	D_TRACKTIME();
	class ndCalculateJointsForce : public ndThreadPoolJob_old
	{
		public:
		ndCalculateJointsForce()
			:m_zero(ndVector::m_zero)
		{
		}

		void JointForce(ndConstraint* const joint, ndInt32 jointIndex)
		{
			#ifdef D_USE_EARLY_OUT_JOINT
			ndVector accNorm(m_zero);
			#endif
			ndBodyKinematic* const body0 = joint->GetBody0();
			ndBodyKinematic* const body1 = joint->GetBody1();
			dAssert(body0);
			dAssert(body1);

			const ndInt32 m0 = body0->m_index;
			const ndInt32 m1 = body1->m_index;
			const ndInt32 rowStart = joint->m_rowStart;
			const ndInt32 rowsCount = joint->m_rowCount;

			const ndInt32 resting = body0->m_equilibrium0 & body1->m_equilibrium0;
			if (!resting)
			{
				ndVector preconditioner0(joint->m_preconditioner0);
				ndVector preconditioner1(joint->m_preconditioner1);

				ndVector forceM0(m_internalForces[m0].m_linear * preconditioner0);
				ndVector torqueM0(m_internalForces[m0].m_angular * preconditioner0);
				ndVector forceM1(m_internalForces[m1].m_linear * preconditioner1);
				ndVector torqueM1(m_internalForces[m1].m_angular * preconditioner1);

				preconditioner0 = preconditioner0.Scale(body0->m_weigh);
				preconditioner1 = preconditioner1.Scale(body1->m_weigh);

				for (ndInt32 j = 0; j < rowsCount; ++j)
				{
					ndRightHandSide* const rhs = &m_rightHandSide[rowStart + j];
					const ndLeftHandSide* const lhs = &m_leftHandSide[rowStart + j];
					const ndVector force(rhs->m_force);

					ndVector a(lhs->m_JMinv.m_jacobianM0.m_linear * forceM0);
					a = a.MulAdd(lhs->m_JMinv.m_jacobianM0.m_angular, torqueM0);
					a = a.MulAdd(lhs->m_JMinv.m_jacobianM1.m_linear, forceM1);
					a = a.MulAdd(lhs->m_JMinv.m_jacobianM1.m_angular, torqueM1);
					a = ndVector(rhs->m_coordenateAccel - rhs->m_force * rhs->m_diagDamp) - a.AddHorizontal();

					dAssert(rhs->m_normalForceIndexFlat >= 0);
					ndVector f(force + a.Scale(rhs->m_invJinvMJt));
					const ndInt32 frictionIndex = rhs->m_normalForceIndexFlat;
					const ndFloat32 frictionNormal = m_rightHandSide[frictionIndex].m_force;
					const ndVector lowerFrictionForce(frictionNormal * rhs->m_lowerBoundFrictionCoefficent);
					const ndVector upperFrictionForce(frictionNormal * rhs->m_upperBoundFrictionCoefficent);

					#ifdef D_USE_EARLY_OUT_JOINT
					a = a & (f < upperFrictionForce) & (f > lowerFrictionForce);
					accNorm = accNorm.MulAdd(a, a);
					#endif

					f = f.GetMax(lowerFrictionForce).GetMin(upperFrictionForce);
					rhs->m_force = f.GetScalar();

					const ndVector deltaForce(f - force);
					const ndVector deltaForce0(deltaForce * preconditioner0);
					const ndVector deltaForce1(deltaForce * preconditioner1);

					forceM0 = forceM0.MulAdd(lhs->m_Jt.m_jacobianM0.m_linear, deltaForce0);
					torqueM0 = torqueM0.MulAdd(lhs->m_Jt.m_jacobianM0.m_angular, deltaForce0);
					forceM1 = forceM1.MulAdd(lhs->m_Jt.m_jacobianM1.m_linear, deltaForce1);
					torqueM1 = torqueM1.MulAdd(lhs->m_Jt.m_jacobianM1.m_angular, deltaForce1);
				}

				const ndFloat32 tol = ndFloat32(0.125f);
				const ndFloat32 tol2 = tol * tol;

				#ifdef D_USE_EARLY_OUT_JOINT
				ndVector maxAccel(accNorm);
				for (ndInt32 k = 0; (k < 4) && (maxAccel.GetScalar() > tol2); k++)
				#else
				for (ndInt32 k = 0; k < 4; ++k)
				#endif
				{
					#ifdef D_USE_EARLY_OUT_JOINT
					maxAccel = m_zero;
					#endif

					for (ndInt32 j = 0; j < rowsCount; ++j)
					{
						ndRightHandSide* const rhs = &m_rightHandSide[rowStart + j];
						const ndLeftHandSide* const lhs = &m_leftHandSide[rowStart + j];
						const ndVector force(rhs->m_force);

						ndVector a(lhs->m_JMinv.m_jacobianM0.m_linear * forceM0);
						a = a.MulAdd(lhs->m_JMinv.m_jacobianM0.m_angular, torqueM0);
						a = a.MulAdd(lhs->m_JMinv.m_jacobianM1.m_linear, forceM1);
						a = a.MulAdd(lhs->m_JMinv.m_jacobianM1.m_angular, torqueM1);
						a = ndVector(rhs->m_coordenateAccel - rhs->m_force * rhs->m_diagDamp) - a.AddHorizontal();

						ndVector f(force + a.Scale(rhs->m_invJinvMJt));
						dAssert(rhs->m_normalForceIndexFlat >= 0);
						const ndInt32 frictionIndex = rhs->m_normalForceIndexFlat;
						const ndFloat32 frictionNormal = m_rightHandSide[frictionIndex].m_force;

						const ndVector lowerFrictionForce(frictionNormal * rhs->m_lowerBoundFrictionCoefficent);
						const ndVector upperFrictionForce(frictionNormal * rhs->m_upperBoundFrictionCoefficent);

						#ifdef D_USE_EARLY_OUT_JOINT
						a = a & (f < upperFrictionForce) & (f > lowerFrictionForce);
						maxAccel = maxAccel.MulAdd(a, a);
						#endif

						f = f.GetMax(lowerFrictionForce).GetMin(upperFrictionForce);
						rhs->m_force = f.GetScalar();

						const ndVector deltaForce(f - force);
						const ndVector deltaForce0(deltaForce * preconditioner0);
						const ndVector deltaForce1(deltaForce * preconditioner1);
						forceM0 = forceM0.MulAdd(lhs->m_Jt.m_jacobianM0.m_linear, deltaForce0);
						torqueM0 = torqueM0.MulAdd(lhs->m_Jt.m_jacobianM0.m_angular, deltaForce0);
						forceM1 = forceM1.MulAdd(lhs->m_Jt.m_jacobianM1.m_linear, deltaForce1);
						torqueM1 = torqueM1.MulAdd(lhs->m_Jt.m_jacobianM1.m_angular, deltaForce1);
					}
				}
			}

			ndVector forceM0(m_zero);
			ndVector torqueM0(m_zero);
			ndVector forceM1(m_zero);
			ndVector torqueM1(m_zero);

			for (ndInt32 j = 0; j < rowsCount; ++j)
			{
				ndRightHandSide* const rhs = &m_rightHandSide[rowStart + j];
				const ndLeftHandSide* const lhs = &m_leftHandSide[rowStart + j];

				const ndVector f(rhs->m_force);
				forceM0 = forceM0.MulAdd(lhs->m_Jt.m_jacobianM0.m_linear, f);
				torqueM0 = torqueM0.MulAdd(lhs->m_Jt.m_jacobianM0.m_angular, f);
				forceM1 = forceM1.MulAdd(lhs->m_Jt.m_jacobianM1.m_linear, f);
				torqueM1 = torqueM1.MulAdd(lhs->m_Jt.m_jacobianM1.m_angular, f);
				rhs->m_maxImpact = dMax(dAbs(f.GetScalar()), rhs->m_maxImpact);
			}

			const ndInt32 index0 = jointIndex * 2 + 0;
			ndJacobian& outBody0 = m_jointPartialForces[index0];
			outBody0.m_linear = forceM0;
			outBody0.m_angular = torqueM0;

			const ndInt32 index1 = jointIndex * 2 + 1;
			ndJacobian& outBody1 = m_jointPartialForces[index1];
			outBody1.m_linear = forceM1;
			outBody1.m_angular = torqueM1;
		}

		virtual void Execute()
		{
			D_TRACKTIME();
			ndScene* const scene = (ndScene*)GetThreadPool();
			ndWorld* const world = scene->GetWorld();
			ndDynamicsUpdate* const me = world->m_solver;

			m_leftHandSide = &me->GetLeftHandSide()[0];
			m_rightHandSide = &me->GetRightHandSide()[0];
			m_internalForces = &me->GetInternalForces()[0];
			m_jointPartialForces = &me->GetTempInternalForces()[0];
			m_jointBodyPairIndexBuffer = &me->GetJointBodyPairIndexBuffer()[0];
			ndArray<ndConstraint*>& jointArray = scene->GetActiveContactArray();

			const ndInt32 threadIndex = GetThreadId();
			const ndInt32 jointCount = jointArray.GetCount();
			const ndInt32 threadCount = GetThreadCount();

			for (ndInt32 i = threadIndex; i < jointCount; i += threadCount)
			{
				ndConstraint* const joint = jointArray[i];
				JointForce(joint, i);
			}
		}

		ndVector m_zero;
		ndJacobian* m_jointPartialForces;
		ndRightHandSide* m_rightHandSide;
		const ndJacobian* m_internalForces;
		const ndLeftHandSide* m_leftHandSide;
		const ndJointBodyPairIndex* m_jointBodyPairIndexBuffer;
	};

	class ndApplyJacobianAccumulatePartialForces : public ndThreadPoolJob_old
	{
		public:
		virtual void Execute()
		{
			D_TRACKTIME();
			const ndVector zero(ndVector::m_zero);
			ndScene* const scene = (ndScene*)GetThreadPool();
			ndWorld* const world = scene->GetWorld();
			ndDynamicsUpdate* const me = (ndDynamicsUpdate*)world->m_solver;

			ndJacobian* const internalForces = &me->GetInternalForces()[0];
			const ndInt32* const bodyIndex = &me->GetJointForceIndexBuffer()[0];
			const ndArray<ndBodyKinematic*>& bodyArray = scene->GetActiveBodyArray();
			const ndJacobian* const jointInternalForces = &me->GetTempInternalForces()[0];
			const ndJointBodyPairIndex* const jointBodyPairIndexBuffer = &me->GetJointBodyPairIndexBuffer()[0];

			const ndStartEnd startEnd(bodyArray.GetCount(), GetThreadId(), GetThreadCount());
			for (ndInt32 i = startEnd.m_start; i < startEnd.m_end; ++i)
			{
				ndVector force(zero);
				ndVector torque(zero);
				const ndBodyKinematic* const body = bodyArray[i];

				const ndInt32 startIndex = bodyIndex[i];
				const ndInt32 mask = body->m_isStatic - 1;
				const ndInt32 count = mask & (bodyIndex[i + 1] - startIndex);
				for (ndInt32 j = 0; j < count; ++j)
				{
					const ndInt32 index = jointBodyPairIndexBuffer[startIndex + j].m_joint;
					force += jointInternalForces[index].m_linear;
					torque += jointInternalForces[index].m_angular;
				}
				internalForces[i].m_linear = force;
				internalForces[i].m_angular = torque;
			}
		}
	};

	ndScene* const scene = m_world->GetScene();
	const ndInt32 passes = m_solverPasses;
	const ndInt32 threadsCount = scene->GetThreadCount();

	for (ndInt32 i = 0; i < passes; ++i)
	{
		scene->SubmitJobs<ndCalculateJointsForce>();
		scene->SubmitJobs<ndApplyJacobianAccumulatePartialForces>();
	}
}

void ndDynamicsUpdate::CalculateForces()
{
	D_TRACKTIME();
	if (m_world->GetScene()->GetActiveContactArray().GetCount())
	{
		m_firstPassCoef = ndFloat32(0.0f);
		if (m_world->m_skeletonList.GetCount())
		{
			InitSkeletons();
		}

		for (ndInt32 step = 0; step < 4; step++)
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
	}
}

void ndDynamicsUpdate::Update()
{
	D_TRACKTIME();
	m_timestep = m_world->GetScene()->GetTimestep();

	BuildIsland();
	if (GetIslands().GetCount())
	{
		IntegrateUnconstrainedBodies();
		InitWeights();
		InitBodyArray();
		InitJacobianMatrix();
		CalculateForces();
		IntegrateBodies();
		DetermineSleepStates();
	}
}
