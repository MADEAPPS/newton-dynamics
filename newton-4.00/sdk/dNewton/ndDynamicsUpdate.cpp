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
	,m_parallelSkeletons(0)
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
	class ndEvaluateKey0
	{
		public:
		ndEvaluateKey0(void* const)
		{
		}

		ndInt32 GetKey(const ndDynamicsUpdate::ndJointBodyPairIndex& entry) const
		{
			return ndInt32(entry.m_body & ((1 << D_MAX_BODY_RADIX_BIT) - 1));
		}
	};

	class ndEvaluateKey1
	{
		public:
		ndEvaluateKey1(void* const)
		{
		}

		ndInt32 GetKey(const ndDynamicsUpdate::ndJointBodyPairIndex& entry) const
		{
			ndInt32 key = ndInt32(entry.m_body >> D_MAX_BODY_RADIX_BIT);
			return key & ((1 << D_MAX_BODY_RADIX_BIT) - 1);
		}
	};

	ndScene* const scene = m_world->GetScene();
	ndArray<ndConstraint*>& jointArray = scene->GetActiveContactArray();
	ndArray<ndJointBodyPairIndex>& bodyJointPairs = GetJointBodyPairIndexBuffer();

	bodyJointPairs.SetCount(jointArray.GetCount() * 2);
	GetTempInternalForces().SetCount(jointArray.GetCount() * 2);

	auto EnumerateJointBodyPairs = ndMakeObject::ndFunction([this, &jointArray](ndInt32 groupId, ndInt32)
	{
		D_TRACKTIME_NAMED(EnumerateJointBodyPairs);
		ndJointBodyPairIndex* const jointBodyBuffer = &GetJointBodyPairIndexBuffer()[0];

		const ndInt32 index = groupId;
		const ndConstraint* const joint = jointArray[index];
		const ndBodyKinematic* const body0 = joint->GetBody0();
		const ndBodyKinematic* const body1 = joint->GetBody1();

		const ndInt32 m0 = body0->m_index;
		const ndInt32 m1 = body1->m_index;
		jointBodyBuffer[index * 2 + 0].m_body = m0;
		jointBodyBuffer[index * 2 + 0].m_joint = index * 2 + 0;
		jointBodyBuffer[index * 2 + 1].m_body = m1;
		jointBodyBuffer[index * 2 + 1].m_joint = index * 2 + 1;
	});
	const ndInt32 jointCount = ndInt32(jointArray.GetCount());
	scene->ParallelExecute(EnumerateJointBodyPairs, jointCount, scene->OptimalGroupBatch(jointCount));

	scene->GetScratchBuffer().SetCount(bodyJointPairs.GetCount() * ndInt32 (sizeof (ndJointBodyPairIndex)));
	ndJointBodyPairIndex* const tempBuffer = (ndJointBodyPairIndex*)&scene->GetScratchBuffer()[0];

	ndCountingSort<ndJointBodyPairIndex, ndEvaluateKey0, D_MAX_BODY_RADIX_BIT>(*scene, &bodyJointPairs[0], tempBuffer, ndInt32 (bodyJointPairs.GetCount()), nullptr, nullptr);
	ndCountingSort<ndJointBodyPairIndex, ndEvaluateKey1, D_MAX_BODY_RADIX_BIT>(*scene, tempBuffer, &bodyJointPairs[0], ndInt32 (bodyJointPairs.GetCount()), nullptr, nullptr);

#ifdef _DEBUG
	for (ndInt32 i = 1; i < bodyJointPairs.GetCount(); ++i)
	{
		ndInt32 key0 = bodyJointPairs[i - 1].m_body;
		ndInt32 key1 = bodyJointPairs[i + 0].m_body;
		ndAssert(key0 <= key1);
	}
#endif

	bodyJointPairs.SetCount(bodyJointPairs.GetCount() + 1);
	bodyJointPairs[bodyJointPairs.GetCount() - 1] = bodyJointPairs[bodyJointPairs.GetCount() - 2];

	ndArray<ndInt32>& bodyJointIndex = GetJointForceIndexBuffer();
	const ndInt32 bodyJointIndexCount = ndInt32 (scene->GetActiveBodyArray().GetCount()) + 1;
	bodyJointIndex.SetCount(bodyJointIndexCount);
	ndMemSet(&bodyJointIndex[0], 0, bodyJointIndexCount);

	for (ndInt32 i = 0; i < ndInt32 (jointArray.GetCount()); ++i)
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
			ndAssert(bodyIndex == i);
		}
	}
#endif
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
				ndAssert(rows > 0);
				m_upperBit = ndUnsigned32(sleep);
				m_lowerBit = ndUnsigned32((1 << 6) - rows - 1);
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
		ndInt32 GetKey(const ndConstraint* const joint) const
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

	ndInt32 jointCount = ndInt32 (jointArray.GetCount());
	jointArray.SetCount(jointCount + jointList.GetCount());

	for (ndJointList::ndNode* node = jointList.GetFirst(); node; node = node->GetNext())
	{
		ndJointBilateralConstraint* const joint = *node->GetInfo();
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
	ndMemSet(movingJoints, 0, threadCount);

	auto MarkFence0 = ndMakeObject::ndFunction([&jointArray](ndInt32 groupId, ndInt32)
	{
		D_TRACKTIME_NAMED(MarkFence0);
		ndConstraint* const joint = jointArray[groupId];
		ndBodyKinematic* const body0 = joint->GetBody0();
		ndBodyKinematic* const body1 = joint->GetBody1();
		const ndInt32 rows = ndInt32(joint->GetRowsCount());
		joint->m_rowCount = rows;

		const ndInt32 equilibrium = body0->m_equilibrium & body1->m_equilibrium;
		if (!equilibrium)
		{
			body0->m_isJointFence0 = 0;
			body1->m_isJointFence0 = body1->m_isStatic;
			ndAssert((body1->m_invMass.m_w == ndFloat32(0.0f)) == body1->m_isStatic);
		}

		body0->m_isConstrained = 1;
		body0->m_equilibrium0 = ndUnsigned8(body0->m_equilibrium0 & equilibrium);
		if (!body1->m_isStatic)
		{
			body1->m_isConstrained = 1;
			body1->m_equilibrium0 = ndUnsigned8(body1->m_equilibrium0 & equilibrium);
		}
	});
	
	auto MarkFence1 = ndMakeObject::ndFunction([&jointArray, &movingJoints](ndInt32 groupId, ndInt32 threadIndex)
	{
		D_TRACKTIME_NAMED(MarkFence1);
		ndConstraint* const joint = jointArray[groupId];
		ndBodyKinematic* const body0 = joint->GetBody0();
		ndBodyKinematic* const body1 = joint->GetBody1();
	
		const ndInt8 resting = body0->m_equilibrium0 & body1->m_equilibrium0;
		joint->m_resting = ndUnsigned8(resting);
	
		const ndInt32 solverSleep0 = body0->m_isJointFence0 & body1->m_isJointFence0;
		if (!solverSleep0)
		{
			body0->m_isJointFence1 = 0;
			body1->m_isJointFence1 = body1->m_isStatic;
			ndAssert((body1->m_invMass.m_w == ndFloat32(0.0f)) == body1->m_isStatic);
		}
		movingJoints[threadIndex] += (1 - resting);
	});
	
	scene->GetScratchBuffer().SetCount((jointArray.GetCount() + 32) * ndInt32 (sizeof (ndConstraint*)));
	ndConstraint** const tempJointBuffer = (ndConstraint**)&scene->GetScratchBuffer()[0];
	
	const ndInt32 numberOfGroups = ndInt32(jointArray.GetCount());
	const ndInt32 groupsBatchSize = scene->OptimalGroupBatch(numberOfGroups);
	scene->ParallelExecute(MarkFence0, numberOfGroups, groupsBatchSize);
	scene->ParallelExecute(MarkFence1, numberOfGroups, groupsBatchSize);

	ndInt32 movingJointCount = 0;
	for (ndInt32 i = 0; i < threadCount; ++i)
	{
		movingJointCount += movingJoints[i];
	}

	const ndInt32 minGroupSize = 1024;
	const ndInt32 itemsPerThreads = (numberOfGroups + threadCount - 1) / threadCount;
	const ndInt32 groupSize = (itemsPerThreads < minGroupSize) ? minGroupSize : itemsPerThreads;
	const ndInt32 numberOfWorkGroups = (numberOfGroups + groupSize - 1) / groupSize;
	ndAssert(numberOfWorkGroups <= threadCount);

	auto Scan0 = ndMakeObject::ndFunction([&jointArray, &histogram, scene, groupSize](ndInt32 groupId, ndInt32)
	{
		D_TRACKTIME_NAMED(Scan0);
		ndInt32* const hist = &histogram[groupId][0];
		ndAssert(scene->GetScratchBuffer().GetCount() >= ndInt32(jointArray.GetCount() * sizeof(ndConstraint*)));
		ndConstraint** const dstBuffer = (ndConstraint**)&scene->GetScratchBuffer()[0];

		hist[0] = 0;
		hist[1] = 0;

		const ndInt32 size = ndInt32(jointArray.GetCount());
		const ndInt32 start = groupId * groupSize;
		const ndInt32 count = ((start + groupSize) < size) ? groupSize : size - start;
		for (ndInt32 i = 0; i < count; ++i)
		{
			ndConstraint* const joint = jointArray[start + i];
			ndBodyKinematic* const body0 = joint->GetBody0();
			ndBodyKinematic* const body1 = joint->GetBody1();
			const ndInt32 key = body0->m_isJointFence1 & body1->m_isJointFence1;
			const ndInt32 entry = hist[key];
			dstBuffer[entry] = joint;
			hist[key] = entry + 1;
		}
	});
	scene->ParallelExecute(Scan0, numberOfWorkGroups, 1);

	ndInt32 scan[2];
	scan[0] = 0;
	scan[1] = 0;
	for (ndInt32 i = 0; i < numberOfWorkGroups; ++i)
	{
		scan[0] += histogram[i][0];
		scan[1] += histogram[i][1];
	}
	
	m_activeJointCount = scan[0];
	ndAssert(m_activeJointCount <= jointArray.GetCount());
	if (!m_activeJointCount)
	{
		jointArray.SetCount(0);
		return;
	}
	
	ndInt32 sum = 0;
	for (ndInt32 i = 0; i < 2; ++i)
	{
		for (ndInt32 j = 0; j < numberOfWorkGroups; ++j)
		{
			ndInt32 partialSum = histogram[j][i];
			histogram[j][i] = sum;
			sum += partialSum;
		}
	}

	auto Sort0 = ndMakeObject::ndFunction([&jointArray, &histogram, scene, groupSize](ndInt32 groupId, ndInt32)
	{
		D_TRACKTIME_NAMED(Sort0);
		ndInt32* const hist = &histogram[groupId][0];
		ndAssert(scene->GetScratchBuffer().GetCount() >= ndInt32(jointArray.GetCount() * sizeof(ndConstraint*)));
		ndConstraint** const dstBuffer = (ndConstraint**)&scene->GetScratchBuffer()[0];

		const ndInt32 size = ndInt32(jointArray.GetCount());
		const ndInt32 start = groupId * groupSize;
		const ndInt32 count = ((start + groupSize) < size) ? groupSize : size - start;
		for (ndInt32 i = 0; i < count; ++i)
		{
			ndConstraint* const joint = jointArray[start + i];
			ndBodyKinematic* const body0 = joint->GetBody0();
			ndBodyKinematic* const body1 = joint->GetBody1();
			const ndInt32 key = body0->m_isJointFence1 & body1->m_isJointFence1;
			const ndInt32 entry = hist[key];
			dstBuffer[entry] = joint;
			hist[key] = entry + 1;
		}
	});
	scene->ParallelExecute(Sort0, numberOfWorkGroups, 1);
	
	#ifdef _DEBUG
		for (ndInt32 i = 0; i < (jointArray.GetCount() - 1); ++i)
		{
			const ndConstraint* const joint0 = tempJointBuffer[i];
			const ndConstraint* const joint1 = tempJointBuffer[i + 1];
			const ndInt32 key0 = (joint0->GetBody0()->m_isJointFence1 & joint0->GetBody1()->m_isJointFence1) ? 1 : 0;
			const ndInt32 key1 = (joint1->GetBody0()->m_isJointFence1 & joint1->GetBody1()->m_isJointFence1) ? 1 : 0;
			ndAssert(key0 <= key1);
		}
	#endif
	
	ndAssert(m_activeJointCount <= jointArray.GetCount());
	jointArray.SetCount(m_activeJointCount);
	
	m_activeJointCount = movingJointCount;
	GetTempInternalForces().SetCount(jointArray.GetCount() * 2);
	GetJointBodyPairIndexBuffer().SetCount(jointArray.GetCount() * 2);
	ndCountingSort<ndConstraint*, ndEvaluateCountRows, 7>(*scene, tempJointBuffer, &jointArray[0], ndInt32 (jointArray.GetCount()), nullptr, nullptr);
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
	for (ndInt32 i = 0; i < ndInt32 (jointArray.GetCount()); ++i)
	{
		ndConstraint* const joint = jointArray[i];
		joint->m_rowStart = rowCount;
		rowCount += joint->m_rowCount;
	}

	m_leftHandSide.SetCount(rowCount);
	m_rightHandSide.SetCount(rowCount);

#ifdef _DEBUG
	ndAssert(m_activeJointCount <= jointArray.GetCount());
	for (ndInt32 i = 0; i < ndInt32 (jointArray.GetCount()); ++i)
	{
		ndConstraint* const joint = jointArray[i];
		ndAssert(joint->m_rowStart < m_leftHandSide.GetCount());
		ndAssert((joint->m_rowStart + joint->m_rowCount) <= rowCount);
	}

	for (ndInt32 i = 1; i < m_activeJointCount; ++i)
	{
		ndConstraint* const joint0 = jointArray[i - 1];
		ndConstraint* const joint1 = jointArray[i - 0];
		ndAssert(!joint0->m_resting);
		ndAssert(!joint1->m_resting);
		ndAssert(joint0->m_rowCount >= joint1->m_rowCount);
		ndAssert(!(joint0->GetBody0()->m_equilibrium0 & joint0->GetBody1()->m_equilibrium0));
		ndAssert(!(joint1->GetBody0()->m_equilibrium0 & joint1->GetBody1()->m_equilibrium0));
	}

	for (ndInt32 i = m_activeJointCount + 1; i < ndInt32 (jointArray.GetCount()); ++i)
	{
		ndConstraint* const joint0 = jointArray[i - 1];
		ndConstraint* const joint1 = jointArray[i - 0];
		ndAssert(joint0->m_resting);
		ndAssert(joint1->m_resting);
		ndAssert(joint0->m_rowCount >= joint1->m_rowCount);
		ndAssert(joint0->GetBody0()->m_equilibrium0 & joint0->GetBody1()->m_equilibrium0);
		ndAssert(joint1->GetBody0()->m_equilibrium0 & joint1->GetBody1()->m_equilibrium0);
	}
#endif

	SortBodyJointScan();
}

void ndDynamicsUpdate::SortIslands()
{
	D_TRACKTIME();
	ndScene* const scene = m_world->GetScene();
	const ndArray<ndBodyKinematic*>& bodyArray = scene->GetActiveBodyArray();
	ndArray<ndBodyKinematic*>& activeBodyArray = GetBodyIslandOrder();
	GetInternalForces().SetCount(bodyArray.GetCount());
	activeBodyArray.SetCount(bodyArray.GetCount());

	const ndInt32 minGroupSize = 1024;
	const ndInt32 threadCount = scene->GetThreadCount();
	const ndInt32 numberOfGroups = ndInt32(bodyArray.GetCount());
	const ndInt32 itemsPerThreads = (numberOfGroups + threadCount - 1) / threadCount;
	const ndInt32 groupSize = (itemsPerThreads < minGroupSize) ? minGroupSize : itemsPerThreads;
	const ndInt32 numberOfWorkGroups = (numberOfGroups + groupSize - 1) / groupSize;
	ndAssert(numberOfWorkGroups <= threadCount);

	ndInt32 histogram[D_MAX_THREADS_COUNT][3];
	auto Scan0 = ndMakeObject::ndFunction([&bodyArray, &histogram, groupSize](ndInt32 groupId, ndInt32)
	{
		D_TRACKTIME_NAMED(Scan0);
		ndInt32* const hist = &histogram[groupId][0];
		hist[0] = 0;
		hist[1] = 0;
		hist[2] = 0;

		ndInt32 map[4];
		map[0] = 0;
		map[1] = 1;
		map[2] = 2;
		map[3] = 2;

		const ndInt32 size = ndInt32(bodyArray.GetCount());
		const ndInt32 start = groupId * groupSize;
		const ndInt32 count = ((start + groupSize) < size) ? groupSize : size - start;
		for (ndInt32 i = 0; i < count; ++i)
		{
			ndBodyKinematic* const body = bodyArray[start + i];
			ndInt32 key = map[body->m_equilibrium0 * 2 + 1 - body->m_isConstrained];
			ndAssert(key < 3);
			hist[key] = hist[key] + 1;
		}
	});
	scene->ParallelExecute(Scan0, numberOfWorkGroups, 1);

	ndInt32 scan[3];
	scan[0] = 0;
	scan[1] = 0;
	scan[2] = 0;

	ndInt32 sum = 0;
	for (ndInt32 i = 0; i < 3; ++i)
	{
		for (ndInt32 j = 0; j < numberOfWorkGroups; ++j)
		{
			ndInt32 partialSum = histogram[j][i];
			histogram[j][i] = sum;
			sum += partialSum;
		}
		scan[i] = sum;
	}

	auto Sort0 = ndMakeObject::ndFunction([&bodyArray, &activeBodyArray, &histogram, groupSize](ndInt32 groupId, ndInt32)
	{
		D_TRACKTIME_NAMED(Sort0);
		ndInt32* const hist = &histogram[groupId][0];
		ndInt32 map[4];
		map[0] = 0;
		map[1] = 1;
		map[2] = 2;
		map[3] = 2;

		const ndInt32 size = ndInt32(bodyArray.GetCount());
		const ndInt32 start = groupId * groupSize;
		const ndInt32 count = ((start + groupSize) < size) ? groupSize : size - start;
		for (ndInt32 i = 0; i < count; ++i)
		{
			ndBodyKinematic* const body = bodyArray[start + i];
			ndInt32 key = map[body->m_equilibrium0 * 2 + 1 - body->m_isConstrained];
			ndAssert(key < 3);
			const ndInt32 entry = hist[key];
			activeBodyArray[entry] = body;
			hist[key] = entry + 1;
		}
	});
	scene->ParallelExecute(Sort0, numberOfWorkGroups, 1);

	activeBodyArray.SetCount(scan[1]);
	m_unConstrainedBodyCount = scan[1] - scan[0];
}

void ndDynamicsUpdate::BuildIsland()
{
	m_unConstrainedBodyCount = 0;
	GetBodyIslandOrder().SetCount(0);
	ndScene* const scene = m_world->GetScene();
	const ndArray<ndBodyKinematic*>& bodyArray = scene->GetActiveBodyArray();
	ndAssert(bodyArray.GetCount() >= 1);
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

	auto IntegrateUnconstrainedBodies = ndMakeObject::ndFunction([this, &scene](ndInt32 groupId, ndInt32)
	{
		D_TRACKTIME_NAMED(IntegrateUnconstrainedBodies);
		ndArray<ndBodyKinematic*>& bodyArray = GetBodyIslandOrder();

		const ndFloat32 timestep = scene->GetTimestep();
		const ndInt32 base = ndInt32 (bodyArray.GetCount() - GetUnconstrainedBodyCount());

		ndBodyKinematic* const body = bodyArray[base + groupId];
		ndAssert(body);
		body->UpdateInvInertiaMatrix();
		body->AddDampingAcceleration(timestep);
		body->IntegrateExternalForce(timestep);
	});

	if (GetUnconstrainedBodyCount())
	{
		D_TRACKTIME();
		const ndInt32 count = GetUnconstrainedBodyCount();
		scene->ParallelExecute(IntegrateUnconstrainedBodies, count, scene->OptimalGroupBatch(count));
	}
}

void ndDynamicsUpdate::InitWeights()
{
	D_TRACKTIME();
	ndScene* const scene = m_world->GetScene();
	m_invTimestep = ndFloat32(1.0f) / m_timestep;
	m_invStepRK = ndFloat32(0.25f);
	m_timestepRK = m_timestep * m_invStepRK;
	m_invTimestepRK = m_invTimestep * ndFloat32(4.0f);

	const ndArray<ndBodyKinematic*>& bodyArray = scene->GetActiveBodyArray();
	const ndInt32 bodyCount = ndInt32 (bodyArray.GetCount());
	GetInternalForces().SetCount(bodyCount);

	ndInt32 extraPassesArray[D_MAX_THREADS_COUNT];
	ndMemSet(extraPassesArray, 1, D_MAX_THREADS_COUNT);

	auto InitWeights = ndMakeObject::ndFunction([this, &bodyArray, &extraPassesArray](ndInt32 groupId, ndInt32 threadIndex)
	{
		D_TRACKTIME_NAMED(InitWeights);
		const ndArray<ndInt32>& jointForceIndexBuffer = GetJointForceIndexBuffer();
		const ndArray<ndJointBodyPairIndex>& jointBodyPairIndex = GetJointBodyPairIndexBuffer();

		const ndInt32 index = jointForceIndexBuffer[groupId];
		const ndJointBodyPairIndex& scan = jointBodyPairIndex[index];
		ndBodyKinematic* const body = bodyArray[scan.m_body];
		ndAssert(body->m_index == scan.m_body);
		ndAssert(body->m_isConstrained <= 1);
		const ndInt32 count = jointForceIndexBuffer[groupId + 1] - index - 1;
		const ndInt32 mask = -ndInt32(body->m_isConstrained & ~body->m_isStatic);
		const ndInt32 weigh = 1 + (mask & count);
		ndAssert(weigh >= 0);
		if (weigh)
		{
			body->m_weigh = ndFloat32(weigh);
		}
		extraPassesArray[threadIndex] = ndMax(extraPassesArray[threadIndex], weigh);
	});

	if (scene->GetActiveContactArray().GetCount())
	{
		const ndArray<ndInt32>& jointForceIndexBuffer = GetJointForceIndexBuffer();
		const ndInt32 jointCount = ndInt32(jointForceIndexBuffer.GetCount()) - 1;
		scene->ParallelExecute(InitWeights, jointCount, scene->OptimalGroupBatch(jointCount));

		ndInt32 extraPasses = 0;
		const ndInt32 threadCount = scene->GetThreadCount();
		for (ndInt32 i = 0; i < threadCount; ++i)
		{
			extraPasses = ndMax(extraPasses, extraPassesArray[i]);
		}

		const ndInt32 conectivity = 7;
		m_solverPasses = ndUnsigned32(m_world->GetSolverIterations() + 2 * extraPasses / conectivity + 2);
	}
}

void ndDynamicsUpdate::InitBodyArray()
{
	D_TRACKTIME();

	ndScene* const scene = m_world->GetScene();
	const ndFloat32 timestep = scene->GetTimestep();

	auto InitBodyArray = ndMakeObject::ndFunction([this, timestep](ndInt32 groupId, ndInt32)
	{
		D_TRACKTIME_NAMED(InitBodyArray);
		const ndArray<ndBodyKinematic*>& bodyArray = GetBodyIslandOrder();

		ndBodyKinematic* const body = bodyArray[groupId];
		ndAssert(body);
		ndAssert(body->m_isConstrained | body->m_isStatic);

		body->UpdateInvInertiaMatrix();
		body->AddDampingAcceleration(timestep);
		const ndVector angularMomentum(body->CalculateAngularMomentum());
		body->m_gyroTorque = body->m_omega.CrossProduct(angularMomentum);
		body->m_gyroAlpha = body->m_invWorldInertiaMatrix.RotateVector(body->m_gyroTorque);

		body->m_accel = body->m_veloc;
		body->m_alpha = body->m_omega;
		body->m_gyroRotation = body->m_rotation;
	});
	const ndArray<ndBodyKinematic*>& bodyArray = GetBodyIslandOrder();
	const ndInt32 count = ndInt32(bodyArray.GetCount() - GetUnconstrainedBodyCount());
	scene->ParallelExecute(InitBodyArray, count, scene->OptimalGroupBatch(count));
}

void ndDynamicsUpdate::GetJacobianDerivatives(ndConstraint* const joint)
{
	ndConstraintDescritor constraintParam;
	ndAssert(joint->GetRowsCount() <= D_CONSTRAINT_MAX_ROWS);
	for (ndInt32 i = ndInt32(joint->GetRowsCount() - 1); i >= 0; i--)
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
		if (!bilareral->m_isInSkeleton && (bilareral->GetSolverModel() == m_jointkinematicAttachment))
		{
			ndSkeletonContainer* const skeleton0 = bilareral->m_body0->GetSkeleton();
			ndSkeletonContainer* const skeleton1 = bilareral->m_body1->GetSkeleton();
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
	const ndInt32 baseIndex = joint->m_rowStart;
	for (ndInt32 i = 0; i < dof; ++i)
	{
		ndAssert(constraintParam.m_forceBounds[i].m_jointForce);

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

		const ndInt32 frictionIndex = constraintParam.m_forceBounds[i].m_normalIndex;
		const ndInt32 mask = frictionIndex >> 31;
		rhs->m_normalForceIndex = frictionIndex;
		rhs->m_normalForceIndexFlat = ~mask & (frictionIndex + baseIndex);

		rhs->SetSanityCheck(joint);
		ndAssert(rhs->SanityCheck());
	}
}

void ndDynamicsUpdate::InitJacobianMatrix()
{
	ndScene* const scene = m_world->GetScene();
	ndBodyKinematic** const bodyArray = &scene->GetActiveBodyArray()[0];
	ndArray<ndConstraint*>& jointArray = scene->GetActiveContactArray();

	auto GetJacobianMatrix = ndMakeObject::ndFunction([this, &jointArray](ndInt32 groupId, ndInt32)
	{
		ndConstraint* const joint = jointArray[groupId];
		GetJacobianDerivatives(joint);
	});

	auto InitJacobianMatrix = ndMakeObject::ndFunction([this, &jointArray](ndInt32 groupId, ndInt32)
	{
		D_TRACKTIME_NAMED(InitJacobianMatrix);
		ndJacobian* const internalForces = &GetTempInternalForces()[0];
		auto BuildJacobianMatrix = [this, &internalForces](ndConstraint* const joint, ndInt32 jointIndex)
		{
			ndAssert(joint->GetBody0());
			ndAssert(joint->GetBody1());
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

			const ndVector zero(ndVector::m_zero);
			ndVector forceAcc0(zero);
			ndVector torqueAcc0(zero);
			ndVector forceAcc1(zero);
			ndVector torqueAcc1(zero);

			const ndVector weigh0(body0->m_weigh);
			const ndVector weigh1(body1->m_weigh);

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
				ndAssert(rhs->m_jointFeebackForce);
				const ndFloat32 force = rhs->m_jointFeebackForce->GetInitialGuess();

				rhs->m_force = isBilateral ? ndClamp(force, rhs->m_lowerBoundFrictionCoefficent, rhs->m_upperBoundFrictionCoefficent) : force;
				rhs->m_maxImpact = ndFloat32(0.0f);
				ndAssert(rhs->SanityCheck());

				const ndJacobian& JtM0 = row->m_Jt.m_jacobianM0;
				const ndJacobian& JtM1 = row->m_Jt.m_jacobianM1;
				const ndVector tmpDiag(
					weigh0 * (JMinvM0.m_linear * JtM0.m_linear + JMinvM0.m_angular * JtM0.m_angular) +
					weigh1 * (JMinvM1.m_linear * JtM1.m_linear + JMinvM1.m_angular * JtM1.m_angular));

				ndFloat32 diag = tmpDiag.AddHorizontal().GetScalar();
				ndAssert(diag > ndFloat32(0.0f));
				rhs->m_diagDamp = diag * rhs->m_diagonalRegularizer;

				diag *= (ndFloat32(1.0f) + rhs->m_diagonalRegularizer);
				rhs->m_JinvMJt = diag;
				rhs->m_invJinvMJt = ndFloat32(1.0f) / diag;

				const ndVector f(rhs->m_force);
				forceAcc0 = forceAcc0 + JtM0.m_linear * f;
				torqueAcc0 = torqueAcc0 + JtM0.m_angular * f;
				forceAcc1 = forceAcc1 + JtM1.m_linear * f;
				torqueAcc1 = torqueAcc1 + JtM1.m_angular * f;
			}

			const ndInt32 index0 = jointIndex * 2 + 0;
			ndJacobian& outBody0 = internalForces[index0];
			outBody0.m_linear = forceAcc0;
			outBody0.m_angular = torqueAcc0;

			const ndInt32 index1 = jointIndex * 2 + 1;
			ndJacobian& outBody1 = internalForces[index1];
			outBody1.m_linear = forceAcc1;
			outBody1.m_angular = torqueAcc1;
		};

		ndConstraint* const joint = jointArray[groupId];
		BuildJacobianMatrix(joint, groupId);
	});

	auto InitJacobianAccumulatePartialForces = ndMakeObject::ndFunction([this, &bodyArray](ndInt32 groupId, ndInt32)
	{
		D_TRACKTIME_NAMED(InitJacobianAccumulatePartialForces);
		const ndVector zero(ndVector::m_zero);
		ndJacobian* const internalForces = &GetInternalForces()[0];
		const ndArray<ndInt32>& bodyIndex = GetJointForceIndexBuffer();

		const ndJacobian* const jointInternalForces = &GetTempInternalForces()[0];
		const ndJointBodyPairIndex* const jointBodyPairIndexBuffer = &GetJointBodyPairIndexBuffer()[0];

		ndVector force(zero);
		ndVector torque(zero);

		const ndInt32 index = bodyIndex[groupId];
		const ndJointBodyPairIndex& scan = jointBodyPairIndexBuffer[index];
		ndBodyKinematic* const body = bodyArray[scan.m_body];

		ndAssert(body->m_isStatic <= 1);
		ndAssert(body->m_index == scan.m_body);
		const ndInt32 mask = ndInt32(body->m_isStatic) - 1;
		const ndInt32 count = mask & (bodyIndex[groupId + 1] - index);

		for (ndInt32 k = 0; k < count; ++k)
		{
			const ndInt32 jointIndex = jointBodyPairIndexBuffer[index + k].m_joint;
			force += jointInternalForces[jointIndex].m_linear;
			torque += jointInternalForces[jointIndex].m_angular;
		}
		internalForces[groupId].m_linear = force;
		internalForces[groupId].m_angular = torque;
	});

	if (scene->GetActiveContactArray().GetCount())
	{
		D_TRACKTIME();
		m_rightHandSide[0].m_force = ndFloat32(1.0f);

		const ndInt32 numberOfJoints = ndInt32(jointArray.GetCount());
		const ndInt32 numberOfJointsBratches = scene->OptimalGroupBatch(numberOfJoints);
		scene->ParallelExecute(GetJacobianMatrix, numberOfJoints, numberOfJointsBratches);
		scene->ParallelExecute(InitJacobianMatrix, numberOfJoints, numberOfJointsBratches);

		const ndInt32 bodyCount = ndInt32(GetJointForceIndexBuffer().GetCount()) - 1;
		scene->ParallelExecute(InitJacobianAccumulatePartialForces, bodyCount, scene->OptimalGroupBatch(bodyCount));
	}
}

void ndDynamicsUpdate::CalculateJointsAcceleration()
{
	D_TRACKTIME();
	ndScene* const scene = m_world->GetScene();
	const ndArray<ndConstraint*>& jointArray = scene->GetActiveContactArray();

	auto CalculateJointsAcceleration = ndMakeObject::ndFunction([this, &jointArray](ndInt32 groupId, ndInt32)
	{
		D_TRACKTIME_NAMED(CalculateJointsAcceleration);
		ndJointAccelerationDecriptor joindDesc;
		joindDesc.m_timestep = m_timestepRK;
		joindDesc.m_invTimestep = m_invTimestepRK;
		joindDesc.m_firstPassCoefFlag = m_firstPassCoef;
		ndArray<ndLeftHandSide>& leftHandSide = m_leftHandSide;
		ndArray<ndRightHandSide>& rightHandSide = m_rightHandSide;

		ndConstraint* const joint = jointArray[groupId];
		const ndInt32 pairStart = joint->m_rowStart;
		joindDesc.m_rowsCount = joint->m_rowCount;
		joindDesc.m_leftHandSide = &leftHandSide[pairStart];
		joindDesc.m_rightHandSide = &rightHandSide[pairStart];
		joint->JointAccelerations(&joindDesc);
	});

	const ndInt32 count = ndInt32(jointArray.GetCount());
	scene->ParallelExecute(CalculateJointsAcceleration, count, scene->OptimalGroupBatch(count));
	m_firstPassCoef = ndFloat32(1.0f);
}

void ndDynamicsUpdate::IntegrateBodiesVelocity()
{
	D_TRACKTIME();
	ndScene* const scene = m_world->GetScene();

	auto IntegrateBodiesVelocity = ndMakeObject::ndFunction([this](ndInt32 groupId, ndInt32)
	{
		D_TRACKTIME_NAMED(IntegrateBodiesVelocity);
		ndArray<ndBodyKinematic*>& bodyArray = GetBodyIslandOrder();
		const ndArray<ndJacobian>& internalForces = GetInternalForces();

		const ndVector timestep4(GetTimestepRK());
		const ndVector speedFreeze2(m_world->m_freezeSpeed2 * ndFloat32(0.1f));

		ndBodyKinematic* const body = bodyArray[groupId];

		ndAssert(body);
		ndAssert(body->m_isConstrained);
		// no necessary anymore because the virtual function handle it.
		//ndAssert(body->GetAsBodyDynamic()); 

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
			body->m_equilibrium0 = ndUnsigned8(equilibrium);
		}
		ndAssert(body->m_veloc.m_w == ndFloat32(0.0f));
		ndAssert(body->m_omega.m_w == ndFloat32(0.0f));
	});

	const ndInt32 count = ndInt32(GetBodyIslandOrder().GetCount() - GetUnconstrainedBodyCount());
	scene->ParallelExecute(IntegrateBodiesVelocity, count, scene->OptimalGroupBatch(count));
}

void ndDynamicsUpdate::UpdateForceFeedback()
{
	D_TRACKTIME();
	ndScene* const scene = m_world->GetScene();
	const ndArray<ndConstraint*>& jointArray = scene->GetActiveContactArray();

	auto UpdateForceFeedback = ndMakeObject::ndFunction([this, &jointArray](ndInt32 groupId, ndInt32)
	{
		D_TRACKTIME_NAMED(UpdateForceFeedback);
		ndArray<ndRightHandSide>& rightHandSide = m_rightHandSide;
		const ndArray<ndLeftHandSide>& leftHandSide = m_leftHandSide;

		const ndVector zero(ndVector::m_zero);
		const ndFloat32 timestepRK = GetTimestepRK();

		ndConstraint* const joint = jointArray[groupId];
		const ndInt32 rows = joint->m_rowCount;
		const ndInt32 first = joint->m_rowStart;

		ndVector force0(zero);
		ndVector force1(zero);
		ndVector torque0(zero);
		ndVector torque1(zero);

		for (ndInt32 k = 0; k < rows; ++k)
		{
			const ndLeftHandSide* const lhs = &leftHandSide[k + first];
			const ndRightHandSide* const rhs = &rightHandSide[k + first];
			ndAssert(ndCheckFloat(rhs->m_force));
			rhs->m_jointFeebackForce->Push(rhs->m_force);
			rhs->m_jointFeebackForce->m_force = rhs->m_force;
			rhs->m_jointFeebackForce->m_impact = rhs->m_maxImpact * timestepRK;

			const ndVector f(rhs->m_force);
			force0 += lhs->m_Jt.m_jacobianM0.m_linear * f;
			torque0 += lhs->m_Jt.m_jacobianM0.m_angular * f;
			force1 += lhs->m_Jt.m_jacobianM1.m_linear * f;
			torque1 += lhs->m_Jt.m_jacobianM1.m_angular * f;
		}
		joint->m_forceBody0 = force0;
		joint->m_torqueBody0 = torque0;
		joint->m_forceBody1 = force1;
		joint->m_torqueBody1 = torque1;
		joint->UpdateParameters();
	});

	const ndInt32 count = ndInt32(jointArray.GetCount());
	scene->ParallelExecute(UpdateForceFeedback, count, scene->OptimalGroupBatch(count));
}

void ndDynamicsUpdate::IntegrateBodies()
{
	D_TRACKTIME();
	ndScene* const scene = m_world->GetScene();
	const ndVector invTime(m_invTimestep);
	const ndFloat32 timestep = scene->GetTimestep();

	auto IntegrateBodies = ndMakeObject::ndFunction([this, timestep, invTime](ndInt32 groupId, ndInt32)
	{
		D_TRACKTIME_NAMED(IntegrateBodies);
		const ndWorld* const world = m_world;
		const ndArray<ndBodyKinematic*>& bodyArray = GetBodyIslandOrder();

		const ndFloat32 speedFreeze2 = world->m_freezeSpeed2;
		const ndFloat32 accelFreeze2 = world->m_freezeAccel2;

		ndBodyKinematic* const body = bodyArray[groupId];
		if (!body->m_equilibrium)
		{
			body->SetAcceleration(invTime * (body->m_veloc - body->m_accel), invTime * (body->m_omega - body->m_alpha));
			body->IntegrateVelocity(timestep);
		}
		body->EvaluateSleepState(speedFreeze2, accelFreeze2);
	});
	const ndArray<ndBodyKinematic*>& bodyArray = GetBodyIslandOrder();
	const ndInt32 count = ndInt32(bodyArray.GetCount());
	scene->ParallelExecute(IntegrateBodies, count, scene->OptimalGroupBatch(count));
}

void ndDynamicsUpdate::DetermineSleepStates()
{
	D_TRACKTIME();

	auto CalculateSleepState = ndMakeObject::ndFunction([this](ndInt32 groupId, ndInt32)
	{
		D_TRACKTIME_NAMED(CalculateSleepState);
		ndScene* const scene = m_world->GetScene();
		const ndArray<ndInt32>& bodyIndex = GetJointForceIndexBuffer();
		const ndJointBodyPairIndex* const jointBodyPairIndexBuffer = &GetJointBodyPairIndexBuffer()[0];
		ndConstraint** const jointArray = &scene->GetActiveContactArray()[0];
		ndBodyKinematic** const bodyArray = &scene->GetActiveBodyArray()[0];

		const ndVector zero(ndVector::m_zero);
		const ndInt32 m = groupId;
		const ndInt32 index = bodyIndex[m];
		ndBodyKinematic* const body = bodyArray[jointBodyPairIndexBuffer[index].m_body];
		ndAssert(body->m_isStatic <= 1);
		ndAssert(body->m_index == jointBodyPairIndexBuffer[index].m_body);
		const ndInt32 mask = ndInt32(body->m_isStatic) - 1;
		const ndInt32 count = mask & (bodyIndex[m + 1] - index);
		if (count)
		{
			ndUnsigned8 equilibrium = body->m_isJointFence0;
			if (equilibrium & body->m_autoSleep)
			{
				for (ndInt32 k = 0; k < count; ++k)
				{
					const ndJointBodyPairIndex& scan = jointBodyPairIndexBuffer[index + k];
					ndConstraint* const joint = jointArray[scan.m_joint >> 1];
					ndBodyKinematic* const body1 = (joint->GetBody0() == body) ? joint->GetBody1() : joint->GetBody0();
					ndAssert(body1 != body);
					equilibrium = ndUnsigned8(equilibrium & body1->m_isJointFence0);
				}
			}
			body->m_equilibrium = ndUnsigned8(equilibrium & body->m_autoSleep);
			if (body->m_equilibrium)
			{
				body->m_veloc = zero;
				body->m_omega = zero;
			}
		}
	});

	ndScene* const scene = m_world->GetScene();
	if (scene->GetActiveContactArray().GetCount())
	{
		const ndArray<ndInt32>& bodyIndex = GetJointForceIndexBuffer();
		const ndInt32 bodyCount = ndInt32(bodyIndex.GetCount()) - 1;
		scene->ParallelExecute(CalculateSleepState, bodyCount, scene->OptimalGroupBatch(bodyCount));
	}
}

void ndDynamicsUpdate::InitSkeletons()
{
	D_TRACKTIME();
	ndScene* const scene = m_world->GetScene();
	const ndArray<ndSkeletonContainer*>& activeSkeletons = m_world->m_activeSkeletons;

	if (activeSkeletons.GetCount())
	{
		for (ndInt32 i = ndInt32(activeSkeletons.GetCount()) - 1; i >= 0; --i)
		{
			ndSkeletonContainer* const skeleton = activeSkeletons[i];
			if (skeleton->m_transientLoopingContacts.GetCount())
			{
				skeleton->AddExtraContacts();
			}
		}

		m_parallelSkeletons = 0;
		ndInt32 threadCount = scene->GetThreadCount();
		if (threadCount > 1)
		{
			for (ndInt32 i = 0; i < activeSkeletons.GetCount(); ++i)
			{
				ndInt32 jointCount = activeSkeletons[i]->GetNodeList().GetCount();
				if (jointCount > D_NUMBER_OF_PARALLER_SKELETON_JOINTS)
				{
					activeSkeletons[i]->InitMassMatrix(scene, &m_leftHandSide[0], &m_rightHandSide[0]);
					m_parallelSkeletons++;
				}
			}
			if (m_parallelSkeletons > threadCount)
			{
				m_parallelSkeletons = 0;
			}
		}

		auto InitSkeletons = ndMakeObject::ndFunction([this, &activeSkeletons](ndInt32 groupId, ndInt32)
		{
			D_TRACKTIME_NAMED(InitSkeletons);
			ndArray<ndRightHandSide>& rightHandSide = m_rightHandSide;
			const ndArray<ndLeftHandSide>& leftHandSide = m_leftHandSide;

			ndSkeletonContainer* const skeleton = activeSkeletons[m_parallelSkeletons + groupId];
			skeleton->InitMassMatrix(nullptr, &leftHandSide[0], &rightHandSide[0]);
		});

		const ndInt32 count = ndInt32(activeSkeletons.GetCount()) - m_parallelSkeletons;
		if (count)
		{
			scene->ParallelExecute(InitSkeletons, count, 1);
		}
	}
}

void ndDynamicsUpdate::UpdateSkeletons()
{
	D_TRACKTIME();
	ndScene* const scene = m_world->GetScene();
	const ndArray<ndSkeletonContainer*>& activeSkeletons = m_world->m_activeSkeletons;

	auto UpdateSkeletons = ndMakeObject::ndFunction([this, &activeSkeletons](ndInt32 groupId, ndInt32)
	{
		D_TRACKTIME_NAMED(UpdateSkeletons);
		ndJacobian* const internalForces = &GetInternalForces()[0];
	
		ndSkeletonContainer* const skeleton = activeSkeletons[m_parallelSkeletons + groupId];
		skeleton->CalculateReactionForces(nullptr, internalForces);
	});

	if (scene->GetThreadCount() > 1)
	{
		ndAssert(m_parallelSkeletons <= activeSkeletons.GetCount());
		ndJacobian* const internalForces = &GetInternalForces()[0];
		for (ndInt32 i = 0; i < m_parallelSkeletons; ++i)
		{
			activeSkeletons[i]->CalculateReactionForces(scene, internalForces);
		}
	}

	const ndInt32 count = ndInt32(activeSkeletons.GetCount()) - m_parallelSkeletons;
	if (count)
	{
		scene->ParallelExecute(UpdateSkeletons, count, 2);
	}
}

void ndDynamicsUpdate::CalculateJointsForce()
{
	D_TRACKTIME();
	const ndUnsigned32 passes = m_solverPasses;
	ndScene* const scene = m_world->GetScene();

	ndArray<ndBodyKinematic*>& bodyArray = scene->GetActiveBodyArray();
	ndArray<ndConstraint*>& jointArray = scene->GetActiveContactArray();

	auto CalculateJointsForce = ndMakeObject::ndFunction([this, &jointArray](ndInt32 groupId, ndInt32)
	{
		D_TRACKTIME_NAMED(CalculateJointsForce);
		ndJacobian* const jointPartialForces = &GetTempInternalForces()[0];

		auto JointForce = [this, &jointPartialForces](ndConstraint* const joint, ndInt32 jointIndex)
		{
			D_TRACKTIME_NAMED(JointForce);
			const ndVector zero(ndVector::m_zero);
			ndBodyKinematic* const body0 = joint->GetBody0();
			ndBodyKinematic* const body1 = joint->GetBody1();
			ndAssert(body0);
			ndAssert(body1);

			const ndInt32 m0 = body0->m_index;
			const ndInt32 m1 = body1->m_index;
			const ndInt32 rowStart = joint->m_rowStart;
			const ndInt32 rowsCount = joint->m_rowCount;

			const ndInt32 resting = body0->m_equilibrium0 & body1->m_equilibrium0;
			if (!resting)
			{
				ndFixSizeArray<ndFloat32, D_CONSTRAINT_MAX_ROWS> diagDamp(rowsCount);
				ndFixSizeArray<ndFloat32, D_CONSTRAINT_MAX_ROWS> JinvMJt(rowsCount);
				ndFixSizeArray<ndFloat32, D_CONSTRAINT_MAX_ROWS> invJinvMJt(rowsCount);
				ndFixSizeArray<ndFloat32, D_CONSTRAINT_MAX_ROWS + 1> force(rowsCount + 1);
				ndFixSizeArray<ndFloat32, D_CONSTRAINT_MAX_ROWS> coordenateAccel(rowsCount);
				ndFixSizeArray<ndFloat32, D_CONSTRAINT_MAX_ROWS> lowerBoundFrictionCoefficent(rowsCount);
				ndFixSizeArray<ndFloat32, D_CONSTRAINT_MAX_ROWS> upperBoundFrictionCoefficent(rowsCount);
				ndFixSizeArray<ndInt32, D_CONSTRAINT_MAX_ROWS> normalForceIndexFlat(rowsCount);
				for (ndInt32 i = 0; i < rowsCount; ++i)
				{
					const ndRightHandSide* const rhs = &m_rightHandSide[rowStart + i];
					ndAssert(rhs->SanityCheck());

					force[i] = rhs->m_force;
					diagDamp[i] = rhs->m_diagDamp;
					JinvMJt[i] = rhs->m_JinvMJt;
					invJinvMJt[i] = rhs->m_invJinvMJt;
					coordenateAccel[i] = rhs->m_coordenateAccel;
					lowerBoundFrictionCoefficent[i] = rhs->m_lowerBoundFrictionCoefficent;
					upperBoundFrictionCoefficent[i] = rhs->m_upperBoundFrictionCoefficent;
					normalForceIndexFlat[i] = rhs->m_normalForceIndexFlat ? rhs->m_normalForceIndexFlat - rowStart : rowsCount;
				}
				force[rowsCount] = ndFloat32(1.0f);

				ndVector forceM0(m_internalForces[m0].m_linear);
				ndVector torqueM0(m_internalForces[m0].m_angular);
				ndVector forceM1(m_internalForces[m1].m_linear);
				ndVector torqueM1(m_internalForces[m1].m_angular);

				const ndFloat32 weight0 = body0->m_weigh;
				const ndFloat32 weight1 = body1->m_weigh;

				const ndFloat32 tol = ndFloat32(0.125f);
				const ndFloat32 tol2 = tol * tol;

				ndFloat32 accNorm = ndFloat32(10.0f);
				for (ndInt32 k = 0; (k < 4) && (accNorm > tol2); ++k)
				{
					accNorm = ndFloat32(0.0f);
					for (ndInt32 i = 0; i < rowsCount; ++i)
					{
						const ndLeftHandSide* const lhs = &m_leftHandSide[rowStart + i];
						const ndFloat32 f0 = force[i];

						ndVector accel(lhs->m_JMinv.m_jacobianM0.m_linear * forceM0);
						accel = accel.MulAdd(lhs->m_JMinv.m_jacobianM0.m_angular, torqueM0);
						accel = accel.MulAdd(lhs->m_JMinv.m_jacobianM1.m_linear, forceM1);
						accel = accel.MulAdd(lhs->m_JMinv.m_jacobianM1.m_angular, torqueM1);
						const ndFloat32 a = coordenateAccel[i] - f0 * diagDamp[i] - accel.AddHorizontal().GetScalar();

						ndAssert(normalForceIndexFlat[i] >= 0);
						const ndInt32 frictionIndex = normalForceIndexFlat[i];
						const ndFloat32 frictionNormal = force[frictionIndex];
						const ndFloat32 lowerFrictionForce = frictionNormal * lowerBoundFrictionCoefficent[i];
						const ndFloat32 upperFrictionForce = frictionNormal * upperBoundFrictionCoefficent[i];

						const ndFloat32 f1 = ndClamp(f0 + a * invJinvMJt[i], lowerFrictionForce, upperFrictionForce);
						//f1 = ndClamp(f1, lowerFrictionForce, upperFrictionForce);
						force[i] = f1;

						const ndFloat32 deltaForce = f1 - f0;
						ndFloat32 residual = deltaForce * JinvMJt[i];
						accNorm += residual * residual;

						const ndVector deltaForce0(deltaForce * weight0);
						const ndVector deltaForce1(deltaForce * weight1);
						forceM0 = forceM0.MulAdd(lhs->m_Jt.m_jacobianM0.m_linear, deltaForce0);
						torqueM0 = torqueM0.MulAdd(lhs->m_Jt.m_jacobianM0.m_angular, deltaForce0);
						forceM1 = forceM1.MulAdd(lhs->m_Jt.m_jacobianM1.m_linear, deltaForce1);
						torqueM1 = torqueM1.MulAdd(lhs->m_Jt.m_jacobianM1.m_angular, deltaForce1);
					}
				}

				for (ndInt32 i = 0; i < rowsCount; ++i)
				{
					ndRightHandSide* const rhs = &m_rightHandSide[rowStart + i];
					rhs->m_force = force[i];
				}
			}

			ndVector forceM0(zero);
			ndVector torqueM0(zero);
			ndVector forceM1(zero);
			ndVector torqueM1(zero);

			for (ndInt32 j = 0; j < rowsCount; ++j)
			{
				ndRightHandSide* const rhs = &m_rightHandSide[rowStart + j];
				const ndLeftHandSide* const lhs = &m_leftHandSide[rowStart + j];

				const ndVector f(rhs->m_force);
				forceM0 = forceM0.MulAdd(lhs->m_Jt.m_jacobianM0.m_linear, f);
				torqueM0 = torqueM0.MulAdd(lhs->m_Jt.m_jacobianM0.m_angular, f);
				forceM1 = forceM1.MulAdd(lhs->m_Jt.m_jacobianM1.m_linear, f);
				torqueM1 = torqueM1.MulAdd(lhs->m_Jt.m_jacobianM1.m_angular, f);
				rhs->m_maxImpact = ndMax(ndAbs(f.GetScalar()), rhs->m_maxImpact);
			}

			const ndInt32 index0 = jointIndex * 2 + 0;
			ndJacobian& outBody0 = jointPartialForces[index0];
			outBody0.m_linear = forceM0;
			outBody0.m_angular = torqueM0;

			const ndInt32 index1 = jointIndex * 2 + 1;
			ndJacobian& outBody1 = jointPartialForces[index1];
			outBody1.m_linear = forceM1;
			outBody1.m_angular = torqueM1;
		};

		ndConstraint* const joint = jointArray[groupId];
		JointForce(joint, groupId);
	});

	auto ApplyJacobianAccumulatePartialForces = ndMakeObject::ndFunction([this, &bodyArray](ndInt32 groupId, ndInt32)
	{
		D_TRACKTIME_NAMED(ApplyJacobianAccumulatePartialForces);
		const ndVector zero(ndVector::m_zero);

		ndJacobian* const internalForces = &GetInternalForces()[0];
		const ndInt32* const bodyIndex = &GetJointForceIndexBuffer()[0];

		const ndJacobian* const jointInternalForces = &GetTempInternalForces()[0];
		const ndJointBodyPairIndex* const jointBodyPairIndexBuffer = &GetJointBodyPairIndexBuffer()[0];

		ndVector force(zero);
		ndVector torque(zero);
		const ndInt32 m = groupId;
		const ndBodyKinematic* const body = bodyArray[m];

		const ndInt32 startIndex = bodyIndex[m];
		const ndInt32 mask = body->m_isStatic - 1;
		const ndInt32 count = mask & (bodyIndex[m + 1] - startIndex);
		for (ndInt32 k = 0; k < count; ++k)
		{
			const ndInt32 index = jointBodyPairIndexBuffer[startIndex + k].m_joint;
			force += jointInternalForces[index].m_linear;
			torque += jointInternalForces[index].m_angular;
		}
		internalForces[m].m_linear = force;
		internalForces[m].m_angular = torque;
	});

	for (ndInt32 i = 0; i < ndInt32(passes); ++i)
	{
		const ndInt32 jointCount = ndInt32(jointArray.GetCount());
		scene->ParallelExecute(CalculateJointsForce, jointCount, scene->OptimalGroupBatch(jointCount) / 2);

		const ndInt32 bodyCount = ndInt32(bodyArray.GetCount());
		scene->ParallelExecute(ApplyJacobianAccumulatePartialForces, bodyCount, scene->OptimalGroupBatch(bodyCount));
	}
}

void ndDynamicsUpdate::CalculateForces()
{
	D_TRACKTIME();
	if (m_world->GetScene()->GetActiveContactArray().GetCount())
	{
		m_firstPassCoef = ndFloat32(0.0f);

		InitSkeletons();
		for (ndInt32 step = 0; step < 4; step++)
		{
			CalculateJointsAcceleration();
			CalculateJointsForce();
			UpdateSkeletons();
			IntegrateBodiesVelocity();
		}
	}
}

void ndDynamicsUpdate::Update()
{
	D_TRACKTIME();
	m_timestep = m_world->GetScene()->GetTimestep();

	BuildIsland();
	IntegrateUnconstrainedBodies();
	InitWeights();
	InitBodyArray();
	InitJacobianMatrix();
	CalculateForces();
	IntegrateBodies();
	UpdateForceFeedback();
	DetermineSleepStates();
}
