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

#include "dCoreStdafx.h"
#include "ndNewtonStdafx.h"
#include "ndWorld.h"
#include "ndBodyDynamic.h"
#include "ndSkeletonList.h"
#include "ndDynamicsUpdate.h"
#include "ndJointBilateralConstraint.h"


#define D_DEFAULT_BUFFER_SIZE		1024

ndDynamicsUpdate::ndDynamicsUpdate(ndWorld* const world)
	:m_islands____(D_DEFAULT_BUFFER_SIZE)
	,m_activeBodies(D_DEFAULT_BUFFER_SIZE)
	,m_bodyIslandOrder____(D_DEFAULT_BUFFER_SIZE)
	,m_jointForcesIndex(D_DEFAULT_BUFFER_SIZE)
	,m_internalForces(D_DEFAULT_BUFFER_SIZE)
	,m_leftHandSide(D_DEFAULT_BUFFER_SIZE * 4)
	,m_rightHandSide(D_DEFAULT_BUFFER_SIZE)
	,m_tempInternalForces(D_DEFAULT_BUFFER_SIZE)
	,m_jointBodyPairIndexBuffer(D_DEFAULT_BUFFER_SIZE)
	,m_world(world)
	,m_timestep(dFloat32(0.0f))
	,m_invTimestep(dFloat32(0.0f))
	,m_firstPassCoef(dFloat32(0.0f))
	,m_invStepRK(dFloat32(0.0f))
	,m_timestepRK(dFloat32(0.0f))
	,m_invTimestepRK(dFloat32(0.0f))
	,m_solverPasses(0)
	,m_activeConstrainedBodyCount(0)
	,m_activeJointCount(0)
	,m_unConstrainedBodyCount____(0)
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
	m_islands____.Resize(D_DEFAULT_BUFFER_SIZE);
	m_activeBodies.Resize(D_DEFAULT_BUFFER_SIZE);
	m_bodyIslandOrder____.Resize(D_DEFAULT_BUFFER_SIZE);
	m_rightHandSide.Resize(D_DEFAULT_BUFFER_SIZE);
	m_internalForces.Resize(D_DEFAULT_BUFFER_SIZE);
	m_leftHandSide.Resize(D_DEFAULT_BUFFER_SIZE * 4);
	m_tempInternalForces.Resize(D_DEFAULT_BUFFER_SIZE);
	m_jointForcesIndex.Resize(D_DEFAULT_BUFFER_SIZE);
	m_jointBodyPairIndexBuffer.Resize(D_DEFAULT_BUFFER_SIZE);
}

void ndDynamicsUpdate::SortBodyJointScan()
{
	class ndSleepingBodiesInfo
	{
		public:
		ndSleepingBodiesInfo()
		{
			m_keyMap[0] = 0;
			m_keyMap[1] = 0;
			m_keyMap[2] = 1;
			m_keyMap[3] = 2;
		}
		
		dInt32 CalculateKey(const ndBodyDynamic* const body) const
		{
			dInt32 entry = body->m_solverSleep1 * 2 + body->m_equilibrium;
			return m_keyMap[entry];
		}

		dInt32 m_keyMap[4];
		dInt32 m_digitSum[4];
		dInt32 m_digitScan[D_MAX_THREADS_COUNT][4];
	};

	class ndEvaluateKey
	{
		public:
		dUnsigned32 GetKey(const ndDynamicsUpdate::ndJointBodyPairIndex& entry) const
		{
			return entry.m_body;
		}
	};
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

	class ndCountSleepingBodies: public ndScene::ndBaseJob
	{
		public:
		virtual void Execute()
		{
			D_TRACKTIME();
			ndWorld* const world = m_owner->GetWorld();
			ndScene* const scene = world->GetScene();
			const dArray<ndBodyKinematic*>& bodyArray = scene->GetActiveBodyArray();

			const dInt32 threadIndex = GetThreadId();
			const dInt32 threadCount = m_owner->GetThreadCount();
			const dInt32 bodyCount = bodyArray.GetCount();
			const dInt32 stride = bodyCount / threadCount;
			const dInt32 start = threadIndex * stride;
			const dInt32 blockSize = (threadIndex != (threadCount - 1)) ? stride : bodyCount - start;

			ndSleepingBodiesInfo& info = *((ndSleepingBodiesInfo*)m_context);
			dInt32* const digitBuffer = &info.m_digitScan[threadIndex][0];
			digitBuffer[0] = 0;
			digitBuffer[1] = 0;
			digitBuffer[2] = 0;
			digitBuffer[3] = 0;

			for (dInt32 i = 0; i < blockSize; i++)
			{
				const ndBodyDynamic* const body = bodyArray[start + i]->GetAsBodyDynamic();
				dInt32 key = info.CalculateKey(body);
				digitBuffer[key] ++;
			}
		}
	};

	class ndSortSleepingBodies : public ndScene::ndBaseJob
	{
		public:
		virtual void Execute()
		{
			D_TRACKTIME();
			ndWorld* const world = m_owner->GetWorld();
			ndScene* const scene = world->GetScene();
			ndDynamicsUpdate* const me = world->m_solver;
			const dArray<ndBodyKinematic*>& bodyArray = scene->GetActiveBodyArray();
			dArray<dInt32>& sortedBodyArray = me->GetActiveBodies();

			const dInt32 threadIndex = GetThreadId();
			const dInt32 threadCount = m_owner->GetThreadCount();
			const dInt32 bodyCount = bodyArray.GetCount();
			const dInt32 stride = bodyCount / threadCount;
			const dInt32 start = threadIndex * stride;
			const dInt32 blockSize = (threadIndex != (threadCount - 1)) ? stride : bodyCount - start;

			ndSleepingBodiesInfo& info = *((ndSleepingBodiesInfo*)m_context);
			dInt32* const digitBuffer = &info.m_digitScan[threadIndex][0];
			for (dInt32 i = 0; i < blockSize; i++)
			{
				const ndBodyDynamic* const body = bodyArray[start + i]->GetAsBodyDynamic();
				dInt32 key = info.CalculateKey(body);
				dInt32 index = digitBuffer[key];
				sortedBodyArray[index] = body->m_index;
				digitBuffer[key] ++;
			}
		}
	};

	ndScene* const scene = m_world->GetScene();
	const dInt32 threadCount = scene->GetThreadCount();
	ndConstraintArray& jointArray = scene->GetActiveContactArray();
	GetTempInternalForces().SetCount(jointArray.GetCount() * 2);
	GetJointBodyPairIndexBuffer().SetCount(jointArray.GetCount() * 2);

	scene->SubmitJobs<ndCountJointBodyPairs>();
	scene->CountingSort<ndJointBodyPairIndex, D_MAX_BODY_RADIX_BIT, ndEvaluateKey>(&GetJointBodyPairIndexBuffer()[0], (ndJointBodyPairIndex*)GetTempBuffer(), GetJointBodyPairIndexBuffer().GetCount(), 0);
	scene->CountingSort<ndJointBodyPairIndex, D_MAX_BODY_RADIX_BIT, ndEvaluateKey>(&GetJointBodyPairIndexBuffer()[0], (ndJointBodyPairIndex*)GetTempBuffer(), GetJointBodyPairIndexBuffer().GetCount(), 1);

	dArray<dInt32>& bodyJointIndex = GetJointForceIndexBuffer();
	const dInt32 bodyCount = scene->GetActiveBodyArray().GetCount() + 1;
	bodyJointIndex.SetCount(bodyCount);
	ClearBuffer(&bodyJointIndex[0], bodyCount * sizeof(dInt32));

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
	for (dInt32 i = 0; i < bodyCount; i++)
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

//#ifndef D_USE_ISLANDS
	ndSleepingBodiesInfo sleepingBodiesInfo;
	scene->SubmitJobs<ndCountSleepingBodies>(&sleepingBodiesInfo);

	dInt32 sum = 0;
	const dInt32 scanSize = 4;
	for (dInt32 j = 0; j < scanSize; j++)
	{
		sleepingBodiesInfo.m_digitSum[j] = sum;
		for (dInt32 i = 0; i < threadCount; i++)
		{
			const dInt32 count = sleepingBodiesInfo.m_digitScan[i][j];
			sleepingBodiesInfo.m_digitScan[i][j] = sum;
			sum += count;
		}
	}

	m_activeConstrainedBodyCount = sleepingBodiesInfo.m_digitSum[1] - sleepingBodiesInfo.m_digitSum[0];

	GetActiveBodies().SetCount(sleepingBodiesInfo.m_digitSum[3]);
	scene->SubmitJobs<ndSortSleepingBodies>(&sleepingBodiesInfo);

	GetActiveBodies().SetCount(sleepingBodiesInfo.m_digitSum[2]);
//#endif
}

void ndDynamicsUpdate::SortJointsScan()
{
	D_TRACKTIME();

	class ndSleep0 : public ndScene::ndBaseJob
	{
		public:
		virtual void Execute()
		{
			D_TRACKTIME();
			ndWorld* const world = m_owner->GetWorld();
			ndScene* const scene = world->GetScene();
			ndConstraintArray& jointArray = scene->GetActiveContactArray();
			const dInt32 count = jointArray.GetCount();
			const dInt32 threadIndex = GetThreadId();
			const dInt32 threadCount = m_owner->GetThreadCount();

			const dInt32 stride = count / threadCount;
			const dInt32 start = threadIndex * stride;
			const dInt32 blockSize = (threadIndex != (threadCount - 1)) ? stride : count - start;

			for (dInt32 i = 0; i < blockSize; i++)
			{
				ndConstraint* const joint = jointArray[i + start];
				ndBodyKinematic* const body0 = joint->GetBody0();
				ndBodyKinematic* const body1 = joint->GetBody1();
				const dInt32 rows = joint->GetRowsCount();
				joint->m_rowCount = rows;

				const dInt32 equilibrium = body0->m_equilibrium & body1->m_equilibrium;
				if (!equilibrium)
				{
					body0->m_solverSleep0 = 0;
					if (body1->m_invMass.m_w > dFloat32(0.0f))
					{
						body1->m_solverSleep0 = 0;
					}
				}

				body0->m_bodyIsConstrained = 1;
				body0->m_resting = body0->m_resting & equilibrium;
				if (body1->m_invMass.m_w > dFloat32(0.0f))
				{
					body1->m_bodyIsConstrained = 1;
					body1->m_resting = body1->m_resting & equilibrium;
				}
			}
		}
	};

	class ndSleep1 : public ndScene::ndBaseJob
	{
		public:
		virtual void Execute()
		{
			D_TRACKTIME();
			ndWorld* const world = m_owner->GetWorld();
			ndScene* const scene = world->GetScene();
			ndConstraintArray& jointArray = scene->GetActiveContactArray();
			const dInt32 count = jointArray.GetCount();
			const dInt32 threadIndex = GetThreadId();
			const dInt32 threadCount = m_owner->GetThreadCount();
			const dInt32 stride = count / threadCount;
			const dInt32 start = threadIndex * stride;
			const dInt32 blockSize = threadIndex != (threadCount - 1) ? stride : count - start;

			dInt32 activeJointCount = 0;
			for (dInt32 i = 0; i < blockSize; i++)
			{
				ndConstraint* const joint = jointArray[i + start];
				ndBodyKinematic* const body0 = joint->GetBody0();
				ndBodyKinematic* const body1 = joint->GetBody1();

				const dInt32 resting = body0->m_resting & body1->m_resting;
				activeJointCount += (1 - resting);
				joint->m_resting = resting;

				const dInt32 solverSleep0 = body0->m_solverSleep0 & body1->m_solverSleep0;
				if (!solverSleep0)
				{
					body0->m_solverSleep1 = 0;
					if (body1->m_invMass.m_w > dFloat32(0.0f))
					{
						body1->m_solverSleep1 = 0;
					}
				}
			}
			*(((dInt32*)m_context) + threadIndex) = activeJointCount;
		}
	};

	class ndScan0 : public ndScene::ndBaseJob
	{
		public:
		virtual void Execute()
		{
			D_TRACKTIME();
			ndWorld* const world = m_owner->GetWorld();
			ndScene* const scene = world->GetScene();
			ndDynamicsUpdate* const me = world->m_solver;
			ndConstraintArray& jointArray = scene->GetActiveContactArray();
			const dInt32 count = jointArray.GetCount();
			const dInt32 threadIndex = GetThreadId();
			const dInt32 threadCount = m_owner->GetThreadCount();
			const dInt32 stride = count / threadCount;
			const dInt32 start = threadIndex * stride;
			const dInt32 blockSize = (threadIndex != (threadCount - 1)) ? stride : count - start;

			dInt32* const histogram = ((dInt32*)m_context) + 2 * threadIndex;
			ndConstraint** const sortBuffer = (ndConstraint**)me->GetTempBuffer();

			histogram[0] = 0;
			histogram[1] = 0;
			for (dInt32 i = 0; i < blockSize; i++)
			{
				ndConstraint* const joint = jointArray[i + start];
				sortBuffer[i + start] = joint;
				ndBodyKinematic* const body0 = joint->GetBody0();
				ndBodyKinematic* const body1 = joint->GetBody1();
				const dInt32 key = body0->m_solverSleep1 & body1->m_solverSleep1;
				histogram[key]++;
			}
		}
	};

	class ndSort0 : public ndScene::ndBaseJob
	{
		public:
		virtual void Execute()
		{
			D_TRACKTIME();
			ndWorld* const world = m_owner->GetWorld();
			ndScene* const scene = world->GetScene();
			ndDynamicsUpdate* const me = world->m_solver;
			ndConstraintArray& jointArray = scene->GetActiveContactArray();
			const dInt32 count = jointArray.GetCount();
			const dInt32 threadIndex = GetThreadId();
			const dInt32 threadCount = m_owner->GetThreadCount();
			const dInt32 stride = count / threadCount;
			const dInt32 start = threadIndex * stride;
			const dInt32 blockSize = (threadIndex != (threadCount - 1)) ? stride : count - start;

			dInt32* const histogram = ((dInt32*)m_context) + 2 * threadIndex;
			ndConstraint** const sortBuffer = (ndConstraint**)me->GetTempBuffer();
			for (dInt32 i = 0; i < blockSize; i++)
			{
				ndConstraint* const joint = sortBuffer[i + start];
				ndBodyKinematic* const body0 = joint->GetBody0();
				ndBodyKinematic* const body1 = joint->GetBody1();
				const dInt32 key = body0->m_solverSleep1 & body1->m_solverSleep1;
				const dInt32 entry = histogram[key];
				jointArray[entry] = joint;
				histogram[key] = entry + 1;
			}
		}
	};

	class EvaluateCountRows
	{
		public:
		dUnsigned32 GetKey(const ndConstraint* const joint) const
		{
			const ndSortKey key(joint->m_resting, joint->m_rowCount);
			return key.m_value;
		}
	};

	ndScene* const scene = m_world->GetScene();

	for (ndSkeletonList::dNode* node = m_world->GetSkeletonList().GetFirst(); node; node = node->GetNext())
	{
		ndSkeletonContainer* const skeleton = &node->GetInfo();
		skeleton->CheckSleepState();
	}

	const ndJointList& jointList = m_world->GetJointList();
	ndConstraintArray& jointArray = scene->GetActiveContactArray();

	dInt32 jointArrayCount = jointArray.GetCount();
	jointArray.SetCount(jointArrayCount + jointList.GetCount());

	for (ndJointList::dNode* node = jointList.GetFirst(); node; node = node->GetNext())
	{
		ndJointBilateralConstraint* const joint = node->GetInfo();
		if (joint->IsActive())
		{
			jointArray[jointArrayCount] = joint;
			jointArrayCount++;
		}
	}
	jointArray.SetCount(jointArrayCount);

	m_leftHandSide.SetCount(jointArray.GetCount() + 32);

	dInt32 histogram[D_MAX_THREADS_COUNT][2];
	dInt32 movingJoints[D_MAX_THREADS_COUNT];
	const dInt32 threadCount = scene->GetThreadCount();

static int xxxx;
xxxx++;
if (xxxx >= 16)
xxxx *= 1;

	scene->SubmitJobs<ndSleep0>();
	scene->SubmitJobs<ndSleep1>(movingJoints);
	scene->SubmitJobs<ndScan0>(histogram);

	dInt32 scan[2];
	scan[0] = 0;
	scan[1] = 0;
	dInt32 movingJointCount = 0;
	for (dInt32 i = 0; i < threadCount; i++)
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

	dInt32 sum = 0;
	for (dInt32 i = 0; i < 2; i++)
	{
		for (dInt32 j = 0; j < threadCount; j++)
		{
			dInt32 partialSum = histogram[j][i];
			histogram[j][i] = sum;
			sum += partialSum;
		}
	}
	scene->SubmitJobs<ndSort0>(histogram);

#ifdef _DEBUG
	for (dInt32 i = 0; i < (jointArray.GetCount() - 1); i++)
	{
		const ndConstraint* const joint0 = jointArray[i];
		const ndConstraint* const joint1 = jointArray[i + 1];
		const dInt32 key0 = (joint0->GetBody0()->m_solverSleep1 & joint0->GetBody1()->m_solverSleep1) ? 1 : 0;
		const dInt32 key1 = (joint1->GetBody0()->m_solverSleep1 & joint1->GetBody1()->m_solverSleep1) ? 1 : 0;
		dAssert(key0 <= key1);
	}
#endif

	dAssert(m_activeJointCount <= jointArray.GetCount());
	jointArray.SetCount(m_activeJointCount);
	for (dInt32 i = 0; i < jointArray.GetCount(); i++)
	{
		ndConstraint* const joint = jointArray[i];
		ndBodyKinematic* const body0 = joint->GetBody0();
		ndBodyKinematic* const body1 = joint->GetBody1();

		if (body1->m_invMass.m_w > dFloat32(0.0f))
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
	m_activeJointCount = movingJointCount;

	GetTempInternalForces().SetCount(jointArray.GetCount() * 2);
	GetJointBodyPairIndexBuffer().SetCount(jointArray.GetCount() * 2);
	scene->CountingSort<ndConstraint*, 7, EvaluateCountRows>(&jointArray[0], (ndConstraint**)GetTempBuffer(), jointArray.GetCount(), 0);
}

void ndDynamicsUpdate::SortJoints()
{
	D_TRACKTIME();

	SortJointsScan();
	ndScene* const scene = m_world->GetScene();
	if (!m_activeJointCount)
	{
		dArray<dInt32>& indexBuffer = GetJointForceIndexBuffer();
		const dArray<ndBodyKinematic*>& bodyArray = scene->GetActiveBodyArray();
		indexBuffer.SetCount(bodyArray.GetCount() + 1);

		const dInt32 bodyCount = bodyArray.GetCount();
		dArray<dInt32>& bodyIndexArray = GetActiveBodies();
		bodyIndexArray.SetCount(bodyCount);

		dInt32 count = 0;
		for (dInt32 i = 0; i < bodyCount; i++)
		{
			const ndBodyDynamic* const body = bodyArray[i]->GetAsBodyDynamic();
			if (!body->m_equilibrium)
			{
				dAssert(i == body->m_index);
				bodyIndexArray[count] = i;
				count++;
			}
		}

		bodyIndexArray.SetCount(count);
		m_activeConstrainedBodyCount = 0;
		return;
	}
	
	ndConstraintArray& jointArray = scene->GetActiveContactArray();

	dInt32 rowCount = 1;
	for (dInt32 i = 0; i < jointArray.GetCount(); i++)
	{
		ndConstraint* const joint = jointArray[i];
		joint->m_rowStart = rowCount;
		rowCount += joint->m_rowCount;
	}

	m_leftHandSide.SetCount(rowCount);
	m_rightHandSide.SetCount(rowCount);

	#ifdef _DEBUG
		dAssert(m_activeJointCount <= jointArray.GetCount());
		for (dInt32 i = 0; i < jointArray.GetCount(); i++)
		{
			ndConstraint* const joint = jointArray[i];
			dAssert(joint->m_rowStart < m_leftHandSide.GetCount());
			dAssert((joint->m_rowStart + joint->m_rowCount) <= rowCount);
		}

		for (dInt32 i = 1; i < m_activeJointCount; i++)
		{
			ndConstraint* const joint0 = jointArray[i - 1];
			ndConstraint* const joint1 = jointArray[i - 0];
			dAssert(!joint0->m_resting);
			dAssert(!joint1->m_resting);
			dAssert(joint0->m_rowCount >= joint1->m_rowCount);
			dAssert(!(joint0->GetBody0()->m_resting & joint0->GetBody1()->m_resting));
			dAssert(!(joint1->GetBody0()->m_resting & joint1->GetBody1()->m_resting));
		}

		for (dInt32 i = m_activeJointCount + 1; i < jointArray.GetCount(); i++)
		{
			ndConstraint* const joint0 = jointArray[i - 1];
			ndConstraint* const joint1 = jointArray[i - 0];
			dAssert(joint0->m_resting);
			dAssert(joint1->m_resting);
			dAssert(joint0->m_rowCount >= joint1->m_rowCount);
			dAssert(joint0->GetBody0()->m_resting & joint0->GetBody1()->m_resting);
			dAssert(joint1->GetBody0()->m_resting & joint1->GetBody1()->m_resting);
		}
	#endif

	SortBodyJointScan();
}

void ndDynamicsUpdate::SortIslands()
{
	D_TRACKTIME();
	ndScene* const scene = m_world->GetScene();
	const dArray<ndBodyKinematic*>& bodyArray = scene->GetActiveBodyArray();
	GetInternalForces().SetCount(bodyArray.GetCount());

	dInt32 bodyCount = 0;
	const dInt32 totalBodyCount = bodyArray.GetCount() - 1;
	ndBodyIndexPair* const buffer0 = (ndBodyIndexPair*)&GetInternalForces()[0];
	for (dInt32 i = 0; i < totalBodyCount; i++)
	{
		ndBodyKinematic* const body = bodyArray[i];
		if (!(body->m_resting & body->m_islandSleep) || body->GetAsBodyPlayerCapsule())
		{
			buffer0[bodyCount].m_body = body;
			if (body->m_invMass.m_w > dFloat32(0.0f))
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

	dArray<ndIsland>& islands = GetIsland____();
	dArray<dInt32>& islandOrder = GetBodyIslandOrder____();

	islands.SetCount(0);
	islandOrder.SetCount(bodyCount);

	m_unConstrainedBodyCount____ = 0;
	if (bodyCount)
	{
		// sort using counting sort o(n)
		dInt32 scans[2];
		scans[0] = 0;
		scans[1] = 0;
		for (dInt32 i = 0; i < bodyCount; i++)
		{
			dInt32 j = 1 - buffer0[i].m_root->m_bodyIsConstrained;
			scans[j] ++;
		}
		scans[1] = scans[0];
		scans[0] = 0;
		ndBodyIndexPair* const buffer2 = buffer0 + bodyCount;
		for (dInt32 i = 0; i < bodyCount; i++)
		{
			const dInt32 key = 1 - buffer0[i].m_root->m_bodyIsConstrained;
			const dInt32 j = scans[key];
			buffer2[j] = buffer0[i];
			scans[key] = j + 1;
		}

		const ndBodyIndexPair* const buffer1 = buffer0 + bodyCount;
		for (dInt32 i = 0; i < bodyCount; i++)
		{
			dAssert(bodyArray[buffer1[i].m_body->m_index] == buffer1[i].m_body);
			dAssert((i == bodyCount - 1) || (buffer1[i].m_root->m_bodyIsConstrained >= buffer1[i + 1].m_root->m_bodyIsConstrained));

			islandOrder[i] = buffer1[i].m_body->m_index;
			if (buffer1[i].m_root->m_rank == -1)
			{
				buffer1[i].m_root->m_rank = 0;
				ndIsland island(buffer1[i].m_root);
				islands.PushBack(island);
			}
			buffer1[i].m_root->m_rank += 1;
		}

		dInt32 start = 0;
		dInt32 islandMaxKeySize = 0;
		dInt32 unConstrainedCount = 0;
		for (dInt32 i = 0; i < islands.GetCount(); i++)
		{
			ndIsland& island = islands[i];
			island.m_start = start;
			island.m_count = island.m_root->m_rank;
			islandMaxKeySize = dMax(islandMaxKeySize, island.m_count);
			start += island.m_count;
			unConstrainedCount += island.m_root->m_bodyIsConstrained ? 0 : 1;
		}

		m_unConstrainedBodyCount____ = unConstrainedCount;
		class EvaluateKey
		{
			public:
			dUnsigned32 GetKey(const ndIsland& island) const
			{
				dUnsigned32 key = island.m_count * 2 + island.m_root->m_bodyIsConstrained;
				const dUnsigned32 maxVal = 1 << (D_MAX_BODY_RADIX_BIT * 2);
				dAssert(key < maxVal);
				return maxVal - key;
			}
		};

		scene->CountingSort<ndIsland, D_MAX_BODY_RADIX_BIT, EvaluateKey>(&islands[0], (ndIsland*)GetTempBuffer(), islands.GetCount(), 0);
		if (islandMaxKeySize >= 1 << (D_MAX_BODY_RADIX_BIT - 1))
		{
			scene->CountingSort<ndIsland, D_MAX_BODY_RADIX_BIT, EvaluateKey>(&islands[0], (ndIsland*)GetTempBuffer(), islands.GetCount(), 1);
		}
	}
}

void ndDynamicsUpdate::BuildIsland()
{
	ndScene* const scene = m_world->GetScene();
	const dArray<ndBodyKinematic*>& bodyArray = scene->GetActiveBodyArray();
	dAssert(bodyArray.GetCount() >= 1);
	if (bodyArray.GetCount() - 1)
	{
		D_TRACKTIME();
		SortJoints();
#ifdef D_USE_ISLANDS
		SortIslands();
#endif
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
			ndScene* const scene = world->GetScene();
			ndDynamicsUpdate* const me = world->m_solver;
			ndBodyKinematic** const bodyArray = &scene->GetActiveBodyArray()[0];

			const dFloat32 timestep = m_timestep;
			const dInt32 threadIndex = GetThreadId();
			const dInt32 threadCount = m_owner->GetThreadCount();
			const dInt32 bodyStart = me->GetConstrainedBodyCount();
			const dInt32 bodyCount = me->GetUnconstrainedBodyCount();

			const dInt32 stride = bodyCount / threadCount;
			const dInt32 start = threadIndex * stride;
			const dInt32 blockSize = (threadIndex != (threadCount - 1)) ? stride : bodyCount - start;

			const dInt32* const activeBodyArray = &me->GetActiveBodies()[bodyStart];
			for (dInt32 i = 0; i < blockSize; i++)
			{
				dInt32 index = activeBodyArray[start + i];
				ndBodyKinematic* const body = bodyArray[index]->GetAsBodyKinematic();
				dAssert(body);
				body->UpdateInvInertiaMatrix();
				body->AddDampingAcceleration(timestep);
				body->IntegrateExternalForce(timestep);
			}
		}
	};

	if (GetUnconstrainedBodyCount())
	{
		D_TRACKTIME();
		ndScene* const scene = m_world->GetScene();
		scene->SubmitJobs<ndIntegrateUnconstrainedBodies>();
	}
}

void ndDynamicsUpdate::InitWeights()
{
	D_TRACKTIME();
	class ndInitWeights : public ndScene::ndBaseJob
	{
		public:
#if 0
		virtual void Execute()
		{
			D_TRACKTIME();
			const ndConstraintArray& jointArray = m_owner->GetActiveContactArray();

			dFloat32 maxExtraPasses = dFloat32(1.0f);
			const dInt32 threadIndex = GetThreadId();
			const dInt32 threadCount = m_owner->GetThreadCount();
			const dInt32 jointCount = jointArray.GetCount();

			const dInt32 stride = jointCount / threadCount;
			const dInt32 start = threadIndex * stride;
			const dInt32 blockSize = (threadIndex != (threadCount - 1)) ? stride : jointCount - start;

			for (dInt32 i = 0; i < blockSize; i++)
			{
				ndConstraint* const constraint = jointArray[i + start];
				ndBodyKinematic* const body0 = constraint->GetBody0();
				ndBodyKinematic* const body1 = constraint->GetBody1();

				if (body1->m_invMass.m_w > dFloat32(0.0f))
				{
					dScopeSpinLock lock(body1->m_lock);
					body1->m_weigh += dFloat32(1.0f);
					maxExtraPasses = dMax(body1->m_weigh, maxExtraPasses);
				}
				dScopeSpinLock lock(body0->m_lock);
				body0->m_weigh += dFloat32(1.0f);
				dAssert(body0->m_invMass.m_w != dFloat32(0.0f));
				maxExtraPasses = dMax(body0->m_weigh, maxExtraPasses);
			}
			dFloat32* const extraPasses = (dFloat32*)m_context;
			extraPasses[threadIndex] = maxExtraPasses;
		}

#else
		virtual void Execute()
		{
			D_TRACKTIME();
			ndWorld* const world = m_owner->GetWorld();
			ndDynamicsUpdate* const me = (ndDynamicsUpdate*)world->m_solver;
			const dArray<ndBodyKinematic*>& bodyArray = m_owner->GetActiveBodyArray();

			const dInt32 bodyCount = me->GetConstrainedBodyCount();
			const dInt32 threadIndex = GetThreadId();
			const dInt32 threadCount = m_owner->GetThreadCount();

			const dInt32 stride = bodyCount / threadCount;
			const dInt32 start = threadIndex * stride;
			const dInt32 blockSize = (threadIndex != (threadCount - 1)) ? stride : bodyCount - start;

			dFloat32 maxExtraPasses = dFloat32(1.0f);
			const dInt32* const activeBodyArray = &me->GetActiveBodies()[0];
			const dInt32* const activeJointsCount = &me->GetJointForceIndexBuffer()[0];
			for (dInt32 i = 0; i < blockSize; i++)
			{
				const dInt32 index = activeBodyArray[start + i];
				ndBodyKinematic* const body = bodyArray[index]->GetAsBodyKinematic();
				dAssert(body->m_invMass.m_w > dFloat32(0.0f));
				const dFloat32 weigh = dFloat32(activeJointsCount[index + 1] - activeJointsCount[index]);
				body->m_weigh = weigh;
				maxExtraPasses = dMax(weigh, maxExtraPasses);
			}
			dFloat32* const extraPasses = (dFloat32*)m_context;
			extraPasses[threadIndex] = maxExtraPasses;
		}
#endif
	};

	ndScene* const scene = m_world->GetScene();
	m_invTimestep = dFloat32(1.0f) / m_timestep;
	m_invStepRK = dFloat32(0.25f);
	m_timestepRK = m_timestep * m_invStepRK;
	m_invTimestepRK = m_invTimestep * dFloat32(4.0f);

	const dArray<ndBodyKinematic*>& bodyArray = scene->GetActiveBodyArray();
	const dInt32 bodyCount = bodyArray.GetCount();
	GetInternalForces().SetCount(bodyCount);

	dFloat32 extraPassesArray[D_MAX_THREADS_COUNT];
	memset(extraPassesArray, 0, sizeof(extraPassesArray));
	scene->SubmitJobs<ndInitWeights>(extraPassesArray);

	dFloat32 extraPasses = dFloat32(0.0f);
	const dInt32 threadCount = scene->GetThreadCount();
	for (dInt32 i = 0; i < threadCount; i++)
	{
		extraPasses = dMax(extraPasses, extraPassesArray[i]);
	}

	const dInt32 conectivity = 7;
	m_solverPasses = m_world->GetSolverIterations() + 2 * dInt32(extraPasses) / conectivity + 1;
}

void ndDynamicsUpdate::InitBodyArray()
{
	D_TRACKTIME();
	class ndInitBodyArray : public ndScene::ndBaseJob
	{
		public:
		virtual void Execute()
		{
			D_TRACKTIME();
			ndWorld* const world = m_owner->GetWorld();
			ndScene* const scene = world->GetScene();
			ndDynamicsUpdate* const me = world->m_solver;
			const dArray<dInt32>& bodyIslandOrder = me->GetBodyIslandOrder____();
			ndBodyKinematic** const bodyArray = &scene->GetActiveBodyArray()[0];

			const dFloat32 timestep = m_timestep;
			const dInt32 threadIndex = GetThreadId();
			const dInt32 threadCount = m_owner->GetThreadCount();
			const dInt32 bodyCount = bodyIslandOrder.GetCount() - me->GetUnconstrainedBodyCount____();

			const dInt32 stride = bodyCount / threadCount;
			const dInt32 start = threadIndex * stride;
			const dInt32 blockSize = (threadIndex != (threadCount - 1)) ? stride : bodyCount - start;

			for (dInt32 i = 0; i < blockSize; i++)
			{
				dInt32 index = bodyIslandOrder[start + i];
				ndBodyDynamic* const body = bodyArray[index]->GetAsBodyDynamic();
				if (body)
				{
					dAssert(body->m_bodyIsConstrained);
					body->UpdateInvInertiaMatrix();
					body->AddDampingAcceleration(timestep);
					const dVector angularMomentum(body->CalculateAngularMomentum());
					body->m_gyroTorque = body->m_omega.CrossProduct(angularMomentum);
					body->m_gyroAlpha = body->m_invWorldInertiaMatrix.RotateVector(body->m_gyroTorque);

					body->m_accel = body->m_veloc;
					body->m_alpha = body->m_omega;
					body->m_gyroRotation = body->m_rotation;
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
	const dInt32 baseIndex = joint->m_rowStart;
	for (dInt32 i = 0; i < dof; i++)
	{
		dAssert(constraintParam.m_forceBounds[i].m_jointForce);

		ndLeftHandSide* const row = &m_leftHandSide[baseIndex + i];
		ndRightHandSide* const rhs = &m_rightHandSide[baseIndex + i];

		row->m_Jt = constraintParam.m_jacobian[i];
		rhs->m_diagDamp = dFloat32(0.0f);
		rhs->m_diagonalRegularizer = dMax(constraintParam.m_diagonalRegularizer[i], dFloat32(1.0e-5f));

		rhs->m_coordenateAccel = constraintParam.m_jointAccel[i];
		rhs->m_restitution = constraintParam.m_restitution[i];
		rhs->m_penetration = constraintParam.m_penetration[i];
		rhs->m_penetrationStiffness = constraintParam.m_penetrationStiffness[i];
		rhs->m_lowerBoundFrictionCoefficent = constraintParam.m_forceBounds[i].m_low;
		rhs->m_upperBoundFrictionCoefficent = constraintParam.m_forceBounds[i].m_upper;
		rhs->m_jointFeebackForce = constraintParam.m_forceBounds[i].m_jointForce;

		dAssert(constraintParam.m_forceBounds[i].m_normalIndex >= -1);
		const dInt32 frictionIndex = constraintParam.m_forceBounds[i].m_normalIndex;
		const dInt32 mask = frictionIndex >> 31;
		rhs->m_normalForceIndex = frictionIndex;
		rhs->m_normalForceIndexFlat = ~mask & (frictionIndex + baseIndex);
	}
}

void ndDynamicsUpdate::InitJacobianMatrix()
{
	class ndInitJacobianMatrix : public ndScene::ndBaseJob
	{
		public:
		ndInitJacobianMatrix()
			:m_zero(dVector::m_zero)
		{
		}

		void BuildJacobianMatrix(ndConstraint* const joint, dInt32 jointIndex)
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
			const dMatrix& invInertia0 = body0->m_invWorldInertiaMatrix;
			const dMatrix& invInertia1 = body1->m_invWorldInertiaMatrix;
			const dVector invMass0(body0->m_invMass[3]);
			const dVector invMass1(body1->m_invMass[3]);

			dVector force0(m_zero);
			dVector torque0(m_zero);
			if (dynBody0)
			{
				force0 = dynBody0->m_externalForce;
				torque0 = dynBody0->m_externalTorque;
			}

			dVector force1(m_zero);
			dVector torque1(m_zero);
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

			dVector forceAcc0(m_zero);
			dVector torqueAcc0(m_zero);
			dVector forceAcc1(m_zero);
			dVector torqueAcc1(m_zero);

			const dVector weigh0(body0->m_weigh * joint->m_preconditioner0);
			const dVector weigh1(body1->m_weigh * joint->m_preconditioner0);

			const dFloat32 preconditioner0 = joint->m_preconditioner0;
			const dFloat32 preconditioner1 = joint->m_preconditioner1;

			const bool isBilateral = joint->IsBilateral();
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

				const dFloat32 extenalAcceleration = -tmpAccel.AddHorizontal().GetScalar();
				rhs->m_deltaAccel = extenalAcceleration;
				rhs->m_coordenateAccel += extenalAcceleration;
				dAssert(rhs->m_jointFeebackForce);
				const dFloat32 force = rhs->m_jointFeebackForce->GetInitialGuess();

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

			const dInt32 index0 = jointIndex * 2 + 0;
			ndJacobian& outBody0 = m_internalForces[index0];
			outBody0.m_linear = forceAcc0;
			outBody0.m_angular = torqueAcc0;

			const dInt32 index1 = jointIndex * 2 + 1;
			ndJacobian& outBody1 = m_internalForces[index1];
			outBody1.m_linear = forceAcc1;
			outBody1.m_angular = torqueAcc1;
		}

		virtual void Execute()
		{
			D_TRACKTIME();
			ndWorld* const world = m_owner->GetWorld();
			ndDynamicsUpdate* const me = world->m_solver;
			m_leftHandSide = &me->GetLeftHandSide()[0];
			m_rightHandSide = &me->GetRightHandSide()[0];
			m_internalForces = &me->GetTempInternalForces()[0];
			m_jointBodyPairIndexBuffer = &me->GetJointBodyPairIndexBuffer()[0];
			ndConstraint** const jointArray = &m_owner->GetActiveContactArray()[0];

			const dInt32 jointCount = m_owner->GetActiveContactArray().GetCount();
			const dInt32 threadIndex = GetThreadId();
			const dInt32 threadCount = m_owner->GetThreadCount();

			for (dInt32 i = threadIndex; i < jointCount; i += threadCount)
			{
				ndConstraint* const joint = jointArray[i];
				me->GetJacobianDerivatives(joint);
				BuildJacobianMatrix(joint, i);
			}
		}

		dVector m_zero;
		ndJacobian* m_internalForces;
		ndLeftHandSide* m_leftHandSide;
		ndRightHandSide* m_rightHandSide;
		const ndJointBodyPairIndex* m_jointBodyPairIndexBuffer;
	};

	class ndInitJacobianAccumulatePartialForces : public ndScene::ndBaseJob
	{
		public:
		virtual void Execute()
		{
			D_TRACKTIME();
			const dVector zero(dVector::m_zero);
			ndWorld* const world = m_owner->GetWorld();
			ndDynamicsUpdate* const me = (ndDynamicsUpdate*)world->m_solver;

			ndJacobian* const internalForces = &me->GetInternalForces()[0];
			const dInt32* const bodyIndex = &me->GetJointForceIndexBuffer()[0];
			const dArray<ndBodyKinematic*>& bodyArray = m_owner->GetActiveBodyArray();
			const ndJacobian* const jointInternalForces = &me->GetTempInternalForces()[0];
			const ndJointBodyPairIndex* const jointBodyPairIndexBuffer = &me->GetJointBodyPairIndexBuffer()[0];

			const dInt32 threadIndex = GetThreadId();
			const dInt32 threadCount = m_owner->GetThreadCount();
			const dInt32 bodyCount = m_owner->GetActiveBodyArray().GetCount();

			const dInt32 stride = bodyCount / threadCount;
			const dInt32 start = threadIndex * stride;
			const dInt32 blockSize = (threadIndex != (threadCount - 1)) ? stride : bodyCount - start;

			for (dInt32 i = 0; i < blockSize; i++)
			{
				dInt32 startIndex = bodyIndex[start + i];
				dInt32 count = bodyIndex[start + i + 1] - startIndex;
				if (count)
				{
					dVector force(zero);
					dVector torque(zero);
					const ndBodyKinematic* const body = bodyArray[i + start];
					if (body->m_invMass.m_w > dFloat32(0.0f))
					{
						for (dInt32 j = 0; j < count; j++)
						{
							dInt32 index = jointBodyPairIndexBuffer[startIndex + j].m_joint;
							force += jointInternalForces[index].m_linear;
							torque += jointInternalForces[index].m_angular;
						}
					}

					internalForces[i + start].m_linear = force;
					internalForces[i + start].m_angular = torque;
				}
			}
		}
	};

	ndScene* const scene = m_world->GetScene();
	if (scene->GetActiveContactArray().GetCount())
	{
		D_TRACKTIME();
		m_rightHandSide[0].m_force = dFloat32(1.0f);
		scene->SubmitJobs<ndInitJacobianMatrix>();
		scene->SubmitJobs<ndInitJacobianAccumulatePartialForces>();
	}
}

void ndDynamicsUpdate::CalculateJointsAcceleration()
{
	D_TRACKTIME();
	class ndCalculateJointsAcceleration : public ndScene::ndBaseJob
	{
		public:
		virtual void Execute()
		{
			D_TRACKTIME();
			ndWorld* const world = m_owner->GetWorld();
			ndDynamicsUpdate* const me = world->m_solver;
			const ndConstraintArray& jointArray = m_owner->GetActiveContactArray();

			ndJointAccelerationDecriptor joindDesc;
			joindDesc.m_timestep = me->m_timestepRK;
			joindDesc.m_invTimestep = me->m_invTimestepRK;
			joindDesc.m_firstPassCoefFlag = me->m_firstPassCoef;
			dArray<ndLeftHandSide>& leftHandSide = me->m_leftHandSide;
			dArray<ndRightHandSide>& rightHandSide = me->m_rightHandSide;

			const dInt32 threadIndex = GetThreadId();
			const dInt32 threadCount = m_owner->GetThreadCount();
			const dInt32 jointCount = jointArray.GetCount();

			for (dInt32 i = threadIndex; i < jointCount; i += threadCount)
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

void ndDynamicsUpdate::IntegrateBodiesVelocity()
{
	D_TRACKTIME();
	class ndIntegrateBodiesVelocity : public ndScene::ndBaseJob
	{
		public:
		virtual void Execute()
		{
			D_TRACKTIME();
			ndWorld* const world = m_owner->GetWorld();
			ndScene* const scene = world->GetScene();
			ndDynamicsUpdate* const me = world->m_solver;
			const dArray<dInt32>& bodyIslandOrder = me->GetBodyIslandOrder____();
			ndBodyKinematic** const bodyArray = &scene->GetActiveBodyArray()[0];
			const dArray<ndJacobian>& internalForces = me->GetInternalForces();

			const dVector timestep4(me->m_timestepRK);
			const dVector speedFreeze2(world->m_freezeSpeed2 * dFloat32(0.1f));

			const dInt32 threadIndex = GetThreadId();
			const dInt32 threadCount = m_owner->GetThreadCount();
			const dInt32 bodyCount = bodyIslandOrder.GetCount() - me->GetUnconstrainedBodyCount____();

			const dInt32 stride = bodyCount / threadCount;
			const dInt32 start = threadIndex * stride;
			const dInt32 blockSize = (threadIndex != (threadCount - 1)) ? stride : bodyCount - start;

			for (dInt32 i = 0; i < blockSize; i++)
			{
				dInt32 index = bodyIslandOrder[start + i];
				ndBodyDynamic* const body = bodyArray[index]->GetAsBodyDynamic();
				if (body)
				{
					dAssert(body->m_index == index);
					dAssert(body->m_bodyIsConstrained);
					const ndJacobian& forceAndTorque = internalForces[index];
					const dVector force(body->GetForce() + forceAndTorque.m_linear);
					const dVector torque(body->GetTorque() + forceAndTorque.m_angular - body->GetGyroTorque());
					const ndJacobian velocStep(body->IntegrateForceAndToque(force, torque, timestep4));

					if (!body->m_resting)
					{
						body->m_veloc += velocStep.m_linear;
						body->m_omega += velocStep.m_angular;
						body->IntegrateGyroSubstep(timestep4);
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
			ndDynamicsUpdate* const me = world->m_solver;
			const ndConstraintArray& jointArray = m_owner->GetActiveContactArray();
			dArray<ndRightHandSide>& rightHandSide = me->m_rightHandSide;

			const dInt32 threadIndex = GetThreadId();
			const dInt32 threadCount = m_owner->GetThreadCount();
			const dInt32 jointCount = jointArray.GetCount();

			const dFloat32 timestepRK = me->m_timestepRK;
			for (dInt32 i = threadIndex; i < jointCount; i += threadCount)
			{
				ndConstraint* const joint = jointArray[i];
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

				if (joint->GetAsBilateral())
				{
					const dArray<ndLeftHandSide>& leftHandSide = me->m_leftHandSide;
					dVector force0(dVector::m_zero);
					dVector force1(dVector::m_zero);
					dVector torque0(dVector::m_zero);
					dVector torque1(dVector::m_zero);

					for (dInt32 j = 0; j < rows; j++)
					{
						const ndRightHandSide* const rhs = &rightHandSide[j + first];
						const ndLeftHandSide* const lhs = &leftHandSide[j + first];
						const dVector f(rhs->m_force);
						force0 += lhs->m_Jt.m_jacobianM0.m_linear * f;
						torque0 += lhs->m_Jt.m_jacobianM0.m_angular * f;
						force1 += lhs->m_Jt.m_jacobianM1.m_linear * f;
						torque1 += lhs->m_Jt.m_jacobianM1.m_angular * f;
					}
					ndJointBilateralConstraint* const bilateral = joint->GetAsBilateral();
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
	class ndIntegrateBodies : public ndScene::ndBaseJob
	{
		public:
		virtual void Execute()
		{
			D_TRACKTIME();
			ndWorld* const world = m_owner->GetWorld();
			ndScene* const scene = world->GetScene();
			ndDynamicsUpdate* const me = world->m_solver;
			const dArray<dInt32>& bodyIslandOrder = me->GetBodyIslandOrder____();
			ndBodyKinematic** const bodyArray = &scene->GetActiveBodyArray()[0];

			const dFloat32 timestep = m_timestep;
			const dVector invTime(me->m_invTimestep);

			const dInt32 threadIndex = GetThreadId();
			const dInt32 threadCount = m_owner->GetThreadCount();
			const dInt32 bodyCount = bodyIslandOrder.GetCount();
			const dInt32 stride = bodyCount / threadCount;
			const dInt32 start = threadIndex * stride;
			const dInt32 blockSize = (threadIndex != (threadCount - 1)) ? stride : bodyCount - start;

			for (dInt32 i = 0; i < blockSize; i++)
			{
				dInt32 index = bodyIslandOrder[start + i];
				ndBodyDynamic* const dynBody = bodyArray[index]->GetAsBodyDynamic();

				// the initial velocity and angular velocity were stored in m_accel and dynBody->m_alpha for memory saving
				if (dynBody)
				{
					if (!dynBody->m_equilibrium)
					{
						dynBody->m_accel = invTime * (dynBody->m_veloc - dynBody->m_accel);
						dynBody->m_alpha = invTime * (dynBody->m_omega - dynBody->m_alpha);
						dynBody->IntegrateVelocity(timestep);
					}
				}
				else
				{
					ndBodyKinematic* const kinBody = dynBody->GetAsBodyKinematic();
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

void ndDynamicsUpdate::DetermineSleepStates()
{
	D_TRACKTIME();
#ifdef D_USE_ISLANDS
	class ndDetermineSleepStates : public ndScene::ndBaseJob
	{
		public:
		ndDetermineSleepStates()
			:m_velocTol(dFloat32(1.0e-8f))
		{
		}

		void UpdateIslandState(dInt32 entry)
		{
			ndWorld* const world = m_owner->GetWorld();
			ndScene* const scene = world->GetScene();
			ndDynamicsUpdate* const me = world->m_solver;
			const ndIsland& island = me->GetIsland____()[entry];
			const dArray<dInt32>& bodyIslandOrder = me->GetBodyIslandOrder____();
			ndBodyKinematic** const bodyIslands = &scene->GetActiveBodyArray()[0];

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

			const dFloat32 speedFreeze = world->m_freezeSpeed2;
			const dFloat32 accelFreeze = world->m_freezeAccel2 * ((count <= D_SMALL_ISLAND_COUNT) ? dFloat32(0.01f) : dFloat32(1.0f));
			const dFloat32 acc2 = D_SOLVER_MAX_ERROR * D_SOLVER_MAX_ERROR;

			const dVector maxAccNorm2((count > 4) ? acc2 : acc2 * dFloat32(0.0625f));
			const dVector velocDragVect(velocityDragCoeff, velocityDragCoeff, velocityDragCoeff, dFloat32(0.0f));

			dInt32 stackSleeping = 1;
			dInt32 sleepCounter = 10000;
			const dInt32 start = island.m_start;

			for (dInt32 i = 0; i < count; i++)
			{
				dInt32 index = bodyIslandOrder[start + i];
				ndBodyDynamic* const dynBody = bodyIslands[index]->GetAsBodyDynamic();
				if (dynBody)
				{
					dAssert(dynBody->m_accel.m_w == dFloat32(0.0f));
					dAssert(dynBody->m_alpha.m_w == dFloat32(0.0f));
					dAssert(dynBody->m_veloc.m_w == dFloat32(0.0f));
					dAssert(dynBody->m_omega.m_w == dFloat32(0.0f));

					dVector accelTest((dynBody->m_accel.DotProduct(dynBody->m_accel) > maxAccNorm2) | (dynBody->m_alpha.DotProduct(dynBody->m_alpha) > maxAccNorm2));
					dynBody->m_accel = dynBody->m_accel & accelTest;
					dynBody->m_alpha = dynBody->m_alpha & accelTest;

					dUnsigned32 equilibrium = (dynBody->m_invMass.m_w == dFloat32(0.0f)) ? 1 : dynBody->m_autoSleep;
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
					ndBodyKinematic* const kinBody = dynBody->GetAsBodyKinematic();
					dAssert(kinBody);
					dUnsigned32 equilibrium = (kinBody->m_invMass.m_w == dFloat32(0.0f)) ? 1 : (kinBody->m_autoSleep & ~kinBody->m_equilibriumOverride);
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
				for (dInt32 i = 0; i < count; i++)
				{
					// force entire island to equilibriumTest
					dInt32 index = bodyIslandOrder[start + i];
					ndBodyDynamic* const body = bodyIslands[index]->GetAsBodyDynamic();

					if (body)
					{
						body->m_accel = dVector::m_zero;
						body->m_alpha = dVector::m_zero;
						body->m_veloc = dVector::m_zero;
						body->m_omega = dVector::m_zero;
						body->m_equilibrium = (body->m_invMass.m_w == dFloat32(0.0f)) ? 1 : body->m_autoSleep;
					}
					else
					{
						ndBodyKinematic* const kinBody = body->GetAsBodyKinematic();
						dAssert(kinBody);
						kinBody->m_veloc = dVector::m_zero;
						kinBody->m_omega = dVector::m_zero;
						kinBody->m_equilibrium = (kinBody->m_invMass.m_w == dFloat32(0.0f)) ? 1 : kinBody->m_autoSleep;
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
					for (dInt32 i = 0; i < count; i++)
					{
						dInt32 index = bodyIslandOrder[start + i];
						ndBodyDynamic* const body = bodyIslands[index]->GetAsBodyDynamic();
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
							dInt32 index = bodyIslandOrder[start + i];
							ndBodyKinematic* const body = bodyIslands[index];
							body->m_equilibrium = 0;
						}
					}
					dInt32 timeScaleSleepCount = dInt32(dFloat32(60.0f) * sleepCounter * m_timestep);

					dInt32 index = D_SLEEP_ENTRIES;
					for (dInt32 i = 1; i < D_SLEEP_ENTRIES; i++)
					{
						if (world->m_sleepTable[i].m_steps > timeScaleSleepCount)
						{
							index = i;
							break;
						}
					}
					index--;

					bool state1 =
						(maxAccel < world->m_sleepTable[index].m_maxAccel) &&
						(maxAlpha < world->m_sleepTable[index].m_maxAlpha) &&
						(maxSpeed < world->m_sleepTable[index].m_maxVeloc) &&
						(maxOmega < world->m_sleepTable[index].m_maxOmega);
					if (state1)
					{
						for (dInt32 i = 0; i < count; i++)
						{
							dInt32 index1 = bodyIslandOrder[start + i];
							ndBodyKinematic* const body = bodyIslands[index1];
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

		virtual void Execute()
		{
			D_TRACKTIME();
			ndWorld* const world = m_owner->GetWorld();
			ndDynamicsUpdate* const me = world->m_solver;
			const dArray<ndIsland>& islandArray = me->GetIsland____();

			const dInt32 threadIndex = GetThreadId();
			const dInt32 threadCount = m_owner->GetThreadCount();
			const dInt32 islandCount = islandArray.GetCount();

			for (dInt32 i = threadIndex; i < islandCount; i += threadCount)
			{
				UpdateIslandState(i);
			}
		}

		dVector m_velocTol;
	};
#else

	class ndDetermineSleepStates : public ndScene::ndBaseJob
	{
		public:
		ndDetermineSleepStates()
		{
		}

		virtual void Execute()
		{
			D_TRACKTIME();
			ndWorld* const world = m_owner->GetWorld();
			ndDynamicsUpdate* const me = world->m_solver;
			const dArray<dInt32>& activeBodyIndexArray = me->GetActiveBodies();
			const dArray<ndBodyKinematic*>& bodyArray = m_owner->GetActiveBodyArray();
			//const ndJointBodyPairIndex* const jointBodyPairIndexBuffer = &me->GetJointBodyPairIndexBuffer()[0];

			const dVector velocTol(dFloat32(1.0e-8f));
			const dVector velocSingleDrag(dFloat32(0.9999f));
			const dVector maxSingleAccNorm2(D_SOLVER_MAX_ERROR * D_SOLVER_MAX_ERROR * dFloat32(0.0625f));
			const dFloat32 speedSingleFreeze = world->m_freezeSpeed2;
			const dFloat32 accelSingleFreeze = world->m_freezeAccel2 * dFloat32(0.01f);

			const dInt32 bodyCount = activeBodyIndexArray.GetCount();
			const dInt32 threadIndex = GetThreadId();
			const dInt32 threadCount = m_owner->GetThreadCount();
			
			const dInt32 stride = bodyCount / threadCount;
			const dInt32 start = threadIndex * stride;
			const dInt32 blockSize = (threadIndex != (threadCount - 1)) ? stride : bodyCount - start;
			
			for (dInt32 i = 0; i < blockSize; i++)
			{
				const dInt32 index = activeBodyIndexArray[i + start];
				ndBodyDynamic* const body = bodyArray[index]->GetAsBodyDynamic();

				const dVector accelTest(
					(body->m_accel.DotProduct(body->m_accel) > maxSingleAccNorm2) |
					(body->m_alpha.DotProduct(body->m_alpha) > maxSingleAccNorm2));
				body->m_accel = body->m_accel & accelTest;
				body->m_alpha = body->m_alpha & accelTest;

				const dUnsigned32 equilibrium = (body->GetInvMass() == dFloat32(0.0f)) ? 1 : body->m_autoSleep;
				const dVector isMovingMask(body->m_veloc + body->m_omega + body->m_accel + body->m_alpha);
				const dVector mask(isMovingMask.TestZero());
				const dInt32 test = mask.GetSignMask() & 7;
				if (test != 7)
				{
					const dFloat32 accel2 = body->m_accel.DotProduct(body->m_accel).GetScalar();
					const dFloat32 alpha2 = body->m_alpha.DotProduct(body->m_alpha).GetScalar();
					const dFloat32 speed2 = body->m_veloc.DotProduct(body->m_veloc).GetScalar();
					const dFloat32 omega2 = body->m_omega.DotProduct(body->m_omega).GetScalar();

					const dUnsigned32 equilibriumTest = (accel2 < accelSingleFreeze) && (alpha2 < accelSingleFreeze) && (speed2 < speedSingleFreeze) && (omega2 < speedSingleFreeze);

					if (equilibriumTest)
					{
						const dVector veloc(body->m_veloc * velocSingleDrag);
						const dVector omega(body->m_omega * velocSingleDrag);
						const dVector velocMask(veloc.DotProduct(veloc) > velocTol);
						const dVector omegaMask(omega.DotProduct(omega) > velocTol);
						body->m_veloc = velocMask & veloc;
						body->m_omega = omegaMask & omega;
					}
					body->m_equilibrium = equilibriumTest & body->m_autoSleep;
				}
				else
				{
					body->m_equilibrium = equilibrium & body->m_autoSleep;
				}
			}
		}
	};

#endif

	ndScene* const scene = m_world->GetScene();
	scene->SubmitJobs<ndDetermineSleepStates>();
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
			const dInt32 threadIndex = GetThreadId();
			ndWorld* const world = m_owner->GetWorld();
			ndDynamicsUpdate* const me = world->m_solver;
			ndSkeletonList::dNode* node = world->GetSkeletonList().GetFirst();
			for (dInt32 i = 0; i < threadIndex; i++)
			{
				node = node ? node->GetNext() : nullptr;
			}

			dArray<ndRightHandSide>& rightHandSide = me->m_rightHandSide;
			const dArray<ndLeftHandSide>& leftHandSide = me->m_leftHandSide;

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
			const dInt32 threadIndex = GetThreadId();
			ndWorld* const world = m_owner->GetWorld();
			ndDynamicsUpdate* const me = world->m_solver;
			ndSkeletonList::dNode* node = world->GetSkeletonList().GetFirst();
			for (dInt32 i = 0; i < threadIndex; i++)
			{
				node = node ? node->GetNext() : nullptr;
			}

			ndJacobian* const internalForces = &me->GetInternalForces()[0];
			const dArray<ndBodyKinematic*>& activeBodies = m_owner->ndScene::GetActiveBodyArray();
			const ndBodyKinematic** const bodyArray = (const ndBodyKinematic**)&activeBodies[0];

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

void ndDynamicsUpdate::CalculateJointsForce()
{
	D_TRACKTIME();
	class ndCalculateJointsForce : public ndScene::ndBaseJob
	{
		public:
		ndCalculateJointsForce()
			:m_zero(dVector::m_zero)
		{
		}
		
		dFloat32 JointForce(ndConstraint* const joint, dInt32 jointIndex)
		{
			dVector accNorm(m_zero);
			ndBodyKinematic* const body0 = joint->GetBody0();
			ndBodyKinematic* const body1 = joint->GetBody1();
			dAssert(body0);
			dAssert(body1);

			const dInt32 m0 = body0->m_index;
			const dInt32 m1 = body1->m_index;
			const dInt32 rowStart = joint->m_rowStart;
			const dInt32 rowsCount = joint->m_rowCount;

			const dInt32 resting = body0->m_resting & body1->m_resting;
			if (!resting)
			{
				dVector preconditioner0(joint->m_preconditioner0);
				dVector preconditioner1(joint->m_preconditioner1);

				dVector forceM0(m_internalForces[m0].m_linear * preconditioner0);
				dVector torqueM0(m_internalForces[m0].m_angular * preconditioner0);
				dVector forceM1(m_internalForces[m1].m_linear * preconditioner1);
				dVector torqueM1(m_internalForces[m1].m_angular * preconditioner1);

				preconditioner0 = preconditioner0.Scale(body0->m_weigh);
				preconditioner1 = preconditioner1.Scale(body1->m_weigh);

				for (dInt32 j = 0; j < rowsCount; j++)
				{
					ndRightHandSide* const rhs = &m_rightHandSide[rowStart + j];
					const ndLeftHandSide* const lhs = &m_leftHandSide[rowStart + j];
					const dVector force(rhs->m_force);

					dVector a(lhs->m_JMinv.m_jacobianM0.m_linear * forceM0);
					a = a.MulAdd(lhs->m_JMinv.m_jacobianM0.m_angular, torqueM0);
					a = a.MulAdd(lhs->m_JMinv.m_jacobianM1.m_linear, forceM1);
					a = a.MulAdd(lhs->m_JMinv.m_jacobianM1.m_angular, torqueM1);
					a = dVector(rhs->m_coordenateAccel - rhs->m_force * rhs->m_diagDamp) - a.AddHorizontal();

					dAssert(rhs->m_normalForceIndexFlat >= 0);
					dVector f(force + a.Scale(rhs->m_invJinvMJt));
					const dInt32 frictionIndex = rhs->m_normalForceIndexFlat;
					const dFloat32 frictionNormal = m_rightHandSide[frictionIndex].m_force;
					const dVector lowerFrictionForce(frictionNormal * rhs->m_lowerBoundFrictionCoefficent);
					const dVector upperFrictionForce(frictionNormal * rhs->m_upperBoundFrictionCoefficent);
					a = a & (f < upperFrictionForce) & (f > lowerFrictionForce);
					f = f.GetMax(lowerFrictionForce).GetMin(upperFrictionForce);
					accNorm = accNorm.MulAdd(a, a);
					rhs->m_force = f.GetScalar();

					const dVector deltaForce(f - force);
					const dVector deltaForce0(deltaForce * preconditioner0);
					const dVector deltaForce1(deltaForce * preconditioner1);

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
					maxAccel = m_zero;
					for (dInt32 j = 0; j < rowsCount; j++)
					{
						ndRightHandSide* const rhs = &m_rightHandSide[rowStart + j];
						const ndLeftHandSide* const lhs = &m_leftHandSide[rowStart + j];
						const dVector force(rhs->m_force);

						dVector a(lhs->m_JMinv.m_jacobianM0.m_linear * forceM0);
						a = a.MulAdd(lhs->m_JMinv.m_jacobianM0.m_angular, torqueM0);
						a = a.MulAdd(lhs->m_JMinv.m_jacobianM1.m_linear, forceM1);
						a = a.MulAdd(lhs->m_JMinv.m_jacobianM1.m_angular, torqueM1);
						a = dVector(rhs->m_coordenateAccel - rhs->m_force * rhs->m_diagDamp) - a.AddHorizontal();
							
						dVector f(force + a.Scale(rhs->m_invJinvMJt));
						dAssert(rhs->m_normalForceIndexFlat >= 0);
						const dInt32 frictionIndex = rhs->m_normalForceIndexFlat;
						const dFloat32 frictionNormal = m_rightHandSide[frictionIndex].m_force;

						const dVector lowerFrictionForce(frictionNormal * rhs->m_lowerBoundFrictionCoefficent);
						const dVector upperFrictionForce(frictionNormal * rhs->m_upperBoundFrictionCoefficent);

						a = a & (f < upperFrictionForce) & (f > lowerFrictionForce);
						f = f.GetMax(lowerFrictionForce).GetMin(upperFrictionForce);
						maxAccel = maxAccel.MulAdd(a, a);
						rhs->m_force = f.GetScalar();

						const dVector deltaForce(f - force);
						const dVector deltaForce0(deltaForce * preconditioner0);
						const dVector deltaForce1(deltaForce * preconditioner1);
						forceM0 = forceM0.MulAdd(lhs->m_Jt.m_jacobianM0.m_linear, deltaForce0);
						torqueM0 = torqueM0.MulAdd(lhs->m_Jt.m_jacobianM0.m_angular, deltaForce0);
						forceM1 = forceM1.MulAdd(lhs->m_Jt.m_jacobianM1.m_linear, deltaForce1);
						torqueM1 = torqueM1.MulAdd(lhs->m_Jt.m_jacobianM1.m_angular, deltaForce1);
					}
				}
			}

			dVector forceM0(m_zero);
			dVector torqueM0(m_zero);
			dVector forceM1(m_zero);
			dVector torqueM1(m_zero);

			for (dInt32 j = 0; j < rowsCount; j++)
			{
				ndRightHandSide* const rhs = &m_rightHandSide[rowStart + j];
				const ndLeftHandSide* const lhs = &m_leftHandSide[rowStart + j];

				const dVector f(rhs->m_force);
				forceM0 = forceM0.MulAdd(lhs->m_Jt.m_jacobianM0.m_linear, f);
				torqueM0 = torqueM0.MulAdd(lhs->m_Jt.m_jacobianM0.m_angular, f);
				forceM1 = forceM1.MulAdd(lhs->m_Jt.m_jacobianM1.m_linear, f);
				torqueM1 = torqueM1.MulAdd(lhs->m_Jt.m_jacobianM1.m_angular, f);
				rhs->m_maxImpact = dMax(dAbs(f.GetScalar()), rhs->m_maxImpact);
			}

			const dInt32 index0 = jointIndex * 2 + 0;
			ndJacobian& outBody0 = m_jointPartialForces[index0];
			outBody0.m_linear = forceM0;
			outBody0.m_angular = torqueM0;

			const dInt32 index1 = jointIndex * 2 + 1;
			ndJacobian& outBody1 = m_jointPartialForces[index1];
			outBody1.m_linear = forceM1;
			outBody1.m_angular = torqueM1;

			return accNorm.GetScalar();
		}

		virtual void Execute()
		{
			D_TRACKTIME();
			ndWorld* const world = m_owner->GetWorld();
			ndDynamicsUpdate* const me = world->m_solver;
			m_leftHandSide = &me->GetLeftHandSide()[0];
			m_rightHandSide = &me->GetRightHandSide()[0];
			m_internalForces = &me->GetInternalForces()[0];
			m_jointPartialForces = &me->GetTempInternalForces()[0];
			m_jointBodyPairIndexBuffer = &me->GetJointBodyPairIndexBuffer()[0];
			ndConstraintArray& jointArray = m_owner->GetActiveContactArray();

			dFloat32 accNorm = dFloat32(0.0f);
			const dInt32 jointCount = jointArray.GetCount();
			const dInt32 bodyCount = m_owner->GetActiveBodyArray().GetCount();
			const dInt32 threadIndex = GetThreadId();
			const dInt32 threadCount = m_owner->GetThreadCount();
		
			for (dInt32 i = threadIndex; i < jointCount; i += threadCount)
			{
				ndConstraint* const joint = jointArray[i];
				accNorm += JointForce(joint, i);
			}

			dFloat32* const accelNorm = (dFloat32*)m_context;
			accelNorm[threadIndex] = accNorm;
		}

		dVector m_zero;
		ndJacobian* m_jointPartialForces;
		ndRightHandSide* m_rightHandSide;
		const ndJacobian* m_internalForces;
		const ndLeftHandSide* m_leftHandSide;
		const ndJointBodyPairIndex* m_jointBodyPairIndexBuffer;
	};

	class ndApplyJacobianAccumulatePartialForces : public ndScene::ndBaseJob
	{
		public:
		virtual void Execute()
		{
			D_TRACKTIME();
			const dVector zero(dVector::m_zero);
			ndWorld* const world = m_owner->GetWorld();
			ndDynamicsUpdate* const me = (ndDynamicsUpdate*)world->m_solver;

			ndJacobian* const internalForces = &me->GetInternalForces()[0];
			const dInt32* const bodyIndex = &me->GetJointForceIndexBuffer()[0];
			const dArray<ndBodyKinematic*>& bodyArray = m_owner->GetActiveBodyArray();
			const ndJacobian* const jointInternalForces = &me->GetTempInternalForces()[0];
			const ndJointBodyPairIndex* const jointBodyPairIndexBuffer = &me->GetJointBodyPairIndexBuffer()[0];

			const dInt32 bodyCount = bodyArray.GetCount();
			const dInt32 threadIndex = GetThreadId();
			const dInt32 threadCount = m_owner->GetThreadCount();

			const dInt32 stride = bodyCount / threadCount;
			const dInt32 start = threadIndex * stride;
			const dInt32 blockSize = (threadIndex != (threadCount - 1)) ? stride : bodyCount - start;

			for (dInt32 i = 0; i < blockSize; i++)
			{
				dInt32 startIndex = bodyIndex[start + i];
				dInt32 count = bodyIndex[start + i + 1] - startIndex;
				if (count)
				{
					const ndBodyKinematic* const body = bodyArray[i + start];
					if (body->m_invMass.m_w > dFloat32(0.0f))
					{
						dVector force(zero);
						dVector torque(zero);
						for (dInt32 j = 0; j < count; j++)
						{
							dInt32 index = jointBodyPairIndexBuffer[startIndex + j].m_joint;
							force += jointInternalForces[index].m_linear;
							torque += jointInternalForces[index].m_angular;
						}

						internalForces[i + start].m_linear = force;
						internalForces[i + start].m_angular = torque;
					}
				}
			}
		}
	};

	ndScene* const scene = m_world->GetScene();
	const dInt32 passes = m_solverPasses;
	const dInt32 threadsCount = scene->GetThreadCount();

	dFloat32 m_accelNorm[D_MAX_THREADS_COUNT];
	dFloat32 accNorm = D_SOLVER_MAX_ERROR * dFloat32(2.0f);

	for (dInt32 i = 0; (i < passes) && (accNorm > D_SOLVER_MAX_ERROR); i++)
	{
		scene->SubmitJobs<ndCalculateJointsForce>(m_accelNorm);
		scene->SubmitJobs<ndApplyJacobianAccumulatePartialForces>();

		accNorm = dFloat32(0.0f);
		for (dInt32 j = 0; j < threadsCount; j++)
		{
			accNorm = dMax(accNorm, m_accelNorm[j]);
		}
	}
}

void ndDynamicsUpdate::CalculateForces()
{
	D_TRACKTIME();
	if (m_world->GetScene()->GetActiveContactArray().GetCount())
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
	}
}

void ndDynamicsUpdate::Update()
{
	D_TRACKTIME();
	m_timestep = m_world->GetScene()->GetTimestep();

	BuildIsland();
	dInt32 count = GetActiveBodies().GetCount();
	if (count)
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
