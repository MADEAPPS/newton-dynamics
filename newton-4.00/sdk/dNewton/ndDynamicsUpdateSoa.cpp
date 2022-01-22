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
#include "ndDynamicsUpdateSoa.h"
#include "ndJointBilateralConstraint.h"

#define D_SSE_WORK_GROUP			4 
#define D_SSE_DEFAULT_BUFFER_SIZE	1024
using namespace ndSoa;

ndDynamicsUpdateSoa::ndDynamicsUpdateSoa(ndWorld* const world)
	:ndDynamicsUpdate(world)
	,m_ordinals(0, 1, 2, 3)
	,m_groupType(D_SSE_DEFAULT_BUFFER_SIZE)
	,m_jointMask(D_SSE_DEFAULT_BUFFER_SIZE)
	,m_soaJointRows(D_SSE_DEFAULT_BUFFER_SIZE)
	,m_soaMassMatrix(D_SSE_DEFAULT_BUFFER_SIZE * 4)
{
}

ndDynamicsUpdateSoa::~ndDynamicsUpdateSoa()
{
	Clear();

	m_jointMask.Resize(D_SSE_DEFAULT_BUFFER_SIZE);
	m_groupType.Resize(D_SSE_DEFAULT_BUFFER_SIZE);
	m_soaJointRows.Resize(D_SSE_DEFAULT_BUFFER_SIZE);
	m_soaMassMatrix.Resize(D_SSE_DEFAULT_BUFFER_SIZE * 4);
}

const char* ndDynamicsUpdateSoa::GetStringId() const
{
	return "sse soa";
}

void ndDynamicsUpdateSoa::DetermineSleepStates()
{
	D_TRACKTIME();
	ndScene* const scene = m_world->GetScene();
	ndFloat32 timestep = scene->GetTimestep();
	auto DetermineSleepStates = ndMakeObject::ndFunction([this, timestep](ndInt32 threadIndex, ndInt32 threadCount)
	{
		D_TRACKTIME();
		auto UpdateIslandState = [this, timestep](const ndIsland& island)
		{
			const ndWorld* const world = m_world;
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
					ndInt32 timeScaleSleepCount = ndInt32(ndFloat32(60.0f) * sleepCounter * timestep);

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
		};

		const ndArray<ndIsland>& islandArray = GetIslands();
		const ndInt32 islandCount = islandArray.GetCount();
		for (ndInt32 i = threadIndex; i < islandCount; i += threadCount)
		{
			const ndIsland& island = islandArray[i];
			UpdateIslandState(island);
		}
	});
	scene->ParallelExecute(DetermineSleepStates);
}

void ndDynamicsUpdateSoa::SortJoints()
{
	D_TRACKTIME();

	SortJointsScan();
	if (!m_activeJointCount)
	{
		return;
	}

	ndScene* const scene = m_world->GetScene();
	ndArray<ndConstraint*>& jointArray = scene->GetActiveContactArray();

	const ndInt32 mask = -ndInt32(D_SSE_WORK_GROUP);
	const ndInt32 jointCount = jointArray.GetCount();
	const ndInt32 soaJointCount = (jointCount + D_SSE_WORK_GROUP - 1) & mask;
	dAssert(jointArray.GetCapacity() > soaJointCount);
	ndConstraint** const jointArrayPtr = &jointArray[0];
	for (ndInt32 i = jointCount; i < soaJointCount; ++i)
	{
		jointArrayPtr[i] = nullptr;
	}
	
	if (m_activeJointCount - jointArray.GetCount())
	{
		const ndInt32 base = m_activeJointCount & mask;
		const ndInt32 count = jointArrayPtr[base + D_SSE_WORK_GROUP - 1] ? D_SSE_WORK_GROUP : jointArray.GetCount() - base;
		dAssert(count <= D_SSE_WORK_GROUP);
		ndConstraint** const array = &jointArrayPtr[base];
		for (ndInt32 j = 1; j < count; ++j)
		{
			ndInt32 slot = j;
			ndConstraint* const joint = array[slot];
			for (; (slot > 0) && (array[slot - 1]->m_rowCount < joint->m_rowCount); slot--)
			{
				array[slot] = array[slot - 1];
			}
			array[slot] = joint;
		}
	}
	
	const ndInt32 soaJointCountBatches = soaJointCount / D_SSE_WORK_GROUP;
	m_jointMask.SetCount(soaJointCountBatches);
	m_groupType.SetCount(soaJointCountBatches);
	m_soaJointRows.SetCount(soaJointCountBatches);

	ndInt32 rowsCount = 0;
	ndInt32 soaJointRowCount = 0;
	auto SetRowStarts = ndMakeObject::ndFunction([this, &jointArray, &rowsCount, &soaJointRowCount](ndInt32 threadIndex, ndInt32 threadCount)
	{
		D_TRACKTIME();
		auto SetRowsCount = [this, &jointArray, &rowsCount]()
		{
			ndInt32 rowCount = 1;
			const ndInt32 count = jointArray.GetCount();
			for (ndInt32 i = 0; i < count; ++i)
			{
				ndConstraint* const joint = jointArray[i];
				joint->m_rowStart = rowCount;
				rowCount += joint->m_rowCount;
			}
			rowsCount = rowCount;
		};

		auto SetSoaRowsCount = [this, &jointArray, &soaJointRowCount]()
		{
			ndInt32 rowCount = 0;
			ndArray<ndInt32>& soaJointRows = m_soaJointRows;
			const ndInt32 count = soaJointRows.GetCount();
			for (ndInt32 i = 0; i < count; ++i)
			{
				const ndConstraint* const joint = jointArray[i * D_SSE_WORK_GROUP];
				soaJointRows[i] = rowCount;
				rowCount += joint->m_rowCount;
			}
			soaJointRowCount = rowCount;
		};

		if (threadCount == 1)
		{
			SetRowsCount();
			SetSoaRowsCount();
		}
		else if (threadIndex == 0)
		{
			SetRowsCount();
		}
		else if (threadIndex == (threadCount - 1))
		{
			SetSoaRowsCount();
		}
	});

	scene->ParallelExecute(SetRowStarts);
	m_leftHandSide.SetCount(rowsCount);
	m_rightHandSide.SetCount(rowsCount);
	m_soaMassMatrix.SetCount(soaJointRowCount);

	#ifdef _DEBUG
		dAssert(m_activeJointCount <= jointArray.GetCount());
		const ndInt32 maxRowCount = m_leftHandSide.GetCount();
		for (ndInt32 i = 0; i < jointArray.GetCount(); ++i)
		{
			ndConstraint* const joint = jointArray[i];
			dAssert(joint->m_rowStart < m_leftHandSide.GetCount());
			dAssert((joint->m_rowStart + joint->m_rowCount) <= maxRowCount);
		}
		
		for (ndInt32 i = 0; i < jointCount; i += D_SSE_WORK_GROUP)
		{
			const ndInt32 count = jointArrayPtr[i + D_SSE_WORK_GROUP - 1] ? D_SSE_WORK_GROUP : jointCount - i;
			for (ndInt32 j = 1; j < count; ++j)
			{
				ndConstraint* const joint0 = jointArrayPtr[i + j - 1];
				ndConstraint* const joint1 = jointArrayPtr[i + j - 0];
				dAssert(joint0->m_rowCount >= joint1->m_rowCount);
			}
		}
	#endif

	SortBodyJointScan();
}

void ndDynamicsUpdateSoa::SortIslands()
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
			return lowKey & ((1 << D_MAX_BODY_RADIX_BIT) - 1);
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

void ndDynamicsUpdateSoa::BuildIsland()
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

void ndDynamicsUpdateSoa::IntegrateUnconstrainedBodies()
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
		scene->ParallelExecute(IntegrateUnconstrainedBodies);
	}
}


void ndDynamicsUpdateSoa::InitWeights()
{
	D_TRACKTIME();
	ndScene* const scene = m_world->GetScene();
	m_invTimestep = ndFloat32(1.0f) / m_timestep;
	m_invStepRK = ndFloat32(0.25f);
	m_timestepRK = m_timestep * m_invStepRK;
	m_invTimestepRK = m_invTimestep * ndFloat32(4.0f);

	const ndArray<ndBodyKinematic*>& bodyArray = scene->GetActiveBodyArray();
	const ndInt32 bodyCount = bodyArray.GetCount();
	GetInternalForces().SetCount(bodyCount);

	ndInt32 extraPassesArray[D_MAX_THREADS_COUNT];

	auto InitWeights = ndMakeObject::ndFunction([this, &bodyArray, &extraPassesArray](ndInt32 threadIndex, ndInt32 threadCount)
	{
		D_TRACKTIME();
		const ndArray<ndInt32>& jointForceIndexBuffer = GetJointForceIndexBuffer();
		const ndArray<ndJointBodyPairIndex>& jointBodyPairIndex = GetJointBodyPairIndexBuffer();

		ndInt32 maxExtraPasses = 1;
		const ndStartEnd startEnd(jointForceIndexBuffer.GetCount() - 1, threadIndex, threadCount);
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
		extraPassesArray[threadIndex] = maxExtraPasses;
	});
	scene->ParallelExecute(InitWeights);

	ndInt32 extraPasses = 0;
	const ndInt32 threadCount = scene->GetThreadCount();
	for (ndInt32 i = 0; i < threadCount; ++i)
	{
		extraPasses = dMax(extraPasses, extraPassesArray[i]);
	}

	const ndInt32 conectivity = 7;
	m_solverPasses = m_world->GetSolverIterations() + 2 * extraPasses / conectivity + 1;
}

void ndDynamicsUpdateSoa::IntegrateBodies()
{
	D_TRACKTIME();
	ndScene* const scene = m_world->GetScene();
	const ndVector invTime(m_invTimestep);
	const ndFloat32 timestep = scene->GetTimestep();

	auto IntegrateBodies = ndMakeObject::ndFunction([this, timestep, invTime](ndInt32 threadIndex, ndInt32 threadCount)
	{
		D_TRACKTIME();
		const ndArray<ndBodyKinematic*>& bodyArray = GetBodyIslandOrder();
		const ndStartEnd startEnd(bodyArray.GetCount(), threadIndex, threadCount);
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
	});
	scene->ParallelExecute(IntegrateBodies);
}

void ndDynamicsUpdateSoa::InitBodyArray()
{
	D_TRACKTIME();
	ndScene* const scene = m_world->GetScene();
	const ndFloat32 timestep = scene->GetTimestep();

	auto InitBodyArray = ndMakeObject::ndFunction([this, timestep](ndInt32 threadIndex, ndInt32 threadCount)
	{
		D_TRACKTIME();
		const ndArray<ndBodyKinematic*>& bodyArray = GetBodyIslandOrder();
		const ndStartEnd startEnd(bodyArray.GetCount() - GetUnconstrainedBodyCount(), threadIndex, threadCount);
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
	});
	scene->ParallelExecute(InitBodyArray);
}

void ndDynamicsUpdateSoa::GetJacobianDerivatives(ndConstraint* const joint)
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
		rhs->m_normalForceIndex = constraintParam.m_forceBounds[i].m_normalIndex;
	}
}

void ndDynamicsUpdateSoa::InitJacobianMatrix()
{
	ndScene* const scene = m_world->GetScene();
	ndBodyKinematic** const bodyArray = &scene->GetActiveBodyArray()[0];
	ndArray<ndConstraint*>& jointArray = scene->GetActiveContactArray();

	auto InitJacobianMatrix = ndMakeObject::ndFunction([this, &jointArray](ndInt32 threadIndex, ndInt32 threadCount)
	{
		D_TRACKTIME();
		ndJacobian* const internalForces = &GetTempInternalForces()[0];
		auto BuildJacobianMatrix = [this, &internalForces](ndConstraint* const joint, ndInt32 jointIndex)
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

			const ndVector zero(ndVector::m_zero);
			ndVector forceAcc0(zero);
			ndVector torqueAcc0(zero);
			ndVector forceAcc1(zero);
			ndVector torqueAcc1(zero);

			#ifdef D_PROGRESSIVE_SLEEP_EXPERIMENT
			const ndVector progressiveSleepWeigh(ndFloat32(0.01f));
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
			ndJacobian& outBody0 = internalForces[index0];
			outBody0.m_linear = forceAcc0;
			outBody0.m_angular = torqueAcc0;

			const ndInt32 index1 = jointIndex * 2 + 1;
			ndJacobian& outBody1 = internalForces[index1];
			outBody1.m_linear = forceAcc1;
			outBody1.m_angular = torqueAcc1;
		};

		const ndInt32 jointCount = jointArray.GetCount();
		for (ndInt32 i = threadIndex; i < jointCount; i += threadCount)
		{
			ndConstraint* const joint = jointArray[i];
			GetJacobianDerivatives(joint);
			BuildJacobianMatrix(joint, i);
		}
	});

	auto InitJacobianAccumulatePartialForces = ndMakeObject::ndFunction([this, &bodyArray](ndInt32 threadIndex, ndInt32 threadCount)
	{
		D_TRACKTIME();
		const ndVector zero(ndVector::m_zero);
		ndJacobian* const internalForces = &GetInternalForces()[0];
		const ndArray<ndInt32>& bodyIndex = GetJointForceIndexBuffer();

		const ndJacobian* const jointInternalForces = &GetTempInternalForces()[0];
		const ndJointBodyPairIndex* const jointBodyPairIndexBuffer = &GetJointBodyPairIndexBuffer()[0];

		const ndStartEnd startEnd(bodyIndex.GetCount() - 1, threadIndex, threadCount);
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
	});
	
	auto TransposeMassMatrix = ndMakeObject::ndFunction([this, &jointArray](ndInt32 threadIndex, ndInt32 threadCount)
	{
		D_TRACKTIME();
		const ndInt32 jointCount = jointArray.GetCount();

		const ndLeftHandSide* const leftHandSide = &GetLeftHandSide()[0];
		const ndRightHandSide* const rightHandSide = &GetRightHandSide()[0];
		ndArray<ndSoa::ndSoaMatrixElement>& massMatrix = m_soaMassMatrix;

		const ndVector zero(ndVector::m_zero);
		const ndVector ordinals(m_ordinals);
		const ndInt32 mask = -ndInt32(D_SSE_WORK_GROUP);
		const ndInt32 soaJointCount = ((jointCount + D_SSE_WORK_GROUP - 1) & mask) / D_SSE_WORK_GROUP;

		ndInt8* const groupType = &m_groupType[0];
		ndVector* const jointMask = &m_jointMask[0];
		const ndInt32* const soaJointRows = &m_soaJointRows[0];

		ndConstraint** const jointsPtr = &jointArray[0];
		for (ndInt32 i = threadIndex; i < soaJointCount; i += threadCount)
		{
			const ndInt32 index = i * D_SSE_WORK_GROUP;
			ndInt32 maxRow = 0;
			ndInt32 minRow = 255;
			ndVector selectMask(ndVector::m_xyzwMask);
			for (ndInt32 j = 0; j < D_SSE_WORK_GROUP; ++j)
			{
				ndConstraint* const joint = jointsPtr[index + j];
				if (joint)
				{
					const ndInt32 maxMask = (maxRow - joint->m_rowCount) >> 8;
					const ndInt32 minMask = (minRow - joint->m_rowCount) >> 8;
					maxRow = maxMask & joint->m_rowCount | ~maxMask & maxRow;
					minRow = ~minMask & joint->m_rowCount | minMask & minRow;
					if (!joint->m_rowCount)
					{
						selectMask[j] = ndFloat32(0.0f);
					}
				}
				else
				{
					minRow = 0;
					selectMask[j] = ndFloat32(0.0f);
				}
			}
			dAssert(maxRow >= 0);
			dAssert(minRow < 255);
			jointMask[i] = selectMask;

			const ndInt8 isUniformGroup = (maxRow == minRow) & (maxRow > 0);
			groupType[i] = isUniformGroup;

			const ndInt32 soaRowBase = soaJointRows[i];
			if (isUniformGroup)
			{
				const ndConstraint* const joint0 = jointsPtr[index + 0];
				const ndConstraint* const joint1 = jointsPtr[index + 1];
				const ndConstraint* const joint2 = jointsPtr[index + 2];
				const ndConstraint* const joint3 = jointsPtr[index + 3];

				const ndInt32 rowCount = joint0->m_rowCount;
				for (ndInt32 j = 0; j < rowCount; ++j)
				{
					ndVector tmp;
					const ndLeftHandSide* const row0 = &leftHandSide[joint0->m_rowStart + j];
					const ndLeftHandSide* const row1 = &leftHandSide[joint1->m_rowStart + j];
					const ndLeftHandSide* const row2 = &leftHandSide[joint2->m_rowStart + j];
					const ndLeftHandSide* const row3 = &leftHandSide[joint3->m_rowStart + j];

					ndSoa::ndSoaMatrixElement& row = massMatrix[soaRowBase + j];
					ndVector::Transpose4x4(
						row.m_Jt.m_jacobianM0.m_linear.m_x,
						row.m_Jt.m_jacobianM0.m_linear.m_y,
						row.m_Jt.m_jacobianM0.m_linear.m_z,
						tmp,
						row0->m_Jt.m_jacobianM0.m_linear,
						row1->m_Jt.m_jacobianM0.m_linear,
						row2->m_Jt.m_jacobianM0.m_linear,
						row3->m_Jt.m_jacobianM0.m_linear);
					ndVector::Transpose4x4(
						row.m_Jt.m_jacobianM0.m_angular.m_x,
						row.m_Jt.m_jacobianM0.m_angular.m_y,
						row.m_Jt.m_jacobianM0.m_angular.m_z,
						tmp,
						row0->m_Jt.m_jacobianM0.m_angular,
						row1->m_Jt.m_jacobianM0.m_angular,
						row2->m_Jt.m_jacobianM0.m_angular,
						row3->m_Jt.m_jacobianM0.m_angular);

					ndVector::Transpose4x4(
						row.m_Jt.m_jacobianM1.m_linear.m_x,
						row.m_Jt.m_jacobianM1.m_linear.m_y,
						row.m_Jt.m_jacobianM1.m_linear.m_z,
						tmp,
						row0->m_Jt.m_jacobianM1.m_linear,
						row1->m_Jt.m_jacobianM1.m_linear,
						row2->m_Jt.m_jacobianM1.m_linear,
						row3->m_Jt.m_jacobianM1.m_linear);
					ndVector::Transpose4x4(
						row.m_Jt.m_jacobianM1.m_angular.m_x,
						row.m_Jt.m_jacobianM1.m_angular.m_y,
						row.m_Jt.m_jacobianM1.m_angular.m_z,
						tmp,
						row0->m_Jt.m_jacobianM1.m_angular,
						row1->m_Jt.m_jacobianM1.m_angular,
						row2->m_Jt.m_jacobianM1.m_angular,
						row3->m_Jt.m_jacobianM1.m_angular);

					ndVector::Transpose4x4(
						row.m_JMinv.m_jacobianM0.m_linear.m_x,
						row.m_JMinv.m_jacobianM0.m_linear.m_y,
						row.m_JMinv.m_jacobianM0.m_linear.m_z,
						tmp,
						row0->m_JMinv.m_jacobianM0.m_linear,
						row1->m_JMinv.m_jacobianM0.m_linear,
						row2->m_JMinv.m_jacobianM0.m_linear,
						row3->m_JMinv.m_jacobianM0.m_linear);
					ndVector::Transpose4x4(
						row.m_JMinv.m_jacobianM0.m_angular.m_x,
						row.m_JMinv.m_jacobianM0.m_angular.m_y,
						row.m_JMinv.m_jacobianM0.m_angular.m_z,
						tmp,
						row0->m_JMinv.m_jacobianM0.m_angular,
						row1->m_JMinv.m_jacobianM0.m_angular,
						row2->m_JMinv.m_jacobianM0.m_angular,
						row3->m_JMinv.m_jacobianM0.m_angular);

					ndVector::Transpose4x4(
						row.m_JMinv.m_jacobianM1.m_linear.m_x,
						row.m_JMinv.m_jacobianM1.m_linear.m_y,
						row.m_JMinv.m_jacobianM1.m_linear.m_z,
						tmp,
						row0->m_JMinv.m_jacobianM1.m_linear,
						row1->m_JMinv.m_jacobianM1.m_linear,
						row2->m_JMinv.m_jacobianM1.m_linear,
						row3->m_JMinv.m_jacobianM1.m_linear);
					ndVector::Transpose4x4(
						row.m_JMinv.m_jacobianM1.m_angular.m_x,
						row.m_JMinv.m_jacobianM1.m_angular.m_y,
						row.m_JMinv.m_jacobianM1.m_angular.m_z,
						tmp,
						row0->m_JMinv.m_jacobianM1.m_angular,
						row1->m_JMinv.m_jacobianM1.m_angular,
						row2->m_JMinv.m_jacobianM1.m_angular,
						row3->m_JMinv.m_jacobianM1.m_angular);

					#ifdef D_NEWTON_USE_DOUBLE
					ndInt64* const normalIndex = (ndInt64*)&row.m_normalForceIndex[0];
					#else
					ndInt32* const normalIndex = (ndInt32*)&row.m_normalForceIndex[0];
					#endif
					for (ndInt32 k = 0; k < D_SSE_WORK_GROUP; ++k)
					{
						const ndConstraint* const soaJoint = jointsPtr[index + k];
						const ndRightHandSide* const rhs = &rightHandSide[soaJoint->m_rowStart + j];
						row.m_force[k] = rhs->m_force;
						row.m_diagDamp[k] = rhs->m_diagDamp;
						row.m_invJinvMJt[k] = rhs->m_invJinvMJt;
						row.m_coordenateAccel[k] = rhs->m_coordenateAccel;
						normalIndex[k] = (rhs->m_normalForceIndex + 1) * D_SSE_WORK_GROUP + k;
						row.m_lowerBoundFrictionCoefficent[k] = rhs->m_lowerBoundFrictionCoefficent;
						row.m_upperBoundFrictionCoefficent[k] = rhs->m_upperBoundFrictionCoefficent;
					}
				}
			}
			else
			{
				for (ndInt32 j = 0; j < maxRow; ++j)
				{
					ndSoa::ndSoaMatrixElement& row = massMatrix[soaRowBase + j];
					row.m_Jt.m_jacobianM0.m_linear.m_x = zero;
					row.m_Jt.m_jacobianM0.m_linear.m_y = zero;
					row.m_Jt.m_jacobianM0.m_linear.m_z = zero;
					row.m_Jt.m_jacobianM0.m_angular.m_x = zero;
					row.m_Jt.m_jacobianM0.m_angular.m_y = zero;
					row.m_Jt.m_jacobianM0.m_angular.m_z = zero;
					row.m_Jt.m_jacobianM1.m_linear.m_x = zero;
					row.m_Jt.m_jacobianM1.m_linear.m_y = zero;
					row.m_Jt.m_jacobianM1.m_linear.m_z = zero;
					row.m_Jt.m_jacobianM1.m_angular.m_x = zero;
					row.m_Jt.m_jacobianM1.m_angular.m_y = zero;
					row.m_Jt.m_jacobianM1.m_angular.m_z = zero;

					row.m_JMinv.m_jacobianM0.m_linear.m_x = zero;
					row.m_JMinv.m_jacobianM0.m_linear.m_y = zero;
					row.m_JMinv.m_jacobianM0.m_linear.m_z = zero;
					row.m_JMinv.m_jacobianM0.m_angular.m_x = zero;
					row.m_JMinv.m_jacobianM0.m_angular.m_y = zero;
					row.m_JMinv.m_jacobianM0.m_angular.m_z = zero;
					row.m_JMinv.m_jacobianM1.m_linear.m_x = zero;
					row.m_JMinv.m_jacobianM1.m_linear.m_y = zero;
					row.m_JMinv.m_jacobianM1.m_linear.m_z = zero;
					row.m_JMinv.m_jacobianM1.m_angular.m_x = zero;
					row.m_JMinv.m_jacobianM1.m_angular.m_y = zero;
					row.m_JMinv.m_jacobianM1.m_angular.m_z = zero;

					row.m_force = zero;
					row.m_diagDamp = zero;
					row.m_invJinvMJt = zero;
					row.m_coordenateAccel = zero;
					row.m_normalForceIndex = ordinals;
					row.m_lowerBoundFrictionCoefficent = zero;
					row.m_upperBoundFrictionCoefficent = zero;
				}

				for (ndInt32 j = 0; j < D_SSE_WORK_GROUP; ++j)
				{
					const ndConstraint* const joint = jointsPtr[index + j];
					if (joint)
					{
						for (ndInt32 k = 0; k < joint->m_rowCount; ++k)
						{
							ndSoa::ndSoaMatrixElement& row = massMatrix[soaRowBase + k];
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

							#ifdef D_NEWTON_USE_DOUBLE
							ndInt64* const normalIndex = (ndInt64*)&row.m_normalForceIndex[0];
							#else
							ndInt32* const normalIndex = (ndInt32*)&row.m_normalForceIndex[0];
							#endif
							normalIndex[j] = (rhs->m_normalForceIndex + 1) * D_SSE_WORK_GROUP + j;
							row.m_lowerBoundFrictionCoefficent[j] = rhs->m_lowerBoundFrictionCoefficent;
							row.m_upperBoundFrictionCoefficent[j] = rhs->m_upperBoundFrictionCoefficent;
						}
					}
				}
			}
		}
	});

	if (scene->GetActiveContactArray().GetCount())
	{
		D_TRACKTIME();
		m_rightHandSide[0].m_force = ndFloat32(1.0f);

		scene->ParallelExecute(InitJacobianMatrix);
		scene->ParallelExecute(InitJacobianAccumulatePartialForces);
		scene->ParallelExecute(TransposeMassMatrix);
	}
}

void ndDynamicsUpdateSoa::UpdateForceFeedback()
{
	D_TRACKTIME();
	ndScene* const scene = m_world->GetScene();
	const ndArray<ndConstraint*>& jointArray = scene->GetActiveContactArray();

	auto UpdateForceFeedback = ndMakeObject::ndFunction([this, &jointArray](ndInt32 threadIndex, ndInt32 threadCount)
	{
		D_TRACKTIME();
		ndArray<ndRightHandSide>& rightHandSide = m_rightHandSide;
		const ndArray<ndLeftHandSide>& leftHandSide = m_leftHandSide;

		const ndVector zero(ndVector::m_zero);
		const ndFloat32 timestepRK = GetTimestepRK();
		const ndStartEnd startEnd(jointArray.GetCount(), threadIndex, threadCount);
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
				ndVector force0(zero);
				ndVector force1(zero);
				ndVector torque0(zero);
				ndVector torque1(zero);

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
	});

	scene->ParallelExecute(UpdateForceFeedback);
}

void ndDynamicsUpdateSoa::InitSkeletons()
{
	D_TRACKTIME();
	ndScene* const scene = m_world->GetScene();
	const ndArray<ndSkeletonContainer*>& activeSkeletons = m_world->m_activeSkeletons;

	auto InitSkeletons = ndMakeObject::ndFunction([this, &activeSkeletons](ndInt32 threadIndex, ndInt32 threadCount)
	{
		D_TRACKTIME();
		ndArray<ndRightHandSide>& rightHandSide = m_rightHandSide;
		const ndArray<ndLeftHandSide>& leftHandSide = m_leftHandSide;

		const ndStartEnd startEnd(activeSkeletons.GetCount(), threadIndex, threadCount);
		for (ndInt32 i = startEnd.m_start; i < startEnd.m_end; ++i)
		{
			ndSkeletonContainer* const skeleton = activeSkeletons[i];
			skeleton->InitMassMatrix(&leftHandSide[0], &rightHandSide[0]);
		}
	});

	if (activeSkeletons.GetCount())
	{
		scene->ParallelExecute(InitSkeletons);
	}
}

void ndDynamicsUpdateSoa::UpdateSkeletons()
{
	D_TRACKTIME();
	ndScene* const scene = m_world->GetScene();
	const ndArray<ndSkeletonContainer*>& activeSkeletons = m_world->m_activeSkeletons;
	const ndBodyKinematic** const bodyArray = (const ndBodyKinematic**)(&scene->GetActiveBodyArray()[0]);

	auto UpdateSkeletons = ndMakeObject::ndFunction([this, &bodyArray, &activeSkeletons](ndInt32 threadIndex, ndInt32 threadCount)
	{
		D_TRACKTIME();
		ndJacobian* const internalForces = &GetInternalForces()[0];
		const ndStartEnd startEnd(activeSkeletons.GetCount(), threadIndex, threadCount);
		for (ndInt32 i = startEnd.m_start; i < startEnd.m_end; ++i)
		{
			ndSkeletonContainer* const skeleton = activeSkeletons[i];
			skeleton->CalculateJointForce(bodyArray, internalForces);
		}
	});

	if (activeSkeletons.GetCount())
	{
		scene->ParallelExecute(UpdateSkeletons);
	}
}

void ndDynamicsUpdateSoa::CalculateJointsAcceleration()
{
	D_TRACKTIME();
	ndScene* const scene = m_world->GetScene();
	const ndArray<ndConstraint*>& jointArray = scene->GetActiveContactArray();

	auto CalculateJointsAcceleration = ndMakeObject::ndFunction([this, &jointArray](ndInt32 threadIndex, ndInt32 threadCount)
	{
		D_TRACKTIME();
		ndJointAccelerationDecriptor joindDesc;
		joindDesc.m_timestep = m_timestepRK;
		joindDesc.m_invTimestep = m_invTimestepRK;
		joindDesc.m_firstPassCoefFlag = m_firstPassCoef;
		ndArray<ndLeftHandSide>& leftHandSide = m_leftHandSide;
		ndArray<ndRightHandSide>& rightHandSide = m_rightHandSide;

		const ndStartEnd startEnd(jointArray.GetCount(), threadIndex, threadCount);
		for (ndInt32 i = startEnd.m_start; i < startEnd.m_end; ++i)
		{
			ndConstraint* const joint = jointArray[i];
			const ndInt32 pairStart = joint->m_rowStart;
			joindDesc.m_rowsCount = joint->m_rowCount;
			joindDesc.m_leftHandSide = &leftHandSide[pairStart];
			joindDesc.m_rightHandSide = &rightHandSide[pairStart];
			joint->JointAccelerations(&joindDesc);
		}
	});

	auto UpdateAcceleration = ndMakeObject::ndFunction([this, &jointArray](ndInt32 threadIndex, ndInt32 threadCount)
	{
		D_TRACKTIME();
		const ndArray<ndRightHandSide>& rightHandSide = m_rightHandSide;

		const ndInt32 jointCount = jointArray.GetCount();
		const ndInt32 mask = -ndInt32(D_SSE_WORK_GROUP);
		const ndInt32* const soaJointRows = &m_soaJointRows[0];
		const ndInt32 soaJointCountBatches = ((jointCount + D_SSE_WORK_GROUP - 1) & mask) / D_SSE_WORK_GROUP;
		const ndInt8* const groupType = &m_groupType[0];

		const ndConstraint* const * jointArrayPtr = &jointArray[0];
		ndSoaMatrixElement* const massMatrix = &m_soaMassMatrix[0];
		for (ndInt32 i = threadIndex; i < soaJointCountBatches; i += threadCount)
		{
			if (groupType[i])
			{
				const ndInt32 soaRowStartBase = soaJointRows[i];
				const ndConstraint* const* jointGroup = &jointArrayPtr[i * D_SSE_WORK_GROUP];
				const ndConstraint* const firstJoint = jointGroup[0];
				const ndInt32 rowCount = firstJoint->m_rowCount;
				for (ndInt32 j = 0; j < D_SSE_WORK_GROUP; ++j)
				{
					const ndConstraint* const Joint = jointGroup[j];
					const ndInt32 base = Joint->m_rowStart;
					for (ndInt32 k = 0; k < rowCount; ++k)
					{
						ndSoaMatrixElement* const row = &massMatrix[soaRowStartBase + k];
						row->m_coordenateAccel[j] = rightHandSide[base + k].m_coordenateAccel;
					}
				}
			}
			else
			{
				const ndInt32 soaRowStartBase = soaJointRows[i];
				const ndConstraint* const * jointGroup = &jointArrayPtr[i * D_SSE_WORK_GROUP];
				for (ndInt32 j = 0; j < D_SSE_WORK_GROUP; ++j)
				{
					const ndConstraint* const Joint = jointGroup[j];
					if (Joint)
					{
						const ndInt32 base = Joint->m_rowStart;
						const ndInt32 rowCount = Joint->m_rowCount;
						for (ndInt32 k = 0; k < rowCount; ++k)
						{
							ndSoaMatrixElement* const row = &massMatrix[soaRowStartBase + k];
							row->m_coordenateAccel[j] = rightHandSide[base + k].m_coordenateAccel;
						}
					}
				}
			}
		}
	});

	scene->ParallelExecute(CalculateJointsAcceleration);

	m_firstPassCoef = ndFloat32(1.0f);
	scene->ParallelExecute(UpdateAcceleration);
}

void ndDynamicsUpdateSoa::IntegrateBodiesVelocity()
{
	D_TRACKTIME();
	ndScene* const scene = m_world->GetScene();
	auto IntegrateBodiesVelocity = ndMakeObject::ndFunction([this](ndInt32 threadIndex, ndInt32 threadCount)
	{
		D_TRACKTIME();
		ndArray<ndBodyKinematic*>& bodyArray = GetBodyIslandOrder();
		const ndArray<ndJacobian>& internalForces = GetInternalForces();

		const ndVector timestep4(GetTimestepRK());
		const ndVector speedFreeze2(m_world->m_freezeSpeed2 * ndFloat32(0.1f));

		const ndStartEnd startEnd(bodyArray.GetCount() - GetUnconstrainedBodyCount(), threadIndex, threadCount);
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
	});
	scene->ParallelExecute(IntegrateBodiesVelocity);
}

void ndDynamicsUpdateSoa::CalculateJointsForce()
{
	D_TRACKTIME();
	ndScene* const scene = m_world->GetScene();
	const ndInt32 passes = m_solverPasses;
	const ndInt32 threadsCount = scene->GetThreadCount();

	ndArray<ndBodyKinematic*>& bodyArray = scene->GetActiveBodyArray();
	ndArray<ndConstraint*>& jointArray = scene->GetActiveContactArray();

	auto CalculateJointsForce = ndMakeObject::ndFunction([this, &jointArray](ndInt32 threadIndex, ndInt32 threadCount)
	{
		D_TRACKTIME();
		const ndInt32 jointCount = jointArray.GetCount();
		ndJacobian* const jointPartialForces = &GetTempInternalForces()[0];

		const ndInt32* const soaJointRows = &m_soaJointRows[0];
		ndSoaMatrixElement* const soaMassMatrix = &m_soaMassMatrix[0];

		auto JointForce = [this, &jointArray, jointPartialForces](ndInt32 group, ndSoaMatrixElement* const massMatrix)
		{
			ndVector weight0;
			ndVector weight1;
			ndSoaVector6 forceM0;
			ndSoaVector6 forceM1;
			ndVector preconditioner0;
			ndVector preconditioner1;
			ndVector normalForce[D_CONSTRAINT_MAX_ROWS + 1];

			const ndInt32 block = group * D_SSE_WORK_GROUP;
			ndConstraint** const jointGroup = &jointArray[block];

			const ndVector zero(ndVector::m_zero);
			const ndInt8 isUniformGruop = m_groupType[group];
			if (isUniformGruop)
			{
				for (ndInt32 i = 0; i < D_SSE_WORK_GROUP; ++i)
				{
					const ndConstraint* const joint = jointGroup[i];
					const ndBodyKinematic* const body0 = joint->GetBody0();
					const ndBodyKinematic* const body1 = joint->GetBody1();

					const ndInt32 m0 = body0->m_index;
					const ndInt32 m1 = body1->m_index;

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
				weight0 = zero;
				weight1 = zero;
				preconditioner0 = zero;
				preconditioner1 = zero;

				forceM0.m_linear.m_x = zero;
				forceM0.m_linear.m_y = zero;
				forceM0.m_linear.m_z = zero;
				forceM0.m_angular.m_x = zero;
				forceM0.m_angular.m_y = zero;
				forceM0.m_angular.m_z = zero;

				forceM1.m_linear.m_x = zero;
				forceM1.m_linear.m_y = zero;
				forceM1.m_linear.m_z = zero;
				forceM1.m_angular.m_x = zero;
				forceM1.m_angular.m_y = zero;
				forceM1.m_angular.m_z = zero;
				for (ndInt32 i = 0; i < D_SSE_WORK_GROUP; ++i)
				{
					const ndConstraint* const joint = jointGroup[i];
					if (joint && joint->m_rowCount)
					{
						const ndBodyKinematic* const body0 = joint->GetBody0();
						const ndBodyKinematic* const body1 = joint->GetBody1();

						const ndInt32 m0 = body0->m_index;
						const ndInt32 m1 = body1->m_index;

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

			#ifdef D_USE_EARLY_OUT_JOINT
			ndVector accNorm(zero);
			#endif
			normalForce[0] = ndVector::m_one;
			const ndInt32 rowsCount = jointGroup[0]->m_rowCount;

			for (ndInt32 j = 0; j < rowsCount; ++j)
			{
				const ndSoaMatrixElement* const row = &massMatrix[j];

				ndVector a(row->m_JMinv.m_jacobianM0.m_linear.m_x * forceM0.m_linear.m_x);
				a = a.MulAdd(row->m_JMinv.m_jacobianM0.m_linear.m_y, forceM0.m_linear.m_y);
				a = a.MulAdd(row->m_JMinv.m_jacobianM0.m_linear.m_z, forceM0.m_linear.m_z);

				a = a.MulAdd(row->m_JMinv.m_jacobianM0.m_angular.m_x, forceM0.m_angular.m_x);
				a = a.MulAdd(row->m_JMinv.m_jacobianM0.m_angular.m_y, forceM0.m_angular.m_y);
				a = a.MulAdd(row->m_JMinv.m_jacobianM0.m_angular.m_z, forceM0.m_angular.m_z);

				a = a.MulAdd(row->m_JMinv.m_jacobianM1.m_linear.m_x, forceM1.m_linear.m_x);
				a = a.MulAdd(row->m_JMinv.m_jacobianM1.m_linear.m_y, forceM1.m_linear.m_y);
				a = a.MulAdd(row->m_JMinv.m_jacobianM1.m_linear.m_z, forceM1.m_linear.m_z);

				a = a.MulAdd(row->m_JMinv.m_jacobianM1.m_angular.m_x, forceM1.m_angular.m_x);
				a = a.MulAdd(row->m_JMinv.m_jacobianM1.m_angular.m_y, forceM1.m_angular.m_y);
				a = a.MulAdd(row->m_JMinv.m_jacobianM1.m_angular.m_z, forceM1.m_angular.m_z);

				a = row->m_coordenateAccel.MulSub(row->m_force, row->m_diagDamp) - a;
				ndVector f(row->m_force.MulAdd(row->m_invJinvMJt, a));

				const ndVector frictionNormal(&normalForce[0].m_x, row->m_normalForceIndex.m_i);
				const ndVector lowerFrictionForce(frictionNormal * row->m_lowerBoundFrictionCoefficent);
				const ndVector upperFrictionForce(frictionNormal * row->m_upperBoundFrictionCoefficent);

				#ifdef D_USE_EARLY_OUT_JOINT
				a = a & (f < upperFrictionForce) & (f > lowerFrictionForce);
				accNorm = accNorm.MulAdd(a, a);
				#endif

				f = f.GetMax(lowerFrictionForce).GetMin(upperFrictionForce);
				normalForce[j + 1] = f;

				const ndVector deltaForce(f - row->m_force);
				const ndVector deltaForce0(deltaForce * preconditioner0);
				const ndVector deltaForce1(deltaForce * preconditioner1);

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

			const ndFloat32 tol = ndFloat32(0.125f);
			const ndFloat32 tol2 = tol * tol;

			#ifdef D_USE_EARLY_OUT_JOINT
			ndVector maxAccel(accNorm);
			for (ndInt32 k = 0; (k < 4) && (maxAccel.GetMax().GetScalar() > tol2); ++k)
			#else
			for (ndInt32 k = 0; k < 4; ++k)
			#endif
			{
				#ifdef D_USE_EARLY_OUT_JOINT
				maxAccel = zero;
				#endif

				for (ndInt32 j = 0; j < rowsCount; ++j)
				{
					const ndSoaMatrixElement* const row = &massMatrix[j];

					ndVector a(row->m_JMinv.m_jacobianM0.m_linear.m_x * forceM0.m_linear.m_x);
					a = a.MulAdd(row->m_JMinv.m_jacobianM0.m_linear.m_y, forceM0.m_linear.m_y);
					a = a.MulAdd(row->m_JMinv.m_jacobianM0.m_linear.m_z, forceM0.m_linear.m_z);

					a = a.MulAdd(row->m_JMinv.m_jacobianM0.m_angular.m_x, forceM0.m_angular.m_x);
					a = a.MulAdd(row->m_JMinv.m_jacobianM0.m_angular.m_y, forceM0.m_angular.m_y);
					a = a.MulAdd(row->m_JMinv.m_jacobianM0.m_angular.m_z, forceM0.m_angular.m_z);

					a = a.MulAdd(row->m_JMinv.m_jacobianM1.m_linear.m_x, forceM1.m_linear.m_x);
					a = a.MulAdd(row->m_JMinv.m_jacobianM1.m_linear.m_y, forceM1.m_linear.m_y);
					a = a.MulAdd(row->m_JMinv.m_jacobianM1.m_linear.m_z, forceM1.m_linear.m_z);

					a = a.MulAdd(row->m_JMinv.m_jacobianM1.m_angular.m_x, forceM1.m_angular.m_x);
					a = a.MulAdd(row->m_JMinv.m_jacobianM1.m_angular.m_y, forceM1.m_angular.m_y);
					a = a.MulAdd(row->m_JMinv.m_jacobianM1.m_angular.m_z, forceM1.m_angular.m_z);

					const ndVector force(normalForce[j + 1]);
					a = row->m_coordenateAccel.MulSub(force, row->m_diagDamp) - a;
					ndVector f(force.MulAdd(row->m_invJinvMJt, a));

					const ndVector frictionNormal(&normalForce[0].m_x, row->m_normalForceIndex.m_i);
					const ndVector lowerFrictionForce(frictionNormal * row->m_lowerBoundFrictionCoefficent);
					const ndVector upperFrictionForce(frictionNormal * row->m_upperBoundFrictionCoefficent);

					#ifdef D_USE_EARLY_OUT_JOINT
					a = a & (f < upperFrictionForce) & (f > lowerFrictionForce);
					maxAccel = maxAccel.MulAdd(a, a);
					#endif

					f = f.GetMax(lowerFrictionForce).GetMin(upperFrictionForce);
					normalForce[j + 1] = f;

					const ndVector deltaForce(f - force);
					const ndVector deltaForce0(deltaForce * preconditioner0);
					const ndVector deltaForce1(deltaForce * preconditioner1);

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

			ndVector mask(m_jointMask[group]);
			for (ndInt32 i = 0; i < D_SSE_WORK_GROUP; ++i)
			{
				const ndConstraint* const joint = jointGroup[i];
				if (joint && joint->m_rowCount)
				{
					const ndBodyKinematic* const body0 = joint->GetBody0();
					const ndBodyKinematic* const body1 = joint->GetBody1();
					dAssert(body0);
					dAssert(body1);
					const ndInt32 resting = body0->m_equilibrium0 & body1->m_equilibrium0;
					if (resting)
					{
						mask[i] = ndFloat32(0.0f);
					}
				}
			}

			forceM0.m_linear.m_x = zero;
			forceM0.m_linear.m_y = zero;
			forceM0.m_linear.m_z = zero;
			forceM0.m_angular.m_x = zero;
			forceM0.m_angular.m_y = zero;
			forceM0.m_angular.m_z = zero;

			forceM1.m_linear.m_x = zero;
			forceM1.m_linear.m_y = zero;
			forceM1.m_linear.m_z = zero;
			forceM1.m_angular.m_x = zero;
			forceM1.m_angular.m_y = zero;
			forceM1.m_angular.m_z = zero;
			for (ndInt32 i = 0; i < rowsCount; ++i)
			{
				ndSoaMatrixElement* const row = &massMatrix[i];
				const ndVector force(row->m_force.Select(normalForce[i + 1], mask));
				row->m_force = force;

				forceM0.m_linear.m_x = forceM0.m_linear.m_x.MulAdd(row->m_Jt.m_jacobianM0.m_linear.m_x, force);
				forceM0.m_linear.m_y = forceM0.m_linear.m_y.MulAdd(row->m_Jt.m_jacobianM0.m_linear.m_y, force);
				forceM0.m_linear.m_z = forceM0.m_linear.m_z.MulAdd(row->m_Jt.m_jacobianM0.m_linear.m_z, force);
				forceM0.m_angular.m_x = forceM0.m_angular.m_x.MulAdd(row->m_Jt.m_jacobianM0.m_angular.m_x, force);
				forceM0.m_angular.m_y = forceM0.m_angular.m_y.MulAdd(row->m_Jt.m_jacobianM0.m_angular.m_y, force);
				forceM0.m_angular.m_z = forceM0.m_angular.m_z.MulAdd(row->m_Jt.m_jacobianM0.m_angular.m_z, force);

				forceM1.m_linear.m_x = forceM1.m_linear.m_x.MulAdd(row->m_Jt.m_jacobianM1.m_linear.m_x, force);
				forceM1.m_linear.m_y = forceM1.m_linear.m_y.MulAdd(row->m_Jt.m_jacobianM1.m_linear.m_y, force);
				forceM1.m_linear.m_z = forceM1.m_linear.m_z.MulAdd(row->m_Jt.m_jacobianM1.m_linear.m_z, force);
				forceM1.m_angular.m_x = forceM1.m_angular.m_x.MulAdd(row->m_Jt.m_jacobianM1.m_angular.m_x, force);
				forceM1.m_angular.m_y = forceM1.m_angular.m_y.MulAdd(row->m_Jt.m_jacobianM1.m_angular.m_y, force);
				forceM1.m_angular.m_z = forceM1.m_angular.m_z.MulAdd(row->m_Jt.m_jacobianM1.m_angular.m_z, force);
			}

			ndJacobian force0[4];
			ndJacobian force1[4];
			ndVector::Transpose4x4(
				force0[0].m_linear,
				force0[1].m_linear,
				force0[2].m_linear,
				force0[3].m_linear,
				forceM0.m_linear.m_x,
				forceM0.m_linear.m_y,
				forceM0.m_linear.m_z, ndVector::m_zero);
			ndVector::Transpose4x4(
				force0[0].m_angular,
				force0[1].m_angular,
				force0[2].m_angular,
				force0[3].m_angular,
				forceM0.m_angular.m_x,
				forceM0.m_angular.m_y,
				forceM0.m_angular.m_z, ndVector::m_zero);

			ndVector::Transpose4x4(
				force1[0].m_linear,
				force1[1].m_linear,
				force1[2].m_linear,
				force1[3].m_linear,
				forceM1.m_linear.m_x,
				forceM1.m_linear.m_y,
				forceM1.m_linear.m_z, ndVector::m_zero);
			ndVector::Transpose4x4(
				force1[0].m_angular,
				force1[1].m_angular,
				force1[2].m_angular,
				force1[3].m_angular,
				forceM1.m_angular.m_x,
				forceM1.m_angular.m_y,
				forceM1.m_angular.m_z, ndVector::m_zero);

			ndRightHandSide* const rightHandSide = &m_rightHandSide[0];
			for (ndInt32 i = 0; i < D_SSE_WORK_GROUP; ++i)
			{
				const ndConstraint* const joint = jointGroup[i];
				if (joint)
				{
					const ndInt32 rowCount = joint->m_rowCount;
					const ndInt32 rowStartBase = joint->m_rowStart;
					for (ndInt32 j = 0; j < rowCount; ++j)
					{
						const ndSoaMatrixElement* const row = &massMatrix[j];
						rightHandSide[j + rowStartBase].m_force = row->m_force[i];
						rightHandSide[j + rowStartBase].m_maxImpact = dMax(dAbs(row->m_force[i]), rightHandSide[j + rowStartBase].m_maxImpact);
					}

					const ndInt32 index0 = (block + i) * 2 + 0;
					ndJacobian& outBody0 = jointPartialForces[index0];
					outBody0.m_linear = force0[i].m_linear;
					outBody0.m_angular = force0[i].m_angular;

					const ndInt32 index1 = (block + i) * 2 + 1;
					ndJacobian& outBody1 = jointPartialForces[index1];
					outBody1.m_linear = force1[i].m_linear;
					outBody1.m_angular = force1[i].m_angular;
				}
			}
		};

		const ndInt32 mask = -ndInt32(D_SSE_WORK_GROUP);
		const ndInt32 soaJointCount = ((jointCount + D_SSE_WORK_GROUP - 1) & mask) / D_SSE_WORK_GROUP;
		for (ndInt32 i = threadIndex; i < soaJointCount; i += threadCount)
		{
			JointForce(i, &soaMassMatrix[soaJointRows[i]]);
		}
	});

	auto ApplyJacobianAccumulatePartialForces = ndMakeObject::ndFunction([this, &bodyArray](ndInt32 threadIndex, ndInt32 threadCount)
	{
		D_TRACKTIME();
		const ndVector zero(ndVector::m_zero);

		ndJacobian* const internalForces = &GetInternalForces()[0];
		const ndInt32* const bodyIndex = &GetJointForceIndexBuffer()[0];
		const ndJacobian* const jointInternalForces = &GetTempInternalForces()[0];
		const ndJointBodyPairIndex* const jointBodyPairIndexBuffer = &GetJointBodyPairIndexBuffer()[0];

		const ndStartEnd startEnd(bodyArray.GetCount(), threadIndex, threadCount);
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
	});

	for (ndInt32 i = 0; i < passes; ++i)
	{
		scene->ParallelExecute(CalculateJointsForce);
		scene->ParallelExecute(ApplyJacobianAccumulatePartialForces);
	}
}

void ndDynamicsUpdateSoa::CalculateForces()
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
		
		UpdateForceFeedback();
	}
}

void ndDynamicsUpdateSoa::Update()
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
