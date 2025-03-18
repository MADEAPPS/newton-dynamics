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

	ndAtomic<ndInt32> iterator(0);
	auto CalculateSleepState = ndMakeObject::ndFunction([this, &iterator](ndInt32, ndInt32)
	{
		D_TRACKTIME_NAMED(CalculateSleepState);
		ndScene* const scene = m_world->GetScene();
		const ndArray<ndInt32>& bodyIndex = GetJointForceIndexBuffer();
		const ndJointBodyPairIndex* const jointBodyPairIndexBuffer = &GetJointBodyPairIndexBuffer()[0];
		ndConstraint** const jointArray = &scene->GetActiveContactArray()[0];
		ndBodyKinematic** const bodyArray = &scene->GetActiveBodyArray()[0];

		const ndVector zero(ndVector::m_zero);
		const ndInt32 bodyCount = ndInt32(bodyIndex.GetCount()) - 1;
		for (ndInt32 i = iterator.fetch_add(D_WORKER_BATCH_SIZE); i < bodyCount; i = iterator.fetch_add(D_WORKER_BATCH_SIZE))
		{
			const ndInt32 maxSpan = ((bodyCount - i) >= D_WORKER_BATCH_SIZE) ? D_WORKER_BATCH_SIZE : bodyCount - i;
			for (ndInt32 j = 0; j < maxSpan; ++j)
			{
				const ndInt32 m = i + j;
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
			}
		}
	});

	ndScene* const scene = m_world->GetScene();
	if (scene->GetActiveContactArray().GetCount())
	{
		scene->ParallelExecute(CalculateSleepState);
	}
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
	const ndInt32 jointCount = ndInt32 (jointArray.GetCount());
	const ndInt32 soaJointCount = (jointCount + D_SSE_WORK_GROUP - 1) & mask;
	ndAssert(jointArray.GetCapacity() > soaJointCount);
	ndConstraint** const jointArrayPtr = &jointArray[0];
	for (ndInt32 i = jointCount; i < soaJointCount; ++i)
	{
		jointArrayPtr[i] = nullptr;
	}
	
	if (m_activeJointCount - jointArray.GetCount())
	{
		const ndInt32 base = m_activeJointCount & mask;
		const ndInt32 count = jointArrayPtr[base + D_SSE_WORK_GROUP - 1] ? D_SSE_WORK_GROUP : ndInt32 (jointArray.GetCount()) - base;
		ndAssert(count <= D_SSE_WORK_GROUP);
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
		D_TRACKTIME_NAMED(SetRowStarts);
		auto SetRowsCount = [&jointArray, &rowsCount]()
		{
			ndInt32 rowCount = 1;
			const ndInt32 count = ndInt32 (jointArray.GetCount());
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
			const ndInt32 count = ndInt32 (soaJointRows.GetCount());
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
		//else if (threadIndex == (threadCount - 1))
		else if (threadIndex == 1)
		{
			SetSoaRowsCount();
		}
	});

	scene->ParallelExecute(SetRowStarts);
	m_leftHandSide.SetCount(rowsCount);
	m_rightHandSide.SetCount(rowsCount);
	m_soaMassMatrix.SetCount(soaJointRowCount);

	#ifdef _DEBUG
		ndAssert(m_activeJointCount <= jointArray.GetCount());
		const ndInt32 maxRowCount = ndInt32 (m_leftHandSide.GetCount());
		for (ndInt32 i = 0; i < ndInt32 (jointArray.GetCount()); ++i)
		{
			ndConstraint* const joint = jointArray[i];
			ndAssert(joint->m_rowStart < ndInt32 (m_leftHandSide.GetCount()));
			ndAssert((joint->m_rowStart + joint->m_rowCount) <= maxRowCount);
		}
		
		for (ndInt32 i = 0; i < jointCount; i += D_SSE_WORK_GROUP)
		{
			const ndInt32 count = jointArrayPtr[i + D_SSE_WORK_GROUP - 1] ? D_SSE_WORK_GROUP : jointCount - i;
			for (ndInt32 j = 1; j < count; ++j)
			{
				ndConstraint* const joint0 = jointArrayPtr[i + j - 1];
				ndConstraint* const joint1 = jointArrayPtr[i + j - 0];
				ndAssert(joint0->m_rowCount >= joint1->m_rowCount);
			}
		}
	#endif

	SortBodyJointScan();
}

void ndDynamicsUpdateSoa::SortIslands()
{
	D_TRACKTIME();
	ndScene* const scene = m_world->GetScene();
	const ndArray<ndBodyKinematic*>& bodyArray = scene->GetActiveBodyArray();
	ndArray<ndBodyKinematic*>& activeBodyArray = GetBodyIslandOrder();
	GetInternalForces().SetCount(bodyArray.GetCount());
	activeBodyArray.SetCount(bodyArray.GetCount());

	ndInt32 histogram[D_MAX_THREADS_COUNT][3];
	auto Scan0 = ndMakeObject::ndFunction([&bodyArray, &histogram](ndInt32 threadIndex, ndInt32 threadCount)
	{
		D_TRACKTIME_NAMED(Scan0);
		ndInt32* const hist = &histogram[threadIndex][0];
		hist[0] = 0;
		hist[1] = 0;
		hist[2] = 0;

		ndInt32 map[4];
		map[0] = 0;
		map[1] = 1;
		map[2] = 2;
		map[3] = 2;
		const ndStartEnd startEnd(ndInt32 (bodyArray.GetCount()), threadIndex, threadCount);
		for (ndInt32 i = startEnd.m_start; i < startEnd.m_end; ++i)
		{
			ndBodyKinematic* const body = bodyArray[i];
			ndInt32 key = map[body->m_equilibrium0 * 2 + 1 - body->m_isConstrained];
			ndAssert(key < 3);
			hist[key] = hist[key] + 1;
		}
	});

	auto Sort0 = ndMakeObject::ndFunction([&bodyArray, &activeBodyArray, &histogram](ndInt32 threadIndex, ndInt32 threadCount)
	{
		D_TRACKTIME_NAMED(Sort0);
		ndInt32* const hist = &histogram[threadIndex][0];

		ndInt32 map[4];
		map[0] = 0;
		map[1] = 1;
		map[2] = 2;
		map[3] = 2;

		const ndStartEnd startEnd(ndInt32 (bodyArray.GetCount()), threadIndex, threadCount);
		for (ndInt32 i = startEnd.m_start; i < startEnd.m_end; ++i)
		{
			ndBodyKinematic* const body = bodyArray[i];
			ndInt32 key = map[body->m_equilibrium0 * 2 + 1 - body->m_isConstrained];
			ndAssert(key < 3);
			const ndInt32 entry = hist[key];
			activeBodyArray[entry] = body;
			hist[key] = entry + 1;
		}
	});

	scene->ParallelExecute(Scan0);

	ndInt32 scan[3];
	scan[0] = 0;
	scan[1] = 0;
	scan[2] = 0;
	const ndInt32 threadCount = scene->GetThreadCount();

	ndInt32 sum = 0;
	for (ndInt32 i = 0; i < 3; ++i)
	{
		for (ndInt32 j = 0; j < threadCount; ++j)
		{
			ndInt32 partialSum = histogram[j][i];
			histogram[j][i] = sum;
			sum += partialSum;
		}
		scan[i] = sum;
	}

	scene->ParallelExecute(Sort0);
	activeBodyArray.SetCount(scan[1]);
	m_unConstrainedBodyCount = scan[1] - scan[0];
}

void ndDynamicsUpdateSoa::BuildIsland()
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

void ndDynamicsUpdateSoa::IntegrateUnconstrainedBodies()
{
	ndScene* const scene = m_world->GetScene();
	ndAtomic<ndInt32> iterator(0);
	auto IntegrateUnconstrainedBodies = ndMakeObject::ndFunction([this, &iterator, &scene](ndInt32, ndInt32)
	{
		D_TRACKTIME_NAMED(IntegrateUnconstrainedBodies);
		ndArray<ndBodyKinematic*>& bodyArray = GetBodyIslandOrder();

		const ndFloat32 timestep = scene->GetTimestep();
		const ndInt32 base = ndInt32 (bodyArray.GetCount() - GetUnconstrainedBodyCount());

		const ndInt32 count = GetUnconstrainedBodyCount();
		for (ndInt32 i = iterator.fetch_add(D_WORKER_BATCH_SIZE); i < count; i = iterator.fetch_add(D_WORKER_BATCH_SIZE))
		{
			const ndInt32 maxSpan = ((count - i) >= D_WORKER_BATCH_SIZE) ? D_WORKER_BATCH_SIZE : count - i;
			for (ndInt32 j = 0; j < maxSpan; ++j)
			{
				ndBodyKinematic* const body = bodyArray[base + i + j];
				ndAssert(body);
				body->UpdateInvInertiaMatrix();
				body->AddDampingAcceleration(timestep);
				body->IntegrateExternalForce(timestep);
			}
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
	const ndInt32 bodyCount = ndInt32 (bodyArray.GetCount());
	GetInternalForces().SetCount(bodyCount);

	ndInt32 extraPassesArray[D_MAX_THREADS_COUNT];

	ndAtomic<ndInt32> iterator(0);
	auto InitWeights = ndMakeObject::ndFunction([this, &iterator, &bodyArray, &extraPassesArray](ndInt32 threadIndex, ndInt32)
	{
		D_TRACKTIME_NAMED(InitWeights);
		const ndArray<ndInt32>& jointForceIndexBuffer = GetJointForceIndexBuffer();
		const ndArray<ndJointBodyPairIndex>& jointBodyPairIndex = GetJointBodyPairIndexBuffer();

		ndInt32 maxExtraPasses = 1;
		const ndInt32 jointCount = ndInt32 (jointForceIndexBuffer.GetCount()) - 1;
		for (ndInt32 i = iterator.fetch_add(D_WORKER_BATCH_SIZE); i < jointCount; i = iterator.fetch_add(D_WORKER_BATCH_SIZE))
		{
			const ndInt32 maxSpan = ((jointCount - i) >= D_WORKER_BATCH_SIZE) ? D_WORKER_BATCH_SIZE : jointCount - i;
			for (ndInt32 j = 0; j < maxSpan; ++j)
			{
				const ndInt32 index = jointForceIndexBuffer[i + j];
				const ndJointBodyPairIndex& scan = jointBodyPairIndex[index];
				ndBodyKinematic* const body = bodyArray[scan.m_body];
				ndAssert(body->m_index == scan.m_body);
				ndAssert(body->m_isConstrained <= 1);
				const ndInt32 count = jointForceIndexBuffer[i + j + 1] - index - 1;
				const ndInt32 mask = -ndInt32(body->m_isConstrained & ~body->m_isStatic);
				const ndInt32 weigh = 1 + (mask & count);
				ndAssert(weigh >= 0);
				if (weigh)
				{
					body->m_weigh = ndFloat32(weigh);
				}
				maxExtraPasses = ndMax(weigh, maxExtraPasses);
			}
		}
		extraPassesArray[threadIndex] = maxExtraPasses;
	});

	if (scene->GetActiveContactArray().GetCount())
	{
		scene->ParallelExecute(InitWeights);

		ndInt32 extraPasses = 0;
		const ndInt32 threadCount = scene->GetThreadCount();
		for (ndInt32 i = 0; i < threadCount; ++i)
		{
			extraPasses = ndMax(extraPasses, extraPassesArray[i]);
		}

		const ndInt32 conectivity = 7;
		m_solverPasses = ndUnsigned32 (m_world->GetSolverIterations() + 2 * extraPasses / conectivity + 2);
	}
}

void ndDynamicsUpdateSoa::IntegrateBodies()
{
	D_TRACKTIME();
	ndScene* const scene = m_world->GetScene();
	const ndVector invTime(m_invTimestep);
	const ndFloat32 timestep = scene->GetTimestep();

	ndAtomic<ndInt32> iterator(0);
	auto IntegrateBodies = ndMakeObject::ndFunction([this, &iterator, timestep, invTime](ndInt32, ndInt32)
	{
		D_TRACKTIME_NAMED(IntegrateBodies);
		const ndWorld* const world = m_world;
		const ndArray<ndBodyKinematic*>& bodyArray = GetBodyIslandOrder();

		const ndFloat32 speedFreeze2 = world->m_freezeSpeed2;
		const ndFloat32 accelFreeze2 = world->m_freezeAccel2;

		const ndInt32 count = ndInt32 (bodyArray.GetCount());
		for (ndInt32 i = iterator.fetch_add(D_WORKER_BATCH_SIZE); i < count; i = iterator.fetch_add(D_WORKER_BATCH_SIZE))
		{
			const ndInt32 maxSpan = ((count - i) >= D_WORKER_BATCH_SIZE) ? D_WORKER_BATCH_SIZE : count - i;
			for (ndInt32 j = 0; j < maxSpan; ++j)
			{
				ndBodyKinematic* const body = bodyArray[i + j];
				if (!body->m_equilibrium)
				{
					body->SetAcceleration(invTime * (body->m_veloc - body->m_accel), invTime * (body->m_omega - body->m_alpha));
					body->IntegrateVelocity(timestep);
				}
				body->EvaluateSleepState(speedFreeze2, accelFreeze2);
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

	ndAtomic<ndInt32> iterator(0);
	auto InitBodyArray = ndMakeObject::ndFunction([this, &iterator, timestep](ndInt32, ndInt32)
	{
		D_TRACKTIME_NAMED(InitBodyArray);
		const ndArray<ndBodyKinematic*>& bodyArray = GetBodyIslandOrder();
		const ndInt32 count = ndInt32 (bodyArray.GetCount() - GetUnconstrainedBodyCount());
		for (ndInt32 i = iterator.fetch_add(D_WORKER_BATCH_SIZE); i < count; i = iterator.fetch_add(D_WORKER_BATCH_SIZE))
		{
			const ndInt32 maxSpan = ((count - i) >= D_WORKER_BATCH_SIZE) ? D_WORKER_BATCH_SIZE : count - i;
			for (ndInt32 j = 0; j < maxSpan; ++j)
			{
				ndBodyKinematic* const body = bodyArray[i + j];
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
			}
		}
	});
	scene->ParallelExecute(InitBodyArray);
}

void ndDynamicsUpdateSoa::GetJacobianDerivatives(ndConstraint* const joint)
{
	ndConstraintDescritor constraintParam;
	ndAssert(joint->GetRowsCount() <= D_CONSTRAINT_MAX_ROWS);
	for (ndInt32 i = ndInt32(joint->GetRowsCount() - 1); i >= 0; --i)
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

		ndAssert(constraintParam.m_forceBounds[i].m_normalIndex >= -2);
		rhs->m_normalForceIndex = constraintParam.m_forceBounds[i].m_normalIndex;
		rhs->SetSanityCheck(joint);
		ndAssert(rhs->SanityCheck());
		if (rhs->m_normalForceIndex == D_OVERRIDE_FRICTION_ROW)
		{
			rhs->m_normalForceIndex = D_INDEPENDENT_ROW;
		}
	}
}

void ndDynamicsUpdateSoa::InitJacobianMatrix()
{
	ndScene* const scene = m_world->GetScene();
	ndBodyKinematic** const bodyArray = &scene->GetActiveBodyArray()[0];
	ndArray<ndConstraint*>& jointArray = scene->GetActiveContactArray();

	ndAtomic<ndInt32> iterator(0);
	auto InitJacobianMatrix = ndMakeObject::ndFunction([this, &iterator, &jointArray](ndInt32, ndInt32)
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
				ndAssert(rhs->SanityCheck());
				rhs->m_maxImpact = ndFloat32(0.0f);

				const ndJacobian& JtM0 = row->m_Jt.m_jacobianM0;
				const ndJacobian& JtM1 = row->m_Jt.m_jacobianM1;
				const ndVector tmpDiag(
					weigh0 * (JMinvM0.m_linear * JtM0.m_linear + JMinvM0.m_angular * JtM0.m_angular) +
					weigh1 * (JMinvM1.m_linear * JtM1.m_linear + JMinvM1.m_angular * JtM1.m_angular));

				ndFloat32 diag = tmpDiag.AddHorizontal().GetScalar();
				ndAssert(diag > ndFloat32(0.0f));
				rhs->m_diagDamp = diag * rhs->m_diagonalRegularizer;

				diag *= (ndFloat32(1.0f) + rhs->m_diagonalRegularizer);
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


		const ndInt32 jointCount = ndInt32 (jointArray.GetCount());
		for (ndInt32 i = iterator.fetch_add(D_WORKER_BATCH_SIZE); i < jointCount; i = iterator.fetch_add(D_WORKER_BATCH_SIZE))
		{
			const ndInt32 maxSpan = ((jointCount - i) >= D_WORKER_BATCH_SIZE) ? D_WORKER_BATCH_SIZE : jointCount - i;
			for (ndInt32 j = 0; j < maxSpan; ++j)
			{
				ndConstraint* const joint = jointArray[i + j];
				GetJacobianDerivatives(joint);
				BuildJacobianMatrix(joint, i + j);
			}
		}
	});

	ndAtomic<ndInt32> iterator1(0);
	auto InitJacobianAccumulatePartialForces = ndMakeObject::ndFunction([this, &iterator1, &bodyArray](ndInt32, ndInt32)
	{
		D_TRACKTIME_NAMED(InitJacobianAccumulatePartialForces);
		const ndVector zero(ndVector::m_zero);
		ndJacobian* const internalForces = &GetInternalForces()[0];
		const ndArray<ndInt32>& bodyIndex = GetJointForceIndexBuffer();

		const ndJacobian* const jointInternalForces = &GetTempInternalForces()[0];
		const ndJointBodyPairIndex* const jointBodyPairIndexBuffer = &GetJointBodyPairIndexBuffer()[0];

		const ndInt32 bodyCount = ndInt32 (bodyIndex.GetCount()) - 1;
		for (ndInt32 i = iterator1.fetch_add(D_WORKER_BATCH_SIZE); i < bodyCount; i = iterator1.fetch_add(D_WORKER_BATCH_SIZE))
		{
			const ndInt32 maxSpan = ((bodyCount - i) >= D_WORKER_BATCH_SIZE) ? D_WORKER_BATCH_SIZE : bodyCount - i;
			for (ndInt32 j = 0; j < maxSpan; ++j)
			{
				ndVector force(zero);
				ndVector torque(zero);

				const ndInt32 m = i + j;
				const ndInt32 index = bodyIndex[m];
				const ndJointBodyPairIndex& scan = jointBodyPairIndexBuffer[index];
				ndBodyKinematic* const body = bodyArray[scan.m_body];

				ndAssert(body->m_isStatic <= 1);
				ndAssert(body->m_index == scan.m_body);
				const ndInt32 mask = ndInt32(body->m_isStatic) - 1;
				const ndInt32 count = mask & (bodyIndex[m + 1] - index);

				for (ndInt32 k = 0; k < count; ++k)
				{
					const ndInt32 jointIndex = jointBodyPairIndexBuffer[index + k].m_joint;
					force += jointInternalForces[jointIndex].m_linear;
					torque += jointInternalForces[jointIndex].m_angular;
				}
				internalForces[m].m_linear = force;
				internalForces[m].m_angular = torque;
			}
		}
	});
	
	ndAtomic<ndInt32> iterator2(0);
	auto TransposeMassMatrix = ndMakeObject::ndFunction([this, &iterator2, &jointArray](ndInt32, ndInt32)
	{
		D_TRACKTIME_NAMED(TransposeMassMatrix);
		const ndInt32 jointCount = ndInt32 (jointArray.GetCount());

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

		for (ndInt32 i = iterator2.fetch_add(D_WORKER_BATCH_SIZE); i < soaJointCount; i = iterator2.fetch_add(D_WORKER_BATCH_SIZE))
		{
			const ndInt32 maxSpan = ((soaJointCount - i) >= D_WORKER_BATCH_SIZE) ? D_WORKER_BATCH_SIZE : soaJointCount - i;
			for (ndInt32 j = 0; j < maxSpan; ++j)
			{
				const ndInt32 m = i + j;
				const ndInt32 index = m * D_SSE_WORK_GROUP;
				ndInt32 maxRow = 0;
				ndInt32 minRow = 255;
				ndVector selectMask(ndVector::m_xyzwMask);
				for (ndInt32 k = 0; k < D_SSE_WORK_GROUP; ++k)
				{
					ndConstraint* const joint = jointsPtr[index + k];
					if (joint)
					{
						const ndInt32 maxMask = (maxRow - joint->m_rowCount) >> 8;
						const ndInt32 minMask = (minRow - joint->m_rowCount) >> 8;
						maxRow = ( maxMask & joint->m_rowCount) | (~maxMask & maxRow);
						minRow = (~minMask & joint->m_rowCount) | ( minMask & minRow);
						if (!joint->m_rowCount)
						{
							selectMask[k] = ndFloat32(0.0f);
						}
					}
					else
					{
						minRow = 0;
						selectMask[k] = ndFloat32(0.0f);
					}
				}
				ndAssert(maxRow >= 0);
				ndAssert(minRow < 255);
				jointMask[m] = selectMask;

				const ndInt8 isUniformGroup = (maxRow == minRow) & (maxRow > 0);
				groupType[m] = isUniformGroup;

				const ndInt32 soaRowBase = soaJointRows[m];
				if (isUniformGroup)
				{
					const ndConstraint* const joint0 = jointsPtr[index + 0];
					const ndConstraint* const joint1 = jointsPtr[index + 1];
					const ndConstraint* const joint2 = jointsPtr[index + 2];
					const ndConstraint* const joint3 = jointsPtr[index + 3];

					const ndInt32 rowCount = joint0->m_rowCount;
					for (ndInt32 k = 0; k < rowCount; ++k)
					{
						ndVector tmp;
						const ndLeftHandSide* const row0 = &leftHandSide[joint0->m_rowStart + k];
						const ndLeftHandSide* const row1 = &leftHandSide[joint1->m_rowStart + k];
						const ndLeftHandSide* const row2 = &leftHandSide[joint2->m_rowStart + k];
						const ndLeftHandSide* const row3 = &leftHandSide[joint3->m_rowStart + k];

						ndSoa::ndSoaMatrixElement& row = massMatrix[soaRowBase + k];
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
						for (ndInt32 n = 0; n < D_SSE_WORK_GROUP; ++n)
						{
							const ndConstraint* const soaJoint = jointsPtr[index + n];
							const ndRightHandSide* const rhs = &rightHandSide[soaJoint->m_rowStart + k];
							row.m_force[n] = rhs->m_force;
							row.m_diagDamp[n] = rhs->m_diagDamp;
							row.m_invJinvMJt[n] = rhs->m_invJinvMJt;
							row.m_coordenateAccel[n] = rhs->m_coordenateAccel;
							normalIndex[n] = (rhs->m_normalForceIndex + 1) * D_SSE_WORK_GROUP + n;
							row.m_lowerBoundFrictionCoefficent[n] = rhs->m_lowerBoundFrictionCoefficent;
							row.m_upperBoundFrictionCoefficent[n] = rhs->m_upperBoundFrictionCoefficent;
							ndAssert(rhs->SanityCheck());
						}
					}
				}
				else
				{
					for (ndInt32 k = 0; k < maxRow; ++k)
					{
						ndSoa::ndSoaMatrixElement& row = massMatrix[soaRowBase + k];
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

					for (ndInt32 k = 0; k < D_SSE_WORK_GROUP; ++k)
					{
						const ndConstraint* const joint = jointsPtr[index + k];
						if (joint)
						{
							for (ndInt32 n = 0; n < joint->m_rowCount; ++n)
							{
								ndSoa::ndSoaMatrixElement& row = massMatrix[soaRowBase + n];
								const ndLeftHandSide* const lhs = &leftHandSide[joint->m_rowStart + n];

								row.m_Jt.m_jacobianM0.m_linear.m_x[k] = lhs->m_Jt.m_jacobianM0.m_linear.m_x;
								row.m_Jt.m_jacobianM0.m_linear.m_y[k] = lhs->m_Jt.m_jacobianM0.m_linear.m_y;
								row.m_Jt.m_jacobianM0.m_linear.m_z[k] = lhs->m_Jt.m_jacobianM0.m_linear.m_z;
								row.m_Jt.m_jacobianM0.m_angular.m_x[k] = lhs->m_Jt.m_jacobianM0.m_angular.m_x;
								row.m_Jt.m_jacobianM0.m_angular.m_y[k] = lhs->m_Jt.m_jacobianM0.m_angular.m_y;
								row.m_Jt.m_jacobianM0.m_angular.m_z[k] = lhs->m_Jt.m_jacobianM0.m_angular.m_z;
								row.m_Jt.m_jacobianM1.m_linear.m_x[k] = lhs->m_Jt.m_jacobianM1.m_linear.m_x;
								row.m_Jt.m_jacobianM1.m_linear.m_y[k] = lhs->m_Jt.m_jacobianM1.m_linear.m_y;
								row.m_Jt.m_jacobianM1.m_linear.m_z[k] = lhs->m_Jt.m_jacobianM1.m_linear.m_z;
								row.m_Jt.m_jacobianM1.m_angular.m_x[k] = lhs->m_Jt.m_jacobianM1.m_angular.m_x;
								row.m_Jt.m_jacobianM1.m_angular.m_y[k] = lhs->m_Jt.m_jacobianM1.m_angular.m_y;
								row.m_Jt.m_jacobianM1.m_angular.m_z[k] = lhs->m_Jt.m_jacobianM1.m_angular.m_z;

								row.m_JMinv.m_jacobianM0.m_linear.m_x[k] = lhs->m_JMinv.m_jacobianM0.m_linear.m_x;
								row.m_JMinv.m_jacobianM0.m_linear.m_y[k] = lhs->m_JMinv.m_jacobianM0.m_linear.m_y;
								row.m_JMinv.m_jacobianM0.m_linear.m_z[k] = lhs->m_JMinv.m_jacobianM0.m_linear.m_z;
								row.m_JMinv.m_jacobianM0.m_angular.m_x[k] = lhs->m_JMinv.m_jacobianM0.m_angular.m_x;
								row.m_JMinv.m_jacobianM0.m_angular.m_y[k] = lhs->m_JMinv.m_jacobianM0.m_angular.m_y;
								row.m_JMinv.m_jacobianM0.m_angular.m_z[k] = lhs->m_JMinv.m_jacobianM0.m_angular.m_z;
								row.m_JMinv.m_jacobianM1.m_linear.m_x[k] = lhs->m_JMinv.m_jacobianM1.m_linear.m_x;
								row.m_JMinv.m_jacobianM1.m_linear.m_y[k] = lhs->m_JMinv.m_jacobianM1.m_linear.m_y;
								row.m_JMinv.m_jacobianM1.m_linear.m_z[k] = lhs->m_JMinv.m_jacobianM1.m_linear.m_z;
								row.m_JMinv.m_jacobianM1.m_angular.m_x[k] = lhs->m_JMinv.m_jacobianM1.m_angular.m_x;
								row.m_JMinv.m_jacobianM1.m_angular.m_y[k] = lhs->m_JMinv.m_jacobianM1.m_angular.m_y;
								row.m_JMinv.m_jacobianM1.m_angular.m_z[k] = lhs->m_JMinv.m_jacobianM1.m_angular.m_z;

								const ndRightHandSide* const rhs = &rightHandSide[joint->m_rowStart + n];
								row.m_force[k] = rhs->m_force;
								row.m_diagDamp[k] = rhs->m_diagDamp;
								row.m_invJinvMJt[k] = rhs->m_invJinvMJt;
								row.m_coordenateAccel[k] = rhs->m_coordenateAccel;

								#ifdef D_NEWTON_USE_DOUBLE
								ndInt64* const normalIndex = (ndInt64*)&row.m_normalForceIndex[0];
								#else
								ndInt32* const normalIndex = (ndInt32*)&row.m_normalForceIndex[0];
								#endif
								normalIndex[k] = (rhs->m_normalForceIndex + 1) * D_SSE_WORK_GROUP + k;
								row.m_lowerBoundFrictionCoefficent[k] = rhs->m_lowerBoundFrictionCoefficent;
								row.m_upperBoundFrictionCoefficent[k] = rhs->m_upperBoundFrictionCoefficent;
								ndAssert(rhs->SanityCheck());
							}
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

	ndAtomic<ndInt32> iterator(0);
	auto UpdateForceFeedback = ndMakeObject::ndFunction([this, &iterator, &jointArray](ndInt32, ndInt32)
	{
		D_TRACKTIME_NAMED(UpdateForceFeedback);
		ndArray<ndRightHandSide>& rightHandSide = m_rightHandSide;
		const ndArray<ndLeftHandSide>& leftHandSide = m_leftHandSide;

		const ndVector zero(ndVector::m_zero);
		const ndFloat32 timestepRK = GetTimestepRK();

		const ndInt32 count = ndInt32 (jointArray.GetCount());
		for (ndInt32 i = iterator.fetch_add(D_WORKER_BATCH_SIZE); i < count; i = iterator.fetch_add(D_WORKER_BATCH_SIZE))
		{
			const ndInt32 maxSpan = ((count - i) >= D_WORKER_BATCH_SIZE) ? D_WORKER_BATCH_SIZE : count - i;
			for (ndInt32 j = 0; j < maxSpan; ++j)
			{
				ndConstraint* const joint = jointArray[i + j];
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

	ndAtomic<ndInt32> iterator(0);
	auto InitSkeletons = ndMakeObject::ndFunction([this, &iterator, &activeSkeletons](ndInt32, ndInt32)
	{
		D_TRACKTIME_NAMED(InitSkeletons);
		ndArray<ndRightHandSide>& rightHandSide = m_rightHandSide;
		const ndArray<ndLeftHandSide>& leftHandSide = m_leftHandSide;

		const ndInt32 count = ndInt32 (activeSkeletons.GetCount());
		for (ndInt32 i = iterator++; i < count; i = iterator++)
		{
			ndSkeletonContainer* const skeleton = activeSkeletons[i];
			skeleton->InitMassMatrix(&leftHandSide[0], &rightHandSide[0]);
		}
	});

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
		scene->ParallelExecute(InitSkeletons);
	}
}

void ndDynamicsUpdateSoa::UpdateSkeletons()
{
	D_TRACKTIME();
	ndScene* const scene = m_world->GetScene();
	const ndArray<ndSkeletonContainer*>& activeSkeletons = m_world->m_activeSkeletons;

	ndAtomic<ndInt32> iterator(0);
	auto UpdateSkeletons = ndMakeObject::ndFunction([this, &iterator, &activeSkeletons](ndInt32, ndInt32)
	{
		D_TRACKTIME_NAMED(UpdateSkeletons);
		ndJacobian* const internalForces = &GetInternalForces()[0];

		const ndInt32 count = ndInt32 (activeSkeletons.GetCount());
		for (ndInt32 i = iterator++; i < count; i = iterator++)
		{
			ndSkeletonContainer* const skeleton = activeSkeletons[i];
			skeleton->CalculateReactionForces(internalForces);
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

	ndAtomic<ndInt32> iterator(0);
	auto CalculateJointsAcceleration = ndMakeObject::ndFunction([this, &iterator, &jointArray](ndInt32, ndInt32)
	{
		D_TRACKTIME_NAMED(CalculateJointsAcceleration);
		ndJointAccelerationDecriptor joindDesc;
		joindDesc.m_timestep = m_timestepRK;
		joindDesc.m_invTimestep = m_invTimestepRK;
		joindDesc.m_firstPassCoefFlag = m_firstPassCoef;
		ndArray<ndLeftHandSide>& leftHandSide = m_leftHandSide;
		ndArray<ndRightHandSide>& rightHandSide = m_rightHandSide;

		const ndInt32 count = ndInt32 (jointArray.GetCount());
		for (ndInt32 i = iterator.fetch_add(D_WORKER_BATCH_SIZE); i < count; i = iterator.fetch_add(D_WORKER_BATCH_SIZE))
		{
			const ndInt32 maxSpan = ((count - i) >= D_WORKER_BATCH_SIZE) ? D_WORKER_BATCH_SIZE : count - i;
			for (ndInt32 j = 0; j < maxSpan; ++j)
			{
				ndConstraint* const joint = jointArray[i + j];
				const ndInt32 pairStart = joint->m_rowStart;
				joindDesc.m_rowsCount = joint->m_rowCount;
				joindDesc.m_leftHandSide = &leftHandSide[pairStart];
				joindDesc.m_rightHandSide = &rightHandSide[pairStart];
				joint->JointAccelerations(&joindDesc);
			}
		}
	});

	ndAtomic<ndInt32> iterator1(0);
	auto UpdateAcceleration = ndMakeObject::ndFunction([this, &iterator1, &jointArray](ndInt32, ndInt32)
	{
		D_TRACKTIME_NAMED(UpdateAcceleration);
		const ndArray<ndRightHandSide>& rightHandSide = m_rightHandSide;

		const ndInt32 jointCount = ndInt32 (jointArray.GetCount());
		const ndInt32 mask = -ndInt32(D_SSE_WORK_GROUP);
		const ndInt32* const soaJointRows = &m_soaJointRows[0];
		const ndInt32 soaJointCountBatches = ((jointCount + D_SSE_WORK_GROUP - 1) & mask) / D_SSE_WORK_GROUP;
		const ndInt8* const groupType = &m_groupType[0];

		const ndConstraint* const * jointArrayPtr = &jointArray[0];
		ndSoaMatrixElement* const massMatrix = &m_soaMassMatrix[0];

		for (ndInt32 i = iterator1.fetch_add(D_WORKER_BATCH_SIZE); i < soaJointCountBatches; i = iterator1.fetch_add(D_WORKER_BATCH_SIZE))
		{
			const ndInt32 maxSpan = ((soaJointCountBatches - i) >= D_WORKER_BATCH_SIZE) ? D_WORKER_BATCH_SIZE : soaJointCountBatches - i;
			for (ndInt32 j = 0; j < maxSpan; ++j)
			{
				const ndInt32 m = i + j;
				if (groupType[m])
				{
					const ndInt32 soaRowStartBase = soaJointRows[m];
					const ndConstraint* const* jointGroup = &jointArrayPtr[m * D_SSE_WORK_GROUP];
					const ndConstraint* const firstJoint = jointGroup[0];
					const ndInt32 rowCount = firstJoint->m_rowCount;
					for (ndInt32 k = 0; k < D_SSE_WORK_GROUP; ++k)
					{
						const ndConstraint* const Joint = jointGroup[k];
						const ndInt32 base = Joint->m_rowStart;
						for (ndInt32 n = 0; n < rowCount; ++n)
						{
							ndSoaMatrixElement* const row = &massMatrix[soaRowStartBase + n];
							row->m_coordenateAccel[k] = rightHandSide[base + n].m_coordenateAccel;
						}
					}
				}
				else
				{
					const ndInt32 soaRowStartBase = soaJointRows[m];
					const ndConstraint* const* jointGroup = &jointArrayPtr[m * D_SSE_WORK_GROUP];
					for (ndInt32 k = 0; k < D_SSE_WORK_GROUP; ++k)
					{
						const ndConstraint* const Joint = jointGroup[k];
						if (Joint)
						{
							const ndInt32 base = Joint->m_rowStart;
							const ndInt32 rowCount = Joint->m_rowCount;
							for (ndInt32 n = 0; n < rowCount; ++n)
							{
								ndSoaMatrixElement* const row = &massMatrix[soaRowStartBase + n];
								row->m_coordenateAccel[k] = rightHandSide[base + n].m_coordenateAccel;
							}
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
	ndAtomic<ndInt32> iterator(0);
	auto IntegrateBodiesVelocity = ndMakeObject::ndFunction([this, &iterator](ndInt32, ndInt32)
	{
		D_TRACKTIME_NAMED(IntegrateBodiesVelocity);
		ndArray<ndBodyKinematic*>& bodyArray = GetBodyIslandOrder();
		const ndArray<ndJacobian>& internalForces = GetInternalForces();

		const ndVector timestep4(GetTimestepRK());
		const ndVector speedFreeze2(m_world->m_freezeSpeed2 * ndFloat32(0.1f));

		const ndInt32 count = ndInt32 (bodyArray.GetCount() - GetUnconstrainedBodyCount());
		for (ndInt32 i = iterator.fetch_add(D_WORKER_BATCH_SIZE); i < count; i = iterator.fetch_add(D_WORKER_BATCH_SIZE))
		{
			const ndInt32 maxSpan = ((count - i) >= D_WORKER_BATCH_SIZE) ? D_WORKER_BATCH_SIZE : count - i;
			for (ndInt32 j = 0; j < maxSpan; ++j)
			{
				ndBodyKinematic* const body = bodyArray[i + j];

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
			}
		}
	});
	scene->ParallelExecute(IntegrateBodiesVelocity);
}

void ndDynamicsUpdateSoa::CalculateJointsForce()
{
	D_TRACKTIME();
	const ndUnsigned32 passes = m_solverPasses;
	ndScene* const scene = m_world->GetScene();

	ndArray<ndBodyKinematic*>& bodyArray = scene->GetActiveBodyArray();
	ndArray<ndConstraint*>& jointArray = scene->GetActiveContactArray();

	ndAtomic<ndInt32> iterator0(0);
	auto CalculateJointsForce = ndMakeObject::ndFunction([this, &iterator0, &jointArray](ndInt32, ndInt32)
	{
		D_TRACKTIME_NAMED(CalculateJointsForce);
		const ndInt32 jointCount = ndInt32 (jointArray.GetCount());
		ndJacobian* const jointPartialForces = &GetTempInternalForces()[0];

		const ndInt32* const soaJointRows = &m_soaJointRows[0];
		ndSoaMatrixElement* const soaMassMatrix = &m_soaMassMatrix[0];

		auto JointForce = [this, &jointArray, jointPartialForces](ndInt32 group, ndSoaMatrixElement* const massMatrix)
		{
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

					preconditioner0[i] = body0->m_weigh;
					preconditioner1[i] = body1->m_weigh;

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

						preconditioner0[i] = body0->m_weigh;
						preconditioner1[i] = body1->m_weigh;

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
			}

			ndVector accNorm(zero);
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

				a = a & (f < upperFrictionForce) & (f > lowerFrictionForce);
				accNorm = accNorm.MulAdd(a, a);

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

			ndVector maxAccel(accNorm);
			for (ndInt32 k = 0; (k < 4) && (maxAccel.GetMax().GetScalar() > tol2); ++k)
			{
				maxAccel = zero;
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

					a = a & (f < upperFrictionForce) & (f > lowerFrictionForce);
					maxAccel = maxAccel.MulAdd(a, a);

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
					ndAssert(body0);
					ndAssert(body1);
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
						rightHandSide[j + rowStartBase].m_maxImpact = ndMax(ndAbs(row->m_force[i]), rightHandSide[j + rowStartBase].m_maxImpact);
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
		for (ndInt32 i = iterator0.fetch_add(D_WORKER_BATCH_SIZE); i < soaJointCount; i = iterator0.fetch_add(D_WORKER_BATCH_SIZE))
		{
			const ndInt32 maxSpan = ((soaJointCount - i) >= D_WORKER_BATCH_SIZE) ? D_WORKER_BATCH_SIZE : soaJointCount - i;
			for (ndInt32 j = 0; j < maxSpan; ++j)
			{
				const ndInt32 m = i + j;
				JointForce(m, &soaMassMatrix[soaJointRows[m]]);
			}
		}
	});

	ndAtomic<ndInt32> iterator1(0);
	auto ApplyJacobianAccumulatePartialForces = ndMakeObject::ndFunction([this, &iterator1, &bodyArray](ndInt32, ndInt32)
	{
		D_TRACKTIME_NAMED(ApplyJacobianAccumulatePartialForces);
		const ndVector zero(ndVector::m_zero);

		ndJacobian* const internalForces = &GetInternalForces()[0];
		const ndInt32* const bodyIndex = &GetJointForceIndexBuffer()[0];
		const ndJacobian* const jointInternalForces = &GetTempInternalForces()[0];
		const ndJointBodyPairIndex* const jointBodyPairIndexBuffer = &GetJointBodyPairIndexBuffer()[0];

		const ndInt32 bodyCount = ndInt32 (bodyArray.GetCount());
		for (ndInt32 i = iterator1.fetch_add(D_WORKER_BATCH_SIZE); i < bodyCount; i = iterator1.fetch_add(D_WORKER_BATCH_SIZE))
		{
			const ndInt32 maxSpan = ((bodyCount - i) >= D_WORKER_BATCH_SIZE) ? D_WORKER_BATCH_SIZE : bodyCount - i;
			for (ndInt32 j = 0; j < maxSpan; ++j)
			{
				ndVector force(zero);
				ndVector torque(zero);
				const ndInt32 m = i + j;
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
			}
		}
	});

	for (ndUnsigned32 i = 0; i < passes; ++i)
	{
		iterator0 = 0;
		iterator1 = 0;
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
	IntegrateUnconstrainedBodies();
	InitWeights();
	InitBodyArray();
	InitJacobianMatrix();
	CalculateForces();
	IntegrateBodies();
	DetermineSleepStates();
}
