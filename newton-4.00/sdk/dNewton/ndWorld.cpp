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
#include "ndWorldScene.h"
#include "ndBodyDynamic.h"
#include "ndSkeletonList.h"
#include "dModels/ndModel.h"
#include "ndDynamicsUpdate.h"
#include "ndDynamicsUpdateSoa.h"
#include "dModels/ndModelNotify.h"
#include "ndJointBilateralConstraint.h"

#ifdef _D_USE_AVX2_SOLVER
	#include "ndWorldSceneAvx2.h"
	#include "ndDynamicsUpdateAvx2.h"
#endif

#ifdef _D_NEWTON_CUDA
	#include "ndCudaUtils.h"
	#include "ndWorldSceneCuda.h"
	#include "ndDynamicsUpdateCuda.h"
#endif

ndWorld::ndWorld()
	:ndClassAlloc()
	,m_scene(nullptr)
	,m_solver(nullptr)
	,m_jointList()
	,m_modelList()
	,m_skeletonList()
	,m_activeSkeletons(256)
	, m_addRemoveBodiesLock()
	,m_addRemoveJointsLock()
	,m_addRemoveModelsLock()
	,m_timestep(ndFloat32 (0.0f))
	,m_freezeAccel2(D_FREEZE_ACCEL2)
	,m_freezeSpeed2(D_FREEZE_SPEED2)
	,m_averageUpdateTime(ndFloat32(0.0f))
	,m_averageTimestepAcc(ndFloat32(0.0f))
	,m_averageFramesCount(ndFloat32(0.0f))
	,m_lastExecutionTime(ndFloat32(0.0f))
	,m_subSteps(1)
	,m_solverMode(ndStandardSolver)
	,m_solverIterations(4)
	,m_inUpdate(false)
{
	// start the engine thread;
	ndBody::m_uniqueIdCount = 0;
	m_solver = new ndDynamicsUpdate(this);
	m_scene = new ndWorldScene(this);

	ndInt32 steps = 1;
	ndFloat32 freezeAccel2 = m_freezeAccel2;
	//ndFloat32 freezeAlpha2 = m_freezeAlpha2;
	ndFloat32 freezeSpeed2 = m_freezeSpeed2;
	//ndFloat32 freezeOmega2 = m_freezeOmega2;
	for (ndInt32 i = 0; i < D_SLEEP_ENTRIES; ++i) 
	{
		m_sleepTable[i].m_maxAccel = freezeAccel2;
		//m_sleepTable[i].m_maxAlpha = freezeAlpha2;
		m_sleepTable[i].m_maxVeloc = freezeSpeed2;
		//m_sleepTable[i].m_maxOmega = freezeOmega2;
		m_sleepTable[i].m_steps = steps;
		steps += 7;
		freezeAccel2 *= ndFloat32(1.5f);
		//freezeAlpha2 *= ndFloat32(1.5f);
		freezeSpeed2 *= ndFloat32(1.5f);
		//freezeOmega2 *= ndFloat32(1.5f);
	}

	m_sleepTable[0].m_maxAccel *= ndFloat32(0.009f);
	//m_sleepTable[0].m_maxAlpha *= ndFloat32(0.009f);

	steps += 300;
	m_sleepTable[D_SLEEP_ENTRIES - 1].m_maxAccel *= ndFloat32(100.0f);
	//m_sleepTable[D_SLEEP_ENTRIES - 1].m_maxAlpha *= ndFloat32(100.0f);
	m_sleepTable[D_SLEEP_ENTRIES - 1].m_maxVeloc = 0.25f;
	//m_sleepTable[D_SLEEP_ENTRIES - 1].m_maxOmega = 0.1f;
	m_sleepTable[D_SLEEP_ENTRIES - 1].m_steps = steps;
}

ndWorld::~ndWorld()
{
	CleanUp();

	delete m_scene;
	delete m_solver;
	ClearCache();
}

void ndWorld::CleanUp()
{
	Sync();
	//m_scene->m_backgroundThread.Terminate();
	m_scene->PrepareCleanup();

	m_activeSkeletons.Resize(256);
	while (m_skeletonList.GetFirst())
	{
		m_skeletonList.Remove(m_skeletonList.GetFirst());
	}

	while (m_jointList.GetFirst())
	{
		ndJointBilateralConstraint* const joint = *m_jointList.GetFirst()->GetInfo();
		RemoveJoint(joint);
	}

	while (m_scene->m_particleSetList.GetFirst())
	{
		ndBody* const body = *m_scene->m_particleSetList.GetFirst()->GetInfo();
		RemoveBody(body);
	}

	while (m_scene->m_bodyList.GetFirst())
	{
		ndBody* const body = *m_scene->m_bodyList.GetFirst()->GetInfo();
		RemoveBody(body);
	}

	while (m_modelList.GetFirst())
	{
		ndModel* const model = *m_modelList.GetFirst()->GetInfo();
		RemoveModel(model);
	}

	ndBody::m_uniqueIdCount = 1;
	m_scene->Cleanup();
}

const char* ndWorld::GetSolverString() const
{
	return m_solver->GetStringId();
}

bool ndWorld::IsHighPerformanceCompute() const
{
	return m_scene->IsHighPerformanceCompute();
}

void ndWorld::ClearCache()
{
	ndFreeListAlloc::Flush();
}

void ndWorld::Sync() const
{
	m_scene->Sync();
}

ndInt32 ndWorld::GetThreadCount() const
{
	return m_scene->GetThreadCount();
}

void ndWorld::SetThreadCount(ndInt32 count)
{
	m_scene->SetThreadCount(count);

	if (m_scene->m_backgroundThread)
	{
		m_scene->m_backgroundThread->Terminate();
		m_scene->m_backgroundThread->SetThreadCount(count);
	}
}

ndInt32 ndWorld::GetSubSteps() const
{
	return m_subSteps;
}

void ndWorld::SetSubSteps(ndInt32 subSteps)
{
	m_subSteps = ndClamp(subSteps, 1, 16);
}

ndScene* ndWorld::GetScene() const
{
	return m_scene;
}

ndInt32 ndWorld::GetSolverIterations() const
{
	return m_solverIterations;
}

void ndWorld::SetSolverIterations(ndInt32 iterations)
{
	m_solverIterations = ndInt32(ndMax(4, iterations));
}

ndContactNotify* ndWorld::GetContactNotify() const
{
	return m_scene->GetContactNotify();
}

void ndWorld::SetContactNotify(ndContactNotify* const notify)
{
	m_scene->SetContactNotify(notify);
}

ndBodyKinematic* ndWorld::GetSentinelBody() const
{
	return m_scene->GetSentinelBody();
}

const ndBodyListView& ndWorld::GetBodyList() const
{
	return m_scene->GetBodyList();
}

const ndJointList& ndWorld::GetJointList() const
{
	return m_jointList;
}

const ndContactArray& ndWorld::GetContactList() const
{
	return m_scene->GetContactArray();
}

const ndSkeletonList& ndWorld::GetSkeletonList() const
{
	return m_skeletonList;
}

const ndBodyList& ndWorld::GetParticleList() const
{
	return m_scene->m_particleSetList;
}

const ndModelList& ndWorld::GetModelList() const
{
	return m_modelList;
}

ndFloat32 ndWorld::GetUpdateTime() const
{
	return m_lastExecutionTime;
}

ndFloat32 ndWorld::GetAverageUpdateTime() const
{
	return m_averageUpdateTime;
}

ndUnsigned32 ndWorld::GetFrameNumber() const
{
	return m_scene->m_frameNumber;
}

ndUnsigned32 ndWorld::GetSubFrameNumber() const
{
	return m_scene->m_subStepNumber;
}

void ndWorld::DebugScene(ndSceneTreeNotiFy* const notify)
{
	m_scene->DebugScene(notify);
}

ndWorld::ndSolverModes ndWorld::GetSelectedSolver() const
{
	return m_solverMode;
}

ndInt32 ndWorld::GetEngineVersion() const
{
	return D_NEWTON_ENGINE_MAJOR_VERSION * 100 + D_NEWTON_ENGINE_MINOR_VERSION;
}

void ndWorld::SendBackgroundTask(ndBackgroundTask* const job)
{
	m_scene->SendBackgroundTask(job);
}

void ndWorld::UpdateTransforms()
{
	m_scene->UpdateTransform();
}

void ndWorld::PreUpdate(ndFloat32)
{
}

void ndWorld::PostUpdate(ndFloat32)
{
}

void ndWorld::OnSubStepPreUpdate(ndFloat32)
{
}

void ndWorld::OnSubStepPostUpdate(ndFloat32)
{
}

ndSharedPtr<ndBody> ndWorld::GetBody(ndBody* const body) const
{
	return m_scene->GetBody(body);
}

ndInt32 ndWorld::CompareJointByInvMass(const ndJointBilateralConstraint* const jointA, const ndJointBilateralConstraint* const jointB, void*)
{
	ndInt32 modeA = jointA->GetSolverModel();
	ndInt32 modeB = jointB->GetSolverModel();

	if (modeA < modeB) 
	{
		return -1;
	}
	else if (modeA > modeB) 
	{
		return 1;
	}
	else 
	{
		ndFloat32 invMassA = ndMin(jointA->GetBody0()->GetInvMass(), jointA->GetBody1()->GetInvMass());
		ndFloat32 invMassB = ndMin(jointB->GetBody0()->GetInvMass(), jointB->GetBody1()->GetInvMass());
		if (invMassA < invMassB) 
		{
			return -1;
		}
		else if (invMassA > invMassB) 
		{
			return 1;
		}
	}
	return 0;
}

void ndWorld::WorkerUpdate(ndInt32 threadIndex)
{
	m_scene->TaskUpdate(threadIndex);
}

void ndWorld::MainUpdate()
{
	D_TRACKTIME();
	ndUnsigned64 timeAcc = ndGetTimeInMicroseconds();

	m_inUpdate = true;
	m_scene->Begin();

	m_scene->SetTimestep(m_timestep);

	PreUpdate(m_timestep);

	ndInt32 const steps = m_subSteps;
	ndFloat32 timestep = m_timestep / (ndFloat32)steps;
	for (ndInt32 i = 0; i < steps; ++i)
	{
		SubStepUpdate(timestep);
	}

	m_scene->SetTimestep(m_timestep);

	ParticleUpdate(m_timestep);

	UpdateTransforms();
	PostModelTransform();
	PostUpdate(m_timestep);
	m_inUpdate = false;

	m_scene->End();

	m_lastExecutionTime = (ndFloat32)(ndGetTimeInMicroseconds() - timeAcc) * ndFloat32(1.0e-6f);
	CalculateAverageUpdateTime();
}

void ndWorld::ThreadFunction()
{
	MainUpdate();
}

void ndWorld::CalculateAverageUpdateTime()
{
	m_averageFramesCount += ndFloat32 (1.0f);
	m_averageTimestepAcc += m_lastExecutionTime;

	ndAssert(m_averageTimestepAcc >= ndFloat32(0.0f));
	const ndFloat32 movingAverageFrames = ndFloat32 (16);
	if (m_averageFramesCount >= movingAverageFrames)
	{
		m_averageUpdateTime = m_averageTimestepAcc/m_averageFramesCount;
		m_averageTimestepAcc = ndFloat32 (0.0f);

		m_averageFramesCount -= movingAverageFrames;
	}
}

void ndWorld::SubStepUpdate(ndFloat32 timestep)
{
	D_TRACKTIME();

	// do physics step
	OnSubStepPreUpdate(timestep);

	m_scene->m_lru = m_scene->m_lru + 1;
	m_scene->SetTimestep(timestep);

	m_scene->BalanceScene();
	// update skeletons topologies
	UpdateSkeletons();

	m_scene->ApplyExtForce();
	m_scene->InitBodyArray();

	// update the collision system
	m_scene->FindCollidingPairs();
	m_scene->CreateNewContacts();
	m_scene->CalculateContacts();
	m_scene->DeleteDeadContacts();

	// update all special bodies.
	m_scene->UpdateSpecial();

	// Update Particle base physics
	//ParticleUpdate();

	// Update all models
	ModelUpdate();

	// calculate internal forces, integrate bodies and update matrices.
	ndAssert(m_solver);
	m_solver->Update();

	// second pass on models
	ModelPostUpdate();


	OnSubStepPostUpdate(timestep);

	m_scene->m_subStepNumber++;
}

void ndWorld::ParticleUpdate(ndFloat32 timestep)
{
	D_TRACKTIME();
	m_scene->ParticleUpdate(timestep);
}

void ndWorld::ModelUpdate()
{
	D_TRACKTIME();
	ndAtomic<ndInt32> iterator(0);
	auto ModelUpdate = ndMakeObject::ndFunction([this, &iterator](ndInt32, ndInt32)
	{
		D_TRACKTIME_NAMED(ModelUpdate);
		const ndFloat32 timestep = m_scene->GetTimestep();
		const ndArray<ndModel*>& modelList = m_modelList.GetUpdateList();

		const ndInt32 modelCount = ndInt32(modelList.GetCount());
		for (ndInt32 i = iterator++; i < modelCount; i = iterator++)
		{
			D_TRACKTIME_NAMED(ModelUpdate);
			ndModel* const model = modelList[i];
			if (*model->m_notifyCallback)
			{
				model->m_notifyCallback->Update(timestep);
			}
		}
	});

	m_modelList.UpdateDirtyList();
	m_scene->ParallelExecute(ModelUpdate);
}

void ndWorld::ModelPostUpdate()
{
	D_TRACKTIME();
	ndAtomic<ndInt32> iterator(0);
	auto ModelPostUpdate = ndMakeObject::ndFunction([this, &iterator](ndInt32, ndInt32)
	{
		D_TRACKTIME_NAMED(ModelPostUpdate);
		const ndFloat32 timestep = m_scene->GetTimestep();
		const ndArray<ndModel*>& modelList = m_modelList.GetUpdateList();

		const ndInt32 modelCount = ndInt32(modelList.GetCount());
		for (ndInt32 i = iterator++; i < modelCount; i = iterator++)
		{
			ndModel* const model = modelList[i];
			if (*model->m_notifyCallback)
			{
				model->m_notifyCallback->PostUpdate(timestep);
			}
		}
	});
	m_scene->ParallelExecute(ModelPostUpdate);
}

void ndWorld::PostModelTransform()
{
	D_TRACKTIME();
	ndAtomic<ndInt32> iterator(0);
	auto PostModelTransform = ndMakeObject::ndFunction([this, &iterator](ndInt32, ndInt32)
	{
		D_TRACKTIME_NAMED(PostModelTransform);
		const ndFloat32 timestep = m_scene->GetTimestep();
		const ndArray<ndModel*>& modelList = m_modelList.GetUpdateList();

		const ndInt32 modelCount = ndInt32(modelList.GetCount());
		for (ndInt32 i = iterator++; i < modelCount; i = iterator++)
		{
			ndModel* const model = modelList[i];
			if (*model->m_notifyCallback)
			{
				model->m_notifyCallback->PostTransformUpdate(timestep);
			}
		}
	});

	m_modelList.UpdateDirtyList();
	m_scene->ParallelExecute(PostModelTransform);
}

bool ndWorld::SkeletonJointTest(ndJointBilateralConstraint* const constraint) const
{
	bool test = true;
	ndAssert(constraint && constraint->GetAsBilateral());
	test = test && (constraint->GetRowsCount() > 0);
	test = test && (constraint->IsSkeleton());
	return test;
}

void ndWorld::UpdateSkeletons()
{
	D_TRACKTIME();
	if (m_skeletonList.m_skelListIsDirty)
	{
		m_skeletonList.m_skelListIsDirty = false;
		while (m_skeletonList.GetFirst())
		{
			m_skeletonList.Remove(m_skeletonList.GetFirst());
		}

		// reset of all bodies dirty state
		const ndArray<ndBodyKinematic*>& bodyArray = m_scene->GetActiveBodyArray();
		for (ndInt32 i = ndInt32(bodyArray.GetCount()) - 1; i >= 0; i--)
		{
			ndBodyKinematic* const body = bodyArray[i];
			body->m_index = -1;
			body->m_skeletonMark = 0;
			body->m_skeletonMark0 = 0;
			body->m_skeletonMark1 = 0;
			body->m_islandParent = body;
		}

		// build connectivity graph and reset of all joint dirty state
		ndDynamicsUpdate& solverUpdate = *m_solver;
		for (ndJointList::ndNode* node = m_jointList.GetFirst(); node; node = node->GetNext())
		{
			ndJointBilateralConstraint* const constraint = *node->GetInfo();
			const bool test = SkeletonJointTest(constraint);
			constraint->m_mark0 = 0;
			constraint->m_mark1 = 0;
			if (test)
			{
				ndBodyKinematic* const body0 = constraint->GetBody0();
				ndBodyKinematic* const body1 = constraint->GetBody1();
				if (body1->GetInvMass() > ndFloat32(0.0f))
				{
					ndBodyKinematic* root0 = solverUpdate.FindRootAndSplit(body0);
					ndBodyKinematic* root1 = solverUpdate.FindRootAndSplit(body1);
					if (root0 != root1)
					{
						if (root0->m_buildSkelIndex > root1->m_buildSkelIndex)
						{
							ndSwap(root0, root1);
						}
						root0->m_islandParent = root1;
						if (root0->m_buildSkelIndex == root1->m_buildSkelIndex)
						{
							root1->m_buildSkelIndex += 1;
							ndAssert(root1->m_buildSkelIndex <= 6);
						}
					}
				}
			}
		}
	
		// find all root nodes for all independent joint arrangements
		ndInt32 inslandCount = 0;
		solverUpdate.m_leftHandSide.SetCount(ndMax(ndInt32 (bodyArray.GetCount()) + 256, 1024));
		ndIslandMember* const islands = (ndIslandMember*)&solverUpdate.m_leftHandSide[0];
		for (ndInt32 i = 0; i < bodyArray.GetCount(); ++i)
		{
			ndBodyKinematic* const body = bodyArray[i];
			if (body->GetInvMass() > ndFloat32(0.0f))
			{
				ndBodyKinematic* const root = solverUpdate.FindRootAndSplit(body);
				if (root->m_index == -1)
				{
					ndIslandMember& entry = islands[inslandCount];
					entry.m_body = body;
					entry.m_root = body;
					root->m_index = inslandCount;
					inslandCount++;
				}
				ndInt32 index = root->m_index;
				ndAssert(index != -1);
				ndIslandMember& entry = islands[index];
				if (body->GetInvMass() < entry.m_body->GetInvMass())
				{
					entry.m_body = body;
				}
			}
		}
	
		// build the root node
		ndInt32 skeletonsId = 0;
		for (ndInt32 i = 0; i < inslandCount; ++i)
		{
			ndSkeletonContainer::ndQueue queuePool;
			ndFixSizeArray<ndBodyKinematic*, 512> stack;

			stack.PushBack(islands[i].m_body);
			ndSkeletonContainer* skeleton = nullptr;
			
			// find if this root node is connected to static bodies 
			// if so, them make that static body the root node and add all the children
			while (stack.GetCount())
			{
				ndBodyKinematic* const rootBody = stack.Pop();
				if (!rootBody->m_skeletonMark1)
				{
					rootBody->m_skeletonMark1 = 1;
					for (ndBodyKinematic::ndJointList::ndNode* jointNode = rootBody->m_jointList.GetFirst(); jointNode; jointNode = jointNode->GetNext())
					{
						ndJointBilateralConstraint* const constraint = jointNode->GetInfo();
						ndAssert(constraint && constraint->GetAsBilateral());
						if (!constraint->m_mark1)
						{
							constraint->m_mark1 = 1;
							const bool test = SkeletonJointTest(constraint);
							if (test && (constraint->GetSolverModel() != m_jointkinematicCloseLoop))
							{
								ndBodyKinematic* const childBody = (constraint->GetBody0() != rootBody) ? constraint->GetBody0() : constraint->GetBody1();
								if (childBody->GetInvMass() == ndFloat32(0.0f))
								{
									if (!skeleton)
									{
										skeleton = m_skeletonList.CreateContatiner(childBody, skeletonsId);
										skeletonsId++;
									}
			
									//dTrace(("%s %d %d\n", constraint->GetClassName(), constraint->GetBody0()->GetId(), constraint->GetBody1()->GetId()));
									constraint->m_mark0 = 1;
									ndAssert(childBody == skeleton->GetRoot()->m_body);
									ndSkeletonContainer::ndNode* const node = skeleton->AddChild((ndJointBilateralConstraint*)constraint, skeleton->GetRoot());
									node->m_body->m_skeletonMark = 1;
									ndAssert(node->m_body != childBody);
									queuePool.Push(node);
								}
								else if (!childBody->m_skeletonMark1)
								{
									stack.PushBack(childBody);
								}
							}
						}
					}
				}
			}
			
			if (queuePool.IsEmpty())
			{
				// if this root node is not static, 
				// them add the first children to this root
				bool hasJoints = false;
				ndBodyKinematic* const rootBody = islands[i].m_body;
				rootBody->m_skeletonMark0 = 1;
				for (ndBodyKinematic::ndJointList::ndNode* jointNode = rootBody->m_jointList.GetFirst(); jointNode; jointNode = jointNode->GetNext())
				{
					ndJointBilateralConstraint* const constraint = jointNode->GetInfo();
				
					const bool test = SkeletonJointTest(constraint);
					if (test && (constraint->GetSolverModel() != m_jointkinematicCloseLoop))
					{
						ndBodyKinematic* const childBody = (constraint->GetBody0() != rootBody) ? constraint->GetBody0() : constraint->GetBody1();
						if (childBody->GetInvMass())
						{
							hasJoints = true;
							break;
						}
					}
				}
			
				if (hasJoints)
				{
					// the root node is not static and has children, 
					// them add the first children to this root
					skeleton = m_skeletonList.CreateContatiner(rootBody, skeletonsId);
					skeletonsId++;
			
					for (ndBodyKinematic::ndJointList::ndNode* jointNode = rootBody->m_jointList.GetFirst(); jointNode; jointNode = jointNode->GetNext())
					{
						ndJointBilateralConstraint* const constraint = jointNode->GetInfo();
						//ndTrace(("%s %d %d\n", constraint->ClassName(), constraint->GetBody0()->GetId(), constraint->GetBody1()->GetId()));
						const bool test = SkeletonJointTest(constraint);
						if (test && (constraint->GetSolverModel() != m_jointkinematicCloseLoop))
						{
							constraint->m_mark0 = 1;
							ndAssert(skeleton->GetRoot()->m_body != ((constraint->GetBody0() != rootBody) ? constraint->GetBody0() : constraint->GetBody1()));
							ndSkeletonContainer::ndNode* const node = skeleton->AddChild((ndJointBilateralConstraint*)constraint, skeleton->GetRoot());
							node->m_body->m_skeletonMark = 1;
							ndAssert(node->m_body == ((constraint->GetBody0() != rootBody) ? constraint->GetBody0() : constraint->GetBody1()));
							queuePool.Push(node);
						}
					}
				}
			}
			
			if (skeleton)
			{
				// add the rest of the children to this skeleton
				ndInt32 loopCount = 0;
				ndJointBilateralConstraint* loopJoints[128];
			
				while (!queuePool.IsEmpty())
				{
					ndInt32 count = queuePool.m_firstIndex - queuePool.m_lastIndex;
					if (count < 0)
					{
						count += queuePool.m_mod;
					}
			
					ndInt32 index = queuePool.m_lastIndex;
					queuePool.Reset();
			
					for (ndInt32 j = 0; j < count; ++j)
					{
						ndSkeletonContainer::ndNode* const parentNode = queuePool[index];
						ndBodyKinematic* const parentBody = parentNode->m_body;
						if (!parentBody->m_skeletonMark0)
						{
							parentBody->m_skeletonMark0 = 1;
							for (ndBodyKinematic::ndJointList::ndNode* jointNode = parentBody->m_jointList.GetFirst(); jointNode; jointNode = jointNode->GetNext())
							{
								ndJointBilateralConstraint* const constraint = jointNode->GetInfo();
								if (!constraint->m_mark0)
								{
									//ndTrace(("%s %d %d\n", constraint->ClassName(), constraint->GetBody0()->GetId(), constraint->GetBody1()->GetId()));
									constraint->m_mark0 = 1;
									if (SkeletonJointTest(constraint))
									{
										ndBodyKinematic* const childBody = (constraint->GetBody0() == parentBody) ? constraint->GetBody1() : constraint->GetBody0();
										if (!childBody->m_skeletonMark && (childBody->GetInvMass() != ndFloat32(0.0f)) && (constraint->GetSolverModel() != m_jointkinematicCloseLoop))
										{
											childBody->m_skeletonMark = 1;
											ndSkeletonContainer::ndNode* const childNode = skeleton->AddChild(constraint, parentNode);
											queuePool.Push(childNode);
										}
										else if (loopCount < ndInt32 ((sizeof(loopJoints) / sizeof(loopJoints[0]))))
										{
											loopJoints[loopCount] = (ndJointBilateralConstraint*)constraint;
											loopCount++;
										}
									}
								}
							}
						}
			
						index++;
						if (index >= queuePool.m_mod)
						{
							index = 0;
						}
					}
				}
				skeleton->Finalize(loopCount, loopJoints);
			}
		}
	
		m_activeSkeletons.SetCount(0);
		ndSkeletonList::Iterator iter(m_skeletonList);
		for (iter.Begin(); iter; iter++)
		{
			ndSkeletonContainer* const skeleton = &iter.GetNode()->GetInfo();
			m_activeSkeletons.PushBack(skeleton);
		}
	}
	
	for (ndInt32 i = 0; i < m_activeSkeletons.GetCount(); ++i)
	{
		ndSkeletonContainer* const skeleton = m_activeSkeletons[i];
		skeleton->ClearCloseLoopJoints();
	}
}

bool ndWorld::RayCast(ndRayCastNotify& callback, const ndVector& globalOrigin, const ndVector& globalDest) const
{
	return m_scene->RayCast(callback, globalOrigin, globalDest);
}

bool ndWorld::ConvexCast(ndConvexCastNotify& callback, const ndShapeInstance& convexShape, const ndMatrix& globalOrigin, const ndVector& globalDest) const
{
	return m_scene->ConvexCast(callback, convexShape, globalOrigin, globalDest);
}

void ndWorld::BodiesInAabb(ndBodiesInAabbNotify& callback, const ndVector& minBox, const ndVector& maxBox) const
{
	m_scene->BodiesInAabb(callback, minBox, maxBox);
}

void ndWorld::SelectSolver(ndSolverModes solverMode)
{
	if (solverMode != m_solverMode)
	{
		Sync();
		delete m_solver;
		switch (solverMode)
		{
			case ndSimdSoaSolver:
			{
				ndWorldScene* const newScene = new ndWorldScene(*((ndWorldScene*)m_scene));
				delete m_scene;
				m_scene = newScene;

				m_solverMode = solverMode;
				m_solver = new ndDynamicsUpdateSoa(this);
				break;
			}

			case ndSimdAvx2Solver:
			{
				#ifdef _D_USE_AVX2_SOLVER
					ndWorldScene* const newScene = new ndWorldSceneAvx2(*((ndWorldScene*)m_scene));
					delete m_scene;
					m_scene = newScene;

					m_solverMode = solverMode;
					m_solver = new ndDynamicsUpdateAvx2(this);
				#else
					ndWorldScene* const newScene = new ndWorldScene(*((ndWorldScene*)m_scene));
					delete m_scene;
					m_scene = newScene;

					m_solverMode = ndSimdSoaSolver;
					m_solver = new ndDynamicsUpdateSoa(this);
				#endif
				break;
			}

			case ndCudaSolver:
			{
				#ifdef _D_NEWTON_CUDA
					ndMemFreeCallback freeMemory;
					ndMemAllocCallback allocMemory;
					ndMemory::GetMemoryAllocators(allocMemory, freeMemory);
					ndCudaContext::SetMemoryAllocators(allocMemory, freeMemory);
					ndWorldScene* const newScene = new ndWorldSceneCuda(*((ndWorldScene*)m_scene));
					delete m_scene;
					m_scene = newScene;
					m_solverMode = solverMode;
					m_solver = new ndDynamicsUpdateCuda(this);
					if (!newScene->IsValid())
					{
						delete m_solver;
						ndWorldScene* const defaultScene = new ndWorldScene(*((ndWorldScene*)m_scene));
						delete m_scene;
						m_scene = defaultScene;

						m_solverMode = ndSimdSoaSolver;
						m_solver = new ndDynamicsUpdateSoa(this);
					}
				#else
					ndWorldScene* const newScene = new ndWorldScene(*((ndWorldScene*)m_scene));
					delete m_scene;
					m_scene = newScene;
					m_solverMode = ndSimdSoaSolver;
					m_solver = new ndDynamicsUpdateSoa(this);
				#endif
				break;
			}

			case ndStandardSolver:
			default:
			{
				ndWorldScene* const newScene = new ndWorldScene(*((ndWorldScene*)m_scene));
				delete m_scene;
				m_scene = newScene;

				m_solverMode = ndStandardSolver;
				m_solver = new ndDynamicsUpdate(this);
				break;
			}
		}

		#ifdef _DEBUG
		for (ndBodyListView::ndNode* node = m_scene->GetBodyList().GetFirst(); node; node = node->GetNext())
		{
			ndBodyKinematic* const body = node->GetInfo()->GetAsBodyKinematic();
			ndAssert(body->GetContactMap().SanityCheck());
		}
		#endif
	}
}

void ndWorld::CollisionUpdate(ndFloat32 timestep)
{
	// wait until previous update complete.
	Sync();
	m_timestep = timestep;

	// update the next frame asynchronous 
	m_scene->TickOne();
}

void ndWorld::Update(ndFloat32 timestep)
{
	// wait until previous update complete.
	Sync();

	// save time state for use by the update callback
	m_timestep = timestep;

	// update the next frame asynchronous 
	m_scene->TickOne();
}

void ndWorld::CalculateJointContacts(ndContact* const contact)
{
	ndBodyKinematic* const body0 = contact->GetBody0();
	ndBodyKinematic* const body1 = contact->GetBody1();
	body0->UpdateCollisionMatrix();
	body1->UpdateCollisionMatrix();

	m_scene->CalculateJointContacts(0, contact);
}

bool ndWorld::ValidateScene() const
{
	return m_scene->ValidateScene();
}


void ndWorld::AddModel(const ndSharedPtr<ndModel>& model)
{
	ndScopeSpinLock lock(m_addRemoveModelsLock);
	m_modelList.AddModel(model, this);
}

void ndWorld::RemoveModel(ndModel* const model)
{
	ndScopeSpinLock lock(m_addRemoveModelsLock);
	m_modelList.RemoveModel(model);
}

void ndWorld::AddJoint(const ndSharedPtr<ndJointBilateralConstraint>& joint)
{
	// if the second body is nullPtr, replace it the sentinel
	ndScopeSpinLock lock(m_addRemoveJointsLock);
	ndAssert(joint->m_body0);
	ndAssert(joint->m_body1);
	ndAssert(joint->IsActive());
	if (joint->m_worldNode == nullptr)
	{
		ndAssert(joint->m_body0Node == nullptr);
		ndAssert(joint->m_body1Node == nullptr);
		if (joint->IsSkeleton())
		{
			m_skeletonList.m_skelListIsDirty = true;
		}
		joint->m_worldNode = m_jointList.Append(joint);
		joint->m_body0Node = joint->GetBody0()->AttachJoint((ndJointBilateralConstraint*)*joint);
		joint->m_body1Node = joint->GetBody1()->AttachJoint((ndJointBilateralConstraint*)*joint);
	}
}

void ndWorld::RemoveJoint(ndJointBilateralConstraint* const joint)
{
	ndScopeSpinLock lock(m_addRemoveJointsLock);
	ndJointList::ndNode* const worldNode = joint->m_worldNode;
	if (worldNode != nullptr)
	{
		ndAssert(joint->m_body0Node != nullptr);
		ndAssert(joint->m_body1Node != nullptr);
		joint->GetBody0()->DetachJoint(joint->m_body0Node);
		joint->GetBody1()->DetachJoint(joint->m_body1Node);

		if (joint->IsSkeleton())
		{
			m_skeletonList.m_skelListIsDirty = true;
			ndSkeletonContainer* const skeleton = joint->GetBody0()->GetSkeleton();
			if (skeleton)
			{
				skeleton->Clear();
			}
		}

		joint->SetActive(true);
		joint->m_worldNode = nullptr;
		joint->m_body0Node = nullptr;
		joint->m_body1Node = nullptr;
		m_jointList.Remove(worldNode);
	}
}

bool ndWorld::AddBody(const ndSharedPtr<ndBody>& body)
{
	ndScopeSpinLock lock(m_addRemoveBodiesLock);
	ndAssert(GetSentinelBody() != body->GetAsBodyKinematic());
	return m_scene->AddBody(body);
}

void ndWorld::RemoveBody(ndBody* const body)
{
	ndScopeSpinLock lock(m_addRemoveBodiesLock);
	ndBodyKinematic* const kinematicBody = body->GetAsBodyKinematic();
	ndAssert(kinematicBody != GetSentinelBody());
	if (kinematicBody->m_sceneNode)
	{
		const ndBodyKinematic::ndJointList& jointList = kinematicBody->GetJointList();
		for (ndBodyKinematic::ndJointList::ndNode* jointNode = jointList.GetFirst(); jointNode; )
		{
			ndJointBilateralConstraint* const joint = jointNode->GetInfo();
			jointNode = jointNode->GetNext();
			RemoveJoint(joint);
		}
		ndSharedPtr<ndBody> sharedBody(GetBody(body));
		m_scene->RemoveBody(sharedBody);
	}
}
