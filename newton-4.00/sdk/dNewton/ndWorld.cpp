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
#include "ndModel.h"
#include "ndWorldScene.h"
#include "ndBodyDynamic.h"
#include "ndSkeletonList.h"
#include "ndDynamicsUpdate.h"
#include "ndBodyParticleSet.h"
#include "ndDynamicsUpdateSoa.h"
#include "ndJointBilateralConstraint.h"

#ifdef _D_USE_AVX2_SOLVER
	#include "ndDynamicsUpdateAvx2.h"
#endif

#ifdef _D_NEWTON_OPENCL
	#include "ndDynamicsUpdateOpencl.h"
#endif

class ndSkeletonQueue : public ndFixSizeArray<ndSkeletonContainer::ndNode*, 1024 * 4>
{
	public:
	ndSkeletonQueue()
		:ndFixSizeArray<ndSkeletonContainer::ndNode*, 1024 * 4>()
		,m_mod(sizeof(m_array) / sizeof(m_array[0]))
	{
		m_lastIndex = 0;
		m_firstIndex = 0;
	}

	void Push(ndSkeletonContainer::ndNode* const node)
	{
		m_firstIndex++;
		if (node->m_joint->GetSolverModel() != m_jointkinematicOpenLoop)
		{
			m_array[m_firstIndex-1] = node;
		}
		else
		{
			const ndInt32 count = m_firstIndex - m_lastIndex;
			ndInt32 slot = count - 1;
			for (; (slot > 0) && (m_array[m_lastIndex + slot - 1]->m_joint->GetSolverModel() != m_jointkinematicOpenLoop); slot--)
			{
				m_array[m_lastIndex + slot] = m_array[m_lastIndex + slot - 1];
			}
			m_array[m_lastIndex + slot] = node;
		}

		if (m_firstIndex >= m_mod)
		{
			m_firstIndex = 0;
		}
		dAssert(m_firstIndex != m_lastIndex);
	}

	void Reset()
	{
		m_lastIndex = m_firstIndex;
	}

	bool IsEmpty() const
	{
		return (m_firstIndex == m_lastIndex);
	}

	ndInt32 m_lastIndex;
	ndInt32 m_firstIndex;
	ndInt32 m_mod;
};

ndWorld::ndWorld()
	:ndClassAlloc()
	,m_scene(nullptr)
	,m_solver(nullptr)
	,m_jointList()
	,m_modelList()
	,m_skeletonList()
	,m_particleSetList()
	,m_activeSkeletons(256)
	,m_timestep(ndFloat32 (0.0f))
	,m_freezeAccel2(D_FREEZE_ACCEL2)
	,m_freezeAlpha2(D_FREEZE_ACCEL2)
	,m_freezeSpeed2(D_FREEZE_SPEED2)
	,m_freezeOmega2(D_FREEZE_SPEED2)
	,m_averageUpdateTime(ndFloat32(0.0f))
	,m_averageTimestepAcc(ndFloat32(0.0f))
	,m_averageFramesCount(ndFloat32(0.0f))
	,m_lastExecutionTime(ndFloat32(0.0f))
	,m_subSteps(1)
	,m_solverMode(ndStandardSolver)
	,m_solverIterations(4)
	,m_frameIndex(0)
	,m_transformsLock()
	,m_inUpdate(false)
	,m_collisionUpdate(true)
{
	// start the engine thread;
	ndBody::m_uniqueIdCount = 0;
	m_scene = new ndWorldDefaultScene(this);
	m_solver = new ndDynamicsUpdate(this);

	ndInt32 steps = 1;
	ndFloat32 freezeAccel2 = m_freezeAccel2;
	ndFloat32 freezeAlpha2 = m_freezeAlpha2;
	ndFloat32 freezeSpeed2 = m_freezeSpeed2;
	ndFloat32 freezeOmega2 = m_freezeOmega2;
	for (ndInt32 i = 0; i < D_SLEEP_ENTRIES; i++) 
	{
		m_sleepTable[i].m_maxAccel = freezeAccel2;
		m_sleepTable[i].m_maxAlpha = freezeAlpha2;
		m_sleepTable[i].m_maxVeloc = freezeSpeed2;
		m_sleepTable[i].m_maxOmega = freezeOmega2;
		m_sleepTable[i].m_steps = steps;
		steps += 7;
		freezeAccel2 *= ndFloat32(1.5f);
		freezeAlpha2 *= ndFloat32(1.5f);
		freezeSpeed2 *= ndFloat32(1.5f);
		freezeOmega2 *= ndFloat32(1.5f);
	}

	m_sleepTable[0].m_maxAccel *= ndFloat32(0.009f);
	m_sleepTable[0].m_maxAlpha *= ndFloat32(0.009f);

	steps += 300;
	m_sleepTable[D_SLEEP_ENTRIES - 1].m_maxAccel *= ndFloat32(100.0f);
	m_sleepTable[D_SLEEP_ENTRIES - 1].m_maxAlpha *= ndFloat32(100.0f);
	m_sleepTable[D_SLEEP_ENTRIES - 1].m_maxVeloc = 0.25f;
	m_sleepTable[D_SLEEP_ENTRIES - 1].m_maxOmega = 0.1f;
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
	m_scene->m_backgroundThread.Terminate();

	m_activeSkeletons.Resize(256);
	while (m_skeletonList.GetFirst())
	{
		m_skeletonList.Remove(m_skeletonList.GetFirst());
	}

	while (m_modelList.GetFirst())
	{
		ndModel* const model = m_modelList.GetFirst()->GetInfo();
		RemoveModel(model);
		delete model;
	}

	while (m_jointList.GetFirst())
	{
		ndJointBilateralConstraint* const joint = m_jointList.GetFirst()->GetInfo();
		RemoveJoint(joint);
		delete joint;
	}

	while (m_particleSetList.GetFirst())
	{
		ndBodyParticleSet* const body = m_particleSetList.GetFirst()->GetInfo();
		RemoveBody(body);
		delete body;
	}

	ndBody::m_uniqueIdCount = 1;
	m_scene->Cleanup();
}

void ndWorld::SelectSolver(ndSolverModes solverMode)
{
	if (solverMode != m_solverMode)
	{
		delete m_solver;
		switch (solverMode)
		{
			case ndSimdSoaSolver:
				m_solverMode = solverMode;
				m_solver = new ndDynamicsUpdateSoa(this);
				break;

			#ifdef _D_USE_AVX2_SOLVER
			case ndSimdAvx2Solver:
				m_solverMode = solverMode;
				m_solver = new ndDynamicsUpdateAvx2(this);
				break;
			#endif

			#ifdef _D_NEWTON_OPENCL
			case ndOpenclSolver1:
				m_solverMode = solverMode;
				m_solver = new ndDynamicsUpdateOpencl(this, 0);
				break;

			case ndOpenclSolver2:
				m_solverMode = solverMode;
				m_solver = new ndDynamicsUpdateOpencl(this, 1);
				break;
			#endif

			case ndStandardSolver:
			default:
				m_solverMode = ndStandardSolver;
				m_solver = new ndDynamicsUpdate(this);
				break;
		}
	}
}

const char* ndWorld::GetSolverString() const
{
	return m_solver->GetStringId();
}

void ndWorld::ClearCache()
{
	ndFreeListAlloc::Flush();
}

void ndWorld::UpdateTransforms()
{
	for (ndBodyParticleSetList::ndNode* node = m_particleSetList.GetFirst(); node; node = node->GetNext())
	{
		ndBodyParticleSet* const particleSet = node->GetInfo();
		particleSet->GetNotifyCallback()->OnTransform(0, particleSet->GetMatrix());
	}

	m_scene->UpdateTransform();
}

void ndWorld::PostUpdate(ndFloat32)
{
	D_TRACKTIME();
	OnPostUpdate(m_timestep);
}

bool ndWorld::AddBody(ndBody* const body)
{
	ndBodyKinematic* const kinematicBody = body->GetAsBodyKinematic();
	dAssert(kinematicBody != GetSentinelBody());
	if (kinematicBody)
	{
		return m_scene->AddBody(kinematicBody);
	}
	else if (body->GetAsBodyParticleSet())
	{
		ndBodyParticleSet* const particleSet = body->GetAsBodyParticleSet();
		dAssert(particleSet->m_listNode == nullptr);
		ndBodyParticleSetList::ndNode* const node = m_particleSetList.Append(particleSet);
		particleSet->m_listNode = node;
	}
	return false;
}

void ndWorld::RemoveBody(ndBody* const body)
{
	dAssert(!m_inUpdate);
	ndBodyKinematic* const kinematicBody = body->GetAsBodyKinematic();
	dAssert(kinematicBody != GetSentinelBody());
	if (kinematicBody)
	{
		const ndJointList& jointList = kinematicBody->GetJointList();
		while (jointList.GetFirst())
		{
			ndJointBilateralConstraint* const joint = jointList.GetFirst()->GetInfo();
			RemoveJoint(joint);
		}

		m_scene->RemoveBody(kinematicBody);
	}
	else if (body->GetAsBodyParticleSet())
	{
		ndBodyParticleSet* const particleSet = body->GetAsBodyParticleSet();
		dAssert(particleSet->m_listNode);
		m_particleSetList.Remove(particleSet->m_listNode);
	}
}

void ndWorld::DeleteBody(ndBody* const body)
{
	dAssert(!m_inUpdate);
	RemoveBody(body);
	ndBodyKinematic* const kinematicBody = body->GetAsBodyKinematic();
	if (kinematicBody)
	{
		while (kinematicBody->m_jointList.GetFirst())
		{
			ndJointBilateralConstraint* const joint = kinematicBody->m_jointList.GetFirst()->GetInfo();
			RemoveJoint(joint);
			delete joint;
		}
	}
	delete body;
}

void ndWorld::AddJoint(ndJointBilateralConstraint* const joint)
{
	// if the second body is nullPtr, replace it the sentinel
	if (joint->m_body1 == nullptr)
	{
		joint->m_body1 = GetSentinelBody();
	}
	if (joint->m_worldNode == nullptr)
	{
		dAssert(joint->m_body0Node == nullptr);
		dAssert(joint->m_body1Node == nullptr);
		if (joint->IsSkeleton())
		{
			m_skeletonList.m_skelListIsDirty = true;
		}
		joint->m_worldNode = m_jointList.Append(joint);
		joint->m_body0Node = joint->GetBody0()->AttachJoint(joint);
		joint->m_body1Node = joint->GetBody1()->AttachJoint(joint);
	}
}

void ndWorld::RemoveJoint(ndJointBilateralConstraint* const joint)
{
	if (joint->m_worldNode != nullptr)
	{
		dAssert(!m_inUpdate);
		dAssert(joint->m_body0Node != nullptr);
		dAssert(joint->m_body1Node != nullptr);
		joint->GetBody0()->DetachJoint(joint->m_body0Node);
		joint->GetBody1()->DetachJoint(joint->m_body1Node);

		m_jointList.Remove(joint->m_worldNode);
		if (joint->IsSkeleton())
		{
			m_skeletonList.m_skelListIsDirty = true;
		}
		joint->m_worldNode = nullptr;
		joint->m_body0Node = nullptr;
		joint->m_body1Node = nullptr;
	}
}

void ndWorld::AddModel(ndModel* const model)
{
	if (!model->m_node)
	{
		model->AddToWorld(this);
		model->m_node = m_modelList.Append(model);
	}
}

void ndWorld::RemoveModel(ndModel* const model)
{
	dAssert(!m_inUpdate);
	if (model->m_node)
	{
		model->RemoveFromToWorld(this);
		m_modelList.Remove(model->m_node);
		model->m_node = nullptr;
	}
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
		ndFloat32 invMassA = dMin(jointA->GetBody0()->GetInvMass(), jointA->GetBody1()->GetInvMass());
		ndFloat32 invMassB = dMin(jointB->GetBody0()->GetInvMass(), jointB->GetBody1()->GetInvMass());
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

void ndWorld::ThreadFunction()
{
	ndUnsigned64 timeAcc = dGetTimeInMicroseconds();
	const bool collisionUpdate = m_collisionUpdate;
	m_inUpdate = true;

	if (collisionUpdate)
	{
		m_collisionUpdate = true;
		m_scene->CollisionOnlyUpdate();
		m_inUpdate = false;
	}
	else
	{
		D_TRACKTIME();
		m_scene->Begin();
		m_collisionUpdate = true;
		m_scene->BalanceScene();

		ndInt32 const steps = m_subSteps;
		ndFloat32 timestep = m_timestep / steps;
		for (ndInt32 i = 0; i < steps; i++)
		{
			SubStepUpdate(timestep);
		}

		m_scene->SetTimestep(m_timestep);
		
		ParticleUpdate(m_timestep);
		UpdateTransformsLock();
		UpdateTransforms();
		PostModelTransform();
		m_inUpdate = false;
		PostUpdate(m_timestep);
		UpdateTransformsUnlock();

		m_scene->End();
	}
	
	m_frameIndex++;
	m_lastExecutionTime = (dGetTimeInMicroseconds() - timeAcc) * ndFloat32(1.0e-6f);
	CalculateAverageUpdateTime();
}

void ndWorld::CalculateAverageUpdateTime()
{
	m_averageFramesCount += ndFloat32 (1.0f);
	m_averageTimestepAcc += m_lastExecutionTime;

	dAssert(m_averageTimestepAcc >= ndFloat32(0.0f));
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

	// do the a pre-physics step
	m_scene->m_lru = m_scene->m_lru + 1;
	m_scene->SetTimestep(timestep);
	m_scene->InitBodyArray();

	// update the collision system
	m_scene->FindCollidingPairs();
	m_scene->CalculateContacts();

	// update all special bodies.
	m_scene->UpdateSpecial();

	// Update Particle base physics
	//ParticleUpdate();

	// Update all models
	UpdateSkeletons();
	ModelUpdate();

	// calculate internal forces, integrate bodies and update matrices.
	dAssert(m_solver);
	m_solver->Update();

	// second pass on models
	ModelPostUpdate();
}

void ndWorld::ParticleUpdate(ndFloat32 timestep)
{
	D_TRACKTIME();
	for (ndBodyParticleSetList::ndNode* node = m_particleSetList.GetFirst(); node; node = node->GetNext())
	{
		ndBodyParticleSet* const body = node->GetInfo();
		body->Update(this, timestep);
	}
}

void ndWorld::ModelUpdate()
{
	D_TRACKTIME();
	auto ModelUpdate = ndMakeObject::ndFunction([this](ndInt32 threadIndex, ndInt32 threadCount)
	{
		D_TRACKTIME();
		const ndFloat32 timestep = m_scene->GetTimestep();
		ndModelList& modelList = m_modelList;
		ndModelList::ndNode* node = modelList.GetFirst();
		for (ndInt32 i = 0; i < threadIndex; i++)
		{
			node = node ? node->GetNext() : nullptr;
		}

		while (node)
		{
			ndModel* const model = node->GetInfo();
			model->Update(this, timestep);

			for (ndInt32 i = 0; i < threadCount; i++)
			{
				node = node ? node->GetNext() : nullptr;
			}
		}
	});
	m_scene->ParallelExecute(ModelUpdate);
}

void ndWorld::ModelPostUpdate()
{
	D_TRACKTIME();
	auto ModelPostUpdate = ndMakeObject::ndFunction([this](ndInt32 threadIndex, ndInt32 threadCount)
	{
		D_TRACKTIME();
		const ndFloat32 timestep = m_scene->GetTimestep();
		ndModelList& modelList = m_modelList;
		ndModelList::ndNode* node = modelList.GetFirst();
		for (ndInt32 i = 0; i < threadIndex; i++)
		{
			node = node ? node->GetNext() : nullptr;
		}

		while (node)
		{
			ndModel* const model = node->GetInfo();
			model->PostUpdate(this, timestep);

			for (ndInt32 i = 0; i < threadCount; i++)
			{
				node = node ? node->GetNext() : nullptr;
			}
		}
	});
	m_scene->ParallelExecute(ModelPostUpdate);
}

void ndWorld::PostModelTransform()
{
	D_TRACKTIME();
	auto PostModelTransform = ndMakeObject::ndFunction([this](ndInt32 threadIndex, ndInt32 threadCount)
	{
		D_TRACKTIME();
		const ndFloat32 timestep = m_scene->GetTimestep();
		ndModelList& modelList = m_modelList;
		ndModelList::ndNode* node = modelList.GetFirst();
		for (ndInt32 i = 0; i < threadIndex; i++)
		{
			node = node ? node->GetNext() : nullptr;
		}

		while (node)
		{
			ndModel* const model = node->GetInfo();
			model->PostTransformUpdate(this, timestep);

			for (ndInt32 i = 0; i < threadCount; i++)
			{
				node = node ? node->GetNext() : nullptr;
			}
		}
	});
	m_scene->ParallelExecute(PostModelTransform);
}

bool ndWorld::SkeletonJointTest(ndJointBilateralConstraint* const constraint) const
{
	bool test = true;
	dAssert(constraint && constraint->GetAsBilateral());
	test = test && (constraint->m_preconditioner0 == ndFloat32(1.0f));
	test = test && (constraint->m_preconditioner1 == ndFloat32(1.0f));
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

		// build connectivity graph and reset of all joint dirty state
		ndDynamicsUpdate& solverUpdate = *m_solver;
		for (ndJointList::ndNode* node = m_jointList.GetFirst(); node; node = node->GetNext())
		{
			ndJointBilateralConstraint* const constraint = node->GetInfo();
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
				}
			}
		}

		// reset of all bodies dirty state
		const ndArray<ndBodyKinematic*>& bodyArray = m_scene->GetActiveBodyArray();
		for (ndInt32 i = bodyArray.GetCount() - 1; i >= 0; i--)
		{
			ndBodyKinematic* const body = bodyArray[i];
			body->m_index = -1;
			body->m_skeletonMark = 0;
			body->m_skeletonMark0 = 0;
			body->m_skeletonMark1 = 0;
		}

		// find all root nodes for all independent joint arrangements
		ndInt32 inslandCount = 0;
		solverUpdate.m_leftHandSide.SetCount(dMax(bodyArray.GetCount() + 256, 1024));
		ndIslandMember* const islands = (ndIslandMember*)&solverUpdate.m_leftHandSide[0];
		for (ndInt32 i = 0; i < bodyArray.GetCount(); i++)
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
				dAssert(index != -1);
				ndIslandMember& entry = islands[index];
				if (body->GetInvMass() < entry.m_body->GetInvMass())
				{
					entry.m_body = body;
				}
			}
		}

		// build the root node
		for (ndInt32 i = 0; i < inslandCount; i++)
		{
			ndSkeletonQueue queuePool;
			ndInt32 stack = 1;
			ndBodyKinematic* stackPool[256];
			stackPool[0] = islands[i].m_body;
			ndSkeletonContainer* skeleton = nullptr;

			// find if this root node is connected to static bodies 
			// if so, them make that static body the root node and add all the children
			while (stack)
			{
				stack--;
				ndBodyKinematic* const rootBody = stackPool[stack];
				if (!rootBody->m_skeletonMark1)
				{
					rootBody->m_skeletonMark1 = 1;
					for (ndJointList::ndNode* jointNode = rootBody->m_jointList.GetFirst(); jointNode; jointNode = jointNode->GetNext())
					{
						ndJointBilateralConstraint* const constraint = jointNode->GetInfo();
						dAssert(constraint && constraint->GetAsBilateral());
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
										skeleton = m_skeletonList.CreateContatiner(childBody);
									}

									//dTrace(("%s %d %d\n", constraint->GetClassName(), constraint->GetBody0()->GetId(), constraint->GetBody1()->GetId()));
									constraint->m_mark0 = 1;
									dAssert(childBody == skeleton->GetRoot()->m_body);
									ndSkeletonContainer::ndNode* const node = skeleton->AddChild((ndJointBilateralConstraint*)constraint, skeleton->GetRoot());
									node->m_body->m_skeletonMark = 1;
									dAssert(node->m_body != childBody);
									queuePool.Push(node);
								}
								else if (!childBody->m_skeletonMark1)
								{
									stackPool[stack] = childBody;
									stack++;
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
				for (ndJointList::ndNode* jointNode = rootBody->m_jointList.GetFirst(); jointNode; jointNode = jointNode->GetNext())
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
					skeleton = m_skeletonList.CreateContatiner(rootBody);

					for (ndJointList::ndNode* jointNode = rootBody->m_jointList.GetFirst(); jointNode; jointNode = jointNode->GetNext())
					{
						ndJointBilateralConstraint* const constraint = jointNode->GetInfo();
						//dTrace(("%s %d %d\n", constraint->GetClassName(), constraint->GetBody0()->GetId(), constraint->GetBody1()->GetId()));
						const bool test = SkeletonJointTest(constraint);
						if (test && (constraint->GetSolverModel() != m_jointkinematicCloseLoop))
						{
							constraint->m_mark0 = 1;
							dAssert(skeleton->GetRoot()->m_body != ((constraint->GetBody0() != rootBody) ? constraint->GetBody0() : constraint->GetBody1()));
							ndSkeletonContainer::ndNode* const node = skeleton->AddChild((ndJointBilateralConstraint*)constraint, skeleton->GetRoot());
							node->m_body->m_skeletonMark = 1;
							dAssert(node->m_body == ((constraint->GetBody0() != rootBody) ? constraint->GetBody0() : constraint->GetBody1()));
							queuePool.Push(node);
						}
					}
				}
			}

			if (skeleton)
			{
				// add the rest the children to this skeleton
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

					for (ndInt32 j = 0; j < count; j++)
					{
						ndSkeletonContainer::ndNode* const parentNode = queuePool[index];
						ndBodyKinematic* const parentBody = parentNode->m_body;
						dAssert(parentBody->GetId() > ndFloat32(0.0f));
						if (!parentBody->m_skeletonMark0)
						{
							parentBody->m_skeletonMark0 = 1;
							for (ndJointList::ndNode* jointNode = parentBody->m_jointList.GetFirst(); jointNode; jointNode = jointNode->GetNext())
							{
								ndJointBilateralConstraint* const constraint = jointNode->GetInfo();
								if (!constraint->m_mark0)
								{
									//dTrace(("%s %d %d\n", constraint->GetClassName(), constraint->GetBody0()->GetId(), constraint->GetBody1()->GetId()));
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

		for (ndInt32 i = bodyArray.GetCount() - 1; i >= 0; i--)
		{
			ndBodyKinematic* const body = bodyArray[i];
			body->PrepareStep(i);
			dAssert (bodyArray[i] == body);
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
		skeleton->ClearSelfCollision();
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

void ndWorld::BodiesInAabb(ndBodiesInAabbNotify& callback) const
{
	m_scene->BodiesInAabb(callback);
}
