/* Copyright (c) <2003-2022> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/

#include "ndSandboxStdafx.h"

#include "ndDemoCamera.h"
#include "ndPhysicsWorld.h"
#include "ndContactCallback.h"
#include "ndDemoEntityNotify.h"
#include "ndDemoEntityManager.h"
#include "ndArchimedesBuoyancyVolume.h"

#define MAX_PHYSICS_STEPS			1
#define MAX_PHYSICS_FPS				60.0f

ndDemoContactCallback::ndDemoContactCallback()
{
}

ndDemoContactCallback::~ndDemoContactCallback()
{
}

ndPhysicsWorld::ndDefferentDeleteEntities::ndDefferentDeleteEntities(ndDemoEntityManager* const manager)
	:ndArray<ndDemoEntity*>()
	,m_manager(manager)
	,m_renderThreadId(std::this_thread::get_id())
{
}

void ndPhysicsWorld::ndDefferentDeleteEntities::Update()
{
	for (ndInt32 i = 0; i < GetCount(); ++i)
	{
		RemoveEntity((*this)[i]);
	}
	SetCount(0);
}

//void ndPhysicsWorld::ndDefferentDeleteEntities::RemoveEntity(ndDemoEntity* const entity)
void ndPhysicsWorld::ndDefferentDeleteEntities::RemoveEntity(ndDemoEntity* const)
{
	ndAssert(0);
	//ndAssert(entity->m_rootNode);
	//if (m_renderThreadId == std::this_thread::get_id())
	//{
	//	m_manager->RemoveEntity(entity);
	//	delete entity;
	//}
	//else
	//{
	//	ndScopeSpinLock lock(entity->m_lock);
	//	if (!entity->m_isDead)
	//	{
	//		entity->m_isDead = true;
	//		PushBack(entity);
	//	}
	//}
}

ndPhysicsWorld::ndPhysicsWorld(ndDemoEntityManager* const manager)
	:ndWorld()
	,m_manager(manager)
	,m_timeAccumulator(0.0f)
	,m_interplationParameter(0.0f)
	,m_deadEntities(manager)
	,m_acceleratedUpdate(false)
{
	ClearCache();
	SetContactNotify(new ndDemoContactCallback);
}

ndPhysicsWorld::~ndPhysicsWorld()
{
	CleanUp();
}

void ndPhysicsWorld::CleanUp()
{
	ndWorld::CleanUp();
}

//void ndPhysicsWorld::RemoveEntity(ndDemoEntity* const entity)
void ndPhysicsWorld::RemoveEntity(ndDemoEntity* const)
{
	ndAssert(0);
	//ndAssert(entity->m_rootNode);
	//m_deadEntities.RemoveEntity(entity);
}

ndDemoEntityManager* ndPhysicsWorld::GetManager() const
{
	return m_manager;
}

void ndPhysicsWorld::PreUpdate(ndFloat32 timestep)
{
	ndWorld::PreUpdate(timestep);
}

void ndPhysicsWorld::RemoveDeadBodies()
{
	const ndBodyListView& bodyList = GetBodyList();
	ndTree<ndInt32, ndBodyKinematic*> deadBodies;
	for (ndBodyListView::ndNode* node = bodyList.GetFirst(); node; )
	{
		ndBodyKinematic* const body = node->GetInfo()->GetAsBodyKinematic();
		node = node->GetNext();

		const ndMatrix matrix = body->GetMatrix();
		if (matrix.m_posit.m_y < -100.0f)
		{
			deadBodies.Insert(0, body);
		}
	}

	while (deadBodies.GetCount())
	{
		ndTree<ndInt32, ndBodyKinematic*>::ndNode* const node = deadBodies.GetRoot();
		if (node->GetInfo())
		{
			ndBodyKinematic* const body = node->GetKey();
			RemoveBody(body);
			deadBodies.Remove(node);
		}
		else
		{
			ndTree<ndInt32, ndBodyKinematic*> filter;
			ndFixSizeArray<ndBodyKinematic*, 1024> array;
			array.PushBack(node->GetKey());
			filter.Insert(0, node->GetKey());
			for (ndInt32 i = 0; i < array.GetCount(); ++i)
			{
				ndBodyKinematic* const body = array[i];
				if (body->GetInvMass() > ndFloat32(0.0f))
				{
					const ndBodyKinematic::ndJointList& joints = body->GetJointList();
					for (ndBodyKinematic::ndJointList::ndNode* jointNode = joints.GetFirst(); jointNode; jointNode = jointNode->GetNext())
					{
						ndJointBilateralConstraint* const joint = jointNode->GetInfo();
						ndBodyKinematic* const body1 = (joint->GetBody0() == body) ? joint->GetBody1() : joint->GetBody0();
						ndTree<ndInt32, ndBodyKinematic*>::ndNode* const deadNode = filter.Insert(0, body1);
						if (deadNode)
						{
							array.PushBack(body1);
						}
					}
				}
			}

			node->GetInfo() = 1;
			for (ndInt32 i = 1; i < array.GetCount(); ++i)
			{
				ndBodyKinematic* const body = array[i];
				ndTree<ndInt32, ndBodyKinematic*>::ndNode* deadNode = deadBodies.Insert(1, body);
				if (!deadNode)
				{
					deadNode = deadBodies.Find(body);
					deadNode->GetInfo() = 1;
				}
			}
		}
	}
}

void ndPhysicsWorld::PostUpdate(ndFloat32 timestep)
{
	ndWorld::PostUpdate(timestep);
	//RemoveDeadBodies();

	ndScopeSpinLock Lock(m_lock);
	const ndBodyListView& bodyArray = GetBodyList();
	const ndArray<ndBodyKinematic*>& view = bodyArray.GetView();
	for (ndInt32 i = ndInt32(view.GetCount()) - 2; i >= 0; --i)
	{
		ndBodyKinematic* const body = view[i];
		ndDemoEntityNotify* const notify = (ndDemoEntityNotify*)body->GetNotifyCallback();
		ndAssert(notify);
		notify->m_entity->SetTransform(notify->m_transform.m_rotation, notify->m_transform.m_position);
	}

	ndDemoCamera* const camera = (ndDemoCamera*)*m_manager->m_renderer->GetCamera();
	ndAssert(camera);
	camera->TickUpdate(timestep);

	//if (m_manager->m_updateCamera)
	//{
	//	m_manager->m_updateCamera(m_manager, m_manager->m_updateCameraContext, timestep);
	//}
	//
	//if (m_manager->m_onPostUpdate)
	//{
	//	m_manager->m_onPostUpdate->Update(m_manager, timestep);
	//	m_manager->m_onPostUpdate->OnDebug(m_manager, m_manager->m_hidePostUpdate);
	//}
}

void ndPhysicsWorld::OnSubStepPostUpdate(ndFloat32 timestep)
{
	ndWorld::OnSubStepPostUpdate(timestep);
	m_manager->OnSubStepPostUpdate(timestep);
}

void ndPhysicsWorld::NormalUpdates()
{
	m_acceleratedUpdate = false;
}

void ndPhysicsWorld::AccelerateUpdates()
{
	m_acceleratedUpdate = true;
}

void ndPhysicsWorld::AdvanceTime(ndFloat32 timestep)
{
	D_TRACKTIME();
	const ndFloat32 descreteStep = (1.0f / MAX_PHYSICS_FPS);

	if (m_acceleratedUpdate)
	{
		Update(descreteStep);
	} 
	else
	{
		ndInt32 maxSteps = MAX_PHYSICS_STEPS;
		m_timeAccumulator += timestep;

		// if the time step is more than max timestep par frame, throw away the extra steps.
		if (m_timeAccumulator > descreteStep * (ndFloat32)maxSteps)
		{
			ndFloat32 steps = ndFloor(m_timeAccumulator / descreteStep) - (ndFloat32)maxSteps;
			ndAssert(steps >= 0.0f);
			m_timeAccumulator -= descreteStep * steps;
		}

		while (m_timeAccumulator > descreteStep)
		{
			Update(descreteStep);
			m_timeAccumulator -= descreteStep;
		}
	}

	{
		ndScopeSpinLock Lock(m_lock);
		ndFloat32 param = m_timeAccumulator / descreteStep;
		m_manager->m_renderer->InterpolateTransforms(param);
	}

	if (m_manager->m_synchronousPhysicsUpdate)
	{
		Sync();
	}

	m_deadEntities.Update();
}