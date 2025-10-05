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

#include "ndPhysicsWorld.h"
#include "ndDemoCameraNode.h"
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

ndPhysicsWorld::ndDeffereDeadBodies::ndDeffereDeadBodies()
	:ndArray<ndBody*>()
	,m_owner(nullptr)
{
}

void ndPhysicsWorld::ndDeffereDeadBodies::RemovePendingBodies()
{
	if (!GetCount())
	{
		return;
	}

	ndTree<ndInt32, ndBodyKinematic*> filter;

	for (ndInt32 i = 0; i < GetCount(); ++i)
	{
		ndBodyKinematic* const body = (*this)[i]->GetAsBodyKinematic();
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
					ndAssert(0);
					PushBack(body1);
				}
			}
		}
	}
		
	for (ndInt32 i = ndInt32 (GetCount()) - 1; i >= 0 ; --i)
	{
		ndBodyKinematic* const body = (*this)[i]->GetAsBodyKinematic();
		ndDemoEntityNotify* const notification = (ndDemoEntityNotify*)body->GetNotifyCallback();
		m_owner->m_defferedDeadEntities.Append(notification->GetUserData());
		m_owner->RemoveBody(body);
	}

	SetCount(0);
}

void ndPhysicsWorld::ndDeffereDeadBodies::RemoveBody(ndBody* const body)
{
	ndScopeSpinLock Lock(m_owner->m_lock);
	PushBack(body);
}

ndPhysicsWorld::ndPhysicsWorld(ndDemoEntityManager* const manager)
	:ndWorld()
	,m_manager(manager)
	,m_timeAccumulator(0.0f)
	,m_interplationParameter(0.0f)
	,m_deadBodies()
	,m_defferedDeadEntities()
	,m_acceleratedUpdate(false)
{
	ClearCache();
	m_deadBodies.m_owner = this;
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

ndDemoEntityManager* ndPhysicsWorld::GetManager() const
{
	return m_manager;
}

void ndPhysicsWorld::PreUpdate(ndFloat32 timestep)
{
	ndWorld::PreUpdate(timestep);
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

void ndPhysicsWorld::PostUpdate(ndFloat32 timestep)
{
	ndWorld::PostUpdate(timestep);

	ndScopeSpinLock Lock(m_lock);

	m_manager->SetNextActiveCamera();

	const ndBodyListView& bodyArray = GetBodyList();
	const ndArray<ndBodyKinematic*>& view = bodyArray.GetView();
	for (ndInt32 i = ndInt32(view.GetCount()) - 2; i >= 0; --i)
	{
		ndBodyKinematic* const body = view[i];
		if (!body->GetSleepState())
		{
			ndDemoEntityNotify* const notify = (ndDemoEntityNotify*)body->GetNotifyCallback();
			if (notify)
			{
				notify->m_entity->SetTransform(notify->m_transform.m_rotation, notify->m_transform.m_position);
			}
		}
	}

	ndDemoCameraNode* const camera = (ndDemoCameraNode*)*m_manager->m_renderer->GetCamera();
	ndAssert(camera);
	camera->TickUpdate(timestep);

	RemoveDeadBodies();
}

void ndPhysicsWorld::RemoveDeadBodies()
{
	m_deadBodies.RemovePendingBodies();
}

void ndPhysicsWorld::DefferedRemoveBody(ndBody* const body)
{
	m_deadBodies.RemoveBody(body);
}

void ndPhysicsWorld::RemoveDeadEntities()
{
	ndList<ndSharedPtr<ndRenderSceneNode>>::ndNode* nextNode;
	for (ndList<ndSharedPtr<ndRenderSceneNode>>::ndNode* node = m_defferedDeadEntities.GetFirst(); node; node = nextNode)
	{
		nextNode = node->GetNext();
		m_manager->RemoveEntity(node->GetInfo());
		m_defferedDeadEntities.Remove(node);
	}
}

void ndPhysicsWorld::AdvanceTime(ndFloat32 timestep)
{
	D_TRACKTIME();
	const ndFloat32 descreteStep = (1.0f / MAX_PHYSICS_FPS);

	if (m_acceleratedUpdate)
	{
		Update(descreteStep);
		RemoveDeadEntities();
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
			RemoveDeadEntities();
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
}