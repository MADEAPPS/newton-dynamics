/* Copyright (c) <2003-2019> <Julio Jerez, Newton Game Dynamics>
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
#include "ndWorldScene.h"
#include "ndBodyDynamic.h"
#include "ndJointBilateralConstraint.h"

ndWorld::ndWorld()
	:dClassAlloc()
	,ndDynamicsUpdate()
	,m_scene(nullptr)
	,m_sentinelBody(nullptr)
	,m_timestep(dFloat32 (0.0f))
	,m_lastExecutionTime(dFloat32(0.0f))
	,m_freezeAccel2(D_FREEZE_ACCEL2)
	,m_freezeAlpha2(D_FREEZE_ACCEL2)
	,m_freezeSpeed2(D_FREEZE_SPEED2)
	,m_freezeOmega2(D_FREEZE_SPEED2)

	,m_subSteps(1)
	,m_solverIterations(4)
	,m_collisionUpdate(true)
{
	// start the engine thread;
	//m_scene = new ndSceneMixed();
	m_scene = new ndWorldMixedScene(this);

	dInt32 steps = 1;
	dFloat32 freezeAccel2 = m_freezeAccel2;
	dFloat32 freezeAlpha2 = m_freezeAlpha2;
	dFloat32 freezeSpeed2 = m_freezeSpeed2;
	dFloat32 freezeOmega2 = m_freezeOmega2;
	for (dInt32 i = 0; i < D_SLEEP_ENTRIES; i++) {
		m_sleepTable[i].m_maxAccel = freezeAccel2;
		m_sleepTable[i].m_maxAlpha = freezeAlpha2;
		m_sleepTable[i].m_maxVeloc = freezeSpeed2;
		m_sleepTable[i].m_maxOmega = freezeOmega2;
		m_sleepTable[i].m_steps = steps;
		steps += 7;
		freezeAccel2 *= dFloat32(1.5f);
		freezeAlpha2 *= dFloat32(1.5f);
		freezeSpeed2 *= dFloat32(1.5f);
		freezeOmega2 *= dFloat32(1.5f);
	}

	m_sleepTable[0].m_maxAccel *= dFloat32(0.009f);
	m_sleepTable[0].m_maxAlpha *= dFloat32(0.009f);

	steps += 300;
	m_sleepTable[D_SLEEP_ENTRIES - 1].m_maxAccel *= dFloat32(100.0f);
	m_sleepTable[D_SLEEP_ENTRIES - 1].m_maxAlpha *= dFloat32(100.0f);
	m_sleepTable[D_SLEEP_ENTRIES - 1].m_maxVeloc = 0.25f;
	m_sleepTable[D_SLEEP_ENTRIES - 1].m_maxOmega = 0.1f;
	m_sleepTable[D_SLEEP_ENTRIES - 1].m_steps = steps;

	m_sentinelBody = new ndBodyDynamic;
}

ndWorld::~ndWorld()
{
	Sync();

	while (m_jointList.GetFirst())
	{
		ndJointBilateralConstraint* const joint = m_jointList.GetFirst()->GetInfo();
		RemoveJoint(joint);
		delete joint;
	}

	delete m_sentinelBody;
	delete m_scene;
	ClearCache();
}

void ndWorld::ClearCache()
{
	ndContact::FlushFreeList();
	ndBodyList::FlushFreeList();
	ndJointList::FlushFreeList();
	ndContactList::FlushFreeList();
	ndContactPointList::FlushFreeList();
	ndScene::ndFitnessList::FlushFreeList();
	ndBodyKinematic::ndContactMap::FlushFreeList();
}

void ndWorld::UpdatePrelisteners()
{
}

void ndWorld::UpdatePostlisteners()
{
}

void ndWorld::UpdateSkeletons()
{
}

void ndWorld::UpdateListenersPostTransform()
{
}

void ndWorld::ApplyExternalForces()
{
	D_TRACKTIME();
	class ndApplyExternalForces: public ndScene::ndBaseJob
	{
		public:
		virtual void Execute()
		{
			D_TRACKTIME();

			//ndWorld* const world = m_owner->GetWorld();
			const dInt32 threadIndex = GetThredID();
			
			const dArray<ndBodyKinematic*>& bodyArray = m_owner->GetActiveBodyArray();
			const dInt32 count = bodyArray.GetCount() - 1;
			for (dInt32 i = m_it->fetch_add(1); i < count; i = m_it->fetch_add(1))
			{
				ndBodyDynamic* const body = bodyArray[i]->GetAsBodyDynamic();
				if (body)
				{
					body->ApplyExternalForces(threadIndex, m_timestep);
				}
			}
		}
	};
	m_scene->SubmitJobs<ndApplyExternalForces>();
}

void ndWorld::SubStepUpdate(dFloat32 timestep)
{
	D_TRACKTIME();

	// do the a pre-physics step
	m_scene->m_lru = m_scene->m_lru + 1;
	m_scene->SetTimestep(timestep);

static int xxxx;
xxxx++;

	m_scene->BuildBodyArray();

	ndBodyKinematic* sentinelBody = m_sentinelBody;
	sentinelBody->PrepareStep(m_scene->GetActiveBodyArray().GetCount());
	sentinelBody->m_resting = 1;
	sentinelBody->m_sleeping = 1;
	sentinelBody->m_equilibrium = 1;
	m_scene->GetActiveBodyArray().PushBack(sentinelBody);

	UpdateSkeletons();
	ApplyExternalForces();
	UpdatePrelisteners();
	//UpdateSleepState();

	// update the collision system
	m_scene->UpdateAabb();
	m_scene->FindCollidingPairs();
	m_scene->CalculateContacts();
	m_scene->DeleteDeadContact();

	// calculate internal forces, integrate bodies and update matrices.
	DynamicsUpdate();
	UpdatePostlisteners();
}

void ndWorld::ThreadFunction()
{
	dUnsigned64 timeAcc = dGetTimeInMicrosenconds();
	const bool collisionUpdate = m_collisionUpdate;
	m_collisionUpdate = true;
	if (collisionUpdate)
	{
		m_scene->CollisionOnlyUpdate();
	}
	else
	{
		D_TRACKTIME();
		m_scene->BalanceBroadPhase();
	
		dInt32 const steps = m_subSteps;
		dFloat32 timestep = m_timestep / steps;
		for (dInt32 i = 0; i < steps; i++)
		{
			SubStepUpdate(timestep);
		}
	
		m_scene->SetTimestep(m_timestep);
		m_scene->TransformUpdate();
		UpdateListenersPostTransform();
		OnPostUpdate(m_timestep);
	}
	m_lastExecutionTime = (dGetTimeInMicrosenconds() - timeAcc) * dFloat32(1.0e-6f);
}

bool ndWorld::AddBody(ndBody* const body)
{
	ndBodyKinematic* const kinematicBody = body->GetAsBodyKinematic();
	dAssert(kinematicBody != m_sentinelBody);
	if (kinematicBody)
	{
		return m_scene->AddBody(kinematicBody);
	}
	return false;
}

void ndWorld::RemoveBody(ndBody* const body)
{
	ndBodyKinematic* const kinematicBody = body->GetAsBodyKinematic();
	dAssert(kinematicBody != m_sentinelBody);
	if (kinematicBody)
	{
		m_scene->RemoveBody(kinematicBody);
	}
}


void ndWorld::AddJoint(ndJointBilateralConstraint* const joint)
{
	dAssert(joint->m_worldNode == nullptr);
	joint->m_worldNode = m_jointList.Append(joint);
}

void ndWorld::RemoveJoint(ndJointBilateralConstraint* const joint)
{
	dAssert(joint->m_worldNode != nullptr);
	m_jointList.Remove(joint->m_worldNode);
	joint->m_worldNode = nullptr;
}
