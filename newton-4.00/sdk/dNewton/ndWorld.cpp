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

#include "ndNewtonStdafx.h"
#include "ndWorld.h"
#include "ndWorldScene.h"
#include "ndBodyDynamic.h"

ndWorld::ndWorld()
	:dClassAlloc()
	,ndDynamicsUpdate()
	,m_scene(nullptr)
	,m_timestep(dFloat32 (0.0f))
	,m_lastExecutionTime(dFloat32(0.0f))
	,m_subSteps(1)
	,m_solverIterations(4)
	,m_collisionUpdate(true)
{
	// start the engine thread;
	//m_scene = new ndSceneMixed();
	m_scene = new ndWorldMixedScene(this);
}

ndWorld::~ndWorld()
{
	Sync();
	delete m_scene;
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
			
			const dArray<ndBodyKinematic*>& bodyArray = m_owner->GetWorkingBodyArray();
			const dInt32 count = bodyArray.GetCount();
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

	m_scene->BuildBodyArray();
	UpdateSkeletons();
	ApplyExternalForces();
	UpdatePrelisteners();
	//UpdateSleepState();

	// update the collision system
	m_scene->UpdateAabb();
	m_scene->FindCollidingPairs();
	m_scene->AttachNewContact();
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
	}
	m_lastExecutionTime = (dGetTimeInMicrosenconds() - timeAcc) * dFloat32(1.0e-6f);
}

