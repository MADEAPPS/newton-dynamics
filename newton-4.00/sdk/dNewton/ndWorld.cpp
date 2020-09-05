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
#include "ndBodyDynamic.h"
#include "ndSceneMixed.h"

class ndWorld::ndWorldMixedScene: public ndSceneMixed
{
	public:
	ndWorldMixedScene(ndWorld* const world)
		:ndSceneMixed()
		,m_world(world)
	{
	}

	void SubStepUpdate(dFloat32 timestep)
	{
		D_TRACKTIME();

		// do the a pre-physics step 
		m_lru = m_lru + 1;
		BuildBodyArray();
		m_world->UpdateSkeletons(timestep);
		m_world->ApplyExternalForces(timestep);
		m_world->UpdatePrelisteners(timestep);
		//UpdateSleepState(timestep);
		
		// update the collision system
		UpdateAabb(m_timestep);
		FindCollidingPairs(m_timestep);
		AttachNewContact();
		CalculateContacts(m_timestep);
		
		// calculate internal forces, integrate bodies and update matrices.
		m_world->UpdateDynamics(timestep);
		m_world->UpdatePostlisteners(timestep);
	}

	void ThreadFunction()
	{
		dUnsigned64 timeAcc = dGetTimeInMicrosenconds();
		const bool collisionUpdate = m_world->m_collisionUpdate;
		m_world->m_collisionUpdate = true;
		if (collisionUpdate)
		{
			ndSceneMixed::ThreadFunction();
		}
		else
		{
			D_TRACKTIME();
			BalanceBroadPhase();

			dInt32 const steps = m_world->m_subSteps;
			dFloat32 timestep = m_world->m_timestep / steps;
			for (dInt32 i = 0; i < steps; i++)
			{
				SubStepUpdate(timestep);
			}

			TransformUpdate(m_world->m_timestep);
			m_world->UpdateListenersPostTransform(m_world->m_timestep);
		}
		m_world->m_lastExecutionTime = (dGetTimeInMicrosenconds() - timeAcc) * dFloat32(1.0e-6f);
	}

	ndWorld* m_world;
};

ndWorld::ndWorld()
	:dClassAlloc()
	,m_scene(nullptr)
	,m_timestep(dFloat32 (0.0f))
	,m_lastExecutionTime(dFloat32(0.0f))
	,m_subSteps(1)
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

void ndWorld::UpdatePrelisteners(dFloat32 timestep)
{
}

void ndWorld::UpdatePostlisteners(dFloat32 timestep)
{
}

void ndWorld::UpdateDynamics(dFloat32 timestep)
{
}

void ndWorld::UpdateSkeletons(dFloat32 timestep)
{
}

void ndWorld::UpdateListenersPostTransform(dFloat32 timestep)
{
}

void ndWorld::ApplyExternalForces(dFloat32 timestep)
{
	D_TRACKTIME();
	class ndApplyExternalForces: public ndScene::ndBaseJob
	{
		public:
		virtual void Execute()
		{
			D_TRACKTIME();

			ndWorld* const world = ((ndWorldMixedScene*)m_owner)->m_world;
			ndScene* const scene = world->GetScene();
			const dInt32 threadIndex = GetThredID();
			
			const dArray<ndBodyKinematic*>& bodyArray = scene->GetWorkingBodyArray();
			const dInt32 count = bodyArray.GetCount();
			
			ndBodyKinematic** const bodies = (ndBodyKinematic**)&bodyArray[0];
			for (dInt32 i = m_it->fetch_add(1); i < count; i = m_it->fetch_add(1))
			{
				ndBodyDynamic* const body = bodies[i]->GetAsBodyDynamic();
				if (body)
				{
					body->ApplyExternalForces(threadIndex, m_timestep);
				}
			}
		}
	};
	m_scene->SubmitJobs<ndApplyExternalForces>(timestep);
}



