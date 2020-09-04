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

	void ThreadFunction()
	{
		m_world->ThreadFunction();
		ThreadFunction();
	}

	ndWorld* m_world;
};

ndWorld::ndWorld()
	:dClassAlloc()
	,m_broadPhase(nullptr)
	,m_timestep(dFloat32 (0.0f))
	,m_subSteps(1)
{
	// start the engine thread;
	m_broadPhase = new ndSceneMixed();
	//m_broadPhase = new ntWorldMixedBroadPhase(this);
}

ndWorld::~ndWorld()
{
	Sync();
	delete m_broadPhase;
}

void ndWorld::Update(dFloat32 timestep)
{
	// wait for previous frame to complete
	Sync();
	Tick();
	m_timestep = timestep;
	Signal();
}

void ndWorld::Signal()
{
	m_broadPhase->Signal();
}

void ndWorld::Tick()
{
	m_broadPhase->Tick();
}

void ndWorld::Sync()
{
	m_broadPhase->dSyncMutex::Sync();
}


void ndWorld::SetThreadCount(dInt32 count)
{
	dAssert(0);
//	dThreadPool& pool = *this;
//	return pool.SetCount(count);
}

void ndWorld::ThreadFunction()
{
	InternalUpdate(m_timestep);
}

//void ndWorld::DispatchJobs(dThreadPoolJob** const jobs)
//{
//	dAssert(0);
////	ExecuteJobs(jobs);
//}

dInt32 ndWorld::GetSubSteps() const
{
	return m_subSteps;
}

void ndWorld::SetSubSteps(dInt32 subSteps)
{
	m_subSteps = dClamp(subSteps, 1, 16);
}

//void ndWorld::ThreadFunction()
//{
//	InternalUpdate(m_timestep);
//	Release();
//}

void ndWorld::InternalUpdate(dFloat32 fullTimestep)
{
	D_TRACKTIME();
	dFloat32 timestep = fullTimestep / m_subSteps;
	for (dInt32 i = 0; i < m_subSteps; i++)
	{
		SubstepUpdate(timestep);
	}

	TransformUpdate(fullTimestep);
	UpdateListenersPostTransform(fullTimestep);
}

void ndWorld::TransformUpdate(dFloat32 timestep)
{
}

void ndWorld::SubstepUpdate(dFloat32 timestep)
{
	D_TRACKTIME();
	UpdateSkeletons(timestep);
	ApplyExternalForces(timestep);
	UpdatePrelisteners(timestep);
	//UpdateSleepState(timestep);
	UpdateBroadPhase(timestep);
	UpdateDynamics(timestep);
	UpdatePostlisteners(timestep);
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
	class ntApplyExternalForces: public ndScene::ndBaseJob
	{
		public:
		virtual void Execute()
		{
			D_TRACKTIME();
			dAssert(0);
			//const dInt32 threadIndex = GetThredID();
			//const dInt32 count = m_world->m_tmpBodyArray.GetCount();
			//
			//ntBodyKinematic** const bodies = &m_world->m_tmpBodyArray[0];
			//for (dInt32 i = m_it->fetch_add(1); i < count; i = m_it->fetch_add(1))
			//{
			//	ndBodyDynamic* const body = bodies[i]->GetAsBodyDynamic();
			//	if (body)
			//	{
			//		body->ApplyExternalForces(threadIndex, m_timestep);
			//	}
			//
			//}
		}
	};
	dAssert(0);
	//SubmitJobs<ntApplyExternalForces>(timestep);
}

void ndWorld::UpdateSleepState(dFloat32 timestep)
{
//	D_TRACKTIME();
}

void ndWorld::UpdateBroadPhase(dFloat32 timestep)
{
	m_broadPhase->Update(timestep);
}

