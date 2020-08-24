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

#include "dNewtonStdafx.h"
#include "dNewton.h"


dNewton::dNewton()
	:dClassAlloc()
	,dSyncMutex()
	,dThread()
	,dThreadPool()
{
	// start the engine thread;
	SetName("newton main thread");
	Start();

	m_subSteps = 1;
	m_timestep = 0.0f;
}

dNewton::~dNewton()
{
	Sync();
	Finish();
}

void dNewton::Update(dFloat32 timestep)
{
	// wait for previous frame to complete
	Sync();
	Tick();
	m_timestep = timestep;
	Signal();
}

void dNewton::Sync()
{
	dSyncMutex::Sync();
}

dInt32 dNewton::GetThreadCount() const
{
	const dThreadPool& pool = *this;
	return pool.GetCount();
}

void dNewton::SetThreadCount(dInt32 count)
{
	dThreadPool& pool = *this;
	return pool.SetCount(count);
}

void dNewton::DispatchJobs(dThreadPoolJob** const jobs)
{
	ExecuteJobs(jobs);
}

dInt32 dNewton::GetSubSteps() const
{
	return m_subSteps;
}

void dNewton::SetSubSteps(dInt32 subSteps)
{
	m_subSteps = dClamp(subSteps, 1, 16);
}

void dNewton::ThreadFunction()
{
	InternalUpdate(m_timestep);
	Release();
}

void dNewton::InternalUpdate(dFloat32 fullTimestep)
{
	dFloat32 timestep = fullTimestep / m_subSteps;
	for (dInt32 i = 0; i < m_subSteps; i++)
	{
		SubstepUpdate(timestep);
	}

	TransformUpdate(fullTimestep);
	UpdateListenersPostTransform(fullTimestep);
}

void dNewton::TransformUpdate(dFloat32 timestep)
{
}

void dNewton::SubstepUpdate(dFloat32 timestep)
{
	UpdateSkeletons(timestep);
	ApplyExternalForces(timestep);
	UpdatePrelisteners(timestep);
	UpdateSleepState(timestep);
	UpdateBroadPhase(timestep);
	UpdateDynamics(timestep);
	UpdatePostlisteners(timestep);
}

void dNewton::UpdatePrelisteners(dFloat32 timestep)
{
}

void dNewton::UpdatePostlisteners(dFloat32 timestep)
{
}

void dNewton::UpdateSleepState(dFloat32 timestep)
{
}

void dNewton::UpdateBroadPhase(dFloat32 timestep)
{
}

void dNewton::UpdateDynamics(dFloat32 timestep)
{
}

void dNewton::UpdateSkeletons(dFloat32 timestep)
{
}

void dNewton::UpdateListenersPostTransform(dFloat32 timestep)
{
}

void dNewton::ApplyExternalForces(dFloat32 timestep)
{
	class xxxxx : public dThreadPoolJob
	{
		public:
		virtual void Execute()
		{

		}

		dNewton* m_me;
	};

	xxxxx xxx[D_MAX_THREADS_COUNT];
	dThreadPoolJob* xxxx[D_MAX_THREADS_COUNT];
	for (int i = 0; i < GetThreadCount(); i++)
	{
		xxx[i].m_me = this;
		xxxx[i] = &xxx[i];
	}
	DispatchJobs(xxxx);
}
