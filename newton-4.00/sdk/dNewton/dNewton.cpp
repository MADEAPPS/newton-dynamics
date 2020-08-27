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
#include "dBody.h"
#include "dNewton.h"
#include "dShapeNull.h"
#include "dDynamicBody.h"
#include "dBroadPhaseMixed.h"

dNewton::dNewton()
	:dClassAlloc()
	,dSyncMutex()
	,dThread()
	,dThreadPool()
	,m_bodyList()
	,m_dynamicBodyArray()
	,m_broadPhase(nullptr)
	,m_timestep(dFloat32 (0.0f))
	,m_subSteps(1)
{
	// start the engine thread;
	SetName("newton main thread");
	Start();

	m_broadPhase = new dBroadPhaseMixed(this);
}

dNewton::~dNewton()
{
	Sync();
	Finish();

	while (m_bodyList.GetFirst())
	{
		dBody* const body = m_bodyList.GetFirst()->GetInfo();
		RemoveBody(body);
		delete body;
	}

	delete m_broadPhase;
}

void dNewton::AddBody(dBody* const body)
{
	dAssert((body->m_newton == nullptr) && (body->m_newtonNode == nullptr));

	dList<dBody*>::dListNode* const node = m_bodyList.Append(body);
	body->SetNewtonNode(this, node);
}

void dNewton::RemoveBody(dBody* const body)
{
	dAssert(body->m_newtonNode && (body->m_newton == this));

	if (body->GetBroadPhaseNode())
	{
		m_broadPhase->RemoveBody(body);
	}

	m_bodyList.Remove(body->m_newtonNode);
	body->SetNewtonNode(nullptr, nullptr);
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
	D_TRACKTIME();
	GetDynamicBodyArray();
	dFloat32 timestep = fullTimestep / m_subSteps;
	for (dInt32 i = 0; i < m_subSteps; i++)
	{
		SubstepUpdate(timestep);
	}

	TransformUpdate(fullTimestep);
	UpdateListenersPostTransform(fullTimestep);
}

void dNewton::GetDynamicBodyArray()
{
	D_TRACKTIME();
	m_dynamicBodyArray.SetCount(m_bodyList.GetCount());
	dDynamicBody** const bodyPtr = &m_dynamicBodyArray[0];
	int index = 0;
	for (dList<dBody*>::dListNode* node = m_bodyList.GetFirst(); node; node = node->GetNext())
	{
		dDynamicBody* const dynBody = node->GetInfo()->GetAsDynamicBody();
		if (dynBody)
		{
			bodyPtr[index] = dynBody;
			index++;

			const dShape* const shape = ((dShape*)dynBody->GetCollisionShape().GetShape())->GetAsShapeNull();
			if (shape)
			{
				dAssert(0);
				if (dynBody->GetBroadPhaseNode())
				{
					m_broadPhase->RemoveBody(dynBody);
				}
			} 
			else if (!dynBody->GetBroadPhaseNode())
			{
				m_broadPhase->AddBody(dynBody);
			}
		}
	}
}

void dNewton::TransformUpdate(dFloat32 timestep)
{
}

void dNewton::SubstepUpdate(dFloat32 timestep)
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

void dNewton::UpdatePrelisteners(dFloat32 timestep)
{
}

void dNewton::UpdatePostlisteners(dFloat32 timestep)
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
	D_TRACKTIME();
	class dApplyExternalForces: public dThreadPoolJob
	{
		public:
		virtual void Execute()
		{
			D_TRACKTIME();
			const dInt32 threadIndex = GetThredID();
			const dInt32 count = m_me->m_dynamicBodyArray.GetCount();
			dDynamicBody** const bodies = &m_me->m_dynamicBodyArray[0];
			for (dInt32 i = m_it->fetch_add(1); i < count; i = m_it->fetch_add(1))
			{
				bodies[i]->ApplyExternalForces(threadIndex, m_timestep);
			}
		}

		std::atomic<int>* m_it;
		dNewton* m_me;
		dFloat32 m_timestep;
	};
	SubmitJobs<dApplyExternalForces>(timestep);
}

void dNewton::UpdateSleepState(dFloat32 timestep)
{
//	D_TRACKTIME();
}

void dNewton::UpdateBroadPhase(dFloat32 timestep)
{
	m_broadPhase->Update(timestep);
}
