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

#ifndef __D_WORLD_H__
#define __D_WORLD_H__

#include "ndNewtonStdafx.h"

class ndWorld;
class ndBodyDynamic;

#define D_NEWTON_ENGINE_VERSION 400

D_MSV_NEWTON_ALIGN_32
class ndWorld
	:public dClassAlloc
	,public dSyncMutex
	,public dThread
	,public dThreadPool
{
	public:
	class ntNewtonBaseJob: public dThreadPoolJob
	{
		public:
		dAtomic<int>* m_it;
		ndWorld* m_world;
		dFloat32 m_timestep;
	};

	D_NEWTON_API ndWorld();
	D_NEWTON_API virtual ~ndWorld();

	dInt32 GetEngineVersion() 
	{
		return D_NEWTON_ENGINE_VERSION;
	}

	D_NEWTON_API void Update(dFloat32 timestep);
	D_NEWTON_API void Sync();

	D_NEWTON_API dInt32 GetThreadCount() const;
	D_NEWTON_API void SetThreadCount(dInt32 count);

	D_NEWTON_API dInt32 GetSubSteps() const;
	D_NEWTON_API void SetSubSteps(dInt32 subSteps);

	D_NEWTON_API void DispatchJobs(dThreadPoolJob** const jobs);

	bool AddBody(ntBody* const body);
	bool RemoveBody(ntBody* const body);

	ntBroadPhase* GetBroadphase() const;

	ntContactNotify* GetContactNotify() const;
	void SetContactNotify(ntContactNotify* const notify);

	protected:
	D_NEWTON_API virtual void SubstepUpdate(dFloat32 timestep);
	D_NEWTON_API virtual void UpdateSkeletons(dFloat32 timestep);
	D_NEWTON_API virtual void UpdateSleepState(dFloat32 timestep);
	D_NEWTON_API virtual void ApplyExternalForces(dFloat32 timestep);
	D_NEWTON_API virtual void UpdatePrelisteners(dFloat32 timestep);
	D_NEWTON_API virtual void UpdatePostlisteners(dFloat32 timestep);
	D_NEWTON_API virtual void UpdateBroadPhase(dFloat32 timestep);
	D_NEWTON_API virtual void UpdateDynamics(dFloat32 timestep);
	
	D_NEWTON_API virtual void InternalUpdate(dFloat32 timestep);
	D_NEWTON_API virtual void TransformUpdate(dFloat32 timestep);
	D_NEWTON_API virtual void UpdateListenersPostTransform(dFloat32 timestep);

	protected:

	private:
	virtual void ThreadFunction();
	void BuildBodyArray();

	template <class T>
	void SubmitJobs(dFloat32 timestep);
	dArray<ntBodyKinematic*> m_tmpBodyArray;
	ntBroadPhase* m_broadPhase;

	dFloat32 m_timestep;
	dInt32 m_subSteps;

	friend class ntBroadPhase;
} D_GCC_NEWTON_ALIGN_32 ;

template <class T>
void ndWorld::SubmitJobs(dFloat32 timestep)
{
	dAtomic<dInt32> it(0);
	T extJob[D_MAX_THREADS_COUNT];
	dThreadPoolJob* extJobPtr[D_MAX_THREADS_COUNT];

	const dInt32 threadCount = GetThreadCount();
	for (int i = 0; i < threadCount; i++)
	{
		extJob[i].m_it = &it;
		extJob[i].m_world = this;
		extJob[i].m_timestep = timestep;
		extJobPtr[i] = &extJob[i];
	}
	DispatchJobs(extJobPtr);
}

inline ntBroadPhase* ndWorld::GetBroadphase() const
{
	return m_broadPhase;
}

inline ntContactNotify* ndWorld::GetContactNotify() const
{
	return m_broadPhase->GetContactNotify();
}

inline void ndWorld::SetContactNotify(ntContactNotify* const notify)
{
	m_broadPhase->SetContactNotify(notify);
}

inline bool ndWorld::AddBody(ntBody* const body)
{
	ntBodyKinematic* const kinematicBody = body->GetAsBodyKinematic();
	if (kinematicBody)
	{
		return m_broadPhase->AddBody(kinematicBody);
	}
	return false;
}

inline bool ndWorld::RemoveBody(ntBody* const body)
{
	ntBodyKinematic* const kinematicBody = body->GetAsBodyKinematic();
	if (kinematicBody)
	{
		return m_broadPhase->RemoveBody(kinematicBody);
	}
	return false;
}


#endif
