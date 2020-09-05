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
class ndWorld: public dClassAlloc
{
	class ndWorldMixedScene;

	public:
	D_NEWTON_API ndWorld();
	D_NEWTON_API virtual ~ndWorld();

	dInt32 GetEngineVersion() 
	{
		return D_NEWTON_ENGINE_VERSION;
	}

	void Sync();
	void Update(dFloat32 timestep);

	dInt32 GetThreadCount() const;
	void SetThreadCount(dInt32 count);

	dInt32 GetSubSteps() const;
	void SetSubSteps(dInt32 subSteps);

	bool AddBody(ndBody* const body);
	bool RemoveBody(ndBody* const body);

	ndScene* GetScene() const;

	dFloat32 GetUpdateTime() const;

	ndContactNotify* GetContactNotify() const;
	void SetContactNotify(ndContactNotify* const notify);
	
	protected:
	D_NEWTON_API virtual void UpdateSkeletons(dFloat32 timestep);
	//D_NEWTON_API virtual void UpdateSleepState(dFloat32 timestep);
	D_NEWTON_API virtual void ApplyExternalForces(dFloat32 timestep);
	D_NEWTON_API virtual void UpdatePrelisteners(dFloat32 timestep);
	D_NEWTON_API virtual void UpdatePostlisteners(dFloat32 timestep);
	D_NEWTON_API virtual void UpdateDynamics(dFloat32 timestep);
	
	D_NEWTON_API virtual void UpdateListenersPostTransform(dFloat32 timestep);

	private:
	ndScene* m_scene;
	dFloat32 m_timestep;
	dFloat32 m_lastExecutionTime;
	dInt32 m_subSteps;
	bool m_collisionUpdate;

	friend class ndScene;
} D_GCC_NEWTON_ALIGN_32 ;


inline void ndWorld::Sync()
{
	m_scene->Sync();
}

inline dInt32 ndWorld::GetThreadCount() const
{
	return m_scene->GetThreadCount();
}

inline void ndWorld::SetThreadCount(dInt32 count)
{
	m_scene->SetCount(count);
}

inline dInt32 ndWorld::GetSubSteps() const
{
	return m_subSteps;
}

inline void ndWorld::SetSubSteps(dInt32 subSteps)
{
	m_subSteps = dClamp(subSteps, 1, 16);
}

inline ndScene* ndWorld::GetScene() const
{
	return m_scene;
}

inline ndContactNotify* ndWorld::GetContactNotify() const
{
	return m_scene->GetContactNotify();
}

inline void ndWorld::SetContactNotify(ndContactNotify* const notify)
{
	m_scene->SetContactNotify(notify);
}

inline bool ndWorld::AddBody(ndBody* const body)
{
	ndBodyKinematic* const kinematicBody = body->GetAsBodyKinematic();
	if (kinematicBody)
	{
		return m_scene->AddBody(kinematicBody);
	}
	return false;
}

inline bool ndWorld::RemoveBody(ndBody* const body)
{
	ndBodyKinematic* const kinematicBody = body->GetAsBodyKinematic();
	if (kinematicBody)
	{
		return m_scene->RemoveBody(kinematicBody);
	}
	return false;
}

inline dFloat32 ndWorld::GetUpdateTime() const
{
	return m_lastExecutionTime;
}

inline void ndWorld::Update(dFloat32 timestep)
{
	// wait until previous update complete.
	Sync();

	// save time state for use by the update callback
	m_timestep = timestep;
	m_collisionUpdate = false;

	// update the next frame asynchronous 
	m_scene->TickOne();
}

#endif
