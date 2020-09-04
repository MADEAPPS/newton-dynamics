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

	D_NEWTON_API void Update(dFloat32 timestep);
	D_NEWTON_API void Sync();

	D_NEWTON_API dInt32 GetThreadCount() const;
	D_NEWTON_API void SetThreadCount(dInt32 count);

	D_NEWTON_API dInt32 GetSubSteps() const;
	D_NEWTON_API void SetSubSteps(dInt32 subSteps);

	bool AddBody(ndBody* const body);
	bool RemoveBody(ndBody* const body);

	ndScene* GetBroadphase() const;

	ndContactNotify* GetContactNotify() const;
	void SetContactNotify(ndContactNotify* const notify);

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
	void Tick();
	void Signal();
	void ThreadFunction();
	ndScene* m_broadPhase;

	dFloat32 m_timestep;
	dInt32 m_subSteps;

	friend class ndScene;
} D_GCC_NEWTON_ALIGN_32 ;

inline ndScene* ndWorld::GetBroadphase() const
{
	return m_broadPhase;
}

inline ndContactNotify* ndWorld::GetContactNotify() const
{
	return m_broadPhase->GetContactNotify();
}

inline void ndWorld::SetContactNotify(ndContactNotify* const notify)
{
	m_broadPhase->SetContactNotify(notify);
}

inline bool ndWorld::AddBody(ndBody* const body)
{
	ndBodyKinematic* const kinematicBody = body->GetAsBodyKinematic();
	if (kinematicBody)
	{
		return m_broadPhase->AddBody(kinematicBody);
	}
	return false;
}

inline bool ndWorld::RemoveBody(ndBody* const body)
{
	ndBodyKinematic* const kinematicBody = body->GetAsBodyKinematic();
	if (kinematicBody)
	{
		return m_broadPhase->RemoveBody(kinematicBody);
	}
	return false;
}

inline dInt32 ndWorld::GetThreadCount() const
{
	return m_broadPhase->GetThreadCount();
}


#endif
