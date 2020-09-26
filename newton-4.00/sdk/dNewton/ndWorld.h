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
#include "ndJointList.h"
#include "ndDynamicsUpdate.h"


class ndWorld;
class ndBodyDynamic;
class ndJointBilateralConstraint;

#define D_NEWTON_ENGINE_MAJOR_VERSION 4
#define D_NEWTON_ENGINE_MINOR_VERSION 00

#define D_SLEEP_ENTRIES			8

D_MSV_NEWTON_ALIGN_32
class ndWorld: public dClassAlloc, public ndDynamicsUpdate
{
	public:
	D_NEWTON_API ndWorld();
	D_NEWTON_API virtual ~ndWorld();

	dInt32 GetEngineVersion() 
	{
		return D_NEWTON_ENGINE_MAJOR_VERSION * 100 + D_NEWTON_ENGINE_MINOR_VERSION;
	}

	void Sync();
	void Update(dFloat32 timestep);

	virtual void OnPostUpdate(dFloat32 timestep);

	dInt32 GetThreadCount() const;
	void SetThreadCount(dInt32 count);

	dInt32 GetSubSteps() const;
	void SetSubSteps(dInt32 subSteps);

	bool AddBody(ndBody* const body);
	void RemoveBody(ndBody* const body);
	const ndBodyList& GetBodyList() const;

	D_NEWTON_API void AddJoint(ndJointBilateralConstraint* const joint);
	D_NEWTON_API void RemoveJoint(ndJointBilateralConstraint* const joint);
	const ndJointList& GetJointList() const;

	ndBodyKinematic* GetSentinelBody() const;

	const dInt32 GetSolverIterations() const;
	void SetSolverIterations(dInt32 iterations);

	ndScene* GetScene() const;

	dFloat32 GetUpdateTime() const;

	ndContactNotify* GetContactNotify() const;
	void SetContactNotify(ndContactNotify* const notify);

	//void SetCollisionUpdate(bool mode);
	//const bool GetCollisionUpdate() const;

	D_NEWTON_API void ClearCache();

	private:
	void ThreadFunction();
	
	protected:
	D_NEWTON_API virtual void UpdateSkeletons();
	D_NEWTON_API virtual void ApplyExternalForces();
	D_NEWTON_API virtual void UpdatePrelisteners();
	D_NEWTON_API virtual void UpdatePostlisteners();
	D_NEWTON_API virtual void UpdateListenersPostTransform();

	private:
	void SubStepUpdate(dFloat32 timestep);

	ndScene* m_scene;
	ndBodyKinematic* m_sentinelBody;
	ndJointList m_jointList;

	dFloat32 m_timestep;
	dFloat32 m_lastExecutionTime;
	dFloat32 m_freezeAccel2;
	dFloat32 m_freezeAlpha2;
	dFloat32 m_freezeSpeed2;
	dFloat32 m_freezeOmega2;
	dgSolverProgressiveSleepEntry m_sleepTable[D_SLEEP_ENTRIES];

	dInt32 m_subSteps;
	dInt32 m_solverIterations;
	bool m_collisionUpdate;

	friend class ndScene;
	friend class ndDynamicsUpdate;
	friend class ndWorldMixedScene;
	friend class ndWorldSegregatedScene;
} D_GCC_NEWTON_ALIGN_32;

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

inline const dInt32 ndWorld::GetSolverIterations() const
{
	return m_solverIterations;
}

inline void ndWorld::SetSolverIterations(dInt32 iterations)
{
	m_solverIterations = dUnsigned32(dMax(4, iterations));
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
	dAssert(kinematicBody != m_sentinelBody);
	if (kinematicBody)
	{
		return m_scene->AddBody(kinematicBody);
	}
	return false;
}

inline void ndWorld::RemoveBody(ndBody* const body)
{
	ndBodyKinematic* const kinematicBody = body->GetAsBodyKinematic();
	dAssert(kinematicBody != m_sentinelBody);
	if (kinematicBody)
	{
		m_scene->RemoveBody(kinematicBody);
	}
}

inline ndBodyKinematic* ndWorld::GetSentinelBody() const
{
	return m_sentinelBody;
}

inline const ndBodyList& ndWorld::GetBodyList() const
{
	return m_scene->GetBodyList();
}

inline const ndJointList& ndWorld::GetJointList() const
{
	return m_jointList;
}

inline dFloat32 ndWorld::GetUpdateTime() const
{
	return m_lastExecutionTime;
}

inline void ndWorld::OnPostUpdate(dFloat32 timestep)
{
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
