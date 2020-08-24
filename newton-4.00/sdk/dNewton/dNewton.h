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

#ifndef _D_NEWTON_H_
#define _D_NEWTON_H_

class dNewton
	:public dClassAlloc
	,public dSyncMutex
	,public dThread
	,public dThreadPool
{
	public:
	D_NEWTON_API dNewton();
	D_NEWTON_API virtual ~dNewton();

	D_NEWTON_API void Update(dFloat32 timestep);
	D_NEWTON_API void Sync();

	D_NEWTON_API dInt32 GetThreadCount() const;
	D_NEWTON_API void SetThreadCount(dInt32 count);

	D_NEWTON_API dInt32 GetSubSteps() const;
	D_NEWTON_API void SetSubSteps(dInt32 subSteps);

	D_NEWTON_API void DispatchJobs(dThreadPoolJob** const jobs);

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

	private:
	virtual void ThreadFunction();

	protected:

	private:
	dFloat32 m_timestep;
	dInt32 m_subSteps;
};

#endif
