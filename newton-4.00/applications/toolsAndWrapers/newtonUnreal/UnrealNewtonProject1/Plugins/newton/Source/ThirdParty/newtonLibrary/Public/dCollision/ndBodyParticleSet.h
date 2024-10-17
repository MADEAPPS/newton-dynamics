/* Copyright (c) <2003-2022> <Julio Jerez, Newton Game Dynamics>
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

#ifndef __ND_BODY_PARTICLE_SET_H__
#define __ND_BODY_PARTICLE_SET_H__

#include "ndCoreStdafx.h"
#include "ndCollisionStdafx.h"
#include "ndBody.h"

//#define D_USE_NEW_FLUID

D_MSV_NEWTON_ALIGN_32
class ndBodyParticleSet: public ndBody, public ndBackgroundTask
{
	public:
	D_COLLISION_API ndBodyParticleSet();
	D_COLLISION_API virtual ~ndBodyParticleSet ();

	const ndVector GetGravity() const;
	void SetGravity(const ndVector& gravity);

	ndArray<ndVector>& GetVelocity();
	ndArray<ndVector>& GetPositions();
	const ndArray<ndVector>& GetVelocity() const;
	const ndArray<ndVector>& GetPositions() const;
	virtual ndBodyParticleSet* GetAsBodyParticleSet();

	bool GetAsynUpdate() const;
	void SetAsynUpdate(bool update);
	ndFloat32 GetParticleRadius() const;
	virtual void SetParticleRadius(ndFloat32 radius);

	D_COLLISION_API virtual void Update(const ndScene* const scene, ndFloat32 timestep) = 0;

	protected:
	ndVector m_box0;
	ndVector m_box1;
	ndVector m_gravity;
	ndArray<ndVector> m_posit;
	ndArray<ndVector> m_veloc;
	ndBodyList::ndNode* m_listNode;
	ndFloat32 m_radius;
	ndFloat32 m_timestep;
	bool m_updateInBackground;
	friend class ndWorld;
	friend class ndScene;
} D_GCC_NEWTON_ALIGN_32 ;

inline ndBodyParticleSet* ndBodyParticleSet::GetAsBodyParticleSet() 
{ 
	return this; 
}

inline ndFloat32 ndBodyParticleSet::GetParticleRadius() const
{
	return m_radius;
}

inline void ndBodyParticleSet::SetParticleRadius(ndFloat32 raidus)
{
	m_radius = raidus;
}

inline ndArray<ndVector>& ndBodyParticleSet::GetPositions()
{
	return m_posit;
}

inline const ndArray<ndVector>& ndBodyParticleSet::GetPositions() const
{
	return m_posit;
}

inline ndArray<ndVector>& ndBodyParticleSet::GetVelocity()
{
	return m_veloc;
}

inline const ndArray<ndVector>& ndBodyParticleSet::GetVelocity() const
{
	return m_veloc;
}

inline const ndVector ndBodyParticleSet::GetGravity() const
{
	return m_gravity;
}

inline void ndBodyParticleSet::SetGravity(const ndVector& gravity)
{
	m_gravity = gravity & ndVector::m_triplexMask;
}

inline bool ndBodyParticleSet::GetAsynUpdate() const
{
	return m_updateInBackground;
}

inline void ndBodyParticleSet::SetAsynUpdate(bool updatType)
{
	m_updateInBackground = updatType;
}


#endif 


