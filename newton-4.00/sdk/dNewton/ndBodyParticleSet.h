/* Copyright (c) <2003-2021> <Julio Jerez, Newton Game Dynamics>
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

#include "ndNewtonStdafx.h"
#include "ndBodyParticleSetList.h"

D_MSV_NEWTON_ALIGN_32
class ndBodyParticleSet: public ndBody
{
	public:
	D_NEWTON_API ndBodyParticleSet();
	D_NEWTON_API ndBodyParticleSet(const ndLoadSaveBase::ndLoadDescriptor& desc);
	D_NEWTON_API virtual ~ndBodyParticleSet ();
	D_NEWTON_API virtual void Save(const ndLoadSaveBase::ndSaveDescriptor& desc) const;

	const ndArray<ndVector>& GetPositions() const;
	virtual ndBodyParticleSet* GetAsBodyParticleSet();

	ndFloat32 GetParticleRadius() const;
	void SetParticleRadius(ndFloat32 raidus);
	
	D_NEWTON_API virtual void AddParticle(const ndFloat32 mass, const ndVector& position, const ndVector& velocity) = 0;

	D_NEWTON_API virtual void Update(const ndWorld* const workd, ndFloat32 timestep) = 0;

	protected:
	ndArray<ndVector> m_posit;
	ndBodyParticleSetList::ndNode* m_listNode;
	ndFloat32 m_radius;
	friend class ndWorld;
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

inline const ndArray<ndVector>& ndBodyParticleSet::GetPositions() const
{
	return m_posit;
}

#endif 


