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

#ifndef __D_BODY_PARTICLE_SET_H__
#define __D_BODY_PARTICLE_SET_H__

#include "ndNewtonStdafx.h"
#include "ndBodyParticleSetList.h"

D_MSV_NEWTON_ALIGN_32
class ndBodyParticleSet: public ndBody
{
	public:
	D_NEWTON_API ndBodyParticleSet();
	D_NEWTON_API ndBodyParticleSet(const nd::TiXmlNode* const xmlNode, const dTree<const ndShape*, dUnsigned32>& shapesCache);
	D_NEWTON_API virtual ~ndBodyParticleSet ();

	D_NEWTON_API virtual ndBodyParticleSet* GetAsBodyParticleSet();
	D_NEWTON_API virtual void Save(nd::TiXmlElement* const rootNode, const char* const assetPath, dInt32 nodeid, const dTree<dUnsigned32, const ndShape*>& shapesCache) const;

	const dArray<dVector>& GetPositions() const ;

	D_NEWTON_API virtual void AddParticle(const dFloat32 mass, const dVector& position, const dVector& velocity) = 0;

	D_NEWTON_API virtual void Update(dFloat32 timestep) = 0;

	protected:
	dArray<dVector> m_posit;

	ndBodyParticleSetList::dListNode* m_listNode;
	friend class ndWorld;
} D_GCC_NEWTON_ALIGN_32 ;

inline ndBodyParticleSet* ndBodyParticleSet::GetAsBodyParticleSet() 
{ 
	return this; 
}

inline const dArray<dVector>& ndBodyParticleSet::GetPositions() const
{
	return m_posit;
}

#endif 


