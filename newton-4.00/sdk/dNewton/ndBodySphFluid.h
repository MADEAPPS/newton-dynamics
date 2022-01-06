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

#ifndef __ND_BODY_SPH_FLUID_H__
#define __ND_BODY_SPH_FLUID_H__

#include "ndNewtonStdafx.h"
#include "ndBodyParticleSet.h"

D_MSV_NEWTON_ALIGN_32
class ndBodySphFluid: public ndBodyParticleSet
{
	public:
	D_NEWTON_API ndBodySphFluid();
	D_NEWTON_API ndBodySphFluid(const ndLoadSaveBase::ndLoadDescriptor& desc);
	D_NEWTON_API virtual ~ndBodySphFluid ();

	virtual ndBodySphFluid* GetAsBodySphFluid();
	D_NEWTON_API virtual void Save(const ndLoadSaveBase::ndSaveDescriptor& desc) const;

	protected:
	virtual bool RayCast(ndRayCastNotify& callback, const ndFastRay& ray, const ndFloat32 maxT) const;

	private:
} D_GCC_NEWTON_ALIGN_32 ;

inline bool ndBodySphFluid::RayCast(ndRayCastNotify&, const ndFastRay&, const ndFloat32) const
{
	return false;
}

inline ndBodySphFluid* ndBodySphFluid::GetAsBodySphFluid()
{ 
	return this; 
}


#endif 


