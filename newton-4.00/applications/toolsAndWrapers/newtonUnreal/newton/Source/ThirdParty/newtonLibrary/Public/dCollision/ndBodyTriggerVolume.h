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

#ifndef __ND_BODY_TRIGGER_VOLUME_H__
#define __ND_BODY_TRIGGER_VOLUME_H__

#include "ndCollisionStdafx.h"
#include "ndBodyKinematicBase.h"

D_MSV_NEWTON_ALIGN_32
class ndBodyTriggerVolume : public ndBodyKinematicBase
{
	public:
	D_CLASS_REFLECTION(ndBodyTriggerVolume, ndBodyKinematicBase)
	D_COLLISION_API ndBodyTriggerVolume();
	D_COLLISION_API virtual ~ndBodyTriggerVolume();

	ndBodyTriggerVolume* GetAsBodyTriggerVolume();

	virtual void OnTrigger(ndBodyKinematic* const body, ndFloat32 timestep);
	virtual void OnTriggerEnter(ndBodyKinematic* const body, ndFloat32 timestep);
	virtual void OnTriggerExit(ndBodyKinematic* const body, ndFloat32 timestep);

	D_COLLISION_API virtual void SpecialUpdate(ndFloat32 timestep);

	private:
	virtual void IntegrateExternalForce(ndFloat32 timestep);

} D_GCC_NEWTON_ALIGN_32;

inline ndBodyTriggerVolume* ndBodyTriggerVolume::GetAsBodyTriggerVolume()
{ 
	return this; 
}

inline void ndBodyTriggerVolume::OnTriggerEnter(ndBodyKinematic* const, ndFloat32)
{
	//dAssert(0);
}

inline void ndBodyTriggerVolume::OnTrigger(ndBodyKinematic* const, ndFloat32)
{
	//dAssert(0);
}

inline void ndBodyTriggerVolume::OnTriggerExit(ndBodyKinematic* const, ndFloat32)
{
	//dAssert(0);
}

inline void ndBodyTriggerVolume::IntegrateExternalForce(ndFloat32) 
{
}

#endif