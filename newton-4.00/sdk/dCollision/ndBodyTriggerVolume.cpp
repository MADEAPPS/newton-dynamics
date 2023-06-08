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

#include "ndCoreStdafx.h"
#include "ndCollisionStdafx.h"
#include "ndContact.h"
#include "ndBodyTriggerVolume.h"

ndBodyTriggerVolume::ndBodyTriggerVolume()
	:ndBodyKinematicBase()
{
}

ndBodyTriggerVolume::~ndBodyTriggerVolume()
{
}

void ndBodyTriggerVolume::SpecialUpdate(ndFloat32 timestep)
{
	const ndContactMap& contactMap = GetContactMap();

	ndBodyKinematic::ndContactMap::Iterator it(contactMap);
	for (it.Begin(); it; it++)
	{
		const ndContact* const contact = *it;
		if (contact->IsActive() && contact->IsInTrigger())
		{
			OnTrigger(contact->GetBody0(), timestep);
		}
	}
}

//void ndBodyTriggerVolume::ApplyExternalForces(ndInt32 threadIndex, ndFloat32 timestep)
//{
//	if (m_notifyCallback)
//	{
//		m_notifyCallback->OnApplyExternalForce(threadIndex, timestep);
//	}
//}
