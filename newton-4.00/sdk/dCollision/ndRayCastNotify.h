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

#ifndef __D_RAYCAST_H__
#define __D_RAYCAST_H__

#include "ndCollisionStdafx.h"
#include "ndBody.h"
#include "ndContact.h"
#include "ndScene.h"
#include "ndShapeInstance.h"

D_MSV_NEWTON_ALIGN_32
class ndRayCastNotify
{
	public: 
	ndRayCastNotify(const ndScene* const scene)
		:m_scene(scene)
	{
	}

	virtual ~ndRayCastNotify()
	{
	}

	dFloat32 TraceRay(const dVector& p0, const dVector& p1)
	{
		return m_scene->RayCast(*this, p0, p1);
	}

	virtual dUnsigned32 OnRayPrecastAction(const ndBody* const body, const ndShapeInstance* const collision)
	{
		return 1;
	}

	virtual dFloat32 OnRayCastAction(const ndContactPoint& contact, dFloat32 intersetParam) = 0;

	protected:
	const ndScene* m_scene;
} D_GCC_NEWTON_ALIGN_32 ;

D_MSV_NEWTON_ALIGN_32
class ndRayCastClosestHitCallback: public ndRayCastNotify
{
	public:
	ndRayCastClosestHitCallback(const ndScene* const scene)
		:ndRayCastNotify(scene)
		,m_param(dFloat32(1.2f))
	{
	}

	dUnsigned32 OnRayPrecastAction(const ndBody* const body, const ndShapeInstance* const collision)
	{
		return ((ndBody*)body)->GetAsBodyTriggerVolume() ? 0 : 1;
	}

	dFloat32 OnRayCastAction(const ndContactPoint& contact, dFloat32 intersetParam)
	{
		if (intersetParam < m_param)
		{
			m_param = intersetParam;
			m_contact = contact;
		}
		return intersetParam;
	}

	ndContactPoint m_contact;
	protected:
	dFloat32 m_param;
} D_GCC_NEWTON_ALIGN_32 ;


#endif
