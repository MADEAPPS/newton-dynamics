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

#include <ndNewton.h>
#include "ndRayCastC.h"

D_MSV_NEWTON_ALIGN_32
class ndRayCastCallback : public ndRayCastNotify
{
	public:
		ndRayCastCallback(
			const ndScene* const scene, void* const userdata,
			ndRayCastFilterCallback filter, ndRayCastPrefilterCallback prefilter)
		:ndRayCastNotify(scene)
		,m_userdata(userdata)
		,m_filter(filter)
		,m_prefilter(prefilter)
	{
	}

	virtual dUnsigned32 OnRayPrecastAction(const ndBody* const body, const ndShapeInstance* const collision)
	{
		return m_prefilter ? m_prefilter(m_userdata, (ndBodyDynamicC)body, (ndShapeInstanceC)collision) : 1;
	}

	dFloat32 OnRayCastAction(const ndContactPoint& contact, dFloat32 intersetParam)
	{
		ndRayCastContactC contactc;
		contactc.m_point.m_x = contact.m_point.m_x;
		contactc.m_point.m_y = contact.m_point.m_y;
		contactc.m_point.m_z = contact.m_point.m_z;
		contactc.m_point.m_w = contact.m_point.m_w;
		contactc.m_normal.m_x = contact.m_normal.m_x;
		contactc.m_normal.m_y = contact.m_normal.m_y;
		contactc.m_normal.m_z = contact.m_normal.m_z;
		contactc.m_normal.m_w = dFloat32 (0.0f);

		contactc.m_body = (ndBodyDynamicC)contact.m_body0;
		contactc.m_shapeInstance = (ndShapeInstanceC)contact.m_shapeInstance0;
		contactc.m_penetration = contact.m_penetration;

		return m_filter ? m_filter(m_userdata, &contactc, intersetParam) : intersetParam;
	}

	void* const m_userdata;
	ndRayCastFilterCallback m_filter;
	ndRayCastPrefilterCallback m_prefilter;
	
} D_GCC_NEWTON_ALIGN_32;

dFloat32 ndWorldRayCast(ndWorldC worldc, dFloat32* const p0, dFloat32* const p1,
	void* const userdata, ndRayCastFilterCallback filter, ndRayCastPrefilterCallback prefilter)
{
	ndWorld* const world = (ndWorld*)worldc;

	ndRayCastCallback rayCaster(world->GetScene(), userdata, filter, prefilter);

	dVector r0(p0[0], p0[1], p0[2], dFloat32(1.0f));
	dVector r1(p1[0], p1[1], p1[2], dFloat32(1.0f));
	return rayCaster.TraceRay(p0, p1);
}
