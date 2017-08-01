/* Copyright (c) <2003-2016> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/

#include "toolbox_stdafx.h"
#include "MousePick.h"


class dMousePickClass
{
	public:
	dMousePickClass ()
		:m_normal(0.0f)
		,m_param (1.0f)
		,m_body(NULL)
	{
	}

	// implement a ray cast pre-filter
	static unsigned RayCastPrefilter (const NewtonBody* body,  const NewtonCollision* const collision, void* const userData)
	{
		// ray cannot pick trigger volumes
		//return NewtonCollisionIsTriggerVolume(collision) ? 0 : 1;

		const NewtonCollision* const parent = NewtonCollisionGetParentInstance(collision);
		if (parent) {
			// you can use this to filter sub collision shapes.  
			dAssert (NewtonCollisionGetSubCollisionHandle (collision));
		}

		return (NewtonBodyGetType(body) == NEWTON_DYNAMIC_BODY) ? 1 : 0;
	}

	static dFloat RayCastFilter (const NewtonBody* const body, const NewtonCollision* const collisionHit, const dFloat* const contact, const dFloat* const normal, dLong collisionID, void* const userData, dFloat intersetParam)
	{
		dFloat mass;
		dFloat Ixx;
		dFloat Iyy;
		dFloat Izz;

		// check if we are hitting a sub shape
		const NewtonCollision* const parent = NewtonCollisionGetParentInstance(collisionHit);
		if (parent) {
			// you can use this to filter sub collision shapes.  
			dAssert (NewtonCollisionGetSubCollisionHandle (collisionHit));
		}

		dMousePickClass* const data = (dMousePickClass*) userData;
		NewtonBodyGetMass (body, &mass, &Ixx, &Iyy, &Izz);
		if ((mass > 0.0f) || (NewtonBodyGetType(body) == NEWTON_KINEMATIC_BODY)) {
			data->m_body = body;
		}

		
		if (intersetParam < data->m_param) {
			data->m_param = intersetParam;
			data->m_normal = dVector (normal[0], normal[1], normal[2]);
		}
		return intersetParam;
	}

	dVector m_normal;
	dFloat m_param;
	const NewtonBody* m_body;
};


NewtonBody* MousePickBody (NewtonWorld* const nWorld, const dVector& origin, const dVector& end, dFloat& paramterOut, dVector& positionOut, dVector& normalOut)
{
	dMousePickClass rayCast;
	NewtonWorldRayCast(nWorld, &origin[0], &end[0], dMousePickClass::RayCastFilter, &rayCast, dMousePickClass::RayCastPrefilter, 0);

	if (rayCast.m_body) {
		positionOut = origin + (end - origin).Scale (rayCast.m_param);
		normalOut = rayCast.m_normal;
		paramterOut = rayCast.m_param;
	}
	return (NewtonBody*) rayCast.m_body;
}