/* Copyright (c) <2009> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/

#include "StdAfx.h"
#include "dVector.h"
#include "FindFloor.h"

#pragma warning (disable: 4100) //unreferenced formal parameter

static dFloat RayCastPlacement (const NewtonBody* body, const dFloat* normal, int collisionID, void* userData, dFloat intersetParam)
{
	dFloat* paramPtr;
	paramPtr = (dFloat*)userData;
	if (intersetParam < paramPtr[0]) {
		paramPtr[0] = intersetParam;
	}
	return paramPtr[0];
}



dFloat FindFloor (const NewtonWorld* world, dFloat x, dFloat z)
{
	dFloat parameter;
	// shot a vertical ray from a high altitude and collect the intersection parameter.
	dVector p0 (x, 1000.0f, z); 
	dVector p1 (x, -1000.0f, z); 

	parameter = 1.2f;
	NewtonWorldRayCast (world, &p0[0], &p1[0], RayCastPlacement, &parameter, NULL);
	//_ASSERTE (parameter < 1.0f);

	// the intersection is the interpolated value
	return 1000.0f - 2000.0f * parameter;
}




