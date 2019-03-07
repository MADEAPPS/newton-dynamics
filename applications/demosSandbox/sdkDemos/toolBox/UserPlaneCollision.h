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


#ifndef _USER_PLANE_COLLISION_H 
#define _USER_PLANE_COLLISION_H 

#include "toolbox_stdafx.h"


class DemoEntityManager;

DemoMesh* CreateVisualPlaneMesh (const dVector& plane, DemoEntityManager* const scene);
NewtonCollision* CreateInfinitePlane (NewtonWorld* const world, const dVector& planeEquation);


#endif
