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
#include "dMatrix.h"
#include "CastFloor.h"

#pragma warning (disable: 4100) //unreferenced formal parameter



static unsigned ConvexCastCallback (const NewtonBody* body, const NewtonCollision* collision, void* userData)
{
	// this convex cast have to skip the casting body
	NewtonBody* me = (NewtonBody*) userData;
	return (me == body) ? 0 : 1;
}


void ConvexCastPlacement (NewtonBody* body)
{
	dFloat param;
	dMatrix matrix;
	NewtonWorld* world;
	NewtonCollision* collision;
	NewtonWorldConvexCastReturnInfo info[16];

	// get the Body Matrix;
	NewtonBodyGetMatrix (body, &matrix[0][0]);

	// add some search for a floor surface withe it +20 unit up and -20 units down
	matrix.m_posit.m_y += 20.0f;
	dVector p (matrix.m_posit);
	p.m_y -= 40.0f;

	// cast the collision shape of the body for +20 to -20 units
	world = NewtonBodyGetWorld(body);
	collision = NewtonBodyGetCollision(body);
	NewtonWorldConvexCast (world, &matrix[0][0], &p[0], collision, &param, body, ConvexCastCallback, info, 16, 0);
	_ASSERTE (param < 1.0f);

	// the point at the intersection param is the floor 
	matrix.m_posit.m_y += (p.m_y - matrix.m_posit.m_y) * param;

	// Set the Body matrix to the new placement Matrix adjusted by the cast proccess.
	NewtonBodySetMatrix(body, &matrix[0][0]);
}

