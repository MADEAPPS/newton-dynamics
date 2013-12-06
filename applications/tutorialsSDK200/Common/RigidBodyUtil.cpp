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
#include "Entity.h"
#include "dVector.h"
#include "dMatrix.h"
#include "dQuaternion.h"
#include "RigidBodyUtil.h"

#pragma warning (disable: 4100) //unreferenced formal parameter


#define GRAVITY	-10.0f

static void DestroyBodyCallback (const NewtonBody* body);
static void SetTransformCallback (const NewtonBody* body, const dFloat* matrix, int threadIndex);
static void ApplyForceAndTorqueCallback (const NewtonBody* body, dFloat timestep, int threadIndex);


void DestroyBodyCallback (const NewtonBody* body)
{
	// for now there is nothing to destroy
}

// Transform callback to set the matrix of a the visual entity
void SetTransformCallback (const NewtonBody* body, const dFloat* matrix, int threadIndex)
{
	Entity* ent;

	// Get the position from the matrix
	dVector posit (matrix[12], matrix[13], matrix[14], 1.0f);
	dQuaternion rotation;

	// we will ignore the Rotation part of matrix and use the quaternion rotation stored in the body
	NewtonBodyGetRotation(body, &rotation.m_q0);

	// get the entity associated with this rigid body
	ent = (Entity*) NewtonBodyGetUserData(body);

	// since this tutorial run the physics and a different fps than the Graphics
	// we need to save the entity current transformation state before updating the new state.
	ent->m_prevPosition = ent->m_curPosition;
	ent->m_prevRotation = ent->m_curRotation;
	if (ent->m_curRotation.DotProduct (rotation) < 0.0f) {
		ent->m_prevRotation.Scale(-1.0f);
	}

	// set the new position and orientation for this entity
	ent->m_curPosition = posit;
	ent->m_curRotation = rotation;
}


// callback to apply external forces to body
void ApplyForceAndTorqueCallback (const NewtonBody* body, dFloat timestep, int threadIndex)
{
	dFloat Ixx;
	dFloat Iyy;
	dFloat Izz;
	dFloat mass;

	// for this tutorial the only external force in the Gravity
	NewtonBodyGetMassMatrix (body, &mass, &Ixx, &Iyy, &Izz);

	dVector gravityForce  (0.0f, mass * GRAVITY, 0.0f, 1.0f);
	NewtonBodySetForce(body, &gravityForce[0]);
}




NewtonBody* CreateRigidBody (NewtonWorld* world, Entity* ent, NewtonCollision* collision, dFloat mass)
{
	dVector minBox;
	dVector maxBox;
	dVector origin;
	dVector inertia;

	// we need to set physics properties to this body
	dMatrix matrix (ent->m_curRotation, ent->m_curPosition);
	//NewtonBodySetMatrix (body, &matrix[0][0]);


	// Now with the collision Shape we can crate a rigid body
	NewtonBody* const body = NewtonCreateBody (world, collision, &matrix[0][0]);

	// bodies can have a destructor. 
	// this is a function callback that can be used to destroy any local data stored 
	// and that need to be destroyed before the body is destroyed. 
	NewtonBodySetDestructorCallback (body, DestroyBodyCallback);

	// save the entity as the user data for this body
	NewtonBodySetUserData (body, ent);


	// we need to set the proper center of mass and inertia matrix for this body
	// the inertia matrix calculated by this function does not include the mass.
	// therefore it needs to be multiplied by the mass of the body before it is used.
	NewtonConvexCollisionCalculateInertialMatrix (collision, &inertia[0], &origin[0]);	

	// set the body mass matrix
	NewtonBodySetMassMatrix (body, mass, mass * inertia.m_x, mass * inertia.m_y, mass * inertia.m_z);

	// set the body origin
	NewtonBodySetCentreOfMass (body, &origin[0]);

	// set the function callback to apply the external forces and torque to the body
	// the most common force is Gravity
	NewtonBodySetForceAndTorqueCallback (body, ApplyForceAndTorqueCallback);

	// set the function callback to set the transformation state of the graphic entity associated with this body 
	// each time the body change position and orientation in the physics world
	NewtonBodySetTransformCallback (body, SetTransformCallback);

	return body;
}

