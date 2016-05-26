#include "stdafx.h"
#include <iostream>
#include <dVector.h>
#include <dMatrix.h>
#include <newton.h>

NewtonBody* CreateBackgroundBody(NewtonWorld* const world)
{
	dFloat points[4][3] = 
	{
		{-100.0f, 0.0f,  100.0f}, 
		{ 100.0f, 0.0f,  100.0f}, 
		{ 100.0f, 0.0f, -100.0f}, 
		{-100.0f, 0.0f, -100.0f}, 
	};

	// crate a collision tree
	NewtonCollision* const collision = NewtonCreateTreeCollision (world, 0);

	// start building the collision mesh
	NewtonTreeCollisionBeginBuild (collision);

	// add the face one at a time
	NewtonTreeCollisionAddFace (collision, 4, &points[0][0], 3 * sizeof (dFloat), 0);

	// finish building the collision
	NewtonTreeCollisionEndBuild (collision, 1);

	// create a body with a collision and locate at the identity matrix position 
	dMatrix matrix (dGetIdentityMatrix());
	NewtonBody* const body = NewtonCreateDynamicBody(world, collision, &matrix[0][0]);

	// do no forget to destroy the collision after you not longer need it
	NewtonDestroyCollision(collision);
	return body;
}

static void ApplyGravity (const NewtonBody* const body, dFloat timestep, int threadIndex)
{
	// apply gravity force to the body
	dFloat mass;
	dFloat Ixx;
	dFloat Iyy;
	dFloat Izz;

	NewtonBodyGetMass(body, &mass, &Ixx, &Iyy, &Izz);
	dVector gravityForce (0.0f, -9.8f * mass, 0.0f, 0.0f);
	NewtonBodySetForce(body, &gravityForce[0]);
}

NewtonBody* CreateFreeFallBall(NewtonWorld* const world)
{
	// crate a collision sphere
	NewtonCollision* const collision = NewtonCreateSphere(world, 1.0f, 0, NULL);

	// create a dynamic body with a sphere shape, and 
	dMatrix matrix (dGetIdentityMatrix());
	matrix.m_posit.m_y = 50.0f;
	NewtonBody* const body = NewtonCreateDynamicBody(world, collision, &matrix[0][0]);

	// set the force callback for applying the force and torque
	NewtonBodySetForceAndTorqueCallback(body, ApplyGravity);

	// set the mass for this body
	dFloat mass = 1.0f;
	NewtonBodySetMassProperties(body, mass, collision);

	// set the linear damping to zero
	NewtonBodySetLinearDamping (body, 0.0f);

	// do no forget to destroy the collision after you not longer need it
	NewtonDestroyCollision(collision);
	return body;

}

int _tmain(int argc, _TCHAR* argv[])
{
	// create a newton world
	NewtonWorld* const world = NewtonCreate ();

	// create a static body to serve as the floor.
	NewtonBody* const background = CreateBackgroundBody (world);
	NewtonBody* const freeFallBall = CreateFreeFallBall (world);

	// for deterministic behavior call this function each time you change the world
	NewtonInvalidateCache (world);

	// run the simulation loop
	for (int i = 0; i < 300; i ++) {
		NewtonUpdate (world, 1.0f/60.f);

		dMatrix matrix;
		NewtonBodyGetMatrix (freeFallBall, &matrix[0][0]);
		std::cout << "height: " << matrix.m_posit.m_y << std::endl;
	}

	// destroy the newton world
	NewtonDestroy (world);
	return 0;
}

