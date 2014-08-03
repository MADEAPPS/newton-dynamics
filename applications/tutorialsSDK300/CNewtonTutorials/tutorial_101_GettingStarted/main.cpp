#include "stdafx.h"
#include <iostream>
#include <dVector.h>
#include <dMatrix.h>
#include <dNewton.h>
#include <dNewtonCollision.h>
#include <dNewtonDynamicBody.h>

// make a custom rigid body for the application to use as the base class of all it dynamics bodies
class MyDynamicBody: public dNewtonDynamicBody
{
	public:
	MyDynamicBody(dNewton* const world, dFloat mass, const dNewtonCollision* const collision, void* const userData, const dMatrix& matrix)
		:dNewtonDynamicBody (world, mass, collision, userData, &matrix[0][0], NULL)
	{
	}

	// the end application need to overload this function from dNetwonBody
	void OnBodyTransform (const dFloat* const matrix, int threadIndex)
	{
		Update (matrix);
	}

	// the end application need to overload this function from dNetwonDynamicBody
	void OnForceAndTorque (dFloat timestep, int threadIndex)
	{
		// apply gravity force to the body
		dFloat mass;
		dFloat Ixx;
		dFloat Iyy;
		dFloat Izz;

		GetMassAndInertia (mass, Ixx, Iyy, Izz);
		dVector gravityForce (0.0f, -9.8f * mass, 0.0f, 0.0f);
		SetForce (&gravityForce[0]);
	}
};

static dNewtonDynamicBody* CreateBackgroundBody(dNewton* const world)
{
	// make a flat quad 
	dFloat points[4][3] = 
	{
		{-100.0f, 0.0f,  100.0f}, 
		{ 100.0f, 0.0f,  100.0f}, 
		{ 100.0f, 0.0f, -100.0f}, 
		{-100.0f, 0.0f, -100.0f}, 
	};

	// create a collision tree instance with collision mask 1 
	dNewtonCollisionMesh collision (world, 1);

	// start building the collision mesh
	collision.BeginFace();

	// add the face one at a time
	collision.AddFace (4, &points[0][0], 3 * sizeof (dFloat), 0);

	// finish building the collision
	collision.EndFace();

	// create a body with a collision and locate at the identity matrix position 
	return new MyDynamicBody (world, 0, &collision, NULL, dGetIdentityMatrix());
}


static dNewtonDynamicBody* CreateFreeFallBall(dNewton* const world)
{
	// crate a collision sphere instance, of radio 1.0 and a collision mask 1 
	dNewtonCollisionSphere collision (world, 1.0f, 1);

	// create a dynamic body with a sphere shape, mass of 1.0 kg, located 50 units above the ground 
	dMatrix matrix (dGetIdentityMatrix());
	matrix.m_posit.m_y = 50.0f;
	dNewtonDynamicBody* const body = new MyDynamicBody (world, 1.0, &collision, NULL, matrix);

	// set the linear damping to zero
	body->SetLinearDrag (0.0f);

	return body;
}


int _tmain(int argc, _TCHAR* argv[])
{
	// create a newton world
	dNewton world;

	// create a static body to serve as the floor.
	dNewtonDynamicBody* const background = CreateBackgroundBody(&world);

	// create a free fall body
	dNewtonDynamicBody* const freeFallBall = CreateFreeFallBall (&world);

	// run the simulation loop
	for (int i = 0; i < 300; i ++) {
		// this function advance the simulation by the timestep, if another call is made before the time step of the previews call is exhausted, the call is ignored
		//world.Update (1.0f/60.f);

		// this function will advance the simulation by the timestep, on each call
		world.UpdateOffLine (1.0f/60.f);

		dMatrix matrix;
		freeFallBall->InterpolateMatrix (1.0f, &matrix[0][0]);
		std::cout << "height: " << matrix.m_posit.m_y << std::endl;
	}

	return 0;
}

