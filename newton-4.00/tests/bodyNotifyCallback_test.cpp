/* Copyright (c) <2003-2019> <Newton Game Dynamics>
*
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
*
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/

#include <cstdio>
#include "ndNewton.h"
#include <gtest/gtest.h>

class ndApplyFixForce : public ndBodyNotify
{
	public:
	ndApplyFixForce()
		:ndBodyNotify(ndVector::m_zero)
		,m_fixForce(1.0f, 0.0f, 0.0f, 0.0f)
	{
	}

	void OnApplyExternalForce(ndInt32 threadIndex, ndFloat32 timestep)
	{
		ndBodyNotify::OnApplyExternalForce(threadIndex, timestep);

		ndBodyKinematic* const body = GetBody()->GetAsBodyKinematic();
		ndVector force(body->GetForce() + m_fixForce);
		body->SetForce(force);
	}

	private:
	ndVector m_fixForce;
};

/* Return pointer to dynamic unit sphere. */
static ndBodyDynamic *BuildSphere(const ndVector& pos, float mass) 
{
  // Create the rigid body and configure gravity for it.
  ndBodyDynamic *const body = new ndBodyDynamic();
  body->SetNotifyCallback(new ndApplyFixForce());

  // Set the position of the sphere in the world.
  ndMatrix matrix(ndGetIdentityMatrix());
  matrix.m_posit = pos;
  body->SetMatrix(matrix);

  // Attach the collision shape and use a convenience function to automatically
  // compute the inertia matrix for the body.
  ndShapeInstance sphere(new ndShapeSphere(1.0f));
  body->SetCollisionShape(sphere);
  body->SetMassMatrix(mass, sphere);

  // Disable damping for the tests to better compare the Newton
  // results with the analytical ones.
  body->SetAngularDamping(ndVector(0.f));
  body->SetLinearDamping(0.f);

  return body;
}


/* Apply a force of 1N for 1 second to a body with 2kg of mass
   and verify that it moved 0.25 meters. */
TEST(RigidBodyNotify, MoveWithUnitForceDifferentMasses) 
{
  ndWorld world;
  world.SetSubSteps(2);

  // Create a sphere at the origin and apply and mass 1 kg.
  //ndBodyDynamic* sphere0 = BuildSphere(ndVector(0.0f, 0.0f, 0.0f, 1.0f), 1.0f);
  ndSharedPtr<ndBody> sphere0 (BuildSphere(ndVector(0.0f, 0.0f, 0.0f, 1.0f), 1.0f));
  world.AddBody(sphere0);

  // Create a sphere at the origin and apply and mass 2 kg.
  //ndBodyDynamic* sphere1 = BuildSphere(ndVector(0.0f, 0.0f, 2.5f, 1.0f), 2.0f);
  ndSharedPtr<ndBody> sphere1 (BuildSphere(ndVector(0.0f, 0.0f, 2.5f, 1.0f), 2.0f));
  world.AddBody(sphere1);

  // Simulate one second.
  for (int i = 0; i < 60; i++) {
    world.Update(1.0f / 60.0f);
    world.Sync();
  }

  ndVector errVec;
  // Verify that the sphere moved 0.5 meters in the X-direction.
  errVec = sphere0->GetPosition() - ndVector(0.5f, 0.f, 0.f, 1.f);
  float err = float(errVec.DotProduct(errVec & ndVector::m_triplexMask).GetScalar());
  EXPECT_NEAR(err, 0, 1E-4);

  // Verify that the sphere moved 0.25 meters in the X-direction.
  errVec = sphere1->GetPosition() - ndVector(0.25f, 0.f, 2.5f, 1.f);
  err = float(errVec.DotProduct(errVec & ndVector::m_triplexMask).GetScalar());
  EXPECT_NEAR(err, 0, 1E-4);
}
