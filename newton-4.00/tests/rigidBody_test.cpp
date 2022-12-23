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

/* Return pointer to dynamic unit sphere. */
static ndBodyDynamic *BuildSphere(const ndVector& pos, const ndVector& gravity={0.f}) 
{
  // Create the rigid body and configure gravity for it.
  ndBodyDynamic *const body = new ndBodyDynamic();
  body->SetNotifyCallback(new ndBodyNotify(gravity));

  // Set the position of the sphere in the world.
  ndMatrix matrix(ndGetIdentityMatrix());
  matrix.m_posit = pos;
  body->SetMatrix(matrix);

  // Attach the collision shape and use a convenience function to automatically
  // compute the inertia matrix for the body.
  ndShapeInstance sphere(new ndShapeSphere(1.0f));
  body->SetCollisionShape(sphere);
  body->SetMassMatrix(2.0f, sphere);

  // Disable damping for the tests to better compare the Newton
  // results with the analytical ones.
  body->SetAngularDamping(ndVector(0.f));
  body->SetLinearDamping(0.f);

  return body;
}


/* Rigid body must not move in the absence of forces. */
TEST(RigidBody, NoMoveWithoutForce) {
  ndWorld world;
  world.SetSubSteps(2);

  // Create a sphere at the origin. No gravity will act on it by default.
  ndVector spherePos = ndVector(0.0f, 0.0f, 0.0f, 1.0f);
  ndSharedPtr<ndBody> sphere (BuildSphere(spherePos));
  world.AddBody(sphere);

  // Sanity check: sphere must be at the origin.
  ndVector errVec = sphere->GetMatrix().m_posit - spherePos;
  ndFloat32 err = errVec.DotProduct(errVec).GetScalar();
  EXPECT_NEAR(err, 0, 1E-6);

  // Simulate one second.
  for (int i = 0; i < 60; i++) {
    world.Update(1.0f / 60.0f);
    world.Sync();
  }

  // Since no forces acted on the sphere it must not have moved.
  EXPECT_NEAR(err, 0, 1E-6);
}


/* Apply a force of 1N for 1 second to a body with 2kg of mass
   and verify that it moved 0.25 meters. */
TEST(RigidBody, MoveWithUnitForce) {
  ndWorld world;
  world.SetSubSteps(2);

  // Create a sphere at the origin and apply a gravity force of 1N.
  ndVector spherePos = ndVector(0.f, 0.f, 0.f, 1.f);
  ndVector gravity = ndVector(1.f, 0.f, 0.f, 1.f);
  ndSharedPtr<ndBody> sphere (BuildSphere(spherePos, gravity));
  world.AddBody(sphere);

  // Sanity check: the distance to the origin must be zero.
  ndVector errVec = sphere->GetMatrix().m_posit - spherePos;
  ndFloat32 err = errVec.DotProduct(errVec & ndVector::m_triplexMask).GetScalar();
  EXPECT_NEAR(err, 0, 1E-6);

  // Simulate one second.
  for (int i = 0; i < 60; i++) {
    world.Update(1.0f / 60.0f);
    world.Sync();
  }

  // Verify that the sphere moved 0.25 meters in the X-direction.
  errVec = sphere->GetMatrix().m_posit - ndVector(0.5f, 0.f, 0.f, 1.f);
  err = errVec.DotProduct(errVec & ndVector::m_triplexMask).GetScalar();
  EXPECT_NEAR(err, 0, 1E-4);
}
