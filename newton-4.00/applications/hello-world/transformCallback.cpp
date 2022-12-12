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


// Subclass the callback for a body to customize the gravity and print the transform.
//
// The default callback for each body will apply the gravity force on each
// object. To override that behavior and change the gravity we have to subclass
// from `ndBodyNotify`.
class MyBodyNotify: public ndBodyNotify
{
public:
  // Default constructor where we need to set the default gravity. The does not
  // matter here since we will manually apply the gravity in the
  // `OnApplyExternalForce` method.
  MyBodyNotify(ndVector defaultGravity) : ndBodyNotify(defaultGravity) {}

  // Newton calls this every frame (unless sleeping). We can use this to apply
  // any forces (including gravity) and torque to the body as we see fit.
  virtual void OnApplyExternalForce(ndInt32 threadIdx, ndFloat32 timestep)
  {
    ndBodyDynamic* const dynamicBody = GetBody()->GetAsBodyDynamic();
    if (dynamicBody)
    {
      printf("OnApplyExternalForce callback: Apply gravity force\n");
      dynamicBody->SetForce(ndVector(0.f, -20.0f, 0.f, 0.f));
      dynamicBody->SetTorque(ndVector::m_zero);
    }
  }

  // Newton calls this every frame (unless sleeping).
  virtual void OnTransform(ndInt32 threadIndex, const ndMatrix &mat)
  {
    ndBodyDynamic *const body = GetBody()->GetAsBodyDynamic();
    printf("OnTransform callback: Pos=(%.1f, %.1f, %.1f)  Vel=(%.1f, %.1f, "
           "%.1f)\n",
           mat.m_posit.m_x, mat.m_posit.m_y, mat.m_posit.m_z, body->GetVelocity().m_x,
           body->GetVelocity().m_y, body->GetVelocity().m_z);
  }
};


// Create a rigid body with a spherical collision shape and return a pointer to it.
ndBodyDynamic* BuildSphere()
{
  // Transform matrix to specify the initial location of the sphere.
  ndMatrix matrix(ndGetIdentityMatrix());
  matrix.m_posit.m_y = 10;

  // Default gravity for the body.
  ndVector defaultGravity = ndVector(0.0f, 0.0f, 0.0f, 0.0f);

  // Create the rigid body and its position.
  ndBodyDynamic *const body = new ndBodyDynamic();
  body->SetNotifyCallback(new MyBodyNotify(defaultGravity));
  body->SetMatrix(matrix);

  // Attach the collision shape and use a convenience function to automatically
  // compute the inertia matrix for the body.
  ndShapeInstance sphere(new ndShapeSphere(1.0f));
  body->SetCollisionShape(sphere);
  body->SetMassMatrix(1.0f, sphere);

  return body;
}

int main(int, const char**)
{
  // Setup the world itself.
  ndWorld world;

  // Two sub-steps per time step is the recommended default. It should provide
  // stable simulations for most standard scenarios including those with joints.
  world.SetSubSteps(2);

  // Add a single sphere to the world.
  //ndBodyDynamic *sphere = BuildSphere();
  ndSharedPtr<ndBodyKinematic> sphere (BuildSphere());
  world.AddBody(sphere);

  // Step the simulation for a few steps to see the callbacks in action.
  for (ndInt32 i = 0; i < 30; i++)
  {
    // Trigger an asynchronous update.
    world.Update(1.0f / 60.0f);

    // We need to wait for Newton to finish before we can safely access its objects.
    world.Sync();

    // Access the body directly to print info about the sphere after each step.
    printf("From Main loop:       Pos=(%.1f, %.1f, %.1f)  Vel=(%.1f, %.1f, %.1f)\n\n",
           sphere->GetMatrix().m_posit.m_x, sphere->GetMatrix().m_posit.m_y,
           sphere->GetMatrix().m_posit.m_z, sphere->GetVelocity().m_x,
           sphere->GetVelocity().m_y, sphere->GetVelocity().m_z);
  }
  return 0;
}
