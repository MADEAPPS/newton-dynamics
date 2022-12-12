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


// Subclass the ContactNotify callback to intercept all contact events.
//
// In this particular case we override `OnContactCallback` to get notified
// whenever the ball hits the ground.
class MyContactNotify : public ndContactNotify {
public:
  MyContactNotify() {};

  virtual void OnContactCallback(const ndContact *const contact,
                                 ndFloat32 timestep) {
    ndBody *body0 = contact->GetBody0();
    ndBody *body1 = contact->GetBody1();
    printf("OnContactCallback: Pos1.y=%.1f -- Pos2.y%.1f\n",
           body0->GetMatrix().m_posit.m_y,
           body1->GetMatrix().m_posit.m_y);
  }
};

// Create box that will serve as a static floor and return a pointer to it.
ndBodyDynamic *BuildFloor() {
  const float thickness = 1.0f;

  // Default gravity for the body.
  ndVector defaultGravity = ndVector(0.0f, 0.0f, 0.0f, 0.0f);

  //ndBodyDynamic *const body = new ndBodyDynamic();
  ndSharedPtr<ndBodyKinematic> body (new ndBodyDynamic());
  body->SetNotifyCallback(new ndBodyNotify(defaultGravity));
  body->SetCollisionShape(new ndShapeBox(200.0f, thickness, 200.f));

  // Position the floor such that the surface is at y=0.
  ndMatrix matrix(ndGetIdentityMatrix());
  matrix.m_posit.m_y = -thickness / 2;
  body->SetMatrix(matrix);

  return body;
}

// Create a rigid body with a spherical collision shape and return a pointer to it.
ndBodyDynamic* BuildSphere()
{
  // Transform matrix to specify the initial location of the sphere.
  ndMatrix matrix(ndGetIdentityMatrix());
  matrix.m_posit.m_y = 9;

  // Default gravity for the body.
  ndVector defaultGravity = ndVector(0.0f, -10.0f, 0.0f, 0.0f);

  // Create the rigid body and its position.
  ndBodyDynamic *const body = new ndBodyDynamic();
  body->SetNotifyCallback(new ndBodyNotify(defaultGravity));
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

  // Install our own ContactNotify callback handler. Newton will call its
  // various methods to signal AABB overlaps or contact events.
  world.SetContactNotify(new MyContactNotify());

  // Create a ball above a floor box.
  ndBodyDynamic *sphere = BuildSphere();
  ndBodyDynamic *floor = BuildFloor();
  world.AddBody(sphere);
  world.AddBody(floor);

  // Step the simulation for a few steps to see the callbacks in action.
  int frameCnt = 0;
  for (ndInt32 i = 0; i < 240; i++)
  {
    world.Update(1.0f / 60.0f);

    // Print the body position every 10 frames.
    if (frameCnt++ % 10 == 0)
    {
      // Wait for Newton to finish its calculations.
      world.Sync();

      ndVector smat = sphere->GetMatrix().m_posit;
      ndVector fmat = floor->GetMatrix().m_posit;
      printf("Frame %3d: Sphere=(%.1f, %.1f, %.1f)  Floor=(%.1f, %.1f, %.1f)\n",
             frameCnt - 1,
             smat.m_x, smat.m_y,
             smat.m_z, fmat.m_x,
             fmat.m_y, fmat.m_z);
    }
  }
  return 0;
}
