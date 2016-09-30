#include <stdio.h>
#include "Newton.h"


void cb_applyForce(const NewtonBody* const body, dFloat timestep, int threadIndex) {
  // Apply a force to the object.
  dFloat force[3] = {0, 1.0, 0};
  NewtonBodySetForce(body, force);

  // Query the state (4x4 matrix) and extract the body's position.
  float state[16];
  NewtonBodyGetMatrix(body, state);
  printf("Time %.3fs: x=%.3f  y=%.3f  z=%.2f\n",
         timestep, state[12], state[13], state[14]);
}


void addSphereToSimulation(NewtonWorld *world) {
  // Transform matrix.
  float	foo[16] = {
    1.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 1.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 1.0f, 0.0f,
    0.0f, 0.0f, 0.0f, 1.0f
  };

  // Create a spherical collision shape.
  const dFloat radius = 1.0;
  NewtonCollision* const collision = NewtonCreateSphere(world, radius, 0, NULL);

  // Create the actual body and assign it the collision shape.
  NewtonBody* const body = NewtonCreateDynamicBody(world, collision, foo);

  // Specify the mass- and inertia properties.
  NewtonBodySetMassMatrix(body, 1.0f, 1, 1, 1);

  // Install force callback. Newton will call it at every step.
  NewtonBodySetForceAndTorqueCallback(body, cb_applyForce);
}


int main (int argc, const char * argv[]) {
  // Create the Newton world.
  NewtonWorld* const world = NewtonCreate();

  // Add the sphere.
  addSphereToSimulation(world);

  // Step the world several times.
  const float timestep = 1.0f / 60;
  for(int i=0; i<60; i++) {
    NewtonUpdate(world, timestep);
  }

  // Clean up.
  NewtonDestroyAllBodies(world);
  NewtonDestroy(world);

  return 0;
}
