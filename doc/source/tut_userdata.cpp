#include <stdio.h>
#include "Newton.h"


// Define a custom data structure to store a body ID.
struct UserDataBody {
  int bodyID;
};


struct UserDataWorld {
  int worldID;
};


void cb_applyForce(const NewtonBody* const body, dFloat timestep, int threadIndex) {
  // Get the user data for this body.
  UserDataBody *myDataBody = (UserDataBody*)NewtonBodyGetUserData(body);
  printf("BodyID: %d\n", myDataBody->bodyID++);

  // We first need to get a pointer to the Newton world, before we can query its
  // user data.
  NewtonWorld *world = NewtonBodyGetWorld(body);
  UserDataWorld *myDataWorld = (UserDataWorld*)NewtonWorldGetUserData(world);
  printf("WorldID: %d\n", myDataWorld->worldID++);
}


void addSphereToSimulation(NewtonWorld *world) {
  // Neutral transformation matrix.
  float	foo[16] = {
    1.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 1.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 1.0f, 0.0f,
    0.0f, 0.0f, 0.0f, 1.0f
  };

  // Create a body with a spherical collision shape, and set its mass/inertia.
  NewtonCollision* const collision = NewtonCreateSphere(world, 1, 0, NULL);
  NewtonBody* const body = NewtonCreateDynamicBody(world, collision, foo);
  NewtonBodySetMassMatrix(body, 1.0f, 1, 1, 1);

  // Install callback. Newton will call it whenever the object moves.
  NewtonBodySetForceAndTorqueCallback(body, cb_applyForce);

  // Attach custom data to this body.
  UserDataBody *myDataBody = new UserDataBody;
  myDataBody->bodyID = 5;
  NewtonBodySetUserData(body, (void *)myDataBody);

  // Attach custom data to the world.
  UserDataWorld *myDataWorld = new UserDataWorld;
  myDataWorld->worldID = 10;
  NewtonWorldSetUserData(world, (void *)myDataWorld);
}


int main (int argc, const char * argv[]) {
  // Create the Newton world.
  NewtonWorld* const world = NewtonCreate();

  // Add the sphere.
  addSphereToSimulation(world);

  // Step the simulation.
  const float timestep = 1.0f / 60;
  for(int i=0; i<5; i++) {
    NewtonUpdate(world, timestep);
  }

  // Clean up.
  NewtonDestroyAllBodies(world);
  NewtonDestroy(world);

  return 0;
}
