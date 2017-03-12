#include <stdio.h>
#include "Newton.h"


// Define a custom data structure to store a body ID.
struct UserData {
  int bodyID=0;
};


void cb_applyForce(const NewtonBody* const body, dFloat timestep, int threadIndex)
{
  // Request the custom data, print the ID, and increment it.
  UserData *mydata = (UserData*)NewtonBodyGetUserData(body);
  printf("BodyID: %d\n", mydata->bodyID++);
}


void addSphereToSimulation(NewtonWorld *world) {
  // fixme: what is this?
  float	foo[16] = {
    1.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 1.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 1.0f, 0.0f,
    0.0f, 0.0f, 0.0f, 1.0f
  };
	
  // Create the sphere. Is this radius or diameter?
  NewtonCollision* const collision = NewtonCreateSphere(world, 1, 0, NULL);

  // fixme: what is this for?
  NewtonBody* const body = NewtonCreateDynamicBody(world, collision, foo);

  // fixme: has no doc string. Is this mass, I_xx, I_yy, and I_zz?
  NewtonBodySetMassMatrix(body, 1.0f, 1, 1, 1);

  // Install callback. Newton will call it whenever the object moves.
  NewtonBodySetForceAndTorqueCallback(body, cb_applyForce);

  // Attach our custom data structure to the body.
  UserData *myData = new UserData;
  myData->bodyID = 5;
  NewtonBodySetUserData(body, (void *)myData);
}


int main (int argc, const char * argv[])
{
  // Create the Newton world.
  NewtonWorld* const world = NewtonCreate();
	
  // Add the sphere.
  addSphereToSimulation(world);

  // Step the (empty) world 60 times in increments of 1/60 second.
  const float timestep = 1.0f / 60;
  for(int i=0; i<5; i++) {
    NewtonUpdate(world, timestep);
  }
	
  // Clean up.
  NewtonDestroyAllBodies(world);
  NewtonDestroy(world);

  return 0;
}
