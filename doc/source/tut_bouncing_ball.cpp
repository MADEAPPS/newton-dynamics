#include <stdio.h>
#include "Newton.h"


// Define a custom data structure to store a body ID.
struct UserData {
  int bodyID=0;
};


void cb_applyForce(const NewtonBody* const body, dFloat timestep, int threadIndex)
{
  // Fetch user data and body position.
  UserData *mydata = (UserData*)NewtonBodyGetUserData(body);
  dFloat pos[4];
  NewtonBodyGetPosition(body, pos);

  // Apply force.
  dFloat force[3] = {0, -1.0, 0};
  NewtonBodySetForce(body, force);

  // Print info to terminal.
  printf("BodyID=%d, Sleep=%d, %.2f, %.2f, %.2f\n",
         mydata->bodyID, NewtonBodyGetSleepState(body), pos[0], pos[1], pos[2]);
}


void addBodies(NewtonWorld *world) {
  // Neutral transform matrix.
  float	tm[16] = {
    1.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 1.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 1.0f, 0.0f,
    0.0f, 0.0f, 0.0f, 1.0f
  };

  // Collision shapes: sphere (our ball), and large box (our ground plane).
  NewtonCollision* const cs_sphere = NewtonCreateSphere(world, 1, 0, NULL);
  NewtonCollision* const cs_ground = NewtonCreateBox(world, 100, 0.1, 100, 0, NULL);

  // Create the bodies and assign them the collision shapes. Note that
  // we need to modify initial transform for the ball to place it a y=2.0
  NewtonBody* const ground = NewtonCreateDynamicBody(world, cs_ground, tm);
  tm[13] = 2.0;
  NewtonBody* const sphere = NewtonCreateDynamicBody(world, cs_sphere, tm);

  // Bodies with zero mass are static.
  NewtonBodySetMassMatrix(ground, 0.0f, 0, 0, 0);

  // Bodies with non-zero mass are dynamic.
  NewtonBodySetMassMatrix(sphere, 1.0f, 1, 1, 1);

  // Install the callbacks to track the body positions.
  NewtonBodySetForceAndTorqueCallback(sphere, cb_applyForce);
  NewtonBodySetForceAndTorqueCallback(ground, cb_applyForce);

  // Attach our custom data structure to the bodies.
  UserData *myData = new UserData[2];
  myData[0].bodyID = 0;
  myData[1].bodyID = 1;
  NewtonBodySetUserData(sphere, (void *)&myData[0]);
  NewtonBodySetUserData(ground, (void *)&myData[1]);
}


int main (int argc, const char * argv[])
{
  // Create the Newton world.
  NewtonWorld* const world = NewtonCreate();

  // Add the bodies.
  addBodies(world);

  // Step the simulation.
  const float timestep = 1.0f / 60;
  for(int i=0; i<300; i++) {
    NewtonUpdate(world, timestep);
  }
	
  // Clean up.
  NewtonDestroyAllBodies(world);
  NewtonDestroy(world);

  return 0;
}
