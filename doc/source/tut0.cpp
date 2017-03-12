#include <stdio.h>
#include "Newton.h"


int main (int argc, const char * argv[])
{
  // Print the library version.
  printf("Hello, this is Newton version %d\n", NewtonWorldGetVersion());

  // Create the Newton world.
  NewtonWorld* const world = NewtonCreate();

  // Step the (empty) world 60 times in increments of 1/60 second.
  const float timestep = 1.0f / 60;
  for(int i=0; i<60; i++) {
    NewtonUpdate(world, timestep);
  }
	
  // Clean up.
  NewtonDestroyAllBodies(world);
  NewtonDestroy(world);

  return 0;
}
