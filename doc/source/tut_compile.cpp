#include <stdio.h>
#include "Newton.h"


int main (int argc, const char * argv[]) {
  // Print the library version.
  printf("Hello, this is Newton version %d\n", NewtonWorldGetVersion());

  // Create the Newton world.
  NewtonWorld* const world = NewtonCreate();

  // Step the (empty) world.
  const float timestep = 1.0f / 60;
  NewtonUpdate(world, timestep);

  // Clean up.
  NewtonDestroyAllBodies(world);
  NewtonDestroy(world);

  return 0;
}
