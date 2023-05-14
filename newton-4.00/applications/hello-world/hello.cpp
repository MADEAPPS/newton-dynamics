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

/* Create a Newton world, step it a few times and print how long it took. */
int main(int, const char**)
{
  // Number of steps to simulate.
  const int numSteps = 100000;

  // Create the world.
  ndWorld world;

  // Two sub-steps per time step is the recommended default. It should provide
  // stable simulations for most standard scenarios including those with joints.
  world.SetSubSteps(2);

  // Step the world and measure how long it takes.
  for (ndInt32 i = 0; i < numSteps; i++)
  {
    // Trigger an asynchronous(!) physics step.
    // NOTE: this function will return immediately.
    world.Update(1.0f / 60.0f);

    /* It is generally a bad idea to access Newton objects at this point since
       it may not have completed the update yet. However, we could do other
       application logic here if we wanted to. */

    // Explicitly wait until Newton has finished its physics step.
    world.Sync();

    /* It is now safe to access Newton objects, add bodies, modify existing ones etc. */
  }

  // Pretty print the results.
  setlocale(LC_NUMERIC, "");
  printf("Ran %'d Newton steps\n", numSteps);
  return 0;
}
