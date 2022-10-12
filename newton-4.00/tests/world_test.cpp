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

#include "ndNewton.h"
#include <gtest/gtest.h>

/* Baseline test: create and destroy an empty Newton world. */
TEST(HelloNewton, CreateWorld) {
  ndWorld world;
  world.SetSubSteps(2);
  world.Update(1.0f / 60.0f);
  world.Sync();
}
