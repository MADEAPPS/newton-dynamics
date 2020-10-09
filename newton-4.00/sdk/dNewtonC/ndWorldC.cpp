/* Copyright (c) <2003-2019> <Julio Jerez, Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 
* 3. This notice may not be removed or altered from any source distribution.
*/

#include <ndNewton.h>
#include "ndWorldC.h"

ndWorldC ndCreateWorld()
{
	return (ndWorldC) new ndWorld();
}

void ndDestroyWorld(ndWorldC worldc)
{
	ndWorld* const world = (ndWorld*)worldc;
	delete world;
}

void ndWorldSync(ndWorldC worldc)
{
	ndWorld* const world = (ndWorld*)worldc;
	world->Sync();
}

void ndWorldSetSubSteps(ndWorldC worldc, int subSteps)
{
	ndWorld* const world = (ndWorld*)worldc;
	world->SetSubSteps(subSteps);
}

void ndWorldSetThreadCount(ndWorldC worldc, int workerThreads)
{
	ndWorld* const world = (ndWorld*)worldc;
	world->SetThreadCount(workerThreads);
}

void ndWorldAddBody(ndWorldC worldc, ndBodyDynamicC bodyc)
{
	ndWorld* const world = (ndWorld*)worldc;
	ndBodyDynamic* const body = (ndBodyDynamic*)bodyc;
	world->AddBody(body);
}

dFloat32 ndWorldGetUpdateTime(ndWorldC worldc)
{
	ndWorld* const world = (ndWorld*)worldc;
	return world->GetUpdateTime();
}

void ndWorldRayCast(ndWorldC worldc)
{
	ndWorld* const world = (ndWorld*)worldc;

}

void ndWorldUpdate(ndWorldC worldc, dFloat32 timestep)
{
	ndWorld* const world = (ndWorld*)worldc;
	world->Update(timestep);
}