//
//  Use this file to import your target's public headers that you would like to expose to Swift.
//

#include "SimpleCxxFunctions.h"
#include <ndNewton.h>
//#include <ndWorld.h>

ndWorld* CreateWorld()
{
    return new ndWorld();
}


void DestroyWorld(ndWorld* const world)
{
    delete world;
}
