/* Copyright (c) <2003-2021> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/

#ifndef __PHYSICS_UTIL__
#define __PHYSICS_UTIL__

//#define DEMO_GRAVITY  ndFloat32(-10.0f)
#define DEMO_GRAVITY  ndFloat32(-0.5f)

class ndDemoEntityManager;

ndVector FindFloor(const ndWorld& world, const ndVector& origin, ndFloat32 dist);
ndBodyKinematic* MousePickBody(ndWorld* const nWorld, const ndVector& origin, const ndVector& end, ndFloat32& paramter, ndVector& positionOut, ndVector& normalOut);

ndBodyKinematic* CreateBody(ndDemoEntityManager* const scene, const ndShapeInstance& shape, const ndMatrix& origin, ndFloat32 mass);

ndBodyKinematic* AddSphere(ndDemoEntityManager* const scene, const ndVector& origin, ndFloat32 mass, ndFloat32 radius);
ndBodyKinematic* AddBox(ndDemoEntityManager* const scene, const ndVector& origin, ndFloat32 mass, ndFloat32 sizex, ndFloat32 sizey, ndFloat32 sizez);
ndBodyKinematic* AddCapsule(ndDemoEntityManager* const scene, const ndVector& origin, ndFloat32 mass, ndFloat32 radius0, ndFloat32 radius1, ndFloat32 high);
ndBodyKinematic* AddConvexHull(ndDemoEntityManager* const scene, const ndVector& origin, ndFloat32 mass, ndFloat32 radius, ndFloat32 high, ndInt32 segments);

void AddPlanks(ndDemoEntityManager* const scene, const ndVector& origin, ndFloat32 mass, ndInt32 count);
void AddCapsulesStacks(ndDemoEntityManager* const scene, const ndVector& origin, ndFloat32 mass, ndFloat32 radius0, ndFloat32 radius1, ndFloat32 high, ndInt32 rows_x, ndInt32 rows_z, ndInt32 columHigh);

#endif