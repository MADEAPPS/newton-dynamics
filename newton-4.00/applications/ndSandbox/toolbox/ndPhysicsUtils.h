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

//#define DEMO_GRAVITY  dFloat32(-10.0f)
#define DEMO_GRAVITY  dFloat32(0.0f)

class ndDemoEntityManager;

dVector FindFloor(const ndWorld& world, const dVector& origin, dFloat32 dist);
ndBodyKinematic* MousePickBody(ndWorld* const nWorld, const dVector& origin, const dVector& end, dFloat32& paramter, dVector& positionOut, dVector& normalOut);

ndBodyKinematic* CreateBody(ndDemoEntityManager* const scene, const ndShapeInstance& shape, const dMatrix& origin, dFloat32 mass);

ndBodyKinematic* AddSphere(ndDemoEntityManager* const scene, const dVector& origin, dFloat32 mass, dFloat32 radius);
ndBodyKinematic* AddBox(ndDemoEntityManager* const scene, const dVector& origin, dFloat32 mass, dFloat32 sizex, dFloat32 sizey, dFloat32 sizez);
ndBodyKinematic* AddCapsule(ndDemoEntityManager* const scene, const dVector& origin, dFloat32 mass, dFloat32 radius0, dFloat32 radius1, dFloat32 high);
ndBodyKinematic* AddConvexHull(ndDemoEntityManager* const scene, const dVector& origin, dFloat32 mass, dFloat32 radius, dFloat32 high, dInt32 segments);

void AddPlanks(ndDemoEntityManager* const scene, const dVector& origin, dFloat32 mass, dInt32 count);
void AddCapsulesStacks(ndDemoEntityManager* const scene, const dVector& origin, dFloat32 mass, dFloat32 radius0, dFloat32 radius1, dFloat32 high, dInt32 rows_x, dInt32 rows_z, dInt32 columHigh);

#endif