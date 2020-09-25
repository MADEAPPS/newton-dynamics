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

#ifndef __PHYSICS_UTIL__
#define __PHYSICS_UTIL__

#define DEMO_GRAVITY  dFloat32(-10.0f)
//#define DEMO_GRAVITY  dFloat32(0.0f)

#ifdef DEMO_CHECK_ASYN_UPDATE
extern int g_checkAsyncUpdate;
#endif

#if 0
enum ndPrimitiveType
{
	_NULL_PRIMITIVE,
	_SPHERE_PRIMITIVE,
	_BOX_PRIMITIVE,
	_CAPSULE_PRIMITIVE,
	_CYLINDER_PRIMITIVE,
	_CONE_PRIMITIVE,
	_CHAMFER_CYLINDER_PRIMITIVE,
	_RANDOM_CONVEX_HULL_PRIMITIVE,
	_REGULAR_CONVEX_HULL_PRIMITIVE,
	_COMPOUND_CONVEX_CRUZ_PRIMITIVE,
};


class ndMakeViualMesh : public dScene::dSceneExportCallback
{
	public:
	ndMakeViualMesh(ndWorld* const world);
	NewtonMesh* CreateVisualMesh(ndBodyKinematic* const body, char* const name, int maxNameSize) const;
	ndWorld* m_world;
};

void ExportScene (ndWorld* const world, const char* const fileName);


class ndDemoMesh;
class ndDemoEntity;
class ndDemoEntityManager;

void GetContactOnBody (ndBodyKinematic* const body);
void HandlecollisionPoints (NewtonJoint* const contactjoint);
NewtonJoint* CheckIfBodiesCollide (ndBodyKinematic* const body0, ndBodyKinematic* const body1);

dCustomJoint* FindJoint(const ndBodyKinematic* const body0, const ndBodyKinematic* const body1);


void PhysicsBodyDestructor (const ndBodyKinematic* body);
void PhysicsApplyGravityForce (const ndBodyKinematic* body, dFloat32 timestep, int threadIndex);

void SetAutoSleepMode (ndWorld* const world, int mode);
void CalculateAABB (const NewtonCollision* const collision, const dMatrix& matrix, dVector& minP, dVector& maxP);

void GenericContactProcess (const NewtonJoint* contactJoint, dFloat32 timestep, int threadIndex);

bool GetLastHit (dVector& posit, dVector& normal);
NewtonCollision* CreateCollisionTree (ndWorld* const world, ndDemoEntity* const entity, int materialID, bool optimize);
NewtonCollision* CreateConvexCollision (ndWorld* const world, const dMatrix& offsetMatrix, const dVector& size, ndPrimitiveType type, int materialID);

ndBodyKinematic* CreatePLYMesh (ndDemoEntityManager* const scene, const char* const name, bool optimized);
ndBodyKinematic* CreateLevelMeshBody (ndWorld* const world, ndDemoEntity* const ent, bool optimize);
ndBodyKinematic* CreateSimpleBody(ndWorld* const world, void* const userData, dFloat32 mass, const dMatrix& matrix, NewtonCollision* const collision, int materialId, bool generalInertia = false);
ndBodyKinematic* CreateSimpleSolid (ndDemoEntityManager* const scene, ndDemoMesh* const mesh, dFloat32 mass, const dMatrix& matrix, NewtonCollision* const collision, int materialId, bool generalInertia = false);
void AddPrimitiveArray (ndDemoEntityManager* const scene, dFloat32 mass, const dVector& origin, const dVector& size, int xCount, int zCount, dFloat32 spacing, ndPrimitiveType type, int materialID, const dMatrix& shapeOffsetMatrix, dFloat32 findFloorElevation = 1000.0f, dFloat32 offsetHigh = 5.0f);

ndBodyKinematic* CreateInstancedSolid(ndDemoEntityManager* const scene, ndDemoEntity* const parent, dFloat32 mass, const dMatrix& matrix, NewtonCollision* const collision, int materialId, bool generalInertia = false);

ndBodyKinematic* AddFloorBox(ndDemoEntityManager* const scene, const dVector& origin, const dVector& size);
ndBodyKinematic* CreateLevelMesh (ndDemoEntityManager* const scene, const char* const levelName, bool optimized);


void LoadLumberYardMesh(ndDemoEntityManager* const scene, const dVector& location, int shapeid);
//void SerializationWorld (const char* const name, ndWorld* const world);
#endif

dVector FindFloor(const ndWorld& world, const dVector& origin, dFloat32 dist);
ndBodyKinematic* MousePickBody(ndWorld* const nWorld, const dVector& origin, const dVector& end, dFloat32& paramter, dVector& positionOut, dVector& normalOut);

#endif