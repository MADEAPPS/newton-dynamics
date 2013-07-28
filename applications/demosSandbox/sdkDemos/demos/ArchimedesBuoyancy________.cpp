/* Copyright (c) <2009> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/

#include <toolbox_stdafx.h>
#include "SkyBox.h"
#include "RenderPrimitive.h"
#include "../OGLMesh.h"
#include "../SceneManager.h"
#include "../MainFrame.h"
#include "../PhysicsUtils.h"
#include "../toolBox/MousePick.h"
#include "../toolBox/OpenGlUtil.h"
#include "../toolBox/DebugDisplay.h"
#include "../toolBox/LevelPrimitive.h"
#include "../toolBox/PlaneCollision.h"
#include "../toolBox/HeightFieldPrimitive.h"
#include "../toolBox/UserHeightFieldCollision.h"


static void SetDemoCallbacks (NewtonFrame& system)
{
	system.m_control = Keyboard;
	system.m_autoSleep = AutoSleep;
	system.m_showIslands = SetShowIslands;
	system.m_showContacts = SetShowContacts; 
	system.m_setMeshCollision = SetShowMeshCollision;
}

static LevelPrimitive* LoadLevelAndSceneRoot (NewtonFrame& system, const char* levelName, int optimized)
{
_ASSERTE (0);
return NULL;
/*
	NewtonWorld* world;
	NewtonBody* floorBody;
	LevelPrimitive* level;

	world = system.m_world;
	// /////////////////////////////////////////////////////////////////////
	//
	// create the sky box,
	OGLModel* sky = new SkyBox ();
	system.AddModel___ (sky);
	sky->Release();


	// Load a level geometry
	level = new LevelPrimitive (levelName, world, optimized);
	system.AddModel___(level);
	level->Release();
	floorBody = level->m_level;

	// set a destructor for this rigid body
	NewtonBodySetDestructorCallback (floorBody, PhysicsBodyDestructor);

	// get the default material ID
	int defaultID;
	defaultID = NewtonMaterialGetDefaultGroupID (world);

	// set default material properties
	NewtonMaterialSetDefaultSoftness (world, defaultID, defaultID, 0.05f);
	NewtonMaterialSetDefaultElasticity (world, defaultID, defaultID, 0.4f);
	NewtonMaterialSetDefaultCollidable (world, defaultID, defaultID, 1);
	NewtonMaterialSetDefaultFriction (world, defaultID, defaultID, 1.0f, 0.5f);
	NewtonMaterialSetCollisionCallback (world, defaultID, defaultID, NULL, NULL, GenericContactProcess); 

//	NewtonMaterialSetSurfaceThickness(world, materialID, materialID, 0.1f);
//	NewtonMaterialSetSurfaceThickness(world, defaultID, defaultID, 0.0f);

	// set the island update callback
	NewtonSetIslandUpdateEvent (world, PhysicsIslandUpdate);

	// save th3 callback
	SetDemoCallbacks (system);

	InitEyePoint (dVector (1.0f, 0.0f, 0.0f), dVector (-40.0f, 10.0f, 0.0f));

	return level;
*/
}



#define USE_AABB_VOLUME_TEST


static int PhysicsBouyancyPlane (const int collisionID, void *context, const dFloat* matrix, dFloat* globalSpacePlane)
{
	const NewtonBody* trigger;
	const dMatrix& globalSpaceMatrix =  *(dMatrix*)matrix;

	trigger = (const NewtonBody*) context;

	// find the fluid surface by casting a ray on the trigger volume in the direction of the gravity

	dVector p0 (globalSpaceMatrix.m_posit);
	dVector p1 (globalSpaceMatrix.m_posit);

	p0.m_y += 100.0f;
	p1.m_y -= 100.0f;

	// get the matrix of the trigger volume
	dMatrix volumeMatrix;
	NewtonBodyGetMatrix(trigger, &volumeMatrix[0][0]);

	// transform the ray to the local space of the trigger volume
	dVector q0 (volumeMatrix.UntransformVector(p0));
	dVector q1 (volumeMatrix.UntransformVector(p1));

	// cast the ray of the collision shape of the trigger volume
	int attribute;
	dFloat param;
	dVector normal;
	param = NewtonCollisionRayCast (NewtonBodyGetCollision(trigger), &q0[0], &q1[0], &normal[0], &attribute);

	_ASSERTE (param >= 0.0f);
	_ASSERTE (param <= 1.0f);

	// now rotate the ray normal  and the intersection point to global space
	normal = volumeMatrix.RotateVector(normal);
	dVector point (p0 + (p1 - p0).Scale(param));

	// return the plane equation
	globalSpacePlane[0] = normal.m_x;
	globalSpacePlane[1] = normal.m_y;
	globalSpacePlane[2] = normal.m_z;
	globalSpacePlane[3] = -(normal % point);
	return 1;	
}

#ifdef USE_AABB_VOLUME_TEST
// this faster but no very acurate since it reports overlap at the aabb level
static int ApplyBuoyancyAABBLevel (const NewtonMaterial* material, const NewtonBody* body0, const NewtonBody* body1, int threadIndex)
{
	const NewtonBody* body;
	const NewtonBody* trigger;
	RenderPrimitive* primitive;

//	_ASSERTE (NewtonBodyGetUserData(body0));
//	_ASSERTE (NewtonBodyGetUserData(body1));

	// get the body with the trigger volume
	if (NewtonCollisionIsTriggerVolume (NewtonBodyGetCollision(body0))) {
		body = body1;
		trigger = body0;
	} else {
		body = body0;
		trigger = body1;
	}

	_ASSERTE (NewtonCollisionIsTriggerVolume (NewtonBodyGetCollision(trigger)));
	_ASSERTE (!NewtonCollisionIsTriggerVolume (NewtonBodyGetCollision(body)));

	dVector gravity (0.0f, -10.0f, 0.0f, 0.0f);

	primitive = (RenderPrimitive*) NewtonBodyGetUserData(body);

//	dFloat density; 
//	dFloat mass;
//	dFloat dommy;
//	dFloat volume;
//	NewtonBodyGetMassMatrix(body, &mass, &dommy, &dommy, &dommy);
//	volume = NewtonConvexCollisionCalculateVolume(NewtonBodyGetCollision(body));
//	density = 1.5f * mass / volume;

	NewtonBodyAddBuoyancyForce (body, primitive->m_density, 0.8f, 0.8f, &gravity[0], PhysicsBouyancyPlane, (void*) trigger);
	return 1;
}

#else 
// this is more accurate since it reports overlap at the shape level but do not calculate contacts
void ApplyBuoyancyContactLevel (const NewtonJoint* contactJoint, dFloat timestep, int threadIndex)
{
	const NewtonBody* body;
	const NewtonBody* trigger;
	RenderPrimitive* primitive;
	const NewtonBody* body0;
	const NewtonBody* body1;

	body0 = NewtonJointGetBody0 (contactJoint);
	body1 = NewtonJointGetBody1 (contactJoint);

	// get the body with the trigger volume
	if (NewtonCollisionIsTriggerVolume (NewtonBodyGetCollision(body0))) {
		body = body1;
		trigger = body0;
	} else {
		body = body0;
		trigger = body1;
	}

	_ASSERTE (NewtonCollisionIsTriggerVolume (NewtonBodyGetCollision(trigger)));
	_ASSERTE (!NewtonCollisionIsTriggerVolume (NewtonBodyGetCollision(body)));

	dVector gravity (0.0f, -10.0f, 0.0f, 0.0f);

	primitive = (RenderPrimitive*) NewtonBodyGetUserData(body);

	NewtonBodyAddBuoyancyForce (body, primitive->m_density, 0.8f, 0.8f, &gravity[0], PhysicsBouyancyPlane, (void*) trigger);
}
#endif


static void CreateSwimmingPool (NewtonFrame& system, int bouyancyForceMaterialID)
{
	NewtonBody* body;

	dVector poolSize (46.0f, 10.0f, 30.0f, 0.0f);
	dMatrix poolLocation (GetIdentityMatrix());
	poolLocation.m_posit.m_x = 27.0f;
	poolLocation.m_posit.m_y = -6.0f;
	body = CreateGenericSolid (system.m_world, &system, "pool", 0.0f, poolLocation, poolSize, _BOX_PRIMITIVE, bouyancyForceMaterialID);

	// destroy the visual model
	system.RemoveModel ((OGLModel*) NewtonBodyGetUserData (body));

	// set the user data to NULL
	NewtonBodySetUserData (body, NULL);

	// set the collision shape as trigger volume
	NewtonCollision* collision;

	collision = NewtonBodyGetCollision(body);
	NewtonCollisionSetAsTriggerVolume(collision, 1);
}


static void AddBoxes(SceneManager* system, dFloat mass, const dVector& origin, const dVector& oriSize, int xCount, int zCount, dFloat spacing, PrimitiveType type, int materialID)
{
_ASSERTE (0);
/*
	dVector size (oriSize);
	dMatrix matrix (GetIdentityMatrix());

	// create the shape and visual mesh as a common data to be re used
	//	type = _BOX_PRIMITIVE;
	NewtonCollision* boxCollision = CreateConvexCollision (system->m_world, GetIdentityMatrix(), size, type, materialID);

	char name[256];
	sprintf (name, "shape%d", type);
	OGLMesh* boxMesh = new OGLMesh (name, boxCollision, "wood_0.tga", "wood_0.tga", "wood_1.tga");

	for (int i = 0; i < xCount; i ++) {
		dFloat x;
		x = origin.m_x + (i - xCount / 2) * spacing;
		for (int j = 0; j < zCount; j ++) {
			dFloat z;
			z = origin.m_z + (j - zCount / 2) * spacing;

			matrix.m_posit.m_x = x;
			matrix.m_posit.m_y = FindFloor (system->m_world, x, z) + 4.0f;
			matrix.m_posit.m_z = z;
			CreateSimpleSolid (system->m_world, system, boxMesh, mass, matrix, boxCollision, materialID);
		}
	}

	// do not forget to release the assets	
	boxMesh->Release(); 
	NewtonReleaseCollision(system->m_world, boxCollision);
*/
}


void ArchimedesBuoyancy (NewtonFrame& system)
{
	NewtonWorld* world;
	LevelPrimitive *level;

	world = system.m_world;

	// create the sky box and the floor,
	level = LoadLevelAndSceneRoot (system, "pitpool.dae", 1);

	dVector posit (0.0f, 0.0f, 0.0f, 0.0f);
	posit.m_y = FindFloor (world, posit.m_x, posit.m_z) + 5.0f;

	InitEyePoint (dVector (1.0f, 0.0f, 0.0f), posit);


	// add a material to control the buoyancy
	int defaultMaterialID;
	int bouyancyForceMaterialID;
	defaultMaterialID = NewtonMaterialGetDefaultGroupID (world);
	bouyancyForceMaterialID = NewtonMaterialCreateGroupID (world);

	// set this material to this trigger volume
#ifdef USE_AABB_VOLUME_TEST
	// this is faster but not very accurate since it report overlap at the aabb level
	NewtonMaterialSetCollisionCallback (world, bouyancyForceMaterialID, defaultMaterialID, NULL, ApplyBuoyancyAABBLevel, NULL); 
#else
	// this is more accurate since it reports overlap at the shape level but do not calculate contacts
	NewtonMaterialSetCollisionCallback (world, bouyancyForceMaterialID, defaultMaterialID, NULL, NULL, ApplyBuoyancyContactLevel); 
#endif


	// Add a large body with a box shape to be use as buoyancy trigger
	CreateSwimmingPool (system, bouyancyForceMaterialID);

	dVector size (1.0f, 0.25f, 0.5f);
	dVector location (cameraEyepoint + cameraDir.Scale (10.0f));

	AddBoxes (& system, 10.0f, location, size, 3, 3, 10.0f, _SPHERE_PRIMITIVE, defaultMaterialID);
	AddBoxes (& system, 10.0f, location, size, 3, 3, 10.0f, _BOX_PRIMITIVE, defaultMaterialID);
	AddBoxes (& system, 10.0f, location, size, 3, 3, 10.0f, _CONE_PRIMITIVE, defaultMaterialID);
	AddBoxes (& system, 10.0f, location, size, 3, 3, 10.0f, _CYLINDER_PRIMITIVE, defaultMaterialID);
	AddBoxes (& system, 10.0f, location, size, 3, 3, 10.0f, _CAPSULE_PRIMITIVE, defaultMaterialID);
	AddBoxes (& system, 10.0f, location, size, 3, 3, 10.0f, _CHAMFER_CYLINDER_PRIMITIVE, defaultMaterialID);
	AddBoxes (& system, 10.0f, location, size, 3, 3, 10.0f, _RANDOM_CONVEX_HULL_PRIMITIVE, defaultMaterialID);
	AddBoxes (& system, 10.0f, location, size, 3, 3, 10.0f, _REGULAR_CONVEX_HULL_PRIMITIVE, defaultMaterialID);
}




