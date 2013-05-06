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
#include "../MainFrame.h"
#include "../SceneManager.h"
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


class Magnet: public RenderPrimitive  
{
	public:
	Magnet (NewtonFrame& system, const dMatrix& matrix, dFloat coremass, int coreID, int magneticFieldID, float magnetStregnth)
		:RenderPrimitive () 
	{	
_ASSERTE (0);
/*

		m_magnetStregnth = magnetStregnth;

		dVector size (2.0f, 2.0f, 2.0f, 0.0f);

		// create the magnetic core
		m_magneticCore = CreateGenericSolid (system.m_world, &system, "magnetCore", 30.0f, matrix, size, _SPHERE_PRIMITIVE, coreID);
		AddMesh (((RenderPrimitive *)NewtonBodyGetUserData (m_magneticCore))->GetMesh());
		system.RemoveModel ((OGLModel *)NewtonBodyGetUserData (m_magneticCore));
		NewtonBodySetUserData (m_magneticCore, this);
		system.AddModel___ (this);
		Release();


		// create the magnetic field
		size = size.Scale (10.0f);
		m_magneticField = CreateGenericSolid (system.m_world, &system, "magnetField", coremass, matrix, size, _SPHERE_PRIMITIVE, magneticFieldID);
		system.RemoveModel ((OGLModel *)NewtonBodyGetUserData (m_magneticField));
		NewtonBodySetUserData (m_magneticField, this);


		// set the collision shape as a trigger volume
		NewtonCollision* collision;
		collision = NewtonBodyGetCollision (m_magneticField);
		NewtonCollisionSetAsTriggerVolume (collision, 1);

		// remove destructor callback
		NewtonBodySetDestructorCallback (m_magneticField, NULL);

		// remove the the trigger transform callback
		NewtonBodySetTransformCallback (m_magneticField, NULL);

		// set a force and torque the make sure the body do not move
		NewtonBodySetForceAndTorqueCallback (m_magneticField, MagneticFieldNoForce);
*/
	}


	static void MagneticFieldNoForce (const NewtonBody* body, dFloat timestep, int threadIndex)
	{
		dMatrix matrix;

		dVector zero (0.0f, 0.0f, 0.0f, 0.0f); 
		NewtonBodySetForce (body, &zero[0]);
		NewtonBodySetTorque (body, &zero[0]);
		NewtonBodySetOmega (body, &zero[0]);
		NewtonBodySetVelocity (body, &zero[0]);

		// telepor the magnetic field trigger to the center of the core
		Magnet* magnet;
		magnet = (Magnet*)NewtonBodyGetUserData(body);
		NewtonBodyGetMatrix (magnet->m_magneticCore, &matrix[0][0]);

		dMatrix posit (GetIdentityMatrix());
		posit.m_posit = matrix.m_posit;
		NewtonBodySetMatrix (body, &posit[0][0]);
	}

	static void MagneticField (const NewtonJoint* contactJoint, dFloat timestep, int threadIndex)
	{
		dFloat magnetStregnth;
		const NewtonBody* body0;
		const NewtonBody* body1; 
		const NewtonBody* magneticField;
		const NewtonBody* magneticPiece;

		body0 = NewtonJointGetBody0 (contactJoint);
		body1 = NewtonJointGetBody1 (contactJoint);

		// get the magnetic field body
		magneticPiece = body0;
		magneticField = body1;
		if (NewtonCollisionIsTriggerVolume (NewtonBodyGetCollision(body0))) {
			magneticPiece = body1;
			magneticField = body0;
		}
		_ASSERTE (NewtonCollisionIsTriggerVolume (NewtonBodyGetCollision(magneticField)));

		// calculate the magnetic force field

		dMatrix center;
		dMatrix location;

		NewtonBodyGetMatrix (magneticField, &center[0][0]);
		NewtonBodyGetMatrix (magneticPiece, &location[0][0]);

		Magnet* magnet;
		magnet = (Magnet*)NewtonBodyGetUserData(magneticField);

		magnetStregnth = magnet->m_magnetStregnth;

		// calculate the magnetic force;

		dFloat den;
		dVector force (center.m_posit - location.m_posit);

		den = force % force;
		den = magnetStregnth / (den * dSqrt (den) + 0.1f);
		force = force.Scale (den);

		// because we are modifiing one of the bodies membber in the call back, there uis a chace that 
		// another materail can be operations on the same object at the same time of aother thread
		// therfore we need to make the assigmnet in a critical section.
		NewtonWorldCriticalSectionLock (NewtonBodyGetWorld (magneticPiece));
		
		// add the magner force
		NewtonBodyAddForce (magneticPiece, &force[0]);

		force = force.Scale (-1.0f);
		NewtonBodyAddForce (magnet->m_magneticCore, &force[0]);

		// also if the body is sleeping fore it to wake up for this frame
		NewtonBodySetFreezeState (magneticPiece, 0);
		NewtonBodySetFreezeState (magnet->m_magneticCore, 0);

		// unlock the critical section
		NewtonWorldCriticalSectionUnlock (NewtonBodyGetWorld (magneticPiece));
	}

	dFloat m_magnetStregnth;
	NewtonBody* m_magneticCore;
	NewtonBody* m_magneticField;
};

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
			//			CreateGenericSolid (system->m_world, system, mass, matrix, size, type, materialID);
			CreateSimpleSolid (system->m_world, system, boxMesh, mass, matrix, boxCollision, materialID);
		}
	}

	// do not forget to release the assets	
	boxMesh->Release(); 
	NewtonReleaseCollision(system->m_world, boxCollision);
*/
}




static void Magnets (NewtonFrame& system, dFloat strength)
{
	NewtonWorld* world;
	LevelPrimitive* scene;

	world = system.m_world;

	// create the sky box and the floor,
	scene = LoadLevelAndSceneRoot (system, "playground.dae", 1);

	dVector posit (0.0f, 0.0f, 0.0f, 1.0f);
	posit.m_y = FindFloor (world, posit.m_x, posit.m_z) + 5.0f;

	InitEyePoint (dVector (1.0f, 0.0f, 0.0f), posit);

	// create a material carrier to colliding with this objects
	int defaultMaterialID;
	int magneticFieldID;
	int magneticPicesID;
	defaultMaterialID = NewtonMaterialGetDefaultGroupID (world);
	magneticFieldID = NewtonMaterialCreateGroupID (world);
	magneticPicesID = NewtonMaterialCreateGroupID (world);

	NewtonMaterialSetCollisionCallback (world, magneticPicesID, magneticFieldID, NULL, NULL, Magnet::MagneticField); 
	

	// create a spherical object to serve are the magnet core
	dMatrix matrix (GetIdentityMatrix());
	matrix.m_posit = posit;
	matrix.m_posit.m_x += 7.0f;
	new Magnet (system, matrix, 20.0f, defaultMaterialID, magneticFieldID, strength);

	// add a material to control the buoyancy
	dVector size (1.0f, 0.25f, 0.5f);
	dVector sphSize (1.0f, 1.0f, 1.0f);
	dVector capSize (1.25f, 0.4f, 1.0f);
	dVector location (cameraEyepoint + cameraDir.Scale (10.0f));

	AddBoxes (&system, 1.0f, location, sphSize, 3, 3, 5.0f, _SPHERE_PRIMITIVE, magneticPicesID);
	AddBoxes (&system, 1.0f, location, size, 3, 3, 5.0f, _BOX_PRIMITIVE, magneticPicesID);
	AddBoxes (&system, 1.0f, location, size, 3, 3, 5.0f, _CONE_PRIMITIVE, magneticPicesID);
	AddBoxes (&system, 1.0f, location, size, 3, 3, 5.0f, _CYLINDER_PRIMITIVE, magneticPicesID);
	AddBoxes (&system, 1.0f, location, capSize, 3, 3, 5.0f, _CAPSULE_PRIMITIVE, magneticPicesID);
	AddBoxes (&system, 1.0f, location, size, 3, 3, 5.0f, _CHAMFER_CYLINDER_PRIMITIVE, magneticPicesID);
	AddBoxes (&system, 1.0f, location, size, 3, 3, 5.0f, _RANDOM_CONVEX_HULL_PRIMITIVE, magneticPicesID);
	AddBoxes (&system, 1.0f, location, size, 3, 3, 5.0f, _REGULAR_CONVEX_HULL_PRIMITIVE, magneticPicesID);

	posit.m_x -= 10.0f;
	InitEyePoint (dVector (1.0f, 0.0f, 0.0f), posit);
}

void Magnets (NewtonFrame& system)
{
	Magnets (system, 200.0f);
}

void Repulsive (NewtonFrame& system)
{
	Magnets (system, -200.0f);
}

