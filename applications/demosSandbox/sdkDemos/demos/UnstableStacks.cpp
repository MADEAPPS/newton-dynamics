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


static void SetDemoCallbacks (NewtonFrame& system)
{
	system.m_control = Keyboard;
	system.m_autoSleep = AutoSleep;
	system.m_showIslands = SetShowIslands;
	system.m_showContacts = SetShowContacts; 
	system.m_setMeshCollision = SetShowMeshCollision;
}

static void BuildFloorAndSceneRoot (NewtonFrame& system)
{
_ASSERTE (0);
/*

	NewtonWorld* world;
	RenderPrimitive* floor;
	NewtonBody* floorBody;
	NewtonCollision* floorCollision;
	OGLMesh* meshInstance;

	world = system.m_world;
	// /////////////////////////////////////////////////////////////////////
	//
	// create the sky box,
	OGLModel* sky = new SkyBox ();
	system.AddModel___ (sky);
	sky->Release();


	// create the the floor graphic objects
	dVector floorSize (100.0f, 2.0f, 100.0f);
	dMatrix location (GetIdentityMatrix());
	location.m_posit.m_y = -5.0f; 

	// create a box for floor 
	floorCollision = NewtonCreateBox (world, floorSize.m_x, floorSize.m_y, floorSize.m_z, 0, NULL); 

	//	meshInstance = OGLMesh::MakeBox (world, size.m_x, size.m_y, size.m_z, "GrassAndDirt.tga");
	meshInstance = new OGLMesh ("floor", floorCollision, "GrassAndDirt.tga", "metal_30.tga", "metal_30.tga");
	floor = new RenderPrimitive (location, meshInstance);
	system.AddModel___ (floor);
	floor->Release();
	meshInstance->Release();

	// create the the floor collision, and body with default values
	floorBody = NewtonCreateBody (world, floorCollision);
	NewtonReleaseCollision (world, floorCollision);


	// set the transformation for this rigid body
	NewtonBodySetMatrix (floorBody, &location[0][0]);

	// save the pointer to the graphic object with the body.
	NewtonBodySetUserData (floorBody, floor);

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
	NewtonMaterialSetSurfaceThickness(world, defaultID, defaultID, 0.0f);

	// set the island update callback
	NewtonSetIslandUpdateEvent (world, PhysicsIslandUpdate);

	// save the callback
	SetDemoCallbacks (system);

	InitEyePoint (dVector (1.0f, 0.0f, 0.0f), dVector (-40.0f, 10.0f, 0.0f));
*/
}

static PrimitiveType GetRandType()
{
	//return _SPHERE_PRIMITIVE;
	//return _BOX_PRIMITIVE;
	//return _CONE_PRIMITIVE;
	//return _CYLINDER_PRIMITIVE;
	//return _CAPSULE_PRIMITIVE;
	//return _CHAMFER_CYLINDER_PRIMITIVE;
	//return _RANDOM_CONVEX_HULL_PRIMITIVE;			  

	int index;
	PrimitiveType probability[] = {
		_BOX_PRIMITIVE, 
		_CYLINDER_PRIMITIVE,
		_RANDOM_CONVEX_HULL_PRIMITIVE,
		_REGULAR_CONVEX_HULL_PRIMITIVE,
	};
	index = dRand()>>8;
	index %= (sizeof (probability) / sizeof (int));
	return probability[index];
}


static void UnstableStacks (NewtonFrame& system, dVector location, int high)
{
	dFloat columnMass;

	// /////////////////////////////////////////////////////////////////////
	//
	// Build a parking lot type structure
	dVector columnBoxSize (3.0f, 1.0f, 1.0f);

	// create the stack
	dMatrix baseMatrix (GetIdentityMatrix());

	// get the floor position in from o the camera
	baseMatrix.m_posit = location;
	baseMatrix.m_posit.m_y += columnBoxSize.m_x;

	// set realistic (extremes in this case for 24 bits precision) masses for the different components
	// note how the newton engine handle different masses ratios without compromising stability, 
	// we recommend the application keep this ration under 100 for contacts and 50 for joints 
	columnMass = 1.0f;

	// create a material carrier to collision with this object
	int defaultMaterialID;
	defaultMaterialID = NewtonMaterialGetDefaultGroupID (system.m_world);

	dMatrix columAlignment (dRollMatrix(3.141592f * 0.5f));
	for (int i = 0; i < high; i ++) { 
		NewtonBody* body;

		dMatrix matrix(columAlignment * baseMatrix);
		char meshName[256];
		sprintf (meshName, "shape%d", rand());
		body = CreateGenericSolid (system.m_world, &system, meshName, columnMass, matrix, columnBoxSize, GetRandType(), defaultMaterialID);
		ConvexCastPlacement (body);

		// set up for another level
		baseMatrix.m_posit.m_y += columnBoxSize.m_x;
	}
}


static void UnstableStruture (NewtonFrame& system, dVector location, int high)
{
	int i;
	dFloat plankMass;
	dFloat columnMass;

	// /////////////////////////////////////////////////////////////////////
	//
	// Build a parking lot type structure

	dVector columnBoxSize (3.0f, 1.0f, 1.0f);
	//	dVector columnBoxSize (3.0f, 3.0f, 1.0f);
	dVector plankBoxSize (6.0f, 1.0f, 6.0f);

	// create the stack
	dMatrix baseMatrix (GetIdentityMatrix());

	// get the floor position in from o the camera
	baseMatrix.m_posit = location;
	baseMatrix.m_posit.m_y += columnBoxSize.m_x;

	// set realistic (extremes in this case for 24 bits precision) masses for the different components
	// note how the newton engine handle different masses ratios without compromising stability, 
	// we recommend the application keep this ration under 100 for contacts and 50 for joints 
	columnMass = 1.0f;
	plankMass = 20.0f;
	//	 plankMass = 1.0f;

	// create a material to collision with this object
	int defaultMaterialID;
	defaultMaterialID = NewtonMaterialGetDefaultGroupID (system.m_world);

	dMatrix columAlignment (dRollMatrix(3.141592f * 0.5f));
	for (i = 0; i < high; i ++) { 
		char meshName[256];
		NewtonBody* body;

		dMatrix matrix(columAlignment * baseMatrix);


		// add the 4 column
		matrix.m_posit.m_x -=  (columnBoxSize.m_z - plankBoxSize.m_x) * 0.5f;
		matrix.m_posit.m_z -=  (columnBoxSize.m_z - plankBoxSize.m_z) * 0.5f;
		sprintf (meshName, "shape%d", rand());
		body = CreateGenericSolid (system.m_world, &system, meshName, columnMass, matrix, columnBoxSize, GetRandType(), defaultMaterialID);
		ConvexCastPlacement (body);

		matrix.m_posit.m_x += columnBoxSize.m_z - plankBoxSize.m_x;
		sprintf (meshName, "shape%d", rand());
		body = CreateGenericSolid (system.m_world, &system, meshName, columnMass, matrix, columnBoxSize, GetRandType(), defaultMaterialID);
		ConvexCastPlacement (body);

		matrix.m_posit.m_z += columnBoxSize.m_z - plankBoxSize.m_z;		
		sprintf (meshName, "shape%d", rand());
		body = CreateGenericSolid (system.m_world, &system, meshName, columnMass, matrix, columnBoxSize, GetRandType(), defaultMaterialID);
		ConvexCastPlacement (body);

		matrix.m_posit.m_x -= columnBoxSize.m_z - plankBoxSize.m_x;
		sprintf (meshName, "shape%d", rand());
		body = CreateGenericSolid (system.m_world, &system, meshName, columnMass, matrix, columnBoxSize, GetRandType(), defaultMaterialID);
		ConvexCastPlacement (body);

		// add a plank
		dVector size (plankBoxSize);
		size.m_x *= 0.85f;
		size.m_z *= 0.85f;
		body = CreateGenericSolid (system.m_world, &system, "box", plankMass, baseMatrix, size, _BOX_PRIMITIVE, defaultMaterialID);
		ConvexCastPlacement (body);

		// set up for another level
		baseMatrix.m_posit.m_y += (columnBoxSize.m_x + plankBoxSize.m_y);
	}
}



void UnstableStacks (NewtonFrame& system)
{
	NewtonWorld* world;
	world = system.m_world;

	// create the sky box and the floor,
	BuildFloorAndSceneRoot (system);

	UnstableStacks (system, dVector (-10.0f, 0.0f, -10.0f, 0.0f), 10);
	UnstableStacks (system, dVector ( 10.0f, 0.0f, -10.0f, 0.0f), 10);
	UnstableStacks (system, dVector (-10.0f, 0.0f,  10.0f, 0.0f), 10);
	UnstableStacks (system, dVector ( 10.0f, 0.0f,  10.0f, 0.0f), 10);
}



void UnstableStruture (NewtonFrame& system)
{
	NewtonWorld* world;
	world = system.m_world;

	// create the sky box and the floor,
	BuildFloorAndSceneRoot (system);

	UnstableStruture (system, dVector (-10.0f, 0.0f, -10.0f, 0.0f), 10);
	UnstableStruture (system, dVector ( 10.0f, 0.0f, -10.0f, 0.0f), 10);
	UnstableStruture (system, dVector (-10.0f, 0.0f,  10.0f, 0.0f), 10);
	UnstableStruture (system, dVector ( 10.0f, 0.0f,  10.0f, 0.0f), 10);
}






