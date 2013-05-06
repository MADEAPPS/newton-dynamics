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
#include "TargaToOpenGl.h"
#include "RenderPrimitive.h"

#include "../OGLMesh.h"
#include "../SceneManager.h"
#include "../PhysicsUtils.h"
#include "../toolBox/MousePick.h"
#include "../toolBox/OpenGlUtil.h"
#include "../toolBox/DebugDisplay.h"

#define DEBRI_ID  1	

// this is the clipper Mesh
static NewtonMesh* meshClipper;

// destroy the clipper Mesh
static void DestroyWorldCallback(const NewtonWorld* newtonWorld)
{
	NewtonMeshDestroy(meshClipper);
}


static void CreateDebriPiece (const NewtonBody* sourceBody, NewtonMesh* mesh, dFloat volume)
{
	dFloat Ixx;
	dFloat Iyy;
	dFloat Izz;
	dFloat mass;
	dFloat shapeVolume;
	NewtonWorld* world;
	NewtonBody* rigidBody;
	NewtonCollision* collision;
	OGLMesh* meshInstance;
	SceneManager* system;
	RenderPrimitive* primitive;
	dVector inertia;
	dVector origin;
	dVector veloc;
	dVector omega;
	dMatrix matrix;

	world = NewtonBodyGetWorld (sourceBody);

	NewtonBodyGetMatrix (sourceBody, &matrix[0][0]);

	NewtonBodyGetMassMatrix (sourceBody, &mass, &Ixx, &Iyy, &Izz);

	// make a visual object
	meshInstance = new OGLMesh();

	meshInstance->BuildFromMesh (mesh);

	// create a visual geometry
	primitive = new RenderPrimitive (matrix, meshInstance);
	meshInstance->Release();

	// save the graphics system
	system = (SceneManager*) NewtonWorldGetUserData(world);
	system->AddModel (primitive);

	collision = NewtonCreateConvexHullFromMesh (world, mesh, 0.1f, DEBRI_ID);

	// calculate the moment of inertia and the relative center of mass of the solid
	shapeVolume = NewtonConvexCollisionCalculateVolume (collision);
	NewtonConvexCollisionCalculateInertialMatrix (collision, &inertia[0], &origin[0]);	

	mass = mass * shapeVolume / volume;
	Ixx = mass * inertia[0];
	Iyy = mass * inertia[1];
	Izz = mass * inertia[2];

	//create the rigid body
	rigidBody = NewtonCreateBody (world, collision);

	// set the correct center of gravity for this body
	NewtonBodySetCentreOfMass (rigidBody, &origin[0]);

	// set the mass matrix
	NewtonBodySetMassMatrix (rigidBody, mass, Ixx, Iyy, Izz);

	// save the pointer to the graphic object with the body.
	NewtonBodySetUserData (rigidBody, primitive);

	// assign the wood id
//	NewtonBodySetMaterialGroupID (rigidBody, NewtonBodyGetMaterialGroupID(source));

	// set continue collision mode
	NewtonBodySetContinuousCollisionMode (rigidBody, 1);

	// set a destructor for this rigid body
	NewtonBodySetDestructorCallback (rigidBody, PhysicsBodyDestructor);

	// set the transform call back function
	NewtonBodySetTransformCallback (rigidBody, PhysicsSetTransform);

	// set the force and torque call back function
	NewtonBodySetForceAndTorqueCallback (rigidBody, PhysicsApplyGravityForce);

	// set the matrix for both the rigid body and the graphic body
	NewtonBodySetMatrix (rigidBody, &matrix[0][0]);
	PhysicsSetTransform (rigidBody, &matrix[0][0], 0);

	NewtonBodyGetVelocity(sourceBody, &veloc[0]);
	NewtonBodyGetOmega(sourceBody, &omega[0]);
	veloc += omega * matrix.RotateVector(origin);

// for now so that I can see the body
veloc = dVector (0, 0, 0, 0);
//	omega = dVector (0, 0, 0, 0);

	NewtonBodySetVelocity(rigidBody, &veloc[0]);
	NewtonBodySetOmega(rigidBody, &omega[0]);

	NewtonReleaseCollision(world, collision);

}


static void DestroyThisBodyCallback (const NewtonBody* body, const NewtonJoint* contactJoint)
{
	NewtonWorld* world;
	NewtonMesh* topMesh;
	NewtonMesh* bottomMesh;
	NewtonMesh* effectMesh;
	RenderPrimitive* srcPrimitive;
	dMatrix matrix;
	dFloat maxForce;
	dVector point;
	dVector dir0;
	dVector dir1;


	// Get the world;
	world = NewtonBodyGetWorld (body);

	// find a the strongest force 
	maxForce = 0.0f;
	for (void* contact = NewtonContactJointGetFirstContact (contactJoint); contact; contact = NewtonContactJointGetNextContact (contactJoint, contact)) {
		dVector force;
		NewtonMaterial* material;

		material = NewtonContactGetMaterial (contact);
		NewtonMaterialGetContactForce (material, &force.m_x);
		if (force.m_x > maxForce) {
			dVector normal;
			NewtonMaterialGetContactPositionAndNormal(material, &point[0], &normal[0]);
			NewtonMaterialGetContactTangentDirections (material, &dir0[0], &dir0[0]);
		}
	}

	// get the visual primitive
	srcPrimitive = (RenderPrimitive*) NewtonBodyGetUserData (body);

	// get the effect mesh that is use to create the debris pieces
	effectMesh = srcPrimitive->m_specialEffect;


	// calculate the cut plane plane
	NewtonBodyGetMatrix (body, &matrix[0][0]);

	dMatrix clipMatrix (dgGrammSchmidt(dir0) * 
						dYawMatrix(30.0f * 3.1416f/180.0f * RandomVariable(1.0f)) * 
						dRollMatrix(30.0f * 3.1416f/180.0f * RandomVariable(1.0f)));

	clipMatrix.m_posit = point;
	clipMatrix.m_posit.m_w = 1.0f;
	clipMatrix = clipMatrix * matrix.Inverse();

	// break the mesh into two pieces
	NewtonMeshClip (effectMesh, meshClipper, &clipMatrix[0][0], &topMesh, &bottomMesh);
	if (topMesh && bottomMesh) {
		dFloat volume;
		NewtonMesh* meshPartA = NULL;
		NewtonMesh* meshPartB = NULL;

		volume = NewtonConvexCollisionCalculateVolume (NewtonBodyGetCollision(body));

		// the clipper was able to make a cut now we can create new debris piece for replacement
		dMatrix clipMatrix1 (dgGrammSchmidt(dir1) * 
							 dYawMatrix(30.0f * 3.1416f/180.0f * RandomVariable(1.0f)) * 
							 dRollMatrix(30.0f * 3.1416f/180.0f * RandomVariable(1.0f)));
		NewtonMeshClip (bottomMesh, meshClipper, &clipMatrix1[0][0], &meshPartA, &meshPartB);
		if (meshPartA && meshPartB) {
			// creat another split (you can make as many depend of the FPS)
			CreateDebriPiece (body, meshPartA, volume);
			CreateDebriPiece (body, meshPartB, volume);

			NewtonMeshDestroy(meshPartA);
			NewtonMeshDestroy(meshPartB);
		} else {
			CreateDebriPiece (body, bottomMesh, volume);
		}
		NewtonMeshDestroy(bottomMesh);


		dMatrix clipMatrix2 (dgGrammSchmidt(dir1) * 
							 dYawMatrix(30.0f * 3.1416f/180.0f * RandomVariable(1.0f)) * 
			                 dRollMatrix(30.0f * 3.1416f/180.0f * RandomVariable(1.0f)));
		NewtonMeshClip (topMesh, meshClipper, &clipMatrix2[0][0], &meshPartA, &meshPartB);
		if (meshPartA && meshPartB) {
			// creat another split (you can make as many depend of the FPS)
			CreateDebriPiece (body, meshPartA, volume);
			CreateDebriPiece (body, meshPartB, volume);

			NewtonMeshDestroy(meshPartA);
			NewtonMeshDestroy(meshPartB);
		} else {
			CreateDebriPiece (body, topMesh, volume);
		}
		NewtonMeshDestroy(topMesh);


		// remove the old visual from graphics world
		SceneManager* system = (SceneManager*) NewtonWorldGetUserData(world);
		delete srcPrimitive;
		system->Remove(srcPrimitive);

		// finally destroy this body;
		NewtonDestroyBody(world, body);
	}
}

static void LoadDestructionCutterMesh (const NewtonWorld* world)
{
	GLuint tex;
	dFloat width; 
	dFloat breadth; 

	dMatrix plane (GetIdentityMatrix());
	dMatrix texMatrix (GetIdentityMatrix());

	width = 10.0f;
	breadth = 10.0f;

	tex = LoadTexture ("destructionDetail.tga");
	texMatrix[1][1] = 4.0f /breadth;
	texMatrix[2][2] = 4.0f /breadth;
	meshClipper = NewtonMeshCreatePlane (world, &plane[0][0], width, breadth, tex, &texMatrix[0][0], &texMatrix[0][0]);
}

static void SetBreakValue (const NewtonBody* body, dFloat speed) 
{
	NewtonCollision* shape;

	// set the max force value that this shape can take before the DestroyBodycallback is called 
	shape = NewtonBodyGetCollision(body);
	if (NewtonCollisionGetMaxBreakImpactImpulse(shape) > 1.0e6f) {
		dFloat mass;
		dFloat Ixx;
		dFloat Iyy;
		dFloat Izz;
		dFloat breakImpulse;

		// set the max force as a factor of the body weight
		NewtonBodyGetMassMatrix(body, &mass, &Ixx, &Iyy, &Izz);

		// break if the impact is higher than speed m/s
		breakImpulse = speed * mass;
		NewtonCollisionSetMaxBreakImpactImpulse(shape, breakImpulse);
	}

}


static void SetDemoCallbacks (SceneManager& system)
{
	system.m_control = Keyboard;
	system.m_autoSleep = AutoSleep;
	system.m_showIslands = SetShowIslands;
	system.m_showContacts = SetShowContacts; 
	system.m_setMeshCollision = SetShowMeshCollision;
}


static void BuildFloorAndSceneRoot (SceneManager& system)
{
	NewtonWorld* world;
	RenderPrimitive* floor;
	NewtonBody* floorBody;
	NewtonCollision* floorCollision;
	OGLMesh* meshInstance;

	world = system.m_world;
	// /////////////////////////////////////////////////////////////////////
	//
	// create the sky box,
	system.AddModel (new SkyBox ());


	// create the the floor graphic objects
	dVector floorSize (100.0f, 2.0f, 100.0f);
	dMatrix location (GetIdentityMatrix());
	location.m_posit.m_y = -5.0f; 

	// create a box for floor 
	floorCollision = NewtonCreateBox (world, floorSize.m_x, floorSize.m_y, floorSize.m_z, 0, NULL); 

	//	meshInstance = OGLMesh::MakeBox (world, size.m_x, size.m_y, size.m_z, "GrassAndDirt.tga");
	meshInstance = new OGLMesh (floorCollision, "GrassAndDirt.tga", "metal_30.tga", "metal_30.tga");
	floor = new RenderPrimitive (location, meshInstance);
	system.AddModel (floor);
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
}

//static void UnstableStruture (SceneManager& system, dVector location, int high)
static void BreakableStruture (SceneManager& system, dVector location, int high)
{
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

	// create a material carrier to to cou collision wit ethsi obejted
	int defaultMaterialID;
	defaultMaterialID = NewtonMaterialGetDefaultGroupID (system.m_world);

	dMatrix columAlignment (dRollMatrix(3.1416f * 0.5f));
	for (int i = 0; i < high; i ++) { 

		NewtonBody* body;
		RenderPrimitive* primitive;

		dMatrix matrix(columAlignment * baseMatrix);


		// add the 4 column
		matrix.m_posit.m_x -=  (columnBoxSize.m_z - plankBoxSize.m_x) * 0.5f;
		matrix.m_posit.m_z -=  (columnBoxSize.m_z - plankBoxSize.m_z) * 0.5f;
		body = CreateGenericSolid (system.m_world, &system, columnMass, matrix, columnBoxSize, _RANDOM_CONVEX_HULL_PRIMITIVE, defaultMaterialID);
		primitive = (RenderPrimitive*) NewtonBodyGetUserData (body);
		SetBreakValue (body, 50.0f);
		ConvexCastPlacement (body);

		matrix.m_posit.m_x += columnBoxSize.m_z - plankBoxSize.m_x;
		body = CreateGenericSolid (system.m_world, &system, columnMass, matrix, columnBoxSize, _RANDOM_CONVEX_HULL_PRIMITIVE, defaultMaterialID);
		primitive = (RenderPrimitive*) NewtonBodyGetUserData (body);
		SetBreakValue (body, 30.0f);
		ConvexCastPlacement (body);

		matrix.m_posit.m_z += columnBoxSize.m_z - plankBoxSize.m_z;		
		body = CreateGenericSolid (system.m_world, &system, columnMass, matrix, columnBoxSize, _RANDOM_CONVEX_HULL_PRIMITIVE, defaultMaterialID);
		primitive = (RenderPrimitive*) NewtonBodyGetUserData (body);
		SetBreakValue (body, 30.0f);
		ConvexCastPlacement (body);

		matrix.m_posit.m_x -= columnBoxSize.m_z - plankBoxSize.m_x;
		body = CreateGenericSolid (system.m_world, &system, columnMass, matrix, columnBoxSize, _RANDOM_CONVEX_HULL_PRIMITIVE, defaultMaterialID);
		primitive = (RenderPrimitive*) NewtonBodyGetUserData (body);
		SetBreakValue (body, 30.0f);
		ConvexCastPlacement (body);

		// add a plank
		dVector size (plankBoxSize);
		size.m_x *= 0.85f;
		size.m_z *= 0.85f;
		body = CreateGenericSolid (system.m_world, &system, plankMass, baseMatrix, size, _BOX_PRIMITIVE, defaultMaterialID);
		primitive = (RenderPrimitive*) NewtonBodyGetUserData (body);
		SetBreakValue (body, 1.0f);
		ConvexCastPlacement (body);

		// set up for another level
		baseMatrix.m_posit.m_y += (columnBoxSize.m_x + plankBoxSize.m_y);
	}

	dFloat mass;
	NewtonBody* body;
	RenderPrimitive* primitive;
	PrimitiveType type = _BOX_PRIMITIVE;
	dVector size (1.0f, 2.0f, 1.0f);
	dMatrix matrix (GetIdentityMatrix());

	mass = 10.0f;
	matrix.m_posit = location;
	matrix.m_posit.m_y = FindFloor (system.m_world, matrix.m_posit.m_x, matrix.m_posit.m_z) + baseMatrix.m_posit.m_y + 35.0f; 

	body = CreateGenericSolid (system.m_world, &system, mass, matrix, size, type, defaultMaterialID);
	primitive = (RenderPrimitive*) NewtonBodyGetUserData (body);
}



void RealTimeSimpleDestruction(SceneManager& system)
{
	NewtonWorld* world;
	world = system.m_world;

	// create the sky box and the floor,
	BuildFloorAndSceneRoot (system);

	// save the system wit the world
	NewtonWorldSetUserData(system.m_world, &system);

	// Set a world destruction callback so that we can destroy all assets created for special effects
	NewtonWorldSetDestructorCallBack (system.m_world, DestroyWorldCallback);

	// Set a Function callback for when a Convex Body collision is destroyed if the force exceeded the Max break value
	NewtonSetDestroyBodyByExeciveForce (system.m_world, DestroyThisBodyCallback); 

	// this will load a assets to create destruction special effects 
	LoadDestructionCutterMesh (system.m_world);

	BreakableStruture (system, dVector (-10.0f, 0.0f, -10.0f, 0.0f), 2);
	BreakableStruture (system, dVector ( 10.0f, 0.0f, -10.0f, 0.0f), 5);
	BreakableStruture (system, dVector (-10.0f, 0.0f,  10.0f, 0.0f), 3);
	BreakableStruture (system, dVector ( 10.0f, 0.0f,  10.0f, 0.0f), 3);
}







