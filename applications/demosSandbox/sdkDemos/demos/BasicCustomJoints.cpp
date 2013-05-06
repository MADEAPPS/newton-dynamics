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


#include "Custom6DOF.h"
#include "CustomGear.h"
#include "CustomHinge.h"
#include "CustomPulley.h"
#include "CustomSlider.h"
#include "CustomWormGear.h"
#include "CustomUniversal.h"
#include "CustomCorkScrew.h"
#include "CustomBallAndSocket.h"
#include "JointLibrary.h"


#define ADD_GEARS
#define USE_SMOOTH_SHAPE



static void SetDemoCallbacks (NewtonFrame& system)
{
	system.m_control = Keyboard;
	system.m_autoSleep = AutoSleep;
	system.m_showIslands = SetShowIslands;
	system.m_showContacts = SetShowContacts; 
	system.m_setMeshCollision = SetShowMeshCollision;
}

static NewtonBody* BuildFloorAndSceneRoot (NewtonFrame& system)
{
_ASSERTE (0);
return NULL;
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

	return floorBody;
*/
}



// create a rope of boxes
static void AddRope (dVector position, NewtonWorld* nWorld, SceneManager* system, const NewtonBody* worldBody, dInt32 linksCount)
{
_ASSERTE (0);
/*
	int i;
	dFloat mass;
	dFloat Ixx;
	dFloat Iyy;
	dFloat Izz;
	NewtonBody* link0;
	NewtonBody* link1;

	NewtonCollision* collision;
	RenderPrimitive* visualObject;

	dVector size (1.0f, 0.15f, 0.15f);
	dMatrix location (dRollMatrix(90.0f * 3.141592f / 180.0f));

	location.m_posit = position;
	location.m_posit.m_w = 1.0f;
	location.m_posit.m_y = linksCount * (size.m_x - size.m_y * 0.5f) + FindFloor (nWorld, location.m_posit.m_x, location.m_posit.m_z) + 2.0f;

	// create a collision primitive to be shared by all links
#ifdef USE_SMOOTH_SHAPE
	collision = NewtonCreateCapsule (nWorld, size.m_y, size.m_x, 0, NULL);
#else
	collision = NewtonCreateCylinder (nWorld, size.m_y, size.m_x, 0, NULL);
#endif


	// calculate a accurate moment of inertia
	dVector origin;
	dVector inertia;
	NewtonConvexCollisionCalculateInertialMatrix (collision, &inertia[0], &origin[0]);	

	mass = 2.0f;
	Ixx = mass * inertia[0];
	Iyy = mass * inertia[1];
	Izz = mass * inertia[2];



	int defaultID;
	defaultID = NewtonMaterialGetDefaultGroupID (nWorld);


	// create on mesh and share it
	OGLMesh* capsuleGeometry;
	capsuleGeometry = new OGLMesh("ropeLink", collision, "camo.tga", "camo.tga", "camo.tga");

	link0 = NULL;
	// create a long vertical rope with limits
	for (i = 0; i < linksCount; i ++) {
		// create the a graphic character (use a visualObject as our body
		visualObject = new RenderPrimitive (location, capsuleGeometry);
		system->AddModel___ (visualObject);
		visualObject->Release();

		//create the rigid body
		link1 = NewtonCreateBody (nWorld, collision);


		// add some damping to each link
		dFloat dampValue = 0.0f; 
		NewtonBodySetLinearDamping (link1, dampValue);

		dampValue = 0.1f;
		dVector angularDamp (dampValue, dampValue, dampValue);
		NewtonBodySetAngularDamping (link1, &angularDamp.m_x);

		// Set Material Id for this object
		NewtonBodySetMaterialGroupID (link1, defaultID);

		// save the pointer to the graphic object with the body.
		NewtonBodySetUserData (link1, visualObject);

		// set a destructor for this rigid body
		NewtonBodySetDestructorCallback (link1, PhysicsBodyDestructor);

		// set the transform call back function
		NewtonBodySetTransformCallback (link1, PhysicsSetTransform);

		// set the force and torque call back function
		NewtonBodySetForceAndTorqueCallback (link1,PhysicsApplyGravityForce);

		// set the mass matrix
		NewtonBodySetMassMatrix (link1, mass, Ixx, Iyy, Izz);

		// set the matrix for both the rigid body and the graphic body
		NewtonBodySetMatrix (link1, &location[0][0]);
		PhysicsSetTransform (link1, &location[0][0], 0);

		dVector pivot (location.m_posit);
		pivot.m_y += (size.m_x - size.m_y) * 0.5f;

#if 0
		dFloat coneAngle = 15.0 * 3.141592f / 180.0f;
		dFloat twistAngle = 5.0 * 3.141592f / 180.0f;
		dMatrix pinAndPivot (GetIdentityMatrix());
		pinAndPivot.m_front = location.m_front.Scale (-1.0f);
		pinAndPivot.m_up = location.m_up;
		pinAndPivot.m_right = pinAndPivot.m_front * pinAndPivot.m_up;
		pinAndPivot.m_posit = pivot;
		CustomConeLimitedBallAndSocket* joint;
		joint = new CustomConeLimitedBallAndSocket(twistAngle, coneAngle, pinAndPivot, link1, link0);
#else

		dMatrix pinAndPivot (GetIdentityMatrix());
		pinAndPivot.m_front = location.m_front;
		pinAndPivot.m_up = location.m_right;
		pinAndPivot.m_right = pinAndPivot.m_front * pinAndPivot.m_up;
		pinAndPivot.m_posit = pivot;
		CustomBallAndSocket* joint;
		joint = new CustomBallAndSocket (pinAndPivot, link1, link0);
#endif


		link0 = link1;
		location.m_posit.m_y -= (size.m_x - size.m_y);
	}

	// release the collision geometry when not need it
	NewtonReleaseCollision (nWorld, collision);

	// release the visual geometry
	capsuleGeometry->Release();
*/
}



// void AddDoubleSwingDoors (dModel* parent, NewtonWorld* nWorld, dVector& position, int count)
static void AddDoubleSwingDoors (dVector position, NewtonWorld* nWorld, SceneManager* system, const NewtonBody* worldBody, int count)
{
_ASSERTE (0);
/*

	dFloat mass;
	dFloat Ixx;
	dFloat Iyy;
	dFloat Izz;
	NewtonBody* link0;
	NewtonBody* link1;
	CustomHinge* joint;
	NewtonCollision* collision;
	RenderPrimitive* visualObject;

	dVector size (2.0f, 5.0f, 0.5f);

	int defaultID;
	defaultID = NewtonMaterialGetDefaultGroupID (nWorld);


	dMatrix location (GetIdentityMatrix());
	location.m_posit = position;
	location.m_posit.m_w = 1.0f;
	location.m_posit.m_y = FindFloor (nWorld, location.m_posit.m_x, location.m_posit.m_z) + 2.0f + size.m_y * 0.5f;

	// create a collision primitive to be shared by all links
	collision = NewtonCreateBox (nWorld, size.m_x, size.m_y, size.m_z, 0, NULL); 

	// calculate a accurate moment of inertia
	dVector origin;
	dVector inertia;
	NewtonConvexCollisionCalculateInertialMatrix (collision, &inertia[0], &origin[0]);	

	// create on mesh and share it
	OGLMesh* boxGeometry;
	boxGeometry = new OGLMesh("doorwing", collision, "camo.tga", "camo.tga", "camo.tga");

	mass = 5.0f;
	Ixx = mass * inertia[0];
	Iyy = mass * inertia[1];
	Izz = mass * inertia[2];

	// make first wing
	{
		// create the a graphic character (use a visualObject as our body
		visualObject = new RenderPrimitive (location, boxGeometry);
		system->AddModel___ (visualObject);
		visualObject->Release();


		//create the rigid body
		link1 = NewtonCreateBody (nWorld, collision);

		// Set Material Id for this object
		NewtonBodySetMaterialGroupID (link1, defaultID);

		// save the pointer to the graphic object with the body.
		NewtonBodySetUserData (link1, visualObject);

		// set a destructor for this rigid body
		NewtonBodySetDestructorCallback (link1, PhysicsBodyDestructor);

		// set the transform call back function
		NewtonBodySetTransformCallback (link1, PhysicsSetTransform);

		// set the force and torque call back function
		NewtonBodySetForceAndTorqueCallback (link1,PhysicsApplyGravityForce);

		// set the mass matrix
		NewtonBodySetMassMatrix (link1, mass, Ixx, Iyy, Izz);

		// set the matrix for both the rigid body and the graphic body
		NewtonBodySetMatrix (link1, &location[0][0]);
		PhysicsSetTransform (link1, &location[0][0], 0);

		dMatrix pinAndPivot (GetIdentityMatrix());
		pinAndPivot.m_front = location.m_up;
		pinAndPivot.m_up = location.m_front;
		pinAndPivot.m_right = pinAndPivot.m_front * pinAndPivot.m_up;
		pinAndPivot.m_posit = location.m_posit;
		pinAndPivot.m_posit.m_x += size.m_x * 0.5f;

		// connect these two bodies by a ball and socket joint
		joint = new CustomHinge (pinAndPivot, link1, NULL);

		joint->EnableLimits (true);
		joint->SetLimis (-30.0f * 3.141592f/180.0f, 30.0f * 3.141592f/180.0f); 
	}


	// make second wing
	for (int i = 0; i < count; i ++) {

		location.m_posit.m_x -= size.m_x;

		// create the a graphic character (use a visualObject as our body
		visualObject = new RenderPrimitive (location, boxGeometry);
		system->AddModel___ (visualObject);
		visualObject->Release();


		//create the rigid body
		link0 = NewtonCreateBody (nWorld, collision);

		// Set Material Id for this object
		NewtonBodySetMaterialGroupID (link0, defaultID);

		// save the pointer to the graphic object with the body.
		NewtonBodySetUserData (link0, visualObject);

		// set a destructor for this rigid body
		NewtonBodySetDestructorCallback (link0, PhysicsBodyDestructor);

		// set the transform call back function
		NewtonBodySetTransformCallback (link0, PhysicsSetTransform);

		// set the force and torque call back function
		NewtonBodySetForceAndTorqueCallback (link0,PhysicsApplyGravityForce);

		// set the mass matrix
		NewtonBodySetMassMatrix (link0, mass, Ixx, Iyy, Izz);

		// set the matrix for both the rigid body and the graphic body
		NewtonBodySetMatrix (link0, &location[0][0]);
		PhysicsSetTransform (link0, &location[0][0], 0);

		dMatrix pinAndPivot (GetIdentityMatrix());
		pinAndPivot.m_front = location.m_up;
		pinAndPivot.m_up = location.m_front;
		pinAndPivot.m_right = pinAndPivot.m_front * pinAndPivot.m_up;
		pinAndPivot.m_posit = location.m_posit;
		pinAndPivot.m_posit.m_x += size.m_x * 0.5f;

		// connect these two bodies by a ball and socket joint
		joint = new CustomHinge (pinAndPivot, link0, link1);

		joint->EnableLimits (true);
		joint->SetLimis (-30.0f * 3.141592f/180.0f, 30.0f * 3.141592f/180.0f); 

		link1 = link0;
	}

	// release the collision geometry when not need it
	NewtonReleaseCollision (nWorld, collision);

	// release the visual geometry
	boxGeometry->Release();
*/
}



struct HingeLocalInfo 
{
//	dFloat m_minLimit;
//	dFloat m_maxLimit;
//	dFloat m_desiredVelocity;
	dFloat m_desiredAngle;
};

static void HingeMotorDestructorCallback (const NewtonUserJoint* me)
{
	free (CustomGetUserData (me));
}

static void HingeControlledByAngle (const NewtonUserJoint* me, dFloat timestep, int threadIndex)
{
	dFloat errorAngle;
	dFloat jointAngle;
	dVector pin;
	HingeLocalInfo* info;
	NewtonJoint* newJoint;
		

	info = (HingeLocalInfo*) CustomGetUserData (me);
	jointAngle = HingeGetJointAngle (me);
	HingeGetPinAxis (me, &pin[0]);
	newJoint = CustomGetNewtonJoint (me);

	errorAngle =  jointAngle - info->m_desiredAngle;
	NewtonUserJointAddAngularRow (newJoint, errorAngle, &pin[0]);

}




//  void AddRollingBeats (dModel* parent, NewtonWorld* nWorld, dVector& posit)
static void AddRollingBeats (dVector position, NewtonWorld* nWorld, SceneManager* system, const NewtonBody* worldBody)
{
_ASSERTE (0);
/*
	dFloat mass;
	dFloat Ixx;
	dFloat Iyy;
	dFloat Izz;
	NewtonBody* bar;
	NewtonCollision* collision;

	dVector size (10.0f, 0.25f, 0.25f);
	dMatrix location (GetIdentityMatrix());

	location.m_posit = position;
	location.m_posit.m_w = 1.0f;
	location.m_posit.m_y = FindFloor (nWorld, location.m_posit.m_x, location.m_posit.m_z) + 2.0f;

	bar = NULL;

	// /////////////////////////////////////////////////////////////////////////////////////             
	//
	// create a bar and attach it to the world with a hinge with limits
	//
	// ////////////////////////////////////////////////////////////////////////////////////
	{
		
		RenderPrimitive* visualObject;

		// create a collision primitive to be shared by all links
		collision = NewtonCreateCylinder (nWorld, size.m_y, size.m_x, 0, NULL); 

		// create a visual object as our body
		OGLMesh* visualGeometry;
		visualGeometry = new OGLMesh("bar", collision, "camo.tga", "camo.tga", "camo.tga");
		visualObject = new RenderPrimitive (location, visualGeometry);
		system->AddModel___ (visualObject);
		visualObject->Release();
		visualGeometry->Release();


		// create the bar body
		bar = NewtonCreateBody(nWorld, collision);
		NewtonReleaseCollision (nWorld, collision);

		// attach graphic object to the rigid body
		NewtonBodySetUserData(bar, visualObject);

		// set a destructor function
		NewtonBodySetDestructorCallback (bar, PhysicsBodyDestructor);

		// set the transform call back function
		NewtonBodySetTransformCallback (bar, PhysicsSetTransform);

		// set the force and torque call back function
		NewtonBodySetForceAndTorqueCallback (bar,PhysicsApplyGravityForce);

		// calculate a accurate moment of inertia
		dVector origin;
		dVector inertia;
		NewtonConvexCollisionCalculateInertialMatrix (collision, &inertia[0], &origin[0]);	

		mass = 5.0f;
		Ixx = mass * inertia[0];
		Iyy = mass * inertia[1];
		Izz = mass * inertia[2];

		// set the mass matrix
		NewtonBodySetMassMatrix (bar, mass, Ixx, Iyy, Izz);

		// set the matrix for both the rigid body and the graphic body
		NewtonBodySetMatrix (bar, &location[0][0]);
		PhysicsSetTransform (bar, &location[0][0], 0);

		// connect these two bodies by a hinge joint

		dMatrix pinAndPivot (GetIdentityMatrix());


		pinAndPivot.m_front = dVector (0.0, 1.0f, 0.0f, 0.0f);
		pinAndPivot.m_up = location.m_front * pinAndPivot.m_front;
		pinAndPivot.m_up = pinAndPivot.m_up.Scale (1.0f /dSqrt (pinAndPivot.m_up % pinAndPivot.m_up));
		pinAndPivot.m_right = pinAndPivot.m_front * pinAndPivot.m_up;
		pinAndPivot.m_posit = location.m_posit - location.m_front.Scale (size.m_x * 0.5f);
#if 1
//		NewtonCustomJoint* joint;
//		joint = new CustomHinge (pinAndPivot, bar, NULL);
		NewtonCustomJoint* joint;
		joint = new Custom6DOF (pinAndPivot, pinAndPivot, bar, NULL);
		((Custom6DOF*)joint)->SetAngularLimits (dVector (-10000000, 0, 0), dVector (10000000, 0, 0));

#else
		HingeLocalInfo* info;
		NewtonUserJoint* joint;
		joint = CreateCustomHinge (&pinAndPivot[0][0], bar, NULL);
		HingeEnableLimits(joint, 1);
		HingeSetLimis (joint, -200.0, 200.0f);
		CustomSetSubmitContraintCallback (joint, HingeControlledByAngle);
		CustomSetDestructorCallback (joint, HingeMotorDestructorCallback);

		info =(HingeLocalInfo*) malloc (sizeof (HingeLocalInfo));
//		info->m_minLimit = -200.0;
//		info->m_maxLimit =  200.0f;
//		info->m_desiredVelocity = 5.0;
		info->m_desiredAngle = 30.0f * 3.141592f/180.0f;
		CustomSetUserData (joint, info);

#endif
	}



	{
		// ////////////////////////////////////////////////////////////////////////////////////
		//
		// add a corkscrew visualObject with limits
		//
		// ////////////////////////////////////////////////////////////////////////////////////
		NewtonBody* beat;
		CustomCorkScrew* joint;
		RenderPrimitive* visualObject;
		dMatrix beatLocation (location);
		dVector beatSize (0.5f, 1.25f, 1.25f);

		beatLocation.m_posit.m_x -= size.m_x * 0.25f;

		// create a collision primitive to be shared by all links
#ifdef USE_SMOOTH_SHAPE
		collision = NewtonCreateChamferCylinder (nWorld, beatSize.m_y, beatSize.m_x, 0, NULL); 
#else
		collision = NewtonCreateCylinder (nWorld, beatSize.m_y, beatSize.m_x, 0, NULL); 
#endif


		// create the a graphic character (use a visualObject as our body
		OGLMesh* visualGeometry;
		visualGeometry = new OGLMesh("wheel_1", collision, "camo.tga", "camo.tga", "camo.tga");
		visualObject = new RenderPrimitive (beatLocation, visualGeometry);
		system->AddModel___ (visualObject);
		visualObject->Release();
		visualGeometry->Release();

		beat = NewtonCreateBody(nWorld, collision);
		NewtonReleaseCollision (nWorld, collision);

		// attach graphic object to the rigid body
		NewtonBodySetUserData(beat, visualObject);

		// set a destructor function
		NewtonBodySetDestructorCallback (beat, PhysicsBodyDestructor);

		// set the transform call back function
		NewtonBodySetTransformCallback (beat, PhysicsSetTransform);

		// set the force and torque call back function
		NewtonBodySetForceAndTorqueCallback (beat,PhysicsApplyGravityForce);

		// calculate a accurate moment of inertia
		dVector origin;
		dVector inertia;
		NewtonConvexCollisionCalculateInertialMatrix (collision, &inertia[0], &origin[0]);	

		mass = 5.0f;
		Ixx = mass * inertia[0];
		Iyy = mass * inertia[1];
		Izz = mass * inertia[2];


		// set the mass matrix
		NewtonBodySetMassMatrix (beat, mass, Ixx, Iyy, Izz);

		// set the matrix for both the rigid body and the graphic body
		NewtonBodySetMatrix (beat, &beatLocation[0][0]);
		PhysicsSetTransform (beat, &beatLocation[0][0], 0);

		// set the pivot relative for the first bar
		const dMatrix& pinAndPivot = beatLocation;
		joint = new CustomCorkScrew (pinAndPivot, beat, bar);

		// calculate the minimum and maximum limit for this joints
		dFloat minLimits = ((location.m_posit.m_x - beatLocation.m_posit.m_x) - size.m_x * 0.5f);
		dFloat maxLimits = ((location.m_posit.m_x - beatLocation.m_posit.m_x) + size.m_x * 0.5f);

		joint->EnableLinearLimits(true);
		joint->SetLinearLimis (minLimits, maxLimits); 
	}

	{
		// ////////////////////////////////////////////////////////////////////////////////////
		//
		// add a sliding visualObject with limits
		//
		NewtonBody* beat;
		CustomSlider* joint;
		RenderPrimitive* visualObject;
		dMatrix beatLocation (location);
		dVector beatSize (0.5f, 2.0f, 2.0f);

		beatLocation.m_posit.m_x += size.m_x * 0.25f;


		// create a collision primitive to be shared by all links
		collision = NewtonCreateBox (nWorld, beatSize.m_x, beatSize.m_y, beatSize.m_z, 0, NULL); 

		// create the a graphic character (use a visualObject as our body)
		OGLMesh* visualGeometry;
		visualGeometry = new OGLMesh("box_1", collision, "camo.tga", "camo.tga", "camo.tga");
		visualObject = new RenderPrimitive (beatLocation, visualGeometry);
		system->AddModel___ (visualObject);
		visualObject->Release();

		visualGeometry->Release();

		beat = NewtonCreateBody(nWorld, collision);
		NewtonReleaseCollision (nWorld, collision);

		// attach graphic object to the rigid body
		NewtonBodySetUserData(beat, visualObject);

		// set a destructor function
		NewtonBodySetDestructorCallback (beat, PhysicsBodyDestructor);

		// set the transform call back function
		NewtonBodySetTransformCallback (beat, PhysicsSetTransform);

		// set the force and torque call back function
		NewtonBodySetForceAndTorqueCallback (beat,PhysicsApplyGravityForce);

		// calculate a accurate moment of inertia
		dVector origin;
		dVector inertia;
		NewtonConvexCollisionCalculateInertialMatrix (collision, &inertia[0], &origin[0]);	

		mass = 5.0f;
		Ixx = mass * inertia[0];
		Iyy = mass * inertia[1];
		Izz = mass * inertia[2];


		// set the mass matrix
		NewtonBodySetMassMatrix (beat, mass, Ixx, Iyy, Izz);

		// set the matrix for both the rigid body and the graphic body
		NewtonBodySetMatrix (beat, &beatLocation[0][0]);
		PhysicsSetTransform (beat, &beatLocation[0][0], 0);

		// set the pivot relative for the first bar
		const dMatrix& pinAndPivot = beatLocation;
		joint = new CustomSlider (pinAndPivot, beat, bar);

		// calculate the minimum and maximum limit for this joints
		dFloat minLimits = ((location.m_posit.m_x - beatLocation.m_posit.m_x) - size.m_x * 0.5f);
		dFloat maxLimits = ((location.m_posit.m_x - beatLocation.m_posit.m_x) + size.m_x * 0.5f);

		joint->EnableLimits(true);
		joint->SetLimis (minLimits, maxLimits); 
	}

	{
		// ////////////////////////////////////////////////////////////////////////////////////
		//
		// add a universal joint visualObject with limits
		//
		// ////////////////////////////////////////////////////////////////////////////////////
		NewtonBody* beat;

		RenderPrimitive* visualObject;
		dMatrix beatLocation (location);
		dVector beatSize (0.5f, 1.25f, 1.25f);

		beatLocation.m_posit.m_x -= size.m_x * 0.5f;

		// create a collision primitive to be shared by all links
#ifdef USE_SMOOTH_SHAPE
		collision = NewtonCreateChamferCylinder (nWorld, beatSize.m_y, beatSize.m_x, 0, NULL); 
#else
		collision = NewtonCreateCylinder (nWorld, beatSize.m_y, beatSize.m_x, 0, NULL); 
#endif


		// create the a graphic character (use a visualObject as our body
		OGLMesh* visualGeometry;
		visualGeometry = new OGLMesh("wheel_1", collision, "camo.tga", "camo.tga", "camo.tga");
		visualObject = new RenderPrimitive (beatLocation, visualGeometry);
		system->AddModel___ (visualObject);
		visualObject->Release();
		visualGeometry->Release();

		beat = NewtonCreateBody(nWorld, collision);
		NewtonReleaseCollision (nWorld, collision);

		// attach graphic object to the rigid body
		NewtonBodySetUserData(beat, visualObject);

		// set a destructor function
		NewtonBodySetDestructorCallback (beat, PhysicsBodyDestructor);

		// set the transform call back function
		NewtonBodySetTransformCallback (beat, PhysicsSetTransform);

		// set the force and torque call back function
		NewtonBodySetForceAndTorqueCallback (beat,PhysicsApplyGravityForce);

		// calculate a accurate moment of inertia
		dVector origin;
		dVector inertia;
		NewtonConvexCollisionCalculateInertialMatrix (collision, &inertia[0], &origin[0]);	

		mass = 5.0f;
		Ixx = mass * inertia[0];
		Iyy = mass * inertia[1];
		Izz = mass * inertia[2];

		// set the mass matrix
		NewtonBodySetMassMatrix (beat, mass, Ixx, Iyy, Izz);

		// set the matrix for both the rigid body and the graphic body
		NewtonBodySetMatrix (beat, &beatLocation[0][0]);
		PhysicsSetTransform (beat, &beatLocation[0][0], 0);

		// set the pivot relative for the first bar
		dMatrix pinAndPivot (beatLocation);
#if 1
		CustomUniversal* joint;
		joint = new CustomUniversal (pinAndPivot, beat, bar);
		joint->EnableLimit_0 (false);
		joint->EnableLimit_1 (true);
		//joint->EnableMotor_0 (true);
#else
		Custom6DOF* joint1 = new Custom6DOF (pinAndPivot, pinAndPivot,
			dVector (0, 0, 0), dVector (0, 0, 0), 
			dVector (-10000000, -1000000, -0), dVector (10000000, 1000000, 0), 
			beat, bar);
#endif

	}

	{
		// ////////////////////////////////////////////////////////////////////////////////////
		//
		// add a universal joint visualObject with limits
		//
		// ////////////////////////////////////////////////////////////////////////////////////
		NewtonBody* beat;
		CustomUniversal* joint;
		RenderPrimitive* visualObject;
		dMatrix beatLocation (location);
		dVector beatSize (0.5f, 1.25f, 1.25f);

		beatLocation.m_posit.m_x += size.m_x * 0.5f;

		// create a collision primitive to be shared by all links
#ifdef USE_SMOOTH_SHAPE
		collision = NewtonCreateChamferCylinder (nWorld, beatSize.m_y, beatSize.m_x, 0, NULL); 
#else
		collision = NewtonCreateCylinder (nWorld, beatSize.m_y, beatSize.m_x, 0, NULL); 
#endif


		// create the a graphic character (use a visualObject as our body
		OGLMesh* visualGeometry;
		visualGeometry = new OGLMesh("wheel_1", collision, "camo.tga", "camo.tga", "camo.tga");
		visualObject = new RenderPrimitive (beatLocation, visualGeometry);
		system->AddModel___ (visualObject);
		visualObject->Release();
		visualGeometry->Release();


		beat = NewtonCreateBody(nWorld, collision);
		NewtonReleaseCollision (nWorld, collision);

		// attach graphic object to the rigid body
		NewtonBodySetUserData(beat, visualObject);

		// set a destructor function
		NewtonBodySetDestructorCallback (beat, PhysicsBodyDestructor);

		// set the transform call back function
		NewtonBodySetTransformCallback (beat, PhysicsSetTransform);

		// set the force and torque call back function
		NewtonBodySetForceAndTorqueCallback (beat,PhysicsApplyGravityForce);

		// calculate a accurate moment of inertia
		dVector origin;
		dVector inertia;
		NewtonConvexCollisionCalculateInertialMatrix (collision, &inertia[0], &origin[0]);	

		mass = 5.0f;
		Ixx = mass * inertia[0];
		Iyy = mass * inertia[1];
		Izz = mass * inertia[2];


		// set the mass matrix
		NewtonBodySetMassMatrix (beat, mass, Ixx, Iyy, Izz);

		// set the matrix for both the rigid body and the graphic body
		NewtonBodySetMatrix (beat, &beatLocation[0][0]);
		PhysicsSetTransform (beat, &beatLocation[0][0], 0);

		// set the pivot relative for the first bar
		dMatrix pinAndPivot (beatLocation);
		joint = new CustomUniversal (pinAndPivot, beat, bar);

		//joint->EnableLimit_0 (false);
		//joint->EnableMotor_0 (true);
		joint->EnableMotor_0 (false);
		joint->EnableMotor_1 (false);
	}
*/
}



static void AddHangingBridge (dVector position, NewtonWorld* nWorld, SceneManager* system, dInt32 linksCount)
{
_ASSERTE (0);
/*
	dFloat mass;
	dFloat Ixx;
	dFloat Iyy;
	dFloat Izz;
	NewtonBody* link0;
	NewtonBody* link1;
	CustomHinge* joint;
	OGLMesh* boxGeometry;
	OGLMesh* linkGeometry;
	RenderPrimitive* visualObject;
	NewtonCollision* boxCollision;
	NewtonCollision* linkCollision;

	//	 dVector size (2.0f, 0.25f, 0.25f);
	dVector boxSize (3.0f, 1.0f, 3.0f);

	int defaultID;
	defaultID = NewtonMaterialGetDefaultGroupID (nWorld);


	dMatrix location (GetIdentityMatrix());
	location.m_posit = position;
	location.m_posit.m_w = 1.0f;
	location.m_posit.m_y = FindFloor (nWorld, location.m_posit.m_x, location.m_posit.m_z) + 5.0f;

	boxCollision = NewtonCreateBox (nWorld, boxSize.m_x, boxSize.m_y, boxSize.m_z, 0, NULL); 
	boxGeometry = new OGLMesh ("slap", boxCollision, "wood_2.tga", "wood_2.tga", "wood_1.tga");
	visualObject = new RenderPrimitive (location, boxGeometry);
	system->AddModel___ (visualObject);
	visualObject->Release();


	link0 = NewtonCreateBody (nWorld, boxCollision);
	NewtonBodySetMaterialGroupID (link0, defaultID);
	NewtonBodySetUserData (link0, visualObject);

	// set a destructor for this rigid body
	NewtonBodySetDestructorCallback (link0, PhysicsBodyDestructor);
	NewtonBodySetTransformCallback (link0, PhysicsSetTransform);
	NewtonBodySetMatrix (link0, &location[0][0]);
	PhysicsSetTransform (link0, &location[0][0], 0);


	dVector linkSize (1.4f, 0.2f, 0.5f);

	location.m_posit.m_z += boxSize.m_z * 0.5f + linkSize.m_z * 0.5f;
	location.m_posit.m_y += boxSize.m_y * 0.5f - linkSize.m_y * 0.5f;;

	linkCollision = NewtonCreateBox (nWorld, linkSize.m_x, linkSize.m_y, linkSize.m_z, 0, NULL); 
	linkGeometry = new OGLMesh("box_2", linkCollision, "plastic_.tga", "plastic_.tga", "plastic_.tga");


	// calculate a accurate moment of inertia
	dVector origin;
	dVector inertia;
	NewtonConvexCollisionCalculateInertialMatrix (linkCollision, &inertia[0], &origin[0]);	

	mass = 2.0f;
	Ixx = mass * inertia[0];
	Iyy = mass * inertia[1];
	Izz = mass * inertia[2];

	// create a long vertical rope with limits
	for (int i = 0; i < linksCount; i ++) {

		// create the a graphic character (use a visualObject as our body
		visualObject = new RenderPrimitive (location, linkGeometry);
		system->AddModel___ (visualObject);
		visualObject->Release();

		//create the rigid body
		link1 = NewtonCreateBody (nWorld, linkCollision);

		// add some damping to each link
		dFloat dampValue = 0.0f; 
		NewtonBodySetLinearDamping (link1, dampValue);

		dampValue = 0.1f;
		dVector angularDamp (dampValue, dampValue, dampValue);
		NewtonBodySetAngularDamping (link1, &angularDamp.m_x);

		// Set Material Id for this object
		NewtonBodySetMaterialGroupID (link1, defaultID);

		// save the pointer to the graphic object with the body.
		NewtonBodySetUserData (link1, visualObject);

		// set a destructor for this rigid body
		NewtonBodySetDestructorCallback (link1, PhysicsBodyDestructor);

		// set the transform call back function
		NewtonBodySetTransformCallback (link1, PhysicsSetTransform);

		// set the force and torque call back function
		NewtonBodySetForceAndTorqueCallback (link1,PhysicsApplyGravityForce);

		// set the mass matrix
		NewtonBodySetMassMatrix (link1, mass, Ixx, Iyy, Izz);

		// set the matrix for both the rigid body and the graphic body
		NewtonBodySetMatrix (link1, &location[0][0]);
		PhysicsSetTransform (link1, &location[0][0], 0);

		dVector pivot (location.m_posit);
		pivot.m_y += linkSize.m_y * 0.5f;
		pivot.m_z -= linkSize.m_z * 0.5f;
		dMatrix pinAndPivot (GetIdentityMatrix());
		pinAndPivot.m_posit = pivot;
		joint = new CustomHinge (pinAndPivot, link0, link1);

		link0 = link1;
		location.m_posit.m_z += linkSize.m_z;
	}

	dVector pivot (location.m_posit);
	pivot.m_y += linkSize.m_y * 0.5f;
	pivot.m_z -= linkSize.m_z * 0.5f;
	dMatrix pinAndPivot (GetIdentityMatrix());
	pinAndPivot.m_posit = pivot;


	location.m_posit.m_z += boxSize.m_z * 0.5f - linkSize.m_z * 0.5f;
	location.m_posit.m_y += linkSize.m_y * 0.5f - boxSize.m_y * 0.5f;

	visualObject = new RenderPrimitive (location, boxGeometry);
	system->AddModel___ (visualObject);
	visualObject->Release();

	link1 = NewtonCreateBody (nWorld, boxCollision);
	NewtonBodySetMaterialGroupID (link1, defaultID);
	NewtonBodySetUserData (link1, visualObject);

	// set a destructor for this rigid body
	NewtonBodySetDestructorCallback (link1, PhysicsBodyDestructor);
	NewtonBodySetTransformCallback (link1, PhysicsSetTransform);
	NewtonBodySetMatrix (link1, &location[0][0]);
	PhysicsSetTransform (link1, &location[0][0], 0);

	joint = new CustomHinge (pinAndPivot, link0, link1);

	// release the collision geometry when not need it
	boxGeometry->Release();
	linkGeometry->Release();

	// release visual geometries
	NewtonReleaseCollision (nWorld, boxCollision);
	NewtonReleaseCollision (nWorld, linkCollision);
*/
}



static void AddGears (dVector position, NewtonWorld* nWorld, SceneManager* system)
{
_ASSERTE (0);
/*

	dFloat Ixx;
	dFloat Iyy;
	dFloat Izz;
	dFloat mass;
	dFloat witdh;
	dFloat radius;
	dFloat smallRadius;
	NewtonBody* maryPopy;
	NewtonBody* centerCicle;
	NewtonCollision* collision;

	witdh = 0.25f;
	radius = 4.0f;
	smallRadius = radius * 0.5f; 
	dMatrix location (dRollMatrix(3.141592f * 0.5f));

	location.m_posit = position;
	location.m_posit.m_w = 1.0f;
	location.m_posit.m_y = FindFloor (nWorld, location.m_posit.m_x, location.m_posit.m_z) + 2.0f;

	{
		// ////////////////////////////////////////////////////////////////////////////////////
		//
		// add a big cylinder fixed the wold by a hinge
		//
		// ////////////////////////////////////////////////////////////////////////////////////

		CustomHinge* joint;
		RenderPrimitive* visualObject;
		dMatrix beatLocation (location);

		// create a collision primitive to be shared by all links
#ifdef	USE_SMOOTH_SHAPE
		collision = NewtonCreateChamferCylinder (nWorld, radius, witdh, 0, NULL); 
#else
		collision = NewtonCreateCylinder (nWorld, radius, witdh, 0, NULL); 
#endif


		// create the a graphic character (use a visualObject as our body
		//visualObject = new BoxPrimitive (beatLocation, beatSize);
		OGLMesh* visualGeometry;
		visualGeometry = new OGLMesh("Wheel_2", collision, "camo.tga", "camo.tga", "camo.tga");
		visualObject = new RenderPrimitive (beatLocation, visualGeometry);
		system->AddModel___ (visualObject);
		visualObject->Release();
		visualGeometry->Release();

		maryPopy = NewtonCreateBody(nWorld, collision);
		NewtonReleaseCollision (nWorld, collision);

		// attach graphic object to the rigid body
		NewtonBodySetUserData(maryPopy, visualObject);

		// set a destructor function
		NewtonBodySetDestructorCallback (maryPopy, PhysicsBodyDestructor);

		// set the transform call back function
		NewtonBodySetTransformCallback (maryPopy, PhysicsSetTransform);

		// set the force and torque call back function
		NewtonBodySetForceAndTorqueCallback (maryPopy,PhysicsApplyGravityForce);

		// calculate a accurate moment of inertia
		dVector origin;
		dVector inertia;
		NewtonConvexCollisionCalculateInertialMatrix (collision, &inertia[0], &origin[0]);	

		mass = 5.0f;
		Ixx = mass * inertia[0];
		Iyy = mass * inertia[1];
		Izz = mass * inertia[2];


		// set the mass matrix
		NewtonBodySetMassMatrix (maryPopy, mass, Ixx, Iyy, Izz);

		// set the matrix for both the rigid body and the graphic body
		NewtonBodySetMatrix (maryPopy, &beatLocation[0][0]);
		PhysicsSetTransform (maryPopy, &beatLocation[0][0], 0);


		// set the pivot relative for the first bar
		//		 dVector pivot (beatLocation.m_posit); 
		//		 dVector pin (beatLocation.m_front);
		const dMatrix& pinAndPivot = beatLocation;
		joint = new CustomHinge (pinAndPivot, maryPopy, NULL);
	}


	{
		// ////////////////////////////////////////////////////////////////////////////////////
		//
		// add handle satellite 
		//
		// ////////////////////////////////////////////////////////////////////////////////////

		NewtonCustomJoint* joint;
		RenderPrimitive* visualObject;
		dMatrix beatLocation (GetIdentityMatrix());
		beatLocation.m_posit = location.m_posit;
		beatLocation.m_posit.m_z += 0.0f;
		beatLocation.m_posit.m_y += 1.5f;
		beatLocation.m_posit.m_x += 8.0f;

		// create a collision primitive to be shared by all links
#ifdef USE_SMOOTH_SHAPE
		collision = NewtonCreateChamferCylinder (nWorld, smallRadius, witdh, 0, NULL); 
#else
		collision = NewtonCreateCylinder (nWorld, smallRadius, witdh, 0, NULL); 
#endif


		// create the a graphic character (use a visualObject as our body
		OGLMesh* visualGeometry;
		visualGeometry = new OGLMesh("Wheel_3", collision, "camo.tga", "camo.tga", "camo.tga");
		visualObject = new RenderPrimitive (beatLocation, visualGeometry);
		system->AddModel___ (visualObject);
		visualObject->Release();
		visualGeometry->Release();

		centerCicle = NewtonCreateBody(nWorld, collision);
		NewtonReleaseCollision (nWorld, collision);

		// attach graphic object to the rigid body
		NewtonBodySetUserData(centerCicle, visualObject);

		// set a destructor function
		NewtonBodySetDestructorCallback (centerCicle, PhysicsBodyDestructor);

		// set the transform call back function
		NewtonBodySetTransformCallback (centerCicle, PhysicsSetTransform);

		// set the force and torque call back function
		NewtonBodySetForceAndTorqueCallback (centerCicle,PhysicsApplyGravityForce);

		// calculate a accurate moment of inertia
		dVector origin;
		dVector inertia;
		NewtonConvexCollisionCalculateInertialMatrix (collision, &inertia[0], &origin[0]);	

		mass = 5.0f * 0.5f;
		Ixx = mass * inertia[0];
		Iyy = mass * inertia[1];
		Izz = mass * inertia[2];

		// set the mass matrix
		NewtonBodySetMassMatrix (centerCicle, mass, Ixx, Iyy, Izz);

		// set the matrix for both the rigid body and the graphic body
		NewtonBodySetMatrix (centerCicle, &beatLocation[0][0]);
		PhysicsSetTransform (centerCicle, &beatLocation[0][0], 0);


		// set the pivot relative for the first bar
		//		 dVector pivot (beatLocation.m_posit); 
		//		 dVector pin (beatLocation.m_front);
		//		 joint = new CustomHinge (pivot, pin, centerCicle, NULL);
		const dMatrix& pinAndPivot = beatLocation;
		joint = new CustomHinge (pinAndPivot,  centerCicle, NULL);

#ifdef ADD_GEARS
		// connect the top and the control gear
		dVector pin (beatLocation.m_front);
		joint = new CustomGear (2.0f, pin, location.m_front, centerCicle, maryPopy);
#endif
	}


	{
		// ////////////////////////////////////////////////////////////////////////////////////
		//
		// add a the center cylinder
		//
		// ////////////////////////////////////////////////////////////////////////////////////
		NewtonCustomJoint* joint;
		RenderPrimitive* visualObject;
		dMatrix beatLocation (location);
		beatLocation.m_posit.m_y += witdh;

		// create a collision primitive to be shared by all links
#ifdef USE_SMOOTH_SHAPE
		collision = NewtonCreateChamferCylinder (nWorld, smallRadius, witdh, 0, NULL); 
#else
		collision = NewtonCreateCylinder (nWorld, smallRadius, witdh, 0, NULL); 
#endif

		OGLMesh* visualGeometry;
		visualGeometry = new OGLMesh("Wheel_4", collision, "camo.tga", "camo.tga", "camo.tga");
		visualObject = new RenderPrimitive (beatLocation, visualGeometry);
		system->AddModel___ (visualObject);
		visualObject->Release();
		visualGeometry->Release();

		centerCicle = NewtonCreateBody(nWorld, collision);
		NewtonReleaseCollision (nWorld, collision);

		// attach graphic object to the rigid body
		NewtonBodySetUserData(centerCicle, visualObject);

		// set a destructor function
		NewtonBodySetDestructorCallback (centerCicle, PhysicsBodyDestructor);

		// set the transform call back function
		NewtonBodySetTransformCallback (centerCicle, PhysicsSetTransform);

		// set the force and torque call back function
		NewtonBodySetForceAndTorqueCallback (centerCicle,PhysicsApplyGravityForce);

		// calculate a accurate moment of inertia
		dVector origin;
		dVector inertia;
		NewtonConvexCollisionCalculateInertialMatrix (collision, &inertia[0], &origin[0]);	

		mass = 5.0f * 0.5f;
		Ixx = mass * inertia[0];
		Iyy = mass * inertia[1];
		Izz = mass * inertia[2];

		// set the mass matrix
		NewtonBodySetMassMatrix (centerCicle, mass, Ixx, Iyy, Izz);

		// set the matrix for both the rigid body and the graphic body
		NewtonBodySetMatrix (centerCicle, &beatLocation[0][0]);
		PhysicsSetTransform (centerCicle, &beatLocation[0][0], 0);


		// set the pivot relative for the first bar
		//		 dVector pivot (beatLocation.m_posit); 
		//		 dVector pin (beatLocation.m_front);
		//		 joint = new CustomHinge (pivot, pin, centerCicle, maryPopy);
		const dMatrix& pinAndPivot = beatLocation;
		joint = new CustomHinge (pinAndPivot, centerCicle, maryPopy);

#ifdef ADD_GEARS
		// connect the marypopy and the center circle with a gear joint
		dVector pin (beatLocation.m_front);
		joint = new CustomGear (-2.0f * radius / smallRadius, pin, location.m_front, centerCicle, maryPopy);
#endif
	}


	const int satelliteCount = 5;
	{
		OGLMesh* visualGeometry;
		// create a collision primitive to be shared by all links
#ifdef USE_SMOOTH_SHAPE
		collision = NewtonCreateChamferCylinder (nWorld, smallRadius, witdh, 0, NULL); 
#else
		collision = NewtonCreateCylinder (nWorld, smallRadius, witdh, 0, NULL); 
#endif

		visualGeometry = new OGLMesh("gear", collision, "camo.tga", "camo.tga", "camo.tga");
		for (int i = 0; i < satelliteCount; i ++) {
			// ////////////////////////////////////////////////////////////////////////////////////
			//
			// add a satellite disk
			//
			// ////////////////////////////////////////////////////////////////////////////////////

			NewtonBody* sattelite;
			NewtonCustomJoint* joint;
			RenderPrimitive* visualObject;
			dMatrix beatLocation (location);

			dMatrix rot (dYawMatrix((dFloat(i) / satelliteCount) * (2.0f * 3.141592f)));
			beatLocation.m_posit += rot.RotateVector(dVector ((smallRadius + 0.1f) * 2.0f, 0.0f, 0.0f));
			beatLocation.m_posit.m_y += witdh;

			// create the a graphic character (use a visualObject as our body
			//visualObject = new BoxPrimitive (beatLocation, beatSize);
			//			visualObject = new ChamferCylinderPrimitive (parent, beatLocation, smallRadius, witdh, "camo.tga");
			visualObject = new RenderPrimitive (beatLocation, visualGeometry);
			system->AddModel___ (visualObject);
			visualObject->Release();


			// create a collision primitive to be shared by all links
			//			collision = NewtonCreateChamferCylinder (nWorld, smallRadius, witdh, NULL); 
			sattelite = NewtonCreateBody(nWorld, collision);

			// attach graphic object to the rigid body
			NewtonBodySetUserData(sattelite, visualObject);

			// set a destructor function
			NewtonBodySetDestructorCallback (sattelite, PhysicsBodyDestructor);

			// set the transform call back function
			NewtonBodySetTransformCallback (sattelite, PhysicsSetTransform);

			// set the force and torque call back function
			NewtonBodySetForceAndTorqueCallback (sattelite, PhysicsApplyGravityForce);

			// calculate a accurate moment of inertia
			dVector origin;
			dVector inertia;
			NewtonConvexCollisionCalculateInertialMatrix (collision, &inertia[0], &origin[0]);	

			mass = 5.0f * 0.5f;
			Ixx = mass * inertia[0];
			Iyy = mass * inertia[1];
			Izz = mass * inertia[2];


			// set the mass matrix
			NewtonBodySetMassMatrix (sattelite, mass, Ixx, Iyy, Izz);

			// set the matrix for both the rigid body and the graphic body
			NewtonBodySetMatrix (sattelite, &beatLocation[0][0]);
			PhysicsSetTransform (sattelite, &beatLocation[0][0], 0);


			// set the pivot relative for the first bar
			//			dVector pivot (beatLocation.m_posit); 
			//			dVector pin (beatLocation.m_front);
			//			joint = new CustomHinge (pivot, pin, sattelite, maryPopy);

			const dMatrix& pinAndPivot = beatLocation;
			joint = new CustomHinge (pinAndPivot, sattelite, maryPopy);

			// connect the satellite and the center circle with a gear joint
#ifdef ADD_GEARS
			dVector pin (beatLocation.m_front);
			joint = new CustomGear (smallRadius / smallRadius, pin, location.m_front, sattelite, centerCicle);
#endif
		}

		visualGeometry->Release();
		NewtonReleaseCollision (nWorld, collision);
	}
*/
}



// these joint help on removing a singularity in the mass matrix when one of the tow claw of the joint hit one limit
// that the time the relation joint is almost collinear with the  joint limit and the hight acceleration lead to incorrect solution.
// the exact solve can deal with this cases but a linearized version do so not perform enough iterations to realize that at this 
// time the relational joint constraint is redundant.
// The best solution is to just no submit the joint when the control object hit a limit. 
class ViseWornGear: public CustomWormGear
{
	public: 
	ViseWornGear (dFloat gearRatio, const dVector& childPin, const dVector& parentPin, NewtonBody* child, NewtonBody* parent, CustomSlider* slider)
		:CustomWormGear (gearRatio, childPin, parentPin, child, parent)
	{
		m_myControlledSlider = slider;

	}

	void SubmitConstraints (dFloat timestep, int threadIndex)
	{
		if (!m_myControlledSlider->JoinHitLimit()) {
			CustomWormGear::SubmitConstraints (timestep, threadIndex);
		}
	}

	CustomSlider* m_myControlledSlider;
};

class VisePully: public CustomPulley
{
	public: 
	VisePully (dFloat pulleyRatio, const dVector& childPin, const dVector& parentPin, NewtonBody* parenPin, NewtonBody* parent, CustomSlider* slider0, CustomSlider* slider1)
		: CustomPulley (pulleyRatio, childPin, parentPin, parenPin, parent)
	{
		m_myControlledSlider0 = slider0;
		m_myControlledSlider1 = slider1;

	}

	void SubmitConstraints (dFloat timestep, int threadIndex)
	{
		if (! (m_myControlledSlider0->JoinHitLimit() || m_myControlledSlider1->JoinHitLimit())) {
			CustomPulley::SubmitConstraints (timestep, threadIndex);
		}
	}

	CustomSlider* m_myControlledSlider0;
	CustomSlider* m_myControlledSlider1;

};

static void AddVise (dVector position, NewtonWorld* nWorld, SceneManager* system)
{
_ASSERTE (0);
/*

	dFloat Ixx;
	dFloat Iyy;
	dFloat Izz;
	dFloat mass;
	NewtonBody* viseBody;
	NewtonBody* viseHandle;
	NewtonBody* viserRightClaw;
	NewtonBody* viserLeftClaw;
	NewtonCollision* collision;

	dVector size (2, 3, 8);
	dMatrix location (GetIdentityMatrix());
	location.m_posit = position;
	location.m_posit.m_w = 1.0f;
	location.m_posit.m_y = FindFloor (nWorld, location.m_posit.m_x, location.m_posit.m_z) + 2.0f + size.m_y * 0.5f;
	{
		// ////////////////////////////////////////////////////////////////////////////////////
		//
		// add a big block for the base of the vise
		//
		// ////////////////////////////////////////////////////////////////////////////////////

		CustomHinge* joint;
		RenderPrimitive* visualObject;
		dMatrix beatLocation (location);

		// create a collision primitive to be shared by all links
		collision = NewtonCreateBox (nWorld, size.m_x, size.m_y, size.m_z, 0, NULL); 


		// create the a graphic character (use a visualObject as our body
		//	 visualObject = new BoxPrimitive (parent, beatLocation, size, "camo.tga");
		// create the a graphic character (use a visualObject as our body)
		OGLMesh* visualGeometry;
		visualGeometry = new OGLMesh ("box4", collision, "camo.tga", "camo.tga", "camo.tga");
		visualObject = new RenderPrimitive (beatLocation, visualGeometry);
		system->AddModel___ (visualObject);
		visualObject->Release();

		visualGeometry->Release();

		viseBody = NewtonCreateBody(nWorld, collision);
		NewtonReleaseCollision (nWorld, collision);

		// attach graphic object to the rigid body
		NewtonBodySetUserData(viseBody, visualObject);

		// set a destructor function
		NewtonBodySetDestructorCallback (viseBody, PhysicsBodyDestructor);

		// set the transform call back function
		NewtonBodySetTransformCallback (viseBody, PhysicsSetTransform);

		// set the force and torque call back function
		NewtonBodySetForceAndTorqueCallback (viseBody,PhysicsApplyGravityForce);

		// calculate a accurate moment of inertia
		dVector origin;
		dVector inertia;
		NewtonConvexCollisionCalculateInertialMatrix (collision, &inertia[0], &origin[0]);	

		mass = 5.0f;
		Ixx = mass * inertia[0];
		Iyy = mass * inertia[1];
		Izz = mass * inertia[2];

		// set the mass matrix
		NewtonBodySetMassMatrix (viseBody, mass, Ixx, Iyy, Izz);

		// set the matrix for both the rigid body and the graphic body
		NewtonBodySetMatrix (viseBody, &beatLocation[0][0]);
		PhysicsSetTransform (viseBody, &beatLocation[0][0], 0);

		// set the pivot relative for the first bar
		dMatrix pinAndPivot (GetIdentityMatrix());
		pinAndPivot.m_front = location.m_up;
		pinAndPivot.m_up = location.m_front;
		pinAndPivot.m_right = pinAndPivot.m_front * pinAndPivot.m_up;
		pinAndPivot.m_posit = location.m_posit;
		joint = new CustomHinge (pinAndPivot, viseBody, NULL);
	}


	{
		// ////////////////////////////////////////////////////////////////////////////////////
		//
		// add vise handle  
		//
		// ////////////////////////////////////////////////////////////////////////////////////
		NewtonCustomJoint* joint;
		RenderPrimitive* visualObject;
		dMatrix beatLocation (GetIdentityMatrix());

		dFloat radius = size.m_y * 0.45f;
		dFloat width = size.m_x * 0.5f;

		beatLocation.m_posit = location.m_posit;
		beatLocation.m_posit.m_x -= ((size.m_x * 0.5f) + (width * 0.6f));

		// create a collision primitive to be shared by all links
#ifdef USE_SMOOTH_SHAPE
		collision = NewtonCreateChamferCylinder (nWorld, radius, width, 0, NULL); 
#else
		collision = NewtonCreateCylinder (nWorld, radius, width, 0, NULL); 
#endif

		// create the a graphic character (use a visualObject as our body
		OGLMesh* visualGeometry;
		visualGeometry = new OGLMesh("wheel_5", collision, "camo.tga", "camo.tga", "camo.tga");
		visualObject = new RenderPrimitive (beatLocation, visualGeometry);
		system->AddModel___ (visualObject);
		visualObject->Release();

		visualGeometry->Release();

		viseHandle = NewtonCreateBody(nWorld, collision);
		NewtonReleaseCollision (nWorld, collision);

		// attach graphic object to the rigid body
		NewtonBodySetUserData(viseHandle, visualObject);

		// set a destructor function
		NewtonBodySetDestructorCallback (viseHandle, PhysicsBodyDestructor);

		// set the transform call back function
		NewtonBodySetTransformCallback (viseHandle, PhysicsSetTransform);

		// set the force and torque call back function
		NewtonBodySetForceAndTorqueCallback (viseHandle,PhysicsApplyGravityForce);

		// calculate a accurate moment of inertia
		dVector origin;
		dVector inertia;
		NewtonConvexCollisionCalculateInertialMatrix (collision, &inertia[0], &origin[0]);	

		mass = 1.0f;
		// use a spherical inertia here, to reduce wobbling due gyroscope side effect
		float II = max (inertia[0], max (inertia[1], inertia[2]));
		Ixx = mass * II;
		Iyy = mass * II;
		Izz = mass * II;

		// set the mass matrix
		NewtonBodySetMassMatrix (viseHandle, mass, Ixx, Iyy, Izz);

		// set the matrix for both the rigid body and the graphic body
		NewtonBodySetMatrix (viseHandle, &beatLocation[0][0]);
		PhysicsSetTransform (viseHandle, &beatLocation[0][0], 0);

		// set the pivot relative for the first bar
		dMatrix pinAndPivot (beatLocation);
		joint = new CustomHinge (pinAndPivot, viseHandle, viseBody);
	}

	CustomSlider* otherSlider;
	{
		// ////////////////////////////////////////////////////////////////////////////////////
		//
		// add right vise side
		//
		// ////////////////////////////////////////////////////////////////////////////////////

		RenderPrimitive* visualObject;
		dMatrix beatLocation (location);
		dVector viseSize (4.0f, 2.0f, 0.5f);
		beatLocation.m_posit.m_y += (size.m_y * 0.5f + viseSize.m_y * 0.5f);
		beatLocation.m_posit.m_z += (size.m_z * 0.5f - viseSize.m_z * 0.5f); 


		// create a collision primitive to be shared by all links
		collision = NewtonCreateBox (nWorld, viseSize.m_x, viseSize.m_y, viseSize.m_z, 0, NULL); 

		// create the a graphic character (use a visualObject as our body
		OGLMesh* visualGeometry;
		visualGeometry = new OGLMesh ("box_5", collision, "camo.tga", "camo.tga", "camo.tga");
		visualObject = new RenderPrimitive (beatLocation, visualGeometry);
		system->AddModel___ (visualObject);
		visualObject->Release();

		visualGeometry->Release();

		viserRightClaw = NewtonCreateBody(nWorld, collision);
		NewtonReleaseCollision (nWorld, collision);

		// attach graphic object to the rigid body
		NewtonBodySetUserData(viserRightClaw, visualObject);

		// set a destructor function
		NewtonBodySetDestructorCallback (viserRightClaw, PhysicsBodyDestructor);

		// set the transform call back function
		NewtonBodySetTransformCallback (viserRightClaw, PhysicsSetTransform);

		// set the force and torque call back function
		NewtonBodySetForceAndTorqueCallback (viserRightClaw,PhysicsApplyGravityForce);

		// calculate a accurate moment of inertia
		dVector origin;
		dVector inertia;
		NewtonConvexCollisionCalculateInertialMatrix (collision, &inertia[0], &origin[0]);	

		mass = 5.0f * 0.5f;
		Ixx = mass * inertia[0];
		Iyy = mass * inertia[1];
		Izz = mass * inertia[2];

		// set the mass matrix
		NewtonBodySetMassMatrix (viserRightClaw, mass, Ixx, Iyy, Izz);

		// set the matrix for both the rigid body and the graphic body
		NewtonBodySetMatrix (viserRightClaw, &beatLocation[0][0]);
		PhysicsSetTransform (viserRightClaw, &beatLocation[0][0], 0);

		// set the pivot relative for the first bar
		CustomSlider* slider;
		dMatrix pinAndPivot (GetIdentityMatrix());
		pinAndPivot.m_front = beatLocation.m_right;
		pinAndPivot.m_up = location.m_front;
		pinAndPivot.m_right = pinAndPivot.m_front * pinAndPivot.m_up;
		pinAndPivot.m_posit = beatLocation.m_posit;
		slider = new CustomSlider (pinAndPivot, viserRightClaw, viseBody);

		slider->EnableLimits(true);
		slider->SetLimis(-(size.m_z - viseSize.m_z * 2.5f) * 0.5f, 0.0f);

		otherSlider = slider;

		// connect the vise handle with right claw
#ifdef ADD_GEARS
		dMatrix handleMatrix;
		NewtonCustomJoint* joint;
		NewtonBodyGetMatrix(viseHandle, &handleMatrix[0][0]);
		joint = new ViseWornGear (4.0f, handleMatrix.m_front, location.m_right, viseHandle, viserRightClaw, slider);
#endif
	}


	{
		// ////////////////////////////////////////////////////////////////////////////////////
		//
		// add left vise side
		//
		// ////////////////////////////////////////////////////////////////////////////////////

		RenderPrimitive* visualObject;
		dMatrix beatLocation (location);
		dVector viseSize (4.0f, 2.0f, 0.5f);
		beatLocation.m_posit.m_y += (size.m_y * 0.5f + viseSize.m_y * 0.5f);
		beatLocation.m_posit.m_z -= (size.m_z * 0.5f - viseSize.m_z * 0.5f); 

		// create a collision primitive to be shared by all links
		collision = NewtonCreateBox (nWorld, viseSize.m_x, viseSize.m_y, viseSize.m_z, 0, NULL); 

		// create the a graphic character (use a visualObject as our body
		OGLMesh* visualGeometry;
		visualGeometry = new OGLMesh ("box_5", collision, "camo.tga", "camo.tga", "camo.tga");
		visualObject = new RenderPrimitive (beatLocation, visualGeometry);
		system->AddModel___ (visualObject);
		visualObject->Release();

		visualGeometry->Release();

		viserLeftClaw = NewtonCreateBody(nWorld, collision);
		NewtonReleaseCollision (nWorld, collision);

		// attach graphic object to the rigid body
		NewtonBodySetUserData(viserLeftClaw, visualObject);

		// set a destructor function
		NewtonBodySetDestructorCallback (viserLeftClaw, PhysicsBodyDestructor);

		// set the transform call back function
		NewtonBodySetTransformCallback (viserLeftClaw, PhysicsSetTransform);

		// set the force and torque call back function
		NewtonBodySetForceAndTorqueCallback (viserLeftClaw,PhysicsApplyGravityForce);

		// calculate a accurate moment of inertia
		dVector origin;
		dVector inertia;
		NewtonConvexCollisionCalculateInertialMatrix (collision, &inertia[0], &origin[0]);	

		mass = 5.0f * 0.5f;
		Ixx = mass * inertia[0];
		Iyy = mass * inertia[1];
		Izz = mass * inertia[2];

		// set the mass matrix
		NewtonBodySetMassMatrix (viserLeftClaw, mass, Ixx, Iyy, Izz);

		// set the matrix for both the rigid body and the graphic body
		NewtonBodySetMatrix (viserLeftClaw, &beatLocation[0][0]);
		PhysicsSetTransform (viserLeftClaw, &beatLocation[0][0], 0);


		// set the pivot relative for the first bar
		CustomSlider* slider;
//		 dVector pivot (beatLocation.m_posit); 
//		 dVector pin (beatLocation.m_right);
		dMatrix pinAndPivot (GetIdentityMatrix());
		pinAndPivot.m_front = beatLocation.m_right;
		pinAndPivot.m_up = location.m_front;
		pinAndPivot.m_right = pinAndPivot.m_front * pinAndPivot.m_up;
		pinAndPivot.m_posit = beatLocation.m_posit;
		slider = new CustomSlider (pinAndPivot, viserLeftClaw, viseBody);

		slider->EnableLimits(true);
		slider->SetLimis(0.0f, size.m_z * 0.5f);

#ifdef ADD_GEARS
//		 dMatrix viserRightClawMatrix;
//		 NewtonCustomJoint* joint;
//		 NewtonBodyGetMatrix(viserRightClaw, &viserRightClawMatrix[0][0]);
//		 joint = new VisePully (1.0f, viserRightClawMatrix.m_right, location.m_right, viserRightClaw, viserLeftClaw, slider, otherSlider);

		dMatrix handleMatrix;
		NewtonCustomJoint* joint;
		NewtonBodyGetMatrix(viseHandle, &handleMatrix[0][0]);
		joint = new ViseWornGear (4.0f, handleMatrix.m_front.Scale (-1.0f), location.m_right, viseHandle, viserLeftClaw, slider);

#endif
	}
*/
}



void BasicCustomJoints (NewtonFrame& system)
{
	NewtonWorld* world;
	NewtonBody* floor; 
	world = system.m_world;

	// create the sky box and the floor,
	floor = BuildFloorAndSceneRoot (system);

	AddRope (dVector (0.0f, 0.0f, -15.0f, 0.0f), system.m_world, &system, floor, 30);
	AddDoubleSwingDoors (dVector (0.0f, 0.0f, 0.0f, 0.0f), system.m_world, &system, floor, 5);
	AddRollingBeats (dVector (0.0f, 0.0f, 15.0f, 0.0f), system.m_world, &system, floor);
	AddHangingBridge (dVector (-10.0f, 0.0f, 0.0f, 0.0f), system.m_world, &system, 50);  
	AddGears (dVector (-20.0f, 0.0f, -20.0f, 0.0f), system.m_world, &system);
	AddVise (dVector (-20.0f, 0.0f, 0.0f, 0.0f), system.m_world, &system);
}

