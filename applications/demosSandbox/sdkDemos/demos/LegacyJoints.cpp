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
static void AddRope (dVector position, NewtonWorld* nWorld, SceneManager* system, const NewtonBody* worldBody)
{
_ASSERTE (0);
/*

	int i;
	int count;
	int defaultID;
	dFloat mass;
	dFloat Ixx;
	dFloat Iyy;
	dFloat Izz;
	NewtonJoint* joint;
	const NewtonBody* link0;
	const NewtonBody* link1;
	OGLMesh* meshInstance;
	NewtonCollision* collision;
	RenderPrimitive* visualObject;

	count = 7;
	defaultID = NewtonMaterialGetDefaultGroupID (nWorld);

	dVector size (2.0f, 0.25f, 0.25f);

	//dMatrix location (GetIdentityMatrix());
	dMatrix location (dRollMatrix(3.141592f * 0.5f));
	location.m_posit = position; 
	location.m_posit.m_y = FindFloor (nWorld, location.m_posit.m_x, location.m_posit.m_z) + size.m_x * count + 1.0f;

	// create a collision primitive to be shared by all links
	collision = NewtonCreateCapsule (nWorld, size.m_y, size.m_x, 0, NULL);

	// calculate a accurate moment of inertia
	dVector origin;
	dVector inertia;
	NewtonConvexCollisionCalculateInertialMatrix (collision, &inertia[0], &origin[0]);	

	mass = 2.0f;
	Ixx = mass * inertia[0];
	Iyy = mass * inertia[1];
	Izz = mass * inertia[2];

	link0 = worldBody;
	meshInstance = new OGLMesh ("ropeLink", collision, "wood_0.tga", "wood_0.tga", "wood_1.tga");

	for (i = 0; i < count; i ++) {
		// create the a graphic character (use a visualObject as our body
		visualObject = new RenderPrimitive (location, meshInstance);
		system->AddModel___ (visualObject);
		visualObject->Release();

		//create the rigid body
		link1 = NewtonCreateBody (nWorld, collision);

		// add some damping to each link
		NewtonBodySetLinearDamping (link1, 0.2f);
		dVector angularDamp (0.2f, 0.2f, 0.2f);
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
		NewtonBodySetForceAndTorqueCallback (link1, PhysicsApplyGravityForce);

		// set the mass matrix
		NewtonBodySetMassMatrix (link1, mass, Ixx, Iyy, Izz);

		// set the matrix for both the rigid body and the graphic body
		NewtonBodySetMatrix (link1, &location[0][0]);
		PhysicsSetTransform (link1, &location[0][0], 0);

		dVector pivot (location.m_posit);
		pivot.m_y += (size.m_x - size.m_y) * 0.5f;


		// connect these two bodies by a ball and socket joint
		joint = NewtonConstraintCreateBall (nWorld, &pivot.m_x, link1, link0);


		// override the stiffness, 
		// adjust this parameter to see foe the joint become more or less strong and stable 
		NewtonJointSetStiffness (joint, 0.9f);
		//dFloat stiffness = NewtonJointGetStiffness (joint);

		// set a twist angle to prevent the links to spin unrealistically
		dVector pin (location.m_front.Scale (-1.0f));
		NewtonBallSetConeLimits (joint, &pin.m_x, 0.0f, 10.0f * 3.141592f / 180.0f);

		link0 = link1;
		location.m_posit.m_y -= (size.m_x - size.m_y);
	}

	// release the collision geometry when not need it
	meshInstance->Release();
	NewtonReleaseCollision (nWorld, collision);
*/
}


static unsigned DoubleDoorUserCallback (const NewtonJoint* hinge, NewtonHingeSliderUpdateDesc* desc)
{
	dFloat angle;
	const dFloat angleLimit = 90.0f;

	angle = NewtonHingeGetJointAngle (hinge);
	if (angle > ((angleLimit / 180.0f) * 3.141592f)) {
		// if the joint angle is large than the predefine interval, stop the hinge
		desc->m_accel = NewtonHingeCalculateStopAlpha (hinge, desc, (angleLimit / 180.0f) * 3.141592f);
		desc->m_maxFriction = 0.0f;
		return 1;
	} else if (angle < ((-angleLimit / 180.0f) * 3.141592f)) {
		// if the joint angle is large than the predefine interval, stop the hinge
		desc->m_accel = NewtonHingeCalculateStopAlpha (hinge, desc, (-angleLimit / 180.0f) * 3.141592f);
		desc->m_minFriction = 0.0f;
		return 1;
	}

	// no action need it if the joint angle is with the limits
	return 0;
}


static void AddDoubleSwingDoors (dVector position, NewtonWorld* nWorld, SceneManager* system, const NewtonBody* worldBody)
{
_ASSERTE (0);
/*

	int defaultID;
	dFloat mass;
	dFloat Ixx;
	dFloat Iyy;
	dFloat Izz;
	NewtonBody* link0;
	NewtonBody* link1;

	NewtonJoint* joint;
	OGLMesh* meshInstance;
	NewtonCollision* collision;
	RenderPrimitive* visualObject;
	dVector size (2.0f, 5.0f, 0.5f);

	defaultID = NewtonMaterialGetDefaultGroupID (nWorld);

	// create 100 tack of 10 boxes each
	dMatrix location (GetIdentityMatrix());
	location.m_posit = position; 
	location.m_posit.m_y = FindFloor (nWorld, location.m_posit.m_x, location.m_posit.m_z) + size.m_y;

	// create a collision primitive to be shared by all links
	collision = NewtonCreateBox (nWorld, size.m_x, size.m_y, size.m_z, 0, NULL); 

	// calculate a accurate moment of inertia
	dVector origin;
	dVector inertia;
	NewtonConvexCollisionCalculateInertialMatrix (collision, &inertia[0], &origin[0]);	

	mass = 2.0f;
	Ixx = mass * inertia[0];
	Iyy = mass * inertia[1];
	Izz = mass * inertia[2];

	meshInstance = new OGLMesh ("swingDoorwing", collision, "wood_0.tga", "wood_0.tga", "wood_1.tga");
	// make first wing
	{
		// create the a graphic character (use a visualObject as our body
		visualObject = new RenderPrimitive (location, meshInstance);
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
		NewtonBodySetForceAndTorqueCallback (link1, PhysicsApplyGravityForce);

		// set the mass matrix
		NewtonBodySetMassMatrix (link1, mass, Ixx, Iyy, Izz);

		// set the matrix for both the rigid body and the graphic body
		NewtonBodySetMatrix (link1, &location[0][0]);
		PhysicsSetTransform (link1, &location[0][0], 0);

		dVector pivot (location.m_posit);
		dVector pin (location.m_up);
		pivot.m_x += size.m_x * 0.5f;

		// connect these two bodies by a ball and sockect joint
		joint = NewtonConstraintCreateHinge (nWorld, &pivot.m_x, &pin.m_x, link1, worldBody);

		// set limit to the door
		NewtonHingeSetUserCallback (joint, DoubleDoorUserCallback);
	}


	// make second wing
	{
		location.m_posit.m_x -= size.m_x;

		// create the a graphic character (use a visualObject as our body
		visualObject = new RenderPrimitive (location, meshInstance);
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
		NewtonBodySetForceAndTorqueCallback (link0, PhysicsApplyGravityForce);

		// set the mass matrix
		NewtonBodySetMassMatrix (link0, mass, Ixx, Iyy, Izz);

		// set the matrix for both the rigid body and the graphic body
		NewtonBodySetMatrix (link0, &location[0][0]);
		PhysicsSetTransform (link0, &location[0][0], 0);

		dVector pivot (location.m_posit);
		dVector pin (location.m_up);
		pivot.m_x += size.m_x * 0.5f;

		// connect these two bodies by a ball and socket joint
		joint = NewtonConstraintCreateHinge (nWorld, &pivot.m_x, &pin.m_x, link0, link1);

		// set limit to the door
		NewtonHingeSetUserCallback (joint, DoubleDoorUserCallback);
	}

	// release the collision geometry when not need it
	meshInstance->Release();
	NewtonReleaseCollision (nWorld, collision);
*/
}


struct JointControlStruct
{
	dFloat m_min;
	dFloat m_max;
};

static void DestroyJointControlStruct(const NewtonJoint* slider)
{
	JointControlStruct* limits;
	limits = (JointControlStruct*) NewtonJointGetUserData (slider);
	delete limits; 
}
/*
static unsigned SliderUpdateEvent (const NewtonJoint* slider, NewtonHingeSliderUpdateDesc* desc)
{
	dFloat distance;
	JointControlStruct* limits;

	limits = (JointControlStruct*) NewtonJointGetUserData (slider);

	distance = NewtonSliderGetJointPosit (slider);
	if (distance < limits->m_min) {
		// if the distance is smaller than the predefine interval, stop the slider
		desc->m_accel = NewtonSliderCalculateStopAccel (slider, desc, limits->m_min);
		desc->m_minFriction = 0.0f;
		return 1;
	} else if (distance > limits->m_max) {
		// if the distance is larger than the predefine interval, stop the slider
		desc->m_accel = NewtonSliderCalculateStopAccel (slider, desc, limits->m_max);
		desc->m_maxFriction = 0.0f;
		return 1;
	}

	// no action need it if the joint angle is with the limits
	return 0;
}
*/
static unsigned CorkScrewUpdateEvent (const NewtonJoint* screw, NewtonHingeSliderUpdateDesc* desc)
{
	unsigned retCode; 
	dFloat distance;
	dFloat omega;
	JointControlStruct* limits;

	// no action need it if the joint angle is with the limits
	retCode = 0;

	limits = (JointControlStruct*) NewtonJointGetUserData (screw);

	// the first entry in NewtonHingeSliderUpdateDesc control the screw linear acceleration 
	distance = NewtonCorkscrewGetJointPosit (screw);
	if (distance < limits->m_min) {
		// if the distance is smaller than the predefine interval, stop the slider
		desc[0].m_accel = NewtonCorkscrewCalculateStopAccel (screw, &desc[0], limits->m_min);
		desc[0].m_minFriction = 0.0f;
		retCode |= 1;
	} else if (distance > limits->m_max) {
		// if the distance is larger than the predefine interval, stop the slider
		desc[0].m_accel = NewtonCorkscrewCalculateStopAccel (screw, &desc[0], limits->m_max);
		desc[0].m_maxFriction = 0.0f;
		retCode |= 1;
	}

	// the second entry in NewtonHingeSliderUpdateDesc control the screw angular acceleration. 
	// Make s small screw motor by setting the angular acceleration of the screw axis
	// We are not going to limit the angular rotation of the screw, but is we did we should or return code with 2,
	omega = NewtonCorkscrewGetJointOmega (screw);
//	desc[1].m_accel = 2.5f - 0.2f * omega;
	desc[1].m_accel = 0.5f * (-10.0f - omega) / desc[1].m_timestep;

	// or with 0x10 to tell newton this axis is active
	retCode |= 2;


	// return the code
	return retCode;

}


static unsigned UniversalUpdateEvent (const NewtonJoint* screw, NewtonHingeSliderUpdateDesc* desc)
{
	unsigned retCode; 
	dFloat angle;
	dFloat omega;
	JointControlStruct* limits;

	// no action need it if the joint angle is with the limits
	retCode = 0;

// change this to #if 0 put a double limit on the universal joint
#if 1
	// the first entry in is the axis fixes of the spinning beat
	// apply a simple motor to this object
	omega = NewtonUniversalGetJointOmega0 (screw);
	desc[0].m_accel = 0.25f * (10.0f - omega) / desc[0].m_timestep;
	// or with 0x10 to tell newton this axis is active
	retCode |= 1;


#else
	// put a top limit on the joint
	limits = (JointControlStruct*) NewtonJointGetUserData (screw);
	angle = NewtonUniversalGetJointAngle0 (screw);
	if (angle < limits->m_min) {
		// if the distance is smaller than the predefine interval, stop the slider
		desc[0].m_accel = NewtonUniversalCalculateStopAlpha0 (screw, &desc[0], limits->m_min);
		retCode |= 1;
	} else if (angle > limits->m_max) {
		// if the distance is larger than the predefine interval, stop the slider
		desc[0].m_accel = NewtonUniversalCalculateStopAlpha0 (screw, &desc[0], limits->m_max);
		retCode |= 1;
	}
#endif



	limits = (JointControlStruct*) NewtonJointGetUserData (screw);

	// the first entry in NewtonHingeSliderUpdateDesc control the screw linear acceleration 
	angle = NewtonUniversalGetJointAngle1 (screw);
	if (angle < limits->m_min) {
		// if the distance is smaller than the predefine interval, stop the slider
		desc[1].m_accel = NewtonUniversalCalculateStopAlpha1 (screw, &desc[0], limits->m_min);
		desc[1].m_maxFriction = 0.0f;
		retCode |= 2;
	} else if (angle > limits->m_max) {
		// if the distance is larger than the predefine interval, stop the slider
		desc[1].m_accel = NewtonUniversalCalculateStopAlpha1 (screw, &desc[0], limits->m_max);
		desc[1].m_minFriction = 0.0f;
		retCode |= 2;
	}
	
	// return the code
	return retCode;

}


static void AddRollingBeats (dVector position, NewtonWorld* nWorld, SceneManager* system, const NewtonBody* worldBody)
{
_ASSERTE (0);
/*

	dFloat mass;
	dFloat Ixx;
	dFloat Iyy;
	dFloat Izz;
	NewtonBody* bar;
	NewtonJoint* joint;
	NewtonCollision* collision;


	dMatrix location (GetIdentityMatrix());
	location.m_posit = position; 
	dVector size (10.0f, 0.25f, 0.25f);

	location.m_posit.m_y = FindFloor (nWorld, location.m_posit.m_x, location.m_posit.m_z) + 4.0f;

	// /////////////////////////////////////////////////////////////////////////////////////             
	//
	// create a bar and attach it to the world with a hinge with limits
	//
	// ////////////////////////////////////////////////////////////////////////////////////
	{
		OGLMesh* meshInstance;
		RenderPrimitive* visualObject;

		// create a collision primitive to be shared by all links
		collision = NewtonCreateCylinder (nWorld, size.m_y, size.m_x, 0, NULL); 

		// create the a graphic character (use a visualObject as our body
		meshInstance = new OGLMesh ("bar", collision, "wood_0.tga", "wood_0.tga", "wood_1.tga");
		visualObject = new RenderPrimitive (location, meshInstance);
		system->AddModel___ (visualObject);
		visualObject->Release();
		meshInstance->Release();

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
		NewtonBodySetForceAndTorqueCallback (bar, PhysicsApplyGravityForce);

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

		dVector pin (0.0f, 1.0f, 0.0f);
		dVector pivot (location.m_posit);
		pivot.m_x -= size.m_x * 0.5f;
		joint = NewtonConstraintCreateHinge (nWorld, &pivot.m_x, &pin.m_x, bar, worldBody);
		NewtonHingeSetUserCallback (joint, DoubleDoorUserCallback);
	}


#if 0
	{
		// ////////////////////////////////////////////////////////////////////////////////////
		//
		// add a sliding visualObject with limits
		//
		NewtonBody* beat;
		OGLMesh* meshInstance;
		RenderPrimitive* visualObject;
		dMatrix beatLocation (location);
		dVector beatSize (0.5f, 2.0f, 2.0f);

		beatLocation.m_posit.m_x += size.m_x * 0.25f;

		// create a collision primitive to be shared by all links
		collision = NewtonCreateBox (nWorld, beatSize.m_x, beatSize.m_y, beatSize.m_z, NULL); 

		// create the a graphic character (use a visualObject as our body
		meshInstance = new OGLMesh (collision, "wood_0.tga", "wood_0.tga", "wood_1.tga");
		visualObject = new RenderPrimitive (root, beatLocation, meshInstance);
		meshInstance->Release();


		beat = NewtonCreateBody(nWorld, collision);
		NewtonReleaseCollision (nWorld, collision);

			// attach graphic object to the rigid body
		NewtonBodySetUserData(beat, visualObject);

		// set a destructor function
		NewtonBodySetDestructorCallback (beat, PhysicsBodyDestructor);

		// set the transform call back function
		NewtonBodySetTransformCallback (beat, PhysicsSetTransform);

		// set the force and torque call back function
		NewtonBodySetForceAndTorqueCallback (beat, PhysicsApplyGravityForce);

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
		dVector pivot (beatLocation.m_posit); 
		dVector pin (beatLocation.m_front);
		joint = NewtonConstraintCreateSlider (nWorld, &pivot.m_x, &pin.m_x, beat, bar);

		// assign a control function for this joint
		NewtonHingeSetUserCallback (joint, SliderUpdateEvent);

		JointControlStruct *limits;
		limits = new JointControlStruct;

		// calculate the minimum and maximum limit for this joints
		limits->m_min = ((location.m_posit.m_x - beatLocation.m_posit.m_x) - size.m_x * 0.5f);
		limits->m_max = ((location.m_posit.m_x - beatLocation.m_posit.m_x) + size.m_x * 0.5f);

		// store local private data with this joint 
		NewtonJointSetUserData (joint, limits);

		// tell this joint to destroy its local private data when destroyed
		NewtonJointSetDestructor (joint, DestroyJointControlStruct);			
	}
#endif

	{
		// ////////////////////////////////////////////////////////////////////////////////////
		//
		// add a corkscrew visualObject with limits
		//
		// ////////////////////////////////////////////////////////////////////////////////////
		NewtonBody* beat;
		OGLMesh* meshInstance;
		RenderPrimitive* visualObject;
		dMatrix beatLocation (location);
		dVector beatSize (0.5f, 1.25f, 1.25f);

		beatLocation.m_posit.m_x -= size.m_x * 0.25f;

		// create a collision primitive to be shared by all links
		collision = NewtonCreateChamferCylinder (nWorld, beatSize.m_y, beatSize.m_x, 0, NULL); 

		// create the a graphic character (use a visualObject as our body
		//visualObject = new BoxPrimitive (beatLocation, beatSize);
		meshInstance = new OGLMesh ("canferCylinder1", collision, "wood_0.tga", "wood_0.tga", "wood_1.tga");
		visualObject = new RenderPrimitive (beatLocation, meshInstance);
		system->AddModel___ (visualObject);
		visualObject->Release();
		meshInstance->Release();


		beat = NewtonCreateBody(nWorld, collision);
		NewtonReleaseCollision (nWorld, collision);

			// attach graphic object to the rigid body
		NewtonBodySetUserData(beat, visualObject);

		// set a destructor function
		NewtonBodySetDestructorCallback (beat, PhysicsBodyDestructor);

		// set the transform call back function
		NewtonBodySetTransformCallback (beat, PhysicsSetTransform);

		// set the force and torque call back function
		NewtonBodySetForceAndTorqueCallback (beat, PhysicsApplyGravityForce);

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
		dVector pivot (beatLocation.m_posit); 
		dVector pin (beatLocation.m_front);
		joint = NewtonConstraintCreateCorkscrew (nWorld, &pivot.m_x, &pin.m_x, beat, bar);
		
		// assign a control function for this joint
		NewtonCorkscrewSetUserCallback (joint, CorkScrewUpdateEvent);

		JointControlStruct *limits;
		limits = new JointControlStruct;

		// calculate the minimum and maximum limit for this joints
		limits->m_min = ((location.m_posit.m_x - beatLocation.m_posit.m_x) - size.m_x * 0.5f);
		limits->m_max = ((location.m_posit.m_x - beatLocation.m_posit.m_x) + size.m_x * 0.5f);

		// store local private data with this joint 
		NewtonJointSetUserData (joint, limits);

		// tell this joint to destroy its local private data when destroyed
		NewtonJointSetDestructor (joint, DestroyJointControlStruct);			
	}

	{
		// ////////////////////////////////////////////////////////////////////////////////////
		//
		// add a universal joint visualObject with limits
		//
		// ////////////////////////////////////////////////////////////////////////////////////
		NewtonBody* beat;
		OGLMesh* meshInstance;
		RenderPrimitive* visualObject;
		dMatrix beatLocation (location);
		dVector beatSize (0.5f, 1.25f, 1.25f);

		beatLocation.m_posit.m_x -= size.m_x * 0.5f;

		// create a collision primitive to be shared by all links
		collision = NewtonCreateChamferCylinder (nWorld, beatSize.m_y, beatSize.m_x, 0, NULL); 

		// create the a graphic character (use a visualObject as our body
		//visualObject = new BoxPrimitive (beatLocation, beatSize);
		meshInstance = new OGLMesh ("canferCylinder2", collision, "wood_0.tga", "wood_0.tga", "wood_1.tga");
		visualObject = new RenderPrimitive (beatLocation, meshInstance);
		system->AddModel___ (visualObject);
		visualObject->Release();

		meshInstance->Release();

		beat = NewtonCreateBody(nWorld, collision);
		NewtonReleaseCollision (nWorld, collision);

		// attach graphic object to the rigid body
		NewtonBodySetUserData(beat, visualObject);

		// set a destructor function
		NewtonBodySetDestructorCallback (beat, PhysicsBodyDestructor);

		// set the transform call back function
		NewtonBodySetTransformCallback (beat, PhysicsSetTransform);

		// set the force and torque call back function
		NewtonBodySetForceAndTorqueCallback (beat, PhysicsApplyGravityForce);

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
		dVector pivot (beatLocation.m_posit); 
		dVector pin0 (beatLocation.m_front);
		dVector pin1 (beatLocation.m_up);
		joint = NewtonConstraintCreateUniversal (nWorld, &pivot.m_x, &pin0.m_x, &pin1.m_x, beat, bar);
		
		// assign a control function for this joint
		NewtonCorkscrewSetUserCallback (joint, UniversalUpdateEvent);

		JointControlStruct *limits;
		limits = new JointControlStruct;

		// calculate the minimum and maximum limit for this joints
		limits->m_min = -30.0f * 3.141592f/ 180.0f;
		limits->m_max = 30.0f * 3.141592f/ 180.0f;

		// store local private data with this joint 
		NewtonJointSetUserData (joint, limits);

		// tell this joint to destroy its local private data when destroyed
		NewtonJointSetDestructor (joint, DestroyJointControlStruct);			
	}
*/
}



void LegacyJoints (NewtonFrame& system)
{
	NewtonBody* floor; 
	NewtonWorld* world;
	world = system.m_world;

	// create the sky box and the floor,
	floor = BuildFloorAndSceneRoot (system);

	AddRope (dVector (0.0f, 0.0f, -15.0f, 0.0f), system.m_world, &system, floor);
	AddDoubleSwingDoors (dVector (0.0f, 0.0f, 0.0f, 0.0f), system.m_world, &system, floor);
	AddRollingBeats (dVector (0.0f, 0.0f, 15.0f, 0.0f), system.m_world, &system, floor);
}

