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


// robot servo controller
class ROBOT_COMPONENTS
{
	public:
	class RobotFrontWheel: public CustomBallAndSocket
	{
		public:
		RobotFrontWheel (const dMatrix& pinsAndPivoFrame, NewtonBody* box, NewtonBody* wheel)
			:CustomBallAndSocket (pinsAndPivoFrame, box, wheel)
		{
		}
	};


	class RobotRearWheel: public CustomHinge
	{
		public:
		RobotRearWheel (const dMatrix& pinsAndPivoFrame, NewtonBody* box, NewtonBody* wheel)
			:CustomHinge ( pinsAndPivoFrame, box, wheel)
		{
		}
	};


	static NewtonBody* CreateRobotBody (SceneManager* parent, NewtonWorld* nWorld, const dMatrix& matrix, const dVector& size)
	{
_ASSERTE (0);
return NULL;
/*

		NewtonBody* boxBody;
		NewtonCollision* boxCollision;

		// create the collision 
		boxCollision = NewtonCreateBox (nWorld, size.m_x, size.m_y, size.m_z, 0, NULL); 


		//create the rigid body
		boxBody = NewtonCreateBody (nWorld, boxCollision);

		// release the collision
		NewtonReleaseCollision (nWorld, boxCollision);


		// create a graphic box
		RenderPrimitive* box;
		OGLMesh* meshInstance;
		meshInstance = new OGLMesh ("robotBody", boxCollision, "wood_0.tga", "wood_0.tga", "wood_1.tga");
		box = new RenderPrimitive (matrix, meshInstance);
		parent->AddModel___ (box);
		box->Release();
		meshInstance->Release();

		int defaultID;
		defaultID = NewtonMaterialGetDefaultGroupID (nWorld);


		// use wood id for robot body
		NewtonBodySetMaterialGroupID (boxBody, defaultID);

		// save the pointer to the graphic object with the body.
		NewtonBodySetUserData (boxBody, box);

		// set a destructor for this rigid body
		NewtonBodySetDestructorCallback (boxBody, PhysicsBodyDestructor);

		// set the transform call back function
		NewtonBodySetTransformCallback (boxBody, PhysicsSetTransform);

		// set the force and torque call back function
		NewtonBodySetForceAndTorqueCallback (boxBody, PhysicsApplyGravityForce);

		// set the mass matrix
		float mass = 20.0f;
		float Ixx = mass * (size.m_y * size.m_y + size.m_z * size.m_z) / 12.0f;
		float Iyy = mass * (size.m_x * size.m_x + size.m_z * size.m_z) / 12.0f;
		float Izz = mass * (size.m_x * size.m_x + size.m_y * size.m_y) / 12.0f;

		NewtonBodySetMassMatrix (boxBody, mass, Ixx, Iyy, Izz);

		// set the matrix for both the rigid bodies and the graphic body
		NewtonBodySetMatrix (boxBody, &matrix[0][0]);
		PhysicsSetTransform (boxBody, &matrix[0][0], 0);

		return boxBody;
*/
	}


	static NewtonBody* CreateWheel (SceneManager* parent, NewtonWorld* nWorld, const dMatrix& location, dFloat radius)
	{
_ASSERTE (0);
return NULL;
/*

		NewtonBody* sphBody;
		NewtonCollision* sphCollision;


		// create the collision 
		sphCollision = NewtonCreateSphere (nWorld, radius, radius, radius, 0, NULL); 

		//create the rigid body
		sphBody = NewtonCreateBody (nWorld, sphCollision);

		// release the collision
		NewtonReleaseCollision (nWorld, sphCollision);

		//	sph = new SpherePrimitive (nGrapphicWorld, location, radius, radius, radius);

		// create a graphic box
		OGLMesh* meshInstance;
		RenderPrimitive* sph;
		meshInstance = new OGLMesh ("wheel", sphCollision, "earthmap.tga", "earthmap.tga", "earthmap.tga");
		sph = new RenderPrimitive (location, meshInstance);
		parent->AddModel___ (sph);
		sph->Release();
		meshInstance->Release();

		// save the pointer to the graphic object with the body.
		NewtonBodySetUserData (sphBody, sph);

		// set a destructor for this rigid body
		NewtonBodySetDestructorCallback (sphBody, PhysicsBodyDestructor);

		// set the transform call back function
		NewtonBodySetTransformCallback (sphBody, PhysicsSetTransform);

		// set the force and torque call back function
		NewtonBodySetForceAndTorqueCallback (sphBody, PhysicsApplyGravityForce);

		// set the mass matrix
		float mass = 2.0f;
		float Ixx = 0.4f * mass * radius * radius;
		NewtonBodySetMassMatrix (sphBody, mass, Ixx, Ixx, Ixx);

		// set the matrix for tboth the rigid nody and the graphic body
		NewtonBodySetMatrix (sphBody, &location[0][0]);
		PhysicsSetTransform (sphBody, &location[0][0], 0);

		return sphBody;
*/
	}
};


static void AddSimpleRobot (dVector position, SceneManager* scene, NewtonWorld* nWorld)
{
	dFloat radius;

	NewtonBody* robotBody;
	NewtonBody* leftWheel;
	NewtonBody* rightWheel;
	NewtonBody* frontWheel;

	radius = 0.5f;

	// create the robot body parts
	dVector size (6.0f, 2.0f, 6.0f);
	dMatrix location (GetIdentityMatrix());
	location.m_posit = position;
	location.m_posit.m_w = 1.0f;
	location.m_posit.m_y = FindFloor (nWorld, location.m_posit.m_x, location.m_posit.m_z) + 2.0f;

	robotBody = ROBOT_COMPONENTS::CreateRobotBody (scene, nWorld, location, size);
	NewtonBodySetAutoSleep (robotBody, 0);


	// create three wheels around the robot body
	dMatrix frontMatrix (location);
	frontMatrix.m_posit.m_x += size.m_x * 0.5f + radius;
	frontMatrix.m_posit.m_y -= size.m_y * 0.5f;
	frontWheel = ROBOT_COMPONENTS::CreateWheel (scene, nWorld, frontMatrix, radius);


	dMatrix rightMatrix (location);
	rightMatrix.m_posit.m_x -= (size.m_x * 0.5f);
	rightMatrix.m_posit.m_y -= size.m_y * 0.5f;
	rightMatrix.m_posit.m_z += (size.m_z * 0.5f + radius);
	rightWheel = ROBOT_COMPONENTS::CreateWheel (scene, nWorld, rightMatrix, radius);


	dMatrix leftMatrix (location);
	leftMatrix.m_posit.m_x -= (size.m_x * 0.5f);
	leftMatrix.m_posit.m_y -= size.m_y * 0.5f;
	leftMatrix.m_posit.m_z -= (size.m_z * 0.5f + radius);
	leftWheel = ROBOT_COMPONENTS::CreateWheel (scene, nWorld, leftMatrix, radius);


	ROBOT_COMPONENTS::RobotRearWheel* leftJoint;
	ROBOT_COMPONENTS::RobotRearWheel* rightJoint;
	ROBOT_COMPONENTS::RobotFrontWheel* frontJoint;

	// connect the wheels to the body
	dMatrix wheelAxis;
	wheelAxis.m_front = frontMatrix.m_right;
	wheelAxis.m_up = frontMatrix.m_up;
	wheelAxis.m_right = wheelAxis.m_front * wheelAxis.m_up;
	wheelAxis.m_posit = frontMatrix.m_posit;
	frontJoint = new ROBOT_COMPONENTS::RobotFrontWheel (wheelAxis, robotBody, frontWheel);

	wheelAxis.m_posit = leftMatrix.m_posit;
	leftJoint = new ROBOT_COMPONENTS::RobotRearWheel (wheelAxis, robotBody, leftWheel);

	wheelAxis.m_posit = rightMatrix.m_posit;
	rightJoint = new ROBOT_COMPONENTS::RobotRearWheel (wheelAxis, robotBody, rightWheel);

}



void BasicRobots (NewtonFrame& system)
{
	NewtonWorld* world;
	NewtonBody* floor; 
	world = system.m_world;

	// create the sky box and the floor,
	floor = BuildFloorAndSceneRoot (system);

	for (int x = 0; x < 3; x ++) {
		for (int z = 0; z < 3; z ++) {
			AddSimpleRobot (dVector (x * 10.0f, 0.0f, z * 10.0f, 1.0f), &system, system.m_world);
		}
	}
}

