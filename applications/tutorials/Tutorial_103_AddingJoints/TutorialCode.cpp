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

#include "StdAfx.h"
#include "Entity.h"
#include "TutorialCode.h"
#include "SceneManager.h"
#include "JointLibrary.h"
#include "RigidBodyUtil.h"
#include "CollisionShapeUtil.h"
#include "CollectInputAndUpdateCamera.h"



// follow 
struct FollowPath
{
	dFloat radius;
	dVector m_origin;
	dFloat m_angle;

	// this calculate the desired position and orientation in the path
	dMatrix BuildMatrix (dFloat timestep)
	{
		dMatrix matrix (dPitchMatrix(-m_angle) * dRollMatrix(-m_angle));
		matrix.m_posit = m_origin;
		matrix.m_posit.m_z += radius * dSin (m_angle);
		matrix.m_posit.m_y += radius * dCos (m_angle);
		return matrix;
	}

	// destructor to be call when the joint is destroyed
	static void Destroy (const NewtonUserJoint* me)
	{
		FollowPath* path;

		path = (FollowPath*) CustomGetUserData(me);
		free (path);
	}

	// the call back is call before any of the constraint rows and collumns to inforce the joint are summited.
	static void EvaluatePath (const NewtonUserJoint* me, dFloat timestep, int threadIndex)
	{
		FollowPath* path;
		path = (FollowPath*) CustomGetUserData(me);
		path->m_angle = dMod (path->m_angle + (45.0f * 3.1416f / 180.0f) * timestep, 2.0f * 3.1416f);

		dMatrix matrix (path->BuildMatrix (timestep));
		CustomKinematicControllerSetTargetMatrix (me, &matrix[0][0]); 
	}
};



void CreateScene (NewtonWorld* world, SceneManager* sceneManager)
{
	Entity* floor;
	Entity* smilly;
	Entity* frowny;
	NewtonBody* floorBody;
	NewtonBody* smillyBody;
	NewtonBody* frownyBody;
	NewtonCollision* shape;

	// initialize the camera
	InitCamera (dVector (-15.0f, 5.0f, 0.0f, 0.0f), dVector (1.0f, 0.0f, 0.0f));


	// Create a large body to be the floor
	floor = sceneManager->CreateEntity();
	floor->LoadMesh ("FloorBox.dat");

	// add static floor Physics
	shape = CreateNewtonBox (world, floor, 0);
	floorBody = CreateRigidBody (world, floor, shape, 0.0f);
	NewtonDestroyCollision(shape);

	// set the Transformation Matrix for this rigid body
	dMatrix matrix (floor->m_curRotation, floor->m_curPosition);
	NewtonBodySetMatrix (floorBody, &matrix[0][0]);

	// add some visual entities.
	smilly = sceneManager->CreateEntity();
	smilly->LoadMesh ("Smilly.dat");
	smilly->m_curPosition.m_y = 10.0f;
	smilly->m_prevPosition = smilly->m_curPosition;

	// add a body with a box shape
	shape = CreateNewtonBox (world, smilly, 0);
	smillyBody = CreateRigidBody (world, smilly, shape, 10.0f);
	NewtonDestroyCollision(shape);


	// add some visual entities.
	frowny = sceneManager->CreateEntity();
	frowny->LoadMesh ("Frowny.dat");
	frowny->m_curPosition.m_z = 0.4f;
	frowny->m_curPosition.m_y = 10.0f + 0.4f;
	frowny->m_prevPosition = frowny->m_curPosition;

	// add a body with a Convex hull shape
	shape = CreateNewtonConvex (world, frowny, 0);
	frownyBody = CreateRigidBody (world, frowny, shape, 10.0f);
	NewtonDestroyCollision(shape);

	
	// we will Link the Frowny Body to the world with Hinge regenerated from a Generic 6DOF joint.
	 matrix = GetIdentityMatrix();
	 matrix.m_posit = frowny->m_curPosition;
	 matrix.m_posit.m_z += 0.2f;
	 matrix.m_posit.m_y += 0.4f;

	 // specify the limits for defining a Hinge around the x axis
	 dVector minLinearLimits (0.0f, 0.0f, 0.0f, 0.0f);
	 dVector maxLinearLimits (0.0f, 0.0f, 0.0f, 0.0f);

	 dVector minAngulaLimits (-0.5f * 3.1416f, 0.0f, 0.0f, 0.0f);
	 dVector maxAngulaLimits ( 0.5f * 3.1416f, 0.0f, 0.0f, 0.0f);

	 NewtonUserJoint* frownyHinge;
	 // Create a 6DOF joint 
	 frownyHinge = CreateCustomJoint6DOF (&matrix[0][0], &matrix[0][0], frownyBody, NULL);

	 // set the hinge Limits
 	 CustomJoint6DOF_SetLinearLimits (frownyHinge, &minLinearLimits[0], &maxLinearLimits[0]);
 	 CustomJoint6DOF_SetAngularLimits (frownyHinge, &minAngulaLimits[0], &maxAngulaLimits[0]);

	
	// now we will link the body of Smilly and Frowny with a specialize Hinge Joint 
	 NewtonUserJoint* smillyFrownyHinge;
	 matrix.m_posit = smilly->m_curPosition;
	 matrix.m_posit.m_z += 0.2f;
	 matrix.m_posit.m_y += 0.4f;

	 smillyFrownyHinge = CreateCustomHinge (&matrix[0][0], smillyBody, frownyBody);
	 HingeEnableLimits (smillyFrownyHinge, 1);
	 HingeSetLimits (smillyFrownyHinge, -0.5f * 3.1416f, 0.5f * 3.1416f);

//	 smillyFrownyHinge = CreateCustomBallAndSocket (&matrix[0][0], smillyBody, frownyBody);
//	 BallAndSocketSetConeAngle (smillyFrownyHinge, 0.5f * 3.1416f);
//	 BallAndSocketSetTwistAngle (smillyFrownyHinge, -0.5f * 3.1416f, 0.5f * 3.1416f);

//	 smillyFrownyHinge = CreateCustomSlider (&matrix[0][0], smillyBody, frownyBody);
//	 SliderEnableLimits (smillyFrownyHinge, 1);
//	 SliderSetLimis (smillyFrownyHinge, -0.5f * 3.1416f, 0.5f * 3.1416f);

	 {
		// adding two Kinematic controlled object
		frowny = sceneManager->CreateEntity();
		frowny->LoadMesh ("Frowny.dat");
		frowny->m_curPosition.m_z = 4.0f;
		frowny->m_curPosition.m_y = 7.0f;
		frowny->m_prevPosition = frowny->m_curPosition;
		shape = CreateNewtonConvex (world, frowny, 0);
		frownyBody = CreateRigidBody (world, frowny, shape, 10.0f);
		NewtonDestroyCollision(shape);
		 
		 
		FollowPath* path;
		NewtonUserJoint* g_pathFollow;

		path = (FollowPath*) malloc (sizeof (FollowPath));

		NewtonBodyGetMatrix (frownyBody, &matrix[0][0]);
		path->m_origin = matrix.m_posit;
		path->m_angle = 0.0f;
		path->radius = 3.0f;

		g_pathFollow = CreateCustomKinematicController (frownyBody, &matrix.m_posit[0]);
		CustomKinematicControllerSetPickMode (g_pathFollow, 1);
		CustomKinematicControllerSetMaxAngularFriction (g_pathFollow, 200.0f);
		CustomKinematicControllerSetMaxLinearFriction (g_pathFollow, 1000.0f);
		CustomSetUserData(g_pathFollow, path);
		CustomSetDestructorCallback(g_pathFollow, FollowPath::Destroy);
		CustomSetSubmitContraintCallback (g_pathFollow, FollowPath::EvaluatePath);

		matrix = path->BuildMatrix (0.0f);
		NewtonBodySetMatrix(frownyBody, &matrix[0][0]);
	 }

	 {
		// adding two Kinematic controlled object
		frowny = sceneManager->CreateEntity();
		frowny->LoadMesh ("Frowny.dat");
		frowny->m_curPosition.m_z = -4.0f;
		frowny->m_curPosition.m_y =  6.0f;
		frowny->m_prevPosition = frowny->m_curPosition;
		shape = CreateNewtonConvex (world, frowny, 0);
		frownyBody = CreateRigidBody (world, frowny, shape, 10.0f);
		NewtonDestroyCollision(shape);


		FollowPath* path;
		NewtonUserJoint* g_pathFollow;

		path = (FollowPath*) malloc (sizeof (FollowPath));

		NewtonBodyGetMatrix (frownyBody, &matrix[0][0]);
		path->m_origin = matrix.m_posit;
		path->m_angle = 3.1416f;
		path->radius = 3.0f;

		g_pathFollow = CreateCustomKinematicController (frownyBody, &matrix.m_posit[0]);
		CustomKinematicControllerSetPickMode (g_pathFollow, 1);
		CustomKinematicControllerSetMaxAngularFriction (g_pathFollow, 200.0f);
		CustomKinematicControllerSetMaxLinearFriction (g_pathFollow, 1000.0f);
		CustomSetUserData(g_pathFollow, path);
		CustomSetDestructorCallback(g_pathFollow, FollowPath::Destroy);
		CustomSetSubmitContraintCallback (g_pathFollow, FollowPath::EvaluatePath);

		matrix = path->BuildMatrix (0.0f);
		NewtonBodySetMatrix(frownyBody, &matrix[0][0]);
	 }
}



