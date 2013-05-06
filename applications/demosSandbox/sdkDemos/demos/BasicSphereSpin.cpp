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
#include "../DemoEntityManager.h"
//#include "SkyBox.h"
//#include "RenderPrimitive.h"
//#include "../OGLMesh.h"
//#include "../MainFrame.h"
//#include "../PhysicsUtils.h"
//#include "../SceneManager.h"
//#include "../toolBox/MousePick.h"
//#include "../toolBox/OpenGlUtil.h"

/*
// set the transformation of a rigid body
static void PhysicsSetTransformSpin (const NewtonBody* body, const dFloat* matrix, int threadIndex)
{
	RenderPrimitive* primitive;

	// get the graphic object form the rigid body
	primitive = (RenderPrimitive*) NewtonBodyGetUserData (body);

	// set the transformation matrix for this rigid body
	dMatrix& mat = *((dMatrix*)matrix);
	primitive->SetMatrix (mat);

	// animate the body by setting the angular velocity
	dVector omega (mat.m_front.Scale (10.0f));
	NewtonBodySetOmega (body, &omega[0]); 

}


static void DoNothing (SceneManager& me, int mode)
{
}
*/

//void BasicSphereSpin (NewtonFrame& system)
void BasicSphereSpin (DemoEntityManager* const scene)
{
_ASSERTE (0);
/*

	NewtonWorld* world;
	RenderPrimitive* box;
	OGLMesh* meshInstance;
	NewtonCollision* collision;
	NewtonBody* rigidBodyBox; 
	dVector size (1.0f, 1.0f, 1.0f, 0.0f);

	world = system.m_world;

	// create the sky box,
	OGLModel* sky = new SkyBox ();
	system.AddModel___ (sky);
	sky->Release();


	// create sphere collision
	collision = NewtonCreateSphere (world, size.m_x, size.m_y * 1.25f, size.m_z * 1.25f, 0, NULL); 

	meshInstance = new OGLMesh ("earth", collision, "earthmap.tga", "metal_30.tga", "camo.tga");
	box = new RenderPrimitive (GetIdentityMatrix(), meshInstance);
	meshInstance->Release();

	// create the rigid body
	rigidBodyBox = NewtonCreateBody (world, collision);


	// save the pointer to the graphic object with the body.
	NewtonBodySetUserData (rigidBodyBox, box);

	// Get Rid of the collision
	NewtonReleaseCollision (world, collision);

	// set the body mass and inertia
	NewtonBodySetMassMatrix (rigidBodyBox, 1.0f, 5.0f, 1.0f, 5.0f);

	// set the transform call back function
	NewtonBodySetTransformCallback (rigidBodyBox, PhysicsSetTransformSpin);


	// set the transformation matrix
	dMatrix matrix (dRollMatrix(90.0f * 3.141592f / 180.0f));
	matrix = matrix * dYawMatrix(-23.0f * 3.141592f / 180.0f);


	matrix.m_posit.m_x = 0.0f;
	matrix.m_posit.m_y = 0.0f;
	matrix.m_posit.m_z = 0.0f;
	NewtonBodySetMatrix (rigidBodyBox, &matrix[0][0]);

	// animate the body by setting the angular velocity
	dVector omega (matrix.m_front.Scale (10.0f));
	NewtonBodySetOmega (rigidBodyBox, &omega[0]); 

//	cameraDir = dVector (0.0f, 0.0f, -1.0f);
//	cameraEyepoint = dVector (0.0f, 0.0f, 4.0f);
	InitEyePoint (dVector (1.0f, 0.0f, 0.0f), dVector (-4.0f, 0.0f, 0.0f));

	system.AddModel___ (box);
	box->Release();


	system.m_control = Keyboard;
	system.m_autoSleep = DoNothing;
	system.m_showIslands = DoNothing;
	system.m_showContacts = DoNothing; 
	system.m_setMeshCollision = DoNothing;
*/
}



