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
#include "DemoEntityManager.h"
#include "DemoCamera.h"
#include "PhysicsUtils.h"
#include "DemoMesh.h"
#include "OpenGlUtil.h"



static void UserContactRestitution (const NewtonJoint* contactJoint, dFloat timestep, int threadIndex)
{
	dFloat Ixx;
	dFloat Iyy;
	dFloat Izz;
	dFloat mass;
	NewtonBody* body;
	NewtonBody* body0;
	NewtonBody* body1;

	// call  the basic call back
	GenericContactProcess (contactJoint, timestep, threadIndex);

	body0 = NewtonJointGetBody0(contactJoint);
	body1 = NewtonJointGetBody1(contactJoint);

	body = body0;
	NewtonBodyGetMass (body, &mass, &Ixx, &Iyy, &Izz);
	if (mass == 0.0f) {
		body = body1;
	}

	NewtonCollision* const collision = NewtonBodyGetCollision(body);
	void* userData = NewtonCollisionGetUserData (collision);
	dFloat restitution = *((dFloat*)&userData);
	for (void* contact = NewtonContactJointGetFirstContact (contactJoint); contact; contact = NewtonContactJointGetNextContact (contactJoint, contact)) {
		NewtonMaterial* const material = NewtonContactGetMaterial (contact);
		NewtonMaterialSetContactElasticity (material, restitution);
	}
}


void Restitution (DemoEntityManager* const scene)
{
	scene->CreateSkyBox();

	// customize the scene after loading
	// set a user friction variable in the body for variable friction demos
	// later this will be done using LUA script
	NewtonWorld* const world = scene->GetNewton();
	dMatrix offsetMatrix (dGetIdentityMatrix());

	int defaultMaterialID = NewtonMaterialGetDefaultGroupID (world);
	NewtonMaterialSetCollisionCallback (world, defaultMaterialID, defaultMaterialID, NULL, UserContactRestitution); 


	CreateLevelMesh (scene, "flatPlane.ngd", 1);

	dVector location (0.0f, 0.0f, 0.0f, 0.0f);

	// create some spheres 
	dVector sphSize (2.0f, 2.0f, 2.0f, 0.0f);
	NewtonCollision* const sphereCollision = CreateConvexCollision (world, offsetMatrix, sphSize, _SPHERE_PRIMITIVE, 0);
	DemoMesh* const sphereMesh = new DemoMesh("sphere", sphereCollision, "smilli.tga", "smilli.tga", "smilli.tga");

	// create some boxes too
	dVector boxSize (2.0f, 2.0f, 2.0f, 0.0f);
	NewtonCollision* const boxCollision = CreateConvexCollision (world, offsetMatrix, boxSize, _BOX_PRIMITIVE, 0);
	DemoMesh* const boxMesh = new DemoMesh("box", boxCollision, "smilli.tga", "smilli.tga", "smilli.tga");

	int zCount = 10;
//	int zCount = 1;
	dFloat spacing = 4.0f;
	dMatrix matrix (dGetIdentityMatrix());
	dVector origin (matrix.m_posit);
	origin.m_x -= 0.0f;

	// create a simple scene
	for (int i = 0; i < zCount; i ++) {
		dFloat restitution = i * 0.1f + 0.083f;
		NewtonBody* body;
		NewtonCollision* collision;

		dFloat z;
		dFloat x;
		dFloat mass;

		x = origin.m_x;
		z = origin.m_z + (i - zCount / 2) * spacing;

		mass = 1.0f;
		matrix.m_posit = FindFloor (world, dVector (x, 100.0f, z), 200.0f);
		matrix.m_posit.m_w = 1.0f;

		matrix.m_posit.m_y += 6.0f;
		body = CreateSimpleSolid (scene, sphereMesh, mass, matrix, sphereCollision, defaultMaterialID);
		NewtonBodySetLinearDamping (body, 0.0f);
		collision = NewtonBodyGetCollision(body);
		
		NewtonCollisionSetUserData (collision, *((void**)&restitution));

		matrix.m_posit.m_y += 6.0f;
		//body = CreateSimpleSolid (scene, boxMesh, mass, matrix, boxCollision, defaultMaterialID);
		body = CreateSimpleSolid (scene, sphereMesh, mass, matrix, sphereCollision, defaultMaterialID);
		NewtonBodySetLinearDamping (body, 0.0f);
		collision = NewtonBodyGetCollision(body);
		NewtonCollisionSetUserData (collision, *((void**)&restitution));

		matrix.m_posit.m_y += 6.0f;
		body = CreateSimpleSolid (scene, sphereMesh, mass, matrix, sphereCollision, defaultMaterialID);
		NewtonBodySetLinearDamping (body, 0.0f);
		collision = NewtonBodyGetCollision(body);
		NewtonCollisionSetUserData (collision, *((void**)&restitution));

		matrix.m_posit.m_y += 6.0f;
		body = CreateSimpleSolid (scene, boxMesh, mass, matrix, boxCollision, defaultMaterialID);
		//body = CreateSimpleSolid (scene, sphereMesh, mass, matrix, sphereCollision, defaultMaterialID);
		NewtonBodySetLinearDamping (body, 0.0f);
		collision = NewtonBodyGetCollision(body);
		NewtonCollisionSetUserData (collision, *((void**)&restitution));
	}

	boxMesh->Release();
	sphereMesh->Release();
	NewtonDestroyCollision(boxCollision);
	NewtonDestroyCollision(sphereCollision);


	dMatrix camMatrix (dGetIdentityMatrix());
	dQuaternion rot (camMatrix);
	origin = dVector (-25.0f, 5.0f, 0.0f, 0.0f);
	scene->SetCameraMatrix(rot, origin);

}

