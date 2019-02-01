/* Copyright (c) <2003-2016> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/

#include "toolbox_stdafx.h"
#include "SkyBox.h"
#include "DemoEntityManager.h"
#include "DemoCamera.h"
#include "PhysicsUtils.h"
#include "DemoMesh.h"
#include "OpenGlUtil.h"

static void UserContactRestitution (const NewtonJoint* contactJoint, dFloat timestep, int threadIndex)
{
	// call  the basic call back
	GenericContactProcess (contactJoint, timestep, threadIndex);

	const NewtonBody* const body0 = NewtonJointGetBody0(contactJoint);
	const NewtonBody* const body1 = NewtonJointGetBody1(contactJoint);

	//now core 3.14 can have per collision user data
	const NewtonCollision* const collision0 = NewtonBodyGetCollision(body0);
	const NewtonCollision* const collision1 = NewtonBodyGetCollision(body1);

	NewtonCollisionMaterial material0;
	NewtonCollisionMaterial material1;
	NewtonCollisionGetMaterial(collision0, &material0);
	NewtonCollisionGetMaterial(collision1, &material1);
	dAssert((material0.m_userId == 1) || (material1.m_userId == 1));
	dFloat restitution = dMax(material0.m_userParam[0], material1.m_userParam[0]);

	for (void* contact = NewtonContactJointGetFirstContact (contactJoint); contact; contact = NewtonContactJointGetNextContact (contactJoint, contact)) {
		NewtonMaterial* const material = NewtonContactGetMaterial (contact);
		NewtonMaterialSetContactElasticity (material, restitution);
	}
}

static void CreateResititionBody (DemoEntityManager* const scene, DemoMesh* const mesh, dFloat mass, const dMatrix& matrix, NewtonCollision* const collision, int materialId, dFloat restCoef)
{
	NewtonCollisionMaterial material;
	NewtonCollisionGetMaterial(collision, &material);
	material.m_userId = 1;

	// save restitution coefficient in param[0]
	material.m_userParam[0] = restCoef;
	NewtonCollisionSetMaterial(collision, &material);

	NewtonBody* const body = CreateSimpleSolid(scene, mesh, mass, matrix, collision, materialId);
	NewtonBodySetLinearDamping(body, 0.0f);
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
	DemoMesh* const sphereMesh = new DemoMesh("sphere", scene->GetShaderCache(), sphereCollision, "smilli.tga", "smilli.tga", "smilli.tga");

	// create some boxes too
	dVector boxSize (2.0f, 2.0f, 2.0f, 0.0f);
	NewtonCollision* const boxCollision = CreateConvexCollision (world, offsetMatrix, boxSize, _BOX_PRIMITIVE, 0);
	DemoMesh* const boxMesh = new DemoMesh("box", scene->GetShaderCache(), boxCollision, "smilli.tga", "smilli.tga", "smilli.tga");

	int zCount = 10;
	dFloat spacing = 4.0f;
	dMatrix matrix (dGetIdentityMatrix());
	dVector origin (matrix.m_posit);
	origin.m_x -= 0.0f;

	// create a simple scene
	for (int i = 0; i < zCount; i ++) {
		dFloat restitution = 0.1f * (i + 1);

		dFloat z;
		dFloat x;
		dFloat mass;

		x = origin.m_x;
		z = origin.m_z + (i - zCount / 2) * spacing;

		mass = 1.0f;
		matrix.m_posit = FindFloor (world, dVector (x, 100.0f, z), 200.0f);
		matrix.m_posit.m_w = 1.0f;

		matrix.m_posit.m_y += 6.0f;
		CreateResititionBody(scene, sphereMesh, mass, matrix, sphereCollision, defaultMaterialID, restitution);

		matrix.m_posit.m_y += 6.0f;
		CreateResititionBody(scene, sphereMesh, mass, matrix, sphereCollision, defaultMaterialID, restitution);

		matrix.m_posit.m_y += 6.0f;
		CreateResititionBody(scene, sphereMesh, mass, matrix, sphereCollision, defaultMaterialID, restitution);

		matrix.m_posit.m_y += 6.0f;
		CreateResititionBody(scene, boxMesh, mass, matrix, boxCollision, defaultMaterialID, restitution);
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

