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
#include "DemoMesh.h"
#include "DemoCamera.h"
#include "PhysicsUtils.h"


static int UserOnAABBOverlap (const NewtonJoint* const contactJoint, dFloat timestep, int threadIndex)
{
	#ifdef _DEBUG
	NewtonBody* const body0 = NewtonJointGetBody0(contactJoint);
	NewtonBody* const body1 = NewtonJointGetBody1(contactJoint);
	NewtonJoint* const contact0 = NewtonBodyFindContact (body0, body1);
	NewtonJoint* const contact1 = NewtonBodyFindContact (body1, body0);
	dAssert (!contact0 || contact0 == contactJoint);
	dAssert (!contact1 || contact1 == contactJoint);
	#endif	
	return 1;
}

static void UserContactFriction (const NewtonJoint* contactJoint, dFloat timestep, int threadIndex)
{
	// call  the basic call back
	GenericContactProcess (contactJoint, timestep, threadIndex);

	const NewtonBody* const body0 = NewtonJointGetBody0(contactJoint);
	const NewtonBody* const body1 = NewtonJointGetBody1(contactJoint);

	//now core 3.14 can have per rampCollision user data
	const NewtonCollision* const collision0 = NewtonBodyGetCollision(body0);
	const NewtonCollision* const collision1 = NewtonBodyGetCollision(body1);

	NewtonCollisionMaterial material0;
	NewtonCollisionMaterial material1;
	NewtonCollisionGetMaterial(collision0, &material0);
	NewtonCollisionGetMaterial(collision1, &material1);
	dAssert ((material0.m_userId == 1) || (material1.m_userId == 1));
	dFloat frictionValue = dMax (material0.m_userParam[0], material1.m_userParam[0]);

	for (void* contact = NewtonContactJointGetFirstContact (contactJoint); contact; contact = NewtonContactJointGetNextContact (contactJoint, contact)) {
		NewtonMaterial* const material = NewtonContactGetMaterial (contact);
		NewtonMaterialSetContactFrictionCoef (material, frictionValue + 0.1f, frictionValue, 0);
		NewtonMaterialSetContactFrictionCoef (material, frictionValue + 0.1f, frictionValue, 1);
	}
}

void LoadRampScene (DemoEntityManager* const scene)
{
	dVector tableSize(40.0f, 0.25f, 40.0f, 0.0f);

	NewtonWorld* const world = scene->GetNewton();
	NewtonCollision* const rampCollision = CreateConvexCollision(world, dGetIdentityMatrix(), tableSize, _BOX_PRIMITIVE, 0);

	DemoMesh* const rampMesh = new DemoMesh("ramp", scene->GetShaderCache(), rampCollision, "wood_3.tga", "wood_3.tga", "wood_3.tga");

	dMatrix matrix = dRollMatrix(30.0f * dDegreeToRad);
	matrix.m_posit.m_x = 0.0f;
	matrix.m_posit.m_z = 0.0f;
	matrix.m_posit.m_y = 9.0f;
	NewtonBody* const tableBody = CreateSimpleSolid(scene, rampMesh, 0.0f, matrix, rampCollision, 0);
	NewtonBodySetMassProperties(tableBody, 0.0f, NewtonBodyGetCollision(tableBody));
	NewtonBodySetMaterialGroupID(tableBody, 0);
	rampMesh->Release();
	NewtonDestroyCollision(rampCollision);


	dVector size (0.8f, 0.25f, 0.5f, 0.0f);
	dMatrix shapeMatrix (dGetIdentityMatrix());
	NewtonCollision* const collision = CreateConvexCollision(world, shapeMatrix, size, _BOX_PRIMITIVE, 0);
	DemoMesh* const geometry = new DemoMesh("cylinder_1", scene->GetShaderCache(), collision, "smilli.tga", "smilli.tga", "smilli.tga");

	dFloat startElevation = 100.0f;

	matrix.m_posit.m_z -= 15.0f;
	matrix.m_posit.m_x += 15.0f;
	dVector floor(FindFloor(world, dVector(matrix.m_posit.m_x, startElevation, matrix.m_posit.m_z, 0.0f), 2.0f * startElevation));

	matrix.m_posit.m_y = floor.m_y + size.m_y / 2.0f;
	for (int i = 0; i < 10; i++) {
		CreateSimpleSolid(scene, geometry, 1.0, matrix, collision, 0);
		matrix.m_posit.m_z += 2;
	}

	geometry->Release();
	NewtonDestroyCollision(collision);
}

void Friction (DemoEntityManager* const scene)
{
	// load the skybox
	scene->CreateSkyBox();

	// load the scene from a ngd file format
	CreateLevelMesh(scene, "flatPlane.ngd", 1);

	// load Ramp scene
	LoadRampScene (scene);

	// set a default material call back
	NewtonWorld* const world = scene->GetNewton();
	int defaultMaterialID = NewtonMaterialGetDefaultGroupID (world);
	NewtonMaterialSetCollisionCallback (world, defaultMaterialID, defaultMaterialID, UserOnAABBOverlap, UserContactFriction); 
	//NewtonMaterialSetDefaultCollidable(world, defaultMaterialID, defaultMaterialID, 0);

	// customize the scene after loading
	// set a user friction variable in the body for variable friction demos
	// later this will be done using LUA script
	int index = 0;
	for (NewtonBody* body = NewtonWorldGetFirstBody(world); body; body = NewtonWorldGetNextBody(world, body)) {
		dFloat Ixx;
		dFloat Iyy;
		dFloat Izz;
		dFloat mass;
		NewtonBodyGetMass (body, &mass, &Ixx, &Iyy, &Izz);
		if (mass > 0.0f) {
			// use the new instance feature to ass per shape information
			NewtonCollision* const collision = NewtonBodyGetCollision(body);

			// use the collision user data to save the coefficient of friction 
			dFloat coefficientOfFriction = dFloat (index) * 0.03f; 

			// save the restitution coefficient with the collision user data
			NewtonCollisionMaterial material;
			NewtonCollisionGetMaterial(collision, &material);
			material.m_userId = 1;

			// save kinetic friction in param[0]
			material.m_userParam[0] = coefficientOfFriction;

			// save static friction in param[1]
			material.m_userParam[1] = coefficientOfFriction;
			NewtonCollisionSetMaterial(collision, &material);

			index ++;
		}
	}

	// place camera into position
	dQuaternion rot;
	dVector origin (-35.0f, 10.0f, 0.0f, 0.0f);
	scene->SetCameraMatrix(rot, origin);

//	ExportScene (scene->GetNewton(), "../../../media/test1.ngd");
}

