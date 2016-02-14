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

#define FRICTION_VAR_NAME "friction"
static dCRCTYPE frictionCRC (dCRC64 (FRICTION_VAR_NAME));


static int UserOnAABBOverlap (const NewtonMaterial* const material, const NewtonBody* const body0, const NewtonBody* const body1, int threadIndex)
{
	return 1;
}

static void UserContactFriction (const NewtonJoint* contactJoint, dFloat timestep, int threadIndex)
{
	dFloat Ixx;
	dFloat Iyy;
	dFloat Izz;
	dFloat mass;

	// call  the basic call back
	GenericContactProcess (contactJoint, timestep, threadIndex);

	const NewtonBody* const body0 = NewtonJointGetBody0(contactJoint);
	const NewtonBody* const body1 = NewtonJointGetBody1(contactJoint);
	const NewtonBody* body = body0;
	NewtonBodyGetMassMatrix (body, &mass, &Ixx, &Iyy, &Izz);
	if (mass == 0.0f) {
		body = body1;
	}

	//now core 300 can have per collision user data 		
	NewtonCollision* const collision = NewtonBodyGetCollision(body);
	void* userData = NewtonCollisionGetUserData (collision);
	dFloat frictionValue = *((float*)&userData);
	for (void* contact = NewtonContactJointGetFirstContact (contactJoint); contact; contact = NewtonContactJointGetNextContact (contactJoint, contact)) {
		NewtonMaterial* const material = NewtonContactGetMaterial (contact);
		NewtonMaterialSetContactFrictionCoef (material, frictionValue + 0.1f, frictionValue, 0);
		NewtonMaterialSetContactFrictionCoef (material, frictionValue + 0.1f, frictionValue, 1);
	}
}


void Friction (DemoEntityManager* const scene)
{
	// load the skybox
	scene->CreateSkyBox();


	// load the scene from a ngd file format
	char fileName[2048];
	GetWorkingFileName ("frictionDemo.ngd", fileName);
	scene->LoadScene (fileName);


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
		NewtonBodyGetMassMatrix (body, &mass, &Ixx, &Iyy, &Izz);
		if (mass > 0.0f) {
			// use the new instance feature to ass per shape information
			NewtonCollision* const collision = NewtonBodyGetCollision(body);

			// use the collision user data to save the coefficient of friction 
			float coefficientOfFriction = dFloat (index) * 0.03f; 
			NewtonCollisionSetUserData (collision, *((void**)&coefficientOfFriction));

//			DemoEntity* const entity = (DemoEntity*) NewtonBodyGetUserData (body);
//			dVariable* const friction = entity->CreateVariable (FRICTION_VAR_NAME);
//			dAssert (friction);
//			friction->SetValue(dFloat (index) * 0.03f);
			index ++;
		}
	}

	// place camera into position
	dQuaternion rot;
	dVector origin (-70.0f, 10.0f, 0.0f, 0.0f);
	scene->SetCameraMatrix(rot, origin);

//	ExportScene (scene->GetNewton(), "../../../media/test1.ngd");
}

