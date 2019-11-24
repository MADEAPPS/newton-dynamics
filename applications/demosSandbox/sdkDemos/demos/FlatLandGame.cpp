/* Copyright (c) <2003-2019> <Newton Game Dynamics>
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
#include "DemoEntityManager.h"
#include "DemoCamera.h"
#include "PhysicsUtils.h"
#include "DemoMesh.h"
#include "toolBox/OpenGlUtil.h"

#define PHYSICS_WORLD_SCALE			50.0f											// conversion from meters to world units

//#define TABLE_WIDTH					(0.508f * PHYSICS_WORLD_SCALE)					// 20 inches
//#define TABLE_LENGTH				(5.08f * PHYSICS_WORLD_SCALE)					// 200 inches (16' 8")
//#define TABLE_HEIGHT				(0.0762f * PHYSICS_WORLD_SCALE)					// 3 inches

#define WEIGHT_DIAMETER				(0.062f * PHYSICS_WORLD_SCALE)
#define WEIGHT_RADIUS				(WEIGHT_DIAMETER * 0.5f)
#define WEIGHT_HEIGHT				(0.025f * PHYSICS_WORLD_SCALE)

#define WEIGHT_MASS					0.360f											// weight of a puck in Kg (mass doesn't need scaling)

#define CAMERA_X					(-11.0f)
#define CAMERA_Y					(0.26f * PHYSICS_WORLD_SCALE)
#define CAMERA_Z					(-3.0f * PHYSICS_WORLD_SCALE)


/*
class PuckEntity: public DemoEntity
{
	public: 
	PuckEntity (DemoEntityManager* const scene, int materialID)
		:DemoEntity (dGetIdentityMatrix(), NULL)
		,m_launched(false)
	{
		scene->Append(this);

		NewtonWorld* const world = scene->GetNewton();

		dVector puckSize(WEIGHT_DIAMETER, WEIGHT_HEIGHT, 0.0f, 0.0f);

		// create the shape and visual mesh as a common data to be re used
		NewtonCollision* const collision = CreateConvexCollision (world, dGetIdentityMatrix(), puckSize, _CYLINDER_PRIMITIVE, materialID);

		// correction: make the puck an upright cylinder, this makes everything simpler  
		dMatrix collisionAligmentMatrix (dRollMatrix(dPi/2.0f));
		NewtonCollisionSetMatrix(collision, &collisionAligmentMatrix[0][0]);

		DemoMesh* const geometry = new DemoMesh("cylinder_1", scene->GetShaderCache(), collision, "smilli.tga", "smilli.tga", "smilli.tga");

		//dMatrix matrix = dRollMatrix(dPi/2.0f);
		dMatrix matrix (dGetIdentityMatrix());
		matrix.m_posit.m_x = -TABLE_LENGTH*0.5f+WEIGHT_DIAMETER;
		matrix.m_posit.m_z = -11.8f;
//matrix.m_posit.m_z += 4.0f;
		matrix.m_posit.m_y = 5.0f;

		m_puckBody = CreateSimpleSolid (scene, geometry, WEIGHT_MASS, matrix, collision, materialID);

		// Set moment of inertia
		// correction: this is deprecated, NewtonBodySetMassProperties produce the exact result
		//dVector I;
		//dFloat Mass = WEIGHT_MASS;
		//dFloat Radius = WEIGHT_RADIUS;
		//dFloat Height = WEIGHT_HEIGHT;
		//I.m_x = I.m_z = Mass*(3.0f*Radius*Radius+Height*Height)/12.0f;
		//I.m_y = Mass*Radius*Radius/2.0f;
		//NewtonBodySetMassMatrix(gPuckBody,Mass, I.m_x, I.m_y, I.m_z);	
		NewtonBodySetMassProperties(m_puckBody, WEIGHT_MASS, NewtonBodyGetCollision(m_puckBody));


		NewtonBodySetMaterialGroupID(m_puckBody, materialID);

		// remember to make continuous collision work with auto sleep mode, right now this is no working
		NewtonBodySetContinuousCollisionMode(m_puckBody, 1);
		NewtonBodySetAutoSleep(m_puckBody, 1);

		// Set callbacks
		NewtonBodySetForceAndTorqueCallback(m_puckBody, NewtonRigidBodySetForceCB);

		// do not forget to release the assets
		geometry->Release(); 
		NewtonDestroyCollision (collision);
	}

	~PuckEntity ()
	{
	}

	void LaunchPuck()
	{
		if (!m_launched) {
			m_launched = true;

			NewtonInvalidateCache (NewtonBodyGetWorld(m_puckBody));
			dVector zeros(0.0f, 0.0f, 0.0f, 0.0f);

			NewtonBodySetVelocity(m_puckBody, &zeros.m_x);
			NewtonBodySetOmega(m_puckBody, &zeros.m_x);
			NewtonBodySetForce(m_puckBody, &zeros.m_x);
			NewtonBodySetTorque(m_puckBody, &zeros.m_x);

			dVector vel(171.299469f, 0.0f, 0.0f);
			NewtonBodySetVelocity(m_puckBody, &vel.m_x);
		}
	}


	void UpdateStuff(DemoEntityManager* const scene)
	{
	}


	virtual void SimulationPreListener(DemoEntityManager* const scene, DemoEntityManager::dListNode* const mynode, dFloat timestep)
	{
//		UpdateStuff(scene);
		int sleepState = NewtonBodyGetSleepState(m_puckBody);
		if (sleepState)
		{
			LaunchPuck();
		}
	}

	bool m_launched;
	NewtonBody* m_puckBody;
};


void NewtonRigidBodySetForceCB(const NewtonBody* const body, dFloat timestep, int threadIndex)
{	
	dFloat mass;
	dFloat Ixx;
	dFloat Iyy;
	dFloat Izz;
	NewtonBodyGetMass(body, &mass, &Ixx, &Iyy, &Izz);
	
	dFloat force[3];
	force[0] = 0.0f;
	force[1] = mass * (DEMO_GRAVITY * PHYSICS_WORLD_SCALE);
	force[2] = 0.0f;
	NewtonBodySetForce(body, force);
}


static void PhysicsNewton_CollisionPuckSurfaceCB(const NewtonJoint *pContactJoint,dFloat fTimeStep,int ThreadIndex)
{
	dVector Position(0.0f);
	
	// Get pointer to body
	NewtonBody* body = NewtonJointGetBody0(pContactJoint);				
	dFloat mass, Ixx, Iyy, Izz;
	NewtonBodyGetMass(body, &mass, &Ixx, &Iyy, &Izz);
	if (mass == 0.0f)
	{
		body = NewtonJointGetBody1(pContactJoint);
	}	

	dVector tableDir(0.0f, 0.0f, 1.0f, 0.0f);
	// Test to see if it is the friction calculation that is causing the side force
	// With this the Puck must go straight because it will be frictionless, the net force should not change direction
	for (void* contact = NewtonContactJointGetFirstContact (pContactJoint); contact; contact = NewtonContactJointGetNextContact (pContactJoint, contact))
	{
		NewtonMaterial* const material = NewtonContactGetMaterial (contact);

		NewtonMaterialContactRotateTangentDirections(material, &tableDir[0]);

		// this the wrong way to make a friction less contact  
//		NewtonMaterialSetContactFrictionCoef (material, 0.0f, 0.0f, 0);
//		NewtonMaterialSetContactFrictionCoef (material, 0.0f, 0.0f, 1);

		//This is the correct way to make a friction less contact		
//		NewtonMaterialSetContactFrictionState (material, 0, 0);
//		NewtonMaterialSetContactFrictionState (material, 0, 1);
	}
}
*/


static void AddGravityBodies (DemoEntityManager* const scene, NewtonBody* const floor)
{
	NewtonWorld* const world = scene->GetNewton();
	dVector size(0.5f, 0.5f, 0.5f, 0.0f);

	dMatrix location (dGetIdentityMatrix());

	location.m_posit.m_y = 4.0f;
	NewtonCollision* const collision = CreateConvexCollision(world, dGetIdentityMatrix(), size, _BOX_PRIMITIVE, 0);
	DemoMesh* const geometry = new DemoMesh("table", scene->GetShaderCache(), collision, "frowny.tga", "logo_php.tga", "smilli.tga");
	for (int i = 0; i < 4; i ++) {
		location.m_posit.m_z = -5.0f;
		for (int j = 0; j < 4; j ++) {
			NewtonBody* const box = CreateSimpleSolid(scene, geometry, 10.0f, location, collision, 0);

			// constrain these object to motion on the plane only
			dMatrix matrix; 
			NewtonBodyGetMatrix (box, &matrix[0][0]);
			new dCustomPlane (matrix.m_posit, matrix.m_front, box);

			location.m_posit.m_z += 2.5f;
		}
		location.m_posit.m_y += 2.0f;
	}

	geometry->Release();
	NewtonDestroyCollision(collision);
}

static NewtonBody* CreateBackground (DemoEntityManager* const scene)
{
	NewtonWorld* const world = scene->GetNewton();
	dVector tableSize(10.0f, 2.0f, 200.0f, 0.0f);

	// create the shape and visual mesh as a common data to be re used
	NewtonCollision* const collision = CreateConvexCollision(world, dGetIdentityMatrix(), tableSize, _BOX_PRIMITIVE, 0);

	DemoMesh* const geometry = new DemoMesh("table", scene->GetShaderCache(), collision, NULL, "wood_3.tga", NULL);

	dMatrix matrix (dGetIdentityMatrix());
	NewtonBody* const tableBody = CreateSimpleSolid(scene, geometry, 0.0f, matrix, collision, 0);

	geometry->Release();
	NewtonDestroyCollision (collision);

	return tableBody;
}


// create physics scene
void FlatLandGame (DemoEntityManager* const scene)
{
	scene->CreateSkyBox();

//	NewtonWorld* const world = scene->GetNewton();

	// make a floor for a 2d world
	NewtonBody* const ground = CreateBackground (scene);

	// add some object constrained to a move on the x plane
	AddGravityBodies (scene, ground);

	// place camera into position
	dMatrix camMatrix (dGetIdentityMatrix());
	dQuaternion rot (camMatrix);
	camMatrix.m_posit.m_y = 10.0f;
	camMatrix.m_posit.m_x = -30.0f;
	scene->SetCameraMatrix(rot, camMatrix.m_posit);
}
