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

#define TABLE_WIDTH					(0.508f * PHYSICS_WORLD_SCALE)					// 20 inches
#define TABLE_LENGTH				(5.08f * PHYSICS_WORLD_SCALE)					// 200 inches (16' 8")
#define TABLE_HEIGHT				(0.0762f * PHYSICS_WORLD_SCALE)					// 3 inches

#define WEIGHT_DIAMETER				(0.062f * PHYSICS_WORLD_SCALE)
#define WEIGHT_RADIUS				(WEIGHT_DIAMETER * 0.5f)
#define WEIGHT_HEIGHT				(0.025f * PHYSICS_WORLD_SCALE)

#define WEIGHT_MASS					0.360f											// weight of a puck in Kg (mass doesn't need scaling)

#define CAMERA_X					(-11.0f)
#define CAMERA_Y					(0.26f * PHYSICS_WORLD_SCALE)
#define CAMERA_Z					(-3.0f * PHYSICS_WORLD_SCALE)

// Material data
enum eSBMaterials
{
	SBMaterial_WEIGHT,
	SBMaterial_SURFACE,

	SB_NUM_MATERIALS
};


static void	NewtonRigidBodySetForceCB(const NewtonBody* const body, dFloat timestep, int threadIndex);
static void PhysicsNewton_CollisionPuckSurfaceCB(const NewtonJoint *pContactJoint,dFloat fTimeStep,int ThreadIndex);
//static void RenderBodyContactsAndTangentDiretions (NewtonBody* const body, dFloat length);


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

// create physics scene
void PuckSlide (DemoEntityManager* const scene)
{
	scene->CreateSkyBox();

	NewtonWorld* const world = scene->GetNewton();

	int	materialGroupIDs[SB_NUM_MATERIALS];

	// Create groups
	for (int i = 0; i < SB_NUM_MATERIALS; i++)
	{
		materialGroupIDs[i] = NewtonMaterialCreateGroupID(world);
	}
	
	// Setup the material data
	NewtonMaterialSetDefaultSoftness(world, materialGroupIDs[SBMaterial_WEIGHT], materialGroupIDs[SBMaterial_SURFACE], 0.15f);
	NewtonMaterialSetDefaultElasticity(world, materialGroupIDs[SBMaterial_WEIGHT], materialGroupIDs[SBMaterial_SURFACE], 0.30f);
	NewtonMaterialSetDefaultFriction(world, materialGroupIDs[SBMaterial_WEIGHT], materialGroupIDs[SBMaterial_SURFACE], 0.05f, 0.04f);


	// setup callbacks for collisions between two material groups
	NewtonMaterialSetCollisionCallback(world,materialGroupIDs[SBMaterial_WEIGHT],materialGroupIDs[SBMaterial_SURFACE],NULL,PhysicsNewton_CollisionPuckSurfaceCB);

	///////
	// Add table
	{
		dVector tableSize(TABLE_LENGTH, TABLE_HEIGHT, TABLE_WIDTH, 0.0f);

		// create the shape and visual mesh as a common data to be re used
		NewtonCollision* const collision = CreateConvexCollision (world, dGetIdentityMatrix(), tableSize, _BOX_PRIMITIVE, materialGroupIDs[SBMaterial_SURFACE]);

		DemoMesh* const geometry = new DemoMesh("cylinder_1", scene->GetShaderCache(), collision, "wood_3.tga", "wood_3.tga", "wood_3.tga");

		dMatrix matrix = dGetIdentityMatrix();
		matrix.m_posit.m_x = 0.0f;
		matrix.m_posit.m_z = 0.0f;
		matrix.m_posit.m_y = 0.0f;
		NewtonBody* const tableBody = CreateSimpleSolid (scene, geometry, 0.0, matrix, collision, materialGroupIDs[SBMaterial_SURFACE]);

		// this is deprecated, use NewtonBodySetMassProperties
		//NewtonBodySetMassMatrix(tableBody, 0.0f, 1.0f, 1.0f, 1.0f);
		NewtonBodySetMassProperties(tableBody, 0.0f, NewtonBodyGetCollision(tableBody));

		NewtonBodySetMaterialGroupID(tableBody, materialGroupIDs[SBMaterial_SURFACE]);

		// it is not wise to se static body to continuous collision mode
		//NewtonBodySetContinuousCollisionMode(tableBody, 1);

		// do not forget to release the assets	
		geometry->Release(); 

		// the collision need to be destroy, the body is using an instance no a reference
		NewtonDestroyCollision (collision);
	}
	///////

	// Add puck
	{
		new PuckEntity (scene, materialGroupIDs[SBMaterial_WEIGHT]);
	}

	// place camera into position
	dMatrix camMatrix (dPitchMatrix(20.0f * dDegreeToRad));
	dQuaternion rot (camMatrix);
	dVector origin (CAMERA_Z, CAMERA_Y, CAMERA_X, 0.0f);
	scene->SetCameraMatrix(rot, origin);
}
