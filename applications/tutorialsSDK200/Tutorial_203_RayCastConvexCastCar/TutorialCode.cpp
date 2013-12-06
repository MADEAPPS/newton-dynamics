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
#include "FindFloor.h"
#include "TutorialCode.h"
#include "SoundManager.h"
#include "SceneManager.h"
#include "RigidBodyUtil.h"
#include "MaterialManager.h"
#include "CollisionShapeUtil.h"
#include "CreateHeightFieldEntity.h"
#include "CollectInputAndUpdateCamera.h"

enum MatrialID 
{
	m_mutiMaterial = 0,
	m_bricks,
	m_grass, 
	m_wood,
	m_metal,
};


#define CAR_WEIGHT					1800.0f	
#define TIRE_MASS					10.0f	
#define LOW_CENTER_OF_MASS_FACTOR	0.75f


// this data is the local position of the tires relative to the car origin
// usually this is saved with the exporter or the programmer can read it from the car mesh
dFloat porcheTirePositions [4][3] = 
{
	{ 1.32f, -0.55f, -0.797f}, 
	{ 1.32f, -0.55f,  0.797f},
	{-1.24f, -0.60f, -0.797f}, 
	{-1.24f, -0.60f,  0.797f}
};
dFloat cadillackirePositions [4][3] = 
{
	{ 2.00f, 0.40f, -0.9f}, 
	{ 2.00f, 0.40f,  0.9f},
	{-1.50f, 0.40f, -0.9f}, 
	{-1.50f, 0.40f,  0.9f}
};


static NewtonUserJoint* CreateRayCastCast (NewtonBody* body, SceneManager* sceneManager, dFloat tirePosit[4][3]);
static void AddTire (NewtonUserJoint* joint, const char* fileName, dVector position, SceneManager* sceneManager);

void CreateScene (NewtonWorld* world, SceneManager* sceneManager)
{
	Entity* floor;
	NewtonBody* floorBody;
	NewtonCollision* shape;
	void* materialManager;
	SoundManager* sndManager;
	PhysicsMaterialInteration matInterations;

	sndManager = sceneManager->GetSoundManager();

	// Create the material for this scene, and attach it to the Newton World
	materialManager = CreateMaterialManager (world, sndManager);

	// add the Material table
	matInterations.m_restitution = 0.6f;
	matInterations.m_staticFriction = 0.6f;
	matInterations.m_kineticFriction = 0.3f;
	matInterations.m_scrapingSound = NULL;

	matInterations.m_impactSound = sndManager->LoadSound ("metalMetal.wav");
	AddMaterilInteraction (materialManager, m_metal, m_metal, &matInterations);

	matInterations.m_impactSound = sndManager->LoadSound ("boxBox.wav");
	AddMaterilInteraction (materialManager, m_wood, m_wood, &matInterations);

	matInterations.m_impactSound = sndManager->LoadSound ("metalBox.wav");
	AddMaterilInteraction (materialManager, m_metal, m_wood, &matInterations);

	matInterations.m_impactSound = sndManager->LoadSound ("grass0.wav");
	AddMaterilInteraction (materialManager, m_wood, m_grass, &matInterations);

	matInterations.m_impactSound = sndManager->LoadSound ("boxHit.wav");
	AddMaterilInteraction (materialManager, m_wood, m_bricks, &matInterations);

	matInterations.m_impactSound = sndManager->LoadSound ("grass1.wav");
	AddMaterilInteraction (materialManager, m_metal, m_grass, &matInterations);

	matInterations.m_impactSound = sndManager->LoadSound ("metal.wav");
	AddMaterilInteraction (materialManager, m_metal, m_bricks, &matInterations);


	// Create a large body to be the floor
	floor = sceneManager->CreateEntity();

	// add scene collision from a level mesh
	int materialMap[] = {m_bricks, m_grass, m_wood,	m_metal};
	shape = CreateHeightFieldCollision (world, "h2.raw", materialMap);
	floorBody = CreateRigidBody (world, floor, shape, 0.0f);
	NewtonDestroyCollision(shape);

	// make a visual mesh for the collision data
	CreateHeightFieldMesh (shape, floor);

	// set the matrix at the origin
	dVector boxP0; 
	dVector boxP1; 
	dMatrix matrix (floor->m_curRotation, floor->m_curPosition);
	NewtonCollisionCalculateAABB (shape, &matrix[0][0], &boxP0.m_x, &boxP1.m_x); 

	// place the origin of the visual mesh at the center of the height field
	matrix.m_posit = (boxP0 + boxP1).Scale (-0.5f);
	matrix.m_posit.m_w = 1.0f;
	floor->m_curPosition = matrix.m_posit;
	floor->m_prevPosition = matrix.m_posit;

	// relocate the body;
	NewtonBodySetMatrix (floorBody, &matrix[0][0]);

	// now we will use the properties of this body to set a proper world size.
	NewtonCollisionCalculateAABB (shape, &matrix[0][0], &boxP0.m_x, &boxP1.m_x); 

	// add some extra padding
	boxP0.m_x -=  50.0f;
	boxP0.m_y -= 500.0f;
	boxP0.m_z -=  50.0f;

	boxP1.m_x +=  50.0f;
	boxP1.m_y += 500.0f;
	boxP1.m_z +=  50.0f;

	// set the new world size
	NewtonSetWorldSize (world, &boxP0[0], &boxP1[0]);


	// add some visual entities.
	dFloat y0 = FindFloor (world, 0.0f, 0.0f) + 2.0f;
	for (int i = 0; i < 1; i ++) {
		Entity* carEnt;
		NewtonBody* carBody;
		NewtonUserJoint* carController;

		carEnt = sceneManager->CreateEntity();
		carEnt->LoadMesh ("porche.dat");
		carEnt->m_curPosition.m_y = y0;
		y0 += 2.0f;
		carEnt->m_prevPosition = carEnt->m_curPosition;

		// add a body with a box shape
		shape = CreateNewtonBox (world, carEnt, m_metal);
		// the mass is the Car weight divide by the Gravity 
		carBody = CreateRigidBody (world, carEnt, shape, CAR_WEIGHT / 10.0f);
		NewtonDestroyCollision(shape);

		// Now create a RayCast Car for to control this body
		carController = CreateRayCastCast (carBody, sceneManager, porcheTirePositions);
	}


	y0 = FindFloor (world, 4.0f, 0.0f) + 2.0f;
	for (int i = 0; i < 1; i ++) {
		Entity* carEnt;
		NewtonBody* carBody;
		NewtonUserJoint* carController;

		carEnt = sceneManager->CreateEntity();
		carEnt->LoadMesh ("cadillac.dat");
		carEnt->m_curPosition.m_z = 4.0f;
		carEnt->m_curPosition.m_y = y0;
		y0 += 2.0f;
		carEnt->m_prevPosition = carEnt->m_curPosition;

		// add a body with a box shape
		shape = CreateNewtonBox (world, carEnt, m_metal);
		// the mass is the Car weight divide by the Gravity 
		carBody = CreateRigidBody (world, carEnt, shape, CAR_WEIGHT / 10.0f);
		NewtonDestroyCollision(shape);

		// Now create a RayCast Car for to control this body
		carController = CreateRayCastCast (carBody, sceneManager, cadillackirePositions);
	}

	// initialize the camera
	InitCamera (dVector (-15.0f, FindFloor (world, -15.0f, 0.0f) + 5.0f, 0.0f), dVector (1.0f, 0.0f, 0.0f));
}



void TireTransformCallback (NewtonUserJoint *car)
{
	int count;

	// the tire geometry is not aligned with the collision shape, 
	// we can use a local rotation matrix to algn the tires correctly.
	dMatrix tireYawMatrix (dYawMatrix(3.1416f * 0.5f));

	// iterate over all tire and calculate the location of the Visual Matrix
	count = DGRaycastVehicleGetTiresCount(car);
	for (int i = 0; i < count; i ++) {
		Entity* tireEnt;
		dMatrix tireMatrix;

		// get THe tie entity.
		tireEnt = (Entity*) DGRaycastVehicleGetTiresUserData (car, i);

		// Get the tire matrix form the joint and calculate the final visual matrix
		DGRaycastVehicleGetTireMatrix (car, i, &tireMatrix[0][0]);
		tireMatrix = tireYawMatrix * tireMatrix;

		// we need to convert the matrix into a quaternion rotation because our entity used quaternion to express rotations
		dQuaternion rotation (tireMatrix);
		// make sure the rotation spinning tire around the shortest path. 
		if (rotation.DotProduct (tireEnt->m_curRotation) < 0.0f) {
			rotation.Scale (-1.0f);
		}

		// move current rotation and position to previews rotation and position, and save the New rotation and position
		tireEnt->m_prevRotation = tireEnt->m_curRotation;
		tireEnt->m_curRotation = rotation;
		tireEnt->m_prevPosition = tireEnt->m_curPosition;
		tireEnt->m_curPosition = tireMatrix.m_posit;
	}
}

NewtonUserJoint* CreateRayCastCast (NewtonBody* body, SceneManager* sceneManager, dFloat tirePosit[4][3])
{
	dVector minBox;
	dVector maxBox;
	dVector origin(0, 0, 0, 1);
	dVector inertia(0, 0, 0, 1);
	NewtonUserJoint* carJoint;
	NewtonCollision* chassisCollision;

	// the first thing we need to do is to place the vehicle center of Mass on a stable position for a car

	// calculate the moment of inertia and the relative center of mass of the solid
	chassisCollision = NewtonBodyGetCollision(body);
	NewtonConvexCollisionCalculateInertialMatrix (chassisCollision, &inertia[0], &origin[0]);
	CalculateBoxdingBox (chassisCollision, minBox, maxBox);

	//displace the center of mass by some value
	origin.m_y -= 0.5f * (maxBox.m_y - minBox.m_y) * LOW_CENTER_OF_MASS_FACTOR;

	// now we Set a new center of mass for this car
	NewtonBodySetCentreOfMass (body, &origin[0]);


	// Next we need to set the internal coordinate system of the car geometry
	// this particular demo the car direction of motion is axis (1, 0, 0)
	// the car up direction (0, 1, 0);
	// the car lateral direction is the cross product of the front and vertical axis
	dMatrix chassisMatrix;
	chassisMatrix.m_front = dVector (1.0f, 0.0f, 0.0f, 0.0f);			// this is the vehicle direction of travel
	chassisMatrix.m_up	  = dVector (0.0f, 1.0f, 0.0f, 0.0f);			// this is the downward vehicle direction
	chassisMatrix.m_right = chassisMatrix.m_front * chassisMatrix.m_up;	// this is in the side vehicle direction (the plane of the wheels)
	chassisMatrix.m_posit = dVector (0.0f, 0.0f, 0.0f, 1.0f);

	// create a vehicle joint with 4 tires
	carJoint = DGRaycastVehicleCreate (4, &chassisMatrix[0][0], body);

	// we need to set a transform call back to render the tire entities
	DGRaycastVehicleSetTireTransformCallback (carJoint, TireTransformCallback);

	// now we will add four tires to this vehicle, 
	AddTire (carJoint, "leftTire.dat",  dVector (tirePosit[0][0], tirePosit[0][1], tirePosit[0][2], 0.0f), sceneManager);
	AddTire (carJoint, "rightTire.dat", dVector (tirePosit[1][0], tirePosit[1][1], tirePosit[1][2], 0.0f), sceneManager);
	AddTire (carJoint, "leftTire.dat",  dVector (tirePosit[2][0], tirePosit[2][1], tirePosit[2][2], 0.0f), sceneManager);
	AddTire (carJoint, "rightTire.dat", dVector (tirePosit[3][0], tirePosit[3][1], tirePosit[3][2], 0.0f), sceneManager);

	return carJoint;
}


void AddTire (NewtonUserJoint* joint, const char* fileName, dVector position, SceneManager* sceneManager)
{
	dFloat width;
	dFloat radius;
	Entity* tireEnt;
	dVector minBox;
	dVector maxBox;

	tireEnt = sceneManager->CreateEntity();
	tireEnt->LoadMesh (fileName);

	tireEnt->GetBBox (minBox, maxBox);

	// find the width and high of the tire shape fro the graphics file
	width = (maxBox.m_z - minBox.m_z);
	radius = (maxBox.m_x - minBox.m_x) * 0.5f;

	// set at normal tire, with all it car dynamic parameters,
	// these parameter are shosen by some trial and error experimentation
	DGRaycastVehicleAddTire (joint, tireEnt, &position[0], TIRE_MASS, radius, width, 2.5f, 0.25f, 150.0f, 5.0f, 1);
}
