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
#include "OpenGlUtil.h"
#include "SkyBox.h"
#include "DemoMesh.h"
#include "DemoEntityManager.h"
#include "DemoCamera.h"
#include "PhysicsUtils.h"
#include "HeightFieldPrimitive.h"


static void GetCollisionSubShape(const NewtonJoint* const contactJoint, NewtonBody* const body)
{
	NewtonCollisionInfoRecord collisionInfo;

	NewtonCollision* const collision = NewtonBodyGetCollision(body);
	NewtonCollisionGetInfo (collision, &collisionInfo);


	int count = 0;
	NewtonCollision* collidingSubShapeArrar[32];

	// see if this is a compound collision or any other collision with sub collision shapes  
	if (collisionInfo.m_collisionType == SERIALIZE_ID_COMPOUND) {

		// to get the torque we need the center of gravity in global space
		dVector origin;
		dMatrix bodyMatrix;
		NewtonBodyGetMatrix(body, &bodyMatrix[0][0]);
		NewtonBodyGetCentreOfMass(body, &origin[0]);
		origin = bodyMatrix.TransformVector(origin);

		for (void* contact = NewtonContactJointGetFirstContact (contactJoint); contact; contact = NewtonContactJointGetNextContact (contactJoint, contact)) {
			// get the material of this contact, 
			// this part contain all contact information, the sub colliding shape, 
			NewtonMaterial* const material = NewtonContactGetMaterial (contact);
			NewtonCollision* const subShape = NewtonMaterialGetBodyCollidingShape (material, body);
			int i = count - 1;
			for (; i >= 0; i --) {
				if (collidingSubShapeArrar[i] == subShape) {
					break;
				}
			}
			if (i < 0) {
				collidingSubShapeArrar[count] = subShape;
				count ++;
				dAssert (count < int (sizeof (collidingSubShapeArrar) / sizeof (collidingSubShapeArrar[0])));

				// you can also get the forces here, however when tho function is call form a contact material
				// we can only get resting forces, impulsive forces can not be read here since they has no being calculated yet.
				// whoever if this function function is call after the NetwonUpdate they the application can read the contact force, that was applied to each contact point
				dVector force;
				dVector posit;
				dVector normal;
				NewtonMaterialGetContactForce (material, body, &force[0]);
				NewtonMaterialGetContactPositionAndNormal (material, body, &posit[0], &normal[0]);
				// the torque on this contact is
				dVector torque ((origin - posit) * force);

				// do what ever you want wit this  


			}
		}
	}

	// here we should have an array of all colling sub shapes
	if (count) {
		// do what you need with this sub shape list
	}
}


/*
static void GettingCollisionSubShapesAfteNetwonUpdate (NewtonWorld* const world)
{
	for (NewtonBody* body = NewtonWorldGetFirstBody(world); body; body = NewtonWorldGetNextBody(world, body)) {
		for (NewtonJoint* contactJoint = NewtonBodyGetFirstContactJoint(body); contactJoint; contactJoint = NewtonBodyGetNextContactJoint(body, contactJoint)) {
			// this call should report the correct reaction forces in each call
			GetCollisionSubShape(contactJoint, body);
		}
	}
}
*/

static void OnGettingTheCollisionSubShapeFromMaterialCallback (const NewtonJoint* const contactJoint, dFloat timestep, int threadIndex)
{
	NewtonBody* const body0 = NewtonJointGetBody0(contactJoint);
	GetCollisionSubShape(contactJoint, body0);

	NewtonBody* const body1 = NewtonJointGetBody1(contactJoint);
	GetCollisionSubShape(contactJoint, body1);
}

static int OnSubShapeAABBOverlapTest (const NewtonMaterial* const material, const NewtonBody* const body0, const void* const collsionNode0, const NewtonBody* const body1, const void* const collsionNode1, int threadIndex)
{
	return 1;
}


static void MakeFunnyCompound (DemoEntityManager* const scene, const dVector& origin)
{
	NewtonWorld* const world = scene->GetNewton();

	// create an empty compound collision
	NewtonCollision* const compound = NewtonCreateCompoundCollision (world, 0);
	
	
#if 1
	NewtonCompoundCollisionBeginAddRemove(compound);	

	// add a bunch of convex collision at random position and orientation over the surface of a big sphere
	float radio = 5.0f;
	for (int i = 0 ; i < 300; i ++) {
		NewtonCollision* collision = NULL;

		float pitch = RandomVariable (1.0f) * 2.0f * 3.1416f;
		float yaw = RandomVariable (1.0f) * 2.0f * 3.1416f;
		float roll = RandomVariable (1.0f) * 2.0f * 3.1416f;

		float x = RandomVariable (0.5f);
		float y = RandomVariable (0.5f);
		float z = RandomVariable (0.5f);
		if ((x == 0.0f) && (y == 0.0f) && (z == 0.0f)){
			x = 0.1f;
		}
		dVector p (x, y, z, 1.0f) ;
		p = p.Scale (radio / dSqrt (p % p));

		dMatrix matrix (dPitchMatrix (pitch) * dYawMatrix (yaw) * dRollMatrix (roll));
		matrix.m_posit = p;
		int r = dRand();	
		switch ((r >>2) & 3) 
		{
			case 0:
			{
				collision = NewtonCreateSphere(world, 0.5, 0, &matrix[0][0]) ;
				break; 
			}

			case 1:
			{
				collision = NewtonCreateCapsule(world, 0.3f, 0.2f, 0.5f, 0, &matrix[0][0]) ;
				break; 
			}

			case 2:
			{
				collision = NewtonCreateCylinder(world, 0.25, 0.5, 0.25, 0, &matrix[0][0]) ;
				break; 
			}

			case 3:
			{
				collision = NewtonCreateCone(world, 0.25, 0.25, 0, &matrix[0][0]) ;
				break; 
			}
		}

		dAssert (collision);
		// we can set a collision id, and use data per sub collision 
		NewtonCollisionSetUserID(collision, i);
		NewtonCollisionSetUserData(collision, (void*) i);

		// add this new collision 
		NewtonCompoundCollisionAddSubCollision (compound, collision);
		NewtonDestroyCollision(collision);
	}
	// finish adding shapes
	NewtonCompoundCollisionEndAddRemove(compound);	

	{
		// remove the first 10 shapes
		// test remove shape form a compound
		NewtonCompoundCollisionBeginAddRemove(compound);	
		void* node = NewtonCompoundCollisionGetFirstNode(compound);
		for (int i = 0; i < 10; i ++) {
			//NewtonCollision* const collision = NewtonCompoundCollisionGetCollisionFromNode(compound, node);
			void* const nextNode = NewtonCompoundCollisionGetNextNode(compound, node);
			NewtonCompoundCollisionRemoveSubCollision(compound, node);
			node = nextNode;
		}
		// finish remove 

		void* handle1 = NewtonCompoundCollisionGetNodeByIndex (compound, 30);
		void* handle2 = NewtonCompoundCollisionGetNodeByIndex (compound, 100);
		NewtonCollision* const shape1 = NewtonCompoundCollisionGetCollisionFromNode (compound, handle1);
		NewtonCollision* const shape2 = NewtonCompoundCollisionGetCollisionFromNode (compound, handle2);

		NewtonCollision* const copyShape1 = NewtonCollisionCreateInstance (shape1);
		NewtonCollision* const copyShape2 = NewtonCollisionCreateInstance (shape2);

		// you can also remove shape by their index
		NewtonCompoundCollisionRemoveSubCollisionByIndex (compound, 30);	
		NewtonCompoundCollisionRemoveSubCollisionByIndex (compound, 100);	

		handle1 = NewtonCompoundCollisionAddSubCollision (compound, copyShape1);
		handle2 = NewtonCompoundCollisionAddSubCollision (compound, copyShape2);
		NewtonDestroyCollision(copyShape1);
		NewtonDestroyCollision(copyShape2);

		NewtonCompoundCollisionEndAddRemove(compound);	
	}

	{
		// show how to modify the children of a compound collision
		NewtonCompoundCollisionBeginAddRemove(compound);	
		for (void* node = NewtonCompoundCollisionGetFirstNode(compound); node; node = NewtonCompoundCollisionGetNextNode(compound, node)) { 
			NewtonCollision* const collision = NewtonCompoundCollisionGetCollisionFromNode(compound, node);
			// you can scale, change the matrix, change the inertia, do anything you want with the change
			NewtonCollisionSetUserData(collision, NULL);
		}
		NewtonCompoundCollisionEndAddRemove(compound);	
	}

//	NewtonCollisionSetScale(compound, 0.5f, 0.25f, 0.125f);

#else 

	//test Yeside compound shape shape
	//	- Rotation="1.5708 -0 0" Translation="0 0 0.024399" Size="0.021 0.096" Pos="0 0 0.115947"
	//	- Rotation="1.5708 -0 0" Translation="0 0 0.056366" Size="0.195 0.024" Pos="0 0 0.147914"
	//	- Rotation="1.5708 -0 0" Translation="0 0 -0.056366" Size="0.0065 0.07 Pos="0 0 0.035182"

	NewtonCompoundCollisionBeginAddRemove(compound);	

	NewtonCollision* collision;
	dMatrix offsetMatrix (dPitchMatrix(1.5708f));
	offsetMatrix.m_posit.m_z = 0.115947f;
	collision = NewtonCreateCylinder (world, 0.021f, 0.096f, 0, &offsetMatrix[0][0]) ;
	NewtonCompoundCollisionAddSubCollision (compound, collision);
	NewtonDestroyCollision(collision);

	offsetMatrix.m_posit.m_z = 0.035182f;
	collision = NewtonCreateCylinder (world, 0.0065f, 0.07f, 0, &offsetMatrix[0][0]) ;
	NewtonCompoundCollisionAddSubCollision (compound, collision);
	NewtonDestroyCollision(collision);

	offsetMatrix.m_posit.m_z = 0.147914f;
	collision = NewtonCreateCylinder (world, 0.195f, 0.024f, 0, &offsetMatrix[0][0]) ;
	NewtonCompoundCollisionAddSubCollision (compound, collision);
	NewtonDestroyCollision(collision);

	NewtonCompoundCollisionEndAddRemove(compound);	

#endif



	// for now we will simple make simple Box,  make a visual Mesh
	DemoMesh* const visualMesh = new DemoMesh ("big ball", compound, "metal_30.tga", "metal_30.tga", "metal_30.tga");

	int instaceCount = 2;
	dMatrix matrix (dGetIdentityMatrix());
	matrix.m_posit = origin;
	for (int ix = 0; ix < instaceCount; ix ++) {
		for (int iz = 0; iz < instaceCount; iz ++) {
			dFloat y = origin.m_y;
			dFloat x = origin.m_x + (ix - instaceCount/2) * 15.0f;
			dFloat z = origin.m_z + (iz - instaceCount/2) * 15.0f;
			matrix.m_posit = FindFloor (world, dVector (x, y + 10.0f, z, 0.0f), 20.0f); ;
			matrix.m_posit.m_y += 15.0f;
			CreateSimpleSolid (scene, visualMesh, 10.0f, matrix, compound, 0);
		}
	}
	visualMesh->Release();

	NewtonDestroyCollision(compound);
}



void CompoundCollision (DemoEntityManager* const scene)
{
	// load the skybox
	scene->CreateSkyBox();

	// load the scene from a ngd file format
//	CreateLevelMesh (scene, "flatPlane.ngd", true);
//	CreateLevelMesh (scene, "playground.ngd", true);
	//	CreateLevelMesh (scene, "sponza.ngd", true);
	CreateHeightFieldTerrain(scene, HEIGHTFIELD_DEFAULT_SIZE, HEIGHTFIELD_DEFAULT_CELLSIZE,
							 1.5f, 0.2f, 200.0f, -50.0f);

	int defaultMaterialID = NewtonMaterialGetDefaultGroupID (scene->GetNewton());

	// set a material callback to get the colliding shape
	NewtonMaterialSetCollisionCallback (scene->GetNewton(), defaultMaterialID, defaultMaterialID, NULL, OnGettingTheCollisionSubShapeFromMaterialCallback);
	NewtonMaterialSetCompoundCollisionCallback(scene->GetNewton(), defaultMaterialID, defaultMaterialID, OnSubShapeAABBOverlapTest);

	dMatrix camMatrix (dRollMatrix(-20.0f * 3.1416f /180.0f) * dYawMatrix(-45.0f * 3.1416f /180.0f));
	dQuaternion rot (camMatrix);
	dVector origin (100.0f, 0.0f, 100.0f, 0.0f);
	dFloat hight = 1000.0f;
	origin = FindFloor (scene->GetNewton(), dVector (origin.m_x, hight, origin .m_z, 0.0f), hight * 2);
	origin.m_y += 10.0f;

	dVector location (origin);
	location.m_x += 40.0f;
	location.m_z += 40.0f;

	// this crash temporarily (I need to make the compound use shape Instance)
	MakeFunnyCompound (scene, location);

	int count = 5;
	dVector size (0.5f, 0.5f, 0.75f, 0.0f);
	dMatrix shapeOffsetMatrix (dGetIdentityMatrix());
	AddPrimitiveArray(scene, 10.0f, location, size, count, count, 5.0f, _BOX_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
	AddPrimitiveArray(scene, 10.0f, location, size, count, count, 5.0f, _BOX_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
	AddPrimitiveArray(scene, 10.0f, location, size, count, count, 5.0f, _CAPSULE_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
	AddPrimitiveArray(scene, 10.0f, location, size, count, count, 5.0f, _CYLINDER_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
	AddPrimitiveArray(scene, 10.0f, location, size, count, count, 5.0f, _CONE_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);


	// place camera into position
//	dQuaternion rot;
//	dVector origin (-40.0f, 10.0f, 0.0f, 0.0f);
	scene->SetCameraMatrix(rot, origin);
	//ExportScene (scene->GetNewton(), "../../../media/test1.ngd");

}


