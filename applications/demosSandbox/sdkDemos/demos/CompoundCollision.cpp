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
#include "OpenGlUtil.h"
#include "SkyBox.h"
#include "DemoMesh.h"
#include "DemoEntityManager.h"
#include "DemoCamera.h"
#include "PhysicsUtils.h"
#include "HeightFieldPrimitive.h"


class dShowAllSubShapes: public dCustomListener
{
	public:
	dShowAllSubShapes(DemoEntityManager* const scene)
		:dCustomListener(scene->GetNewton(), "Pick compound subs shapes")
		,m_body(NULL)
		,m_mouseDown(false)
	{
	}

	void PreUpdate(dFloat timestep)
	{
		NewtonWorld* const workd = GetWorld();
		DemoEntityManager* const scene = (DemoEntityManager*)NewtonWorldGetUserData(workd);

		bool mouseClick = scene->GetMouseKeyState(0);
		if (!m_mouseDown && mouseClick) {
			if (!m_body) {
				// pick a body form teh scene
				int mouseX;
				int mouseY;

				DemoCamera* const camera = scene->GetCamera();
				scene->GetMousePosition(mouseX, mouseY);

				dFloat x = dFloat(mouseX);
				dFloat y = dFloat(mouseY);
				dVector p0(camera->ScreenToWorld(dVector(x, y, 0.0f, 0.0f)));
				dVector p1(camera->ScreenToWorld(dVector(x, y, 1.0f, 0.0f)));
/*
				dFloat param;
				dVector posit;
				dVector normal;
				NewtonBody* const body = MousePickBody(scene->GetNewton(), p0, p1, param, posit, normal);
				if (body) {

					// if we picked a body see if it has a compound shape
					NewtonCollision* const collision = NewtonBodyGetCollision(body);
					int NewtonCollisionGetType(const NewtonCollision* const collision);
					if (NewtonCollisionGetType(collision) == SERIALIZE_ID_COMPOUND) {
						m_body = body;
						RayCastAllSubShapes(p0, p1);
					}
				}
*/
				RayCastCompoundsAllSubShapes(p0, p1);
			}

		}
		else if (m_mouseDown && !mouseClick) {
			// release the body
			m_body = NULL;
		}

		m_mouseDown = mouseClick;
	}

	static dFloat PickCompound(const NewtonBody* const body, const NewtonCollision* const shapeHit, const dFloat* const hitContact, const dFloat* const hitNormal, dLong collisionID, void* const userData, dFloat intersectParam)
	{
		NewtonCollision* const collision = NewtonBodyGetCollision(body);
		if (NewtonCollisionGetType(collision) == SERIALIZE_ID_COMPOUND) {
			dShowAllSubShapes* const me = (dShowAllSubShapes*)userData;
			if (intersectParam < me->m_param) {
				me->m_param = intersectParam;
				me->m_body = body;
				return intersectParam;
			}
		}
		return 1.2f;
	}

	void RayCastCompoundsAllSubShapes(const dVector& origin, const dVector& end)
	{
		m_param = 1.0f;
		m_body = NULL;
		NewtonWorld* const world = GetWorld();
		NewtonWorldRayCast(world, &origin[0], &end[0], PickCompound, this, NULL, 0);
		if (m_body) {
			// code to be implements here
			dMatrix matrix;
			NewtonBodyGetMatrix(m_body, &matrix[0][0]);
			dVector localP0(matrix.UntransformVector(origin));
			dVector localP1(matrix.UntransformVector(end));

			NewtonCollision* const compoundCollision = NewtonBodyGetCollision(m_body);
			dAssert(NewtonCollisionGetType(compoundCollision) == SERIALIZE_ID_COMPOUND);
			for (void* node = NewtonCompoundCollisionGetFirstNode(compoundCollision); node; node = NewtonCompoundCollisionGetNextNode(compoundCollision, node)) {
				dVector normal;
				dLong attribute;
				NewtonCollision* const subShape = NewtonCompoundCollisionGetCollisionFromNode(compoundCollision, node);
				dFloat xxx = NewtonCollisionRayCast(subShape, &localP0[0], &localP1[0], &normal[0], &attribute);
				if (xxx < 1.0f) {
					dTrace (("sub shape hit\n"))
				}
			}
		}
	}


	const NewtonBody* m_body;
	dFloat m_param;
	bool m_mouseDown;
};

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
		dVector origin(0.0f);
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
				dVector force(0.0f);
				dVector posit(0.0f);
				dVector normal(0.0f);
				NewtonMaterialGetContactForce (material, body, &force[0]);
				NewtonMaterialGetContactPositionAndNormal (material, body, &posit[0], &normal[0]);
				// the torque on this contact is
				dVector torque ((origin - posit).CrossProduct(force));

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

static int OnSubShapeAABBOverlapTest (const NewtonJoint* const contact, dFloat timestep, const NewtonBody* const body0, const void* const collsionNode0, const NewtonBody* const body1, const void* const collsionNode1, int threadIndex)
{
	return 1;
}


static void TesselateTriangle (int level, const dVector& p0, const dVector& p1, const dVector& p2, int& count, dFloat* const ouput)
{
	if (level) {
		dAssert (dAbs (p0.DotProduct3(p0) - dFloat (1.0f)) < dFloat (1.0e-4f));
		dAssert (dAbs (p1.DotProduct3(p1) - dFloat (1.0f)) < dFloat (1.0e-4f));
		dAssert (dAbs (p2.DotProduct3(p2) - dFloat (1.0f)) < dFloat (1.0e-4f));
		dVector p01 ((p0 + p1).Scale (0.5f));
		dVector p12 ((p1 + p2).Scale (0.5f));
		dVector p20 ((p2 + p0).Scale (0.5f));

		p01 = p01.Scale (1.0f / dSqrt (p01.DotProduct3(p01)));
		p12 = p12.Scale (1.0f / dSqrt (p12.DotProduct3(p12)));
		p20 = p20.Scale (1.0f / dSqrt (p20.DotProduct3(p20)));

		dAssert (dAbs (p01.DotProduct3(p01) - dFloat (1.0f)) < dFloat (1.0e-4f));
		dAssert (dAbs (p12.DotProduct3(p12) - dFloat (1.0f)) < dFloat (1.0e-4f));
		dAssert (dAbs (p20.DotProduct3(p20) - dFloat (1.0f)) < dFloat (1.0e-4f));

		TesselateTriangle (level - 1, p0,  p01, p20, count, ouput);
		TesselateTriangle (level - 1, p1,  p12, p01, count, ouput);
		TesselateTriangle (level - 1, p2,  p20, p12, count, ouput);
		TesselateTriangle (level - 1, p01, p12, p20, count, ouput);
	} else {
		dVector p ((p0 + p1 + p2).Scale (1.0f / 3.0f));
		p = p.Scale (1.0f / dSqrt (p.DotProduct3(p)));
		ouput[count * 3 + 0] = p.m_x;
		ouput[count * 3 + 1] = p.m_y;
		ouput[count * 3 + 2] = p.m_z;
		count ++;
		dAssert (count < 700);
	}
}

static int BuildPointArray (dFloat* const points)
{
	dVector p0 ( dFloat (1.0f), dFloat (0.0f), dFloat (0.0f), dFloat (0.0f)); 
	dVector p1 (-dFloat (1.0f), dFloat (0.0f), dFloat (0.0f), dFloat (0.0f)); 
	dVector p2 ( dFloat (0.0f), dFloat (1.0f), dFloat (0.0f), dFloat (0.0f)); 
	dVector p3 ( dFloat (0.0f),-dFloat (1.0f), dFloat (0.0f), dFloat (0.0f));
	dVector p4 ( dFloat (0.0f), dFloat (0.0f), dFloat (1.0f), dFloat (0.0f));
	dVector p5 ( dFloat (0.0f), dFloat (0.0f),-dFloat (1.0f), dFloat (0.0f));

	int level = 3;
	int count = 0;
	TesselateTriangle (level, p4, p0, p2, count, points);
	TesselateTriangle (level, p4, p2, p1, count, points);
	TesselateTriangle (level, p4, p1, p3, count, points);
	TesselateTriangle (level, p4, p3, p0, count, points);
	TesselateTriangle (level, p5, p2, p0, count, points);
	TesselateTriangle (level, p5, p1, p2, count, points);
	TesselateTriangle (level, p5, p3, p1, count, points);
	TesselateTriangle (level, p5, p0, p3, count, points);
	return count;
}


static void MakeFunnyCompound (DemoEntityManager* const scene, const dVector& origin)
{
	NewtonWorld* const world = scene->GetNewton();

	// create an empty compound collision
	NewtonCollision* const compound = NewtonCreateCompoundCollision (world, 0);
	
	dFloat points[1000][3];
	int pointsCount = BuildPointArray (&points[0][0]);

	NewtonCompoundCollisionBeginAddRemove(compound);	

	// add a bunch of convex collision at random position and orientation over the surface of a big sphere
	dFloat radio = 5.0f;
	for (int i = 0 ; i < pointsCount; i ++) {
		NewtonCollision* collision = NULL;

		dFloat pitch = dGaussianRandom  (1.0f) * 2.0f * dPi;
		dFloat yaw = dGaussianRandom  (1.0f) * 2.0f * dPi;
		dFloat roll = dGaussianRandom  (1.0f) * 2.0f * dPi;

		dFloat x = points[i][0] * radio;
		dFloat y = points[i][1] * radio;
		dFloat z = points[i][2] * radio;

		dMatrix matrix (dPitchMatrix (pitch) * dYawMatrix (yaw) * dRollMatrix (roll));
		matrix.m_posit = dVector (x, y, z, 1.0f);
		int r = dRand();	
//		int r = 0;
		switch (r % 4) 
		{
			case 0:
			{
				collision = NewtonCreateSphere(world, 0.5f, 0, &matrix[0][0]) ;
				break; 
			}

			case 1:
			{
				collision = NewtonCreateCapsule(world, 0.3f, 0.2f, 0.5f, 0, &matrix[0][0]) ;
				break; 
			}

			case 2:
			{
				collision = NewtonCreateCylinder(world, 0.25f, 0.5f, 0.25f, 0, &matrix[0][0]) ;
				break; 
			}

			case 3:
			{
				collision = NewtonCreateCone(world, 0.25f, 0.25f, 0, &matrix[0][0]) ;
				break; 
			}

			default:
				collision = NewtonCreateBox(world, 0.5f, 0.5f, 0.5f, 0, &matrix[0][0]) ;
				break; 

		}

		dAssert (collision);
		// we can set a collision id, and use data per sub collision 
		NewtonCollisionSetUserID(collision, i);
		//NewtonCollisionSetUserData(collision, (void*) i);

		// add this new collision 
		NewtonCompoundCollisionAddSubCollision (compound, collision);
		NewtonDestroyCollision(collision);
	}
	// finish adding shapes
	NewtonCompoundCollisionEndAddRemove(compound);	

#if 1
// test the interface
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

		void* handle1 = NewtonCompoundCollisionGetNodeByIndex (compound, pointsCount / 3);
		void* handle2 = NewtonCompoundCollisionGetNodeByIndex (compound, pointsCount * 2 / 3);
		NewtonCollision* const shape1 = NewtonCompoundCollisionGetCollisionFromNode (compound, handle1);
		NewtonCollision* const shape2 = NewtonCompoundCollisionGetCollisionFromNode (compound, handle2);

		NewtonCollision* const copyShape1 = NewtonCollisionCreateInstance (shape1);
		NewtonCollision* const copyShape2 = NewtonCollisionCreateInstance (shape2);

		// you can also remove shape by their index
		void* handle3 = NewtonCompoundCollisionGetNodeByIndex (compound, pointsCount / 4);
		if (handle3) {
			NewtonCompoundCollisionRemoveSubCollisionByIndex (compound, pointsCount / 4);	
		}
		void* handle4 = NewtonCompoundCollisionGetNodeByIndex (compound, pointsCount * 3 / 4);	
		if (handle4) {
			NewtonCompoundCollisionRemoveSubCollisionByIndex (compound, pointsCount * 3 / 4);	
		}

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
#endif

	// for now we will simple make simple Box,  make a visual Mesh
	DemoMesh* const visualMesh = new DemoMesh ("big ball", scene->GetShaderCache(), compound, "metal_30.tga", "metal_30.tga", "metal_30.tga");

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
			NewtonBody* const body =  CreateSimpleSolid (scene, visualMesh, 10.0f, matrix, compound, 0);
			if ((ix == 0) && (iz == 0)) {
				// test general scaling
				NewtonCollision* const shape = NewtonBodyGetCollision(body);
				NewtonCollisionSetScale(shape, 1.5f, 0.75f, 1.0f);
				DemoEntity* const entity = (DemoEntity*)NewtonBodyGetUserData(body);
				DemoMesh* const mesh = new DemoMesh ("big ball", scene->GetShaderCache(), shape, "metal_30.tga", "metal_30.tga", "metal_30.tga");
				entity->SetMesh(mesh, entity->GetMeshMatrix());
				mesh->Release();
			}
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
	CreateHeightFieldTerrain(scene, HEIGHTFIELD_DEFAULT_SIZE, HEIGHTFIELD_DEFAULT_CELLSIZE, 1.5f, 0.2f, 200.0f, -50.0f);

	int defaultMaterialID = NewtonMaterialGetDefaultGroupID (scene->GetNewton());

	// set a material callback to get the colliding shape
	NewtonMaterialSetDefaultElasticity(scene->GetNewton(), defaultMaterialID, defaultMaterialID, 0.1f);
	NewtonMaterialSetCollisionCallback (scene->GetNewton(), defaultMaterialID, defaultMaterialID, NULL, OnGettingTheCollisionSubShapeFromMaterialCallback);
	NewtonMaterialSetCompoundCollisionCallback(scene->GetNewton(), defaultMaterialID, defaultMaterialID, OnSubShapeAABBOverlapTest);

	dMatrix camMatrix (dRollMatrix(-20.0f * dDegreeToRad) * dYawMatrix(-45.0f * dDegreeToRad));
	dQuaternion rot (camMatrix);
	dVector origin (100.0f, 0.0f, 100.0f, 0.0f);
	dFloat hight = 1000.0f;
	origin = FindFloor (scene->GetNewton(), dVector (origin.m_x, hight, origin .m_z, 0.0f), hight * 2);
	origin.m_y += 10.0f;

	dVector location (origin);
	location.m_x += 40.0f;
	location.m_z += 40.0f;

	int count = 5;
	dVector size (0.5f, 0.5f, 0.75f, 0.0f);
	dMatrix shapeOffsetMatrix (dGetIdentityMatrix());
	AddPrimitiveArray(scene, 10.0f, location, size, count, count, 5.0f, _BOX_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
	AddPrimitiveArray(scene, 10.0f, location, size, count, count, 5.0f, _BOX_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
	AddPrimitiveArray(scene, 10.0f, location, size, count, count, 5.0f, _CAPSULE_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
	AddPrimitiveArray(scene, 10.0f, location, size, count, count, 5.0f, _CYLINDER_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
	AddPrimitiveArray(scene, 10.0f, location, size, count, count, 5.0f, _CONE_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);

	// this crash temporarily (I need to make the compound use shape Instance)
	MakeFunnyCompound(scene, location);

	// add listenr for ray tracing all sub shapes ofa compound
	new dShowAllSubShapes(scene);

	// place camera into position
//	dQuaternion rot;
//	dVector origin (-40.0f, 10.0f, 0.0f, 0.0f);
//origin.m_x += 20.0f;	
//origin.m_z += 20.0f;	
	scene->SetCameraMatrix(rot, origin);
	//ExportScene (scene->GetNewton(), "test1.ngd");
}


