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
#include "../DemoEntityManager.h"
#include "../DemoCamera.h"
#include "../PhysicsUtils.h"

#if 0
#include "SkyBox.h"
#include "dList.h"
#include "dTree.h"
#include "dHeap.h"
#include "TargaToOpenGl.h"
#include "RenderPrimitive.h"
#include "../OGLMesh.h"
#include "../SceneManager.h"
#include "../PhysicsUtils.h"
#include "../toolBox/MousePick.h"
#include "../toolBox/OpenGlUtil.h"
#include "../toolBox/DebugDisplay.h"

#define DEBRI_ID  1	

class PrefFabDebriElement
{
	public:
	dFloat m_Ixx;
	dFloat m_Iyy;
	dFloat m_Izz;
	dFloat m_mass;
	dVector m_com;

	OGLMesh* m_mesh;
	NewtonCollision* m_shape;
};

class PrefFabDebriList: public dList<PrefFabDebriElement> 
{
	public:
	PrefFabDebriList()
	{

	}

	~PrefFabDebriList()
	{
	}

	void Destroy (const NewtonWorld* newtonWorld)
	{
		for (dListNode* node = GetFirst(); node; node = node->GetNext()) {
			NewtonReleaseCollision(newtonWorld, node->GetInfo().m_shape);
			node->GetInfo().m_mesh->Release();
		}
	}
};

class PrefFabDebrisDatabase: public dTree<PrefFabDebriList, NewtonCollision*>
{
	public:
	void LoadDestructionCutterMesh (const NewtonWorld* world)
	{
		GLuint tex;
		dFloat width; 
		dFloat breadth; 

		dMatrix plane (GetIdentityMatrix());
		dMatrix texMatrix (GetIdentityMatrix());

		width = 10.0f;
		breadth = 10.0f;

		tex = LoadTexture ("destructionDetail.tga");
		texMatrix[1][1] = 4.0f /breadth;
		texMatrix[2][2] = 4.0f /breadth;
		m_meshClipper = NewtonMeshCreatePlane (world, &plane[0][0], width, breadth, tex, &texMatrix[0][0], &texMatrix[0][0]);
	}

	void Destroy (const NewtonWorld* newtonWorld)
	{
		NewtonMeshDestroy(m_meshClipper);

		while (GetRoot()) {
			GetRoot()->GetInfo().Destroy(newtonWorld);
			Remove(GetRoot());
		}
	}


	void SetBreakValue (const NewtonBody* body, dFloat impulse) 
	{
		NewtonCollision* shape;

		// set the max force value that this shape can take before the DestroyBodycallback is called 
		shape = NewtonBodyGetCollision(body);
		if (NewtonCollisionGetMaxBreakImpactImpulse(shape) > 1.0e6f) {
//			dFloat mass;
//			dFloat Ixx;
//			dFloat Iyy;
//			dFloat Izz;
//			dFloat breakImpulse;

			// set the max force as a factor of the body weight
//			NewtonBodyGetMassMatrix(body, &mass, &Ixx, &Iyy, &Izz);

			// break if the impact is higher than speed m/s
//			breakImpulse = speed * mass;
			NewtonCollisionSetMaxBreakImpactImpulse(shape, impulse);
		}
	}


	void BuildPrefabDebris (const NewtonBody* body, dFloat impulseValue)
	{
_ASSERTE (0);
/*
		NewtonWorld* world;
		NewtonCollision* collision;
		RenderPrimitive* srcPrimitive;

		// only add new collision shape

		collision = NewtonBodyGetCollision(body);
		if (!prefabDebrisDatabase.Find(collision)) {

			SetBreakValue (body, impulseValue);

			// Get the world;
			world = NewtonBodyGetWorld (body);

			// get the visual primitive
			srcPrimitive = (RenderPrimitive*) NewtonBodyGetUserData (body);

			// Recursive break the Mesh into a series of smaller meshes to resemble the debri
			dDownHeap<NewtonMesh*, dFloat> heap (64);

			heap.Push(srcPrimitive->m_specialEffect, 1.0f);
			while ((heap.GetCount() < 10) && (heap.Value(0) > 0.2f)) {
				dFloat x;
				dFloat y;
				dFloat z;
				NewtonMesh* meshA;
				NewtonMesh* meshB;
				NewtonMesh* worseMesh;
				dMatrix planeMatrix;

				// get the mesh with the longest axis
				worseMesh = heap[0]; 
				heap.Pop();

				// calculate the plane equation along teh longe axis
				NewtonMeshCalculateOOBB (worseMesh, &planeMatrix[0][0], &x, &y, &z);
				
				// randomize the clip plane a little
				dVector posit = planeMatrix.m_posit + planeMatrix.m_front.Scale (x * RandomVariable(0.5f));
				planeMatrix = planeMatrix * dYawMatrix(3.1416f * RandomVariable(1.0f));
				planeMatrix = planeMatrix * dRollMatrix(3.1416f * RandomVariable(1.0f));
				planeMatrix.m_posit = posit;


				// try to split this mesh along the plane equation
				meshA = NULL;
				meshB = NULL;
				NewtonMeshClip (worseMesh, m_meshClipper, &planeMatrix[0][0], &meshA, &meshB);
				if (meshA && meshB) {
					// is the mesh was successfully clipped add the two child mesh to the heap
					if (!(NewtonMeshIsOpenMesh(meshA) || NewtonMeshIsOpenMesh(meshB))) { 
						NewtonMeshCalculateOOBB (meshA, &planeMatrix[0][0], &x, &y, &z);
						heap.Push (meshA, x);

						NewtonMeshCalculateOOBB (meshB, &planeMatrix[0][0], &x, &y, &z);
						heap.Push (meshB, x);

						// delete the parent mesh if this si no the original mesh
						if (worseMesh != srcPrimitive->m_specialEffect) {
							NewtonMeshDestroy(worseMesh);
						}

					} else {
						// if the split utility generate degenerated meshes 
						// then delete the two degenerating child and add the parent to the heap with a low cost
						NewtonMeshDestroy(meshA);
						NewtonMeshDestroy(meshB);
						heap.Push(worseMesh, 0.0f);
					}
				} else {
					// if the clipper fail to clip the mesh then add the mesh to the heap with a low cost
					heap.Push(worseMesh, 0.0f);
				}
			}

			// get the effect mesh that is use to create the debris pieces
			dFloat Ixx;
			dFloat Iyy;
			dFloat Izz;
			dFloat mass;
			dFloat volume;
			dFloat density;
			PrefFabDebriList* prefaceDrebriComponets;

			prefaceDrebriComponets = &prefabDebrisDatabase.Insert (collision)->GetInfo();

			NewtonBodyGetMassMatrix (body, &mass, &Ixx, &Iyy, &Izz);
			volume = NewtonConvexCollisionCalculateVolume (collision);

			density = mass / volume;
			for (int i = 0; i < heap.GetCount(); i ++) {
				dFloat shapeVolume;
				NewtonMesh* mesh;
				OGLMesh* visualMesh;
				dVector inertia;
				dVector origin;


				// add a new debris entry
				PrefFabDebriElement& debriPiece = prefaceDrebriComponets->Append()->GetInfo();

				mesh = heap[i];
				visualMesh = new OGLMesh(dMesh::D_STATIC_MESH);
				visualMesh->BuildFromMesh (mesh);

				debriPiece.m_mesh = visualMesh;
				debriPiece.m_shape = NewtonCreateConvexHullFromMesh (world, mesh, 0.1f, DEBRI_ID);

				// calculate the moment of inertia and the relative center of mass of the solid
				shapeVolume = NewtonConvexCollisionCalculateVolume (debriPiece.m_shape);
				NewtonConvexCollisionCalculateInertialMatrix (debriPiece.m_shape, &inertia[0], &origin[0]);	

				debriPiece.m_mass = shapeVolume * density;
				debriPiece.m_Ixx = debriPiece.m_mass * inertia[0];
				debriPiece.m_Iyy = debriPiece.m_mass * inertia[1];
				debriPiece.m_Izz = debriPiece.m_mass * inertia[2];
				debriPiece.m_com = origin;

				NewtonMeshDestroy (mesh);
			}
		}
*/
	}

	NewtonMesh* m_meshClipper;
	static PrefFabDebrisDatabase prefabDebrisDatabase;	
};

PrefFabDebrisDatabase PrefFabDebrisDatabase::prefabDebrisDatabase;	


// destroy the clipper Mesh
static void DestroyWorldCallback(const NewtonWorld* newtonWorld)
{
	PrefFabDebrisDatabase::prefabDebrisDatabase.Destroy(newtonWorld);
}


static void DestroyThisBodyCallback (const NewtonBody* body, const NewtonJoint* contactJoint)
{
	PrefFabDebrisDatabase::dTreeNode *debrisPiecesNode;

	// find a the strongest force 
	debrisPiecesNode = PrefFabDebrisDatabase::prefabDebrisDatabase.Find(NewtonBodyGetCollision(body));
	if (debrisPiecesNode) {
		dMatrix matrix;
		NewtonWorld* world;
		SceneManager* system;
		NewtonBody* rigidBody;
		dVector veloc;
		dVector omega;


		// Get the world;
		world = NewtonBodyGetWorld (body);
		system = (SceneManager*) NewtonWorldGetUserData(world);

		NewtonBodyGetOmega(body, &omega[0]);
		NewtonBodyGetVelocity(body, &veloc[0]);
		NewtonBodyGetMatrix (body, &matrix[0][0]);

		veloc = veloc.Scale (0.25f);
		omega = omega.Scale (0.25f);

		for (PrefFabDebriList::dListNode* node = debrisPiecesNode->GetInfo().GetFirst(); node; node = node->GetNext()) {
			OGLMesh* meshInstance;
			NewtonCollision* collision;
			RenderPrimitive* primitive;
			PrefFabDebriElement& debriData = node->GetInfo();
		
			// make a visual object
			meshInstance = debriData.m_mesh;

			// create a visual geometry
			primitive = new RenderPrimitive (matrix, meshInstance);

			// save the graphics system
			system->AddModel (primitive);

			collision = debriData.m_shape;

			// calculate the moment of inertia and the relative center of mass of the solid
			//create the rigid body
			rigidBody = NewtonCreateBody (world, collision);

			// set the correct center of gravity for this body
			NewtonBodySetCentreOfMass (rigidBody, &debriData.m_com[0]);

			// set the mass matrix
			NewtonBodySetMassMatrix (rigidBody, debriData.m_mass, debriData.m_Ixx, debriData.m_Iyy, debriData.m_Izz);

			// save the pointer to the graphic object with the body.
			NewtonBodySetUserData (rigidBody, primitive);

			// assign the wood id
			//	NewtonBodySetMaterialGroupID (rigidBody, NewtonBodyGetMaterialGroupID(source));

			// set continue collision mode
			NewtonBodySetContinuousCollisionMode (rigidBody, 1);

			// set a destructor for this rigid body
			NewtonBodySetDestructorCallback (rigidBody, PhysicsBodyDestructor);

			// set the transform call back function
			NewtonBodySetTransformCallback (rigidBody, PhysicsSetTransform);

			// set the force and torque call back function
			NewtonBodySetForceAndTorqueCallback (rigidBody, PhysicsApplyGravityForce);

			// set the matrix for both the rigid body and the graphic body
			NewtonBodySetMatrix (rigidBody, &matrix[0][0]);
			PhysicsSetTransform (rigidBody, &matrix[0][0], 0);

			dVector debriOmega (omega);
			dVector debriVeloc (veloc + omega * matrix.RotateVector(debriData.m_com));

			// for now so that I can see the body
//debriVeloc = dVector (0, 0, 0, 0);
//debriVeloc.Scale (0.25f);
//debriVeloc.Omega
//omega = dVector (0, 0, 0, 0);

			NewtonBodySetOmega(rigidBody, &debriOmega[0]);
			NewtonBodySetVelocity(rigidBody, &debriVeloc[0]);
		}

		RenderPrimitive* srcPrimitive;
		srcPrimitive = (RenderPrimitive*) NewtonBodyGetUserData (body);

		// remove the old visual from graphics world
		delete srcPrimitive;
		system->Remove(srcPrimitive);

		// finally destroy this body;
		NewtonDestroyBody(world, body);

	}
}


static void SetDemoCallbacks (SceneManager& system)
{
	system.m_control = Keyboard;
	system.m_autoSleep = AutoSleep;
	system.m_showIslands = SetShowIslands;
	system.m_showContacts = SetShowContacts; 
	system.m_setMeshCollision = SetShowMeshCollision;
}


static void BuildFloorAndSceneRoot (SceneManager& system)
{
	NewtonWorld* world;
	RenderPrimitive* floor;
	NewtonBody* floorBody;
	NewtonCollision* floorCollision;
	OGLMesh* meshInstance;

	world = system.m_world;
	// /////////////////////////////////////////////////////////////////////
	//
	// create the sky box,
	system.AddModel (new SkyBox ());


	// create the the floor graphic objects
	dVector floorSize (100.0f, 2.0f, 100.0f);
	dMatrix location (GetIdentityMatrix());
	location.m_posit.m_y = -5.0f; 

	// create a box for floor 
	floorCollision = NewtonCreateBox (world, floorSize.m_x, floorSize.m_y, floorSize.m_z, 0, NULL); 

	//	meshInstance = OGLMesh::MakeBox (world, size.m_x, size.m_y, size.m_z, "GrassAndDirt.tga");
	meshInstance = new OGLMesh (floorCollision, "GrassAndDirt.tga", "metal_30.tga", "metal_30.tga");
	floor = new RenderPrimitive (location, meshInstance);
	system.AddModel (floor);
	meshInstance->Release();

	// create the the floor collision, and body with default values
	floorBody = NewtonCreateBody (world, floorCollision);
	NewtonReleaseCollision (world, floorCollision);


	// set the transformation for this rigid body
	NewtonBodySetMatrix (floorBody, &location[0][0]);

	// save the pointer to the graphic object with the body.
	NewtonBodySetUserData (floorBody, floor);

	// set a destructor for this rigid body
	NewtonBodySetDestructorCallback (floorBody, PhysicsBodyDestructor);


	// get the default material ID
	int defaultID;
	defaultID = NewtonMaterialGetDefaultGroupID (world);

	// set default material properties
	NewtonMaterialSetDefaultSoftness (world, defaultID, defaultID, 0.05f);
	NewtonMaterialSetDefaultElasticity (world, defaultID, defaultID, 0.4f);
	NewtonMaterialSetDefaultCollidable (world, defaultID, defaultID, 1);
	NewtonMaterialSetDefaultFriction (world, defaultID, defaultID, 1.0f, 0.5f);
	NewtonMaterialSetCollisionCallback (world, defaultID, defaultID, NULL, NULL, GenericContactProcess); 

	//	NewtonMaterialSetSurfaceThickness(world, materialID, materialID, 0.1f);
	NewtonMaterialSetSurfaceThickness(world, defaultID, defaultID, 0.0f);

	// set the island update callback
	NewtonSetIslandUpdateEvent (world, PhysicsIslandUpdate);

	// save the callback
	SetDemoCallbacks (system);

	InitEyePoint (dVector (1.0f, 0.0f, 0.0f), dVector (-40.0f, 10.0f, 0.0f));
}


//static void UnstableStruture (SceneManager& system, dVector location, int high)
static void BreakableStruture (SceneManager& system, dVector location, int high)
{
	dFloat plankMass;
	dFloat columnMass;

	// /////////////////////////////////////////////////////////////////////
	//
	// Build a parking lot type structure

	dVector columnBoxSize (3.0f, 1.0f, 1.0f);
	//	dVector columnBoxSize (3.0f, 3.0f, 1.0f);
	dVector plankBoxSize (6.0f, 1.0f, 6.0f);

	// create the stack
	dMatrix baseMatrix (GetIdentityMatrix());

	// get the floor position in from o the camera
	baseMatrix.m_posit = location;
	baseMatrix.m_posit.m_y += columnBoxSize.m_x;

	// set realistic (extremes in this case for 24 bits precision) masses for the different components
	// note how the newton engine handle different masses ratios without compromising stability, 
	// we recommend the application keep this ration under 100 for contacts and 50 for joints 
	columnMass = 1.0f;
	plankMass = 20.0f;
	//	 plankMass = 1.0f;

	// create a material carrier to to cou collision wit ethsi obejted
	int defaultMaterialID;
	defaultMaterialID = NewtonMaterialGetDefaultGroupID (system.m_world);

	dMatrix columAlignment (dRollMatrix(3.1416f * 0.5f));
	for (int i = 0; i < high; i ++) { 

		NewtonBody* body;
		RenderPrimitive* primitive;

		dMatrix matrix(columAlignment * baseMatrix);


		// add the 4 column
		matrix.m_posit.m_x -=  (columnBoxSize.m_z - plankBoxSize.m_x) * 0.5f;
		matrix.m_posit.m_z -=  (columnBoxSize.m_z - plankBoxSize.m_z) * 0.5f;
		body = CreateGenericSolid (system.m_world, &system, columnMass, matrix, columnBoxSize, _RANDOM_CONVEX_HULL_PRIMITIVE, defaultMaterialID);
		primitive = (RenderPrimitive*) NewtonBodyGetUserData (body);
		PrefFabDebrisDatabase::prefabDebrisDatabase.BuildPrefabDebris (body, 30.0f);
		ConvexCastPlacement (body);

		matrix.m_posit.m_x += columnBoxSize.m_z - plankBoxSize.m_x;
		body = CreateGenericSolid (system.m_world, &system, columnMass, matrix, columnBoxSize, _RANDOM_CONVEX_HULL_PRIMITIVE, defaultMaterialID);
		primitive = (RenderPrimitive*) NewtonBodyGetUserData (body);
		PrefFabDebrisDatabase::prefabDebrisDatabase.BuildPrefabDebris (body, 30.0f);
		ConvexCastPlacement (body);

		matrix.m_posit.m_z += columnBoxSize.m_z - plankBoxSize.m_z;		
		body = CreateGenericSolid (system.m_world, &system, columnMass, matrix, columnBoxSize, _RANDOM_CONVEX_HULL_PRIMITIVE, defaultMaterialID);
		primitive = (RenderPrimitive*) NewtonBodyGetUserData (body);
		PrefFabDebrisDatabase::prefabDebrisDatabase.BuildPrefabDebris (body, 30.0f);
		ConvexCastPlacement (body);

		matrix.m_posit.m_x -= columnBoxSize.m_z - plankBoxSize.m_x;
		body = CreateGenericSolid (system.m_world, &system, columnMass, matrix, columnBoxSize, _RANDOM_CONVEX_HULL_PRIMITIVE, defaultMaterialID);
		primitive = (RenderPrimitive*) NewtonBodyGetUserData (body);
		PrefFabDebrisDatabase::prefabDebrisDatabase.BuildPrefabDebris (body, 30.0f);
		ConvexCastPlacement (body);

		// add a plank
		dVector size (plankBoxSize);
		size.m_x *= 0.85f;
		size.m_z *= 0.85f;
		body = CreateGenericSolid (system.m_world, &system, plankMass, baseMatrix, size, _BOX_PRIMITIVE, defaultMaterialID);
		primitive = (RenderPrimitive*) NewtonBodyGetUserData (body);
		PrefFabDebrisDatabase::prefabDebrisDatabase.BuildPrefabDebris (body, 80.0f);
		ConvexCastPlacement (body);

		// set up for another level
		baseMatrix.m_posit.m_y += (columnBoxSize.m_x + plankBoxSize.m_y);
	}

/*
	dFloat mass;
	NewtonBody* body;
	PrimitiveType type = _BOX_PRIMITIVE;
	dVector size (1.0f, 2.0f, 1.0f);
	dMatrix matrix (GetIdentityMatrix());

	mass = 10.0f;
	matrix.m_posit = location;
	matrix.m_posit.m_y = FindFloor (system.m_world, matrix.m_posit.m_x, matrix.m_posit.m_z) + baseMatrix.m_posit.m_y + 35.0f; 
//	body = CreateGenericSolid (system.m_world, &system, mass, matrix, size, type, defaultMaterialID);
*/
}



void PrefabSimpleDestruction(SceneManager& system)
{
	NewtonWorld* world;
	world = system.m_world;

	// create the sky box and the floor,
	BuildFloorAndSceneRoot (system);

	// save the system wit the world
	NewtonWorldSetUserData(system.m_world, &system);

	// Set a world destruction callback so that we can destroy all assets created for special effects
	NewtonWorldSetDestructorCallBack (system.m_world, DestroyWorldCallback);

	// Set a Function callback for when a Convex Body collision is destroyed if the force exceeded the Max break value
	NewtonSetDestroyBodyByExeciveForce (system.m_world, DestroyThisBodyCallback); 

	// this will load a assets to create destruction special effects 
	PrefFabDebrisDatabase::prefabDebrisDatabase.LoadDestructionCutterMesh (system.m_world);

	BreakableStruture (system, dVector (-10.0f, 0.0f, -10.0f, 0.0f), 2);
	BreakableStruture (system, dVector ( 10.0f, 0.0f, -10.0f, 0.0f), 1);
	BreakableStruture (system, dVector (-10.0f, 0.0f,  10.0f, 0.0f), 3);
	BreakableStruture (system, dVector ( 10.0f, 0.0f,  10.0f, 0.0f), 3);
}

#endif





