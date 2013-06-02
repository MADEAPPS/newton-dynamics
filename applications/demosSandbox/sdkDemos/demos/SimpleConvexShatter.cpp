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
#include "TargaToOpenGl.h"
#include "DemoMesh.h"
#include "DemoEntityManager.h"
#include "DemoCamera.h"
#include "PhysicsUtils.h"

#define INITIAL_DELAY				1000
//#define INITIAL_DELAY				0
#define NUMBER_OF_INTERNAL_PARTS		20
//#define NUMBER_OF_INTERNAL_PARTS		4

#define BREAK_FORCE_IN_GRAVITIES	10
//#define BREAK_FORCE_IN_GRAVITIES	1

#if 0
static void CreateSimpleVoronoiShatter (DemoEntityManager* const scene)
{
	// create a collision primitive
//	dVector size (2.0f, 2.0f, 2.0f);
//	dVector size = dVector (10.0f, 0.5f, 10.0f, 0.0f);
	dVector size = dVector (5.0f, 5.0f, 5.0f, 0.0f);
	NewtonWorld* const world = scene->GetNewton();

//	NewtonCollision* const collision = CreateConvexCollision (world, GetIdentityMatrix(), size, _BOX_PRIMITIVE, 0);
	NewtonCollision* const collision = CreateConvexCollision (world, GetIdentityMatrix(), size, _CAPSULE_PRIMITIVE, 0);
//	NewtonCollision* const collision = CreateConvexCollision (world, GetIdentityMatrix(), size, _SPHERE_PRIMITIVE, 0);
//	NewtonCollision* const collision = CreateConvexCollision (world, GetIdentityMatrix(), size, _REGULAR_CONVEX_HULL_PRIMITIVE, 0);
//	NewtonCollision* const collision = CreateConvexCollision (world, GetIdentityMatrix(), size, _RANDOM_CONVEX_HULL_PRIMITIVE, 0);
	
	

	// create a newton mesh from the collision primitive
	NewtonMesh* const mesh = NewtonMeshCreateFromCollision(collision);

	// apply a simple Box Mapping
	int tex0 = LoadTexture("reljef.tga");
	NewtonMeshApplyBoxMapping(mesh, tex0, tex0, tex0);

	// pepper the bing box of the mesh with random points
	dVector points[NUMBER_OF_INTERNAL_PARTS + 100];
	int count = 0;

	while (count < NUMBER_OF_INTERNAL_PARTS) {
		dFloat x = RandomVariable(size.m_x);
		dFloat y = RandomVariable(size.m_y);
		dFloat z = RandomVariable(size.m_z);
		if ((x <= size.m_x) && (x >= -size.m_x) && (y <= size.m_y) && (y >= -size.m_y) && (z <= size.m_z) && (z >= -size.m_z)){
			points[count] = dVector (x, y, z);
			count ++;
		}
	} 

count = 4;

	// Create the array of convex pieces from the mesh
	int interior = LoadTexture("KAMEN-stup.tga");
//	int interior = LoadTexture("camo.tga");
	dMatrix textureMatrix (GetIdentityMatrix());
	textureMatrix[0][0] = 1.0f / size.m_x;
	textureMatrix[1][1] = 1.0f / size.m_y;
	NewtonMesh* const convexParts = NewtonMeshVoronoiDecomposition (mesh, count, sizeof (dVector), &points[0].m_x, interior, &textureMatrix[0][0]);
//	NewtonMesh* const convexParts = NewtonMeshConvexDecomposition (mesh, 1000000);

#if 1
dScene xxxx(world);
dScene::dTreeNode* const modelNode = xxxx.CreateSceneNode(xxxx.GetRootNode());
dScene::dTreeNode* const meshNode = xxxx.CreateMeshNode(modelNode);
dMeshNodeInfo* const modelMesh = (dMeshNodeInfo*)xxxx.GetInfoFromNode(meshNode);
modelMesh->ReplaceMesh (convexParts);
xxxx.Serialize("../../../media/xxx.ngd");
#endif

	DemoEntity* const entity = new DemoEntity(NULL);
	entity->SetMatrix(*scene, dQuaternion(), dVector (0, 10, 0, 0));
	entity->InterpolateMatrix (*scene, 1.0f);
	
	
	scene->Append (entity);
	DemoMesh* const mesh1 = new DemoMesh(convexParts);
	entity->SetMesh(mesh1);
	mesh1->Release();

/*
DemoEntity* const entity2 = new DemoEntity(NULL);
entity2->SetMatrix(*scene, dQuaternion(), dVector (0, 10, 0, 0));
entity2->InterpolateMatrix (*scene, 1.0f);

scene->Append (entity2);
DemoMesh* const mesh2 = new DemoMesh(mesh);
entity2->SetMesh(mesh2);
mesh2->Release();
*/

	// make sure the assets are released before leaving the function
	if (convexParts) {
		NewtonMeshDestroy (convexParts);
	}
	NewtonMeshDestroy (mesh);
	NewtonDestroyCollision (collision);
}
#endif


class ShatterAtom
{
	public:
	dVector m_centerOfMass;
	dVector m_momentOfInirtia;
	DemoMesh* m_mesh;
	NewtonCollision* m_collision;
	dFloat m_massFraction;
};

class ShatterEffect: public dList<ShatterAtom> 
{
	public:

	ShatterEffect(NewtonWorld* const world, NewtonMesh* const mesh, int interiorMaterial)
		:dList<ShatterAtom>(), m_world (world)
	{
		// first we populate the bounding Box area with few random point to get some interior subdivisions.
		// the subdivision are local to the point placement, by placing these points visual ally with a 3d tool
		// and have precise control of how the debris are created.
		// the number of pieces is equal to the number of point inside the Mesh plus the number of point on the mesh 
		dVector size;
		dMatrix matrix(GetIdentityMatrix()); 
		NewtonMeshCalculateOOBB(mesh, &matrix[0][0], &size.m_x, &size.m_y, &size.m_z);

		// pepper the inside of the BBox box of the mesh with random points
		int count = 0;
		dVector points[NUMBER_OF_INTERNAL_PARTS + 1];
		while (count < NUMBER_OF_INTERNAL_PARTS) {			
			dFloat x = RandomVariable(size.m_x);
			dFloat y = RandomVariable(size.m_y);
			dFloat z = RandomVariable(size.m_z);
			if ((x <= size.m_x) && (x >= -size.m_x) && (y <= size.m_y) && (y >= -size.m_y) && (z <= size.m_z) && (z >= -size.m_z)){
				points[count] = dVector (x, y, z);
				count ++;
			}
		} 

		// create a texture matrix, for applying the material's UV to all internal faces
		dMatrix textureMatrix (GetIdentityMatrix());
		textureMatrix[0][0] = 1.0f / size.m_x;
		textureMatrix[1][1] = 1.0f / size.m_y;

		// now we call create we decompose the mesh into several convex pieces 
		//NewtonMesh* const debriMeshPieces = NewtonMeshVoronoiDecomposition (mesh, count, sizeof (dVector), &points[0].m_x, interiorMaterial, &textureMatrix[0][0]);
		dFloat dist = 0.2f;
		NewtonMesh* const debriMeshPieces = NewtonMeshCreateVoronoiConvexDecomposition (m_world, count, &points[0].m_x, sizeof (dVector), interiorMaterial, &textureMatrix[0][0], dist);
		dAssert (debriMeshPieces);

		// TODO:  clip the voronoid convexes against the mesh 
		//


		// Get the volume of the original mesh
		NewtonCollision* const collision = NewtonCreateConvexHullFromMesh (m_world, mesh, 0.0f, 0);
		dFloat volume = NewtonConvexCollisionCalculateVolume (collision);
		NewtonDestroyCollision(collision);

		// now we iterate over each pieces and for each one we create a visual entity and a rigid body
		NewtonMesh* nextDebri;
		for (NewtonMesh* debri = NewtonMeshCreateFirstLayer (debriMeshPieces); debri; debri = nextDebri) {
			nextDebri = NewtonMeshCreateNextLayer (debriMeshPieces, debri); 

			NewtonCollision* const collision = NewtonCreateConvexHullFromMesh (m_world, debri, 0.0f, 0);
			if (collision) {
				ShatterAtom& atom = Append()->GetInfo();
				atom.m_mesh = new DemoMesh(debri);
				atom.m_collision = collision;
				NewtonConvexCollisionCalculateInertialMatrix (atom.m_collision, &atom.m_momentOfInirtia[0], &atom.m_centerOfMass[0]);	
				dFloat debriVolume = NewtonConvexCollisionCalculateVolume (atom.m_collision);
				atom.m_massFraction = debriVolume / volume;
			}
			NewtonMeshDestroy(debri);
		}

		NewtonMeshDestroy(debriMeshPieces);
	}

	ShatterEffect (const ShatterEffect& list)
		:dList<ShatterAtom>(), m_world(list.m_world)
	{
		for (dListNode* node = list.GetFirst(); node; node = node->GetNext()) {
			ShatterAtom& atom = Append(node->GetInfo())->GetInfo();
			atom.m_mesh->AddRef();
			atom.m_collision = NewtonCollisionCreateInstance (atom.m_collision);
		}
	}

	~ShatterEffect()
	{
		for (dListNode* node = GetFirst(); node; node = node->GetNext()) {
			ShatterAtom& atom = node->GetInfo();
			NewtonDestroyCollision (atom.m_collision);
			atom.m_mesh->Release();
		}
	}

	NewtonWorld* m_world;
};



class SimpleShatterEffectEntity: public DemoEntity
{
	public:
	SimpleShatterEffectEntity (DemoMesh* const mesh, const ShatterEffect& columnDebris)
		:DemoEntity (GetIdentityMatrix(), NULL), m_delay (INITIAL_DELAY), m_effect(columnDebris), m_myBody(NULL)
	{
		SetMesh(mesh);
	}

	~SimpleShatterEffectEntity ()
	{
	}


	void SimulationLister(DemoEntityManager* const scene, DemoEntityManager::dListNode* const mynode, dFloat timeStep)
	{
		dAssert (0);
/*
		m_delay --;
		if (m_delay > 0) {
			return;
		}

		// see if the net force on the body comes fr a high impact collision
		dFloat maxInternalForce = 0.0f;
		for (NewtonJoint* joint = NewtonBodyGetFirstContactJoint(m_myBody); joint; joint = NewtonBodyGetNextContactJoint(m_myBody, joint)) {
			for (void* contact = NewtonContactJointGetFirstContact (joint); contact; contact = NewtonContactJointGetNextContact (joint, contact)) {
				//dVector point;
				//dVector normal;	
				dVector contactForce;
				NewtonMaterial* const material = NewtonContactGetMaterial (contact);
				//NewtonMaterialGetContactPositionAndNormal (material, &point.m_x, &normal.m_x);
				NewtonMaterialGetContactForce(material, m_myBody, &contactForce[0]);
				dFloat forceMag = contactForce % contactForce;
				if (forceMag > maxInternalForce) {
					maxInternalForce = forceMag;
				}
			}
		}

		

		// if the force is bigger than 4 Gravities, It is considered a collision force
		dFloat maxForce = BREAK_FORCE_IN_GRAVITIES * m_myweight;

		if (maxInternalForce > (maxForce * maxForce)) {
			NewtonWorld* const world = NewtonBodyGetWorld(m_myBody);

			dFloat Ixx; 
			dFloat Iyy; 
			dFloat Izz; 
			dFloat mass; 
			NewtonBodyGetMassMatrix(m_myBody, &mass, &Ixx, &Iyy, &Izz);

			dVector com;
			dVector veloc;
			dVector omega;
			dMatrix bodyMatrix;

			NewtonBodyGetVelocity(m_myBody, &veloc[0]);
			NewtonBodyGetOmega(m_myBody, &omega[0]);
			NewtonBodyGetCentreOfMass(m_myBody, &com[0]);
			NewtonBodyGetMatrix(m_myBody, &bodyMatrix[0][0]);
			com = bodyMatrix.TransformVector (com);

			dMatrix matrix (GetCurrentMatrix());
			dQuaternion rotation (matrix);
			for (ShatterEffect::dListNode* node = m_effect.GetFirst(); node; node = node->GetNext()) {
				ShatterAtom& atom = node->GetInfo();

				DemoEntity* const entity = new DemoEntity (dMatrix (rotation, matrix.m_posit), NULL);
				entity->SetMesh (atom.m_mesh);
				//entity->SetMatrix(*scene, rotation, matrix.m_posit);
				//entity->InterpolateMatrix (*scene, 1.0f);
				scene->Append(entity);

				int materialId = 0;

				dFloat debriMass = mass * atom.m_massFraction;
				dFloat Ixx = debriMass * atom.m_momentOfInirtia.m_x;
				dFloat Iyy = debriMass * atom.m_momentOfInirtia.m_y;
				dFloat Izz = debriMass * atom.m_momentOfInirtia.m_z;

				//create the rigid body
				NewtonBody* const rigidBody = NewtonCreateBody (world, atom.m_collision, &matrix[0][0]);

				// set the correct center of gravity for this body
				NewtonBodySetCentreOfMass (rigidBody, &atom.m_centerOfMass[0]);

				// calculate the center of mas of the debris
				dVector center (matrix.TransformVector(atom.m_centerOfMass));

				// calculate debris initial velocity
				dVector v (veloc + omega * (center - com));

				// set initial velocity
				NewtonBodySetVelocity(rigidBody, &v[0]);
				NewtonBodySetOmega(rigidBody, &omega[0]);

				// set the  debrie center of mass
				NewtonBodySetCentreOfMass (rigidBody, &atom.m_centerOfMass[0]);


				// set the mass matrix
				NewtonBodySetMassMatrix (rigidBody, debriMass, Ixx, Iyy, Izz);

				// activate 
				//	NewtonBodyCoriolisForcesMode (blockBoxBody, 1);

				// save the pointer to the graphic object with the body.
				NewtonBodySetUserData (rigidBody, entity);

				// assign the wood id
				NewtonBodySetMaterialGroupID (rigidBody, materialId);

				//  set continue collision mode
				//	NewtonBodySetContinuousCollisionMode (rigidBody, continueCollisionMode);

				// set a destructor for this rigid body
				NewtonBodySetDestructorCallback (rigidBody, PhysicsBodyDestructor);

				// set the transform call back function
				NewtonBodySetTransformCallback (rigidBody, DemoEntity::TransformCallback);

				// set the force and torque call back function
				NewtonBodySetForceAndTorqueCallback (rigidBody, PhysicsApplyGravityForce);
			}

			NewtonDestroyBody(world, m_myBody);
			scene->RemoveEntity	(mynode);
		}
*/
	};

	int m_delay;
	ShatterEffect m_effect;
	NewtonBody* m_myBody;
	dFloat m_myweight; 
};


static void AddShatterEntity (DemoEntityManager* const scene, DemoMesh* const visualMesh, NewtonCollision* const collision, const ShatterEffect& shatterEffect, const dVector& location)
{
	dAssert (0);

/*
	dQuaternion rotation;
	SimpleShatterEffectEntity* const entity = new SimpleShatterEffectEntity (visualMesh, shatterEffect);
	entity->SetMatrix(*scene, rotation, location);
	entity->InterpolateMatrix (*scene, 1.0f);
	scene->Append(entity);

	dVector origin;
	dVector inertia;
	NewtonConvexCollisionCalculateInertialMatrix (collision, &inertia[0], &origin[0]);	

float mass = 10.0f;
int materialId = 0;

	dFloat Ixx = mass * inertia[0];
	dFloat Iyy = mass * inertia[1];
	dFloat Izz = mass * inertia[2];

	//create the rigid body
	dMatrix matrix (GetIdentityMatrix());
	matrix.m_posit = location;

	NewtonWorld* const world = scene->GetNewton();
	NewtonBody* const rigidBody = NewtonCreateBody (world, collision, &matrix[0][0]);


	entity->m_myBody = rigidBody;
	entity->m_myweight = dAbs (mass * DEMO_GRAVITY);

	// set the correct center of gravity for this body
	NewtonBodySetCentreOfMass (rigidBody, &origin[0]);

	// set the mass matrix
	NewtonBodySetMassMatrix (rigidBody, mass, Ixx, Iyy, Izz);

	// activate 
	//	NewtonBodyCoriolisForcesMode (blockBoxBody, 1);

	// save the pointer to the graphic object with the body.
	NewtonBodySetUserData (rigidBody, entity);

	// assign the wood id
	NewtonBodySetMaterialGroupID (rigidBody, materialId);

	//  set continue collision mode
	//	NewtonBodySetContinuousCollisionMode (rigidBody, continueCollisionMode);

	// set a destructor for this rigid body
	NewtonBodySetDestructorCallback (rigidBody, PhysicsBodyDestructor);

	// set the transform call back function
	NewtonBodySetTransformCallback (rigidBody, DemoEntity::TransformCallback);

	// set the force and torque call back function
	NewtonBodySetForceAndTorqueCallback (rigidBody, PhysicsApplyGravityForce);
*/
}



static void AddShatterPrimitive (DemoEntityManager* const scene, dFloat mass, const dVector& origin, const dVector& size, int xCount, int zCount, dFloat spacing, PrimitiveType type, int materialID, const dMatrix& shapeOffsetMatrix)
{

	// create the shape and visual mesh as a common data to be re used
	NewtonWorld* const world = scene->GetNewton();
	NewtonCollision* const collision = CreateConvexCollision (world, shapeOffsetMatrix, size, type, materialID);


	// create a newton mesh from the collision primitive
	NewtonMesh* const mesh = NewtonMeshCreateFromCollision(collision);

	// apply a material map
	int externalMaterial = LoadTexture("reljef.tga");
	int internalMaterial = LoadTexture("KAMEN-stup.tga");
	NewtonMeshApplyBoxMapping(mesh, externalMaterial, externalMaterial, externalMaterial);

	// create a newton mesh from the collision primitive
	ShatterEffect shatter (world, mesh, internalMaterial);

	DemoMesh* const visualMesh = new DemoMesh(mesh);

	dFloat startElevation = 100.0f;
	dMatrix matrix (GetIdentityMatrix());
	for (int i = 0; i < xCount; i ++) {
		dFloat x = origin.m_x + (i - xCount / 2) * spacing;
		for (int j = 0; j < zCount; j ++) {
			dFloat z = origin.m_z + (j - zCount / 2) * spacing;

			matrix.m_posit.m_x = x;
			matrix.m_posit.m_z = z;
			dVector floor (FindFloor (world, dVector (matrix.m_posit.m_x, startElevation, matrix.m_posit.m_z, 0.0f), 2.0f * startElevation));
			matrix.m_posit.m_y = floor.m_y + 1.0f;
			AddShatterEntity (scene, visualMesh, collision, shatter, matrix.m_posit);
		}
	}


	// do not forget to release the assets	
	NewtonMeshDestroy (mesh);
	visualMesh->Release(); 
	NewtonDestroyCollision (collision);

}




void SimpleConvexShatter (DemoEntityManager* const scene)
{
	// load the skybox
	scene->CreateSkyBox();

	// load the scene from a ngd file format
	CreateLevelMesh (scene, "flatPlane.ngd", false);
//	CreateLevelMesh (scene, "sponza.ngd", false);
//	CreateLevelMesh (scene, "sponza.ngd", true);

	// create a shattered mesh array
//CreateSimpleVoronoiShatter (scene);

	int defaultMaterialID = NewtonMaterialGetDefaultGroupID (scene->GetNewton());
	dVector location (0.0f, 0.0f, 0.0f, 0.0f);
	dVector size (0.5f, 0.5f, 0.5f, 0.0f);
	dMatrix shapeOffsetMatrix (GetIdentityMatrix());

	int count = 5;


//	AddShatterPrimitive(scene, 10.0f, location, size, count, count, 3.0f, _BOX_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
//	AddShatterPrimitive(scene, 10.0f, location, size, count, count, 3.0f, _REGULAR_CONVEX_HULL_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
//	AddShatterPrimitive(scene, 10.0f, location, size, count, count, 3.0f, _RANDOM_CONVEX_HULL_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
	AddShatterPrimitive(scene, 10.0f, location, size, count, count, 3.0f, _SPHERE_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
//	AddShatterPrimitive(scene, 10.0f, location, size, count, count, 3.0f, _CYLINDER_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
//	AddShatterPrimitive(scene, 10.0f, location, size, count, count, 3.0f, _CONE_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
//	AddShatterPrimitive(scene, 10.0f, location, size, count, count, 3.0f, _CAPSULE_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);

//for (int i = 0; i < 1; i ++)
//AddShatterPrimitive(scene, 10.0f, location, size, count, count, 1.7f, _REGULAR_CONVEX_HULL_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);

	// place camera into position
	dQuaternion rot;
//	dVector origin (-40.0f, 10.0f, 0.0f, 0.0f);
	dVector origin (-15.0f, 5.0f, 0.0f, 0.0f);
	scene->SetCameraMatrix(rot, origin);

}


