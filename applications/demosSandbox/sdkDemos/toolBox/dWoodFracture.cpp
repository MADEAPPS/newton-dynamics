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

// dGeometry.cpp: implementation of the dGeometry class.
//
//////////////////////////////////////////////////////////////////////
#include "toolbox_stdafx.h"
#include "dWoodFracture.h"

WoodFractureAtom::WoodFractureAtom()
	:m_centerOfMass(0.0f)
	, m_momentOfInirtia(0.0f)
{
}

//////////////////////////////

WoodFractureEffect::WoodFractureEffect(NewtonWorld* const world)
	:dList<WoodFractureAtom>()
	, m_world(world)
{
}

WoodFractureEffect::WoodFractureEffect(const WoodFractureEffect& list)
	:dList<WoodFractureAtom>()
	, m_world(list.m_world)
{
	for (dListNode* node = list.GetFirst(); node; node = node->GetNext()) {
		WoodFractureAtom& atom = Append(node->GetInfo())->GetInfo();
		atom.m_mesh->AddRef();
		atom.m_collision = NewtonCollisionCreateInstance(atom.m_collision);
		NewtonCollisionSetUserID(atom.m_collision, 16);
	}
}

WoodFractureEffect::~WoodFractureEffect()
{
	for (dListNode* node = GetFirst(); node; node = node->GetNext()) {
		WoodFractureAtom& atom = node->GetInfo();
		NewtonDestroyCollision(atom.m_collision);
		atom.m_mesh->Release();
	}
}

//////////////////////////

SimpleWoodFracturedEffectEntity::SimpleWoodFracturedEffectEntity(DemoMesh* const mesh, const WoodFractureEffect& columnDebris)
	:DemoEntity(dGetIdentityMatrix(), NULL), m_delay(INITIAL_DELAY), m_effect(columnDebris), m_myBody(NULL)
{
	SetMesh(mesh, dGetIdentityMatrix());
}

SimpleWoodFracturedEffectEntity::~SimpleWoodFracturedEffectEntity()
{
}

void SimpleWoodFracturedEffectEntity::SimulationPostListener(DemoEntityManager* const scene, DemoEntityManager::dListNode* const mynode, dFloat timeStep)
{
	// see if the net force on the body comes fr a high impact collision
	dFloat breakImpact = 0.0f;
	for (NewtonJoint* joint = NewtonBodyGetFirstContactJoint(m_myBody); joint; joint = NewtonBodyGetNextContactJoint(m_myBody, joint)) {
		for (void* contact = NewtonContactJointGetFirstContact(joint); contact; contact = NewtonContactJointGetNextContact(joint, contact)) {
			dVector contactForce;
			NewtonMaterial* const material = NewtonContactGetMaterial(contact);
			dFloat impulseImpact = NewtonMaterialGetContactMaxNormalImpact(material);
			if (impulseImpact > breakImpact) {
				breakImpact = impulseImpact;
			}
		}
	}


	// if the force is bigger than N time Gravities, It is considered a collision force
	breakImpact *= m_myMassInverse;
	//		breakImpact = 1000.0f;
	if (breakImpact > BREAK_IMPACT_IN_METERS_PER_SECONDS) {
		NewtonWorld* const world = NewtonBodyGetWorld(m_myBody);

		dMatrix bodyMatrix;
		dVector com(0.0f);
		dVector veloc(0.0f);
		dVector omega(0.0f);
		dFloat Ixx;
		dFloat Iyy;
		dFloat Izz;
		dFloat mass;

		NewtonBodyGetVelocity(m_myBody, &veloc[0]);
		NewtonBodyGetOmega(m_myBody, &omega[0]);
		NewtonBodyGetCentreOfMass(m_myBody, &com[0]);
		NewtonBodyGetMatrix(m_myBody, &bodyMatrix[0][0]);
		NewtonBodyGetMass(m_myBody, &mass, &Ixx, &Iyy, &Izz);

		com = bodyMatrix.TransformVector(com);
		dMatrix matrix(GetCurrentMatrix());
		dQuaternion rotation(matrix);

		// we need to lock the world before creation a bunch of bodies
		scene->Lock(m_lock);

		for (WoodFractureEffect::dListNode* node = m_effect.GetFirst(); node; node = node->GetNext()) {
			WoodFractureAtom& atom = node->GetInfo();

			DemoEntity* const entity = new DemoEntity(dMatrix(rotation, matrix.m_posit), NULL);
			entity->SetMesh(atom.m_mesh, dGetIdentityMatrix());
			scene->Append(entity);

			int defaultMaterialID = NewtonMaterialGetDefaultGroupID(scene->GetNewton());
			//int materialId = 0;

			dFloat debriMass = mass * atom.m_massFraction;

			//create the rigid body
			NewtonBody* const rigidBody = NewtonCreateDynamicBody(world, atom.m_collision, &matrix[0][0]);

			// calculate debris initial velocity
			dVector center(matrix.TransformVector(atom.m_centerOfMass));
			dVector v(veloc + omega.CrossProduct(center - com));

			// set initial velocity
			NewtonBodySetVelocity(rigidBody, &v[0]);
			NewtonBodySetOmega(rigidBody, &omega[0]);

			// set the debris mass properties, mass, center of mass, and inertia 
			NewtonBodySetMassProperties(rigidBody, debriMass, atom.m_collision);

			// save the pointer to the graphic object with the body.
			NewtonBodySetUserData(rigidBody, entity);

			// assign the wood id
			NewtonBodySetMaterialGroupID(rigidBody, defaultMaterialID);

			//  set continuous collision mode
			//	NewtonBodySetContinuousCollisionMode (rigidBody, continueCollisionMode);

			// set a destructor for this rigid body
			NewtonBodySetDestructorCallback(rigidBody, PhysicsBodyDestructor);

			// set the transform call back function
			NewtonBodySetTransformCallback(rigidBody, DemoEntity::TransformCallback);

			// set the force and torque call back function
			NewtonBodySetForceAndTorqueCallback(rigidBody, PhysicsApplyGravityForce);
			//
			NewtonCollision* const collision = NewtonBodyGetCollision(rigidBody);
			if (NewtonCollisionGetUserID(collision) == 16) {
				NewtonBodySetGyroscopicTorque(rigidBody, 1);
				NewtonCollisionMaterial material;
				NewtonCollisionGetMaterial(collision, &material);
				material.m_userParam[0].m_int = -1;
				NewtonCollisionSetMaterial(collision, &material);
			}
		}

		NewtonDestroyBody(m_myBody);
		scene->RemoveEntity(mynode);

		// unlock the work after done with the effect 
		scene->Unlock(m_lock);
	}
}

void SimpleWoodFracturedEffectEntity::AddFracturedWoodEntity(DemoEntityManager* const scene, DemoMesh* const visualMesh, NewtonCollision* const collision, const WoodFractureEffect& fractureEffect, const dVector& location)
{
	dQuaternion rotation;
	SimpleWoodFracturedEffectEntity* const entity = new SimpleWoodFracturedEffectEntity(visualMesh, fractureEffect); 
	entity->SetMatrix(*scene, rotation, location);
	entity->InterpolateMatrix(*scene, 1.0f);
	scene->Append(entity);

	dVector origin(0.0f);
	dVector inertia(0.0f);
	NewtonConvexCollisionCalculateInertialMatrix(collision, &inertia[0], &origin[0]);

	dFloat mass = 10.0f;
	//int materialId = 0;
	int defaultMaterialID = NewtonMaterialGetDefaultGroupID(scene->GetNewton());

	//create the rigid body
	dMatrix matrix(dGetIdentityMatrix());
	matrix.m_posit = location;

	NewtonWorld* const world = scene->GetNewton();
	NewtonBody* const rigidBody = NewtonCreateDynamicBody(world, collision, &matrix[0][0]);

	entity->m_myBody = rigidBody;
	entity->m_myMassInverse = 1.0f / mass;

	// set the correct center of gravity for this body
	//NewtonBodySetCentreOfMass (rigidBody, &origin[0]);

	// set the mass matrix
	NewtonBodySetMassProperties(rigidBody, mass, collision);

	// save the pointer to the graphic object with the body.
	NewtonBodySetUserData(rigidBody, entity);

	// assign the wood id
	NewtonBodySetMaterialGroupID(rigidBody, defaultMaterialID);

	//  set continuous collision mode
	//	NewtonBodySetContinuousCollisionMode (rigidBody, continueCollisionMode);

	// set a destructor for this rigid body
	NewtonBodySetDestructorCallback(rigidBody, PhysicsBodyDestructor);

	// set the transform call back function
	NewtonBodySetTransformCallback(rigidBody, DemoEntity::TransformCallback);

	// set the force and torque call back function
	NewtonBodySetForceAndTorqueCallback(rigidBody, PhysicsApplyGravityForce);
}

////////////////////////////////////////

WoodDelaunayEffect::WoodDelaunayEffect(NewtonWorld* const world, NewtonMesh* const mesh, int interiorMaterial)
	:WoodFractureEffect(world)
{
	// first we populate the bounding Box area with few random point to get some interior subdivisions.
	// the subdivision are local to the point placement, by placing these points visual ally with a 3d tool
	// and have precise control of how the debris are created.
	// the number of pieces is equal to the number of point inside the Mesh plus the number of point on the mesh 
	dVector size(0.0f);
	dMatrix matrix(dGetIdentityMatrix());
	NewtonMeshCalculateOOBB(mesh, &matrix[0][0], &size.m_x, &size.m_y, &size.m_z);

	// create a texture matrix, for applying the material's UV to all internal faces
	dMatrix textureMatrix(dGetIdentityMatrix());
	textureMatrix[0][0] = 1.0f / size.m_x;
	textureMatrix[1][1] = 1.0f / size.m_y;

	// Get the volume of the original mesh
	NewtonCollision* const collision1 = NewtonCreateConvexHullFromMesh(m_world, mesh, 0.0f, 16);
	dFloat volume = NewtonConvexCollisionCalculateVolume(collision1);
	NewtonDestroyCollision(collision1);

	// now we call create we decompose the mesh into several convex pieces 
	NewtonMesh* const debriMeshPieces = NewtonMeshCreateTetrahedraIsoSurface(mesh);
	dAssert(debriMeshPieces);

	DemoEntityManager* const scene = (DemoEntityManager*)NewtonWorldGetUserData(world);

	// now we iterate over each pieces and for each one we create a visual entity and a rigid body
	NewtonMesh* nextDebri;
	for (NewtonMesh* debri = NewtonMeshCreateFirstLayer(debriMeshPieces); debri; debri = nextDebri) {
		// get next segment piece
		nextDebri = NewtonMeshCreateNextLayer(debriMeshPieces, debri);

		//clip the Delaunay convexes against the mesh, make a convex hull collision shape
		NewtonCollision* const collision = NewtonCreateConvexHullFromMesh(m_world, debri, 0.0f, 16);
		if (collision) {
			// we have a piece which has a convex collision  representation, add that to the list
			WoodFractureAtom& atom = Append()->GetInfo();
			atom.m_mesh = new DemoMesh(debri, scene->GetShaderCache());
			atom.m_collision = collision;
			NewtonConvexCollisionCalculateInertialMatrix(atom.m_collision, &atom.m_momentOfInirtia[0], &atom.m_centerOfMass[0]);
			dFloat debriVolume = NewtonConvexCollisionCalculateVolume(atom.m_collision);
			atom.m_massFraction = debriVolume / volume;
		}
		NewtonMeshDestroy(debri);
	}

	NewtonMeshDestroy(debriMeshPieces);
}

void WoodDelaunayEffect::AddFracturedWoodPrimitive(DemoEntityManager* const scene, dFloat mass, const dVector& origin, const dVector& size, int xCount, int zCount, dFloat spacing, PrimitiveType type, int materialID, const dMatrix& shapeOffsetMatrix)
{
	// create the shape and visual mesh as a common data to be re used
	NewtonWorld* const world = scene->GetNewton();
	NewtonCollision* const collision = CreateConvexCollision(world, shapeOffsetMatrix, size, type, materialID);

	// create a newton mesh from the collision primitive
	NewtonMesh* const mesh = NewtonMeshCreateFromCollision(collision);

	// apply a material map
	int externalMaterial = LoadTexture("reljef.tga");
	int internalMaterial = LoadTexture("concreteBrick.tga");
	dMatrix aligmentUV(dGetIdentityMatrix());

	NewtonMeshApplyBoxMapping(mesh, externalMaterial, externalMaterial, externalMaterial, &aligmentUV[0][0]);

	// create a newton mesh from the collision primitive
	WoodDelaunayEffect fracture(world, mesh, internalMaterial);

	DemoMesh* const visualMesh = new DemoMesh(mesh, scene->GetShaderCache());

	dFloat startElevation = 100.0f;
	dMatrix matrix(dGetIdentityMatrix());
	for (int i = 0; i < xCount; i++) {
		dFloat x = origin.m_x + (i - xCount / 2) * spacing;
		for (int j = 0; j < zCount; j++) {
			dFloat z = origin.m_z + (j - zCount / 2) * spacing;

			matrix.m_posit.m_x = x;
			matrix.m_posit.m_z = z;
			dVector floor(FindFloor(world, dVector(matrix.m_posit.m_x, startElevation, matrix.m_posit.m_z, 0.0f), 2.0f * startElevation));
			matrix.m_posit.m_y = floor.m_y + 1.0f;
			SimpleWoodFracturedEffectEntity::AddFracturedWoodEntity(scene, visualMesh, collision, fracture, matrix.m_posit);
		}
	}

	// do not forget to release the assets	
	NewtonMeshDestroy(mesh);
	visualMesh->Release();
	NewtonDestroyCollision(collision);
}

/////////////////////////////////////////////////////////

WoodVoronoidEffect::WoodVoronoidEffect(NewtonWorld* const world, NewtonMesh* const mesh, int interiorMaterial)
	:WoodFractureEffect(world)
{
	// first we populate the bounding Box area with few random point to get some interior subdivisions.
	// the subdivision are local to the point placement, by placing these points visual ally with a 3d tool
	// and have precise control of how the debris are created.
	// the number of pieces is equal to the number of point inside the Mesh plus the number of point on the mesh 
	dVector size(0.0f);
	dMatrix matrix(dGetIdentityMatrix());
	NewtonMeshCalculateOOBB(mesh, &matrix[0][0], &size.m_x, &size.m_y, &size.m_z);

	dVector points[NUMBER_OF_INTERNAL_PARTS + 8];

	int count = 0;
	// pepper the inside of the BBox box of the mesh with random points
	while (count < NUMBER_OF_INTERNAL_PARTS) {
		dFloat x = 0.0f;//dGaussianRandom(size.m_x);
		dFloat y = 0.0f;//dGaussianRandom(size.m_y);
		dFloat z = dGaussianRandom(size.m_z);
		if ((x <= size.m_x) && (x >= -size.m_x) && (y <= size.m_y) && (y >= -size.m_y) && (z <= size.m_z) && (z >= -size.m_z)) {
			points[count] = dVector(x, y, z);
			count++;
		}
	}

	// add the bounding box as a safeguard area
	points[count + 0] = dVector(size.m_x, size.m_y, size.m_z, 0.0f);
	points[count + 1] = dVector(size.m_x, size.m_y, -size.m_z, 0.0f);
	points[count + 2] = dVector(size.m_x, -size.m_y, size.m_z, 0.0f);
	points[count + 3] = dVector(size.m_x, -size.m_y, -size.m_z, 0.0f);
	points[count + 4] = dVector(-size.m_x, size.m_y, size.m_z, 0.0f);
	points[count + 5] = dVector(-size.m_x, size.m_y, -size.m_z, 0.0f);
	points[count + 6] = dVector(-size.m_x, -size.m_y, size.m_z, 0.0f);
	points[count + 7] = dVector(-size.m_x, -size.m_y, -size.m_z, 0.0f);
	count += 8;


	// create a texture matrix, for applying the material's UV to all internal faces
	dMatrix textureMatrix(dGetIdentityMatrix());
	textureMatrix[0][0] = 1.0f / size.m_x;
	textureMatrix[1][1] = 1.0f / size.m_y;

	// Get the volume of the original mesh
	NewtonCollision* const collision1 = NewtonCreateConvexHullFromMesh(m_world, mesh, 0.0f, 16);
	dFloat volume = NewtonConvexCollisionCalculateVolume(collision1);
	NewtonDestroyCollision(collision1);

	// now we call create we decompose the mesh into several convex pieces 
	NewtonMesh* const debriMeshPieces = NewtonMeshCreateVoronoiConvexDecomposition(m_world, count, &points[0].m_x, sizeof(dVector), interiorMaterial, &textureMatrix[0][0]);
	dAssert(debriMeshPieces);

	DemoEntityManager* const scene = (DemoEntityManager*)NewtonWorldGetUserData(world);

	// now we iterate over each pieces and for each one we create a visual entity and a rigid body
	NewtonMesh* nextDebri;
	for (NewtonMesh* debri = NewtonMeshCreateFirstLayer(debriMeshPieces); debri; debri = nextDebri) {
		// get next segment piece
		nextDebri = NewtonMeshCreateNextLayer(debriMeshPieces, debri);

		//clip the voronoi convexes against the mesh 
		NewtonMesh* const fracturePiece = NewtonMeshConvexMeshIntersection(mesh, debri);
		if (fracturePiece) {
			// make a convex hull collision shape
			NewtonCollision* const collision = NewtonCreateConvexHullFromMesh(m_world, fracturePiece, 0.0f, 16);
			if (collision) {
				// we have a piece which has a convex collision  representation, add that to the list
				WoodFractureAtom& atom = Append()->GetInfo();
				atom.m_mesh = new DemoMesh(fracturePiece, scene->GetShaderCache());
				atom.m_collision = collision;
				NewtonConvexCollisionCalculateInertialMatrix(atom.m_collision, &atom.m_momentOfInirtia[0], &atom.m_centerOfMass[0]);
				dFloat debriVolume = NewtonConvexCollisionCalculateVolume(atom.m_collision);
				atom.m_massFraction = debriVolume / volume;
			}
			NewtonMeshDestroy(fracturePiece);
		}

		NewtonMeshDestroy(debri);
	}

	NewtonMeshDestroy(debriMeshPieces);
}

void WoodVoronoidEffect::AddFracturedWoodPrimitive(DemoEntityManager* const scene, dFloat mass, const dVector& origin, const dVector& size, int xCount, int zCount, dFloat spacing, int stype, int materialID, const dMatrix& shapeOffsetMatrix)
{
	// create the shape and visual mesh as a common data to be re used
	NewtonWorld* const world = scene->GetNewton();

	NewtonCollision* const collision = NewtonCreateCylinder(world, size.m_x * 0.5f, size.m_z * 0.5f, size.m_y, stype, NULL);

	//NewtonCollisionSetUserID(collision, 16);

	// create a newton mesh from the collision primitive
	NewtonMesh* const mesh = NewtonMeshCreateFromCollision(collision);

	// apply a material map
	int externalMaterial = LoadTexture("reljef.tga");
	//int internalMaterial = LoadTexture("KAMEN-stup.tga");
	int internalMaterial = LoadTexture("concreteBrick.tga");

	dMatrix aligmentUV(dGetIdentityMatrix());
	NewtonMeshApplyBoxMapping(mesh, externalMaterial, externalMaterial, externalMaterial, &aligmentUV[0][0]);

	// create a newton mesh from the collision primitive
	WoodVoronoidEffect fracture(world, mesh, internalMaterial);

	DemoMesh* const visualMesh = new DemoMesh(mesh, scene->GetShaderCache());

	dFloat startElevation = 100.0f;
	dMatrix matrix(dGetIdentityMatrix());
	for (int i = 0; i < xCount; i++) {
		dFloat x = origin.m_x + (i - xCount / 2) * spacing;
		for (int j = 0; j < zCount; j++) {
			dFloat z = origin.m_z + (j - zCount / 2) * spacing;

			matrix.m_posit.m_x = x;
			matrix.m_posit.m_z = z;
			dVector floor(FindFloor(world, dVector(matrix.m_posit.m_x, startElevation, matrix.m_posit.m_z, 0.0f), 2.0f * startElevation));
			matrix.m_posit.m_y = floor.m_y + 1.0f;
			SimpleWoodFracturedEffectEntity::AddFracturedWoodEntity(scene, visualMesh, collision, fracture, matrix.m_posit);
		}
	}

	// do not forget to release the assets	
	NewtonMeshDestroy(mesh);
	visualMesh->Release();
	NewtonDestroyCollision(collision);
}