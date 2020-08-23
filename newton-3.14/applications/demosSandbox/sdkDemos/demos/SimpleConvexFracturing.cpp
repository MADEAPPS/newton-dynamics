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
#include "SkyBox.h"
#include "TargaToOpenGl.h"
#include "DemoMesh.h"
#include "DemoEntityManager.h"
#include "DemoCamera.h"
#include "PhysicsUtils.h"

#define NUMBER_OF_INTERNAL_PARTS				10
#define D_SIMPLE_FRACTIRE_BREAK_IMPACT_SPEED    10.0f
#define D_SIMPLE_FRACTURE_LISTENER				"SimpleFractureListener"

class dSimpleFractureListener: public dCustomParallelListener
{
	class dFractureAtom
	{
		public:
		dFractureAtom()
			:m_centerOfMass(0.0f)
			,m_momentOfInertia(0.0f)
			,m_mesh(NULL)
			,m_collision(NULL)
			,m_massFraction(0.0f)
		{
		}

		dVector m_centerOfMass;
		dVector m_momentOfInertia;
		DemoMesh* m_mesh;
		NewtonCollision* m_collision;
		dFloat m_massFraction;
	};

	class dSimpleVoronoidEffect: public dList<dFractureAtom>
	{
		public:
		dSimpleVoronoidEffect(NewtonWorld* const world, NewtonMesh* const mesh, int interiorMaterial)
			:m_body(NULL)
			,m_isDead(false)
		{
			DemoEntityManager* const scene = (DemoEntityManager*)NewtonWorldGetUserData(world);

			// first we populate the bounding Box area with few random point to get some interior subdivisions.
			// the subdivision are local to the point placement, by placing these points visual ally with a 3d tool
			// and have precise control of how the debris are created.
			// the number of pieces is equal to the number of point inside the Mesh plus the number of point on the mesh 
			dVector size(0.0f);
			dMatrix matrix(dGetIdentityMatrix());
			NewtonMeshCalculateOOBB(mesh, &matrix[0][0], &size.m_x, &size.m_y, &size.m_z);

			dVector points[32];
			points[0] = dVector(-size.m_x * 0.5f, -size.m_y * 0.5f, -size.m_z * 0.5f);
			points[1] = dVector(-size.m_x * 0.5f, -size.m_y * 0.5f, size.m_z * 0.5f);
			points[2] = dVector(-size.m_x * 0.5f, size.m_y * 0.5f, -size.m_z * 0.5f);
			points[3] = dVector(-size.m_x * 0.5f, size.m_y * 0.5f, size.m_z * 0.5f);

			points[4] = dVector(size.m_x * 0.5f, -size.m_y * 0.5f, -size.m_z * 0.5f);
			points[5] = dVector(size.m_x * 0.5f, -size.m_y * 0.5f, size.m_z * 0.5f);
			points[6] = dVector(size.m_x * 0.5f, size.m_y * 0.5f, -size.m_z * 0.5f);
			points[7] = dVector(size.m_x * 0.5f, size.m_y * 0.5f, size.m_z * 0.5f);

			int count = 0;
			while (count < NUMBER_OF_INTERNAL_PARTS) {
				dFloat x = dGaussianRandom(size.m_x);
				dFloat y = dGaussianRandom(size.m_y);
				dFloat z = dGaussianRandom(size.m_z);
				if ((x <= size.m_x) && (x >= -size.m_x) && (y <= size.m_y) && (y >= -size.m_y) && (z <= size.m_z) && (z >= -size.m_z)) {
					points[count] = dVector(x, y, z);
					count++;
				}
			}

			size = size.Scale(1.1f);
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
			NewtonCollision* const collision1 = NewtonCreateConvexHullFromMesh(world, mesh, 0.0f, 16);
			dFloat volume = NewtonConvexCollisionCalculateVolume(collision1);
			NewtonDestroyCollision(collision1);

			// now we call create we decompose the mesh into several convex pieces 
			NewtonMesh* const debriMeshPieces = NewtonMeshCreateVoronoiConvexDecomposition(world, count, &points[0].m_x, sizeof(dVector), interiorMaterial, &textureMatrix[0][0]);
			dAssert(debriMeshPieces);

			// now we iterate over each pieces and for each one we create a visual entity and a rigid body
			NewtonMesh* nextDebri;
			for (NewtonMesh* debri = NewtonMeshCreateFirstLayer(debriMeshPieces); debri; debri = nextDebri) {
				// get next segment piece
				nextDebri = NewtonMeshCreateNextLayer(debriMeshPieces, debri);

				//clip the voronoi cell convexes against the mesh 
				NewtonMesh* const fracturePiece = NewtonMeshConvexMeshIntersection(mesh, debri);
				if (fracturePiece) {
					// make a convex hull collision shape
					NewtonCollision* const collision = NewtonCreateConvexHullFromMesh(world, fracturePiece, 0.0f, 16);
					if (collision) {
						// we have a piece which has a convex collision  representation, add that to the list
						dFractureAtom& atom = Append()->GetInfo();
						atom.m_mesh = new DemoMesh(fracturePiece, scene->GetShaderCache());
						NewtonConvexCollisionCalculateInertialMatrix(collision, &atom.m_momentOfInertia[0], &atom.m_centerOfMass[0]);
						dFloat debriVolume = NewtonConvexCollisionCalculateVolume(collision);
						atom.m_massFraction = debriVolume / volume;
						atom.m_collision = collision;
					}
					NewtonMeshDestroy(fracturePiece);
				}
				NewtonMeshDestroy(debri);
			}
			NewtonMeshDestroy(debriMeshPieces);
		}

		dSimpleVoronoidEffect(const dSimpleVoronoidEffect& list)
			:m_body(NULL)
			,m_isDead(false)
		{
			for (dListNode* node = list.GetFirst(); node; node = node->GetNext()) {
				dFractureAtom& atom = Append(node->GetInfo())->GetInfo();
				atom.m_mesh->AddRef();
				atom.m_collision = NewtonCollisionCreateInstance(atom.m_collision);
				NewtonCollisionSetUserID(atom.m_collision, 16);
			}
		}

		~dSimpleVoronoidEffect()
		{
			for (dListNode* node = GetFirst(); node; node = node->GetNext()) {
				dFractureAtom& atom = node->GetInfo();
				NewtonDestroyCollision(atom.m_collision);
				atom.m_mesh->Release();
			}
		}

		NewtonBody* m_body;
		bool m_isDead;
	};

	public:
	dSimpleFractureListener(NewtonWorld* const world)
		:dCustomParallelListener(world, D_SIMPLE_FRACTURE_LISTENER)
		,m_effectList()
		,m_lock(0)
	{
	}

	void PostUpdate(dFloat timestep, int threadID)
	{
		NewtonWorld* const world = GetWorld();
		const int threadCount = NewtonGetThreadsCount(world);

		dList<dSimpleVoronoidEffect>::dListNode* node = m_effectList.GetFirst();
		for (int i = 0; i < threadID; i++) {
			node = node ? node->GetNext() : NULL;
		}

		if (node) {
			do {
				dSimpleVoronoidEffect& effect = node->GetInfo();
				UpdateEffect(effect, timestep);
				for (int i = 0; i < threadCount; i++) {
					node = node ? node->GetNext() : NULL;
				}
			} while (node);
		}
	}

	void PostUpdate(dFloat timestep)
	{
		dCustomParallelListener::PostUpdate(timestep);

		NewtonWorld* const world = GetWorld();
		DemoEntityManager* const scene = (DemoEntityManager*)NewtonWorldGetUserData(world);

		dList<dSimpleVoronoidEffect>::dListNode* nextNode;
		for (dList<dSimpleVoronoidEffect>::dListNode* node = m_effectList.GetFirst(); node; node = nextNode) {
			nextNode = node->GetNext();
			dSimpleVoronoidEffect& effect = node->GetInfo();
			if (effect.m_isDead) {
				DemoEntity* const visualEntiry = (DemoEntity*)NewtonBodyGetUserData(effect.m_body);
				NewtonDestroyBody(effect.m_body);
				scene->RemoveEntity(visualEntiry);
				m_effectList.Remove(node);
			}
		}
	}

	void UpdateEffect(dSimpleVoronoidEffect& effect, dFloat timestep)
	{
		// see if the net force on the body comes fr a high impact collision
		dFloat breakImpact = 0.0f;
		for (NewtonJoint* joint = NewtonBodyGetFirstContactJoint(effect.m_body); joint; joint = NewtonBodyGetNextContactJoint(effect.m_body, joint)) {
			for (void* contact = NewtonContactJointGetFirstContact(joint); contact; contact = NewtonContactJointGetNextContact(joint, contact)) {
				dVector contactForce;
				const NewtonCollision* const coll0 = NewtonContactGetCollision0(contact);
				const NewtonCollision* const coll1 = NewtonContactGetCollision1(contact);
				const NewtonMaterial* const material = NewtonContactGetMaterial(contact);
				dFloat impulseImpact = NewtonMaterialGetContactMaxNormalImpact(material);

				if (NewtonCollisionGetUserID(coll0) == 4) {
					// m_tirePart
					if (impulseImpact > breakImpact) {
						breakImpact = impulseImpact * 20.0f;
					}

				}
				else if (NewtonCollisionGetUserID(coll1) == 8) {
					// m_linkPart
					if (impulseImpact > breakImpact) {
						breakImpact = impulseImpact * 20.0f;
					}
				}
				else if (impulseImpact > breakImpact) {
					// all othe types
					breakImpact = impulseImpact;
				}
			}
		}

		dFloat invMass;
		dFloat invIxx;
		dFloat invIyy;
		dFloat invIzz;
		NewtonBodyGetInvMass(effect.m_body, &invMass, &invIxx, &invIyy, &invIzz);

		// if the force is bigger than N time Grabity, It is considered a collision force
		breakImpact *= invMass;

		if (breakImpact > D_SIMPLE_FRACTIRE_BREAK_IMPACT_SPEED) {
			dMatrix bodyMatrix;
			dVector com(0.0f);
			dVector veloc(0.0f);
			dVector omega(0.0f);
			dFloat Ixx;
			dFloat Iyy;
			dFloat Izz;
			dFloat mass;

			NewtonWorld* const world = GetWorld();
			// create the shape and visual mesh as a common data to be re used
			DemoEntityManager* const scene = (DemoEntityManager*)NewtonWorldGetUserData(world);

			NewtonBodyGetVelocity(effect.m_body, &veloc[0]);
			NewtonBodyGetOmega(effect.m_body, &omega[0]);
			NewtonBodyGetCentreOfMass(effect.m_body, &com[0]);
			NewtonBodyGetMatrix(effect.m_body, &bodyMatrix[0][0]);
			NewtonBodyGetMass(effect.m_body, &mass, &Ixx, &Iyy, &Izz);

			NewtonCollisionMaterial material;
			int defaultMaterialID = NewtonBodyGetMaterialGroupID(effect.m_body);
			NewtonCollisionGetMaterial(NewtonBodyGetCollision(effect.m_body), &material);

			DemoEntity* const visualEntiry = (DemoEntity*)NewtonBodyGetUserData(effect.m_body);

			com = bodyMatrix.TransformVector(com);
			dMatrix matrix(visualEntiry->GetCurrentMatrix());
			dQuaternion rotation(matrix);

			effect.m_isDead = true;

			// we need to lock the world before creation a bunch of bodies
			scene->Lock(m_lock);

			for (dSimpleVoronoidEffect::dListNode* node = effect.GetFirst(); node; node = node->GetNext()) {
				dFractureAtom& atom = node->GetInfo();

				DemoEntity* const entity = new DemoEntity(dMatrix(rotation, matrix.m_posit), NULL);
				entity->SetMesh(atom.m_mesh, dGetIdentityMatrix());
				scene->Append(entity);

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

				dFloat mass;
				dFloat Ixx;
				dFloat Iyy;
				dFloat Izz;
				NewtonBodyGetMass(rigidBody, &mass, &Ixx, &Iyy, &Izz);
				if (Iyy > 10.0f * Ixx) {
					Iyy *= 0.25f;
				}
				if (Izz > 5.0f * Ixx) {
					Izz *= 0.25f;
				}
				NewtonBodySetMassMatrix(rigidBody, mass, Ixx, Iyy, Izz);

				// save the pointer to the graphic object with the body.
				NewtonBodySetUserData(rigidBody, entity);

				// assign the wood id
				NewtonBodySetMaterialGroupID(rigidBody, defaultMaterialID);

				// set high accuracy simulation
				NewtonBodySetGyroscopicTorque(rigidBody, 1);

				// set a destructor for this rigid body
				NewtonBodySetDestructorCallback(rigidBody, PhysicsBodyDestructor);

				// set the transform call back function
				NewtonBodySetTransformCallback(rigidBody, DemoEntity::TransformCallback);

				// set the force and torque call back function
				NewtonBodySetForceAndTorqueCallback(rigidBody, PhysicsApplyGravityForce);

				NewtonCollision* const collision = NewtonBodyGetCollision(rigidBody);
				NewtonCollisionSetMaterial(collision, &material);
			}

			// unlock the work after done with the effect 
			scene->Unlock(m_lock);
		}
	}

/*
	void AddFracturedWoodPrimitive(
		dFloat density,
		const dVector& origin, const dVector& size,
		int xCount, int zCount, dFloat spacing, int stype, int materialID, const dMatrix& shapeOffsetMatrix)
	{
		// create the shape and visual mesh as a common data to be re used
		NewtonWorld* const world = GetWorld();

		// create the shape and visual mesh as a common data to be re used
		DemoEntityManager* const scene = (DemoEntityManager*)NewtonWorldGetUserData(world);

		NewtonCollision* const collision = NewtonCreateCylinder(world, size.m_x * 0.5f, size.m_z * 0.5f, size.m_y, stype, NULL);

		// create a newton mesh from the collision primitive
		NewtonMesh* const mesh = NewtonMeshCreateFromCollision(collision);

		// apply a material map
		int externalMaterial = LoadTexture("reljef.tga");
		int internalMaterial = LoadTexture("concreteBrick.tga");

		dMatrix aligmentUV(dGetIdentityMatrix());
		NewtonMeshApplyBoxMapping(mesh, externalMaterial, externalMaterial, externalMaterial, &aligmentUV[0][0]);

		// make a visual mesh for display
		DemoMesh* const visualMesh = new DemoMesh(mesh, scene->GetShaderCache());

		// create a  mesh fracture from the newton mesh primitive
		dSimpleVoronoidEffect fracture(world, mesh, internalMaterial);

		dFloat startElevation = 100.0f;
		dMatrix matrix(dGetIdentityMatrix());

		for (int i = 0; i < xCount; i++) {
			dFloat x = origin.m_x + (i - xCount / 2) * spacing;
			for (int j = 0; j < zCount; j++) {
				dFloat z = origin.m_z + (j - zCount / 2) * spacing;

				matrix.m_posit.m_x = x;
				matrix.m_posit.m_z = z;
				dVector floor(FindFloor(world, dVector(matrix.m_posit.m_x, startElevation, matrix.m_posit.m_z, 0.0f), 2.0f * startElevation));
				matrix.m_posit.m_y = floor.m_y + 0.5f;
				dFloat mass = density * NewtonConvexCollisionCalculateVolume(collision);
				NewtonBody* const body = CreateSimpleSolid(scene, visualMesh, mass, matrix, collision, materialID);
				dSimpleVoronoidEffect& newEffect = m_effectList.Append(fracture)->GetInfo();
				newEffect.m_body = body;
			}
		}

		// do not forget to release the assets	
		NewtonMeshDestroy(mesh);
		visualMesh->Release();
		NewtonDestroyCollision(collision);
	}
*/
	void AddFracturedPrimitive(dFloat mass, const dVector& origin, const dVector& size, int xCount, int zCount, dFloat spacing, PrimitiveType type, int materialID, const dMatrix& shapeOffsetMatrix)
	{
		NewtonWorld* const world = GetWorld();
		DemoEntityManager* const scene = (DemoEntityManager*) NewtonWorldGetUserData(world);
		NewtonCollision* const collision = CreateConvexCollision(world, shapeOffsetMatrix, size, type, materialID);

		// create a newton mesh from the collision primitive
		NewtonMesh* const mesh = NewtonMeshCreateFromCollision(collision);

		// apply a material map
		int externalMaterial = LoadTexture("reljef.tga");
		//int internalMaterial = LoadTexture("KAMEN-stup.tga");
		int internalMaterial = LoadTexture("concreteBrick.tga");

		dMatrix aligmentUV(dGetIdentityMatrix());
		NewtonMeshApplyBoxMapping(mesh, externalMaterial, externalMaterial, externalMaterial, &aligmentUV[0][0]);

		// create a newton mesh from the collision primitive
		//VoronoidEffect fracture(world, mesh, internalMaterial);
		dSimpleVoronoidEffect fracture(world, mesh, internalMaterial);

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
				//SimpleFracturedEffectEntity::AddFracturedEntity(scene, visualMesh, collision, fracture, matrix.m_posit);
				NewtonBody* const body = CreateSimpleSolid(scene, visualMesh, mass, matrix, collision, materialID);
				dSimpleVoronoidEffect& newEffect = m_effectList.Append(fracture)->GetInfo();
				newEffect.m_body = body;
			}
		}

		// do not forget to release the assets	
		NewtonMeshDestroy(mesh);
		visualMesh->Release();
		NewtonDestroyCollision(collision);
	}


	dList<dSimpleVoronoidEffect> m_effectList;
	unsigned m_lock;
};

void SimpleConvexFracturing(DemoEntityManager* const scene)
{
	// load the skybox
	scene->CreateSkyBox();

	// load the scene from a ngd file format
	CreateLevelMesh(scene, "flatPlane.ngd", false);
	dVector location(0.0f, 0.0f, 0.0f, 0.0f);
	dVector size(0.75f, 0.75f, 0.75f, 0.0f);
	dMatrix shapeOffsetMatrix(dGetIdentityMatrix());

	dSimpleFractureListener* const fractureManager = new dSimpleFractureListener(scene->GetNewton());

	int count = 5;
	int defaultMaterialID = NewtonMaterialGetDefaultGroupID(scene->GetNewton());
	fractureManager->AddFracturedPrimitive(10.0f, location, size, count, count, 5.0f, _SPHERE_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
	fractureManager->AddFracturedPrimitive(10.0f, location, size, count, count, 5.0f, _CAPSULE_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
	fractureManager->AddFracturedPrimitive(10.0f, location, size, count, count, 5.0f, _BOX_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
	fractureManager->AddFracturedPrimitive(10.0f, location, size, count, count, 5.0f, _CYLINDER_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
	fractureManager->AddFracturedPrimitive(10.0f, location, size, count, count, 5.0f, _CONE_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
	fractureManager->AddFracturedPrimitive(10.0f, location, size, count, count, 5.0f, _REGULAR_CONVEX_HULL_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
	fractureManager->AddFracturedPrimitive(10.0f, location, size, count, count, 5.0f, _RANDOM_CONVEX_HULL_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);

	// place camera into position
	dQuaternion rot;
	dVector origin(-15.0f, 5.0f, 0.0f, 0.0f);
	scene->SetCameraMatrix(rot, origin);
}



