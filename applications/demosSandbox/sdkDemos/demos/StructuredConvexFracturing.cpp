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

#define MAX_POINT_CLOUD_SIZE		500
#define POINT_DENSITY_PER_METERS	0.5f  
#define POISON_VARIANCE				(0.75f * POINT_DENSITY_PER_METERS)



static int MakeRandomPoisonPointCloud(NewtonMesh* const mesh, dVector* const points)
{
	dVector size;
	dMatrix matrix(GetIdentityMatrix()); 
	NewtonMeshCalculateOOBB(mesh, &matrix[0][0], &size.m_x, &size.m_y, &size.m_z);

	dVector minBox (matrix.m_posit - matrix[0].Scale (size.m_x) - matrix[1].Scale (size.m_y) - matrix[2].Scale (size.m_z));
	dVector maxBox (matrix.m_posit + matrix[0].Scale (size.m_x) + matrix[1].Scale (size.m_y) + matrix[2].Scale (size.m_z));

	size = maxBox - minBox;
	int xCount = int (size.m_x / POINT_DENSITY_PER_METERS) + 1;
	int yCount = int (size.m_y / POINT_DENSITY_PER_METERS) + 1;
	int zCount = int (size.m_z / POINT_DENSITY_PER_METERS) + 1;

	int count = 0;
	dFloat z0 = minBox.m_z;
	for (int iz = 0; (iz < zCount) && (count < MAX_POINT_CLOUD_SIZE); iz ++) {
		dFloat y0 = minBox.m_y;
		for (int iy = 0; (iy < yCount) && (count < MAX_POINT_CLOUD_SIZE); iy ++) {
			dFloat x0 = minBox.m_x;
			for (int ix = 0; (ix < xCount) && (count < MAX_POINT_CLOUD_SIZE); ix ++) {

				dFloat x = x0;
				dFloat y = y0;
				dFloat z = z0;
				x += RandomVariable(POISON_VARIANCE);
				y += RandomVariable(POISON_VARIANCE);
				z += RandomVariable(POISON_VARIANCE);
				points[count] = dVector (x, y, z);
				count ++;
				x0 += POINT_DENSITY_PER_METERS;
			}
			y0 += POINT_DENSITY_PER_METERS;
		}
		z0 += POINT_DENSITY_PER_METERS;
	}
	

	return count;
}


static void OnReconstructMainMeshCallBack (NewtonBody* const body, NewtonFracturedCompoundMeshPart* const mainMesh, const NewtonCollision* const fracturedCompoundCollision)
{
	DemoEntity* const entity = (DemoEntity*)NewtonBodyGetUserData(body);
	DemoMesh* const visualMesh = entity->GetMesh();
	dAssert (NewtonCollisionGetType(fracturedCompoundCollision) == SERIALIZE_ID_FRACTURED_COMPOUND);

	visualMesh->RemoveAll();
	for (void* segment = NewtonFracturedCompoundMeshPartGetFirstSegment(mainMesh); segment; segment = NewtonFracturedCompoundMeshPartGetNextSegment (segment)) {
		DemoSubMesh* const subMesh = visualMesh->AddSubMesh();

		int material = NewtonFracturedCompoundMeshPartGetMaterial (segment); 
		int indexCount = NewtonFracturedCompoundMeshPartGetIndexCount (segment); 

		subMesh->m_textureHandle = AddTextureRef ((GLuint)material);

		subMesh->AllocIndexData (indexCount);
		subMesh->m_indexCount = NewtonFracturedCompoundMeshPartGetIndexStream (fracturedCompoundCollision, mainMesh, segment, (int*)subMesh->m_indexes); 
	}
}


static void AddMeshVertexwData (DemoMesh* const visualMesh, NewtonFracturedCompoundMeshPart* const fractureMesh, const NewtonCollision* const fracturedCompoundCollision)
{
	int vertexCount = NewtonFracturedCompoundCollisionGetVertexCount (fracturedCompoundCollision, fractureMesh); 
	const dFloat* const vertex = NewtonFracturedCompoundCollisionGetVertexPositions (fracturedCompoundCollision, fractureMesh);
	const dFloat* const normal = NewtonFracturedCompoundCollisionGetVertexNormals(fracturedCompoundCollision, fractureMesh);
	const dFloat* const uv = NewtonFracturedCompoundCollisionGetVertexUVs (fracturedCompoundCollision, fractureMesh);

	visualMesh->AllocVertexData (vertexCount);
	dAssert (vertexCount == visualMesh->m_vertexCount);
	memcpy (visualMesh->m_vertex, vertex, 3 * vertexCount * sizeof (dFloat));
	memcpy (visualMesh->m_normal, normal, 3 * vertexCount * sizeof (dFloat));
	memcpy (visualMesh->m_uv, uv, 2 * vertexCount * sizeof (dFloat));
}

static void OnEmitFracturedChunk (NewtonBody* const chunkBody, NewtonFracturedCompoundMeshPart* const fractureChunkMesh, const NewtonCollision* const fracturedCompoundCollision)
{
	NewtonWorld* const world = NewtonBodyGetWorld(chunkBody);
	DemoEntityManager* const scene = (DemoEntityManager*)NewtonWorldGetUserData(world);

	// set the force an torque call back
	NewtonBodySetForceAndTorqueCallback (chunkBody, PhysicsApplyGravityForce);

	// set the transform callback 
	NewtonBodySetTransformCallback (chunkBody, DemoEntity::TransformCallback);

	// create the visual entity and mesh, and set the use data
	dMatrix matrix;
	NewtonBodyGetMatrix (chunkBody, &matrix[0][0]);

	DemoEntity* const visualChunkEntity = new DemoEntity(matrix, NULL);
	scene->Append(visualChunkEntity);

	NewtonBodySetUserData (chunkBody, visualChunkEntity);

	// create the mesh geometry and attach it to the entity
	DemoMesh* const visualChunkMesh = new DemoMesh ("fracturedChuckMesh");
	visualChunkEntity->SetMesh (visualChunkMesh, GetIdentityMatrix());
	visualChunkMesh->Release();

	// add the vertex data
	AddMeshVertexwData (visualChunkMesh, fractureChunkMesh, fracturedCompoundCollision);

	// add the mesh indices
	OnReconstructMainMeshCallBack (chunkBody, fractureChunkMesh, fracturedCompoundCollision);
}

static void CreateVisualEntity (DemoEntityManager* const scene, NewtonBody* const body)
{
	dMatrix matrix;
	NewtonBodyGetMatrix(body, &matrix[0][0]);

	DemoEntity* const visualEntity = new DemoEntity(matrix, NULL);
	scene->Append(visualEntity);

	// set the entiry as the user data;
	NewtonBodySetUserData (body, visualEntity);

	// create the mesh geometry and attach it to the entity
	DemoMesh* const visualMesh = new DemoMesh ("fraturedMainMesh");
	visualEntity->SetMesh (visualMesh, GetIdentityMatrix());
	visualMesh->Release();

	// create the mesh and set the vertex array, but not the sub meshes
	NewtonCollision* const fracturedCompoundCollision = NewtonBodyGetCollision(body);
	dAssert (NewtonCollisionGetType(fracturedCompoundCollision) == SERIALIZE_ID_FRACTURED_COMPOUND);

	// add the vertex data
	NewtonFracturedCompoundMeshPart* const mainMesh = NewtonFracturedCompoundGetMainMesh (fracturedCompoundCollision);
	AddMeshVertexwData (visualMesh, mainMesh, fracturedCompoundCollision);

	// now add the sub mesh by calling the call back
	OnReconstructMainMeshCallBack (body, mainMesh, fracturedCompoundCollision);
}


static void AddStructuredFractured (DemoEntityManager* const scene, const dVector& origin, int materialID, const char* const assetName)
{
	// create the shape and visual mesh as a common data to be re used
	NewtonWorld* const world = scene->GetNewton();


#if 0
	// load the mesh asset
	DemoEntity entity(GetIdentityMatrix(), NULL);	
	entity.LoadNGD_mesh (assetName, world);
	DemoMesh* const mesh = entity.GetMesh();
	dAssert (mesh);

	// convert the mesh to a newtonMesh
	NewtonMesh* const solidMesh = mesh->CreateNewtonMesh (world, entity.GetMeshMatrix() * entity.GetCurrentMatrix());
#else
	int externalMaterial = LoadTexture("wood_0.tga");
	NewtonCollision* const collision = CreateConvexCollision (world, GetIdentityMatrix(), dVector (3.0f, 3.0f, 3.0f, 0.0), _BOX_PRIMITIVE, 0);
	NewtonMesh* const solidMesh = NewtonMeshCreateFromCollision(collision);
	NewtonDestroyCollision(collision);
	//NewtonMeshTriangulate(solidMesh);
	NewtonMeshApplyBoxMapping (solidMesh, externalMaterial, externalMaterial, externalMaterial);
#endif


	// create a random point cloud
	dVector points[MAX_POINT_CLOUD_SIZE];
	int pointCount = MakeRandomPoisonPointCloud (solidMesh, points);

	// create and interiors material for texturing the fractured pieces
	//int internalMaterial = LoadTexture("KAMEN-stup.tga");
	int internalMaterial = LoadTexture("concreteBrick.tga");

	// crate a texture matrix for uv mapping of fractured pieces
	dMatrix textureMatrix (GetIdentityMatrix());
	textureMatrix[0][0] = 1.0f / 2.0f;
	textureMatrix[1][1] = 1.0f / 2.0f;

	/// create the fractured collision and mesh
	int debreePhysMaterial = NewtonMaterialGetDefaultGroupID(world);
	NewtonCollision* structuredFracturedCollision = NewtonCreateFracturedCompoundCollision (world, solidMesh, 0, debreePhysMaterial, pointCount, &points[0][0], sizeof (dVector), internalMaterial, &textureMatrix[0][0],
																							OnReconstructMainMeshCallBack, OnEmitFracturedChunk);

// uncomment this to test serialization
#if 0
	FILE* file = fopen ("serialize.bin", "wb");
	NewtonCollisionSerialize (world, structuredFracturedCollision, DemoEntityManager::SerializeFile, file);
	NewtonDestroyCollision (structuredFracturedCollision);
	fclose (file);

	file = fopen ("serialize.bin", "rb");
	structuredFracturedCollision = NewtonCreateCollisionFromSerialization (world, DemoEntityManager::DeserializeFile, file);
	NewtonFracturedCompoundSetCallbacks (structuredFracturedCollision, OnReconstructMainMeshCallBack, OnEmitFracturedChunk);
	fclose (file);
#endif	

#if 1
	void* nextNode;
	dList<void*> detachableNodes;
	NewtonCompoundCollisionBeginAddRemove(structuredFracturedCollision);	
	for (void* node = NewtonCompoundCollisionGetFirstNode(structuredFracturedCollision); node; node = nextNode) { 
		nextNode = NewtonCompoundCollisionGetNextNode(structuredFracturedCollision, node);
		if (NewtonFracturedCompoundIsNodeFreeToDetach (structuredFracturedCollision, node)) {
			detachableNodes.Append(node);
		}
	}

	for (dList<void*>::dListNode* node = detachableNodes.GetFirst(); node; node = node->GetNext()) { 
		NewtonCompoundCollisionRemoveSubCollision (structuredFracturedCollision, node->GetInfo());
	}
	NewtonCompoundCollisionEndAddRemove(structuredFracturedCollision);	
#endif
	

    dVector com;
    dVector inertia;
    NewtonConvexCollisionCalculateInertialMatrix (structuredFracturedCollision, &inertia[0], &com[0]);	

    //float mass = 10.0f;
    //int materialId = 0;
    //create the rigid body
	dMatrix matrix (GetIdentityMatrix());
	matrix.m_posit = origin;
	matrix.m_posit.m_y = 20.0;
	matrix.m_posit.m_w = 1.0f;
    NewtonBody* const rigidBody = NewtonCreateDynamicBody (world, structuredFracturedCollision, &matrix[0][0]);

	// set the mass and center of mass
	dFloat density = 1.0f;
	dFloat mass = density * NewtonConvexCollisionCalculateVolume (structuredFracturedCollision);
	NewtonBodySetMassProperties (rigidBody, mass, structuredFracturedCollision);


	// set the transform call back function
	NewtonBodySetTransformCallback (rigidBody, DemoEntity::TransformCallback);

	// set the force and torque call back function
	NewtonBodySetForceAndTorqueCallback (rigidBody, PhysicsApplyGravityForce);

	// create the entity and visual mesh and attach to the body as user data
	CreateVisualEntity (scene, rigidBody);

    // assign the wood id
//    NewtonBodySetMaterialGroupID (rigidBody, materialId);

    // set a destructor for this rigid body
//    NewtonBodySetDestructorCallback (rigidBody, PhysicsBodyDestructor);

	// release the interior texture
//	ReleaseTexture (internalMaterial);

	// delete the solid mesh since it no longed needed
	NewtonMeshDestroy (solidMesh);

	// destroy the fracture collision
	NewtonDestroyCollision (structuredFracturedCollision);
}



void StructuredConvexFracturing (DemoEntityManager* const scene)
{
	// load the skybox
	scene->CreateSkyBox();

	// load the scene from a ngd file format
	//CreateLevelMesh (scene, "ruinsFloor.ngd", true);
	CreateLevelMesh (scene, "flatPlane.ngd", true);
	//CreateLevelMesh (scene, "sponza.ngd", true);
	//CreateLevelMesh (scene, "sponza.ngd", true);
	AddPrimitiveArray (scene, 0.0f, dVector (0.0f, 0.0f, 0.0f, 0.0f), dVector (100.0f, 1.0f, 100.0f, 0.0f), 1, 1, 0, _BOX_PRIMITIVE, 0, GetIdentityMatrix());



	AddStructuredFractured (scene, dVector (0.0f, 0.0f, 0.0f, 0.0f), 0, "colum.ngd");


	// create a shattered mesh array
	//CreateSimpleVoronoiFracture (scene);

	//int defaultMaterialID = NewtonMaterialGetDefaultGroupID (scene->GetNewton());
	dVector location (0.0f, 0.0f, 0.0f, 0.0f);
	dVector size (0.75f, 0.75f, 0.75f, 0.0f);
	dMatrix shapeOffsetMatrix (GetIdentityMatrix());

	// place camera into position
	dQuaternion rot (dVector (0.0f, 1.0f, 0.0f, 0.0f), -30.0f * 3.141592f / 180.0f); 
	dVector origin (-45.0f, 20.0f, -15.0f, 0.0f);
	scene->SetCameraMatrix(rot, origin);
}



