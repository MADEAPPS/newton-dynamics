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
#include "SkyBox.h"
#include "DemoMesh.h"
#include "DemoEntityManager.h"
#include "DemoCamera.h"
#include "PhysicsUtils.h"
#include "DebugDisplay.h"


#ifdef USE_TEST_ALL_FACE_USER_RAYCAST_CALLBACK
static dFloat AllRayHitCallback (const NewtonBody* const body, const NewtonCollision* const treeCollision, dFloat intersection, dFloat* const normal, int faceId, void* const usedData)
{
	return intersection;
}
#endif


/*
void ScenePrimitive::DebugCallback (const NewtonBody* bodyWithTreeCollision, const NewtonBody* body, int faceID, int vertexCount, const dFloat* vertex, int vertexstrideInBytes)
{
	// the application can use this function for debugging purpose by writing the 
	// face to a global variable then display which face of the mesh are been used for collision.
	// ,,,,,,,,,,,
	// ..........

	// the application can use this function also to modify the collision geometry by changing the face ID
	// this could be uses full to make surface change for example from dry to wet, or breaking glass
	// for this the application should used the functions:
	// int id = NewtonTreeCollisionGetFaceAtribute (treeCollision, indexArray); 
	// NewtonTreeCollisionSetFaceAtribute (treeCollision, indexArray, new id);

	if (DebugDisplayOn()) {
		dVector face[64];
		int stride = vertexstrideInBytes / sizeof (dFloat);
		for (int j = 0; j < vertexCount; j ++) {
			face [j] = dVector (vertex[j * stride + 0], vertex[j * stride + 1] , vertex[j * stride + 2]);
		}
		DebugDrawPolygon (vertexCount, face);
	}
}
*/



class ComplexScene: public DemoEntity 
{
	public:
	ComplexScene (NewtonCollision* const sceneCollision)
		:DemoEntity (dGetIdentityMatrix(), NULL)
		,m_sceneCollision (sceneCollision)
	{
		
	}

	~ComplexScene ()
	{
		// release all meshes		
		for (void* proxyNode = NewtonSceneCollisionGetFirstNode(m_sceneCollision); proxyNode; proxyNode = NewtonSceneCollisionGetNextNode(m_sceneCollision, proxyNode)) {
			NewtonCollision* const collision = NewtonSceneCollisionGetCollisionFromNode (m_sceneCollision, proxyNode);
			DemoMesh* const mesh = (DemoMesh*) NewtonCollisionGetUserData(collision);
			mesh->Release();
		}
	}

	virtual void Render(dFloat timeStep, DemoEntityManager* const scene) const
	{
		// save the model matrix before changing it Matrix
		glPushMatrix();

		// Set The matrix for this entity Node
		glMultMatrix(&m_matrix[0][0]);

		// Render mesh if there is one 
		for (void* proxyNode = NewtonSceneCollisionGetFirstNode(m_sceneCollision); proxyNode; proxyNode = NewtonSceneCollisionGetNextNode(m_sceneCollision, proxyNode)) {
			dMatrix matrix;
			NewtonCollision* const collision = NewtonSceneCollisionGetCollisionFromNode (m_sceneCollision, proxyNode);
			DemoMesh* const mesh = (DemoMesh*) NewtonCollisionGetUserData(collision);

			glPushMatrix();
			NewtonCollisionGetMatrix (collision, &matrix[0][0]);
			glMultMatrix(&matrix[0][0]);
			mesh->Render(scene);

			glPopMatrix();
		}

		// restore the matrix before leaving
		glPopMatrix();
	}

	

	void AddCollisionTreeMesh (DemoEntityManager* const scene)
	{
		// open the level data
		char fullPathName[2048];
		dGetWorkingFileName ("playground.ngd", fullPathName);

		scene->LoadScene (fullPathName);

		// find the visual mesh and make a collision tree
		NewtonWorld* const world = scene->GetNewton();
		DemoEntity* const entity = scene->GetLast()->GetInfo();
		DemoMesh* const mesh = (DemoMesh*)entity->GetMesh();
		dAssert (mesh->IsType(DemoMesh::GetRttiType()));

		NewtonCollision* const tree = NewtonCreateTreeCollision(world, 0);
		NewtonTreeCollisionBeginBuild(tree);

		dFloat* const vertex = mesh->m_vertex;
		for (DemoMesh::dListNode* node = mesh->GetFirst(); node; node = node->GetNext()){
			DemoSubMesh* const subMesh = &node->GetInfo();
			unsigned int* const indices = subMesh->m_indexes;
			int trianglesCount = subMesh->m_indexCount;
			for (int i = 0; i < trianglesCount; i += 3) {

				dVector face[3];
				int index = indices[i + 0] * 3;
				face[0] = dVector (vertex[index + 0], vertex[index + 1], vertex[index + 2]);

				index = indices[i + 1] * 3;
				face[1] = dVector (vertex[index + 0], vertex[index + 1], vertex[index + 2]);

				index = indices[i + 2] * 3;
				face[2] = dVector (vertex[index + 0], vertex[index + 1], vertex[index + 2]);

				int matID = 0;
				//matID = matID == 2 ? 1 : 2 ;
				NewtonTreeCollisionAddFace(tree, 3, &face[0].m_x, sizeof (dVector), matID);
			}
		}
		NewtonTreeCollisionEndBuild (tree, 0);

		// add the collision tree to the collision scene
		void* const proxy = NewtonSceneCollisionAddSubCollision (m_sceneCollision, tree);

		// destroy the original tree collision
		NewtonDestroyCollision (tree);

		// set the parameter on the added collision share
		dMatrix matrix (entity->GetCurrentMatrix());
		NewtonCollision* const collisionTree = NewtonSceneCollisionGetCollisionFromNode (m_sceneCollision, proxy);

		NewtonSceneCollisionSetSubCollisionMatrix (m_sceneCollision, proxy, &matrix[0][0]);	
		NewtonCollisionSetUserData(collisionTree, mesh);

		// set the application level callback
#ifdef USE_STATIC_MESHES_DEBUG_COLLISION
		NewtonStaticCollisionSetDebugCallback (collisionTree, ShowMeshCollidingFaces);
#endif

		mesh->AddRef();

		scene->RemoveEntity (entity);

	#ifdef USE_TEST_ALL_FACE_USER_RAYCAST_CALLBACK
		// set a ray cast callback for all face ray cast 
		NewtonTreeCollisionSetUserRayCastCallback (collisionTree, AllRayHitCallback);
		dVector p0 (0,  100, 0, 0);
		dVector p1 (0, -100, 0, 0);
		dVector normal(0.0f);
		dLong id;
		dFloat parameter;
		parameter = NewtonCollisionRayCast (collisionTree, &p0[0], &p1[0], &normal[0], &id);
	#endif
	}


	void AddCollisionHeightFieldMesh(DemoEntityManager* const scene)
	{
		int size = 101;
		dFloat* const elevation = new dFloat[size * size];

		//y = a * sin(x)* (sin (z)) ^2
		for (int i = 0; i < size; i ++) {
			dFloat z = dPi * i / (size - 1);
			dFloat z2 = dSin (z);
			z2 = z2 * z2 * 5.0f;
			for (int j = 0; j < size; j ++) {
				dFloat x = dPi * j / (size - 1);
				dFloat y = z2 * dSin(x);
				elevation[j * size + i] = y;
			}
		}
		dFloat cellsize = 0.25f;
		DemoMesh* const mesh = new DemoMesh ("terrain", scene->GetShaderCache(), elevation, size, cellsize, 1.0f/16.0f, size - 1);
		
		int width = size;
		int height = size;
		char* const attibutes = new char [size * size];
		memset (attibutes, 0, width * height * sizeof (char));
		NewtonCollision* const collision = NewtonCreateHeightFieldCollision (scene->GetNewton(), width, height, 1, 0, elevation, attibutes, 1.0f, cellsize, cellsize, 0);

		void* const proxy  = NewtonSceneCollisionAddSubCollision (m_sceneCollision, collision);
		NewtonDestroyCollision(collision);


		// set the parameter on the added collision share
		dMatrix matrix (dGetIdentityMatrix());
		matrix.m_posit.m_y = -2.0f;
		matrix.m_posit.m_x = -15.0f;
		matrix.m_posit.m_z = -10.0f;
		NewtonCollision* const terrainCollision = NewtonSceneCollisionGetCollisionFromNode (m_sceneCollision, proxy );

		NewtonSceneCollisionSetSubCollisionMatrix (m_sceneCollision, proxy, &matrix[0][0]);	
		NewtonCollisionSetUserData(terrainCollision, mesh);

		delete[] elevation;
		delete[] attibutes;
	}

	void AddCollisionSingleShape(DemoEntityManager* const scene)
	{
		dMatrix offsetMatrix (dGetIdentityMatrix());
		NewtonCollision* const box = NewtonCreateBox (scene->GetNewton(), 4.0f, 0.1f, 4.0f, 0, &offsetMatrix[0][0]) ;
		DemoMesh* const mesh = new DemoMesh (" box", scene->GetShaderCache(),  box, "metal_30.tga", "metal_30.tga", "metal_30.tga");

		// add compound to the scene collision
		dMatrix matrix;
		void* proxy;
		NewtonCollision* shape;
		
		matrix = dRollMatrix(15.0f * dDegreeToRad);
		matrix.m_posit.m_y = 4.0f;
		matrix.m_posit.m_x = 3.0f;
		matrix.m_posit.m_z = 0.0f;
		proxy = NewtonSceneCollisionAddSubCollision (m_sceneCollision, box);
		shape = NewtonSceneCollisionGetCollisionFromNode (m_sceneCollision, proxy);
		NewtonSceneCollisionSetSubCollisionMatrix (m_sceneCollision, proxy, &matrix[0][0]);	
		NewtonCollisionSetUserData(shape, mesh);
		mesh->AddRef();

		
		matrix = dRollMatrix(-15.0f * dDegreeToRad);
		matrix.m_posit.m_y = 8.0f;
		matrix.m_posit.m_x = -3.0f;
		matrix.m_posit.m_z = 0.0f;
		proxy = NewtonSceneCollisionAddSubCollision (m_sceneCollision, box);
		shape = NewtonSceneCollisionGetCollisionFromNode (m_sceneCollision, proxy);
		NewtonSceneCollisionSetSubCollisionMatrix (m_sceneCollision, proxy, &matrix[0][0]);	
		NewtonCollisionSetUserData(shape, mesh);
		mesh->AddRef();


		NewtonDestroyCollision(box);
		mesh->Release();

	}

	void AddPrimitives(DemoEntityManager* const scene)
	{
		// start adding shapes to this scene collisions 
		NewtonSceneCollisionBeginAddRemove (m_sceneCollision);

		// add a collision tree to the scene
		AddCollisionTreeMesh(scene);

		// add a hight field to the scene
		AddCollisionHeightFieldMesh(scene);

		// add compound 
		AddCollisionSingleShape(scene);

		// finalize adding shapes to this scene collisions 
		NewtonSceneCollisionEndAddRemove (m_sceneCollision);
	}

	NewtonCollision* m_sceneCollision;

};

// called when the a body touches the aabb of the scene collision, in most case must return true and let
// OnSubShapeAABBOverlapTest handle specific overlap with sun collision shapes in the scene collision
//static int OnBodyAABBOverlap (const NewtonMaterial* const material, const NewtonBody* const body0, const NewtonBody* const body1, int threadIndex)
static int OnBodyAABBOverlap(const NewtonJoint* const contactJoint, dFloat timestep, int threadIndex)
{
	return 1;
}


static int OnSubShapeAABBOverlapTest (const NewtonJoint* const contact, dFloat timestep, const NewtonBody* const body0, const void* const collsionNode0, const NewtonBody* const body1, const void* const collsionNode1, int threadIndex)
{
	return 1;
}


static void OnContactCollision (const NewtonJoint* contactJoint, dFloat timestep, int threadIndex)
{

}


void SceneCollision (DemoEntityManager* const scene)
{
	NewtonWorld* world = scene->GetNewton();

	// add the Sky
	scene->CreateSkyBox();

	// create a body and with a scene collision
	NewtonCollision* const sceneCollision = NewtonCreateSceneCollision (scene->GetNewton(), 0);

	// create a visual scene empty mesh
	ComplexScene* const visualMesh = new ComplexScene(sceneCollision);
	scene->Append (visualMesh);


	// add some shapes
	visualMesh->AddPrimitives(scene);

	// this is optional, finish the scene construction, optimize the collision scene
	dMatrix matrix (dGetIdentityMatrix());


	// create the level body and add it to the world
	NewtonBody* const level = NewtonCreateDynamicBody (world, sceneCollision, &matrix[0][0]);

	// replace the collision with the newly created one the  
	visualMesh->m_sceneCollision = NewtonBodyGetCollision(level);

	// set the reference to the visual
	NewtonBodySetUserData(level, visualMesh);

	// do not forget to release the collision
	NewtonDestroyCollision (sceneCollision);


	// add few objects
	int defaultMaterialID = NewtonMaterialGetDefaultGroupID (world);
	NewtonMaterialSetCollisionCallback (world, defaultMaterialID, defaultMaterialID, OnBodyAABBOverlap, OnContactCollision); 
	NewtonMaterialSetCompoundCollisionCallback(world, defaultMaterialID, defaultMaterialID, OnSubShapeAABBOverlapTest);



	dVector location (0.0f, 0.0f, 0.0f, 0.0f);
	dVector size (0.25f, 0.25f, 0.25f, 0.0f);
	dMatrix shapeOffsetMatrix (dGetIdentityMatrix());

	int count = 5;
	AddPrimitiveArray(scene, 10.0f, location, size, count, count, 1.7f, _SPHERE_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
	AddPrimitiveArray(scene, 10.0f, location, size, count, count, 1.7f, _BOX_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
	AddPrimitiveArray(scene, 10.0f, location, size, count, count, 1.7f, _CYLINDER_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
	AddPrimitiveArray(scene, 10.0f, location, size, count, count, 1.7f, _CHAMFER_CYLINDER_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
	AddPrimitiveArray(scene, 10.0f, location, size, count, count, 1.7f, _CONE_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
	AddPrimitiveArray(scene, 10.0f, location, size, count, count, 1.7f, _CAPSULE_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
	AddPrimitiveArray(scene, 10.0f, location, size, count, count, 1.7f, _RANDOM_CONVEX_HULL_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
	AddPrimitiveArray(scene, 10.0f, location, size, count, count, 1.7f, _COMPOUND_CONVEX_CRUZ_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);

	dMatrix camMatrix (dRollMatrix(-20.0f * dDegreeToRad) * dYawMatrix(-45.0f * dDegreeToRad));
	dQuaternion rot (camMatrix);
	dVector origin (-15.0f, 5.0f, -5.0f, 0.0f);
	scene->SetCameraMatrix(rot, origin);
}