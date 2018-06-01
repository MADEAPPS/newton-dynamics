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
#include "TargaToOpenGl.h"
#include "DemoMesh.h"
#include "DemoEntityManager.h"
#include "DemoCamera.h"
#include "PhysicsUtils.h"


static NewtonBody* CreateSimpleNewtonMeshBox (DemoEntityManager* const scene, const dVector& origin, const dVector& scale, dFloat mass)
{
	// the vertex array, vertices's has for values, x, y, z, w
	// w is use as a id to have multiple copy of the same very, like for example mesh that share more than two edges.
	// in most case w can be set to 0.0
	static dFloat64 BoxPoints[] = {
		-1.0, -1.0, -1.0, 0.0,
		-1.0, -1.0,  1.0, 0.0,
		-1.0,  1.0,  1.0, 0.0,
		-1.0,  1.0, -1.0, 0.0,
		 1.0, -1.0, -1.0, 0.0,
		 1.0, -1.0,  1.0, 0.0,
		 1.0,  1.0,  1.0, 0.0,
		 1.0,  1.0, -1.0, 0.0,
	};

	// the vertex index list is an array of all the face, in any order, the can be convex or concave, 
	// and has and variable umber of indices
	static int BoxIndices[] = { 
		2,3,0,1,  // this is quad
		5,2,1,    // triangle
		6,2,5,    // another triangle 
		5,1,0,4,  // another quad
		2,7,3,    // and so on 
		6,7,2,
		3,4,0,
		7,4,3,
		7,5,4,
		6,5,7
	};

	// the number of index for each face is specified by an array of consecutive face index
	static int faceIndexList [] = {4, 3, 3, 4, 3, 3, 3, 3, 3, 3}; 
  
	// each face can have an arbitrary index that the application can use as a material index
	// for example the index point to a texture, we can have the each face of the cube with a different texture
	static int faceMateriaIndexList [] = {0, 4, 4, 2, 3, 3, 3, 3, 3, 3}; 


	// the normal is specified per vertex and each vertex can have a unique normal or a duplicated
	// for example a cube has 6 normals
	static dFloat normal[] = {
		1.0, 0.0, 0.0,
		-1.0, 0.0, 0.0,
		0.0, 1.0, 0.0,
		0.0, -1.0, 0.0,
		0.0, 0.0, 1.0,
		0.0, 0.0, -1.0,
	};

	static int faceNormalIndex [] = {
		0, 0, 0, 0, // first face uses the first normal of each vertex
		3, 3, 3,    // second face uses the third normal
		3, 3, 3,    // third face uses the fifth normal
		1, 1, 1, 1, // third face use the second normal
		2, 2, 2,    // and so on
		2, 2, 2,    
		4, 2, 1,    // a face can have per vertex normals
		4, 4, 4,    
		5, 5, 5,    // two coplanar face can even has different normals 
		3, 2, 0,    
	}; 

	dBigVector array[8];
	dBigVector scale1 (scale);
	for (int i = 0; i < 8; i ++) {
		dBigVector p(&BoxPoints[i * 4]);
		array[i] = scale1 * p;
	}

	NewtonMeshVertexFormat vertexFormat;
	NewtonMeshClearVertexFormat(&vertexFormat);

	vertexFormat.m_faceCount = 10;
	vertexFormat.m_faceIndexCount = faceIndexList;
	vertexFormat.m_faceMaterial = faceMateriaIndexList;

	vertexFormat.m_vertex.m_data = &array[0][0];
	vertexFormat.m_vertex.m_indexList = BoxIndices;
	vertexFormat.m_vertex.m_strideInBytes = sizeof (dBigVector);

	vertexFormat.m_normal.m_data = normal;
	vertexFormat.m_normal.m_indexList = faceNormalIndex;
	vertexFormat.m_normal.m_strideInBytes = 3 * sizeof (dFloat);

	// all channel are now optionals so we not longer has to pass default values
//	vertexFormat.m_uv0.m_data = uv0;
//	vertexFormat.m_uv0.m_indexList = uv0_indexList;
//	vertexFormat.m_uv0.m_strideInBytes = 2 * sizeof (dFloat);

	// now we create and empty mesh
	NewtonMesh* const newtonMesh = NewtonMeshCreate(scene->GetNewton());
	NewtonMeshBuildFromVertexListIndexList(newtonMesh, &vertexFormat);

	// now we can use this mesh for lot of stuff, we can apply UV, we can decompose into convex, 
	NewtonCollision* const collision = NewtonCreateConvexHullFromMesh(scene->GetNewton(), newtonMesh, 0.001f, 0);

	// for now we will simple make simple Box,  make a visual Mesh
	DemoMesh* const visualMesh = new DemoMesh (newtonMesh);

	dMatrix matrix (dGetIdentityMatrix());
	matrix.m_posit = origin;
	matrix.m_posit.m_w = 1.0f;
	NewtonBody* const body = CreateSimpleSolid (scene, visualMesh, mass, matrix, collision, 0);

	visualMesh->Release();
	NewtonDestroyCollision(collision);
	NewtonMeshDestroy (newtonMesh);

	return body;
}

#if 0
void UsingNewtonMeshTool (DemoEntityManager* const scene)
{
	// load the skybox
	scene->CreateSkyBox();

	// load the scene from a ngd file format
	CreateLevelMesh (scene, "flatPlane.ngd", true);

	NewtonSetContactMergeTolerance (scene->GetNewton(), 1.0e-3f);
	CreateSimpleNewtonMeshBox (scene, dVector (0.0f, 2.0f, 0.0f), dVector (1.0f, 0.5f, 2.0f, 0.0f), 1.0f);

	dQuaternion rot;
	dVector origin(-10.0f, 5.0f, 0.0f, 0.0f);
	scene->SetCameraMatrix(rot, origin);
}
#else


static NewtonBody* CreateBox(DemoEntityManager* const scene, dFloat mass, const dMatrix& location, const dVector& size)
{
	NewtonWorld* const world = scene->GetNewton();
	int materialID = NewtonMaterialGetDefaultGroupID(world);
	NewtonCollision* const collision = CreateConvexCollision(world, dGetIdentityMatrix(), size, _BOX_PRIMITIVE, 0);
	DemoMesh* const geometry = new DemoMesh("primitive", collision, "smilli.tga", "smilli.tga", "smilli.tga");

	NewtonBody* const body = CreateSimpleSolid(scene, geometry, mass, location, collision, materialID);

	geometry->Release();
	NewtonDestroyCollision(collision);
	return body;
}


static NewtonBody* CreateSphere(DemoEntityManager* const scene, dFloat mass, const dMatrix& location, const dVector& size)
{
	NewtonWorld* const world = scene->GetNewton();
	int materialID = NewtonMaterialGetDefaultGroupID(world);
	NewtonCollision* const collision = CreateConvexCollision(world, dGetIdentityMatrix(), size, _SPHERE_PRIMITIVE, 0);
	DemoMesh* const geometry = new DemoMesh("primitive", collision, "smilli.tga", "smilli.tga", "smilli.tga");

	NewtonBody* const body = CreateSimpleSolid(scene, geometry, mass, location, collision, materialID);

	geometry->Release();
	NewtonDestroyCollision(collision);
	return body;
}


// black_birth demo
void UsingNewtonMeshTool(DemoEntityManager* const scene)
{
	// load the skybox
	scene->CreateSkyBox();

	// load the scene from a ngd file format
	CreateLevelMesh(scene, "flatPlane.ngd", true);

	dMatrix matrix;
#if 0
	// make the chassis
	dMatrix location (dGetIdentityMatrix());
	location.m_posit = dVector (0.0f, 1.2f, 0.0f, 1.0f);
	NewtonBody* const chassis = CreateBox(scene, 2500.0f, location, dVector (1.1f, 0.8f, 1.0f, 0.0f));
	NewtonBodyGetMatrix(chassis, &matrix[0][0]);
    dCustomSlider* const slider = new dCustomSlider (matrix, chassis, NULL);

	// make the fork
	location.m_posit += dVector(-1.0f, 0.0f, 0.0f, 0.0f);
	NewtonBody* const fork = CreateBox(scene, 50.0f, location, dVector(1.0f, 0.1f, 0.1f, 0.0f));
	dMatrix jointMatrix (dYawMatrix(dPi * 0.5f) * location);
	dVector inertia;
	NewtonBodyGetMass(fork, &inertia.m_w, &inertia.m_x, &inertia.m_y, &inertia.m_z);
	inertia.m_x = inertia.m_y;
	NewtonBodySetMassMatrix(fork, inertia.m_w, inertia.m_x, inertia.m_y, inertia.m_z);

	dCustomSliderActuator* const actuator = new dCustomSliderActuator (jointMatrix, fork, chassis);
	actuator->SetPreconditioner(10.0f);
	//dCustomSlider* const actuator = new dCustomSlider (jointMatrix, fork, chassis);

	// make obstacle
	location.m_posit += dVector(-2.0f, 0.0f, 0.0f, 0.0f);
	CreateBox(scene, 0.0f, location, dVector (1.1f, 0.8f, 1.0f, 0.0f));
#else

	dMatrix location(dGetIdentityMatrix());
	location.m_posit = dVector(0.0f, 0.5f, 0.0f, 1.0f);
	NewtonBody* const sphe0 = CreateSphere(scene, 1.0f, location, dVector(1.0f));

	location.m_posit += dVector(0.0f, 1.0f, 0.0f, 0.0f);
	NewtonBody* const sphe1 = CreateSphere(scene, 100.0f, location, dVector(1.0f));
#endif

	dQuaternion rot;
	dVector origin(-10.0f, 5.0f, 0.0f, 0.0f);
	scene->SetCameraMatrix(rot, origin);
}


#endif

