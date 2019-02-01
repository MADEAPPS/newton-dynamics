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
#include "HeightFieldPrimitive.h"

static NewtonMesh* MakeBox (DemoEntityManager* const scene, const dVector& size, const char* const texture)
{
	NewtonWorld* const world = scene->GetNewton();

	// create a mesh from a collision object
	NewtonCollision* const box = NewtonCreateBox(world, size.m_x, size.m_y, size.m_z, 0, NULL);
	NewtonMesh* const boxMesh = NewtonMeshCreateFromCollision(box);

	int texID = LoadTexture(texture);
	dMatrix aligmentUV(dGetIdentityMatrix());
	NewtonMeshApplyBoxMapping(boxMesh, texID, texID, texID, &aligmentUV[0][0]);

	// delete auxiliary collision shape
	NewtonDestroyCollision(box);

	return boxMesh;
}


static NewtonMesh* MakeCruz (DemoEntityManager* const scene, dFloat lengh, dFloat width, const char* const texture)
{

	dVector size (lengh, width, width, 0.0f);

	// make a elongated Box and combine the along x, y , z axis
	NewtonMesh* const boxMesh = MakeBox (scene, size, texture);
return boxMesh;
/*
	// rotate 90 degree around y
	dMatrix yawMatrix (dYawMatrix(90.0f * dDegreeToRad));
	NewtonMesh* const auxiliaryMesh = NewtonMeshUnion (boxMesh, boxMesh, &yawMatrix[0][0]);
	NewtonMeshPolygonize(auxiliaryMesh);

	// rotate 90 degree around z
	dMatrix rollMatrix (dRollMatrix(90.0f * dDegreeToRad));
	NewtonMesh* const cruz = NewtonMeshUnion (auxiliaryMesh, boxMesh, &rollMatrix[0][0]);
	NewtonMeshPolygonize(cruz);

	// delete auxiliary meshes
	NewtonMeshDestroy(boxMesh);

	NewtonMeshDestroy(auxiliaryMesh);
	return cruz;
*/
}



static void CreateBooleanPhysics (DemoEntityManager* const scene, NewtonMesh* const mesh, const dVector& origin, int instaceCount)
{
	NewtonWorld* const world = scene->GetNewton();

	// make a visual geometry from the mesh
	DemoMesh* const visualMesh = new DemoMesh (mesh, scene->GetShaderCache());

	// create a convex approximation form the original mesh, 32 convex max and no more than 100 vertex convex hulls
	NewtonMesh* const convexApproximation = NewtonMeshApproximateConvexDecomposition (mesh, 0.01f, 0.2f, 32, 100, NULL, NULL);

	// create a compound collision by creation a convex hull of each segment of the source mesh 
	NewtonCollision* const compound = NewtonCreateCompoundCollisionFromMesh (world, convexApproximation, 0.001f, 0, 0);
//	NewtonCollision* const compound = NewtonCreateConvexHullFromMesh(world, mesh, 0.001f, 0);

	dMatrix matrix (dGetIdentityMatrix());
	matrix.m_posit = origin;
	for (int ix = 0; ix < instaceCount; ix ++) {
		for (int iz = 0; iz < instaceCount; iz ++) {
			dFloat y = origin.m_y;
			dFloat x = origin.m_x + (ix - instaceCount/2) * 10.0f;
			dFloat z = origin.m_z + (iz - instaceCount/2) * 10.0f;
			matrix.m_posit = FindFloor (world, dVector (x, y + 10.0f, z, 0.0f), 20.0f); ;
			matrix.m_posit.m_y += 2.0f;
			CreateSimpleSolid (scene, visualMesh, 10.0f, matrix, compound, 0);
		}
	}

	visualMesh->Release();
	NewtonDestroyCollision(compound);
	NewtonMeshDestroy (convexApproximation);
}


void SimpleBooleanOperations (DemoEntityManager* const scene)
{
	// load the skybox
	scene->CreateSkyBox();

dAssert (0);
return;
/*
	// load the scene from a ngd file format
	NewtonBody* const body = CreateLevelMesh (scene, "flatPlane.ngd", true);
//	NewtonBody* const body = CreateLevelMesh (scene, "playground.ngd", true);
//	NewtonBody* const body = CreateLevelMesh (scene, "sponza.ngd", true);
//	NewtonBody* const body = CreateHeightFieldTerrain (scene, 10, 8.0f, 1.5f, 0.2f, 200.0f, -50.0f);

	dMatrix originMatrix;
	NewtonBodyGetMatrix(body, &originMatrix[0][0]);

	dMatrix camMatrix (dRollMatrix(-20.0f * dDegreeToRad) * dYawMatrix(-45.0f * dDegreeToRad));
	//dMatrix camMatrix (GetIdentityMatrix());
	dQuaternion rot (camMatrix);
	dVector origin (originMatrix.m_posit);
	dFloat hight = 1000.0f;
	origin = FindFloor (scene->GetNewton(), dVector (origin.m_x, hight, origin .m_z, 0.0f), hight * 2);
	

	dVector location (origin);
	location.m_x += 20.0f;
	location.m_z += 20.0f;
	location.m_y += 2.0f;

	// make a cruz to use as the carving shape shape
	NewtonMesh* const cruz = MakeCruz (scene, 2.0f, 0.5f, "wood_1.tga");

	// make a box for boolean operations
	NewtonMesh* const box = MakeBox (scene, dVector (1.0f, 1.0f, 1.0f, 0.0f) , "wood_0.tga");

	dMatrix alignMatrix (dGetIdentityMatrix());

	// extract the cruz from the base model
//	NewtonMesh* const boxCruzDiff = NewtonMeshDifference(box, cruz, &alignMatrix[0][0]);
	
	// make the union of these two shapes
//	NewtonMesh* const boxCruzUnion = NewtonMeshUnion (box, cruz, &alignMatrix[0][0]);

	// make the intersection of these two shapes 
	NewtonMesh* const boxCruzIntersection = NewtonMeshIntersection (box, cruz, &alignMatrix[0][0]);

	// now place these few instances of these meshes in the world as physics objects
//	CreateBooleanPhysics (scene, cruz, location + dVector (0, 0, 0, 0), 3);
//	CreateBooleanPhysics (scene, boxCruzUnion, location + dVector (3, 0, 0, 0), 3);
	CreateBooleanPhysics (scene, boxCruzIntersection, location + dVector (0, 0, 3.0, 0), 3);
//	CreateBooleanPhysics (scene, boxCruzDiff, location + dVector (3.0, 0, 3.0, 0), 3);


	// destroy the shapes
	NewtonMeshDestroy(box);
	NewtonMeshDestroy(cruz);
//	NewtonMeshDestroy(boxCruzDiff);
//	NewtonMeshDestroy(boxCruzUnion);
	NewtonMeshDestroy(boxCruzIntersection);

//	ExportScene (scene->GetNewton(), "../../../media/test1.ngd");

	origin.m_y += 10.0f;
	scene->SetCameraMatrix(rot, origin);
*/
}



