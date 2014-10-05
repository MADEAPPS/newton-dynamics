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
#include "DemoEntityManager.h"
#include "DemoCamera.h"
#include "DemoMesh.h"
#include "PhysicsUtils.h"

static void BuildJenga (DemoEntityManager* const scene, dFloat mass, const dVector& origin, const dVector& size, int count)
{
	// build a standard block stack of 20 * 3 boxes for a total of 60
	NewtonWorld* const world = scene->GetNewton();

	dVector blockBoxSize (0.8f, 0.5f, 0.8f * 3.0f);
	blockBoxSize = blockBoxSize.Scale (1.0f);

	// create the stack
	dMatrix baseMatrix (dGetIdentityMatrix());

	// for the elevation of the floor at the stack position
	baseMatrix.m_posit.m_x = origin.m_x;
	baseMatrix.m_posit.m_z = origin.m_z;

	dFloat startElevation = 100.0f;
	dVector floor (FindFloor (world, dVector (baseMatrix.m_posit.m_x, startElevation, baseMatrix.m_posit.m_z, 0.0f), 2.0f * startElevation));
	baseMatrix.m_posit.m_y = floor.m_y + blockBoxSize.m_y / 2.0f;

	// set realistic mass and inertia matrix for each block
	mass = 5.0f;

	// create a 90 degree rotation matrix
	dMatrix rotMatrix (dYawMatrix (3.141592f * 0.5f));

	// create a material to control collision with this objects
	int defaultMaterialID;
	defaultMaterialID = NewtonMaterialGetDefaultGroupID (world);

	// separate a bit the block alone the horizontal direction
	dFloat gap = 0.01f;

	// create the shape and visual mesh as a common data to be re used
	NewtonCollision* const collision = CreateConvexCollision (world, dGetIdentityMatrix(), blockBoxSize, _BOX_PRIMITIVE, defaultMaterialID);
	DemoMesh* const geometry = new DemoMesh("box", collision, "wood_0.tga", "wood_0.tga", "wood_0.tga");

	for (int i = 0; i < count; i ++) { 
		dMatrix matrix(baseMatrix);
		matrix.m_posit -= matrix.m_front.Scale (blockBoxSize.m_x - gap); 

		for (int j = 0; j < 3; j ++) { 
			CreateSimpleSolid (scene, geometry, mass, matrix, collision, defaultMaterialID);
			matrix.m_posit += matrix.m_front.Scale (blockBoxSize.m_x + gap);  
		}

		baseMatrix = rotMatrix * baseMatrix;
		baseMatrix.m_posit += matrix.m_up.Scale (blockBoxSize.m_y * 0.99f);
	}

	// do not forget to release the assets	
	geometry->Release(); 
	NewtonDestroyCollision (collision);
}



static void BuildPyramid (DemoEntityManager* const scene, dFloat mass, const dVector& origin, const dVector& size, int count, PrimitiveType type, const dMatrix& shapeMatrix = dGetIdentityMatrix())
{
	dMatrix matrix (dGetIdentityMatrix());
	matrix.m_posit = origin;
	matrix.m_posit.m_w = 1.0f;

	// create the shape and visual mesh as a common data to be re used
	NewtonWorld* const world = scene->GetNewton();

	int defaultMaterialID;
	defaultMaterialID = NewtonMaterialGetDefaultGroupID (world);

	//dMatrix shapeMatrix (dRollMatrix(3.141692f * 0.5f));
	NewtonCollision* const collision = CreateConvexCollision (world, shapeMatrix, size, type, defaultMaterialID);

	DemoMesh* const geometry = new DemoMesh("cylinder_1", collision, "wood_4.tga", "wood_4.tga", "wood_1.tga");
	//DemoMesh____* const geometry = new DemoMesh____("cylinder_1", collision, "smilli.tga", "smilli.tga", "smilli.tga");

	//	matrix = dRollMatrix(3.141592f/2.0f);
	dFloat startElevation = 100.0f;
	dVector floor (FindFloor (world, dVector (matrix.m_posit.m_x, startElevation, matrix.m_posit.m_z, 0.0f), 2.0f * startElevation));

	matrix.m_posit.m_y = floor.m_y + size.m_y / 2.0f; 

	// get the dimension from shape itself
	dVector minP;
	dVector maxP;
	CalculateAABB (collision, dGetIdentityMatrix(), minP, maxP);

	//dFloat stepz = size.m_z + 0.01f;
	dFloat stepz = maxP.m_z - minP.m_z + 0.01f;
	dFloat stepy = maxP.m_y - minP.m_y;

	float y0 = matrix.m_posit.m_y + stepy / 2.0f;
	float z0 = matrix.m_posit.m_z - stepz * count / 2;

	matrix.m_posit.m_y = y0;
	for (int j = 0; j < count; j ++) {
		matrix.m_posit.m_z = z0;
		for (int i = 0; i < (count - j) ; i ++) {
			CreateSimpleSolid (scene, geometry, mass, matrix, collision, defaultMaterialID);
			matrix.m_posit.m_z += stepz;
		}
		z0 += stepz * 0.5f;
		matrix.m_posit.m_y += stepy;
	}

	// do not forget to release the assets	
	geometry->Release(); 
	NewtonDestroyCollision (collision);
}




void BasicBoxStacks (DemoEntityManager* const scene)
{
	// load the skybox
	scene->CreateSkyBox();

	CreateLevelMesh (scene, "flatPlane.ngd", 1);

	// load the scene from a ngd file format
#if 0
	char fileName[2048];
	//GetWorkingFileName ("boxStacks_1.ngd", fileName);
	//GetWorkingFileName ("boxStacks_3.ngd", fileName);
	//GetWorkingFileName ("boxStacks.ngd", fileName);
	//GetWorkingFileName ("pyramid20x20.ngd", fileName);
	GetWorkingFileName ("pyramid40x40.ngd", fileName);
	scene->LoadScene (fileName);
#else
	
	int high = 30;
	PrimitiveType selection[] = {_BOX_PRIMITIVE, _CYLINDER_PRIMITIVE, _TAPERED_CYLINDER_PRIMITIVE, _REGULAR_CONVEX_HULL_PRIMITIVE};
	for (int i = 0; i < 1; i ++) {
		int index = i % (sizeof (selection) / sizeof (selection[0]));

//index = 0;
		dMatrix shapeMatrix (dRollMatrix(0.5f * 3.14159f));
		if (selection[index] == _BOX_PRIMITIVE) {
			shapeMatrix = dGetIdentityMatrix();
		}
//		BuildPyramid (scene, 10.0f, dVector(-10.0f + i * 4.0f, 0.0f, 0.0f, 0.0f), dVector (0.5f, 0.25f, 1.62f/2.0f, 0.0), high, selection[index], shapeMatrix);
//		BuildPyramid (scene, 10.0f, dVector(  0.0f + i * 4.0f, 0.0f, 0.0f, 0.0f), dVector (0.5f, 0.25f, 1.62f/2.0f, 0.0), high, _BOX_PRIMITIVE);
//		BuildPyramid (scene, 10.0f, dVector( 10.0f + i * 4.0f, 0.0f, 0.0f, 0.0f), dVector (0.5f, 0.25f, 1.62f/2.0f, 0.0), high, _CYLINDER_PRIMITIVE, dRollMatrix(0.5f * 3.14159f));
//		BuildPyramid (scene, 10.0f, dVector( 20.0f + i * 4.0f, 0.0f, 0.0f, 0.0f), dVector (0.5f, 0.25f, 1.62f/2.0f, 0.0), high, _TAPERED_CYLINDER_PRIMITIVE, dRollMatrix(0.5f * 3.14159f));
//		BuildPyramid (scene, 10.0f, dVector( 30.0f + i * 4.0f, 0.0f, 0.0f, 0.0f), dVector (0.5f, 0.25f, 1.62f/2.0f, 0.0), high, _REGULAR_CONVEX_HULL_PRIMITIVE, dRollMatrix(0.5f * 3.14159f));
	}

	//high = 20;
	high = 1;
	for (int i = 0; i < 1; i ++) {
		for (int j = 0; j < 1; j ++) {
			BuildJenga (scene, 5.0f, dVector(-15.0f + j * 8, 0.0f, 10.0f + i * 8, 0.0f), dVector (0.5f, 0.25f, 1.62f/2.0f, 0.0), high);
		}
	}
#endif


	// place camera into position
	dQuaternion rot;
	dVector origin (-40.0f, 10.0f, 0.0f, 0.0f);
	scene->SetCameraMatrix(rot, origin);


	//	ExportScene (scene->GetNewton(), "../../../media/test1.ngd");
}


