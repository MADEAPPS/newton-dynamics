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
#include "DemoMesh.h"
#include "PhysicsUtils.h"

//static void BuildJenga (NewtonFrame& system, dVector origin, int hight)
static void BuildJenga (DemoEntityManager* const scene, dFloat mass, const dVector& origin, const dVector& size, int count)
{
//	int i;
//	int j;
//	dFloat gap;
//	dFloat mass;

	// build a standard block stack of 20 * 3 boxes for a total of 60
	NewtonWorld* const world = scene->GetNewton();

	dVector blockBoxSize (0.8f, 0.5f, 0.8f * 3.0f);
	blockBoxSize = blockBoxSize.Scale (1.0f);

	// create the stack
	dMatrix baseMatrix (GetIdentityMatrix());

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
	NewtonCollision* const collision = CreateConvexCollision (world, GetIdentityMatrix(), blockBoxSize, _BOX_PRIMITIVE, defaultMaterialID);
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



static void BuildPyramid (DemoEntityManager* const scene, dFloat mass, const dVector& origin, const dVector& size, int count)
{
	dMatrix matrix (GetIdentityMatrix());
	matrix.m_posit = origin;
	matrix.m_posit.m_w = 1.0f;

	// create the shape and visual mesh as a common data to be re used
	NewtonWorld* const world = scene->GetNewton();
	NewtonCollision* const collision = CreateConvexCollision (world, GetIdentityMatrix(), size, _BOX_PRIMITIVE, 0);


	//	DemoMesh* const geometry = new DemoMesh("cylinder_1", collision, "wood_0.tga", "wood_0.tga", "wood_1.tga");
	DemoMesh* const geometry = new DemoMesh("cylinder_1", collision, "smilli.tga", "smilli.tga", "smilli.tga");

	//	matrix = dRollMatrix(3.141592f/2.0f);
	dFloat startElevation = 100.0f;
	dVector floor (FindFloor (world, dVector (matrix.m_posit.m_x, startElevation, matrix.m_posit.m_z, 0.0f), 2.0f * startElevation));

	matrix.m_posit.m_y = floor.m_y + size.m_y / 2.0f; 

	dFloat stepz = size.m_z + 0.01f;
	dFloat stepy = size.m_y;

	float y0 = matrix.m_posit.m_z + stepy / 2.0f;
	float z0 = matrix.m_posit.m_z - stepz * count / 2;

	matrix.m_posit.m_y = y0;
	for (int j = 0; j < count; j ++) {

		matrix.m_posit.m_z = z0;
		for (int i = 0; i < (count - j) ; i ++) {
			CreateSimpleSolid (scene, geometry, mass, matrix, collision, 0);
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
	CreateLevelMesh (scene, "flatPlane.ngd", 1);
	for (int i = 0; i < 10; i ++) {
		BuildPyramid (scene, 10.0f, dVector(-10.0f + i * 2.0f, 0.0f, 0.0f, 0.0f), dVector (0.5f, 0.25f, 1.62f/2.0f, 0.0), 20);
	}
	
	for (int i = 0; i < 2; i ++) {
		for (int j = 0; j < 2; j ++) {
			BuildJenga (scene, 5.0f, dVector(-10.0f + j * 8, 0.0f, 10.0f + i * 8, 0.0f), dVector (0.5f, 0.25f, 1.62f/2.0f, 0.0), 100);
		}
	}
#endif


	// place camera into position
	dQuaternion rot;
	dVector origin (-40.0f, 10.0f, 0.0f, 0.0f);
	scene->SetCameraMatrix(rot, origin);


	//	ExportScene (scene->GetNewton(), "../../../media/test1.ngd");
}


