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

#if 1
static void BuildPyramid (DemoEntityManager* const scene, dFloat mass, const dVector& origin, const dVector& size, int count)
{
	dMatrix matrix (GetIdentityMatrix());

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
	//BuildPyramid (scene, 10.0f, dVector(0.0f, 0.0f, 0.0f, 0.0f), dVector (0.5f, 0.25f, 1.62f/2.0f, 0.0), 5);
	BuildPyramid (scene, 10.0f, dVector(0.0f, 0.0f, 0.0f, 0.0f), dVector (0.5f, 0.25f, 1.62f/2.0f, 0.0), 50);
#endif


	// place camera into position
	dQuaternion rot;
	dVector origin (-40.0f, 10.0f, 0.0f, 0.0f);
	scene->SetCameraMatrix(rot, origin);


	//	ExportScene (scene->GetNewton(), "../../../media/test1.ngd");
}




#else



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

#include "CustomBallAndSocket.h"
#include "CustomKinematicController.h"


static NewtonBody* BuildBox (DemoEntityManager* const scene, dFloat mass, const dVector& origin, const dVector& size)
{
	dMatrix matrix (GetIdentityMatrix());

	// create the shape and visual mesh as a common data to be re used
	NewtonWorld* const world = scene->GetNewton();
	NewtonCollision* const collision = CreateConvexCollision (world, GetIdentityMatrix(), size, _BOX_PRIMITIVE, 0);


	DemoMesh* const geometry = mass 
		? new DemoMesh("DynBox", collision, "smilli.tga", "smilli.tga", "smilli.tga")
		: new DemoMesh("StaticBox", collision, "wood_0.tga", "wood_0.tga", "wood_1.tga");

	matrix.m_posit = origin; 

	NewtonBody *body = CreateSimpleSolid (scene, geometry, mass, matrix, collision, 0);

	// do not forget to release the assets   
	geometry->Release(); 
	NewtonDestroyCollision (collision);

	return body;
}




void BasicBoxStacks (DemoEntityManager* const scene)
{
	BuildBox (scene, 0.0f, dVector(0,-5,0), dVector (100, 10, 100)); // ground

#if 0

	// provoke CCD assert
	NewtonBody* body = BuildBox (scene, 10.0f, dVector(0,3,1), dVector (4,4,4));
	NewtonBodySetContinuousCollisionMode (body, 1);
	CustomKinematicController *kj = new CustomKinematicController(body, dVector (0,3,0));
	kj->SetMaxLinearFriction (1000);
	kj->SetTargetPosit (dVector (0,0,0));

#elif 1
/*
	// provoke 'penetration bug'
	NewtonBody* body = BuildBox (scene, 1.0f, dVector(0,3,1), dVector (4,4,4));
	CustomKinematicController *kj = new CustomKinematicController(body, dVector (0,3,0));
	dQuaternion q (dVector(1,0,0), 0.7f);
	kj->SetPickMode (1);
	kj->SetTargetPosit (dVector (0,0,0));
	kj->SetTargetRotation (q);
	kj->SetMaxLinearFriction (500);
	kj->SetMaxAngularFriction (500);

// provoke explosion bug
   dVector p (0,3,3);
   NewtonBody* body[2] = {0, BuildBox (scene, 1.0f, p, dVector (4,4,4))};
   NewtonBodySetContinuousCollisionMode (body[1], 1);
   for (int i=0; i<4; i++)
   {
      p[0] += 6;
      body[i&1] = BuildBox (scene, 1.0f, p, dVector (4,4,4));
      NewtonBodySetContinuousCollisionMode (body[i&1], 1);
      dMatrix matrix (GetIdentityMatrix());
      matrix.m_posit = p;// + dVector(-3,0,0);
      new CustomBallAndSocket (matrix, body[!(i&1)], body[i&1]); 
   }


   // provoke assert in CalculateAngularDerivative
   NewtonBody* body = BuildBox (scene, 1.0f, dVector(0,3,0), dVector (4,4,4));
   CustomKinematicController *kj = new CustomKinematicController(body, dVector (0,3,0));
   kj->SetMaxLinearFriction (1000); 
   kj->SetTargetPosit (dVector (0,0,0));
   NewtonBodySetContinuousCollisionMode (body, 1);

	// provoke Minchovski assert
	NewtonBody* body = BuildBox (scene, 1.0f, dVector(0,2,0), dVector (4,4,4));
	NewtonBodySetContinuousCollisionMode (body, 1);
	body = BuildBox (scene, 1.0f, dVector(0,8,3.995f), dVector (4,4,4));
	dVector t (0.0f, 0.1f, 5.0f);
	NewtonBodySetOmega (body, (const float*)&t.m_x);
	NewtonBodySetContinuousCollisionMode (body, 1);

	// provoke another kind of Minkovski
	dVector center (0,30,-12);
	NewtonBody* parentBody = BuildBox (scene, 1.0f, center, dVector (4,4,4));
	NewtonBodySetContinuousCollisionMode (parentBody, 1);
	for (float a = 0.0f; a<3.14; a+=0.2f) {
		NewtonBody* body[2] = {0, parentBody};
		dVector dir (cos(a), 0, sin(a));
		dVector p = center;
		for (int i=0; i<4; i++) {
			p += dir.Scale(5.0f);
			body[i&1] = BuildBox (scene, 1.0f, p, dVector (4,4,4));
			NewtonBodySetContinuousCollisionMode (body[i&1], 1);
			dMatrix matrix (GetIdentityMatrix());
			matrix.m_posit = p;
			new CustomBallAndSocket (matrix, body[!(i&1)], body[i&1]); 
		}
	}

	// provoke 3rd. kind of Minkovski
	dVector center (0,30,-12);
	NewtonBody* parentBody = BuildBox (scene, 1.0f, center, dVector (4,4,4));
	NewtonBodySetContinuousCollisionMode (parentBody, 1);
	for (float a = 0.0f; a<3.14*2; a+=1.0) {
		NewtonBody* body[2] = {0, parentBody};
		dVector dir (cos(a), 0, sin(a));
		dVector p = center;
		for (int i=0; i<2; i++) {
			p += dir.Scale(5.0f);
			body[i&1] = BuildBox (scene, 1.0f, p, dVector (4,4,4));
			NewtonBodySetContinuousCollisionMode (body[i&1], 1);
			dMatrix matrix (GetIdentityMatrix());
			matrix.m_posit = p;
			new CustomBallAndSocket (matrix, body[!(i&1)], body[i&1]); 

			dVector t (0.0f, 0.1f, 5.0f);
			NewtonBodySetOmega (body[i&1], (const float*)&t.m_x);
		}
	}
*/

	// provoke angular joint explosion
   {
	   dVector origin (0,5,0);
	   NewtonBody* body = BuildBox (scene, 1.0f, origin, dVector (6,1,8));
	   CustomKinematicController *kj = new CustomKinematicController (body, origin);
	   dQuaternion q (dVector(1,0,0), float(PI*0.5));
	   kj->SetPickMode (1);
	   kj->SetTargetPosit (origin);
	   kj->SetTargetRotation (q);
	   kj->SetMaxLinearFriction (500);
	   kj->SetMaxAngularFriction (500);

	   NewtonBody* poorBox = BuildBox (scene, 3.0f, dVector (0,2,4), dVector (5,2,10));
	   //NewtonBody* poorBox = BuildBox (scene, 0.5f, dVector (0,2,3), dVector (3,2,3));
	   //NewtonBody* poorBox = BuildBox (scene, 0.5f, dVector (0,2,0), dVector (3,2,3));
	   //NewtonBody* poorBox = BuildBox (scene, 0.5f, dVector (0,2,0.7f), dVector (8,3.0f,1));

	   NewtonBodySetContinuousCollisionMode (poorBox, 1); NewtonBodySetContinuousCollisionMode (body, 1);
   }

#else

	// provoke 'penetration bug' linear only

	NewtonBody* body = BuildBox (scene, 1.0f, dVector(0,3,1), dVector (4,4,4));
	CustomKinematicController *kj = new CustomKinematicController(body, dVector (0,3,0));
	kj->SetTargetPosit (dVector (0,-10,0));
	kj->SetMaxLinearFriction (500);

#endif


	// place camera into position
	dQuaternion rot;
	dVector origin (-20.0f, 2.0f, 0.0f, 0.0f);
	scene->SetCameraMatrix(rot, origin);


	//   ExportScene (scene->GetNewton(), "../../../media/test1.ngd");
}

#endif