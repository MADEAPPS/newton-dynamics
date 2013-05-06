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
#include "../DemoMesh.h"
#include "../DemoEntityManager.h"
#include "../DemoCamera.h"
#include "PhysicsUtils.h"
#include "HeightFieldPrimitive.h"

static void PlaceLargeFloorBox (DemoEntityManager* const scene)
{
	NewtonWorld* const world = scene->GetNewton();
	int defaultMaterialID = NewtonMaterialGetDefaultGroupID (world);

	AddPrimitiveArray(scene, 10.0f, dVector(0,0,0,0), dVector (100, 1, 100, 0), 1, 1, 0, _BOX_PRIMITIVE, defaultMaterialID, GetIdentityMatrix());
	for (NewtonBody* body = NewtonWorldGetFirstBody(world); body; body = NewtonWorldGetNextBody(world, body)) {
		NewtonBodySetMassProperties(body, 0, NewtonBodyGetCollision(body));
		const dMatrix& mat = GetIdentityMatrix();
		NewtonBodySetMatrix (body, &mat[0][0]);
		NewtonBodyGetTransformCallback(body) (body, &mat[0][0], 0);
		NewtonBodyGetTransformCallback(body) (body, &mat[0][0], 0);
	}
}


void ContinueCollision (DemoEntityManager* const scene)
{
	// load the skybox
	scene->CreateSkyBox();


	// load the mesh
	//PlaceLargeFloorBox (scene);
//	CreateLevelMesh (scene, "flatPlane.ngd", 1);
//	CreateLevelMesh (scene, "sponza.ngd", optimization);
//	CreateLevelMesh (scene, "cattle.ngd", fileName);
//	CreateLevelMesh (scene, "playground.ngd", fileName);
	CreateHeightFieldTerrain (scene, 9, 8.0f, 1.5f, 0.2f, 200.0f, -50.0f);

	dMatrix camMatrix (dRollMatrix(-20.0f * 3.1416f /180.0f) * dYawMatrix(-45.0f * 3.1416f /180.0f));
	dQuaternion rot (camMatrix);
	dVector origin (-5.0f, 5.0f, -10.0f, 0.0f);
//	dVector origin (-10.0f, 5.0f, -10.0f, 0.0f);
//	dVector origin (-15.0f, 5.0f, -10.0f, 0.0f);
	scene->SetCameraMatrix(rot, origin);


	NewtonWorld* const world = scene->GetNewton();
	int defaultMaterialID = NewtonMaterialGetDefaultGroupID (world);

	dVector location (0.0f, 0.0f, 0.0f, 0.0f);
//	dVector size (0.5f, 0.5f, 0.5f, 0.0f);
//	dVector size (0.025f, 0.025f, 0.025f, 0.0f); 
	dVector size (0.2f, 0.2f, 0.2f, 0.0f);

location.m_x = 4.0f;
location.m_z = 2.0f;
	int count = 8;
//	int count = 1;

	// this is still a work in progress
	dMatrix shapeOffsetMatrix (GetIdentityMatrix());
	AddPrimitiveArray(scene, 10.0f, location, size, count, count, 1.7f, _SPHERE_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
//AddPrimitiveArray(scene, 10.0f, location, size, count, count, 1.7f, _SPHERE_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
//AddPrimitiveArray(scene, 10.0f, location, size, count, count, 1.7f, _SPHERE_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
//AddPrimitiveArray(scene, 10.0f, location, size, count, count, 1.7f, _SPHERE_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
//AddPrimitiveArray(scene, 10.0f, location, size, count, count, 1.7f, _SPHERE_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
//AddPrimitiveArray(scene, 10.0f, location, size, count, count, 1.7f, _SPHERE_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
//AddPrimitiveArray(scene, 10.0f, location, size, count, count, 1.7f, _SPHERE_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
//	AddPrimitiveArray(scene, 10.0f, location, size, count, count, 1.7f, _BOX_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
//	AddPrimitiveArray(scene, 10.0f, location, size, count, count, 1.7f, _CAPSULE_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
//	AddPrimitiveArray(scene, 10.0f, location, size, count, count, 1.7f, _CYLINDER_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
//	AddPrimitiveArray(scene, 10.0f, location, size, count, count, 1.7f, _CHAMFER_CYLINDER_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
//	AddPrimitiveArray(scene, 10.0f, location, size, count, count, 1.7f, _BOX_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
//	AddPrimitiveArray(scene, 10.0f, location, size, count, count, 1.7f, _CONE_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
//	AddPrimitiveArray(scene, 10.0f, location, size, count, count, 1.7f, _CAPSULE_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
//	AddPrimitiveArray(scene, 10.0f, location, size, count, count, 1.7f, _REGULAR_CONVEX_HULL_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
//	AddPrimitiveArray(scene, 10.0f, location, size, count, count, 1.7f, _RANDOM_CONVEX_HULL_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);

	// set continue collision mode on all dynamics bodies
	for (NewtonBody* body = NewtonWorldGetFirstBody(world); body; body = NewtonWorldGetNextBody(world, body)) {
		float Ix;
		float Iy;
		float Iz;
		float mass;
		NewtonBodyGetMassMatrix(body, &mass, &Ix, &Iy, &Iz);
		if (mass > 0.0f) {
			NewtonBodySetContinuousCollisionMode(body, 1);
		}
	}

}


