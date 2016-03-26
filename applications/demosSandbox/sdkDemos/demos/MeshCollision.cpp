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
#include "DemoMesh.h"
#include "DemoEntityManager.h"
#include "DemoCamera.h"
#include "PhysicsUtils.h"


static void SimpleMeshLevel (DemoEntityManager* const scene, bool optimization)
{
	// load the skybox
	scene->CreateSkyBox();

//float xxx[] = {2.3257, -3.6253, -2.7155, 2.7075, -3.6253, -2.7155, 2.7075, -3.6253, -2.3337, 2.3257, -3.6253, -2.3337};
//NewtonCollision* aaa = NewtonCreateConvexHull(scene->GetNewton(), sizeof (xxx) /(3 * sizeof (float)), xxx, 3 * sizeof (float), 0, 0, 0);


	// load the scene from a ngd file format
//	CreateLevelMesh (scene, "flatPlane.ngd", optimization);
	CreateLevelMesh (scene, "sponza.ngd", optimization);
//	CreateLevelMesh (scene, "cattle.ngd", fileName);
//	CreateLevelMesh (scene, "playground.ngd", optimization);
	
	// place camera into position
//	dQuaternion rot;
//	dVector posit (0.0f, 1.0f, 0.0f, 0.0f);
//	scene->SetCameraMatrix(rot, posit);

	dMatrix camMatrix (dRollMatrix(-20.0f * 3.1416f /180.0f) * dYawMatrix(-45.0f * 3.1416f /180.0f));
	dQuaternion rot (camMatrix);
	dVector origin (-30.0f, 40.0f, -15.0f, 0.0f);
	scene->SetCameraMatrix(rot, origin);

	NewtonWorld* const world = scene->GetNewton();
	int defaultMaterialID = NewtonMaterialGetDefaultGroupID (world);
	dVector location (0.0f, 0.0f, 0.0f, 0.0f);
	dVector size (0.5f, 0.5f, 1.0f, 0.0f);

//	int count = 1;
	int count = 6;
	dMatrix shapeOffsetMatrix (dGetIdentityMatrix());
	for (int i = 0; i < 3; i ++) {
		AddPrimitiveArray(scene, 10.0f, location, size, count, count, 3.0f, _SPHERE_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
		AddPrimitiveArray(scene, 10.0f, location, size, count, count, 3.0f, _BOX_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
		AddPrimitiveArray(scene, 10.0f, location, size, count, count, 3.0f, _CAPSULE_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
		AddPrimitiveArray(scene, 10.0f, location, size, count, count, 3.0f, _CYLINDER_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
//		AddPrimitiveArray(scene, 10.0f, location, size, count, count, 3.0f, _TAPERED_CAPSULE_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
//		AddPrimitiveArray(scene, 10.0f, location, size, count, count, 3.0f, _TAPERED_CYLINDER_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
		AddPrimitiveArray(scene, 10.0f, location, size, count, count, 3.0f, _CHAMFER_CYLINDER_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
		AddPrimitiveArray(scene, 10.0f, location, size, count, count, 3.0f, _CONE_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
		AddPrimitiveArray(scene, 10.0f, location, size, count, count, 3.0f, _REGULAR_CONVEX_HULL_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
		AddPrimitiveArray(scene, 10.0f, location, size, count, count, 3.0f, _RANDOM_CONVEX_HULL_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
		AddPrimitiveArray(scene, 10.0f, location, size, count, count, 3.0f, _COMPOUND_CONVEX_CRUZ_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
	}


count = 8;
for (int i = 0; i < 50; i ++){
//AddPrimitiveArray(scene, 10.0f, location, size, count, count, 1.7f, _REGULAR_CONVEX_HULL_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
//AddPrimitiveArray(scene, 10.0f, location, size, count, count, 1.0f, _SPHERE_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
}

}

void OptimizedMeshLevelCollision (DemoEntityManager* const scene)
{
	SimpleMeshLevel (scene, true);
}

void SimpleMeshLevelCollision (DemoEntityManager* const scene)
{
	SimpleMeshLevel (scene, false);
}