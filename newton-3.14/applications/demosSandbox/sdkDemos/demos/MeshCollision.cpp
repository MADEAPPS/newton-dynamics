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
#include "DemoMesh.h"
#include "DemoEntityManager.h"
#include "DemoCamera.h"
#include "PhysicsUtils.h"


static void SimpleMeshLevel (DemoEntityManager* const scene, bool optimization)
{
	// load the skybox
	scene->CreateSkyBox();

	// load the scene from a ngd file format
//	CreateLevelMesh (scene, "flatPlane.ngd", optimization);
	CreateLevelMesh (scene, "sponza.ngd", optimization);

//	dMatrix camMatrix (dRollMatrix(-20.0f * dDegreeToRad) * dYawMatrix(-45.0f * dDegreeToRad));
	dMatrix camMatrix (dGetIdentityMatrix());
	dQuaternion rot (camMatrix);
	dVector origin (-40.0f, 40.0f, 0.0f, 0.0f);
	scene->SetCameraMatrix(rot, origin);

	NewtonWorld* const world = scene->GetNewton();
	int defaultMaterialID = NewtonMaterialGetDefaultGroupID (world);
	dVector location (0.0f, 0.0f, 0.0f, 0.0f);
	dVector size (0.25f, 0.25f, 0.5f, 0.0f);
	size = size.Scale (2.0f);

	int count = 5;
	dMatrix shapeOffsetMatrix (dGetIdentityMatrix());
	for (int i = 0; i < 20; i ++) {
		AddPrimitiveArray(scene, 10.0f, location, size, count, count, 3.0f, _SPHERE_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
		AddPrimitiveArray(scene, 10.0f, location, size, count, count, 3.0f, _BOX_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
		AddPrimitiveArray(scene, 10.0f, location, size, count, count, 3.0f, _CAPSULE_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
		AddPrimitiveArray(scene, 10.0f, location, size, count, count, 3.0f, _CYLINDER_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
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