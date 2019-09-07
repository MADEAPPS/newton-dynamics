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
#include "TargaToOpenGl.h"
#include "DemoMesh.h"
#include "DemoEntityManager.h"
#include "DemoCamera.h"
#include "PhysicsUtils.h"
#include "DebugDisplay.h"
#include "HeightFieldPrimitive.h"

#if 0
// bug repro demo
void LoadHeights(dFloat* elevation, int width, int height)
{
	dFloat max_height = 0.2f;
	for (int i = 0; i < width*height; ++i) {
		elevation[i] = rand() * max_height / RAND_MAX;
	}
}

void HeightFieldCollision(DemoEntityManager* const scene)
{
	// load the sky box
	scene->CreateSkyBox();
	int size = 64;

	dFloat* elevation = new dFloat[size * size];

	LoadHeights(elevation, size, size);

	dFloat extent = 128.0f; // world size
	dFloat horizontal_scale = 128.0f / (size - 1);

	// create the attribute map
	char* const attibutes = new char[size * size];
	memset(attibutes, 0, size * size * sizeof (char));
	NewtonCollision* height_field = NewtonCreateHeightFieldCollision(scene->GetNewton(), size, size, 0, 0, elevation, attibutes, 1.0f, horizontal_scale, 0);
	NewtonStaticCollisionSetDebugCallback(height_field, ShowMeshCollidingFaces);


	float offset = -extent * 0.5f;

	dMatrix mLocal(dGetIdentityMatrix());
	mLocal.m_posit = dVector(offset, 0, offset);
	NewtonCollisionSetMatrix(height_field, &mLocal[0][0]);

	dMatrix matrix(dGetIdentityMatrix());
	NewtonBody* terrain_body = NewtonCreateDynamicBody(scene->GetNewton(), height_field, &matrix[0][0]);
	NewtonBodySetMassProperties(terrain_body, 0.0, NewtonBodyGetCollision(terrain_body));
	NewtonDestroyCollision(height_field);

	NewtonCollision* collision_pool[3] = {
		NewtonCreateBox(scene->GetNewton(), 1, 1, 1, 0, nullptr),
		NewtonCreateSphere(scene->GetNewton(), 0.5, 0, nullptr),
		NewtonCreateCapsule(scene->GetNewton(), 0.5, 0.5, 1.0, 0, nullptr),
	};

	const int num_bodies_x = 10;
	const int num_bodies_z = 10;

	dFloat usable = extent;

	dFloat spacing_x = usable / (num_bodies_x + 1);
	dFloat spacing_z = usable / (num_bodies_z + 1);
	dFloat start = -usable * 0.5f;

	dVector center(start, 10, start);
	for (int i = 0; i < num_bodies_z; ++i) {
		for (int j = 0; j < num_bodies_x; ++j) {
			dFloat offset_x = spacing_x * (j + 1);
			dFloat offset_z = spacing_z * (i + 1);
			matrix.m_posit = center + dVector(offset_x, 0, offset_z);
			NewtonCollision* col = collision_pool[0];
			NewtonBody* body = NewtonCreateDynamicBody(scene->GetNewton(), col, &matrix[0][0]);
			NewtonBodySetForceAndTorqueCallback(body, PhysicsApplyGravityForce);
			NewtonBodySetMassProperties(body, 1.0, NewtonBodyGetCollision(body));
		}
	}

	for (int i = 0; i < 3; ++i) {
		NewtonDestroyCollision(collision_pool[i]);
	}
	delete[] attibutes;
	delete[] elevation;

	dMatrix locationTransform(dGetIdentityMatrix());
	locationTransform.m_posit.m_y = 2.0f;

	scene->SetCameraMatrix(dQuaternion(locationTransform), locationTransform.m_posit + dVector(0, 5, 0));

	scene->m_debugDisplayMode = 1;
}

#else
void HeightFieldCollision (DemoEntityManager* const scene)
{
	// load the sky box
	scene->CreateSkyBox();

	CreateHeightFieldTerrain(scene, HEIGHTFIELD_DEFAULT_SIZE, HEIGHTFIELD_DEFAULT_CELLSIZE, 1.5f, 0.2f, 200.0f, -50.0f);


	dMatrix locationTransform (dGetIdentityMatrix());
	locationTransform.m_posit = FindFloor (scene->GetNewton(), dVector(126, 50, 50), 100.0f);
	locationTransform.m_posit.m_y += 2.0f;

	const int defaultMaterialID = NewtonMaterialGetDefaultGroupID (scene->GetNewton());
	const dVector location (locationTransform.m_posit + dVector(20, 20, 0));
	const dVector size (0.5f, 0.5f, 0.75f, 0.0f);
	const int count = 5;
	const dMatrix shapeOffsetMatrix (dGetIdentityMatrix());

	AddPrimitiveArray(scene, 10.0f, location, size, count, count, 5.0f, _SPHERE_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
	AddPrimitiveArray(scene, 10.0f, location, size, count, count, 5.0f, _BOX_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
	AddPrimitiveArray(scene, 10.0f, location, size, count, count, 5.0f, _CAPSULE_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
	AddPrimitiveArray(scene, 10.0f, location, size, count, count, 5.0f, _CYLINDER_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
	AddPrimitiveArray(scene, 10.0f, location, size, count, count, 5.0f, _CONE_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
	AddPrimitiveArray(scene, 10.0f, location, size, count, count, 5.0f, _CHAMFER_CYLINDER_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
	AddPrimitiveArray(scene, 10.0f, location, size, count, count, 5.0f, _REGULAR_CONVEX_HULL_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
	AddPrimitiveArray(scene, 10.0f, location, size, count, count, 5.0f, _COMPOUND_CONVEX_CRUZ_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
	AddPrimitiveArray(scene, 10.0f, location, size, count, count, 5.0f, _RANDOM_CONVEX_HULL_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);

	scene->SetCameraMatrix(dQuaternion(locationTransform), locationTransform.m_posit + dVector(-20.0f, 5.0f, 0.0f, 0.0f));
}


#endif

