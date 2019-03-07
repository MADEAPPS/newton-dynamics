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
#include "DebugDisplay.h"
#include "UserPlaneCollision.h"
#include "HeightFieldPrimitive.h"


static NewtonBody* CreatePlaneCollision (DemoEntityManager* const scene, const dVector& planeEquation)
{
	NewtonCollision* const planeCollision = CreateInfinitePlane (scene->GetNewton(), planeEquation);

	// create the the rigid body for
	dMatrix matrix (dGetIdentityMatrix());
	NewtonBody* const body = NewtonCreateDynamicBody(scene->GetNewton(), planeCollision, &matrix[0][0]);

	// create a visual mesh
	DemoMesh* const mesh = CreateVisualPlaneMesh (planeEquation, scene);
	DemoEntity* const entity = new DemoEntity(matrix, NULL);

	NewtonCollisionGetMatrix(planeCollision, &matrix[0][0]);

	scene->Append (entity);
	entity->SetMesh(mesh, matrix);
	mesh->Release();

	// save the pointer to the graphic object with the body.
	NewtonBodySetUserData (body, entity);

	// destroy the collision shape
	NewtonDestroyCollision (planeCollision);
	return body;
}

static void AddUniformScaledPrimitives (DemoEntityManager* const scene, dFloat mass, const dVector& origin, const dVector& size, int xCount, int zCount, dFloat spacing, PrimitiveType type, int materialID, const dMatrix& shapeOffsetMatrix)
{
	// create the shape and visual mesh as a common data to be re used
	NewtonWorld* const world = scene->GetNewton();
	NewtonCollision* const collision = CreateConvexCollision (world, &shapeOffsetMatrix[0][0], size, type, materialID);

	dFloat startElevation = 1000.0f;
	dMatrix matrix (dRollMatrix(-dPi/2.0f));
	for (int i = 0; i < xCount; i ++) {
		dFloat x = origin.m_x + (i - xCount / 2) * spacing;
		for (int j = 0; j < zCount; j ++) {

			dFloat scale = 0.5f + 2.0f * dFloat (dRand()) / dFloat(dRAND_MAX);

			NewtonCollisionSetScale (collision, scale, scale, scale);
			DemoMesh* const geometry = new DemoMesh("cylinder_1", scene->GetShaderCache(), collision, "smilli.tga", "smilli.tga", "smilli.tga");

			dFloat z = origin.m_z + (j - zCount / 2) * spacing;
			matrix.m_posit.m_x = x;
			matrix.m_posit.m_z = z;
			dVector floor (FindFloor (world, dVector (matrix.m_posit.m_x, startElevation, matrix.m_posit.m_z, 0.0f), 2.0f * startElevation));
			matrix.m_posit.m_y = floor.m_y + 4.f;

			// create a solid
			CreateSimpleSolid (scene, geometry, mass, matrix, collision, materialID);

			// release the mesh
			geometry->Release(); 
		}
	}

	// do not forget to delete the collision
	NewtonDestroyCollision (collision);
}


void UserHeightFieldCollision (DemoEntityManager* const scene)
{
	// load the sky box
	scene->CreateSkyBox();

	CreateHeightFieldTerrain(scene, HEIGHTFIELD_DEFAULT_SIZE, HEIGHTFIELD_DEFAULT_CELLSIZE, 1.5f, 0.2f, 200.0f, -50.0f);
	

	dMatrix camMatrix (dRollMatrix(-20.0f * dDegreeToRad) * dYawMatrix(-45.0f * dDegreeToRad));
	dQuaternion rot (camMatrix);
	dVector origin (250.0f, 0.0f, 250.0f, 0.0f);
	dFloat height = 1000.0f;
	origin = FindFloor (scene->GetNewton(), dVector (origin.m_x, height, origin .m_z, 0.0f), height * 2);
	origin.m_y += 10.0f;

	scene->SetCameraMatrix(rot, origin);

	int defaultMaterialID = NewtonMaterialGetDefaultGroupID (scene->GetNewton());
	dVector location (origin);
	location.m_x += 20.0f;
	location.m_z += 20.0f;
	dVector size (0.5f, 0.5f, 0.75f, 0.0f);

	int count = 5;
//	int count = 1;
	dMatrix shapeOffsetMatrix (dGetIdentityMatrix());
	AddPrimitiveArray(scene, 10.0f, location, size, count, count, 5.0f, _SPHERE_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
	AddPrimitiveArray(scene, 10.0f, location, size, count, count, 5.0f, _BOX_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
	AddPrimitiveArray(scene, 10.0f, location, size, count, count, 5.0f, _CAPSULE_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
	AddPrimitiveArray(scene, 10.0f, location, size, count, count, 5.0f, _CYLINDER_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
	AddPrimitiveArray(scene, 10.0f, location, size, count, count, 5.0f, _CONE_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
	AddPrimitiveArray(scene, 10.0f, location, size, count, count, 5.0f, _CHAMFER_CYLINDER_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
	AddPrimitiveArray(scene, 10.0f, location, size, count, count, 5.0f, _REGULAR_CONVEX_HULL_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
	AddPrimitiveArray(scene, 10.0f, location, size, count, count, 5.0f, _RANDOM_CONVEX_HULL_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
	AddPrimitiveArray(scene, 10.0f, location, size, count, count, 5.0f, _COMPOUND_CONVEX_CRUZ_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
}



void UserPlaneCollision (DemoEntityManager* const scene)
{
	// load the sky box
	scene->CreateSkyBox();

	//	create a user define infinite plane collision, and use it as a floor
	CreatePlaneCollision (scene, dVector (0.0f, 1.0f, 0.0f, 0.0f));


	dMatrix camMatrix (dRollMatrix(-20.0f * dDegreeToRad) * dYawMatrix(-45.0f * dDegreeToRad));
	dQuaternion rot (camMatrix);
	dVector origin (0.0f, 0.0f, 0.0f, 0.0f);
//	dFloat hight = 1000.0f;
//	origin = FindFloor (scene->GetNewton(), dVector (origin.m_x, hight, origin .m_z, 0.0f), hight * 2);
	origin.m_y += 10.0f;

	scene->SetCameraMatrix(rot, origin);

	int defaultMaterialID = NewtonMaterialGetDefaultGroupID (scene->GetNewton());
	dVector location (origin);
	location.m_x += 20.0f;
	location.m_z += 20.0f;
//	dVector size (0.5f, 0.5f, 0.75f, 0.0f);
	dVector size (1.0f, 1.0f, 1.0f, 0.0f);

	int count = 1;
	dMatrix shapeOffsetMatrix (dGetIdentityMatrix());
	AddUniformScaledPrimitives(scene, 10.0f, location, size, count, count, 4.0f, _BOX_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
//	AddPrimitiveArray(scene, 10.0f, location, size, count, count, 5.0f, _SPHERE_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
//	AddPrimitiveArray(scene, 10.0f, location, size, count, count, 5.0f, _BOX_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
//	AddPrimitiveArray(scene, 10.0f, location, size, count, count, 5.0f, _CAPSULE_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
//	AddPrimitiveArray(scene, 10.0f, location, size, count, count, 5.0f, _CYLINDER_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
//	AddPrimitiveArray(scene, 10.0f, location, size, count, count, 5.0f, _CONE_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
//	AddPrimitiveArray(scene, 10.0f, location, size, count, count, 5.0f, _CHAMFER_CYLINDER_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
//	AddPrimitiveArray(scene, 10.0f, location, size, count, count, 5.0f, _REGULAR_CONVEX_HULL_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
//	AddPrimitiveArray(scene, 10.0f, location, size, count, count, 5.0f, _RANDOM_CONVEX_HULL_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
//	AddPrimitiveArray(scene, 10.0f, location, size, count, count, 5.0f, _COMPOUND_CONVEX_CRUZ_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
}