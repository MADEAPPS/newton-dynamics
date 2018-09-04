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
#include "DemoMesh.h"
#include "DemoEntityManager.h"
#include "DemoCamera.h"
#include "PhysicsUtils.h"


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

			dFloat scale = 0.75f + 1.0f * (dFloat (dRand()) / dFloat(dRAND_MAX) - 0.5f);
			//scale = 1.0f;
			// test skin thinckness
			NewtonCollisionSetSkinThickness(collision, 0.01f);
			NewtonCollisionSetScale (collision, scale, scale, scale);
			DemoMesh* const geometry = new DemoMesh("cylinder_1", collision, "smilli.tga", "smilli.tga", "smilli.tga");

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


static void AddNonUniformScaledPrimitives(DemoEntityManager* const scene, dFloat mass, const dVector& origin, const dVector& size, int xCount, int zCount, dFloat spacing, PrimitiveType type, int materialID, const dMatrix& shapeOffsetMatrix)
{
	// create the shape and visual mesh as a common data to be re used
	NewtonWorld* const world = scene->GetNewton();
	NewtonCollision* const collision = CreateConvexCollision(world, dGetIdentityMatrix(), size, type, materialID);

	dFloat startElevation = 1000.0f;
	dMatrix matrix(dGetIdentityMatrix());
	//matrix = dPitchMatrix(-45.0f * dgDEG2RAD);
	for (int i = 0; i < xCount; i++) {
		dFloat x = origin.m_x + (i - xCount / 2) * spacing;
		for (int j = 0; j < zCount; j++) {

			dFloat scalex = 1.0f + 1.5f * (dFloat(dRand()) / dFloat(dRAND_MAX) - 0.5f);
			dFloat scaley = 1.0f + 1.5f * (dFloat(dRand()) / dFloat(dRAND_MAX) - 0.5f);
			dFloat scalez = 1.0f + 1.5f * (dFloat(dRand()) / dFloat(dRAND_MAX) - 0.5f);

			dFloat z = origin.m_z + (j - zCount / 2) * spacing;
			matrix.m_posit.m_x = x;
			matrix.m_posit.m_z = z;
			dVector floor(FindFloor(world, dVector(matrix.m_posit.m_x, startElevation, matrix.m_posit.m_z, 0.0f), 2.0f * startElevation));
			matrix.m_posit.m_y = floor.m_y + 8.0f;

			// create a solid
			//NewtonBody* const body = CreateSimpleSolid (scene, geometry, mass, matrix, collision, materialID);
			NewtonBody* const body = CreateSimpleSolid(scene, NULL, mass, matrix, collision, materialID);

			DemoEntity* entity = (DemoEntity*)NewtonBodyGetUserData(body);
			NewtonCollisionSetScale(collision, scalex, scaley, scalez);
			DemoMesh* const geometry = new DemoMesh("cylinder_1", collision, "smilli.tga", "smilli.tga", "smilli.tga");
			entity->SetMesh(geometry, dGetIdentityMatrix());
			NewtonBodySetCollisionScale(body, scalex, scaley, scalez);

			dVector omega(0.0f);
			NewtonBodySetOmega(body, &omega[0]);

			// release the mesh
			geometry->Release();
		}
	}

	// do not forget to delete the collision
	NewtonDestroyCollision(collision);
}



static void CreateScaleStaticMesh (DemoEntity* const entity, NewtonCollision* const collision, DemoEntityManager* const scene, const dMatrix& location, const dVector& scale)
{
	// now make a scale version of the same model
	// first Get The mesh and make a scale copy
	DemoMesh* const mesh = (DemoMesh*)entity->GetMesh();
	dAssert (mesh->IsType(DemoMesh::GetRttiType()));

	// since the render does no handle scale, we will made a duplicated mesh with the scale baked in
	DemoMesh* const scaledMesh =  new DemoMesh (*mesh);
	// now scale the vertex list 
	for (int i = 0 ; i < mesh->m_vertexCount; i ++) {
		scaledMesh->m_vertex[i * 3 + 0] *= scale.m_x;
		scaledMesh->m_vertex[i * 3 + 1] *= scale.m_y;
		scaledMesh->m_vertex[i * 3 + 2] *= scale.m_z;
	}
	// re-optimize for render
	scaledMesh->OptimizeForRender();

	DemoEntity* const scaledEntity = new DemoEntity(location, NULL);
	scene->Append (scaledEntity);
	scaledEntity->SetMesh(scaledMesh, dGetIdentityMatrix());
	scaledMesh->Release();

	// now make a body with a scaled collision mesh
	// note: the collision do not have to be destroyed because we did no create a new collision we are simple using an existing one 
	NewtonBody* const scaledBody = NewtonCreateDynamicBody(scene->GetNewton(), collision, &location[0][0]);

	NewtonBodySetUserData(scaledBody, scaledEntity);

	// apply the scale to the new body collision; 
//	// note: scaling a collision mesh does not updates the broadPhase, this function is a until that do update broad phase
	NewtonBodySetCollisionScale (scaledBody, scale.m_x, scale.m_y, scale.m_z);
}

void NonUniformScaledCollision(DemoEntityManager* const scene)
{
	// load the skybox
	scene->CreateSkyBox();

	// load the scene from a ngd file format
	AddFloorBox(scene, dVector (0.0f, 0.0f, 0.0f, 0.0f), dVector (100.0f, 1.0f, 100.0f, 0.0f));
	//CreateLevelMesh(scene, "flatPlane.ngd", 1);
	//CreateLevelMesh (scene, "sponza.ngd", 1);
	//CreateLevelMesh (scene, "cattle.ngd", fileName);
	//CreateLevelMesh (scene, "playground.ngd", 1);

	dMatrix camMatrix(dRollMatrix(-20.0f * dDegreeToRad) * dYawMatrix(-45.0f * dDegreeToRad));
	dQuaternion rot(camMatrix);
	//	dVector origin (-30.0f, 40.0f, -15.0f, 0.0f);
	dVector origin(-10.0f, 5.0f, -15.0f, 0.0f);
	scene->SetCameraMatrix(rot, origin);

	NewtonWorld* const world = scene->GetNewton();
	int defaultMaterialID = NewtonMaterialGetDefaultGroupID(world);
	dVector location(0.0f, 0.0f, 0.0f, 0.0f);
	dVector size0(0.5f, 0.5f, 0.5f, 0.0f);
	dVector size1(0.25f, 0.5f, 0.5f, 0.0f);

	int count = 5;
	dMatrix shapeOffsetMatrix(dRollMatrix(dPi / 2.0f));
	shapeOffsetMatrix.m_posit.m_y = 0.0f;

	AddNonUniformScaledPrimitives(scene, 10.0f, location, size0, count, count, 4.0f, _SPHERE_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
	AddNonUniformScaledPrimitives(scene, 10.0f, location, size0, count, count, 4.0f, _BOX_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
	AddNonUniformScaledPrimitives(scene, 10.0f, location, size0, count, count, 4.0f, _CAPSULE_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
	AddNonUniformScaledPrimitives(scene, 10.0f, location, size1, count, count, 4.0f, _CAPSULE_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
	AddNonUniformScaledPrimitives(scene, 10.0f, location, size1, count, count, 4.0f, _CYLINDER_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
	AddNonUniformScaledPrimitives(scene, 10.0f, location, size0, count, count, 4.0f, _CYLINDER_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
	AddNonUniformScaledPrimitives(scene, 10.0f, location, size0, count, count, 4.0f, _CHAMFER_CYLINDER_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
	AddNonUniformScaledPrimitives(scene, 10.0f, location, size0, count, count, 4.0f, _CONE_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
	AddNonUniformScaledPrimitives(scene, 10.0f, location, size0, count, count, 4.0f, _REGULAR_CONVEX_HULL_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
	AddNonUniformScaledPrimitives(scene, 10.0f, location, size0, count, count, 4.0f, _RANDOM_CONVEX_HULL_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
	AddNonUniformScaledPrimitives(scene, 10.0f, location, size0, count, count, 4.0f, _COMPOUND_CONVEX_CRUZ_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
}


void UniformScaledCollision(DemoEntityManager* const scene)
{
	// load the skybox
	scene->CreateSkyBox();

	// load the scene from a ngd file format
	AddFloorBox(scene, dVector(0.0f, 0.0f, 0.0f, 0.0f), dVector(100.0f, 1.0f, 100.0f, 0.0f));
	//CreateLevelMesh(scene, "flatPlane.ngd", 0);
	//CreateLevelMesh (scene, "sponza.ngd", 0);
	//CreateLevelMesh (scene, "cattle.ngd", fileName);
	//CreateLevelMesh (scene, "playground.ngd", 0);

	dMatrix camMatrix(dRollMatrix(-20.0f * dDegreeToRad) * dYawMatrix(-45.0f * dDegreeToRad));
	dQuaternion rot(camMatrix);
	//	dVector origin (-30.0f, 40.0f, -15.0f, 0.0f);
	dVector origin(-5.0f, 5.0f, -15.0f, 0.0f);
	scene->SetCameraMatrix(rot, origin);

	NewtonWorld* const world = scene->GetNewton();
	int defaultMaterialID = NewtonMaterialGetDefaultGroupID(world);
	dVector location(0.0f, 0.0f, 0.0f, 0.0f);
	dVector size0(0.5f, 0.5f, 0.5f, 0.0f);
	dVector size1(0.25f, 0.5f, 0.5f, 0.0f);

	dMatrix shapeOffsetMatrix(dRollMatrix(dPi / 2.0f));

	int count = 6;
	AddUniformScaledPrimitives(scene, 10.0f, location, size0, count, count, 4.0f, _SPHERE_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
	AddUniformScaledPrimitives(scene, 10.0f, location, size0, count, count, 4.0f, _BOX_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
	AddUniformScaledPrimitives(scene, 10.0f, location, size0, count, count, 4.0f, _CAPSULE_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
	AddUniformScaledPrimitives(scene, 10.0f, location, size1, count, count, 4.0f, _CAPSULE_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
	AddUniformScaledPrimitives(scene, 10.0f, location, size1, count, count, 4.0f, _CYLINDER_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
	AddUniformScaledPrimitives(scene, 10.0f, location, size0, count, count, 4.0f, _CYLINDER_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
	AddUniformScaledPrimitives(scene, 10.0f, location, size0, count, count, 4.0f, _CHAMFER_CYLINDER_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
	AddUniformScaledPrimitives(scene, 10.0f, location, size0, count, count, 4.0f, _CONE_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
	AddUniformScaledPrimitives(scene, 10.0f, location, size0, count, count, 4.0f, _REGULAR_CONVEX_HULL_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
	AddUniformScaledPrimitives(scene, 10.0f, location, size0, count, count, 4.0f, _RANDOM_CONVEX_HULL_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
	AddUniformScaledPrimitives(scene, 10.0f, location, size0, count, count, 4.0f, _COMPOUND_CONVEX_CRUZ_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
}


void ScaledMeshCollision (DemoEntityManager* const scene)
{
	// load the skybox
	scene->CreateSkyBox();

	// load the scene from a ngd file format
	//AddFloorBox(scene, dVector (0.0f, -0.5f, 0.0f, 0.0f), dVector (100.0f, 1.0f, 100.0f, 0.0f));
	CreateLevelMesh (scene, "flatPlane.ngd", 1);
	//CreateLevelMesh (scene, "flatPlaneDoubleFace.ngd", 1);
	//CreateLevelMesh (scene, "sponza.ngd", 0);
	//CreateLevelMesh (scene, "cattle.ngd", fileName);
	//CreateLevelMesh (scene, "playground.ngd", 0);

	//dMatrix camMatrix (dRollMatrix(-20.0f * dDegreeToRad) * dYawMatrix(-45.0f * dDegreeToRad));
	dMatrix camMatrix (dGetIdentityMatrix());
	dQuaternion rot (camMatrix);
	dVector origin (-15.0f, 5.0f, 0.0f, 0.0f);
	//origin = origin.Scale (0.25f);
	scene->SetCameraMatrix(rot, origin);


	NewtonWorld* const world = scene->GetNewton();
	int defaultMaterialID = NewtonMaterialGetDefaultGroupID (world);
	dVector location (0.0f, 0.0f, 0.0f, 0.0f);

	dMatrix matrix (dGetIdentityMatrix());
	matrix.m_posit = location;
	matrix.m_posit.m_x = 0.0f;
	matrix.m_posit.m_y = 0.0f;
	matrix.m_posit.m_z = 0.0f;
	matrix.m_posit.m_w = 1.0f;

	DemoEntity teaPot (dGetIdentityMatrix(), NULL);
	teaPot.LoadNGD_mesh("teapot.ngd", world);
	//teaPot.LoadNGD_mesh("box.ngd", world);

	NewtonCollision* const staticCollision = CreateCollisionTree (world, &teaPot, 0, true);
//	CreateScaleStaticMesh(&teaPot, staticCollision, scene, matrix, dVector(1.0f, 1.0f, 1.0f, 0.0f));

	matrix.m_posit.m_z = -5.0f;
//	CreateScaleStaticMesh(&teaPot, staticCollision, scene, matrix, dVector(0.5f, 0.5f, 2.0f, 0.0f));

	matrix.m_posit.m_z = 5.0f;
	CreateScaleStaticMesh (&teaPot, staticCollision, scene, matrix, dVector (3.0f, 3.0f, 1.5f, 0.0f));

	matrix.m_posit.m_z = 0.0f;
	matrix.m_posit.m_x = -5.0f;
//	CreateScaleStaticMesh(&teaPot, staticCollision, scene, matrix, dVector(0.5f, 0.5f, 0.5f, 0.0f));

	matrix.m_posit.m_x = 5.0f;
//	CreateScaleStaticMesh(&teaPot, staticCollision, scene, matrix, dVector(2.0f, 2.0f, 2.0f, 0.0f));

	// do not forget to destroy the collision mesh helper
	NewtonDestroyCollision(staticCollision);

	dVector size0 (0.5f, 0.5f, 0.5f, 0.0f);
	dVector size1 (0.25f, 0.5f, 0.5f, 0.0f);
	dMatrix shapeOffsetMatrix (dRollMatrix(dPi/2.0f));

	int count = 3;
	AddNonUniformScaledPrimitives(scene, 10.0f, location, size0, count, count, 5.0f, _SPHERE_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
	AddNonUniformScaledPrimitives(scene, 10.0f, location, size0, count, count, 5.0f, _BOX_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
	AddNonUniformScaledPrimitives(scene, 10.0f, location, size0, count, count, 5.0f, _CAPSULE_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
	AddNonUniformScaledPrimitives(scene, 10.0f, location, size1, count, count, 5.0f, _CAPSULE_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
	AddNonUniformScaledPrimitives(scene, 10.0f, location, size1, count, count, 5.0f, _CYLINDER_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
	AddNonUniformScaledPrimitives(scene, 10.0f, location, size0, count, count, 5.0f, _CYLINDER_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
	AddNonUniformScaledPrimitives(scene, 10.0f, location, size0, count, count, 5.0f, _CHAMFER_CYLINDER_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
	AddNonUniformScaledPrimitives(scene, 10.0f, location, size0, count, count, 5.0f, _CONE_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
	AddNonUniformScaledPrimitives(scene, 10.0f, location, size0, count, count, 5.0f, _REGULAR_CONVEX_HULL_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
	AddNonUniformScaledPrimitives(scene, 10.0f, location, size0, count, count, 5.0f, _RANDOM_CONVEX_HULL_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);

	origin.m_x -= 4.0f;
	origin.m_y += 1.0f;
	scene->SetCameraMatrix(rot, origin);	
}
