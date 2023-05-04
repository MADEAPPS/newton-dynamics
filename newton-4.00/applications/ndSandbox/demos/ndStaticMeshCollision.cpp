/* Copyright (c) <2003-2022> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/

#include "ndSandboxStdafx.h"
#include "ndSkyBox.h"
#include "ndDemoMesh.h"
#include "ndDemoCamera.h"
#include "ndLoadFbxMesh.h"
#include "ndPhysicsUtils.h"
#include "ndPhysicsWorld.h"
#include "ndCompoundScene.h"
#include "ndMakeStaticMap.h"
#include "ndDemoEntityManager.h"
#include "ndBasicPlayerCapsule.h"
#include "ndHeightFieldPrimitive.h"

#if 0
void ndStaticMeshCollisionDemo (ndDemoEntityManager* const scene)
{
	ndMatrix heighfieldLocation (ndGetIdentityMatrix());
	heighfieldLocation.m_posit.m_x = -200.0f;
	heighfieldLocation.m_posit.m_z = -200.0f;

	//BuildPlayArena(scene);
	//BuildFlatPlane(scene, true);
	//BuildGridPlane(scene, 400, 4.0f, 0.0f);
	//BuildCompoundScene(scene, ndGetIdentityMatrix());
	//BuildHeightFieldTerrain(scene, heighfieldLocation);
	//BuildStaticMesh(scene, "flatPlane.fbx", false);
	//BuildStaticMesh(scene, "track.fbx", false);
	//BuildStaticMesh(scene, "testObject.fbx", false);
	//BuildStaticMesh(scene, "marine_rocks_corsica.fbx", false);
	BuildStaticMesh(scene, "marineRocks1.fbx", false);
	//BuildStaticMesh(scene, "marineRocks2.fbx", false);

	ndMatrix location(ndGetIdentityMatrix());
	location.m_posit.m_y += 2.0f;

	ndMatrix localAxis(ndGetIdentityMatrix());
	localAxis[0] = ndVector(0.0f, 1.0f, 0.0f, 0.0f);
	localAxis[1] = ndVector(1.0f, 0.0f, 0.0f, 0.0f);
	localAxis[2] = localAxis[0].CrossProduct(localAxis[1]);

	ndSharedPtr<ndDemoEntity> man(ndDemoEntity::LoadFbx("walker.fbx", scene));

	//ndFloat32 height = 1.9f;
	//ndFloat32 radio = 0.5f;
	//ndFloat32 mass = 100.0f;
	//new ndBasicPlayerCapsule(scene, man, localAxis, location, mass, radio, height, height/4.0f, true);
	
	location.m_posit.m_x += 8.0f;
	location.m_posit.m_z -= 2.0f;
	//new ndBasicPlayerCapsule(scene, man, localAxis, location, mass, radio, height, height / 4.0f);
	
	location.m_posit.m_z += 4.0f;
	//new ndBasicPlayerCapsule(scene, man, localAxis, location, mass, radio, height, height / 4.0f);

	class PlaceMatrix : public ndMatrix
	{
		public:
		PlaceMatrix(ndFloat32 x, ndFloat32 y, ndFloat32 z)
			:ndMatrix(ndGetIdentityMatrix())
		{
			m_posit.m_x = x;
			m_posit.m_y = y;
			m_posit.m_z = z;
		}
	};

	AddBox(scene, PlaceMatrix(3.0f, 1.0f, 0.0f), 30.0f, 2.0f, 0.25f, 2.5f);
	AddBox(scene, PlaceMatrix(3.0f, 1.5f, 1.125f), 30.0f, 2.0f, 0.25f, 2.5f);
	AddBox(scene, PlaceMatrix(1.0f, 1.0f, 0.0f), 30.0f, 1.0f, 0.25f, 1.0f);
	AddConvexHull(scene, PlaceMatrix(0.0f, 1.0f, 2.0f), 10.0f, 0.6f, 1.0f, 15);
	AddConvexHull(scene, PlaceMatrix(0.0f, 1.0f, 0.0f), 10.0f, 0.7f, 1.0f, 10);
	AddConvexHull(scene, PlaceMatrix(1.0f, 1.0f, 0.0f), 10.0f, 0.5f, 1.2f, 6);
	AddConvexHull(scene, PlaceMatrix(1.0f, 1.0f, 2.0f), 10.0f, 0.6f, 1.0f, 15);
	AddConvexHull(scene, PlaceMatrix(2.0f, 1.0f, 0.0f), 10.0f, 0.7f, 1.0f, 10);
	AddConvexHull(scene, PlaceMatrix(1.0f, 1.0f, 1.0f), 10.0f, 0.5f, 1.2f, 6);

	//AddCapsulesStacks(scene, PlaceMatrix(45.0f, 0.0f, 0.0f), 10.0f, 0.5f, 0.5f, 1.0f, 5, 8, 7);

	ndQuaternion rot(ndYawMatrix(30.0f * ndDegreeToRad));
	//ndVector origin(-5.0f, 4.0f, 0.0f, 1.0f);
	ndVector origin(-3.0f, 0.0f, 2.0f, 1.0f);
	scene->SetCameraMatrix(rot, origin);

	//ndFileFormatSave xxxxSave;
	//xxxxSave.SaveWorld(scene->GetWorld(), "xxxx.nd");
	//ndFileFormatLoad xxxxLoad;
	//xxxxLoad.Load("xxxx.nd");
	//// offset bodies positions for calibration;
	//const ndList<ndSharedPtr<ndBody>>& bodyList = xxxxLoad.GetBodyList();
	//for (ndList<ndSharedPtr<ndBody>>::ndNode* node = bodyList.GetFirst(); node; node = node->GetNext())
	//{
	//	ndSharedPtr<ndBody>& body = node->GetInfo();
	//	ndMatrix bodyMatrix(body->GetMatrix());
	//	bodyMatrix.m_posit.m_x += 4.0f;
	//	body->SetMatrix(bodyMatrix);
	//}
	//xxxxLoad.AddToWorld(scene->GetWorld());
}
#else


static void BuildHeightField(ndDemoEntityManager* const scene)
{
	size_t iDim = 64;
	ndFloat32 dSize = 128, dMaxHeight = 0.5;
	std::vector<ndFloat32> aData; aData.resize(iDim * iDim);
	ndFloat32 fHorizontalScale = ndFloat32(128.0 / ndFloat32(iDim - 1));
	ndShapeInstance shape(new ndShapeHeightfield(ndInt32 (iDim), ndInt32(iDim), ndShapeHeightfield::m_normalDiagonals, fHorizontalScale, fHorizontalScale));
	ndMatrix mLocal(ndGetIdentityMatrix());
	mLocal.m_posit = ndVector(-(dSize * 0.5), 0.0, -(dSize * 0.5), 1.0);
	shape.SetLocalMatrix(mLocal);
	auto pShapeHeightField = shape.GetShape()->GetAsShapeHeightfield();

	for (int i = 0; i < ndInt32(iDim * iDim); ++i)
		pShapeHeightField->GetElevationMap()[i] = ndReal(ndFloat32(rand()) * ndFloat32 (2.0) * dMaxHeight / RAND_MAX);

	pShapeHeightField->UpdateElevationMapAabb();
	ndMatrix uvMatrix(ndGetIdentityMatrix());
	uvMatrix[0][0] *= 0.025f;
	uvMatrix[1][1] *= 0.025f;
	uvMatrix[2][2] *= 0.025f;

	ndSharedPtr<ndDemoMeshInterface>geometry(new ndDemoMesh("box", scene->GetShaderCache(), &shape, "marbleCheckBoard.tga", "marbleCheckBoard.tga", "marbleCheckBoard.tga", 1.0f, uvMatrix, false));
	ndMatrix location(ndGetIdentityMatrix());
	ndDemoEntity* const entity = new ndDemoEntity(location, nullptr);
	entity->SetMesh(geometry);

	ndBodyKinematic* const body = new ndBodyDynamic();
	body->SetMatrix(location);
	body->SetCollisionShape(shape);
	ndSharedPtr<ndBody> bodyPtr(body);
	scene->GetWorld()->AddBody(bodyPtr);
	scene->AddEntity(entity);
}

static void AddBodies(ndDemoEntityManager* const scene)
{
	const int NUM_BOXES_ROWS = 10;
	const int NUM_BOXES_COLS = 10;
	ndFloat32 dUsableSpace = 128;
	ndFloat32 dSpacing_x = dUsableSpace / (NUM_BOXES_COLS + 1);
	ndFloat32 dSpacing_z = dUsableSpace / (NUM_BOXES_ROWS + 1);
	ndFloat32 dStart = -dUsableSpace * 0.5f;
	ndVector vStart(dStart, 10.0f, dStart, 1.0f);
	ndMatrix location(ndGetIdentityMatrix());
	ndMatrix uvMatrix(ndGetIdentityMatrix());
	uvMatrix[0][0] *= 0.025f;
	uvMatrix[1][1] *= 0.025f;
	uvMatrix[2][2] *= 0.025f;

	for (int i = 0; i < NUM_BOXES_ROWS; ++i)
	{
		for (int j = 0; j < NUM_BOXES_COLS; ++j)
		{
			ndFloat32 dOffset_x = dSpacing_x * ndFloat32(j + 1);
			ndFloat32 dOffset_z = dSpacing_z * ndFloat32(i + 1);
			location.m_posit = vStart + ndVector(dOffset_x, ndFloat32(0.0), dOffset_z, ndFloat32(0.0));
			ndBodyKinematic* const body = AddBox(scene, location, 1.0f, 1.0f, 1.0f, 1.0f);
			body->SetMatrix(location);
		}
	}
}

//void ndHeightFieldTest(ndDemoEntityManager* const scene)
void ndStaticMeshCollisionDemo(ndDemoEntityManager* const scene)
{
	// build the height field
	BuildHeightField(scene);
	AddBodies(scene);

	ndQuaternion rot;
	ndVector origin(0.0f, 5.0f, 0.0f, 1.0f);
	scene->SetCameraMatrix(rot, origin);
}
#endif