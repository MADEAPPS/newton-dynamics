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
#include "HeightFieldPrimitive.h"

static int ReportProgress (dFloat normalizedProgressPercent, void* const userData)
{
	return 1; 
}


static void CreateConvexAproximation (const char* const name, DemoEntityManager* const scene, const dVector& origin, int instaceCount, const char* const texture)
{
	char fileName[2048];
	dGetWorkingFileName (name, fileName);

	NewtonWorld* const world = scene->GetNewton();
	dScene compoundTestMesh (world);
	compoundTestMesh.Deserialize(fileName);

	// freeze the scale and pivot on the model 
	compoundTestMesh.FreezeScale();
//	compoundTestMesh.FreezeGeometryPivot ();
	
	dMeshNodeInfo* meshInfo = NULL;
	dMatrix scale (dGetIdentityMatrix());
	for (dScene::dTreeNode* node = compoundTestMesh.GetFirstNode (); node; node = compoundTestMesh.GetNextNode (node)) {
		dNodeInfo* info = compoundTestMesh.GetInfoFromNode(node);
		if (info->GetTypeId() == dMeshNodeInfo::GetRttiType()) {
			for (void* link = compoundTestMesh.GetFirstParentLink(node); link; link = compoundTestMesh.GetNextParentLink (node, link)) {
				dScene::dTreeNode* const node = compoundTestMesh.GetNodeFromLink(link);
				dNodeInfo* const info = compoundTestMesh.GetInfoFromNode(node);
				if (info->GetTypeId() == dSceneNodeInfo::GetRttiType()) {
					scale = ((dSceneNodeInfo*)info)->GetGeometryTransform();
					break;
				}
			}
			
			meshInfo = (dMeshNodeInfo*) info;
			break;
		}
	}

	NewtonMeshApplyTransform(meshInfo->GetMesh(), &scale[0][0]);

	dAssert (meshInfo);
	dAssert (meshInfo->GetMesh());
#if 1
	NewtonMesh* const mesh = meshInfo->GetMesh();
#else
	dGetWorkingFileName ("mesh.off", fileName);
	NewtonMesh* const mesh = NewtonMeshLoadOFF(world, fileName);
//	dMatrix scale (GetIdentityMatrix());
//	//dFloat scaleMag = 0.05f;
//	dFloat scaleMag = 1.0f;
//	scale[0][0] = scaleMag;
//	scale[1][1] = scaleMag;
//	scale[2][2] = scaleMag;
//	NewtonMesApplyTransform (mesh, &scale[0][0]);
#endif
	//NewtonMesh* const newtonMesh = NewtonMeshSimplify(mesh, 500, ReportProgress);

	// create a convex approximation form the original mesh, 32 convex max and no more than 100 vertex convex hulls
//	NewtonMesh* const convexApproximation = NewtonMeshApproximateConvexDecomposition (mesh, 0.01f, 0.2f, 32, 100, ReportProgress, scene);
	NewtonMesh* const convexApproximation = NewtonMeshApproximateConvexDecomposition (mesh, 0.01f, 0.2f, 256, 100, ReportProgress, scene);
//	NewtonMesh* const convexApproximation = NewtonMeshApproximateConvexDecomposition (mesh, 0.00001f, 0.0f, 256, 100, ReportProgress, scene);

	// create a compound collision by creation a convex hull of each segment of the source mesh 
	NewtonCollision* const compound = NewtonCreateCompoundCollisionFromMesh (world, convexApproximation, 0.001f, 0, 0);

	// test collision mode
//	NewtonCollisionSetCollisonMode(compound, 0);
	
	// make a visual Mesh
	int tex = LoadTexture(texture);
	dMatrix aligmentUV(dGetIdentityMatrix());

	NewtonMeshApplyBoxMapping(mesh, tex, tex, tex, &aligmentUV[0][0]);
	DemoMesh* const visualMesh = new DemoMesh (mesh, scene->GetShaderCache());

	dMatrix matrix (dGetIdentityMatrix());
	matrix.m_posit = origin;
	for (int ix = 0; ix < instaceCount; ix ++) {
		for (int iz = 0; iz < instaceCount; iz ++) {
			dFloat y = origin.m_y;
			dFloat x = origin.m_x + (ix - instaceCount/2) * 10.0f;
			dFloat z = origin.m_z + (iz - instaceCount/2) * 10.0f;
			matrix.m_posit = FindFloor (world, dVector (x, y + 10.0f, z, 0.0f), 20.0f); ;
			matrix.m_posit.m_y += 2.0f;
			CreateSimpleSolid (scene, visualMesh, 10.0f, matrix, compound, 0);
		}
	}

	visualMesh->Release();

	NewtonDestroyCollision(compound);
	NewtonMeshDestroy (convexApproximation);
}



void SimpleConvexApproximation (DemoEntityManager* const scene)
{
	// load the skybox
	scene->CreateSkyBox();

dAssert(0);
return;
/*
	// load the scene from a ngd file format
	NewtonBody* const body = CreateLevelMesh (scene, "flatPlane.ngd", true);
//	NewtonBody* const body = CreateLevelMesh (scene, "sponza.ngd", true);
//	NewtonBody* const body = CreateHeightFieldTerrain (scene, 10, 8.0f, 1.5f, 0.2f, 200.0f, -50.0f);

	dMatrix originMatrix;
	NewtonBodyGetMatrix(body, &originMatrix[0][0]);

	dMatrix camMatrix (dRollMatrix(-20.0f * dDegreeToRad) * dYawMatrix(-45.0f * dDegreeToRad));
	dQuaternion rot (camMatrix);
	dVector origin (originMatrix.m_posit);
	dFloat hight = 1000.0f;
	origin = FindFloor (scene->GetNewton(), dVector (origin.m_x, hight, origin .m_z, 0.0f), hight * 2);
	

	dVector location (origin);
	location.m_x += 20.0f;
	location.m_z += 20.0f;
	location.m_y += 2.0f;

	int defaultMaterialID = NewtonMaterialGetDefaultGroupID (scene->GetNewton());

	// convex approximate some file meshes 
	CreateConvexAproximation ("lshape.ngd", scene, location, 3, "camo.tga");
	CreateConvexAproximation ("hollowBox.ngd", scene, location, 3, "KAMEN.tga");
//	CreateConvexAproximation ("hollowCylinder.ngd", scene, location, 3, "frowny.tga");
//	CreateConvexAproximation ("chair.ngd", scene, location, 3, "checker.tga");
//	CreateConvexAproximation ("cow.ngd", scene, location + dVector (10, 0, 0, 0), 3, "cow.tga");
//	CreateConvexAproximation ("camel.ngd", scene, location + dVector (17, 0, 0, 0), 3, "jirafe.tga");
//	CreateConvexAproximation ("tree.ngd", scene, location, 1, "KAMEN.tga");
//	CreateConvexAproximation ("beetle.ngd", scene, location, 1, "KAMEN.tga");
//	CreateConvexAproximation ("armadello.ngd", scene, location, 1, "KAMEN.tga");
	
	dVector size (0.5f, 0.5f, 0.75f, 0.0f);
	dMatrix shapeOffsetMatrix (dGetIdentityMatrix());
	

	int count = 5;
	AddPrimitiveArray(scene, 10.0f, location, size, count, count, 5.0f, _BOX_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
	AddPrimitiveArray(scene, 10.0f, location, size, count, count, 5.0f, _CAPSULE_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
	AddPrimitiveArray(scene, 10.0f, location, size, count, count, 5.0f, _CYLINDER_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
	AddPrimitiveArray(scene, 10.0f, location, size, count, count, 5.0f, _CONE_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);

//	ExportScene (scene->GetNewton(), "test1.ngd");

	origin.m_y += 10.0f;
	scene->SetCameraMatrix(rot, origin);
*/
}


