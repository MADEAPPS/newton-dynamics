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


#include <toolbox_stdafx.h>
#include "SkyBox.h"
#include "TargaToOpenGl.h"
#include "DemoMesh.h"
#include "DemoEntityManager.h"
#include "DemoCamera.h"
#include "PhysicsUtils.h"
#include "DebugDisplay.h"

#if NEWTON_MAJOR_VERSION == 3
class SimpleSoftBodyEntity: public DemoEntity
{
	public:
	SimpleSoftBodyEntity(DemoEntityManager* const scene, const dVector& location)
		:DemoEntity(dGetIdentityMatrix(), NULL)
	{
		dMatrix matrix (dGetIdentityMatrix());

		matrix.m_posit.m_x = location.m_x;
		matrix.m_posit.m_y = location.m_y;
		matrix.m_posit.m_z = location.m_z;
		ResetMatrix(*scene, matrix);

		// add an new entity to the world
		scene->Append(this);
	}

	void BuildSoftBody (DemoEntityManager* const scene, PrimitiveType type, int materialID)
	{
//		NewtonWorld* const world = scene->GetNewton();
//		LoadNGD_mesh(meshName, world);
/*
		NewtonCollision* const box = NewtonCreateBox(world, 1.0f, 1.0f, 1.0f, 0, NULL);
		NewtonMesh* const newtonMesh = NewtonMeshCreateFromCollision (box);
		NewtonMeshTriangulate(newtonMesh);
		DemoMesh* mesh1 = new DemoMesh(newtonMesh);
		SetMesh (mesh1, dGetIdentityMatrix());
		mesh1->Release();
		NewtonMeshDestroy (newtonMesh);
		NewtonDestroyCollision(box);
		DemoMesh* const mesh = (DemoMesh*) GetMesh();

		int faceCount = 0;
		int subMeshIndex = 0;
		int faceList[2024 * 3];
		int materialIndex[2024];
		int faceIndexCount[1024];

		NewtonMesh* const softBodyMesh = NewtonMeshCreate(world);
		for (dList<DemoSubMesh>::dListNode* segmentNode = mesh->GetFirst(); segmentNode; segmentNode = segmentNode->GetNext()) {
			const DemoSubMesh& subMesh = segmentNode->GetInfo();
			for (int i = 0; i < subMesh.m_indexCount / 3; i++) {
				faceIndexCount[i + faceCount] = 3;
				materialIndex[i + faceCount] = subMeshIndex;
				faceList[faceCount * 3 + i * 3 + 0] = subMesh.m_indexes[i * 3 + 0];
				faceList[faceCount * 3 + i * 3 + 1] = subMesh.m_indexes[i * 3 + 1];
				faceList[faceCount * 3 + i * 3 + 2] = subMesh.m_indexes[i * 3 + 2];
			}
			subMeshIndex++;
			faceCount += subMesh.m_indexCount / 3;
		}

		dFloat64 tmpVertex[2024][3];
		for (int i = 0; i < mesh->m_vertexCount; i++) {
			tmpVertex[i][0] = mesh->m_vertex[i * 3 + 0];
			tmpVertex[i][1] = mesh->m_vertex[i * 3 + 1];
			tmpVertex[i][2] = mesh->m_vertex[i * 3 + 2];
		}

		NewtonMeshVertexFormat vertexFormat;
		NewtonMeshClearVertexFormat(&vertexFormat);

		vertexFormat.m_faceCount = faceCount;
		vertexFormat.m_faceIndexCount = faceIndexCount;
		vertexFormat.m_faceMaterial = materialIndex;

		vertexFormat.m_vertex.m_data = &tmpVertex[0][0];
		vertexFormat.m_vertex.m_indexList = faceList;
		vertexFormat.m_vertex.m_strideInBytes = 3 * sizeof (dFloat64);

		vertexFormat.m_normal.m_data = mesh->m_normal;
		vertexFormat.m_normal.m_indexList = faceList;
		vertexFormat.m_normal.m_strideInBytes = 3 * sizeof (dFloat);

		vertexFormat.m_uv0.m_data = mesh->m_normal;
		vertexFormat.m_uv0.m_indexList = faceList;
		vertexFormat.m_uv0.m_strideInBytes = 2 * sizeof (dFloat);
		NewtonMeshBuildFromVertexListIndexList (softBodyMesh, &vertexFormat);

		NewtonMeshOptimize(softBodyMesh);
		NewtonCollision* const deformableCollision = NewtonCreateDeformableSolid(world, softBodyMesh, 0);
		

//		dgVector xxx[4];
//		NewtonDeformableMeshConstraintParticle (deformableCollision, 0, &xxx[0].m_x, NULL);
//		NewtonDeformableMeshConstraintParticle (deformableCollision, 0, &xxx[1].m_x, NULL);
//		NewtonDeformableMeshConstraintParticle (deformableCollision, 0, &xxx[2].m_x, NULL);
//		NewtonDeformableMeshConstraintParticle (deformableCollision, 0, &xxx[3].m_x, NULL);

		dFloat mass = 8.0f;
		CreateRigidBody (scene, mass, deformableCollision);

		NewtonDestroyCollision(deformableCollision);
		NewtonMeshDestroy(softBodyMesh);
*/
		dFloat mass = 5.0f;
		NewtonWorld* const world = scene->GetNewton();

		dVector size(1.0f, 1.0f, 1.0f, 0.0f);
		NewtonCollision* const collisionBox = CreateConvexCollision(world, dGetIdentityMatrix(), size, type, materialID);
		NewtonMesh* const newtonMesh = NewtonMeshCreateFromCollision(collisionBox);

		int tex = LoadTexture("smilli.tga");
		NewtonMeshApplyBoxMapping (newtonMesh, tex, tex, tex);
//		NewtonMeshTriangulate(newtonMesh);

		DemoMesh* const mesh1 = new DemoMesh(newtonMesh);
		SetMesh(mesh1, dGetIdentityMatrix());

		// make a deformable collision mesh
//		NewtonCollision* const deformableCollision = NewtonCreateDeformableSolid(world, newtonMesh, materialID);
		NewtonCollision* const deformableCollision = NewtonCreateClothPatch(world, newtonMesh, materialID);

		//create a rigid body with a deformable mesh
		CreateRigidBody (scene, mass, deformableCollision);

		// do not forget to destroy this objects, else you get bad memory leaks.
		mesh1->Release();
		NewtonMeshDestroy (newtonMesh);
		NewtonDestroyCollision(collisionBox);
		NewtonDestroyCollision(deformableCollision);
	}

	NewtonMesh* CreateFlatClothPatch(DemoEntityManager* const scene, int size_x, int size_z)
	{
		size_x += 1;
		size_z += 1;
		dAssert(size_x < 128);
		dAssert(size_z < 128);
		dFloat dimension = 0.25f;
		dVector points[128][128];

		dFloat y = 0.0f;
		int enumerator = 0;
		for (int i = 0; i < size_z; i++) {
			dFloat z = (i - size_z / 2) * dimension;
			for (int j = 0; j < size_x; j++) {
				dFloat x = (j - size_x / 2) * dimension;
				points[i][j] = dVector(x, y, z, dFloat(enumerator));
				enumerator++;
			}
		}

		NewtonMesh* const clothPatch = NewtonMeshCreate(scene->GetNewton());
		NewtonMeshBeginBuild(clothPatch);
		for (int i = 0; i < size_z - 1; i++) {
			//if (i & 1) {
			if (1) {
				for (int j = 0; j < size_x - 1; j++) {
					NewtonMeshBeginFace(clothPatch);
						NewtonMeshAddPoint(clothPatch, points[i + 0][j + 0].m_x, points[i + 0][j + 0].m_y, points[i + 0][j + 0].m_z);
						NewtonMeshAddMaterial(clothPatch, 0);

						NewtonMeshAddPoint(clothPatch, points[i + 0][j + 1].m_x, points[i + 0][j + 1].m_y, points[i + 0][j + 1].m_z);
						NewtonMeshAddMaterial(clothPatch, 0);

						NewtonMeshAddPoint(clothPatch, points[i + 1][j + 1].m_x, points[i + 1][j + 1].m_y, points[i + 1][j + 1].m_z);
						NewtonMeshAddMaterial(clothPatch, 0);
					NewtonMeshEndFace(clothPatch);

					NewtonMeshBeginFace(clothPatch);
						NewtonMeshAddPoint(clothPatch, points[i + 0][j + 0].m_x, points[i + 0][j + 0].m_y, points[i + 0][j + 0].m_z);
						NewtonMeshAddMaterial(clothPatch, 0);

						NewtonMeshAddPoint(clothPatch, points[i + 1][j + 1].m_x, points[i + 1][j + 1].m_y, points[i + 1][j + 1].m_z);
						NewtonMeshAddMaterial(clothPatch, 0);

						NewtonMeshAddPoint(clothPatch, points[i + 1][j + 0].m_x, points[i + 1][j + 0].m_y, points[i + 1][j + 0].m_z);
						NewtonMeshAddMaterial(clothPatch, 0);
					NewtonMeshEndFace(clothPatch);
				}
			} else {
				for (int j = 0; j < size_x - 1; j++) {
					NewtonMeshBeginFace(clothPatch);
						NewtonMeshAddPoint(clothPatch, points[i + 0][j + 0].m_x, points[i + 0][j + 0].m_y, points[i + 0][j + 0].m_z);
						NewtonMeshAddMaterial(clothPatch, 0);

						NewtonMeshAddPoint(clothPatch, points[i + 0][j + 1].m_x, points[i + 0][j + 1].m_y, points[i + 0][j + 1].m_z);
						NewtonMeshAddMaterial(clothPatch, 0);

						NewtonMeshAddPoint(clothPatch, points[i + 1][j + 0].m_x, points[i + 1][j + 0].m_y, points[i + 1][j + 0].m_z);
						NewtonMeshAddMaterial(clothPatch, 0);
					NewtonMeshEndFace(clothPatch);

					NewtonMeshBeginFace(clothPatch);
						NewtonMeshAddPoint(clothPatch, points[i + 0][j + 1].m_x, points[i + 0][j + 1].m_y, points[i + 0][j + 1].m_z);
						NewtonMeshAddMaterial(clothPatch, 0);

						NewtonMeshAddPoint(clothPatch, points[i + 1][j + 1].m_x, points[i + 1][j + 1].m_y, points[i + 1][j + 1].m_z);
						NewtonMeshAddMaterial(clothPatch, 0);

						NewtonMeshAddPoint(clothPatch, points[i + 1][j + 0].m_x, points[i + 1][j + 0].m_y, points[i + 1][j + 0].m_z);
						NewtonMeshAddMaterial(clothPatch, 0);
					NewtonMeshEndFace(clothPatch);
				}
			}
		}
		NewtonMeshEndBuild(clothPatch);

		int material = LoadTexture("persianRug.tga");
		NewtonMeshApplyBoxMapping(clothPatch, material, material, material);
		return clothPatch;
	}


	void BuildClothPatch (DemoEntityManager* const scene, int size_x, int size_z)
	{
		NewtonWorld* const world = scene->GetNewton();

		NewtonMesh* const clothPatch = CreateFlatClothPatch(scene, size_x, size_z);
		NewtonCollision* const deformableCollision = NewtonCreateClothPatch(world, clothPatch, 0);

		dFloat mass = 8.0f;
		CreateRigidBody(scene, mass, deformableCollision);

		DemoMesh* const mesh = new DemoMesh(clothPatch);
		SetMesh(mesh, dGetIdentityMatrix());

		// do not forget to destroy this objects, else you get bad memory leaks.
		mesh->Release();
		NewtonMeshDestroy(clothPatch);
		NewtonDestroyCollision(deformableCollision);
	}


	void CreateRigidBody(DemoEntityManager* const scene, dFloat mass, NewtonCollision* const deformableCollision)
	{
		//create the rigid body
		NewtonWorld* const world = scene->GetNewton();
		dMatrix matrix(GetCurrentMatrix());

		//matrix.m_posit.m_y = FindFloor (world, matrix.m_posit.m_x, matrix.m_posit.m_z) + 4.0f;
		SetMatrix(*scene, dQuaternion(), matrix.m_posit);
		SetMatrix(*scene, dQuaternion(), matrix.m_posit);
		NewtonBody* const deformableBody = NewtonCreateDynamicBody(world, deformableCollision, &matrix[0][0]);

		// set the mass matrix
		NewtonBodySetMassProperties(deformableBody, mass, deformableCollision);

		// save the pointer to the graphic object with the body.
		NewtonBodySetUserData(deformableBody, this);

		// assign the wood id
		//	NewtonBodySetMaterialGroupID (deformableBody, materialId);

		// set a destructor for this rigid body
		NewtonBodySetDestructorCallback(deformableBody, PhysicsBodyDestructor);

		// set the transform call back function
		NewtonBodySetTransformCallback(deformableBody, DemoEntity::TransformCallback);

		// set the force and torque call back function
		NewtonBodySetForceAndTorqueCallback(deformableBody, PhysicsApplyGravityForce);
	}

	virtual void Render(dFloat timeStep, DemoEntityManager* const scene) const
	{
		glDisable(GL_CULL_FACE);
		DemoEntity::Render(timeStep, scene);
		glEnable(GL_CULL_FACE);
	}

	NewtonCollision* m_softCollision;
};

#endif


void SoftBodies(DemoEntityManager* const scene)
{
	// load the sky box
	scene->CreateSkyBox();

	// load the scene from a ngd file format
	CreateLevelMesh (scene, "flatPlane.ngd", 1);
	//CreateLevelMesh (scene, "playground.ngd", 1);

	dVector location (0.0f, 4.0f, 0.0f, 0.0f);

	//AddPrimitiveArray(scene, 10.0f, location, size, count, count, 3.0f, _SPHERE_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
	SimpleSoftBodyEntity* const entity = new SimpleSoftBodyEntity(scene, location);
	entity->BuildSoftBody (scene, _BOX_PRIMITIVE, 0);
	
	dQuaternion rot;
	dVector origin (location.m_x - 10.0f, 2.0f, location.m_z, 0.0f);
	scene->SetCameraMatrix(rot, origin);
}

void ClothPatch(DemoEntityManager* const scene)
{
	// load the skybox
	scene->CreateSkyBox();

	// load the scene from a ngd file format
	CreateLevelMesh(scene, "flatPlane.ngd", 1);
	//CreateLevelMesh (scene, "playground.ngd", 1);

	//	dVector location (8.0f, 0.0f, -10.0f, 0.0f) ;
	dVector location(0.0f, 5.0f, 0.0f, 0.0f);

	SimpleSoftBodyEntity* const entity = new SimpleSoftBodyEntity(scene, location);
	//entity->BuildClothPatch(scene, 50, 50);
	entity->BuildClothPatch(scene, 1, 1);

	dQuaternion rot;
	dVector origin(location.m_x - 10.0f, 2.0f, location.m_z, 0.0f);
	scene->SetCameraMatrix(rot, origin);
}



