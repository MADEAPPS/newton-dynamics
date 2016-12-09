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

		// now create a visual representation for this entity
		//	CreateVisualMesh ();
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

	void BuildClothPatch (DemoEntityManager* const scene, int size_x, int size_z)
	{
		NewtonWorld* const world = scene->GetNewton();

		size_x += 1;
		size_z += 1;
		dAssert (size_x < 128);
		dAssert (size_z < 128);
		dFloat dimension = 0.25f;
//		dVector points[128][128];
		dVector points[2][2];

		dFloat y = 0.0f;
		int enumerator = 0;
		for (int i = 0; i < size_z; i ++) {
			dFloat z = (i - size_z / 2) * dimension;
			for (int j = 0; j < size_x; j ++) {
				dFloat x = (j - size_x / 2) * dimension;
				points[i][j] = dVector (x, y, z, dFloat (enumerator));
				enumerator ++;
			}
		}

		NewtonMesh* const clothPatch = NewtonMeshCreate(world);
		NewtonMeshBeginBuild (clothPatch);
		for (int i = 0; i < size_z - 1; i ++) {
//			if (i & 1) {
			if (1) {
				for (int j = 0; j < size_x - 1; j ++) {
					NewtonMeshBeginFace (clothPatch);
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
				for (int j = 0; j < size_x; j ++) {
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
		NewtonMeshEndBuild (clothPatch);

		//int material = LoadTexture("persianRug.tga");
		//NewtonMeshApplyBoxMapping(mesh, material, material, material);

		NewtonCollision* const deformableCollision = NewtonCreateClothPatch(world, clothPatch, 0);

		dFloat mass = 8.0f;
		CreateRigidBody(scene, mass, deformableCollision);

		NewtonDestroyCollision(deformableCollision);
		NewtonMeshDestroy(clothPatch);

/*
		// now create a soft collision mesh
		NewtonClothPatchMaterial bendMaterial;
		NewtonClothPatchMaterial structuralMaterial;

		structuralMaterial.m_damper = 100.0f;
		structuralMaterial.m_stiffness = 5000.0f;

		bendMaterial.m_damper = 10.0f;
		bendMaterial.m_stiffness = 200.0f;

		NewtonCollision* const softCollisionMesh = NewtonCreateClothPatch (world, mesh, 0, &structuralMaterial, &bendMaterial);

		int particlesCount = NewtonDeformableMeshGetParticleCount (softCollisionMesh);

		int index0 = -1; 
		int index1 = -1; 
		int index2 = -1; 
		int index3 = -1; 
		dVector p0 (1.0e10f, 1.0e10f, 1.0e10f, 0.0f);
		dVector p1 (1.0e10f, 1.0e10f, -1.0e10f, 0.0f);
		dVector p2 (-1.0e10f, 1.0e10f, -1.0e10f, 0.0f);
		dVector p3 (-1.0e10f, 1.0e10f,  1.0e10f, 0.0f);
		for (int i = 0; i < particlesCount; i++ ) {
			dVector p(0.0f);
			NewtonDeformableMeshGetParticlePosition (softCollisionMesh, i, &p.m_x);
			if ((p.m_x < p0.m_x) || (p.m_z < p0.m_z)) {
				index0 = i;
				p0 = p;
			}

			NewtonDeformableMeshGetParticlePosition (softCollisionMesh, i, &p.m_x);
			if ((p.m_x < p1.m_x) || (p.m_z > p1.m_z)) {
				index1 = i;
				p1 = p;
			}

			NewtonDeformableMeshGetParticlePosition (softCollisionMesh, i, &p.m_x);
			if ((p.m_x > p2.m_x) || (p.m_z > p2.m_z)) {
				index2 = i;
				p2 = p;
			}

			NewtonDeformableMeshGetParticlePosition (softCollisionMesh, i, &p.m_x);
			if ((p.m_x > p3.m_x) || (p.m_z < p3.m_z)) {
				index3 = i;
				p3 = p;
			}
		}

		// constraint the four corner of this patch to the world
		NewtonDeformableMeshBeginConfiguration (softCollisionMesh);
//		NewtonDeformableMeshConstraintParticle (softCollisionMesh, index0, &p0.m_x, NULL);
//		NewtonDeformableMeshConstraintParticle (softCollisionMesh, index1, &p1.m_x, NULL);
		NewtonDeformableMeshConstraintParticle (softCollisionMesh, index2, &p2.m_x, NULL);
		NewtonDeformableMeshConstraintParticle (softCollisionMesh, index3, &p3.m_x, NULL);
		NewtonDeformableMeshEndConfiguration (softCollisionMesh);


		//NewtonDeformableMeshSetSkinThickness (softCollisionMesh, 1.0f);
		NewtonDeformableMeshSetSkinThickness (softCollisionMesh, 0.05f);

		// destroy the auxiliary objects
		NewtonMeshDestroy(mesh);
		return softCollisionMesh;
*/	
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

	void CreateVisualMesh ()
	{
/*
		DemoMesh* const mesh = (DemoMesh*)GetMesh();
		dAssert (mesh->IsType(DemoMesh::GetRttiType()));

		// now create a visual representation for this mesh
		int vertexCount = NewtonDeformableMeshGetVertexCount (m_softCollision); 
		mesh->AllocVertexData(vertexCount);

		NewtonDeformableMeshUpdateRenderNormals(m_softCollision);
		NewtonDeformableMeshGetVertexStreams (m_softCollision, 3 * sizeof (dFloat), (dFloat*) mesh->m_vertex, 3 * sizeof (dFloat), (dFloat*) mesh->m_normal, 2 * sizeof (dFloat), (dFloat*) mesh->m_uv);

		for (NewtonDeformableMeshSegment* segmentNode = NewtonDeformableMeshGetFirstSegment (m_softCollision); segmentNode; segmentNode = NewtonDeformableMeshGetNextSegment (m_softCollision, segmentNode)) {
			int materialID = NewtonDeformableMeshSegmentGetMaterialID(m_softCollision, segmentNode);
			int indexCount = NewtonDeformableMeshSegmentGetIndexCount(m_softCollision, segmentNode);
			const int* const indexList = NewtonDeformableMeshSegmentGetIndexList(m_softCollision, segmentNode);

			DemoSubMesh* const segment = mesh->AddSubMesh();
			const char* const textName = FindTextureById(materialID);
			dAssert (textName);
			if (textName) {
				segment->m_textureHandle = materialID;
				//strcpy (segment->m_textureName, textName);
				//segment->m_shiness = material->GetShininess();
				//segment->m_ambient = material->GetAmbientColor();
				//segment->m_diffuse = material->GetDiffuseColor();
				//segment->m_specular = material->GetSpecularColor();
			}
			segment->m_indexCount = indexCount;
			segment->AllocIndexData (indexCount);
			for (int i = 0; i < indexCount; i ++) {
				segment->m_indexes[i] = indexList[i];
			}
		}
*/	
	}


	virtual void Render(dFloat timeStep, DemoEntityManager* const scene) const
	{
/*
		DemoMesh* const mesh = (DemoMesh*)GetMesh();
		dAssert (mesh->IsType(DemoMesh::GetRttiType()));

		// regenerate the soft body mesh for rendering
		NewtonDeformableMeshUpdateRenderNormals(m_softCollision);

		// update the vertex array
		NewtonDeformableMeshGetVertexStreams (m_softCollision, 3 * sizeof (dFloat), (dFloat*) mesh->m_vertex, 3 * sizeof (dFloat), (dFloat*) mesh->m_normal, 2 * sizeof (dFloat), (dFloat*) mesh->m_uv);
*/
		// proceed with normal rendering
		DemoEntity::Render(timeStep, scene);
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
	dVector location(0.0f, 2.0f, 0.0f, 0.0f);



	SimpleSoftBodyEntity* const entity = new SimpleSoftBodyEntity(scene, location);
	entity->BuildClothPatch(scene, 1, 1);

	dQuaternion rot;
	dVector origin(location.m_x - 10.0f, 2.0f, location.m_z, 0.0f);
	scene->SetCameraMatrix(rot, origin);
}



