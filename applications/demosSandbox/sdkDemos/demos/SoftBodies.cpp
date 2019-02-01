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


class SimpleSoftBodyEntity: public DemoEntity
{
	public:
	class ClothPatchMesh: public DemoMesh
	{
		public:
		ClothPatchMesh(DemoEntityManager* const scene, NewtonMesh* const clothPatchMesh, NewtonBody* const body)
			:DemoMesh(clothPatchMesh, scene->GetShaderCache())
			,m_body(body)
		{
			ResetOptimization();
			//NewtonCollision* const deformableCollision = NewtonBodyGetCollision(m_body);

			int pointCount = NewtonMeshGetPointCount(clothPatchMesh);
			const int* const indexMap = NewtonMeshGetIndexToVertexMap(clothPatchMesh);
			m_indexMap = new int[pointCount];
			for (int i = 0; i < pointCount; i++) {
				int j = indexMap[i];
				m_indexMap[i] = j;
			}
		}

		~ClothPatchMesh()
		{
			if (m_indexMap) {
				delete[] m_indexMap;
			}
		}

		void Render(DemoEntityManager* const scene)
		{
			NewtonCollision* const deformableCollision = NewtonBodyGetCollision(m_body);
			dAssert((NewtonCollisionGetType(deformableCollision) == SERIALIZE_ID_CLOTH_PATCH) || (NewtonCollisionGetType(deformableCollision) == SERIALIZE_ID_DEFORMABLE_SOLID));

			const dFloat* const particles = NewtonDeformableMeshGetParticleArray(deformableCollision);
			int stride = NewtonDeformableMeshGetParticleStrideInBytes(deformableCollision) / sizeof (dFloat);

			// calculate vertex skinning
			for (int i = 0; i < m_vertexCount; i++) {
				int index = m_indexMap[i] * stride;
				m_vertex[i * 3 + 0] = particles[index + 0];
				m_vertex[i * 3 + 1] = particles[index + 1];
				m_vertex[i * 3 + 2] = particles[index + 2];

				// clear the normal for next loop
				m_normal[i * 3 + 0] = 0.0f;
				m_normal[i * 3 + 1] = 0.0f;
				m_normal[i * 3 + 2] = 0.0f;
			}

			// calculate vertex normals 
			int normalStride = 3;
			for (DemoMesh::dListNode* segmentNode = GetFirst(); segmentNode; segmentNode = segmentNode->GetNext()) {
				const DemoSubMesh& subSegment = segmentNode->GetInfo();
				for (int i = 0; i < subSegment.m_indexCount; i += 3) {
					int i0 = subSegment.m_indexes[i + 0] * normalStride;
					int i1 = subSegment.m_indexes[i + 1] * normalStride;
					int i2 = subSegment.m_indexes[i + 2] * normalStride;
					dVector p0(m_vertex[i0], m_vertex[i0 + 1], m_vertex[i0 + 2], 0.0f);
					dVector p1(m_vertex[i1], m_vertex[i1 + 1], m_vertex[i1 + 2], 0.0f);
					dVector p2(m_vertex[i2], m_vertex[i2 + 1], m_vertex[i2 + 2], 0.0f);
					dVector p10(p1 - p0);
					dVector p20(p2 - p0);
					dVector normal(p10.CrossProduct(p20));
					normal = normal.Scale(1.0f / dSqrt(normal.DotProduct3(normal)));

					m_normal[i0 + 0] += normal.m_x;
					m_normal[i0 + 1] += normal.m_y;
					m_normal[i0 + 2] += normal.m_z;

					m_normal[i1 + 0] += normal.m_x;
					m_normal[i1 + 1] += normal.m_y;
					m_normal[i1 + 2] += normal.m_z;

					m_normal[i2 + 0] += normal.m_x;
					m_normal[i2 + 1] += normal.m_y;
					m_normal[i2 + 2] += normal.m_z;
				}
			}

			// normalize all the normals
			for (int i = 0; i < m_vertexCount; i++) {
				dVector n(m_normal[i * 3 + 0], m_normal[i * 3 + 1], m_normal[i * 3 + 2], 0.0f);
				n = n.Scale(1.0f / dSqrt(n.DotProduct3(n)));
				m_normal[i * 3 + 0] = n.m_x;
				m_normal[i * 3 + 1] = n.m_y;
				m_normal[i * 3 + 2] = n.m_z;
			}

			glDisable(GL_CULL_FACE);
			DemoMesh::Render(scene);
			glEnable(GL_CULL_FACE);
		}

		NewtonBody* m_body;
		int* m_indexMap;
	};


	class TetrahedraSoftMesh: public DemoMesh
	{
		public:
		TetrahedraSoftMesh (DemoEntityManager* const scene, NewtonMesh* const tetrahedraMesh, NewtonBody* const body)
			:DemoMesh(tetrahedraMesh, scene->GetShaderCache())
			,m_body (body)
		{
			ResetOptimization();
			
dAssert (0);
/*
			NewtonCollision* const deformableCollision = NewtonBodyGetCollision(m_body);
			int pointCount = NewtonMeshGetPointCount(tetrahedraMesh);
			const int* const indexMap = NewtonMeshGetIndexToVertexMap(tetrahedraMesh);
			const int* const solidIndexList = NewtonDeformableMeshGetIndexToVertexMap(deformableCollision);

			m_indexMap = new int[pointCount];
			for (int i = 0; i < pointCount; i++) {
				int j = indexMap[i];
				m_indexMap[i] = solidIndexList[j];
			}
*/
		}

		~TetrahedraSoftMesh()
		{
			if (m_indexMap) {
				delete[] m_indexMap;
			}
		}

		void Render(DemoEntityManager* const scene)
		{
			NewtonCollision* const deformableCollision = NewtonBodyGetCollision(m_body);
			dAssert((NewtonCollisionGetType(deformableCollision) == SERIALIZE_ID_CLOTH_PATCH) || (NewtonCollisionGetType(deformableCollision) == SERIALIZE_ID_DEFORMABLE_SOLID));

			const dFloat* const particles = NewtonDeformableMeshGetParticleArray(deformableCollision);
			int stride = NewtonDeformableMeshGetParticleStrideInBytes(deformableCollision) / sizeof (dFloat);

			// calculate vertex skinning
			for (int i = 0; i < m_vertexCount; i++) {
				int index = m_indexMap[i] * stride;
				m_vertex[i * 3 + 0] = particles[index + 0];
				m_vertex[i * 3 + 1] = particles[index + 1];
				m_vertex[i * 3 + 2] = particles[index + 2];
			}

			DemoMesh::Render(scene);
		}

		NewtonBody* m_body;
		int* m_indexMap;
	};

	class LinearBlendMeshTetra: public DemoMesh
	{
		public:
		class WeightIndexPair
		{
			public:
			int m_index[4];
			dFloat m_weight[4];
		};


		LinearBlendMeshTetra (DemoEntityManager* const scene, NewtonMesh* const skinMesh, NewtonBody* const body)
			:DemoMesh(skinMesh, scene->GetShaderCache())
			,m_body (body)
			,m_weightSet(NULL)
		{
			ResetOptimization();
			dAssert (0);
/*
			NewtonCollision* const deformableCollision = NewtonBodyGetCollision(m_body);

			int skinPointCount = NewtonMeshGetPointCount(skinMesh);
			const int* const deformableIndexMap = NewtonDeformableMeshGetIndexToVertexMap(deformableCollision);

			int weightIndex[16];
			dFloat weightValue[16];
			m_weightSet = new WeightIndexPair[skinPointCount];
			for (int i = 0; i < skinPointCount; i++) {
				int weightCount = NewtonMeshGetVertexWeights(skinMesh, i, weightIndex, weightValue);
				for (int k = 0; k < weightCount; k ++) {
					m_weightSet[i].m_index[k] = deformableIndexMap[weightIndex[k]];
					m_weightSet[i].m_weight[k] = weightValue[k];
				}
			}
*/
		}

		~LinearBlendMeshTetra()
		{
			if (m_weightSet) {
				delete[] m_weightSet;
			}
		}

		void Render (DemoEntityManager* const scene)
		{
			NewtonCollision* const deformableCollision = NewtonBodyGetCollision(m_body);
			dAssert((NewtonCollisionGetType(deformableCollision) == SERIALIZE_ID_CLOTH_PATCH) || (NewtonCollisionGetType(deformableCollision) == SERIALIZE_ID_DEFORMABLE_SOLID));

			const dFloat* const particles = NewtonDeformableMeshGetParticleArray(deformableCollision);
			int stride = NewtonDeformableMeshGetParticleStrideInBytes(deformableCollision) / sizeof (dFloat);

			// calculate vertex skinning
			for (int i = 0; i < m_vertexCount; i++) {
				dVector p (0.0f);
				const WeightIndexPair& weightSet = m_weightSet[i];
				for (int j = 0; j < 4; j ++) {
					int index = weightSet.m_index[j] * stride;
					p.m_x += particles[index + 0] * weightSet.m_weight[j];
					p.m_y += particles[index + 1] * weightSet.m_weight[j];
					p.m_z += particles[index + 2] * weightSet.m_weight[j];
				}
				m_vertex[i * 3 + 0] = p.m_x;
				m_vertex[i * 3 + 1] = p.m_y;
				m_vertex[i * 3 + 2] = p.m_z;

				// clear the normal for next loop
				m_normal[i * 3 + 0] = 0.0f;
				m_normal[i * 3 + 1] = 0.0f;
				m_normal[i * 3 + 2] = 0.0f;
			}

			// calculate vertex normals 
			int normalStride = 3;
			for (DemoMesh::dListNode* segmentNode = GetFirst(); segmentNode; segmentNode = segmentNode->GetNext()) {
				const DemoSubMesh& subSegment = segmentNode->GetInfo();
				for (int i = 0; i < subSegment.m_indexCount; i += 3) {
					int i0 = subSegment.m_indexes[i + 0] * normalStride;
					int i1 = subSegment.m_indexes[i + 1] * normalStride;
					int i2 = subSegment.m_indexes[i + 2] * normalStride;
					dVector p0(m_vertex[i0], m_vertex[i0 + 1], m_vertex[i0 + 2], 0.0f);
					dVector p1(m_vertex[i1], m_vertex[i1 + 1], m_vertex[i1 + 2], 0.0f);
					dVector p2(m_vertex[i2], m_vertex[i2 + 1], m_vertex[i2 + 2], 0.0f);
					dVector p10(p1 - p0);
					dVector p20(p2 - p0);
					dVector normal(p10.CrossProduct(p20));
					normal = normal.Scale(1.0f / dSqrt(normal.DotProduct3(normal)));

					m_normal[i0 + 0] += normal.m_x;
					m_normal[i0 + 1] += normal.m_y;
					m_normal[i0 + 2] += normal.m_z;

					m_normal[i1 + 0] += normal.m_x;
					m_normal[i1 + 1] += normal.m_y;
					m_normal[i1 + 2] += normal.m_z;

					m_normal[i2 + 0] += normal.m_x;
					m_normal[i2 + 1] += normal.m_y;
					m_normal[i2 + 2] += normal.m_z;
				}
			}

			// normalize all the normals
			for (int i = 0; i < m_vertexCount; i++) {
				dVector n(m_normal[i * 3 + 0], m_normal[i * 3 + 1], m_normal[i * 3 + 2], 0.0f);
				n = n.Scale (1.0f / dSqrt(n.DotProduct3(n)));
				m_normal[i * 3 + 0] = n.m_x;
				m_normal[i * 3 + 1] = n.m_y;
				m_normal[i * 3 + 2] = n.m_z;
			}


			DemoMesh::Render(scene);
		}

		NewtonBody* m_body;
		//int* m_indexMap;
		WeightIndexPair* m_weightSet;
	};


	SimpleSoftBodyEntity(DemoEntityManager* const scene, const dVector& location)
		:DemoEntity(dGetIdentityMatrix(), NULL)
		,m_body(NULL)
	{
		dMatrix matrix (dGetIdentityMatrix());

		matrix.m_posit.m_x = location.m_x;
		matrix.m_posit.m_y = location.m_y;
		matrix.m_posit.m_z = location.m_z;
		ResetMatrix(*scene, matrix);

		// add an new entity to the world
		scene->Append(this);
	}

	~SimpleSoftBodyEntity()
	{
	}


	NewtonBody* CreateRigidBody(DemoEntityManager* const scene, dFloat mass, NewtonCollision* const deformableCollision)
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

		return deformableBody;
	}

	NewtonMesh* CreateQuadClothPatch(DemoEntityManager* const scene, int size_x, int size_z)
	{
		size_x += 1;
		size_z += 1;
		dAssert(size_x <= 129);
		dAssert(size_z <= 129);
		dFloat dimension = 0.125f;

		dBigVector* const points = new dBigVector[size_x * size_z];
		int* const faceIndexCount = new int[(size_x - 1) * (size_z - 1)];
		int* const faceVertexIndex = new int[4 * (size_x - 1) * (size_z - 1)];

		dFloat y = 0.0f;
		int vertexCount = 0;
		for (int i = 0; i < size_z; i++) {
			dFloat z = (i - size_z / 2) * dimension;
			for (int j = 0; j < size_x; j++) {
				dFloat x = (j - size_x / 2) * dimension;
				points[vertexCount] = dVector(x, y, z, 0.0f);
				vertexCount++;
			}
		}
		
		int faceCount = 0;
		for (int i = 0; i < size_z - 1; i++) {
			for (int j = 0; j < size_x - 1; j++) {
				faceIndexCount[faceCount] = 4;
				faceVertexIndex[faceCount * 4 + 0] = (i + 0) * size_x + j + 0;
				faceVertexIndex[faceCount * 4 + 1] = (i + 0) * size_x + j + 1;
				faceVertexIndex[faceCount * 4 + 2] = (i + 1) * size_x + j + 1;
				faceVertexIndex[faceCount * 4 + 3] = (i + 1) * size_x + j + 0;
				faceCount++;
			}
		}

		dMatrix aligmentUV(dGetIdentityMatrix());
		NewtonMeshVertexFormat vertexFormat;
		NewtonMeshClearVertexFormat(&vertexFormat);

		vertexFormat.m_faceCount = faceCount;
		vertexFormat.m_faceIndexCount = faceIndexCount;

		vertexFormat.m_vertex.m_data = &points[0][0];
		vertexFormat.m_vertex.m_indexList = faceVertexIndex;
		vertexFormat.m_vertex.m_strideInBytes = sizeof(dBigVector);

		NewtonMesh* const clothPatch = NewtonMeshCreate(scene->GetNewton());
		NewtonMeshBuildFromVertexListIndexList(clothPatch, &vertexFormat);

		int material = LoadTexture("persianRug.tga");
		NewtonMeshApplyBoxMapping(clothPatch, material, material, material, &aligmentUV[0][0]);

		delete[] points;
		delete[] faceIndexCount;
		delete[] faceVertexIndex;
		return clothPatch;
	}


	void AddTetra (NewtonMesh* const tetrahedra, int i0, int i1, int i2, int i3, const dVector* const tetra, int layer)
	{
		NewtonMeshBeginFace(tetrahedra);
			NewtonMeshAddPoint(tetrahedra, tetra[i0].m_x, tetra[i0].m_y, tetra[i0].m_z);
			NewtonMeshAddLayer(tetrahedra, layer);
			NewtonMeshAddPoint(tetrahedra, tetra[i1].m_x, tetra[i1].m_y, tetra[i1].m_z);
			NewtonMeshAddLayer(tetrahedra, layer);
			NewtonMeshAddPoint(tetrahedra, tetra[i2].m_x, tetra[i2].m_y, tetra[i2].m_z);
			NewtonMeshAddLayer(tetrahedra, layer);
		NewtonMeshEndFace(tetrahedra);

		NewtonMeshBeginFace(tetrahedra);
			NewtonMeshAddPoint(tetrahedra, tetra[i3].m_x, tetra[i3].m_y, tetra[i3].m_z);
			NewtonMeshAddLayer(tetrahedra, layer);
			NewtonMeshAddPoint(tetrahedra, tetra[i0].m_x, tetra[i0].m_y, tetra[i0].m_z);
			NewtonMeshAddLayer(tetrahedra, layer);
			NewtonMeshAddPoint(tetrahedra, tetra[i2].m_x, tetra[i2].m_y, tetra[i2].m_z);
			NewtonMeshAddLayer(tetrahedra, layer);
		NewtonMeshEndFace(tetrahedra);

		NewtonMeshBeginFace(tetrahedra);
			NewtonMeshAddPoint(tetrahedra, tetra[i3].m_x, tetra[i3].m_y, tetra[i3].m_z);
			NewtonMeshAddLayer(tetrahedra, layer);
			NewtonMeshAddPoint(tetrahedra, tetra[i1].m_x, tetra[i1].m_y, tetra[i1].m_z);
			NewtonMeshAddLayer(tetrahedra, layer);
			NewtonMeshAddPoint(tetrahedra, tetra[i0].m_x, tetra[i0].m_y, tetra[i0].m_z);
			NewtonMeshAddLayer(tetrahedra, layer);
		NewtonMeshEndFace(tetrahedra);

		NewtonMeshBeginFace(tetrahedra);
			NewtonMeshAddPoint(tetrahedra, tetra[i3].m_x, tetra[i3].m_y, tetra[i3].m_z);
			NewtonMeshAddLayer(tetrahedra, layer);
			NewtonMeshAddPoint(tetrahedra, tetra[i2].m_x, tetra[i2].m_y, tetra[i2].m_z);
			NewtonMeshAddLayer(tetrahedra, layer);
			NewtonMeshAddPoint(tetrahedra, tetra[i1].m_x, tetra[i1].m_y, tetra[i1].m_z);
			NewtonMeshAddLayer(tetrahedra, layer);
		NewtonMeshEndFace(tetrahedra);
	}

	void BuildTetraSolidBlock(NewtonMesh* const tetrahedra, int x, int y, int z, float width, float height, float depth)
	{
		dVector points[1024];

		//make the point array;
		int index = 0;
		for (int i = 0; i <= x; ++i) {
			for (int j = 0; j <= y; ++j) {
				for (int k = 0; k <= z; ++k) {
					points[index] = dVector(width*i, height*k, depth*j);
// temp hack until I fix the com bug
points[index] -= dVector(width * 0.5f, height * 0.5f, depth * 0.5f, 0.0f);
					index++;
					dAssert (index < sizeof (points) / sizeof (points[0]));
				}
			}
		}

		// create the simple for cube one a a time but interleave them so that the neighbors cubes shared faces.  
		int layer = 0;
		for (int i = 0; i < x; ++i) {
			for (int j = 0; j < y; ++j) {
				for (int k = 0; k < z; ++k) {
					int p0 = (i * (y + 1) + j) * (z + 1) + k;
					int p1 = p0 + 1;
					int p3 = ((i + 1) * (y + 1) + j) * (z + 1) + k;
					int p2 = p3 + 1;
					int p7 = ((i + 1) * (y + 1) + (j + 1)) * (z + 1) + k;
					int p6 = p7 + 1;
					int p4 = (i * (y + 1) + (j + 1)) * (z + 1) + k;
					int p5 = p4 + 1;

					if ((i + j + k) & 1) {
						AddTetra (tetrahedra, p1, p2, p3, p6, points, layer + 0);
						AddTetra (tetrahedra, p3, p6, p7, p4, points, layer + 1);
						AddTetra (tetrahedra, p1, p4, p5, p6, points, layer + 2);
						AddTetra (tetrahedra, p1, p3, p0, p4, points, layer + 3);
						AddTetra (tetrahedra, p1, p6, p3, p4, points, layer + 4);
					} else {
						AddTetra (tetrahedra, p2, p0, p1, p5, points, layer + 0);
						AddTetra (tetrahedra, p2, p7, p3, p0, points, layer + 1);
						AddTetra (tetrahedra, p2, p5, p6, p7, points, layer + 2);
						AddTetra (tetrahedra, p0, p7, p4, p5, points, layer + 3);
						AddTetra (tetrahedra, p2, p0, p5, p7, points, layer + 4);
					}
					layer += 5;
				}
			}
		}
	}

	void BuildRegularTetrahedra (DemoEntityManager* const scene, int materialID)
	{
		dFloat mass = 5.0f;
		NewtonWorld* const world = scene->GetNewton();

		dVector tetra[] = { dVector(-1.0f, 0.0f, -0.71f, 0.0f),
							dVector(1.0f, 0.0f, -0.71f, 0.0f),
							dVector(0.0f, -1.0f, 0.71f, 0.0f),
							dVector(0.0f, 1.0f, 0.71f, 0.0f) };

		NewtonMesh* const tetrahedra = NewtonMeshCreate(scene->GetNewton());
		NewtonMeshBeginBuild(tetrahedra);
			AddTetra (tetrahedra, 0, 1, 2, 3, tetra, 0);
		NewtonMeshEndBuild(tetrahedra);

		dMatrix aligmentUV(dGetIdentityMatrix());
		int material = LoadTexture("smilli.tga");
		NewtonMeshApplyBoxMapping (tetrahedra, material, material, material, &aligmentUV[0][0]);
		NewtonMeshCalculateVertexNormals (tetrahedra, 60.0f * dDegreeToRad);

		// make a deformable collision mesh
		NewtonCollision* const deformableCollision = NewtonCreateDeformableSolid(world, tetrahedra, materialID);

		//create a rigid body with a deformable mesh
		m_body = CreateRigidBody (scene, mass, deformableCollision);

		// create the soft body mesh
		DemoMesh* const mesh = new TetrahedraSoftMesh(scene, tetrahedra, m_body);
		SetMesh(mesh, dGetIdentityMatrix());

		// do not forget to destroy this objects, else you get bad memory leaks.
		mesh->Release ();
		NewtonMeshDestroy (tetrahedra);
		NewtonDestroyCollision(deformableCollision);
	}
	
	void BuildTetraHedraCube(DemoEntityManager* const scene, int materialID)
	{
		dFloat mass = 5.0f;
		NewtonWorld* const world = scene->GetNewton();

		NewtonMesh* const tetraCube = NewtonMeshCreate(scene->GetNewton());
		NewtonMeshBeginBuild(tetraCube);
			BuildTetraSolidBlock(tetraCube, 2, 2, 15, 0.5f, 0.5f, 0.5f);
		NewtonMeshEndBuild(tetraCube);

		dMatrix aligmentUV(dGetIdentityMatrix());
		int material = LoadTexture("smilli.tga");
		NewtonMeshApplyBoxMapping(tetraCube, material, material, material, &aligmentUV[0][0]);
		NewtonMeshCalculateVertexNormals(tetraCube, 60.0f * dDegreeToRad);

		// make a deformable collision mesh
		NewtonCollision* const deformableCollision = NewtonCreateDeformableSolid(world, tetraCube, materialID);

		//create a rigid body with a deformable mesh
		m_body = CreateRigidBody(scene, mass, deformableCollision);

		// create the soft body mesh
		DemoMesh* const mesh = new TetrahedraSoftMesh(scene, tetraCube, m_body);
		SetMesh(mesh, dGetIdentityMatrix());

		// do not forget to destroy this objects, else you get bad memory leaks.
		mesh->Release ();
		NewtonDestroyCollision(deformableCollision);
		NewtonMeshDestroy(tetraCube);
	}

	void LoadTetrahedraCube(DemoEntityManager* const scene, int materialID)
	{
		dFloat mass = 5.0f;
		NewtonWorld* const world = scene->GetNewton();

		char name[2048];
		dGetWorkingFileName ("box.tet", name);
		NewtonMesh* const tetraCube = NewtonMeshLoadTetrahedraMesh(scene->GetNewton(), name);

		dMatrix aligmentUV(dGetIdentityMatrix());
		int material = LoadTexture("smilli.tga");
		NewtonMeshApplyBoxMapping(tetraCube, material, material, material, &aligmentUV[0][0]);
		NewtonMeshCalculateVertexNormals(tetraCube, 60.0f * dDegreeToRad);

		// make a deformable collision mesh
		NewtonCollision* const deformableCollision = NewtonCreateDeformableSolid(world, tetraCube, materialID);

		//create a rigid body with a deformable mesh
		m_body = CreateRigidBody(scene, mass, deformableCollision);

		// create the soft body mesh
		//m_mesh = new TetrahedraSoftMesh(tetraCube, m_body);
		DemoMesh* const mesh = new TetrahedraSoftMesh(scene, tetraCube, m_body);
		SetMesh(mesh, dGetIdentityMatrix());

		// do not forget to destroy this objects, else you get bad memory leaks.
		mesh->Release ();
		NewtonDestroyCollision(deformableCollision);
		NewtonMeshDestroy(tetraCube);
	}

	void CreateTetrahedraPrimitive(DemoEntityManager* const scene, int materialID)
	{
		dFloat mass = 5.0f;
		dVector size (1.0f);

		NewtonWorld* const world = scene->GetNewton();
		NewtonCollision* const primitiveShape = CreateConvexCollision (world, dGetIdentityMatrix(), size, _SPHERE_PRIMITIVE, materialID);
		//NewtonCollision* const primitiveShape = CreateConvexCollision (world, dGetIdentityMatrix(), size, _BOX_PRIMITIVE, materialID);
		NewtonMesh* const skinMesh = NewtonMeshCreateFromCollision(primitiveShape);

		dMatrix aligmentUV(dGetIdentityMatrix());
		int material = LoadTexture("smilli.tga");
		NewtonMeshApplySphericalMapping(skinMesh, material, &aligmentUV[0][0]);

		// now now make an tetrahedra iso surface approximation of this mesh
		NewtonMesh* const tetraIsoSurface = NewtonMeshCreateTetrahedraIsoSurface(skinMesh);

		// calculate the linear blend weight for the tetrahedra mesh
		NewtonCreateTetrahedraLinearBlendSkinWeightsChannel (tetraIsoSurface, skinMesh);
		NewtonDestroyCollision(primitiveShape);

		// make a deformable collision mesh
		NewtonCollision* const deformableCollision = NewtonCreateDeformableSolid(world, tetraIsoSurface, materialID);

		//create a rigid body with a deformable mesh
		m_body = CreateRigidBody(scene, mass, deformableCollision);

		// create the soft body mesh
		//DemoMesh* const mesh = new TetrahedraSoftMesh(tetraIsoSurface, m_body);
		DemoMesh* const mesh = new LinearBlendMeshTetra(scene, skinMesh, m_body);
		SetMesh(mesh, dGetIdentityMatrix());
		
		// do not forget to destroy this objects, else you get bad memory leaks.
		mesh->Release ();
		NewtonMeshDestroy(skinMesh);
		NewtonDestroyCollision(deformableCollision);
		NewtonMeshDestroy(tetraIsoSurface);
	}

	void BuildClothPatch (DemoEntityManager* const scene, int size_x, int size_z)
	{
		NewtonWorld* const world = scene->GetNewton();
		
		NewtonMesh* const clothPatch = CreateQuadClothPatch(scene, size_x, size_z);

		// create the array of points;
		int vertexCount = NewtonMeshGetVertexCount(clothPatch);
		int stride = NewtonMeshGetVertexStrideInByte (clothPatch) / sizeof (dFloat64); 
		const dFloat64* const meshPoints = NewtonMeshGetVertexArray (clothPatch); 

		dVector* const points = new dVector[vertexCount];
		for (int i =0; i < vertexCount; i ++ ) {
			points[i].m_x = dFloat (meshPoints[i * stride + 0]);
			points[i].m_y = dFloat (meshPoints[i * stride + 1]);
			points[i].m_z = dFloat (meshPoints[i * stride + 2]);
			points[i].m_w = 0.0f;
		}

		dFloat mass = 8.0f;
		// set the particle masses 
		dFloat unitMass = mass / vertexCount;
		dFloat* const clothMass = new dFloat[vertexCount];
		for (int i =0; i < vertexCount; i ++ ) {			
			clothMass[i] = unitMass;
		}

		int linksCount = 0;
		const int maxLinkCount = size_x * size_z * 16;

		// create the structual constation array;
		dFloat structuralSpring = dAbs(mass * DEMO_GRAVITY) / 0.01f;
		dFloat structuralDamper = 30.0f;

		int* const links = new int[2 * maxLinkCount];
		dFloat* const spring = new dFloat[maxLinkCount];
		dFloat* const damper = new dFloat[maxLinkCount];
		for (void* edgeNode = NewtonMeshGetFirstEdge (clothPatch); edgeNode; edgeNode = NewtonMeshGetNextEdge (clothPatch, edgeNode)) {
			int v0;
			int v1;
			NewtonMeshGetEdgeIndices (clothPatch, edgeNode, &v0, &v1);
			links[linksCount * 2 + 0] = v0;
			links[linksCount * 2 + 1] = v1;
			spring[linksCount] = structuralSpring;
			damper[linksCount] = structuralDamper;
			linksCount ++;
			dAssert (linksCount <= maxLinkCount);
		}
		

		// add shear constraints
		dFloat shearSpring = structuralSpring;
		dFloat shearDamper = structuralDamper;
		for (void* faceNode = NewtonMeshGetFirstFace (clothPatch); faceNode; faceNode = NewtonMeshGetNextFace (clothPatch, faceNode)) {
			if (!NewtonMeshIsFaceOpen(clothPatch, faceNode)) {
				int face[8];
				int indexCount = NewtonMeshGetFaceIndexCount (clothPatch, faceNode);
				NewtonMeshGetFaceIndices (clothPatch, faceNode, face);
				for (int i = 2; i < indexCount - 1; i ++) {
					links[linksCount * 2 + 0] = face[0];
					links[linksCount * 2 + 1] = face[i];
					spring[linksCount] = shearSpring;
					damper[linksCount] = shearDamper;

					linksCount ++;
					dAssert (linksCount <= maxLinkCount);
				}
				for (int i = 3; i < indexCount; i ++) {
					links[linksCount * 2 + 0] = face[1];
					links[linksCount * 2 + 1] = face[i];
					spring[linksCount] = shearSpring;
					damper[linksCount] = shearDamper;
					linksCount ++;
					dAssert (linksCount <= maxLinkCount);
				}
			}
		}

//linksCount = 0;
		NewtonCollision* const deformableCollision = NewtonCreateMassSpringDamperSystem(world, 0, 
													 &points[0].m_x, vertexCount, sizeof (dVector), clothMass,
													 links, linksCount, spring, damper);
		
		m_body = CreateRigidBody(scene, mass, deformableCollision);

		DemoMesh* const mesh = new ClothPatchMesh (scene, clothPatch, m_body);
		SetMesh(mesh, dGetIdentityMatrix());

		// do not forget to destroy this objects, else you get bad memory leaks.
		mesh->Release();
		NewtonDestroyCollision(deformableCollision);
		NewtonMeshDestroy(clothPatch);
		delete[] links;
		delete[] damper;
		delete[] spring;
		delete[] clothMass;
		delete[] points;
	}
	
	NewtonBody* m_body;
};



void SoftBodies(DemoEntityManager* const scene)
{
	// load the sky box
	scene->CreateSkyBox();

	// load the scene from a ngd file format
	CreateLevelMesh(scene, "flatPlane.ngd", 1);
	//CreateLevelMesh (scene, "playground.ngd", 1);

	dVector location(0.0f, 6.0f, 0.0f, 0.0f);

	SimpleSoftBodyEntity* const entity = new SimpleSoftBodyEntity(scene, location);
	//entity->BuildRegularTetrahedra(scene, 0);
	//entity->LoadTetrahedraCube (scene, 0);
	entity->CreateTetrahedraPrimitive (scene, 0);

	dQuaternion rot;
	dVector origin(location.m_x - 10.0f, 2.0f, location.m_z, 0.0f);
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
	entity->BuildClothPatch(scene, 16, 16);

	dQuaternion rot;
	dVector origin(location.m_x - 10.0f, 2.0f, location.m_z, 0.0f);
	scene->SetCameraMatrix(rot, origin);
}
