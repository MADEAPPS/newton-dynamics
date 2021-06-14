/* Copyright (c) <2003-2021> <Newton Game Dynamics>
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
#include "ndDemoMesh.h"
#include "ndLoadFbxMesh.h"
#include "ndPhysicsWorld.h"
#include "ndMakeStaticMap.h"
#include "ndDemoEntityManager.h"
#include "ndDemoSplinePathMesh.h"

ndBodyKinematic* BuildFloorBox(ndDemoEntityManager* const scene)
{
	ndPhysicsWorld* const world = scene->GetWorld();

	ndShapeInstance box(new ndShapeBox(200.0f, 1.0f, 200.f));
	dMatrix uvMatrix(dGetIdentityMatrix());
	uvMatrix[0][0] *= 0.025f;
	uvMatrix[1][1] *= 0.025f;
	uvMatrix[2][2] *= 0.025f;
	ndDemoMesh* const geometry = new ndDemoMesh("box", scene->GetShaderCache(), &box, "marbleCheckBoard.tga", "marbleCheckBoard.tga", "marbleCheckBoard.tga", 1.0f, uvMatrix);

	dMatrix matrix(dGetIdentityMatrix());
	matrix.m_posit.m_y = -0.5f;
	ndDemoEntity* const entity = new ndDemoEntity(matrix, nullptr);
	entity->SetMesh(geometry, dGetIdentityMatrix());

	ndBodyDynamic* const body = new ndBodyDynamic();
	body->SetNotifyCallback(new ndDemoEntityNotify(scene, entity));
	body->SetMatrix(matrix);
	body->SetCollisionShape(box);

	world->AddBody(body);

	scene->AddEntity(entity);
	geometry->Release();
	return body;
}

void BuildFlatPlane1(ndDemoEntityManager* const, bool)
{
	ndMeshEffect meshEffect;

	dVector floor[] =
	{
		{ 100.0f, 0.0f,  100.0f, 1.0f },
		{ 100.0f, 0.0f, -100.0f, 1.0f },
		{ -100.0f, 0.0f, -100.0f, 1.0f },
		{ -100.0f, 0.0f,  100.0f, 1.0f },
	};
	dInt32 index[][3] = { { 0, 1, 2 },{ 0, 2, 3 } };

	meshEffect.BeginBuild();
	for (dInt32 i = 0; i < 2; i++)
	{
		meshEffect.BeginBuildFace();
		for (dInt32 j = 0; j < 3; j++)
		{
			dFloat64 x = floor[index[i][j]].m_x;
			dFloat64 y = floor[index[i][j]].m_y;
			dFloat64 z = floor[index[i][j]].m_z;
			meshEffect.AddPoint(x, y, z);

			// you can add these other face attributes
			//AddLayer(dInt32 layer);
			//AddMaterial(dInt32 materialIndex);
			//AddNormal(dFloat32 x, dFloat32 y, dFloat32 z);
			//AddBinormal(dFloat32 x, dFloat32 y, dFloat32 z);
			//AddVertexColor(dFloat32 x, dFloat32 y, dFloat32 z, dFloat32 w);
			//AddUV0(dFloat32 u, dFloat32 v);
			//AddUV1(dFloat32 u, dFloat32 v);
		}
		meshEffect.EndBuildFace();
	}
	meshEffect.EndBuild(0.0f);
}

ndBodyKinematic* BuildFlatPlane(ndDemoEntityManager* const scene, bool optimized)
{
	ndPhysicsWorld* const world = scene->GetWorld();
	dVector floor[] =
	{
		{ 100.0f, 0.0f,  100.0f, 1.0f },
		{ 100.0f, 0.0f, -100.0f, 1.0f },
		{ -100.0f, 0.0f, -100.0f, 1.0f },
		{ -100.0f, 30.0f,  100.0f, 1.0f },
	};
	dInt32 index[][3] = { { 0, 1, 2 },{ 0, 2, 3 } };

	dPolygonSoupBuilder meshBuilder;
	meshBuilder.Begin();
	meshBuilder.AddFaceIndirect(&floor[0].m_x, sizeof(dVector), 31, &index[0][0], 3);
	meshBuilder.AddFaceIndirect(&floor[0].m_x, sizeof(dVector), 31, &index[1][0], 3);
	meshBuilder.End(optimized);

	ndShapeInstance box(new ndShapeStaticBVH(meshBuilder));
	dMatrix uvMatrix(dGetIdentityMatrix());
	uvMatrix[0][0] *= 0.025f;
	uvMatrix[1][1] *= 0.025f;
	uvMatrix[2][2] *= 0.025f;
	ndDemoMesh* const geometry = new ndDemoMesh("box", scene->GetShaderCache(), &box, "marbleCheckBoard.tga", "marbleCheckBoard.tga", "marbleCheckBoard.tga", 1.0f, uvMatrix);

	dMatrix matrix(dGetIdentityMatrix());
	ndDemoEntity* const entity = new ndDemoEntity(matrix, nullptr);
	entity->SetMesh(geometry, dGetIdentityMatrix());

	ndBodyDynamic* const body = new ndBodyDynamic();
	body->SetNotifyCallback(new ndDemoEntityNotify(scene, entity));
	body->SetMatrix(matrix);
	body->SetCollisionShape(box);

	world->AddBody(body);

	scene->AddEntity(entity);
	geometry->Release();
	return body;
}

ndBodyKinematic* BuildStaticMesh(ndDemoEntityManager* const scene, const char* const meshName, bool optimized)
{
	fbxDemoEntity* const entity = LoadFbxMesh(meshName);
	entity->BuildRenderMeshes(scene);
	scene->AddEntity(entity);

	dPolygonSoupBuilder meshBuilder;
	meshBuilder.Begin();
	
	dInt32 stack = 1;
	fbxDemoEntity* entBuffer[1024];
	dMatrix matrixBuffer[1024];
	entBuffer[0] = entity;
	matrixBuffer[0] = entity->GetCurrentMatrix().Inverse();

	while (stack)
	{
		stack--;
		fbxDemoEntity* const ent = entBuffer[stack];
		dMatrix matrix (ent->GetCurrentMatrix() * matrixBuffer[stack]);
	
		if (ent->m_fbxMeshEffect)
		{
			//dInt32 vertexCount = ent->m_fbxMeshEffect->GetVertexCount();
			dInt32 vertexStride = ent->m_fbxMeshEffect->GetVertexStrideInByte() / sizeof (dFloat64);
			const dFloat64* const vertexData = ent->m_fbxMeshEffect->GetVertexPool();
	
			dInt32 mark = ent->m_fbxMeshEffect->IncLRU();
			dPolyhedra::Iterator iter(*ent->m_fbxMeshEffect);
			
			dVector face[256];
			dMatrix worldMatrix(ent->GetMeshMatrix() * matrix);
			for (iter.Begin(); iter; iter++)
			{
				dEdge* const edge = &(*iter);
				if ((edge->m_incidentFace >= 0) && (edge->m_mark != mark))
				{
					dInt32 count = 0;
					dEdge* ptr = edge;
					do
					{
						dInt32 i = ptr->m_incidentVertex * vertexStride;
						dVector point(dFloat32(vertexData[i + 0]), dFloat32(vertexData[i + 1]), dFloat32(vertexData[i + 2]), dFloat32(1.0f));
						face[count] = worldMatrix.TransformVector(point);
						count++;
						ptr->m_mark = mark;
						ptr = ptr->m_next;
					} while (ptr != edge);
	
					dInt32 materialIndex = ent->m_fbxMeshEffect->GetFaceMaterial(edge);
					meshBuilder.AddFace(&face[0].m_x, sizeof(dVector), 3, materialIndex);
				}
			}
		}
	
		for (fbxDemoEntity* child = (fbxDemoEntity*)ent->GetChild(); child; child = (fbxDemoEntity*)child->GetSibling())
		{
			entBuffer[stack] = child;
			matrixBuffer[stack] = matrix;
			stack++;
		}
	}
	meshBuilder.End(optimized);
	ndShapeInstance shape(new ndShapeStaticBVH(meshBuilder));

	dMatrix matrix(entity->GetCurrentMatrix());
	ndBodyDynamic* const body = new ndBodyDynamic();
	body->SetNotifyCallback(new ndDemoEntityNotify(scene, entity));
	body->SetMatrix(matrix);
	body->SetCollisionShape(shape);
	scene->GetWorld()->AddBody(body);

	entity->CleanIntermediate();
	return body;
}

ndBodyKinematic* BuildSplineTrack(ndDemoEntityManager* const scene, const char* const meshName, bool optimized)
{
	dFloat64 knots[] = { 0.0f, 1.0f / 3.0f, 2.0f / 3.0f, 1.0f };
	dBigVector control[] =
	{
		dBigVector(-16.0f, 1.0f, -10.0f, 1.0f),
		dBigVector(-36.0f, 1.0f,   4.0f, 1.0f),
		dBigVector(  4.0f, 1.0f,  15.0f, 1.0f),
		dBigVector( 44.0f, 1.0f,   4.0f, 1.0f),
		dBigVector(  4.0f, 1.0f, -22.0f, 1.0f),
		dBigVector(-16.0f, 1.0f, -10.0f, 1.0f),
	};

	dBezierSpline spline;
	spline.CreateFromKnotVectorAndControlPoints(3, sizeof(knots) / sizeof(knots[0]), knots, control);

	// recreate the spline from sample points at equally spaced distance 
	//ndPhysicsWorld* const world = scene->GetWorld();

	dMatrix matrix(dGetIdentityMatrix());
	ndDemoSplinePathMesh* const splineMesh = new ndDemoSplinePathMesh(spline, scene->GetShaderCache(), 500);
	splineMesh->SetColor(dVector(1.0f, 0.0f, 0.0f, 1.0f));
	ndDemoEntity* const splineEntity = new ndDemoEntity(matrix, nullptr);
	scene->AddEntity(splineEntity);
	splineEntity->SetMesh(splineMesh, dGetIdentityMatrix());
	splineMesh->SetVisible(true);
	splineMesh->Release();

	{
		// try different degrees
		//dBigVector points[4];
		//points[0] = dBigVector(-16.0f, 1.0f, -10.0f, 1.0f);
		//points[1] = dBigVector(-36.0f, 1.0f, 4.0f, 1.0f);
		//points[2] = dBigVector(4.0f, 1.0f, 15.0f, 1.0f);
		//points[3] = dBigVector(44.0f, 1.0f, 4.0f, 1.0f);
		dInt32 size = sizeof(control) / sizeof(control[0]);

		dBigVector derivP0(control[1] - control[size-1]);
		dBigVector derivP1(control[0] - control[size - 2]);

		dBezierSpline spline1;
		spline1.GlobalCubicInterpolation(size, control, derivP0, derivP1);

		//dFloat64 u = (knots[1] + knots[2]) * 0.5f;
		//spline.InsertKnot(u);
		//spline.InsertKnot(u);
		//spline.InsertKnot(u);
		//spline.InsertKnot(u);
		//spline.InsertKnot(u);
		//spline.RemoveKnot(u, 1.0e-3f);
		ndDemoSplinePathMesh* const splineMesh1 = new ndDemoSplinePathMesh(spline1, scene->GetShaderCache(), 500);
		splineMesh1->SetColor(dVector(0.0f, 1.0f, 0.0f, 1.0f));
		ndDemoEntity* const splineEntity1 = new ndDemoEntity(matrix, nullptr);
		scene->AddEntity(splineEntity1);
		splineEntity1->SetMesh(splineMesh1, dGetIdentityMatrix());
		splineMesh1->SetVisible(true);
		splineMesh1->Release();
	}


	return BuildStaticMesh(scene, meshName, optimized);
}
