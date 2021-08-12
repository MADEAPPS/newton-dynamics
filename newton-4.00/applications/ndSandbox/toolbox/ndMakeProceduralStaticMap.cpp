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
#include "ndDemoEntity.h"
#include "ndLoadFbxMesh.h"
#include "ndPhysicsWorld.h"
#include "ndMakeStaticMap.h"
#include "ndDemoEntityManager.h"
#include "ndMakeProceduralStaticMap.h"

class ndRegularProceduralGrid : public ndShapeStaticProceduralMesh
{
	public:
	ndRegularProceduralGrid(dFloat32 sizex, dFloat32 sizey, dFloat32 sizez)
		:ndShapeStaticProceduralMesh(sizex, sizey, sizez)
	{
	}

	dFloat32 RayCast(ndRayCastNotify&, const dVector& localP0, const dVector& localP1, dFloat32 maxT, const ndBody* const, ndContactPoint& contactOut) const
	{
		dAssert(0);
		return 0;
	}
};

ndDemoEntity* BuildVisualEntiry(ndDemoEntityManager* const scene, dInt32 grids, dFloat32 gridSize, dFloat32 perturbation)
{
	dVector origin(-grids * gridSize * 0.5f, 0.0f, -grids * gridSize * 0.5f, 0.0f);

	dArray<dVector> points;
	for (dInt32 iz = 0; iz <= grids; iz++)
	{
		dFloat32 z0 = origin.m_z + iz * gridSize;
		for (dInt32 ix = 0; ix <= grids; ix++)
		{
			dFloat32 x0 = origin.m_x + ix * gridSize;
			points.PushBack(dVector(x0, dGaussianRandom(perturbation), z0, 1.0f));
		}
	}

	ndMeshEffect meshEffect;
	meshEffect.BeginBuild();
	for (dInt32 iz = 0; iz < grids; iz++)
	{
		for (dInt32 ix = 0; ix < grids; ix++)
		{
			dVector p0(points[(ix + 0) * (grids + 1) + iz + 0]);
			dVector p1(points[(ix + 1) * (grids + 1) + iz + 0]);
			dVector p2(points[(ix + 1) * (grids + 1) + iz + 1]);
			dVector p3(points[(ix + 0) * (grids + 1) + iz + 1]);

			meshEffect.BeginBuildFace();
			meshEffect.AddPoint(p0.m_x, p0.m_y, p0.m_z);
			meshEffect.AddPoint(p1.m_x, p1.m_y, p1.m_z);
			meshEffect.AddPoint(p2.m_x, p2.m_y, p2.m_z);
			meshEffect.EndBuildFace();

			meshEffect.BeginBuildFace();
			meshEffect.AddPoint(p0.m_x, p0.m_y, p0.m_z);
			meshEffect.AddPoint(p2.m_x, p2.m_y, p2.m_z);
			meshEffect.AddPoint(p3.m_x, p3.m_y, p3.m_z);
			meshEffect.EndBuildFace();
		}
	}
	meshEffect.EndBuild(0.0f);

	dPolygonSoupBuilder meshBuilder;

	meshBuilder.Begin();
	dInt32 vertexStride = meshEffect.GetVertexStrideInByte() / sizeof(dFloat64);
	const dFloat64* const vertexData = meshEffect.GetVertexPool();

	dInt32 mark = meshEffect.IncLRU();

	dVector face[16];
	dPolyhedra::Iterator iter(meshEffect);
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
				face[count] = point;
				count++;
				ptr->m_mark = mark;
				ptr = ptr->m_next;
			} while (ptr != edge);

			dInt32 materialIndex = meshEffect.GetFaceMaterial(edge);
			meshBuilder.AddFace(&face[0].m_x, sizeof(dVector), 3, materialIndex);
		}
	}
	meshBuilder.End(false);

	dFloat32 uvScale = 1.0 / 16.0f;

	ndShapeInstance plane(new ndShapeStatic_bvh(meshBuilder));
	dMatrix uvMatrix(dGetIdentityMatrix());
	uvMatrix[0][0] *= uvScale;
	uvMatrix[1][1] *= uvScale;
	uvMatrix[2][2] *= uvScale;
	ndDemoMesh* const geometry = new ndDemoMesh("plane", scene->GetShaderCache(), &plane, "marbleCheckBoard.tga", "marbleCheckBoard.tga", "marbleCheckBoard.tga", 1.0f, uvMatrix);

	dMatrix matrix(dGetIdentityMatrix());
	ndDemoEntity* const entity = new ndDemoEntity(matrix, nullptr);
	entity->SetMesh(geometry, dGetIdentityMatrix());

	scene->AddEntity(entity);
	geometry->Release();
	return entity;
}

ndBodyKinematic* BuildProceduralMap(ndDemoEntityManager* const scene, dInt32 grids, dFloat32 gridSize, dFloat32 perturbation)
{
#if 1
	dVector origin(-grids * gridSize * 0.5f, 0.0f, -grids * gridSize * 0.5f, 0.0f);
	
	dArray<dVector> points;
	for (dInt32 iz = 0; iz <= grids; iz++)
	{
		dFloat32 z0 = origin.m_z + iz * gridSize;
		for (dInt32 ix = 0; ix <= grids; ix++)
		{
			dFloat32 x0 = origin.m_x + ix * gridSize;
			points.PushBack(dVector(x0, dGaussianRandom(perturbation), z0, 1.0f));
		}
	}
	
	ndMeshEffect meshEffect;
	meshEffect.BeginBuild();
	for (dInt32 iz = 0; iz < grids; iz++)
	{ 
		for (dInt32 ix = 0; ix < grids; ix++)
		{
			dVector p0(points[(ix + 0) * (grids + 1) + iz + 0]);
			dVector p1(points[(ix + 1) * (grids + 1) + iz + 0]);
			dVector p2(points[(ix + 1) * (grids + 1) + iz + 1]);
			dVector p3(points[(ix + 0) * (grids + 1) + iz + 1]);
	
			meshEffect.BeginBuildFace();
			meshEffect.AddPoint(p0.m_x, p0.m_y, p0.m_z);
			meshEffect.AddPoint(p1.m_x, p1.m_y, p1.m_z);
			meshEffect.AddPoint(p2.m_x, p2.m_y, p2.m_z);
			meshEffect.EndBuildFace();
	
			meshEffect.BeginBuildFace();
			meshEffect.AddPoint(p0.m_x, p0.m_y, p0.m_z);
			meshEffect.AddPoint(p2.m_x, p2.m_y, p2.m_z);
			meshEffect.AddPoint(p3.m_x, p3.m_y, p3.m_z);
			meshEffect.EndBuildFace();
		}
	}
	meshEffect.EndBuild(0.0f);
	
	
	dPolygonSoupBuilder meshBuilder;
	meshBuilder.Begin();
	
	dInt32 vertexStride = meshEffect.GetVertexStrideInByte() / sizeof(dFloat64);
	const dFloat64* const vertexData = meshEffect.GetVertexPool();
	
	dInt32 mark = meshEffect.IncLRU();
	
	dVector face[16];
	dPolyhedra::Iterator iter(meshEffect);
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
				face[count] = point;
				count++;
				ptr->m_mark = mark;
				ptr = ptr->m_next;
			} while (ptr != edge);
	
			dInt32 materialIndex = meshEffect.GetFaceMaterial(edge);
			meshBuilder.AddFace(&face[0].m_x, sizeof(dVector), 3, materialIndex);
		}
	}
	meshBuilder.End(false);
	ndShapeInstance plane(new ndShapeStatic_bvh(meshBuilder));
#else
	ndShapeInstance plane(new ndRegularProceduralGrid(2.0f * grids * gridSize, 1.0f, 2.0f * grids * gridSize));
#endif

	dMatrix matrix(dGetIdentityMatrix());
	ndPhysicsWorld* const world = scene->GetWorld();

	ndBodyDynamic* const body = new ndBodyDynamic();
	body->SetMatrix(matrix);
	body->SetCollisionShape(plane);
	body->SetNotifyCallback(new ndDemoEntityNotify(scene, BuildVisualEntiry(scene, grids, gridSize, perturbation)));
	world->AddBody(body);
	return body;
}

