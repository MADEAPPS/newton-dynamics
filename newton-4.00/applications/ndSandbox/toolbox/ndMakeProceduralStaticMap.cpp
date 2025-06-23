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
#include "ndDemoMesh.h"
#include "ndDemoEntity.h"
#include "ndDebugDisplay.h"
#include "ndPhysicsWorld.h"
#include "ndMakeStaticMap.h"
#include "ndDemoEntityNotify.h"
#include "ndDemoEntityManager.h"
#include "ndMakeProceduralStaticMap.h"

class ndRegularProceduralGrid : public ndShapeStaticProceduralMesh
{
	public:
	D_CLASS_REFLECTION(ndRegularProceduralGrid, ndShapeStaticProceduralMesh)

	ndRegularProceduralGrid(ndFloat32 gridSize, ndFloat32 sizex, ndFloat32 sizey, ndFloat32 sizez, const ndVector& planeEquation)
		:ndShapeStaticProceduralMesh(sizex, sizey, sizez)
		,m_planeEquation(planeEquation)
		,m_gridSize(gridSize)
		,m_invGridSize(ndFloat32 (1.0f)/ m_gridSize)
	{
	}

	virtual void DebugShape(const ndMatrix&, ndShapeDebugNotify& notify) const override
	{
		ndDebugNotify& debugDraw = (ndDebugNotify&)notify;
		ndBodyKinematic* const body = debugDraw.m_body;
		
		// this demo will iterate over the all the body contact pairs drawing the face in aabb.
		ndArray<ndVector> vertex;
		ndArray<ndInt32> faceList;
		ndArray<ndInt32> faceMaterial;
		ndArray<ndInt32> indexListList;

		ndMatrix myMatrix(body->GetMatrix());
		ndBodyKinematic::ndContactMap& contactJoints = body->GetContactMap();
		ndBodyKinematic::ndContactMap::Iterator it(contactJoints);
		for (it.Begin(); it; it++)
		{
			const ndContact* const contact = it.GetNode()->GetInfo();
			if (contact->IsActive())
			{
				ndBodyKinematic* const body0 = contact->GetBody0();
				ndShapeInstance& collision = body0->GetCollisionShape();

				ndVector minP; 
				ndVector maxP;
				ndMatrix matrix(body0->GetMatrix());
				//collision.CalculateAabb(matrix.Inverse(), minP, maxP);
				collision.CalculateAabb(matrix * myMatrix.OrthoInverse(), minP, maxP);

				vertex.SetCount(0);
				faceList.SetCount(0);
				faceMaterial.SetCount(0);
				indexListList.SetCount(0);
				GetCollidingFaces(minP, maxP, vertex, faceList, faceMaterial, indexListList);

				ndInt32 index = 0;
				ndVector color(50.0f / 255.0f, 100.0f / 255.0f, 200.0f / 255.0f, 1.0f);

				for (ndInt32 i = 0; i < faceList.GetCount(); ++i)
				{
					ndVector points[32];
					ndInt32 vCount = faceList[i];
					for (ndInt32 j = 0; j < vCount; ++j)
					{
						ndInt32 k = indexListList[index + j];
						points[j] = myMatrix.TransformVector(vertex[k]);
					}

					// I do not know why this is not rendering
					//RenderPolygon(debugDraw.m_manager, points, vCount, color);
					index += vCount;
				}
			}
		}
	}

	virtual ndFloat32 RayCast(ndRayCastNotify&, const ndVector& localP0, const ndVector& localP1, ndFloat32, const ndBody* const, ndContactPoint& contactOut) const override
	{
		ndVector segment(ndVector::m_triplexMask & (localP1 - localP0));
		ndFloat32 den = m_planeEquation.DotProduct(segment).GetScalar();
		ndFloat32 num = m_planeEquation.DotProduct(localP0).GetScalar() + m_planeEquation.m_w;
		ndFloat32 t = -num / den;
		contactOut.m_point = localP0 + segment.Scale(t);
		contactOut.m_normal = m_planeEquation;
		return ndClamp (t, ndFloat32 (0.0f), ndFloat32 (1.2f));
	}

	virtual void GetCollidingFaces(ndPolygonMeshDesc* const data) const override
	{
		ndShapeStaticProceduralMesh::GetCollidingFaces(data);
	}

	virtual void GetCollidingFaces(const ndVector& minBox, const ndVector& maxBox, ndArray<ndVector>& vertex, ndArray<ndInt32>& faceList, ndArray<ndInt32>& faceMaterial, ndArray<ndInt32>& indexListList) const
	{
		// generate the point cloud
		ndVector p0(minBox.Scale(m_invGridSize).Floor());
		ndVector p1(maxBox.Scale(m_invGridSize).Floor() + ndVector::m_one);
		ndVector origin(p0.Scale(m_gridSize) & ndVector::m_triplexMask);
		ndInt32 count_x = ndInt32(p1.m_x - p0.m_x);
		ndInt32 count_z = ndInt32(p1.m_z - p0.m_z);

		origin.m_y = 0.0f;
		for (ndInt32 iz = 0; iz <= count_z; iz++)
		{
			ndVector point(origin);
			for (ndInt32 ix = 0; ix <= count_x; ix++)
			{
				vertex.PushBack(point);
				point.m_x += m_gridSize;
			}
			origin.m_z += m_gridSize;
		}

		// generate the face array
		const ndInt32 stride = count_x + 1;
		for (ndInt32 iz = 0; iz < count_z; iz++)
		{
			for (ndInt32 ix = 0; ix < count_x; ix++)
			{
				faceList.PushBack(4);
				indexListList.PushBack((iz + 0) * stride + ix + 0);
				indexListList.PushBack((iz + 1) * stride + ix + 0);
				indexListList.PushBack((iz + 1) * stride + ix + 1);
				indexListList.PushBack((iz + 0) * stride + ix + 1);

				faceMaterial.PushBack(0);
			}
		}
	}

	virtual ndUnsigned64 GetHash(ndUnsigned64 hash) const override
	{
		hash = ndCRC64(&m_planeEquation[0], sizeof(ndVector), hash);
		hash = ndCRC64(&m_gridSize, sizeof(ndFloat32), hash);
		return hash;
	}

	ndVector m_planeEquation;
	ndFloat32 m_gridSize;
	ndFloat32 m_invGridSize;
};

static ndSharedPtr<ndDemoEntity> BuildVisualEntity(ndDemoEntityManager* const scene, ndInt32 grids, ndFloat32 gridSize, ndFloat32 perturbation)
{
	ndVector origin(-(ndFloat32)grids * gridSize * 0.5f, 0.0f, -(ndFloat32)grids * gridSize * 0.5f, 1.0f);

	ndArray<ndVector> points;
	for (ndInt32 iz = 0; iz <= grids; iz++)
	{
		ndFloat32 z0 = origin.m_z + (ndFloat32)iz * gridSize;
		for (ndInt32 ix = 0; ix <= grids; ix++)
		{
			ndFloat32 x0 = origin.m_x + (ndFloat32)ix * gridSize;
			points.PushBack(ndVector(x0, ndGaussianRandom(0.0f, perturbation), z0, 1.0f));
		}
	}

	ndMeshEffect meshEffect;
	meshEffect.BeginBuild();

	ndMeshEffect::ndMaterial material;
	ndArray<ndMeshEffect::ndMaterial>& materialArray = meshEffect.GetMaterials();
	strcpy(material.m_textureName, "marbleCheckBoard.png");
	materialArray.PushBack(material);

	ndFloat32 uvScale = 1.0 / 16.0f;
	for (ndInt32 iz = 0; iz < grids; iz++)
	{
		for (ndInt32 ix = 0; ix < grids; ix++)
		{
			ndVector p0(points[(ix + 0) * (grids + 1) + iz + 0]);
			ndVector p1(points[(ix + 1) * (grids + 1) + iz + 0]);
			ndVector p2(points[(ix + 1) * (grids + 1) + iz + 1]);
			ndVector p3(points[(ix + 0) * (grids + 1) + iz + 1]);

			meshEffect.BeginBuildFace();
				meshEffect.AddMaterial(0);
				meshEffect.AddPoint(p0.m_x, p0.m_y, p0.m_z);
				meshEffect.AddNormal(0.0f, 1.0f, 0.0f);
				meshEffect.AddUV0(p0.m_x * uvScale, p0.m_z * uvScale);

				meshEffect.AddMaterial(0);
				meshEffect.AddPoint(p1.m_x, p1.m_y, p1.m_z);
				meshEffect.AddNormal(0.0f, 1.0f, 0.0f);
				meshEffect.AddUV0(p1.m_x * uvScale, p1.m_z * uvScale);

				meshEffect.AddMaterial(0);
				meshEffect.AddPoint(p2.m_x, p2.m_y, p2.m_z);
				meshEffect.AddNormal(0.0f, 1.0f, 0.0f);
				meshEffect.AddUV0(p2.m_x * uvScale, p2.m_z * uvScale);
			meshEffect.EndBuildFace();

			meshEffect.BeginBuildFace();
				meshEffect.AddMaterial(0);
				meshEffect.AddPoint(p0.m_x, p0.m_y, p0.m_z);
				meshEffect.AddNormal(0.0f, 1.0f, 0.0f);
				meshEffect.AddUV0(p0.m_x * uvScale, p0.m_z * uvScale);

				meshEffect.AddMaterial(0);
				meshEffect.AddPoint(p2.m_x, p2.m_y, p2.m_z);
				meshEffect.AddNormal(0.0f, 1.0f, 0.0f);
				meshEffect.AddUV0(p2.m_x * uvScale, p2.m_z * uvScale);

				meshEffect.AddMaterial(0);
				meshEffect.AddPoint(p3.m_x, p3.m_y, p3.m_z);
				meshEffect.AddNormal(0.0f, 1.0f, 0.0f);
				meshEffect.AddUV0(p3.m_x * uvScale, p3.m_z * uvScale);
			meshEffect.EndBuildFace();
		}
	}
	meshEffect.EndBuild(0.0f);

	ndSharedPtr<ndDemoMeshInterface>geometry (new ndDemoMesh("plane", &meshEffect, scene->GetShaderCache()));

	ndMatrix matrix(ndGetIdentityMatrix());
	ndSharedPtr<ndDemoEntity>entity (new ndDemoEntity(matrix));
	entity->SetMesh(geometry);
	entity->SetShadowMode(false);

	scene->AddEntity(entity);
	return entity;
}

ndBodyKinematic* BuildProceduralMap(ndDemoEntityManager* const scene, ndInt32 grids, ndFloat32 gridSize, ndFloat32 perturbation)
{
	ndPlane planeEquation(ndVector(0.0f, 1.0f, 0.0f, 0.0f));
	ndShapeInstance plane(new ndRegularProceduralGrid(gridSize, 2.0f * (ndFloat32)grids * gridSize, 1.0f, 2.0f * (ndFloat32)grids * gridSize, planeEquation));

	ndMatrix matrix(ndGetIdentityMatrix());
	ndPhysicsWorld* const world = scene->GetWorld();

	ndSharedPtr<ndBody> body (new ndBodyDynamic());
	body->SetMatrix(matrix);
	body->GetAsBodyDynamic()->SetCollisionShape(plane);
	body->SetNotifyCallback(new ndDemoEntityNotify(scene, BuildVisualEntity(scene, grids, gridSize, perturbation)));

	world->AddBody(body);
	return body->GetAsBodyDynamic();
}

