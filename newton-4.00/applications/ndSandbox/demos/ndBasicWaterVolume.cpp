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

#include "ndSandboxStdafx.h"
#include "ndSkyBox.h"
#include "ndTargaToOpenGl.h"
#include "ndDemoMesh.h"
#include "ndDemoCamera.h"
#include "ndPhysicsUtils.h"
#include "ndPhysicsWorld.h"
#include "ndMakeStaticMap.h"
#include "ndDemoEntityManager.h"
#include "ndArchimedesBuoyancyVolume.h"


class ndIsoSurfaceMesh : public ndDemoMesh
{
	public:
	ndIsoSurfaceMesh(const ndShaderPrograms& shaderCache)
		:ndDemoMesh("isoSurface")
	{
		m_shader = shaderCache.m_diffuseEffect;

		ndDemoSubMesh* const segment = AddSubMesh();
		//segment->m_material.m_textureHandle = (GLuint)material;
		segment->m_material.m_textureHandle = (GLuint)1;

		segment->SetOpacity(1.0f);
		segment->m_segmentStart = 0;
		segment->m_indexCount = 0;
	}

	void UpdateBuffers(const dArray<ndMeshPointUV>& points, const dArray<dInt32>& indexList)
	{
		OptimizeForRender(points, indexList);

		ndDemoSubMesh& segment = GetFirst()->GetInfo();
		segment.m_indexCount = indexList.GetCount();
	}
};

class ndWaterVolumeEntity : public ndDemoEntity
{
	public:
	ndWaterVolumeEntity(ndDemoEntityManager* const scene, const dMatrix& location, const dVector& size, ndBodySphFluid* const fluidBody, dFloat32 radius)
		:ndDemoEntity(location, nullptr)
		,m_fluidBody(fluidBody)
		,m_hasNewMesh(false)
	{
		ndShapeInstance box(new ndShapeBox(9.0f, 10.0f, 9.0f));
		dMatrix uvMatrix(dGetIdentityMatrix());
		uvMatrix[0][0] *= 1.0f / 20.0f;
		uvMatrix[1][1] *= 1.0f / 10.0f;
		uvMatrix[2][2] *= 1.0f / 20.0f;
		uvMatrix.m_posit = dVector(0.5f, 0.5f, 0.5f, 1.0f);

		ndDemoMesh* const geometry = new ndDemoMesh("fluidVolume", scene->GetShaderCache(), &box, "metal_30.tga", "metal_30.tga", "logo_php.tga", 0.5f, uvMatrix);
		SetMesh(geometry, dGetIdentityMatrix());

		scene->AddEntity(this);
		geometry->Release();

		//ndShapeInstance shape(new ndShapeSphere(radius));
		ndShapeInstance shape(new ndShapeBox(radius * 2.0f, radius * 2.0f, radius * 2.0f));

		m_meshParticle = new ndDemoMeshIntance("shape", scene->GetShaderCache(), &shape, "marble.tga", "marble.tga", "marble.tga");

		m_isoSurfaceMesh0 = new ndIsoSurfaceMesh(scene->GetShaderCache());
		m_isoSurfaceMesh1 = new ndIsoSurfaceMesh(scene->GetShaderCache());
	}

	~ndWaterVolumeEntity()
	{
		dScopeSpinLock lock(m_lock);
		m_meshParticle->Release();
		m_isoSurfaceMesh0->Release();
		m_isoSurfaceMesh1->Release();
	}

	void Render(dFloat32 timeStep, ndDemoEntityManager* const scene, const dMatrix& matrix) const
	{
		dMatrix nodeMatrix(m_matrix * matrix);

		//const dArray<dVector>& positions = m_fluidBody->GetPositions();
		//m_meshParticle->SetParticles(positions.GetCount(), &positions[0]);
		//nodeMatrix.m_posit.m_y += 1.0f;
		//m_meshParticle->Render(scene, nodeMatrix);
	
		nodeMatrix.m_posit.m_y += 0.25f;
		//nodeMatrix.m_posit.m_z += 0.0f;

		m_isoSurfaceMesh0->Render(scene, nodeMatrix);
		dScopeSpinLock lock(m_lock);
		if (m_hasNewMesh)
		{
			m_hasNewMesh = false;
			m_isoSurfaceMesh1->UpdateBuffers(m_points, m_indexList);
			dSwap(m_isoSurfaceMesh0, m_isoSurfaceMesh1);
		}
	}

	void UpdateIsoSuface(const dIsoSurface& isoSurface)
	{
		if (isoSurface.GetVertexCount())
		{
			m_points.SetCount(isoSurface.GetVertexCount());
			const dVector* const points = isoSurface.GetPoints();
			const dVector* const normals = isoSurface.GetNormals();
			for (dInt32 i = 0; i < isoSurface.GetVertexCount(); i++)
			{
				m_points[i].m_posit = ndMeshVector(GLfloat(points[i].m_x), GLfloat(points[i].m_y), GLfloat(points[i].m_z));
				m_points[i].m_normal = ndMeshVector(GLfloat(normals[i].m_x), GLfloat(normals[i].m_y), GLfloat(normals[i].m_z));
				m_points[i].m_uv.m_u = GLfloat(0.0f);
				m_points[i].m_uv.m_v = GLfloat(0.0f);
			}

			m_indexList.SetCount(isoSurface.GetIndexCount());
			const dUnsigned64* const indexList = isoSurface.GetIndexList();
			for (dInt32 i = 0; i < isoSurface.GetIndexCount(); i++)
			{
				m_indexList[i] = dInt32(indexList[i]);
			}

			dScopeSpinLock lock(m_lock);
			m_hasNewMesh = true;
		}
	}

	ndBodySphFluid* m_fluidBody;
	ndDemoMeshIntance* m_meshParticle;

	dArray<dInt32> m_indexList;
	dArray<ndMeshPointUV> m_points;
	mutable bool m_hasNewMesh;
	mutable dSpinLock m_lock;
	mutable ndIsoSurfaceMesh* m_isoSurfaceMesh0;
	mutable ndIsoSurfaceMesh* m_isoSurfaceMesh1;
};

class ndWaterVolumeCallback: public ndDemoEntityNotify
{
	public:
	ndWaterVolumeCallback(ndDemoEntityManager* const manager, ndDemoEntity* const entity)
		:ndDemoEntityNotify(manager, entity)
	{
	}

	void OnTranform(dInt32 threadIndex, const dMatrix& matrix)
	{
		ndBodySphFluid* const fluid = GetBody()->GetAsBodySphFluid();
		dAssert(fluid);

		//fluid->GenerateIsoSurface(m_manager->GetWorld(), 0.25f);
		fluid->GenerateIsoSurface(m_manager->GetWorld(), 0.15f);
		const dIsoSurface& isoSurface = fluid->GetIsoSurface();

		ndWaterVolumeEntity* const entity = (ndWaterVolumeEntity*)GetUserData();
		entity->UpdateIsoSuface(isoSurface);
	}
};

static void AddWaterVolume(ndDemoEntityManager* const scene, const dMatrix& location)
{
	ndPhysicsWorld* const world = scene->GetWorld();

	dMatrix matrix(location);
	dVector floor(FindFloor(*world, matrix.m_posit, 200.0f));
	matrix.m_posit = floor;
	matrix.m_posit.m_w = 1.0f;

	dFloat32 diameter = 0.25f;
	ndBodySphFluid* const fluidObject = new ndBodySphFluid();
	ndWaterVolumeEntity* const entity = new ndWaterVolumeEntity(scene, matrix, dVector(20.0f, 10.0f, 20.0f, 0.0f), fluidObject, diameter * 0.25f);

	fluidObject->SetNotifyCallback(new ndWaterVolumeCallback(scene, entity));
	fluidObject->SetMatrix(matrix);

	fluidObject->SetParticleRadius(diameter * 0.5f);

	//dInt32 particleCountPerAxis = 32;
	dInt32 particleCountPerAxis = 7;
	dFloat32 spacing = diameter * 1.0f;

	dFloat32 offset = spacing * particleCountPerAxis / 2.0f;
	dVector origin(-offset, 1.0f, -offset, dFloat32(0.0f));
	origin += matrix.m_posit;
origin.m_x = 0.0f;
origin.m_z = 0.0f;
origin.m_y = 1.0f;
	
	for (dInt32 z = 0; z < particleCountPerAxis; z++)
	//for (dInt32 z = 0; z < 1; z++)
	{
		//for (dInt32 y = 0; y < particleCountPerAxis; y++)
		for (dInt32 y = 0; y < 1; y++)
		{
			//for (dInt32 x = 0; x < particleCountPerAxis; x++)
			for (dInt32 x = 0; x < 1; x++)
			{
				dVector posit(x * spacing, y * spacing, z * spacing, 0.0f);
				fluidObject->AddParticle(0.1f, origin + posit, dVector::m_zero);

				//dVector xxxx(dVector::m_zero);
				//fluidObject->AddParticle(0.1f, xxxx, dVector::m_zero);

				//posit += dVector(0.01f, 0.01f, 0.01f, 0.0f);
				//fluidObject->AddParticle(0.1f, origin + posit, dVector::m_zero);
			}
		}
	}

	world->AddBody(fluidObject);
}

void ndBasicWaterVolume (ndDemoEntityManager* const scene)
{
	// build a floor
	//BuildFlatPlane(scene, true);

	dMatrix location(dGetIdentityMatrix());

	// adding a water volume
	AddWaterVolume(scene, location);

	dQuaternion rot;
	dVector origin(-15.0f, 4.0f, 0.0f, 0.0f);
	scene->SetCameraMatrix(rot, origin);
}
