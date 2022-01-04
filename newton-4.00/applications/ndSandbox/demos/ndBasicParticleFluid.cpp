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
#include "ndSkyBox.h"
#include "ndTargaToOpenGl.h"
#include "ndDemoMesh.h"
#include "ndDemoCamera.h"
#include "ndPhysicsUtils.h"
#include "ndPhysicsWorld.h"
#include "ndMakeStaticMap.h"
#include "ndDemoEntityManager.h"
#include "ndArchimedesBuoyancyVolume.h"

class ndIsoSurfaceParticleVolume : public ndBodySphFluid, public ndBackgroundJob
{
	public:
	ndIsoSurfaceParticleVolume(ndFloat32 radius)
		:ndBodySphFluid()
		,ndBackgroundJob()
		,m_indexList(1024)
		,m_points(1024)
		,m_meshIsReady(0)
	{
		SetParticleRadius(radius);
	}

	void UpdateIsoSurface()
	{
		D_TRACKTIME();
		m_isoSurface.GenerateMesh(GetPositions(), GetParticleRadius() * ndFloat32 (2.0f));
		const ndArray<ndVector>& points = m_isoSurface.GetPoints();
		const ndArray<ndVector>& normals = m_isoSurface.GetNormals();
		dAssert(points.GetCount());
		m_points.SetCount(points.GetCount());
		for (ndInt32 i = 0; i < points.GetCount(); ++i)
		{
			m_points[i].m_posit = glVector3(GLfloat(points[i].m_x), GLfloat(points[i].m_y), GLfloat(points[i].m_z));
			m_points[i].m_normal = glVector3(GLfloat(normals[i].m_x), GLfloat(normals[i].m_y), GLfloat(normals[i].m_z));
			m_points[i].m_uv.m_u = GLfloat(0.0f);
			m_points[i].m_uv.m_v = GLfloat(0.0f);
		}
		
		const ndArray<ndIsoSurface::ndTriangle>& indexList = m_isoSurface.GetTriangles();
		m_indexList.SetCount(indexList.GetCount() * 3);
		for (ndInt32 i = 0; i < indexList.GetCount(); i++)
		{
			m_indexList[i * 3 + 0] = indexList[i].m_index[0];
			m_indexList[i * 3 + 1] = indexList[i].m_index[1];
			m_indexList[i * 3 + 2] = indexList[i].m_index[2];
		}
		m_meshIsReady.store(1);
	}
		
	virtual void Execute()
	{
		UpdateIsoSurface();
	}

	ndArray<ndInt32> m_indexList;
	ndArray<glPositionNormalUV> m_points;
	ndIsoSurface m_isoSurface;
	ndAtomic<ndInt32> m_meshIsReady;
};

class ndIsoSurfaceMesh : public ndDemoMesh
{
	public:
	ndIsoSurfaceMesh(const ndShaderPrograms& shaderCache, ndDemoMesh* const parentMesh)
		:ndDemoMesh("isoSurface")
		,m_parentMesh(parentMesh)
	{
		m_shader = shaderCache.m_diffuseEffect;

		ndDemoSubMesh* const segment = AddSubMesh();
		//segment->m_material.m_textureHandle = (GLuint)material;
		segment->m_material.m_textureHandle = (GLuint)1;

		segment->m_material.m_diffuse = ndVector(0.1f, 0.6f, 0.9f, 0.0f);
		segment->m_material.m_ambient = ndVector(0.1f, 0.6f, 0.9f, 0.0f);

		//segment->SetOpacity(0.4f);
		segment->SetOpacity(1.0f);
		segment->m_segmentStart = 0;
		segment->m_indexCount = 0;
		//m_hasTransparency = true;
		m_hasTransparency = false;
	}

	void UpdateBuffers(const ndArray<glPositionNormalUV>& points, const ndArray<ndInt32>& indexList)
	{
		OptimizeForRender(&points[0], points.GetCount(), &indexList[0], indexList.GetCount());

		ndDemoSubMesh& segment = GetFirst()->GetInfo();
		segment.m_indexCount = indexList.GetCount();
	}

	//void RenderTransparency(ndDemoEntityManager* const scene, const ndMatrix& modelMatrix)
	void RenderTransparency(ndDemoEntityManager* const, const ndMatrix&)
	{
		dAssert(0);
		dTrace(("implement traparent iso surface render\n"));
		//ndDemoMesh::RenderTransparency(scene, modelMatrix);
	}

	ndDemoMesh* m_parentMesh;
};

class ndWaterVolumeEntity : public ndDemoEntity
{
	public:
	ndWaterVolumeEntity(ndDemoEntityManager* const scene, const ndMatrix& location, const ndVector&, ndIsoSurfaceParticleVolume* const fluidBody)
		:ndDemoEntity(location, nullptr)
		,m_fluidBody(fluidBody)
	{
		ndShapeInstance box(new ndShapeBox(9.0f, 10.0f, 9.0f));
		ndMatrix uvMatrix(dGetIdentityMatrix());
		uvMatrix[0][0] *= 1.0f / 20.0f;
		uvMatrix[1][1] *= 1.0f / 10.0f;
		uvMatrix[2][2] *= 1.0f / 20.0f;
		uvMatrix.m_posit = ndVector(0.5f, 0.5f, 0.5f, 1.0f);

		ndDemoMesh* const geometry = new ndDemoMesh("fluidVolume", scene->GetShaderCache(), &box, "metal_30.tga", "metal_30.tga", "logo_php.tga", 0.5f, uvMatrix);
		SetMesh(geometry, dGetIdentityMatrix());

		scene->AddEntity(this);
		m_isoSurfaceMesh0 = new ndIsoSurfaceMesh(scene->GetShaderCache(), geometry);
		m_isoSurfaceMesh1 = new ndIsoSurfaceMesh(scene->GetShaderCache(), geometry);
		geometry->Release();
	}

	~ndWaterVolumeEntity()
	{
		ndScopeSpinLock lock(m_lock);
		m_isoSurfaceMesh0->Release();
		m_isoSurfaceMesh1->Release();
	}

	void Render(ndFloat32, ndDemoEntityManager* const scene, const ndMatrix&) const
	{
		// for now the mesh in is global space I need to fix that
		//ndMatrix nodeMatrix(m_matrix * matrix);

		if (m_fluidBody->m_meshIsReady.load())
		{
			ndScopeSpinLock lock(m_lock);

			if (m_fluidBody->JobState() == ndBackgroundJob::m_jobCompleted)
			{
				m_fluidBody->m_meshIsReady.store(0);
				const ndArray<ndInt32>& indexList = m_fluidBody->m_indexList;
				const ndArray<glPositionNormalUV>& points = m_fluidBody->m_points;
				m_isoSurfaceMesh1->UpdateBuffers(points, indexList);
				dSwap(m_isoSurfaceMesh0, m_isoSurfaceMesh1);
			}
		}

		ndMatrix nodeMatrix(dGetIdentityMatrix());
		//nodeMatrix.m_posit.m_y += 0.125f;
	
		// render the fluid;
		ndScopeSpinLock lock(m_lock);
		glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
		m_isoSurfaceMesh0->Render(scene, nodeMatrix);
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
		
		// render the cage;
		//ndDemoEntity::Render(timeStep, scene, matrix);
	}

	ndIsoSurfaceParticleVolume* m_fluidBody;
	mutable ndSpinLock m_lock;
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

	void OnTransform(ndInt32, const ndMatrix&)
	{
		ndWaterVolumeEntity* const entity = (ndWaterVolumeEntity*)GetUserData();
		ndScopeSpinLock lock(entity->m_lock);
		//entity->m_fluidBody->UpdateIsoSurface();

		if (entity->m_fluidBody->JobState() == ndBackgroundJob::m_jobCompleted)
		{
			ndPhysicsWorld* const world = m_manager->GetWorld();
			world->SendBackgroundJob(entity->m_fluidBody);
		}
	}
};

static void AddWaterVolume(ndDemoEntityManager* const scene, const ndMatrix& location)
{
	ndPhysicsWorld* const world = scene->GetWorld();

	ndMatrix matrix(location);
	ndVector floor(FindFloor(*world, matrix.m_posit, 200.0f));
	matrix.m_posit = floor;
	matrix.m_posit.m_w = 1.0f;

	ndFloat32 diameter = 0.25f;
	ndIsoSurfaceParticleVolume* const fluidObject = new ndIsoSurfaceParticleVolume(diameter * 0.5f);
	ndWaterVolumeEntity* const entity = new ndWaterVolumeEntity(scene, matrix, ndVector(20.0f, 10.0f, 20.0f, 0.0f), fluidObject);

	fluidObject->SetNotifyCallback(new ndWaterVolumeCallback(scene, entity));
	fluidObject->SetMatrix(matrix);

	fluidObject->SetParticleRadius(diameter * 0.5f);

	ndInt32 particleCountPerAxis = 40;
	//ndInt32 particleCountPerAxis = 32;
	//ndInt32 particleCountPerAxis = 10;
	//ndInt32 particleCountPerAxis = 1;
	ndFloat32 spacing = diameter * 1.0f;

	ndFloat32 offset = spacing * particleCountPerAxis / 2.0f;
	ndVector origin(-offset, 1.0f, -offset, ndFloat32(0.0f));

	matrix.m_posit += origin;
matrix.m_posit = ndVector (2.0f, 2.0f, 2.0f, 0.0f);
	
	ndArray<ndVector>& particles = fluidObject->GetPositions();
	for (ndInt32 y = 0; y < particleCountPerAxis; y++)
	{
		for (ndInt32 z = 0; z < particleCountPerAxis; z++)
		{
			for (ndInt32 x = 0; x < particleCountPerAxis; x++)
			{
				const ndVector posit (matrix.TransformVector(ndVector (x * spacing, y * spacing, z * spacing, ndFloat32 (1.0f))));
				particles.PushBack(posit);
			}
		}
	}

	// make sure we have the first surface generated before rendering.
	fluidObject->UpdateIsoSurface();

	// add particle volume to world
	world->AddBody(fluidObject);
}

void ndBasicParticleFluid (ndDemoEntityManager* const scene)
{
	// build a floor
	BuildFlatPlane(scene, true);

	ndMatrix location(dGetIdentityMatrix());

	// adding a water volume
	AddWaterVolume(scene, location);

	ndQuaternion rot;
	ndVector origin(-15.0f, 4.0f, 0.0f, 0.0f);
	scene->SetCameraMatrix(rot, origin);
}
