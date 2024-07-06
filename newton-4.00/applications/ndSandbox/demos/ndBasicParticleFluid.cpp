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
#include "ndDebugDisplay.h"
#include "ndPhysicsUtils.h"
#include "ndPhysicsWorld.h"
#include "ndTargaToOpenGl.h"
#include "ndMakeStaticMap.h"
#include "ndDemoEntityNotify.h"
#include "ndDemoEntityManager.h"
#include "ndArchimedesBuoyancyVolume.h"

class ndIsoSurfaceParticleVolume : public ndBodySphFluid
{
	public:
	ndIsoSurfaceParticleVolume(ndFloat32 radius)
		:ndBodySphFluid()
		,m_indexList(1024)
		,m_points(1024)
		,m_meshIsReady(0)
	{
		SetParticleRadius(radius);
		SetGravity(ndVector(ndFloat32(0.0f), DEMO_GRAVITY, ndFloat32(0.0f), ndFloat32(0.0f)));
	}

	void BuildTriangleList()
	{
		D_TRACKTIME();
		const ndArray<ndVector>& points = m_isoSurface.GetPoints();
		ndAssert(points.GetCount());

		m_points.SetCount(points.GetCount());
		m_indexList.SetCount(points.GetCount());

		const ndVector origin(m_isoSurface.GetOrigin());
		for (ndInt32 i = 0; i < points.GetCount(); i += 3)
		{
			const ndVector p0(origin + points[i + 0]);
			const ndVector p1(origin + points[i + 1]);
			const ndVector p2(origin + points[i + 2]);

			const ndVector vec1(p1 - p0);
			const ndVector vec2(p2 - p0);
			const ndVector cross(vec1.CrossProduct(vec2));
			const ndVector normal(cross.Normalize());

			m_points[i + 0].m_posit = glVector3(GLfloat(p0.m_x), GLfloat(p0.m_y), GLfloat(p0.m_z));
			m_points[i + 1].m_posit = glVector3(GLfloat(p1.m_x), GLfloat(p1.m_y), GLfloat(p1.m_z));
			m_points[i + 2].m_posit = glVector3(GLfloat(p2.m_x), GLfloat(p2.m_y), GLfloat(p2.m_z));

			m_points[i + 0].m_normal = glVector3(GLfloat(normal.m_x), GLfloat(normal.m_y), GLfloat(normal.m_z));
			m_points[i + 1].m_normal = glVector3(GLfloat(normal.m_x), GLfloat(normal.m_y), GLfloat(normal.m_z));
			m_points[i + 2].m_normal = glVector3(GLfloat(normal.m_x), GLfloat(normal.m_y), GLfloat(normal.m_z));

			m_points[i + 0].m_uv.m_u = GLfloat(0.0f);
			m_points[i + 0].m_uv.m_v = GLfloat(0.0f);
			m_points[i + 1].m_uv.m_u = GLfloat(0.0f);
			m_points[i + 1].m_uv.m_v = GLfloat(0.0f);
			m_points[i + 2].m_uv.m_u = GLfloat(0.0f);
			m_points[i + 2].m_uv.m_v = GLfloat(0.0f);

			m_indexList[i + 0] = i + 0;
			m_indexList[i + 1] = i + 1;
			m_indexList[i + 2] = i + 2;
		}
	}

	void BuildIndexList()
	{
		D_TRACKTIME();
		const ndArray<ndVector>& points = m_isoSurface.GetPoints();
		if (points.GetCount())
		{
			m_points.SetCount(points.GetCount() * 3);
			m_indexList.SetCount(points.GetCount() * 3);

			ndReal* const posit = &m_points[0].m_posit.m_x;
			ndReal* const normal = &m_points[0].m_normal.m_x;
			ndInt32 pointCount = m_isoSurface.GenerateListIndexList(&m_indexList[0], sizeof(glPositionNormalUV) / sizeof(GLfloat), posit, normal);

			const ndVector origin(m_isoSurface.GetOrigin());
			for (ndInt32 i = 0; i < pointCount; ++i)
			{
				m_points[i].m_posit.m_x += GLfloat(origin.m_x);
				m_points[i].m_posit.m_y += GLfloat(origin.m_y);
				m_points[i].m_posit.m_z += GLfloat(origin.m_z);
				m_points[i].m_uv.m_u = GLfloat(0.0f);
				m_points[i].m_uv.m_v = GLfloat(0.0f);
			}
		}
	}

	void UpdateIsoSurface()
	{
		D_TRACKTIME();
		ndArray<ndVector>& pointCloud = GetPositions();
		ndFloat32 gridSpacing = 2.0f * GetParticleRadius();
		m_isoSurface.GenerateMesh(pointCloud, gridSpacing);

#if 1
		BuildIndexList();
#else
		BuildTriangleList();
#endif
		m_meshIsReady.store(1);
	}

	ndArray<ndInt32> m_indexList;
	ndArray<glPositionNormalUV> m_points;
	ndIsoSurface m_isoSurface;
	ndAtomic<ndInt32> m_meshIsReady;
};

class ndIsoSurfaceMesh : public ndDemoMesh
{
	public:
	ndIsoSurfaceMesh(const ndShaderCache& shaderCache, ndSharedPtr<ndDemoMeshInterface> parentMesh)
		:ndDemoMesh("isoSurface")
		,m_parentMesh(parentMesh)
	{
		m_shader = shaderCache.m_diffuseEffect;

		ndDemoSubMesh* const segment = AddSubMesh();
		GLuint tex = LoadTexture("default.png");
		segment->m_material.SetTexture(int(tex));
		ReleaseTexture(tex);

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
		if (points.GetCount())
		{
			OptimizeForRender(&points[0], ndInt32(points.GetCount()), &indexList[0], ndInt32(indexList.GetCount()));

			ndDemoSubMesh& segment = GetFirst()->GetInfo();
			segment.m_indexCount = ndInt32(indexList.GetCount());
		}
	}

	//void RenderTransparency(ndDemoEntityManager* const scene, const ndMatrix& modelMatrix)
	void RenderTransparency(ndDemoEntityManager* const, const ndMatrix&)
	{
		ndAssert(0);
		ndTrace(("implement traparent iso surface render\n"));
		//ndDemoMesh::RenderTransparency(scene, modelMatrix);
	}

	//ndDemoMesh* m_parentMesh;
	ndSharedPtr<ndDemoMeshInterface> m_parentMesh;
};

class ndWaterVolumeEntity : public ndDemoEntity
{
	public:
	ndWaterVolumeEntity(ndDemoEntityManager* const scene, const ndMatrix& location, const ndVector&, ndIsoSurfaceParticleVolume* const fluidBody)
		:ndDemoEntity(location, nullptr)
		,m_fluidBody(fluidBody)
		,m_isoSurfaceMesh0()
		,m_isoSurfaceMesh1()
	{
		ndShapeInstance box(new ndShapeBox(9.0f, 10.0f, 9.0f));
		ndMatrix uvMatrix(ndGetIdentityMatrix());
		uvMatrix[0][0] *= 1.0f / 20.0f;
		uvMatrix[1][1] *= 1.0f / 10.0f;
		uvMatrix[2][2] *= 1.0f / 20.0f;
		uvMatrix.m_posit = ndVector(0.5f, 0.5f, 0.5f, 1.0f);

		ndSharedPtr<ndDemoMeshInterface>geometry (new ndDemoMesh("fluidVolume", scene->GetShaderCache(), &box, "metal_30.png", "metal_30.png", "logo_php.png", 0.5f, uvMatrix));
		SetMesh(geometry);
		
		m_isoSurfaceMesh0 = ndSharedPtr<ndIsoSurfaceMesh>(new ndIsoSurfaceMesh(scene->GetShaderCache(), geometry));
		m_isoSurfaceMesh1 = ndSharedPtr<ndIsoSurfaceMesh>(new ndIsoSurfaceMesh(scene->GetShaderCache(), geometry));
	}

	~ndWaterVolumeEntity()
	{
		//ndScopeSpinLock lock(m_lock);
		//delete m_isoSurfaceMesh0;
		//delete m_isoSurfaceMesh1;
	}

	void Render(ndFloat32, ndDemoEntityManager* const scene, const ndMatrix&) const
	{
		// for now the mesh in is global space I need to fix that
		//ndMatrix nodeMatrix(m_matrix * matrix);

		if (m_fluidBody->m_meshIsReady.load())
		{
			ndScopeSpinLock lock(m_lock);

			if (m_fluidBody->TaskState() == ndBackgroundTask::m_taskCompleted)
			{
				m_fluidBody->m_meshIsReady.store(0);
				const ndArray<ndInt32>& indexList = m_fluidBody->m_indexList;
				const ndArray<glPositionNormalUV>& points = m_fluidBody->m_points;
				m_isoSurfaceMesh1->UpdateBuffers(points, indexList);
				ndSwap(m_isoSurfaceMesh0, m_isoSurfaceMesh1);
			}
		}

		ndMatrix nodeMatrix(ndGetIdentityMatrix());
		//nodeMatrix.m_posit.m_y += 0.125f;
	
		// render the fluid;
#if 0
		ndScopeSpinLock lock(m_lock);
		//glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
		m_isoSurfaceMesh0->Render(scene, nodeMatrix);
		//glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
#else
		//RenderParticles(scene);
#endif
		RenderParticles(scene);

		// render the cage;
		//ndDemoEntity::Render(timeStep, scene, matrix);
	}
	
	ndIsoSurfaceParticleVolume* m_fluidBody;
	mutable ndSpinLock m_lock;
	mutable ndSharedPtr<ndIsoSurfaceMesh> m_isoSurfaceMesh0;
	mutable ndSharedPtr<ndIsoSurfaceMesh> m_isoSurfaceMesh1;
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
		//ndWaterVolumeEntity* const entity = (ndWaterVolumeEntity*)GetUserData();
		//ndScopeSpinLock lock(entity->m_lock);
		//if (entity->m_fluidBody->taskState() == ndBackgroundTask::m_taskCompleted)
		//{
		//	ndPhysicsWorld* const world = m_manager->GetWorld();
		//	world->SendBackgroundJob(entity->m_fluidBody);
		//}
	}
};

static void BuildBox(const ndMatrix& matrix, ndIsoSurfaceParticleVolume* const surface, ndInt32 size)
{
	ndFloat32 spacing = 2.0f * surface->GetParticleRadius();
	ndArray<ndVector>& veloc = surface->GetVelocity();
	ndArray<ndVector>& posit = surface->GetPositions();

spacing *= 0.9f;
	ndVector v(ndVector::m_zero);
	for (ndInt32 z = 0; z < size; z++)
	//for (ndInt32 z = 0; z < 1; z++)
	{
		//for (ndInt32 y = 0; y < size; y++)
		for (ndInt32 y = 0; y < 1; y++)
		{
			//for (ndInt32 x = 0; x < size; x++)
			for (ndInt32 x = 0; x < 1; x++)
			{
				ndVector p(matrix.TransformVector(ndVector((ndFloat32)x * spacing, (ndFloat32)y * spacing, (ndFloat32)z * spacing, ndFloat32(1.0f))));
				p.m_x = spacing * (ndFloat32)ndInt32(p.m_x / spacing);
				p.m_y = spacing * (ndFloat32)ndInt32(p.m_y / spacing);
				p.m_z = spacing * (ndFloat32)ndInt32(p.m_z / spacing);

				//p.m_x += ndGaussianRandom(0.0f, spacing * 0.01f);
				//p.m_y += ndGaussianRandom(0.0f, spacing * 0.01f);
				//p.m_z += ndGaussianRandom(0.0f, spacing * 0.01f);
				posit.PushBack(p);
				veloc.PushBack(v);
			}
		}
	}
}

static void BuildHollowBox(const ndMatrix& matrix, ndIsoSurfaceParticleVolume* const surface, ndInt32 size)
{
	ndFloat32 spacing = 2.0f * surface->GetParticleRadius();
	ndArray<ndVector>& veloc = surface->GetVelocity();
	ndArray<ndVector>& posit = surface->GetPositions();

	ndVector v(ndVector::m_zero);
	for (ndInt32 z = 0; z < size; z++)
	{
		for (ndInt32 y = 0; y < size; y++)
		{
			for (ndInt32 x = 0; x < size; x++)
			{
				if (x < 1 ||
					y < 1 ||
					z < 1 ||
					x >= (size - 1) ||
					y >= (size - 1) ||
					z >= (size - 1))
				{
					const ndVector p(matrix.TransformVector(ndVector((ndFloat32)x * spacing, (ndFloat32)y * spacing, (ndFloat32)z * spacing, ndFloat32(1.0f))));
					posit.PushBack(p);
					veloc.PushBack(v);
				}
			}
		}
	}
}

static void BuildSphere(const ndMatrix& matrix, ndIsoSurfaceParticleVolume* const surface, ndInt32 size)
{
	ndFloat32 spacing = 2.0f * surface->GetParticleRadius();
	ndArray<ndVector>& veloc = surface->GetVelocity();
	ndArray<ndVector>& posit = surface->GetPositions();

	ndVector v(ndVector::m_zero);
	ndInt32 radius2 = (size / 2) * (size / 2);
	for (ndInt32 z = 0; z < size; z++)
	{
		for (ndInt32 y = 0; y < size; y++)
		{
			for (ndInt32 x = 0; x < size; x++)
			{
				ndInt32 x0 = x - size / 2;
				ndInt32 y0 = y - size / 2;
				ndInt32 z0 = z - size / 2;
				ndFloat32 tesRadius = ndFloat32 (x0 * x0 + y0 * y0 + z0 * z0);
				if (tesRadius < (ndFloat32)radius2)
				{
					const ndVector p(matrix.TransformVector(ndVector((ndFloat32)x * spacing, (ndFloat32)y * spacing, (ndFloat32)z * spacing, ndFloat32(1.0f))));
					posit.PushBack(p);
					veloc.PushBack(v);
				}
			}
		}
	}
}

static void AddWaterVolume(ndDemoEntityManager* const scene, const ndMatrix& location)
{
	ndSetRandSeed(1);
	ndPhysicsWorld* const world = scene->GetWorld();

	ndMatrix matrix(location);
	ndVector floor(FindFloor(*world, matrix.m_posit, 200.0f));
	matrix.m_posit = floor;
	matrix.m_posit.m_w = 1.0f;

	//ndFloat32 diameter = 0.25f;
	//ndFloat32 diameter = 0.15f;
	//ndFloat32 diameter = 0.125f;
	ndFloat32 diameter = 1.0f;
	ndIsoSurfaceParticleVolume* const fluidObject = new ndIsoSurfaceParticleVolume(diameter * 0.5f);
	ndWaterVolumeEntity* const entity = new ndWaterVolumeEntity(scene, matrix, ndVector(20.0f, 10.0f, 20.0f, 0.0f), fluidObject);

	fluidObject->SetNotifyCallback(new ndWaterVolumeCallback(scene, entity));
	fluidObject->SetMatrix(matrix);
	fluidObject->SetParticleRadius(diameter * 0.5f);
	
	//ndInt32 particleCountPerAxis = 1;
	//ndInt32 particleCountPerAxis = 2;
	//ndInt32 particleCountPerAxis = 3;
	ndInt32 particleCountPerAxis = 4;
	//ndInt32 particleCountPerAxis = 8;
	//ndInt32 particleCountPerAxis = 10;
	//ndInt32 particleCountPerAxis = 32;
	//ndInt32 particleCountPerAxis = 40;
	//ndInt32 particleCountPerAxis = 64;
	ndFloat32 spacing = diameter;
	
	ndFloat32 offset = spacing * (ndFloat32)particleCountPerAxis / 2.0f;
	ndVector origin(-offset, 1.0f, -offset, ndFloat32(0.0f));
	
	matrix.m_posit += origin;
	matrix.m_posit = ndVector (2.0f, 2.0f, 2.0f, 1.0f);
	
	BuildBox(matrix, fluidObject, particleCountPerAxis);
	//BuildHollowBox(matrix, fluidObject, particleCountPerAxis);
	//BuildSphere(matrix, fluidObject, particleCountPerAxis);
	
	matrix.m_posit.m_z -= 15.0f;
	//BuildHollowBox(matrix, fluidObject, particleCountPerAxis);
	
	// make sure we have the first surface generated before rendering.
	fluidObject->UpdateIsoSurface();

	// add particle volume to world
	scene->AddEntity(entity);
	ndSharedPtr<ndBody> body(fluidObject);
	world->AddBody(body);
}

void ndBasicParticleFluid (ndDemoEntityManager* const scene)
{
	// build a floor
	BuildFlatPlane(scene, true);

	ndMatrix location(ndGetIdentityMatrix());

	// adding a water volume
	AddWaterVolume(scene, location);

	ndQuaternion rot;
	ndVector origin(-10.0f, 5.0f, 3.0f, 1.0f);
	scene->SetCameraMatrix(rot, origin);
}
