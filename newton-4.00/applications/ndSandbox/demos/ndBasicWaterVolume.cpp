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

class ndWaterVolumeEntity : public ndDemoEntity
{
	public:
	ndWaterVolumeEntity(ndDemoEntityManager* const scene, const dMatrix& location, const dVector& size, ndBodySphFluid* const fluidBody, dFloat32 radius)
		:ndDemoEntity(location, nullptr)
		,m_fluidBody(fluidBody)
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
	}

	~ndWaterVolumeEntity()
	{
		m_meshParticle->Release();
	}

	void Render(dFloat32 timeStep, ndDemoEntityManager* const scene, const dMatrix& matrix) const
	{
		dMatrix nodeMatrix(m_matrix * matrix);

		const dArray<dVector>& positions = m_fluidBody->GetPositions();
		m_meshParticle->SetParticles(positions.GetCount(), &positions[0]);
		m_meshParticle->Render(scene, nodeMatrix);
		//ndDemoEntity::Render(timeStep, scene, matrix);
	}

	ndBodySphFluid* m_fluidBody;
	ndDemoMeshIntance* m_meshParticle;
};

class ndWaterVolumeCallback: public ndDemoEntityNotify
{
	public:
	ndWaterVolumeCallback(ndDemoEntityManager* const manager, ndDemoEntity* const entity)
		:ndDemoEntityNotify(manager, entity)
	{
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

	dInt32 particleCountPerAxis = 32;
	//dInt32 particleCountPerAxis = 12;
	dFloat32 spacing = diameter * 1.0f;

	dFloat32 offset = spacing * particleCountPerAxis / 2.0f;
	dVector origin(-offset, 1.0f, -offset, dFloat32(0.0f));
	origin += matrix.m_posit;
	
	for (dInt32 z = 0; z < particleCountPerAxis; z++)
	//for (dInt32 z = 0; z < 1; z++)
	{
		for (dInt32 y = 0; y < particleCountPerAxis; y++)
		//for (dInt32 y = 0; y < 1; y++)
		{
			for (dInt32 x = 0; x < particleCountPerAxis; x++)
			{
				dVector posit(x * spacing, y * spacing, z * spacing, 0.0f);
				fluidObject->AddParticle(0.1f, origin + posit, dVector::m_zero);

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
	BuildFlatPlane(scene, true);

	dMatrix location(dGetIdentityMatrix());

	// adding a water volume
	AddWaterVolume(scene, location);

	dQuaternion rot;
	dVector origin(-15.0f, 4.0f, 0.0f, 0.0f);
	scene->SetCameraMatrix(rot, origin);
}
