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
	ndWaterVolumeEntity(ndDemoEntityManager* const scene, const dMatrix& location, const dVector& size)
		:ndDemoEntity(location, nullptr)
	{
		ndShapeInstance box(new ndShapeBox(20.0f, 10.0f, 20.0f));
		dMatrix uvMatrix(dGetIdentityMatrix());
		uvMatrix[0][0] *= 1.0f / 20.0f;
		uvMatrix[1][1] *= 1.0f / 10.0f;
		uvMatrix[2][2] *= 1.0f / 20.0f;
		uvMatrix.m_posit = dVector(0.5f, 0.5f, 0.5f, 1.0f);

		ndDemoMesh* const geometry = new ndDemoMesh("fluidVolume", scene->GetShaderCache(), &box, "metal_30.tga", "metal_30.tga", "logo_php.tga", 0.5f, uvMatrix);
		SetMesh(geometry, dGetIdentityMatrix());

		scene->AddEntity(this);
		geometry->Release();
	}
};


static void AddWaterVolume(ndDemoEntityManager* const scene, const dMatrix& location)
{
	ndPhysicsWorld* const world = scene->GetWorld();

	dMatrix matrix(location);
	dVector floor(FindFloor(*world, matrix.m_posit, 200.0f));
	matrix.m_posit = floor;
	matrix.m_posit.m_w = 1.0f;
	matrix.m_posit.m_y += 4.0f;
	ndWaterVolumeEntity* const entity = new ndWaterVolumeEntity(scene, matrix, dVector(20.0f, 10.0f, 20.0f, 0.0f));

	ndBodySphFluid* const fluidObject = new ndBodySphFluid();
	fluidObject->SetMatrix(matrix);

	fluidObject->AddParticle(0.1f, dVector(matrix.m_posit), dVector::m_zero);
	

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
	dVector origin(-40.0f, 5.0f, 0.0f, 0.0f);
	scene->SetCameraMatrix(rot, origin);
}
