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

static void AddWaterVolume(ndDemoEntityManager* const scene, const dMatrix& location)
{
	ndPhysicsWorld* const world = scene->GetWorld();

	ndShapeInstance box(new ndShapeBox(20.0f, 10.0f, 20.0f));
	dMatrix uvMatrix(dGetIdentityMatrix());
	uvMatrix[0][0] *= 1.0f / 20.0f;
	uvMatrix[1][1] *= 1.0f / 10.0f;
	uvMatrix[2][2] *= 1.0f / 20.0f;
	uvMatrix.m_posit = dVector(0.5f, 0.5f, 0.5f, 1.0f);
	ndDemoMesh* const geometry = new ndDemoMesh("trigger", scene->GetShaderCache(), &box, "metal_30.tga", "metal_30.tga", "logo_php.tga", 0.5f, uvMatrix);

	dMatrix matrix(location);
	dVector floor(FindFloor(*world, matrix.m_posit, 200.0f));
	matrix.m_posit = floor;
	matrix.m_posit.m_w = 1.0f;
	matrix.m_posit.m_y += 4.0f;

	ndDemoEntity* const entity = new ndDemoEntity(matrix, nullptr);
	entity->SetMesh(geometry, dGetIdentityMatrix());

	ndBodySphFluid* const fluidObject = new ndBodySphFluid();
	fluidObject->SetMatrix(matrix);
	world->AddBody(fluidObject);

	//body->SetCollisionShape(box);
	//body->SetNotifyCallback(new ndDemoEntityNotify(scene, entity));

	scene->AddEntity(entity);
	geometry->Release();
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
