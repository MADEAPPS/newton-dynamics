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

static void AddTrigger(ndDemoEntityManager* const scene)
{
	ndPhysicsWorld* const world = scene->GetWorld();

	ndShapeInstance box(new ndShapeBox(20.0f, 10.0f, 20.0f));
	dMatrix uvMatrix(dGetIdentityMatrix());
	uvMatrix[0][0] *= 1.0f / 20.0f;
	uvMatrix[1][1] *= 1.0f / 10.0f;
	uvMatrix[2][2] *= 1.0f / 20.0f;
	uvMatrix.m_posit = dVector(0.5f, 0.5f, 0.5f, 1.0f);
	ndDemoMesh* const geometry = new ndDemoMesh("trigger", scene->GetShaderCache(), &box, "metal_30.tga", "metal_30.tga", "logo_php.tga", 0.5f, uvMatrix);

	dVector floor(FindFloor(*world, dVector(0.0f, 100.0f, 0.0f, 0.0f), 200.0f));
	dMatrix matrix(dGetIdentityMatrix());
	matrix.m_posit = floor;
	matrix.m_posit.m_w = 1.0f;
	matrix.m_posit.m_y += 2.0f;

	ndDemoEntity* const entity = new ndDemoEntity(matrix, nullptr);
	entity->SetMesh(geometry, dGetIdentityMatrix());

	ndBodyTriggerVolume* const body = new ndArchimedesBuoyancyVolume();
	body->SetNotifyCallback(new ndDemoEntityNotify(scene, entity));
	body->SetMatrix(matrix);
	body->SetCollisionShape(box);

	world->AddBody(body);

	scene->AddEntity(entity);
	geometry->Release();
}

static void AddBox(ndDemoEntityManager* const scene, const dVector& origin, dFloat32 density, dFloat32 mass)
{
	ndBodyKinematic* const body = AddBox(scene, origin, mass, 1.0f, 1.0f, 1.0f);
	ndShapeMaterial material;
	material.m_userParam[0].m_floatData = density;
	body->GetCollisionShape().SetMaterial(material);
}

static void AddSphere1(ndDemoEntityManager* const scene, const dVector& origin, dFloat32 density, dFloat32 mass)
{
	ndBodyKinematic* const body = AddSphere(scene, origin, mass, 0.5f);
	ndShapeMaterial material;
	material.m_userParam[0].m_floatData = density;
	body->GetCollisionShape().SetMaterial(material);
}

static void AddCapsule(ndDemoEntityManager* const scene, const dVector& origin, dFloat32 density, dFloat32 mass)
{
	ndBodyKinematic* const body = AddCapsule(scene, origin, mass, 0.5f, 0.5f, 1.0f);
	ndShapeMaterial material;
	material.m_userParam[0].m_floatData = density;
	body->GetCollisionShape().SetMaterial(material);
}

static void AddConvexHull(ndDemoEntityManager* const scene, const dVector& origin, const dInt32 segments, dFloat32 radius, dFloat32 high, dFloat32 density, dFloat32 mass)
{
	ndBodyKinematic* const body = AddConvexHull(scene, origin, mass, radius, high, segments);
	ndShapeMaterial material;
	material.m_userParam[0].m_floatData = density;
	body->GetCollisionShape().SetMaterial(material);
}

void ndBasicTrigger (ndDemoEntityManager* const scene)
{
	// build a floor
	BuildFloorBox(scene);

	// build a floor
	AddTrigger(scene);

	AddBox(scene, dVector(0.0f, 5.0f, -3.0f, 1.0f), 0.6f, 10.0f);
	AddSphere1(scene, dVector(0.0f, 5.0f, 0.0f, 1.0f), 0.5f, 10.0f);
	AddCapsule(scene, dVector(0.0f, 5.0f, 3.0f, 1.0f), 0.7f, 10.0f);
	AddConvexHull(scene, dVector(-2.0f, 5.0f, -2.0f, 1.0f), 7, 1.0f, 1.5f, 0.8f, 10.0f);
	AddConvexHull(scene, dVector(-2.0f, 5.0f,  2.0f, 1.0f), 21, 1.0f, 1.5f, 0.7f, 10.0f);
	AddConvexHull(scene, dVector( 2.0f, 5.0f,  3.0f, 1.0f), 210, 1.0f, 1.5f, 0.9f, 10.0f);

	dQuaternion rot;
	dVector origin(-40.0f, 5.0f, 0.0f, 0.0f);
	scene->SetCameraMatrix(rot, origin);
}
