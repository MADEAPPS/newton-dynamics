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
#include "ndPhysicsUtils.h"
#include "ndPhysicsWorld.h"
#include "ndMakeStaticMap.h"
#include "ndContactCallback.h"
#include "ndDemoEntityNotify.h"
#include "ndDemoEntityManager.h"
#include "ndArchimedesBuoyancyVolume.h"

static void AddTrigger(ndDemoEntityManager* const scene)
{
	ndPhysicsWorld* const world = scene->GetWorld();

	ndShapeInstance box(new ndShapeBox(20.0f, 10.0f, 20.0f));
	ndMatrix uvMatrix(ndGetIdentityMatrix());
	uvMatrix[0][0] *= 1.0f / 20.0f;
	uvMatrix[1][1] *= 1.0f / 10.0f;
	uvMatrix[2][2] *= 1.0f / 20.0f;
	uvMatrix.m_posit = ndVector(0.5f, 0.5f, 0.5f, 1.0f);
	ndSharedPtr<ndDemoMeshInterface>geometry (new ndDemoMesh("trigger", scene->GetShaderCache(), &box, "metal_30.png", "metal_30.png", "logo_php.png", 0.5f, uvMatrix));

	ndVector floor(FindFloor(*world, ndVector(0.0f, 100.0f, 0.0f, 0.0f), 200.0f));
	ndMatrix matrix(ndGetIdentityMatrix());
	matrix.m_posit = floor;
	matrix.m_posit.m_w = 1.0f;
	matrix.m_posit.m_y += 2.0f;

	ndSharedPtr<ndDemoEntity>entity = new ndDemoEntity(matrix);
	entity->SetMesh(geometry);

	ndSharedPtr<ndBody> body (new ndArchimedesBuoyancyVolume());
	body->SetNotifyCallback(new ndDemoEntityNotify(scene, entity));
	body->SetMatrix(matrix);
	body->GetAsBodyKinematic()->SetCollisionShape(box);

	world->AddBody(body);
	scene->AddEntity(entity);
}

static void AddBox(ndDemoEntityManager* const scene, const ndMatrix& origin, ndFloat32 density, ndFloat32 mass)
{
	ndBodyKinematic* const body = AddBox(scene, origin, mass, 1.0f, 1.0f, 1.0f);
	ndShapeMaterial material;
	material.m_userParam[ndDemoContactCallback::m_density].m_floatData = density;
	body->GetCollisionShape().SetMaterial(material);
}

static void AddSphere1(ndDemoEntityManager* const scene, const ndMatrix& origin, ndFloat32 density, ndFloat32 mass)
{
	ndBodyKinematic* const body = AddSphere(scene, origin, mass, 0.5f);
	ndShapeMaterial material;
	material.m_userParam[ndDemoContactCallback::m_density].m_floatData = density;
	body->GetCollisionShape().SetMaterial(material);
}

static void AddCapsule(ndDemoEntityManager* const scene, const ndMatrix& origin, ndFloat32 density, ndFloat32 mass)
{
	ndBodyKinematic* const body = AddCapsule(scene, origin, mass, 0.5f, 0.5f, 1.0f);
	ndShapeMaterial material;
	material.m_userParam[ndDemoContactCallback::m_density].m_floatData = density;
	body->GetCollisionShape().SetMaterial(material);
}

static void AddConvexHull(ndDemoEntityManager* const scene, const ndMatrix& origin, const ndInt32 segments, ndFloat32 radius, ndFloat32 high, ndFloat32 density, ndFloat32 mass)
{
	ndBodyKinematic* const body = AddConvexHull(scene, origin, mass, radius, high, segments);
	ndShapeMaterial material;
	material.m_userParam[ndDemoContactCallback::m_density].m_floatData = density;
	body->GetCollisionShape().SetMaterial(material);
}

void ndBasicTrigger (ndDemoEntityManager* const scene)
{
	// build a floor
	BuildFloorBox(scene, ndGetIdentityMatrix());

	// build a floor
	AddTrigger(scene);

	class PlaceMatrix : public ndMatrix
	{
		public:
		PlaceMatrix(ndFloat32 x, ndFloat32 y, ndFloat32 z)
			:ndMatrix(ndGetIdentityMatrix())
		{
			m_posit.m_x = x;
			m_posit.m_y = y;
			m_posit.m_z = z;
		}
	};

	AddBox(scene, PlaceMatrix(0.0f, 20.0f, -3.0f), 0.6f, 10.0f);
	AddSphere1(scene, PlaceMatrix(0.0f, 5.0f, 0.0f), 0.5f, 10.0f);
	AddCapsule(scene, PlaceMatrix(0.0f, 5.0f, 3.0f), 0.7f, 10.0f);
	AddConvexHull(scene, PlaceMatrix(-2.0f, 5.0f, -2.0f), 7, 1.0f, 1.5f, 0.8f, 10.0f);
	AddConvexHull(scene, PlaceMatrix(-2.0f, 5.0f,  2.0f), 21, 1.0f, 1.5f, 0.7f, 10.0f);
	AddConvexHull(scene, PlaceMatrix( 2.0f, 5.0f,  3.0f), 210, 1.0f, 1.5f, 0.9f, 10.0f);

	ndQuaternion rot;
	ndVector origin(-40.0f, 5.0f, 0.0f, 1.0f);
	scene->SetCameraMatrix(rot, origin);
}
