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
#include "ndDemoInstanceEntity.h"


static void AddShape(ndDemoEntityManager* const scene, const ndMatrix& location,
	//ndDemoInstanceEntity* const rootEntity,
	ndSharedPtr<ndDemoEntity>& root,
	const ndShapeInstance& shape, ndFloat32 mass, ndFloat32 density)
{
	ndMatrix matrix(location);
	ndPhysicsWorld* const world = scene->GetWorld();

	ndSharedPtr<ndBody> body (new ndBodyDynamic());
	ndSharedPtr<ndDemoEntity> entity (new ndDemoEntity(matrix));
	root->AddChild(entity);

	body->SetNotifyCallback(new ndDemoEntityNotify(scene, entity));
	body->SetMatrix(matrix);
	body->GetAsBodyDynamic()->SetCollisionShape(shape);
	body->GetAsBodyDynamic()->SetMassMatrix(mass, shape);

	const ndVector omega(ndGaussianRandom(0.0f, 2.0f), ndGaussianRandom(0.0f, 4.0f), ndGaussianRandom(0.0f, 3.0f), 0.0f);
	body->SetOmega(omega);

	ndBodyNotify* const notification = body->GetNotifyCallback();
	notification->SetGravity(ndVector::m_zero);

	// save the density with the body shape.
	ndShapeMaterial material;
	material.m_userParam[ndDemoContactCallback::m_density].m_floatData = density;
	body->GetAsBodyDynamic()->GetCollisionShape().SetMaterial(material);

	world->AddBody(body);
}

//static void AddSphere(ndDemoEntityManager* const scene, const ndVector& origin, ndFloat32 density)
//{
//	ndFloat32 diameter = 0.5f;
//	ndShapeInstance shape(new ndShapeSphere(diameter));
//
//	ndMatrix matrix(ndPitchMatrix(15.0f*ndDegreeToRad) * ndRollMatrix(15.0f*ndDegreeToRad));
//	matrix.m_posit = origin;
//
//	AddShape(scene, matrix, shape, 10.0f, density);
//}
//
//static void AddCapsule(ndDemoEntityManager* const scene, const ndVector& origin, ndFloat32 density)
//{
//	ndFloat32 diameter = 1.0f;
//	ndShapeInstance shape(new ndShapeCapsule(diameter * 0.5f, diameter * 0.5f, diameter * 1.0f));
//
//	//ndMatrix matrix(ndRollMatrix(90.0f * ndDegreeToRad));
//	ndMatrix matrix(ndPitchMatrix(35.0f*ndDegreeToRad) * ndRollMatrix(25.0f*ndDegreeToRad));
//	matrix.m_posit = origin;
//
//	AddShape(scene, matrix, shape, 10.0f, density);
//}

//static void AddConvexHull(ndDemoEntityManager* const scene, const ndVector& origin, const ndInt32 segments, ndFloat32 density)
//{
//	ndVector points[1024];
//	ndInt32 count = 0;
//	for (ndInt32 i = 0; i < segments; ++i)
//	{
//		ndFloat32 y = 0.7f * dCos((ndFloat32(2.0f) * dPi) * i / segments);
//		ndFloat32 z = 0.7f * dSin((ndFloat32(2.0f) * dPi) * i / segments);
//		points[count++] = ndVector(-0.5f, 0.7f * y, 0.7f* z, 0.0f);
//		points[count++] = ndVector( 0.5f, 0.7f * y, 0.7f* z, 0.0f);
//		//points[count++] = ndVector(0.25f, y, z, 0.0f);
//		points[count++] = ndVector(-0.25f, y, z, 0.0f);
//	}
//
//	//ndShapeInstance shape(new ndShapeBox(1.0f, 2.0f, 0.7f));
//	ndShapeInstance shape(new ndShapeConvexHull(count, sizeof (ndVector), 0.0f, &points[0].m_x));
//	ndMatrix matrix(ndPitchMatrix(135.0f*ndDegreeToRad) * ndRollMatrix(75.0f*ndDegreeToRad));
//	matrix.m_posit = origin;
//
//	AddShape(scene, matrix, shape, 10.0f, density);
//}

static void AddBox(ndDemoEntityManager* const scene, const ndVector& origin, ndFloat32 density, int count)
{
	ndShapeInstance shape(new ndShapeBox(1.0f, 2.0f, 0.7f));
	ndMatrix matrix(ndPitchMatrix(10.0f*ndDegreeToRad) * ndRollMatrix(150.0f*ndDegreeToRad));
	matrix.m_posit = origin;

	ndSharedPtr<ndDemoMeshIntance> geometry (new ndDemoMeshIntance("shape", scene->GetShaderCache(), &shape, "marble.png", "marble.png", "marble.png"));

	//ndDemoInstanceEntity* const rootEntity = new ndDemoInstanceEntity(geometry);
	ndSharedPtr<ndDemoEntity>rootEntity(new ndDemoInstanceEntity(geometry));
	scene->AddEntity(rootEntity);

	ndFloat32 step = 4.0f;
	for (ndInt32 j = 0; j < count; ++j)
	{
		for (ndInt32 k = 0; k < count; ++k)
		{
			for (ndInt32 i = 0; i < count; ++i)
			{
				ndVector posit(step * (ndFloat32)(i - count/2), step * (ndFloat32)j, step * (ndFloat32)(k - count / 2), 0.0f);
				ndQuaternion rotation(ndGaussianRandom(0.0f, 1.0f), ndGaussianRandom(0.0f, 1.0f), ndGaussianRandom(0.0f, 1.0f), ndGaussianRandom(0.0f, 1.0f) + 0.1f);
				ndMatrix location(ndCalculateMatrix(rotation, origin + posit));
				AddShape(scene, location, rootEntity, shape, 10.0f, density);
			}
		}
	}
}

void ndBasicGpuRigidBody(ndDemoEntityManager* const scene)
{
	// build a floor
	BuildFloorBox(scene, ndGetIdentityMatrix());

	//ndInt32 count = 50;
	//ndInt32 count = 40;
	//ndInt32 count = 32;
	//ndInt32 count = 28;
	ndInt32 count = 20;
	//ndInt32 count = 15;
	//ndInt32 count = 10;
	//ndInt32 count = 9;
	//ndInt32 count = 8;
	//ndInt32 count = 7;
	//ndInt32 count = 6;
	//ndInt32 count = 5;
	//ndInt32 count = 4;
	//ndInt32 count = 3;
	//ndInt32 count = 2;
	//ndInt32 count = 1;
	AddBox(scene, ndVector(0.0f, 1.0f, -3.0f, 1.0f), 1.0f, count);

	ndQuaternion rot;
	ndVector origin(-15.0f - 5.0f * (ndFloat32)count, 10.0f, 0.0f, 1.0f);
	scene->SetCameraMatrix(rot, origin);
}
