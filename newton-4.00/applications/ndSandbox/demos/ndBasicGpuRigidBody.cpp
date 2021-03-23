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

//class ndZeroGravityNotify : public ndDemoEntityNotify
//{
//	public:
//	ndZeroGravityNotify(ndDemoEntityManager* const manager, ndDemoEntity* const entity)
//		:ndDemoEntityNotify(manager, entity)
//	{
//	}
//
//	//void OnTranform(dInt32 threadIndex, const dMatrix& matrix)
//	//{
//	//	dMatrix parentMatrix(m_chassis->GetMatrix());
//	//	dMatrix localMatrix(matrix * parentMatrix.Inverse());
//	//
//	//	dQuaternion rot(localMatrix);
//	//	dScopeSpinLock lock(m_entity->GetLock());
//	//	m_entity->SetMatrixUsafe(rot, localMatrix.m_posit);
//	//}
//};

static void AddShape(ndDemoEntityManager* const scene, const dMatrix& location,
	const ndShapeInstance& shape, dFloat32 mass, dFloat32 density)
{
	ndDemoMesh* const mesh = new ndDemoMesh("shape", scene->GetShaderCache(), &shape, "marble.tga", "marble.tga", "marble.tga");

	dMatrix matrix(location);
	ndPhysicsWorld* const world = scene->GetWorld();

	//dVector floor(FindFloor(*world, matrix.m_posit + dVector(0.0f, 100.0f, 0.0f, 0.0f), 200.0f));
	dVector floor(dVector::m_zero);
	matrix.m_posit.m_y = floor.m_y + 10.0f;

	ndBodyDynamic* const body = new ndBodyDynamic();
	ndDemoEntity* const entity = new ndDemoEntity(matrix, nullptr);
	entity->SetMesh(mesh, dGetIdentityMatrix());

	body->SetNotifyCallback(new ndDemoEntityNotify(scene, entity));
	body->SetMatrix(matrix);
	body->SetCollisionShape(shape);
	body->SetMassMatrix(mass, shape);

	ndBodyNotify* const notification = body->GetNotifyCallback();
	notification->SetGravity(dVector::m_zero);

	// save the density with the body shape.
	ndShapeMaterial material;
	material.m_userParam[0].m_floatData = density;
	body->GetCollisionShape().SetMaterial(material);

	world->AddBody(body);
	scene->AddEntity(entity);

	mesh->Release();
}

//static void AddSphere(ndDemoEntityManager* const scene, const dVector& origin, dFloat32 density)
//{
//	dFloat32 diameter = 0.5f;
//	ndShapeInstance shape(new ndShapeSphere(diameter));
//
//	dMatrix matrix(dPitchMatrix(15.0f*dDegreeToRad) * dRollMatrix(15.0f*dDegreeToRad));
//	matrix.m_posit = origin;
//
//	AddShape(scene, matrix, shape, 10.0f, density);
//}
//
//static void AddCapsule(ndDemoEntityManager* const scene, const dVector& origin, dFloat32 density)
//{
//	dFloat32 diameter = 1.0f;
//	ndShapeInstance shape(new ndShapeCapsule(diameter * 0.5f, diameter * 0.5f, diameter * 1.0f));
//
//	//dMatrix matrix(dRollMatrix(90.0f * dDegreeToRad));
//	dMatrix matrix(dPitchMatrix(35.0f*dDegreeToRad) * dRollMatrix(25.0f*dDegreeToRad));
//	matrix.m_posit = origin;
//
//	AddShape(scene, matrix, shape, 10.0f, density);
//}

//static void AddConvexHull(ndDemoEntityManager* const scene, const dVector& origin, const dInt32 segments, dFloat32 density)
//{
//	dVector points[1024];
//	dInt32 count = 0;
//	for (dInt32 i = 0; i < segments; i++)
//	{
//		dFloat32 y = 0.7f * dCos((dFloat32(2.0f) * dPi) * i / segments);
//		dFloat32 z = 0.7f * dSin((dFloat32(2.0f) * dPi) * i / segments);
//		points[count++] = dVector(-0.5f, 0.7f * y, 0.7f* z, 0.0f);
//		points[count++] = dVector( 0.5f, 0.7f * y, 0.7f* z, 0.0f);
//		//points[count++] = dVector(0.25f, y, z, 0.0f);
//		points[count++] = dVector(-0.25f, y, z, 0.0f);
//	}
//
//	//ndShapeInstance shape(new ndShapeBox(1.0f, 2.0f, 0.7f));
//	ndShapeInstance shape(new ndShapeConvexHull(count, sizeof (dVector), 0.0f, &points[0].m_x));
//	dMatrix matrix(dPitchMatrix(135.0f*dDegreeToRad) * dRollMatrix(75.0f*dDegreeToRad));
//	matrix.m_posit = origin;
//
//	AddShape(scene, matrix, shape, 10.0f, density);
//}

static void AddBox(ndDemoEntityManager* const scene, const dVector& origin, dFloat32 density)
{
	ndShapeInstance shape(new ndShapeBox(1.0f, 2.0f, 0.7f));
	dMatrix matrix(dPitchMatrix(10.0f*dDegreeToRad) * dRollMatrix(150.0f*dDegreeToRad));
	matrix.m_posit = origin;

	AddShape(scene, matrix, shape, 10.0f, density);
}

void ndBasicGpuRigidBody(ndDemoEntityManager* const scene)
{
	// build a floor
	//BuildFloorBox(scene);

	AddBox(scene, dVector(0.0f, 0.0f, -3.0f, 1.0f), 0.6f);

	dQuaternion rot;
	dVector origin(-40.0f, 5.0f, 0.0f, 0.0f);
	scene->SetCameraMatrix(rot, origin);
}
