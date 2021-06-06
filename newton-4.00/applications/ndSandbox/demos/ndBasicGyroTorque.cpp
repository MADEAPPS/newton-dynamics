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
#include "ndDemoCameraManager.h"

class ndAsymetricInertiaBody: public ndBodyDynamic
{
	public:
	ndAsymetricInertiaBody()
		:ndBodyDynamic()
		,m_principalAxis(dGetIdentityMatrix())
	{
	}
	
	virtual void SetMassMatrix(dFloat32 mass, const dMatrix& inertia)
	{
		m_principalAxis = inertia;
		dVector eigenValues(m_principalAxis.EigenVectors());
		dMatrix massMatrix(dGetIdentityMatrix());
		massMatrix[0][0] = eigenValues[0];
		massMatrix[1][1] = eigenValues[1];
		massMatrix[2][2] = eigenValues[2];
		ndBodyDynamic::SetMassMatrix(mass, massMatrix);
	}

	virtual dMatrix CalculateInvInertiaMatrix() const
	{
		dMatrix matrix(m_principalAxis * m_matrix);
		matrix.m_posit = dVector::m_wOne;
		dMatrix diagonal(dGetIdentityMatrix());
		diagonal[0][0] = m_invMass[0];
		diagonal[1][1] = m_invMass[1];
		diagonal[2][2] = m_invMass[2];
		return matrix * diagonal * matrix.Inverse();
	}
	
	dMatrix m_principalAxis;
};

static void DzhanibekovEffect(ndDemoEntityManager* const scene, dFloat32 mass, dFloat32 angularSpeed, const dVector& origin)
{
	dMatrix matrix(dGetIdentityMatrix());
	matrix.m_posit = origin;
	matrix.m_posit.m_w = 1.0f;

	ndPhysicsWorld* const world = scene->GetWorld();

	//dVector floor(FindFloor(*world, matrix.m_posit + dVector(0.0f, 100.0f, 0.0f, 0.0f), 200.0f));
	matrix.m_posit.m_y += 5.0f;

	ndShapeInstance shape(new ndShapeBox(2.0f, 0.5f, 1.0));
	ndDemoMesh* const mesh = new ndDemoMesh("shape", scene->GetShaderCache(), &shape, "marble.tga", "marble.tga", "marble.tga");

	dVector omega(0.1f, 0.0f, angularSpeed, 0.0f);
	ndBodyDynamic* const body = new ndBodyDynamic();
	ndDemoEntity* const entity = new ndDemoEntity(matrix, nullptr);
	entity->SetMesh(mesh, dGetIdentityMatrix());
	body->SetNotifyCallback(new ndDemoEntityNotify(scene, entity, 0.0f));

	body->SetOmega(omega);
	body->SetMatrix(matrix);
	body->SetCollisionShape(shape);
	body->SetMassMatrix(mass, shape);

	world->AddBody(body);
	scene->AddEntity(entity);

	mesh->Release();
}

static void Phitop(ndDemoEntityManager* const scene, dFloat32 mass, dFloat32 angularSpeed, const dVector& origin)
{
	dMatrix matrix(dPitchMatrix(15.0f * dDegreeToRad));
	matrix.m_posit = origin;
	matrix.m_posit.m_w = 1.0f;

	ndPhysicsWorld* const world = scene->GetWorld();

	//dVector floor(FindFloor(*world, matrix.m_posit + dVector(0.0f, 100.0f, 0.0f, 0.0f), 200.0f));
	matrix.m_posit.m_y += 0.5f;

	ndShapeInstance shape(new ndShapeSphere(1.0f));
	shape.SetScale(dVector (0.5f, 0.5f, 1.0f, 0.0f));

	ndDemoMesh* const mesh = new ndDemoMesh("shape", scene->GetShaderCache(), &shape, "marble.tga", "marble.tga", "marble.tga");

	dVector omega(0.0f, angularSpeed, 0.0f, 0.0f);
	ndBodyDynamic* const body = new ndBodyDynamic();
	ndDemoEntity* const entity = new ndDemoEntity(matrix, nullptr);
	entity->SetMesh(mesh, dGetIdentityMatrix());
	body->SetNotifyCallback(new ndDemoEntityNotify(scene, entity));

	body->SetOmega(omega);
	body->SetMatrix(matrix);
	body->SetCollisionShape(shape);
	body->SetMassMatrix(mass, shape);

	world->AddBody(body);
	scene->AddEntity(entity);

	mesh->Release();
}

static void RattleBack(ndDemoEntityManager* const scene, dFloat32 mass, const dVector& origin)
{
	dMatrix matrix(dPitchMatrix(15.0f * dDegreeToRad));
	matrix.m_posit = origin;
	matrix.m_posit.m_w = 1.0f;

	ndPhysicsWorld* const world = scene->GetWorld();

	//dVector floor(FindFloor(*world, matrix.m_posit + dVector(0.0f, 100.0f, 0.0f, 0.0f), 200.0f));
	matrix.m_posit.m_y += 0.25f;

	dMatrix shapeMatrix(dYawMatrix(5.0f * dDegreeToRad));

	ndShapeInstance shape(new ndShapeSphere(1.0f));
	shape.SetLocalMatrix(shapeMatrix);
	shape.SetScale(dVector(0.3f, 0.25f, 1.0f, 0.0f));

	ndDemoMesh* const mesh = new ndDemoMesh("shape", scene->GetShaderCache(), &shape, "marble.tga", "marble.tga", "marble.tga");

	ndBodyDynamic* const body = new ndAsymetricInertiaBody();
	ndDemoEntity* const entity = new ndDemoEntity(matrix, nullptr);
	entity->SetMesh(mesh, dGetIdentityMatrix());
	body->SetNotifyCallback(new ndDemoEntityNotify(scene, entity));

	body->SetMatrix(matrix);
	body->SetCollisionShape(shape);
	body->SetMassMatrix(mass, shape);
	body->SetCentreOfMass(dVector(0.0f, -0.1f, 0.0f, 0.0f));

	world->AddBody(body);
	scene->AddEntity(entity);

	mesh->Release();
}

static void PrecessingTop(ndDemoEntityManager* const scene, const dVector& origin)
{
	ndPhysicsWorld* const world = scene->GetWorld();

	ndShapeInstance shape(new ndShapeCone(0.7f, 1.0f));
	shape.SetLocalMatrix(dRollMatrix(-90.0f * dDegreeToRad));

	dMatrix matrix(dPitchMatrix(15.0f * dDegreeToRad));
	matrix.m_posit = origin;
	matrix.m_posit.m_w = 1.0f;
	ndDemoMesh* const geometry = new ndDemoMesh("shape", scene->GetShaderCache(), &shape, "marble.tga", "marble.tga", "marble.tga");
	ndDemoEntity* const entity = new ndDemoEntity(matrix, nullptr);
	entity->SetMesh(geometry, dGetIdentityMatrix());

	//dVector floor(FindFloor(*world, matrix.m_posit + dVector(0.0f, 100.0f, 0.0f, 0.0f), 200.0f));
	matrix.m_posit.m_y += 1.0f;

	const dFloat32 mass = 1.0f;
	ndBodyDynamic* const body = new ndBodyDynamic();
	body->SetNotifyCallback(new ndDemoEntityNotify(scene, entity));
	body->SetMatrix(matrix);
	body->SetCollisionShape(shape);
	body->SetMassMatrix(mass, shape);
	body->SetOmega(matrix.m_up.Scale(40.0f));

	world->AddBody(body);
	scene->AddEntity(entity);
	geometry->Release();
}

void ndBasicAngularMomentum (ndDemoEntityManager* const scene)
{
	// build a floor
	BuildFloorBox(scene);

	DzhanibekovEffect(scene, 10.0f, 5.0f, dVector(0.0f, 0.0f, -2.0f, 0.0f));
	DzhanibekovEffect(scene, 10.0f, -5.0f, dVector(0.0f, 0.0f, 0.0f, 0.0f));
	DzhanibekovEffect(scene, 10.0f, 10.0f, dVector(0.0f, 0.0f, 2.0f, 0.0f));

	Phitop(scene, 10.0f, 25.0f, dVector(10.0f, 0.0f, -4.0f, 0.0f));
	Phitop(scene, 10.0f, -25.0f, dVector(10.0f, 0.0f, 0.0f, 0.0f));
	Phitop(scene, 10.0f, 35.0f, dVector(10.0f, 0.0f, 4.0f, 0.0f));

	PrecessingTop(scene, dVector(5.0f, 0.0f, -4.0f, 0.0f));
	PrecessingTop(scene, dVector(5.0f, 0.0f, 0.0f, 0.0f));
	PrecessingTop(scene, dVector(5.0f, 0.0f, 4.0f, 0.0f));

	RattleBack(scene, 10.0f, dVector(0.0f, 0.0f, -4.0f, 0.0f));
	RattleBack(scene, 10.0f, dVector(0.0f, 0.0f, 0.0f, 0.0f));
	RattleBack(scene, 10.0f, dVector(0.0f, 0.0f, 4.0f, 0.0f));
	
	scene->GetCameraManager()->SetPickMode(true);

	dQuaternion rot;
	dVector origin(-15.0f, 5.0f, 0.0f, 0.0f);
	scene->SetCameraMatrix(rot, origin);
}
