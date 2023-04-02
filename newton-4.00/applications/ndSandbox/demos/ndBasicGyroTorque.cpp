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
#include "ndDemoEntityNotify.h"
#include "ndDemoEntityManager.h"
#include "ndDemoCameraManager.h"

static void DzhanibekovEffect(ndDemoEntityManager* const scene, ndFloat32 mass, ndFloat32 angularSpeed, const ndVector& origin)
{
	ndMatrix matrix(ndGetIdentityMatrix());
	matrix.m_posit = origin;
	matrix.m_posit.m_w = 1.0f;

	ndPhysicsWorld* const world = scene->GetWorld();
	matrix.m_posit.m_y += 5.0f;

	ndShapeInstance shape(new ndShapeBox(2.0f, 0.5f, 1.0));
	ndSharedPtr<ndDemoMeshInterface> mesh (new ndDemoMesh("shape", scene->GetShaderCache(), &shape, "marble.tga", "marble.tga", "marble.tga"));

	ndVector omega(0.1f, 0.0f, angularSpeed, 0.0f);
	ndBodyKinematic* const body = new ndBodyDynamic();
	ndDemoEntity* const entity = new ndDemoEntity(matrix, nullptr);
	entity->SetMesh(mesh);
	body->SetNotifyCallback(new ndDemoEntityNotify(scene, entity, nullptr, 0.0f));

	body->SetOmega(omega);
	body->SetMatrix(matrix);
	body->SetCollisionShape(shape);
	body->SetMassMatrix(mass, shape);

	ndSharedPtr<ndBody> bodyPtr(body);
	world->AddBody(bodyPtr);
	scene->AddEntity(entity);
}

static void Phitop(ndDemoEntityManager* const scene, ndFloat32 mass, ndFloat32 angularSpeed, const ndVector& origin)
{
	ndMatrix matrix(ndPitchMatrix(15.0f * ndDegreeToRad));
	matrix.m_posit = origin;
	matrix.m_posit.m_w = 1.0f;

	ndPhysicsWorld* const world = scene->GetWorld();
	matrix.m_posit.m_y += 0.5f;

	ndShapeInstance shape(new ndShapeSphere(1.0f));
	shape.SetScale(ndVector (0.5f, 0.5f, 1.0f, 0.0f));

	ndSharedPtr<ndDemoMeshInterface> mesh(new ndDemoMesh("shape", scene->GetShaderCache(), &shape, "marble.tga", "marble.tga", "marble.tga"));

	ndVector omega(0.0f, angularSpeed, 0.0f, 0.0f);
	ndBodyKinematic* const body = new ndBodyDynamic();
	ndDemoEntity* const entity = new ndDemoEntity(matrix, nullptr);
	entity->SetMesh(mesh);
	body->SetNotifyCallback(new ndDemoEntityNotify(scene, entity));

	body->SetOmega(omega);
	body->SetMatrix(matrix);
	body->SetCollisionShape(shape);
	body->SetMassMatrix(mass, shape);

	ndSharedPtr<ndBody> bodyPtr(body);
	world->AddBody(bodyPtr);
	scene->AddEntity(entity);
}

static void RattleBack(ndDemoEntityManager* const scene, ndFloat32 mass, ndFloat32 speed, const ndVector& origin)
{
	ndMatrix matrix(ndYawMatrix(45.0f * ndDegreeToRad));

	matrix.m_posit = origin;
	matrix.m_posit.m_w = 1.0f;

	ndPhysicsWorld* const world = scene->GetWorld();

	ndMatrix shapeMatrix(ndYawMatrix(5.0f * ndDegreeToRad));
	//ndMatrix shapeMatrix(ndYawMatrix(10.0f * ndDegreeToRad));

	ndShapeInstance shape(new ndShapeSphere(1.0f));
	shape.SetScale(ndVector(0.3f, 0.25f, 1.0f, 0.0f));
	ndSharedPtr<ndDemoMeshInterface> mesh(new ndDemoMesh("shape", scene->GetShaderCache(), &shape, "marble.tga", "marble.tga", "marble.tga"));

	matrix = FindFloor(*world, matrix, shape, 200.0f);

	ndBodyKinematic* const body = new ndBodyDynamic();
	ndDemoEntity* const entity = new ndDemoEntity(matrix, nullptr);
	entity->SetMesh(mesh);
	body->SetNotifyCallback(new ndDemoEntityNotify(scene, entity));

	body->SetMatrix(matrix);
	body->SetCollisionShape(shape);
	// skew the inertia matrix for rattle effect
	shape.SetLocalMatrix(shapeMatrix);
	//body->SetMassMatrix(mass, shape, true);
	body->SetMassMatrix(mass, shape);
	body->SetCentreOfMass(ndVector(0.0f, -0.1f, 0.0f, 0.0f));

	ndVector omega(0.01f, speed, 0.0f, 0.0f);
	body->SetOmega(omega);

	ndSharedPtr<ndBody> bodyPtr(body);
	world->AddBody(bodyPtr);
	scene->AddEntity(entity);
}

static void PrecessingTop(ndDemoEntityManager* const scene, const ndVector& origin)
{
	ndPhysicsWorld* const world = scene->GetWorld();

	ndShapeInstance shape(new ndShapeCone(0.7f, 1.0f));
	shape.SetLocalMatrix(ndRollMatrix(-90.0f * ndDegreeToRad));

	ndMatrix matrix(ndPitchMatrix(15.0f * ndDegreeToRad));
	matrix.m_posit = origin;
	matrix.m_posit.m_w = 1.0f;

	ndSharedPtr<ndDemoMeshInterface> mesh(new ndDemoMesh("shape", scene->GetShaderCache(), &shape, "marble.tga", "marble.tga", "marble.tga"));

	ndDemoEntity* const entity = new ndDemoEntity(matrix, nullptr);
	entity->SetMesh(mesh);
	matrix.m_posit.m_y += 1.0f;

	const ndFloat32 mass = 1.0f;
	ndBodyKinematic* const body = new ndBodyDynamic();
	body->SetNotifyCallback(new ndDemoEntityNotify(scene, entity));
	body->SetMatrix(matrix);
	body->SetCollisionShape(shape);
	body->SetMassMatrix(mass, shape);
	body->SetOmega(matrix.m_up.Scale(40.0f));

	ndSharedPtr<ndBody> bodyPtr(body);
	world->AddBody(bodyPtr);
	scene->AddEntity(entity);
}

static void CreateFlyWheel(ndDemoEntityManager* const scene, const ndVector& origin, ndFloat32 mass, ndFloat32 speed, ndFloat32 radius, ndFloat32 lenght, ndFloat32 tiltAnsgle)
{
	ndPhysicsWorld* const world = scene->GetWorld();
	
	ndFloat32 smallRadius = 0.0625f;
	ndShapeInstance rod(new ndShapeCapsule(smallRadius * 0.5f, smallRadius * 0.5f, lenght));
	ndShapeInstance wheel(new ndShapeCylinder(radius, radius, 0.125f));
	
	ndMatrix offset(ndGetIdentityMatrix());
	offset.m_posit.m_x = lenght * 0.5f;
	wheel.SetLocalMatrix(offset);
	
	ndShapeInstance flyWheelShape(new ndShapeCompound());
	ndShapeCompound* const compound = flyWheelShape.GetShape()->GetAsShapeCompound();
	compound->BeginAddRemove();
	compound->AddCollision(&rod);
	compound->AddCollision(&wheel);
	compound->EndAddRemove();
	
	ndMatrix matrix(ndRollMatrix(tiltAnsgle * ndDegreeToRad));
	matrix.m_posit = origin;
	matrix.m_posit.m_y += 5.0f;
	matrix.m_posit.m_w = 1.0f;
	
	ndDemoEntity* const entity = new ndDemoEntity(ndGetIdentityMatrix(), nullptr);
	
	ndSharedPtr<ndDemoMeshInterface> mesh(new ndDemoMesh("primitive", scene->GetShaderCache(), &flyWheelShape, "smilli.tga", "smilli.tga", "smilli.tga"));
	entity->SetMesh(mesh);
	
	ndBodyKinematic* const body = new ndBodyDynamic();
	body->SetNotifyCallback(new ndDemoEntityNotify(scene, entity));
	body->SetMatrix(matrix);
	body->SetCollisionShape(flyWheelShape);
	body->SetMassMatrix(mass, flyWheelShape);
	
	ndVector omega(matrix.m_front.Scale (speed));
	body->SetOmega(omega);
	
	matrix.m_posit -= matrix.m_front.Scale(lenght * 0.5f);
	//ndJointSpherical* const joint = new ndJointSpherical(matrix, *body, world->GetSentinelBody());
	ndSharedPtr<ndJointBilateralConstraint> joint (new ndJointSpherical(matrix, body, world->GetSentinelBody()));
	
	ndSharedPtr<ndBody> bodyPtr(body);
	world->AddBody(bodyPtr);
	world->AddJoint(joint);
	scene->AddEntity(entity);
}

void ndBasicAngularMomentum (ndDemoEntityManager* const scene)
{
	// build a floor
	BuildFloorBox(scene, ndGetIdentityMatrix()); 

	// should spins very slowly, with a tilt angle of 30 degrees
	//CreateFlyWheel(scene, ndVector(15.0f, 0.0f, -12.0f, 0.0f), 10.0f, 50.0f, 0.6f, 0.5f, 30.0f);
	//CreateFlyWheel(scene, ndVector(15.0f, 0.0f, -10.0f, 0.0f), 10.0f, 100.0f, 0.6f, 0.5f, 5.0f);
	//CreateFlyWheel(scene, ndVector(15.0f, 0.0f,  -8.0f, 0.0f), 10.0f, -60.0f, 0.6f, 0.5f, 5.0f);
	//
	//DzhanibekovEffect(scene, 10.0f, 5.0f, ndVector(15.0f, 0.0f, -4.0f, 0.0f));
	//DzhanibekovEffect(scene, 10.0f, -5.0f, ndVector(15.0f, 0.0f, 0.0f, 0.0f));
	//DzhanibekovEffect(scene, 10.0f, 10.0f, ndVector(15.0f, 0.0f, 4.0f, 0.0f));
	//
	//Phitop(scene, 10.0f,  25.0f, ndVector(25.0f, 0.0f, -10.0f, 0.0f));
	//Phitop(scene, 10.0f, -25.0f, ndVector(25.0f, 0.0f, 0.0f, 0.0f));
	//Phitop(scene, 10.0f,  35.0f, ndVector(25.0f, 0.0f, 10.0f, 0.0f));
	//
	//PrecessingTop(scene, ndVector(5.0f, 0.0f, -4.0f, 0.0f));
	//PrecessingTop(scene, ndVector(5.0f, 0.0f, 0.0f, 0.0f));
	//PrecessingTop(scene, ndVector(5.0f, 0.0f, 4.0f, 0.0f));

	RattleBack(scene, 10.0f, -5.0f, ndVector(0.0f, 0.0f, -4.0f, 0.0f));
	//RattleBack(scene, 10.0f, 5.0f, ndVector(0.0f, 0.0f, 0.0f, 0.0f));
	//RattleBack(scene, 10.0f, -5.0f, ndVector(0.0f, 0.0f,  4.0f, 0.0f));
	
	scene->GetCameraManager()->SetPickMode(true);


	ndFileFormat xxxx;
	xxxx.CollectScene(scene->GetWorld());
	xxxx.SaveBodies("xxxx.nd");

	ndQuaternion rot;
	ndVector origin(-15.0f, 5.0f, 0.0f, 1.0f);
	scene->SetCameraMatrix(rot, origin);
}
