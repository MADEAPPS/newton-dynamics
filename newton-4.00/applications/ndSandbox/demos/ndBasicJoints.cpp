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
#include "ndDemoSplinePathMesh.h"
#include "ndDemoEntityManager.h"
#include "ndDemoInstanceEntity.h"

class ndFollowSplinePath : public ndJointFollowPath
{
	public:
	ndFollowSplinePath(const dMatrix& pinAndPivotFrame, ndBodyDynamic* const child, ndBodyDynamic* const pathBody)
		:ndJointFollowPath(pinAndPivotFrame, child, pathBody)
	{
	}

	void GetPointAndTangentAtLocation(const dVector& location, dVector& positOut, dVector& tangentOut) const
	{
		ndDemoEntity* const pathEntity = (ndDemoEntity*)GetBody1()->GetNotifyCallback()->GetUserData();
		ndDemoSplinePathMesh* const mesh = (ndDemoSplinePathMesh*)pathEntity->GetMesh();
		const dBezierSpline& spline = mesh->m_curve;
		
		dMatrix matrix(GetBody1()->GetMatrix());
		
		dVector p(matrix.UntransformVector(location));
		dBigVector point;
		dFloat64 knot = spline.FindClosestKnot(point, p, 4);
		dBigVector tangent(spline.CurveDerivative(knot));
		tangent = tangent.Scale(1.0 / dSqrt(tangent.DotProduct(tangent).GetScalar()));
		positOut = matrix.TransformVector(point);
		tangentOut = tangent;
	}
};

static ndBodyDynamic* MakePrimitive(ndDemoEntityManager* const scene, const dMatrix& matrix, const ndShapeInstance& shape, ndDemoMesh* const mesh, dFloat32 mass)
{
	ndPhysicsWorld* const world = scene->GetWorld();
	ndDemoEntity* const entity = new ndDemoEntity(matrix, nullptr);
	entity->SetMesh(mesh, dGetIdentityMatrix());
	ndBodyDynamic* const body = new ndBodyDynamic();
	body->SetNotifyCallback(new ndDemoEntityNotify(scene, entity));
	body->SetMatrix(matrix);
	body->SetCollisionShape(shape);
	body->SetMassMatrix(mass, shape);
	world->AddBody(body);
	scene->AddEntity(entity);
	return body;
}

static void BuildBallSocket(ndDemoEntityManager* const scene, const dVector& origin)
{
	dFloat32 mass = 1.0f;
	dFloat32 diameter = 0.5f;
	ndShapeInstance shape(new ndShapeCapsule(diameter * 0.5f, diameter * 0.5f, diameter * 1.0f));
	ndDemoMesh* const mesh = new ndDemoMesh("shape", scene->GetShaderCache(), &shape, "marble.tga", "marble.tga", "marble.tga");

	//dMatrix matrix(dGetIdentityMatrix());
	dMatrix matrix(dRollMatrix(90.0f * dDegreeToRad));
	matrix.m_posit = origin;
	matrix.m_posit.m_w = 1.0f;

	ndPhysicsWorld* const world = scene->GetWorld();

	dVector floor(FindFloor(*world, matrix.m_posit + dVector(0.0f, 100.0f, 0.0f, 0.0f), 200.0f));
	matrix.m_posit.m_y = floor.m_y;

	matrix.m_posit.m_y += 1.0f;
	ndBodyDynamic* const body0 = MakePrimitive(scene, matrix, shape, mesh, mass);
	matrix.m_posit.m_y += 1.0f;
	ndBodyDynamic* const body1 = MakePrimitive(scene, matrix, shape, mesh, mass);

	dMatrix bodyMatrix0(body0->GetMatrix());
	dMatrix bodyMatrix1(body1->GetMatrix());
	dMatrix pinMatrix(bodyMatrix0);
	pinMatrix.m_posit = (bodyMatrix0.m_posit + bodyMatrix1.m_posit).Scale(0.5f);
	ndJointBallAndSocket* const joint0 = new ndJointBallAndSocket(pinMatrix, body0, body1);
	world->AddJoint(joint0);
	
	bodyMatrix1.m_posit.m_y += 0.5f;
	ndBodyDynamic* const fixBody = world->GetSentinelBody();
	ndJointBallAndSocket* const joint1 = new ndJointBallAndSocket(bodyMatrix1, body1, fixBody);
	world->AddJoint(joint1);

	mesh->Release();
}

static void BuildBall(ndDemoEntityManager* const scene, const dVector& origin, dFloat32 mass, dFloat32 diameter)
{
	dMatrix matrix(dRollMatrix(90.0f * dDegreeToRad));
	matrix.m_posit = origin;
	matrix.m_posit.m_w = 1.0f;
	//
	ndShapeInstance shape2(new ndShapeSphere(diameter * 0.5f));
	ndDemoMesh* const mesh2 = new ndDemoMesh("shape2", scene->GetShaderCache(), &shape2, "marble.tga", "marble.tga", "marble.tga");
	matrix.m_posit.m_y += 5.0f;
	ndBodyDynamic* const bodyA = MakePrimitive(scene, matrix, shape2, mesh2, mass);
	bodyA;
	mesh2->Release();
}

static void BuildSlider(ndDemoEntityManager* const scene, const dVector& origin, dFloat32 mass, dFloat32 diameter)
{
	//ndShapeInstance shape(new ndShapeCapsule(diameter * 0.5f, diameter * 0.5f, diameter * 1.0f));
	ndShapeInstance shape(new ndShapeBox(diameter, diameter, diameter));
	ndDemoMesh* const mesh = new ndDemoMesh("shape", scene->GetShaderCache(), &shape, "marble.tga", "marble.tga", "marble.tga");

	dMatrix matrix(dRollMatrix(90.0f * dDegreeToRad));
	matrix.m_posit = origin;
	matrix.m_posit.m_w = 1.0f;

	ndPhysicsWorld* const world = scene->GetWorld();

	dVector floor(FindFloor(*world, matrix.m_posit + dVector(0.0f, 100.0f, 0.0f, 0.0f), 200.0f));
	matrix.m_posit.m_y = floor.m_y;

	matrix.m_posit.m_y += 2.0f;

	ndBodyDynamic* const fixBody = world->GetSentinelBody();
	ndBodyDynamic* const body = MakePrimitive(scene, matrix, shape, mesh, mass);
	
	ndJointSlider* const joint = new ndJointSlider(matrix, body, fixBody);

	dFloat32 regularizer = diameter > 0.5f ? 0.9f : 0.1f;
	joint->SetAsSpringDamper(true, regularizer, 500.0f, 0.05f);
	joint->EnableLimits(true, -1.0f, 1.0f);
	world->AddJoint(joint);

	mesh->Release();
}

static void BuildHinge(ndDemoEntityManager* const scene, const dVector& origin, dFloat32 mass, dFloat32 diameter)
{
	ndShapeInstance shape(new ndShapeBox(diameter, diameter, diameter));
	ndDemoMesh* const mesh = new ndDemoMesh("shape", scene->GetShaderCache(), &shape, "marble.tga", "marble.tga", "marble.tga");

	dMatrix matrix(dRollMatrix(90.0f * dDegreeToRad));
	matrix.m_posit = origin;
	matrix.m_posit.m_w = 1.0f;

	ndPhysicsWorld* const world = scene->GetWorld();

	dVector floor(FindFloor(*world, matrix.m_posit + dVector(0.0f, 100.0f, 0.0f, 0.0f), 200.0f));
	matrix.m_posit.m_y = floor.m_y;

	matrix.m_posit.m_y += 2.0f;

	ndBodyDynamic* const fixBody = world->GetSentinelBody();
	ndBodyDynamic* const body = MakePrimitive(scene, matrix, shape, mesh, mass);

	ndJointHinge* const joint = new ndJointHinge(matrix, body, fixBody);
	//joint->SetAsSpringDamper(true, 500.0f, 5.0f);
	//joint->SetFriction(mass * 10.0f * 2.0f);
	//joint->EnableLimits(true, -1.0f, 1.0f);
	world->AddJoint(joint);

	mesh->Release();
}

static void BuildGear(ndDemoEntityManager* const scene, const dVector& origin, dFloat32 mass, dFloat32 diameter)
{
	ndShapeInstance shape(new ndShapeBox(diameter, diameter, diameter));
	ndDemoMesh* const mesh = new ndDemoMesh("shape", scene->GetShaderCache(), &shape, "marble.tga", "marble.tga", "marble.tga");

	dMatrix matrix(dRollMatrix(90.0f * dDegreeToRad));
	matrix.m_posit = origin;
	matrix.m_posit.m_w = 1.0f;

	ndPhysicsWorld* const world = scene->GetWorld();

	dVector floor(FindFloor(*world, matrix.m_posit + dVector(0.0f, 100.0f, 0.0f, 0.0f), 200.0f));
	matrix.m_posit.m_y = floor.m_y;

	matrix.m_posit.m_y += 2.0f;

	ndBodyDynamic* const fixBody = world->GetSentinelBody();
	ndBodyDynamic* const body0 = MakePrimitive(scene, matrix, shape, mesh, mass);

	matrix.m_posit.m_y += diameter * 1.5f;
	ndBodyDynamic* const body1 = MakePrimitive(scene, matrix, shape, mesh, mass);

	world->AddJoint(new ndJointHinge(matrix, body0, fixBody));
	world->AddJoint(new ndJointHinge(matrix, body1, fixBody));

	dVector pin(matrix.m_front);
	world->AddJoint(new ndJointGear(5.0f, pin, body0, pin, body1));

	mesh->Release();
}

static void AddPathFollow(ndDemoEntityManager* const scene, const dVector& origin)
{
	dMatrix matrix(dGetIdentityMatrix());
	matrix.m_posit = origin;

	// create a Bezier Spline path for AI car to drive
	ndShapeInstance box(new ndShapeBox(1.0f, 1.0f, 1.0f));
	ndBodyDynamic* const pathBody = MakePrimitive(scene, matrix, box, nullptr, 0.0f);
	ndDemoEntity* const rollerCosterPath = (ndDemoEntity*)pathBody->GetNotifyCallback()->GetUserData();

	dBezierSpline spline;
	dFloat64 knots[] = { 0.0f, 1.0f / 5.0f, 2.0f / 5.0f, 3.0f / 5.0f, 4.0f / 5.0f, 1.0f };

	dBigVector control[] =
	{
		dBigVector(100.0f - 100.0f, 20.0f, 200.0f - 250.0f, 1.0f),
		dBigVector(150.0f - 100.0f, 10.0f, 150.0f - 250.0f, 1.0f),
		dBigVector(175.0f - 100.0f, 30.0f, 250.0f - 250.0f, 1.0f),
		dBigVector(200.0f - 100.0f, 70.0f, 250.0f - 250.0f, 1.0f),
		dBigVector(215.0f - 100.0f, 20.0f, 250.0f - 250.0f, 1.0f),
		dBigVector(150.0f - 100.0f, 50.0f, 350.0f - 250.0f, 1.0f),
		dBigVector(50.0f - 100.0f, 30.0f, 250.0f - 250.0f, 1.0f),
		dBigVector(100.0f - 100.0f, 20.0f, 200.0f - 250.0f, 1.0f),
	};

	spline.CreateFromKnotVectorAndControlPoints(3, sizeof(knots) / sizeof(knots[0]), knots, control);

	ndDemoSplinePathMesh* const mesh = new ndDemoSplinePathMesh(spline, scene->GetShaderCache(), 500);
	rollerCosterPath->SetMesh(mesh, dGetIdentityMatrix());

	mesh->SetVisible(true);
	//mesh->SetRenderResolution(500);
	mesh->Release();

	const dInt32 count = 32;
	//const dInt32 count = 2;

	dBigVector point0;
	dVector positions[count + 1];
	dFloat64 knot = spline.FindClosestKnot(point0, dBigVector(dVector(100.0f - 100.0f, 20.0f, 200.0f - 250.0f, 1.0f)), 4);
	positions[0] = point0;
	for (dInt32 i = 0; i < count; i++) 
	{
		dBigVector point1;
		dBigVector tangent(spline.CurveDerivative(knot));
		tangent = tangent.Scale(1.0 / dSqrt(tangent.DotProduct(tangent).GetScalar()));
		knot = spline.FindClosestKnot(point1, dBigVector(point0 + tangent.Scale(2.0f)), 4);
		point0 = point1;
		positions[i + 1] = point1;
	}

	dMatrix pathBodyMatrix (pathBody->GetMatrix());
	dFloat32 attachmentOffset = 0.8f;

	ndShapeInstance shape(new ndShapeChamferCylinder(0.5f, 0.5f));
	ndDemoMeshIntance* const instanceMesh = new ndDemoMeshIntance("shape", scene->GetShaderCache(), &shape, "marble.tga", "marble.tga", "marble.tga");
	ndDemoInstanceEntity* const rootEntity = new ndDemoInstanceEntity(instanceMesh);
	scene->AddEntity(rootEntity);
	instanceMesh->Release();

	ndBodyDynamic* bodies[count];
	ndPhysicsWorld* const world = scene->GetWorld();
	for (dInt32 i = 0; i < count; i++) 
	{
		dVector location0(positions[i + 0].m_x, positions[i + 0].m_y, positions[i + 0].m_z, 0.0);
		dVector location1(positions[i + 1].m_x, positions[i + 1].m_y, positions[i + 1].m_z, 0.0);
		
		location0 = pathBodyMatrix.TransformVector(location0);
		location1 = pathBodyMatrix.TransformVector(location1);
		
		dVector dir(location1 - location0);
		dir.m_w = 0.0f;
		matrix.m_front = dir.Scale(1.0f / dSqrt(dir.DotProduct(dir).GetScalar()));
		matrix.m_right = matrix.m_front.CrossProduct(matrix.m_up);
		matrix.m_right = matrix.m_right.Scale(1.0f / dSqrt(matrix.m_right.DotProduct(matrix.m_right).GetScalar()));
		matrix.m_up = matrix.m_right.CrossProduct(matrix.m_front);
		matrix.m_posit = pathBodyMatrix.TransformVector(dVector(positions[i].m_x, positions[i].m_y - attachmentOffset, positions[i].m_z, 1.0));
		dMatrix matrix1(dYawMatrix(0.5f * dPi) * matrix);
		
		ndBodyDynamic* const body = new ndBodyDynamic();
		ndDemoEntity* const entity = new ndDemoEntity(matrix1, rootEntity);
		
		body->SetNotifyCallback(new ndDemoEntityNotify(scene, entity));
		body->SetMatrix(matrix1);
		body->SetCollisionShape(shape);
		body->SetMassMatrix(1.0f, shape);
		
		world->AddBody(body);
		
		bodies[i] = body;
		matrix.m_posit = pathBodyMatrix.TransformVector(dVector(positions[i].m_x, positions[i].m_y, positions[i].m_z, 1.0));
		world->AddJoint(new ndFollowSplinePath(matrix, body, pathBody));
		
		dVector veloc(dir.Scale(20.0f));
		body->SetVelocity(veloc);
	}

	for (dInt32 i = 1; i < count; i++) 
	{
		ndBodyDynamic* const box0 = bodies[i - 1];
		ndBodyDynamic* const box1 = bodies[i - 0];

		dMatrix matrix0(box0->GetMatrix());
		dMatrix matrix1(box1->GetMatrix());

		matrix0.m_posit.m_y += attachmentOffset;
		matrix1.m_posit.m_y += attachmentOffset;

		world->AddJoint(new ndJointFixDistance(matrix1.m_posit, matrix0.m_posit, box1, box0));
	}
}


void ndBasicJoints (ndDemoEntityManager* const scene)
{
	// build a floor
	BuildFloorBox(scene);

	BuildBallSocket(scene, dVector(0.0f, 0.0f, 1.0f, 0.0f));
	BuildGear(scene, dVector(0.0f, 0.0f, -4.0f, 1.0f), 100.0f, 0.75f);
	BuildHinge(scene, dVector(0.0f, 0.0f, -2.0f, 1.0f), 10.0f, 0.5f);
	BuildBall(scene, dVector(0.0f, 0.0f, 0.0f, 1.0f), 10.0f, 0.5f);
	BuildSlider(scene, dVector(0.0f, 0.0f, 2.5f, 1.0f), 10.0f, 0.5f);
	BuildSlider(scene, dVector(0.0f, 0.0f, 4.0f, 1.0f), 100.0f, 0.75f);
	AddPathFollow(scene, dVector(40.0f, 0.0f, 0.0f, 1.0f));
	
	dQuaternion rot;
	dVector origin(-20.0f, 5.0f, 0.0f, 0.0f);
	scene->SetCameraMatrix(rot, origin);
}
