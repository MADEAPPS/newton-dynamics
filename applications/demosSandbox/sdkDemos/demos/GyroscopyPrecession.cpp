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


#include "toolbox_stdafx.h"
#include "SkyBox.h"
#include "DemoEntityManager.h"
#include "DemoCamera.h"
#include "PhysicsUtils.h"
#include "DemoMesh.h"
#include "OpenGlUtil.h"

// zero gravity for some effects
static void ZeroGravityForce(const NewtonBody* body, dFloat timestep, int threadIndex)
{
}

static void PhiTopClampOmega(const NewtonBody* body, dFloat timestep, int threadIndex)
{
	PhysicsApplyGravityForce(body, timestep, threadIndex);

	dVector omega(0.0f);
	NewtonBodyGetOmega(body, &omega[0]);
	dFloat mag2 = omega.DotProduct3(omega);
	dFloat maxMag = 50.0f;
	if (mag2 > (maxMag * maxMag)) {
		omega = omega.Normalize().Scale(maxMag);
		NewtonBodySetOmega(body, &omega[0]);
	}
}

static NewtonBody* DzhanibekovEffect(DemoEntityManager* const scene, const dVector& posit, dVector omega, dFloat radius, dFloat lenght)
{
	NewtonWorld* const world = scene->GetNewton();

	dMatrix offset(dYawMatrix (90.0f * dDegreeToRad));

	dFloat shortLength = lenght * 0.3f;
	offset.m_posit.m_z = radius * 0.75f + shortLength * 0.5f;
#if 1
	NewtonCollision* const longCylinder = NewtonCreateCylinder(world, radius, radius, lenght, 0, NULL);
	NewtonCollision* const shortCylinder = NewtonCreateCylinder(world, radius, radius, shortLength, 0, &offset[0][0]);
	NewtonCollision* const dzhanibekovShape = NewtonCreateCompoundCollision(world, 0);
	NewtonCompoundCollisionBeginAddRemove(dzhanibekovShape);
	NewtonCompoundCollisionAddSubCollision(dzhanibekovShape, longCylinder);
	NewtonCompoundCollisionAddSubCollision(dzhanibekovShape, shortCylinder);
	NewtonCompoundCollisionEndAddRemove(dzhanibekovShape);
	NewtonDestroyCollision(longCylinder);
	NewtonDestroyCollision(shortCylinder);
#else
	NewtonCollision* const dzhanibekovShape = NewtonCreateBox(world, lenght, lenght * 0.25f, lenght * 0.5f, 0, NULL);
#endif

	dMatrix matrix(dGetIdentityMatrix());
	matrix.m_posit = posit;
	matrix.m_posit.m_x += lenght * 0.5f;
	matrix.m_posit.m_w = 1.0f;

	DemoMesh* const geometry = new DemoMesh("primitive", scene->GetShaderCache(), dzhanibekovShape, "smilli.tga", "smilli.tga", "smilli.tga");
	NewtonBody* const dzhanibekovBody = CreateSimpleSolid(scene, geometry, 10.0f, matrix, dzhanibekovShape, 0);

	NewtonBodySetGyroscopicTorque(dzhanibekovBody, 1);
	NewtonBodySetMassProperties(dzhanibekovBody, 10.0f, dzhanibekovShape);

	dFloat m, x, y, z;
	NewtonBodyGetMass(dzhanibekovBody, &m, &x, &y, &z);

	NewtonBodySetOmega(dzhanibekovBody, &omega[0]);

	dVector damp(0.0f);
	NewtonBodySetLinearDamping(dzhanibekovBody, 0.0f);
	NewtonBodySetAngularDamping(dzhanibekovBody, &damp[0]);
	NewtonBodySetForceAndTorqueCallback(dzhanibekovBody, ZeroGravityForce);

	geometry->Release();
	NewtonDestroyCollision(dzhanibekovShape);

	return dzhanibekovBody;
}

static NewtonBody* FrisbeePreccesion(DemoEntityManager* const scene, const dVector& posit, dFloat omega, dFloat radio, dFloat precessingAngle)
{
	NewtonWorld* const world = scene->GetNewton();

	dMatrix offset(dRollMatrix(90.0f * dDegreeToRad));
	NewtonCollision* const ballShape = NewtonCreateCylinder(world, radio, radio, radio * 0.25f, 0, &offset[0][0]);
	//NewtonCollision* const ballShape = NewtonCreateSphere(world, radio, 0, NULL);
	//NewtonCollisionSetScale(ballShape, 1.0f, 0.5f, 1.0f);

	dMatrix matrix(dGetIdentityMatrix());
	matrix.m_posit = posit;
	matrix.m_posit.m_w = 1.0f;

	DemoMesh* const geometry = new DemoMesh("primitive", scene->GetShaderCache(), ballShape, "smilli.tga", "smilli.tga", "smilli.tga");
	NewtonBody* const ball = CreateSimpleSolid(scene, geometry, 10.0f, matrix, ballShape, 0);

	NewtonBodySetGyroscopicTorque(ball, 1);
	NewtonBodySetMassProperties(ball, 10.0f, ballShape);

	dFloat m, Ixx, Iyy, Izz;
	NewtonBodyGetMass(ball, &m, &Ixx, &Iyy, &Izz);

	omega = 2.0f;
	dVector angVelocity (0.0f, omega, 0.0f, 0.0f);
	angVelocity = dPitchMatrix(precessingAngle * dDegreeToRad).RotateVector(angVelocity);

	NewtonBodySetOmega(ball, &angVelocity[0]);

	dVector damp(0.0f);
	NewtonBodySetLinearDamping(ball, 0.0f);
	NewtonBodySetAngularDamping(ball, &damp[0]);
	NewtonBodySetForceAndTorqueCallback(ball, ZeroGravityForce);

	geometry->Release();
	NewtonDestroyCollision(ballShape);

	return ball;
}

static NewtonBody* PhiTop(DemoEntityManager* const scene, const dVector& posit, dFloat omega, dFloat radio)
{
	NewtonWorld* const world = scene->GetNewton();

	NewtonCollision* const ballShape = NewtonCreateSphere(world, radio, 0, NULL);
	NewtonCollisionSetScale(ballShape, 0.5f, 0.5f, 1.0f);

	dMatrix matrix(dPitchMatrix(10.0f * dDegreeToRad));
	matrix.m_posit = posit;
	matrix.m_posit.m_w = 1.0f;

	DemoMesh* const geometry = new DemoMesh("primitive", scene->GetShaderCache(), ballShape, "smilli.tga", "smilli.tga", "smilli.tga");
	NewtonBody* const phiTop = CreateSimpleSolid(scene, geometry, 10.0f, matrix, ballShape, 0);

	// the PhiTop spins too fast and gyro torque add to much energy, they require higher simulation rate  
	NewtonBodySetGyroscopicTorque(phiTop, 1);
	NewtonBodySetMassProperties(phiTop, 10.0f, ballShape);
	NewtonBodySetForceAndTorqueCallback(phiTop, PhiTopClampOmega);

	dFloat m, Ixx, Iyy, Izz;
	NewtonBodyGetMass(phiTop, &m, &Ixx, &Iyy, &Izz);

	dVector angVelocity(0.0f, omega, 0.0f, 0.0f);
	NewtonBodySetOmega(phiTop, &angVelocity[0]);

	dVector damp(0.0f);
	NewtonBodySetLinearDamping(phiTop, 0.0f);
	NewtonBodySetAngularDamping(phiTop, &damp[0]);

	geometry->Release();
	NewtonDestroyCollision(ballShape);

	return phiTop;
}

static NewtonBody* RattleBack(DemoEntityManager* const scene, const dVector& posit, dFloat omega, dFloat radio)
{
	NewtonWorld* const world = scene->GetNewton();

	dMatrix shapeMatrix(dYawMatrix(5.0f * dDegreeToRad));
	NewtonCollision* const ballShape = NewtonCreateSphere(world, radio, 0, &shapeMatrix[0][0]);
	NewtonCollisionSetScale(ballShape, 0.3f, 0.25f, 1.0f);

	dMatrix matrix(dGetIdentityMatrix());
	matrix.m_posit = posit;
	matrix.m_posit.m_w = 1.0f;
	DemoMesh* const geometry = new DemoMesh("primitive", scene->GetShaderCache(), ballShape, "smilli.tga", "smilli.tga", "smilli.tga");
	NewtonBody* const ball = CreateSimpleSolid(scene, geometry, 10.0f, matrix, ballShape, 0, true);

	NewtonBodySetGyroscopicTorque(ball, 1);
	NewtonBodySetMassProperties(ball, 10.0f, ballShape);
	
	dFloat m, Ixx, Iyy, Izz;
	NewtonBodyGetMass(ball, &m, &Ixx, &Iyy, &Izz);

	dVector com(0.0f, -0.1f, 0.0f, 0.0);
	NewtonBodySetCentreOfMass(ball, &com[0]);
	dVector angVelocity(0.0f, omega, 0.1f, 0.0f);
	NewtonBodySetOmega(ball, &angVelocity[0]);

	dVector damp(0.0f);
	NewtonBodySetLinearDamping(ball, 0.0f);
	NewtonBodySetAngularDamping(ball, &damp[0]);

	geometry->Release();
	NewtonDestroyCollision(ballShape);

	return ball;
}

static NewtonBody* CreateFlyWheel (DemoEntityManager* const scene, const dVector& posit, dFloat speed, dFloat radius, dFloat lenght)
{
	NewtonWorld* const world = scene->GetNewton();

	dMatrix offset(dGetIdentityMatrix());
	offset.m_posit.m_x = lenght * 0.5f;
	dFloat smallRadius = 0.0625f;
	NewtonCollision* const rod = NewtonCreateCapsule(world, smallRadius * 0.5f, smallRadius * 0.5f, lenght, 0, NULL);
	NewtonCollision* const wheel = NewtonCreateCylinder(world, radius, radius, 0.125f, 0, &offset[0][0]);

	NewtonCollision* const flyWheelShape = NewtonCreateCompoundCollision(world, 0);
	NewtonCompoundCollisionBeginAddRemove(flyWheelShape);

	NewtonCompoundCollisionAddSubCollision(flyWheelShape, rod);
	NewtonCompoundCollisionAddSubCollision(flyWheelShape, wheel);
	NewtonCompoundCollisionEndAddRemove(flyWheelShape);

	dMatrix matrix(dGetIdentityMatrix());
	matrix.m_posit = posit;
	matrix.m_posit.m_x += lenght * 0.5f;
	matrix.m_posit.m_w = 1.0f;

	DemoMesh* const geometry = new DemoMesh("primitive", scene->GetShaderCache(), flyWheelShape, "smilli.tga", "smilli.tga", "smilli.tga");
	NewtonBody* const wheelBody = CreateSimpleSolid(scene, geometry, 10.0f, matrix, flyWheelShape, 0);

	NewtonBodySetGyroscopicTorque(wheelBody, 1);
	NewtonBodySetMassProperties(wheelBody, 10.0f, flyWheelShape);

	dFloat m;
	dFloat x;
	dFloat y;
	dFloat z;
	NewtonBodyGetMass(wheelBody, &m, &x, &y, &z);

	dVector damp(0.0f);
	NewtonBodySetLinearDamping(wheelBody, 0.0f);
	NewtonBodySetAngularDamping(wheelBody, &damp[0]);

	dVector omega(speed, 0.0f, 0.0f);
	NewtonBodySetOmega(wheelBody, &omega[0]);

	matrix.m_posit.m_x -= lenght * 0.5f;

	geometry->Release();
	NewtonDestroyCollision(flyWheelShape);
	NewtonDestroyCollision(wheel);
	NewtonDestroyCollision(rod);

	return wheelBody;
}

static void CreateBicycleWheel(DemoEntityManager* const scene, const dVector& posit, dFloat speed, dFloat radius, dFloat lenght, dFloat tiltAnsgle)
{
	speed *= -1.0f;
	NewtonBody* const flyWheel = CreateFlyWheel(scene, posit, speed, radius, lenght);

	dMatrix matrix(dGetIdentityMatrix());
	NewtonBodyGetMatrix(flyWheel, &matrix[0][0]);

	dVector omega(speed, 0.0f, 0.0f);
	dMatrix rotation(dRollMatrix(tiltAnsgle * dDegreeToRad));
	NewtonBodyGetOmega(flyWheel, &omega[0]);
	omega = rotation.RotateVector(omega);
	matrix = rotation * matrix;
	NewtonBodySetMatrix(flyWheel, &matrix[0][0]);
	NewtonBodySetOmega(flyWheel, &omega[0]);

	matrix.m_posit -= matrix.m_front.Scale (lenght * 0.5f);
	new dCustomBallAndSocket(matrix, flyWheel, NULL);
}

static void PrecessingTop(DemoEntityManager* const scene, const dVector& posit)
{
	NewtonWorld* const world = scene->GetNewton();

	dMatrix shapeMatrix(dRollMatrix(-90.0f * dDegreeToRad));
	NewtonCollision* const cone = NewtonCreateCone(world, 0.7f, 1.0f, 0, &shapeMatrix[0][0]);

	dMatrix matrix(dPitchMatrix(15.0f * dDegreeToRad));
	matrix.m_posit = posit;
	matrix.m_posit.m_w = 1.0f;
	DemoMesh* const geometry = new DemoMesh("primitive", scene->GetShaderCache(), cone, "smilli.tga", "smilli.tga", "smilli.tga");
	NewtonBody* const gyroTop = CreateSimpleSolid(scene, geometry, 10.0f, matrix, cone, 0);

	NewtonBodySetGyroscopicTorque(gyroTop, 1);
	NewtonBodySetMassProperties(gyroTop, 10.0f, cone);

	dFloat m, Ixx, Iyy, Izz;
	NewtonBodyGetMass(gyroTop, &m, &Ixx, &Iyy, &Izz);

	dVector omega (matrix.m_up.Scale (40.0f));
	NewtonBodySetOmega(gyroTop, &omega[0]);

	dVector damp(0.0f);
	NewtonBodySetLinearDamping(gyroTop, 0.0f);
	NewtonBodySetAngularDamping(gyroTop, &damp[0]);

	geometry->Release();
	NewtonDestroyCollision(cone);
}

static void TippeTop(DemoEntityManager* const scene, const dVector& posit, dVector omega, dFloat radius, dFloat lenght)
{
	NewtonBody* const top = CreateFlyWheel(scene, posit, 100.0f, 0.5f, 0.3f);

	dMatrix matrix;
//	dVector omega;

	dMatrix rotation(dRollMatrix(75.0f * dDegreeToRad));

	NewtonBodyGetOmega(top, &omega[0]);
	NewtonBodyGetMatrix(top, &matrix[0][0]);

	omega = rotation.RotateVector(omega);
	matrix = rotation * matrix;

	NewtonBodySetMatrix(top, &matrix[0][0]);
	NewtonBodySetOmega(top, &omega[0]);
}

static void TestGyroContacts(DemoEntityManager* const scene, const dVector& posit, int enableGyro)
{
	NewtonWorld* const world = scene->GetNewton();

	NewtonCollision* const shape = NewtonCreateBox(world, .125f, 0.125f, 0.5f, 0, NULL);

	dMatrix matrix(dGetIdentityMatrix());
	matrix.m_posit = posit;
	matrix.m_posit.m_w = 1.0f;
	DemoMesh* const geometry = new DemoMesh("primitive", scene->GetShaderCache(), shape, "wood_1.tga", "wood_1.tga", "wood_1.tga");

	NewtonBody* const bar0 = CreateSimpleSolid(scene, geometry, 10.0f, matrix, shape, 0);
	NewtonBodySetGyroscopicTorque(bar0, enableGyro);
	NewtonBodySetMassProperties(bar0, 10.0f, shape);

	dMatrix matrix1 (dPitchMatrix (90.0f * dDegreeToRad) * matrix);
	matrix1.m_posit.m_z += 0.25f + 0.125f * 0.5f;
	matrix1.m_posit.m_y -= (0.25f - 0.125f * 0.5f);
	NewtonBody* const bar1 = CreateSimpleSolid(scene, geometry, 10.0f, matrix1, shape, 0);
	NewtonBodySetGyroscopicTorque(bar1, enableGyro);
	NewtonBodySetMassProperties(bar1, 10.0f, shape);
	dMatrix pivotMatrix1(matrix);
	pivotMatrix1.m_posit.m_y += 0.25f;
	new dCustomSixdof(pivotMatrix1, bar0, bar1);

	dMatrix matrix2(dPitchMatrix(90.0f * dDegreeToRad) * matrix);
	matrix2.m_posit.m_z -= 0.25f + 0.125f * 0.5f;
	matrix2.m_posit.m_y -= (0.25f - 0.125f * 0.5f);
	NewtonBody* const bar2 = CreateSimpleSolid(scene, geometry, 10.0f, matrix2, shape, 0);
	NewtonBodySetGyroscopicTorque(bar2, enableGyro);
	NewtonBodySetMassProperties(bar2, 10.0f, shape);
	dMatrix pivotMatrix2(matrix);
	pivotMatrix2.m_posit.m_y -= 0.25f;
	new dCustomSixdof(pivotMatrix2, bar0, bar2);


	geometry->Release();
	NewtonDestroyCollision(shape);
}


void GyroscopyPrecession(DemoEntityManager* const scene)
{
	scene->CreateSkyBox();
	dMatrix offsetMatrix(dGetIdentityMatrix());

	CreateLevelMesh(scene, "flatPlane.ngd", 1);

	NewtonWorld* const world = scene->GetNewton();
	int defaultMaterialID = NewtonMaterialGetDefaultGroupID(world);
	NewtonMaterialSetDefaultFriction(world, defaultMaterialID, defaultMaterialID, 1.0f, 1.0f);
	NewtonMaterialSetDefaultElasticity(world, defaultMaterialID, defaultMaterialID, 0.6f);

	// should spins very slowly, with a tilt angle of 30 degrees
//	CreateBicycleWheel(scene, dVector(0.0f, 3.0f, -8.0f, 1.0f), 100.0f, 0.6f, 0.3f, 30.0f);
//	
//	// should spins slowly, twice precession speed.
//	CreateBicycleWheel(scene, dVector(0.0f, 3.0f, -6.0f, 1.0f), 50.0f, 0.6f, 0.3f, 0.0f);
//
//	// spin twice as fast, slow precession 
//	CreateBicycleWheel(scene, dVector(0.0f, 3.0f, -4.0f, 1.0f), 100.0f, 0.6f, 0.3f, 0.0f);
//
//	// should just flops
//	CreateBicycleWheel(scene, dVector(0.0f, 3.0f, -2.0f, 1.0f), 0.0f, 0.6f, 0.3f, 0.0f);
//
//	// a thin disk should precess with an angular velocity twice the spin rate, 
//	// this is also known as the Frisbee theorem
//	FrisbeePreccesion(scene, dVector(0.0f, 3.0f, -10.0f, 1.0f), 10.0f, 1.0f, 15.0f);
//
//	// intermediate Axis Theorem
//	DzhanibekovEffect(scene, dVector(0.0f, 3.0f,  0.0f, 1.0f), dVector (0.01f, 0.01f, 10.0f), 0.25f, 2.0f);
//	DzhanibekovEffect(scene, dVector(0.0f, 3.0f,  2.0f, 1.0f), dVector (0.01f, 0.01f,-15.0f), 0.25f, 2.0f);
//
//	//the effect only happens is there is a residual angular velocity on the other two axis
//	//for perfectly axis aligned velocity the body is in unstable equilibrium and should not flip.
//	DzhanibekovEffect(scene, dVector(0.0f, 3.0f,  4.0f, 1.0f), dVector (0.01f, 10.0f, 0.01f), 0.25f, 2.0f);
//	DzhanibekovEffect(scene, dVector(0.0f, 3.0f,  6.0f, 1.0f), dVector (10.0f, 0.01f, 0.01f), 0.25f, 2.0f);
//	DzhanibekovEffect(scene, dVector(0.0f, 3.0f,  8.0f, 1.0f), dVector (0.0f, 0.0f, 10.0f), 0.25f, 2.0f);
//
//	// interesting and strange effect generated by and skew inertia
//	RattleBack(scene, dVector(-2.0f, 0.5f, - 3.0, 1.0f), 0.0f, 1.0f);
//	RattleBack(scene, dVector(-2.0f, 0.5f, - 6.0, 1.0f), 2.0f, 1.0f);
//	RattleBack(scene, dVector(-2.0f, 0.5f, - 9.0, 1.0f), -2.0f, 1.0f);

	TestGyroContacts(scene, dVector(-4.0f, 1.5f, -5.0, 1.0f), 1);
	TestGyroContacts(scene, dVector(-4.0f, 1.5f, -2.0, 1.0f), 0);

	// place a toy tops
	int topsCount = 4;
topsCount = 1;
	const dFloat spacing = 3.0f;
	for (int i = 0; i < topsCount; i++) {
		for (int j = 0; j < topsCount; j++) {
//			PrecessingTop(scene, dVector(spacing * j, 0.5f, -spacing * i - spacing, 1.0f));
		}
		//PhiTop(scene, dVector(30.0f, 0.4f, -spacing * i - spacing, 1.0f), i * 5.0f + 10.0f, 1.0f);
		//TippeTop(scene, dVector(-6.0f, 0.3f, -spacing * i - spacing, 1.0f), 0.0f, 0.0f, 0.0f);
	}

	// place camera into position
	dMatrix camMatrix(dGetIdentityMatrix());
	dQuaternion rot(camMatrix);
	dVector origin(-10.0f, 2.0f, -5.0f, 0.0f);
	scene->SetCameraMatrix(rot, origin);
}
