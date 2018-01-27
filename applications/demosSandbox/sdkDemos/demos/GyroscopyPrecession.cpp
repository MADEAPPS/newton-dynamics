/* Copyright (c) <2003-2016> <Newton Game Dynamics>
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


static NewtonBody* CreateFryWheel (DemoEntityManager* const scene, const dVector& posit, dFloat speed, dFloat radius, dFloat lenght)
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

	DemoMesh* const geometry = new DemoMesh("primitive", flyWheelShape, "smilli.tga", "smilli.tga", "smilli.tga");
	NewtonBody* const wheelBody = CreateSimpleSolid(scene, geometry, 10.0f, matrix, flyWheelShape, 0);
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
	NewtonBody* const flyWheel = CreateFryWheel(scene, posit, speed, radius, lenght);

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
	dCustom6dof* const fixPoint = new dCustom6dof(matrix, flyWheel, NULL);
	fixPoint->DisableRotationX();
	fixPoint->DisableRotationY();
	fixPoint->DisableRotationZ();
}

static void PrecessingTop(DemoEntityManager* const scene, const dVector& posit)
{
	NewtonBody* const top = CreateFryWheel(scene, posit, 100.0f, 0.5f, 0.3f);

	dMatrix matrix;
	dVector omega;

	dMatrix rotation(dRollMatrix(75.0f * dDegreeToRad));
	
	NewtonBodyGetOmega(top, &omega[0]);
	NewtonBodyGetMatrix(top, &matrix[0][0]);

	omega = rotation.RotateVector(omega);
	matrix = rotation * matrix;

	NewtonBodySetMatrix(top, &matrix[0][0]);
	NewtonBodySetOmega(top, &omega[0]);
}


void GyroscopyPrecession(DemoEntityManager* const scene)
{
	scene->CreateSkyBox();
	dMatrix offsetMatrix(dGetIdentityMatrix());

	CreateLevelMesh(scene, "flatPlane.ngd", 1);

	NewtonWorld* const world = scene->GetNewton();
	int defaultMaterialID = NewtonMaterialGetDefaultGroupID(world);
	NewtonMaterialSetDefaultFriction(world, defaultMaterialID, defaultMaterialID, 1.0f, 1.0f);
	NewtonMaterialSetDefaultElasticity(world, defaultMaterialID, defaultMaterialID, 0.1f);

	// should spins very slowly, with a tilt angle of 45 degrees
	CreateBicycleWheel(scene, dVector(0.0f, 3.0f, -4.0f, 1.0f), 100.0f, 0.6f, 0.3f, 30.0f);

	// spin twice as fast
	CreateBicycleWheel(scene, dVector(0.0f, 3.0f, -2.0f, 1.0f), 50.0f, 0.6f, 0.3f, 0.0f);

	// should spins very slowly
	CreateBicycleWheel(scene, dVector (0.0f, 3.0f, 0.0f, 1.0f), 100.0f, 0.6f, 0.3f, 0.0f);

	// should just flops
	CreateBicycleWheel(scene, dVector (0.0f, 3.0f, 2.0f, 1.0f), 0.0f, 0.6f, 0.3f, 0.0f);

	// place a toy top
	const int topsCount = 4;
	for (int i = 0; i < topsCount; i ++) {
		for (int j = 0; j < topsCount; j ++) {
			// should translate for a moment then spins in place (so far is wrong)
			PrecessingTop(scene, dVector(-2.0f * i - 2.0f, 3.0f, 2.0f * j - 2.0f, 1.0f));
		}
	}

	// place camera into position
	dMatrix camMatrix(dGetIdentityMatrix());
	dQuaternion rot(camMatrix);
	dVector origin(-15.0f, 2.0f, 0.0f, 0.0f);
	scene->SetCameraMatrix(rot, origin);
}



