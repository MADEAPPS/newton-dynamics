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


#include <toolbox_stdafx.h>
#include "SkyBox.h"
#include "DemoEntityManager.h"
#include "DemoCamera.h"
#include "PhysicsUtils.h"
#include "DemoMesh.h"
#include "OpenGlUtil.h"

// add force and torque to rigid body
static void ApplyGravityForce(const NewtonBody* body, dFloat timestep, int threadIndex)
{
	dFloat Ixx;
	dFloat Iyy;
	dFloat Izz;
	dFloat mass;

	NewtonWorld* const world = NewtonBodyGetWorld(body);
	DemoEntityManager* const scene = (DemoEntityManager*)NewtonWorldGetUserData(world);

	bool bThrust = false;

	if (scene->GetKeyState(' ')) {
		bThrust = true;
	}
	NewtonBodyGetMass(body, &mass, &Ixx, &Iyy, &Izz);

	// no gravity for this example
	dVector force(dVector(0.0f, 0.0f, 0.0f).Scale(0.0f)); 

	dVector torque(dVector(0.0f, 0.0f, 1.0f).Scale(0.0));
	if (bThrust)
	{
		// ten million newtons, so the bodies will "rise" slowly...
		torque = dVector(0.0f, 0.0f, 1.0f).Scale(1000000000.0); 
	} else {
		// ten million newtons, so the bodies will "rise" slowly...
		torque = dVector(0.0f, 0.0f, 1.0f).Scale(0.0); 
	}

	// pressing P prints out distance between objects...
	if (scene->GetKeyState('P')) {
		const NewtonBody* body1 = NULL;
		for (NewtonBody* bodyZ = NewtonWorldGetFirstBody(world); bodyZ; bodyZ = NewtonWorldGetNextBody(world, bodyZ)) {
			if (bodyZ != body) {
				body1 = bodyZ;
			}
		}

		dMatrix matrix;
		dVector bodyVectors[2];
		dVector positionVector(0.0f);

		NewtonBodyGetMatrix(body, &matrix[0][0]);
		NewtonBodyGetPosition(body, &positionVector[0]);
		bodyVectors[0] = matrix.UntransformVector(positionVector);

		NewtonBodyGetPosition(body1, &positionVector[0]);
		bodyVectors[1] = matrix.UntransformVector(positionVector);

		dVector diff = bodyVectors[1] - bodyVectors[0];
		dFloat distance = dSqrt(diff.DotProduct3(diff));

		// Nominal object distance at the beginning is 35, so we will subtract that here to get to zero as a starting point.
		printf("Object distance: %f  local posit(%f %f %f)\n", distance - 35.0f, diff[0], diff[1], diff[2]);
		dTrace(("Object distance: %f local posit(%f %f %f)\n", distance - 35.0f, diff[0], diff[1], diff[2]));
	}

	// Apply torque only to the Core object
	NewtonBodyAddTorque(body, &torque.m_x);
}


void MishosHingeTest(DemoEntityManager* const scene)
{
	scene->CreateSkyBox();

	NewtonWorld* const world = scene->GetNewton();
	dMatrix offset(dGetIdentityMatrix());
	dVector damp(0.0f, 0.0f, 0.0f);

	// Make rocket core body...
	NewtonCollision* const CoreShape = NewtonCreateCylinder(world, 5.0f, 5.0f, 60.0f, 1, &offset[0][0]);
	NewtonCollision* const CoreCompound = NewtonCreateCompoundCollision(world, 0);
	NewtonCompoundCollisionBeginAddRemove(CoreCompound);
	NewtonCompoundCollisionAddSubCollision(CoreCompound, CoreShape);
	NewtonCompoundCollisionEndAddRemove(CoreCompound);
	dMatrix matrixCore(0.0f, 0.0f, 1.5708f, dVector(0.0f, -30.0f, 0.0f));
	dFloat CoreMass = 979452.0f;
	DemoMesh* const geometryCore = new DemoMesh("Core Mesh", CoreCompound, "metal_30.tga", "metal_30.tga", "metal_30.tga");
	NewtonBody* const CoreBody = CreateSimpleSolid(scene, geometryCore, CoreMass, matrixCore, CoreCompound, 0);
	NewtonBodySetForceAndTorqueCallback(CoreBody, ApplyGravityForce);
	DemoEntity* const entityCore = (DemoEntity*)NewtonBodyGetUserData(CoreBody);
	entityCore->SetNameID("Core");
	NewtonBodySetLinearDamping(CoreBody, 0.0f);
	NewtonBodySetAngularDamping(CoreBody, &damp[0]);
	geometryCore->Release();
	NewtonDestroyCollision(CoreCompound);
	NewtonDestroyCollision(CoreShape);

	// Make payload ...  
	NewtonCollision* const PayloadShape = NewtonCreateCylinder(world, 5.0f, 5.0f, 10.0f, 1, &offset[0][0]);
	NewtonCollision* const PayloadCompound = NewtonCreateCompoundCollision(world, 0);
	NewtonCompoundCollisionBeginAddRemove(PayloadCompound);
	NewtonCompoundCollisionAddSubCollision(PayloadCompound, PayloadShape);
	NewtonCompoundCollisionEndAddRemove(PayloadCompound);
	dMatrix matrixPayload(0.0f, 0.0f, 1.5708f, dVector(0.0f, 5.0f, 0.0f));
	dFloat PayloadMass = 979452.0f;
	DemoMesh* const geometryPayload = new DemoMesh("Payload Stage", PayloadCompound, "metalblu.tga", "metalblu.tga", "metalblu.tga");
	NewtonBody* const PayloadBody = CreateSimpleSolid(scene, geometryPayload, PayloadMass, matrixPayload, PayloadCompound, 0);
	//NewtonBodySetForceAndTorqueCallback(PayloadBody, ApplyGravityForce);
	NewtonBodySetForceAndTorqueCallback(PayloadBody, NULL);
	DemoEntity* const entityPayload = (DemoEntity*)NewtonBodyGetUserData(PayloadBody);
	entityPayload->SetNameID("Payload");
	NewtonBodySetLinearDamping(PayloadBody, 0.0f);
	NewtonBodySetAngularDamping(PayloadBody, &damp[0]);
	geometryPayload->Release();
	NewtonDestroyCollision(PayloadCompound);
	NewtonDestroyCollision(PayloadShape);

	// Lock these two bodies with a hard hinge...
	dMatrix  mSourceMatrix;
	NewtonBodyGetMatrix(PayloadBody, &mSourceMatrix[0][0]);
	dCustomHinge* pHinge = NULL;
	pHinge = new dCustomHinge(mSourceMatrix, PayloadBody, CoreBody);
	pHinge->EnableMotor(false, 0.0);
	pHinge->SetAsSpringDamper(false, 0.0, 0.0, 0.0);
	pHinge->EnableLimits(true);
	pHinge->SetLimits(0.0, 0.0);
//	pHinge->SetStiffness(1.00f); // Tried this but it doesn't change anything...

	// place camera into position and rotate towards objects
	dMatrix camMatrix(dGetIdentityMatrix());
	dQuaternion dqRotY(dVector(0.0f, 1.0f, 0.0f), 90.0f * 3.141592f / 180.0f);
	dQuaternion rot(camMatrix);
	dVector origin(0.0, 0.0f, 120.0f, 0.0f);
	scene->SetCameraMatrix(rot*dqRotY, origin);

	NewtonSerializeToFile(scene->GetNewton(), "RocketLaunchFar.bin", NULL, NULL);
}