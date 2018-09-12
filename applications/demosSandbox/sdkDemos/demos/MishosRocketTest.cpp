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

#if 0
	// mass in "MishosRocketTest" is set to one million kg
	dVector force(dVector(0.0f, 1.0f, 0.0f).Scale(mass * DEMO_GRAVITY)); // Force calculated here is 9.8 million N

	dVector thrust(dVector(0.0f, 1.0f, 0.0f).Scale(0.0));
	if (bThrust)
		thrust = dVector(0.0f, 1.0f, 0.0f).Scale(10000000.0); // ten million newtons, so the bodies will "rise" slowly...
	else
		thrust = dVector(0.0f, 1.0f, 0.0f).Scale(0.0); // ten million newtons, so the bodies will "rise" slowly...

	dVector resultant = thrust + force;
	NewtonBodySetForce(body, &resultant.m_x);

#else
	dVector force(dVector(0.0f, 0.0f, 0.0f).Scale(0.0f)); // no gravity for this example

	dVector torque(dVector(0.0f, 0.0f, 1.0f).Scale(0.0));
	if (bThrust)
	{
		torque = dVector(0.0f, 0.0f, 1.0f).Scale(10000000.0); // ten million newtons, so the bodies will "rise" slowly...
	} else
		torque = dVector(0.0f, 0.0f, 1.0f).Scale(0.0); // ten million newtons, so the bodies will "rise" slowly...

													   // pressing P prints out distance between objects...
	if (scene->GetKeyState('P')) {
		int counter = 0;
		dVector bodyVectors[2];
		for (NewtonBody* bodyZ = NewtonWorldGetFirstBody(world); bodyZ; bodyZ = NewtonWorldGetNextBody(world, bodyZ)) {
			dVector positionVector;
			NewtonBodyGetPosition(bodyZ, &positionVector[0]);
			bodyVectors[counter] = positionVector;
			counter++;
		}
		//printf("position body 1: %f %f %f\n", bodyVectors[0].m_x, bodyVectors[0].m_y, bodyVectors[0].m_z);
		//printf("position body 2: %f %f %f\n", bodyVectors[1].m_x, bodyVectors[1].m_y, bodyVectors[1].m_z);
		dVector diff = bodyVectors[1] - bodyVectors[0];
		double distance = dSqrt(diff.DotProduct3(diff));
		printf("Object distance: %f\n", distance - 35.0f); // Nominal object distance at the beginning is 35, so we will subtract that here to get to zero as a starting point.
	}
	dVector resultant = torque + force; // force is null, no gravity...

										// Apply torque only to the Core object, not Payload
	DemoEntity* const entity = (DemoEntity*)NewtonBodyGetUserData(body);
	if (entity->GetName().Find("Core") != -1)
		NewtonBodyAddTorque(body, &resultant.m_x);
#endif
}

/*
void MishosRocketTest(DemoEntityManager* const scene)
{
	// set DEMO_GRAVITY constant to turn on off gravity

	scene->CreateSkyBox();

	//NewtonBody* const levelMeshBody = CreateLevelMesh(scene, "flatPlane.ngd", 1);
	//dMatrix offsetPlane(dGetIdentityMatrix());
	////offsetPlane.m_posit.m_y = 6378135.0f;
	//offsetPlane.m_posit.m_y = 20.0f;
	//NewtonBodySetMatrix(levelMeshBody, &offsetPlane[0][0]);

	NewtonWorld* const world = scene->GetNewton();
	dMatrix offset(dGetIdentityMatrix());
	dFloat vertOffset = 6378135.0f;
	//dFloat vertOffset = 0.0f;
	dVector damp(0.0f);

	// Make Launch platform as a solid static box...
	NewtonCollision* const CoreShape = NewtonCreateBox(world, 20.0f, 10.0f, 20.0f, 0, &offset[0][0]);
	NewtonCollision* const CoreCompound = NewtonCreateSceneCollision(world, 0);
	NewtonSceneCollisionBeginAddRemove(CoreCompound);
	NewtonSceneCollisionAddSubCollision(CoreCompound, CoreShape);
	NewtonSceneCollisionEndAddRemove(CoreCompound);
	dMatrix matrixCore(0.0f, 0.0f, 0.0f, dVector(0.0f, vertOffset - 5.0f, 0.0f));
	dFloat CoreMass = 979452.0f; // irrelevant, this is a static object, here for conformity
	DemoMesh* const geometryCore = new DemoMesh("Launch Platform", CoreCompound, "metal_30.tga", "metal_30.tga", "metal_30.tga");
	NewtonBody* const CoreBody = CreateSimpleSolid(scene, geometryCore, CoreMass, matrixCore, CoreCompound, 0);
	//NewtonBodySetMassProperties(PayloadBody, PayloadMass, PayloadCompound);
	NewtonBodySetLinearDamping(CoreBody, 0.0f);
	NewtonBodySetAngularDamping(CoreBody, &damp[0]);
	geometryCore->Release();
	NewtonDestroyCollision(CoreCompound);
	NewtonDestroyCollision(CoreShape);

	// Make Core Stage
	NewtonCollision* const PayloadShape = NewtonCreateCylinder(world, 4.25f, 4.25f, 59.8f, 1, &offset[0][0]);
	NewtonCollision* const PayloadCompound = NewtonCreateCompoundCollision(world, 0);
	NewtonCompoundCollisionBeginAddRemove(PayloadCompound);
	NewtonCompoundCollisionAddSubCollision(PayloadCompound, PayloadShape);
	NewtonCompoundCollisionEndAddRemove(PayloadCompound);
	dMatrix matrixPayload(0.0f, 0.0f, 1.5708f, dVector(0.0f, vertOffset + 30.1f, 0.0f));
	dFloat PayloadMass = 979452.0f;
	DemoMesh* const geometryPayload = new DemoMesh("CoreStage", PayloadCompound, "metal_30.tga", "metal_30.tga", "metal_30.tga");
	NewtonBody* const PayloadBody = CreateSimpleSolid(scene, geometryPayload, PayloadMass, matrixPayload, PayloadCompound, 0);
	//NewtonBodySetMassProperties(PayloadBody, PayloadMass, PayloadCompound);
	NewtonBodySetLinearDamping(PayloadBody, 0.0f);
	NewtonBodySetAngularDamping(PayloadBody, &damp[0]);
	geometryPayload->Release();
	NewtonDestroyCollision(PayloadCompound);
	NewtonDestroyCollision(PayloadShape);

	// place camera into position and rotate towards objects
	dMatrix camMatrix(dGetIdentityMatrix());
	dQuaternion dqRotY(dVector(0.0f, 1.0f, 0.0f), 90.0f * 3.141592f / 180.0f);
	dQuaternion rot(camMatrix);
	dVector origin(0.0, vertOffset + 5.0f, 120.0f, 0.0f);
	scene->SetCameraMatrix(rot*dqRotY, origin);

	NewtonSerializeToFile(scene->GetNewton(), "RocketLaunchFar.bin", NULL, NULL);
}
*/

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
	NewtonBodySetForceAndTorqueCallback(PayloadBody, ApplyGravityForce);
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