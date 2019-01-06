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

#if 0
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
	dVector force(0.0f); 
	dVector torque(0.0f);

	if (bThrust)
	{
		// ten million newtons, so the bodies will "rise" slowly...
		//torque.m_x = 1000000000.0; 
		torque.m_x = mass * 500.0f;
	}


	const NewtonBody* body1 = NULL;
	for (NewtonBody* bodyZ = NewtonWorldGetFirstBody(world); bodyZ; bodyZ = NewtonWorldGetNextBody(world, bodyZ)) {
		if (bodyZ != body) {
			body1 = bodyZ;
		}
	}

	// pressing P prints out distance between objects...
	if (scene->GetKeyState('P')) {
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
		//printf("Object distance: %f  local posit(%f %f %f)\n", distance - 35.0f, diff[0], diff[1], diff[2]);
		dTrace(("Object distance: %f local posit(%f %f %f)\n", distance - 35.0f, diff[0], diff[1], diff[2]));
	}


	dVector posit0(0.0f);
	dVector posit1(0.0f);
	NewtonBodyGetPosition(body, &posit0[0]);
	NewtonBodyGetPosition(body1, &posit1[0]);
	dVector diff (posit1 - posit0);
	dFloat distance = dSqrt(diff.DotProduct3(diff)) - 35.0f;

	dVector veloc;
	NewtonBodyGetVelocity(body1, &veloc[0]);
	dTrace(("d(%f) v(%f %f %f)\n", distance, veloc[0], veloc[1], veloc[2]));

	// Apply torque only to the Core object
	NewtonBodyAddTorque(body, &torque.m_x);
}


void MishosHingeTest(DemoEntityManager* const scene)
{
	scene->CreateSkyBox();

	NewtonWorld* const world = scene->GetNewton();
	dMatrix offset(dGetIdentityMatrix());
	dVector damp(0.0f, 0.0f, 0.0f);

	dFloat CoreMass = 979452.0f;
	dFloat PayloadMass = 979452.0f;

	CoreMass = 1.0f;
	PayloadMass = 1.0f;


	// Make rocket core body...
//	NewtonCollision* const CoreShape = NewtonCreateCylinder(world, 5.0f, 5.0f, 60.0f, 1, &offset[0][0]);
	NewtonCollision* const CoreShape = NewtonCreateSphere(world, 5.0f, 1, &offset[0][0]);
	NewtonCollision* const CoreCompound = NewtonCreateCompoundCollision(world, 0);
	NewtonCompoundCollisionBeginAddRemove(CoreCompound);
	NewtonCompoundCollisionAddSubCollision(CoreCompound, CoreShape);
	NewtonCompoundCollisionEndAddRemove(CoreCompound);
//	dMatrix matrixCore(0.0f, 0.0f, 1.5708f, dVector(0.0f, -30.0f, 0.0f));
	dMatrix matrixCore(0.0f, 0.0f, 1.5708f, dVector(0.0f, 0.0f, 0.0f));
	
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
	//NewtonCollision* const PayloadShape = NewtonCreateCylinder(world, 5.0f, 5.0f, 10.0f, 1, &offset[0][0]);
	NewtonCollision* const PayloadShape = NewtonCreateSphere(world, 5.0f, 1, &offset[0][0]);
	NewtonCollision* const PayloadCompound = NewtonCreateCompoundCollision(world, 0);
	NewtonCompoundCollisionBeginAddRemove(PayloadCompound);
	NewtonCompoundCollisionAddSubCollision(PayloadCompound, PayloadShape);
	NewtonCompoundCollisionEndAddRemove(PayloadCompound);
//	dMatrix matrixPayload(0.0f, 0.0f, 1.5708f, dVector(0.0f, 5.0f, 0.0f));
	dMatrix matrixPayload(0.0f, 0.0f, 1.5708f, dVector(0.0f, 35.0f, 0.0f));
	
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

#if 1
	dFloat w = 10.0f;
	dVector omega(w, 0.0f, 0.0f, 0.0f);
	NewtonBodySetOmega(CoreBody, &omega[0]);
	NewtonBodySetOmega(PayloadBody, &omega[0]);

	dVector veloc(0.0f, 0.0f, 0.5f * 35.0f * w, 0.0f);
	NewtonBodySetVelocity(PayloadBody, &veloc[0]);

	veloc.m_z *= -1.0f;
	NewtonBodySetVelocity(CoreBody, &veloc[0]);
#endif	

	// Lock these two bodies with a hard hinge...
	dMatrix  mSourceMatrix;
	NewtonBodyGetMatrix(PayloadBody, &mSourceMatrix[0][0]);
//	dCustomHinge* pHinge = NULL;
//	pHinge = new dCustomHinge(mSourceMatrix, PayloadBody, CoreBody);
//	pHinge->EnableMotor(false, 0.0);
//	pHinge->SetAsSpringDamper(false, 0.0, 0.0, 0.0);
//	pHinge->EnableLimits(true);
//	pHinge->SetLimits(0.0, 0.0);
	new dCustomBallAndSocket(mSourceMatrix, PayloadBody, CoreBody);

	// place camera into position and rotate towards objects
//	dMatrix camMatrix(dGetIdentityMatrix());
//	dQuaternion dqRotY(dVector(0.0f, 1.0f, 0.0f), 90.0f * 3.141592f / 180.0f);
//	dQuaternion rot(camMatrix);
//	dVector origin(0.0, 0.0f, 120.0f, 0.0f);
//	scene->SetCameraMatrix(rot*dqRotY, origin);
	dVector origin(matrixCore.m_posit);
	origin.m_x -= 120.0f;
	dQuaternion rot;
	scene->SetCameraMatrix(rot, origin);

	NewtonSerializeToFile(scene->GetNewton(), "RocketLaunchFar.bin", NULL, NULL);
}

#else

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


dCustomHinge* pHinge1 = NULL;
dCustomHinge* pHinge[3] = { NULL, NULL, NULL };

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
	dVector force(0.0f);
	dVector torque(0.0f);

	if (bThrust)
	{
		// ten million newtons, so the bodies will "rise" slowly...
		//torque.m_x = 1000000000.0; 
		torque.m_x = mass * 500.0f;
	}


	const NewtonBody* body1 = NULL;
	for (NewtonBody* bodyZ = NewtonWorldGetFirstBody(world); bodyZ; bodyZ = NewtonWorldGetNextBody(world, bodyZ)) {
		if (bodyZ != body) {
			body1 = bodyZ;
		}
	}

	// pressing P prints out distance between objects...
	if (scene->GetKeyState('P')) {
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
		//printf("Object distance: %f  local posit(%f %f %f)\n", distance - 35.0f, diff[0], diff[1], diff[2]);
		dTrace(("Object distance: %f local posit(%f %f %f)\n", distance - 35.0f, diff[0], diff[1], diff[2]));
	}


	dVector posit0(0.0f);
	dVector posit1(0.0f);
	NewtonBodyGetPosition(body, &posit0[0]);
	NewtonBodyGetPosition(body1, &posit1[0]);
	dVector diff(posit1 - posit0);
	dFloat distance = dSqrt(diff.DotProduct3(diff)) - 35.0f;

	dVector veloc;
	NewtonBodyGetVelocity(body1, &veloc[0]);
	dTrace(("d(%f) v(%f %f %f)\n", distance, veloc[0], veloc[1], veloc[2]));

	// Apply torque only to the Core object
	NewtonBodyAddTorque(body, &torque.m_x);
}

dMap<dCustomJoint*, dCustomJoint*> garbageCollectedJoints;
NewtonPostUpdateCallback gParentCallback;

static void GarbageCollectJoints(const NewtonWorld* const world, dFloat timestep)
{
	if (gParentCallback) {
		gParentCallback(world, timestep);
	}

	while (garbageCollectedJoints.GetCount()) {
		delete garbageCollectedJoints.GetRoot()->GetInfo();
		garbageCollectedJoints.Remove(garbageCollectedJoints.GetRoot());
	}
}

static void BreakHinge(const NewtonBody* body, dFloat timestep, int threadIndex)
{
	NewtonWorld* const world = NewtonBodyGetWorld(body);
	DemoEntityManager* const scene = (DemoEntityManager*)NewtonWorldGetUserData(world);

	static unsigned atomicLock = 0;
	if (scene->GetKeyState('1')) {
		dCustomScopeLock lock(&atomicLock);
		if (!garbageCollectedJoints.Find(pHinge1)) {
			garbageCollectedJoints.Insert(pHinge1, pHinge1);
			pHinge1 = NULL;
		}
	}
	if (scene->GetKeyState('2')) {
		dCustomScopeLock lock(&atomicLock);
		if (pHinge[0]) {
			if (!garbageCollectedJoints.Find(pHinge[0])) {
				garbageCollectedJoints.Insert(pHinge[0], pHinge[0]);
				pHinge[0] = NULL;
			}
			if (!garbageCollectedJoints.Find(pHinge[1])) {
				garbageCollectedJoints.Insert(pHinge[1], pHinge[1]);
				pHinge[1] = NULL;
			}
			if (!garbageCollectedJoints.Find(pHinge[2])) {
				garbageCollectedJoints.Insert(pHinge[2], pHinge[2]);
				pHinge[2] = NULL;
			}
		}
	}
}

void MishosHingeTest(DemoEntityManager* const scene)
{
	scene->CreateSkyBox();

	NewtonWorld* const world = scene->GetNewton();
	dMatrix offset(dGetIdentityMatrix());
	dVector damp(0.0f, 0.0f, 0.0f);

	dFloat CoreMass = 979452.0f;
	dFloat PayloadMass = 979452.0f;
	dFloat CVXMass = 979452.0f;

	double dVeryFar = 6371000.0f;
	//  double dVeryFar = 0.0f;

	//CoreMass = 1.0f;
	//PayloadMass = 1.0f;
	//CVXMass = 1.0f;

	// Make rocket core body...
	NewtonCollision* const CoreShape = NewtonCreateCylinder(world, 3.0f, 3.0f, 10.0f, 1, &offset[0][0]);
	NewtonCollision* const CoreCompound = NewtonCreateCompoundCollision(world, 0);
	NewtonCompoundCollisionBeginAddRemove(CoreCompound);
	NewtonCompoundCollisionAddSubCollision(CoreCompound, CoreShape);
	NewtonCompoundCollisionEndAddRemove(CoreCompound);
	//  dMatrix matrixCore(0.0f, 0.0f, 1.5708f, dVector(0.0f, -30.0f, 0.0f));
	dMatrix matrixCore(0.0f, 0.0f, 1.5708f, dVector(dVeryFar, 0.0f, 0.0f));

	DemoMesh* const geometryCore = new DemoMesh("Core Mesh", CoreCompound, "metal_30.tga", "metal_30.tga", "metal_30.tga");
	NewtonBody* const CoreBody = CreateSimpleSolid(scene, geometryCore, CoreMass, matrixCore, CoreCompound, 0);
	NewtonBodySetForceAndTorqueCallback(CoreBody, BreakHinge);
	DemoEntity* const entityCore = (DemoEntity*)NewtonBodyGetUserData(CoreBody);
	entityCore->SetNameID("Core");
	NewtonBodySetLinearDamping(CoreBody, 0.0f);
	NewtonBodySetAngularDamping(CoreBody, &damp[0]);
	geometryCore->Release();
	NewtonDestroyCollision(CoreCompound);
	NewtonDestroyCollision(CoreShape);

	// Make payload ...  
	NewtonCollision* const PayloadShape = NewtonCreateCylinder(world, 2.7f, 1.5f, 5.0f, 1, &offset[0][0]);
	NewtonCollision* const PayloadCompound = NewtonCreateCompoundCollision(world, 0);
	NewtonCompoundCollisionBeginAddRemove(PayloadCompound);
	NewtonCompoundCollisionAddSubCollision(PayloadCompound, PayloadShape);
	NewtonCompoundCollisionEndAddRemove(PayloadCompound);
	//  dMatrix matrixPayload(0.0f, 0.0f, 1.5708f, dVector(0.0f, 5.0f, 0.0f));
	dMatrix matrixPayload(0.0f, 0.0f, 1.5708f, dVector(dVeryFar, 7.5f, 0.0f));

	DemoMesh* const geometryPayload = new DemoMesh("Payload Stage", PayloadCompound, "metal_30.tga", "metal_30.tga", "metal_30.tga");
	NewtonBody* const PayloadBody = CreateSimpleSolid(scene, geometryPayload, PayloadMass, matrixPayload, PayloadCompound, 0);
	NewtonBodySetForceAndTorqueCallback(PayloadBody, BreakHinge);
	DemoEntity* const entityPayload = (DemoEntity*)NewtonBodyGetUserData(PayloadBody);
	entityPayload->SetNameID("Payload");
	NewtonBodySetLinearDamping(PayloadBody, 0.0f);
	NewtonBodySetAngularDamping(PayloadBody, &damp[0]);
	geometryPayload->Release();
	NewtonDestroyCollision(PayloadCompound);
	NewtonDestroyCollision(PayloadShape);

	NewtonCollision* CVXCompound;
	NewtonCollision* CVXShape;
	dMatrix matrixCVX;
	DemoMesh* geometryCVX;
	NewtonBody* CVXBody[3];

	dVector positions[] = { dVector(dVeryFar + 0.0f, 7.2f, 1.982f), dVector(dVeryFar + 1.717f, 7.2f, -0.991f),dVector(dVeryFar + -1.717f, 7.2f,-0.991f) };


	for (int i = 0; i < 3; i++)
	{
		// Make convex Body ...  
		CVXCompound = NewtonCreateCompoundCollision(world, 0);
		NewtonCompoundCollisionBeginAddRemove(CVXCompound);

		offset = dMatrix(0.0f, 0.0f, 0.0f, dVector(0.0f, 0.0f, 0.537f));
		CVXShape = NewtonCreateBox(world, 1.791f, 4.4f, 0.082f, 1, &offset[0][0]);
		NewtonCompoundCollisionAddSubCollision(CVXCompound, CVXShape);
		NewtonDestroyCollision(CVXShape);

		offset = dMatrix(0.0f, 0.698f, 0.0f, dVector(1.62f, 0.0f, -0.052f));
		CVXShape = NewtonCreateBox(world, 1.791f, 4.4f, 0.082f, 1, &offset[0][0]);
		NewtonCompoundCollisionAddSubCollision(CVXCompound, CVXShape);
		NewtonDestroyCollision(CVXShape);

		offset = dMatrix(0.0f, -0.698f, 0.0f, dVector(-1.62f, 0.0f, -0.052f));
		CVXShape = NewtonCreateBox(world, 1.791f, 4.4f, 0.082f, 1, &offset[0][0]);
		NewtonCompoundCollisionAddSubCollision(CVXCompound, CVXShape);
		NewtonDestroyCollision(CVXShape);

		NewtonCompoundCollisionEndAddRemove(CVXCompound);
		matrixCVX = dMatrix(0.0f, i*2.09439f, 0.0f, positions[i]);

		geometryCVX = new DemoMesh("CVX Stage", CVXCompound, "metal_30.tga", "metal_30.tga", "metal_30.tga");
		CVXBody[i] = CreateSimpleSolid(scene, geometryCVX, CVXMass, matrixCVX, CVXCompound, 0);
		NewtonDestroyCollision(CVXCompound);
		geometryCVX->Release();
		NewtonBodySetForceAndTorqueCallback(CVXBody[i], BreakHinge);
		NewtonBodySetLinearDamping(CVXBody[i], 0.0f);
		NewtonBodySetAngularDamping(CVXBody[i], &damp[0]);

		// Lock shroud to upper cylinder
		dMatrix  mSourceMatrix;
		NewtonBodyGetMatrix(CVXBody[i], &mSourceMatrix[0][0]);
		pHinge[i] = new dCustomHinge(mSourceMatrix, CVXBody[i], PayloadBody);
		pHinge[i]->EnableMotor(false, 0.0);
		pHinge[i]->SetAsSpringDamper(false, 0.0, 0.0, 0.0);
		pHinge[i]->EnableLimits(true);
		pHinge[i]->SetLimits(0.0, 0.0);
		pHinge[i]->SetStiffness(1.00f);
	}

	// Lock cylinders
	dMatrix  mSourceMatrix;
	NewtonBodyGetMatrix(PayloadBody, &mSourceMatrix[0][0]);

	pHinge1 = new dCustomHinge(mSourceMatrix, PayloadBody, CoreBody);
	pHinge1->EnableMotor(false, 0.0);
	pHinge1->SetAsSpringDamper(false, 0.0, 0.0, 0.0);
	pHinge1->EnableLimits(true);
	pHinge1->SetLimits(0.0, 0.0);
	pHinge1->SetStiffness(1.00f);

	dVector origin(matrixCore.m_posit);
	origin.m_x -= 20.0f;
	origin.m_y += 20.0f;
	dQuaternion rot(dVector(0.0f, 0.0f, 1.0f), -0.6f);
	scene->SetCameraMatrix(rot, origin);

	NewtonSerializeToFile(scene->GetNewton(), "RocketLaunchFar.bin", NULL, NULL);


	// chain the call back so that we cal delete object after the newton update
	gParentCallback = NewtonGetPostUpdateCallback(world);
	NewtonSetPostUpdateCallback(world, GarbageCollectJoints);
}


void MishosHingeTestOLD(DemoEntityManager* const scene)
{
	scene->CreateSkyBox();

	NewtonWorld* const world = scene->GetNewton();
	dMatrix offset(dGetIdentityMatrix());
	dVector damp(0.0f, 0.0f, 0.0f);

	dFloat CoreMass = 979452.0f;
	dFloat PayloadMass = 979452.0f;

	CoreMass = 1.0f;
	PayloadMass = 1.0f;

	// chain the call back so that we cal delete object after the newton update
	gParentCallback = NewtonGetPostUpdateCallback(world);
	NewtonSetPostUpdateCallback(world, GarbageCollectJoints);

	// Make rocket core body...
	//  NewtonCollision* const CoreShape = NewtonCreateCylinder(world, 5.0f, 5.0f, 60.0f, 1, &offset[0][0]);
	NewtonCollision* const CoreShape = NewtonCreateSphere(world, 5.0f, 1, &offset[0][0]);
	NewtonCollision* const CoreCompound = NewtonCreateCompoundCollision(world, 0);
	NewtonCompoundCollisionBeginAddRemove(CoreCompound);
	NewtonCompoundCollisionAddSubCollision(CoreCompound, CoreShape);
	NewtonCompoundCollisionEndAddRemove(CoreCompound);
	//  dMatrix matrixCore(0.0f, 0.0f, 1.5708f, dVector(0.0f, -30.0f, 0.0f));
	dMatrix matrixCore(0.0f, 0.0f, 1.5708f, dVector(0.0f, 0.0f, 0.0f));

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
	//NewtonCollision* const PayloadShape = NewtonCreateCylinder(world, 5.0f, 5.0f, 10.0f, 1, &offset[0][0]);
	NewtonCollision* const PayloadShape = NewtonCreateSphere(world, 5.0f, 1, &offset[0][0]);
	NewtonCollision* const PayloadCompound = NewtonCreateCompoundCollision(world, 0);
	NewtonCompoundCollisionBeginAddRemove(PayloadCompound);
	NewtonCompoundCollisionAddSubCollision(PayloadCompound, PayloadShape);
	NewtonCompoundCollisionEndAddRemove(PayloadCompound);
	//  dMatrix matrixPayload(0.0f, 0.0f, 1.5708f, dVector(0.0f, 5.0f, 0.0f));
	dMatrix matrixPayload(0.0f, 0.0f, 1.5708f, dVector(0.0f, 35.0f, 0.0f));

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

#if 1
	dFloat w = 10.0f;
	dVector omega(w, 0.0f, 0.0f, 0.0f);
	NewtonBodySetOmega(CoreBody, &omega[0]);
	NewtonBodySetOmega(PayloadBody, &omega[0]);

	dVector veloc(0.0f, 0.0f, 0.5f * 35.0f * w, 0.0f);
	NewtonBodySetVelocity(PayloadBody, &veloc[0]);

	veloc.m_z *= -1.0f;
	NewtonBodySetVelocity(CoreBody, &veloc[0]);
#endif  

	// Lock these two bodies with a hard hinge...
	dMatrix  mSourceMatrix;
	NewtonBodyGetMatrix(PayloadBody, &mSourceMatrix[0][0]);
	//  dCustomHinge* pHinge = NULL;
	//  pHinge = new dCustomHinge(mSourceMatrix, PayloadBody, CoreBody);
	//  pHinge->EnableMotor(false, 0.0);
	//  pHinge->SetAsSpringDamper(false, 0.0, 0.0, 0.0);
	//  pHinge->EnableLimits(true);
	//  pHinge->SetLimits(0.0, 0.0);
	new dCustomBallAndSocket(mSourceMatrix, PayloadBody, CoreBody);

	// place camera into position and rotate towards objects
	//  dMatrix camMatrix(dGetIdentityMatrix());
	//  dQuaternion dqRotY(dVector(0.0f, 1.0f, 0.0f), 90.0f * 3.141592f / 180.0f);
	//  dQuaternion rot(camMatrix);
	//  dVector origin(0.0, 0.0f, 120.0f, 0.0f);
	//  scene->SetCameraMatrix(rot*dqRotY, origin);
	dVector origin(matrixCore.m_posit);
	origin.m_x -= 120.0f;
	dQuaternion rot;
	scene->SetCameraMatrix(rot, origin);

	NewtonSerializeToFile(scene->GetNewton(), "RocketLaunchFar.bin", NULL, NULL);
}

#endif