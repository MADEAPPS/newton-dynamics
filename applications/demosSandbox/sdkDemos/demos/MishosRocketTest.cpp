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
	DemoEntity* ent = (DemoEntity*)NewtonBodyGetUserData(body);

	const dString& name = ent->GetName();

	bool bSpaceBar = false;

	if (scene->GetKeyState(' ')) {
		bSpaceBar = true;
	}
	NewtonBodyGetMass(body, &mass, &Ixx, &Iyy, &Izz);

	// no gravity for this example
	dVector force(0.0f);
	dVector torque(0.0f);

	if (bSpaceBar)
	{
		force.m_y = 1779288.64f; // LAS rocket motor output, Newtons
	}

	const NewtonBody* otherBody = NULL;
	for (NewtonBody* bodyZ = NewtonWorldGetFirstBody(world); bodyZ; bodyZ = NewtonWorldGetNextBody(world, bodyZ)) {
		if (bodyZ != body) {
			otherBody = bodyZ;
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

		NewtonBodyGetPosition(otherBody, &positionVector[0]);
		bodyVectors[1] = matrix.UntransformVector(positionVector);

#ifdef _DEBUG
		dVector diff = bodyVectors[1] - bodyVectors[0];
		dFloat distance = dSqrt(diff.DotProduct3(diff));
		//Nominal object distance at the beginning is 35, so we will subtract that here to get to zero as a starting point.
		printf("Object distance: %f  local posit(%f %f %f)\n", distance - 35.0f, diff[0], diff[1], diff[2]);
		dTrace(("Object distance: %f local posit(%f %f %f)\n", distance - 35.0f, diff[0], diff[1], diff[2]));
#endif
	}

#ifdef _DEBUG
	dVector posit0(0.0f);
	dVector posit1(0.0f);
	NewtonBodyGetPosition(body, &posit0[0]);
	NewtonBodyGetPosition(otherBody, &posit1[0]);
	dVector diff(posit1 - posit0);
	dFloat distance = dSqrt(diff.DotProduct3(diff));

	dVector veloc;
	NewtonBodyGetVelocity(otherBody, &veloc[0]);
	dTrace(("d(%f) v(%f %f %f)\n", distance, veloc[0], veloc[1], veloc[2]));
	if (bSpaceBar)
	{
		printf("Object distance: %f  local posit(%f %f %f)\n", distance - 30.0f, diff[0], diff[1], diff[2]);
	}
#endif
	
	static dString las("LAS");
	if (name == las)
		NewtonBodyAddForce(body, &force.m_x);
}


void MishosHingeTest(DemoEntityManager* const scene)
{
	scene->CreateSkyBox();

	NewtonWorld* const world = scene->GetNewton();
	dMatrix offset(dGetIdentityMatrix());
	dVector damp(0.0f, 0.0f, 0.0f);

	dFloat CapsuleMass = 10387.0f;
	dFloat LASMass = 1000.0f;

	// Make Orion Capsule body...
	NewtonCollision* const CapsuleShape = NewtonCreateCylinder(world, 20.0f, 6.0f, 20.0f, 1, &offset[0][0]);
	NewtonCollision* const CapsuleCompound = NewtonCreateCompoundCollision(world, 0);
	NewtonCompoundCollisionBeginAddRemove(CapsuleCompound);
	NewtonCompoundCollisionAddSubCollision(CapsuleCompound, CapsuleShape);
	NewtonCompoundCollisionEndAddRemove(CapsuleCompound);
	dMatrix matrixCapsule(0.0f, 0.0f, 1.5708f, dVector(0.0f, 0.0f, 0.0f));

	DemoMesh* const geometryCapsule = new DemoMesh("Capsule Mesh", scene->GetShaderCache(), CapsuleCompound, "metal_30.tga", "metal_30.tga", "metal_30.tga");
	NewtonBody* const CapsuleBody = CreateSimpleSolid(scene, geometryCapsule, CapsuleMass, matrixCapsule, CapsuleCompound, 0);
	NewtonBodySetForceAndTorqueCallback(CapsuleBody, ApplyGravityForce);
	DemoEntity* const entityCapsule = (DemoEntity*)NewtonBodyGetUserData(CapsuleBody);
	entityCapsule->SetNameID("Capsule");
	NewtonBodySetLinearDamping(CapsuleBody, 0.0f);
	NewtonBodySetAngularDamping(CapsuleBody, &damp[0]);
	geometryCapsule->Release();
	NewtonDestroyCollision(CapsuleCompound);
	NewtonDestroyCollision(CapsuleShape);

	// Make LAS ...  
	NewtonCollision* const LASShape = NewtonCreateCylinder(world, 6.0f, 2.0f, 40.0f, 1, &offset[0][0]);
	NewtonCollision* const LASCompound = NewtonCreateCompoundCollision(world, 0);
	NewtonCompoundCollisionBeginAddRemove(LASCompound);
	NewtonCompoundCollisionAddSubCollision(LASCompound, LASShape);
	NewtonCompoundCollisionEndAddRemove(LASCompound);

	dMatrix matrixLAS(0.0f, 0.0f, 1.5708f, dVector(0.0f, 30.0f, 0.0f));
	DemoMesh* const geometryLAS = new DemoMesh("LAS Stage", scene->GetShaderCache(), LASCompound, "metalblu.tga", "metalblu.tga", "metalblu.tga");
	NewtonBody* const LASBody = CreateSimpleSolid(scene, geometryLAS, LASMass, matrixLAS, LASCompound, 0);
	NewtonBodySetForceAndTorqueCallback(LASBody, ApplyGravityForce);
	DemoEntity* const entityLAS = (DemoEntity*)NewtonBodyGetUserData(LASBody);
	entityLAS->SetNameID("LAS");
	NewtonBodySetLinearDamping(LASBody, 0.0f);
	NewtonBodySetAngularDamping(LASBody, &damp[0]);
	geometryLAS->Release();
	NewtonDestroyCollision(LASCompound);
	NewtonDestroyCollision(LASShape);

#if 0
	dFloat w = 10.0f;
	dVector omega(w, 0.0f, 0.0f, 0.0f);
	NewtonBodySetOmega(CapsuleBody, &omega[0]);
	NewtonBodySetOmega(LASBody, &omega[0]);

	dVector veloc(0.0f, 0.0f, 0.5f * 35.0f * w, 0.0f);
	NewtonBodySetVelocity(LASBody, &veloc[0]);

	veloc.m_z *= -1.0f;
	NewtonBodySetVelocity(CapsuleBody, &veloc[0]);
#endif

	// Lock these two bodies with a hard hinge...
	dMatrix mSourceMatrix;
	NewtonBodyGetMatrix(LASBody, &mSourceMatrix[0][0]);
	dCustomHinge* pHinge = NULL;
	pHinge = new dCustomHinge(mSourceMatrix, LASBody, CapsuleBody);
	pHinge->EnableMotor(false, 0.0);
	pHinge->SetAsSpringDamper(false, 0.0, 0.0, 0.0);
	pHinge->EnableLimits(true);
	pHinge->SetLimits(0.0, 0.0);

	dVector origin(matrixCapsule.m_posit);
	origin.m_x -= 120.0f;
	origin.m_y += 60.0f;
	dQuaternion rot;
	scene->SetCameraMatrix(rot, origin);
}
