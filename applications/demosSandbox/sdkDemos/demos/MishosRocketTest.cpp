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

enum LinkType
{
	NONE,
	LINK_FIXED,
	LINK_BALLSOCKET
};
static void ParachuteLoad(const NewtonBody* body, dFloat timestep, int threadIndex);
static void Nothing(const NewtonBody* body, dFloat timestep, int threadIndex) { return; }

static NewtonBody* GetObject(NewtonWorld* const world, const dString& strName)
{
	NewtonBody* object = NULL;
	for (NewtonBody* bodyZ = NewtonWorldGetFirstBody(world); bodyZ; bodyZ = NewtonWorldGetNextBody(world, bodyZ))
	{
		DemoEntity* ent = (DemoEntity*)NewtonBodyGetUserData(bodyZ);
		const dString& name = ent->GetName();
		if (name == strName) {
			object = bodyZ;
			return object;
		}
	}
	return NULL;
}

static void   CreateCapsule(NewtonWorld* const world)
{
	dMatrix offset(dGetIdentityMatrix());
	dVector damp(0.0f, 0.0f, 0.0f);
	dFloat CapsuleMass = 10387.0f;

	DemoEntityManager* const scene = (DemoEntityManager*)NewtonWorldGetUserData(world);

	// Make Orion Capsule body...
	NewtonCollision* const CapsuleShape = NewtonCreateCylinder(world, 20.0f, 6.0f, 20.0f, 1, &offset[0][0]);
	NewtonCollision* const CapsuleCompound = NewtonCreateCompoundCollision(world, 0);
	NewtonCompoundCollisionBeginAddRemove(CapsuleCompound);
	NewtonCompoundCollisionAddSubCollision(CapsuleCompound, CapsuleShape);
	NewtonCompoundCollisionEndAddRemove(CapsuleCompound);
	dMatrix matrixCapsule(0.0f, 0.0f, 1.5708f, dVector(0.0f, 0.0f, 0.0f));

	DemoMesh* const geometryCapsule = new DemoMesh("Capsule Mesh", scene->GetShaderCache(), CapsuleCompound, "metal_30.tga", "metal_30.tga", "metal_30.tga");
	NewtonBody* const CapsuleBody = CreateSimpleSolid(scene, geometryCapsule, CapsuleMass, matrixCapsule, CapsuleCompound, 0);
//	NewtonBodySetForceAndTorqueCallback(CapsuleBody, ParachuteLoad);
	DemoEntity* const entityCapsule = (DemoEntity*)NewtonBodyGetUserData(CapsuleBody);
	entityCapsule->SetNameID("Capsule");
	NewtonBodySetLinearDamping(CapsuleBody, 0.0f);
	NewtonBodySetAngularDamping(CapsuleBody, &damp[0]);
	geometryCapsule->Release();
	NewtonDestroyCollision(CapsuleCompound);
	NewtonDestroyCollision(CapsuleShape);

	dVector origin(matrixCapsule.m_posit);
	origin.m_x -= 120.0f;
	origin.m_y += 30.0f;
	dQuaternion rot;
	scene->SetCameraMatrix(rot, origin);
}

static void   CreateParachute(NewtonWorld* const world, dVector location)
{
	dMatrix offset(dGetIdentityMatrix());
	dVector damp(0.0f, 0.0f, 0.0f);
	dFloat ParachuteMass = 20.0f;

	DemoEntityManager* const scene = (DemoEntityManager*)NewtonWorldGetUserData(world);

	// Make Parachute ...  
	NewtonCollision* const ParachuteShape = NewtonCreateCylinder(world, 0.2f, 40.0f, 40.0f, 1, &offset[0][0]);
	NewtonCollision* const ParachuteCompound = NewtonCreateCompoundCollision(world, 0);
	NewtonCompoundCollisionBeginAddRemove(ParachuteCompound);
	NewtonCompoundCollisionAddSubCollision(ParachuteCompound, ParachuteShape);
	NewtonCompoundCollisionEndAddRemove(ParachuteCompound);

	dMatrix matrixParachute(0.0f, 0.0f, 1.5708f, location + dVector(0.0f, 30.0f, 0.0f));
	DemoMesh* const geometryParachute = new DemoMesh("Parachute Stage", scene->GetShaderCache(), ParachuteCompound, "metalblu.tga", "metalblu.tga", "metalblu.tga");
	NewtonBody* const ParachuteBody = CreateSimpleSolid(scene, geometryParachute, ParachuteMass, matrixParachute, ParachuteCompound, 0);
	NewtonBodySetForceAndTorqueCallback(ParachuteBody, Nothing);
	DemoEntity* const entityParachute = (DemoEntity*)NewtonBodyGetUserData(ParachuteBody);
	entityParachute->SetNameID("Parachute");
	NewtonBodySetLinearDamping(ParachuteBody, 0.0f);
	NewtonBodySetAngularDamping(ParachuteBody, &damp[0]);
	geometryParachute->Release();
	NewtonDestroyCollision(ParachuteCompound);
	NewtonDestroyCollision(ParachuteShape);
}

static void   DeployParachute(NewtonWorld* const world, const dString& strParent, const dString& strChild, int nLinkType = NONE)
{
	static bool bOnce = false;

	if (bOnce)
		return;

	bOnce = true;

	NewtonBody* parent = GetObject(world, strParent);

	dMatrix  mParentMatrix;
	NewtonBodyGetMatrix(parent, &mParentMatrix[0][0]);
	CreateParachute(world, mParentMatrix.m_posit);

	NewtonBody* child = GetObject(world, strChild);

	switch (nLinkType)
	{
	case LINK_FIXED:
	{
		dCustomHinge* const pHinge = new dCustomHinge(mParentMatrix, child, parent);
		if (pHinge)
		{
			pHinge->EnableMotor(false, 0.0);
			pHinge->SetAsSpringDamper(false, 0.0, 0.0, 0.0);
			pHinge->EnableLimits(true);
			pHinge->SetLimits(0.0, 0.0);
		}
		else
			printf("\nFAILED TO LINK WITH FIXED JOINT\n");
	}
	break;
	case LINK_BALLSOCKET:
	{
		dMatrix pinMatrix(dGrammSchmidt(dVector(0.0f, -1.0f, 0.0f, 0.0f)));
		pinMatrix.m_posit = mParentMatrix.m_posit + dVector(0.0f, 10.0f, 0.0f, 0.0f);
		dCustomBallAndSocket* const pBASHinge = new dCustomBallAndSocket(pinMatrix, child, parent);
		if (pBASHinge)
		{
			pBASHinge->EnableCone(true);
			pBASHinge->EnableTwist(true);
			pBASHinge->SetConeLimits(30.0f * dDegreeToRad);
		}
		else
			printf("\nFAILED TO LINK THROUGH BALL AND SOCKET\n");
	}
	break;
	default:
		break;
	}
}

static void ParachuteLoad(const NewtonBody* body, dFloat timestep, int threadIndex)
{
	NewtonWorld* const world = NewtonBodyGetWorld(body);
	DemoEntityManager* const scene = (DemoEntityManager*)NewtonWorldGetUserData(world);
	bool bSpaceBar = false;

	if (scene->GetKeyState(' ')) {
		bSpaceBar = true;
	}

	if (bSpaceBar)
		DeployParachute(world, "Capsule", "Parachute", LINK_BALLSOCKET);
}

void MishosHingeTest(DemoEntityManager* const scene)
{
	scene->CreateSkyBox();
	NewtonWorld* const world = scene->GetNewton();
	CreateCapsule(world);
	DeployParachute(world, "Capsule", "Parachute", LINK_BALLSOCKET);
}
