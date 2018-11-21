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

#include "dAnimationStdAfx.h"
#include "dAnimationAcyclicJoint.h"
#include "dAnimationCharacterRigManager.h"

dAnimationCharacterRigManager::dAnimationCharacterRigManager(NewtonWorld* const world)
	:dCustomControllerManager<dAnimationCharacterRig>(world, D_ANINAMTION_CHARACTER_RIG_NAME)
{
}

dAnimationCharacterRigManager::~dAnimationCharacterRigManager()
{
}

dAnimationCharacterRig* dAnimationCharacterRigManager::CreateCharacterRig(NewtonBody* const body, const dMatrix& vehicleFrame, NewtonApplyForceAndTorque forceAndTorque, dFloat gravityMag)
{
//	dAnimationCharacterRig* const vehicle = CreateController();
//	vehicle->Init(body, vehicleFrame, forceAndTorque, gravityMag);
//	return vehicle;
	dAssert(0);
	return NULL;
}

dAnimationCharacterRig* dAnimationCharacterRigManager::CreateCharacterRig(NewtonCollision* const chassisShape, const dMatrix& vehicleFrame, dFloat mass, NewtonApplyForceAndTorque forceAndTorque, dFloat gravityMag)
{
//	dAnimationCharacterRig* const vehicle = CreateController();
//	vehicle->Init(chassisShape, mass, vehicleFrame, forceAndTorque, gravityMag);
//	return vehicle;
	dAssert(0);
	return NULL;
}

void dAnimationCharacterRigManager::DestroyController(dAnimationCharacterRig* const vehicle)
{
	dAssert(0);
//	vehicle->Cleanup();
	dCustomControllerManager<dAnimationCharacterRig>::DestroyController(vehicle);
}

void dAnimationCharacterRigManager::OnDebug(dCustomJoint::dDebugDisplay* const debugContext)
{
	dAssert(0);
	for (dCustomControllerManager<dAnimationCharacterRig>::dListNode* vehicleNode = GetFirst(); vehicleNode; vehicleNode = vehicleNode->GetNext()) {
//		dAnimationCharacterRig* const vehicle = &vehicleNode->GetInfo();
//		vehicle->Debug(debugContext);
	}
}

