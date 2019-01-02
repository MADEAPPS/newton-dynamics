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
#include "dAnimationRigJoint.h"
#include "dAnimationInverseDynamicsManager.h"

dAnimationInverseDynamicsManager::dAnimationInverseDynamicsManager(NewtonWorld* const world)
	:dCustomControllerManager<dAnimationInverseDynamicsController>(world, D_ANINAMTION_INVERSE_DYNAMICS_MANAGER)
{
}

dAnimationInverseDynamicsManager::~dAnimationInverseDynamicsManager()
{
}

dAnimationInverseDynamicsController* dAnimationInverseDynamicsManager::CreateCharacterRig(NewtonBody* const body, const dMatrix& localFrame)
{
	dAnimationInverseDynamicsController* const rig = CreateController();
	rig->Init(body, localFrame);
	return rig;
}

dAnimationInverseDynamicsController* dAnimationInverseDynamicsManager::CreateCharacterRig(NewtonCollision* const chassisShape, const dMatrix& rigFrame, dFloat mass, NewtonApplyForceAndTorque forceAndTorque, dFloat gravityMag)
{
//	dAnimationCharacterRig* const rig = CreateController();
//	rig->Init(chassisShape, mass, rigFrame, forceAndTorque, gravityMag);
//	return rig;
	dAssert(0);
	return NULL;
}

void dAnimationInverseDynamicsManager::DestroyController(dAnimationInverseDynamicsController* const rig)
{
//	rig->Cleanup();
	dCustomControllerManager<dAnimationInverseDynamicsController>::DestroyController(rig);
}

void dAnimationInverseDynamicsManager::OnDebug(dCustomJoint::dDebugDisplay* const debugContext)
{
	for (dCustomControllerManager<dAnimationInverseDynamicsController>::dListNode* rigNode = GetFirst(); rigNode; rigNode = rigNode->GetNext()) {
		dAnimationInverseDynamicsController* const rig = &rigNode->GetInfo();
		rig->Debug(debugContext);
	}
}

