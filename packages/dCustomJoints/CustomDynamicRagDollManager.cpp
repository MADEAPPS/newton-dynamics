/* Copyright (c) <2009> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/

//////////////////////////////////////////////////////////////////////
#include <CustomJointLibraryStdAfx.h>
#include <CustomDynamicRagDollManager.h>

CustomDynamicRagDollManager::CustomDynamicRagDollManager(NewtonWorld* const world)
	:CustomArticulaledTransformManager(world, DYNAMIC_RAGDOLL_PLUGIN_NAME)
{
	dAssert (0);
}

CustomDynamicRagDollManager::~CustomDynamicRagDollManager()
{
	dAssert (0);
}


void CustomDynamicRagDollManager::Debug() const 
{
	dAssert (0);
}

CustomArticulatedTransformController* CustomDynamicRagDollManager::CreateTransformController(void* const userData)
{
	dAssert (0);
	CustomArticulatedTransformController* const controller = CustomArticulaledTransformManager::CreateTransformController(userData);
	return controller;
}

void CustomDynamicRagDollManager::OnPreUpdate(CustomArticulatedTransformController* const constroller, dFloat timestep, int threadIndex) const
{
	dAssert (0);
}

void CustomDynamicRagDollManager::OnUpdateTransform(const CustomArticulatedTransformController::dSkeletonBone* const bone, const dMatrix& localMatrix) const
{
	dAssert (0);
}
