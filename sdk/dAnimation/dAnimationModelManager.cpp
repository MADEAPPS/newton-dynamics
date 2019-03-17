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
#include "dAnimationModelManager.h"


dAnimationModelManager::dAnimationModelManager(NewtonWorld* const world, const char* const name)
	:dCustomListener(world, name)
{
}

dAnimationModelManager::~dAnimationModelManager()
{
}

dAnimationJointRoot* dAnimationModelManager::CreateModel(NewtonBody* const bone, const dMatrix& bindMatrix)
{
	dAssert (0);
	return NULL;
}

void dAnimationModelManager::DestroyModel(dAnimationJointRoot* const model)
{
	dAssert (0);
}

void dAnimationModelManager::OnDestroy()
{
	for (dList<dAnimationJointRoot>::dListNode* node = m_controllerList.GetFirst(); node;) {
		dAssert (0);
		//dCustomTransformController* const controller = &node->GetInfo();
		//node = node->GetNext();
		//DestroyController(controller);
	}

}

void dAnimationModelManager::PreUpdate(dFloat timestep)
{
	dTrace(("fix this function %s \n", __FUNCTION__));
}

void dAnimationModelManager::PostUpdate(dFloat timestep)
{
	dTrace(("fix this function %s \n", __FUNCTION__));
}
