/* Copyright (c) <2003-2019> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/

#include "dModelStdAfx.h"
#include "dModelManager.h"

dModelManager::dModelManager(NewtonWorld* const world, const char* const name)
	:dCustomParallelListener(world, name)
	//,m_controllerList()
	//,m_timestep(0.0f)
{
}

dModelManager::~dModelManager()
{
}

/*
void dModelManager::AddModel(dAnimationJointRoot* const model)
{
	dAssert(!model->m_managerNode);
	model->m_manager = this;
	model->m_managerNode = m_controllerList.Append(model);
}

void dModelManager::RemoveModel(dAnimationJointRoot* const model)
{
	dAssert(model->m_managerNode);
	dAssert(model->m_manager == this);
	dAssert(model->m_managerNode->GetInfo() == model);
	m_controllerList.Remove(model->m_managerNode);
	model->m_manager = NULL;
	model->m_managerNode = NULL;
}

dAnimationJoint* dModelManager::GetFirstJoint(const dAnimationJointRoot* const model) const
{
	return GetFirstJoint((dAnimationJoint*)model);
}

dAnimationJoint* dModelManager::GetFirstJoint(const dAnimationJoint* const root) const
{
	const dAnimationJoint* joint = root;
	while (joint->GetChildren().GetCount()) {
		joint = joint->GetChildren().GetFirst()->GetInfo();
	}
	return (dAnimationJoint*)joint;
}

dAnimationJoint* dModelManager::GetNextJoint(const dAnimationJoint* const joint) const
{
	dAnimationJoint* const parentJoint = joint->GetParent();
	if (!parentJoint) {
		return NULL;
	}

	dAssert(joint->GetNode());
	const dAnimationJointChildren::dListNode* const siblingNode = joint->GetNode()->GetNext();
	if (siblingNode) {
		return GetFirstJoint(siblingNode->GetInfo());
	}
	return parentJoint;
}


void dModelManager::OnDestroy()
{
	while (m_controllerList.GetFirst()) {
		dAnimationJointRoot* const model = m_controllerList.GetFirst()->GetInfo();
		dAssert(model->m_managerNode == m_controllerList.GetFirst());
		delete model;
	}
}


void dModelManager::PreUpdate(dFloat timestep, int threadID)
{
	D_TRACKTIME();
	NewtonWorld* const world = GetWorld();
	const int threadCount = NewtonGetThreadsCount(world);

	m_timestep = timestep;
	dList<dAnimationJointRoot*>::dListNode* node = m_controllerList.GetFirst();
	for (int i = 0; i < threadID; i++) {
		node = node ? node->GetNext() : NULL;
	}
	if (node) {
		dAnimationJointRoot* const model = node->GetInfo();
		//model->PreUpdate(timestep);
		OnPreUpdate(model, timestep);
		do {
			for (int i = 0; i < threadCount; i++) {
				node = node ? node->GetNext() : NULL;
			}
		} while (node);
	}
}

*/

/*
void dModelManager::OnDebug(dCustomJoint::dDebugDisplay* const debugContext)
{
	dAssert (0);
}

void dModelManager::OnPreUpdate(dCustomTransformController* const controller, dFloat timestep, int threadIndex) const
{
	dAssert (0);
}

void dModelManager::OnUpdateTransform(const dCustomTransformController::dSkeletonBone* const bone, const dMatrix& localMatrix) const
{
	dAssert (0);
}
*/

void dModelManager::AddRoot(dModelRootNode* const root)
{
	dList<dPointer<dModelRootNode>>::dListNode* const node = m_controllerList.Append();
	node->GetInfo().SetData(root);
}


void dModelManager::PostUpdate(dFloat timestep, int threadID)
{
	D_TRACKTIME();
	NewtonWorld* const world = GetWorld();
	const int threadCount = NewtonGetThreadsCount(world);

	dList<dPointer<dModelRootNode>>::dListNode* node = m_controllerList.GetFirst();
	for (int i = 0; i < threadID; i++) {
		node = node ? node->GetNext() : NULL;
	}

	if (node) {
		dModelRootNode* const model = node->GetInfo().GetData();
		model->PostUpdate(this, timestep);
		//OnPostUpdate(model, timestep);
		//model->UpdateTransforms(timestep);
		do {
			for (int i = 0; i < threadCount; i++) {
				node = node ? node->GetNext() : NULL;
			}
		} while (node);
	}
}
