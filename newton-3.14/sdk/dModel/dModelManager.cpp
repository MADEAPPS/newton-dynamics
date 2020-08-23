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
#include "dModelRootNode.h"

dModelManager::dModelManager(NewtonWorld* const world, const char* const name)
	:dCustomParallelListener(world, name)
{
}

dModelManager::~dModelManager()
{
	while (m_modelList.GetCount()) {
		RemoveAndDeleteRoot(m_modelList.GetFirst()->GetInfo());
	}
}

void dModelManager::AddRoot(dModelRootNode* const root)
{
	dAssert(!root->m_node);
	dAssert(!root->m_manager);
	root->m_node = m_modelList.Append(root);
	root->m_manager = this;
}

void dModelManager::RemoveRoot(dModelRootNode* const root)
{
	if (root->m_node) {
		dList<dModelRootNode*>::dListNode* const node = (dList<dModelRootNode*>::dListNode*) root->m_node;
		root->m_node = NULL;
		root->m_manager = NULL;
		m_modelList.Remove(node);
	}
}

void dModelManager::RemoveAndDeleteRoot(dModelRootNode* const root)
{
	RemoveRoot(root);
	delete root;
}

void dModelManager::OnDebug(dCustomJoint::dDebugDisplay* const debugContext)
{
	for (dList<dModelRootNode*>::dListNode* node = m_modelList.GetFirst(); node; node = node->GetNext()) {
		dModelRootNode* const model = node->GetInfo();
		OnDebug(model, debugContext);
	}
}

void dModelManager::UpdateLocalTranforms(dModelRootNode* const model) const
{
	dMatrix parentMatrixPool[128];
	const dModelNode* stackPool[128];

	int stack = 1;
	stackPool[0] = model;
	parentMatrixPool[0] = dGetIdentityMatrix();

	while (stack) {
		dMatrix matrix;
		stack--;

		dMatrix parentMatrix(parentMatrixPool[stack]);
		const dModelNode* const bone = stackPool[stack];

		NewtonBodyGetMatrix(bone->GetBody(), &matrix[0][0]);
		OnUpdateTransform(bone, matrix * parentMatrix * bone->GetBindMatrix());

		parentMatrix = matrix.Inverse();
		for (dModelChildrenList::dListNode* ptrNode = bone->m_children.GetFirst(); ptrNode; ptrNode = ptrNode->GetNext()) {
			parentMatrixPool[stack] = parentMatrix;
			stackPool[stack] = ptrNode->GetInfo();
			stack++;
		}
	}
}

void dModelManager::PreUpdate(dFloat timestep, int threadID)
{
	D_TRACKTIME();
	NewtonWorld* const world = GetWorld();
	const int threadCount = NewtonGetThreadsCount(world);

	dList<dModelRootNode*>::dListNode* node = m_modelList.GetFirst();
	for (int i = 0; i < threadID; i++) {
		node = node ? node->GetNext() : NULL;
	}

	if (node) {
		do {
			dModelRootNode* const model = node->GetInfo();
			OnPreUpdate(model, timestep, threadID);
//			OnPreUpdate(model, timestep);
			for (int i = 0; i < threadCount; i++) {
				node = node ? node->GetNext() : NULL;
			}
		} while (node);
	}
}

void dModelManager::PostUpdate(dFloat timestep, int threadID)
{
	D_TRACKTIME();
	NewtonWorld* const world = GetWorld();
	const int threadCount = NewtonGetThreadsCount(world);

	dList<dModelRootNode*>::dListNode* node = m_modelList.GetFirst();
	for (int i = 0; i < threadID; i++) {
		node = node ? node->GetNext() : NULL;
	}

	if (node) {
		do {
			dModelRootNode* const model = node->GetInfo();
			OnPostUpdate(model, timestep, threadID);
//			OnPostUpdate(model, timestep);
			for (int i = 0; i < threadCount; i++) {
				node = node ? node->GetNext() : NULL;
			}
		} while (node);
	}
}

void dModelManager::PostStep(dFloat timestep, int threadID)
{
	D_TRACKTIME();
	NewtonWorld* const world = GetWorld();
	const int threadCount = NewtonGetThreadsCount(world);

	dList<dModelRootNode*>::dListNode* node = m_modelList.GetFirst();
	for (int i = 0; i < threadID; i++) {
		node = node ? node->GetNext() : NULL;
	}

	if (node) {
		do {
			dModelRootNode* const model = node->GetInfo();
			if (model->m_localTransformMode) {
				UpdateLocalTranforms(model);
			}
			for (int i = 0; i < threadCount; i++) {
				node = node ? node->GetNext() : NULL;
			}
		} while (node);
	}
}
