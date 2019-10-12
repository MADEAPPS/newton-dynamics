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
{
}

dModelManager::~dModelManager()
{
}

void dModelManager::OnDebug(dCustomJoint::dDebugDisplay* const debugContext)
{
	for (dList<dPointer<dModelRootNode>>::dListNode* node = m_controllerList.GetFirst(); node; node = node->GetNext()) {
		dModelRootNode* const model = node->GetInfo().GetData();
		OnDebug(model, debugContext);
	}
}

void dModelManager::AddRoot(dModelRootNode* const root)
{
	dList<dPointer<dModelRootNode>>::dListNode* const node = m_controllerList.Append();
	node->GetInfo().SetData(root);
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
			stackPool[stack] = ptrNode->GetInfo().GetData();
			stack++;
		}
	}
}

void dModelManager::PreUpdate(dFloat timestep, int threadID)
{
	D_TRACKTIME();
	NewtonWorld* const world = GetWorld();
	const int threadCount = NewtonGetThreadsCount(world);

	dList<dPointer<dModelRootNode>>::dListNode* node = m_controllerList.GetFirst();
	for (int i = 0; i < threadID; i++) {
		node = node ? node->GetNext() : NULL;
	}

	if (node) {
		do {
			dModelRootNode* const model = node->GetInfo().GetData();
			OnPreUpdate(model, timestep);
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

	dList<dPointer<dModelRootNode>>::dListNode* node = m_controllerList.GetFirst();
	for (int i = 0; i < threadID; i++) {
		node = node ? node->GetNext() : NULL;
	}

	if (node) {
		do {
			dModelRootNode* const model = node->GetInfo().GetData();
			OnPostUpdate(model, timestep);
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

	dList<dPointer<dModelRootNode>>::dListNode* node = m_controllerList.GetFirst();
	for (int i = 0; i < threadID; i++) {
		node = node ? node->GetNext() : NULL;
	}

	if (node) {
		do {
			dModelRootNode* const model = node->GetInfo().GetData();
			if (model->m_localTransformMode) {
				UpdateLocalTranforms(model);
			}
			for (int i = 0; i < threadCount; i++) {
				node = node ? node->GetNext() : NULL;
			}
		} while (node);
	}
}
