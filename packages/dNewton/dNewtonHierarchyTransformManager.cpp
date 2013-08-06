/* Copyright (c) <2003-2011> <Julio Jerez, Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 
* 3. This notice may not be removed or altered from any source distribution.
*/

#include "dStdAfxNewton.h"
#include "dNewton.h"
#include "dNewtonBody.h"
#include "dNewtonCollision.h"
#include "dNewtonHierarchyTransformManager.h"


dNewtonHierarchyTransformManager::dNewtonHierarchyTransformManager (dNewton* const world)
	:CustomHierarchicalTransformManager (world->GetNewton())
{
}

dNewtonHierarchyTransformManager::~dNewtonHierarchyTransformManager ()
{
}

dNewtonHierarchyTransformManager::dNewtonHierarchyTransformController* dNewtonHierarchyTransformManager::GetFirstController() const
{
	dAssert (0);
	CustomListNode* const node = GetFirst();
	if (node) {
		return (dNewtonHierarchyTransformManager::dNewtonHierarchyTransformController*) NewtonBodyGetUserData (node->GetInfo().GetBody());
	}
	return NULL;
}

dNewtonHierarchyTransformManager::dNewtonHierarchyTransformController* dNewtonHierarchyTransformManager::GetNextController(const dNewtonHierarchyTransformController* const controller) const
{
	dAssert (0);
	dAssert (controller);
	dAssert (FindNodeFromInfo(*controller->m_controller));
	CustomListNode* const node = GetNodeFromInfo(*controller->m_controller)->GetNext();
	if (node) {
		return (dNewtonHierarchyTransformManager::dNewtonHierarchyTransformController*) NewtonBodyGetUserData (node->GetInfo().GetBody());
	}
	return NULL;
}

NEWTON_API void dNewtonHierarchyTransformManager::DestroyController (CustomHierarchicalTransformController* const controller)
{
	dNewtonHierarchyTransformController* const myController = (dNewtonHierarchyTransformController*) controller->GetUserData();
	controller->SetUserData(NULL);
	if (myController) {
		delete myController;
	}
	CustomHierarchicalTransformManager::DestroyController (controller);
}

dNewtonHierarchyTransformManager::dNewtonHierarchyTransformController::dNewtonHierarchyTransformController (dNewtonHierarchyTransformManager* const manager, bool projectError)
	:dNewtonAlloc()
{
	m_controller = manager->CreateTransformController(this, projectError);
}

dNewtonHierarchyTransformManager::dNewtonHierarchyTransformController::~dNewtonHierarchyTransformController ()
{
	if (m_controller->GetUserData()) {
		m_controller->SetUserData (NULL);
		dNewtonHierarchyTransformManager* const manager = (dNewtonHierarchyTransformManager*)m_controller->GetManager();
		manager->DestroyController (m_controller);
	}
}


void dNewtonHierarchyTransformManager::dNewtonHierarchyTransformController::DisableAllSelfCollision ()
{
	m_controller->DisableAllSelfCollision();
}

void dNewtonHierarchyTransformManager::dNewtonHierarchyTransformController::SetDefaultSelfCollisionMask ()
{
	m_controller->SetDefaultSelfCollisionMask ();
}

void dNewtonHierarchyTransformManager::dNewtonHierarchyTransformController::SetSelfCollisionMask (void* const boneNode0, void* const boneNode1, bool mode)
{
	CustomHierarchicalTransformController::dSkeletonBone* const bone0 = (CustomHierarchicalTransformController::dSkeletonBone*) boneNode0;
	CustomHierarchicalTransformController::dSkeletonBone* const bone1 = (CustomHierarchicalTransformController::dSkeletonBone*) boneNode1;
	m_controller->SetSelfCollisionMask (bone0, bone1,  mode);
}

bool dNewtonHierarchyTransformManager::dNewtonHierarchyTransformController::SelfCollisionTest (const void* const boneNode0, const void* const boneNode1) const
{
	CustomHierarchicalTransformController::dSkeletonBone* const bone0 = (CustomHierarchicalTransformController::dSkeletonBone*) boneNode0;
	CustomHierarchicalTransformController::dSkeletonBone* const bone1 = (CustomHierarchicalTransformController::dSkeletonBone*) boneNode1;
	return m_controller->SelfCollisionTest (bone0, bone1);
}


void* dNewtonHierarchyTransformManager::dNewtonHierarchyTransformController::AddBone (dNewtonBody* const bone, const dFloat* const bindMatrix, void* const parentBone)
{
	return m_controller->AddBone (bone->GetNewtonBody(), dMatrix (bindMatrix), (CustomHierarchicalTransformController::dSkeletonBone*) parentBone);
}

void dNewtonHierarchyTransformManager::OnPreUpdate (CustomHierarchicalTransformController* const controller, dFloat timestep, int threadIndex) const
{
	dNewtonHierarchyTransformController* const dcontroller = (dNewtonHierarchyTransformController*) controller->GetUserData();
	dcontroller->OnPreUpdate (timestep);
}

void dNewtonHierarchyTransformManager::OnUpdateTransform (const CustomHierarchicalTransformController::dSkeletonBone* const bone, const dMatrix& localMatrix) const
{
	dNewtonBody* const boneBody = (dNewtonBody*)NewtonBodyGetUserData (bone->m_body);
	dNewtonHierarchyTransformController* const controller = (dNewtonHierarchyTransformController*)bone->m_myController->GetUserData();
	controller->OnUpdateBoneTransform (boneBody, &localMatrix[0][0]);
}

void dNewtonHierarchyTransformManager::DisableAllSelfCollision (CustomHierarchicalTransformController* const controller)
{
	dNewtonHierarchyTransformController* const myController = (dNewtonHierarchyTransformController*) controller->GetUserData();
	dAssert (myController);
	myController->DisableAllSelfCollision();
}

void dNewtonHierarchyTransformManager::SetDefaultSelfCollisionMask (CustomHierarchicalTransformController* const controller)
{
	dNewtonHierarchyTransformController* const myController = (dNewtonHierarchyTransformController*) controller->GetUserData();
	dAssert (myController);
	myController->SetDefaultSelfCollisionMask();
}

void dNewtonHierarchyTransformManager::SetSelfCollisionMask (void* const boneNode0, void* const boneNode1, bool mode)
{
	CustomHierarchicalTransformController::dSkeletonBone* const bone0 = (CustomHierarchicalTransformController::dSkeletonBone*) boneNode0;
	CustomHierarchicalTransformController::dSkeletonBone* const bone1 = (CustomHierarchicalTransformController::dSkeletonBone*) boneNode1;
	dAssert (bone0->m_myController == bone1->m_myController);
	bone0->m_myController->SetSelfCollisionMask (bone0, bone1, mode);
}

bool dNewtonHierarchyTransformManager::SelfCollisionTest (const void* const boneNode0, const void* const boneNode1) const
{
	CustomHierarchicalTransformController::dSkeletonBone* const bone0 = (CustomHierarchicalTransformController::dSkeletonBone*) boneNode0;
	CustomHierarchicalTransformController::dSkeletonBone* const bone1 = (CustomHierarchicalTransformController::dSkeletonBone*) boneNode1;
	return bone0->m_myController->SelfCollisionTest (bone0, bone1);
}

