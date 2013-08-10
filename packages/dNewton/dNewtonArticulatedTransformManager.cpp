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
#include "dNewtonArticulatedTransformManager.h"


dNewtonArticulatedTransformManager::dNewtonArticulatedTransformManager (dNewton* const world)
	:CustomArticulaledTransformManager (world->GetNewton())
{
}

dNewtonArticulatedTransformManager::~dNewtonArticulatedTransformManager ()
{
}

dNewtonArticulatedTransformManager::dNewtonArticulatedTransformController* dNewtonArticulatedTransformManager::GetFirstController() const
{
	dAssert (0);
	CustomListNode* const node = GetFirst();
	if (node) {
		return (dNewtonArticulatedTransformManager::dNewtonArticulatedTransformController*) NewtonBodyGetUserData (node->GetInfo().GetBody());
	}
	return NULL;
}

dNewtonArticulatedTransformManager::dNewtonArticulatedTransformController* dNewtonArticulatedTransformManager::GetNextController(const dNewtonArticulatedTransformController* const controller) const
{
	dAssert (0);
	dAssert (controller);
	dAssert (FindNodeFromInfo(*controller->m_controller));
	CustomListNode* const node = GetNodeFromInfo(*controller->m_controller)->GetNext();
	if (node) {
		return (dNewtonArticulatedTransformManager::dNewtonArticulatedTransformController*) NewtonBodyGetUserData (node->GetInfo().GetBody());
	}
	return NULL;
}

NEWTON_API void dNewtonArticulatedTransformManager::DestroyController (CustomArcticulatedTransformController* const controller)
{
	dNewtonArticulatedTransformController* const myController = (dNewtonArticulatedTransformController*) controller->GetUserData();
	controller->SetUserData(NULL);
	if (myController) {
		delete myController;
	}
	CustomArticulaledTransformManager::DestroyController (controller);
}

dNewtonArticulatedTransformManager::dNewtonArticulatedTransformController::dNewtonArticulatedTransformController (dNewtonArticulatedTransformManager* const manager, bool projectError)
	:dNewtonAlloc()
{
	m_controller = manager->CreateTransformController(this, projectError);
}

dNewtonArticulatedTransformManager::dNewtonArticulatedTransformController::~dNewtonArticulatedTransformController ()
{
	if (m_controller->GetUserData()) {
		m_controller->SetUserData (NULL);
		dNewtonArticulatedTransformManager* const manager = (dNewtonArticulatedTransformManager*)m_controller->GetManager();
		manager->DestroyController (m_controller);
	}
}


void dNewtonArticulatedTransformManager::dNewtonArticulatedTransformController::DisableAllSelfCollision ()
{
	m_controller->DisableAllSelfCollision();
}

void dNewtonArticulatedTransformManager::dNewtonArticulatedTransformController::SetDefaultSelfCollisionMask ()
{
	m_controller->SetDefaultSelfCollisionMask ();
}

void dNewtonArticulatedTransformManager::dNewtonArticulatedTransformController::SetSelfCollisionMask (void* const boneNode0, void* const boneNode1, bool mode)
{
	CustomArcticulatedTransformController::dSkeletonBone* const bone0 = (CustomArcticulatedTransformController::dSkeletonBone*) boneNode0;
	CustomArcticulatedTransformController::dSkeletonBone* const bone1 = (CustomArcticulatedTransformController::dSkeletonBone*) boneNode1;
	m_controller->SetSelfCollisionMask (bone0, bone1,  mode);
}

bool dNewtonArticulatedTransformManager::dNewtonArticulatedTransformController::SelfCollisionTest (const void* const boneNode0, const void* const boneNode1) const
{
	CustomArcticulatedTransformController::dSkeletonBone* const bone0 = (CustomArcticulatedTransformController::dSkeletonBone*) boneNode0;
	CustomArcticulatedTransformController::dSkeletonBone* const bone1 = (CustomArcticulatedTransformController::dSkeletonBone*) boneNode1;
	return m_controller->SelfCollisionTest (bone0, bone1);
}


void* dNewtonArticulatedTransformManager::dNewtonArticulatedTransformController::AddBone (dNewtonBody* const bone, const dFloat* const bindMatrix, void* const parentBone)
{
	return m_controller->AddBone (bone->GetNewtonBody(), dMatrix (bindMatrix), (CustomArcticulatedTransformController::dSkeletonBone*) parentBone);
}

void dNewtonArticulatedTransformManager::OnPreUpdate (CustomArcticulatedTransformController* const controller, dFloat timestep, int threadIndex) const
{
	dNewtonArticulatedTransformController* const dcontroller = (dNewtonArticulatedTransformController*) controller->GetUserData();
	dcontroller->OnPreUpdate (timestep);
}

void dNewtonArticulatedTransformManager::OnUpdateTransform (const CustomArcticulatedTransformController::dSkeletonBone* const bone, const dMatrix& localMatrix) const
{
	dNewtonBody* const boneBody = (dNewtonBody*)NewtonBodyGetUserData (bone->m_body);
	dNewtonArticulatedTransformController* const controller = (dNewtonArticulatedTransformController*)bone->m_myController->GetUserData();
	controller->OnUpdateBoneTransform (boneBody, &localMatrix[0][0]);
}

void dNewtonArticulatedTransformManager::DisableAllSelfCollision (CustomArcticulatedTransformController* const controller)
{
	dNewtonArticulatedTransformController* const myController = (dNewtonArticulatedTransformController*) controller->GetUserData();
	dAssert (myController);
	myController->DisableAllSelfCollision();
}

void dNewtonArticulatedTransformManager::SetDefaultSelfCollisionMask (CustomArcticulatedTransformController* const controller)
{
	dNewtonArticulatedTransformController* const myController = (dNewtonArticulatedTransformController*) controller->GetUserData();
	dAssert (myController);
	myController->SetDefaultSelfCollisionMask();
}

void dNewtonArticulatedTransformManager::SetSelfCollisionMask (void* const boneNode0, void* const boneNode1, bool mode)
{
	CustomArcticulatedTransformController::dSkeletonBone* const bone0 = (CustomArcticulatedTransformController::dSkeletonBone*) boneNode0;
	CustomArcticulatedTransformController::dSkeletonBone* const bone1 = (CustomArcticulatedTransformController::dSkeletonBone*) boneNode1;
	dAssert (bone0->m_myController == bone1->m_myController);
	bone0->m_myController->SetSelfCollisionMask (bone0, bone1, mode);
}

bool dNewtonArticulatedTransformManager::SelfCollisionTest (const void* const boneNode0, const void* const boneNode1) const
{
	CustomArcticulatedTransformController::dSkeletonBone* const bone0 = (CustomArcticulatedTransformController::dSkeletonBone*) boneNode0;
	CustomArcticulatedTransformController::dSkeletonBone* const bone1 = (CustomArcticulatedTransformController::dSkeletonBone*) boneNode1;
	return bone0->m_myController->SelfCollisionTest (bone0, bone1);
}

