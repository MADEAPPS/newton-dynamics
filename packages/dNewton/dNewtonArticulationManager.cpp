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
#include "dNewtonArticulationManager.h"


dNewtonArticulationManager::dNewtonArticulationManager (dNewton* const world, bool applyLocalTransform)
	:CustomArticulaledTransformManager (world->GetNewton(), applyLocalTransform)
{
}

dNewtonArticulationManager::~dNewtonArticulationManager ()
{
}

dNewtonArticulationManager::dNewtonArticulationController* dNewtonArticulationManager::GetFirstController() const
{
	dAssert (0);
	CustomListNode* const node = GetFirst();
	if (node) {
		return (dNewtonArticulationManager::dNewtonArticulationController*) NewtonBodyGetUserData (node->GetInfo().GetBody());
	}
	return NULL;
}

dNewtonArticulationManager::dNewtonArticulationController* dNewtonArticulationManager::GetNextController(const dNewtonArticulationController* const controller) const
{
	dAssert (0);
	dAssert (controller);
	dAssert (FindNodeFromInfo(*controller->m_controller));
	CustomListNode* const node = GetNodeFromInfo(*controller->m_controller)->GetNext();
	if (node) {
		return (dNewtonArticulationManager::dNewtonArticulationController*) NewtonBodyGetUserData (node->GetInfo().GetBody());
	}
	return NULL;
}

void dNewtonArticulationManager::DestroyController (CustomArticulatedTransformController* const controller)
{
	dNewtonArticulationController* const myController = (dNewtonArticulationController*) controller->GetUserData();
	controller->SetUserData(NULL);
	if (myController) {
		delete myController;
	}
	CustomArticulaledTransformManager::DestroyController (controller);
}

dNewtonArticulationManager::dNewtonArticulationController::dNewtonArticulationController (dNewtonArticulationManager* const manager, bool projectError)
	:dNewtonAlloc()
{
	m_controller = manager->CreateTransformController(this, projectError);
}

dNewtonArticulationManager::dNewtonArticulationController::~dNewtonArticulationController ()
{
	if (m_controller->GetUserData()) {
		m_controller->SetUserData (NULL);
		dNewtonArticulationManager* const manager = (dNewtonArticulationManager*)m_controller->GetManager();
		manager->DestroyController (m_controller);
	}
}


void dNewtonArticulationManager::dNewtonArticulationController::DisableAllSelfCollision ()
{
	m_controller->DisableAllSelfCollision();
}

void dNewtonArticulationManager::dNewtonArticulationController::SetDefaultSelfCollisionMask ()
{
	m_controller->SetDefaultSelfCollisionMask ();
}


int dNewtonArticulationManager::dNewtonArticulationController::GetBoneCount() const
{
	return m_controller->GetBoneCount();
}

void* dNewtonArticulationManager::dNewtonArticulationController::GetBone(int bonexIndex) const
{
	return (void*)(m_controller->GetBone(bonexIndex));
}

dNewtonBody* dNewtonArticulationManager::dNewtonArticulationController::GetBoneBody (int index) const
{
	NewtonBody* const body = m_controller->GetBoneBody(index);
	dAssert(body);
	return (dNewtonBody*)NewtonBodyGetUserData (body);
}

dNewtonBody* dNewtonArticulationManager::dNewtonArticulationController::GetBoneBody (void* const bone) const
{
	NewtonBody* const body = m_controller->GetBoneBody((CustomArticulatedTransformController::dSkeletonBone*)bone);
	dAssert(body);
	return (dNewtonBody*)NewtonBodyGetUserData (body);
}

void* dNewtonArticulationManager::dNewtonArticulationController::GetBoneParent (const void* const bone) const
{
	return (void*)(m_controller->GetParent((CustomArticulatedTransformController::dSkeletonBone*)bone));
}





void dNewtonArticulationManager::dNewtonArticulationController::SetSelfCollisionMask (void* const boneNode0, void* const boneNode1, bool mode)
{
	CustomArticulatedTransformController::dSkeletonBone* const bone0 = (CustomArticulatedTransformController::dSkeletonBone*) boneNode0;
	CustomArticulatedTransformController::dSkeletonBone* const bone1 = (CustomArticulatedTransformController::dSkeletonBone*) boneNode1;
	m_controller->SetSelfCollisionMask (bone0, bone1,  mode);
}

bool dNewtonArticulationManager::dNewtonArticulationController::SelfCollisionTest (const void* const boneNode0, const void* const boneNode1) const
{
	CustomArticulatedTransformController::dSkeletonBone* const bone0 = (CustomArticulatedTransformController::dSkeletonBone*) boneNode0;
	CustomArticulatedTransformController::dSkeletonBone* const bone1 = (CustomArticulatedTransformController::dSkeletonBone*) boneNode1;
	return m_controller->SelfCollisionTest (bone0, bone1);
}


void* dNewtonArticulationManager::dNewtonArticulationController::AddBone (dNewtonBody* const boneBody, const dFloat* const bindMatrix, void* const parentBone)
{
	CustomArticulatedTransformController::dSkeletonBone* const parent = (CustomArticulatedTransformController::dSkeletonBone*) parentBone;
	CustomArticulatedTransformController::dSkeletonBone* const bone = m_controller->AddBone (boneBody->GetNewtonBody(), dMatrix (bindMatrix), parent);
	if (parent) {
		dNewtonBody* const parentBody = (dNewtonBody*) NewtonBodyGetUserData(parent->m_body);
		parentBody->AttachChild (boneBody);
	}

	// save the bone articulation in the body
	boneBody->SetBoneArticulation(bone);

	return bone;
}

void dNewtonArticulationManager::OnPreUpdate (CustomArticulatedTransformController* const controller, dFloat timestep, int threadIndex) const
{
	dNewtonArticulationController* const dcontroller = (dNewtonArticulationController*) controller->GetUserData();
	dcontroller->OnPreUpdate (timestep);
}

void dNewtonArticulationManager::OnUpdateTransform (const CustomArticulatedTransformController::dSkeletonBone* const bone, const dMatrix& localMatrix) const
{
	dNewtonBody* const boneBody = (dNewtonBody*)NewtonBodyGetUserData (bone->m_body);
	dNewtonArticulationController* const controller = (dNewtonArticulationController*)bone->m_myController->GetUserData();
	controller->OnUpdateBoneTransform (boneBody, &localMatrix[0][0]);
}

void dNewtonArticulationManager::DisableAllSelfCollision (CustomArticulatedTransformController* const controller)
{
	dNewtonArticulationController* const myController = (dNewtonArticulationController*) controller->GetUserData();
	dAssert (myController);
	myController->DisableAllSelfCollision();
}

void dNewtonArticulationManager::SetDefaultSelfCollisionMask (CustomArticulatedTransformController* const controller)
{
	dNewtonArticulationController* const myController = (dNewtonArticulationController*) controller->GetUserData();
	dAssert (myController);
	myController->SetDefaultSelfCollisionMask();
}

void dNewtonArticulationManager::SetSelfCollisionMask (void* const boneNode0, void* const boneNode1, bool mode)
{
	CustomArticulatedTransformController::dSkeletonBone* const bone0 = (CustomArticulatedTransformController::dSkeletonBone*) boneNode0;
	CustomArticulatedTransformController::dSkeletonBone* const bone1 = (CustomArticulatedTransformController::dSkeletonBone*) boneNode1;
	dAssert (bone0->m_myController == bone1->m_myController);
	bone0->m_myController->SetSelfCollisionMask (bone0, bone1, mode);
}

bool dNewtonArticulationManager::SelfCollisionTest (const void* const boneNode0, const void* const boneNode1) const
{
	CustomArticulatedTransformController::dSkeletonBone* const bone0 = (CustomArticulatedTransformController::dSkeletonBone*) boneNode0;
	CustomArticulatedTransformController::dSkeletonBone* const bone1 = (CustomArticulatedTransformController::dSkeletonBone*) boneNode1;
	return bone0->m_myController->SelfCollisionTest (bone0, bone1);
}


