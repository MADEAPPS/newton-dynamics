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

//////////////////////////////////////////////////////////////////////
#include "dCustomJointLibraryStdAfx.h"
#include "dCustomJoint.h"
#include "dCustomArcticulatedTransformManager.h"


dCustomArticulaledTransformManager::dCustomArticulaledTransformManager(NewtonWorld* const world, const char* const name)
	:dCustomControllerManager<dCustomArticulatedTransformController>(world, name)
{
}

dCustomArticulaledTransformManager::~dCustomArticulaledTransformManager()
{
}

dCustomArticulatedTransformController* dCustomArticulaledTransformManager::CreateTransformController ()
{
	dCustomArticulatedTransformController* const controller = (dCustomArticulatedTransformController*) CreateController();
	controller->Init ();
	return controller;
}

void dCustomArticulaledTransformManager::SetCollisionMask (dCustomArticulatedTransformController::dSkeletonBone* const bone0, dCustomArticulatedTransformController::dSkeletonBone* const bone1, bool mode)
{
	dAssert (bone0->m_myController);
	dAssert (bone0->m_myController == bone1->m_myController);
	dCustomArticulatedTransformController* const controller = bone0->m_myController; 
	controller->SetSelfCollisionMask (bone0, bone1, mode);
}

void dCustomArticulaledTransformManager::SetDefaultSelfCollisionMask (dCustomArticulatedTransformController* const controller)
{
	controller->SetDefaultSelfCollisionMask();
}

void dCustomArticulaledTransformManager::DisableAllSelfCollision (dCustomArticulatedTransformController* const controller)
{
	controller->DisableAllSelfCollision ();
}

bool dCustomArticulaledTransformManager::SelfCollisionTest (const dCustomArticulatedTransformController::dSkeletonBone* const bone0, const dCustomArticulatedTransformController::dSkeletonBone* const bone1) const
{
	dCustomArticulatedTransformController* const controller0 = bone0->m_myController; 
	dCustomArticulatedTransformController* const controller1 = bone1->m_myController; 
	return (controller0 == controller1) ? controller0->SelfCollisionTest (bone0, bone1) : false;
}

dCustomArticulatedTransformController::dCustomArticulatedTransformController()
	:m_collisionAggregate(NULL)
{
}

dCustomArticulatedTransformController::~dCustomArticulatedTransformController()
{
	if (m_collisionAggregate) {
		NewtonCollisionAggregateDestroy(m_collisionAggregate);
	}
}


void dCustomArticulatedTransformController::Init ()
{
	dCustomArticulaledTransformManager* const manager = (dCustomArticulaledTransformManager*) GetManager();
	NewtonWorld* const world = manager->GetWorld();

	m_calculateLocalTransform = false;
	m_collisionAggregate = NewtonCollisionAggregateCreate(world);
}

void dCustomArticulatedTransformController::PreUpdate(dFloat timestep, int threadIndex)
{
	dCustomArticulaledTransformManager* const manager = (dCustomArticulaledTransformManager*) GetManager();
	manager->OnPreUpdate(this, timestep, threadIndex);
}

void dCustomArticulatedTransformController::PostUpdate(dFloat timestep, int threadIndex)
{
	if (m_calculateLocalTransform) {

		dCustomArticulaledTransformManager* const manager = (dCustomArticulaledTransformManager*) GetManager();

		dMatrix parentMatrixPool[128];
		dList<dSkeletonBone>::dListNode* stackPool[128];

		int stack = 1;
		stackPool[0] = m_bones.GetFirst();
		parentMatrixPool[0] = dGetIdentityMatrix();

		while (stack) {
			dMatrix matrix;
			stack --;

			dMatrix parentMatrix (parentMatrixPool[stack]);
			dList<dSkeletonBone>::dListNode* const node = stackPool[stack];

			const dSkeletonBone& bone = node->GetInfo();
			NewtonBodyGetMatrix(bone.m_body, &matrix[0][0]);
			manager->OnUpdateTransform (&bone, matrix * parentMatrix * bone.m_bindMatrix);

			parentMatrix = matrix.Inverse();
			for (dList<dSkeletonBone>::dListNode* node = bone.GetFirst(); node; node = node->GetNext()) {
				parentMatrixPool[stack] = parentMatrix;
				stackPool[stack] = node;
				stack ++;
			}
		}
	}
}

dCustomArticulatedTransformController::dSkeletonBone* dCustomArticulatedTransformController::AddBone (NewtonBody* const boneBody, const dMatrix& bindMatrix, dSkeletonBone* const parentBone)
{
	dSkeletonBone* const bone = parentBone ? &parentBone->Append()->GetInfo() : &m_bones.Append()->GetInfo();

	bone->m_body = boneBody;
	bone->m_myController = this;
	bone->m_parent = parentBone;
	bone->m_bindMatrix = bindMatrix;

	if (m_collisionAggregate) {
		NewtonCollisionAggregateAddBody (m_collisionAggregate, boneBody);
	}
	return bone;
}

dCustomArticulatedTransformController::dSkeletonBone* dCustomArticulatedTransformController::AddRoot (NewtonBody* const boneBody, const dMatrix& bindMatrix)
{
	return AddBone (boneBody, bindMatrix, NULL);
}

dCustomArticulatedTransformController::dSkeletonBone* dCustomArticulatedTransformController::GetRoot () const
{
	return m_bones.GetCount() ? &m_bones.GetFirst()->GetInfo() : NULL;
}

/*
int dCustomArticulatedTransformController::GetBoneCount() const
{
	return m_boneCount;
}

const dCustomArticulatedTransformController::dSkeletonBone* dCustomArticulatedTransformController::GetBone(int index) const
{
	return &m_bones[index];
}

dCustomArticulatedTransformController::dSkeletonBone* dCustomArticulatedTransformController::GetBone(int index)
{
	return &m_bones[index];
}

const dCustomArticulatedTransformController::dSkeletonBone* dCustomArticulatedTransformController::GetParent(const dSkeletonBone* const bone) const
{
	dAssert (bone->m_myController == this);
	return bone->m_parent;
}


NewtonBody* dCustomArticulatedTransformController::GetBoneBody (const dSkeletonBone* const bone) const
{
	dAssert (bone->m_myController == this);
	return bone->m_body;
}

NewtonBody* dCustomArticulatedTransformController::GetBoneBody (int index) const
{
	return GetBone(index)->m_body;
}
*/

void dCustomArticulatedTransformController::SetDefaultSelfCollisionMask ()
{
//	dAssert (0);
/*
	for (int i = 0; i < m_boneCount; i ++) {
		dSkeletonBone& bone = m_bones[i];
		bone.m_bitField.SetBit (i);
	}

	for (int i = 0; i < m_boneCount; i ++) {
		dSkeletonBone& bone = m_bones[i];
		if (bone.m_parent) {
			SetSelfCollisionMask (&bone, bone.m_parent, false);
		}
	}
*/
}

void dCustomArticulatedTransformController::DisableAllSelfCollision ()
{
//	dAssert (0);
/*
	if (m_collisionAggregate) {
		NewtonCollisionAggregateSetSelfCollision (m_collisionAggregate, 0);
	}
	for (int i = 0; i < m_boneCount; i ++) {
		for (int j = i + 1; j < m_boneCount; j ++) {
			SetSelfCollisionMask (&m_bones[i], &m_bones[j], false);
		}
	}
*/
}

void dCustomArticulatedTransformController::SetSelfCollisionMask (dSkeletonBone* const bone0, dSkeletonBone* const bone1, bool mode)
{
	dAssert (0);
/*
	dAssert (bone0->m_myController);
	dAssert (bone1->m_myController);
	dAssert (bone0->m_myController == this);
	dAssert (bone1->m_myController == this);

	int boneId0 = int (bone0 - m_bones);
	int boneId1 = int (bone1 - m_bones);
	dAssert (boneId0 != boneId1);

	if (mode) {
		bone0->m_bitField.SetBit (boneId1);
		bone1->m_bitField.SetBit (boneId0);
	} else {
		bone0->m_bitField.ResetBit (boneId1);
		bone1->m_bitField.ResetBit (boneId0);
	}
*/
}

bool dCustomArticulatedTransformController::SelfCollisionTest (const dSkeletonBone* const bone0, const dSkeletonBone* const bone1) const
{
//dAssert (0);
return false;
/*
return false;
	bool state = true;
	dAssert (bone0->m_myController);
	dAssert (bone1->m_myController);
	if (bone0->m_myController == bone1->m_myController) {
		int id1 = int (bone1 - m_bones);
		state = bone0->m_bitField.TestMask(id1);
	}
	return state;
*/
}

