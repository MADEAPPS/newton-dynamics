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
#include <dCustomJointLibraryStdAfx.h>
#include <dCustomJoint.h>
#include <CustomArcticulatedTransformManager.h>


CustomArticulaledTransformManager::CustomArticulaledTransformManager(NewtonWorld* const world, const char* const name)
	:CustomControllerManager<CustomArticulatedTransformController>(world, name)
{
}

CustomArticulaledTransformManager::~CustomArticulaledTransformManager()
{
}

CustomArticulatedTransformController* CustomArticulaledTransformManager::CreateTransformController (void* const userData)
{
	CustomArticulatedTransformController* const controller = (CustomArticulatedTransformController*) CreateController();
	controller->Init (userData);
	return controller;
}


void CustomArticulaledTransformManager::SetCollisionMask (CustomArticulatedTransformController::dSkeletonBone* const bone0, CustomArticulatedTransformController::dSkeletonBone* const bone1, bool mode)
{
	dAssert (bone0->m_myController);
	dAssert (bone0->m_myController == bone1->m_myController);
	CustomArticulatedTransformController* const controller = bone0->m_myController; 
	controller->SetSelfCollisionMask (bone0, bone1, mode);
}

void CustomArticulaledTransformManager::SetDefaultSelfCollisionMask (CustomArticulatedTransformController* const controller)
{
	controller->SetDefaultSelfCollisionMask();
}

void CustomArticulaledTransformManager::DisableAllSelfCollision (CustomArticulatedTransformController* const controller)
{
	controller->DisableAllSelfCollision ();
}

bool CustomArticulaledTransformManager::SelfCollisionTest (const CustomArticulatedTransformController::dSkeletonBone* const bone0, const CustomArticulatedTransformController::dSkeletonBone* const bone1) const
{
	CustomArticulatedTransformController* const controller0 = bone0->m_myController; 
	CustomArticulatedTransformController* const controller1 = bone1->m_myController; 
	return (controller0 == controller1) ? controller0->SelfCollisionTest (bone0, bone1) : false;
}

CustomArticulatedTransformController::CustomArticulatedTransformController()
	:m_collisionAggregate(NULL)
	,m_boneCount(0)
{
}

CustomArticulatedTransformController::~CustomArticulatedTransformController()
{
	if (m_collisionAggregate) {
		NewtonCollisionAggregateDestroy(m_collisionAggregate);
	}
}


void CustomArticulatedTransformController::Init (void* const userData)
{
	CustomArticulaledTransformManager* const manager = (CustomArticulaledTransformManager*) GetManager();
	NewtonWorld* const world = manager->GetWorld();

	m_boneCount = 0;
	m_userData = userData;
	m_calculateLocalTransform = false;
	m_collisionAggregate = NewtonCollisionAggregateCreate(world);
}

void CustomArticulatedTransformController::PreUpdate(dFloat timestep, int threadIndex)
{
	CustomArticulaledTransformManager* const manager = (CustomArticulaledTransformManager*) GetManager();
	manager->OnPreUpdate(this, timestep, threadIndex);
}

void CustomArticulatedTransformController::PostUpdate(dFloat timestep, int threadIndex)
{
	if (m_calculateLocalTransform) {
		CustomArticulaledTransformManager* const manager = (CustomArticulaledTransformManager*) GetManager();
		for (int i = 0; i < m_boneCount; i ++) {
			const dSkeletonBone& bone = m_bones[i];
			dMatrix matrix;
			NewtonBodyGetMatrix(bone.m_body, &matrix[0][0]);
			if (bone.m_parent) {
				dMatrix parentMatrix;
				NewtonBodyGetMatrix(bone.m_parent->m_body, &parentMatrix[0][0]);
				matrix = matrix * parentMatrix.Inverse() * bone.m_bindMatrix;
			}
			manager->OnUpdateTransform (&bone, matrix);
		}
	}
}

CustomArticulatedTransformController::dSkeletonBone* CustomArticulatedTransformController::AddBone (NewtonBody* const bone, const dMatrix& bindMatrix, dSkeletonBone* const parentBone)
{
	m_bones[m_boneCount].m_body = bone;
	m_bones[m_boneCount].m_myController = this;
	m_bones[m_boneCount].m_parent = parentBone;
	m_bones[m_boneCount].m_bindMatrix = bindMatrix;

	if (m_collisionAggregate) {
		NewtonCollisionAggregateAddBody (m_collisionAggregate, bone);
	}

	m_boneCount ++;
	dAssert (m_boneCount < D_HIERACHICAL_CONTROLLER_MAX_BONES);
	return &m_bones[m_boneCount - 1];
}

int CustomArticulatedTransformController::GetBoneCount() const
{
	return m_boneCount;
}

const CustomArticulatedTransformController::dSkeletonBone* CustomArticulatedTransformController::GetBone(int index) const
{
	return &m_bones[index];
}

CustomArticulatedTransformController::dSkeletonBone* CustomArticulatedTransformController::GetBone(int index)
{
	return &m_bones[index];
}

const CustomArticulatedTransformController::dSkeletonBone* CustomArticulatedTransformController::GetParent(const dSkeletonBone* const bone) const
{
	dAssert (bone->m_myController == this);
	return bone->m_parent;
}

NewtonBody* CustomArticulatedTransformController::GetBoneBody (const dSkeletonBone* const bone) const
{
	dAssert (bone->m_myController == this);
	return bone->m_body;
}

NewtonBody* CustomArticulatedTransformController::GetBoneBody (int index) const
{
	return GetBone(index)->m_body;
}


void CustomArticulatedTransformController::SetDefaultSelfCollisionMask ()
{
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
}

void CustomArticulatedTransformController::DisableAllSelfCollision ()
{
	if (m_collisionAggregate) {
		NewtonCollisionAggregateSetSelfCollision (m_collisionAggregate, 0);
	}
	for (int i = 0; i < m_boneCount; i ++) {
		for (int j = i + 1; j < m_boneCount; j ++) {
			SetSelfCollisionMask (&m_bones[i], &m_bones[j], false);
		}
	}
}

void CustomArticulatedTransformController::SetSelfCollisionMask (dSkeletonBone* const bone0, dSkeletonBone* const bone1, bool mode)
{
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
}

bool CustomArticulatedTransformController::SelfCollisionTest (const dSkeletonBone* const bone0, const dSkeletonBone* const bone1) const
{
	bool state = true;
	dAssert (bone0->m_myController);
	dAssert (bone1->m_myController);
	if (bone0->m_myController == bone1->m_myController) {
		int id1 = int (bone1 - m_bones);
		state = bone0->m_bitField.TestMask(id1);
	}
	return state;
}

