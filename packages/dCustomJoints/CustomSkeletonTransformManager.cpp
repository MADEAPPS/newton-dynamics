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


// NewtonPlayerControllerManager.h: interface for the NewtonPlayerControllerManager class.
//
//////////////////////////////////////////////////////////////////////
#include "CustomJointLibraryStdAfx.h"
#include "CustomSkeletonTransformManager.h"


CustomSkeletonTransformManager::CustomSkeletonTransformManager(NewtonWorld* const world)
	:CustomControllerManager<CustomSkeletonTransformController>(world, SKELETON_TRANSFORM_PLUGIN_NAME)
{
}

CustomSkeletonTransformManager::~CustomSkeletonTransformManager()
{
}

CustomSkeletonTransformController* CustomSkeletonTransformManager::CreateTransformController (void* const userData)
{
	CustomSkeletonTransformController* const controller = (CustomSkeletonTransformController*) CreateController();
	controller->Init (userData);
	return controller;
}



//void CustomSkeletonTransformManager::PostUpdate(dFloat timestep)
//{
//	CustomControllerManager<CustomSkeletonTransformController>::PostUpdate(timestep);
//}


void CustomSkeletonTransformManager::SetCollisionMask (CustomSkeletonTransformController::dSkeletonBone* const bone0, CustomSkeletonTransformController::dSkeletonBone* const bone1, bool mode)
{
	dAssert (bone0->m_myController);
	dAssert (bone0->m_myController == bone1->m_myController);
	CustomSkeletonTransformController* const controller = bone0->m_myController; 
	controller->SetSelfCollisionMask (bone0, bone1, mode);
}

void CustomSkeletonTransformManager::SetDefaultSelfCollisionMask (CustomSkeletonTransformController* const controller)
{
	controller->SetDefaultSelfCollisionMask();
}

void CustomSkeletonTransformManager::DisableAllSelfCollision (CustomSkeletonTransformController* const controller)
{
	controller->DisableAllSelfCollision ();
}

bool CustomSkeletonTransformManager::SelfCollisionTest (const CustomSkeletonTransformController::dSkeletonBone* const bone0, const CustomSkeletonTransformController::dSkeletonBone* const bone1) const
{
	CustomSkeletonTransformController* const controller0 = bone0->m_myController; 
	CustomSkeletonTransformController* const controller1 = bone1->m_myController; 
	return (controller0 == controller1) ? controller0->SelfCollisionTest (bone0, bone1) : false;
}


CustomSkeletonTransformController::CustomSkeletonTransformController()
{
}

CustomSkeletonTransformController::~CustomSkeletonTransformController()
{
}


void CustomSkeletonTransformController::Init (void* const userData)
{
	m_boneCount = 0;
	m_userData = userData;
}

void CustomSkeletonTransformController::PreUpdate(dFloat timestep, int threadIndex)
{
	CustomSkeletonTransformManager* const manager = (CustomSkeletonTransformManager*) GetManager();
	manager->OnPreUpdate(this, timestep, threadIndex);
}

void CustomSkeletonTransformController::PostUpdate(dFloat timestep, int threadIndex)
{
	CustomSkeletonTransformManager* const manager = (CustomSkeletonTransformManager*) GetManager();
	for (int i = 0; i < m_boneCount; i ++) {
		const dSkeletonBone& bone = m_bones[i];
		dMatrix matrix;
		NewtonBodyGetMatrix(bone.m_body, &matrix[0][0]);
		if (!bone.m_parent) {
			manager->OnUpdateTransform (&bone, matrix);
		} else {
			dMatrix parentMatrix;
			NewtonBodyGetMatrix(bone.m_parent->m_body, &parentMatrix[0][0]);
			manager->OnUpdateTransform (&bone, matrix * parentMatrix.Inverse() * bone.m_bindMatrix);
		}
	}
}

CustomSkeletonTransformController::dSkeletonBone* CustomSkeletonTransformController::AddBone (NewtonBody* const bone, const dMatrix& bindMatrix, dSkeletonBone* const parentBone)
{
	m_bones[m_boneCount].m_body = bone;
	m_bones[m_boneCount].m_myController = this;
	m_bones[m_boneCount].m_parent = parentBone;
	m_bones[m_boneCount].m_bindMatrix = bindMatrix;

	m_boneCount ++;
	dAssert (m_boneCount < D_SKELETON_CONTROLLER_MAX_BONES);
	return &m_bones[m_boneCount - 1];
}

void CustomSkeletonTransformController::SetDefaultSelfCollisionMask ()
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

void CustomSkeletonTransformController::DisableAllSelfCollision ()
{
	for (int i = 0; i < m_boneCount; i ++) {
		for (int j = i + 1; j < m_boneCount; j ++) {
			SetSelfCollisionMask (&m_bones[i], &m_bones[j], false);
		}
	}
}

void CustomSkeletonTransformController::SetSelfCollisionMask (dSkeletonBone* const bone0, dSkeletonBone* const bone1, bool mode)
{
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

bool CustomSkeletonTransformController::SelfCollisionTest (const dSkeletonBone* const bone0, const dSkeletonBone* const bone1) const
{
	int id1 = int (bone1 - m_bones);
	bool state = bone0->m_bitField.TestMask(id1);
	return state;
}

