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



void CustomSkeletonTransformManager::PostUpdate(dFloat timestep)
{
	CustomControllerManager<CustomSkeletonTransformController>::PostUpdate(timestep);
}


void CustomSkeletonTransformController::Init (void* const userData)
{
	m_boneCount = 0;
	m_usertData = userData;
}


void CustomSkeletonTransformController::PostUpdate(dFloat timestep, int threadIndex)
{
	CustomSkeletonTransformManager* const manager = (CustomSkeletonTransformManager*) GetManager();
//	NewtonWorld* const world = manager->GetWorld();

	for (int i = 0; i < m_boneCount; i ++) {
		const dSkeletonBone& bone = m_bones[i];
		dMatrix matrix;
		NewtonBodyGetMatrix(bone.m_body, &matrix[0][0]);
		if (!bone.m_parent) {
			manager->UpdateTransform (&bone, matrix);
		} else {
			dMatrix parentMatrix;
			NewtonBodyGetMatrix(bone.m_parent->m_body, &parentMatrix[0][0]);
			manager->UpdateTransform (&bone, matrix * parentMatrix.Inverse() * bone.m_bindMatrix);
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

void CustomSkeletonTransformController::SetDefaultBitFieldMask ()
{
	for (int i = 0; i < m_boneCount; i ++) {
		dSkeletonBone& bone = m_bones[i];
		bone.m_bitField.ResetBit(i);
		if (bone.m_parent) {
			int parentId = int (bone.m_parent - m_bones);
			bone.m_bitField.ResetBit(parentId);
			bone.m_parent->m_bitField.ResetBit(i);
		}
	}
}

bool CustomSkeletonTransformController::TestCollisionMask (dSkeletonBone* const bone0, dSkeletonBone* const bone1) const
{
	int id1 = int (bone1 - m_bones);
	bool state = bone0->m_bitField.TestMask(id1);
	return state;
}

