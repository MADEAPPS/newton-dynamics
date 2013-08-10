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

//////////////////////////////////////////////////////////////////////
#include "CustomJointLibraryStdAfx.h"
#include "CustomArcticulatedTransformManager.h"


CustomArticulaledTransformManager::CustomArticulaledTransformManager(NewtonWorld* const world)
	:CustomControllerManager<CustomArcticulatedTransformController>(world, HIERACHICAL_ARTICULATED_PLUGIN_NAME)
{
}

CustomArticulaledTransformManager::~CustomArticulaledTransformManager()
{
}

CustomArcticulatedTransformController* CustomArticulaledTransformManager::CreateTransformController (void* const userData, bool errorCorrectionMode)
{
	CustomArcticulatedTransformController* const controller = (CustomArcticulatedTransformController*) CreateController();
	controller->Init (userData, errorCorrectionMode);
	return controller;
}


void CustomArticulaledTransformManager::SetCollisionMask (CustomArcticulatedTransformController::dSkeletonBone* const bone0, CustomArcticulatedTransformController::dSkeletonBone* const bone1, bool mode)
{
	dAssert (bone0->m_myController);
	dAssert (bone0->m_myController == bone1->m_myController);
	CustomArcticulatedTransformController* const controller = bone0->m_myController; 
	controller->SetSelfCollisionMask (bone0, bone1, mode);
}

void CustomArticulaledTransformManager::SetDefaultSelfCollisionMask (CustomArcticulatedTransformController* const controller)
{
	controller->SetDefaultSelfCollisionMask();
}

void CustomArticulaledTransformManager::DisableAllSelfCollision (CustomArcticulatedTransformController* const controller)
{
	controller->DisableAllSelfCollision ();
}

bool CustomArticulaledTransformManager::SelfCollisionTest (const CustomArcticulatedTransformController::dSkeletonBone* const bone0, const CustomArcticulatedTransformController::dSkeletonBone* const bone1) const
{
	CustomArcticulatedTransformController* const controller0 = bone0->m_myController; 
	CustomArcticulatedTransformController* const controller1 = bone1->m_myController; 
	return (controller0 == controller1) ? controller0->SelfCollisionTest (bone0, bone1) : false;
}


CustomArcticulatedTransformController::CustomArcticulatedTransformController()
{
}

CustomArcticulatedTransformController::~CustomArcticulatedTransformController()
{
}


void CustomArcticulatedTransformController::Init (void* const userData, bool errorCorrection)
{
	m_boneCount = 0;
	m_userData = userData;
	SetErrorProjectionMode (errorCorrection);
}

void CustomArcticulatedTransformController::SetErrorProjectionMode (bool mode)
{
	m_errorProjectionMode = mode;
}

bool CustomArcticulatedTransformController::GetErrorProjectionMode () const
{
	return m_errorProjectionMode;
}


void CustomArcticulatedTransformController::PreUpdate(dFloat timestep, int threadIndex)
{
	CustomArticulaledTransformManager* const manager = (CustomArticulaledTransformManager*) GetManager();
	manager->OnPreUpdate(this, timestep, threadIndex);
}


void CustomArcticulatedTransformController::PostUpdate(dFloat timestep, int threadIndex)
{
	if (m_errorProjectionMode && m_boneCount && (NewtonBodyGetSleepState(m_bones[0].m_body) == 0)) {
		for (int i = 1; i < m_boneCount; i ++) {
			const dSkeletonBone* const bone = &m_bones[i];
			dAssert (bone->m_parent);
			const NewtonBody* const child = bone->m_body;
			const NewtonBody* const parent = bone->m_parent->m_body;
			for (NewtonJoint* joint = NewtonBodyGetFirstJoint(child); joint; joint = NewtonBodyGetNextJoint(child, joint)) {
				if ((NewtonJointGetBody0(joint) == parent) || (NewtonJointGetBody1(joint) == parent)) {
					CustomJoint* const cJoint = (CustomJoint*) NewtonJointGetUserData(joint);
					cJoint->ProjectError ();
					break;
				}
			}
		}
	}

	CustomArticulaledTransformManager* const manager = (CustomArticulaledTransformManager*) GetManager();
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

CustomArcticulatedTransformController::dSkeletonBone* CustomArcticulatedTransformController::AddBone (NewtonBody* const bone, const dMatrix& bindMatrix, dSkeletonBone* const parentBone)
{
	m_bones[m_boneCount].m_body = bone;
	m_bones[m_boneCount].m_myController = this;
	m_bones[m_boneCount].m_parent = parentBone;
	m_bones[m_boneCount].m_bindMatrix = bindMatrix;

	m_boneCount ++;
	dAssert (m_boneCount < D_HIERACHICAL_CONTROLLER_MAX_BONES);
	return &m_bones[m_boneCount - 1];
}

void CustomArcticulatedTransformController::SetDefaultSelfCollisionMask ()
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

void CustomArcticulatedTransformController::DisableAllSelfCollision ()
{
	for (int i = 0; i < m_boneCount; i ++) {
		for (int j = i + 1; j < m_boneCount; j ++) {
			SetSelfCollisionMask (&m_bones[i], &m_bones[j], false);
		}
	}
}

void CustomArcticulatedTransformController::SetSelfCollisionMask (dSkeletonBone* const bone0, dSkeletonBone* const bone1, bool mode)
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

bool CustomArcticulatedTransformController::SelfCollisionTest (const dSkeletonBone* const bone0, const dSkeletonBone* const bone1) const
{
	int id1 = int (bone1 - m_bones);
	bool state = bone0->m_bitField.TestMask(id1);
	return state;
}

