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
}

CustomSkeletonTransformController::dSkeletonBone* CustomSkeletonTransformController::AddBone (NewtonBody* const bone, dSkeletonBone* const parentBone)
{
	m_bones[m_boneCount].m_body = bone;
	m_bones[m_boneCount].m_myController = this;
	m_bones[m_boneCount].m_boneIndex = m_boneCount;
	m_bones[m_boneCount].m_parentIndex = parentBone ? parentBone->m_parentIndex : -1;

	m_boneCount ++;
	dAssert (m_boneCount < D_SKELETON_CONTROLLER_MAX_BONES);
	return &m_bones[m_boneCount - 1];
}

void CustomSkeletonTransformController::SetDefaultBitFieldMask ()
{

}

//bool TestCollisionMask (CustomSkeletonTransformController* const controller, int bone0, int bone1) const
//{
//	return false;
//}

