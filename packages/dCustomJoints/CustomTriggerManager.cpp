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
#include "CustomTriggerManager.h"


CustomTriggerManager::CustomTriggerManager(NewtonWorld* const world)
	:CustomControllerManager<CustomTriggerController>(world, TRIGGER_PLUGIN_NAME)
	,m_lru(0)
{
}

CustomTriggerManager::~CustomTriggerManager()
{
}

	
CustomTriggerController* CustomTriggerManager::CreateTrigger (const dMatrix& matrix, NewtonCollision* const convexShape, void* const userData)
{
	CustomTriggerController* const trigger = (CustomTriggerController*) CreateController();
	trigger->Init (convexShape, matrix, userData);
	return trigger;
}

void CustomTriggerManager::PreUpdate(dFloat timestep)
{
	m_lru ++;
	CustomControllerManager<CustomTriggerController>::PreUpdate(timestep);
}


void CustomTriggerController::PostUpdate(dFloat timestep, int threadIndex)
{
}


void CustomTriggerController::Init (NewtonCollision* const convexShape, const dMatrix& matrix, void* const userData)
{
	m_userData = userData;

	NewtonWorld* const world = GetManager()->GetWorld();

	// create a trigger body and place in the scene
	NewtonBody* const body = NewtonCreateKinematicBody(world, convexShape, &matrix[0][0]);
	SetBody (body);
	
	// set this shape do not collide with other bodies
	NewtonCollision* const collision = NewtonBodyGetCollision (body);
	NewtonCollisionSetCollisonMode(collision, 0);
}


void CustomTriggerController::PreUpdate(dFloat timestep, int threadIndex)
{
	CustomTriggerManager* const manager = (CustomTriggerManager*)GetManager();

	unsigned lru = manager->m_lru;

	NewtonBody* const triggerBody = GetBody();
	for (NewtonJoint* joint = NewtonBodyGetFirstContactJoint (triggerBody); joint; joint = NewtonBodyGetNextContactJoint (triggerBody, joint)) {
		NewtonBody* const body0 = NewtonJointGetBody0(joint);
		NewtonBody* const body1 = NewtonJointGetBody1(joint);
		NewtonBody* const passangerBody = (body0 != triggerBody) ? body0 : body1; 
		PassangerManifest::dTreeNode* node = m_manifest.Find (passangerBody);
		if (!node) {
			node = m_manifest.Insert (passangerBody);
			manager->EventCallback (this, CustomTriggerManager::m_enterTrigger, passangerBody);
		} else {
			manager->EventCallback (this, CustomTriggerManager::m_inTrigger, passangerBody);
		}
		node->GetInfo() = lru;
	}
	
	PassangerManifest::Iterator iter (m_manifest);
	for (iter.Begin(); iter; ) {
		PassangerManifest::dTreeNode* const node = iter.GetNode();
		iter ++;
		if (node->GetInfo() != lru) {
			manager->EventCallback (this, CustomTriggerManager::m_exitTrigger, node->GetKey());			
			m_manifest.Remove(node);
		}
	}
}
