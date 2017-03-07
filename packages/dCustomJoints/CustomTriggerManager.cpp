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


// NewtonPlayerControllerManager.h: interface for the NewtonPlayerControllerManager class.
//
//////////////////////////////////////////////////////////////////////
#include <dCustomJointLibraryStdAfx.h>
#include <dCustomJoint.h>
#include <CustomTriggerManager.h>

CustomTriggerManager::CustomTriggerManager(NewtonWorld* const world)
	:CustomControllerManager<CustomTriggerController>(world, TRIGGER_PLUGIN_NAME)
	,m_lock(0)
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
	for (dListNode* node = GetFirst(); node; node = node->GetNext()) {
		NewtonDispachThreadJob (m_world, UpdateTrigger, &node->GetInfo());
	}
	NewtonSyncThreadJobs(m_world);

	// bypass the entire Post Update call by not calling the base class
	//CustomControllerManager<CustomTriggerController>::PreUpdate(timestep);
}


void CustomTriggerManager::OnDestroyBody (NewtonBody* const body)
{
	for (dListNode* node = GetFirst(); node; node = node->GetNext()) {
		CustomTriggerController& controller = node->GetInfo();
		dTree<NewtonBody*,NewtonBody*>& manifest = controller.m_manifest;
		dTree<NewtonBody*,NewtonBody*>::dTreeNode* const passengerNode = manifest.Find (body);
		if (passengerNode) {
			//EventCallback (&controller, m_exitTrigger, body);

			dCustomScopeLock lock (&m_lock);
			manifest.Remove (passengerNode);
		}
	}
}

void CustomTriggerManager::UpdateTrigger (CustomTriggerController* const controller)
{
	NewtonBody* const triggerBody = controller->GetBody();
	dTree<NewtonBody*,NewtonBody*>& manifest = controller->m_manifest;

	for (NewtonJoint* joint = NewtonBodyGetFirstContactJoint (triggerBody); joint; joint = NewtonBodyGetNextContactJoint (triggerBody, joint)) {

		int isActive = NewtonJointIsActive (joint);
		NewtonBody* const body0 = NewtonJointGetBody0(joint);
		NewtonBody* const body1 = NewtonJointGetBody1(joint);
		NewtonBody* const passangerBody = (body0 != triggerBody) ? body0 : body1; 
		
		if (isActive) {
			dTree<NewtonBody*,NewtonBody*>::dTreeNode* const passengerNode = manifest.Find (passangerBody);
			if (passengerNode) {
				EventCallback (controller, m_inTrigger, passangerBody);

			} else {
				dCustomScopeLock lock (&m_lock);
				manifest.Insert (passangerBody, passangerBody);
				EventCallback (controller, m_enterTrigger, passangerBody);
			} 
		} else {
			dTree<NewtonBody*,NewtonBody*>::dTreeNode* const passengerNode = manifest.Find (passangerBody);

			if (passengerNode) {
				EventCallback (controller, m_exitTrigger, passangerBody);

				dCustomScopeLock lock (&m_lock);
				manifest.Remove (passengerNode);
			}
		}
	}
}


void CustomTriggerManager::UpdateTrigger (NewtonWorld* const world, void* const context, int threadIndex)
{
	CustomTriggerController* const controller = (CustomTriggerController*)context;
	CustomTriggerManager* const me = (CustomTriggerManager*)controller->GetManager();
	me->UpdateTrigger(controller);
}



CustomTriggerController::CustomTriggerController()
	:CustomControllerBase()
	,m_manifest()
{
}

CustomTriggerController::~CustomTriggerController()
{
	NewtonDestroyBody(m_body);
}


void CustomTriggerController::Init (NewtonCollision* const convexShape, const dMatrix& matrix, void* const userData)
{
	m_userData = userData;

	NewtonWorld* const world = ((CustomTriggerManager*)GetManager())->GetWorld();

	// create a trigger body and place in the scene
	m_body = NewtonCreateKinematicBody(world, convexShape, &matrix[0][0]);
	
	// set this shape do not collide with other bodies
	NewtonCollision* const collision = NewtonBodyGetCollision (m_body);
	NewtonCollisionSetMode(collision, 0);
}


void CustomTriggerController::PostUpdate(dFloat timestep, int threadIndex)
{
}

void CustomTriggerController::PreUpdate(dFloat timestep, int threadIndex)
{
}
