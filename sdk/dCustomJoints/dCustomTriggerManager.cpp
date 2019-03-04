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
#include "dCustomJointLibraryStdAfx.h"
#include "dCustomJoint.h"
#include "dCustomTriggerManager.h"

dCustomTriggerManager::dCustomTriggerManager(NewtonWorld* const world)
	:dCustomListener(world, TRIGGER_PLUGIN_NAME)
	,m_triggerList()
	,m_lock(0)
{
}

dCustomTriggerManager::~dCustomTriggerManager()
{
}
	
dCustomTriggerController* dCustomTriggerManager::CreateTrigger (const dMatrix& matrix, NewtonCollision* const convexShape, void* const userData)
{
//	dAssert(0);
//	return NULL;
//	dCustomTriggerController* const trigger = (dCustomTriggerController*) CreateController();
//	trigger->Init (convexShape, matrix, userData);
//	return trigger;

	NewtonWorld* const world = GetWorld();
	dCustomTriggerController& trigger = m_triggerList.Append()->GetInfo();

	trigger.m_userData = userData;

	// create a trigger body and place in the scene
	trigger.m_kinematicBody = NewtonCreateKinematicBody(world, convexShape, &matrix[0][0]);

	// set this shape do not collide with other bodies
	NewtonCollision* const collision = NewtonBodyGetCollision (trigger.m_kinematicBody);
	NewtonCollisionSetMode(collision, 0);

	return &trigger;
}

void dCustomTriggerManager::PreUpdate(dFloat timestep)
{
//	dAssert(0);
	dTrace(("complete the trigger\n"));
/*
	for (dListNode* node = GetFirst(); node; node = node->GetNext()) {
		NewtonDispachThreadJob (m_world, UpdateTrigger, &node->GetInfo(), "UpdateTrigger");
	}
	NewtonSyncThreadJobs(m_world);
*/
}

void dCustomTriggerManager::OnDestroyBody (NewtonBody* const body)
{
	dAssert(0);
/*
	for (dListNode* node = GetFirst(); node; node = node->GetNext()) {
		dCustomTriggerController& controller = node->GetInfo();
		dTree<NewtonBody*,NewtonBody*>& manifest = controller.m_manifest;
		dTree<NewtonBody*,NewtonBody*>::dTreeNode* const passengerNode = manifest.Find (body);
		if (passengerNode) {
			//EventCallback (&controller, m_exitTrigger, body);

			dCustomScopeLock lock (&m_lock);
			manifest.Remove (passengerNode);
		}
	}
*/
}

void dCustomTriggerManager::UpdateTrigger (dCustomTriggerController* const controller)
{
	dAssert(0);
/*
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
				{
					dCustomScopeLock lock(&m_lock);
					manifest.Insert(passangerBody, passangerBody);
				}
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
*/
}

void dCustomTriggerManager::UpdateTrigger (NewtonWorld* const world, void* const context, int threadIndex)
{
	dAssert(0);
/*
	dCustomTriggerController* const controller = (dCustomTriggerController*)context;
	dCustomTriggerManager* const me = (dCustomTriggerManager*)controller->GetManager();
	me->UpdateTrigger(controller);
*/
}


void dCustomTriggerController::Debug(dCustomJoint::dDebugDisplay* const debugContext) const
{
	dAssert(0);
}

void dCustomTriggerController::PostUpdate(dFloat timestep, int threadIndex)
{
}

void dCustomTriggerController::PreUpdate(dFloat timestep, int threadIndex)
{
}
