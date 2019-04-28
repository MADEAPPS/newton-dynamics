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
	:dCustomParallelListener(world, TRIGGER_PLUGIN_NAME)
	,m_triggerList()
	,m_timestep(0.0f)
	,m_lock(0)
	,m_lru(0)
{
}

dCustomTriggerManager::~dCustomTriggerManager()
{
	dAssert (m_triggerList.GetCount()== 0);
}

void dCustomTriggerManager::OnDestroy()
{
	for (dList<dCustomTriggerController>::dListNode* node = m_triggerList.GetFirst(); node;) {
		dCustomTriggerController& controller = node->GetInfo();
		node = node->GetNext();
		DestroyTrigger(&controller);
	}
}
	
dCustomTriggerController* dCustomTriggerManager::CreateTrigger (const dMatrix& matrix, NewtonCollision* const convexShape, void* const userData)
{
	NewtonWorld* const world = GetWorld();
	dCustomTriggerController& trigger = m_triggerList.Append()->GetInfo();

	// initialize this trigger volume
	trigger.m_manager = this;
	trigger.m_userData = userData;
	trigger.m_kinematicBody = NewtonCreateKinematicBody(world, convexShape, &matrix[0][0]);

	// set this shape do not collide with other bodies
	NewtonCollision* const collision = NewtonBodyGetCollision (trigger.m_kinematicBody);
	NewtonCollisionSetMode(collision, 0);

	return &trigger;
}

void dCustomTriggerManager::DestroyTrigger (dCustomTriggerController* const trigger)
{
	dList<dCustomTriggerController>::dListNode* const node = m_triggerList.GetNodeFromInfo(*trigger);
	m_triggerList.Remove(node);
}


void dCustomTriggerManager::OnDestroyBody (NewtonBody* const body)
{
	for (dList<dCustomTriggerController>::dListNode* node = m_triggerList.GetFirst(); node; node = node->GetNext()) {
		dCustomTriggerController& controller = node->GetInfo();
		dTree<unsigned,NewtonBody*>& manifest = controller.m_manifest;
		dTree<unsigned,NewtonBody*>::dTreeNode* const passengerNode = manifest.Find (body);
		if (passengerNode) {
			EventCallback (&controller, m_exitTrigger, body);

			dCustomScopeLock lock (&m_lock);
			manifest.Remove (passengerNode);
		}
	}
}

void dCustomTriggerManager::UpdateTrigger (dCustomTriggerController* const controller)
{
	NewtonBody* const triggerBody = controller->GetBody();
	dTree<unsigned,NewtonBody*>& manifest = controller->m_manifest;

	m_lru++;
	for (NewtonJoint* joint = NewtonBodyGetFirstContactJoint (triggerBody); joint; joint = NewtonBodyGetNextContactJoint (triggerBody, joint)) {
		dAssert (NewtonJointIsActive (joint));
		//int isActive = NewtonJointIsActive (joint);
		NewtonBody* const body0 = NewtonJointGetBody0(joint);
		NewtonBody* const body1 = NewtonJointGetBody1(joint);
		NewtonBody* const cargoBody = (body0 != triggerBody) ? body0 : body1; 

		dTree<unsigned, NewtonBody*>::dTreeNode* node = manifest.Find(cargoBody); 

		if (!node) {
			dCustomScopeLock lock(&m_lock);
			node = manifest.Insert(m_lru, cargoBody);
			EventCallback (controller, m_enterTrigger, cargoBody);
		} else {
			EventCallback (controller, m_inTrigger, cargoBody);
		}
		node->GetInfo() = m_lru;
	}

	dTree<unsigned, NewtonBody*>::Iterator iter(manifest);
	for (iter.Begin(); iter;) {
		dTree<unsigned, NewtonBody*>::dTreeNode* const node = iter.GetNode();
		iter++;
		if (node->GetInfo() != m_lru) {
			NewtonBody* const cargoBody = node->GetKey();
			EventCallback (controller, m_exitTrigger, cargoBody);

			dCustomScopeLock lock(&m_lock);
			manifest.Remove(cargoBody);
		}
	}
}

void dCustomTriggerManager::PreUpdate(dFloat timestep, int threadID)
{
	D_TRACKTIME();
	NewtonWorld* const world = GetWorld();
	const int threadCount = NewtonGetThreadsCount(world);

	m_timestep = timestep;
	dList<dCustomTriggerController>::dListNode* node = m_triggerList.GetFirst();
	for (int i = 0; i < threadID; i++) {
		node = node ? node->GetNext() : NULL;
	}
	if (node) {
		dCustomTriggerController* const controller = &node->GetInfo();
		UpdateTrigger(controller);
		do {
			for (int i = 0; i < threadCount; i++) {
				node = node ? node->GetNext() : NULL;
			}
		} while (node);
	}
}


