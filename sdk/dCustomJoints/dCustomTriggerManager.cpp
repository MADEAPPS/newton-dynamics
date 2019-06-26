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
	,m_pairCache ()
	,m_timestep(0.0f)
	,m_cacheCount(0)
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
		dCustomTriggerController::dTriggerManifest::dTreeNode* const passengerNode = controller.m_manifest.Find (body);
		if (passengerNode) {
			OnExit (&controller, body);

			dCustomScopeLock lock (&m_lock);
			controller.m_manifest.Remove (passengerNode);
		}
	}
}

void dCustomTriggerManager::OnDebug(dCustomJoint::dDebugDisplay* const debugContext)
{
	for (dList<dCustomTriggerController>::dListNode* node = GetControllersList().GetFirst(); node; node = node->GetNext()) {
		const dCustomTriggerController& controller = node->GetInfo();

		dCustomTriggerController::dTriggerManifest::Iterator iter (controller.m_manifest);
		for (iter.Begin(); iter; iter++) {
			const NewtonBody* const body = iter.GetKey();
			OnDebug(debugContext, &controller, body);
		}
	}
}


static int xxxxx;
void dCustomTriggerManager::PreUpdate(dFloat timestep, int threadID)
{
	D_TRACKTIME();

	NewtonWorld* const world = GetWorld();
	const int threadCount = NewtonGetThreadsCount(world);

	for (int i = threadID; i < m_cacheCount; i += threadCount) {
		dTriggerGuestPair& cacheEntry = m_pairCache[i];
		if (cacheEntry.m_bodyNode->GetInfo() != m_lru) {
			cacheEntry.m_bodyNode->GetInfo() = m_lru;
			dTrace(("in trigger body:%d lru:%d frame:%d\n", NewtonBodyGetID(cacheEntry.m_bodyNode->GetKey()), cacheEntry.m_bodyNode->GetInfo(), xxxxx));
			WhileIn (cacheEntry.m_trigger, cacheEntry.m_bodyNode->GetKey());
		}
	}
}

void dCustomTriggerManager::PreUpdate(dFloat timestep)
{
	m_lru++;
	m_cacheCount = 0;
	m_timestep = timestep;

xxxxx++;
	for (dList<dCustomTriggerController>::dListNode* triggerNode = GetControllersList().GetFirst(); triggerNode; triggerNode = triggerNode->GetNext()) {
		dCustomTriggerController& controller = triggerNode->GetInfo();

		NewtonBody* const triggerBody = controller.GetBody();
		dCustomTriggerController::dTriggerManifest& manifest = controller.m_manifest;

		for (NewtonJoint* joint = NewtonBodyGetFirstContactJoint(triggerBody); joint; joint = NewtonBodyGetNextContactJoint(triggerBody, joint)) {
			dAssert(NewtonJointIsActive(joint));
			NewtonBody* const body0 = NewtonJointGetBody0(joint);
			NewtonBody* const body1 = NewtonJointGetBody1(joint);
			NewtonBody* cargoBody = (body0 != triggerBody) ? body0 : body1;
			dCustomTriggerController::dTriggerManifest::dTreeNode* uniqueEntryNode = manifest.Find(cargoBody);
			if (!uniqueEntryNode) {
				dCustomScopeLock lock(&m_lock);
				uniqueEntryNode = manifest.Insert(m_lru, cargoBody);
				dTrace(("entering trigger body:%d lru:%d frame:%d\n", NewtonBodyGetID(cargoBody), m_lru, xxxxx));
				OnEnter(&controller, cargoBody);
			}
			dTriggerGuestPair& cacheEntry = m_pairCache[m_cacheCount];
			cacheEntry.m_trigger = &controller;
			cacheEntry.m_bodyNode = uniqueEntryNode;
			m_cacheCount++;
		}
	}

	dCustomParallelListener::PreUpdate(timestep);

	for (dList<dCustomTriggerController>::dListNode* node = GetControllersList().GetFirst(); node; node = node->GetNext()) {
		dCustomTriggerController* const controller = &node->GetInfo();
		dCustomTriggerController::dTriggerManifest::Iterator iter(controller->m_manifest);

		for (iter.Begin(); iter;) {
			dCustomTriggerController::dTriggerManifest::dTreeNode* const node = iter.GetNode();
			iter++;
			if (node->GetInfo() != m_lru) {
				NewtonBody* const cargoBody = node->GetKey();

				dTrace(("exiting trigger body:%d lru:%d frame:%d\n\n", NewtonBodyGetID(cargoBody), node->GetInfo(), xxxxx));
				OnExit(controller, cargoBody);
				dCustomScopeLock lock(&m_lock);
				controller->m_manifest.Remove(cargoBody);
			}
		}
	}
}