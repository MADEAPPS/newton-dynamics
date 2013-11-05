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
#include "CustomJoint.h"
#include "CustomTriggerManager.h"



CustomTriggerController::PassangerManifest::PassangerManifest ()
	:m_count(0)
	,m_capacity(0)
	,m_passangerList (NULL)
{
}

CustomTriggerController::PassangerManifest::~PassangerManifest ()
{
	if (m_passangerList) {
		NewtonFree (m_passangerList);
	}
}



CustomTriggerController::Passenger* CustomTriggerController::PassangerManifest::Find (NewtonBody* const body)
{
	int i0 = 0;
	int i1 = m_count - 1;
	while ((i1 - i0) >= 4) {
		int i = (i1 + i0) >> 1;
		if (body < m_passangerList[i]->GetInfo().m_body) {
			i1 = i;
		} else {
			i0 = i;
		}
	}

//	dAssert (!m_count || (m_passangerList[i0].m_body <= body));
	for (int i = i0; (i < m_count) && (m_passangerList[i]->GetInfo().m_body <= body); i ++) {
		if (m_passangerList[i]->GetInfo().m_body == body) {
			return &m_passangerList[i]->GetInfo();
		}
	}
	return NULL;
}

CustomTriggerController::Passenger* CustomTriggerController::PassangerManifest::Insert (NewtonBody* const body, CustomTriggerController* const controller)
{
	dAssert (m_count <= m_capacity);

	CustomListNode* const node = Append();
	Passenger* const passenger = &node->GetInfo();
	passenger->m_lru = 0;
	passenger->m_body = body;
	passenger->m_controller = controller;
	
	if (m_count == m_capacity) {
		if (m_capacity == 0) {
			m_capacity = 4;
			m_passangerList = (CustomListNode**) NewtonAlloc(m_capacity * sizeof (CustomListNode*));
		} else {
			CustomListNode** const oldList = m_passangerList;
			m_passangerList = (CustomListNode**) NewtonAlloc(2 * m_capacity * sizeof (CustomListNode*));
			memcpy (m_passangerList, oldList, m_capacity * sizeof (CustomListNode*));
			NewtonFree (oldList);
			m_capacity = m_capacity * 2;
		}
	}

	int index = m_count;
	for (int i = m_count - 1; i >= 0; i --) {
		if (m_passangerList[i]->GetInfo().m_body > body) {
			index = i;
			m_passangerList[i + 1] = m_passangerList[i];
		} else {
			break;
		}
	}
	m_passangerList[index] = node;
	m_count = GetCount();

	return passenger;
}

void CustomTriggerController::PassangerManifest::Pack ()
{
	for (int index = 0; index < m_count; index ++) {
		if (!m_passangerList[index]) {
			for (int i = index + 1; i < m_count; i ++) {
				if (m_passangerList[i]) {
					m_passangerList[index] = m_passangerList[i];
					m_passangerList[i] = NULL;
					break;
				}
			}
		}
	}
	m_count = GetCount();	

	#ifdef _DEBUG
	for (int i = 0; i < m_count; i ++) {
		CustomListNode* const node = m_passangerList[i];
		dAssert (node);
		dAssert (node->GetInfo().m_body);
	}
	#endif
}

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
	for (CustomListNode* node = GetFirst(); node; node = node->GetNext()) {
		UpdateTrigger (&node->GetInfo());
	}
	CustomControllerManager<CustomTriggerController>::PreUpdate(timestep);
}

void CustomTriggerManager::EnterTriggerKernel (NewtonWorld* const world, void* const context, int threadIndex)
{
	const CustomTriggerController::Passenger* const passenger = (CustomTriggerController::Passenger*) context;
	CustomTriggerController* const controller = passenger->m_controller;
	CustomTriggerManager* const manager = (CustomTriggerManager*)controller->GetManager();
	manager->EventCallback (controller, m_enterTrigger, passenger->m_body);
}


void CustomTriggerManager::InTriggerKernel (NewtonWorld* const world, void* const context, int threadIndex)
{
	const CustomTriggerController::Passenger* const passenger = (CustomTriggerController::Passenger*) context;
	CustomTriggerController* const controller = passenger->m_controller;
	CustomTriggerManager* const manager = (CustomTriggerManager*)controller->GetManager();
	manager->EventCallback (controller, m_inTrigger, passenger->m_body);
}


void CustomTriggerManager::UpdateTrigger (CustomTriggerController* const controller)
{
	NewtonBody* const triggerBody = controller->GetBody();
	CustomTriggerController::PassangerManifest& manifest = controller->m_manifest;

	for (NewtonJoint* joint = NewtonBodyGetFirstContactJoint (triggerBody); joint; joint = NewtonBodyGetNextContactJoint (triggerBody, joint)) {
		NewtonBody* const body0 = NewtonJointGetBody0(joint);
		NewtonBody* const body1 = NewtonJointGetBody1(joint);
		NewtonBody* const passangerBody = (body0 != triggerBody) ? body0 : body1; 

		CustomTriggerController::Passenger* passenger = manifest.Find (passangerBody);
		if (!passenger) {
			passenger = manifest.Insert (passangerBody, controller);
			NewtonDispachThreadJob (m_world, EnterTriggerKernel, passenger);
		} else {
			NewtonDispachThreadJob (m_world, InTriggerKernel, passenger);
		}
		passenger->m_lru = m_lru;
	}
	NewtonSyncThreadJobs(m_world);
}


CustomTriggerController::CustomTriggerController()
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
	NewtonCollisionSetCollisonMode(collision, 0);
}


void CustomTriggerController::PostUpdate(dFloat timestep, int threadIndex)
{
}

void CustomTriggerController::PreUpdate(dFloat timestep, int threadIndex)
{
	CustomTriggerManager* const manager = (CustomTriggerManager*)GetManager();
	unsigned lru = manager->m_lru;

	bool packArray = false;
	for (int i = 0; i < m_manifest.m_count; i ++) {
		dAssert (m_manifest.m_passangerList[i]);
		Passenger* const passenger = &m_manifest.m_passangerList[i]->GetInfo();
		if (passenger->m_lru != lru) {
			packArray = true;
			manager->EventCallback (this, CustomTriggerManager::m_exitTrigger, passenger->m_body);	
			m_manifest.Remove(m_manifest.m_passangerList[i]);
			m_manifest.m_passangerList[i] = NULL;
		}
	}
	if (packArray) {
		m_manifest.Pack();
	}
}
