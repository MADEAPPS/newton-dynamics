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


CustomTriggerController::PassangerManifest::PassangerManifest ()
	:m_count(0)
	,m_capacity(0)
	,m_passangerList (NULL)
{
}

CustomTriggerController::PassangerManifest::~PassangerManifest ()
{
	if (m_passangerList) {
		NewtonFree(m_passangerList);
	}
}



CustomTriggerController::PassangerManifest::Passenger* CustomTriggerController::PassangerManifest::Find (NewtonBody* const m_body)
{
	// for now just do a lnera search
	// if m_count is even large that 16 then its time to chnag ethe algorithm 
	dAssert (m_count < 16);
	for (int i = 0; i < m_count; i ++) {
		if (m_passangerList[i].m_body == m_body) {
			return &m_passangerList[i];
		}
	}
	return NULL;
}

CustomTriggerController::PassangerManifest::Passenger* CustomTriggerController::PassangerManifest::Insert (NewtonBody* const m_body)
{
	dAssert (m_count <= m_capacity);

	if (m_count == m_capacity) {
		if (m_capacity == 0) {
			m_capacity = 4;
			m_passangerList = (Passenger*) NewtonAlloc(m_capacity * sizeof (Passenger));
		} else {
			Passenger* const oldList = m_passangerList;
			m_passangerList = (Passenger*) NewtonAlloc(2 * m_capacity * sizeof (Passenger));
			memcpy (m_passangerList, oldList, m_capacity * sizeof (Passenger));
			NewtonFree(oldList);
			m_capacity = m_capacity * 2;
		}
	}
	Passenger* const passenger = &m_passangerList[m_count];
	passenger->m_lru = 0;
	passenger->m_body = m_body;
	m_count ++;
	return passenger;
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
	CustomControllerManager<CustomTriggerController>::PreUpdate(timestep);
}


void CustomTriggerController::PostUpdate(dFloat timestep, int threadIndex)
{
}


void CustomTriggerController::SetUserData(void* const userData)
{
	m_userData = userData;
}

const void* CustomTriggerController::GetUserData() const
{
	return m_userData;
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
		//PassangerManifest::dTreeNode* node = m_manifest.Find (passangerBody);
		PassangerManifest::Passenger* passenger = m_manifest.Find (passangerBody);
		if (!passenger) {
			//node = m_manifest.Insert (passangerBody);
			passenger = m_manifest.Insert (passangerBody);
			manager->EventCallback (this, CustomTriggerManager::m_enterTrigger, passangerBody);
		} else {
			manager->EventCallback (this, CustomTriggerManager::m_inTrigger, passangerBody);
		}
		passenger->m_lru = lru;
	}
	
	for (int i = 0; i < m_manifest.m_count; i ++) {
		PassangerManifest::Passenger* const node = &m_manifest.m_passangerList[i];
		if (node->m_lru != lru) {
			manager->EventCallback (this, CustomTriggerManager::m_exitTrigger, node->m_body);			
			m_manifest.m_passangerList[i] = m_manifest.m_passangerList[m_manifest.m_count - 1];
			i --;
			m_manifest.m_count --;
		}
	}
}
