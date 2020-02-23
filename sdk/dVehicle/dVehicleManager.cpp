/* Copyright (c) <2003-2019> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/

#include "dStdafxVehicle.h"
#include "dVehicle.h"
#include "dVehicleNode.h"
#include "dVehicleManager.h"

dVehicleManager::dVehicleManager(NewtonWorld* const world, const char* const managerName)
	:dCustomParallelListener(world, managerName)
	,m_list()
{
}

dVehicleManager::~dVehicleManager()
{
	while (m_list.GetCount()) {
		RemoveAndDeleteRoot(m_list.GetFirst()->GetInfo());
	}
}

void dVehicleManager::AddRoot(dVehicle* const root)
{
	dAssert(!root->m_manager);
	dAssert(!root->m_managerNode);
	root->m_managerNode = m_list.Append(root);
	root->m_manager = this;
}

void dVehicleManager::RemoveRoot(dVehicle* const root)
{
	if (root->m_managerNode) {
		OnRemove (root);
		dList<dVehicle*>::dListNode* const node = (dList<dVehicle*>::dListNode*) root->m_managerNode;
		root->m_managerNode = NULL;
		root->m_manager = NULL;
		m_list.Remove(node);
	}
}

void dVehicleManager::RemoveAndDeleteRoot(dVehicle* const root)
{
	RemoveRoot(root);
	delete root;
}

void dVehicleManager::OnDestroyBody(NewtonBody* const body)
{
	for (dList<dVehicle*>::dListNode* node = m_list.GetFirst(); node; node = node->GetNext()) {
		dVehicle* const vehicle = node->GetInfo();
		if (vehicle->m_newtonBody == body) {
			RemoveAndDeleteRoot(vehicle);
			break;
		}
	}
}

void dVehicleManager::OnDebug(dCustomJoint::dDebugDisplay* const debugContext)
{
	for (dList<dVehicle*>::dListNode* node = m_list.GetFirst(); node; node = node->GetNext()) {
		dVehicle* const vehicle = node->GetInfo();
		OnDebug(vehicle, debugContext);
		vehicle->Debug(debugContext);
	}
}

void dVehicleManager::PostUpdate(dFloat timestep, int threadID)
{
	D_TRACKTIME();
	NewtonWorld* const world = GetWorld();
	const int threadCount = NewtonGetThreadsCount(world);

	dList<dVehicle*>::dListNode* node = m_list.GetFirst();
	for (int i = 0; i < threadID; i++) {
		node = node ? node->GetNext() : NULL;
	}

	if (node) {
		do {
			dVehicle* const vehicle = node->GetInfo();
			//OnPostUpdate(vehicle, timestep);
			vehicle->PostUpdate(timestep);
			for (int i = 0; i < threadCount; i++) {
				node = node ? node->GetNext() : NULL;
			}
		} while (node);
	}
}

void dVehicleManager::PreUpdate(dFloat timestep, int threadID)
{
	D_TRACKTIME();
	NewtonWorld* const world = GetWorld();
	const int threadCount = NewtonGetThreadsCount(world);

	dList<dVehicle*>::dListNode* node = m_list.GetFirst();
	for (int i = 0; i < threadID; i++) {
		node = node ? node->GetNext() : NULL;
	}

	if (node) {
		do {
			dVehicle* const vehicle = node->GetInfo();
			//OnPreUpdate(vehicle, timestep);
			vehicle->PreUpdate(timestep);
			for (int i = 0; i < threadCount; i++) {
				node = node ? node->GetNext() : NULL;
			}
		} while (node);
	}
}

void dVehicleManager::PostStep(dFloat timestep, int threadID)
{
	D_TRACKTIME();
	NewtonWorld* const world = GetWorld();
	const int threadCount = NewtonGetThreadsCount(world);

	dList<dVehicle*>::dListNode* node = m_list.GetFirst();
	for (int i = 0; i < threadID; i++) {
		node = node ? node->GetNext() : NULL;
	}

	if (node) {
		do {
			dVehicle* const vehicle = node->GetInfo();
			OnUpdateTransform(vehicle);
			for (int i = 0; i < threadCount; i++) {
				node = node ? node->GetNext() : NULL;
			}
		} while (node);
	}
}

