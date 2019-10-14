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
#include "dVehicleNode.h"
#include "dVehicleChassis.h"
#include "dVehicleManager.h"

dVehicleManager::dVehicleManager(NewtonWorld* const world)
	:dCustomParallelListener(world, D_VEHICLE_MANAGER_NAME)
	,m_list()
{
}

dVehicleManager::~dVehicleManager()
{
	while (m_list.GetCount()) {
		RemoveAndDeleteRoot(m_list.GetFirst()->GetInfo());
	}
}

void dVehicleManager::AddRoot(dVehicleChassis* const root)
{
	dAssert(!root->m_node);
	dAssert(!root->m_manager);
	root->m_node = m_list.Append(root);
	root->m_manager = this;
}

void dVehicleManager::RemoveRoot(dVehicleChassis* const root)
{
	if (root->m_node) {
		dList<dVehicleChassis*>::dListNode* const node = (dList<dVehicleChassis*>::dListNode*) root->m_node;
		root->m_node = NULL;
		root->m_manager = NULL;
		m_list.Remove(node);
	}
}

void dVehicleManager::RemoveAndDeleteRoot(dVehicleChassis* const root)
{
	RemoveRoot(root);
	delete root;
}


#if 0
dVehicleChassis* dVehicleManager::CreateSingleBodyVehicle(NewtonBody* const body, const dMatrix& vehicleFrame, NewtonApplyForceAndTorque forceAndTorque, dFloat gravityMag)
{
	dVehicleChassis* const vehicle = &m_list.Append()->GetInfo();
	vehicle->Init(body, vehicleFrame, forceAndTorque, gravityMag);
	return vehicle;
}

dVehicleChassis* dVehicleManager::CreateSingleBodyVehicle(NewtonCollision* const chassisShape, const dMatrix& vehicleFrame, dFloat mass, NewtonApplyForceAndTorque forceAndTorque, dFloat gravityMag)
{
	dVehicleChassis* const vehicle = &m_list.Append()->GetInfo();
	vehicle->Init(GetWorld(), chassisShape, mass, vehicleFrame, forceAndTorque, gravityMag);
	return vehicle;
}

void dVehicleManager::DestroyController(dVehicleChassis* const vehicle)
{
//	dAssert();
//	vehicle->Cleanup();
	dList<dVehicleChassis>::dListNode* const node = m_list.GetNodeFromInfo(*vehicle);
	m_list.Remove(node);
}

#endif

void dVehicleManager::OnDebug(dCustomJoint::dDebugDisplay* const debugContext)
{
	for (dList<dVehicleChassis*>::dListNode* vehicleNode = m_list.GetFirst(); vehicleNode; vehicleNode = vehicleNode->GetNext()) {
		dVehicleChassis* const vehicle = vehicleNode->GetInfo();
		OnDebug(vehicle, debugContext);
		vehicle->Debug(debugContext);
	}
}

void dVehicleManager::PostUpdate(dFloat timestep, int threadID)
{
	D_TRACKTIME();
	NewtonWorld* const world = GetWorld();
	const int threadCount = NewtonGetThreadsCount(world);

	dList<dVehicleChassis*>::dListNode* node = m_list.GetFirst();
	for (int i = 0; i < threadID; i++) {
		node = node ? node->GetNext() : NULL;
	}

	if (node) {
		do {
			dVehicleChassis* const chassis = node->GetInfo();
			OnPostUpdate(chassis, timestep);
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

	dList<dVehicleChassis*>::dListNode* node = m_list.GetFirst();
	for (int i = 0; i < threadID; i++) {
		node = node ? node->GetNext() : NULL;
	}

	if (node) {
		do {
			dVehicleChassis* const chassis = node->GetInfo();
			OnUpdateTransform(chassis);
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

	dList<dVehicleChassis*>::dListNode* node = m_list.GetFirst();
	for (int i = 0; i < threadID; i++) {
		node = node ? node->GetNext() : NULL;
	}

	if (node) {
		do {
			dVehicleChassis* const chassis = node->GetInfo();
			OnPreUpdate(chassis, timestep);
			chassis->PreUpdate(timestep);
			for (int i = 0; i < threadCount; i++) {
				node = node ? node->GetNext() : NULL;
			}
		} while (node);
	}
}

