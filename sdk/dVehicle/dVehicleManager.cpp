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
#include "dVehicleManager.h"

dVehicleManager::dVehicleManager(NewtonWorld* const world)
	:dCustomParallelListener(world, D_VEHICLE_MANAGER_NAME)
	,m_list()
{
}

dVehicleManager::~dVehicleManager()
{
}

dVehicleChassis* dVehicleManager::CreateSingleBodyVehicle(NewtonBody* const body, const dMatrix& vehicleFrame, dFloat gravityMag)
{
	dVehicleChassis* const vehicle = &m_list.Append()->GetInfo();
	vehicle->Init(body, vehicleFrame, gravityMag);
	return vehicle;
}

void dVehicleManager::DestroyController(dVehicleChassis* const vehicle)
{
//	dAssert();
//	vehicle->Cleanup();
	dAssert (0);
	dList<dVehicleChassis>::dListNode* const node = m_list.GetNodeFromInfo(*vehicle);
	m_list.Remove(node);
}

void dVehicleManager::OnDebug(dCustomJoint::dDebugDisplay* const debugContext)
{
	for (dList<dVehicleChassis>::dListNode* vehicleNode = m_list.GetFirst(); vehicleNode; vehicleNode = vehicleNode->GetNext()) {
		dVehicleChassis* const vehicle = &vehicleNode->GetInfo();
		vehicle->Debug(debugContext);
	}
}

void dVehicleManager::PreUpdate(dFloat timestep, int threadID)
{
	NewtonWorld* const world = GetWorld();
	const int threadCount = NewtonGetThreadsCount(world);

	dList<dVehicleChassis>::dListNode* node = m_list.GetFirst();
	for (int i = 0; i < threadID; i++) {
		node = node ? node->GetNext() : NULL;
	}

	if (node) {
		do {
			dVehicleChassis& chassis = node->GetInfo();
			chassis.PreUpdate(timestep);
			for (int i = 0; i < threadCount; i++) {
				node = node ? node->GetNext() : NULL;
			}
		} while (node);
	}
}

void dVehicleManager::PostUpdate(dFloat timestep, int threadID)
{
	NewtonWorld* const world = GetWorld();
	const int threadCount = NewtonGetThreadsCount(world);

	dList<dVehicleChassis>::dListNode* node = m_list.GetFirst();
	for (int i = 0; i < threadID; i++) {
		node = node ? node->GetNext() : NULL;
	}

	if (node) {
		do {
			dVehicleChassis& chassis = node->GetInfo();
			chassis.PostUpdate(timestep);
			for (int i = 0; i < threadCount; i++) {
				node = node ? node->GetNext() : NULL;
			}
		} while (node);
	}
}

void dVehicleManager::PostStep(dFloat timestep, int threadID)
{
	NewtonWorld* const world = GetWorld();
	const int threadCount = NewtonGetThreadsCount(world);

	dList<dVehicleChassis>::dListNode* node = m_list.GetFirst();
	for (int i = 0; i < threadID; i++) {
		node = node ? node->GetNext() : NULL;
	}

	if (node) {
		do {
			const dVehicleChassis& chassis = node->GetInfo();
			OnUpdateTransform(&chassis, timestep);
			for (int i = 0; i < threadCount; i++) {
				node = node ? node->GetNext() : NULL;
			}
		} while (node);
	}
}