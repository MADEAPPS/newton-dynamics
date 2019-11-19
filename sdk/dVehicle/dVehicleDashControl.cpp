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
#include "dVehicleTire.h"
#include "dVehicleEngine.h"
#include "dVehicleDashControl.h"

// ****************************************************************************
//
// ****************************************************************************
void dVehicleSteeringControl::Update(dFloat timestep)
{
	m_isSleeping = true;
	for (dList<dVehicleTire*>::dListNode* node = m_tires.GetFirst(); node; node = node->GetNext()) {
		dVehicleTire* const tire = node->GetInfo();
		const dTireInfo& info = tire->GetInfo();

		dFloat angle = tire->GetSteeringAngle();
		dFloat targetAngle = m_param * info.m_maxSteeringAngle;
		if (angle < targetAngle) {
			angle += info.m_steerRate * timestep;
			if (angle > targetAngle) {
				angle = targetAngle;
			}
		} else if (angle > targetAngle) {
			angle -= info.m_steerRate * timestep;
			if (angle < targetAngle) {
				angle = targetAngle;
			}
		}
		tire->SetSteeringAngle(angle);
		m_isSleeping &= dAbs(targetAngle - angle) < 1.0e-4f;
	}
}


void dVehicleBrakeControl::Update(dFloat timestep)
{
	m_isSleeping = true;
	for (dList<dVehicleTire*>::dListNode* node = m_tires.GetFirst(); node; node = node->GetNext()) {
		dVehicleTire* const tire = node->GetInfo();
		dFloat torque = dMax(m_maxTorque * m_param, tire->GetBrakeTorque());
		tire->SetBrakeTorque(torque);
	}
}

void dVehicleEngineControl::Update(dFloat timestep)
{
	if (m_engine) {
		m_engine->UpdateAutomaticGearBox(timestep);
		m_engine->SetThrottle (m_param, timestep);
	}
}

void dVehicleEngineControl::SetDifferential (int mode)
{
	if (m_engine) {
		m_engine->SetDifferentialMode(mode);
	}
}

void dVehicleEngineControl::SetGear (int gear)
{
	if (m_engine) {
		dAssert(0);
//		m_engine->SetGear(gear);
	}
}

void dVehicleEngineControl::SetClutch (dFloat clutch)
{
	if (m_engine) {
		dAssert(0);
//		m_engine->SetClutch(clutch);
	}
}
