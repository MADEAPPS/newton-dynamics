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


#include "dStdafxVehicle.h"
#include "dVehicleDashControl.h"

class dVehicleChassis;

dVehicleDashControl::dVehicleDashControl(dVehicleChassis* const vehicle)
	:m_vehicle(vehicle)
	,m_param(0.0f)
	,m_paramMemory(0.0f)
	,m_timer(60)
{
}

dVehicleDashControl::~dVehicleDashControl()
{
}

void dVehicleDashControl::SetParam(dFloat param)
{
	m_paramMemory = m_param;
	m_param = param;
}

bool dVehicleDashControl::ParamChanged() const
{
	m_timer--;
	if (dAbs(m_paramMemory - m_param) > 1.e-3f) {
		m_timer = 30;
	}
	return m_timer > 0;
}



