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


#ifndef __D_VEHICLE_DASH_CONTROL_H__
#define __D_VEHICLE_DASH_CONTROL_H__

#include "dStdafxVehicle.h"

class dVehicleChassis;

class dVehicleDashControl: public dCustomAlloc
{
	public:
	DVEHICLE_API dVehicleDashControl(dVehicleChassis* const vehicle);
	DVEHICLE_API virtual ~dVehicleDashControl();

	DVEHICLE_API void SetParam(dFloat param);
	DVEHICLE_API dFloat GetParam() const{return m_param;}

	DVEHICLE_API bool ParamChanged() const;
	virtual void Update(dFloat timestep) = 0;

	protected:
	dVehicleChassis* m_vehicle;
	dFloat m_param;
	dFloat m_paramMemory;
	mutable dFloat m_timer;
};

#endif 

