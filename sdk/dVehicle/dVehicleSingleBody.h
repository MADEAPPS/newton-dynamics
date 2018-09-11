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


#ifndef __D_SINGLE_BODY_VEHICLE_H__
#define __D_SINGLE_BODY_VEHICLE_H__

#include "dStdafxVehicle.h"
#include "dVehicleInterface.h"
#include "dVehicleTireInterface.h"

class dVehicleSingleBody: public dVehicleInterface
{
	public:
	DVEHICLE_API dVehicleSingleBody(dVehicleChassis* const chassis);
	DVEHICLE_API virtual ~dVehicleSingleBody();

	DVEHICLE_API dVehicleTireInterface* AddTire (const dMatrix& locationInGlobalSpace, const dVehicleTireInterface::dTireInfo& tireInfo);
};


#endif 

