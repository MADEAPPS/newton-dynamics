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


#ifndef __D_VEHICLE_INTERFACE_H__
#define __D_VEHICLE_INTERFACE_H__

#include "dStdafxVehicle.h"
#include "dVehicleNode.h"
#include "dVehicleTireInterface.h"

class dVehicleChassis;


class dVehicleInterface: public dVehicleNode
{
	public:
	DVEHICLE_API dVehicleInterface(dVehicleChassis* const chassis);
	DVEHICLE_API virtual ~dVehicleInterface();

	virtual dMatrix GetMatrix () const = 0;
	virtual dVehicleTireInterface* AddTire (const dVector& locationInGlobalSpace, const dVehicleTireInterface::dTireInfo& tireInfo) = 0;

	DVEHICLE_API dVehicleChassis* GetChassis() const;
	DVEHICLE_API virtual dVehicleInterface* GetAsVehicle() const {return (dVehicleInterface*)this;} 

	protected:
	dVehicleChassis* m_chassis;
};


#endif 

