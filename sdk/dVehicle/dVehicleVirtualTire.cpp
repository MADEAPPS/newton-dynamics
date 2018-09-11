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
#include "dVehicleVirtualTire.h"

dVehicleVirtualTire::dVehicleVirtualTire(dVehicleNode* const parent, const dMatrix& location, const dTireInfo& info)
	:dVehicleTireInterface(parent, location, info)
{
}

dVehicleVirtualTire::~dVehicleVirtualTire()
{
}

NewtonCollision* dVehicleVirtualTire::GetCollisionShape() const
{
	return NULL;
}
