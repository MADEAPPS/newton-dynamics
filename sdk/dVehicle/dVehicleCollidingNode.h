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


#ifndef __D_VEHICLE_COLLIDING_NODE_H__
#define __D_VEHICLE_COLLIDING_NODE_H__

#include "dStdafxVehicle.h"
#include "dVehicleNode.h"

class dVehicleMultiBody;

class dVehicleCollidingNode: public dVehicleNode
{
	public:
	dVehicleCollidingNode();
	~dVehicleCollidingNode();
	NewtonBody* GetNewtonBody() const { return m_body;}

	private:
	NewtonBody* m_body;
	friend class dVehicleTire;
	friend class dVehicleMultiBody;
};


#endif 

