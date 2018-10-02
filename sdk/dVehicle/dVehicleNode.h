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


#ifndef __D_VEHICLE_NODE_H__
#define __D_VEHICLE_NODE_H__

#include "dStdafxVehicle.h"

class dVehicleInterface;
class dVehicleTireInterface;

class dVehicleNode: public dContainersAlloc
{
	public:
	DVEHICLE_API dVehicleNode(dVehicleNode* const parent);
	DVEHICLE_API virtual ~dVehicleNode();

	DVEHICLE_API void* GetUserData();
	DVEHICLE_API void SetUserData(void* const userData);
	DVEHICLE_API virtual void Debug(dCustomJoint::dDebugDisplay* const debugContext) const;
	
	DVEHICLE_API virtual void InitRigiBody(dFloat timestep);

	virtual dVehicleInterface* GetAsVehicle() const {return NULL;} 
	virtual dVehicleTireInterface* GetAsTire() const {return NULL;} 

	void* m_userData;
	dVehicleNode* m_parent;
	dList<dVehicleNode*> m_children;
	dComplentaritySolver::dBodyState m_body;
	dComplentaritySolver::dBilateralJoint* m_articulation;

	int m_solverIndex;
};


#endif 

