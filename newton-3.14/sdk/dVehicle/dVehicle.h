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


#ifndef __D_VEHICLE_H__
#define __D_VEHICLE_H__

#include "dStdafxVehicle.h"
#include "dVehicleNode.h"

class dTireInfo;
class dEngineInfo;
class dVehicleManager;
class dMultiBodyVehicleDashControl;

class dVehicle: public dVehicleNode
{
	public:
	DVEHICLE_API dVehicle (NewtonBody* const rootBody, const dMatrix& localFrame, dFloat gravityMag);
	DVEHICLE_API ~dVehicle();

	NewtonBody* GetBody() const { return m_newtonBody; }
	dVehicleManager* GetManager() const {return m_manager;}

	dVehicle* GetAsVehicle() const { return (dVehicle*)this; }
	const dMatrix& GetLocalFrame() const { return m_localFrame; }
	const dVector& GetGravity() const {return m_gravity;}
	virtual bool CheckSleeping() {return false;}

	protected:
	virtual void PreUpdate(dFloat timestep) {};
	virtual void PostUpdate(dFloat timestep) {};

	dMatrix m_localFrame;
	dVector m_gravity;
	dVector m_obbSize;
	dVector m_obbOrigin;

	NewtonBody* m_newtonBody;
	void* m_managerNode;
	dVehicleManager* m_manager;

	friend class dMultiBodyVehicleTire;
	friend class dVehicleManager;
};


#endif 

