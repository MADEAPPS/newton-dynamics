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
class dVehicleDashControl;

class dVehicle: public dVehicleNode
{
	public:
	class dDriverInput
	{
		public:
		dDriverInput()
			:m_throttle(0.0f)
			,m_brakePedal(0.0f)
			,m_clutchPedal(0.0f)
			,m_steeringValue(0.0f)
			,m_handBrakeValue(0.0f)
			,m_gear(0)
			,m_ignitionKey(0)
			,m_differentialMode(0)
			,m_manualTransmission(0)
		{
		}

		dFloat m_throttle;
		dFloat m_brakePedal;
		dFloat m_clutchPedal;
		dFloat m_steeringValue;
		dFloat m_handBrakeValue;
		int m_gear;
		int m_ignitionKey;
		int m_differentialMode;
		int m_manualTransmission;
	};

	DVEHICLE_API dVehicle (NewtonBody* const rootBody, const dMatrix& localFrame, dFloat gravityMag);
	DVEHICLE_API ~dVehicle();

	NewtonBody* GetBody() const { return m_newtonBody; }
	dVehicle* GetAsVehicle() const { return (dVehicle*)this; }
	const dMatrix& GetLocalFrame() const { return m_localFrame; }
	const dVector& GetGravity() const {return m_gravity;}

	void AddControl (dVehicleDashControl* const control);
	void RemoveControl (dVehicleDashControl* const control);
	int GetControlCount() const {return m_controlerCount;}

	virtual bool CheckSleeping() {return false;}
	virtual void ApplyDriverInputs(const dDriverInput& driveInputs, dFloat timestep) {}

	protected:
	virtual void PreUpdate(dFloat timestep) {};
	virtual void PostUpdate(dFloat timestep) {};

	dMatrix m_localFrame;
	dVector m_gravity;
	dVector m_obbSize;
	dVector m_obbOrigin;
	dArray<dVehicleDashControl*> m_control;
	NewtonBody* m_newtonBody;
	void* m_managerNode;
	dVehicleManager* m_manager;
	int m_controlerCount;

	friend class dVehicleTire;
	friend class dVehicleManager;
};


#endif 

