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


#ifndef __D_VEHICLE_DASH_CONTROL_H__
#define __D_VEHICLE_DASH_CONTROL_H__

#include "dStdafxVehicle.h"

class dVehicleTire;
class dVehicleEngine;
class dVehicleChassis;

class dVehicleDashControl
{
	public:
	dVehicleDashControl()
		:m_vehicle(NULL)
		,m_param(0.0f)
		,m_paramMemory(0.0f)
		,m_timer(60)
	{
	}

	virtual ~dVehicleDashControl()
	{
	}

	void Init(dVehicleChassis* const vehicle)
	{
		m_vehicle = vehicle;
	}

	void SetParam(dFloat param)
	{
		m_paramMemory = m_param;
		m_param = param;
	}


	dFloat GetParam() const
	{
		return m_param;
	}

	bool ParamChanged() const
	{
		m_timer--;
		if (dAbs(m_paramMemory - m_param) > 1.e-3f) {
			m_timer = 30;
		}
		return m_timer > 0;
	}

	virtual void Update(dFloat timestep) = 0;

	protected:
	dVehicleChassis* m_vehicle;
	dFloat m_param;
	dFloat m_paramMemory;
	mutable dFloat m_timer;
};

class dVehicleTireControl: public dVehicleDashControl
{
	protected:
	dVehicleTireControl()
		:m_tires()
		,m_isSleeping(false)
	{
	}

	public:
	void AddTire(dVehicleTire* const tire)
	{
		m_tires.Append(tire);
	}

	protected:
	dList<dVehicleTire*> m_tires;
	bool m_isSleeping;
	friend class dVehicleChassis;
};

class dVehicleSteeringControl: public dVehicleTireControl
{
	protected:
	virtual void Update(dFloat timestep);
	friend class dVehicleChassis;
};

class dVehicleBrakeControl: public dVehicleTireControl
{
	public:
	dVehicleBrakeControl()
	{
	}

	dFloat GetBrakeTorque() const
	{
		return m_maxTorque;
	}

	void SetBrakeTorque(dFloat torque)
	{
		m_maxTorque = dAbs(torque);
	}

	protected:
	virtual void Update(dFloat timestep);
	dFloat m_maxTorque;
	friend class dVehicleChassis;
};

class dVehicleEngineControl: public dVehicleDashControl 
{
	public:
	dVehicleEngine* GetEngine() const {return m_engine;}
	void SetEngine (dVehicleEngine* const engine) {m_engine = engine;}
//	void SetGear (int gear);
//	void SetClutch (dFloat clutch);
	virtual void Update(dFloat timestep) {}

	private:
	dVehicleEngine* m_engine;
};


#endif 

