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


#ifndef __D_VEHICLE_ENGINE_H__
#define __D_VEHICLE_ENGINE_H__

#include "dStdafxVehicle.h"
#include "dVehicleNode.h"
#include "dVehicleLoopJoint.h"

class dVehicleChassis;
class dVehicleDifferential;

class dEngineInfo
{
	public:
	enum dGearRatioIndex
	{
		m_reverseGear = 0,
		m_neutralGear = 1,
		m_firstGear = 2,
		m_gears_count = 10,
	};

	dEngineInfo()
	{
		memset(this, 0, sizeof(dEngineInfo));

		m_mass = 50.0f;
		m_armatureRadius = 0.125f;
		m_idleTorque = 200.0f;				// IDLE_TORQUE
		m_rpmAtIdleTorque = 450.0f;			// IDLE_TORQUE_RPM
		m_peakTorque = 500.0f;				// PEAK_TORQUE
		m_rpmAtPeakTorque = 3000.0f;		// PEAK_TORQUE_RPM
		m_peakHorsePower = 400.0f;			// PEAK_HP
		m_rpmAtPeakHorsePower = 5200.0f;	// PEAK_HP_RPM
		m_rpmAtRedLine = 6000.0f;			// REDLINE_TORQUE_RPM

		m_gearsCount = 8;
		m_crownGear = 4.0f;
		m_clutchTorque = 1000.0f;
		m_gearRatios[dEngineInfo::m_reverseGear] = -2.90f;	// reverse
		m_gearRatios[dEngineInfo::m_neutralGear] = 0.0f;    // neutral
		m_gearRatios[dEngineInfo::m_firstGear + 0] = 2.66f; // GEAR_1
		m_gearRatios[dEngineInfo::m_firstGear + 1] = 1.78f;	// GEAR_2
		m_gearRatios[dEngineInfo::m_firstGear + 2] = 1.30f;	// GEAR_3
		m_gearRatios[dEngineInfo::m_firstGear + 3] = 1.00f;	// GEAR_4
		m_gearRatios[dEngineInfo::m_firstGear + 4] = 0.74f;	// GEAR_5
		m_gearRatios[dEngineInfo::m_firstGear + 5] = 0.50f;	// GEAR_6
	}

	dFloat m_mass;
	dFloat m_armatureRadius;

	dFloat m_idleTorque;
	dFloat m_rpmAtIdleTorque;
	dFloat m_peakTorque;
	dFloat m_rpmAtPeakTorque;
	dFloat m_peakHorsePower;
	dFloat m_rpmAtPeakHorsePower;
	dFloat m_rpmAtRedLine;

	dFloat m_crownGear;
	dFloat m_clutchTorque;
	dFloat m_gearRatios[m_gears_count];
	int m_gearsCount;
};

class dVehicleEngine: public dVehicleNode, public dComplementaritySolver::dBilateralJoint
{
	class dEngineTorqueNode
	{
		public:
		dEngineTorqueNode() {}
		dEngineTorqueNode(dFloat rpm, dFloat torque)
			:m_rpm(rpm)
			,m_torque(torque)
		{
		}

		dFloat m_rpm;
		dFloat m_torque;
	};

	class dEngineMetricInfo: public dEngineInfo
	{
		public:
		dEngineMetricInfo(const dEngineInfo& info);
		dFloat m_peakPowerTorque;
		//dFloat m_crownGearRatio;

		dFloat GetTorque (dFloat rpm) const;

		dEngineTorqueNode m_torqueCurve[5];
	};

	public:
	DVEHICLE_API dVehicleEngine(dVehicleChassis* const chassis, const dEngineInfo& info, dVehicleDifferential* const differential);
	DVEHICLE_API virtual ~dVehicleEngine();

	protected:
/*
	virtual dFloat GetRpm() const;
	virtual dFloat GetRedLineRpm() const;

	void ApplyExternalForce(dFloat timestep);
	void InitEngineTorqueCurve();
	void Integrate(dFloat timestep);

	void SetGear (int gear);
	void SetClutch (dFloat clutch);
	void SetThrottle (dFloat throttle);

	void SetInfo(const dEngineInfo& info);
	
	dComplementaritySolver::dBilateralJoint* GetProxyJoint();
	int GetKinematicLoops(dAnimIDRigKinematicLoopJoint** const jointArray);

	void Debug(dCustomJoint::dDebugDisplay* const debugContext) const;
	

	
	dEngineBlockJoint m_blockJoint;
	dEngineCrankJoint m_crankJoint;
	dGearBoxJoint m_gearBox;
	dFloat m_omega;
*/
	dEngineMetricInfo m_metricInfo;
	dVehicleDifferential* m_differential;
	friend class dVehicleChassis;
};


#endif 

