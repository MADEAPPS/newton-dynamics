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

class dVehicleMultiBody;
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
		m_topSpeedInMetersPerSeconds = 50.0f;					// 50 m/s (160 kmh) top speed

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
	dFloat m_topSpeedInMetersPerSeconds;

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

		dFloat GetTorque (dFloat rpm) const;

		dEngineTorqueNode m_torqueCurve[5];
	};

	class dGearBoxAndClutchJoint: public dVehicleLoopJoint
	{
		public:
		dGearBoxAndClutchJoint()
			:dVehicleLoopJoint()
			,m_gearRatio(2.6f)
			,m_crowndGear(6.0f)
			,m_clutchTorque(D_COMPLEMENTARITY_MAX_FRICTION_BOUND)
		{
			m_isActive = true;
		}

		void SetGearRatio(dFloat ratio)
		{
			m_gearRatio = ratio;
		}

		void SetClutchTorque(dFloat clutchTorque)
		{
			m_clutchTorque = clutchTorque;
		}

		private:
		int GetMaxDof() const { return 1; }

		void JacobianDerivative(dComplementaritySolver::dParamInfo* const constraintParams);
		void UpdateSolverForces(const dComplementaritySolver::dJacobianPair* const jacobians) const { dAssert(0); }

		dFloat m_gearRatio;
		dFloat m_crowndGear;
		dFloat m_clutchTorque;
	};

	public:
	DVEHICLE_API dVehicleEngine(dVehicleMultiBody* const chassis, const dEngineInfo& info, dVehicleDifferential* const differential);
	DVEHICLE_API virtual ~dVehicleEngine();

	DVEHICLE_API void SetInfo(const dEngineInfo& info);
	const dEngineInfo& GetInfo() const { return m_info; }
	
	DVEHICLE_API dFloat GetSpeed() const;
	dFloat GetTopSpeed() const {return m_info.m_topSpeedInMetersPerSeconds;}

	DVEHICLE_API dFloat GetRpm() const;
	DVEHICLE_API dFloat GetRedLineRpm() const;
	//void SetClutch (dFloat clutch);

	DVEHICLE_API void SetIgnition(bool mode);
	bool GetIgnition() const { return m_ignitionKey0; }
	
	DVEHICLE_API void SetGear (dEngineInfo::dGearRatioIndex gear);
	DVEHICLE_API dEngineInfo::dGearRatioIndex GetGear () const {return m_currentGear;}

	DVEHICLE_API void UpdateAutomaticGearBox(dFloat timestep);
	DVEHICLE_API void SetThrottle (dFloat throttle, dFloat timestep);

	int GetDifferentialMode() const {return m_differentialMode;}
	DVEHICLE_API void SetDifferentialMode(int differentialMode);

	DVEHICLE_API bool InputChanged() const;

	protected:
	void CalculateFreeDof();
	void ApplyExternalForce();
	void InitEngineTorqueCurve();
	void Integrate(dFloat timestep);
	int GetKinematicLoops(dVehicleLoopJoint** const jointArray);

	dVehicleEngine* GetAsEngine() const { return (dVehicleEngine*)this;}
	dComplementaritySolver::dBilateralJoint* GetJoint() { return this; }

	const void Debug(dCustomJoint::dDebugDisplay* const debugContext) const;
	void JacobianDerivative(dComplementaritySolver::dParamInfo* const constraintParams);
	void UpdateSolverForces(const dComplementaritySolver::dJacobianPair* const jacobians) const;

	dMatrix m_localAxis;
	dEngineInfo m_info;
	dEngineMetricInfo m_metricInfo;
	dGearBoxAndClutchJoint m_gearBox;
	dVehicleDifferential* m_differential;
	dFloat m_omega;
	dFloat m_throttle;
	dFloat m_throttleSpeed;
	int m_differentialMode;
	dEngineInfo::dGearRatioIndex m_currentGear;
	bool m_ignitionKey0;
	bool m_ignitionKey1;

	friend class dVehicleMultiBody;
};


#endif 

