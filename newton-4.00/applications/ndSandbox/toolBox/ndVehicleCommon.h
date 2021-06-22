/* Copyright (c) <2003-2021> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/

#ifndef __D_VEHICLE_COMMON_UTIL__
#define __D_VEHICLE_COMMON_UTIL__

#include "ndSandboxStdafx.h"
#include "ndDemoEntity.h"

#define AUTOMATION_TRANSMISSION_FRAME_DELAY 120

class ndVehicleDectriptor
{
	public:
	class ndTorqueTap
	{
		public:
		ndTorqueTap() {}
		ndTorqueTap(dFloat32 rpm, dFloat32 torqueInPoundFeet)
			:m_radPerSeconds(rpm * 0.105f)
			,m_torqueInNewtonMeters(torqueInPoundFeet * 1.36f)
		{
		}
		dFloat32 m_radPerSeconds;
		dFloat32 m_torqueInNewtonMeters;
	};

	class ndEngineTorqueCurve
	{
		public:
		ndEngineTorqueCurve();
	
		void Init(dFloat32 fuelInjectionRate,
			dFloat32 idleTorquePoundFoot, dFloat32 idleRmp,
			dFloat32 horsePower, dFloat32 rpm0, dFloat32 rpm1,
			dFloat32 horsePowerAtRedLine, dFloat32 redLineRpm);

		dFloat32 GetFuelRate() const;
		dFloat32 GetIdleRadPerSec() const;
		dFloat32 GetRedLineRadPerSec() const;
		dFloat32 GetLowGearShiftRadPerSec() const;
		dFloat32 GetHighGearShiftRadPerSec() const;
		dFloat32 GetTorque(dFloat32 omegaInRadPerSeconds) const;
	
		private:
		ndTorqueTap m_torqueCurve[5];
		dFloat32 m_fuelInjectionRate;
	};

	class ndGearBox
	{
		public:
		dInt32 m_gearsCount;
		union
		{
			struct
			{
				dFloat32 m_fowardRatios[5];
				dFloat32 m_reverseRatio;
				dFloat32 m_neutral;
			};
			dFloat32 m_ratios[8];
		};

		dFloat32 m_idleClutchTorque;
		dFloat32 m_lockedClutchTorque;
		dFloat32 m_crownGearRatio;
		dFloat32 m_torqueConverter;
		bool m_manual;
	};

	class ndTireDefinition
	{
		public:
		dFloat32 m_mass;
		dFloat32 m_steeringAngle;
		dFloat32 m_springK;
		dFloat32 m_damperC;
		dFloat32 m_regularizer;
		dFloat32 m_upperStop;
		dFloat32 m_lowerStop;
		dFloat32 m_brakeTorque;
		dFloat32 m_verticalOffset;
		dFloat32 m_handBrakeTorque;
		dFloat32 m_laterialStiffness;
		dFloat32 m_longitudinalStiffness ;
	};

	enum ndDifferentialType
	{
		m_rearWheelDrive,
		m_frontWheelDrive,
		m_fourWheeldrive,
		m_eightWheeldrive,
	};

	enum ndTorsionBarType
	{
		m_noWheelAxle,
		m_rearWheelAxle,
		m_frontWheelAxle,
		m_fourWheelAxle,
	};

	ndVehicleDectriptor() {}
	ndVehicleDectriptor(const char* const fileName);

	dVector m_comDisplacement;
	char m_name[32];

	dFloat32 m_chassisMass;
	ndEngineTorqueCurve m_engine;
	ndGearBox m_transmission;
	ndTireDefinition m_frontTire;
	ndTireDefinition m_rearTire;

	dFloat32 m_frictionCoefficientScale;

	dFloat32 m_motorMass;
	dFloat32 m_motorRadius;

	dFloat32 m_differentialMass;
	dFloat32 m_differentialRadius;
	dFloat32 m_slipDifferentialRmpLock;
	ndDifferentialType m_differentialType;

	dFloat32 m_torsionBarSpringK;
	dFloat32 m_torsionBarDamperC;
	dFloat32 m_torsionBarRegularizer;
	ndTorsionBarType m_torsionBarType;
};

class ndBasicVehicle : public ndMultiBodyVehicle
{
	public:
	D_CLASS_RELECTION(ndBasicVehicle);

	ndBasicVehicle(const ndVehicleDectriptor& desc);
	virtual ~ndBasicVehicle();

	bool IsPlayer() const;
	virtual void SetAsPlayer(ndDemoEntityManager* const scene, bool mode = true);

	protected:
	void ApplyInputs(ndWorld* const world, dFloat32 timestep);
	dFloat32 GetFrictionCoeficient(const ndMultiBodyVehicleTireJoint* const, const ndContactMaterial&) const;

	ndVehicleDectriptor m_configuration;
	dFloat32 m_steerAngle;

	ndDemoEntityManager::ndKeyTrigger m_parking;
	ndDemoEntityManager::ndKeyTrigger m_ignition;
	ndDemoEntityManager::ndKeyTrigger m_neutralGear;
	ndDemoEntityManager::ndKeyTrigger m_reverseGear;
	ndDemoEntityManager::ndKeyTrigger m_forwardGearUp;
	ndDemoEntityManager::ndKeyTrigger m_forwardGearDown;
	ndDemoEntityManager::ndKeyTrigger m_manualTransmission;

	dInt32 m_currentGear;
	dInt32 m_autoGearShiftTimer;
	bool m_isPlayer;
	bool m_isParked;
	bool m_isManualTransmission;
};

class ndVehicleSelector : public ndModel
{
	public:
	ndVehicleSelector();
	void Update(ndWorld* const, dFloat32){}
	void PostUpdate(ndWorld* const world, dFloat32);

	ndDemoEntityManager::ndKeyTrigger m_changeVehicle;
};

#endif