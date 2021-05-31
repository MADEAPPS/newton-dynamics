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

#ifndef __PHYSICS_UTIL__
#define __PHYSICS_UTIL__

#define DEMO_GRAVITY  dFloat32(-10.0f)
//#define DEMO_GRAVITY  dFloat32(0.0f)

class ndDemoEntityManager;

class nvVehicleDectriptor
{
	public:
	class ndEngineTorqueCurve
	{
		class ndTorqueTap
		{
			public:
			ndTorqueTap() {}
			ndTorqueTap(dFloat32 rpm, dFloat32 torqueInPoundFeet)
				:m_radPerSeconds(rpm * 0.105f)
				, m_torqueInNewtonMeters(torqueInPoundFeet * 1.36f)
			{
			}

			dFloat32 m_radPerSeconds;
			dFloat32 m_torqueInNewtonMeters;
		};

		public:
		ndEngineTorqueCurve()
		{
			// take from the data sheet of a 2005 dodge viper, 
			// some values are missing so I have to improvise them
			dFloat32 fuelInjectionRate = 10.0f;
			dFloat32 idleTorquePoundFoot = 100.0f;
			dFloat32 idleRmp = 800.0f;
			dFloat32 horsePower = 400.0f;
			dFloat32 rpm0 = 5000.0f;
			dFloat32 rpm1 = 6200.0f;
			dFloat32 horsePowerAtRedLine = 100.0f;
			dFloat32 redLineRpm = 8000.0f;
			Init(fuelInjectionRate, idleTorquePoundFoot, idleRmp,
				horsePower, rpm0, rpm1, horsePowerAtRedLine, redLineRpm);
		}

		void Init(dFloat32 fuelInjectionRate,
			dFloat32 idleTorquePoundFoot, dFloat32 idleRmp,
			dFloat32 horsePower, dFloat32 rpm0, dFloat32 rpm1,
			dFloat32 horsePowerAtRedLine, dFloat32 redLineRpm)
		{
			m_fuelInjectionRate = fuelInjectionRate;
			m_torqueCurve[0] = ndTorqueTap(0.0f, idleTorquePoundFoot);
			m_torqueCurve[1] = ndTorqueTap(idleRmp, idleTorquePoundFoot);

			dFloat32 power = horsePower * 746.0f;
			dFloat32 omegaInRadPerSec = rpm0 * 0.105f;
			dFloat32 torqueInPoundFood = (power / omegaInRadPerSec) / 1.36f;
			m_torqueCurve[2] = ndTorqueTap(rpm0, torqueInPoundFood);

			power = horsePower * 746.0f;
			omegaInRadPerSec = rpm1 * 0.105f;
			torqueInPoundFood = (power / omegaInRadPerSec) / 1.36f;
			m_torqueCurve[3] = ndTorqueTap(rpm0, torqueInPoundFood);

			power = horsePowerAtRedLine * 746.0f;
			omegaInRadPerSec = redLineRpm * 0.105f;
			torqueInPoundFood = (power / omegaInRadPerSec) / 1.36f;
			m_torqueCurve[4] = ndTorqueTap(redLineRpm, torqueInPoundFood);
		}

		dFloat32 GetFuelRate() const
		{
			return m_fuelInjectionRate;
		}

		dFloat32 GetIdleRadPerSec() const
		{
			return m_torqueCurve[1].m_radPerSeconds;
		}

		dFloat32 GetLowGearShiftRadPerSec() const
		{
			return m_torqueCurve[2].m_radPerSeconds;
		}

		dFloat32 GetHighGearShiftRadPerSec() const
		{
			return m_torqueCurve[3].m_radPerSeconds;
		}

		dFloat32 GetRedLineRadPerSec() const
		{
			const int maxIndex = sizeof(m_torqueCurve) / sizeof(m_torqueCurve[0]);
			return m_torqueCurve[maxIndex - 1].m_radPerSeconds;
		}

		dFloat32 GetTorque(dFloat32 omegaInRadPerSeconds) const
		{
			const int maxIndex = sizeof(m_torqueCurve) / sizeof(m_torqueCurve[0]);
			omegaInRadPerSeconds = dClamp(omegaInRadPerSeconds, dFloat32(0.0f), m_torqueCurve[maxIndex - 1].m_radPerSeconds);

			for (dInt32 i = 1; i < maxIndex; i++)
			{
				if (omegaInRadPerSeconds <= m_torqueCurve[i].m_radPerSeconds)
				{
					dFloat32 omega0 = m_torqueCurve[i - 0].m_radPerSeconds;
					dFloat32 omega1 = m_torqueCurve[i - 1].m_radPerSeconds;

					dFloat32 torque0 = m_torqueCurve[i - 0].m_torqueInNewtonMeters;
					dFloat32 torque1 = m_torqueCurve[i - 1].m_torqueInNewtonMeters;

					dFloat32 torque = torque0 + (omegaInRadPerSeconds - omega0) * (torque1 - torque0) / (omega1 - omega0);
					return torque;
				}
			}

			return m_torqueCurve[maxIndex - 1].m_torqueInNewtonMeters;
		}

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
		dFloat32 m_crownGearRatio;
		dFloat32 m_torqueConverter;
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
		dFloat32 m_verticalOffset;
		dFloat32 m_laterialStiffeness;
		dFloat32 m_longitudinalStiffeness;
	};

	enum ndDifferentialType
	{
		m_rearWheelDrive,
		m_frontWheelDrive,
		m_fourWheeldrive,
	};

	enum ndTorsionBarType
	{
		m_noWheelAxle,
		m_rearWheelAxle,
		m_frontWheelAxle,
		m_fourWheelAxle,
	};

	nvVehicleDectriptor(const char* const fileName)
		:m_comDisplacement(dVector::m_zero)
	{
		strncpy(m_name, fileName, sizeof(m_name));

		dFloat32 fuelInjectionRate = 10.0f;
		dFloat32 idleTorquePoundFoot = 100.0f;
		dFloat32 idleRmp = 900.0f;
		dFloat32 horsePower = 400.0f;
		dFloat32 rpm0 = 5000.0f;
		dFloat32 rpm1 = 6200.0f;
		dFloat32 horsePowerAtRedLine = 100.0f;
		dFloat32 redLineRpm = 8000.0f;
		m_engine.Init(fuelInjectionRate, idleTorquePoundFoot, idleRmp,
			horsePower, rpm0, rpm1, horsePowerAtRedLine, redLineRpm);

		m_chassisMass = 1000.0f;
		m_transmission.m_gearsCount = 4;
		m_transmission.m_neutral = 0.0f;
		m_transmission.m_crownGearRatio = 10.0f;
		m_transmission.m_reverseRatio = -3.0f;
		m_transmission.m_fowardRatios[0] = 2.25f;
		m_transmission.m_fowardRatios[1] = 1.60f;
		m_transmission.m_fowardRatios[2] = 1.10f;
		m_transmission.m_fowardRatios[3] = 0.80f;
		m_transmission.m_torqueConverter = 1000.0f;

		m_frontTire.m_mass = 20.0f;
		m_frontTire.m_steeringAngle = 35.0f;
		m_frontTire.m_springK = 1000.0f;
		m_frontTire.m_damperC = 20.0f;
		m_frontTire.m_regularizer = 0.1f;
		m_frontTire.m_upperStop = -0.05f;
		m_frontTire.m_lowerStop = 0.2f;
		m_frontTire.m_verticalOffset = 0.0f;
		m_frontTire.m_laterialStiffeness = 100.0f / 1000.0f;
		m_frontTire.m_longitudinalStiffeness = 600.0f / 1000.0f;

		m_rearTire.m_steeringAngle = 0.0f;
		m_rearTire.m_springK = 1000.0f;
		m_rearTire.m_damperC = 20.0f;
		m_rearTire.m_regularizer = 0.1f;
		m_rearTire.m_upperStop = -0.05f;
		m_rearTire.m_lowerStop = 0.2f;
		m_frontTire.m_verticalOffset = 0.0f;
		m_rearTire.m_laterialStiffeness = 100.0f / 1000.0f;
		m_rearTire.m_longitudinalStiffeness = 600.0f / 1000.0f;

		m_brakeTorque = 1500.0f;
		m_handBrakeTorque = 1500.0f;

		m_motorMass = 20.0f;
		m_motorRadius = 0.25f;

		m_differentialMass = 20.0f;
		m_differentialRadius = 0.25f;
		m_frictionCoefficientScale = 1.5f;

		m_torsionBarSpringK = 100.0f;
		m_torsionBarDamperC = 10.0f;
		m_torsionBarRegularizer = 0.15f;
		m_torsionBarType = m_noWheelAxle;

		m_differentialType = m_rearWheelDrive;
	}

	dVector m_comDisplacement;
	char m_name[32];

	dFloat32 m_chassisMass;
	ndEngineTorqueCurve m_engine;
	ndGearBox m_transmission;
	ndTireDefinition m_frontTire;
	ndTireDefinition m_rearTire;

	dFloat32 m_brakeTorque;
	dFloat32 m_handBrakeTorque;
	dFloat32 m_frictionCoefficientScale;

	dFloat32 m_motorMass;
	dFloat32 m_motorRadius;

	dFloat32 m_differentialMass;
	dFloat32 m_differentialRadius;
	ndDifferentialType m_differentialType;

	dFloat32 m_torsionBarSpringK;
	dFloat32 m_torsionBarDamperC;
	dFloat32 m_torsionBarRegularizer;
	ndTorsionBarType m_torsionBarType;
};

dVector FindFloor(const ndWorld& world, const dVector& origin, dFloat32 dist);
ndBodyKinematic* MousePickBody(ndWorld* const nWorld, const dVector& origin, const dVector& end, dFloat32& paramter, dVector& positionOut, dVector& normalOut);

ndBodyKinematic* AddSphere(ndDemoEntityManager* const scene, const dVector& origin, dFloat32 mass, dFloat32 radius);
ndBodyKinematic* AddBox(ndDemoEntityManager* const scene, const dVector& origin, dFloat32 mass, dFloat32 sizex, dFloat32 sizey, dFloat32 sizez);
ndBodyKinematic* AddCapsule(ndDemoEntityManager* const scene, const dVector& origin, dFloat32 mass, dFloat32 radius0, dFloat32 radius1, dFloat32 high);
ndBodyKinematic* AddConvexHull(ndDemoEntityManager* const scene, const dVector& origin, dFloat32 mass, dFloat32 radius, dFloat32 high, dInt32 segments);


void AddPlanks(ndDemoEntityManager* const scene, const dVector& origin);
void AddCapsulesStacks(ndDemoEntityManager* const scene, const dVector& origin, dFloat32 mass, dFloat32 radius0, dFloat32 radius1, dFloat32 high, dInt32 rows_x, dInt32 rows_z, dInt32 columHigh);


#endif