/* Copyright (c) <2009> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/


// NewtonVehicleControllerManager.h: interface for the NewtonVehicleControllerManager class.
//
//////////////////////////////////////////////////////////////////////

#ifndef D_CUSTOM_VEHICLE_CONTROLLER_COMPONENT_H_
#define D_CUSTOM_VEHICLE_CONTROLLER_COMPONENT_H_

#include <CustomJointLibraryStdAfx.h>
#include <CustomAlloc.h>


class CustomVehicleController;
class CustomVehicleControllerBodyState;
class CustomVehicleControllerBodyStateTire;

class CustomVehicleControllerComponent: public CustomAlloc  
{
	public:

	class dInterpolationCurve
	{
		public:
		class dNot
		{
			public:
			dFloat m_param;
			dFloat m_value;
		};

		dInterpolationCurve ()
		{	
			m_count = 0;
		}

		~dInterpolationCurve ()
		{
		}

		CUSTOM_JOINTS_API void InitalizeCurve (int points, const dFloat* const steps, const dFloat* const values);
		CUSTOM_JOINTS_API dFloat GetValue (dFloat param) const;

		dNot m_nodes[6];
		int m_count;
	};

	void SetParam(dFloat param)
	{
		m_param = param;
	}

	dFloat GetParam() const 
	{
		return m_param;
	}

	protected:
	CustomVehicleControllerComponent (CustomVehicleController* const controller)
		:m_controller(controller)
		,m_param(0.0f)
	{
	}

	virtual ~CustomVehicleControllerComponent()
	{
	}

	CUSTOM_JOINTS_API virtual void Update (dFloat timestep) = 0;

	CustomVehicleController* m_controller; 
	dFloat m_param;
};

class CustomVehicleControllerComponentEngine: public CustomVehicleControllerComponent
{
	public:
	class dGearBox
	{
		public:
		enum dGearID
		{
			m_reverseGear = 0,
			m_newtralGear,
			m_firstGear,
			m_maxGears = 16
		};

		class dGearState
		{
			public:
			dGearState (dFloat ratio, dFloat shiftUp, dFloat shiftDown, dGearID id) 
				:m_ratio(ratio)
				,m_shiftUp (shiftUp)
				,m_shiftDown (shiftDown)
				,m_next(NULL)
				,m_prev(NULL)
				,m_id(id)
			{
			}

			virtual ~dGearState()
			{
			}
			virtual dGearState* Update(CustomVehicleController* const vehicle);

			dFloat m_ratio;
			dFloat m_shiftUp;
			dFloat m_shiftDown;
			dGearState* m_next;
			dGearState* m_prev;
			dGearState* m_neutral;
			dGearState* m_reverse;
			dGearID m_id;
		};

		class dReverseGearState: public dGearState
		{
			public:
			dReverseGearState (dFloat ratio)
				:dGearState(ratio, 1000.0f, -1000.0f, m_reverseGear)
			{
			}

			dGearState* Update(CustomVehicleController* const vehicle)
			{
				return NULL;
			}
		};

		class dNeutralGearState: public dGearState
		{
			public:
			dNeutralGearState (dGearState* const first, dGearState* const reverse)
				:dGearState(0.0f, 1000.0f, -1000.0f, m_newtralGear)
			{
				m_next = first;
				m_prev = reverse;
			}
			dGearState* Update(CustomVehicleController* const vehicle);
		};

		CUSTOM_JOINTS_API dGearBox (CustomVehicleController* const controller, dFloat reverseGearRatio, int gearCount, const dFloat* const gearBoxRatios);
		CUSTOM_JOINTS_API ~dGearBox ();
		CUSTOM_JOINTS_API void Update (dFloat timestep);
		CUSTOM_JOINTS_API dFloat GetGearRatio(int gear) const;


		CUSTOM_JOINTS_API int GetGear() const;
		CUSTOM_JOINTS_API void SetGear (int gear);
		CUSTOM_JOINTS_API int GetGearCount() const;

		bool GetTransmissionMode () const;
		void SetTransmissionMode (bool mode);
		void SetOptimalShiftLimits (dFloat minShift, dFloat maxShift);

		dGearState* m_gears[m_maxGears];
		dGearState* m_currentGear;
		CustomVehicleController* m_controller;
		int m_gearsCount;
		bool m_automatic;
	};


	CUSTOM_JOINTS_API CustomVehicleControllerComponentEngine (CustomVehicleController* const controller, dGearBox* const gearBox, CustomVehicleControllerBodyStateTire* const leftTire, CustomVehicleControllerBodyStateTire* const righTire);
	CUSTOM_JOINTS_API ~CustomVehicleControllerComponentEngine();

	CUSTOM_JOINTS_API virtual void Update (dFloat timestep);

	CUSTOM_JOINTS_API void InitEngineTorqueCurve (
		dFloat vehicleSpeedInKilometerPerHours, dFloat engineMomentOfInertia,
		dFloat idleTorqueInPoundFoot, dFloat revolutionsPerMinutesAtIdleTorque, 
		dFloat peakTorqueInPoundFoot, dFloat revolutionsPerMinutesAtPeakTorque, 
		dFloat peakHorsePower, dFloat revolutionsPerMinutesAtPeakHorsePower, 
		dFloat torqueArRedLineInPoundFoot, dFloat revolutionsPerMinutesAtRedLineTorque);

	CUSTOM_JOINTS_API int GetGear () const;
	CUSTOM_JOINTS_API void SetGear (int gear);
	CUSTOM_JOINTS_API dFloat GetRPM () const;
	CUSTOM_JOINTS_API dFloat GetTopRPM () const;
	CUSTOM_JOINTS_API dFloat GetSpeed () const;
	CUSTOM_JOINTS_API dFloat GetTopSpeed () const;
	CUSTOM_JOINTS_API dFloat GetInertia() const;
	CUSTOM_JOINTS_API void SetInertia(dFloat inertia);
	CUSTOM_JOINTS_API bool GetTransmissionMode () const;
	CUSTOM_JOINTS_API void SetTransmissionMode (bool mode);


	dGearBox* GetGearBox() const;
	dFloat GetIdleResistance () const;
	dFloat GetRedLineResistance () const;
	dFloat GetIdleRadianPerSeconds () const;
	dFloat GetDifferencialGearRatio () const;
	dFloat GetTorque (dFloat radianPerSeconds) const;

	void SetTopSpeed (dFloat topSpeedMeterPerSecunds);
	dList<CustomVehicleControllerBodyStateTire>::dListNode* GetLeftTireNode() const;
	dList<CustomVehicleControllerBodyStateTire>::dListNode* GetRightTireNode() const;

	protected:
	dGearBox* m_gearBox;
	dList<CustomVehicleControllerBodyStateTire>::dListNode* m_leftTire;
	dList<CustomVehicleControllerBodyStateTire>::dListNode* m_righTire;
	dInterpolationCurve m_torqueCurve;

	dFloat m_speedMPS;
	dFloat m_topSpeedMPS;
	dFloat m_momentOfInertia;
	dFloat m_engineResistance;
	dFloat m_engineIdleResistance;
	dFloat m_differentialGearRatio;
	dFloat m_radiansPerSecundsAtRedLine;
	dFloat m_radiansPerSecundsAtPeakPower;
	dFloat m_radiansPerSecundsAtIdleTorque;
};

class CustomVehicleControllerComponentBrake: public CustomVehicleControllerComponent
{
	public:
	CUSTOM_JOINTS_API CustomVehicleControllerComponentBrake (CustomVehicleController* const controller, dFloat maxBrakeTorque);
	CUSTOM_JOINTS_API void AddBrakeTire (CustomVehicleControllerBodyStateTire* const tire);
	CUSTOM_JOINTS_API virtual void Update (dFloat timestep);

	dFloat m_maxBrakeTorque;
	dList<dList<CustomVehicleControllerBodyStateTire>::dListNode*> m_brakeTires;
};

class CustomVehicleControllerComponentSteering: public CustomVehicleControllerComponent
{
	public:
	class dTireSignPair
	{
		public:
		dFloat m_sign;
		dList<CustomVehicleControllerBodyStateTire>::dListNode* m_tireNode;
	};

	CUSTOM_JOINTS_API CustomVehicleControllerComponentSteering (CustomVehicleController* const controller, dFloat maxAngleInRadians);
	CUSTOM_JOINTS_API void AddSteeringTire (CustomVehicleControllerBodyStateTire* const tire, dFloat sign);
	CUSTOM_JOINTS_API virtual void Update (dFloat timestep);

	dList<dTireSignPair> m_steeringTires;
	dFloat m_maxAngle;
};


#endif 

