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

#ifndef D_CUSTOM_VEHICLE_CONTROLLER_BODY_STATES_H_
#define D_CUSTOM_VEHICLE_CONTROLLER_BODY_STATES_H_

#include <CustomJointLibraryStdAfx.h>
#include <CustomAlloc.h>
#include <CustomVehicleControllerJoint.h>

class CustomVehicleController;

class CustomVehicleControllerBodyState: public dComplemtaritySolver::dBodyState
{
	public:
	CUSTOM_JOINTS_API dFloat GetMass () const;
	CUSTOM_JOINTS_API const dMatrix& GetMatrix () const;
	CUSTOM_JOINTS_API const dMatrix& GetLocalMatrix () const;
	CUSTOM_JOINTS_API const dVector& GetCenterOfMass () const;

	CUSTOM_JOINTS_API CustomVehicleController* GetController() const;

	protected:
	CUSTOM_JOINTS_API CustomVehicleControllerBodyState();
	CUSTOM_JOINTS_API virtual ~CustomVehicleControllerBodyState() {}

	CUSTOM_JOINTS_API void UpdateInertia();
	CUSTOM_JOINTS_API void Init(CustomVehicleController* const controller);
	CUSTOM_JOINTS_API virtual void IntegrateForce (dFloat timestep, const dVector& force, const dVector& torque);
	CUSTOM_JOINTS_API virtual void ApplyNetForceAndTorque (dFloat invTimestep, const dVector& veloc, const dVector& omega);

	CustomVehicleController* m_controller;

	friend class CustomVehicleController;
	friend class CustomVehicleControllerJoint;
	friend class CustomVehicleControllerTireJoint;
	friend class CustomVehicleControllerBodyStateTire;
	friend class CustomVehicleControllerComponentBrake;
	friend class CustomVehicleControllerEngineIdleJoint;
	friend class CustomVehicleControllerComponentEngine;
	friend class CustomVehicleControllerBodyStateChassis;
	friend class CustomVehicleControllerTireContactJoint;
	friend class CustomVehicleControllerComponentSteering;
	friend class CustomVehicleControllerEngineDifferencialJoint;
};



class CustomVehicleControllerBodyStateContact: public CustomVehicleControllerBodyState
{
	public:
	CUSTOM_JOINTS_API void Init (CustomVehicleController* const controller, const NewtonBody* const body);
	CUSTOM_JOINTS_API const NewtonBody* GetNewtonBody() const;

	private:
	CUSTOM_JOINTS_API virtual void ApplyNetForceAndTorque (dFloat invTimestep, const dVector& veloc, const dVector& omega);
	CUSTOM_JOINTS_API void UpdateDynamicInputs();

	NewtonBody* m_newtonBody;
	dFloat m_maxExternalAccel2;
	dFloat m_maxExternalAlpha2;

	friend class CustomVehicleController;
	friend class CustomVehicleControllerTireJoint;
	friend class CustomVehicleControllerBodyStateTire;
	friend class CustomVehicleControllerComponentBrake;
	friend class CustomVehicleControllerComponentEngine;
	friend class CustomVehicleControllerTireContactJoint;
	friend class CustomVehicleControllerComponentSteering;
};


class CustomVehicleControllerBodyStateChassis: public CustomVehicleControllerBodyState
{
	public:
	CUSTOM_JOINTS_API dFloat GetAerodynamicsDowforceCoeficient () const;
	CUSTOM_JOINTS_API void SetAerodynamicsDownforceCoefficient (dFloat maxDownforceInGravities, dFloat topSpeed);

	CUSTOM_JOINTS_API void SetDryRollingFrictionTorque (dFloat torque);
	CUSTOM_JOINTS_API dFloat GetDryRollingFrictionTorque () const;

	void PutToSleep(); 
	bool IsSleeping () const;
	

	private:
	CUSTOM_JOINTS_API void Init (CustomVehicleController* const controller, const dMatrix& localframe);
	CUSTOM_JOINTS_API void UpdateDynamicInputs();
	CUSTOM_JOINTS_API virtual void ApplyNetForceAndTorque (dFloat invTimestep, const dVector& veloc, const dVector& omega);

	dVector m_com;
	dVector m_comOffset;
	dVector m_gravity;

	dFloat m_gravityMag;
	dFloat m_dryRollingFrictionTorque;
	dFloat m_aerodynamicsDownForceCoefficient;

	friend class CustomVehicleController;
	friend class CustomVehicleControllerTireJoint;
	friend class CustomVehicleControllerTireContactJoint;
	friend class CustomVehicleControllerBodyStateTire;
	friend class CustomVehicleControllerComponentBrake;
	friend class CustomVehicleControllerComponentEngine;
	friend class CustomVehicleControllerComponentSteering;
	friend class CustomVehicleControllerComponentTrackSkidSteering;
};


class CustomVehicleControllerBodyStateTire: public CustomVehicleControllerBodyState
{
	public:
	class TireCreationInfo
	{
		public:
		dVector m_location;
		dFloat m_mass;
		dFloat m_radio;
		dFloat m_width;
		dFloat m_dampingRatio;
		dFloat m_springStrength;
		dFloat m_suspesionlenght;
		dFloat m_lateralStiffness;
		dFloat m_longitudialStiffness;
		dFloat m_aligningMomentTrail;
		void* m_userData;
	};

	CUSTOM_JOINTS_API void Init (CustomVehicleController* const controller, const TireCreationInfo& tireInfo);

	CUSTOM_JOINTS_API void GetInfo(TireCreationInfo& tireInfo) const;
	CUSTOM_JOINTS_API void UpdateInfo(const TireCreationInfo& tireInfo);

	CUSTOM_JOINTS_API void* GetUserData() const;
	CUSTOM_JOINTS_API dMatrix CalculateLocalMatrix () const;
	CUSTOM_JOINTS_API dMatrix CalculateGlobalMatrix () const;

	CUSTOM_JOINTS_API dFloat GetTireRadio () const
	{
		return m_radio;
	}

	CUSTOM_JOINTS_API dFloat GetTireRotationSpeed () const
	{
		return m_rotationalSpeed;
	}

	CUSTOM_JOINTS_API const dVector& GetTireLoad () const
	{
		return m_tireLoad;
	}

	CUSTOM_JOINTS_API const dVector& GetLateralForce () const
	{
		return m_lateralForce;
	}

	CUSTOM_JOINTS_API const dVector& GetLongitudinalForce () const
	{
		return m_longitudinalForce;
	}

	CUSTOM_JOINTS_API dFloat GetLateralSlip () const
	{
		return m_lateralSlip;
	}

	CUSTOM_JOINTS_API dFloat GetLongitudinalSlip () const
	{
		return m_longitudinalSlip;
	}

	CUSTOM_JOINTS_API dFloat GetAligningTorque () const
	{
		return m_aligningTorque;
	}
	
	CUSTOM_JOINTS_API int GetContactCount() const;
	CUSTOM_JOINTS_API const CustomVehicleControllerBodyStateContact* GetContactBody(int index) const;
	CUSTOM_JOINTS_API const CustomVehicleControllerTireContactJoint* GetContactJoint(int index) const;


	private:
	CUSTOM_JOINTS_API void UpdateTransform ();
	CUSTOM_JOINTS_API dMatrix CalculateSteeringMatrix () const;
	CUSTOM_JOINTS_API dMatrix CalculateSuspensionMatrix () const;
	CUSTOM_JOINTS_API void UpdateDynamicInputs(dFloat timestep);
	CUSTOM_JOINTS_API void Collide (CustomControllerConvexCastPreFilter& filter, dFloat timestepInv, int threadId);
	CUSTOM_JOINTS_API virtual void ApplyNetForceAndTorque (dFloat invTimestep, const dVector& veloc, const dVector& omega);
	CUSTOM_JOINTS_API void CalculateRollingResistance (dFloat topSpeed);


	dVector m_tireLoad;
	dVector m_lateralForce;
	dVector m_longitudinalForce;
	CustomVehicleControllerTireJoint m_chassisJoint;
	CustomVehicleControllerTireContactJoint m_contactJoint[2];
	int m_contactCount;

	dFloat m_radio;
	dFloat m_width;
	dFloat m_posit;
	dFloat m_speed;
	dFloat m_locationSign;
	dFloat m_brakeTorque;
	dFloat m_engineTorque;
	dFloat m_rotationalSpeed;
	dFloat m_dampingRatio;
	dFloat m_rotationAngle;
	dFloat m_steeringAngle;
	dFloat m_restSprunMass;
	dFloat m_springStrength;
	dFloat m_suspensionLenght;
	dFloat m_lateralStiffness;
	dFloat m_maxAngularVelocity;
	dFloat m_aligningMomentTrail;
	dFloat m_longitudialStiffness;

	dFloat m_lateralSlip;
	dFloat m_longitudinalSlip;
	dFloat m_aligningTorque;
	
	void* m_userData;
	NewtonCollision* m_shape;

	friend class CustomVehicleController;
	friend class CustomVehicleControllerTireJoint;
	friend class CustomVehicleControllerComponentBrake;
	friend class CustomVehicleControllerComponentEngine;
	friend class CustomVehicleControllerEngineIdleJoint;
	friend class CustomVehicleControllerTireContactJoint;
	friend class CustomVehicleControllerBodyStateChassis;
	friend class CustomVehicleControllerComponentSteering;
	friend class CustomVehicleControllerEngineDifferencialJoint;
};



#endif 

