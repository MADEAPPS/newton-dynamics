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
	friend class CustomVehicleControllerContactJoint;
	friend class CustomVehicleControllerEngineGearJoint;
	friend class CustomVehicleControllerEngineIdleJoint;
	friend class CustomVehicleControllerBodyStateTire;
	friend class CustomVehicleControllerBodyStateChassis;
	friend class CustomVehicleControllerBodyStateEngine;
	friend class CustomVehicleControllerComponentBrake;
	friend class CustomVehicleControllerComponentEngine;
	friend class CustomVehicleControllerComponentSteering;
};


class CustomVehicleControllerBodyStateChassis: public CustomVehicleControllerBodyState
{
	public:
	CUSTOM_JOINTS_API dFloat GetAerodynamicsDowforceCoeficient () const;
	CUSTOM_JOINTS_API void SetAerodynamicsDownforceCoefficient (dFloat maxDownforceInGravities, dFloat topSpeed);

	CUSTOM_JOINTS_API void SetDryRollingFrictionTorque (dFloat torque);
	CUSTOM_JOINTS_API dFloat GetDryRollingFrictionTorque () const;

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
	friend class CustomVehicleControllerContactJoint;
	friend class CustomVehicleControllerBodyStateTire;
	friend class CustomVehicleControllerBodyStateEngine;
	friend class CustomVehicleControllerComponentBrake;
	friend class CustomVehicleControllerComponentEngine;
	friend class CustomVehicleControllerComponentSteering;
};


class CustomVehicleControllerBodyStateEngine: public CustomVehicleControllerBodyState
{
	public:
	CUSTOM_JOINTS_API void Init (CustomVehicleController* const controller);

	CUSTOM_JOINTS_API void Update (dFloat timestep, CustomVehicleController* const controller);
	CUSTOM_JOINTS_API void ApplyNetForceAndTorque (dFloat invTimestep, const dVector& veloc, const dVector& omega);
	CUSTOM_JOINTS_API int CalculateActiveJoints (CustomVehicleController* const controller, CustomVehicleControllerJoint** const jointArray);

	CustomVehicleControllerEngineGearJoint m_leftTire;
	CustomVehicleControllerEngineGearJoint m_rightTire;
	CustomVehicleControllerEngineIdleJoint m_idleFriction;
	dFloat m_radianPerSecund;

	friend class CustomVehicleController;
	friend class CustomVehicleControllerTireJoint;
	friend class CustomVehicleControllerEngineGearJoint;
	friend class CustomVehicleControllerEngineIdleJoint;
	friend class CustomVehicleControllerBodyStateTire;
	friend class CustomVehicleControllerBodyStateChassis;
	friend class CustomVehicleControllerContactJoint;
	friend class CustomVehicleControllerComponentBrake;
	friend class CustomVehicleControllerComponentEngine;
	friend class CustomVehicleControllerComponentSteering;
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

	CUSTOM_JOINTS_API void* GetUserData() const;
	CUSTOM_JOINTS_API dMatrix CalculateLocalMatrix () const;
	CUSTOM_JOINTS_API dMatrix CalculateGlobalMatrix () const;

	CUSTOM_JOINTS_API dMatrix CalculateSteeringMatrix () const;
	CUSTOM_JOINTS_API dMatrix CalculateSuspensionMatrix () const;

	const dVector& GetTireLoad () const
	{
		return m_tireLoad;
	}

	const dVector& GetLateralForce () const
	{
		return m_lateralForce;
	}

	const dVector& GetLongitudinalForce () const
	{
		return m_longitudinalForce;
	}


	private:
	CUSTOM_JOINTS_API void UpdateDynamicInputs(dFloat timestep);
	CUSTOM_JOINTS_API void Collide (CustomControllerConvexCastPreFilter& filter, dFloat timestepInv);
	CUSTOM_JOINTS_API void UpdateTransform ();
/*
	CUSTOM_JOINTS_API dFloat GetAdhesionCoefficient() const;
	CUSTOM_JOINTS_API void SetAdhesionCoefficient(dFloat Coefficient);
	CUSTOM_JOINTS_API virtual void IntegrateForce (dFloat timestep, const dVector& force, const dVector& torque);
*/
	CUSTOM_JOINTS_API virtual void ApplyNetForceAndTorque (dFloat invTimestep, const dVector& veloc, const dVector& omega);

	dVector m_tireLoad;
	dVector m_lateralForce;
	dVector m_longitudinalForce;
	dFloat m_radio;
	dFloat m_width;
	dFloat m_posit;
	dFloat m_speed;
	
	dFloat m_breakTorque;
	dFloat m_engineTorque;
	dFloat m_rotatonSpeed;
	dFloat m_rotationAngle;
	dFloat m_steeringAngle;
	dFloat m_restSprunMass;
	dFloat m_dampingRatio;
	dFloat m_springStrength;
	dFloat m_suspensionlenght;
	dFloat m_lateralStiffness;
	dFloat m_longitudialStiffness;
	dFloat m_aligningMomentTrail;
	
	//dFloat m_adhesionCoefficient; 
	//dFloat m_idleRollingResistance;
	//dFloat m_dryFrictionTorque;
	//dFloat m_engineTorqueResistance;

	void* m_userData;
	NewtonCollision* m_shape;
	CustomVehicleControllerTireJoint m_chassisJoint;
	CustomVehicleControllerContactJoint m_contactJoint;

	friend class CustomVehicleController;
	friend class CustomVehicleControllerTireJoint;
	friend class CustomVehicleControllerContactJoint;
	friend class CustomVehicleControllerEngineGearJoint;
	friend class CustomVehicleControllerEngineIdleJoint;
	friend class CustomVehicleControllerComponentBrake;
	friend class CustomVehicleControllerComponentEngine;
	friend class CustomVehicleControllerComponentSteering;
	friend class CustomVehicleControllerBodyStateEngine;
	friend class CustomVehicleControllerBodyStateChassis;
};



#endif 

