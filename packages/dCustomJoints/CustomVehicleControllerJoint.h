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

#ifndef D_CUSTOM_VEHICLE_CONTROLLER_JOINT_H_
#define D_CUSTOM_VEHICLE_CONTROLLER_JOINT_H_

#include <CustomJointLibraryStdAfx.h>
#include <CustomAlloc.h>


class CustomVehicleController;
class CustomVehicleControllerBodyState;

class CustomVehicleControllerJoint
{
	public:
	class dJacobian
	{
		public:
		dVector m_linear;
		dVector m_angular;
	};

	class dJacobianPair
	{
		public:
		dJacobian m_jacobian_IM0;
		dJacobian m_jacobian_IM1;
	};

	class dJacobianColum
	{
		public:
		dFloat m_force;
		dFloat m_diagDamp;
		dFloat m_deltaAccel;
		dFloat m_invDJMinvJt;
		dFloat m_coordenateAccel;
		dFloat m_jointLowFriction;
		dFloat m_jointHighFriction;
	};

	class dParamInfo
	{
		public:
		dJacobianPair m_jacobians[8];
		dFloat m_jointAccel[8];
		dFloat m_jointLowFriction[8];
		dFloat m_jointHighFriction[8];
		int m_count;
		dFloat m_timestep;
		dFloat m_timestepInv;
	};

	class dPointDerivativeParam
	{
		public:
		dVector m_r0;
		dVector m_posit0;
		dVector m_veloc0;
		dVector m_centripetal0;

		dVector m_r1;
		dVector m_posit1;
		dVector m_veloc1;
		dVector m_centripetal1;
	};

	class dJointAccelerationDecriptor
	{
		public:
		int m_rowsCount;
		dFloat m_timeStep;
		dFloat m_invTimeStep;
		dFloat m_firstPassCoefFlag;
		dJacobianPair* m_rowMatrix;
		dJacobianColum* m_colMatrix;
	};


	CustomVehicleControllerJoint(){}
	virtual ~CustomVehicleControllerJoint(){}

	CUSTOM_JOINTS_API virtual void Init(CustomVehicleController* const controller, CustomVehicleControllerBodyState* const state0, CustomVehicleControllerBodyState* const state1);

	CUSTOM_JOINTS_API virtual void UpdateSolverForces (const dJacobianPair* const jacobians) const = 0; 
	CUSTOM_JOINTS_API virtual void JacobianDerivative (dParamInfo* const constraintParams) = 0; 
	CUSTOM_JOINTS_API virtual void JointAccelerations (dJointAccelerationDecriptor* const accelParam);

	CUSTOM_JOINTS_API void InitPointParam (dPointDerivativeParam& param, const dVector& pivot) const;
	CUSTOM_JOINTS_API void AddAngularRowJacobian (dParamInfo* const constraintParams, const dVector& dir, dFloat jointAngle);
	CUSTOM_JOINTS_API void AddLinearRowJacobian (dParamInfo* const constraintParams, const dVector& pivot, const dVector& dir);
	CUSTOM_JOINTS_API void AddAngularRowJacobian (dParamInfo* const constraintParams, const dVector& dir0, const dVector& dir1, dFloat ratio);
	CUSTOM_JOINTS_API void CalculatePointDerivative (dParamInfo* const constraintParams, const dVector& dir, const dPointDerivativeParam& param);

	int m_rowIsMotor[8];
	dFloat m_motorAcceleration[8];
	dFloat m_jointFeebackForce[8];
	CustomVehicleControllerBodyState* m_state0;
	CustomVehicleControllerBodyState* m_state1;
	CustomVehicleController* m_controller; 
	int m_start;
	int m_count;

	friend class CustomVehicleController;
	friend class CustomVehicleControllerBodyState;
	friend class CustomVehicleControllerTireJoint;
	friend class CustomVehicleControllerBodyStateTire;
	friend class CustomVehicleControllerBodyStateChassis;
	friend class CustomVehicleControllerContactJoint;
	friend class CustomVehicleControllerComponentBrake;
	friend class CustomVehicleControllerComponentEngine;
	friend class CustomVehicleControllerComponentSteering;

};

class CustomVehicleControllerEngineGearJoint: public CustomVehicleControllerJoint
{
	public:
	CUSTOM_JOINTS_API virtual void JacobianDerivative (dParamInfo* const constraintParams); 
	CUSTOM_JOINTS_API virtual void UpdateSolverForces (const dJacobianPair* const jacobians) const;

	dFloat m_powerTrainGain;
};

class CustomVehicleControllerEngineIdleJoint: public CustomVehicleControllerJoint
{
	public:
	CUSTOM_JOINTS_API virtual void JacobianDerivative (dParamInfo* const constraintParams); 
	CUSTOM_JOINTS_API virtual void UpdateSolverForces (const dJacobianPair* const jacobians) const;

	dFloat m_omega;
	dFloat m_friction;
};


class CustomVehicleControllerTireJoint: public CustomVehicleControllerJoint
{
	public:
	CUSTOM_JOINTS_API virtual void JacobianDerivative (dParamInfo* const constraintParams); 
	CUSTOM_JOINTS_API virtual void UpdateSolverForces (const dJacobianPair* const jacobians) const; 
};

class CustomVehicleControllerContactJoint: public CustomVehicleControllerJoint
{
	public:
	CUSTOM_JOINTS_API CustomVehicleControllerContactJoint ();
	CUSTOM_JOINTS_API virtual void UpdateSolverForces (const dJacobianPair* const jacobians) const; 
	CUSTOM_JOINTS_API virtual void JacobianDerivative (dParamInfo* const constraintParams); 
	CUSTOM_JOINTS_API virtual void JointAccelerations (dJointAccelerationDecriptor* const accelParam);

	int m_contactCount;
	NewtonWorldConvexCastReturnInfo m_contacts[4];
};


#endif 

