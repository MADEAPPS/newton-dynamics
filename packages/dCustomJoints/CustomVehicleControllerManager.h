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

#ifndef D_CUSTOM_VEHICLE_CONTROLLER_MANAGER_H_
#define D_CUSTOM_VEHICLE_CONTROLLER_MANAGER_H_

#include <CustomJointLibraryStdAfx.h>
#include <CustomAlloc.h>
#include <CustomControllerManager.h>
#include <CustomVehicleControllerComponent.h>
#include <CustomVehicleControllerBodyState.h>

#define VEHICLE_PLUGIN_NAME			"__vehicleManager__"

class CustomVehicleControllerComponent;
class CustomVehicleControllerComponentBrake;
class CustomVehicleControllerComponentEngine;
class CustomVehicleControllerComponentSteering;


class CustomVehicleController: public CustomControllerBase
{
	public:
	class TireList: public dList<CustomVehicleControllerBodyStateTire>
	{
		public:
		TireList()
			:dList<CustomVehicleControllerBodyStateTire>()
		{
		}
	};

/*
	public:
	
	CUSTOM_JOINTS_API TireBodyState* GetFirstTire () const ;
	CUSTOM_JOINTS_API TireBodyState* GetNextTire (TireBodyState* const tire) const;

	CUSTOM_JOINTS_API void* GetUserData (TireBodyState* const tireNode) const;
	CUSTOM_JOINTS_API dMatrix GetTireLocalMatrix (TireBodyState* const tireNode) const;
	CUSTOM_JOINTS_API dMatrix GetTireGlobalMatrix (TireBodyState* const tireNode) const;

	CUSTOM_JOINTS_API const ChassisBodyState& GetChassisState () const;

	CUSTOM_JOINTS_API void SetLongitudinalSlipRatio(dFloat maxLongitudinalSlipRatio);
	CUSTOM_JOINTS_API void SetLateralSlipAngle(dFloat maxLongitudinalSlipAngleIndDegrees);

*/	

	CUSTOM_JOINTS_API CustomVehicleControllerComponentBrake* GetBrakes() const;
	CUSTOM_JOINTS_API CustomVehicleControllerComponentEngine* GetEngine() const;
	CUSTOM_JOINTS_API CustomVehicleControllerComponentBrake* GetHandBrakes() const;
	CUSTOM_JOINTS_API CustomVehicleControllerComponentSteering* GetSteering() const;

	CUSTOM_JOINTS_API void SetCenterOfGravity(const dVector& comRelativeToGeomtriCenter);
	CUSTOM_JOINTS_API CustomVehicleControllerBodyStateTire* AddTire (const CustomVehicleControllerBodyStateTire::TireCreationInfo& tireInfo);

	CUSTOM_JOINTS_API void SetBrakes(CustomVehicleControllerComponentBrake* const brakes);
	CUSTOM_JOINTS_API void SetEngine(CustomVehicleControllerComponentEngine* const engine);
	CUSTOM_JOINTS_API void SetHandBrakes(CustomVehicleControllerComponentBrake* const brakes);
	CUSTOM_JOINTS_API void SetSteering(CustomVehicleControllerComponentSteering* const steering);


	protected:
	CUSTOM_JOINTS_API void Cleanup();
	CUSTOM_JOINTS_API void Init (NewtonCollision* const chassisShape, const dMatrix& vehicleFrame, dFloat mass, const dVector& gravityVector);
	CUSTOM_JOINTS_API virtual void PreUpdate(dFloat timestep, int threadIndex);
	CUSTOM_JOINTS_API virtual void PostUpdate(dFloat timestep, int threadIndex);

	CUSTOM_JOINTS_API int GetActiveJoints(CustomVehicleControllerJoint** const jointArray);
	CUSTOM_JOINTS_API int BuildJacobianMatrix (int jointCount, CustomVehicleControllerJoint** const jointArray, dFloat timestep, CustomVehicleControllerJoint::JacobianPair* const jacobianArray, CustomVehicleControllerJoint::JacobianColum* const jacobianColumnArray);
	CUSTOM_JOINTS_API void CalculateReactionsForces(int jointCount, CustomVehicleControllerJoint** const jointArray, dFloat timestep, CustomVehicleControllerJoint::JacobianPair* const jacobianArray, CustomVehicleControllerJoint::JacobianColum* const jacobianColumnArray);

//	CUSTOM_JOINTS_API void UpdateTireTransforms ();

	CustomVehicleControllerBodyState m_staticWorld;
	CustomVehicleControllerBodyStateEngine m_engineState;
	CustomVehicleControllerBodyStateChassis m_chassisState;
	CustomVehicleControllerComponent::dInterpolationCurve m_tireLateralSlipAngle;
	CustomVehicleControllerComponent::dInterpolationCurve m_tireLongitidialSlipRatio;

	TireList m_tireList;
	dList<CustomVehicleControllerBodyState*> m_stateList;
	NewtonCollision* m_tireCastShape;

	CustomVehicleControllerComponentBrake* m_brakes;
	CustomVehicleControllerComponentEngine* m_engine;
	CustomVehicleControllerComponentBrake* m_handBrakes;
	CustomVehicleControllerComponentSteering* m_steering; 

	friend class CustomVehicleControllerManager;
	friend class CustomVehicleControllerTireJoint;
	
	friend class CustomVehicleControllerContactJoint;
	friend class CustomVehicleControllerEngineIdleJoint;
	friend class CustomVehicleControllerEngineGearJoint;
	
	friend class CustomVehicleControllerBodyStateTire;
	friend class CustomVehicleControllerComponentBrake;
	friend class CustomVehicleControllerComponentEngine;
	friend class CustomVehicleControllerComponentSteering;
	
};


class CustomVehicleControllerManager: public CustomControllerManager<CustomVehicleController> 
{
	public:
	CUSTOM_JOINTS_API CustomVehicleControllerManager(NewtonWorld* const world);
	CUSTOM_JOINTS_API virtual ~CustomVehicleControllerManager();

	CUSTOM_JOINTS_API virtual CustomVehicleController* CreateVehicle (NewtonCollision* const chassisShape, const dMatrix& vehicleFrame, dFloat mass, const dVector& gravityVector);
	CUSTOM_JOINTS_API virtual void DestroyController (CustomVehicleController* const controller);
};


#endif 

