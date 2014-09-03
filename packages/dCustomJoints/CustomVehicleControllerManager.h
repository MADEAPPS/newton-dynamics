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
	class dTireForceSolverSolver;
	class dWeightDistibutionSolver;
	class dTireList: public dList<CustomVehicleControllerBodyStateTire>
	{
		public:
		dTireList()
			:dList<CustomVehicleControllerBodyStateTire>()
		{
		}
	};


	public:
	CUSTOM_JOINTS_API CustomVehicleControllerBodyStateTire* GetFirstTire () const ;
	CUSTOM_JOINTS_API CustomVehicleControllerBodyStateTire* GetNextTire (CustomVehicleControllerBodyStateTire* const tire) const;

	CUSTOM_JOINTS_API const CustomVehicleControllerBodyStateChassis& GetChassisState () const;

	CUSTOM_JOINTS_API dFloat GetAerodynamicsDowforceCoeficient () const;
	CUSTOM_JOINTS_API void SetAerodynamicsDownforceCoefficient (dFloat maxDownforceInGravities, dFloat topSpeed);

	CUSTOM_JOINTS_API void SetDryRollingFrictionTorque (dFloat torque);
	CUSTOM_JOINTS_API dFloat GetDryRollingFrictionTorque () const;

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


	CUSTOM_JOINTS_API void Finalize();


	protected:
	CUSTOM_JOINTS_API void Cleanup();
	CUSTOM_JOINTS_API void Init (NewtonCollision* const chassisShape, const dMatrix& vehicleFrame, dFloat mass, const dVector& gravityVector);
	CUSTOM_JOINTS_API virtual void PreUpdate(dFloat timestep, int threadIndex);
	CUSTOM_JOINTS_API virtual void PostUpdate(dFloat timestep, int threadIndex);

//	CustomVehicleControllerBodyState m_staticWorld;
	CustomVehicleControllerBodyStateChassis m_chassisState;

	dTireList m_tireList;
	dList<CustomVehicleControllerBodyState*> m_stateList;
	NewtonCollision* m_tireCastShape;
	CustomVehicleControllerComponentBrake* m_brakes;
	CustomVehicleControllerComponentEngine* m_engine;
	CustomVehicleControllerComponentBrake* m_handBrakes;
	CustomVehicleControllerComponentSteering* m_steering; 
	bool m_finalized;

	friend class CustomVehicleControllerManager;
	friend class CustomVehicleControllerTireJoint;
	
	friend class CustomVehicleControllerContactJoint;
	friend class CustomVehicleControllerEngineIdleJoint;
	friend class CustomVehicleControllerEngineDifferencialJoint;
	
	friend class CustomVehicleControllerBodyStateTire;
	friend class CustomVehicleControllerBodyStateEngine;
	friend class CustomVehicleControllerBodyStateChassis;

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

