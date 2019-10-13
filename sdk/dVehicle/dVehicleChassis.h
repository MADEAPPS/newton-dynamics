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


#ifndef __D_VEHICLE_CHASSIS_H__
#define __D_VEHICLE_CHASSIS_H__

#include "dStdafxVehicle.h"
#include "dVehicleNode.h"
#include "dVehicleSolver.h"

class dVehicleChassis: public dVehicleNode
{
	public:
#if 0
	class dDriverInput
	{
		public:
		dDriverInput()
			:m_throttle(0.0f)
			,m_brakePedal(0.0f)
			,m_clutchPedal(0.0f)
			,m_steeringValue(0.0f)
			,m_handBrakeValue(0.0f)
			,m_gear(0)
			,m_ignitionKey(0)
			,m_lockDifferential(0)
			,m_manualTransmission(0)
		{
		}

		dFloat m_throttle;
		dFloat m_brakePedal;
		dFloat m_clutchPedal;
		dFloat m_steeringValue;
		dFloat m_handBrakeValue;
		int m_gear;
		int m_ignitionKey;
		int m_lockDifferential;
		int m_manualTransmission;
	};

	private:
	class dCollectCollidingBodies
	{
		public:
		dCollectCollidingBodies(NewtonBody* const me)
			:m_exclude(me)
			,m_count(0)
			,m_staticCount(0)
		{
		}

		NewtonBody* m_exclude;
		int m_count;
		int m_staticCount;
		NewtonBody* m_array[16];
	};
#endif

	public:
	DVEHICLE_API dVehicleChassis (NewtonBody* const rootBody, const dMatrix& bindMatrix, dFloat gravityMag);
	DVEHICLE_API ~dVehicleChassis();

	NewtonBody* GetBody() const { return m_chassisBody; }
#if 0
	
	dVehicleInterface* GetVehicle() const {return m_vehicle;}
	DVEHICLE_API dVehicleTireInterface* AddTire (const dMatrix& locationInGlobalSpace, const dVehicleTireInterface::dTireInfo& tireInfo);
	DVEHICLE_API dVehicleDifferentialInterface* AddDifferential(dVehicleTireInterface* const leftTire, dVehicleTireInterface* const rightTire);
	DVEHICLE_API dVehicleEngineInterface* AddEngine(const dVehicleEngineInterface::dEngineInfo& engineInfo, dVehicleDifferentialInterface* const differential);

	DVEHICLE_API dVehicleBrakeControl* GetBrakeControl();
	DVEHICLE_API dVehicleEngineControl* GetEngineControl();
	DVEHICLE_API dVehicleBrakeControl* GetHandBrakeControl();
	DVEHICLE_API dVehicleSteeringControl* GetSteeringControl ();

	DVEHICLE_API void ApplyDriverInputs(const dDriverInput& driveInputs, dFloat timestep);

	DVEHICLE_API void Finalize();

	void ApplyExternalForces(dFloat timestep);

	private:
	void PreUpdate(dFloat timestep);
	void PostUpdate(dFloat timestep);
	void Debug(dCustomJoint::dDebugDisplay* const debugContext) const;

	void Init(NewtonBody* const body, const dMatrix& localFrame, NewtonApplyForceAndTorque forceAndTorque, dFloat gravityMag);
	void Init(NewtonWorld* const world, NewtonCollision* const chassisShape, dFloat mass, const dMatrix& localFrame, NewtonApplyForceAndTorque forceAndTorque, dFloat gravityMag);
//	void Cleanup();
	
	void CalculateTireContacts(dFloat timestep);
	void CalculateSuspensionForces(dFloat timestep);

	static int OnAABBOverlap(const NewtonBody * const body, void* const me);
	
	dMatrix m_localFrame;
//	dVehicleSolver m_solver;
	dVector m_gravity;
	dVector m_obbSize;
	dVector m_obbOrigin;

	NewtonBody* m_body;
	dVehicleInterface* m_vehicle;
	dVehicleBrakeControl* m_brakeControl;
	dVehicleEngineControl* m_engineControl;
	dVehicleBrakeControl* m_handBrakeControl;
	dVehicleSteeringControl* m_steeringControl;

	friend class dVehicleSolver;
	friend class dVehicleManager;
	friend class dVehicleVirtualTire;
	friend class dVehicleVirtualDifferential;
#endif

	private:
	//void PreUpdate(dFloat timestep);
	//void PostUpdate(dFloat timestep);

	NewtonBody* m_chassisBody;
	void* m_node;
	dVehicleManager* m_manager;

	friend class dVehicleManager;
};


#endif 

