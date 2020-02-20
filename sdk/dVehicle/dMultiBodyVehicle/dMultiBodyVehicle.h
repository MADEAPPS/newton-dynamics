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


#ifndef __D_VEHICLE_MULTIBODY_H__
#define __D_VEHICLE_MULTIBODY_H__

#include "dStdafxVehicle.h"
#include "dVehicle.h"
#include "dVehicleNode.h"
#include "dVehicleSolver.h"
#include "dMultiBodyVehicleDashControl.h"
#include "dMultiBodyVehicleCollisionNode.h"

class dTireInfo;
class dEngineInfo;
class dVehicleLoopJoint;
class dMultiBodyVehicleTire;
class dMultiBodyVehicleDifferential;


class dCollectCollidingBodies
{
	public:
	dCollectCollidingBodies(NewtonBody* const me)
		:m_exclude(me)
		,m_count(0)
	{
	}

	NewtonBody* m_exclude;
	int m_count;
	NewtonBody* m_array[16];
};

class dMultiBodyVehicle: public dVehicle, public dVehicleSolver
{
	public:
	class dDriverInput
	{
		public:
		dDriverInput()
			:m_throttle(0.0f)
			, m_brakePedal(0.0f)
			, m_clutchPedal(0.0f)
			, m_steeringValue(0.0f)
			, m_handBrakeValue(0.0f)
			, m_gear(0)
			, m_ignitionKey(0)
			, m_differentialMode(0)
			, m_manualTransmission(0)
		{
		}

		dFloat m_throttle;
		dFloat m_brakePedal;
		dFloat m_clutchPedal;
		dFloat m_steeringValue;
		dFloat m_handBrakeValue;
		int m_gear;
		int m_ignitionKey;
		int m_differentialMode;
		int m_manualTransmission;
	};

	DVEHICLE_API dMultiBodyVehicle (NewtonBody* const rootBody, const dMatrix& localFrame, dFloat gravityMag);
	DVEHICLE_API ~dMultiBodyVehicle();

	DVEHICLE_API void Finalize();
	DVEHICLE_API const void Debug(dCustomJoint::dDebugDisplay* const debugContext) const;
	DVEHICLE_API dMultiBodyVehicleTire* AddTire(const dMatrix& locationInGlobalSpace, const dTireInfo& tireInfo);
	DVEHICLE_API dMultiBodyVehicleEngine* AddEngine(const dEngineInfo& engineInfo, dMultiBodyVehicleDifferential* const differential);
	DVEHICLE_API dMultiBodyVehicleDifferential* AddDifferential(dFloat mass, dFloat radius, dMultiBodyVehicleTire* const leftTire, dMultiBodyVehicleTire* const rightTire);
	DVEHICLE_API dMultiBodyVehicleDifferential* AddDifferential(dFloat mass, dFloat radius, dMultiBodyVehicleDifferential* const differential0, dMultiBodyVehicleDifferential* const differential1);

	DVEHICLE_API dVehicleBrakeControl* GetBrakeControl();
	DVEHICLE_API dVehicleEngineControl* GetEngineControl();
	DVEHICLE_API dVehicleBrakeControl* GetHandBrakeControl();
	DVEHICLE_API dVehicleSteeringControl* GetSteeringControl();

	dMultiBodyVehicle* GetAsVehicleMultiBody() const { return (dMultiBodyVehicle*)this; }
	DVEHICLE_API virtual void ApplyDriverInputs(const dDriverInput& driveInputs, dFloat timestep);

	private:
	void ApplyExternalForce();
	void Integrate(dFloat timestep);
	void CalculateFreeDof(dFloat timestep);
	void CalculateTireContacts(dFloat timestep);
	virtual int GetKinematicLoops(dVehicleLoopJoint** const jointArray);

	virtual bool CheckSleeping();
	dMultiBodyVehicleCollisionNode* FindCollideNode(dVehicleNode* const node0, NewtonBody* const body);

	void PreUpdate(dFloat timestep);
	//void PostUpdate(dFloat timestep);

	static int OnAABBOverlap(const NewtonBody * const body, void* const me);

	dArray<dMultiBodyVehicleCollisionNode> m_collidingNodes;
	dVehicleBrakeControl m_brakeControl;
	dVehicleEngineControl m_engineControl;
	dVehicleBrakeControl m_handBrakeControl;
	dVehicleSteeringControl m_steeringControl;
	int m_collidingIndex;
	int m_sleepCounter;

	friend class dMultiBodyVehicleTire;
	friend class dVehicleSolver;
	friend class dVehicleManager;
};

#endif 

