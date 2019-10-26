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
#include "dVehicleDashControl.h"

class dTireInfo;
class dEngineInfo;
class dVehicleTire;
class dVehicleLoopJoint;
class dVehicleDifferential;

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

class dVehicleMultiBody: public dVehicle, public dVehicleSolver
{
	public:
	DVEHICLE_API dVehicleMultiBody (NewtonBody* const rootBody, const dMatrix& localFrame, dFloat gravityMag);
	DVEHICLE_API ~dVehicleMultiBody();

	dVehicleMultiBody* GetAsVehicleMultiBody() const { return (dVehicleMultiBody*)this; }

	DVEHICLE_API void Finalize();
	DVEHICLE_API const void Debug(dCustomJoint::dDebugDisplay* const debugContext) const;
	DVEHICLE_API dVehicleTire* AddTire(const dMatrix& locationInGlobalSpace, const dTireInfo& tireInfo);
	DVEHICLE_API dVehicleEngine* AddEngine(const dEngineInfo& engineInfo, dVehicleDifferential* const differential);
	DVEHICLE_API dVehicleDifferential* AddDifferential(dFloat mass, dFloat radius, dVehicleTire* const leftTire, dVehicleTire* const rightTire);

	DVEHICLE_API dVehicleBrakeControl* GetBrakeControl();
	DVEHICLE_API dVehicleEngineControl* GetEngineControl();
	DVEHICLE_API dVehicleBrakeControl* GetHandBrakeControl();
	DVEHICLE_API dVehicleSteeringControl* GetSteeringControl();

	private:
	void CalculateFreeDof();
	void ApplyExternalForce();
	void Integrate(dFloat timestep);
	void CalculateTireContacts(dFloat timestep);
	void CalculateSuspensionForces(dFloat timestep);
	virtual int GetKinematicLoops(dVehicleLoopJoint** const jointArray);
	virtual void ApplyDriverInputs(const dDriverInput& driveInputs, dFloat timestep);

	void PreUpdate(dFloat timestep);
	//void PostUpdate(dFloat timestep);

	static int OnAABBOverlap(const NewtonBody * const body, void* const me);

	dVehicleNode m_groundProxyBody;
	dVehicleBrakeControl m_brakeControl;
	dVehicleEngineControl m_engineControl;
	dVehicleBrakeControl m_handBrakeControl;
	dVehicleSteeringControl m_steeringControl;

	friend class dVehicleTire;
	friend class dVehicleSolver;
	friend class dVehicleManager;
};

#endif 

