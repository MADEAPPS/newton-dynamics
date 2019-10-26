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


#ifndef __D_VEHICLE_MANAGER_H__
#define __D_VEHICLE_MANAGER_H__

#include "dStdafxVehicle.h"


#define D_VEHICLE_MANAGER_NAME			"__VehicleManager__"

class dVehicle;
class dVehicleNode;

class dVehicleManager: public dCustomParallelListener
{
	public:
	DVEHICLE_API dVehicleManager(NewtonWorld* const world);
	DVEHICLE_API virtual ~dVehicleManager();

	DVEHICLE_API void AddRoot(dVehicle* const root);
	DVEHICLE_API void RemoveRoot(dVehicle* const root);
	DVEHICLE_API void RemoveAndDeleteRoot(dVehicle* const root);
	DVEHICLE_API virtual void UpdateDriverInput(dVehicle* const vehicle, dFloat timestep) {}

	virtual void OnUpdateTransform(const dVehicle* const vehicle) const {}
	virtual void OnPreUpdate(dVehicle* const model, dFloat timestep) const {};
	virtual void OnPostUpdate(dVehicle* const model, dFloat timestep) const {};
	virtual void OnDebug(dVehicle* const model, dCustomJoint::dDebugDisplay* const debugContext) {}

	protected:
	DVEHICLE_API void PostStep(dFloat timestep, int threadID);
	DVEHICLE_API void PreUpdate(dFloat timestep, int threadID);
	DVEHICLE_API void PostUpdate(dFloat timestep, int threadID);
	DVEHICLE_API void OnDebug(dCustomJoint::dDebugDisplay* const debugContext);

	dList<dVehicle*> m_list;
};



#endif 

