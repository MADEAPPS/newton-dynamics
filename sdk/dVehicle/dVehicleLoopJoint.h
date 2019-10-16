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

#ifndef __D_VEHICLE_LOOP_JOINT_H__
#define __D_VEHICLE_LOOP_JOINT_H__

#include "dStdafxVehicle.h"

class dVehicleNode;

class dVehicleLoopJoint: public dComplementaritySolver::dBilateralJoint
{
	public:
	dVehicleLoopJoint();
	virtual ~dVehicleLoopJoint() {}
	bool IsActive() const { return m_isActive; }
	void SetOwners(dVehicleNode* const owner0, dVehicleNode* const owner1);

	dVehicleNode* GetOwner0() const { return (dVehicleNode*)m_state0;}
	dVehicleNode* GetOwner1() const { return (dVehicleNode*)m_state1;}

	virtual void Debug(dCustomJoint::dDebugDisplay* const debugDisplay) const {}
	virtual int GetMaxDof() const = 0;

	dVehicleNode* m_owner0;
	dVehicleNode* m_owner1;
	bool m_isActive;
};

#endif

