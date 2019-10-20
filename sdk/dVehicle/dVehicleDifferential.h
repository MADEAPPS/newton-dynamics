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


#ifndef __D_VEHICLE_VIRTUAL_DIFFERENTIAL_H__
#define __D_VEHICLE_VIRTUAL_DIFFERENTIAL_H__

#include "dStdafxVehicle.h"
#include "dVehicleNode.h"

class dVehicleChassis;

class dVehicleDifferential: public dVehicleNode, public dComplementaritySolver::dBilateralJoint
{
	public:
	DVEHICLE_API dVehicleDifferential(dVehicleChassis* const chassis, dFloat mass, dFloat radius, dVehicleNode* const leftNode, dVehicleNode* const rightNode);
	DVEHICLE_API virtual ~dVehicleDifferential();
	
	protected:
	void CalculateFreeDof();
	void ApplyExternalForce();
	void Integrate(dFloat timestep);
	//int GetKinematicLoops(dAnimIDRigKinematicLoopJoint** const jointArray);
	//void Debug(dCustomJoint::dDebugDisplay* const debugContext) const;

	dComplementaritySolver::dBilateralJoint* GetJoint() {return this;}
	void JacobianDerivative(dComplementaritySolver::dParamInfo* const constraintParams);
	void UpdateSolverForces(const dComplementaritySolver::dJacobianPair* const jacobians) const;

	dMatrix m_localAxis;
	//dDifferentialMount m_differential;
	//dTireAxleJoint m_leftAxle;
	//dTireAxleJoint m_rightAxle;
	//dVehicleTireInterface* m_leftTire;
	//dVehicleTireInterface* m_rightTire;
	dFloat m_diffOmega;
	dFloat m_shaftOmega;

	friend class dVehicleChassis;
};


#endif 

