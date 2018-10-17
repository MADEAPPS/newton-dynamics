/* Copyright (c) <2003-2016> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/


#ifndef __D_VEHICLE_VIRTUAL_JOINTS_H__
#define __D_VEHICLE_VIRTUAL_JOINTS_H__

#include "dStdafxVehicle.h"

class dVehicleNode;
class dVehicleVirtualTire;

#define D_TIRE_MAX_ELASTIC_DEFORMATION		(0.05f)
#define D_TIRE_MAX_ELASTIC_NORMAL_STIFFNESS (10.0f / D_TIRE_MAX_ELASTIC_DEFORMATION)


class dKinematicLoopJoint : public dComplementaritySolver::dBilateralJoint
{
	public:
	dKinematicLoopJoint();
	~dKinematicLoopJoint(){}
	bool IsActive() const { return m_isActive; }
	dVehicleNode* GetOwner0() const{return m_owner0;}
	dVehicleNode* GetOwner1() const{return m_owner1;}
	void SetOwners(dVehicleNode* const owner0, dVehicleNode* const owner1);

	virtual int GetMaxDof() const = 0;

	dVehicleNode* m_owner0;
	dVehicleNode* m_owner1;
	bool m_isActive;
};


class dTireJoint: public dComplementaritySolver::dBilateralJoint
{
	public:
	dTireJoint();
	void JacobianDerivative(dComplementaritySolver::dParamInfo* const constraintParams);
	void UpdateSolverForces(const dComplementaritySolver::dJacobianPair* const jacobians) const{dAssert (0);}

	dVehicleVirtualTire* m_tire;
};

class dTireContact: public dKinematicLoopJoint
{
	public:
	dTireContact();
	void JacobianDerivative(dComplementaritySolver::dParamInfo* const constraintParams);
	void UpdateSolverForces(const dComplementaritySolver::dJacobianPair* const jacobians) const { dAssert(0); }
	void SetContact (const dMatrix& contact, dFloat penetration);
	int GetMaxDof() const { return 3;}

	dMatrix m_contact;
	dFloat m_penetration;
};



#endif 

