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

#ifndef __D_ANIMATION_JOINT_RAGDOLL_H__
#define __D_ANIMATION_JOINT_RAGDOLL_H__

#include "dAnimationStdAfx.h"
#include "dAnimationJoint.h"



class dAnimationJointRagdoll: public dAnimationJoint, public dAnimationContraint
{
	class dRagdollMotor;
	class dRagdollMotor_1dof;
	class dRagdollMotor_2dof;
	class dRagdollMotor_3dof;

	public:
	enum dRagdollMotorType
	{
		m_oneDof,
		m_twoDof,
		m_threeDof,
	};

	dAnimationJointRagdoll(dRagdollMotorType type, const dMatrix& pinAndPivotInGlobalSpace, NewtonBody* const body, const dMatrix& bindMarix, dAnimationJoint* const parent);
	virtual ~dAnimationJointRagdoll();

	protected:
	virtual void RigidBodyToStates();
	virtual void UpdateJointAcceleration();
	
	virtual void JacobianDerivative(dComplementaritySolver::dParamInfo* const constraintParams);
	virtual void UpdateSolverForces(const dComplementaritySolver::dJacobianPair* const jacobians) const;

	dComplementaritySolver::dJacobian m_jacobial01[3];
	dComplementaritySolver::dJacobian m_jacobial10[3];
	dVector m_rowAccel;
	int m_rows;
};



#endif

