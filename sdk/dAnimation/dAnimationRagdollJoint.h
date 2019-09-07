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

#ifndef __D_ANIMATION_RAGDOLL_JOINT_H__
#define __D_ANIMATION_RAGDOLL_JOINT_H__

#include "dAnimationStdAfx.h"
#include "dAnimationJoint.h"

#define D_TEST_JOINT

class dAnimationRagdollJoint: public dAnimationJoint, public dAnimationContraint
{
	class dRagdollMotor;
	class dRagdollMotor_0dof;
	class dRagdollMotor_1dof;
	class dRagdollMotor_2dof;
	class dRagdollMotor_3dof;

	public:
	enum dRagdollMotorType
	{
		m_zeroDof,
		m_oneDof,
		m_twoDof,
		m_threeDof,
	};

	dAnimationRagdollJoint(dRagdollMotorType type, const dMatrix& pinAndPivotInGlobalSpace, NewtonBody* const body, const dMatrix& bindMarix, dAnimationJoint* const parent);
	virtual ~dAnimationRagdollJoint();

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

