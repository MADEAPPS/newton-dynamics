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

#include "dAnimationStdAfx.h"
#include "dAnimationRigHinge.h"
#include "dAnimationCharacterRigManager.h"

dAnimationRigHinge::dAnimationRigHinge(const dMatrix& basicMatrix, dAnimationRigJoint* const parent, NewtonBody* const body)
	:dAnimationRigLimb(parent, body)
	,dCustomHinge (basicMatrix, body, parent->GetNewtonBody())
	,m_rowAccel(0.0f)
{
//	EnableMotor(true, 0.0f);
	EnableLimits(true);
}

dAnimationRigHinge::~dAnimationRigHinge()
{
}

void dAnimationRigHinge::SubmitConstraints (dFloat timestep, int threadIndex)
{
	dCustomHinge::SubmitConstraints (timestep, threadIndex);

//	dFloat angle = GetJointAngle();
//	dFloat speed = GetJointOmega();
	if (!m_limitReached) {
		NewtonJoint* const joint = dCustomHinge::GetJoint();
		NewtonUserJointSetRowAcceleration(joint, m_rowAccel);
	}
}

void dAnimationRigHinge::JacobianDerivative(dComplementaritySolver::dParamInfo* const constraintParams)
{
	m_rowAccel = 0.0f;
	NewtonImmediateModeConstraint descriptor;
	NewtonJoint* const newtonJoint = dCustomHinge::GetJoint();
	int rows = NewtonUserJointSubmitImmediateModeConstraint(newtonJoint, &descriptor, constraintParams->m_timestep);
	dAssert (rows == 6);

	for (int i = 0; i < 5; i ++) {
		constraintParams->m_jacobians[i].m_jacobian_J01.m_linear = dVector (descriptor.m_jacobian01[i][0], descriptor.m_jacobian01[i][1], descriptor.m_jacobian01[i][2], dFloat (0.0f));
		constraintParams->m_jacobians[i].m_jacobian_J01.m_angular = dVector (descriptor.m_jacobian01[i][3], descriptor.m_jacobian01[i][4], descriptor.m_jacobian01[i][5], dFloat (0.0f));
		constraintParams->m_jacobians[i].m_jacobian_J10.m_linear = dVector(descriptor.m_jacobian10[i][0], descriptor.m_jacobian10[i][1], descriptor.m_jacobian10[i][2], dFloat(0.0f));
		constraintParams->m_jacobians[i].m_jacobian_J10.m_angular = dVector(descriptor.m_jacobian10[i][3], descriptor.m_jacobian10[i][4], descriptor.m_jacobian10[i][5], dFloat(0.0f));
		constraintParams->m_jointAccel[i] = descriptor.m_jointAccel[i];
		constraintParams->m_jointLowFrictionCoef[i] = descriptor.m_minFriction[i];
		constraintParams->m_jointHighFrictionCoef[i] = descriptor.m_maxFriction[i];
		constraintParams->m_normalIndex[i] = 0;
	}

	m_jacobial01.m_linear = dVector (descriptor.m_jacobian01[5][0], descriptor.m_jacobian01[5][1], descriptor.m_jacobian01[5][2], dFloat (0.0f));
	m_jacobial01.m_angular = dVector (descriptor.m_jacobian01[5][3], descriptor.m_jacobian01[5][4], descriptor.m_jacobian01[5][5], dFloat (0.0f));

	m_jacobial10.m_linear = dVector(descriptor.m_jacobian10[5][0], descriptor.m_jacobian10[5][1], descriptor.m_jacobian10[5][2], dFloat(0.0f));
	m_jacobial10.m_angular = dVector(descriptor.m_jacobian10[5][3], descriptor.m_jacobian10[5][4], descriptor.m_jacobian10[5][5], dFloat(0.0f));

	constraintParams->m_count = 5;
}

void dAnimationRigHinge::UpdateJointAcceleration()
{
	dComplementaritySolver::dBodyState* const body0 = GetBody();
	dComplementaritySolver::dBodyState* const body1 = m_parent->GetBody();
	const dVector& accel0 = body0->GetForce().Scale (body0->GetInvMass());
	const dVector& alpha0 = body0->GetInvInertia().RotateVector(body0->GetTorque());
	const dVector& accel1 = body1->GetForce().Scale (body1->GetInvMass());;
	const dVector& alpha1 = body1->GetInvInertia().RotateVector(body1->GetTorque());

	dVector accel (accel0 * m_jacobial01.m_linear + alpha0 * m_jacobial01.m_angular + accel1 * m_jacobial10.m_linear + alpha1 * m_jacobial10.m_angular); 
	m_rowAccel = accel.m_x + accel.m_y + accel.m_z;

//dTrace (("%f\n", m_rowAccel))
	dAnimationRigLimb::UpdateJointAcceleration();
}