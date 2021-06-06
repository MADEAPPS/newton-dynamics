/* Copyright (c) <2003-2021> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/

#include "dCoreStdafx.h"
#include "ndNewtonStdafx.h"
#include "ndJointHinge.h"

ndJointHinge::ndJointHinge(const dMatrix& pinAndPivotFrame, ndBodyKinematic* const child, ndBodyKinematic* const parent)
	:ndJointBilateralConstraint(6, child, parent, pinAndPivotFrame)
	,m_jointAngle(dFloat32(0.0f))
	,m_jointSpeed(dFloat32(0.0f))
	,m_minLimit(dFloat32(0.0f))
	,m_maxLimit(dFloat32(0.0f))
	,m_friction(dFloat32(0.0f))
	,m_hasLimits(false)
	,m_limitReached(false)
{
}

ndJointHinge::~ndJointHinge()
{
}

void ndJointHinge::EnableLimits(bool state, dFloat32 minLimit, dFloat32 maxLimit)
{
	m_hasLimits = state;
	dAssert(minLimit <= 0.0f);
	dAssert(maxLimit >= 0.0f);
	m_minLimit = minLimit;
	m_maxLimit = maxLimit;

	// adding one extra dof, this makes the mass matrix ill conditioned, 
	// but it could work with the direct solver
	m_maxDof = (m_hasLimits) ? 7 : 6;
}

void ndJointHinge::SetFriction(dFloat32 frictionTorque)
{
	m_friction = dAbs(frictionTorque);
}

dFloat32 ndJointHinge::GetFriction() const
{
	return m_friction;
}

void ndJointHinge::SubmitConstraintLimits(ndConstraintDescritor& desc, const dMatrix& matrix0, const dMatrix& matrix1)
{
	m_limitReached = true;
	if ((m_minLimit > dFloat32 (-1.e-4f)) && (m_maxLimit < dFloat32(1.e-4f)))
	{
		AddAngularRowJacobian(desc, &matrix1.m_front[0], -m_jointAngle);
		//NewtonUserJointSetRowStiffness(m_joint, 1.0f);
	}
	else 
	{
		//dFloat32 restoringOmega = 0.25f;
		const dFloat32 step = dMax(dAbs(m_jointSpeed * desc.m_timestep), dFloat32(3.0f * dDegreeToRad));
		const dFloat32 angle = m_jointAngle;
		if (angle <= m_minLimit) 
		{
			dAssert(0);
		//	NewtonUserJointAddAngularRow(m_joint, 0.0f, &matrix0.m_front[0]);
		//	const dFloat32 invtimestep = 1.0f / timestep;
		//	const dFloat32 error0 = angle - m_minLimit;
		//	const dFloat32 error1 = error0 + restoringOmega * timestep;
		//	if (error1 > 0.0f) {
		//		restoringOmega = -error0 * invtimestep;
		//	}
		//	const dFloat32 stopAccel = NewtonUserJointCalculateRowZeroAcceleration(m_joint) + restoringOmega * invtimestep;
		//	NewtonUserJointSetRowAcceleration(m_joint, stopAccel);
		//	NewtonUserJointSetRowStiffness(m_joint, m_stiffness);
		//	NewtonUserJointSetRowMinimumFriction(m_joint, -m_friction);
		}
		else if (angle >= m_maxLimit) 
		{
			dAssert(0);
		//	NewtonUserJointAddAngularRow(m_joint, 0.0f, &matrix0.m_front[0]);
		//	const dFloat32 invtimestep = 1.0f / timestep;
		//	const dFloat32 error0 = angle - m_maxLimit;
		//	const dFloat32 error1 = error0 - restoringOmega * timestep;
		//	if (error1 < 0.0f) {
		//		restoringOmega = error0 * invtimestep;
		//	}
		//	const dFloat32 stopAccel = NewtonUserJointCalculateRowZeroAcceleration(m_joint) - restoringOmega / timestep;
		//	NewtonUserJointSetRowAcceleration(m_joint, stopAccel);
		//	NewtonUserJointSetRowStiffness(m_joint, m_stiffness);
		//	NewtonUserJointSetRowMaximumFriction(m_joint, m_friction);
		}
		else if ((angle - step) <= m_minLimit) 
		{
			AddAngularRowJacobian(desc, &matrix0.m_front[0], dFloat32(0.0f));
			const dFloat32 stopAccel = GetMotorZeroAcceleration(desc);
			SetMotorAcceleration(desc, stopAccel);
			SetLowerFriction(desc, -m_friction);
		}
		else if ((angle + step) >= m_maxLimit) 
		{
			AddAngularRowJacobian(desc, &matrix0.m_front[0], dFloat32 (0.0f));
			const dFloat32 stopAccel = GetMotorZeroAcceleration(desc);
			SetMotorAcceleration(desc, stopAccel);
			SetHighFriction(desc, m_friction);
		}
		else if (m_friction != 0.0f) 
		{
			m_limitReached = false;
			AddAngularRowJacobian(desc, &matrix0.m_front[0], dFloat32(0.0f));
			SetMotorAcceleration(desc, -m_jointSpeed * desc.m_invTimestep);
			SetLowerFriction(desc, -m_friction);
			SetHighFriction(desc, m_friction);
		}
	}
}

void ndJointHinge::JacobianDerivative(ndConstraintDescritor& desc)
{
	dMatrix matrix0;
	dMatrix matrix1;
	CalculateGlobalMatrix(matrix0, matrix1);

	AddLinearRowJacobian(desc, matrix0.m_posit, matrix1.m_posit, matrix1[0]);
	AddLinearRowJacobian(desc, matrix0.m_posit, matrix1.m_posit, matrix1[1]);
	AddLinearRowJacobian(desc, matrix0.m_posit, matrix1.m_posit, matrix1[2]);

	// save the current joint Omega
	dVector omega0(m_body0->GetOmega());
	dVector omega1(m_body1->GetOmega());

	// the joint angle can be determined by getting the angle between any two non parallel vectors
	const dFloat32 deltaAngle = AnglesAdd(-CalculateAngle(matrix0.m_up, matrix1.m_up, matrix1.m_front), -m_jointAngle);
	m_jointAngle += deltaAngle;
	m_jointSpeed = matrix1.m_front.DotProduct(omega0 - omega1).GetScalar();

	// two rows to restrict rotation around around the parent coordinate system
	//const dFloat32 angleError = m_maxLimitError;
	const dFloat32 angle0 = CalculateAngle(matrix0.m_front, matrix1.m_front, matrix1.m_up);
	AddAngularRowJacobian(desc, matrix1.m_up, angle0);
	//if (dAbs(angle0) > angleError) 
	//{
	//	dAssert(0);
	//	//const dFloat32 alpha = NewtonUserJointCalculateRowZeroAcceleration(m_joint) + dFloat32(0.25f) * angle0 / (timestep * timestep);
	//	//NewtonUserJointSetRowAcceleration(m_joint, alpha);
	//}

	const dFloat32 angle1 = CalculateAngle(matrix0.m_front, matrix1.m_front, matrix1.m_right);
	AddAngularRowJacobian(desc, matrix1.m_right, angle1);
	//if (dAbs(angle1) > angleError) 
	//{
	//	dAssert(0);
	//	//const dFloat32 alpha = NewtonUserJointCalculateRowZeroAcceleration(m_joint) + dFloat32(0.25f) * angle1 / (timestep * timestep);
	//	//NewtonUserJointSetRowAcceleration(m_joint, alpha);
	//}

	if (m_hasLimits)
	{
		SubmitConstraintLimits(desc, matrix0, matrix1);
	}
	else if (m_friction != dFloat32 (0.0f)) 
	{
		AddAngularRowJacobian(desc, &matrix0.m_front[0], dFloat32(0.0f));
		SetMotorAcceleration(desc, -m_jointSpeed * desc.m_invTimestep);
		SetLowerFriction(desc, -m_friction);
		SetHighFriction(desc, m_friction);
	}
}


