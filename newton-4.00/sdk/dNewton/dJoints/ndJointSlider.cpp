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

#include "dCoreStdafx.h"
#include "ndNewtonStdafx.h"
#include "ndJointSlider.h"

ndJointSlider::ndJointSlider(const dMatrix& pinAndPivotFrame, ndBodyKinematic* const child, ndBodyKinematic* const parent)
	:ndJointBilateralConstraint(6, child, parent, pinAndPivotFrame)
	,m_posit(dFloat32 (0.0f))
	,m_speed(dFloat32(0.0f))
	,m_springK(dFloat32(0.0f))
	,m_damperC(dFloat32(0.0f))
	,m_minLimit(dFloat32(0.0f))
	,m_maxLimit(dFloat32(0.0f))
	,m_friction(dFloat32(0.0f))
	,m_springDamperRegularizer(dFloat32(0.1f))
	,m_hasLimits(false)
	,m_isStringDamper(false)
{
}

ndJointSlider::~ndJointSlider()
{
}

void ndJointSlider::SetFriction(dFloat32 friction)
{
	m_friction = dAbs(friction);
}

void ndJointSlider::EnableLimits(bool state, dFloat32 minLimit, dFloat32 maxLimit)
{
	m_hasLimits = state;
	dAssert(minLimit <= 0.0f);
	dAssert(maxLimit >= 0.0f);
	m_minLimit = minLimit;
	m_maxLimit = maxLimit;

	// adding one extra dof, this makes the mass matrix ill conditioned, 
	// but it could work with the direct solver
	m_maxDof = (m_isStringDamper && m_hasLimits) ? 7 : 6;
}

void ndJointSlider::SetAsSpringDamper(bool state, dFloat32 regularizer, dFloat32 spring, dFloat32 damper)
{
	m_springK = dAbs(spring);
	m_damperC = dAbs(damper);
	m_springDamperRegularizer = dClamp(regularizer, dFloat32(1.0e-2f), dFloat32(0.99f));
	m_isStringDamper = state;

	// adding one extra dof, this makes the mass matrix ill conditioned, 
	// but it could work with the direct solver
	m_maxDof = (m_isStringDamper && m_hasLimits) ? 7 : 6;
}

void ndJointSlider::SubmitConstraintLimits(ndConstraintDescritor& desc, const dMatrix& matrix0, const dMatrix& matrix1)
{
	if ((m_minLimit == dFloat32 (0.0f)) && (m_maxLimit == dFloat32(0.0f)))
	{
		AddLinearRowJacobian(desc, matrix0.m_posit, matrix1.m_posit, matrix1.m_front);
	}
	else 
	{
		dFloat32 x = m_posit + m_speed * desc.m_timestep;
		if (x < m_minLimit)
		{
			dVector p1(matrix1.m_posit + matrix1.m_front.Scale (m_minLimit));
			AddLinearRowJacobian(desc, matrix0.m_posit, p1, matrix1.m_front);
			SetLowerFriction(desc, -m_friction); 
		}
		else if (x > m_maxLimit)
		{
			dVector p1(matrix1.m_posit + matrix1.m_front.Scale(m_maxLimit));
			AddLinearRowJacobian(desc, matrix0.m_posit, matrix0.m_posit, matrix1.m_front);
			SetHighFriction(desc, m_friction);
		}
		else if (m_friction > dFloat32 (0.0f)) 
		{
			AddLinearRowJacobian(desc, matrix0.m_posit, matrix0.m_posit, matrix1.m_front);
			SetMotorAcceleration(desc, -m_speed * desc.m_invTimestep);
			SetLowerFriction(desc, -m_friction);
			SetHighFriction(desc, m_friction);
		}
	}
}

void ndJointSlider::SubmitConstraintLimitSpringDamper(ndConstraintDescritor& desc, const dMatrix& matrix0, const dMatrix& matrix1)
{
	// add spring damper row
	AddLinearRowJacobian(desc, matrix0.m_posit, matrix1.m_posit, matrix1.m_front);
	SetMassSpringDamperAcceleration(desc, m_springDamperRegularizer, m_springK, m_damperC);

	dFloat32 x = m_posit + m_speed * desc.m_timestep;
	if (x < m_minLimit)
	{
		const dVector p1(matrix1.m_posit + matrix1.m_front.Scale(m_minLimit));
		AddLinearRowJacobian(desc, matrix0.m_posit, p1, matrix1.m_front);
		SetLowerFriction(desc, dFloat32 (0.0f));
		const dFloat32 stopAccel = GetMotorZeroAcceleration(desc);
		SetMotorAcceleration(desc, stopAccel);
	}
	else if (x > m_maxLimit)
	{
		const dVector p1(matrix1.m_posit + matrix1.m_front.Scale(m_maxLimit));
		AddLinearRowJacobian(desc, matrix0.m_posit, p1, matrix1.m_front);
		SetHighFriction(desc, dFloat32(0.0f));
		const dFloat32 stopAccel = GetMotorZeroAcceleration(desc);
		SetMotorAcceleration(desc, stopAccel);
	}
}

void ndJointSlider::JacobianDerivative(ndConstraintDescritor& desc)
{
	dMatrix matrix0;
	dMatrix matrix1;

	// calculate the position of the pivot point and the Jacobian direction vectors, in global space. 
	CalculateGlobalMatrix(matrix0, matrix1);

	// calculate position and speed	
	const dVector veloc0(m_body0->GetVelocityAtPoint(matrix0.m_posit));
	const dVector veloc1(m_body1->GetVelocityAtPoint(matrix1.m_posit));

	const dVector& pin = matrix1[0];
	const dVector& p0 = matrix0.m_posit;
	const dVector& p1 = matrix1.m_posit;
	const dVector prel(p0 - p1);
	const dVector vrel(veloc0 - veloc1);

	m_speed = vrel.DotProduct(matrix1.m_front).GetScalar();
	m_posit = prel.DotProduct(matrix1.m_front).GetScalar();
	const dVector projectedPoint = p1 + pin.Scale(pin.DotProduct(prel).GetScalar());

	AddLinearRowJacobian(desc, p0, projectedPoint, matrix1[1]);
	AddLinearRowJacobian(desc, p0, projectedPoint, matrix1[2]);

	const dFloat32 angleError = m_maxAngleError;
	const dFloat32 angle0 = CalculateAngle(matrix0.m_up, matrix1.m_up, matrix1.m_front);
	AddAngularRowJacobian(desc, matrix1.m_front, angle0);
	if (dAbs(angle0) > angleError) 
	{
		dAssert(0);
	//	const dFloat32 alpha = NewtonUserJointCalculateRowZeroAcceleration(m_joint) + dFloat32(0.25f) * angle0 / (timestep * timestep);
	//	NewtonUserJointSetRowAcceleration(m_joint, alpha);
	}
	
	const dFloat32 angle1 = CalculateAngle(matrix0.m_front, matrix1.m_front, matrix1.m_up);
	AddAngularRowJacobian(desc, matrix1.m_up, angle1);
	if (dAbs(angle1) > angleError) 
	{
		dAssert(0);
	//	const dFloat32 alpha = NewtonUserJointCalculateRowZeroAcceleration(m_joint) + dFloat32(0.25f) * angle1 / (timestep * timestep);
	//	NewtonUserJointSetRowAcceleration(m_joint, alpha);
	}
	
	const dFloat32 angle2 = CalculateAngle(matrix0.m_front, matrix1.m_front, matrix1.m_right);
	AddAngularRowJacobian(desc, matrix1.m_right, angle2);
	//NewtonUserJointSetRowStiffness(m_joint, m_stiffness);
	if (dAbs(angle2) > angleError) 
	{
		dAssert(0);
	//	const dFloat32 alpha = NewtonUserJointCalculateRowZeroAcceleration(m_joint) + dFloat32(0.25f) * angle2 / (timestep * timestep);
	//	NewtonUserJointSetRowAcceleration(m_joint, alpha);
	}

	if (m_hasLimits)
	{
		if (m_isStringDamper)
		{
			// spring damper with limits
			SubmitConstraintLimitSpringDamper(desc, matrix0, matrix1);
		}
		else
		{
			// only hard limits
			SubmitConstraintLimits(desc, matrix0, matrix1);
		}
	}
	else if (m_isStringDamper)
	{
		// spring damper without limits
		AddLinearRowJacobian(desc, matrix0.m_posit, matrix1.m_posit, matrix1.m_front);
		SetMassSpringDamperAcceleration(desc, m_springDamperRegularizer, m_springK, m_damperC);
	}
}


