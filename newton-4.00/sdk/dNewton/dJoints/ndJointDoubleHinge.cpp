/* Copyright (c) <2003-2022> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/

#include "ndCoreStdafx.h"
#include "ndNewtonStdafx.h"
#include "ndJointDoubleHinge.h"

ndJointDoubleHinge::ndAxisParam::ndAxisParam()
	:m_angle(ndFloat32(0.0f))
	,m_omega(ndFloat32(0.0f))
	,m_springK(ndFloat32(0.0f))
	,m_damperC(ndFloat32(0.0f))
	,m_minLimit(ndFloat32(-1.0e10f))
	,m_maxLimit(ndFloat32(1.0e10f))
	,m_offsetAngle(ndFloat32(0.0f))
	,m_springDamperRegularizer(ndFloat32(0.1f))
{
}

ndJointDoubleHinge::ndJointDoubleHinge()
	:ndJointBilateralConstraint()
	,m_axis0()
	,m_axis1()
{
	m_maxDof = 8;
}

ndJointDoubleHinge::ndJointDoubleHinge(const ndMatrix& pinAndPivotFrame, ndBodyKinematic* const child, ndBodyKinematic* const parent)
	:ndJointBilateralConstraint(8, child, parent, pinAndPivotFrame)
	,m_axis0()
	,m_axis1()
{
}

ndJointDoubleHinge::~ndJointDoubleHinge()
{
}

ndFloat32 ndJointDoubleHinge::GetAngle0() const
{
	return m_axis0.m_angle;
}

ndFloat32 ndJointDoubleHinge::GetOmega0() const
{
	return m_axis0.m_omega;
}

void ndJointDoubleHinge::SetLimits0(ndFloat32 minLimit, ndFloat32 maxLimit)
{
	ndAssert(minLimit <= 0.0f);
	ndAssert(maxLimit >= 0.0f);
	m_axis0.m_minLimit = minLimit;
	m_axis0.m_maxLimit = maxLimit;
}

void ndJointDoubleHinge::GetLimits0(ndFloat32& minLimit, ndFloat32& maxLimit)
{
	minLimit = m_axis0.m_minLimit;
	maxLimit = m_axis0.m_maxLimit;
}

ndFloat32 ndJointDoubleHinge::GetOffsetAngle0() const
{
	return m_axis0.m_offsetAngle;
}

void ndJointDoubleHinge::SetOffsetAngle0(ndFloat32 angle)
{
	m_axis0.m_offsetAngle = angle;
}

void ndJointDoubleHinge::SetAsSpringDamper0(ndFloat32 regularizer, ndFloat32 spring, ndFloat32 damper)
{
	m_axis0.m_springK = ndAbs(spring);
	m_axis0.m_damperC = ndAbs(damper);
	m_axis0.m_springDamperRegularizer = ndClamp(regularizer, ND_SPRING_DAMP_MIN_REG, ndFloat32(0.99f));
}

void ndJointDoubleHinge::GetSpringDamper0(ndFloat32& regularizer, ndFloat32& spring, ndFloat32& damper) const
{
	spring = m_axis0.m_springK;
	damper = m_axis0.m_damperC;
	regularizer = m_axis0.m_springDamperRegularizer;
}

ndFloat32 ndJointDoubleHinge::GetAngle1() const
{
	return m_axis1.m_angle;
}

ndFloat32 ndJointDoubleHinge::GetOmega1() const
{
	return m_axis1.m_omega;
}

void ndJointDoubleHinge::SetLimits1(ndFloat32 minLimit, ndFloat32 maxLimit)
{
	ndAssert(minLimit <= 0.0f);
	ndAssert(maxLimit >= 0.0f);
	m_axis1.m_minLimit = minLimit;
	m_axis1.m_maxLimit = maxLimit;
}

void ndJointDoubleHinge::GetLimits1(ndFloat32& minLimit, ndFloat32& maxLimit)
{
	minLimit = m_axis1.m_minLimit;
	maxLimit = m_axis1.m_maxLimit;
}

ndFloat32 ndJointDoubleHinge::GetOffsetAngle1() const
{
	return m_axis1.m_offsetAngle;
}

void ndJointDoubleHinge::SetOffsetAngle1(ndFloat32 angle)
{
	m_axis1.m_offsetAngle = angle;
}

void ndJointDoubleHinge::SetAsSpringDamper1(ndFloat32 regularizer, ndFloat32 spring, ndFloat32 damper)
{
	m_axis1.m_springK = ndAbs(spring);
	m_axis1.m_damperC = ndAbs(damper);
	m_axis1.m_springDamperRegularizer = ndClamp(regularizer, ND_SPRING_DAMP_MIN_REG, ndFloat32(0.99f));
}

void ndJointDoubleHinge::GetSpringDamper1(ndFloat32& regularizer, ndFloat32& spring, ndFloat32& damper) const
{
	spring = m_axis1.m_springK;
	damper = m_axis1.m_damperC;
	regularizer = m_axis1.m_springDamperRegularizer;
}

void ndJointDoubleHinge::DebugJoint(ndConstraintDebugCallback& debugCallback) const
{
	ndMatrix matrix0;
	ndMatrix matrix1;
	CalculateGlobalMatrix(matrix0, matrix1);

	debugCallback.DrawFrame(matrix0);
	debugCallback.DrawFrame(matrix1);

	const int subdiv = 12;
	ndVector arch[subdiv + 1];
	const ndFloat32 radius = debugCallback.m_debugScale;

	if ((m_axis0.m_maxLimit > ndFloat32(1.0e-3f)) || (m_axis0.m_minLimit < -ndFloat32(1.0e-3f)))
	{
		// show pitch angle limits
		ndVector point(ndFloat32(0.0f), ndFloat32(radius), ndFloat32(0.0f), ndFloat32(0.0f));
	
		ndFloat32 minAngle = m_axis0.m_minLimit;
		ndFloat32 maxAngle = m_axis0.m_maxLimit;
		if ((maxAngle - minAngle) >= ndPi * ndFloat32(2.0f))
		{
			minAngle = 0.0f;
			maxAngle = ndPi * ndFloat32(2.0f);
		}
	
		ndFloat32 angleStep = (maxAngle - minAngle) / subdiv;
		ndFloat32 angle0 = minAngle;
	
		ndVector color(ndVector(0.5f, 0.0f, 0.0f, 0.0f));
		for (ndInt32 i = 0; i <= subdiv; ++i)
		{
			arch[i] = matrix0.TransformVector(ndPitchMatrix(angle0).RotateVector(point));
			debugCallback.DrawLine(matrix1.m_posit, arch[i], color);
			angle0 += angleStep;
		}
	
		for (ndInt32 i = 0; i < subdiv; ++i)
		{
			debugCallback.DrawLine(arch[i], arch[i + 1], color);
		}
	}

	if ((m_axis1.m_maxLimit > ndFloat32(1.0e-3f)) || (m_axis1.m_minLimit < -ndFloat32(1.0e-3f)))
	{
		// show yaw angle limits
		ndVector point(ndFloat32(radius), ndFloat32(0.0f), ndFloat32(0.0f), ndFloat32(0.0f));
	
		ndFloat32 minAngle = m_axis1.m_minLimit;
		ndFloat32 maxAngle = m_axis1.m_maxLimit;
		if ((maxAngle - minAngle) >= ndPi * ndFloat32(2.0f))
		{
			minAngle = 0.0f;
			maxAngle = ndPi * ndFloat32(2.0f);
		}
	
		ndFloat32 angleStep = (maxAngle - minAngle) / subdiv;
		ndFloat32 angle0 = minAngle;
	
		ndVector color(ndVector(0.0f, 0.5f, 0.0f, 0.0f));
		for (ndInt32 i = 0; i <= subdiv; ++i) 
		{
			arch[i] = matrix1.TransformVector(ndYawMatrix(angle0).RotateVector(point));
			debugCallback.DrawLine(matrix1.m_posit, arch[i], color);
			angle0 += angleStep;
		}
	
		for (ndInt32 i = 0; i < subdiv; ++i)
		{
			debugCallback.DrawLine(arch[i], arch[i + 1], color);
		}
	}
}

void ndJointDoubleHinge::ApplyBaseRows(ndConstraintDescritor& desc, const ndMatrix& matrix0, const ndMatrix& matrix1)
{
	AddLinearRowJacobian(desc, matrix0.m_posit, matrix1.m_posit, matrix1[0]);
	AddLinearRowJacobian(desc, matrix0.m_posit, matrix1.m_posit, matrix1[1]);
	AddLinearRowJacobian(desc, matrix0.m_posit, matrix1.m_posit, matrix1[2]);

	const ndVector frontDir((matrix0.m_front - matrix1.m_up.Scale(matrix0.m_front.DotProduct(matrix1.m_up).GetScalar())).Normalize());
	const ndVector sideDir(frontDir.CrossProduct(matrix1.m_up));
	const ndFloat32 angle = CalculateAngle(matrix0.m_front, frontDir, sideDir);
	AddAngularRowJacobian(desc, sideDir, angle);
	
	// not happy with this method because it is a penalty system, 
	// but is hard for a first order integrator to prevent the side angle 
	// from drifting, even an implicit one expanding the Jacobian partial 
	// derivatives still has a hard time.
	// nullifying the gyro torque generated by the two angular velocities.
	const ndFloat32 alphaRollError = GetMotorZeroAcceleration(desc) + ndFloat32 (0.5f) * angle * desc.m_invTimestep * desc.m_invTimestep;
	SetMotorAcceleration(desc, alphaRollError);

	// save the current joint Omega
	const ndVector omega0(m_body0->GetOmega());
	const ndVector omega1(m_body1->GetOmega());
	
	// calculate joint parameters, angles and omega
	const ndFloat32 deltaAngle0 = ndAnglesAdd(-CalculateAngle(matrix0.m_up, matrix1.m_up, frontDir), -m_axis0.m_angle);
	m_axis0.m_angle += deltaAngle0;
	m_axis0.m_omega = frontDir.DotProduct(omega0 - omega1).GetScalar();
	
	const ndFloat32 deltaAngle1 = ndAnglesAdd(-CalculateAngle(frontDir, matrix1.m_front, matrix1.m_up), -m_axis1.m_angle);
	m_axis1.m_angle += deltaAngle1;
	m_axis1.m_omega = matrix1.m_up.DotProduct(omega0 - omega1).GetScalar();
}

ndFloat32 ndJointDoubleHinge::PenetrationOmega(ndFloat32 penetration) const
{
	ndFloat32 param = ndClamp(penetration, ndFloat32(0.0f), D_MAX_DOUBLE_HINGE_PENETRATION) / D_MAX_DOUBLE_HINGE_PENETRATION;
	ndFloat32 omega = D_MAX_DOUBLE_HINGE_RECOVERY_SPEED * param;
	return omega;
}

void ndJointDoubleHinge::ClearMemory()
{
	ndMatrix matrix0;
	ndMatrix matrix1;
	CalculateGlobalMatrix(matrix0, matrix1);

	ndJointBilateralConstraint::ClearMemory();

	// save the current joint Omega
	const ndVector omega0(m_body0->GetOmega());
	const ndVector omega1(m_body1->GetOmega());

	const ndVector frontDir((matrix0.m_front - matrix1.m_up.Scale(matrix0.m_front.DotProduct(matrix1.m_up).GetScalar())).Normalize());

	// calculate joint parameters, angles and omega
	const ndFloat32 deltaAngle0 = CalculateAngle(matrix0.m_up, matrix1.m_up, frontDir);
	m_axis0.m_angle = deltaAngle0;
	m_axis0.m_offsetAngle = deltaAngle0;
	m_axis0.m_omega = frontDir.DotProduct(omega0 - omega1).GetScalar();

	const ndFloat32 deltaAngle1 = CalculateAngle(frontDir, matrix1.m_front, matrix1.m_up);
	m_axis1.m_angle += deltaAngle1;
	m_axis1.m_offsetAngle = deltaAngle1;
	m_axis1.m_omega = matrix1.m_up.DotProduct(omega0 - omega1).GetScalar();
}

void ndJointDoubleHinge::SubmitLimits(ndConstraintDescritor& desc, const ndMatrix& matrix0, const ndMatrix& matrix1)
{
	if ((m_axis0.m_minLimit > (ndFloat32(-1.0f) * ndDegreeToRad)) && (m_axis0.m_maxLimit < (ndFloat32(1.0f) * ndDegreeToRad)))
	{
		AddAngularRowJacobian(desc, &matrix1.m_front[0], -m_axis0.m_angle);
	}
	else
	{
		const ndFloat32 angle = m_axis0.m_angle + m_axis0.m_omega * desc.m_timestep;
		if (angle < m_axis0.m_minLimit)
		{
			AddAngularRowJacobian(desc, &matrix0.m_front[0], ndFloat32(0.0f));
			const ndFloat32 stopAccel = GetMotorZeroAcceleration(desc);
			const ndFloat32 penetration = angle - m_axis0.m_minLimit;
			const ndFloat32 recoveringAceel = -desc.m_invTimestep * PenetrationOmega(-penetration);
			SetMotorAcceleration(desc, stopAccel - recoveringAceel);
			SetLowerFriction(desc, ndFloat32(0.0f));
		}
		else if (angle > m_axis0.m_maxLimit)
		{
			AddAngularRowJacobian(desc, &matrix0.m_front[0], ndFloat32(0.0f));
			const ndFloat32 stopAccel = GetMotorZeroAcceleration(desc);
			const ndFloat32 penetration = angle - m_axis0.m_maxLimit;
			const ndFloat32 recoveringAceel = desc.m_invTimestep * PenetrationOmega(penetration);
			SetMotorAcceleration(desc, stopAccel - recoveringAceel);
			SetHighFriction(desc, ndFloat32(0.0f));
		}
	}

	if ((m_axis1.m_minLimit > (ndFloat32(-1.0f) * ndDegreeToRad)) && (m_axis1.m_maxLimit < (ndFloat32(1.0f) * ndDegreeToRad)))
	{
		AddAngularRowJacobian(desc, &matrix1.m_up[0], -m_axis1.m_angle);
	}
	else
	{
		const ndFloat32 angle = m_axis1.m_angle + m_axis1.m_omega * desc.m_timestep;
		if (angle < m_axis1.m_minLimit)
		{
			AddAngularRowJacobian(desc, &matrix1.m_up[0], ndFloat32(0.0f));
			const ndFloat32 stopAccel = GetMotorZeroAcceleration(desc);
			const ndFloat32 penetration = angle - m_axis1.m_minLimit;
			const ndFloat32 recoveringAceel = -desc.m_invTimestep * PenetrationOmega(-penetration);
			SetMotorAcceleration(desc, stopAccel - recoveringAceel);
			SetLowerFriction(desc, ndFloat32(0.0f));
		}
		else if (angle > m_axis1.m_maxLimit)
		{
			AddAngularRowJacobian(desc, &matrix1.m_up[0], ndFloat32(0.0f));
			const ndFloat32 stopAccel = GetMotorZeroAcceleration(desc);
			const ndFloat32 penetration = angle - m_axis1.m_maxLimit;
			const ndFloat32 recoveringAceel = desc.m_invTimestep * PenetrationOmega(penetration);
			SetMotorAcceleration(desc, stopAccel - recoveringAceel);
			SetHighFriction(desc, ndFloat32(0.0f));
		}
	}
}

void ndJointDoubleHinge::SubmitSpringDamper0(ndConstraintDescritor& desc, const ndMatrix& matrix0, const ndMatrix&)
{
	AddAngularRowJacobian(desc, matrix0.m_front, m_axis0.m_offsetAngle - m_axis0.m_angle);
	SetMassSpringDamperAcceleration(desc, m_axis0.m_springDamperRegularizer, m_axis0.m_springK, m_axis0.m_damperC);
}

void ndJointDoubleHinge::SubmitSpringDamper1(ndConstraintDescritor& desc, const ndMatrix&, const ndMatrix& matrix1)
{
	AddAngularRowJacobian(desc, matrix1.m_up, m_axis1.m_offsetAngle - m_axis1.m_angle);
	SetMassSpringDamperAcceleration(desc, m_axis1.m_springDamperRegularizer, m_axis1.m_springK, m_axis1.m_damperC);
}

void ndJointDoubleHinge::JacobianDerivative(ndConstraintDescritor& desc)
{
	ndMatrix matrix0;
	ndMatrix matrix1;
	CalculateGlobalMatrix(matrix0, matrix1);

	ApplyBaseRows(desc, matrix0, matrix1);
	
	if (m_axis0.m_springDamperRegularizer && ((m_axis0.m_springK > ndFloat32(0.0f)) || (m_axis0.m_damperC > ndFloat32(0.0f))))
	{
		// spring damper with limits
		SubmitSpringDamper0(desc, matrix0, matrix1);
	}

	if (m_axis1.m_springDamperRegularizer && ((m_axis1.m_springK > ndFloat32(0.0f)) || (m_axis1.m_damperC > ndFloat32(0.0f))))
	{
		// spring damper with limits
		SubmitSpringDamper1(desc, matrix0, matrix1);
	}

	SubmitLimits(desc, matrix0, matrix1);
}

