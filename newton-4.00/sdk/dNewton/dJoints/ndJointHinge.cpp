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
#include "ndJointHinge.h"

#define D_MAX_HINGE_RECOVERY_SPEED	ndFloat32 (0.25f)
#define D_MAX_HINGE_PENETRATION		(ndFloat32 (4.0f) * ndDegreeToRad)

ndJointHinge::ndJointHinge()
	:ndJointBilateralConstraint()
	,m_angle(ndFloat32(0.0f))
	,m_omega(ndFloat32(0.0f))
	,m_springK(ndFloat32(0.0f))
	,m_damperC(ndFloat32(0.0f))
	,m_minLimit(ndFloat32(-1.0e10f))
	,m_maxLimit(ndFloat32(1.0e10f))
	,m_targetAngle(ndFloat32(0.0f))
	,m_springDamperRegularizer(ndFloat32(0.1f))
	,m_limitState(0)
{
	m_maxDof = 7;
}

ndJointHinge::ndJointHinge(const ndMatrix& pinAndPivotFrame, ndBodyKinematic* const child, ndBodyKinematic* const parent)
	:ndJointBilateralConstraint(7, child, parent, pinAndPivotFrame)
	,m_angle(ndFloat32(0.0f))
	,m_omega(ndFloat32(0.0f))
	,m_springK(ndFloat32(0.0f))
	,m_damperC(ndFloat32(0.0f))
	,m_minLimit(ndFloat32(-1.0e10f))
	,m_maxLimit(ndFloat32(1.0e10f))
	,m_targetAngle(ndFloat32(0.0f))
	,m_springDamperRegularizer(ndFloat32(0.1f))
	,m_limitState(0)
{
}

ndJointHinge::ndJointHinge(const ndMatrix& pinAndPivotInChild, const ndMatrix& pinAndPivotInParent, ndBodyKinematic* const child, ndBodyKinematic* const parent)
	:ndJointBilateralConstraint(6, child, parent, pinAndPivotInChild)
	,m_angle(ndFloat32(0.0f))
	,m_omega(ndFloat32(0.0f))
	,m_springK(ndFloat32(0.0f))
	,m_damperC(ndFloat32(0.0f))
	,m_minLimit(ndFloat32(-1.0e10f))
	,m_maxLimit(ndFloat32(1.0e10f))
	,m_targetAngle(ndFloat32(0.0f))
	,m_springDamperRegularizer(ndFloat32(0.1f))
	,m_limitState(0)
{
	ndMatrix tmp;
	CalculateLocalMatrix(pinAndPivotInChild, m_localMatrix0, tmp);
	CalculateLocalMatrix(pinAndPivotInParent, tmp, m_localMatrix1);
}

ndJointHinge::~ndJointHinge()
{
}

ndFloat32 ndJointHinge::GetAngle() const
{
	return m_angle;
}

ndFloat32 ndJointHinge::GetOmega() const
{
	return m_omega;
}

bool ndJointHinge::GetLimitState() const
{
	return m_limitState ? true : false;
}

void ndJointHinge::SetLimitState(bool state)
{
	m_limitState = state ? 1 : 0;
	if (m_limitState)
	{
		SetLimits(m_minLimit, m_maxLimit);
	}
}

void ndJointHinge::SetLimits(ndFloat32 minLimit, ndFloat32 maxLimit)
{
	#ifdef _DEBUG
	if (minLimit > 0.0f)
	{
		ndTrace (("warning: %s minLimit %f larger than zero\n", __FUNCTION__, minLimit))
	}
	if (maxLimit < 0.0f)
	{
		ndTrace(("warning: %s m_maxLimit %f smaller than zero\n", __FUNCTION__, maxLimit))
	}
	#endif

	m_minLimit = minLimit;
	m_maxLimit = maxLimit;
	if (m_angle > m_maxLimit)
	{
		//const ndFloat32 deltaAngle = ndAnglesAdd(m_angle, -m_maxLimit);
		//m_angle = m_maxLimit + deltaAngle;
		m_angle = m_maxLimit;
	} 
	else if (m_angle < m_minLimit)
	{
		//const ndFloat32 deltaAngle = ndAnglesAdd(m_angle, -m_minLimit);
		//m_angle = m_minLimit + deltaAngle;
		m_angle = m_minLimit;
	}
}

void ndJointHinge::GetLimits(ndFloat32& minLimit, ndFloat32& maxLimit) const
{
	minLimit = m_minLimit;
	maxLimit = m_maxLimit;
}

ndFloat32 ndJointHinge::GetTargetAngle() const
{
	return m_targetAngle;
}

void ndJointHinge::SetTargetAngle(ndFloat32 angle)
{
	m_targetAngle = angle;
}

void ndJointHinge::SetAsSpringDamper(ndFloat32 regularizer, ndFloat32 spring, ndFloat32 damper)
{
	m_springK = ndAbs(spring);
	m_damperC = ndAbs(damper);
	m_springDamperRegularizer = ndClamp(regularizer, ND_SPRING_DAMP_MIN_REG, ndFloat32(0.99f));
}

void ndJointHinge::GetSpringDamper(ndFloat32& regularizer, ndFloat32& spring, ndFloat32& damper) const
{
	spring = m_springK;
	damper = m_damperC;
	regularizer = m_springDamperRegularizer;
}

void ndJointHinge::DebugJoint(ndConstraintDebugCallback& debugCallback) const
{
	ndMatrix matrix0;
	ndMatrix matrix1;
	CalculateGlobalMatrix(matrix0, matrix1);

	debugCallback.DrawFrame(matrix0);
	debugCallback.DrawFrame(matrix1, 0.5f);

	const ndInt32 subdiv = 8;
	const ndFloat32 radius = debugCallback.m_debugScale;
	ndVector arch[subdiv + 1];

	ndFloat32 deltaTwist = m_maxLimit - m_minLimit;
	if ((deltaTwist > ndFloat32(1.0e-3f)) && (deltaTwist <= ndFloat32(2.0f) * ndPi))
	{
		ndMatrix pitchMatrix(matrix1);
		pitchMatrix.m_posit = matrix1.m_posit;

		ndVector point(ndFloat32(0.0f), ndFloat32(radius), ndFloat32(0.0f), ndFloat32(0.0f));

		ndFloat32 angleStep = ndMin(deltaTwist, ndFloat32(2.0f * ndPi)) / subdiv;
		ndFloat32 angle0 = m_minLimit;

		ndVector color(ndFloat32(0.4f), ndFloat32(0.0f), ndFloat32(0.0f), ndFloat32(0.0f));
		for (ndInt32 i = 0; i <= subdiv; ++i)
		{
			arch[i] = pitchMatrix.TransformVector(ndPitchMatrix(angle0).RotateVector(point));
			debugCallback.DrawLine(pitchMatrix.m_posit, arch[i], color);
			angle0 += angleStep;
		}

		for (ndInt32 i = 0; i < subdiv; ++i)
		{
			debugCallback.DrawLine(arch[i], arch[i + 1], color);
		}
	}
}

void ndJointHinge::SubmitSpringDamper(ndConstraintDescritor& desc, const ndMatrix& matrix0, const ndMatrix& )
{
	// add spring damper row
	AddAngularRowJacobian(desc, matrix0.m_front, m_targetAngle - m_angle);
	SetMassSpringDamperAcceleration(desc, m_springDamperRegularizer, m_springK, m_damperC);
}

void ndJointHinge::ApplyBaseRows(ndConstraintDescritor& desc, const ndMatrix& matrix0, const ndMatrix& matrix1)
{
	AddLinearRowJacobian(desc, matrix0.m_posit, matrix1.m_posit, matrix1[0]);
	AddLinearRowJacobian(desc, matrix0.m_posit, matrix1.m_posit, matrix1[1]);
	AddLinearRowJacobian(desc, matrix0.m_posit, matrix1.m_posit, matrix1[2]);

	// two rows to restrict rotation around around the parent coordinate system
	const ndFloat32 angle0 = CalculateAngle(matrix0.m_front, matrix1.m_front, matrix1.m_up);
	AddAngularRowJacobian(desc, matrix1.m_up, angle0);

	const ndFloat32 angle1 = CalculateAngle(matrix0.m_front, matrix1.m_front, matrix1.m_right);
	AddAngularRowJacobian(desc, matrix1.m_right, angle1);

	// save the current joint Omega
	const ndVector omega0(m_body0->GetOmega());
	const ndVector omega1(m_body1->GetOmega());

	// the joint angle can be determined by getting the angle between any two non parallel vectors
	const ndFloat32 deltaAngle = ndAnglesAdd(-CalculateAngle(matrix0.m_up, matrix1.m_up, matrix1.m_front), -m_angle);
	m_angle += deltaAngle;
	m_omega = matrix1.m_front.DotProduct(omega0 - omega1).GetScalar();
}

ndFloat32 ndJointHinge::PenetrationOmega(ndFloat32 penetration) const
{
	ndFloat32 param = ndClamp(penetration, ndFloat32(0.0f), D_MAX_HINGE_PENETRATION) / D_MAX_HINGE_PENETRATION;
	ndFloat32 omega = D_MAX_HINGE_RECOVERY_SPEED * param;
	return omega;
}

void ndJointHinge::ClearMemory()
{
	ndMatrix matrix0;
	ndMatrix matrix1;
	CalculateGlobalMatrix(matrix0, matrix1);

	ndJointBilateralConstraint::ClearMemory();

	// save the current joint Omega
	const ndVector omega0(m_body0->GetOmega());
	const ndVector omega1(m_body1->GetOmega());

	// the joint angle can be determined by getting the angle between any two non parallel vectors
	m_angle = CalculateAngle(matrix0.m_up, matrix1.m_up, matrix1.m_front);
	m_omega = matrix1.m_front.DotProduct(omega0 - omega1).GetScalar();
	m_targetAngle = m_angle;
}

ndInt32 ndJointHinge::GetKinematicState(ndKinematicState* const state) const
{
	state->m_posit = m_angle;
	state->m_velocity = m_omega;
	return 1;
}

void ndJointHinge::SubmitLimits(ndConstraintDescritor& desc, const ndMatrix& matrix0, const ndMatrix& matrix1)
{
	m_hitLimits = false;
	if (m_limitState)
	{
		if ((m_minLimit > (ndFloat32(-1.0f) * ndDegreeToRad)) && (m_maxLimit < (ndFloat32(1.0f) * ndDegreeToRad)))
		{
			AddAngularRowJacobian(desc, &matrix1.m_front[0], -m_angle);
		}
		else
		{
			const ndFloat32 angle = m_angle + m_omega * desc.m_timestep;
			if (angle < m_minLimit)
			{
				m_hitLimits = true;
				AddAngularRowJacobian(desc, &matrix0.m_front[0], ndFloat32(0.0f));
				const ndFloat32 stopAccel = GetMotorZeroAcceleration(desc);
				const ndFloat32 penetration = angle - m_minLimit;
				const ndFloat32 recoveringAceel = -desc.m_invTimestep * PenetrationOmega(-penetration);
				SetMotorAcceleration(desc, stopAccel - recoveringAceel);
				SetLowerFriction(desc, ndFloat32(0.0f));
			}
			else if (angle > m_maxLimit)
			{
				m_hitLimits = true;
				AddAngularRowJacobian(desc, &matrix0.m_front[0], ndFloat32(0.0f));
				const ndFloat32 stopAccel = GetMotorZeroAcceleration(desc);
				const ndFloat32 penetration = angle - m_maxLimit;
				const ndFloat32 recoveringAceel = desc.m_invTimestep * PenetrationOmega(penetration);
				SetMotorAcceleration(desc, stopAccel - recoveringAceel);
				SetHighFriction(desc, ndFloat32(0.0f));
			}
		}
	}
}

void ndJointHinge::JacobianDerivative(ndConstraintDescritor& desc)
{
	ndMatrix matrix0;
	ndMatrix matrix1;
	CalculateGlobalMatrix(matrix0, matrix1);

	ApplyBaseRows(desc, matrix0, matrix1);
	if (m_springDamperRegularizer && ((m_springK > ndFloat32(0.0f)) || (m_damperC > ndFloat32(0.0f))))
	{
		// spring damper with limits
		SubmitSpringDamper(desc, matrix0, matrix1);
	}
	SubmitLimits(desc, matrix0, matrix1);
}
