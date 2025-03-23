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
#include "ndJointSlider.h"

#define D_MAX_SLIDER_RECOVERY_SPEED	ndFloat32 (0.5f)
#define D_MAX_SLIDER_PENETRATION	ndFloat32 (0.05f)

ndJointSlider::ndJointSlider()
	:ndJointBilateralConstraint()
	,m_posit(ndFloat32(0.0f))
	,m_speed(ndFloat32(0.0f))
	,m_springK(ndFloat32(0.0f))
	,m_damperC(ndFloat32(0.0f))
	,m_minLimit(ndFloat32(-1.0e10f))
	,m_maxLimit(ndFloat32(1.0e10f))
	,m_positOffset(ndFloat32(0.0f))
	,m_springDamperRegularizer(ndFloat32(0.1f))
	,m_maxForce(D_LCP_MAX_VALUE)
	,m_limitState(0)
	,m_forceState(0)
{
	m_maxDof = 7;
}

ndJointSlider::ndJointSlider(const ndMatrix& pinAndPivotFrame, ndBodyKinematic* const child, ndBodyKinematic* const parent)
	:ndJointBilateralConstraint(7, child, parent, pinAndPivotFrame)
	,m_posit(ndFloat32 (0.0f))
	,m_speed(ndFloat32(0.0f))
	,m_springK(ndFloat32(0.0f))
	,m_damperC(ndFloat32(0.0f))
	,m_minLimit(ndFloat32(-1.0e10f))
	,m_maxLimit(ndFloat32(1.0e10f))
	,m_positOffset(ndFloat32(0.0f))
	,m_springDamperRegularizer(ndFloat32(0.1f))
	,m_maxForce(D_LCP_MAX_VALUE)
	,m_limitState(0)
	,m_forceState(0)
{
}

ndJointSlider::ndJointSlider(const ndMatrix& pinAndPivotInChild, const ndMatrix& pinAndPivotInParent, ndBodyKinematic* const child, ndBodyKinematic* const parent)
	:ndJointBilateralConstraint(7, child, parent, pinAndPivotInChild)
	,m_posit(ndFloat32(0.0f))
	,m_speed(ndFloat32(0.0f))
	,m_springK(ndFloat32(0.0f))
	,m_damperC(ndFloat32(0.0f))
	,m_minLimit(ndFloat32(-1.0e10f))
	,m_maxLimit(ndFloat32(1.0e10f))
	,m_positOffset(ndFloat32(0.0f))
	,m_springDamperRegularizer(ndFloat32(0.1f))
	,m_maxForce(D_LCP_MAX_VALUE)
	,m_limitState(0)
	,m_forceState(0)
{
	ndMatrix tmp;
	CalculateLocalMatrix(pinAndPivotInChild, m_localMatrix0, tmp);
	CalculateLocalMatrix(pinAndPivotInParent, tmp, m_localMatrix1);
}

ndJointSlider::~ndJointSlider()
{
}

ndFloat32 ndJointSlider::GetSpeed() const
{
	return m_speed;
}

ndFloat32 ndJointSlider::GetPosit() const
{
	return m_posit;
}

ndFloat32 ndJointSlider::GetTargetPosit() const
{
	return m_positOffset;
}

void ndJointSlider::SetTargetPosit(ndFloat32 offset)
{
	m_positOffset = offset;
}

bool ndJointSlider::GetLimitState() const
{
	return m_limitState ? true : false;
}

void ndJointSlider::SetLimitState(bool state)
{
	m_limitState = state ? 1 : 0;
	if (m_limitState)
	{
		SetLimits(m_minLimit, m_maxLimit);
	}
}

void ndJointSlider::SetLimits(ndFloat32 minLimit, ndFloat32 maxLimit)
{
#ifdef _DEBUG
	if (minLimit > 0.0f)
	{
		ndTrace(("warning: %s minLimit %f larger than zero\n", __FUNCTION__, minLimit))
	}
	if (maxLimit < 0.0f)
	{
		ndTrace(("warning: %s m_maxLimit %f smaller than zero\n", __FUNCTION__, maxLimit))
	}
#endif

	m_minLimit = minLimit;
	m_maxLimit = maxLimit;
	if (m_posit > m_maxLimit)
	{
		m_posit = m_maxLimit;
	}
	else if (m_posit < m_minLimit)
	{
		m_posit = m_minLimit;
	}
}

void ndJointSlider::GetLimits(ndFloat32& minLimit, ndFloat32& maxLimit) const
{
	minLimit = m_minLimit;
	maxLimit = m_maxLimit;
}

bool ndJointSlider::GetMaxForceState() const
{
	return m_forceState ? true : false;
}

void ndJointSlider::SetMaxForceState(bool state)
{
	m_forceState = state ? true : false;
}

ndFloat32 ndJointSlider::GetMaxForce() const
{
	return m_maxForce;
}

void ndJointSlider::SetMaxForce(ndFloat32 force)
{
	m_maxForce = ndClamp(force, ndFloat32(0.1f), D_LCP_MAX_VALUE);
}

void ndJointSlider::SetAsSpringDamper(ndFloat32 regularizer, ndFloat32 spring, ndFloat32 damper)
{
	m_springK = ndAbs(spring);
	m_damperC = ndAbs(damper);
	m_springDamperRegularizer = ndClamp(regularizer, ND_SPRING_DAMP_MIN_REG, ndFloat32(0.99f));
}

void ndJointSlider::GetSpringDamper(ndFloat32& regularizer, ndFloat32& spring, ndFloat32& damper) const
{
	spring = m_springK;
	damper = m_damperC;
	regularizer = m_springDamperRegularizer;
}

void ndJointSlider::DebugJoint(ndConstraintDebugCallback& debugCallback) const
{
	ndMatrix matrix0;
	ndMatrix matrix1;
	CalculateGlobalMatrix(matrix0, matrix1);

	debugCallback.DrawFrame(matrix0);
	debugCallback.DrawFrame(matrix1, 0.5f);

}

ndFloat32 ndJointSlider::PenetrationSpeed(ndFloat32 penetration) const
{
	ndFloat32 param = ndClamp(penetration, ndFloat32(0.0f), D_MAX_SLIDER_PENETRATION) / D_MAX_SLIDER_PENETRATION;
	ndFloat32 speed = D_MAX_SLIDER_RECOVERY_SPEED * param;
	return speed;
}

ndInt32 ndJointSlider::GetKinematicState(ndKinematicState* const state) const
{
	state->m_posit = m_posit;
	state->m_velocity = m_speed;
	return 1;
}

void ndJointSlider::ClearMemory()
{
	ndMatrix matrix0;
	ndMatrix matrix1;
	CalculateGlobalMatrix(matrix0, matrix1);

	ndJointBilateralConstraint::ClearMemory();

	const ndVector& p0 = matrix0.m_posit;
	const ndVector& p1 = matrix1.m_posit;
	const ndVector veloc0(m_body0->GetVelocityAtPoint(matrix0.m_posit));
	const ndVector veloc1(m_body1->GetVelocityAtPoint(matrix1.m_posit));

	const ndVector prel(p0 - p1);
	const ndVector vrel(veloc0 - veloc1);

	m_speed = vrel.DotProduct(matrix1.m_front).GetScalar();
	m_posit = prel.DotProduct(matrix1.m_front).GetScalar();
	m_positOffset = m_posit;
}

void ndJointSlider::SubmitLimits(ndConstraintDescritor& desc, const ndMatrix& matrix0, const ndMatrix& matrix1)
{
	if (m_limitState)
	{
		if ((m_minLimit == ndFloat32(0.0f)) && (m_maxLimit == ndFloat32(0.0f)))
		{
			AddLinearRowJacobian(desc, matrix0.m_posit, matrix1.m_posit, matrix1.m_front);
		}
		else
		{
			ndFloat32 x = m_posit + m_speed * desc.m_timestep;
			if (x < m_minLimit)
			{
				ndVector p1(matrix1.m_posit + matrix1.m_front.Scale(m_minLimit));
				AddLinearRowJacobian(desc, matrix0.m_posit, p1, matrix1.m_front);
				const ndFloat32 stopAccel = GetMotorZeroAcceleration(desc);
				const ndFloat32 penetration = x - m_minLimit;
				const ndFloat32 recoveringAceel = -desc.m_invTimestep * PenetrationSpeed(-penetration);
				SetMotorAcceleration(desc, stopAccel - recoveringAceel);
				SetLowerFriction(desc, ndFloat32(0.0f));
			}
			else if (x > m_maxLimit)
			{
				AddLinearRowJacobian(desc, matrix0.m_posit, matrix0.m_posit, matrix1.m_front);
				const ndFloat32 stopAccel = GetMotorZeroAcceleration(desc);
				const ndFloat32 penetration = x - m_maxLimit;
				const ndFloat32 recoveringAceel = desc.m_invTimestep * PenetrationSpeed(penetration);
				SetMotorAcceleration(desc, stopAccel - recoveringAceel);
				SetHighFriction(desc, ndFloat32(0.0f));
			}
		}
	}
}

void ndJointSlider::SubmitSpringDamper(ndConstraintDescritor& desc, const ndMatrix& matrix0, const ndMatrix& matrix1)
{
	// add spring damper row
	const ndVector p1(matrix1.m_posit + matrix1.m_front.Scale(m_positOffset));
	AddLinearRowJacobian(desc, matrix0.m_posit, p1, matrix1.m_front);
	SetMassSpringDamperAcceleration(desc, m_springDamperRegularizer, m_springK, m_damperC);
	if (m_forceState)
	{
		SetHighFriction(desc, m_maxForce);
		SetLowerFriction(desc, -m_maxForce);
	}
}

void ndJointSlider::ApplyBaseRows(ndConstraintDescritor& desc, const ndMatrix& matrix0, const ndMatrix& matrix1)
{
	const ndVector veloc0(m_body0->GetVelocityAtPoint(matrix0.m_posit));
	const ndVector veloc1(m_body1->GetVelocityAtPoint(matrix1.m_posit));

	const ndVector& pin = matrix1[0];
	const ndVector& p0 = matrix0.m_posit;
	const ndVector& p1 = matrix1.m_posit;
	const ndVector prel(p0 - p1);
	const ndVector vrel(veloc0 - veloc1);

	m_speed = vrel.DotProduct(matrix1.m_front).GetScalar();
	m_posit = prel.DotProduct(matrix1.m_front).GetScalar();
	const ndVector projectedPoint = p1 + pin.Scale(pin.DotProduct(prel).GetScalar());

	AddLinearRowJacobian(desc, p0, projectedPoint, matrix1[1]);
	AddLinearRowJacobian(desc, p0, projectedPoint, matrix1[2]);

	const ndFloat32 angle0 = CalculateAngle(matrix0.m_up, matrix1.m_up, matrix1.m_front);
	AddAngularRowJacobian(desc, matrix1.m_front, angle0);

	const ndFloat32 angle1 = CalculateAngle(matrix0.m_front, matrix1.m_front, matrix1.m_up);
	AddAngularRowJacobian(desc, matrix1.m_up, angle1);

	const ndFloat32 angle2 = CalculateAngle(matrix0.m_front, matrix1.m_front, matrix1.m_right);
	AddAngularRowJacobian(desc, matrix1.m_right, angle2);
}

void ndJointSlider::JacobianDerivative(ndConstraintDescritor& desc)
{
	ndMatrix matrix0;
	ndMatrix matrix1;

	// calculate the position of the pivot point and the Jacobian direction vectors, in global space. 
	CalculateGlobalMatrix(matrix0, matrix1);

	ApplyBaseRows(desc, matrix0, matrix1);

	if (m_springDamperRegularizer && ((m_springK > ndFloat32(0.0f)) || (m_damperC > ndFloat32(0.0f))))
	{
		SubmitSpringDamper(desc, matrix0, matrix1);
	}

	SubmitLimits(desc, matrix0, matrix1);
}
