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
#include "ndJointHingeActuator.h"

/*
ndJointHingeActuator::ndJointHingeActuator(const dMatrix& pinAndPivotFrame, NewtonBody* const child, NewtonBody* const parent)
	:dCustomHinge (pinAndPivotFrame, child, parent)
	,m_targetAngle(0.0f)
	,m_maxTorque(D_CUSTOM_LARGE_VALUE)
{
	m_friction = 0.0f;
	dAssert(m_options.m_value == 0);
	SetAngularRate(dPi);
	SetMinAngularLimit(-180.0f * dDegreeToRad);
	SetMaxAngularLimit(180.0f * dDegreeToRad);
}

ndJointHingeActuator::ndJointHingeActuator(const dMatrix& pinAndPivotFrame, dFloat32 angularRate, dFloat32 minAngle, dFloat32 maxAngle, NewtonBody* const child, NewtonBody* const parent)
	:dCustomHinge (pinAndPivotFrame, child, parent)
	,m_targetAngle(0.0f)
	,m_maxTorque(D_CUSTOM_LARGE_VALUE)
{
	m_friction = 0.0f;
	dAssert(m_options.m_value == 0);
	SetAngularRate(angularRate);
	SetMinAngularLimit(minAngle);
	SetMaxAngularLimit(maxAngle);
}

ndJointHingeActuator::~ndJointHingeActuator()
{
}

dFloat32 ndJointHingeActuator::GetTargetAngle() const
{
	return GetJointAngle();
}

dFloat32 ndJointHingeActuator::GetMinAngularLimit() const
{
	return m_minAngle;
}

dFloat32 ndJointHingeActuator::GetMaxAngularLimit() const
{
	return m_maxAngle;
}

dFloat32 ndJointHingeActuator::GetAngularRate() const
{
	return m_motorSpeed;
}


void ndJointHingeActuator::SetMinAngularLimit(dFloat32 limit)
{
	m_minAngle = limit;
}

void ndJointHingeActuator::SetMaxAngularLimit(dFloat32 limit)
{
	m_maxAngle = limit;
}


void ndJointHingeActuator::SetAngularRate(dFloat32 rate)
{
	EnableMotor(false, rate);
}

void ndJointHingeActuator::SetTargetAngle(dFloat32 angle)
{
	angle = dClamp (angle, m_minAngle, m_maxAngle);
	if (dAbs (angle - m_targetAngle.GetAngle()) > dFloat32 (1.0e-3f)) {
		NewtonBodySetSleepState(m_body0, 0);
		m_targetAngle.SetAngle (dClamp (angle, m_minAngle, m_maxAngle));
	}
}

dFloat32 ndJointHingeActuator::GetActuatorAngle() const
{
	return GetJointAngle();
}

dFloat32 ndJointHingeActuator::GetMaxTorque() const
{
    return m_maxTorque;
}

void ndJointHingeActuator::SetMaxTorque(dFloat32 torque)
{
    m_maxTorque = dAbs (torque);
}

void ndJointHingeActuator::SubmitAngularRow(const dMatrix& matrix0, const dMatrix& matrix1, dFloat32 timestep)
{
	// make sure not other option is activated
	m_options.m_value = 0;

	dCustomHinge::SubmitAngularRow(matrix0, matrix1, timestep);

	dAssert(m_motorSpeed >= 0.0f);
	const dFloat32 angle = m_curJointAngle.GetAngle();
	const dFloat32 targetAngle = m_targetAngle.GetAngle();

	dFloat32 invTimeStep = 1.0f / timestep;
	dFloat32 step = m_motorSpeed * timestep;
	dFloat32 currentSpeed = 0.0f;

	if (angle < (targetAngle - step)) {
		currentSpeed = m_motorSpeed;
	} else if (angle < targetAngle) {
		currentSpeed = 0.3f * (targetAngle - angle) * invTimeStep;
	} else if (angle > (targetAngle + step)) {
		currentSpeed = -m_motorSpeed;
	} else if (angle > targetAngle) {
		currentSpeed = 0.3f * (targetAngle - angle) * invTimeStep;
	}

	NewtonUserJointAddAngularRow(m_joint, 0.0f, &matrix0.m_front[0]);
	dFloat32 accel = NewtonUserJointCalculateRowZeroAcceleration(m_joint) + currentSpeed * invTimeStep;
	NewtonUserJointSetRowAcceleration(m_joint, accel);
	NewtonUserJointSetRowStiffness(m_joint, m_stiffness);
	if (angle > GetMaxAngularLimit()) {
		NewtonUserJointSetRowMaximumFriction(m_joint, m_maxTorque);
	} else if (angle < GetMinAngularLimit()) {
		NewtonUserJointSetRowMinimumFriction(m_joint, -m_maxTorque);
	} else {
		NewtonUserJointSetRowMinimumFriction(m_joint, -m_maxTorque);
		NewtonUserJointSetRowMaximumFriction(m_joint, m_maxTorque);
	}
}
*/