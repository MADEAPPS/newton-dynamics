/* Copyright (c) <2003-2016> <Julio Jerez, Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 
* 3. This notice may not be removed or altered from any source distribution.
*/

#include "dCustomJointLibraryStdAfx.h"
#include "dCustomHingeActuator.h"

IMPLEMENT_CUSTOM_JOINT(dCustomHingeActuator);

dCustomHingeActuator::dCustomHingeActuator(const dMatrix& pinAndPivotFrame, NewtonBody* const child, NewtonBody* const parent)
	:dCustomHinge (pinAndPivotFrame, child, parent)
	,m_targetAngle(0.0f)
	,m_maxToque(D_CUSTOM_LARGE_VALUE)
{
	m_friction = 0.0f;
	m_options.m_value = 0;
	SetAngularRate(dPi);
	SetMinAngularLimit(-180.0f * dDegreeToRad);
	SetMaxAngularLimit(180.0f * dDegreeToRad);
}

dCustomHingeActuator::dCustomHingeActuator(const dMatrix& pinAndPivotFrame, dFloat angularRate, dFloat minAngle, dFloat maxAngle, NewtonBody* const child, NewtonBody* const parent)
	:dCustomHinge (pinAndPivotFrame, child, parent)
	,m_targetAngle(0.0f)
	,m_maxToque(D_CUSTOM_LARGE_VALUE)
{
	m_friction = 0.0f;
	m_options.m_value = 0;
	SetAngularRate(angularRate);
	SetMinAngularLimit(minAngle);
	SetMaxAngularLimit(maxAngle);
}

dCustomHingeActuator::~dCustomHingeActuator()
{
}

void dCustomHingeActuator::Deserialize(NewtonDeserializeCallback callback, void* const userData)
{
	callback(userData, &m_targetAngle, sizeof(dAngularIntegration));
	callback(userData, &m_maxToque, sizeof(dFloat));
}

void dCustomHingeActuator::Serialize(NewtonSerializeCallback callback, void* const userData) const
{
	dCustomHinge::Serialize (callback, userData);
	callback(userData, &m_targetAngle, sizeof(dAngularIntegration));
	callback(userData, &m_maxToque, sizeof(dFloat));
}

bool dCustomHingeActuator::GetEnableFlag () const
{
	dAssert (0);
	return 0;
//	return m_actuatorFlag;
}

dFloat dCustomHingeActuator::GetTargetAngle() const
{
	return GetJointAngle();
}

dFloat dCustomHingeActuator::GetMinAngularLimit() const
{
	return m_minAngle;
}

dFloat dCustomHingeActuator::GetMaxAngularLimit() const
{
	return m_maxAngle;
}

dFloat dCustomHingeActuator::GetAngularRate() const
{
	return m_motorSpeed;
}


void dCustomHingeActuator::SetMinAngularLimit(dFloat limit)
{
	m_minAngle = limit;
}

void dCustomHingeActuator::SetMaxAngularLimit(dFloat limit)
{
	m_maxAngle = limit;
}


void dCustomHingeActuator::SetAngularRate(dFloat rate)
{
	EnableMotor(false, rate);
}

void dCustomHingeActuator::SetTargetAngle(dFloat angle)
{
	m_targetAngle.SetAngle (dClamp (angle, m_minAngle, m_maxAngle));
}

void dCustomHingeActuator::SetEnableFlag (bool flag)
{
	dAssert (0);
//	m_actuatorFlag = flag;
}

dFloat dCustomHingeActuator::GetActuatorAngle() const
{
	return GetJointAngle();
}

dFloat dCustomHingeActuator::GetMaxTorque() const
{
    return m_maxToque;
}

void dCustomHingeActuator::SetMaxTorque(dFloat torque)
{
    m_maxToque = dAbs (torque);
}


void dCustomHingeActuator::SubmitAngularRow(const dMatrix& matrix0, const dMatrix& matrix1, const dVector& eulers, dFloat timestep)
{
	dCustomHinge::SubmitAngularRow(matrix0, matrix1, eulers, timestep);

	dFloat jointAngle = GetActuatorAngle();
	dFloat targetAngle = m_targetAngle.GetAngle();
	dFloat relAngle = jointAngle - targetAngle;
	dFloat currentSpeed = GetJointOmega();
	dFloat step = dFloat(2.0f) * m_motorSpeed * timestep;

	dFloat desiredSpeed = (dAbs(relAngle) > dAbs(step)) ? -dSign(relAngle) * m_motorSpeed : -dFloat(0.1f) * relAngle / timestep;
	dFloat accel = (desiredSpeed - currentSpeed) / timestep;

	NewtonUserJointAddAngularRow(m_joint, relAngle, &matrix0.m_front[0]);
	NewtonUserJointSetRowAcceleration(m_joint, accel);
	NewtonUserJointSetRowMinimumFriction (m_joint, -m_maxToque);
	NewtonUserJointSetRowMaximumFriction (m_joint,  m_maxToque);
	NewtonUserJointSetRowStiffness (m_joint, m_stiffness);
}


