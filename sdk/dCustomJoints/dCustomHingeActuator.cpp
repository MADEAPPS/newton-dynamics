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
	,m_angle(0.0f)
	,m_minAngle(-D_CUSTOM_LARGE_VALUE)
	,m_maxAngle( D_CUSTOM_LARGE_VALUE)
	,m_angularRate(0.0f)
	,m_maxForce(1.0e20f)
{
	dAssert (0);
//	m_actuatorFlag = false;
	EnableLimits(false);
}

dCustomHingeActuator::dCustomHingeActuator(const dMatrix& pinAndPivotFrame, dFloat angularRate, dFloat minAngle, dFloat maxAngle, NewtonBody* const child, NewtonBody* const parent)
	:dCustomHinge (pinAndPivotFrame, child, parent)
	,m_angle(0.0f)
	,m_minAngle(minAngle)
	,m_maxAngle(maxAngle)
	,m_angularRate(angularRate)
    ,m_maxForce(1.0e20f)
{
	dAssert (0);
//	m_actuatorFlag = true;
	EnableLimits(false);
}

dCustomHingeActuator::~dCustomHingeActuator()
{
}

void dCustomHingeActuator::Serialize(NewtonSerializeCallback callback, void* const userData) const
{
	dAssert (0);
}

void dCustomHingeActuator::Deserialize(NewtonDeserializeCallback callback, void* const userData)
{
	dAssert (0);
}


bool dCustomHingeActuator::GetEnableFlag () const
{
	dAssert (0);
	return 0;
//	return m_actuatorFlag;
}

dFloat dCustomHingeActuator::GetTargetAngle() const
{
	return m_angle;
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
	return m_angularRate;
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
	m_angularRate = rate;
}

void dCustomHingeActuator::SetTargetAngle(dFloat angle)
{
	m_angle = dClamp (angle, m_minAngle, m_maxAngle);
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

dFloat dCustomHingeActuator::GetMaxForcePower() const
{
    return m_maxForce;
}

void dCustomHingeActuator::SetMaxForcePower(dFloat force)
{
    m_maxForce = dAbs (force);
}


void dCustomHingeActuator::SubmitConstraintsFreeDof (dFloat timestep, const dMatrix& matrix0, const dMatrix& matrix1)
{
	dAssert (0);
/*
	if (m_actuatorFlag) {
		dFloat jointangle = GetActuatorAngle();
		dFloat relAngle = jointangle - m_angle;
		dFloat currentSpeed = GetJointOmega();
		dFloat step = dFloat(2.0f) * m_angularRate * timestep;
		dFloat desiredSpeed = (dAbs(relAngle) > dAbs(step)) ? -dSign(relAngle) * m_angularRate : -dFloat(0.1f) * relAngle / timestep;
		dFloat accel = (desiredSpeed - currentSpeed) / timestep;
		NewtonUserJointAddAngularRow(m_joint, relAngle, &matrix0.m_front[0]);
		NewtonUserJointSetRowAcceleration(m_joint, accel);
        NewtonUserJointSetRowMinimumFriction (m_joint, -m_maxForce);
        NewtonUserJointSetRowMaximumFriction (m_joint,  m_maxForce);
		NewtonUserJointSetRowStiffness (m_joint, 1.0f);
	} else {
		dCustomHinge::SubmitConstraintsFreeDof (timestep, matrix0, matrix1);
	}
*/
}


