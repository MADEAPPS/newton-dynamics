/* Copyright (c) <2003-2011> <Julio Jerez, Newton Game Dynamics>
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

#include "CustomJointLibraryStdAfx.h"
#include "CustomHingeActuator.h"

CustomHingeActuator::CustomHingeActuator(const dMatrix& pinAndPivotFrame, NewtonBody* const child, NewtonBody* const parent)
	:CustomHinge (pinAndPivotFrame, child, parent)
	,m_angle(0.0f)
	,m_minAngle(-D_CUSTOM_LARGE_VALUE)
	,m_maxAngle( D_CUSTOM_LARGE_VALUE)
	,m_angularRate(0.0f)
	,m_maxForce(1.0e20f)
	,m_flag(false)
{
	EnableLimits(false);
}

CustomHingeActuator::CustomHingeActuator(const dMatrix& pinAndPivotFrame, dFloat angularRate, dFloat minAngle, dFloat maxAngle, NewtonBody* const child, NewtonBody* const parent)
	:CustomHinge (pinAndPivotFrame, child, parent)
	,m_angle(0.0f)
	,m_minAngle(minAngle)
	,m_maxAngle(maxAngle)
	,m_angularRate(angularRate)
    ,m_maxForce(1.0e20f)
	,m_flag(true)
{
	EnableLimits(false);
}

CustomHingeActuator::~CustomHingeActuator()
{
}

bool CustomHingeActuator::GetEnableFlag () const
{
	return m_flag;
}

dFloat CustomHingeActuator::GetTargetAngle() const
{
	return m_angle;
}

dFloat CustomHingeActuator::GetMinAngularLimit() const
{
	return m_minAngle;
}

dFloat CustomHingeActuator::GetMaxAngularLimit() const
{
	return m_maxAngle;
}

dFloat CustomHingeActuator::GetAngularRate() const
{
	return m_angularRate;
}


void CustomHingeActuator::SetMinAngularLimit(dFloat limit)
{
	m_minAngle = limit;
}

void CustomHingeActuator::SetMaxAngularLimit(dFloat limit)
{
	m_maxAngle = limit;
}


void CustomHingeActuator::SetAngularRate(dFloat rate)
{
	m_angularRate = rate;
}

void CustomHingeActuator::SetTargetAngle(dFloat angle)
{
	m_angle = dClamp (angle, m_minAngle, m_maxAngle);
}

void CustomHingeActuator::SetEnableFlag (bool flag)
{
	m_flag = flag;
}

dFloat CustomHingeActuator::GetActuatorAngle() const
{
	return GetJointAngle();
}

dFloat CustomHingeActuator::GetMaxForcePower() const
{
    return m_maxForce;
}

void CustomHingeActuator::SetMaxForcePower(dFloat force)
{
    m_maxForce = dAbs (force);
}


void CustomHingeActuator::GetInfo (NewtonJointRecord* const info) const
{
	dAssert (0);
}


void CustomHingeActuator::SubmitConstraintsFreeDof (dFloat timestep, const dMatrix& matrix0, const dMatrix& matrix1)
{
	if (m_flag) {
		dFloat jointangle = GetActuatorAngle();
		dFloat relAngle = jointangle - m_angle;
		NewtonUserJointAddAngularRow (m_joint, -relAngle, &matrix0.m_front[0]);

		dFloat step = m_angularRate * timestep;
		if (dAbs (relAngle) > 2.0f * dAbs (step)) {
			dFloat desiredSpeed = -dSign(relAngle) * m_angularRate;
			dFloat currentSpeed = GetJointOmega ();
			dFloat accel = (desiredSpeed - currentSpeed) / timestep;
			NewtonUserJointSetRowAcceleration (m_joint, accel);
		}
        NewtonUserJointSetRowMinimumFriction (m_joint, -m_maxForce);
        NewtonUserJointSetRowMaximumFriction (m_joint,  m_maxForce);
		NewtonUserJointSetRowStiffness (m_joint, 1.0f);
	} else {
		CustomHinge::SubmitConstraintsFreeDof (timestep, matrix0, matrix1);
	}
}


