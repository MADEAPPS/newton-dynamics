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


CustomHingeActuator::CustomHingeActuator(const dMatrix& pinAndPivotFrame, dFloat angularRate, dFloat minAngle, dFloat maxAngle, NewtonBody* const child, NewtonBody* const parent)
	:CustomHinge (pinAndPivotFrame, child, parent)
	,m_angle(0.0f)
	,m_minAngle(minAngle)
	,m_maxAngle(maxAngle)
	,m_angularRate(angularRate)
	,m_flag(true)
{
	EnableLimits(false);
}

CustomHingeActuator::~CustomHingeActuator()
{
}

bool CustomHingeActuator::GetEnableFlag (bool flag) const
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

void CustomHingeActuator::GetInfo (NewtonJointRecord* const info) const
{
	dAssert (0);
}


void CustomHingeActuator::SubmitConstraints (dFloat timestep, int threadIndex)
{
	CustomHinge::SubmitConstraints (timestep, threadIndex);

	if (m_flag) {
		dMatrix matrix0;
		dMatrix matrix1;

		CalculateGlobalMatrix (matrix0, matrix1);
		dFloat jointangle = GetJointAngle();
		dFloat relAngle = m_angle - jointangle;
		NewtonUserJointAddAngularRow (m_joint, relAngle, &matrix0.m_front[0]);

		dFloat step = m_angularRate * timestep;
		if (dAbs (relAngle) > 2.0f * dAbs (step)) {
			dFloat speed0 = dClamp (relAngle / timestep, -m_angularRate, m_angularRate);
			dFloat speed1 = GetJointOmega ();
			dFloat accel = (speed0 - speed1) / timestep;
			NewtonUserJointSetRowAcceleration (m_joint, accel);

dVector xxx (matrix0.m_posit - matrix1.m_posit);
dTrace (("%f %f %f\n", xxx.m_x, xxx.m_y, xxx.m_z))

		}
		NewtonUserJointSetRowStiffness (m_joint, 1.0f);
	}
}


