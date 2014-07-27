/* Copyright (c) <2009> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/



// CustomUniversal.cpp: implementation of the CustomUniversal class.
//
//////////////////////////////////////////////////////////////////////
#include "CustomJointLibraryStdAfx.h"
#include "CustomUniversalActuator.h"



CustomUniversalActuator::CustomUniversalActuator (const dMatrix& pinAndPivotFrame, dFloat angularRate0, dFloat minAngle0, dFloat maxAngle0, dFloat angularRate1, dFloat minAngle1, dFloat maxAngle1, NewtonBody* const child, NewtonBody* const parent)
	:CustomUniversal(pinAndPivotFrame, child, parent)
	,m_angle0(0.0f)
	,m_minAngle0(minAngle0)
	,m_maxAngle0(maxAngle0)
	,m_angularRate0(angularRate0)
    ,m_maxForce0(1.0e10f)
	,m_angle1(0.0f)
	,m_minAngle1(minAngle1)
	,m_maxAngle1(maxAngle1)
	,m_angularRate1(angularRate1)
    ,m_maxForce1(1.0e10f)
	,m_flag0(true)
	,m_flag1(true)
{
	EnableLimit_0(false);
	EnableLimit_1(false);
}

CustomUniversalActuator::~CustomUniversalActuator()
{
}

bool CustomUniversalActuator::GetEnableFlag0 () const
{
	return m_flag0;
}

dFloat CustomUniversalActuator::GetTargetAngle0() const
{
	return m_angle0;
}

dFloat CustomUniversalActuator::GetAngularRate0() const
{
	return m_angularRate0;
}

dFloat CustomUniversalActuator::GetMinAngularLimit0() const
{
	return m_minAngle0;
}

dFloat CustomUniversalActuator::GetMaxAngularLimit0() const
{
	return m_maxAngle0;
}

bool CustomUniversalActuator::GetEnableFlag1 () const
{
	return m_flag1;
}

dFloat CustomUniversalActuator::GetTargetAngle1() const
{
	return m_angle1;
}

dFloat CustomUniversalActuator::GetAngularRate1() const
{
	return m_angularRate1;
}

dFloat CustomUniversalActuator::GetMinAngularLimit1() const
{
	return m_minAngle1;
}

dFloat CustomUniversalActuator::GetMaxAngularLimit1() const
{
	return m_maxAngle1;
}


void CustomUniversalActuator::SetEnableFlag0 (bool flag)
{
	m_flag0 = flag;
}

void CustomUniversalActuator::SetTargetAngle0(dFloat angle)
{
	m_angle0 = dClamp (angle, m_minAngle0, m_maxAngle0);
}

void CustomUniversalActuator::SetMinAngularLimit0(dFloat limit)
{
	m_minAngle0 = limit;
}

void CustomUniversalActuator::SetMaxAngularLimit0(dFloat limit)
{
	m_maxAngle0 = limit;
}

dFloat CustomUniversalActuator::GetMaxForcePower0() const
{
    return m_maxForce0;
}

void CustomUniversalActuator::SetMaxForcePower0(dFloat force)
{
    m_maxForce0 = dAbs (force);
}


void CustomUniversalActuator::SetAngularRate0(dFloat rate)
{
	m_angularRate0 = rate;
}

void CustomUniversalActuator::SetEnableFlag1 (bool flag)
{
	m_flag1 = flag;
}

void CustomUniversalActuator::SetTargetAngle1(dFloat angle)
{
	m_angle1 = dClamp (angle, m_minAngle1, m_maxAngle1);
}


void CustomUniversalActuator::SetAngularRate1(dFloat rate)
{
	m_angularRate1 = rate;
}

void CustomUniversalActuator::SetMinAngularLimit1(dFloat limit)
{
	m_minAngle1 = limit;
}

void CustomUniversalActuator::SetMaxAngularLimit1(dFloat limit)
{
	m_maxAngle1 = limit;
}

dFloat CustomUniversalActuator::GetActuatorAngle0() const
{
	return GetJointAngle_0();
}

dFloat CustomUniversalActuator::GetActuatorAngle1() const
{
	return GetJointAngle_1();
}

dFloat CustomUniversalActuator::GetMaxForcePower1() const
{
    return m_maxForce1;
}

void CustomUniversalActuator::SetMaxForcePower1(dFloat force)
{
    m_maxForce1 = dAbs (force);
}



void CustomUniversalActuator::GetInfo (NewtonJointRecord* const info) const
{
	dAssert (0);
}

void CustomUniversalActuator::SubmitConstraints (dFloat timestep, int threadIndex)
{
	CustomUniversal::SubmitConstraints (timestep, threadIndex);

	if (m_flag0 | m_flag1){
		dMatrix matrix0;
		dMatrix matrix1;

		CalculateGlobalMatrix (matrix0, matrix1);
		if (m_flag0) {
			dFloat jointAngle = GetJointAngle_0();
			dFloat relAngle = jointAngle - m_angle0;
			NewtonUserJointAddAngularRow (m_joint, relAngle, &matrix0.m_front[0]);
			dFloat step = m_angularRate0 * timestep;
			if (dAbs (relAngle) > 2.0f * dAbs (step)) {
				dFloat desiredSpeed = dSign(relAngle) * m_angularRate0;
				dFloat currentSpeed = GetJointOmega_0 ();
				dFloat accel = (desiredSpeed - currentSpeed) / timestep;
				NewtonUserJointSetRowAcceleration (m_joint, accel);
			}
            NewtonUserJointSetRowMinimumFriction (m_joint, -m_maxForce0);
            NewtonUserJointSetRowMaximumFriction (m_joint,  m_maxForce0);
			NewtonUserJointSetRowStiffness (m_joint, 1.0f);
		}

		if (m_flag1) {
			dFloat jointAngle = GetJointAngle_1();
			dFloat relAngle = jointAngle - m_angle1;
			NewtonUserJointAddAngularRow (m_joint, relAngle, &matrix1.m_up[0]);
			dFloat step = m_angularRate1 * timestep;
			if (dAbs (relAngle) > 2.0f * dAbs (step)) {
				dFloat desiredSpeed = dSign(relAngle) * m_angularRate1;
				dFloat currentSpeed = GetJointOmega_1 ();
				dFloat accel = (desiredSpeed - currentSpeed) / timestep;
				NewtonUserJointSetRowAcceleration (m_joint, accel);
			}
            NewtonUserJointSetRowMinimumFriction (m_joint, -m_maxForce1);
            NewtonUserJointSetRowMaximumFriction (m_joint,  m_maxForce1);
			NewtonUserJointSetRowStiffness (m_joint, 1.0f);
		}
	}
}


