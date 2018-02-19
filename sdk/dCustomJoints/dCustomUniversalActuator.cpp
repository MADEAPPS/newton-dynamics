/* Copyright (c) <2003-2016> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/


// dCustomUniversal.cpp: implementation of the dCustomUniversal class.
//
//////////////////////////////////////////////////////////////////////
#include "dCustomJointLibraryStdAfx.h"
#include "dCustomUniversalActuator.h"

IMPLEMENT_CUSTOM_JOINT(dCustomUniversalActuator);

dCustomUniversalActuator::dCustomUniversalActuator(const dMatrix& pinAndPivotFrame, NewtonBody* const child, NewtonBody* const parent)
	:dCustomUniversal(pinAndPivotFrame, child, parent)
	,m_targetAngle0(0.0f)
	,m_targetAngle1(0.0f)
	,m_angularRate1(0.0f)
	,m_maxTorque0(D_CUSTOM_LARGE_VALUE)
	,m_maxTorque1(D_CUSTOM_LARGE_VALUE)
{
	m_friction = 0.0f;
	m_options.m_value = 0;
	SetHardMiddleAxis(true);
}

dCustomUniversalActuator::dCustomUniversalActuator (const dMatrix& pinAndPivotFrame, dFloat angularRate0, dFloat minAngle0, dFloat maxAngle0, dFloat angularRate1, dFloat minAngle1, dFloat maxAngle1, NewtonBody* const child, NewtonBody* const parent)
	:dCustomUniversal(pinAndPivotFrame, child, parent)
	,m_targetAngle0(0.0f)
	,m_targetAngle1(0.0f)
	,m_angularRate1(0.0f)
	,m_maxTorque0(D_CUSTOM_LARGE_VALUE)
	,m_maxTorque1(D_CUSTOM_LARGE_VALUE)
{
	m_friction = 0.0f;
	m_options.m_value = 0;
	SetHardMiddleAxis(true);
}


dCustomUniversalActuator::~dCustomUniversalActuator()
{
}

void dCustomUniversalActuator::Deserialize(NewtonDeserializeCallback callback, void* const userData)
{
	callback(userData, &m_targetAngle0, sizeof(dAngularIntegration));
	callback(userData, &m_targetAngle1, sizeof(dAngularIntegration));
	callback(userData, &m_angularRate1, sizeof(dFloat));
	callback(userData, &m_maxTorque0, sizeof(dFloat));
	callback(userData, &m_maxTorque1, sizeof(dFloat));
}

void dCustomUniversalActuator::Serialize(NewtonSerializeCallback callback, void* const userData) const 
{ 
	dCustomUniversal::Serialize(callback, userData);

	callback(userData, &m_targetAngle0, sizeof(dAngularIntegration));
	callback(userData, &m_targetAngle1, sizeof(dAngularIntegration));
	callback(userData, &m_angularRate1, sizeof(dFloat));
	callback(userData, &m_maxTorque0, sizeof(dFloat));
	callback(userData, &m_maxTorque1, sizeof(dFloat));
}

dFloat dCustomUniversalActuator::GetTargetAngle1() const
{
	return GetJointAngle();
}

dFloat dCustomUniversalActuator::GetMinAngularLimit1() const
{
	return m_minAngle;
}

dFloat dCustomUniversalActuator::GetMaxAngularLimit1() const
{
	return m_maxAngle;
}

dFloat dCustomUniversalActuator::GetAngularRate1() const
{
	return m_motorSpeed;
}


void dCustomUniversalActuator::SetMinAngularLimit1(dFloat limit)
{
	m_minAngle = limit;
}

void dCustomUniversalActuator::SetMaxAngularLimit1(dFloat limit)
{
	m_maxAngle = limit;
}

void dCustomUniversalActuator::SetAngularRate1(dFloat rate)
{
	EnableMotor(false, rate);
}

void dCustomUniversalActuator::SetTargetAngle1(dFloat angle)
{
//	m_targetAngle1.SetAngle(dClamp(angle, m_minAngle1, m_maxAngle1));
}

dFloat dCustomUniversalActuator::GetActuatorAngle1() const
{
	return GetJointAngle();
}

dFloat dCustomUniversalActuator::GetMaxTorque1() const
{
	return m_maxTorque1;
}

void dCustomUniversalActuator::SetMaxTorque1(dFloat torque)
{
	m_maxTorque1 = dAbs(torque);
}


void dCustomUniversalActuator::SubmitAngularRow(const dMatrix& matrix0, const dMatrix& matrix1, const dVector& eulers, dFloat timestep)
{
	dCustomUniversal::SubmitAngularRow(matrix0, matrix1, eulers, timestep);
/*
	dFloat jointAngle0 = GetJointAngle();
	dFloat targetAngle0 = m_targetAngle0.GetAngle();
	dFloat relAngle0 = jointAngle0 - targetAngle0;
	dFloat currentSpeed0 = GetJointOmega();
	dFloat step0 = dFloat(2.0f) * m_motorSpeed * timestep;

	dFloat desiredSpeed0 = (dAbs(relAngle0) > dAbs(step0)) ? -dSign(relAngle0) * m_motorSpeed : -dFloat(0.1f) * relAngle0 / timestep;
//desiredSpeed0 = 5.0f;
	dFloat accel0 = (desiredSpeed0 - currentSpeed0) / timestep;
//dTrace(("%f %f\n", desiredSpeed0, currentSpeed0));

	NewtonUserJointAddAngularRow(m_joint, 0.0f, &matrix0.m_front[0]);
	NewtonUserJointSetRowAcceleration(m_joint, accel0);
	NewtonUserJointSetRowMinimumFriction(m_joint, -m_maxTorque0);
	NewtonUserJointSetRowMaximumFriction(m_joint, m_maxTorque0);
	NewtonUserJointSetRowStiffness(m_joint, m_stiffness);
*/

/*
	if (m_actuator_0 | m_actuator_1){

		if (m_actuator_1) {
			dFloat jointAngle = GetJointAngle_1();
			dFloat relAngle = jointAngle - m_angle1;

			dFloat currentSpeed = GetJointOmega_1();
			dFloat step = dFloat(2.0f) * m_angularRate1 * timestep;
			dFloat desiredSpeed = (dAbs(relAngle) > dAbs(step)) ? dSign(relAngle) * m_angularRate1 : dFloat(0.1f) * relAngle / timestep;
			dFloat accel = (desiredSpeed - currentSpeed) / timestep;

			NewtonUserJointAddAngularRow(m_joint, relAngle, &matrix1.m_up[0]);
			NewtonUserJointSetRowAcceleration(m_joint, accel);

            NewtonUserJointSetRowMinimumFriction (m_joint, -m_maxForce1);
            NewtonUserJointSetRowMaximumFriction (m_joint,  m_maxForce1);
			NewtonUserJointSetRowStiffness (m_joint, 1.0f);
		}
	}
*/

	dFloat jointAngle1 = GetJointAngle2();
	dFloat relAngle1 = jointAngle1 - m_targetAngle1.GetAngle();

	dFloat currentSpeed1 = GetJointOmega2();
	dFloat step1 = dFloat(2.0f) * m_angularRate1 * timestep;
	dFloat desiredSpeed1 = (dAbs(relAngle1) > dAbs(step1)) ? dSign(relAngle1) * m_angularRate1 : dFloat(0.1f) * relAngle1 / timestep;

desiredSpeed1 = 5.0f;
	dFloat accel1 = (desiredSpeed1 - currentSpeed1) / timestep;

	NewtonUserJointAddAngularRow(m_joint, relAngle1, &matrix1.m_up[0]);
	NewtonUserJointSetRowAcceleration(m_joint, accel1);
	NewtonUserJointSetRowMinimumFriction(m_joint, -m_maxTorque1);
	NewtonUserJointSetRowMaximumFriction(m_joint, m_maxTorque1);
	NewtonUserJointSetRowStiffness(m_joint, m_stiffness);
}


