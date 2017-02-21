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

// CustomSlider.cpp: implementation of the CustomSlider class.
//
//////////////////////////////////////////////////////////////////////
#include "CustomJointLibraryStdAfx.h"
#include "CustomSliderActuator.h"


IMPLEMENT_CUSTOM_JOINT(CustomSliderActuator);

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

CustomSliderActuator::CustomSliderActuator (const dMatrix& pinAndPivotFrame, NewtonBody* const child, NewtonBody* const parent)
	:CustomSlider(pinAndPivotFrame, child, parent)
	,m_posit(0.0f)
	,m_minPosit(-D_CUSTOM_LARGE_VALUE)
	,m_maxPosit( D_CUSTOM_LARGE_VALUE)
	,m_linearRate(0.0f)
	,m_maxForce(1.0e20f)
{
	m_actuatorFlag = false;
	EnableLimits(false);
}

CustomSliderActuator::CustomSliderActuator (const dMatrix& pinAndPivotFrame, dFloat speed, dFloat minPosit, dFloat maxPosit, NewtonBody* const child, NewtonBody* const parent)
	:CustomSlider(pinAndPivotFrame, child, parent)
	,m_posit(0.0f)
	,m_minPosit(minPosit)
	,m_maxPosit(maxPosit)
	,m_linearRate(speed)
    ,m_maxForce(D_CUSTOM_LARGE_VALUE)
{
	m_actuatorFlag = true;
	EnableLimits(false);
}

CustomSliderActuator::CustomSliderActuator(NewtonBody* const child, NewtonBody* const parent, NewtonDeserializeCallback callback, void* const userData)
	:CustomSlider(child, parent, callback, userData)
{
	callback(userData, &m_posit, sizeof(dFloat));
	callback(userData, &m_minPosit, sizeof(dFloat));
	callback(userData, &m_maxPosit, sizeof(dFloat));
	callback(userData, &m_linearRate, sizeof(dFloat));
	callback(userData, &m_maxForce, sizeof(dFloat));
}

void CustomSliderActuator::Serialize(NewtonSerializeCallback callback, void* const userData) const
{
	CustomSlider::Serialize(callback, userData);

	callback(userData, &m_posit, sizeof(dFloat));
	callback(userData, &m_minPosit, sizeof(dFloat));
	callback(userData, &m_maxPosit, sizeof(dFloat));
	callback(userData, &m_linearRate, sizeof(dFloat));
	callback(userData, &m_maxForce, sizeof(dFloat));
}


CustomSliderActuator::~CustomSliderActuator()
{
}


bool CustomSliderActuator::GetEnableFlag () const
{
	return m_actuatorFlag;
}

dFloat CustomSliderActuator::GetTargetPosit() const
{
	return m_posit;
}

dFloat CustomSliderActuator::GetLinearRate() const
{
	return m_linearRate;
}


dFloat CustomSliderActuator::GetMinPositLimit() const
{
	return m_minPosit;
}

dFloat CustomSliderActuator::GetMaxPositLimit() const
{
	return m_maxPosit;
}


void CustomSliderActuator::SetTargetPosit(dFloat posit)
{
	m_posit = dClamp (posit, m_minPosit, m_maxPosit);
}


void CustomSliderActuator::SetMinPositLimit(dFloat limit)
{
	m_minPosit = limit;
}

void CustomSliderActuator::SetMaxPositLimit(dFloat limit)
{
	m_maxPosit = limit;
}

void CustomSliderActuator::SetLinearRate(dFloat rate)
{
	m_linearRate = rate;
}

void CustomSliderActuator::SetEnableFlag (bool flag)
{
	m_actuatorFlag = flag;
}


dFloat CustomSliderActuator::GetActuatorPosit() const
{
	return GetJointPosit();
}

dFloat CustomSliderActuator::GetMaxForcePower() const
{
    return m_maxForce;
}

void CustomSliderActuator::SetMaxForcePower(dFloat force)
{
    m_maxForce = dAbs (force);
}


void CustomSliderActuator::GetInfo (NewtonJointRecord* const info) const
{
	dAssert (0);
}

void CustomSliderActuator::SubmitConstraintsFreeDof(dFloat timestep, const dMatrix& matrix0, const dMatrix& matrix1)
{
	if (m_actuatorFlag) {
		dVector posit1 (matrix1.m_posit);
		dVector posit0 (matrix0.m_posit - matrix1.m_front.Scale (m_posit));
		dFloat jointosit = GetJointPosit();
		dFloat relPosit = m_posit - jointosit;
		dFloat step = dFloat(2.0f) * m_linearRate * timestep;

		dFloat currentSpeed = GetJointSpeed();
		dFloat desiredSpeed = (dAbs(relPosit) > dAbs(step)) ? dSign(relPosit) * m_linearRate : dFloat(0.1f) * relPosit / timestep;
		dFloat accel = (desiredSpeed - currentSpeed) / timestep;

		NewtonUserJointAddLinearRow(m_joint, &posit0[0], &posit1[0], &matrix1.m_front[0]);
		NewtonUserJointSetRowAcceleration(m_joint, accel);
        NewtonUserJointSetRowMinimumFriction (m_joint, -m_maxForce);
        NewtonUserJointSetRowMaximumFriction (m_joint,  m_maxForce);
		NewtonUserJointSetRowStiffness (m_joint, 1.0f);
	} else {
		CustomSlider::SubmitConstraintsFreeDof (timestep, matrix0, matrix1);
	}
}


