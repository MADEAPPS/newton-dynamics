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



// CustomSlider.cpp: implementation of the CustomSlider class.
//
//////////////////////////////////////////////////////////////////////
#include "CustomJointLibraryStdAfx.h"
#include "CustomSliderActuator.h"

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////


CustomSliderActuator::CustomSliderActuator (const dMatrix& pinAndPivotFrame, dFloat speed, dFloat minPosit, dFloat maxPosit, NewtonBody* const child, NewtonBody* const parent)
	:CustomSlider(pinAndPivotFrame, child, parent)
	,m_linearRate(speed)
	,m_posit(0.0f)
	,m_minPosit(minPosit)
	,m_maxPosit(maxPosit)
	,m_flag(true)
{
}

CustomSliderActuator::~CustomSliderActuator()
{
}


bool CustomSliderActuator::GetEnableFlag (bool flag) const
{
	return m_flag;
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
	m_flag = flag;
}


dFloat CustomSliderActuator::GetActuatorPosit() const
{
	return GetJointPosit();
}


void CustomSliderActuator::GetInfo (NewtonJointRecord* const info) const
{
	dAssert (0);
}

void CustomSliderActuator::SubmitConstraints (dFloat timestep, int threadIndex)
{
	CustomSlider::SubmitConstraints (timestep, threadIndex);

	if (m_flag) {
		dMatrix matrix0;
		dMatrix matrix1;

		CalculateGlobalMatrix (matrix0, matrix1);

		dVector posit1 (matrix1.m_posit);
		dVector posit0 (matrix0.m_posit - matrix1.m_front.Scale (m_posit));
		NewtonUserJointAddLinearRow (m_joint, &posit0[0], &posit1[0], &matrix1.m_front[0]);

		dFloat relPosit = m_posit - GetJointPosit();
		dFloat step = m_linearRate * timestep;
		if (dAbs (relPosit) > 2.0f * dAbs (step)) {
//			dFloat speed = GetJointSpeed ();
//			dFloat accel = (relPosit >= 0.0f) ? (m_linearRate - speed) / timestep : -(m_linearRate + speed) / timestep;
			dFloat speed0 = dClamp (relPosit / timestep, -m_linearRate, m_linearRate);
			dFloat speed1 = GetJointSpeed ();;
			dFloat accel = (speed0 - speed1) / timestep;
			NewtonUserJointSetRowAcceleration (m_joint, accel);
		}

dVector xxx (matrix0.m_posit - matrix1.m_posit);
dTrace (("%f %f %f\n", xxx.m_x, xxx.m_y, xxx.m_z))

		NewtonUserJointSetRowStiffness (m_joint, 1.0f);
	}
}
