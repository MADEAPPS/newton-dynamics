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

#include "dStdAfxNewton.h"
#include "dNewton.h"
#include "dNewtonBody.h"
#include "dNewtonActuatorJoints.h"


dNewtonHingeActuator::dNewtonHingeActuator(const dFloat* const pinAndPivotFrame, dFloat angularRate, dFloat minAngle, dFloat maxAngle, dNewtonDynamicBody* const body0, dNewtonDynamicBody* const body1)
	:dNewtonJoint(m_hingeActuator)
{
	SetJoint (new dCustomHingeActuator (pinAndPivotFrame, angularRate, minAngle, maxAngle, body0->GetNewtonBody(), body1 ? body1->GetNewtonBody() : NULL));
}

bool dNewtonHingeActuator::GetEnableFlag () const
{
	return ((dCustomHingeActuator*)m_joint)->GetEnableFlag();
}

dFloat dNewtonHingeActuator::GetTargetAngle() const
{
	return ((dCustomHingeActuator*)m_joint)->GetTargetAngle();
}

dFloat dNewtonHingeActuator::GetMinAngularLimit() const
{
	return ((dCustomHingeActuator*)m_joint)->GetMinAngularLimit();
}

dFloat dNewtonHingeActuator::GetMaxAngularLimit() const
{
	return ((dCustomHingeActuator*)m_joint)->GetMaxAngularLimit();
}

dFloat dNewtonHingeActuator::GetAngularRate() const
{
	return ((dCustomHingeActuator*)m_joint)->GetAngularRate();
}


void dNewtonHingeActuator::SetMinAngularLimit(dFloat limit)
{
	((dCustomHingeActuator*)m_joint)->SetMinAngularLimit(limit);
}

void dNewtonHingeActuator::SetMaxAngularLimit(dFloat limit)
{
	((dCustomHingeActuator*)m_joint)->SetMaxAngularLimit(limit);
}


void dNewtonHingeActuator::SetAngularRate(dFloat rate)
{
	((dCustomHingeActuator*)m_joint)->SetAngularRate(rate);
}

void dNewtonHingeActuator::SetTargetAngle(dFloat angle)
{
	((dCustomHingeActuator*)m_joint)->SetTargetAngle(angle);
}

void dNewtonHingeActuator::SetEnableFlag (bool flag)
{
	((dCustomHingeActuator*)m_joint)->SetEnableFlag (flag);
}

dFloat dNewtonHingeActuator::GetActuatorAngle() const
{
	return ((dCustomHingeActuator*)m_joint)->GetActuatorAngle();
}


dNewtonSliderActuator::dNewtonSliderActuator(const dFloat* const pinAndPivotFrame, dFloat speed, dFloat minPosit, dFloat maxPosit, dNewtonDynamicBody* const body0, dNewtonDynamicBody* const body1)
	:dNewtonJoint(m_sliderActuator)
{
	SetJoint (new dCustomSliderActuator (pinAndPivotFrame, speed, minPosit, maxPosit, body0->GetNewtonBody(), body1 ? body1->GetNewtonBody() : NULL)); 
}

bool dNewtonSliderActuator::GetEnableFlag () const
{
	return ((dCustomSliderActuator*)m_joint)->GetEnableFlag ();
}

dFloat dNewtonSliderActuator::GetTargetPosit() const
{
	return ((dCustomSliderActuator*)m_joint)->GetTargetPosit();
}

dFloat dNewtonSliderActuator::GetLinearRate() const
{
	return ((dCustomSliderActuator*)m_joint)->GetLinearRate();
}


dFloat dNewtonSliderActuator::GetMinPositLimit() const
{
	return ((dCustomSliderActuator*)m_joint)->GetMinPositLimit();
}

dFloat dNewtonSliderActuator::GetMaxPositLimit() const
{
	return ((dCustomSliderActuator*)m_joint)->GetMaxPositLimit();
}


void dNewtonSliderActuator::SetTargetPosit(dFloat posit)
{
	((dCustomSliderActuator*)m_joint)->SetTargetPosit(posit);
}


void dNewtonSliderActuator::SetMinPositLimit(dFloat limit)
{
	((dCustomSliderActuator*)m_joint)->SetMinPositLimit(limit);
}

void dNewtonSliderActuator::SetMaxPositLimit(dFloat limit)
{
	((dCustomSliderActuator*)m_joint)->SetMaxPositLimit(limit);
}

void dNewtonSliderActuator::SetLinearRate(dFloat rate)
{
	((dCustomSliderActuator*)m_joint)->SetLinearRate (rate);
}

void dNewtonSliderActuator::SetEnableFlag (bool flag)
{
	((dCustomSliderActuator*)m_joint)->SetEnableFlag (flag);
}

dFloat dNewtonSliderActuator::GetActuatorPosit() const
{
	return ((dCustomSliderActuator*)m_joint)->GetActuatorPosit();
}



dNewtonUniversalActuator::dNewtonUniversalActuator(const dFloat* const pinAndPivotFrame, dFloat angularRate0, dFloat minAngle0, dFloat maxAngle0, dFloat angularRate1, dFloat minAngle1, dFloat maxAngle1, dNewtonDynamicBody* const body0, dNewtonDynamicBody* const body1)
	:dNewtonJoint(m_universalActuator)
{
	SetJoint (new dCustomUniversalActuator (pinAndPivotFrame, angularRate0, minAngle0, maxAngle0, angularRate1, minAngle1, maxAngle1, body0->GetNewtonBody(), body1 ? body1->GetNewtonBody() : NULL));
}


void dNewtonUniversalActuator::SetEnableFlag0 (bool flag)
{
	((dCustomUniversalActuator*)m_joint)->SetEnableFlag0 (flag);
}


void dNewtonUniversalActuator::SetTargetAngle0(dFloat angle)
{
	((dCustomUniversalActuator*)m_joint)->SetTargetAngle0 (angle);
}

void dNewtonUniversalActuator::SetMinAngularLimit0(dFloat limit)
{
//	dCustomUniversalActuator* const joint = (dCustomUniversalActuator*)m_joint;
	dAssert (0);
//	joint->SetLimits_0(limit, joint->GetMaxAngularLimit_0());
}

void dNewtonUniversalActuator::SetMaxAngularLimit0(dFloat limit)
{
//	dCustomUniversalActuator* const joint = (dCustomUniversalActuator*)m_joint;
	dAssert (0);
//	joint->SetLimits_0(joint->GetMinAngularLimit_0(), limit);
}

void dNewtonUniversalActuator::SetAngularRate0(dFloat rate)
{
	((dCustomUniversalActuator*)m_joint)->SetAngularRate0(rate);
}


void dNewtonUniversalActuator::SetEnableFlag1 (bool flag)
{
	((dCustomUniversalActuator*)m_joint)->SetEnableFlag1 (flag);
}

void dNewtonUniversalActuator::SetTargetAngle1(dFloat angle)
{
	((dCustomUniversalActuator*)m_joint)->SetTargetAngle1 (angle);
}

void dNewtonUniversalActuator::SetMinAngularLimit1(dFloat limit)
{
//	dCustomUniversalActuator* const joint = (dCustomUniversalActuator*)m_joint;
	dAssert (0);
//	joint->SetLimits_1(limit, joint->GetMaxAngularLimit_1());
}

void dNewtonUniversalActuator::SetMaxAngularLimit1(dFloat limit)
{
//	dCustomUniversalActuator* const joint = (dCustomUniversalActuator*)m_joint;
	dAssert (0);
//	joint->SetLimits_1(joint->GetMaxAngularLimit_1(), limit);
}

void dNewtonUniversalActuator::SetAngularRate1(dFloat rate)
{
	((dCustomUniversalActuator*)m_joint)->SetAngularRate1(rate);
}



bool dNewtonUniversalActuator::GetEnableFlag0 () const
{
	return ((dCustomUniversalActuator*)m_joint)->GetEnableFlag0();
}

dFloat dNewtonUniversalActuator::GetTargetAngle0() const
{
	return ((dCustomUniversalActuator*)m_joint)->GetTargetAngle0();
}

dFloat dNewtonUniversalActuator::GetAngularRate0() const
{
	return ((dCustomUniversalActuator*)m_joint)->GetAngularRate0();
}

dFloat dNewtonUniversalActuator::GetMinAngularLimit0() const
{
	dAssert (0);
	return 0;
//	return ((dCustomUniversalActuator*)m_joint)->GetMinAngularLimit_0();
}

dFloat dNewtonUniversalActuator::GetMaxAngularLimit0() const
{
	dAssert (0);
	return 0;
//	return ((dCustomUniversalActuator*)m_joint)->GetMaxAngularLimit_0();
}

bool dNewtonUniversalActuator::GetEnableFlag1 () const
{
	return ((dCustomUniversalActuator*)m_joint)->GetEnableFlag1();
}

dFloat dNewtonUniversalActuator::GetTargetAngle1() const
{
	return ((dCustomUniversalActuator*)m_joint)->GetTargetAngle1();
}

dFloat dNewtonUniversalActuator::GetAngularRate1() const
{
	return ((dCustomUniversalActuator*)m_joint)->GetAngularRate1();
}

dFloat dNewtonUniversalActuator::GetMinAngularLimit1() const
{
	dAssert (0);
	return 0;
//	return ((dCustomUniversalActuator*)m_joint)->GetMinAngularLimit_1();
}

dFloat dNewtonUniversalActuator::GetMaxAngularLimit1() const
{
	dAssert (0);
	return 0;
//	return ((dCustomUniversalActuator*)m_joint)->GetMaxAngularLimit_1();
}

dFloat dNewtonUniversalActuator::GetActuatorAngle0() const
{
	dAssert (0);
	return 0;
//	return ((dCustomUniversalActuator*)m_joint)->GetJointAngle_0();
}

dFloat dNewtonUniversalActuator::GetActuatorAngle1() const
{
	dAssert (0);
	return 0;
//	return ((dCustomUniversalActuator*)m_joint)->GetJointAngle_1();
}

