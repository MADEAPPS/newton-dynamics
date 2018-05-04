/* 
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


#include "stdafx.h"
#include "dNewtonJointSlider.h"


dNewtonJointSlider::dNewtonJointSlider(const dMatrix pintAndPivotMatrix, void* const body0, void* const body1)
	:dNewtonJoint()
{
	dMatrix bodyMatrix;
	NewtonBody* const netwonBody0 = (NewtonBody*)body0;
	NewtonBody* const netwonBody1 = (NewtonBody*)body1;
	NewtonBodyGetMatrix(netwonBody0, &bodyMatrix[0][0]);

	dMatrix matrix(pintAndPivotMatrix * bodyMatrix);
	dCustomSlider* const joint = new dCustomSlider(matrix, netwonBody0, netwonBody1);
	SetJoint(joint);
}


void dNewtonJointSlider::SetLimits(bool enable, dFloat minDistance, dFloat maxDistance)
{
	dCustomSlider* const joint = (dCustomSlider*)m_joint;
	joint->EnableLimits(enable);
	if (enable) {
		joint->SetLimits(dMin(minDistance, 0.0f), dMax(maxDistance, 0.0f));
	}
}

void dNewtonJointSlider::SetAsSpringDamper(bool enable, dFloat forceMixing, dFloat springConst, dFloat damperConst)
{
	dCustomSlider* const joint = (dCustomSlider*)m_joint;
	joint->SetAsSpringDamper(enable, dClamp(forceMixing, 0.7f, 0.99f), dAbs(springConst), dAbs(damperConst));
}


dNewtonJointSliderActuator::dNewtonJointSliderActuator(const dMatrix pintAndPivotMatrix, void* const body0, void* const body1)
	:dNewtonJoint()
{
	dMatrix bodyMatrix;
	NewtonBody* const netwonBody0 = (NewtonBody*)body0;
	NewtonBody* const netwonBody1 = (NewtonBody*)body1;
	NewtonBodyGetMatrix(netwonBody0, &bodyMatrix[0][0]);

	dMatrix matrix(pintAndPivotMatrix * bodyMatrix);
	dCustomSliderActuator* const joint = new dCustomSliderActuator(matrix, netwonBody0, netwonBody1);
	SetJoint(joint);
	joint->SetEnableFlag(true);
}

dFloat dNewtonJointSliderActuator::GetPosition() const
{
	dCustomSliderActuator* const joint = (dCustomSliderActuator*)m_joint;
	return joint->GetActuatorPosit();
}

void dNewtonJointSliderActuator::SetMaxForce(dFloat force)
{
	dCustomSliderActuator* const joint = (dCustomSliderActuator*)m_joint;
	joint->SetMaxForcePower(dAbs(force));
}

void dNewtonJointSliderActuator::SetSpeed(dFloat speed)
{
	dCustomSliderActuator* const joint = (dCustomSliderActuator*)m_joint;
	joint->SetLinearRate(dAbs(speed));
}

void dNewtonJointSliderActuator::SetTargetPosition(dFloat position, dFloat minLimit, dFloat maxLimit)
{
	dCustomSliderActuator* const joint = (dCustomSliderActuator*)m_joint;
	joint->SetMinPositLimit(dMin(minLimit, dFloat(0.0f)));
	joint->SetMaxPositLimit(dMax(maxLimit, dFloat(0.0f)));
	joint->SetTargetPosit(dClamp (position, joint->GetMinPositLimit(), joint->GetMaxPositLimit()));
}

