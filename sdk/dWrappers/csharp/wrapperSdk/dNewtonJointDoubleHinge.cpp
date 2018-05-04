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
#include "dNewtonJointDoubleHinge.h"

dNewtonJointDoubleHinge::dNewtonJointDoubleHinge(const dMatrix pintAndPivotMatrix, void* const body0, void* const body1)
	:dNewtonJoint()
{
	dMatrix bodyMatrix;

	NewtonBody* const netwonBody0 = (NewtonBody*)body0;
	NewtonBody* const netwonBody1 = (NewtonBody*)body1;
	NewtonBodyGetMatrix(netwonBody0, &bodyMatrix[0][0]);

	dMatrix matrix(pintAndPivotMatrix * bodyMatrix);
	dCustomDoubleHinge* const joint = new dCustomDoubleHinge(matrix, netwonBody0, netwonBody1);
	SetJoint(joint);
}

void dNewtonJointDoubleHinge::SetLimits_0(bool enable, dFloat minVal, dFloat maxAngle)
{
	dCustomDoubleHinge* const joint = (dCustomDoubleHinge*)m_joint;
	joint->EnableLimits(enable);
	if (enable) {
		joint->SetLimits(dMin(minVal * DEGREES_TO_RAD, 0.0f), dMax(maxAngle * DEGREES_TO_RAD, 0.0f));
	}
}

void dNewtonJointDoubleHinge::SetLimits_1(bool enable, dFloat minVal, dFloat maxAngle)
{
	dCustomDoubleHinge* const joint = (dCustomDoubleHinge*)m_joint;
	joint->EnableLimits1(enable);
	if (enable) {
		joint->SetLimits1(dMin(minVal * DEGREES_TO_RAD, 0.0f), dMax(maxAngle * DEGREES_TO_RAD, 0.0f));
	}
}


dNewtonJointDoubleHingeActuator::dNewtonJointDoubleHingeActuator(const dMatrix pintAndPivotMatrix, void* const body0, void* const body1)
	:dNewtonJoint()
{
	dMatrix bodyMatrix;
	NewtonBody* const netwonBody0 = (NewtonBody*)body0;
	NewtonBody* const netwonBody1 = (NewtonBody*)body1;
	NewtonBodyGetMatrix(netwonBody0, &bodyMatrix[0][0]);

	dMatrix matrix(pintAndPivotMatrix * bodyMatrix);
	dCustomDoubleHingeActuator* const joint = new dCustomDoubleHingeActuator(matrix, netwonBody0, netwonBody1);
	SetJoint(joint);
	//joint->SetEnableFlag0(true);
	//joint->SetEnableFlag1(true);
}

dFloat dNewtonJointDoubleHingeActuator::GetAngle0() const
{
	dCustomDoubleHingeActuator* const joint = (dCustomDoubleHingeActuator*) m_joint;
	return joint->GetActuatorAngle0() * RAD_TO_DEGREES;
}

void dNewtonJointDoubleHingeActuator::SetMaxToque0(dFloat torque)
{
	dCustomDoubleHingeActuator* const joint = (dCustomDoubleHingeActuator*) m_joint;
	joint->SetMaxTorque0(dAbs(torque));
}

void dNewtonJointDoubleHingeActuator::SetAngularRate0(dFloat rate)
{
	dCustomDoubleHingeActuator* const joint = (dCustomDoubleHingeActuator*) m_joint;
	joint->SetAngularRate0(rate * DEGREES_TO_RAD);
}

void dNewtonJointDoubleHingeActuator::SetTargetAngle0(dFloat angle, dFloat minLimit, dFloat maxLimit)
{
	dCustomDoubleHingeActuator* const joint = (dCustomDoubleHingeActuator*) m_joint;
	joint->SetLimits(dMin(minLimit * DEGREES_TO_RAD, dFloat(0.0f)), dMax(maxLimit * DEGREES_TO_RAD, dFloat(0.0f)));
	joint->SetTargetAngle0(dClamp (angle * DEGREES_TO_RAD, joint->GetMinAngularLimit0(), joint->GetMaxAngularLimit0()));
}


dFloat dNewtonJointDoubleHingeActuator::GetAngle1() const
{
	dCustomDoubleHingeActuator* const joint = (dCustomDoubleHingeActuator*)m_joint;
	return joint->GetActuatorAngle0() * RAD_TO_DEGREES;
}

void dNewtonJointDoubleHingeActuator::SetMaxToque1(dFloat torque)
{
	dCustomDoubleHingeActuator* const joint = (dCustomDoubleHingeActuator*)m_joint;
	joint->SetMaxTorque1(dAbs(torque));
}

void dNewtonJointDoubleHingeActuator::SetAngularRate1(dFloat rate)
{
	dCustomDoubleHingeActuator* const joint = (dCustomDoubleHingeActuator*)m_joint;
	joint->SetAngularRate1(rate * DEGREES_TO_RAD);
}

void dNewtonJointDoubleHingeActuator::SetTargetAngle1(dFloat angle, dFloat minLimit, dFloat maxLimit)
{
	dCustomDoubleHingeActuator* const joint = (dCustomDoubleHingeActuator*)m_joint;
	joint->SetLimits1(dMin(minLimit * DEGREES_TO_RAD, dFloat(0.0f)), dMax(maxLimit * DEGREES_TO_RAD, dFloat(0.0f)));
	joint->SetTargetAngle1(dClamp(angle * DEGREES_TO_RAD, joint->GetMinAngularLimit1(), joint->GetMaxAngularLimit1()));
}
