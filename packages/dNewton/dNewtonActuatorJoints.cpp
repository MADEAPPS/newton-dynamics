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

#include "dStdAfxNewton.h"
#include "dNewton.h"
#include "dNewtonBody.h"
#include "dNewtonActuatorJoints.h"


dFloat dNewtonHingeActuator::GetActuatorAngle() const
{
	CustomHinge* const hinge = (CustomHinge*) m_joint;
	return hinge->GetJointAngle();
}

void dNewtonHingeActuator::OnSubmitConstraint (dFloat timestep, int threadIndex)
{
	dNewtonHingeJoint::OnSubmitConstraint (timestep, threadIndex);

	if (m_flag) {
		dMatrix matrix0;
		dMatrix matrix1;

		CustomHinge* const customHinge = (CustomHinge*) m_joint;
		NewtonJoint* const newtonHinge = m_joint->GetJoint();

		customHinge->CalculateGlobalMatrix (matrix0, matrix1);
		dFloat relAngle = m_angle - customHinge->GetJointAngle();
		NewtonUserJointAddAngularRow (newtonHinge, relAngle, &matrix0.m_front[0]);

		dFloat step = m_angularRate * timestep;
		if (dAbs (relAngle) > 2.0f * dAbs (step)) {
			dFloat speed = customHinge->GetJointOmega ();
			dFloat accel = (relAngle >= 0.0f) ? (m_angularRate - speed) / timestep : -(m_angularRate + speed) / timestep;
			NewtonUserJointSetRowAcceleration (newtonHinge, accel);
		}
		NewtonUserJointSetRowStiffness (newtonHinge, 1.0f);
	}
}


dFloat dNewtonSliderActuator::GetActuatorPosit() const
{
	CustomSlider* const slider = (CustomSlider*) m_joint;
	return slider->GetJointPosit();
}

void dNewtonSliderActuator::OnSubmitConstraint (dFloat timestep, int threadIndex)
{
	dNewtonSliderJoint::OnSubmitConstraint (timestep, threadIndex);
/*
	if (m_flag) {
		dMatrix matrix0;
		dMatrix matrix1;

		CustomSlider* const customSlider = (CustomSlider*) m_joint;
		NewtonJoint* const newtonSlider = m_joint->GetJoint();

		//NewtonBodySetSleepState(m_joint->GetBody0(), false);
		//NewtonBodySetSleepState(m_joint->GetBody1(), false);

		dFloat posit = customSlider->GetJointPosit();
		customSlider->CalculateGlobalMatrix (matrix0, matrix1);

		dFloat relPosit = m_posit - posit;
		dVector posit1 (matrix0.m_posit + matrix1.m_front.Scale (relPosit));
		NewtonUserJointAddLinearRow (newtonSlider, &matrix0.m_posit[0], &posit1[0], &matrix1.m_front[0]);
		dFloat step = m_linearRate * timestep;
		if (dAbs (relPosit) > 2.0f * dAbs (step)) {
			dFloat speed = customSlider->GetJointSpeed ();
			//dFloat accel = (relPosit >= 0.0f) ? (m_linearRate - speed) / timestep : -(m_linearRate + speed) / timestep;
//			dFloat accel = (relPosit >= 0.0f) ? 0.1f : -0.1f;
//			NewtonUserJointSetRowAcceleration (newtonSlider, accel);
		}
		NewtonUserJointSetRowStiffness (newtonSlider, 1.0f);
	}
*/
}


dFloat dNewtonUniversalActuator::GetActuatorAngle0() const
{
	CustomUniversal* const customUniversal = (CustomUniversal*) m_joint;
	return customUniversal->GetJointAngle_0();
}

dFloat dNewtonUniversalActuator::GetActuatorAngle1() const
{
	CustomUniversal* const customUniversal = (CustomUniversal*) m_joint;
	return customUniversal->GetJointAngle_1();
}

void dNewtonUniversalActuator::OnSubmitConstraint (dFloat timestep, int threadIndex)
{
	dNewtonUniversalJoint::OnSubmitConstraint (timestep, threadIndex);

	if (m_flag0 | m_flag1){
		dMatrix matrix0;
		dMatrix matrix1;
		CustomUniversal* const customUniversal = (CustomUniversal*) m_joint;
		NewtonJoint* const newtonUniversal = m_joint->GetJoint();

		customUniversal->CalculateGlobalMatrix (matrix0, matrix1);
		if (m_flag0) {
			dFloat angle = customUniversal->GetJointAngle_0();
			dFloat relAngle = m_angle0 - angle;
			NewtonUserJointAddAngularRow (newtonUniversal, relAngle, &matrix0.m_front[0]);
			dFloat step = m_angularRate0 * timestep;
			if (dAbs (relAngle) > 2.0f * dAbs (step)) {
				dFloat speed = customUniversal->GetJointOmega_0 ();
				dFloat accel = (relAngle >= 0.0f) ? (m_angularRate0 - speed) / timestep : -(m_angularRate0 + speed) / timestep;
				NewtonUserJointSetRowAcceleration (newtonUniversal, accel);
			}
			NewtonUserJointSetRowStiffness (newtonUniversal, 1.0f);
		}

		if (m_flag1) {
			dFloat angle = customUniversal->GetJointAngle_1();
			dFloat relAngle = m_angle1 - angle;
			NewtonUserJointAddAngularRow (newtonUniversal, relAngle, &matrix1.m_up[0]);
			dFloat step = m_angularRate1 * timestep;
			if (dAbs (relAngle) > 2.0f * dAbs (step)) {
				dFloat speed = customUniversal->GetJointOmega_1 ();
				dFloat accel = (relAngle >= 0.0f) ? (m_angularRate1 - speed) / timestep : -(m_angularRate1 + speed) / timestep;
				NewtonUserJointSetRowAcceleration (newtonUniversal, accel);
			}
			NewtonUserJointSetRowStiffness (newtonUniversal, 1.0f);
		}
	}
}
