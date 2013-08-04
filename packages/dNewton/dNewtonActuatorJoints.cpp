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

	CustomHinge* const hinge = (CustomHinge*) m_joint;
	NewtonJoint* const joint = m_joint->GetJoint();
	dFloat angle = hinge->GetJointAngle();

	dMatrix matrix0;
	dMatrix matrix1;
	hinge->CalculateGlobalMatrix (matrix0, matrix1);

	dFloat relAngle = angle - m_angle;
	NewtonUserJointAddAngularRow (joint, relAngle, &matrix0.m_front[0]);
}


dFloat dNewtonSliderActuator::GetActuatorPosit() const
{
	CustomSlider* const slider = (CustomSlider*) m_joint;
	return slider->GetJointPosit();
}

void dNewtonSliderActuator::OnSubmitConstraint (dFloat timestep, int threadIndex)
{
	dNewtonSliderJoint::OnSubmitConstraint (timestep, threadIndex);

	CustomSlider* const hinge = (CustomSlider*) m_joint;
	NewtonJoint* const joint = m_joint->GetJoint();

	dFloat posit = hinge->GetJointPosit();

	dMatrix matrix0;
	dMatrix matrix1;
	hinge->CalculateGlobalMatrix (matrix0, matrix1);

	dFloat relPosit = posit - m_posit;
	dVector posit1 (matrix0.m_posit - matrix0.m_front.Scale (relPosit));
	NewtonUserJointAddLinearRow (joint, &matrix0.m_posit[0], &posit1[0], &matrix0.m_front[0]);
}


dFloat dNewtonUniversalActuator::GetActuatorAngle0() const
{
	dAssert (0);
	return 0;
}

dFloat dNewtonUniversalActuator::GetActuatorAngle1() const
{
	dAssert (0);
	return 0;
}

void dNewtonUniversalActuator::OnSubmitConstraint (dFloat timestep, int threadIndex)
{
	dNewtonUniversalJoint::OnSubmitConstraint (timestep, threadIndex);


}
