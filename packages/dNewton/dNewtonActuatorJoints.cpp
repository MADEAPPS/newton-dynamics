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

	// the joint angle can be determine by getting the angle between any two non parallel vectors
	if (angle < m_minAngle) {
		dAssert(0);
/*
		dFloat relAngle = angle - m_minAngle;
		// the angle was clipped save the new clip limit
		m_curJointAngle.m_angle = m_minAngle;

		// tell joint error will minimize the exceeded angle error
		NewtonUserJointAddAngularRow (m_joint, relAngle, &matrix0.m_front[0]);

		// need high stiffness here
		NewtonUserJointSetRowStiffness (m_joint, 1.0f);

		// allow the joint to move back freely 
		NewtonUserJointSetRowMaximumFriction (m_joint, 0.0f);
*/

	} else if (angle  > m_maxAngle) {
		dAssert(0);
/*
		dFloat relAngle = angle - m_maxAngle;

		// the angle was clipped save the new clip limit
		m_curJointAngle.m_angle = m_maxAngle;

		// tell joint error will minimize the exceeded angle error
		NewtonUserJointAddAngularRow (m_joint, relAngle, &matrix0.m_front[0]);

		// need high stiffness here
		NewtonUserJointSetRowStiffness (m_joint, 1.0f);

		// allow the joint to move back freely
		NewtonUserJointSetRowMinimumFriction (m_joint, 0.0f);
*/
	} else {
		dFloat relAngle = angle - m_angle;
		NewtonUserJointAddAngularRow (joint, relAngle, &matrix0.m_front[0]);
	}
}
