/* Copyright (c) <2003-2019> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/

#include "dCoreStdafx.h"
#include "ndNewtonStdafx.h"
#include "ndJointHinge.h"

ndJointHinge::ndJointHinge(const dMatrix& pinAndPivotFrame, ndBodyKinematic* const child, ndBodyKinematic* const parent)
	:ndJointBilateralConstraint(6, child, parent, pinAndPivotFrame)
{
}

ndJointHinge::~ndJointHinge()
{
}

void ndJointHinge::JacobianDerivative(ndConstraintDescritor& desc)
{
	dMatrix matrix0;
	dMatrix matrix1;
	CalculateGlobalMatrix(matrix0, matrix1);

	AddLinearRowJacobian(desc, matrix0.m_posit, matrix1.m_posit, matrix1[0]);
	AddLinearRowJacobian(desc, matrix0.m_posit, matrix1.m_posit, matrix1[1]);
	AddLinearRowJacobian(desc, matrix0.m_posit, matrix1.m_posit, matrix1[2]);

	// the joint angle can be determined by getting the angle between any two non parallel vectors
	//m_curJointAngle.Update(CalculateAngle(matrix1.m_up, matrix0.m_up, matrix0.m_front));

	// save the current joint Omega
	dVector omega0(m_body0->GetOmega());
	dVector omega1(m_body1->GetOmega());;
	//m_jointOmega = matrix0.m_front.DotProduct3(omega0 - omega1);

	// two rows to restrict rotation around around the parent coordinate system
	const dFloat32 angleError = m_maxAngleError;
	const dFloat32 angle0 = CalculateAngle(matrix0.m_front, matrix1.m_front, matrix1.m_up);
	//NewtonUserJointAddAngularRow(m_joint, angle0, &matrix1.m_up[0]);
	AddAngularRowJacobian(desc, &matrix1.m_up[0], angle0);
	//NewtonUserJointSetRowStiffness(m_joint, m_stiffness);
	if (dAbs(angle0) > angleError) 
	{
		dAssert(0);
		//const dFloat32 alpha = NewtonUserJointCalculateRowZeroAcceleration(m_joint) + dFloat32(0.25f) * angle0 / (timestep * timestep);
		//NewtonUserJointSetRowAcceleration(m_joint, alpha);
	}

	const dFloat32 angle1 = CalculateAngle(matrix0.m_front, matrix1.m_front, matrix1.m_right);
	//NewtonUserJointAddAngularRow(m_joint, angle1, &matrix1.m_right[0]);
	AddAngularRowJacobian(desc, &matrix1.m_right[0], angle0);
	//NewtonUserJointSetRowStiffness(m_joint, m_stiffness);
	if (dAbs(angle1) > angleError) 
	{
		dAssert(0);
		//const dFloat32 alpha = NewtonUserJointCalculateRowZeroAcceleration(m_joint) + dFloat32(0.25f) * angle1 / (timestep * timestep);
		//NewtonUserJointSetRowAcceleration(m_joint, alpha);
	}


	//if (m_options.m_option2) 
	//{
	//	// the joint is motor
	//	dFloat32 accel = (m_motorSpeed - m_jointOmega) / timestep;
	//	NewtonUserJointAddAngularRow(m_joint, 0.0f, &matrix0.m_front[0]);
	//	NewtonUserJointSetRowAcceleration(m_joint, accel);
	//	NewtonUserJointSetRowStiffness(m_joint, m_stiffness);
	//	NewtonUserJointSetRowMinimumFriction(m_joint, -m_friction);
	//	NewtonUserJointSetRowMaximumFriction(m_joint, m_friction);
	//}
	//else 
	//{
	//	// the joint is not motor
	//	if (m_options.m_option0) 
	//	{
	//		if (m_options.m_option1) 
	//		{
	//			SubmitConstraintLimitSpringDamper(matrix0, matrix1, timestep);
	//		}
	//		else {
	//			SubmitConstraintLimits(matrix0, matrix1, timestep);
	//		}
	//	}
	//	else if (m_options.m_option1) 
	//	{
	//		SubmitConstraintSpringDamper(matrix0, matrix1, timestep);
	//	}
	//	else if (m_friction != 0.0f) 
	//	{
	//		NewtonUserJointAddAngularRow(m_joint, 0, &matrix0.m_front[0]);
	//		NewtonUserJointSetRowStiffness(m_joint, m_stiffness);
	//		NewtonUserJointSetRowAcceleration(m_joint, -m_jointOmega / timestep);
	//		NewtonUserJointSetRowMinimumFriction(m_joint, -m_friction);
	//		NewtonUserJointSetRowMaximumFriction(m_joint, m_friction);
	//	}
	//}
}


