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
	AddAngularRowJacobian(desc, &matrix1.m_up[0], angle0);
	if (dAbs(angle0) > angleError) 
	{
		dAssert(0);
		//const dFloat32 alpha = NewtonUserJointCalculateRowZeroAcceleration(m_joint) + dFloat32(0.25f) * angle0 / (timestep * timestep);
		//NewtonUserJointSetRowAcceleration(m_joint, alpha);
	}

	const dFloat32 angle1 = CalculateAngle(matrix0.m_front, matrix1.m_front, matrix1.m_right);
	AddAngularRowJacobian(desc, &matrix1.m_right[0], angle1);
	if (dAbs(angle1) > angleError) 
	{
		dAssert(0);
		//const dFloat32 alpha = NewtonUserJointCalculateRowZeroAcceleration(m_joint) + dFloat32(0.25f) * angle1 / (timestep * timestep);
		//NewtonUserJointSetRowAcceleration(m_joint, alpha);
	}
}


