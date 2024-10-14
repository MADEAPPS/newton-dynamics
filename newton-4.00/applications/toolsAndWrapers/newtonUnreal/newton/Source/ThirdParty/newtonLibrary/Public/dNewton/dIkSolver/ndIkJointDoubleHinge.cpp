/* Copyright (c) <2003-2022> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/

#include "ndCoreStdafx.h"
#include "ndNewtonStdafx.h"
#include "ndIkJointDoubleHinge.h"

ndIkJointDoubleHinge::ndIkJointDoubleHinge()
	:ndJointDoubleHinge()
	,ndJointBilateralConstraint::ndIkInterface()
{
}

ndIkJointDoubleHinge::ndIkJointDoubleHinge(const ndMatrix& pinAndPivotFrame, ndBodyKinematic* const child, ndBodyKinematic* const parent)
	:ndJointDoubleHinge(pinAndPivotFrame, child, parent)
	,ndJointBilateralConstraint::ndIkInterface()
{
}

ndIkJointDoubleHinge::~ndIkJointDoubleHinge()
{
}

void ndIkJointDoubleHinge::JacobianDerivative(ndConstraintDescritor& desc)
{
	ndMatrix matrix0;
	ndMatrix matrix1;
	CalculateGlobalMatrix(matrix0, matrix1);

	ApplyBaseRows(desc, matrix0, matrix1);
	if (!m_ikMode)
	{
		const ndVector pin0(matrix0.m_front);
		ndFloat32 accel0 = (pin0 * m_accel0.m_angular - pin0 * m_accel1.m_angular).AddHorizontal().GetScalar();
		AddAngularRowJacobian(desc, pin0, 0.0f);
		SetMotorAcceleration(desc, accel0);
		SetDiagonalRegularizer(desc, m_defualtRegularizer);

		const ndVector pin1(matrix1.m_up);
		ndFloat32 accel1 = (pin1 * m_accel0.m_angular - pin1 * m_accel1.m_angular).AddHorizontal().GetScalar();
		AddAngularRowJacobian(desc, pin1, 0.0f);
		SetMotorAcceleration(desc, accel1);
		SetDiagonalRegularizer(desc, m_defualtRegularizer);
	}
	SubmitLimits(desc, matrix0, matrix1);
}


