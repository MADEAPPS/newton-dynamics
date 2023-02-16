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
#include "ndIkJointHinge.h"

ndIkJointHinge::ndIkJointHinge(const ndMatrix& pinAndPivotFrame, ndBodyKinematic* const child, ndBodyKinematic* const parent)
	:ndJointHinge(pinAndPivotFrame, child, parent)
	,ndJointBilateralConstraint::ndIkInterface()
{
}

ndIkJointHinge::ndIkJointHinge(const ndMatrix& pinAndPivotInChild, const ndMatrix& pinAndPivotInParent, ndBodyKinematic* const child, ndBodyKinematic* const parent)
	:ndJointHinge(pinAndPivotInChild, pinAndPivotInParent, child, parent)
	,ndJointBilateralConstraint::ndIkInterface()
{
}

ndIkJointHinge::~ndIkJointHinge()
{
}

void ndIkJointHinge::JacobianDerivative(ndConstraintDescritor& desc)
{
	ndMatrix matrix0;
	ndMatrix matrix1;
	CalculateGlobalMatrix(matrix0, matrix1);

	ApplyBaseRows(desc, matrix0, matrix1);
	if (!m_ikMode)
	{
		const ndVector pin(matrix0.m_front);
		ndFloat32 accel = (pin * m_accel0.m_angular - pin * m_accel1.m_angular).AddHorizontal().GetScalar();
		AddAngularRowJacobian(desc, pin, 0.0f);
		SetMotorAcceleration(desc, accel);
		SetDiagonalRegularizer(desc, m_defualRegularizer);
	}
	SubmitLimits(desc, matrix0, matrix1);
}


