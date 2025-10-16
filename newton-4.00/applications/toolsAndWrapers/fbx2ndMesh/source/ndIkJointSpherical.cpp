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
#include "ndIkJointSpherical.h"

ndIkJointSpherical::ndIkJointSpherical()
	:ndJointSpherical()
	,ndJointBilateralConstraint::ndIkInterface()
{
}

ndIkJointSpherical::ndIkJointSpherical(const ndMatrix& pinAndPivotFrame, ndBodyKinematic* const child, ndBodyKinematic* const parent)
	:ndJointSpherical(pinAndPivotFrame, child, parent)
	,ndJointBilateralConstraint::ndIkInterface()
{
}

ndIkJointSpherical::~ndIkJointSpherical()
{
}

void ndIkJointSpherical::DebugJoint(ndConstraintDebugCallback& debugCallback) const
{
	ndJointSpherical::DebugJoint(debugCallback);
}

void ndIkJointSpherical::SubmitAccel(const ndMatrix&, const ndMatrix& matrix1, ndConstraintDescritor& desc)
{
	// if we have the alpha, there is not need to find a shortest path, any set of axis will do 
	for (ndInt32 i = 0; i < 3; ++i)
	{
		ndFloat32 accel = (matrix1[i] * m_accel0.m_angular - matrix1[i] * m_accel1.m_angular).AddHorizontal().GetScalar();
		AddAngularRowJacobian(desc, matrix1[i], ndFloat32(0.0f));
		SetMotorAcceleration(desc, accel);
		SetDiagonalRegularizer(desc, m_defualtRegularizer);
	}
}

void ndIkJointSpherical::JacobianDerivative(ndConstraintDescritor& desc)
{
	ndMatrix matrix0;
	ndMatrix matrix1;
	CalculateGlobalMatrix(matrix0, matrix1);
	ApplyBaseRows(matrix0, matrix1, desc);
	if (!m_ikMode)
	{
		SubmitAccel(matrix0, matrix1, desc);
	}
	SubmitLimits(matrix0, matrix1, desc);
}

ndInt32 ndIkJointSpherical::GetKinematicState(ndKinematicState* const state) const
{
	ndMatrix matrix0;
	ndMatrix matrix1;
	ndVector euler1;

	const ndBodyKinematic* const body0 = GetBody0();
	const ndBodyKinematic* const body1 = GetBody1();
	CalculateGlobalMatrix(matrix0, matrix1);
	
	const ndMatrix relMatrix(matrix0 * matrix1.Transpose3x3());
	const ndVector euler(relMatrix.CalcPitchYawRoll(euler1));
	state[0].m_posit = euler.m_x;
	state[1].m_posit = euler.m_y;
	state[2].m_posit = euler.m_z;

	const ndVector relOmega(matrix1.UnrotateVector(body0->GetOmega() - body1->GetOmega()));
	state[0].m_velocity = relOmega.m_x;
	state[1].m_velocity = relOmega.m_y;
	state[2].m_velocity = relOmega.m_z;

	return 3;
}