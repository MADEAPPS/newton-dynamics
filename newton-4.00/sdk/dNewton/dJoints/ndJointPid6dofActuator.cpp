/* Copyright (c) <2003-2021> <Newton Game Dynamics>
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
#include "ndJointPid6dofActuator.h"

ndJointPid6dofActuator::ndJointPid6dofActuator(const dMatrix& pinAndPivotFrame, ndBodyKinematic* const child, ndBodyKinematic* const parent)
	:ndJointPid3dofActuator(pinAndPivotFrame, child, parent)
{
}

ndJointPid6dofActuator::~ndJointPid6dofActuator()
{
}

void ndJointPid6dofActuator::SubmitLinearLimits(const dMatrix& matrix0, const dMatrix& matrix1, ndConstraintDescritor& desc)
{
	const dVector step (matrix0.m_posit - matrix1.m_posit);

dFloat32 ks = 1000.0f;
dFloat32 kd = 100.0f;
dFloat32 regularizer = 0.005f;

	if (step.DotProduct(step).GetScalar() < dFloat32(0.01f * 0.01f))
	{
		// Cartesian motion
		AddLinearRowJacobian(desc, matrix0.m_posit, matrix1.m_posit, matrix1[0]);
		SetMassSpringDamperAcceleration(desc, regularizer, ks, kd);

		AddLinearRowJacobian(desc, matrix0.m_posit, matrix1.m_posit, matrix1[1]);
		SetMassSpringDamperAcceleration(desc, regularizer, ks, kd);

		AddLinearRowJacobian(desc, matrix0.m_posit, matrix1.m_posit, matrix1[2]);
		SetMassSpringDamperAcceleration(desc, regularizer, ks, kd);
	}
	else
	{
		const dMatrix basis(step.Normalize());
		
		// move alone the diagonal;
		AddLinearRowJacobian(desc, matrix0.m_posit, matrix1.m_posit, basis[0]);
		SetMassSpringDamperAcceleration(desc, regularizer, ks, kd);

		AddLinearRowJacobian(desc, matrix1.m_posit, matrix1.m_posit, basis[1]);
		AddLinearRowJacobian(desc, matrix1.m_posit, matrix1.m_posit, basis[2]);
	}
}

