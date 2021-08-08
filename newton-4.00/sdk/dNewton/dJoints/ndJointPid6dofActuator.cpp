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

#define D_SMALL_DISTANCE_ERROR dFloat32 (1.0e-2f)
#define D_SMALL_DISTANCE_ERROR2 (D_SMALL_DISTANCE_ERROR * D_SMALL_DISTANCE_ERROR)

ndJointPid6dofActuator::ndJointPid6dofActuator(const dMatrix& pinAndPivotFrame, ndBodyKinematic* const child, ndBodyKinematic* const parent)
	:ndJointPid3dofActuator(pinAndPivotFrame, child, parent)
	,m_linearSpring(dFloat32 (1000.0f))
	,m_linearDamper(dFloat32(50.0f))
	,m_linearRegularizer(dFloat32(5.0e-3f))
{
}

void ndJointPid6dofActuator::GetLinearSpringDamperRegularizer(dFloat32& spring, dFloat32& damper, dFloat32& regularizer) const
{
	spring = m_linearSpring;
	damper = m_linearDamper;
	regularizer = m_linearRegularizer;
}

void ndJointPid6dofActuator::SetLinearSpringDamperRegularizer(dFloat32 spring, dFloat32 damper, dFloat32 regularizer)
{
	m_linearSpring = dMax(spring, dFloat32(0.0f));
	m_linearDamper = dMax(damper, dFloat32(0.0f));
	m_linearRegularizer = dMax (regularizer, dFloat32 (1.0e-4f));
}

ndJointPid6dofActuator::~ndJointPid6dofActuator()
{
}

void ndJointPid6dofActuator::SubmitLinearLimits(const dMatrix& matrix0, const dMatrix& matrix1, ndConstraintDescritor& desc)
{
	const dVector step (matrix0.m_posit - matrix1.m_posit);
	if (step.DotProduct(step).GetScalar() <= D_SMALL_DISTANCE_ERROR2)
	//if (1)
	{
		// Cartesian motion
		AddLinearRowJacobian(desc, matrix0.m_posit, matrix1.m_posit, matrix1[0]);
		SetMassSpringDamperAcceleration(desc, m_linearRegularizer, m_linearSpring, m_linearDamper);

		AddLinearRowJacobian(desc, matrix0.m_posit, matrix1.m_posit, matrix1[1]);
		SetMassSpringDamperAcceleration(desc, m_linearRegularizer, m_linearSpring, m_linearDamper);

		AddLinearRowJacobian(desc, matrix0.m_posit, matrix1.m_posit, matrix1[2]);
		SetMassSpringDamperAcceleration(desc, m_linearRegularizer, m_linearSpring, m_linearDamper);
	}
	else
	{
		const dMatrix basis(step.Normalize());
		
		// move alone the diagonal;
		AddLinearRowJacobian(desc, matrix0.m_posit, matrix1.m_posit, basis[0]);
		SetMassSpringDamperAcceleration(desc, m_linearRegularizer, m_linearSpring, m_linearDamper);

		AddLinearRowJacobian(desc, matrix1.m_posit, matrix1.m_posit, basis[1]);
		AddLinearRowJacobian(desc, matrix1.m_posit, matrix1.m_posit, basis[2]);
	}
}

void ndJointPid6dofActuator::JacobianDerivative(ndConstraintDescritor& desc)
{
	ndJointPid3dofActuator::JacobianDerivative(desc);
}