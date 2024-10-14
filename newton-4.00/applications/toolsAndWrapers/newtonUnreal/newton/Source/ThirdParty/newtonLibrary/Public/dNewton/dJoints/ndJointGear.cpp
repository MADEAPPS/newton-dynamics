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
#include "ndJointGear.h"

ndJointGear::ndJointGear()
	:ndJointBilateralConstraint()
{
	m_maxDof = 1;
}

ndJointGear::ndJointGear(ndFloat32 gearRatio,
	const ndVector& body0Pin, ndBodyKinematic* const body0,
	const ndVector& body1Pin, ndBodyKinematic* const body1)
	:ndJointBilateralConstraint(1, body0, body1, ndGetIdentityMatrix())
	,m_gearRatio(gearRatio)
{
	// calculate the two local matrix of the pivot point
	ndMatrix dommyMatrix;

	// calculate the local matrix for body body0
	ndMatrix pinAndPivot0(ndGramSchmidtMatrix(body0Pin));
	CalculateLocalMatrix(pinAndPivot0, m_localMatrix0, dommyMatrix);
	m_localMatrix0.m_posit = ndVector::m_wOne;

	// calculate the local matrix for body body1  
	ndMatrix pinAndPivot1(ndGramSchmidtMatrix(body1Pin));
	CalculateLocalMatrix(pinAndPivot1, dommyMatrix, m_localMatrix1);
	m_localMatrix1.m_posit = ndVector::m_wOne;

	// set as kinematic loop
	SetSolverModel(m_jointkinematicOpenLoop);
}

ndJointGear::~ndJointGear()
{
}


ndFloat32 ndJointGear::GetRatio() const
{
	return m_gearRatio;
}

void ndJointGear::SetRatio(ndFloat32 ratio)
{
	m_gearRatio = ratio;
}

void ndJointGear::JacobianDerivative(ndConstraintDescritor& desc)
{
	ndMatrix matrix0;
	ndMatrix matrix1;

	// calculate the position of the pivot point and the Jacobian direction vectors, in global space. 
	CalculateGlobalMatrix(matrix0, matrix1);

	AddAngularRowJacobian(desc, matrix0.m_front, ndFloat32(0.0f));

	ndJacobian& jacobian0 = desc.m_jacobian[desc.m_rowsCount - 1].m_jacobianM0;
	ndJacobian& jacobian1 = desc.m_jacobian[desc.m_rowsCount - 1].m_jacobianM1;

	jacobian0.m_angular = matrix0.m_front.Scale(m_gearRatio);
	jacobian1.m_angular = matrix1.m_front;

	const ndVector& omega0 = m_body0->GetOmega();
	const ndVector& omega1 = m_body1->GetOmega();

	const ndVector relOmega(omega0 * jacobian0.m_angular + omega1 * jacobian1.m_angular);
	const ndFloat32 w = relOmega.AddHorizontal().GetScalar() * ndFloat32(0.5f);
	SetMotorAcceleration(desc, -w * desc.m_invTimestep);
}

