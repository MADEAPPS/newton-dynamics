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
#include "ndJointFixDistance.h"

ndJointFixDistance::ndJointFixDistance()
	:ndJointBilateralConstraint()
{
	m_maxDof = 1;
}

ndJointFixDistance::ndJointFixDistance(const ndVector& pivotInChildInGlobalSpace, const ndVector& pivotInParentInGlobalSpace, ndBodyKinematic* const child, ndBodyKinematic* const parent)
	:ndJointBilateralConstraint(1, child, parent, ndGetIdentityMatrix())
{
	m_localMatrix0 = ndGetIdentityMatrix();
	m_localMatrix1 = ndGetIdentityMatrix();
	m_localMatrix0.m_posit = child->GetMatrix().UntransformVector(pivotInChildInGlobalSpace);
	m_localMatrix1.m_posit = parent->GetMatrix().UntransformVector(pivotInParentInGlobalSpace);

	ndVector dist(pivotInChildInGlobalSpace - pivotInParentInGlobalSpace);
	m_distance = ndSqrt(dist.DotProduct(dist).GetScalar());
}

ndJointFixDistance::~ndJointFixDistance()
{
}

ndFloat32 ndJointFixDistance::GetDistance() const
{
	return m_distance;
}

void ndJointFixDistance::SetDistance(ndFloat32 dist)
{
	m_distance = dist;
}

void ndJointFixDistance::JacobianDerivative(ndConstraintDescritor& desc)
{
	ndMatrix matrix0;
	ndMatrix matrix1;

	// calculate the position of the pivot point and the Jacobian direction vectors, in global space. 
	CalculateGlobalMatrix(matrix0, matrix1);

	ndVector p0(matrix0.m_posit);
	ndVector p1(matrix1.m_posit);

	ndVector dir(p1 - p0);
	ndFloat32 mag2 = dir.DotProduct(dir).GetScalar();
	if (mag2 > ndFloat32(1.0e-3f))
	{
		dir = dir.Scale(1.0f / ndSqrt(mag2));
		ndFloat32 x = ndSqrt (mag2) - m_distance;

		ndMatrix matrix(ndGramSchmidtMatrix(dir));
		ndVector com0(m_body0->GetCentreOfMass());
		ndMatrix body0Matrix(m_body0->GetMatrix());
		ndVector veloc0(m_body0->GetVelocityAtPoint(p0));

		ndVector com1(m_body1->GetCentreOfMass());
		ndMatrix body1Matrix(m_body1->GetMatrix());
		ndVector veloc1(m_body1->GetVelocityAtPoint(p1));

		ndFloat32 v((veloc0 - veloc1).DotProduct(dir).GetScalar());
		ndFloat32 a = (x - v * desc.m_timestep) * desc.m_invTimestep * desc.m_invTimestep;

		ndVector r0 ((p0 - body0Matrix.TransformVector(com0)).CrossProduct(matrix.m_front));
		ndVector r1 ((p1 - body1Matrix.TransformVector(com1)).CrossProduct(matrix.m_front));

		AddLinearRowJacobian(desc, p0, p0, matrix0.m_right);
		ndJacobian& jacobian0 = desc.m_jacobian[desc.m_rowsCount - 1].m_jacobianM0;
		ndJacobian& jacobian1 = desc.m_jacobian[desc.m_rowsCount - 1].m_jacobianM1;

		jacobian0.m_linear = matrix[0];
		jacobian0.m_angular = r0;

		jacobian1.m_linear = matrix[0].Scale(ndFloat32 (-1.0f));
		jacobian1.m_angular = r1.Scale(ndFloat32(-1.0f));

		SetMotorAcceleration(desc, a);
	}
}

