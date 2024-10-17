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
#include "ndJointFix6dof.h"

ndJointFix6dof::ndJointFix6dof()
	:ndJointBilateralConstraint()
	,m_softness(ndFloat32(0.0f))
	,m_maxForce(D_MAX_BOUND)
	,m_maxTorque(D_MAX_BOUND)
{
	m_maxDof = 6;
}

ndJointFix6dof::ndJointFix6dof(const ndMatrix& frameInGlbalSpace, ndBodyKinematic* const body0, ndBodyKinematic* const body1)
	:ndJointBilateralConstraint(6, body0, body1, frameInGlbalSpace)
	,m_softness(ndFloat32(0.0f))
	,m_maxForce(D_MAX_BOUND)
	,m_maxTorque(D_MAX_BOUND)
{
}

ndJointFix6dof::ndJointFix6dof(ndBodyKinematic* const body0, ndBodyKinematic* const body1, const ndMatrix& globalMatrixBody0, const ndMatrix& globalMatrixBody1)
	:ndJointBilateralConstraint(6, body0, body1, globalMatrixBody0, globalMatrixBody1)
	,m_softness(ndFloat32(0.0f))
	,m_maxForce(D_MAX_BOUND)
	,m_maxTorque(D_MAX_BOUND)
{
}

ndJointFix6dof::~ndJointFix6dof()
{
}

void ndJointFix6dof::SetAsSoftJoint(bool)
{
	ndAssert(0);
	//SetSolverModel(mode ? m_secundaryCloseLoop : m_primaryOpenLoop);
}

void ndJointFix6dof::SetRegularizer(ndFloat32 regularizer)
{
	m_softness = ndClamp(regularizer, ndFloat32(0.0f), ndFloat32(1.0f));
}

ndFloat32 ndJointFix6dof::GetRegularizer() const
{
	return m_softness;
}

ndFloat32 ndJointFix6dof::GetMaxForce() const
{
	return m_maxForce;
}

ndFloat32 ndJointFix6dof::GetMaxTorque() const
{
	return m_maxTorque;
}

void ndJointFix6dof::JacobianDerivative(ndConstraintDescritor& desc)
{
	ndMatrix matrix0;
	ndMatrix matrix1;

	ndAssert(IsActive());
	// calculate the position of the pivot point and the Jacobian direction vectors, in global space. 
	CalculateGlobalMatrix(matrix0, matrix1);

	for (ndInt32 i = 0; i < 3; ++i)
	{
		AddLinearRowJacobian(desc, matrix0.m_posit, matrix1.m_posit, matrix1[i]);
		SetLowerFriction(desc, -m_maxForce);
		SetHighFriction(desc, m_maxForce);
		SetDiagonalRegularizer(desc, m_softness);
	}

	ndFloat32 cosAngle = matrix1.m_front.DotProduct(matrix0.m_front).GetScalar();
	if (cosAngle >= ndFloat32(0.998f)) 
	{
		// about 3.5 degree deviation, consider small angular approximation  
		SubmitAngularAxisCartisianApproximation(desc, matrix0, matrix1);
	}
	else 
	{
		// beyond 3.5 degree need to decompose the relative matrix into an orthonormal basics 
		SubmitAngularAxis(desc, matrix0, matrix1);
	}
}

void ndJointFix6dof::SubmitAngularAxisCartisianApproximation(ndConstraintDescritor& desc, const ndMatrix& matrix0, const ndMatrix& matrix1)
{
	// since very small angle rotation commute, we can issue
	// three angle around the matrix1 axis in any order.
	ndFloat32 angle0 = CalculateAngle(matrix0.m_front, matrix1.m_front, matrix1.m_up);
	AddAngularRowJacobian(desc, matrix1.m_up, angle0);
	SetLowerFriction(desc, -m_maxTorque);
	SetHighFriction(desc, m_maxTorque);
	SetDiagonalRegularizer(desc, m_softness);

	ndFloat32 angle1 = CalculateAngle(matrix0.m_front, matrix1.m_front, matrix1.m_right);
	AddAngularRowJacobian(desc, matrix1.m_right, angle1);
	SetLowerFriction(desc, -m_maxTorque);
	SetHighFriction(desc, m_maxTorque);
	SetDiagonalRegularizer(desc, m_softness);
	
	ndFloat32 angle2 = CalculateAngle(matrix0.m_up, matrix1.m_up, matrix1.m_front);
	AddAngularRowJacobian(desc, matrix1.m_front, angle2);
	SetLowerFriction(desc, -m_maxTorque);
	SetHighFriction(desc, m_maxTorque);
	SetDiagonalRegularizer(desc, m_softness);
}

void ndJointFix6dof::SubmitAngularAxis(ndConstraintDescritor& desc, const ndMatrix& matrix0, const ndMatrix& matrix1)
{
	// calculate cone angle
	ndVector lateralDir(matrix1.m_front.CrossProduct(matrix0.m_front));
	ndAssert(lateralDir.DotProduct(lateralDir).GetScalar() > ndFloat32 (1.0e-6f));
	lateralDir = lateralDir.Normalize();
	ndFloat32 coneAngle = ndAcos(ndClamp(matrix1.m_front.DotProduct(matrix0.m_front).GetScalar(), ndFloat32(-1.0f), ndFloat32(1.0f)));
	ndMatrix coneRotation(ndCalculateMatrix(ndQuaternion(lateralDir, coneAngle), matrix1.m_posit));

	AddAngularRowJacobian(desc, lateralDir, -coneAngle);
	SetLowerFriction(desc, -m_maxTorque);
	SetHighFriction(desc, m_maxTorque);
	SetDiagonalRegularizer(desc, m_softness);

	ndVector sideDir(lateralDir.CrossProduct(matrix0.m_front));
	AddAngularRowJacobian(desc, sideDir, ndFloat32(0.0f));
	SetLowerFriction(desc, -m_maxTorque);
	SetHighFriction(desc, m_maxTorque);
	SetDiagonalRegularizer(desc, m_softness);

	// calculate pitch angle
	ndMatrix pitchMatrix(matrix1 * coneRotation * matrix0.OrthoInverse());
	ndFloat32 pitchAngle = ndAtan2(pitchMatrix[1][2], pitchMatrix[1][1]);
	AddAngularRowJacobian(desc, matrix0.m_front, pitchAngle);
	SetLowerFriction(desc, -m_maxTorque);
	SetHighFriction(desc, m_maxTorque);
	SetDiagonalRegularizer(desc, m_softness);
	//dTrace(("%f %f\n", coneAngle * dRadToDegree, pitchAngle * dRadToDegree));
}

