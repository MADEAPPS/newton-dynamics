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
#include "ndJointPlane.h"

ndJointPlane::ndJointPlane()
	:ndJointBilateralConstraint()
	,m_angle(ndFloat32(0.0f))
	,m_omega(ndFloat32(0.0f))
	,m_posit_y(ndFloat32(0.0f))
	,m_posit_z(ndFloat32(0.0f))
	,m_speed_y(ndFloat32(0.0f))
	,m_speed_z(ndFloat32(0.0f))
	,m_enableControlRotation(true)
{
	m_maxDof = 5;
}

ndJointPlane::ndJointPlane (const ndVector& pivot, const ndVector& normal, ndBodyKinematic* const child, ndBodyKinematic* const parent)
	:ndJointBilateralConstraint(5, child, parent, ndGetIdentityMatrix())
	,m_angle(ndFloat32(0.0f))
	,m_omega(ndFloat32(0.0f))
	,m_posit_y(ndFloat32(0.0f))
	,m_posit_z(ndFloat32(0.0f))
	,m_speed_y(ndFloat32(0.0f))
	,m_speed_z(ndFloat32(0.0f))
	,m_enableControlRotation(true)
{
	ndMatrix pinAndPivotFrame(ndGramSchmidtMatrix(normal));
	pinAndPivotFrame.m_posit = pivot;
	pinAndPivotFrame.m_posit.m_w = 1.0f;
	// calculate the two local matrix of the pivot point
	CalculateLocalMatrix (pinAndPivotFrame, m_localMatrix0, m_localMatrix1);
}

ndJointPlane::~ndJointPlane()
{
}

void ndJointPlane::UpdateParameters()
{
	ndMatrix matrix0;
	ndMatrix matrix1;

	// calculate the position of the pivot point and the Jacobian direction vectors, in global space. 
	CalculateGlobalMatrix(matrix0, matrix1);

	// Restrict the movement on the pivot point along all two orthonormal axis direction perpendicular to the motion
	const ndVector veloc0(m_body0->GetVelocityAtPoint(matrix0.m_posit));
	const ndVector veloc1(m_body1->GetVelocityAtPoint(matrix1.m_posit));
	const ndVector veloc(veloc0 - veloc1);
	const ndVector step(matrix0.m_posit - matrix1.m_posit);

	m_posit_y = step.DotProduct(matrix1.m_up).GetScalar();
	m_posit_z = step.DotProduct(matrix1.m_right).GetScalar();
	m_speed_y = veloc.DotProduct(matrix1.m_up).GetScalar();
	m_speed_z = veloc.DotProduct(matrix1.m_right).GetScalar();

	if (m_enableControlRotation)
	{
		const ndVector omega0(m_body0->GetOmega());
		const ndVector omega1(m_body1->GetOmega());
		const ndFloat32 deltaAngle = ndAnglesAdd(-CalculateAngle(matrix0.m_up, matrix1.m_up, matrix1.m_front), -m_angle);
		m_angle += deltaAngle;
		m_omega = matrix1.m_front.DotProduct(omega0 - omega1).GetScalar();
	}
}

void ndJointPlane::JacobianDerivative(ndConstraintDescritor& desc)
{
	ndMatrix matrix0;
	ndMatrix matrix1;

	// calculate the position of the pivot point and the Jacobian direction vectors, in global space. 
	CalculateGlobalMatrix (matrix0, matrix1);

	// Restrict the movement on the pivot point along all two orthonormal axis direction perpendicular to the motion
	const ndVector& dir = matrix1[0];
	const ndVector& p0 = matrix0.m_posit;
	const ndVector& p1 = matrix1.m_posit;
	AddLinearRowJacobian(desc, p0, p1, dir);
	
	const ndFloat32 dist = ndFloat32 (0.25f) * dir.DotProduct((p1 - p0) & ndVector::m_triplexMask).GetScalar();
	const ndFloat32 accel = GetMotorZeroAcceleration(desc) + dist * desc.m_invTimestep * desc.m_invTimestep;

	SetMotorAcceleration(desc, accel);

	// construct an orthogonal coordinate system with these two vectors
	if (m_enableControlRotation) 
	{
		AddAngularRowJacobian(desc, matrix1.m_up, CalculateAngle(matrix0.m_front, matrix1.m_front, matrix1.m_up));
		AddAngularRowJacobian(desc, matrix1.m_right, CalculateAngle(matrix0.m_front, matrix1.m_front, matrix1.m_right));
	}
}

