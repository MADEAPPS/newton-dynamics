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
#include "ndJointUpVector.h"

ndJointUpVector::ndJointUpVector()
	:ndJointBilateralConstraint()
	,m_angle(ndFloat32(0.0f))
	,m_omega(ndFloat32(0.0f))
{
	m_maxDof = 2;
}

ndJointUpVector::ndJointUpVector(const ndVector& normal, ndBodyKinematic* const child, ndBodyKinematic* const parent)
	:ndJointBilateralConstraint(2, child, parent, ndGetIdentityMatrix())
	,m_angle(ndFloat32(0.0f))
	,m_omega(ndFloat32(0.0f))
{
	ndMatrix matrix(ndGramSchmidtMatrix(normal));
	matrix.m_posit = child->GetMatrix().m_posit;

	CalculateLocalMatrix (matrix, m_localMatrix0, m_localMatrix1);
}

ndJointUpVector::~ndJointUpVector()
{
}

// by animating the orientation of the pin vector the application can change the orientation of the picked object
void ndJointUpVector::SetPinDir (const ndVector& pin)
{
	m_localMatrix1 = ndGramSchmidtMatrix(pin);
}

void ndJointUpVector::UpdateParameters()
{
	ndMatrix matrix0;
	ndMatrix matrix1;
	CalculateGlobalMatrix(matrix0, matrix1);

	// save the current joint Omega
	const ndVector omega0(m_body0->GetOmega());
	const ndVector omega1(m_body1->GetOmega());

	// the joint angle can be determined by getting the angle between any two non parallel vectors
	const ndFloat32 deltaAngle = ndAnglesAdd(-CalculateAngle(matrix0.m_up, matrix1.m_up, matrix1.m_front), -m_angle);
	m_angle += deltaAngle;
	if (m_angle > (ndPi * ndFloat32(2.0f)))
	{
		m_angle = ndMod(m_angle, ndPi * ndFloat32(2.0f));
	}
	else if (m_angle < -(ndPi * ndFloat32(2.0f)))
	{
		m_angle = ndMod(m_angle, ndPi * ndFloat32(2.0f));
	}
	m_omega = matrix1.m_front.DotProduct(omega0 - omega1).GetScalar();
}

void ndJointUpVector::JacobianDerivative(ndConstraintDescritor& desc)
{
	ndMatrix matrix0;
	ndMatrix matrix1;

	// calculate the position of the pivot point and the Jacobian direction vectors, in global space. 
	CalculateGlobalMatrix (matrix0, matrix1);
  
	// if the body has rotated by some amount, the there will be a plane of rotation
	ndVector lateralDir (matrix0.m_front.CrossProduct(matrix1.m_front));
	ndAssert(lateralDir.m_w == ndFloat32(0.0f));
	ndFloat32 mag = lateralDir.DotProduct(lateralDir).GetScalar();
	if (mag > 1.0e-6f) 
	{
		// if the side vector is not zero, it means the body has rotated
		mag = ndSqrt (mag);
		lateralDir = lateralDir.Scale (1.0f / mag);
		ndFloat32 angle = ndAsin (mag);

		// add an angular constraint to correct the error angle
		//NewtonUserJointAddAngularRow (m_joint, angle, &lateralDir[0]);
		AddAngularRowJacobian(desc, lateralDir, angle);

		// in theory only one correction is needed, but this produces instability as the body may move sideway.
		// a lateral correction prevent this from happening.
		ndVector frontDir (lateralDir.CrossProduct(matrix1.m_front));
		AddAngularRowJacobian(desc, frontDir, ndFloat32 (0.0f));
 	} 
	else 
	{
		// if the angle error is very small then two angular correction along the plane axis do the trick
		AddAngularRowJacobian(desc, matrix0.m_up, ndFloat32(0.0f));
		AddAngularRowJacobian(desc, matrix0.m_right, ndFloat32(0.0f));
	}
}

