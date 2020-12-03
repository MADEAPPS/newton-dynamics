/* Copyright (c) <2003-2019> <Newton Game Dynamics>
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
#include "ndJointSlider.h"

ndJointSlider::ndJointSlider(const dMatrix& pinAndPivotFrame, ndBodyKinematic* const child, ndBodyKinematic* const parent)
	:ndJointBilateralConstraint(6, child, parent, pinAndPivotFrame)
	,m_posit(dFloat32 (0.0f))
	,m_speed(dFloat32(0.0f))
	,m_springK(dFloat32(0.0f))
	,m_damperC(dFloat32(0.0f))
	,m_isStringDamper(false)
{
}

ndJointSlider::~ndJointSlider()
{
}

void ndJointSlider::SetAsSpringDamper(bool state, dFloat32 spring, dFloat32 damper)
{
	m_springK = dAbs(spring);
	m_damperC = dAbs(damper);
	m_isStringDamper = state;
}

void ndJointSlider::SubmitSpringDamper(ndConstraintDescritor& desc, const dMatrix& matrix)
{
	dVector p0(matrix.m_posit);
	dVector p1(matrix.m_posit + matrix.m_front.Scale (m_posit));
	AddLinearRowJacobian(desc, p1, p0, matrix[0]);
	SetMassSpringDamperAcceleration(desc, m_springK, m_damperC);
}

void ndJointSlider::JacobianDerivative(ndConstraintDescritor& desc)
{
	dMatrix matrix0;
	dMatrix matrix1;

	// calculate the position of the pivot point and the Jacobian direction vectors, in global space. 
	CalculateGlobalMatrix(matrix0, matrix1);

	// calculate position and speed	
	const dVector veloc0(m_body0->GetVelocityAtPoint(matrix0.m_posit));
	const dVector veloc1(m_body1->GetVelocityAtPoint(matrix1.m_posit));

	const dVector& pin = matrix1[0];
	const dVector& p0 = matrix0.m_posit;
	const dVector& p1 = matrix1.m_posit;
	const dVector prel(p0 - p1);
	const dVector vrel(veloc0 - veloc1);

	m_speed = vrel.DotProduct(matrix1.m_front).GetScalar();
	m_posit = prel.DotProduct(matrix1.m_front).GetScalar();
	const dVector projectedPoint = p1 + pin.Scale(pin.DotProduct(prel).GetScalar());

	AddLinearRowJacobian(desc, p0, projectedPoint, matrix1[1]);
	//NewtonUserJointSetRowStiffness(m_joint, m_stiffness);

	AddLinearRowJacobian(desc, &p0[0], &projectedPoint[0], matrix1[2]);
	//NewtonUserJointSetRowStiffness(m_joint, m_stiffness);

	//SubmitAngularRow(matrix0, matrix1, timestep);


	const dFloat32 angleError = m_maxAngleError;

	const dFloat32 angle0 = CalculateAngle(matrix0.m_up, matrix1.m_up, matrix1.m_front);
	AddAngularRowJacobian(desc, matrix1.m_front, angle0);
	//NewtonUserJointSetRowStiffness(m_joint, m_stiffness);
	if (dAbs(angle0) > angleError) 
	{
		dAssert(0);
	//	const dFloat32 alpha = NewtonUserJointCalculateRowZeroAcceleration(m_joint) + dFloat32(0.25f) * angle0 / (timestep * timestep);
	//	NewtonUserJointSetRowAcceleration(m_joint, alpha);
	}
	
	const dFloat32 angle1 = CalculateAngle(matrix0.m_front, matrix1.m_front, matrix1.m_up);
	AddAngularRowJacobian(desc, matrix1.m_up, angle1);
	//NewtonUserJointSetRowStiffness(m_joint, m_stiffness);
	if (dAbs(angle1) > angleError) 
	{
		dAssert(0);
	//	const dFloat32 alpha = NewtonUserJointCalculateRowZeroAcceleration(m_joint) + dFloat32(0.25f) * angle1 / (timestep * timestep);
	//	NewtonUserJointSetRowAcceleration(m_joint, alpha);
	}
	
	const dFloat32 angle2 = CalculateAngle(matrix0.m_front, matrix1.m_front, matrix1.m_right);
	AddAngularRowJacobian(desc, matrix1.m_right, angle2);
	//NewtonUserJointSetRowStiffness(m_joint, m_stiffness);
	if (dAbs(angle2) > angleError) 
	{
		dAssert(0);
	//	const dFloat32 alpha = NewtonUserJointCalculateRowZeroAcceleration(m_joint) + dFloat32(0.25f) * angle2 / (timestep * timestep);
	//	NewtonUserJointSetRowAcceleration(m_joint, alpha);
	}

	if (m_isStringDamper)
	{
		SubmitSpringDamper(desc, matrix1);
	}
	//if (m_options.m_option0) {
	//	if (m_options.m_option1) {
	//		SubmitConstraintLimitSpringDamper(matrix0, matrix1, p0, p1, timestep);
	//	}
	//	else {
	//		SubmitConstraintLimits(matrix0, matrix1, p0, p1, timestep);
	//	}
	//}
	//else if (m_options.m_option1) {
	//	SubmitConstraintSpringDamper(matrix0, matrix1, p0, p1, timestep);
	//}
	//else if (m_friction != 0.0f) {
	//	NewtonUserJointAddLinearRow(m_joint, &p0[0], &p1[0], &matrix1.m_front[0]);
	//	NewtonUserJointSetRowAcceleration(m_joint, -m_speed / timestep);
	//	NewtonUserJointSetRowStiffness(m_joint, m_stiffness);
	//	NewtonUserJointSetRowMinimumFriction(m_joint, -m_friction);
	//	NewtonUserJointSetRowMaximumFriction(m_joint, m_friction);
	//}
}


