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
#include "ndJointWheel.h"

ndJointWheel::ndJointWheel(const dMatrix& pinAndPivotFrame, ndBodyKinematic* const child, ndBodyKinematic* const parent, const ndWheelDescriptor& info)
	:ndJointBilateralConstraint(6, child, parent, pinAndPivotFrame)
	,m_baseFrame(m_localMatrix1)
	,m_info(info)
	,m_posit(dFloat32 (0.0f))
	,m_speed(dFloat32(0.0f))
	,m_regularizer(info.m_regularizer)
	,m_normalizedBrake(dFloat32(0.0f))
	,m_normalidedSteering(dFloat32(0.0f))
	,m_normalizedHandBrake(dFloat32(0.0f))
{
}

ndJointWheel::~ndJointWheel()
{
}

void ndJointWheel::SetBrake(dFloat32 normalizedBrake)
{
	m_normalizedBrake = dClamp (normalizedBrake, dFloat32 (0.0f), dFloat32 (1.0f));
}

void ndJointWheel::SetHandBrake(dFloat32 normalizedBrake)
{
	m_normalizedHandBrake = dClamp(normalizedBrake, dFloat32(0.0f), dFloat32(1.0f));
}

void ndJointWheel::SetSteering(dFloat32 normalidedSteering)
{
	m_normalidedSteering = dClamp(normalidedSteering, dFloat32(-1.0f), dFloat32(1.0f));
}

void ndJointWheel::CalculateTireSteeringMatrix()
{
	const dMatrix steeringMatrix(dYawMatrix(m_normalidedSteering * m_info.m_steeringAngle));
	m_localMatrix1 = steeringMatrix * m_baseFrame;
}

void ndJointWheel::UpdateTireSteeringAngleMatrix()
{
	dMatrix tireMatrix;
	dMatrix chassisMatrix;

	CalculateTireSteeringMatrix();
	CalculateGlobalMatrix(tireMatrix, chassisMatrix);

	const dVector relPosit(tireMatrix.m_posit - chassisMatrix.m_posit);
	const dFloat32 distance = relPosit.DotProduct(chassisMatrix.m_up).GetScalar();
	const dFloat32 spinAngle = -CalculateAngle(tireMatrix.m_up, chassisMatrix.m_up, chassisMatrix.m_front);

	dMatrix newTireMatrix(dPitchMatrix(spinAngle) * chassisMatrix);
	newTireMatrix.m_posit = chassisMatrix.m_posit + chassisMatrix.m_up.Scale(distance);

	const dMatrix tireBodyMatrix(m_localMatrix0.Inverse() * newTireMatrix);
	m_body0->SetMatrix(tireBodyMatrix);
}

dMatrix ndJointWheel::CalculateUpperBumperMatrix() const
{
	dMatrix matrix(m_localMatrix1 * m_body1->GetMatrix());
	matrix.m_posit += matrix.m_up.Scale(m_info.m_maxLimit);
	return matrix;
}

void ndJointWheel::JacobianDerivative(ndConstraintDescritor& desc)
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

	m_speed = vrel.DotProduct(matrix1.m_up).GetScalar();
	m_posit = prel.DotProduct(matrix1.m_up).GetScalar();
	const dVector projectedPoint = p1 + pin.Scale(pin.DotProduct(prel).GetScalar());

	const dFloat32 angle0 = CalculateAngle(matrix0.m_front, matrix1.m_front, matrix1.m_up);
	const dFloat32 angle1 = CalculateAngle(matrix0.m_front, matrix1.m_front, matrix1.m_right);

	AddLinearRowJacobian(desc, p0, projectedPoint, matrix1[0]);
	AddLinearRowJacobian(desc, p0, projectedPoint, matrix1[2]);
	AddAngularRowJacobian(desc, matrix1.m_up, angle0);
	AddAngularRowJacobian(desc, matrix1.m_right, angle1);
	
	// add suspension spring damper row
	AddLinearRowJacobian(desc, matrix0.m_posit, matrix1.m_posit, matrix1.m_up);
	SetMassSpringDamperAcceleration(desc, m_regularizer, m_info.m_springK, m_info.m_damperC);

//return;
m_normalizedBrake = 1.0f;
	const dFloat32 brakeFrictionTorque = dMax(m_normalizedBrake * m_info.m_brakeTorque, m_normalizedHandBrake * m_info.m_handBrakeTorque);
	if (brakeFrictionTorque > dFloat32(0.0f))
	{
		const dFloat32 brakesToChassisInfluence = dFloat32 (0.25f);

		AddAngularRowJacobian(desc, matrix1.m_front, dFloat32(0.0f));
		const dVector tireOmega(m_body0->GetOmega());
		const dVector chassisOmega(m_body1->GetOmega());

		ndJacobian& jacobian0 = desc.m_jacobian[desc.m_rowsCount - 1].m_jacobianM0;
		ndJacobian& jacobian1 = desc.m_jacobian[desc.m_rowsCount - 1].m_jacobianM1;
		jacobian1.m_angular = jacobian1.m_angular.Scale(brakesToChassisInfluence);

		dFloat32 w0 = tireOmega.DotProduct(jacobian0.m_angular).GetScalar();
		dFloat32 w1 = chassisOmega.DotProduct(jacobian1.m_angular).GetScalar();
		dFloat32 wRel = (w0 + w1) * dFloat32 (0.35f);

		SetMotorAcceleration(desc, -wRel * desc.m_invTimestep);

		SetHighFriction(desc, brakeFrictionTorque);
		SetLowerFriction(desc, -brakeFrictionTorque);
	}
	else
	{ 
		// add suspension limits alone the vertical axis 
		const dFloat32 x = m_posit + m_speed * desc.m_timestep;
		if (x < m_info.m_minLimit)
		{
			AddLinearRowJacobian(desc, matrix0.m_posit, matrix1.m_posit, matrix1.m_up);
			SetLowerFriction(desc, dFloat32(0.0f));
			const dFloat32 stopAccel = GetMotorZeroAcceleration(desc);
			SetMotorAcceleration(desc, stopAccel);
		}
		else if (x > m_info.m_maxLimit)
		{
			AddLinearRowJacobian(desc, matrix0.m_posit, matrix1.m_posit, matrix1.m_up);
			SetHighFriction(desc, dFloat32(0.0f));
			const dFloat32 stopAccel = GetMotorZeroAcceleration(desc);
			SetMotorAcceleration(desc, stopAccel);
		}
	}
	dAssert(desc.m_rowsCount <= 6);
}
