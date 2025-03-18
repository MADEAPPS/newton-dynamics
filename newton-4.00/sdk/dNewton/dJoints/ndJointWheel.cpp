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
#include "ndJointWheel.h"

ndJointWheel::ndJointWheel()
	:ndJointBilateralConstraint()
	,m_baseFrame(m_localMatrix1)
	,m_info()
	,m_posit(ndFloat32(0.0f))
	,m_speed(ndFloat32(0.0f))
	,m_regularizer(m_info.m_regularizer)
	,m_normalizedBrake(ndFloat32(0.0f))
	,m_normalidedSteering(ndFloat32(0.0f))
	,m_normalidedSteering0(ndFloat32(0.0f))
	,m_normalizedHandBrake(ndFloat32(0.0f))
	,m_IsAapplyingBreaks(false)
	//,m_vcdMode(false)
{
	m_maxDof = 7;
}

ndJointWheel::ndJointWheel(const ndMatrix& pinAndPivotFrame, ndBodyKinematic* const child, ndBodyKinematic* const parent, const ndWheelDescriptor& info)
	:ndJointBilateralConstraint(7, child, parent, pinAndPivotFrame)
	,m_baseFrame(m_localMatrix1)
	,m_info(info)
	,m_posit(ndFloat32 (0.0f))
	,m_speed(ndFloat32(0.0f))
	,m_regularizer(info.m_regularizer)
	,m_normalizedBrake(ndFloat32(0.0f))
	,m_normalidedSteering(ndFloat32(0.0f))
	,m_normalidedSteering0(ndFloat32(0.0f))
	,m_normalizedHandBrake(ndFloat32(0.0f))
	,m_IsAapplyingBreaks(false)
	//,m_vcdMode(false)
{
}

ndJointWheel::~ndJointWheel()
{
}

const ndWheelDescriptor& ndJointWheel::GetInfo() const
{
	return m_info;
}

void ndJointWheel::SetInfo(const ndWheelDescriptor& info)
{
	m_info = info;
}

ndFloat32 ndJointWheel::GetPosit() const
{
	return m_posit;
}

ndFloat32 ndJointWheel::SetSpeed() const
{
	return m_speed;
}

ndFloat32 ndJointWheel::GetBreak() const
{
	return m_normalizedBrake;
}

ndFloat32 ndJointWheel::GetSteering() const
{
	return m_normalidedSteering;
}

ndFloat32 ndJointWheel::GetHandBreak() const
{
	return m_normalizedHandBrake;
}

void ndJointWheel::SetBreak(ndFloat32 normalizedBrake)
{
	m_normalizedBrake = ndClamp (normalizedBrake, ndFloat32 (0.0f), ndFloat32 (1.0f));
}

void ndJointWheel::SetHandBreak(ndFloat32 normalizedBrake)
{
	m_normalizedHandBrake = ndClamp(normalizedBrake, ndFloat32(0.0f), ndFloat32(1.0f));
}

void ndJointWheel::SetSteering(ndFloat32 normalidedSteering)
{
	m_normalidedSteering = ndClamp(normalidedSteering, ndFloat32(-1.0f), ndFloat32(1.0f));
}

void ndJointWheel::UpdateTireSteeringAngleMatrix()
{
	ndMatrix tireMatrix;
	ndMatrix chassisMatrix;
	m_localMatrix1 = ndYawMatrix(m_normalidedSteering * m_info.m_steeringAngle) * m_baseFrame;

	CalculateGlobalMatrix(tireMatrix, chassisMatrix);
	const ndVector localRelPosit(chassisMatrix.UntransformVector(tireMatrix.m_posit));
	const ndFloat32 distance = ndClamp(localRelPosit.m_y, m_info.m_lowerStop, m_info.m_upperStop);

	const ndFloat32 spinAngle = -CalculateAngle(tireMatrix.m_up, chassisMatrix.m_up, chassisMatrix.m_front);
	ndMatrix newTireMatrix(ndPitchMatrix(spinAngle) * chassisMatrix);
	newTireMatrix.m_posit = chassisMatrix.m_posit + chassisMatrix.m_up.Scale(distance);

	const ndMatrix tireBodyMatrix(m_localMatrix0.OrthoInverse() * newTireMatrix);
	m_body0->SetMatrix(tireBodyMatrix);
}

ndMatrix ndJointWheel::CalculateBaseFrame() const
{
	return m_localMatrix1 * m_body1->GetMatrix();
}

ndMatrix ndJointWheel::CalculateUpperBumperMatrix() const
{
	ndMatrix matrix(m_localMatrix1 * m_body1->GetMatrix());
	matrix.m_posit += matrix.m_up.Scale(m_info.m_upperStop);
	return matrix;
}

void ndJointWheel::DebugJoint(ndConstraintDebugCallback& context) const
{
	ndMatrix matrix0;
	ndMatrix matrix1;
	CalculateGlobalMatrix(matrix0, matrix1);

	//draw reference frame
	context.DrawFrame(matrix1);

	// draw upper bumper
	ndMatrix upperBumberMatrix(CalculateUpperBumperMatrix());
	ndMatrix tireBaseFrame(CalculateBaseFrame());
	context.DrawFrame(tireBaseFrame);

	// show tire center of mass;
	ndBodyKinematic* const tireBody = GetBody0()->GetAsBodyKinematic();
	ndMatrix tireFrame(tireBody->GetMatrix());
	context.DrawFrame(tireFrame);
	upperBumberMatrix.m_posit = tireFrame.m_posit;
	context.DrawFrame(upperBumberMatrix);
}

void ndJointWheel::JacobianDerivative(ndConstraintDescritor& desc)
{
	ndMatrix matrix0;
	ndMatrix matrix1;

	// calculate the position of the pivot point and the Jacobian direction vectors, in global space. 
	CalculateGlobalMatrix(matrix0, matrix1);

	// calculate position and speed	
	const ndVector veloc0(m_body0->GetVelocityAtPoint(matrix0.m_posit));
	const ndVector veloc1(m_body1->GetVelocityAtPoint(matrix1.m_posit));

	const ndVector& pin = matrix1[0];
	const ndVector& p0 = matrix0.m_posit;
	const ndVector& p1 = matrix1.m_posit;
	const ndVector prel(p0 - p1);
	const ndVector vrel(veloc0 - veloc1);

	m_speed = vrel.DotProduct(matrix1.m_up).GetScalar();
	m_posit = prel.DotProduct(matrix1.m_up).GetScalar();
	const ndVector projectedPoint = p1 + pin.Scale(pin.DotProduct(prel).GetScalar());

	const ndFloat32 angle0 = CalculateAngle(matrix0.m_front, matrix1.m_front, matrix1.m_up);
	const ndFloat32 angle1 = CalculateAngle(matrix0.m_front, matrix1.m_front, matrix1.m_right);

	AddLinearRowJacobian(desc, p0, projectedPoint, matrix1[0]);
	AddLinearRowJacobian(desc, p0, projectedPoint, matrix1[2]);
	AddAngularRowJacobian(desc, matrix1.m_up, angle0);
	AddAngularRowJacobian(desc, matrix1.m_right, angle1);
	AddLinearRowJacobian(desc, matrix0.m_posit, matrix1.m_posit, matrix1.m_up);
	SetMassSpringDamperAcceleration(desc, m_regularizer, m_info.m_springK, m_info.m_damperC);

	m_IsAapplyingBreaks = false;
	const ndFloat32 brakeFrictionTorque = ndMax(m_normalizedBrake * m_info.m_brakeTorque, m_normalizedHandBrake * m_info.m_handBrakeTorque);
	if (brakeFrictionTorque > ndFloat32(0.0f))
	{
		m_IsAapplyingBreaks = true;
		const ndFloat32 brakesToChassisInfluence = ndFloat32 (0.125f);

		AddAngularRowJacobian(desc, matrix1.m_front, ndFloat32(0.0f));
		const ndVector tireOmega(m_body0->GetOmega());
		const ndVector chassisOmega(m_body1->GetOmega());

		ndJacobian& jacobian0 = desc.m_jacobian[desc.m_rowsCount - 1].m_jacobianM0;
		ndJacobian& jacobian1 = desc.m_jacobian[desc.m_rowsCount - 1].m_jacobianM1;
		jacobian1.m_angular = jacobian1.m_angular.Scale(brakesToChassisInfluence);

		ndFloat32 w0 = tireOmega.DotProduct(jacobian0.m_angular).GetScalar();
		ndFloat32 w1 = chassisOmega.DotProduct(jacobian1.m_angular).GetScalar();
		ndFloat32 wRel = (w0 + w1) * ndFloat32 (0.35f);
		//ndTrace(("(%d: %f)\n", m_body0->GetId(), wRel));
		SetMotorAcceleration(desc, -wRel * desc.m_invTimestep);
		SetHighFriction(desc, brakeFrictionTorque);
		SetLowerFriction(desc, -brakeFrictionTorque);
	}

	// add suspension limits alone the vertical axis 
	const ndFloat32 x = m_posit + m_speed * desc.m_timestep;
	if (x >= m_info.m_upperStop)
	{
		AddLinearRowJacobian(desc, matrix0.m_posit, matrix1.m_posit, matrix1.m_up);
		const ndFloat32 stopAccel = GetMotorZeroAcceleration(desc);
		SetMotorAcceleration(desc, stopAccel);
		SetLowerFriction(desc, ndFloat32(0.0f));
	}
	else if (x <= m_info.m_lowerStop)
	{
		AddLinearRowJacobian(desc, matrix0.m_posit, matrix1.m_posit, matrix1.m_up);
		const ndFloat32 stopAccel = GetMotorZeroAcceleration(desc);
		SetMotorAcceleration(desc, stopAccel);
		SetHighFriction(desc, ndFloat32(0.0f));
	}
}

