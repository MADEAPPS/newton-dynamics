/* Copyright (c) <2003-2022> <Julio Jerez, Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 
* 3. This notice may not be removed or altered from any source distribution.
*/

#include "ndCoreStdafx.h"
#include "ndNewtonStdafx.h"
#include "ndBodyDynamic.h"
#include "ndMultiBodyVehicle.h"
#include "dJoints/ndJointWheel.h"
#include "ndMultiBodyVehicleMotor.h"
#include "ndMultiBodyVehicleGearBox.h"


ndMultiBodyVehicleMotor::ndMultiBodyVehicleMotor()
	:ndJointBilateralConstraint()
	,m_vehicle(nullptr)
	,m_omega(ndFloat32(0.0f))
	,m_maxOmega(ndFloat32(100.0f))
	,m_omegaStep(ndFloat32(16.0f))
	,m_targetOmega(ndFloat32(0.0f))
	,m_engineTorque(ndFloat32(0.0f))
	,m_internalFriction(ndFloat32(100.0f))
{
	m_maxDof = 3;
	ndAssert(0);
}

ndMultiBodyVehicleMotor::ndMultiBodyVehicleMotor(ndBodyKinematic* const motor, ndMultiBodyVehicle* const vehicelModel)
	:ndJointBilateralConstraint(3, motor, vehicelModel->m_chassis, motor->GetMatrix())
	,m_vehicle(vehicelModel)
	,m_omega(ndFloat32(0.0f))
	,m_maxOmega(ndFloat32(100.0f))
	,m_omegaStep(ndFloat32(16.0f))
	,m_targetOmega(ndFloat32(0.0f))
	,m_engineTorque(ndFloat32(0.0f))
	,m_internalFriction(ndFloat32(100.0f))
{
}

ndMultiBodyVehicleMotor::ndMultiBodyVehicleMotor(ndBodyKinematic* const motor, ndBodyKinematic* const chassis)
	:ndJointBilateralConstraint(3, motor, chassis, motor->GetMatrix())
	,m_vehicle(nullptr)
	,m_omega(ndFloat32(0.0f))
	,m_maxOmega(ndFloat32(100.0f))
	,m_omegaStep(ndFloat32(16.0f))
	,m_targetOmega(ndFloat32(0.0f))
	,m_engineTorque(ndFloat32(0.0f))
	,m_internalFriction(ndFloat32(100.0f))
{
}

void ndMultiBodyVehicleMotor::SetVehicleOwner(ndMultiBodyVehicle* const vehicle)
{
	m_vehicle = vehicle;
}

void ndMultiBodyVehicleMotor::AlignMatrix()
{
	ndMatrix matrix0;
	ndMatrix matrix1;
	CalculateGlobalMatrix(matrix0, matrix1);

	m_body0->SetMatrixNoSleep(matrix1);
	m_body0->SetVelocityNoSleep(m_body1->GetVelocity());

	const ndVector omega0(m_body0->GetOmega());
	const ndVector omega1(m_body1->GetOmega());

	const ndVector wx(matrix1.m_front.Scale(matrix1.m_front.DotProduct(omega0).GetScalar()));
	const ndVector wy(matrix1.m_up.Scale(matrix1.m_up.DotProduct(omega1).GetScalar()));
	const ndVector wz(matrix1.m_right.Scale (matrix1.m_right.DotProduct(omega1).GetScalar()));
	const ndVector omega(wx + wy + wz);
	m_body0->SetOmegaNoSleep(omega);
}

void ndMultiBodyVehicleMotor::SetFrictionLoss(ndFloat32 newtonMeters)
{
	m_internalFriction = ndAbs(newtonMeters);
}

void ndMultiBodyVehicleMotor::SetMaxRpm(ndFloat32 redLineRpm)
{
	m_maxOmega = ndMax(redLineRpm / dRadPerSecToRpm, ndFloat32 (0.0f));
}

void ndMultiBodyVehicleMotor::SetOmegaAccel(ndFloat32 rpmStep)
{
	m_omegaStep = ndAbs(rpmStep / dRadPerSecToRpm);
}

void ndMultiBodyVehicleMotor::SetTorqueAndRpm(ndFloat32 newtonMeters, ndFloat32 rpm)
{
	m_engineTorque = ndMax(newtonMeters, ndFloat32(0.0f));
	m_targetOmega = ndClamp(rpm / dRadPerSecToRpm, ndFloat32(0.0f), m_maxOmega);
}

ndFloat32 ndMultiBodyVehicleMotor::GetRpm() const
{
	return m_omega * dRadPerSecToRpm;
}

ndFloat32 ndMultiBodyVehicleMotor::CalculateAcceleration(ndConstraintDescritor& desc)
{
	const ndVector& motorOmega = m_body0->GetOmega();
	const ndJacobian& motorJacobian = desc.m_jacobian[desc.m_rowsCount - 1].m_jacobianM0;
	const ndVector relOmega(motorOmega * motorJacobian.m_angular);

	ndFloat32 currentOmega = relOmega.AddHorizontal().GetScalar();
	if (currentOmega < ndFloat32(0.0f))
	{
		const ndVector clippedOmega(motorOmega - motorJacobian.m_angular * relOmega);
		m_body0->SetOmega(clippedOmega);
		currentOmega = 0;
	}

	m_omega = currentOmega;
	ndFloat32 omegaStep = ndClamp(m_targetOmega - m_omega, -m_omegaStep, m_omegaStep);
	ndFloat32 accel = omegaStep * desc.m_invTimestep;
	return accel;
}

void ndMultiBodyVehicleMotor::JacobianDerivative(ndConstraintDescritor& desc)
{
	ndMatrix matrix0;
	ndMatrix matrix1;
	CalculateGlobalMatrix(matrix0, matrix1);
	
	// two rows to restrict rotation around around the parent coordinate system
	const ndFloat32 angle0 = CalculateAngle(matrix0.m_front, matrix1.m_front, matrix1.m_up);
	const ndFloat32 angle1 = CalculateAngle(matrix0.m_front, matrix1.m_front, matrix1.m_right);
	
	AddAngularRowJacobian(desc, matrix1.m_up, angle0);
	AddAngularRowJacobian(desc, matrix1.m_right, angle1);
	
	// add rotor joint acceleration
	AddAngularRowJacobian(desc, matrix0.m_front * ndVector::m_negOne, ndFloat32(0.0f));
	
	const ndFloat32 accel = CalculateAcceleration(desc);
	const ndFloat32 torque = ndMax(m_engineTorque, m_internalFriction);
	SetMotorAcceleration(desc, accel);
	SetHighFriction(desc, torque);
	SetLowerFriction(desc, -m_internalFriction);
	SetDiagonalRegularizer(desc, ndFloat32(0.1f));
	
	// add torque coupling to chassis.
	//ndMultiBodyVehicleGearBox* const gearBox = m_vehicle ? m_vehicle->m_gearBox : nullptr;
	if (m_vehicle && m_vehicle->m_gearBox && (ndAbs(m_vehicle->m_gearBox->GetRatio()) > ndFloat32(0.0f)))
	{
		ndJacobian& chassisJacobian = desc.m_jacobian[desc.m_rowsCount - 1].m_jacobianM1;
		chassisJacobian.m_angular = ndVector::m_zero;
	}
}
