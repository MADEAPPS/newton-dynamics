/* Copyright (c) <2003-2019> <Julio Jerez, Newton Game Dynamics>
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

#include "dCoreStdafx.h"
#include "ndNewtonStdafx.h"
#include "ndJointWheel.h"
#include "ndBodyDynamic.h"
#include "ndMultiBodyVehicle.h"
#include "ndMultiBodyVehicleMotor.h"
#include "ndMultiBodyVehicleGearBox.h"

#define D_ENGINE_NOMINAL_TORQUE (dFloat32(900.0f))
#define D_ENGINE_NOMINAL_RPM (dFloat32(9000.0f / 9.55f))

ndMultiBodyVehicleMotor::ndMultiBodyVehicleMotor(ndBodyKinematic* const motor, ndMultiBodyVehicle* const vehicelModel)
	:ndJointBilateralConstraint(3, motor, vehicelModel->m_chassis, motor->GetMatrix())
	,m_omega(dFloat32(0.0f))
	,m_maxOmega(D_ENGINE_NOMINAL_RPM)
	,m_idleOmega(D_ENGINE_NOMINAL_RPM * dFloat32(0.1f))
	,m_throttle(dFloat32(0.0f))
	,m_gasValve(D_ENGINE_NOMINAL_RPM * dFloat32(0.02f))
	,m_engineTorque(D_ENGINE_NOMINAL_TORQUE)
	,m_vehicelModel(vehicelModel)
	,m_startEngine(false)
{
}

void ndMultiBodyVehicleMotor::AlignMatrix()
{
	dMatrix matrix0;
	dMatrix matrix1;
	CalculateGlobalMatrix(matrix0, matrix1);

	//matrix1.m_posit += matrix1.m_up.Scale(1.0f);

	m_body0->SetMatrix(matrix1);
	m_body0->SetVelocity(m_body1->GetVelocity());

	const dVector omega0(m_body0->GetOmega());
	const dVector omega1(m_body1->GetOmega());

	const dVector wx(matrix1.m_front.Scale(matrix1.m_front.DotProduct(omega0).GetScalar()));
	const dVector wy(matrix1.m_up.Scale(matrix1.m_up.DotProduct(omega1).GetScalar()));
	const dVector wz(matrix1.m_right.Scale (matrix1.m_right.DotProduct(omega1).GetScalar()));
	const dVector omega(wx + wy + wz);

	//dVector error(omega1 - omega);
	//dTrace(("(%f %f %f)\n", error.m_x, error.m_y, error.m_z));
	m_body0->SetOmega(omega);
}

void ndMultiBodyVehicleMotor::SetGasValve(dFloat32 gasValve)
{
	m_gasValve = dAbs(gasValve);
}

void ndMultiBodyVehicleMotor::SetStart(bool startkey)
{
	m_startEngine = startkey;
}

void ndMultiBodyVehicleMotor::SetMaxRpm(dFloat32)
{
	dAssert(0);
}

void ndMultiBodyVehicleMotor::SetEngineTorque(dFloat32)
{
	dAssert(0);
}

void ndMultiBodyVehicleMotor::SetThrottle(dFloat32 param)
{
	m_throttle = dClamp(param, dFloat32(0.0f), dFloat32(1.0f));
}

dFloat32 ndMultiBodyVehicleMotor::CalculateAcceleration(ndConstraintDescritor& desc)
{
	const dVector& omega0 = m_body0->GetOmega();
	const ndJacobian& jacobian0 = desc.m_jacobian[desc.m_rowsCount - 1].m_jacobianM0;
	const dVector relOmega(omega0 * jacobian0.m_angular);

	m_omega = -relOmega.AddHorizontal().GetScalar();
	const dFloat32 throttleOmega = dClamp(m_throttle * m_maxOmega, m_idleOmega, m_maxOmega);
	const dFloat32 deltaOmega = throttleOmega - m_omega;
	dFloat32 omegaError = dClamp(deltaOmega, -m_gasValve, m_gasValve);
	//dTrace(("%f %f\n", throttleOmega, m_omega));
	return -omegaError * desc.m_invTimestep;
}

void ndMultiBodyVehicleMotor::JacobianDerivative(ndConstraintDescritor& desc)
{
	dMatrix matrix0;
	dMatrix matrix1;
	CalculateGlobalMatrix(matrix0, matrix1);

	// two rows to restrict rotation around around the parent coordinate system
	const dFloat32 angle0 = CalculateAngle(matrix0.m_front, matrix1.m_front, matrix1.m_up);
	const dFloat32 angle1 = CalculateAngle(matrix0.m_front, matrix1.m_front, matrix1.m_right);

	AddAngularRowJacobian(desc, matrix1.m_up, angle0);
	AddAngularRowJacobian(desc, matrix1.m_right, angle1);

	// add rotor joint
	AddAngularRowJacobian(desc, matrix0.m_front, dFloat32(0.0f));
	const dFloat32 accel = CalculateAcceleration(desc);
	if (m_startEngine)
	{
		const ndMultiBodyVehicleGearBox* const gearBox = m_vehicelModel->m_gearBox;
		if (gearBox && dAbs(gearBox->GetRatio()) > dFloat32(0.0f))
		{
			ndJacobian& jacobian = desc.m_jacobian[desc.m_rowsCount - 1].m_jacobianM1;
			jacobian.m_angular = dVector::m_zero;
		}

		if (m_omega <= dFloat32(0.0f))
		{
			// engine rpm can not be negative
			dFloat32 stopAccel = (m_omega - 0.5f) * desc.m_invTimestep;
			SetMotorAcceleration(desc, stopAccel);
			SetHighFriction(desc, m_engineTorque);
		}
		else if (m_omega >= m_maxOmega)
		{
			// engine rpm can not pass maximum allowed
			dFloat32 stopAccel = (m_omega - m_maxOmega) * desc.m_invTimestep;
			SetMotorAcceleration(desc, stopAccel);
			SetLowerFriction(desc, -m_engineTorque);
		}
		else
		{
			// set engine gas and save the current joint omega
			SetMotorAcceleration(desc, accel);
			SetHighFriction(desc, m_engineTorque);
			SetLowerFriction(desc, -m_engineTorque);
		}
	}
	else
	{
		SetHighFriction(desc, m_engineTorque * dFloat32(4.0f));
		SetLowerFriction(desc, -m_engineTorque * dFloat32(4.0f));
		SetMotorAcceleration(desc, m_omega * desc.m_invTimestep);
	}
}

