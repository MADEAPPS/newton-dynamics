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
#include "ndWorld.h"
#include "ndBodyDynamic.h"
#include "ndMultiBodyVehicleRotor.h"

// approximately the a 10 cylinder 450 hp viper engine fore the late nineties
#define D_ENGINE_NOMINAL_TORQUE (dFloat32(600.0f))
#define D_ENGINE_NOMINAL_RPM (dFloat32(9000.0f / 9.55f))

ndMultiBodyVehicleRotor::ndMultiBodyVehicleRotor(ndBodyKinematic* const motor, ndWorld* const world)
	:ndJointBilateralConstraint(1, motor, world->GetSentinelBody(), motor->GetMatrix())
	,m_omega(dFloat32(0.0f))
	,m_maxOmega(D_ENGINE_NOMINAL_RPM)
	,m_idleOmega(D_ENGINE_NOMINAL_RPM * dFloat32(0.1f))
	,m_throttle(dFloat32(0.0f))
	,m_gasValve(D_ENGINE_NOMINAL_RPM * dFloat32(0.02f))
	,m_engineTorque(D_ENGINE_NOMINAL_TORQUE)
	,m_startEngine(false)
{
	SetSolverModel(m_jointkinematicCloseLoop);
}

void ndMultiBodyVehicleRotor::SetGasValve(dFloat32 gasValveSeconds)
{
	m_gasValve = dAbs(gasValveSeconds);
}

void ndMultiBodyVehicleRotor::SetStart(bool startkey)
{
	m_startEngine = startkey;
}


void ndMultiBodyVehicleRotor::SetMaxRpm(dFloat32)
{
	dAssert(0);
}

void ndMultiBodyVehicleRotor::SetEngineTorque(dFloat32)
{
	dAssert(0);
}

void ndMultiBodyVehicleRotor::SetThrottle(dFloat32 param)
{
	m_throttle = dClamp(param, dFloat32(0.0f), dFloat32(1.0f));
}

dFloat32 ndMultiBodyVehicleRotor::CalculateAcceleration(ndConstraintDescritor& desc)
{
	const dVector& omega0 = m_body0->GetOmega();
	const ndJacobian& jacobian0 = desc.m_jacobian[desc.m_rowsCount - 1].m_jacobianM0;
	const dVector relOmega(omega0 * jacobian0.m_angular);
	m_omega = -relOmega.AddHorizontal().GetScalar();

	dFloat32 desiredSpeed = m_omega;
	dFloat32 diff = m_throttle * m_maxOmega - m_omega;

	if (diff > dFloat32(1.0e-3f))
	{
		if (diff <= m_gasValve) 
		{
			diff *= dFloat32 (0.5f);
		}
		desiredSpeed += diff;
	}
	else if (diff < dFloat32(-1.0e-3f))
	{
		if (diff > -m_gasValve)
		{
			diff *= dFloat32(0.5f);
		}
		desiredSpeed += diff;
	}

	desiredSpeed = dClamp(desiredSpeed, m_idleOmega, m_maxOmega);
	return (m_omega - desiredSpeed) * desc.m_invTimestep;
}

void ndMultiBodyVehicleRotor::JacobianDerivative(ndConstraintDescritor& desc)
{
	dMatrix matrix0;
	dMatrix matrix1;
	CalculateGlobalMatrix(matrix0, matrix1);

	// add rotor joint
	AddAngularRowJacobian(desc, matrix0.m_front, dFloat32(0.0f));
	const dFloat32 accel = CalculateAcceleration(desc);
	if (m_startEngine)
	{
		// set engine gas and save the current joint Omega
		SetHighFriction(desc, m_engineTorque);
		SetLowerFriction(desc, -m_engineTorque);
		SetMotorAcceleration(desc, accel);
	}
	else
	{
		SetHighFriction(desc, m_engineTorque * dFloat32 (4.0f));
		SetLowerFriction(desc, -m_engineTorque * dFloat32(4.0f));
		SetMotorAcceleration(desc, m_omega * desc.m_invTimestep);
	}
}
