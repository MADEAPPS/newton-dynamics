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

#include "dStdafxVehicle.h"
#include "dVehicleTire.h"
#include "dVehicleTireJoint.h"

// *******************************************************************
// tire
// *******************************************************************

dTireJoint::dTireJoint()
	:dComplementaritySolver::dBilateralJoint()
	,m_tire(NULL)
{
}

void dTireJoint::JacobianDerivative(dComplementaritySolver::dParamInfo* const constraintParams)
{
	dAssert (0);
/*
	dComplementaritySolver::dBodyState* const tire = m_state0;
	dComplementaritySolver::dBodyState* const chassis = m_state1;

	const dVector& omega = chassis->GetOmega();
	const dMatrix& tireMatrix = tire->GetMatrix();

	// lateral force
	AddLinearRowJacobian(constraintParams, tireMatrix.m_posit, tireMatrix.m_front, omega);

	// longitudinal force
	AddLinearRowJacobian(constraintParams, tireMatrix.m_posit, tireMatrix.m_up, omega);

	// angular constraints	
	AddAngularRowJacobian(constraintParams, tireMatrix.m_up, omega, 0.0f);
	AddAngularRowJacobian(constraintParams, tireMatrix.m_right, omega, 0.0f);

	// dry rolling friction (for now contact, but it should be a function of the tire angular velocity)
	//int index = constraintParams->m_count;
	//AddAngularRowJacobian(constraintParams, tire->m_matrix[0], 0.0f);
	//constraintParams->m_jointLowFriction[index] = -chassis->m_dryRollingFrictionTorque;
	//constraintParams->m_jointHighFriction[index] = chassis->m_dryRollingFrictionTorque;

	dFloat brakeTorque = m_tire->GetBrakeTorque();
//brakeTorque = 1000;
	if (brakeTorque > 1.0e-3f) {

		int index = constraintParams->m_count;
		AddAngularRowJacobian(constraintParams, tireMatrix.m_front, omega, 0.0f);

		const dVector& omega0 = m_state0->GetOmega();
		const dVector& omega1 = m_state1->GetOmega();
		const dComplementaritySolver::dJacobian &jacobian0 = constraintParams->m_jacobians[index].m_jacobian_J01;
		const dComplementaritySolver::dJacobian &jacobian1 = constraintParams->m_jacobians[index].m_jacobian_J10;
		const dVector relVeloc(omega0 * jacobian0.m_angular + omega1 * jacobian1.m_angular);
		dFloat relOmega = -(relVeloc.m_x + relVeloc.m_y + relVeloc.m_z);
		constraintParams->m_jointAccel[index] = relOmega * constraintParams->m_timestepInv;
		constraintParams->m_jointLowFrictionCoef[index] = -brakeTorque;
		constraintParams->m_jointHighFrictionCoef[index] = brakeTorque;
	}
	m_tire->SetBrakeTorque(0.0f);
*/
}


