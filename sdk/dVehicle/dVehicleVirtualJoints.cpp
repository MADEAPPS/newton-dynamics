/* Copyright (c) <2003-2016> <Newton Game Dynamics>
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
#include "dVehicleNode.h"
#include "dVehicleVirtualJoints.h"


dKinematicLoopJoint::dKinematicLoopJoint()
	:dComplementaritySolver::dBilateralJoint()
	,m_owner0(NULL)
	,m_owner1(NULL)
	,m_isActive(false)
{
}

void dKinematicLoopJoint::SetOwners(dVehicleNode* const owner0, dVehicleNode* const owner1)
{
	m_owner0 = owner0;
	m_owner1 = owner1;
	Init(m_owner0->GetBody(), m_owner1->GetBody());
}


dTireJoint::dTireJoint()
	:dComplementaritySolver::dBilateralJoint()
{
}

void dTireJoint::JacobianDerivative(dComplementaritySolver::dParamInfo* const constraintParams)
{
	dComplementaritySolver::dBodyState* const tire = m_state0;
	dComplementaritySolver::dBodyState* const chassis = m_state1;

	const dVector& omega = chassis->GetOmega();
	const dMatrix& tireMatrix = tire->GetMatrix();

	// lateral force
	AddLinearRowJacobian(constraintParams, tireMatrix.m_posit, tireMatrix.m_front, omega);

	// longitudinal force
	AddLinearRowJacobian(constraintParams, tireMatrix.m_posit, tireMatrix.m_right, omega);

	// angular constraints	
	AddAngularRowJacobian(constraintParams, tireMatrix.m_up, omega, 0.0f);
	AddAngularRowJacobian(constraintParams, tireMatrix.m_right, omega, 0.0f);

	/*
	// dry rolling friction (for now contact, bu it should be a function of the tire angular velocity)
	int index = constraintParams->m_count;
	AddAngularRowJacobian(constraintParams, tire->m_matrix[0], 0.0f);
	constraintParams->m_jointLowFriction[index] = -chassis->m_dryRollingFrictionTorque;
	constraintParams->m_jointHighFriction[index] = chassis->m_dryRollingFrictionTorque;

	// check if the brakes are applied
	if (tire->m_brakeTorque > 1.0e-3f) {
	// brake is on override rolling friction value
	constraintParams->m_jointLowFriction[index] = -tire->m_brakeTorque;
	constraintParams->m_jointHighFriction[index] = tire->m_brakeTorque;
	}

	// clear the input variable after there are res
	tire->m_brakeTorque = 0.0f;
	*/
}


dTireContact::dTireContact()
	:dKinematicLoopJoint()
	,m_point(0.0f)
	,m_normal(0.0f)
	,m_lateralDir(0.0f)
	,m_longitudinalDir(0.0f)
	,m_penetration(0.0f)
{
	m_jointFeebackForce[0] = 0.0f;
	m_jointFeebackForce[1] = 0.0f;
	m_jointFeebackForce[2] = 0.0f;
}

void dTireContact::SetContact(const dMatrix& contact, dFloat penetration)
{
	m_isActive = true;
	m_point = contact.m_posit;
	m_normal = contact.m_front;
	m_lateralDir = contact.m_right;
	m_longitudinalDir = contact.m_up;

	m_penetration = dClamp (penetration, dFloat(-D_TIRE_MAX_ELASTIC_DEFORMATION), dFloat(D_TIRE_MAX_ELASTIC_DEFORMATION));
}

void dTireContact::JacobianDerivative(dComplementaritySolver::dParamInfo* const constraintParams)
{
	dVector omega(0.0f);

	// normal constraint
	const dVector& veloc0 = m_state0->GetVelocity();
	const dVector& omega0 = m_state0->GetOmega();
	const dVector& veloc1 = m_state1->GetVelocity();
	const dVector& omega1 = m_state1->GetOmega();

	int n = 0;
	{
		// normal force row
		AddLinearRowJacobian(constraintParams, m_point, m_normal, omega);
		const dComplementaritySolver::dJacobian &jacobian0 = constraintParams->m_jacobians[n].m_jacobian_J01;
		const dComplementaritySolver::dJacobian &jacobian1 = constraintParams->m_jacobians[n].m_jacobian_J10;
		const dVector relVeloc(veloc0 * jacobian0.m_linear + omega0 * jacobian0.m_angular + veloc1 * jacobian1.m_linear + omega1 * jacobian1.m_angular);
		dFloat relSpeed = -(relVeloc.m_x + relVeloc.m_y + relVeloc.m_z);
		relSpeed += D_TIRE_MAX_ELASTIC_NORMAL_STIFFNESS * m_penetration;
		constraintParams->m_jointLowFrictionCoef[n] = 0.0f;
		constraintParams->m_jointAccel[n] = relSpeed * constraintParams->m_timestepInv;
		n++;
	}

	{
		// longitudinal force row
		AddLinearRowJacobian(constraintParams, m_point, m_longitudinalDir, omega);
		const dComplementaritySolver::dJacobian &jacobian0 = constraintParams->m_jacobians[n].m_jacobian_J01;
		const dComplementaritySolver::dJacobian &jacobian1 = constraintParams->m_jacobians[n].m_jacobian_J10;
		const dVector relVeloc(veloc0 * jacobian0.m_linear + omega0 * jacobian0.m_angular + veloc1 * jacobian1.m_linear + omega1 * jacobian1.m_angular);
		dFloat relSpeed = relVeloc.m_x + relVeloc.m_y + relVeloc.m_z;


		constraintParams->m_jointAccel[n] = -relSpeed * constraintParams->m_timestepInv;
		constraintParams->m_normalIndex[n] = -n;
		constraintParams->m_jointLowFrictionCoef[n] = -0.7f;
		constraintParams->m_jointHighFrictionCoef[n] = 0.7f;
		n++;
	}

	dFloat lateralSlip = 0.0f;
	{
		// lateral force row
		AddLinearRowJacobian(constraintParams, m_point, m_lateralDir, omega);
		const dComplementaritySolver::dJacobian &jacobian0 = constraintParams->m_jacobians[n].m_jacobian_J01;
		const dComplementaritySolver::dJacobian &jacobian1 = constraintParams->m_jacobians[n].m_jacobian_J10;
		constraintParams->m_normalIndex[n] = -n;
		constraintParams->m_jointLowFrictionCoef[n] = -0.7f;
		constraintParams->m_jointHighFrictionCoef[n] = 0.7f;

		const dVector relVeloc(veloc0 * jacobian0.m_linear + omega0 * jacobian0.m_angular + veloc1 * jacobian1.m_linear + omega1 * jacobian1.m_angular);
		dFloat lateralSpeed = relVeloc.m_x + relVeloc.m_y + relVeloc.m_z;
		dFloat longitudinalSpeed = dAbs (m_longitudinalDir.DotProduct3(relVeloc)) + 0.01f;

		lateralSlip = lateralSpeed / longitudinalSpeed;
		dFloat maxLateralSpeed = D_TIRE_MAX_LATERAL_SLIP * longitudinalSpeed;
		dFloat slideSpeed = lateralSpeed - dClamp (lateralSpeed, - maxLateralSpeed, maxLateralSpeed);

		constraintParams->m_jointAccel[n] = -slideSpeed * constraintParams->m_timestepInv;

		n++;
	}



	m_dof = n;
	m_count = n;
	constraintParams->m_count = n;
}

