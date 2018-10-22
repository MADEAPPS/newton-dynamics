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
	,m_friction(0.7f)
{
	m_jointFeebackForce[0] = 0.0f;
	m_jointFeebackForce[1] = 0.0f;
	m_jointFeebackForce[2] = 0.0f;
}

void dTireContact::SetContact(const dVector& posit, const dVector& normal, const dVector& lateralDir, dFloat penetration, dFloat friction)
{
	m_point = posit;
	m_normal = normal;
	m_lateralDir = lateralDir;
	m_longitudinalDir = m_normal.CrossProduct(m_lateralDir);

	m_isActive = true;
	m_friction = friction;
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

static int xxx;
xxx ++;
if (xxx > 4637){
xxx *=1;
if (GetOwner0()->GetIndex() == 0)
xxx *=1;
}

	dFloat lateralSlip = 0.0f;
	dFloat longitudialSlip = 0.0f;
	{
		// longitudinal force row
		AddLinearRowJacobian(constraintParams, m_point, m_longitudinalDir, omega);
		const dComplementaritySolver::dJacobian &jacobian0 = constraintParams->m_jacobians[n].m_jacobian_J01;
		const dComplementaritySolver::dJacobian &jacobian1 = constraintParams->m_jacobians[n].m_jacobian_J10;

		constraintParams->m_normalIndex[n] = -n;
		constraintParams->m_jointLowFrictionCoef[n] = -m_friction;
		constraintParams->m_jointHighFrictionCoef[n] = m_friction;
		
		dVector linearVeloc(veloc0 * jacobian0.m_linear + veloc1 * jacobian1.m_linear + omega1 * jacobian1.m_angular);

		dFloat omegaSpeed = omega0.DotProduct3(jacobian0.m_angular);
		dFloat linearSpeed = linearVeloc.m_x + linearVeloc.m_y + linearVeloc.m_z;

		dFloat relSpeed = omegaSpeed + linearSpeed;
		omegaSpeed = dAbs (omegaSpeed);
		linearSpeed = dAbs (linearSpeed);
		if (!((omegaSpeed < 0.1f) && (linearSpeed < 0.1f))) {
			if (relSpeed < 0.0f) {
				dFloat speedDen = dMax (linearSpeed, 0.01f);
				longitudialSlip = relSpeed / speedDen;
				dFloat slipLimit = 2.0f * dMax (linearSpeed, 0.0f);

				relSpeed = 0.0f;
				if ((omegaSpeed - slipLimit) > 0.0f) {
					relSpeed = -(omegaSpeed - slipLimit);
				}
			} else {
				dFloat speedDen = dMax(omegaSpeed, 0.01f);
				longitudialSlip = relSpeed / speedDen;
				dFloat slipLimit = 2.0f * dMax(omegaSpeed, 0.0f);

				relSpeed = 0.0f;
				if ((linearSpeed - slipLimit) > 0.0f) {
					relSpeed = linearSpeed - slipLimit;
				} 
			}
		}

		//const dVector relVeloc(veloc0 * jacobian0.m_linear + omega0 * jacobian0.m_angular + veloc1 * jacobian1.m_linear + omega1 * jacobian1.m_angular);
		//dFloat relSpeed = relVeloc.m_x + relVeloc.m_y + relVeloc.m_z;
		constraintParams->m_jointAccel[n] = -relSpeed * constraintParams->m_timestepInv;
		n++;
	}

	{
		// lateral force row
		AddLinearRowJacobian(constraintParams, m_point, m_lateralDir, omega);
		const dComplementaritySolver::dJacobian &jacobian0 = constraintParams->m_jacobians[n].m_jacobian_J01;
		const dComplementaritySolver::dJacobian &jacobian1 = constraintParams->m_jacobians[n].m_jacobian_J10;

		constraintParams->m_normalIndex[n] = -n;
		constraintParams->m_jointLowFrictionCoef[n] = -m_friction;
		constraintParams->m_jointHighFrictionCoef[n] = m_friction;

		const dVector relVeloc(veloc0 * jacobian0.m_linear + omega0 * jacobian0.m_angular + veloc1 * jacobian1.m_linear + omega1 * jacobian1.m_angular);
		dFloat lateralSpeed = relVeloc.m_x + relVeloc.m_y + relVeloc.m_z;
		dFloat longitudinalSpeed = dAbs (m_longitudinalDir.DotProduct3(relVeloc)) + 0.01f;

		lateralSlip = lateralSpeed / longitudinalSpeed;
		dFloat maxLateralSpeed = D_TIRE_MAX_LATERAL_SLIP * longitudinalSpeed;
		dFloat slideSpeed = lateralSpeed - dClamp (lateralSpeed, - maxLateralSpeed, maxLateralSpeed);
		constraintParams->m_jointAccel[n] = -slideSpeed * constraintParams->m_timestepInv;
		n++;
	}

dTrace (("(%d =%f) ", GetOwner0()->GetIndex(), longitudialSlip));
if (GetOwner0()->GetIndex() == 3)
dTrace (("\n"));

	m_dof = n;
	m_count = n;
	constraintParams->m_count = n;
}

