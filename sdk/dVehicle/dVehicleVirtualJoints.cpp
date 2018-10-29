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
#include "dVehicleChassis.h"
#include "dVehicleSingleBody.h"
#include "dVehicleVirtualTire.h"
#include "dVehicleTireInterface.h"
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
}


dTireContact::dTireContact()
	:dKinematicLoopJoint()
	,m_point(0.0f)
	,m_normal(0.0f)
	,m_lateralDir(0.0f)
	,m_longitudinalDir(0.0f)
	,m_penetration(0.0f)
	,m_staticFriction(1.0f)
	,m_kineticFriction(1.0f)
	,m_load(0.0f)
	,m_tireModel()
{
	m_jointFeebackForce[0] = 0.0f;
	m_jointFeebackForce[1] = 0.0f;
	m_jointFeebackForce[2] = 0.0f;
	memset(m_normalFilter, 0, sizeof(m_normalFilter));
	memset(m_isActiveFilter, 0, sizeof(m_isActiveFilter));
}

void dTireContact::ResetContact ()
{
	if (m_isActive == false) {
		m_jointFeebackForce[0] = 0.0f;
		m_jointFeebackForce[1] = 0.0f;
		m_jointFeebackForce[2] = 0.0f;
		memset(m_normalFilter, 0, sizeof(m_normalFilter));
		memset(m_isActiveFilter, 0, sizeof(m_isActiveFilter));
	}
	m_load = 0.0f;
	m_isActive = false;
	for (int i = sizeof(m_isActiveFilter) / sizeof(m_isActiveFilter[0]) - 1; i > 0; i--) {
		m_normalFilter[i] = m_normalFilter[i - 1];
		m_isActiveFilter[i] = m_isActiveFilter[i - 1];
	}
}

void dTireContact::SetContact(const dVector& posit, const dVector& normal, const dVector& longitudinalDir, dFloat penetration, dFloat staticFriction, dFloat kineticFriction)
{
	m_point = posit;
	m_normal = normal;
	m_longitudinalDir = longitudinalDir;
	m_lateralDir = m_longitudinalDir.CrossProduct(m_normal);

	m_isActive = true;
	m_isActiveFilter[0] = true;
	m_normalFilter[0] = m_jointFeebackForce[0];

	dFloat load = 0.0f;
	for (int i = 0; i < sizeof(m_isActiveFilter) / sizeof(m_isActiveFilter[0]); i++) {
		load += m_normalFilter[i];
	}
	m_load = load * (1.0f / (sizeof(m_isActiveFilter) / sizeof(m_isActiveFilter[0])));

	m_staticFriction = staticFriction;
	m_kineticFriction = kineticFriction;
	m_penetration = dClamp (penetration, dFloat(-D_TIRE_MAX_ELASTIC_DEFORMATION), dFloat(D_TIRE_MAX_ELASTIC_DEFORMATION));
}

void dTireContact::TireForces(dFloat longitudinalSlip, dFloat lateralSlip, dFloat frictionCoef)
{
	dVehicleTireInterface* const tire = GetOwner0()->GetAsTire();
	dAssert (tire);

	const dVehicleTireInterface::dTireInfo& tireInfo = tire->GetInfo();

	dFloat v = dAbs (lateralSlip);
	dFloat u = dAbs (longitudinalSlip);
	dFloat f = frictionCoef * m_load;

	dFloat invden = 1.0f / (1.0f + u);

	dFloat y = tireInfo.m_corneringStiffness * v * invden;
	dFloat x = tireInfo.m_longitudinalStiffness * u * invden;
	dFloat mag = dSqrt (x * x + y * y);

	if (mag < (3.0f * f)) {
		dFloat den = dMax (f, dFloat(1.0f));
		f = mag * (1.0f - mag / (3.0f * den) + mag * mag / (27 * den * den));
	} 
	mag = dMax (mag, dFloat(1.0f));

	m_tireModel.m_alingMoment = 0.0f;
	m_tireModel.m_lateralForce = y * f / mag;
	m_tireModel.m_longitunalForce = x * f / mag;
	dAssert(m_tireModel.m_lateralForce >= 0.0f);
	dAssert(m_tireModel.m_longitunalForce >= 0.0f);
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

	int lateralIndex = 0;
	int longitudinaIndex = 0;
	dFloat lateralSlip = 0.0f;
	dFloat longitudialSlip = 0.0f;
	dFloat frictionCoef = m_staticFriction;
	{
		// longitudinal force row
		AddLinearRowJacobian(constraintParams, m_point, m_longitudinalDir, omega);
		const dComplementaritySolver::dJacobian &jacobian0 = constraintParams->m_jacobians[n].m_jacobian_J01;
		const dComplementaritySolver::dJacobian &jacobian1 = constraintParams->m_jacobians[n].m_jacobian_J10;

		dVector linearVeloc(veloc0 * jacobian0.m_linear + veloc1 * jacobian1.m_linear + omega1 * jacobian1.m_angular);
		dFloat omegaSpeed = omega0.DotProduct3(jacobian0.m_angular);
		dFloat linearSpeed = linearVeloc.m_x + linearVeloc.m_y + linearVeloc.m_z;
		dFloat relSpeed = omegaSpeed + linearSpeed;

		omegaSpeed = dAbs (omegaSpeed);
		linearSpeed = dAbs (linearSpeed);
		if (!((omegaSpeed < 0.1f) && (linearSpeed < 0.1f))) {
			if (relSpeed < 0.0f) {
				dFloat speedDen = dMax (linearSpeed, dFloat(0.01f));
				longitudialSlip = relSpeed / speedDen;
				dFloat slipLimit = 2.0f * dMax (linearSpeed, dFloat(0.0f));
				if ((omegaSpeed - slipLimit) > 0.0f) {
					frictionCoef = m_kineticFriction;
				}
			} else {
				dFloat speedDen = dMax(omegaSpeed, dFloat(0.01f));
				//longitudialSlip = relSpeed / speedDen;
				longitudialSlip = dClamp (relSpeed / speedDen, dFloat(0.0f), dFloat(3.0f));
				dFloat slipLimit = 2.0f * dMax(omegaSpeed, dFloat(0.0f));
				if ((linearSpeed - slipLimit) > 0.0f) {
					frictionCoef = m_kineticFriction;
				} 
			}
		}

		constraintParams->m_jointAccel[n] = -relSpeed * constraintParams->m_timestepInv;
		longitudinaIndex = n;
		n++;
	}

	{
		// lateral force row
		AddLinearRowJacobian(constraintParams, m_point, m_lateralDir, omega);
		const dComplementaritySolver::dJacobian &jacobian0 = constraintParams->m_jacobians[n].m_jacobian_J01;
		const dComplementaritySolver::dJacobian &jacobian1 = constraintParams->m_jacobians[n].m_jacobian_J10;

		const dVector relVeloc(veloc0 * jacobian0.m_linear + omega0 * jacobian0.m_angular + veloc1 * jacobian1.m_linear + omega1 * jacobian1.m_angular);
		dFloat lateralSpeed = relVeloc.m_x + relVeloc.m_y + relVeloc.m_z;
		dFloat longitudinalSpeed = dAbs (m_longitudinalDir.DotProduct3(relVeloc)) + 0.01f;
		lateralSlip = lateralSpeed / longitudinalSpeed;

		dFloat maxLateralSpeed = D_TIRE_MAX_LATERAL_SLIP * longitudinalSpeed;
		if (lateralSpeed > maxLateralSpeed) {
			frictionCoef = m_kineticFriction;
		} else if (lateralSpeed < -maxLateralSpeed) {
			frictionCoef = m_kineticFriction;
		}
		
		constraintParams->m_jointAccel[n] = -lateralSpeed * constraintParams->m_timestepInv;
		lateralIndex = n;
		n ++;
	}

	TireForces(longitudialSlip, lateralSlip, frictionCoef);

	constraintParams->m_jointLowFrictionCoef[lateralIndex] = -m_tireModel.m_lateralForce;
	constraintParams->m_jointHighFrictionCoef[lateralIndex] = m_tireModel.m_lateralForce;
	constraintParams->m_jointLowFrictionCoef[longitudinaIndex] = -m_tireModel.m_longitunalForce;
	constraintParams->m_jointHighFrictionCoef[longitudinaIndex] = m_tireModel.m_longitunalForce;

//dTrace (("(%d = %f) ", GetOwner0()->GetIndex(), longitudialSlip));
//dTrace (("(%d = %f %f %f) ", GetOwner0()->GetIndex(), lateralSlip, m_tireModel.m_lateralForce, m_load));
//dTrace (("(%d = %f %f %f) ", GetOwner0()->GetIndex(), longitudialSlip, m_tireModel.m_longitunalForce, m_load));
//if (GetOwner0()->GetIndex() == 3)
//dTrace (("\n"));

	m_dof = n;
	m_count = n;
	constraintParams->m_count = n;
}

