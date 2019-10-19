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
#include "dVehicleTireContact.h"

dVehicleTireContact::dVehicleTireContact()
	:dVehicleLoopJoint()
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

void dVehicleTireContact::ResetContact()
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

void dVehicleTireContact::SetContact(const dVector& posit, const dVector& normal, const dVector& longitudinalDir, dFloat penetration, dFloat staticFriction, dFloat kineticFriction)
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
	m_penetration = dClamp(penetration, dFloat(-D_TIRE_MAX_ELASTIC_DEFORMATION), dFloat(D_TIRE_MAX_ELASTIC_DEFORMATION));
}

void dVehicleTireContact::JacobianDerivative(dComplementaritySolver::dParamInfo* const constraintParams)
{
	//dAssert (0);
	dTrace(("%s\n", __FUNCTION__));

#if 0
	FILE* file_xxx = fopen("xxxxx.csv", "wb");
	fprintf(file_xxx, "tireforce\n");
	m_load = 1000.0f;
	for (int i = 0; i < 100; i++) {
		float u = 0.01f * i;
		TireForces(0.0f, u, 1.0f);
		//fprintf(file_xxx, "%f,\n", m_tireModel.m_longitunalForce);
		fprintf(file_xxx, "%f,\n", m_tireModel.m_lateralForce);
	}
	fclose(file_xxx);
#endif

	// normal constraint
	const dVector& veloc0 = m_state0->GetVelocity();
	const dVector& omega0 = m_state0->GetOmega();
	const dVector& veloc1 = m_state1->GetVelocity();
	const dVector& omega1 = m_state1->GetOmega();

//	int n = 0;
	{
		// normal force row
		int index = constraintParams->m_count;
		AddLinearRowJacobian(constraintParams, m_point, m_normal);
		const dComplementaritySolver::dJacobian &jacobian0 = constraintParams->m_jacobians[index].m_jacobian_J01;
		const dComplementaritySolver::dJacobian &jacobian1 = constraintParams->m_jacobians[index].m_jacobian_J10;
		const dVector relVeloc(veloc0 * jacobian0.m_linear + omega0 * jacobian0.m_angular + veloc1 * jacobian1.m_linear + omega1 * jacobian1.m_angular);
		dFloat relSpeed = -(relVeloc.m_x + relVeloc.m_y + relVeloc.m_z);
		relSpeed += D_TIRE_MAX_ELASTIC_NORMAL_STIFFNESS * m_penetration;
		constraintParams->m_jointLowFrictionCoef[index] = 0.0f;
		constraintParams->m_jointAccel[index] = relSpeed * constraintParams->m_timestepInv;
	}


	int lateralIndex = 0;
	int longitudinaIndex = 0;
	dFloat lateralSlip = 0.4f;
	dFloat longitudialSlip = 0.4f;
	dFloat frictionCoef = m_staticFriction;
	{
		// longitudinal force row
		int index = constraintParams->m_count;
		AddLinearRowJacobian(constraintParams, m_point, m_longitudinalDir);
		const dComplementaritySolver::dJacobian &jacobian0 = constraintParams->m_jacobians[index].m_jacobian_J01;
		const dComplementaritySolver::dJacobian &jacobian1 = constraintParams->m_jacobians[index].m_jacobian_J10;

		dVector linearVeloc(veloc0 * jacobian0.m_linear + veloc1 * jacobian1.m_linear + omega1 * jacobian1.m_angular);
		dFloat omegaSpeed = omega0.DotProduct3(jacobian0.m_angular);
		dFloat linearSpeed = linearVeloc.m_x + linearVeloc.m_y + linearVeloc.m_z;
		dFloat relSpeed = omegaSpeed + linearSpeed;

		omegaSpeed = dAbs(omegaSpeed);
		linearSpeed = dAbs(linearSpeed);
		if (!((omegaSpeed < 0.2f) && (linearSpeed < 0.2f))) {
			if (relSpeed < 0.0f) {
				dFloat speedDen = dMax(linearSpeed, dFloat(0.01f));
				longitudialSlip = dClamp(dAbs(relSpeed / speedDen), dFloat(0.0f), dFloat(20.0f));
				dFloat slipLimit = 2.0f * dMax(linearSpeed, dFloat(0.0f));
				if ((omegaSpeed - slipLimit) > 0.0f) {
					frictionCoef = m_kineticFriction;
				}
			} else {
				dFloat speedDen = dMax(omegaSpeed, dFloat(0.01f));
				longitudialSlip = dClamp(dAbs(relSpeed / speedDen), dFloat(0.0f), dFloat(4.0f));
				dFloat slipLimit = 2.0f * dMax(omegaSpeed, dFloat(0.0f));
				if ((linearSpeed - slipLimit) > 0.0f) {
					frictionCoef = m_kineticFriction;
				}
			}
		}

		constraintParams->m_jointAccel[index] = -relSpeed * constraintParams->m_timestepInv;
		longitudinaIndex = index;
	}

#if 0
	{
		// lateral force row
		int index = constraintParams->m_count;
		AddLinearRowJacobian(constraintParams, m_point, m_lateralDir);
		const dComplementaritySolver::dJacobian &jacobian0 = constraintParams->m_jacobians[index].m_jacobian_J01;
		const dComplementaritySolver::dJacobian &jacobian1 = constraintParams->m_jacobians[index].m_jacobian_J10;

		const dVector relVeloc(veloc0 * jacobian0.m_linear + omega0 * jacobian0.m_angular + veloc1 * jacobian1.m_linear + omega1 * jacobian1.m_angular);
		dFloat lateralSpeed = relVeloc.m_x + relVeloc.m_y + relVeloc.m_z;
		dFloat longitudinalSpeed = dAbs(m_longitudinalDir.DotProduct3(relVeloc)) + 0.01f;
		lateralSlip = lateralSpeed / longitudinalSpeed;

		dFloat maxLateralSpeed = D_TIRE_MAX_LATERAL_SLIP * longitudinalSpeed;
		if (lateralSpeed > maxLateralSpeed) {
			frictionCoef = m_kineticFriction;
		} else if (lateralSpeed < -maxLateralSpeed) {
			frictionCoef = m_kineticFriction;
		}

		constraintParams->m_jointAccel[index] = -lateralSpeed * constraintParams->m_timestepInv;
		lateralIndex = index;
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
#endif
	m_dof = constraintParams->m_count;
}

