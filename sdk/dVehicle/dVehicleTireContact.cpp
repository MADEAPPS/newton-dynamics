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
#include "dVehicleNode.h"
#include "dVehicleTire.h"
#include "dVehicleMultiBody.h"
#include "dVehicleTireContact.h"

dVehicleTireContact::dVehicleTireContact()
	:dVehicleLoopJoint()
	,m_point(0.0f)
	,m_normal(0.0f)
	,m_lateralDir(0.0f)
	,m_longitudinalDir(0.0f)
	,m_penetration(0.0f)
	,m_staticFriction(1.0f)
	,m_tireModel()
{
	m_jointFeebackForce[0] = 0.0f;
	m_jointFeebackForce[1] = 0.0f;
	m_jointFeebackForce[2] = 0.0f;
}

void dVehicleTireContact::ResetContact()
{
	if (m_isActive == false) {
		m_jointFeebackForce[0] = 0.0f;
		m_jointFeebackForce[1] = 0.0f;
		m_jointFeebackForce[2] = 0.0f;
	}
	m_isActive = false;
}

void dVehicleTireContact::SetContact(const dVector& posit, const dVector& normal, const dVector& longitudinalDir, dFloat penetration, dFloat staticFriction)
{
	m_point = posit;
	m_normal = normal;
	m_longitudinalDir = longitudinalDir;
	m_lateralDir = m_longitudinalDir.CrossProduct(m_normal);

	m_isActive = true;
	m_staticFriction = staticFriction;
	m_penetration = dClamp(penetration, dFloat(-D_TIRE_MAX_ELASTIC_DEFORMATION), dFloat(D_TIRE_MAX_ELASTIC_DEFORMATION));
}

void dVehicleTireContact::Debug(dCustomJoint::dDebugDisplay* const debugContext, dFloat scale) const
{
	dVehicleTire* const tire = GetOwner0()->GetAsTire();
	dVehicleMultiBody* const chassis = GetOwner0()->GetParent()->GetAsVehicleMultiBody();

	dAssert(tire);
	dAssert(chassis);

	const dMatrix& tireMatrix = tire->GetProxyBody().GetMatrix();
	const dMatrix& chassisMatrix = chassis->GetProxyBody().GetMatrix();

	dVector localPosit(chassisMatrix.UntransformVector(tireMatrix.m_posit));
	dVector origin(m_point + m_normal.Scale(1.0f / 32.0f));
	if (localPosit.m_z > 0.0f) {
		origin += m_lateralDir.Scale(1.0f / 4.0f);
	} else {
		origin -= m_lateralDir.Scale(1.0f / 4.0f);
	}

	scale *= 4.0f;

	// show tire load
	debugContext->SetColor(dVector(0.0f, 0.0f, 1.0f, 1.0f));
	//dVector p1(origin + m_normal.Scale(scale * m_jointFeebackForce[0]));
	dVector p1(origin + m_normal.Scale(scale * m_tireModel.m_tireLoad));
	debugContext->DrawLine(origin, p1);

	scale *= 1.0f;
	// show tire longitudinal force
	debugContext->SetColor(dVector(0.0f, 1.0f, 0.0f, 1.0f));
	dVector p2(origin + m_longitudinalDir.Scale(scale * m_tireModel.m_longitunalForce));
	debugContext->DrawLine(origin, p2);

	// show tire lateral force
	debugContext->SetColor(dVector(1.0f, 0.0f, 0.0f, 1.0f));
	dVector p3(origin + m_lateralDir.Scale(scale * m_tireModel.m_lateralForce));

	debugContext->DrawLine(origin, p3);
}

void dVehicleTireContact::JacobianDerivative(dComplementaritySolver::dParamInfo* const constraintParams)
{
	const dVector& veloc0 = m_state0->GetVelocity();
	const dVector& omega0 = m_state0->GetOmega();
	const dVector& veloc1 = m_state1->GetVelocity();
	const dVector& omega1 = m_state1->GetOmega();
	const dVehicleTire* const tire = GetOwner0()->GetAsTire();

	{
		// normal constraint
		int index = constraintParams->m_count;
		AddContactRowJacobian(constraintParams, m_point, m_normal, 0.0f);
		constraintParams->m_jointLowFrictionCoef[index] = 0.0f;
		constraintParams->m_frictionCallback[index] = this;
		constraintParams->m_jointAccel[index] += D_TIRE_MAX_ELASTIC_NORMAL_STIFFNESS * m_penetration * constraintParams->m_timestepInv;
	}

	{
		// longitudinal force row
		int index = constraintParams->m_count;
		AddContactRowJacobian(constraintParams, m_point, m_longitudinalDir, 0.0f);
		const dComplementaritySolver::dJacobian &jacobian0 = constraintParams->m_jacobians[index].m_jacobian_J01;
		const dComplementaritySolver::dJacobian &jacobian1 = constraintParams->m_jacobians[index].m_jacobian_J10;

		dVector linearVeloc(veloc0 * jacobian0.m_linear + veloc1 * jacobian1.m_linear + omega1 * jacobian1.m_angular);
		dFloat omegaSpeed = omega0.DotProduct3(jacobian0.m_angular);
		dFloat linearSpeed = linearVeloc.m_x + linearVeloc.m_y + linearVeloc.m_z;
		dFloat relSpeed = omegaSpeed + linearSpeed;

		omegaSpeed = dAbs(omegaSpeed);
		linearSpeed = dAbs(linearSpeed);

		m_tireModel.m_lateralSlip = linearSpeed;
		m_tireModel.m_longitudinalSlip = 0.1f;
		if ((omegaSpeed > 0.2f) || (linearSpeed > 0.2f)) {
			if (relSpeed < 0.0f) {
				dFloat speedDen = dMax(linearSpeed, dFloat(0.01f));
				m_tireModel.m_longitudinalSlip = dClamp(dAbs(relSpeed / speedDen), dFloat(0.0f), dFloat(20.0f));
			} else {
				dFloat speedDen = dMax(omegaSpeed, dFloat(0.01f));
				m_tireModel.m_longitudinalSlip = dClamp(dAbs(relSpeed / speedDen), dFloat(0.0f), dFloat(4.0f));
			}
		}
	}

	{
		// lateral force row
		int index = constraintParams->m_count;
		AddContactRowJacobian(constraintParams, m_point, m_lateralDir, 0.0f);
		const dComplementaritySolver::dJacobian &jacobian0 = constraintParams->m_jacobians[index].m_jacobian_J01;
		const dComplementaritySolver::dJacobian &jacobian1 = constraintParams->m_jacobians[index].m_jacobian_J10;

		//const dVector relVeloc(veloc0 * jacobian0.m_linear + omega0 * jacobian0.m_angular + veloc1 * jacobian1.m_linear + omega1 * jacobian1.m_angular);
		const dVector relVeloc(veloc0 * jacobian0.m_linear + veloc1 * jacobian1.m_linear + omega1 * jacobian1.m_angular);
		dFloat lateralSpeed = relVeloc.m_x + relVeloc.m_y + relVeloc.m_z;
		dAssert ((m_tireModel.m_lateralSlip + 1.0e-3f) > 0.0f);
		m_tireModel.m_lateralSlip = lateralSpeed / (m_tireModel.m_lateralSlip + 1.0e-3f);
	}


	const dTireInfo& tireInfo = tire->GetInfo();

	dFloat v = dAbs(m_tireModel.m_lateralSlip);
	dFloat u = dAbs(m_tireModel.m_longitudinalSlip);
	dFloat invden = 1.0f / (1.0f + u);

//if (tire->GetIndex() == 3) {
//dTrace(("index=%d u=%4.3f v=%4.3f\n", tire->GetIndex(), u, v));
//}

	m_tireModel.m_lateralSlip = v * invden;
	m_tireModel.m_longitudinalSlip = u * invden;
	dFloat y = tireInfo.m_corneringStiffness * m_tireModel.m_lateralSlip;
	dFloat x = tireInfo.m_longitudinalStiffness * m_tireModel.m_longitudinalSlip;
	m_tireModel.m_gammaStiffness = dSqrt(x * x + y * y);

	m_dof = constraintParams->m_count;
}

void dVehicleTireContact::SpecialSolverFrictionCallback(const dFloat* const load, dFloat* const lowFriction, dFloat* const highFriction) const
{
	dFloat f = dMax(m_staticFriction * load[0], dFloat(1.0f));
	dFloat g = m_tireModel.m_gammaStiffness;

	dVehicleTire* const tire = GetOwner0()->GetAsTire();
	const dTireInfo& tireInfo = tire->GetInfo();
	m_tireModel.m_tireLoad = load[0];
	m_tireModel.m_alingMoment = 0.0f;
	m_tireModel.m_lateralForce = load[2];
	m_tireModel.m_longitunalForce = load[1];

	if (g < (0.1f * load[0])) {
		// low speed just apply static friction 
		lowFriction[1] = -f;
		highFriction[1] = f;
		lowFriction[2] = -f;
		highFriction[2] = f;
	} else {
		// apply brush tire model

		dFloat r = g / f;
		if (g < (3.0f * f)) {
			f = g * (1.0f - (1.0f / 3.0f) * r + (1.0f / 27.0f) * r * r);
		}
		r = f / (g + 1.0e-3f);

		dFloat y = tireInfo.m_corneringStiffness * m_tireModel.m_lateralSlip;
		dFloat x = tireInfo.m_longitudinalStiffness * m_tireModel.m_longitudinalSlip;

		dAssert(x >= 0.0f);
		dAssert(y >= 0.0f);

		lowFriction[1] = -x * r;
		highFriction[1] = x * r;

		lowFriction[2] = -y * r;
		highFriction[2] = y * r;
	}
}
