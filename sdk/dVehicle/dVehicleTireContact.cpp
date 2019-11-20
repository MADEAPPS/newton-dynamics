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

#define D_TIRE_CONTACT_PATCH_CONE		dFloat (0.8f) 
#define D_TIRE_CONTACT_MAX_SIDE_SLIP	dFloat (0.15f) 
#define D_TIRE_CONTACT_MAX_LONG_SLIP	dFloat (10.0f) 

dVehicleTireContact::dVehicleTireContact()
	:dVehicleLoopJoint()
	,m_point(0.0f)
	,m_normal(0.0f)
	,m_lateralDir(0.0f)
	,m_longitudinalDir(0.0f)
	,m_collidingNode(NULL)
	,m_penetration(0.0f)
	,m_staticFriction(1.0f)
	,m_tireModel()
	,m_isContactPatch(false)
	,m_isLowSpeed(false)
{
}

void dVehicleTireContact::ResetContact()
{
	m_isActive = false;
}

void dVehicleTireContact::SetContact(dVehicleCollidingNode* const node, const dVector& posit, const dVector& normal, const dVector& longitudinalDir, dFloat penetration, dFloat staticFriction, bool isPatch)
{
	m_point = posit;
	m_normal = normal;
	m_collidingNode = node;
	m_longitudinalDir = longitudinalDir;
	m_lateralDir = m_longitudinalDir.CrossProduct(m_normal);

	m_isActive = true;
	m_isContactPatch = false;
	m_staticFriction = staticFriction;
	dAssert (penetration >= -1.0e-3f);
	m_penetration = dMax (penetration, dFloat (0.0f));
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
	//dVector origin(m_point + m_normal.Scale(1.0f / 32.0f));
	dVector origin(tireMatrix.m_posit);
	if (localPosit.m_z > 0.0f) {
		origin += m_lateralDir.Scale(tire->GetInfo().m_width * 0.5f * 1.25f);
	} else {
		origin -= m_lateralDir.Scale(tire->GetInfo().m_width * 0.5f * 1.25f);
	}

	scale *= 4.0f;

	// show tire load
	debugContext->SetColor(dVector(0.0f, 0.0f, 1.0f, 1.0f));
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
	dVector zero (0.0f);

	m_isLowSpeed = false;
	m_isContactPatch = false;
	{
		// normal constraint
		int index = constraintParams->m_count;
		AddContactRowJacobian(constraintParams, m_point, m_normal, 0.0f);
		constraintParams->m_jointLowFrictionCoef[index] = 0.0f;
		constraintParams->m_frictionCallback[index] = this;

		dFloat penetration = dMin (m_penetration, D_TIRE_MAX_ELASTIC_DEFORMATION);
		dFloat recoverAccel = D_TIRE_PENETRATION_RECOVERING_SPEED * penetration * constraintParams->m_timestepInv;
		constraintParams->m_jointAccel[index] += recoverAccel;

		const dMatrix& tireMatrix = m_state0->GetMatrix();
		if (m_collidingNode->GetNewtonBody() && (tireMatrix.m_right.DotProduct3(m_normal) > D_TIRE_CONTACT_PATCH_CONE)) {
			dComplementaritySolver::dJacobian &jacobian1 = constraintParams->m_jacobians[index].m_jacobian_J10;
			jacobian1.m_linear = zero;
			jacobian1.m_angular = zero;
			m_isContactPatch = true;
		}
	}

float xxx0 = 0;
float xxx1 = 0;

	{
		// longitudinal force row
		int index = constraintParams->m_count;
		AddContactRowJacobian(constraintParams, m_point, m_longitudinalDir, 0.0f);
		constraintParams->m_jointLowFrictionCoef[index] = -1.0e5f;
		constraintParams->m_jointHighFrictionCoef[index] = 1.0e5f;

		const dComplementaritySolver::dJacobian &jacobian0 = constraintParams->m_jacobians[index].m_jacobian_J01;
		const dComplementaritySolver::dJacobian &jacobian1 = constraintParams->m_jacobians[index].m_jacobian_J10;

		dVector linearVeloc(veloc0 * jacobian0.m_linear + veloc1 * jacobian1.m_linear + omega1 * jacobian1.m_angular);
		dFloat omegaSpeed = omega0.DotProduct3(jacobian0.m_angular);
		dFloat linearSpeed = linearVeloc.m_x + linearVeloc.m_y + linearVeloc.m_z;
		dFloat relSpeed = omegaSpeed + linearSpeed;

		omegaSpeed = dAbs(omegaSpeed);
		linearSpeed = dAbs(linearSpeed);

xxx0 = omegaSpeed;
xxx1 = linearSpeed;

		m_isLowSpeed = (omegaSpeed < 1.0f);

		m_tireModel.m_lateralSlip = linearSpeed;
		m_tireModel.m_longitudinalSlip = 0.1f;
		if ((omegaSpeed > 0.2f) || (linearSpeed > 0.2f)) {
			if (relSpeed < 0.0f) {
				dFloat speedDen = dMax(linearSpeed, dFloat(0.01f));
				m_tireModel.m_longitudinalSlip = dClamp(dAbs(relSpeed / speedDen), dFloat(0.0f), D_TIRE_CONTACT_MAX_LONG_SLIP);
			} else {
				dFloat speedDen = dMax(omegaSpeed, dFloat(0.01f));
				m_tireModel.m_longitudinalSlip = dClamp(dAbs(relSpeed / speedDen), dFloat(0.0f), D_TIRE_CONTACT_MAX_LONG_SLIP);
			}
		}

		if (m_isContactPatch) {
			dComplementaritySolver::dJacobian &jacobian2 = constraintParams->m_jacobians[index].m_jacobian_J10;
			jacobian2.m_linear = zero;
			jacobian2.m_angular = zero;
		}
	}

	{
		// lateral force row
		int index = constraintParams->m_count;
		AddContactRowJacobian(constraintParams, m_point, m_lateralDir, 0.0f);
		constraintParams->m_jointLowFrictionCoef[index] = -1.0e5f;
		constraintParams->m_jointHighFrictionCoef[index] = 1.0e5f;

		const dComplementaritySolver::dJacobian &jacobian0 = constraintParams->m_jacobians[index].m_jacobian_J01;
		const dComplementaritySolver::dJacobian &jacobian1 = constraintParams->m_jacobians[index].m_jacobian_J10;

		const dVector relVeloc(veloc0 * jacobian0.m_linear + veloc1 * jacobian1.m_linear + omega1 * jacobian1.m_angular);
		dFloat lateralSpeed = relVeloc.m_x + relVeloc.m_y + relVeloc.m_z;
		dAssert ((m_tireModel.m_lateralSlip + 1.0e-3f) > 0.0f);
		m_tireModel.m_lateralSlip = lateralSpeed / (m_tireModel.m_lateralSlip + 1.0e-3f);
		// clamp lateral slip to a max of +- 10 degree
		m_tireModel.m_lateralSlip = dClamp (m_tireModel.m_lateralSlip, -D_TIRE_CONTACT_MAX_SIDE_SLIP, D_TIRE_CONTACT_MAX_SIDE_SLIP);

		if (m_isContactPatch) {
			dComplementaritySolver::dJacobian &jacobian2 = constraintParams->m_jacobians[index].m_jacobian_J10;
			jacobian2.m_linear = zero;
			jacobian2.m_angular = zero;
		}
	}

	if (m_isLowSpeed) {
		m_tireModel.m_lateralSlip = 0.0f;
		m_tireModel.m_longitudinalSlip = 0.0f;
	}

	const dTireInfo& tireInfo = tire->GetInfo();
	dFloat v = dAbs(m_tireModel.m_lateralSlip);
	dFloat u = dAbs(m_tireModel.m_longitudinalSlip);
	dFloat invden = 1.0f / (1.0f + u);

if (tire->GetIndex() == 2) {
//dTrace(("%d w=%f v=%f s=%f %f) ", tire->Get Index(), u, v));
dTrace(("%d w=%f v=%f s=%f ", tire->GetIndex(), xxx0, xxx1, u, v));
}

	m_tireModel.m_lateralSlip = v * invden;
	m_tireModel.m_longitudinalSlip = u * invden;
	dFloat y = tireInfo.m_corneringStiffness * m_tireModel.m_lateralSlip;
	dFloat x = tireInfo.m_longitudinalStiffness * m_tireModel.m_longitudinalSlip;
	m_tireModel.m_gammaStiffness = dSqrt(x * x + y * y);

	m_dof = constraintParams->m_count;
}

void dVehicleTireContact::SpecialSolverFrictionCallback(const dFloat* const load, dFloat* const lowFriction, dFloat* const highFriction) const
{
	dAssert (load[0] >= 0.0f);
	dFloat f = m_staticFriction * load[0];

	m_tireModel.m_tireLoad = load[0];
	m_tireModel.m_alingMoment = 0.0f;
	m_tireModel.m_longitunalForce = load[1];
	m_tireModel.m_lateralForce = load[2];

	lowFriction[1] = -f;
	highFriction[1] = f;
	lowFriction[2] = -f;
	highFriction[2] = f;
	
	if (!m_isLowSpeed &&  f > 10.0f) {
		dVehicleTire* const tire = GetOwner0()->GetAsTire();
		const dTireInfo& tireInfo = tire->GetInfo();

		dFloat g = m_tireModel.m_gammaStiffness;

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
