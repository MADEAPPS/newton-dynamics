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
#include "dVehicleMultiBody.h"
#include "dVehicleDifferential.h"


dVehicleDifferential::dVehicleDifferential(dVehicleMultiBody* const chassis, dFloat mass, dFloat radius, dVehicleNode* const leftNode, dVehicleNode* const rightNode)
	:dVehicleNode(chassis)
	,dBilateralJoint()
	,m_localAxis(dYawMatrix (90.0f * dDegreeToRad))
	,m_leftAxle()
	,m_rightAxle()
	,m_leftNode(leftNode)
	,m_rightNode(rightNode)
	,m_diffOmega(0.0f)
	,m_shaftOmega(0.0f)
{
	Init(&m_proxyBody, &GetParent()->GetProxyBody());

	dFloat inertia = (2.0f / 5.0f) * mass * radius * radius;

	m_proxyBody.SetMass(mass);
	m_proxyBody.SetInertia(inertia, inertia, inertia);
	m_proxyBody.UpdateInertia();

	// set the tire joint
	m_leftAxle.SetOwners(this, m_leftNode);
	m_rightAxle.SetOwners(this, m_rightNode);

	m_leftAxle.Init(&m_proxyBody, &m_leftNode->GetProxyBody());
	m_rightAxle.Init(&m_proxyBody, &m_rightNode->GetProxyBody());

	m_leftAxle.m_diffSign = -1.0f;
	m_rightAxle.m_diffSign = 1.0f;
}

dVehicleDifferential::~dVehicleDifferential()
{
}

const void dVehicleDifferential::Debug(dCustomJoint::dDebugDisplay* const debugContext) const
{
//	dAssert(0);
//	dVehicleDifferentialInterface::Debug(debugContext);
}

int dVehicleDifferential::GetKinematicLoops(dVehicleLoopJoint** const jointArray)
{
	jointArray[0] = &m_leftAxle;
	jointArray[1] = &m_rightAxle;
	return 2;
}

void dVehicleDifferential::CalculateFreeDof()
{
	dVehicleMultiBody* const chassisNode = GetParent()->GetAsVehicleMultiBody();
	dComplementaritySolver::dBodyState* const chassisBody = &chassisNode->GetProxyBody();
	const dMatrix chassisMatrix(m_localAxis * chassisBody->GetMatrix());

	dVector omega(m_proxyBody.GetOmega());
	dVector chassisOmega(chassisBody->GetOmega());
	dVector relativeOmega(omega - chassisOmega);
	m_diffOmega = chassisMatrix.m_up.DotProduct3(relativeOmega);
	m_shaftOmega = chassisMatrix.m_front.DotProduct3(relativeOmega);
}

void dVehicleDifferential::Integrate(dFloat timestep)
{
	m_proxyBody.IntegrateForce(timestep, m_proxyBody.GetForce(), m_proxyBody.GetTorque());
}

void dVehicleDifferential::ApplyExternalForce()
{
	dVehicleMultiBody* const chassisNode = GetParent()->GetAsVehicleMultiBody();
	dComplementaritySolver::dBodyState* const chassisBody = &chassisNode->GetProxyBody();
	
	dMatrix matrix (chassisBody->GetMatrix());
	matrix.m_posit = matrix.TransformVector(chassisBody->GetCOM());
	m_proxyBody.SetMatrix(matrix);
	
	matrix = m_localAxis * matrix;
	m_proxyBody.SetVeloc(chassisBody->GetVelocity());
	m_proxyBody.SetOmega(chassisBody->GetOmega() + matrix.m_front.Scale (m_shaftOmega) + matrix.m_up.Scale (m_diffOmega));
	m_proxyBody.SetTorque(dVector(0.0f));

	m_proxyBody.SetForce(chassisNode->GetGravity().Scale(m_proxyBody.GetMass()));
}

void dVehicleDifferential::UpdateSolverForces(const dComplementaritySolver::dJacobianPair* const jacobians) const
{
	dAssert (0);
}

void dVehicleDifferential::JacobianDerivative(dComplementaritySolver::dParamInfo* const constraintParams)
{
	dComplementaritySolver::dBodyState* const diffBody = m_state0;
	dMatrix matrix (m_localAxis * diffBody->GetMatrix());
	
	/// three rigid attachment to chassis
	AddLinearRowJacobian(constraintParams, matrix.m_posit, matrix.m_front);
	AddLinearRowJacobian(constraintParams, matrix.m_posit, matrix.m_up);
	AddLinearRowJacobian(constraintParams, matrix.m_posit, matrix.m_right);

	// angular constraints	
	AddAngularRowJacobian(constraintParams, matrix.m_right, 0.0f);

	const dVector& omega = m_state0->GetOmega();
	dFloat omegax = matrix.m_front.DotProduct3(omega);
	dFloat omegay = matrix.m_up.DotProduct3(omega);

	dFloat slipDiffOmega = dMax(dFloat (8.0f), 0.5f * dAbs(omegax));
	if (omegay > slipDiffOmega) {
		int index = constraintParams->m_count;
		AddAngularRowJacobian(constraintParams, matrix.m_up, 0.0f);
		constraintParams->m_jointAccel[index] -= slipDiffOmega * constraintParams->m_timestepInv;
		constraintParams->m_jointHighFrictionCoef[index] = 0.0f;

	} else if (omegay < -slipDiffOmega) {
		int index = constraintParams->m_count;
		AddAngularRowJacobian(constraintParams, matrix.m_up, 0.0f);
		constraintParams->m_jointAccel[index] -= -slipDiffOmega * constraintParams->m_timestepInv;
		constraintParams->m_jointLowFrictionCoef[index] = 0.0f;
	}
}

void dVehicleDifferential::dTireAxleJoint::JacobianDerivative(dComplementaritySolver::dParamInfo* const constraintParams)
{
	dVehicleDifferential* const differential = GetOwner0()->GetAsDifferential();
	dAssert (differential);

	dMatrix diffMatrix (differential->m_localAxis * m_state0->GetMatrix());
	const dMatrix& tireMatrix = m_state1->GetMatrix();

	AddAngularRowJacobian(constraintParams, tireMatrix.m_front, 0.0f);

	dComplementaritySolver::dJacobian &jacobian0 = constraintParams->m_jacobians[0].m_jacobian_J01;
	dComplementaritySolver::dJacobian &jacobian1 = constraintParams->m_jacobians[0].m_jacobian_J10;

	jacobian0.m_angular = diffMatrix.m_front + diffMatrix.m_up.Scale(m_diffSign);

	const dVector& omega0 = m_state0->GetOmega();
	const dVector& omega1 = m_state1->GetOmega();

	const dVector relOmega(omega0 * jacobian0.m_angular + omega1 * jacobian1.m_angular);
	dFloat w = relOmega.m_x + relOmega.m_y + relOmega.m_z;

	constraintParams->m_jointAccel[0] = -w * constraintParams->m_timestepInv;

	m_dof = 1;
	m_count = 1;
	constraintParams->m_count = 1;
}

