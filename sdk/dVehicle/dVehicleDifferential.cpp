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
#include "dVehicleChassis.h"
#include "dVehicleDifferential.h"


dVehicleDifferential::dVehicleDifferential(dVehicleChassis* const chassis, dFloat mass, dFloat radius, dVehicleNode* const leftNode, dVehicleNode* const rightNode)
	:dVehicleNode(chassis)
	,dBilateralJoint()
	,m_localAxis(dYawMatrix (90.0f * dDegreeToRad))
//	,m_differential()
//	,m_leftAxle()
//	,m_rightAxle()
//	,m_leftTire(leftTire)
//	,m_rightTire(rightTire)
	,m_diffOmega(0.0f)
	,m_shaftOmega(0.0f)
{
	Init(&m_proxyBody, &GetParent()->GetProxyBody());

	//dFloat mass = 0.75f * 0.5f * (m_leftTire->GetInfo().m_mass + m_rightTire->GetInfo().m_mass);
	//dFloat radius = m_leftTire->GetInfo().m_radio * 0.5f;
	dFloat inertia = (2.0f / 3.0f) * mass * radius * radius;

	m_proxyBody.SetMass(mass);
	m_proxyBody.SetInertia(inertia, inertia, inertia);
	m_proxyBody.UpdateInertia();

/*
	// set the tire joint
	m_differential.Init(&m_proxyBody, m_parent->GetProxyBody());
	m_leftAxle.Init(&m_proxyBody, m_leftTire->GetProxyBody());
	m_rightAxle.Init(&m_proxyBody, m_rightTire->GetProxyBody());

	m_leftAxle.SetOwners(this, m_leftTire);
	m_rightAxle.SetOwners(this, m_rightTire);

	m_leftAxle.m_diffSign = -1.0f;
	m_rightAxle.m_diffSign = 1.0f;
*/
}

dVehicleDifferential::~dVehicleDifferential()
{
}

#if 0
dComplementaritySolver::dBilateralJoint* dVehicleDifferential::GetProxyJoint()
{
	return &m_differential;
}

void dVehicleDifferential::Debug(dCustomJoint::dDebugDisplay* const debugContext) const
{
	dAssert(0);
//	dVehicleDifferentialInterface::Debug(debugContext);
}

int dVehicleDifferential::GetKinematicLoops(dAnimIDRigKinematicLoopJoint** const jointArray)
{
	dAssert(0);
	return 0;
/*
	jointArray[0] = &m_leftAxle;
	jointArray[1] = &m_rightAxle;
	return dVehicleDifferentialInterface::GetKinematicLoops(&jointArray[2]) + 2;
*/
}
#endif

void dVehicleDifferential::CalculateFreeDof()
{
	dVehicleChassis* const chassisNode = GetParent()->GetAsVehicle();
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
	dVehicleChassis* const chassisNode = GetParent()->GetAsVehicle();
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
	//dComplementaritySolver::dBodyState* const chassis = m_state1;
	dComplementaritySolver::dBodyState* const differential = m_state0;
	//dComplementaritySolver::dBodyState* const chassisBody = m_state1;
	
	//const dVector& omega = chassis->GetOmega();
	dMatrix matrix (m_localAxis * differential->GetMatrix());
	
	/// three rigid attachment to chassis
	AddLinearRowJacobian(constraintParams, matrix.m_posit, matrix.m_front);
	AddLinearRowJacobian(constraintParams, matrix.m_posit, matrix.m_up);
	AddLinearRowJacobian(constraintParams, matrix.m_posit, matrix.m_right);

	// angular constraints	
	AddAngularRowJacobian(constraintParams, matrix.m_right, 0.0f);
	//if (m_slipeOn) {
	//	dAssert(0);
	//}
}

