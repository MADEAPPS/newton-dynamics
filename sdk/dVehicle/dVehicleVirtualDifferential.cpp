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
#include "dVehicleChassis.h"
#include "dVehicleSingleBody.h"
#include "dVehicleVirtualDifferential.h"


dVehicleVirtualDifferential::dVehicleVirtualDifferential(dVehicleNode* const parent, dVehicleTireInterface* const leftTire, dVehicleTireInterface* const rightTire)
	:dVehicleDifferentialInterface(parent)
	,m_differential()
	,m_leftAxle()
	,m_rightAxle()
	,m_leftTire(leftTire)
	,m_rightTire(rightTire)
	,m_diffOmega(0.0f)
	,m_shaftOmega(0.0f)
{
	SetWorld(parent->GetWorld());

	dFloat mass = 0.75f * 0.5f * (m_leftTire->GetInfo().m_mass + m_rightTire->GetInfo().m_mass);
	dFloat radius = m_leftTire->GetInfo().m_radio * 0.5f;
	dFloat inertia = 0.7f * mass * radius * radius;

	m_body.SetMass(mass);
	m_body.SetInertia(inertia, inertia, inertia);
	m_body.UpdateInertia();

	// set the tire joint
	m_differential.Init(&m_body, m_parent->GetBody());
	m_leftAxle.Init(&m_body, m_leftTire->GetBody());
	m_rightAxle.Init(&m_body, m_rightTire->GetBody());

	m_leftAxle.SetOwners(this, m_leftTire);
	m_rightAxle.SetOwners(this, m_rightTire);

	m_leftAxle.m_diffSign = -1.0f;
	m_rightAxle.m_diffSign = 1.0f;
}

dVehicleVirtualDifferential::~dVehicleVirtualDifferential()
{
}

dComplementaritySolver::dBilateralJoint* dVehicleVirtualDifferential::GetJoint()
{
	return &m_differential;
}

void dVehicleVirtualDifferential::Debug(dCustomJoint::dDebugDisplay* const debugContext) const
{
	dVehicleDifferentialInterface::Debug(debugContext);
}

int dVehicleVirtualDifferential::GetKinematicLoops(dAnimationKinematicLoopJoint** const jointArray)
{
	jointArray[0] = &m_leftAxle;
	jointArray[1] = &m_rightAxle;
	return dVehicleDifferentialInterface::GetKinematicLoops(&jointArray[2]) + 2;
}

void dVehicleVirtualDifferential::ApplyExternalForce(dFloat timestep)
{
	dVehicleSingleBody* const chassisNode = (dVehicleSingleBody*)m_parent;
	dComplementaritySolver::dBodyState* const chassisBody = chassisNode->GetBody();

	dMatrix matrix(chassisBody->GetMatrix());
	matrix.m_posit = matrix.TransformVector(chassisBody->GetCOM());
	m_body.SetMatrix(matrix);

	m_body.SetVeloc(chassisBody->GetVelocity());
	m_body.SetOmega(chassisBody->GetOmega() + matrix.m_right.Scale (m_shaftOmega) + matrix.m_up.Scale (m_diffOmega));
	m_body.SetTorque(dVector(0.0f));
	m_body.SetForce(chassisNode->m_gravity.Scale(m_body.GetMass()));

	dVehicleDifferentialInterface::ApplyExternalForce(timestep);
}

void dVehicleVirtualDifferential::Integrate(dFloat timestep)
{
	dVehicleDifferentialInterface::Integrate(timestep);

	dVehicleSingleBody* const chassis = (dVehicleSingleBody*)m_parent;
	dComplementaritySolver::dBodyState* const chassisBody = chassis->GetBody();

	const dMatrix chassisMatrix(chassisBody->GetMatrix());

	dVector omega(m_body.GetOmega());
	dVector chassisOmega(chassisBody->GetOmega());
	dVector localOmega(omega - chassisOmega);
	m_diffOmega = chassisMatrix.m_up.DotProduct3(localOmega);
	m_shaftOmega = chassisMatrix.m_right.DotProduct3(localOmega);
}
