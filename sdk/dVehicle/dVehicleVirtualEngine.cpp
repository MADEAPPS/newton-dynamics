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
#include "dVehicleVirtualEngine.h"


dVehicleVirtualEngine::dVehicleVirtualEngine(dVehicleNode* const parent, const dEngineInfo& info, dVehicleDifferentialInterface* const differential)
	:dVehicleEngineInterface(parent, info, differential)
	,m_joint()
	,m_omega(0.0f)
{
	SetWorld(parent->GetWorld());

	dVector inertia(0.0f);
	inertia = dVector(0.7f * m_info.m_mass * m_info.m_armatureRadius * m_info.m_armatureRadius);

	m_body.SetMass(m_info.m_mass);
	m_body.SetInertia(inertia.m_x, inertia.m_y, inertia.m_z);
	m_body.UpdateInertia();

	// set the tire joint
	m_joint.Init(&m_body, m_parent->GetBody());
/*
	m_leftDifferential.Init(&m_body, m_leftTire->GetBody());
	m_rightDifferential.Init(&m_body, m_rightTire->GetBody());

	m_leftDifferential.SetOwners(this, m_leftTire);
	m_rightDifferential.SetOwners(this, m_rightTire);
*/
}

dVehicleVirtualEngine::~dVehicleVirtualEngine()
{
}

dComplementaritySolver::dBilateralJoint* dVehicleVirtualEngine::GetJoint()
{
	return &m_joint;
}

void dVehicleVirtualEngine::Debug(dCustomJoint::dDebugDisplay* const debugContext) const
{
	dVehicleEngineInterface::Debug(debugContext);
}

void dVehicleVirtualEngine::ApplyExternalForce()
{
	dVehicleSingleBody* const chassisNode = (dVehicleSingleBody*)m_parent;
	dComplementaritySolver::dBodyState* const chassisBody = chassisNode->GetBody();

	const dMatrix& matrix(chassisBody->GetMatrix());
	m_body.SetMatrix(matrix);

	m_body.SetVeloc(chassisBody->GetVelocity());
	m_body.SetOmega(chassisBody->GetOmega() + matrix.m_right.Scale (m_omega));
	m_body.SetTorque(dVector(0.0f));
	m_body.SetForce(chassisNode->m_gravity.Scale(m_body.GetMass()));

	dVehicleEngineInterface::ApplyExternalForce();
}

void dVehicleVirtualEngine::Integrate(dFloat timestep)
{
	dVehicleEngineInterface::Integrate(timestep);

	dVehicleSingleBody* const chassis = (dVehicleSingleBody*)m_parent;
	dComplementaritySolver::dBodyState* const chassisBody = chassis->GetBody();

	const dMatrix chassisMatrix(chassisBody->GetMatrix());

	dVector omega(m_body.GetOmega());
	dVector chassisOmega(chassisBody->GetOmega());
	dVector localOmega(omega - chassisOmega);
	m_omega = chassisMatrix.m_right.DotProduct3(localOmega);
}
