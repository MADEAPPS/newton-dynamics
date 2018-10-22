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
#include "dVehicleVirtualTire.h"

dVehicleSingleBody::dVehicleSingleBody(dVehicleChassis* const chassis)
	:dVehicleInterface(chassis)
	,m_gravity(0.0f)
	,m_groundNode(NULL)
	,m_newtonBody(chassis->GetBody())
{
	dVector tmp;
	dComplementaritySolver::dBodyState* const chassisBody = GetBody();
	m_groundNode.SetWorld(m_world);
	m_groundNode.SetLoopNode(true);
	
	// set the inertia matrix;
	NewtonBodyGetMass(m_newtonBody, &tmp.m_w, &tmp.m_x, &tmp.m_y, &tmp.m_z);
	chassisBody->SetMass(tmp.m_w);
	chassisBody->SetInertia(tmp.m_x, tmp.m_y, tmp.m_z);

	dMatrix matrix (dGetIdentityMatrix());
	NewtonBodyGetCentreOfMass(m_newtonBody, &matrix.m_posit[0]);
	matrix.m_posit.m_w = 1.0f;
	chassisBody->SetLocalMatrix(matrix);
}

dVehicleSingleBody::~dVehicleSingleBody()
{
}

dVehicleTireInterface* dVehicleSingleBody::AddTire (const dMatrix& locationInGlobalSpace, const dVehicleTireInterface::dTireInfo& tireInfo, const dMatrix& localFrame)
{
	return new dVehicleVirtualTire(this, locationInGlobalSpace, tireInfo, localFrame);
}

dMatrix dVehicleSingleBody::GetMatrix () const
{
	return m_body.GetMatrix();
}

void dVehicleSingleBody::CalculateNodeAABB(const dMatrix& matrix, dVector& minP, dVector& maxP) const
{
	NewtonCollision* const collision = NewtonBodyGetCollision(m_newtonBody);
	CalculateAABB(collision, matrix, minP, maxP);
}

void dVehicleSingleBody::RigidBodyToStates()
{
	dVector vector;
	dMatrix matrix;
	dComplementaritySolver::dBodyState* const chassisBody = GetBody();

	// get data from engine rigid body and copied to the vehicle chassis body
	NewtonBodyGetMatrix(m_newtonBody, &matrix[0][0]);
	chassisBody->SetMatrix(matrix);

static int xxx;
xxx++;
if (xxx == 1500)
{
//	NewtonBodyGetVelocity(m_newtonBody, &vector[0]);
//	vector.m_x += 2.0f;
//	NewtonBodySetVelocity(m_newtonBody, &vector[0]);
}


	NewtonBodyGetVelocity(m_newtonBody, &vector[0]);
	chassisBody->SetVeloc(vector);

	NewtonBodyGetOmega(m_newtonBody, &vector[0]);
	chassisBody->SetOmega(vector);

	NewtonBodyGetForce(m_newtonBody, &vector[0]);
	chassisBody->SetForce(vector);

	NewtonBodyGetTorque(m_newtonBody, &vector[0]);
	chassisBody->SetTorque(vector);

	chassisBody->UpdateInertia();
	
	dVehicleInterface::RigidBodyToStates();
}

void dVehicleSingleBody::StatesToRigidBody(dFloat timestep)
{
	dComplementaritySolver::dBodyState* const chassisBody = GetBody();

	dVector force(chassisBody->GetForce());
	dVector torque(chassisBody->GetTorque());
	NewtonBodySetForce(m_newtonBody, &force[0]);
	NewtonBodySetTorque(m_newtonBody, &torque[0]);

	dVehicleInterface::StatesToRigidBody(timestep);
}

int dVehicleSingleBody::GetKinematicLoops(dKinematicLoopJoint** const jointArray)
{
	m_groundNode.SetIndex(-1);
	return dVehicleInterface::GetKinematicLoops(jointArray);
}

void dVehicleSingleBody::ApplyExternalForce()
{
	dVector force(0.0f);
	dVector torque;
	
	dComplementaritySolver::dBodyState* const chassisBody = GetBody();
	dComplementaritySolver::dBodyState* const groundBody = m_groundNode.GetBody();

	groundBody->SetForce(force);
	groundBody->SetTorque(force);

	NewtonBodyGetForce(m_newtonBody, &force[0]);
	chassisBody->SetForce(force);

	NewtonBodyGetTorque(m_newtonBody, &torque[0]);
	chassisBody->SetTorque(torque);
	
	m_gravity = force.Scale (chassisBody->GetInvMass());
	dVehicleInterface::ApplyExternalForce();
}

