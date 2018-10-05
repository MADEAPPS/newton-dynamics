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
{
	dVector tmp;
	NewtonBody* const newtonBody = m_chassis->GetBody();
	dComplementaritySolver::dBodyState* const chassisBody = GetBody();
	
	// set the inertia matrix;
	NewtonBodyGetMass(newtonBody, &tmp.m_w, &tmp.m_x, &tmp.m_y, &tmp.m_z);
	chassisBody->SetMass(tmp.m_w);
	chassisBody->SetInertia(tmp.m_x, tmp.m_y, tmp.m_z);

	dMatrix matrix (dGetIdentityMatrix());
	NewtonBodyGetCentreOfMass(newtonBody, &matrix.m_posit[0]);
	matrix.m_posit.m_w = 1.0f;
	chassisBody->SetLocalMatrix(matrix);
}

dVehicleSingleBody::~dVehicleSingleBody()
{
}

dVehicleTireInterface* dVehicleSingleBody::AddTire (const dMatrix& locationInGlobalSpace, const dVehicleTireInterface::dTireInfo& tireInfo)
{
	return new dVehicleVirtualTire(this, locationInGlobalSpace, tireInfo);
}

dMatrix dVehicleSingleBody::GetMatrix () const
{
	dMatrix matrix;
	NewtonBody* const chassisBody = m_chassis->GetBody();
	NewtonBodyGetMatrix(chassisBody, &matrix[0][0]);
	return matrix;
}

void dVehicleSingleBody::InitRigiBody(dFloat timestep)
{
	dVector vector;
	dMatrix matrix;

	NewtonBody* const newtonBody = m_chassis->GetBody();
	dComplementaritySolver::dBodyState* const chassisBody = GetBody();

	// get data from engine rigid body and copied to the vehicle chassis body
	NewtonBodyGetMatrix(newtonBody, &matrix[0][0]);
	chassisBody->SetMatrix(matrix);

	NewtonBodyGetVelocity(newtonBody, &vector[0]);
	chassisBody->SetVeloc(vector);

	NewtonBodyGetOmega(newtonBody, &vector[0]);
	chassisBody->SetOmega(vector);

	NewtonBodyGetForce(newtonBody, &vector[0]);
	chassisBody->SetVeloc(vector);

	NewtonBodyGetForce(newtonBody, &vector[0]);
	chassisBody->SetForce(vector);

	NewtonBodyGetTorque(newtonBody, &vector[0]);
	chassisBody->SetTorque(vector);

	chassisBody->UpdateInertia();

	dVehicleInterface::InitRigiBody(timestep);
}
