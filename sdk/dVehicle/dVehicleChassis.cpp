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
#include "dVehicleManager.h"
#include "dVehicleChassis.h"
#include "dVehicleSingleBody.h"

dVehicleChassis::dVehicleChassis ()
	:m_localFrame(dGetIdentityMatrix())
	,m_solver()
	,m_vehicle(NULL)
{
}

void dVehicleChassis::Init(NewtonCollision* const chassisShape, dFloat mass, const dMatrix& localFrame, NewtonApplyForceAndTorque forceAndTorque, dFloat gravityMag)
{
	dVehicleManager* const manager = (dVehicleManager*)GetManager();
	NewtonWorld* const world = manager->GetWorld();

	// create a body and call the low level init function
	dMatrix locationMatrix(dGetIdentityMatrix());
	NewtonBody* const body = NewtonCreateDynamicBody(world, chassisShape, &locationMatrix[0][0]);

	// set vehicle mass, inertia and center of mass
	NewtonBodySetMassProperties(body, mass, chassisShape);

	// initialize 
	Init(body, localFrame, forceAndTorque, gravityMag);
}

void dVehicleChassis::Init(NewtonBody* const body, const dMatrix& localFrame, NewtonApplyForceAndTorque forceAndTorque, dFloat gravityMag)
{
	m_body = body;
	NewtonBodySetForceAndTorqueCallback(m_body, forceAndTorque);
	m_vehicle = new dVehicleSingleBody(this);

	m_localFrame = localFrame;
	m_localFrame.m_posit = dVector(0.0f, 0.0f, 0.0f, 1.0f);
	dAssert(m_localFrame.TestOrthogonal());

/*
	m_speed = 0.0f;
	m_sideSlip = 0.0f;
	m_prevSideSlip = 0.0f;
	m_finalized = false;
	m_gravityMag = dAbs(gravityMag);
	m_weightDistribution = 0.5f;
	m_aerodynamicsDownForce0 = 0.0f;
	m_aerodynamicsDownForce1 = 0.0f;
	m_aerodynamicsDownSpeedCutOff = 0.0f;
	m_aerodynamicsDownForceCoefficient = 0.0f;


	m_forceAndTorqueCallback = forceAndTorque;

	dCustomVehicleControllerManager* const manager = (dCustomVehicleControllerManager*)GetManager();
	NewtonWorld* const world = manager->GetWorld();

	// set linear and angular drag to zero
	dVector drag(0.0f);
	NewtonBodySetLinearDamping(m_body, 0.0f);
	NewtonBodySetAngularDamping(m_body, &drag[0]);

	// set the standard force and torque call back
	NewtonBodySetForceAndTorqueCallback(body, m_forceAndTorqueCallback);

	m_contactFilter = new dTireFrictionModel(this);

	m_engine = NULL;
	m_brakesControl = NULL;
	m_engineControl = NULL;
	m_steeringControl = NULL;
	m_handBrakesControl = NULL;

	m_collisionAggregate = NewtonCollisionAggregateCreate(world);
	NewtonCollisionAggregateSetSelfCollision(m_collisionAggregate, 0);
	NewtonCollisionAggregateAddBody(m_collisionAggregate, m_body);

	m_bodyList.Append(m_body);

	SetAerodynamicsDownforceCoefficient(0.5f, 0.4f, 1.0f);

#ifdef D_PLOT_ENGINE_CURVE 
	file_xxx = fopen("vehiceLog.csv", "wb");
	fprintf(file_xxx, "eng_rpm, eng_torque, eng_nominalTorque,\n");
#endif
*/
}

void dVehicleChassis::Cleanup()
{
	if (m_vehicle) {
		delete m_vehicle;
	}
}



dVehicleTireInterface* dVehicleChassis::AddTire (const dMatrix& locationInGlobalSpace, const dVehicleTireInterface::dTireInfo& tireInfo)
{
	return m_vehicle->AddTire(locationInGlobalSpace, tireInfo);
}

void dVehicleChassis::Debug(dCustomJoint::dDebugDisplay* const debugContext) const
{
	m_vehicle->Debug(debugContext);
}

void dVehicleChassis::Finalize()
{
	m_solver.Finalize(this);
}

void dVehicleChassis::InitRigiBody(dFloat timestep)
{
	m_vehicle->InitRigiBody(timestep);
}


void dVehicleChassis::PostUpdate(dFloat timestep, int threadIndex)
{
	//	dAssert (0);
}

void dVehicleChassis::PreUpdate(dFloat timestep, int threadIndex)
{
	//	dAssert (0);
	m_solver.Update(timestep);
}
