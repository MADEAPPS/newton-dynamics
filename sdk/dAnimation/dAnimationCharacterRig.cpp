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

#include "dAnimationStdAfx.h"
#include "dAnimationCharacterRig.h"

dAnimationCharacterRig::dAnimationCharacterRig ()
{
}

/*
void dAnimationCharacterRig::Init(NewtonCollision* const chassisShape, dFloat mass, const dMatrix& localFrame, NewtonApplyForceAndTorque forceAndTorque, dFloat gravityMag)
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

void dAnimationCharacterRig::Init(NewtonBody* const body, const dMatrix& localFrame, NewtonApplyForceAndTorque forceAndTorque, dFloat gravityMag)
{
	m_body = body;
	NewtonBodySetForceAndTorqueCallback(m_body, forceAndTorque);
	m_vehicle = new dVehicleSingleBody(this);

	m_localFrame = localFrame;
	m_localFrame.m_posit = dVector(0.0f, 0.0f, 0.0f, 1.0f);
	dAssert(m_localFrame.TestOrthogonal());

	m_gravity = dVector (0.0f, -dAbs(gravityMag), 0.0f, 0.0f);

	// set linear and angular drag to zero
	dVector drag(0.0f);
	NewtonBodySetLinearDamping(m_body, 0.0f);
	NewtonBodySetAngularDamping(m_body, &drag[0]);
}

void dAnimationCharacterRig::Cleanup()
{
	if (m_brakeControl) {
		delete m_brakeControl;
	}

	if (m_handBrakeControl) {
		delete m_handBrakeControl;
	}

	if (m_engineControl) {
		delete m_engineControl;
	}

	if (m_steeringControl) {
		delete m_steeringControl;
	}

	if (m_vehicle) {
		delete m_vehicle;
	}
}

dVehicleBrakeControl* dAnimationCharacterRig::GetBrakeControl()
{
	if (!m_brakeControl) {
		m_brakeControl = new dVehicleBrakeControl(this);
	}
	return m_brakeControl;
}

dVehicleBrakeControl* dAnimationCharacterRig::GetHandBrakeControl()
{
	if (!m_handBrakeControl) {
		m_handBrakeControl = new dVehicleBrakeControl(this);
	}
	return m_handBrakeControl;
}

dVehicleSteeringControl* dAnimationCharacterRig::GetSteeringControl ()
{
	if (!m_steeringControl) {
		m_steeringControl = new dVehicleSteeringControl(this);
	}
	return m_steeringControl;
}

dVehicleEngineControl* dAnimationCharacterRig::GetEngineControl()
{
	if (!m_engineControl) {
		m_engineControl = new dVehicleEngineControl(this);
	}
	return m_engineControl;
}

dVehicleTireInterface* dAnimationCharacterRig::AddTire (const dMatrix& locationInGlobalSpace, const dVehicleTireInterface::dTireInfo& tireInfo)
{
	return m_vehicle->AddTire(locationInGlobalSpace, tireInfo, m_localFrame);
}

dVehicleDifferentialInterface* dAnimationCharacterRig::AddDifferential(dVehicleTireInterface* const leftTire, dVehicleTireInterface* const rightTire)
{
	return m_vehicle->AddDifferential(leftTire, rightTire);
}

dVehicleEngineInterface* dAnimationCharacterRig::AddEngine(const dVehicleEngineInterface::dEngineInfo& engineInfo, dVehicleDifferentialInterface* const differential)
{
	return m_vehicle->AddEngine(engineInfo, differential);
}

void dAnimationCharacterRig::Debug(dCustomJoint::dDebugDisplay* const debugContext) const
{
	m_vehicle->Debug(debugContext);
}

void dAnimationCharacterRig::Finalize()
{
	dVector minP;
	dVector maxP;
	m_vehicle->CalculateNodeAABB(dGetIdentityMatrix(), minP, maxP);

	const dList<dAnimationAcyclicJoint*>& children = m_vehicle->GetChildren();
	for (dList<dAnimationAcyclicJoint*>::dListNode* tireNode = children.GetFirst(); tireNode; tireNode = tireNode->GetNext()) {
		dVehicleNode* const node = (dVehicleNode*)tireNode->GetInfo();
		dVehicleVirtualTire* const tire = (dVehicleVirtualTire*)node->GetAsTire();
		if (tire) {
			dVector tireMinP;
			dVector tireMaxP;

			dMatrix tireMatrix(tire->GetHardpointMatrix(0.0f));
			tire->CalculateNodeAABB(tireMatrix, tireMinP, tireMaxP);

			minP = minP.Min (tireMinP);
			maxP = maxP.Max (tireMaxP);
		}
	}

	m_obbOrigin = (maxP + minP).Scale (0.5f);
	m_obbSize = (maxP - minP).Scale (0.5f) + dVector (0.1f, 0.1f, 0.1f, 0.0f);

	m_vehicle->RigidBodyToStates();
	m_solver.Finalize(this);
}

void dAnimationCharacterRig::ApplyExternalForces(dFloat timestep)
{
	CalculateSuspensionForces(timestep);
	CalculateTireContacts(timestep);
}

int dAnimationCharacterRig::OnAABBOverlap(const NewtonBody * const body, void* const context)
{
	dCollectCollidingBodies* const bodyList = (dCollectCollidingBodies*)context;
	if (body != bodyList->m_exclude) {
		dFloat mass;
		dFloat Ixx;
		dFloat Iyy;
		dFloat Izz;
		NewtonBodyGetMass(body, &mass, &Ixx, &Iyy, &Izz);
		bodyList->m_array[bodyList->m_count] = (NewtonBody*)body;
		if (mass == 0.0f) {
			for (int i = bodyList->m_count; i > 0; i --) {
				bodyList->m_array[i] = bodyList->m_array[i - 1];
			}
			bodyList->m_array[0] = (NewtonBody*)body;
			bodyList->m_staticCount++;
		}
		bodyList->m_count++;
		
		dAssert(bodyList->m_count < sizeof(bodyList->m_array) / sizeof(bodyList->m_array[1]));
	}
	return 1;
}

void dAnimationCharacterRig::CalculateTireContacts(dFloat timestep)
{
	dComplementaritySolver::dBodyState* const chassisBody = m_vehicle->GetBody();
	const dMatrix& matrix = chassisBody->GetMatrix();
	dVector origin(matrix.TransformVector(m_obbOrigin));
	dVector size(matrix.m_front.Abs().Scale(m_obbSize.m_x) + matrix.m_up.Abs().Scale(m_obbSize.m_y) + matrix.m_right.Abs().Scale(m_obbSize.m_z));

	dVector p0 (origin - size);
	dVector p1 (origin + size);

	dCollectCollidingBodies bodyList(GetBody());
	NewtonWorld* const world = NewtonBodyGetWorld(GetBody());
	NewtonWorldForEachBodyInAABBDo(world, &p0.m_x, &p1.m_x, OnAABBOverlap, &bodyList);

	const dList<dAnimationAcyclicJoint*>& children = m_vehicle->GetChildren();
	for (dList<dAnimationAcyclicJoint*>::dListNode* tireNode = children.GetFirst(); tireNode; tireNode = tireNode->GetNext()) {
		dVehicleNode* const node = (dVehicleNode*)tireNode->GetInfo();
		dVehicleVirtualTire* const tire = (dVehicleVirtualTire*)node->GetAsTire();
		if (tire) {
			tire->CalculateContacts(bodyList, timestep);
		}
	}
}

void dAnimationCharacterRig::PostUpdate(dFloat timestep, int threadIndex)
{
	m_vehicle->RigidBodyToStates();
}

void dAnimationCharacterRig::PreUpdate(dFloat timestep, int threadIndex)
{
	dVehicleManager* const manager = (dVehicleManager*)GetManager();
	manager->UpdateDriverInput(this, timestep);

	if (m_steeringControl) {
		m_steeringControl->Update(timestep);
	}

	if (m_brakeControl) {
		m_brakeControl->Update(timestep);
	}

	if (m_handBrakeControl) {
		m_handBrakeControl->Update(timestep);
	}

	if (m_engineControl) {
		m_engineControl->Update(timestep);
	}
	
	m_vehicle->RigidBodyToStates();
	m_solver.Update(timestep);
	m_vehicle->Integrate(timestep);
	m_vehicle->StatesToRigidBody(timestep);
}

void dAnimationCharacterRig::ApplyDriverInputs(const dDriverInput& driveInputs, dFloat timestep)
{
	if (m_steeringControl) {
		m_steeringControl->SetParam(driveInputs.m_steeringValue);
	}

	if (m_brakeControl) {
		m_brakeControl->SetParam(driveInputs.m_brakePedal);
	}

	if (m_handBrakeControl) {
		m_handBrakeControl->SetParam(driveInputs.m_handBrakeValue);
	}

if (m_engineControl) {
m_engineControl->SetParam(driveInputs.m_throttle);
m_engineControl->SetClutch(driveInputs.m_clutchPedal);
m_engineControl->SetGear(dVehicleEngineInterface::m_firstGear);
}
}

void dAnimationCharacterRig::CalculateSuspensionForces(dFloat timestep)
{
	const int maxSize = 64;
	dComplementaritySolver::dJacobianPair m_jt[maxSize];
	dComplementaritySolver::dJacobianPair m_jInvMass[maxSize];
	dVehicleVirtualTire* tires[maxSize];
	dFloat massMatrix[maxSize * maxSize];
	dFloat accel[maxSize];
	
	dComplementaritySolver::dBodyState* const chassisBody = m_vehicle->GetBody();

	const dMatrix& chassisMatrix = chassisBody->GetMatrix(); 
	const dMatrix& chassisInvInertia = chassisBody->GetInvInertia();
	dVector chassisOrigin (chassisMatrix.TransformVector (chassisBody->GetCOM()));
	dFloat chassisInvMass = chassisBody->GetInvMass();

	int tireCount = 0;
	const dList<dAnimationAcyclicJoint*>& children = m_vehicle->GetChildren();
	for (dList<dAnimationAcyclicJoint*>::dListNode* tireNode = children.GetFirst(); tireNode; tireNode = tireNode->GetNext()) {
		dVehicleNode* const node = (dVehicleNode*)tireNode->GetInfo();
		dVehicleVirtualTire* const tire = (dVehicleVirtualTire*)node->GetAsTire();
		if (tire) {
			const dVehicleVirtualTire::dTireInfo& info = tire->m_info;
			tires[tireCount] = tire;
			dFloat x = tire->m_position;
			dFloat v = tire->m_speed;
			dFloat weight = 1.0f;

			dComplementaritySolver::dBodyState* const tireBody = tire->GetBody();

			const dFloat invMass = tireBody->GetInvMass();
			const dFloat kv = info.m_dampingRatio * invMass;
			const dFloat ks = info.m_springStiffness * invMass;
			accel[tireCount] = -NewtonCalculateSpringDamperAcceleration(timestep, ks * weight, x, kv, v);

			const dMatrix& tireMatrix = tireBody->GetMatrix(); 
			const dMatrix& tireInvInertia = tireBody->GetInvInertia();
			dFloat tireMass = tireBody->GetInvMass();

			m_jt[tireCount].m_jacobian_J01.m_linear = chassisMatrix.m_up.Scale(-1.0f);
			m_jt[tireCount].m_jacobian_J01.m_angular = dVector(0.0f);
			m_jt[tireCount].m_jacobian_J10.m_linear = chassisMatrix.m_up;
			m_jt[tireCount].m_jacobian_J10.m_angular = (tireMatrix.m_posit - chassisOrigin).CrossProduct(chassisMatrix.m_up);

			m_jInvMass[tireCount].m_jacobian_J01.m_linear = m_jt[tireCount].m_jacobian_J01.m_linear.Scale(tireMass);
			m_jInvMass[tireCount].m_jacobian_J01.m_angular = tireInvInertia.RotateVector(m_jt[tireCount].m_jacobian_J01.m_angular);
			m_jInvMass[tireCount].m_jacobian_J10.m_linear = m_jt[tireCount].m_jacobian_J10.m_linear.Scale(chassisInvMass);
			m_jInvMass[tireCount].m_jacobian_J10.m_angular = chassisInvInertia.RotateVector(m_jt[tireCount].m_jacobian_J10.m_angular);

			tireCount++;
		}
	}

	for (int i = 0; i < tireCount; i++) {
		dFloat* const row = &massMatrix[i * tireCount];

		dFloat aii = m_jInvMass[i].m_jacobian_J01.m_linear.DotProduct3(m_jt[i].m_jacobian_J01.m_linear) + m_jInvMass[i].m_jacobian_J01.m_angular.DotProduct3(m_jt[i].m_jacobian_J01.m_angular) +
					 m_jInvMass[i].m_jacobian_J10.m_linear.DotProduct3(m_jt[i].m_jacobian_J10.m_linear) + m_jInvMass[i].m_jacobian_J10.m_angular.DotProduct3(m_jt[i].m_jacobian_J10.m_angular);

		row[i] = aii * 1.0001f;
		for (int j = i + 1; j < tireCount; j++) {
			dFloat aij = m_jInvMass[i].m_jacobian_J10.m_linear.DotProduct3(m_jt[j].m_jacobian_J10.m_linear) + m_jInvMass[i].m_jacobian_J10.m_angular.DotProduct3(m_jt[j].m_jacobian_J10.m_angular);
			row[j] = aij;
			massMatrix[j * tireCount + i] = aij;
		}
	}

	dCholeskyFactorization(tireCount, massMatrix);
	dCholeskySolve(tireCount, tireCount, massMatrix, accel);

	dVector chassisForce(0.0f);
	dVector chassisTorque(0.0f);
	for (int i = 0; i < tireCount; i++) {
		dVehicleVirtualTire* const tire = tires[i];
		dComplementaritySolver::dBodyState* const tireBody = tire->GetBody();

		dVector tireForce(m_jt[i].m_jacobian_J01.m_linear.Scale(accel[i]));
		tireBody->SetForce(tireBody->GetForce() + tireForce);
		chassisForce += m_jt[i].m_jacobian_J10.m_linear.Scale(accel[i]);
		chassisTorque += m_jt[i].m_jacobian_J10.m_angular.Scale(accel[i]);
	}
	chassisBody->SetForce(chassisBody->GetForce() + chassisForce);
	chassisBody->SetTorque(chassisBody->GetTorque() + chassisTorque);
}
*/


void dAnimationCharacterRig::Debug(dCustomJoint::dDebugDisplay* const debugContext) const
{
	dAssert(0);
//	m_vehicle->Debug(debugContext);
}

void dAnimationCharacterRig::PreUpdate(dFloat timestep, int threadIndex)
{
	dAssert(0);
//	dVehicleManager* const manager = (dVehicleManager*)GetManager();
}

void dAnimationCharacterRig::PostUpdate(dFloat timestep, int threadIndex)
{
//	m_vehicle->RigidBodyToStates();
}
