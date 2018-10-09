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

dVehicleVirtualTire::dVehicleVirtualTire(dVehicleNode* const parent, const dMatrix& locationInGlobalSpace, const dTireInfo& info)
	:dVehicleTireInterface(parent)
	,m_info(info)
	,m_joint()
	,m_dynamicContactBodyNode(NULL, true)
	,m_omega(0.0f)
	,m_speed(0.0f)
	,m_position(0.0f)
	,m_tireLoad(0.0f)
	,m_tireAngle(0.0f)
	,m_steeringAngle(0.0f)
	,m_invSuspensionLength(m_info.m_suspensionLength > 0.0f ? 1.0f/m_info.m_suspensionLength : 0.0f)
{
	SetWorld(parent->GetWorld());
	m_dynamicContactBodyNode.SetWorld(m_world);

	//dVehicleSingleBody* const chassisNode = (dVehicleSingleBody*) m_parent;
	//dVehicleChassis* const chassis = chassisNode->GetChassis();
	//NewtonBody* const newtonBody = chassis->GetBody();
	//NewtonWorld* const world = NewtonBodyGetWorld(newtonBody);

	m_tireShape = NewtonCreateChamferCylinder(m_world, 0.5f, 1.0f, 0, NULL);
	NewtonCollisionSetScale(m_tireShape, m_info.m_width, m_info.m_radio, m_info.m_radio);
dAssert(0);
/*
	dMatrix chassisMatrix;
	NewtonBodyGetMatrix(newtonBody, &chassisMatrix[0][0]);

	dMatrix alignMatrix(dGetIdentityMatrix());
	alignMatrix.m_front = dVector (0.0f, 0.0f, 1.0f, 0.0f);
	alignMatrix.m_up = dVector (0.0f, 1.0f, 0.0f, 0.0f);
	alignMatrix.m_right = alignMatrix.m_front.CrossProduct(alignMatrix.m_up);

	m_matrix = alignMatrix * chassis->m_localFrame;
	m_matrix.m_posit = chassis->m_localFrame.UntransformVector(chassisMatrix.UntransformVector(locationInGlobalSpace.m_posit));

	m_bindingRotation = locationInGlobalSpace * (m_matrix * chassisMatrix).Inverse();
	m_bindingRotation.m_posit = dVector (0.0f, 0.0f, 0.0f, 1.0f);

	dVector com;
	dVector inertia(0.0f);
	NewtonConvexCollisionCalculateInertialMatrix(m_tireShape, &inertia[0], &com[0]);
	// simplify calculation by making wheel inertia spherical
	inertia = dVector(m_info.m_mass * dMax(dMax(inertia.m_x, inertia.m_y), inertia.m_z));

	dComplementaritySolver::dBodyState* const chassisBody = GetBody();
	chassisBody->SetMass(m_info.m_mass);
	chassisBody->SetInertia(inertia.m_x, inertia.m_y, inertia.m_z);
	chassisBody->UpdateInertia();

	// set the tire joint
	m_joint.Init (&m_body, m_parent->GetBody());
	m_joint.m_tire = this;

	for (int i = 0 ; i < sizeof (m_contactsJoints) / sizeof (m_contactsJoints[0]) - 1; i ++) {
		m_contactsJoints[i].SetOwners (this, &chassis->m_groundNode);
	}
	m_contactsJoints[sizeof (m_contactsJoints) / sizeof (m_contactsJoints[0]) - 1].SetOwners(this, &m_dynamicContactBodyNode);

m_omega = -10.0f;
*/
}

dVehicleVirtualTire::~dVehicleVirtualTire()
{
	NewtonDestroyCollision(m_tireShape);
}

NewtonCollision* dVehicleVirtualTire::GetCollisionShape() const
{
	return m_tireShape;
}

void dVehicleVirtualTire::CalculateNodeAABB(const dMatrix& matrix, dVector& minP, dVector& maxP) const
{
	CalculateAABB(GetCollisionShape(), matrix, minP, maxP);
}

dComplementaritySolver::dBilateralJoint* dVehicleVirtualTire::GetJoint()
{
	return &m_joint;
}

void dVehicleVirtualTire::RenderDebugTire(void* userData, int vertexCount, const dFloat* const faceVertec, int id)
{
	dCustomJoint::dDebugDisplay* const debugContext = (dCustomJoint::dDebugDisplay*) userData;

	int index = vertexCount - 1;
	dVector p0(faceVertec[index * 3 + 0], faceVertec[index * 3 + 1], faceVertec[index * 3 + 2]);
	for (int i = 0; i < vertexCount; i++) {
		dVector p1(faceVertec[i * 3 + 0], faceVertec[i * 3 + 1], faceVertec[i * 3 + 2]);
		debugContext->DrawLine(p0, p1);
		p0 = p1;
	}
}

dMatrix dVehicleVirtualTire::GetHardpointMatrix (dFloat param) const
{
	dMatrix matrix(dYawMatrix(m_steeringAngle) * m_matrix);
	matrix.m_posit += m_matrix.m_up.Scale(param * m_info.m_suspensionLength - m_info.m_pivotOffset);
	return matrix;
}

dMatrix dVehicleVirtualTire::GetLocalMatrix () const
{
	return m_bindingRotation * dPitchMatrix(m_tireAngle) * GetHardpointMatrix(m_position * m_invSuspensionLength);
}

dMatrix dVehicleVirtualTire::GetGlobalMatrix () const
{
	dMatrix newtonBodyMatrix;
dAssert(0);
return newtonBodyMatrix;
/*
	dVehicleSingleBody* const chassisNode = (dVehicleSingleBody*)m_parent->GetAsVehicle();
	dAssert (chassisNode);
	dVehicleChassis* const chassis = chassisNode->GetChassis();
	NewtonBodyGetMatrix(chassis->GetBody(), &newtonBodyMatrix[0][0]);
	return GetLocalMatrix() * newtonBodyMatrix;
*/
}

void dVehicleVirtualTire::Debug(dCustomJoint::dDebugDisplay* const debugContext) const
{
	dVehicleTireInterface::Debug(debugContext);

	dMatrix trieMatrix (GetGlobalMatrix ());
	NewtonCollisionForEachPolygonDo(m_tireShape, &trieMatrix[0][0], RenderDebugTire, debugContext);
}

void dVehicleVirtualTire::SetSteeringAngle(dFloat steeringAngle)
{
	m_steeringAngle = steeringAngle;
}

void dVehicleVirtualTire::InitRigiBody(dFloat timestep)
{
	dAssert(0);
/*
	dVehicleSingleBody* const chassisNode = (dVehicleSingleBody*)m_parent;
	dVehicleChassis* const chassis = chassisNode->GetChassis();
	dComplementaritySolver::dBodyState* const tireBody = GetBody();
	dComplementaritySolver::dBodyState* const chassisBody = chassisNode->GetBody();

	dMatrix tireMatrix (GetHardpointMatrix (m_position * m_invSuspensionLength) * chassisBody->GetMatrix());
	tireBody->SetMatrix(tireMatrix);

	tireBody->SetOmega(chassisBody->GetOmega() + tireMatrix.m_front.Scale(m_omega));
	tireBody->SetVeloc(chassisBody->CalculatePointVelocity (tireMatrix.m_posit) + tireMatrix.m_up.Scale(m_speed));

	tireBody->SetTorque(dVector (0.0f));
	tireBody->SetForce(chassis->m_gravity.Scale (tireBody->GetMass()));

	dVehicleTireInterface::InitRigiBody(timestep);

m_tireAngle = dMod(m_tireAngle + m_omega * timestep, 2.0f * dPi);
*/
}

int dVehicleVirtualTire::GetKinematicLoops(dKinematicLoopJoint** const jointArray)
{
	int count = 0;
	for (int i = 0; i < sizeof (m_contactsJoints) / sizeof (m_contactsJoints[0]); i ++) {
		dKinematicLoopJoint* const loop = &m_contactsJoints[i];
		if (loop->IsActive ()) {
			jointArray[count] = loop;
			dAssert (!loop->GetOwner0()->IsLoopNode());
			if (loop->GetOwner1()->IsLoopNode()) {
				loop->GetOwner1()->SetIndex(-1);
			}
			count ++;
		}
	}
	return dVehicleTireInterface::GetKinematicLoops(&jointArray[count]) + count;
}

void dVehicleVirtualTire::dTireJoint::JacobianDerivative(dComplementaritySolver::dParamInfo* const constraintParams)
{
	// Restrict the movement on the pivot point along all two orthonormal direction
	dComplementaritySolver::dBodyState* const tire = m_state0;
	dComplementaritySolver::dBodyState* const chassis = m_state1;

	const dVector& omega = chassis->GetOmega();
	const dMatrix& tireMatrix = tire->GetMatrix();

	// lateral force
	AddLinearRowJacobian(constraintParams, tireMatrix.m_posit, tireMatrix.m_front, omega);

	// longitudinal force
	AddLinearRowJacobian(constraintParams, tireMatrix.m_posit, tireMatrix.m_right, omega);

	// angular constraints	
	AddAngularRowJacobian(constraintParams, tireMatrix.m_up, omega, 0.0f);
	AddAngularRowJacobian(constraintParams, tireMatrix.m_right, omega, 0.0f);

/*
	// dry rolling friction (for now contact, bu it should be a function of the tire angular velocity)
	int index = constraintParams->m_count;
	AddAngularRowJacobian(constraintParams, tire->m_matrix[0], 0.0f);
	constraintParams->m_jointLowFriction[index] = -chassis->m_dryRollingFrictionTorque;
	constraintParams->m_jointHighFriction[index] = chassis->m_dryRollingFrictionTorque;

	// check if the brakes are applied
	if (tire->m_brakeTorque > 1.0e-3f) {
		// brake is on override rolling friction value
		constraintParams->m_jointLowFriction[index] = -tire->m_brakeTorque;
		constraintParams->m_jointHighFriction[index] = tire->m_brakeTorque;
	}

	// clear the input variable after there are res
	tire->m_brakeTorque = 0.0f;
*/
}


void dVehicleVirtualTire::CalculateContacts(const dVehicleChassis::dCollectCollidingBodies& bodyArray, dFloat timestep)
{
	dAssert(0);
/*
	for (int i = 0; i < sizeof(m_contactsJoints) / sizeof(m_contactsJoints[0]); i++) {
		m_contactsJoints[i].m_isActive = false;
	}
	if (bodyArray.m_count) {
		dVehicleSingleBody* const chassisNode = (dVehicleSingleBody*)m_parent;
		dComplementaritySolver::dBodyState* const chassisBody = chassisNode->GetBody();

		dMatrix tireMatrix (GetHardpointMatrix (1.0f) * chassisBody->GetMatrix());
		dVector veloc0 (tireMatrix.m_up.Scale (-m_info.m_suspensionLength));
		dVector tmp (0.0f);

		dVector contact(0.0f);
		dVector normal(0.0f);
		dFloat penetration(0.0f);

		NewtonWorld* const world = NewtonBodyGetWorld(chassisNode->GetChassis()->GetBody());
		for (int i = 0; i < bodyArray.m_count; i ++) {
			dMatrix matrixB;
			dLong attributeA;
			dLong attributeB;
			dFloat impactParam;

			NewtonBodyGetMatrix(bodyArray.m_array[i], &matrixB[0][0]);
			NewtonCollision* const otherShape = NewtonBodyGetCollision (bodyArray.m_array[i]);

			int count = NewtonCollisionCollideContinue(world, 1, 1.0f,
				m_tireShape, &tireMatrix[0][0], &veloc0[0], &tmp[0],
				otherShape, &matrixB[0][0], &tmp[0], &tmp[0], 
				&impactParam, &contact[0], &normal[0], &penetration,
				&attributeA, &attributeB, 0);
		}
	}
*/
}

void dVehicleVirtualTire::dContact::JacobianDerivative(dComplementaritySolver::dParamInfo* const constraintParams)
{

}


