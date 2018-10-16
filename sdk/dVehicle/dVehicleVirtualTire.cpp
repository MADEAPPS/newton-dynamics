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

#define D_TIRE_MAX_ELASTIC_DEFORMATION		(0.05f)
#define D_TIRE_MAX_ELASTIC_NORMAL_STIFFNESS (10.0f / D_TIRE_MAX_ELASTIC_DEFORMATION)

dVehicleVirtualTire::dVehicleVirtualTire(dVehicleNode* const parent, const dMatrix& locationInGlobalSpace, const dTireInfo& info, const dMatrix& localFrame)
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

	dVehicleSingleBody* const chassisNode = (dVehicleSingleBody*) m_parent;
	//dVehicleChassis* const chassis = chassisNode->GetChassis();
	NewtonBody* const newtonBody = chassisNode->m_newtonBody;
	//NewtonWorld* const world = NewtonBodyGetWorld(newtonBody);

	m_tireShape = NewtonCreateChamferCylinder(m_world, 0.5f, 1.0f, 0, NULL);
	NewtonCollisionSetScale(m_tireShape, m_info.m_width, m_info.m_radio, m_info.m_radio);

	dMatrix chassisMatrix;
	NewtonBodyGetMatrix(newtonBody, &chassisMatrix[0][0]);

	dMatrix alignMatrix(dGetIdentityMatrix());
	alignMatrix.m_front = dVector (0.0f, 0.0f, 1.0f, 0.0f);
	alignMatrix.m_up = dVector (0.0f, 1.0f, 0.0f, 0.0f);
	alignMatrix.m_right = alignMatrix.m_front.CrossProduct(alignMatrix.m_up);

	m_matrix = alignMatrix * localFrame;
	m_matrix.m_posit = localFrame.UntransformVector(chassisMatrix.UntransformVector(locationInGlobalSpace.m_posit));

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
		m_contactsJoints[i].SetOwners (this, &chassisNode->m_groundNode);
	}
	m_contactsJoints[sizeof (m_contactsJoints) / sizeof (m_contactsJoints[0]) - 1].SetOwners(this, &m_dynamicContactBodyNode);

//m_omega = -10.0f;
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

	dVehicleSingleBody* const chassisNode = (dVehicleSingleBody*)m_parent->GetAsVehicle();
	dAssert (chassisNode);
	return GetLocalMatrix() * chassisNode->GetBody()->GetMatrix();
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

void dVehicleVirtualTire::ApplyExternalForce()
{
	dVehicleSingleBody* const chassisNode = (dVehicleSingleBody*)m_parent;
	dComplementaritySolver::dBodyState* const tireBody = GetBody();
	dComplementaritySolver::dBodyState* const chassisBody = chassisNode->GetBody();

	dMatrix tireMatrix (GetHardpointMatrix (m_position * m_invSuspensionLength) * chassisBody->GetMatrix());
	tireBody->SetMatrix(tireMatrix);

	tireBody->SetOmega(chassisBody->GetOmega() + tireMatrix.m_front.Scale(m_omega));
	tireBody->SetVeloc(chassisBody->CalculatePointVelocity (tireMatrix.m_posit) + tireMatrix.m_up.Scale(m_speed));

	tireBody->SetTorque(dVector (0.0f));
	tireBody->SetForce(chassisNode->m_gravity.Scale (tireBody->GetMass()));

	dVehicleTireInterface::ApplyExternalForce();

m_tireAngle = dMod(m_tireAngle + m_omega * 0.001f, 2.0f * dPi);

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


void dVehicleVirtualTire::Integrate(dFloat timestep)
{
	// get the 
	dVehicleTireInterface::Integrate(timestep);

	dVehicleSingleBody* const chassis = (dVehicleSingleBody*)m_parent;
	dComplementaritySolver::dBodyState* const chassisBody = chassis->GetBody();
	
	const dMatrix chassisMatrix (chassis->GetBody()->GetMatrix());
	const dMatrix tireMatrix(GetHardpointMatrix(0.0f) * chassisBody->GetMatrix());
//	const dMatrix& tireMatrix = m_body.GetMatrix();
//	m_position = dClamp (matrix0.m_up.DotProduct3(tireMatrix.m_posit - matrix0.m_posit), dFloat (0.0f), m_info.m_suspensionLength);

	dVector chassisOmega(chassisBody->GetOmega());
	dVector chassisVeloc(chassisBody->GetVelocity());
	dVector tireOmega(m_body.GetOmega());
	dVector tireVeloc(m_body.GetVelocity());
	dVector chassinPointVeloc (chassisVeloc + chassisOmega.CrossProduct(tireMatrix.m_posit - chassisMatrix.m_posit));
	dVector veloc (tireVeloc - chassinPointVeloc);

	m_speed = tireMatrix.m_up.DotProduct3(veloc);
	m_position = dClamp (m_position + m_speed * timestep, dFloat (0.0f), m_info.m_suspensionLength);
//dTrace (("%f %f\n", m_speed, m_position));
}


void dVehicleVirtualTire::dTireJoint::JacobianDerivative(dComplementaritySolver::dParamInfo* const constraintParams)
{
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
	for (int i = 0; i < sizeof(m_contactsJoints) / sizeof(m_contactsJoints[0]); i++) {
		dContact* const contact = &m_contactsJoints[i];
		if (contact->m_isActive == false) {
			contact->m_jointFeebackForce[0] = 0.0f;
			contact->m_jointFeebackForce[1] = 0.0f;
			contact->m_jointFeebackForce[2] = 0.0f;
		}
		contact->m_isActive = false;
	}

	if (bodyArray.m_count) {
		dVehicleSingleBody* const chassisNode = (dVehicleSingleBody*)m_parent;
		dComplementaritySolver::dBodyState* const chassisBody = chassisNode->GetBody();

		const dMatrix& chassisMatrix = chassisBody->GetMatrix();
		dMatrix tireMatrix (GetHardpointMatrix (1.0f) * chassisMatrix);
		dVector veloc0 (tireMatrix.m_up.Scale (-m_info.m_suspensionLength));
		dVector tmp (0.0f);
		
		dVector contact(0.0f);
		dVector normal(0.0f);
		dFloat penetration(0.0f);

		dFloat param = 1.0f - m_position * m_invSuspensionLength;

		int contactCount = 0;
		for (int i = 0; i < bodyArray.m_count; i ++) {
			dMatrix matrixB;
			dLong attributeA;
			dLong attributeB;
			dFloat impactParam;

			NewtonBodyGetMatrix(bodyArray.m_array[i], &matrixB[0][0]);
			NewtonCollision* const otherShape = NewtonBodyGetCollision (bodyArray.m_array[i]);

			int count = NewtonCollisionCollideContinue(m_world, 1, 1.0f,
				m_tireShape, &tireMatrix[0][0], &veloc0[0], &tmp[0],
				otherShape, &matrixB[0][0], &tmp[0], &tmp[0], 
				&impactParam, &contact[0], &normal[0], &penetration,
				&attributeA, &attributeB, 0);
			if (count) {
				// calculate tire penetration
				dFloat dist = (param - impactParam) * m_info.m_suspensionLength;
				if (dist > -D_TIRE_MAX_ELASTIC_DEFORMATION) {

					normal.m_w = 0.0f;
					penetration = normal.DotProduct3(tireMatrix.m_up.Scale(dist));

					// calculate contact matrix
					dMatrix contactMatrix;
					contactMatrix[0] = normal;
					contactMatrix[1] = normal.CrossProduct(tireMatrix.m_front);
					dAssert(contactMatrix[1].DotProduct3(contactMatrix[1]) > 0.0f);
					contactMatrix[1] = contactMatrix[1].Normalize();
					contactMatrix[2] = contactMatrix[0].CrossProduct(contactMatrix[1]);
					contactMatrix[3] = contact - tireMatrix.m_up.Scale (dist);
					contactMatrix[3].m_w = 1.0f;
					m_contactsJoints[contactCount].SetContact(contactMatrix, penetration);
					contactCount ++;
				}
			}
		}
	}
}

dVehicleVirtualTire::dContact::dContact()
	:dKinematicLoopJoint()
	,m_contact(dGetIdentityMatrix())
	,m_penetration(0.0f)
{
	m_jointFeebackForce[0] = 0.0f;
	m_jointFeebackForce[1] = 0.0f;
	m_jointFeebackForce[2] = 0.0f;
}

void dVehicleVirtualTire::dContact::SetContact(const dMatrix& contact, dFloat penetration)
{
	m_isActive = true;
	m_contact = contact;
	m_penetration = dClamp (penetration, dFloat(-D_TIRE_MAX_ELASTIC_DEFORMATION), dFloat(D_TIRE_MAX_ELASTIC_DEFORMATION));
}

void dVehicleVirtualTire::dContact::JacobianDerivative(dComplementaritySolver::dParamInfo* const constraintParams)
{
//	dComplementaritySolver::dBodyState* const tire = m_state0;
//	dComplementaritySolver::dBodyState* const other = m_state1;

//	const dVector& omega = chassis->GetOmega();
//	const dMatrix& tireMatrix = tire->GetMatrix();
	dVector omega(0.0f);

	// normal constraint
	AddLinearRowJacobian(constraintParams, m_contact.m_posit, m_contact[0], omega);

	dComplementaritySolver::dJacobian &jacobian0 = constraintParams->m_jacobians[0].m_jacobian_J01;
	dComplementaritySolver::dJacobian &jacobian1 = constraintParams->m_jacobians[0].m_jacobian_J10;

	const dVector& veloc0 = m_state0->GetVelocity();
	const dVector& omega0 = m_state0->GetOmega();
	const dVector& veloc1 = m_state1->GetVelocity();
	const dVector& omega1 = m_state1->GetOmega();
	const dVector relVeloc(veloc0 * jacobian0.m_linear + omega0 * jacobian0.m_angular + veloc1 * jacobian1.m_linear + omega1 * jacobian1.m_angular);
	dFloat relSpeed = -(relVeloc.m_x + relVeloc.m_y + relVeloc.m_z);

	relSpeed += D_TIRE_MAX_ELASTIC_NORMAL_STIFFNESS * m_penetration;
	constraintParams->m_jointLowFriction[0] = 0.0f;
	constraintParams->m_jointAccel[0] = relSpeed * constraintParams->m_timestepInv;
	
	m_dof = 1;
	m_count = 1;
	constraintParams->m_count = 1;
}

