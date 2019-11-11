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
#include "dVehicleNode.h"
#include "dVehicleTire.h"
#include "dVehicleMultiBody.h"
#include "dVehicleCollidingNode.h"

#define D_FREE_ROLLING_TORQUE_COEF	dFloat (0.5f) 
#define D_LOAD_ROLLING_TORQUE_COEF	dFloat (0.01f) 

dVehicleTire::dVehicleTire(dVehicleMultiBody* const chassis, const dMatrix& locationInGlobalSpace, const dTireInfo& info)
	:dVehicleNode(chassis)
	,dBilateralJoint()
	,m_matrix(dGetIdentityMatrix())
	,m_bindingRotation(dGetIdentityMatrix())
	,m_info(info)
	,m_tireShape(NULL)
	,m_omega(0.0f)
	,m_speed(0.0f)
	,m_position(0.0f)
	,m_tireAngle(0.0f)
	,m_brakeTorque(0.0f)
	,m_steeringAngle(0.0f)
	,m_invSuspensionLength(m_info.m_suspensionLength > 0.0f ? 1.0f / m_info.m_suspensionLength : 0.0f)
	,m_contactCount(0)
{
	Init(&m_proxyBody, &GetParent()->GetProxyBody());
	
	NewtonBody* const chassisBody = chassis->GetBody();
	NewtonWorld* const world = NewtonBodyGetWorld(chassis->GetBody());

#if 1
	// tire shaped more like real tires with rim and carcase, harder in collision
	m_tireShape = NewtonCreateChamferCylinder(world, 0.75f, 0.5f, 0, NULL);
	NewtonCollisionSetScale(m_tireShape, 2.0f * m_info.m_width, m_info.m_radio, m_info.m_radio);
#elif 1
	// tire shape like real tires with rim and carcase, harder in collision
	m_tireShape = NewtonCreateChamferCylinder(world, 0.5f, 1.0f, 0, NULL);
	NewtonCollisionSetScale(m_tireShape, m_info.m_width, m_info.m_radio, m_info.m_radio);
#else
	// tire is a simple chamfered cylinder, faster collision
	m_tireShape = NewtonCreateChamferCylinder(world, m_info.m_width*0.25f, m_info.m_radio*1.5f, 0, NULL);
#endif

	dMatrix chassisMatrix;
	NewtonBodyGetMatrix(chassisBody, &chassisMatrix[0][0]);

	dMatrix alignMatrix(dGetIdentityMatrix());
	alignMatrix.m_front = dVector(0.0f, 0.0f, 1.0f, 0.0f);
	alignMatrix.m_up = dVector(1.0f, 0.0f, 0.0f, 0.0f);
	alignMatrix.m_right = alignMatrix.m_front.CrossProduct(alignMatrix.m_up);

	const dMatrix& localFrame = chassis->GetLocalFrame();
	m_matrix = alignMatrix * localFrame;
	m_matrix.m_posit = localFrame.UntransformVector(chassisMatrix.UntransformVector(locationInGlobalSpace.m_posit));

	m_bindingRotation = locationInGlobalSpace * (m_matrix * chassisMatrix).Inverse();
	m_bindingRotation.m_posit = dVector(0.0f, 0.0f, 0.0f, 1.0f);

	dVector com(0.0f);
	dVector inertia(0.0f);
	NewtonConvexCollisionCalculateInertialMatrix(m_tireShape, &inertia[0], &com[0]);
	// simplify calculation by making wheel inertia spherical
	inertia = dVector(m_info.m_mass * dMax(dMax(inertia.m_x, inertia.m_y), inertia.m_z));

	m_proxyBody.SetMass(m_info.m_mass);
	m_proxyBody.SetInertia(inertia.m_x, inertia.m_y, inertia.m_z);
	m_proxyBody.UpdateInertia();
}

dVehicleTire::~dVehicleTire()
{
	if (m_tireShape) {
		NewtonDestroyCollision(m_tireShape);
	}
}

NewtonCollision* dVehicleTire::GetCollisionShape() const
{
	return m_tireShape;
}

void dVehicleTire::CalculateNodeAABB(const dMatrix& matrix, dVector& minP, dVector& maxP) const
{
	CalculateAABB(GetCollisionShape(), matrix, minP, maxP);
}

dFloat dVehicleTire::GetBrakeTorque() const
{
	return m_brakeTorque;
}

void dVehicleTire::SetBrakeTorque(dFloat brakeTorque)
{
	m_brakeTorque = dAbs(brakeTorque);
}

const dTireInfo& dVehicleTire::GetInfo() const
{
	return m_info;
}

void dVehicleTire::SetSteeringAngle(dFloat steeringAngle)
{
	m_steeringAngle = steeringAngle;
}

dFloat dVehicleTire::GetSteeringAngle() const
{
	return m_steeringAngle;
}

int dVehicleTire::GetKinematicLoops(dVehicleLoopJoint** const jointArray)
{
	int count = 0;
	for (int i = 0; i < sizeof (m_contactsJoints) / sizeof (m_contactsJoints[0]); i ++) {
		dVehicleLoopJoint* const contact = &m_contactsJoints[i];
		if (contact->IsActive ()) {
			jointArray[count] = contact;
			count ++;
		}
	}
	return count;
}

void dVehicleTire::CalculateContacts(const dCollectCollidingBodies& bodyArray, dFloat timestep)
{
	for (int i = 0; i < sizeof(m_contactsJoints) / sizeof(m_contactsJoints[0]); i++) {
		m_contactsJoints[i].ResetContact();
	}

	int contactCount = 0;
	dFloat friction = m_info.m_frictionCoefficient;

	dVehicleMultiBody* const chassisNode = GetParent()->GetAsVehicleMultiBody();
	NewtonWorld* const world = NewtonBodyGetWorld(chassisNode->GetBody());
	dComplementaritySolver::dBodyState* const chassisBody = &chassisNode->GetProxyBody();

	dMatrix matrixB;
	dMatrix tireMatrix (GetHardpointMatrix(m_position * m_invSuspensionLength) * chassisBody->GetMatrix());
	
	const int maxContactCount = sizeof (m_contactsJoints) / sizeof (m_contactsJoints[0]);
	dFloat points[maxContactCount][3];
	dFloat normals[maxContactCount][3];
	dFloat penetrations[maxContactCount];
	dLong attributeA[maxContactCount];
	dLong attributeB[maxContactCount];

	for (int i = 0; (i < bodyArray.m_count) && (contactCount < maxContactCount); i++) {
		// calculate tire contact collision with rigid bodies

		NewtonBodyGetMatrix(bodyArray.m_array[i], &matrixB[0][0]);
		NewtonCollision* const otherShape = NewtonBodyGetCollision(bodyArray.m_array[i]);
		int count = NewtonCollisionCollide(world, maxContactCount, 
										   m_tireShape, &tireMatrix[0][0], 
										   otherShape, &matrixB[0][0],
										   &points[0][0], &normals[0][0], penetrations, attributeA, attributeB, 0);

		for (int j = 0; (j < count) && (contactCount < maxContactCount); j ++) {
			dVector normal (normals[j][0], normals[j][1], normals[j][2], dFloat (0.0f));
			dVector longitudinalDir(normal.CrossProduct(tireMatrix.m_front));
			if (longitudinalDir.DotProduct3(longitudinalDir) < 0.1f) {
				longitudinalDir = normal.CrossProduct(tireMatrix.m_up.CrossProduct(normal));
				dAssert(longitudinalDir.DotProduct3(longitudinalDir) > 0.1f);
			}

			longitudinalDir = longitudinalDir.Normalize();
			dVehicleCollidingNode* const collidingNode = chassisNode->FindCollideNode(this, bodyArray.m_array[i]);
			m_contactsJoints[contactCount].SetOwners(this, collidingNode);

			dVector contact (points[j][0], points[j][1], points[j][2], dFloat (1.0f));
			m_contactsJoints[contactCount].SetContact(collidingNode, contact, normal, longitudinalDir, penetrations[j], friction, false);
			contactCount++;
			dAssert(contactCount <= sizeof(m_contactsJoints) / sizeof(m_contactsJoints[0]));
		}
	}

	if (contactCount > 1) {
		// filter contact that are too close 
		dFloat angle[maxContactCount];
		for (int i = 0; i < contactCount; i ++) {
			angle[i] = tireMatrix.m_right.DotProduct3(m_contactsJoints[i].m_normal);
		}

		for (int i = 0; i < contactCount - 1; i ++) {
			for (int j = contactCount - 1; j > i; j --) {
				if (angle[j - 1] > angle[j]) {
					dSwap (angle[j - 1], angle[j]);
					dSwap (m_contactsJoints[j - 1], m_contactsJoints[j]); 
				}
			}
		}

		for (int i = contactCount - 1; i > 0; i --) {
			const dVector& p = m_contactsJoints[i].m_point;
			for (int j = i - 1; j >= 0; j --) {
				const dVector error (m_contactsJoints[j].m_point - p);
				dFloat error2 = error.DotProduct3(error);
				if (error2 < 3.0e-2f) {			
					m_contactsJoints[j] = m_contactsJoints[i];
					contactCount --;
					break;
				}
			}
		}

		m_contactCount = contactCount; 
	}
}

dMatrix dVehicleTire::GetHardpointMatrix(dFloat param) const
{
	dMatrix matrix(dRollMatrix(m_steeringAngle) * m_matrix);
	matrix.m_posit += m_matrix.m_right.Scale(param * m_info.m_suspensionLength - m_info.m_pivotOffset);
	return matrix;
}

dMatrix dVehicleTire::GetLocalMatrix() const
{
	return m_bindingRotation * dPitchMatrix(m_tireAngle) * GetHardpointMatrix(m_position * m_invSuspensionLength);
}

dMatrix dVehicleTire::GetGlobalMatrix() const
{
	dMatrix newtonBodyMatrix;
	dVehicleMultiBody* const chassisNode = GetParent()->GetAsVehicleMultiBody();
	dAssert(chassisNode);
	return GetLocalMatrix() * chassisNode->GetProxyBody().GetMatrix();
}

void dVehicleTire::RenderDebugTire(void* userData, int vertexCount, const dFloat* const faceVertec, int id)
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

const void dVehicleTire::Debug(dCustomJoint::dDebugDisplay* const debugContext) const
{
	debugContext->SetColor(dVector(0.0f, 0.4f, 0.7f, 1.0f));
	dMatrix tireMatrix(m_bindingRotation.Transpose() * GetGlobalMatrix());
	NewtonCollisionForEachPolygonDo(m_tireShape, &tireMatrix[0][0], RenderDebugTire, debugContext);

	dVehicleMultiBody* const chassis = GetParent()->GetAsVehicleMultiBody();
	dAssert (chassis);

	// render tireState matrix
	dComplementaritySolver::dBodyState* const chassisBody = &chassis->GetProxyBody();
	dVector weight (chassis->GetGravity().Scale(chassisBody->GetMass()));
//	dFloat scale (1.0f / dSqrt (weight.DotProduct3(weight)));
	for (int i = 0; i < sizeof (m_contactsJoints)/sizeof (m_contactsJoints[0]); i ++) {
		const dVehicleTireContact* const contact = &m_contactsJoints[i];
		if (contact->IsActive()) {
//			contact->Debug(debugContext, scale);
			debugContext->SetColor(dVector(1.0f, 0.0f, 0.0f, 1.0f));
			debugContext->DrawPoint(contact->m_point, 4.0f);
			debugContext->DrawLine(contact->m_point, contact->m_point + contact->m_normal.Scale (2.0f));
		}
	}
}

void dVehicleTire::Integrate(dFloat timestep)
{
	m_proxyBody.IntegrateForce(timestep, m_proxyBody.GetForce(), m_proxyBody.GetTorque());

	m_proxyBody.IntegrateVelocity(timestep);
	for (int i = 0; i < m_contactCount; i ++) {
		const dVehicleTireContact* const contact = &m_contactsJoints[i];
		if (contact->m_isActive && contact->m_collidingNode->m_body) {

			dMatrix matrix;
			dVector com;
			NewtonBodyGetMatrix(contact->m_collidingNode->m_body, &matrix[0][0]);
			NewtonBodyGetCentreOfMass(contact->m_collidingNode->m_body, &com[0]);

			dVector r (contact->m_point - matrix.TransformVector(com));
			dVector force (contact->m_normal.Scale (-contact->m_tireModel.m_tireLoad) +
						   contact->m_longitudinalDir.Scale (-contact->m_tireModel.m_longitunalForce) + 
						   contact->m_lateralDir.Scale (-contact->m_tireModel.m_lateralForce));
			if (contact->m_isContactPatch) {
				dFloat invMass;
				dFloat invIxx;
				dFloat invIyy;
				dFloat invIzz;
				NewtonBodyGetInvMass(contact->m_collidingNode->m_body, &invMass, &invIxx, &invIyy, &invIzz);
				dAssert (invMass > 0.0f);
				dVector accel (force.Scale (invMass));
				dFloat mag2 = accel.DotProduct3(accel);
				const dFloat maxAccel = 100.0f;
				if (mag2 > maxAccel * maxAccel) {
					force = accel.Scale (maxAccel / (invMass * dSqrt (mag2)));
				}
			}

			dVector torque (r.CrossProduct(force));
			NewtonBodyAddForce(contact->m_collidingNode->m_body, &force[0]);
			NewtonBodyAddTorque(contact->m_collidingNode->m_body, &torque[0]);
		}
	}
}

void dVehicleTire::ApplyExternalForce()
{
	dVehicleMultiBody* const chassisNode = GetParent()->GetAsVehicleMultiBody();
	dComplementaritySolver::dBodyState* const chassisBody = &chassisNode->GetProxyBody();

	dMatrix tireMatrix(GetHardpointMatrix(m_position * m_invSuspensionLength) * chassisBody->GetMatrix());
	m_proxyBody.SetMatrix(tireMatrix);

	m_proxyBody.SetOmega(chassisBody->GetOmega() + tireMatrix.m_front.Scale(m_omega));
	m_proxyBody.SetVeloc(chassisBody->CalculatePointVelocity(tireMatrix.m_posit) + tireMatrix.m_right.Scale(m_speed));

	m_proxyBody.SetTorque(dVector(0.0f));
	m_proxyBody.SetForce(chassisNode->GetGravity().Scale(m_proxyBody.GetMass()));
}

void dVehicleTire::CalculateFreeDof()
{
	dVehicleMultiBody* const chassis = GetParent()->GetAsVehicleMultiBody();
	dComplementaritySolver::dBodyState* const chassisBody = &chassis->GetProxyBody();

	dMatrix chassisMatrix(GetHardpointMatrix(0.0f) * chassisBody->GetMatrix());
	dMatrix tireMatrix (m_proxyBody.GetMatrix());

	dVector tireOmega(m_proxyBody.GetOmega());
	dVector chassisOmega(chassisBody->GetOmega());
	dVector relativeOmega(tireOmega - chassisOmega);
	m_omega = tireMatrix.m_front.DotProduct3(relativeOmega);
	dAssert(tireMatrix.m_front.DotProduct3(chassisMatrix.m_front) > 0.998f);

	dFloat cosAngle = tireMatrix.m_right.DotProduct3(chassisMatrix.m_right);
	dFloat sinAngle = chassisMatrix.m_front.DotProduct3(chassisMatrix.m_right.CrossProduct(tireMatrix.m_right));
	m_tireAngle += dAtan2(sinAngle, cosAngle);
	while (m_tireAngle < 0.0f)
	{
		m_tireAngle += 2.0f * dPi;
	}
	m_tireAngle = dMod(m_tireAngle, dFloat(2.0f * dPi));

	dVector tireVeloc(m_proxyBody.GetVelocity());
	dVector chassisPointVeloc(chassisBody->CalculatePointVelocity(tireMatrix.m_posit));
	dVector localVeloc(tireVeloc - chassisPointVeloc);
	m_speed = tireMatrix.m_right.DotProduct3(localVeloc);

	m_position = chassisMatrix.m_right.DotProduct3(tireMatrix.m_posit - chassisMatrix.m_posit);
	if (m_position <= -m_info.m_suspensionLength * 0.25f) {
		m_speed = 0.0f;
		m_position = 0.0f;
	} else if (m_position >= m_info.m_suspensionLength * 1.25f) {
		m_speed = 0.0f;
		m_position = m_info.m_suspensionLength;
	}
}

void dVehicleTire::UpdateSolverForces(const dComplementaritySolver::dJacobianPair* const jacobians) const
{
	dAssert(0);
}

void dVehicleTire::JacobianDerivative(dComplementaritySolver::dParamInfo* const constraintParams)
{
	dComplementaritySolver::dBodyState* const tireState = m_state0;
	const dMatrix& tireMatrix = tireState->GetMatrix();

	// lateral force
	AddLinearRowJacobian(constraintParams, tireMatrix.m_posit, tireMatrix.m_front);

	// longitudinal force
	AddLinearRowJacobian(constraintParams, tireMatrix.m_posit, tireMatrix.m_up);

	// angular constraints
	AddAngularRowJacobian(constraintParams, tireMatrix.m_up, 0.0f);
	AddAngularRowJacobian(constraintParams, tireMatrix.m_right, 0.0f);

	if (m_position < 0.0f) {
		int index = constraintParams->m_count;
		AddLinearRowJacobian(constraintParams, tireMatrix.m_posit, tireMatrix.m_right);
		dFloat speed = -0.1f;
		dFloat step = 2.0f * speed * constraintParams->m_timestep;
		if (m_position > step) {
			speed = 0.25f * m_position * constraintParams->m_timestepInv;
		}
		dFloat accel = GetRowAccelaration(constraintParams) - speed * constraintParams->m_timestepInv;
		SetRowAccelaration(constraintParams, accel);
		constraintParams->m_jointLowFrictionCoef[index] = 0.0f;
	} else if (m_position >= m_info.m_suspensionLength) {
		int index = constraintParams->m_count;
		AddLinearRowJacobian(constraintParams, tireMatrix.m_posit, tireMatrix.m_right);
		dFloat speed = 0.1f;
		dFloat step = 2.0f * speed * constraintParams->m_timestep;
		if ((m_position - m_info.m_suspensionLength) > step) {
			speed = 0.25f * (m_position - m_info.m_suspensionLength) * constraintParams->m_timestepInv;
		}
		dFloat accel = GetRowAccelaration(constraintParams) - speed * constraintParams->m_timestepInv;
		SetRowAccelaration(constraintParams, accel);
		constraintParams->m_jointHighFrictionCoef[index] = 0.0f;
	}

	dFloat Ixx;
	dFloat Iyy;
	dFloat Izz;
	tireState->GetInertia (Ixx, Iyy, Izz);
	dFloat rollingFriction = dMax (m_brakeTorque, Ixx * D_FREE_ROLLING_TORQUE_COEF);
	for (int i = 0; i < sizeof (m_contactsJoints) / sizeof (m_contactsJoints[0]) - 1; i++) {
		if (m_contactsJoints[i].IsActive()) {
			rollingFriction = dMax (rollingFriction, m_contactsJoints[i].m_tireModel.m_tireLoad * D_LOAD_ROLLING_TORQUE_COEF);
		}
	}

	int index = constraintParams->m_count;
	AddAngularRowJacobian(constraintParams, tireMatrix.m_front, 0.0f);
	constraintParams->m_jointLowFrictionCoef[index] = -rollingFriction;
	constraintParams->m_jointHighFrictionCoef[index] = rollingFriction;

	m_brakeTorque = 0.0f;
}