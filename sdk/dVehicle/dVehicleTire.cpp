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

#define D_FREE_ROLLING_TIRE_TORQUE 30.0f

dVehicleTire::dVehicleTire(dVehicleMultiBody* const chassis, const dMatrix& locationInGlobalSpace, const dTireInfo& info)
	:dVehicleNode(chassis)
	,dBilateralJoint()
	,m_matrix(dGetIdentityMatrix())
	,m_bindingRotation(dGetIdentityMatrix())
	,m_dynamicContactBodyNode(NULL)
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

	m_tireShape = NewtonCreateChamferCylinder(world, 0.5f, 1.0f, 0, NULL);
	NewtonCollisionSetScale(m_tireShape, m_info.m_width, m_info.m_radio, m_info.m_radio);

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
	
	for (int i = 0 ; i < sizeof (m_contactsJoints) / sizeof (m_contactsJoints[0]) - 1; i ++) {
		m_contactsJoints[i].SetOwners (this, &chassis->m_groundProxyBody);
	}
	m_contactsJoints[sizeof (m_contactsJoints) / sizeof (m_contactsJoints[0]) - 1].SetOwners(this, &m_dynamicContactBodyNode);
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
		dVehicleLoopJoint* const loop = &m_contactsJoints[i];
		if (loop->IsActive ()) {
			jointArray[count] = loop;
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
	if (bodyArray.m_staticCount) {
		dVehicleMultiBody* const chassisNode = GetParent()->GetAsVehicleMultiBody();
		NewtonWorld* const world = NewtonBodyGetWorld(chassisNode->GetBody());
		dComplementaritySolver::dBodyState* const chassisBody = &chassisNode->GetProxyBody();

		const dMatrix& chassisMatrix = chassisBody->GetMatrix();
		dMatrix tireMatrix (GetHardpointMatrix (1.0f) * chassisMatrix);
		dVector veloc0 (tireMatrix.m_right.Scale (-m_info.m_suspensionLength));
		dVector tmp (0.0f);

		dVector contact(0.0f);
		dVector normal(0.0f);
		dFloat penetration(0.0f);
		dFloat param = 1.0f - m_position * m_invSuspensionLength;

		for (int i = 0; i < bodyArray.m_staticCount; i ++) {
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

			if (count) {
				// calculate tire penetration
				dFloat dist = (param - impactParam) * m_info.m_suspensionLength;
				if (dist > -D_TIRE_MAX_ELASTIC_DEFORMATION) {

					normal.m_w = 0.0f;
					penetration = normal.DotProduct3(tireMatrix.m_right.Scale(dist));

					dVector longitudinalDir (normal.CrossProduct(tireMatrix.m_front));
					if (longitudinalDir.DotProduct3(longitudinalDir) < 0.1f) {
						//lateralDir = normal.CrossProduct(tireMatrix.m_front.CrossProduct(normal));
						longitudinalDir = normal.CrossProduct(tireMatrix.m_up.CrossProduct(normal));
						dAssert(longitudinalDir.DotProduct3(longitudinalDir) > 0.1f);
					}
					longitudinalDir = longitudinalDir.Normalize();

					contact -= tireMatrix.m_up.Scale (dist);
					contact.m_w = 1.0f;
					m_contactsJoints[contactCount].SetContact(contact, normal, longitudinalDir, penetration, friction);
					contactCount ++;
				}
			}
		}
	}

	if (bodyArray.m_count > bodyArray.m_staticCount) {
		// for now ignore tire collision with dynamics bodies,
		dAssert (0);
	}

	if (contactCount > 1) {
		for (int i = 0; i < contactCount - 1; i ++) {
			const dVector& n = m_contactsJoints[i].m_normal;
			for (int j = contactCount - 1; j > i; j --) {
				dFloat val = dAbs (n.DotProduct3(m_contactsJoints[j].m_normal));
				if (val > 0.99f) {
					m_contactsJoints[j] = m_contactsJoints[contactCount - 1];
					contactCount --;
				}
			}
		}
	}

	m_contactCount = contactCount; 
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
	dVehicleNode::Debug(debugContext);

	debugContext->SetColor(dVector(0.0f, 0.4f, 0.7f, 1.0f));
	dMatrix tireMatrix(m_bindingRotation.Transpose() * GetGlobalMatrix());
	NewtonCollisionForEachPolygonDo(m_tireShape, &tireMatrix[0][0], RenderDebugTire, debugContext);

	dVehicleMultiBody* const chassis = GetParent()->GetAsVehicleMultiBody();
	dAssert (chassis);

	// render tire matrix
	dComplementaritySolver::dBodyState* const chassisBody = &chassis->GetProxyBody();
	//dMatrix hubTireMatrix(GetHardpointMatrix(m_position * m_invSuspensionLength) * chassisBody->GetMatrix());
	//debugContext->DrawFrame(hubTireMatrix, 1.0f);
	//debugContext->DrawFrame(tireMatrix, 1.0f);

	dVector weight (chassis->GetGravity().Scale(chassisBody->GetMass()));
	dFloat scale (1.0f / dSqrt (weight.DotProduct3(weight)));
	for (int i = 0; i < sizeof (m_contactsJoints)/sizeof (m_contactsJoints[0]); i ++) {
		const dVehicleTireContact* const contact = &m_contactsJoints[i];
		if (contact->IsActive()) {
			contact->Debug(debugContext, scale);
		}
	}
}

void dVehicleTire::Integrate(dFloat timestep)
{
	m_proxyBody.IntegrateForce(timestep, m_proxyBody.GetForce(), m_proxyBody.GetTorque());
	m_proxyBody.IntegrateVelocity(timestep);
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

/*
if (m_index == 3) {
	dVector veloc(m_proxyBody.GetVelocity());
	float xxxx = dAbs (veloc.DotProduct3(tireMatrix.m_up)) + 0.01f;
	float yyyy = veloc.DotProduct3(tireMatrix.m_front);
	dTrace(("tireBeta =%f\n", dAtan2(yyyy, xxxx) * dRadToDegree));
}
*/

}

void dVehicleTire::CalculateFreeDof()
{
	dVehicleMultiBody* const chassis = GetParent()->GetAsVehicleMultiBody();
	dComplementaritySolver::dBodyState* const chassisBody = &chassis->GetProxyBody();

	//const dMatrix chassisMatrix(chassisBody->GetMatrix());
	//const dMatrix tireMatrix(GetHardpointMatrix(0.0f) * chassisBody->GetMatrix());
	//const dMatrix tireMatrix(GetHardpointMatrix(0.0f) * chassisBody->GetMatrix());

	dMatrix chassisMatrix(GetHardpointMatrix(0.0f) * chassisBody->GetMatrix());
	dMatrix tireMatrix (m_proxyBody.GetMatrix());

	dVector tireOmega(m_proxyBody.GetOmega());
	dVector chassisOmega(chassisBody->GetOmega());
	dVector relativeOmega(tireOmega - chassisOmega);
	m_omega = tireMatrix.m_front.DotProduct3(relativeOmega);
	dAssert(tireMatrix.m_front.DotProduct3(chassisMatrix.m_front) > 0.999f);

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
	dComplementaritySolver::dBodyState* const tire = m_state0;
	const dMatrix& tireMatrix = tire->GetMatrix();

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
	
	if ((m_brakeTorque > 1.0e-3f) || !m_contactCount)  {
		int index = constraintParams->m_count;
		AddAngularRowJacobian(constraintParams, tireMatrix.m_front, 0.0f);

		const dFloat brake = m_contactCount ? m_brakeTorque : D_FREE_ROLLING_TIRE_TORQUE;
		constraintParams->m_jointLowFrictionCoef[index] = -brake;
		constraintParams->m_jointHighFrictionCoef[index] = brake;
	}
	m_brakeTorque = 0.0f;
}