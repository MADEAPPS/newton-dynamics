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

//#define USE_TIRE_SMALL_PATCH
#ifdef USE_TIRE_SMALL_PATCH
#define D_TIRE_CONTACT_PATCH_CONE	dFloat (0.8f) 
#else
#define D_TIRE_CONTACT_PATCH_CONE	dFloat (0.5f) 
#endif


static int xxxx;

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

//	m_tireShape = NewtonCreateChamferCylinder(world, 0.5f, 1.0f, 0, NULL);
//	NewtonCollisionSetScale(m_tireShape, m_info.m_width, m_info.m_radio, m_info.m_radio);
//	m_tireShape = NewtonCreateChamferCylinder(world, 0.75f, 0.5f, 0, NULL);
//	NewtonCollisionSetScale(m_tireShape, 2.0f * m_info.m_width, m_info.m_radio, m_info.m_radio);
	m_tireShape = NewtonCreateChamferCylinder(world, m_info.m_width*0.25f, m_info.m_radio*1.5f, 0, NULL);

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

	dVehicleMultiBody* const chassisNode = GetParent()->GetAsVehicleMultiBody();
	NewtonWorld* const world = NewtonBodyGetWorld(chassisNode->GetBody());
	dComplementaritySolver::dBodyState* const chassisBody = &chassisNode->GetProxyBody();

	const dMatrix& chassisMatrix = chassisBody->GetMatrix();
	dMatrix tireMatrix (GetHardpointMatrix (1.0f) * chassisMatrix);
	dVector veloc0 (tireMatrix.m_right.Scale (-m_info.m_suspensionLength));
	dVector tmp (0.0f);

	dFloat points[2][3];
	dFloat normals[2][3];
	dFloat penetrations[2];
	dFloat param = 1.0f - m_position * m_invSuspensionLength;

if ((m_index == 0) && xxxx >= 4883) {
xxxx*=1;
}

	for (int i = 0; (i < bodyArray.m_count) && (contactCount < sizeof(m_contactsJoints) / sizeof(m_contactsJoints[0])); i ++) {
		dMatrix matrixB;
		dLong attributeA[2];
		dLong attributeB[2];
		dFloat impactParam;

		NewtonBodyGetMatrix(bodyArray.m_array[i], &matrixB[0][0]);
		NewtonCollision* const otherShape = NewtonBodyGetCollision (bodyArray.m_array[i]);

		int count = NewtonCollisionCollideContinue(world, 2, 1.0f,
			m_tireShape, &tireMatrix[0][0], &veloc0[0], &tmp[0],
			otherShape, &matrixB[0][0], &tmp[0], &tmp[0],
			&impactParam, &points[0][0], &normals[0][0], penetrations,
			attributeA, attributeB, 0);

		for (int j = 0; j < count; j ++) {
			// calculate tire contact patch collision
			dVector normal (normals[j][0], normals[j][1], normals[j][2], dFloat (0.0f));
			#ifdef USE_TIRE_SMALL_PATCH
			dFloat projection = normal.DotProduct3(tireMatrix.m_right);
			if (projection > D_TIRE_CONTACT_PATCH_CONE) {
			#else
			dFloat projection = dAbs (normal.DotProduct3(tireMatrix.m_front));
			if (projection < D_TIRE_CONTACT_PATCH_CONE) {
			#endif
		
				dFloat dist = (param - impactParam) * m_info.m_suspensionLength;
				if (dist > -D_TIRE_MAX_ELASTIC_DEFORMATION) {
					dFloat penetration = normal.DotProduct3(tireMatrix.m_right.Scale(dist));

					dVector longitudinalDir (normal.CrossProduct(tireMatrix.m_front));
					if (longitudinalDir.DotProduct3(longitudinalDir) < 0.1f) {
						longitudinalDir = normal.CrossProduct(tireMatrix.m_up.CrossProduct(normal));
						dAssert(longitudinalDir.DotProduct3(longitudinalDir) > 0.1f);
					}
					longitudinalDir = longitudinalDir.Normalize();

					dVector contact (points[j][0], points[j][1], points[j][2], dFloat (1.0f));
					contact -= tireMatrix.m_up.Scale (dist);

					dVehicleCollidingNode* const collidingNode = chassisNode->FindCollideNode(this, bodyArray.m_array[i]);
					//dVehicleCollidingNode* const collidingNode = chassisNode->FindCollideNode(this, NULL);
					m_contactsJoints[contactCount].SetOwners (this, collidingNode);

					m_contactsJoints[contactCount].SetContact(collidingNode, contact, normal, longitudinalDir, penetration, friction, true);
					contactCount ++;
					dAssert(contactCount <= sizeof(m_contactsJoints) / sizeof(m_contactsJoints[0]));
if ((m_index == 0) && xxxx > 800) {
dTrace(("x(%f) p(%f %f %f) n(%f %f %f) ", penetrations[j], contact[0], contact[1], contact[2], normal[0], normal[1], normal[2]));
}
				}
			}
		}
	}
	
/*
	tireMatrix = GetHardpointMatrix(m_position * m_invSuspensionLength) * chassisMatrix;
	for (int i = 0; (i < bodyArray.m_count) && (contactCount < sizeof(m_contactsJoints) / sizeof(m_contactsJoints[0])); i++) {
		// calculate tire contact collision with rigid bodies
		dMatrix matrixB;
		dLong attributeA[2];
		dLong attributeB[2];

		NewtonBodyGetMatrix(bodyArray.m_array[i], &matrixB[0][0]);
		NewtonCollision* const otherShape = NewtonBodyGetCollision(bodyArray.m_array[i]);

		int count = NewtonCollisionCollide(world, 1, m_tireShape, &tireMatrix[0][0], otherShape, &matrixB[0][0],
										  &points[0][0], &normals[0][0], penetrations, attributeA, attributeB, 0);

		for (int j = 0; j < count; j ++) {
			dVector normal (normals[j][0], normals[j][1], normals[j][2], dFloat (0.0f));
			#ifdef USE_TIRE_SMALL_PATCH
			dFloat projection = normal.DotProduct3(tireMatrix.m_right);
			if (projection < D_TIRE_CONTACT_PATCH_CONE) {
			#else
			dFloat projection = dAbs (normal.DotProduct3(tireMatrix.m_front));
			if (projection > D_TIRE_CONTACT_PATCH_CONE) {
			#endif
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
	}

	// filter contact that are too close 
	for (int i = 0; i < contactCount - 1; i ++) {
		const dVector& n = m_contactsJoints[i].m_normal;
		for (int j = contactCount - 1; j > i; j --) {
			dFloat val = dAbs (n.DotProduct3(m_contactsJoints[j].m_normal));
			if (val > 0.996f) {
				m_contactsJoints[j] = m_contactsJoints[contactCount - 1];
				contactCount --;
			}
		}
	}
*/
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
	debugContext->SetColor(dVector(0.0f, 0.4f, 0.7f, 1.0f));
	dMatrix tireMatrix(m_bindingRotation.Transpose() * GetGlobalMatrix());
	NewtonCollisionForEachPolygonDo(m_tireShape, &tireMatrix[0][0], RenderDebugTire, debugContext);

	dVehicleMultiBody* const chassis = GetParent()->GetAsVehicleMultiBody();
	dAssert (chassis);

	// render tireState matrix
	dComplementaritySolver::dBodyState* const chassisBody = &chassis->GetProxyBody();
	dVector weight (chassis->GetGravity().Scale(chassisBody->GetMass()));
	dFloat scale (1.0f / dSqrt (weight.DotProduct3(weight)));
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

if ((m_index == 0) && xxxx > 800) {
dMatrix matrix (m_proxyBody.GetMatrix());
dFloat s = m_proxyBody.GetVelocity().DotProduct3(matrix.m_right);
dTrace (("v(%f)\n", s));
}

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
			dVector torque (r.CrossProduct(force));

			NewtonBodyAddForce(contact->m_collidingNode->m_body, &force[0]);
			NewtonBodyAddTorque(contact->m_collidingNode->m_body, &torque[0]);
		}
	}
}

void dVehicleTire::ApplyExternalForce()
{

if (m_index == 0)
xxxx ++;

if ((m_index == 0) && xxxx >= 4883) {
dTrace (("%d ", xxxx));
}

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