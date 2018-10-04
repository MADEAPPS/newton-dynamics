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
	,m_tireOmega(0.0f)
	,m_tireAngle(0.0f)
	,m_steeringAngle(0.0f)
{
	dVehicleSingleBody* const chassisNode = (dVehicleSingleBody*) m_parent;
	dVehicleChassis* const chassis = chassisNode->GetChassis();
	NewtonBody* const newtonBody = chassis->GetBody();
	NewtonWorld* const world = NewtonBodyGetWorld(newtonBody);

	m_tireShape = NewtonCreateChamferCylinder(world, 0.5f, 1.0f, 0, NULL);
	NewtonCollisionSetScale(m_tireShape, m_info.m_width, m_info.m_radio, m_info.m_radio);

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

	dComplentaritySolver::dBodyState* const chassisBody = GetBody();
	chassisBody->SetMass(m_info.m_mass);
	chassisBody->SetInertia(inertia.m_x, inertia.m_y, inertia.m_z);
	chassisBody->UpdateInertia();

	m_joint.Init (&m_body, m_parent->GetBody());

	m_tireOmega = -10.0f;
}

dVehicleVirtualTire::~dVehicleVirtualTire()
{
	NewtonDestroyCollision(m_tireShape);
}

NewtonCollision* dVehicleVirtualTire::GetCollisionShape() const
{
	return m_tireShape;
}

dComplentaritySolver::dBilateralJoint* dVehicleVirtualTire::GetJoint()
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

dMatrix dVehicleVirtualTire::GetLocalMatrix () const
{
	return m_bindingRotation * dPitchMatrix(m_tireAngle) * dYawMatrix(m_steeringAngle) * m_matrix;
}

dMatrix dVehicleVirtualTire::GetGlobalMatrix () const
{
	dVehicleSingleBody* const chassisNode = (dVehicleSingleBody*)m_parent->GetAsVehicle();
	dAssert (chassisNode);
	dVehicleChassis* const chassis = chassisNode->GetChassis();
	NewtonBody* const chassisBody = chassis->GetBody();

	dMatrix chassisMatrix;
	NewtonBodyGetMatrix(chassisBody, &chassisMatrix[0][0]);
	return GetLocalMatrix () * chassisMatrix;
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
	dVehicleSingleBody* const chassisNode = (dVehicleSingleBody*)m_parent;
	dVehicleChassis* const chassis = chassisNode->GetChassis();
	NewtonBody* const newtonBody = chassis->GetBody();


	dVehicleTireInterface::InitRigiBody(timestep);

m_tireAngle = dMod(m_tireAngle + m_tireOmega * timestep, 2.0f * dPi);
}
