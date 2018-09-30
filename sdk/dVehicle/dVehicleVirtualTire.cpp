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


dVehicleVirtualTire::dVehicleVirtualTire(dVehicleNode* const parent, const dVector& locationInGlobalSpace, const dTireInfo& info)
	:dVehicleTireInterface(parent, locationInGlobalSpace, info)
	,m_info(info)
{
	dVehicleSingleBody* const chassisNode = (dVehicleSingleBody*) m_parent;
	dVehicleChassis* const chassis = chassisNode->GetChassis();
	NewtonBody* const chassisBody = chassis->GetBody();
	NewtonWorld* const world = NewtonBodyGetWorld(chassisBody);

	m_tireShape = NewtonCreateChamferCylinder(world, 0.5f, 1.0f, 0, NULL);
	NewtonCollisionSetScale(m_tireShape, m_info.m_width, m_info.m_radio, m_info.m_radio);

	dMatrix chassisMatrix;
	NewtonBodyGetMatrix(chassisBody, &chassisMatrix[0][0]);

	dMatrix alignMatrix(dGetIdentityMatrix());
	alignMatrix.m_front = dVector (0.0f, 0.0f, 1.0f, 0.0f);
	alignMatrix.m_up = dVector (0.0f, 1.0f, 0.0f, 0.0f);
	alignMatrix.m_right = alignMatrix.m_front.CrossProduct(alignMatrix.m_up);

	m_matrix = alignMatrix * chassis->m_localFrame;
	m_matrix.m_posit = chassis->m_localFrame.UntransformVector(chassisMatrix.UntransformVector(locationInGlobalSpace));
}

dVehicleVirtualTire::~dVehicleVirtualTire()
{
	NewtonDestroyCollision(m_tireShape);
}

NewtonCollision* dVehicleVirtualTire::GetCollisionShape() const
{
	return m_tireShape;
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
	return m_matrix;
}

dMatrix dVehicleVirtualTire::GetGlobalMatrix () const
{
	dVehicleSingleBody* const chassisNode = (dVehicleSingleBody*)m_parent->GetAsVehicle();
	dAssert (chassisNode);
	dVehicleChassis* const chassis = chassisNode->GetChassis();
	NewtonBody* const chassisBody = chassis->GetBody();

	dMatrix chassisMatrix;
	NewtonBodyGetMatrix(chassisBody, &chassisMatrix[0][0]);
	return m_matrix * chassisMatrix;
}

void dVehicleVirtualTire::Debug(dCustomJoint::dDebugDisplay* const debugContext) const
{
	dVehicleTireInterface::Debug(debugContext);

	dMatrix trieMatrix (GetGlobalMatrix ());
	NewtonCollisionForEachPolygonDo(m_tireShape, &trieMatrix[0][0], RenderDebugTire, debugContext);
}
