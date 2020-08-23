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


// NewtonCustomJoint.cpp: implementation of the NewtonCustomJoint class.
//
//////////////////////////////////////////////////////////////////////
#include "dStdafxVehicle.h"
#include "dPlayerIKNode.h"

dPlayerIKNode::dPlayerIKNode()
	:dVehicleNode(NULL)
	,m_bindMatrix(dGetIdentityMatrix())
	,m_shape(NULL)
{
}

dPlayerIKNode::dPlayerIKNode(dVehicleNode* const parent, void* const userData, const dMatrix& bindMatrix, NewtonCollision* const shape)
	:dVehicleNode(parent)
	,m_bindMatrix(bindMatrix)
{
	SetUserData(userData);
	m_shape = NewtonCollisionCreateInstance(shape);
}

dPlayerIKNode::~dPlayerIKNode()
{
	if (m_shape) {
		NewtonDestroyCollision(m_shape);
	}
}


void dPlayerIKNode::RenderDebugSkeleton(void* userData, int vertexCount, const dFloat* const faceVertec, int id)
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


const void dPlayerIKNode::Debug(dCustomJoint::dDebugDisplay* const debugContext) const
{
	dVehicleNode::Debug(debugContext);
	if (m_shape) {
		debugContext->SetColor(dVector(0.0f, 0.4f, 0.7f, 1.0f));
		//dMatrix tireMatrix(m_bindingRotation.Transpose() * GetGlobalMatrix());
		dMatrix tireMatrix(dGetIdentityMatrix());
		NewtonCollisionForEachPolygonDo(m_shape, &tireMatrix[0][0], RenderDebugSkeleton, debugContext);
	}
}