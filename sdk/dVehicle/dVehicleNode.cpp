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
#include "dVehicleNode.h"

dVehicleNode::dVehicleNode(dVehicleNode* const parent)
	:dContainersAlloc()
	,m_userData(NULL)
	,m_parent(parent)
	,m_body()
	,m_articulation(NULL)
	,m_solverIndex(-1)
{
	if (parent) {
		parent->m_children.Append(this);
	}
}

dVehicleNode::~dVehicleNode()
{
	if (m_articulation) {
		delete m_articulation;
	}
	for (dList<dVehicleNode*>::dListNode* child = m_children.GetFirst(); child; child = child->GetNext()) {
		delete child->GetInfo();
	}
}

void* dVehicleNode::GetUserData()
{
	return m_userData;
}

void dVehicleNode::SetUserData(void* const userData)
{
	m_userData = userData;
}


void dVehicleNode::Debug(dCustomJoint::dDebugDisplay* const debugContext) const
{
	for (dList<dVehicleNode*>::dListNode* child = m_children.GetFirst(); child; child = child->GetNext()) {
		child->GetInfo()->Debug(debugContext);
	}
}
