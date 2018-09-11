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
	,m_parent(parent)
{
	if (parent) {
		parent->m_children.Append(this);
	}
}

dVehicleNode::~dVehicleNode()
{
	for (dList<dVehicleNode*>::dListNode* child = m_children.GetFirst(); child; child = child->GetNext()) {
		delete child->GetInfo();
	}
}

