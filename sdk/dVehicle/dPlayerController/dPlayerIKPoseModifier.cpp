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
#include "dPlayerController.h"
#include "dPlayerIKPoseModifier.h"

dPlayerIKPoseModifier::dPlayerIKPoseModifier(dPlayerController* const controller, dAnimationBlendTreeNode* const input)
	:dPlayerIKNode()
	,dAnimationBlendTreeNode(input)
	,m_controller(controller)
{
}

dPlayerIKPoseModifier::~dPlayerIKPoseModifier()
{
}

void* dPlayerIKPoseModifier::operator new (size_t size)
{
	return dPlayerIKNode::malloc(size);
}

void dPlayerIKPoseModifier::operator delete (void* ptr)
{
	dPlayerIKNode::free(ptr);
}

void dPlayerIKPoseModifier::Init (void* const userData, const dMatrix& bindMatrix, NewtonCollision* const shape)
{
	m_bindMatrix = bindMatrix;
	SetUserData(userData);
	m_shape = NewtonCollisionCreateInstance(shape);
}

void dPlayerIKPoseModifier::Finalize()
{
	dPlayerIKNode* array[256];
	
	dFloat volume = 0.0f;
	int nodeCount = GetNodeArray (array);
	for (int i = 0; i < nodeCount; i ++) {
		const dPlayerIKNode* const node = array[i];
		volume += NewtonConvexCollisionCalculateVolume (node->m_shape);
	}
	dFloat density = m_controller->GetMass() / volume;
}

const void dPlayerIKPoseModifier::Debug(dCustomJoint::dDebugDisplay* const debugContext) const
{
	dPlayerIKNode::Debug(debugContext);
}

int dPlayerIKPoseModifier::GetNodeArray (dPlayerIKNode** const array) const
{
	dPlayerIKNode* stackMem[32];
	stackMem[0] = (dPlayerIKNode*)this;
	int stack = 1;

	int count = 0;
	while (stack) {
		stack --;
		dPlayerIKNode* const node = stackMem[stack];
		array[count] = node;
		count ++;

		for (dVehicleNodeChildrenList::dListNode* childNode = node->GetChildrenList().GetFirst(); childNode; childNode = childNode->GetNext()) {
			dPlayerIKNode* const child = childNode->GetInfo()->GetAsPlayerIKNode();
			dAssert (child);
			stackMem[stack] = child;
			stack ++;
		}
	}
	return count;
}

void dPlayerIKPoseModifier::Evaluate(dAnimationPose& output, dFloat timestep)
{
	m_input->Evaluate(output, timestep);
}
