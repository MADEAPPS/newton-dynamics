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


void dPlayerIKPoseModifier::Evaluate(dAnimationPose& output, dFloat timestep)
{
	m_input->Evaluate(output, timestep);
}
