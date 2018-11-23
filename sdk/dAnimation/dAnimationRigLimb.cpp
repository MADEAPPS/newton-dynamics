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

#include "dAnimationStdAfx.h"
#include "dAnimationRigLimb.h"
#include "dAnimationRigEffector.h"
#include "dAnimationCharacterRigManager.h"


dAnimationRigLimb::dAnimationRigLimb(dAnimationRigJoint* const parent, NewtonBody* const body)
	:dAnimationRigJoint(parent)
	,m_newtonBody(body)
	,m_effector(NULL)
{
	Init (body);
}

dAnimationRigLimb::~dAnimationRigLimb()
{
	if (m_effector) {
		delete m_effector;
	}
}

NewtonBody* dAnimationRigLimb::GetNewtonBody() const 
{ 
	return m_newtonBody; 
}


int dAnimationRigLimb::GetKinematicLoops(dAnimationKinematicLoopJoint** const jointArray)
{
	int count = 0;
	if (m_effector) {
		jointArray[count] = m_effector;
		count ++;
	}
	return dAnimationRigJoint::GetKinematicLoops(&jointArray[count]) + count;
}
