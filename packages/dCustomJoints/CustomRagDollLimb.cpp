/* Copyright (c) <2009> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/

#include "CustomJointLibraryStdAfx.h"

#include "CustomRagDollLimb.h"


dInitRtti(CustomRagDollLimb);



/*
"CustomRagDollLimb.h"::"CustomRagDollLimb.h" ()
	:CustomJoint()
{
	m_bonesCount = 0;

	m_joints[0] = NULL;
	m_bonesBodies[0] = NULL;
	m_bonesParents[0] = NULL;
}



"CustomRagDollLimb.h"::~"CustomRagDollLimb.h"(void)
{
}


void "CustomRagDollLimb.h"::GetInfo (NewtonJointRecord* const info) const
{

}

void "CustomRagDollLimb.h"::SubmitConstraints (dFloat timestep, int threadIndex)
{

}


const NewtonBody* "CustomRagDollLimb.h"::GetBone (int bodenIndex) const
{
	return m_bonesBodies[bodenIndex];
}

const NewtonBody* "CustomRagDollLimb.h"::GetParentBone (int bodenIndex) const
{
    return m_bonesParents[bodenIndex];
}

const CustomJoint *"CustomRagDollLimb.h"::GetJoint (int bodenIndex) const
{
	return m_joints[bodenIndex];
}

int "CustomRagDollLimb.h"::GetBoneCount () const
{
	return m_bonesCount;
}

void "CustomRagDollLimb.h"::SetBoneTwistLimits (int bodenIndex, dFloat minAngle, dFloat maxAngle)
{
	if (bodenIndex > 0) {
		m_joints[bodenIndex]->SetTwistAngle (minAngle, maxAngle);
	}

}


void "CustomRagDollLimb.h"::SetBoneConeLimits (int bodenIndex, dFloat angle)
{
	if (bodenIndex > 0) {
		m_joints[bodenIndex]->SetConeAngle (angle);
	}
}
*/



CustomRagDollLimb::CustomRagDollLimb (const dMatrix& pinAndPivotFrame, NewtonBody* const child, NewtonBody* const parent)
	:CustomLimitBallAndSocket(pinAndPivotFrame, child, parent)
{
}

void CustomRagDollLimb::SubmitConstraints (dFloat timestep, int threadIndex)
{
	CustomLimitBallAndSocket::SubmitConstraints (timestep, threadIndex);
}


