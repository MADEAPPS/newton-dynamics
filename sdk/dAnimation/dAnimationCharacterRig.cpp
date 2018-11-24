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
#include "dAnimationRigHinge.h"
#include "dAnimationCharacterRig.h"
#include "dAnimationCharacterRigManager.h"

dAnimationCharacterRig::dAnimationCharacterRig ()
	:dCustomControllerBase()
	,dAnimationRigJoint(NULL)
	,m_staticWorld(NULL)
	,m_solver()
{
	m_root = this;
	m_staticWorld.SetLoopNode(true);
}

dAnimationCharacterRig::~dAnimationCharacterRig ()
{
}

void dAnimationCharacterRig::Init(NewtonBody* const body)
{
	dCustomControllerBase::m_body = body;
	dAnimationRigJoint::Init(body);
}

void dAnimationCharacterRig::Debug(dCustomJoint::dDebugDisplay* const debugContext) const
{
	dAnimationRigJoint::Debug(debugContext);
}

void dAnimationCharacterRig::Finalize()
{
	m_solver.Finalize(this);
}

NewtonBody* dAnimationCharacterRig::GetNewtonBody() const 
{
	return dCustomControllerBase::GetBody();
}


void dAnimationCharacterRig::PreUpdate(dFloat timestep, int threadIndex)
{
	RigidBodyToStates();
	m_solver.Update(timestep);
	UpdateJointAcceleration();
}

void dAnimationCharacterRig::PostUpdate(dFloat timestep, int threadIndex)
{
	dAnimationCharacterRigManager* const manager = (dAnimationCharacterRigManager*)GetManager();
	UpdateLocalTransforms (manager);
}
