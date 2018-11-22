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
#include "dAnimationCharacterRig.h"
#include "dAnimationCharacterRigManager.h"

dAnimationCharacterRig::dAnimationCharacterRig ()
	:dCustomControllerBase()
	,dAnimationRigJoint(NULL, NULL)
{
}

dAnimationCharacterRig::~dAnimationCharacterRig ()
{
}

void dAnimationCharacterRig::Init(NewtonBody* const body)
{
	dAnimationRigJoint::m_body = body;
	dCustomControllerBase::m_body = body;
}

void dAnimationCharacterRig::Debug(dCustomJoint::dDebugDisplay* const debugContext) const
{

	dAnimationRigJoint::Debug(debugContext);
}

void dAnimationCharacterRig::PreUpdate(dFloat timestep, int threadIndex)
{
//	dAssert(0);
//	dVehicleManager* const manager = (dVehicleManager*)GetManager();
}

NewtonBody* dAnimationCharacterRig::GetNewtonBody() const 
{
	return dCustomControllerBase::GetBody();
}

void dAnimationCharacterRig::PostUpdate(dFloat timestep, int threadIndex)
{
//	m_vehicle->RigidBodyToStates();

//	dAssert(m_bones.GetCount() == 1);

	dAnimationCharacterRigManager* const manager = (dAnimationCharacterRigManager*)GetManager();

	UpdateLocalTransforms (manager);
}
