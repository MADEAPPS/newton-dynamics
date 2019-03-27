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
#include "dAnimationJointRoot.h"
#include "dAnimationModelManager.h"

dAnimationJointRoot::dAnimationJointRoot(NewtonBody* const body, const dMatrix& bindMarix)
	:dAnimationJoint(body, bindMarix, NULL)
	,m_solver()
	,m_staticBody()
	,m_manager(NULL)
	,m_managerNode(NULL)
	,m_calculateLocalTransform(true)
{
}

dAnimationJointRoot::~dAnimationJointRoot()
{
	if (m_manager) {
		dAssert(m_managerNode);
		m_manager->RemoveModel(this);
	}
}

void dAnimationJointRoot::PreUpdate(dAnimationModelManager* const manager, dFloat timestep) const
{
	dAssert(manager);
	dAnimationJoint::PreUpdate(manager, timestep);
}