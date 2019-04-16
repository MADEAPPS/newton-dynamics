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
#include "dAnimationRagdollRoot.h"

dAnimationRagdollRoot::dAnimationRagdollRoot(NewtonBody* const body, const dMatrix& bindMarix)
	:dAnimationJointRoot(body, bindMarix)
{
}

dAnimationRagdollRoot::~dAnimationRagdollRoot()
{
}

void dAnimationRagdollRoot::PreUpdate(dFloat timestep)
{
	dAnimationJointRoot::RigidBodyToStates();
	m_proxyBody.SetForce(dVector(0.0f));
	m_proxyBody.SetTorque(dVector(0.0f));

	NewtonBodySetSleepState(GetBody(), 0);
//	m_hipEffector->SetTarget();
	//if (m_animationTree) {
	//	m_animationTree->Update(timestep);
	//}
	m_solver.Update(timestep);
	UpdateJointAcceleration();
}

