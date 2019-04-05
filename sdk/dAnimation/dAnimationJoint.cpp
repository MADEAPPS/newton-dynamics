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
#include "dAnimationJoint.h"
#include "dAnimationModelManager.h"

dAnimationJoint::dAnimationJoint(NewtonBody* const body, const dMatrix& bindMarix, dAnimationJoint* const parent)
	:dCustomAlloc()
	,m_proxyBody()
	,m_bindMatrix(bindMarix)
	,m_userData(NULL)
	,m_body(body)
	,m_joint(NULL)
	,m_parent(parent)
	,m_proxyJoint(NULL)
	,m_children()
{
	if (m_parent) {
		m_parent->m_children.Append(this);
	}
	m_proxyBody.m_owner = this;
}

dAnimationJoint::~dAnimationJoint()
{
	for (dAnimationJointChildren::dListNode* node = m_children.GetFirst(); node; node = node->GetNext()) {
		delete node->GetInfo();
	}
}

void dAnimationJoint::RigidBodyToStates()
{
	if (m_body) {
		dVector vector;
		dMatrix matrix;

		// get data from engine rigid body and copied to the vehicle chassis body
		//NewtonBody* const newtonBody = m_chassis->GetBody();
		NewtonBodyGetMatrix(m_body, &matrix[0][0]);
		m_proxyBody.SetMatrix(matrix);

		NewtonBodyGetVelocity(m_body, &vector[0]);
		m_proxyBody.SetVeloc(vector);

		NewtonBodyGetOmega(m_body, &vector[0]);
		m_proxyBody.SetOmega(vector);

		NewtonBodyGetForce(m_body, &vector[0]);
		m_proxyBody.SetForce(vector);

		NewtonBodyGetTorque(m_body, &vector[0]);
		m_proxyBody.SetTorque(vector);

		m_proxyBody.UpdateInertia();
	}

	for (dAnimationJointChildren::dListNode* node = m_children.GetFirst(); node; node = node->GetNext()) {
		node->GetInfo()->RigidBodyToStates();
	}
}

void dAnimationJoint::ApplyExternalForce(dFloat timestep)
{
	for (dAnimationJointChildren::dListNode* node = m_children.GetFirst(); node; node = node->GetNext()) {
		node->GetInfo()->ApplyExternalForce(timestep);
	}
}
