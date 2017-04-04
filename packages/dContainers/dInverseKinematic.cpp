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

#include "dContainersStdAfx.h"
#include "dInverseKinematic.h"


dInverseKinematic::dIKNode::dIKNode()
	:dContainersAlloc()
	,m_matrix(dGetIdentityMatrix())
	,m_pin(0.0f)
	,m_parent(NULL)
	,m_child(NULL)
	,m_sibling(NULL)
{
}

dInverseKinematic::dIKNode::dIKNode(dIKNode* const parent)
	:dContainersAlloc()
	,m_matrix(dGetIdentityMatrix())
	,m_pin(0.0f)
	,m_parent(parent)
	,m_child(NULL)
	,m_sibling(NULL)
{
	if (m_parent) {
		m_sibling = m_parent->m_child;
		m_parent->m_child = this;
	}
}

dInverseKinematic::dIKNode::~dIKNode()
{
	dIKNode* next;
	for (dIKNode* ptr = m_child; ptr; ptr = next) {
		next = ptr->m_sibling;
		delete ptr;
	}
}


dInverseKinematic::dInverseKinematic()
	:dContainersAlloc()
	,m_ikRoot(NULL)
{
}

dInverseKinematic::~dInverseKinematic()
{
	if (m_ikRoot) {
		delete m_ikRoot;
	}
}
