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


class dInverseKinematic::dIKNode: public dContainersAlloc
{
	public:
	dIKNode(dIKNode* const parent)
		:dContainersAlloc()
		,m_parent(parent)
		,m_child(NULL)
		,m_sibling(NULL)
	{
		if (m_parent) {
			m_sibling = m_parent->m_child;
			m_parent->m_child = this;
		}
	}

	virtual ~dIKNode()
	{
		dIKNode* next;
		for (dIKNode* ptr = m_child; ptr; ptr = next) {
			next = ptr->m_sibling;
			delete ptr;
		}
	}

	virtual void SetPin(const dVector& pin) = 0;
	virtual void SetPivot(const dVector& pivot) = 0;

	protected:
	dIKNode* m_parent;
	dIKNode* m_child;
	dIKNode* m_sibling;
};


class dIKNodeRoot: public dInverseKinematic::dIKNode
{
	public:
	dIKNodeRoot()
		:dIKNode(NULL)
		,m_matrix(dGetIdentityMatrix())
	{
	}

	virtual void SetPin(const dVector& )
	{
	}

	virtual void SetPivot(const dVector& pivot)
	{
		m_matrix.m_posit = pivot;
		m_matrix.m_posit.m_w = 1.0f;
	}

	dMatrix m_matrix;
};


class dIKNodeLink: public dInverseKinematic::dIKNode
{
	public:
	dIKNodeLink(dIKNode* const parent)
		:dIKNode(parent)
		,m_axis (1.0f, 0.0f, 0.0f, 0.0f)
		,m_pivot (0.0f)
		,m_jointAngle(0.0f)
	{
	}

	virtual void SetPin(const dVector& pin)
	{
		dFloat mag = pin.DotProduct3(pin);
		if (mag > 1.0e-8f) {
			m_axis = pin.Scale(1.0f / dSqrt(mag));
		} else {
			m_axis = dVector(1.0f, 0.0f, 0.0f, 0.0f);
		}
		m_axis.m_w = 0.0f;
	}


	virtual void SetPivot(const dVector& pivot)
	{
		m_pivot = pivot;
		m_pivot.m_w = 1.0f;
	}

	dVector m_axis;
	dVector m_pivot;
	dFloat m_jointAngle;
};


dInverseKinematic::dInverseKinematic()
	:dContainersAlloc()
	,m_root(new dIKNodeRoot())
{
}

dInverseKinematic::~dInverseKinematic()
{
	delete m_root;
}

dInverseKinematic::dIKNode* dInverseKinematic::GetRoot() const
{
	return m_root;
}

dInverseKinematic::dIKNode* dInverseKinematic::AttachLink(dIKNode* const parent) const
{
	return new dIKNodeLink(parent);
}

void dInverseKinematic::SetPivot(dIKNode* const node, const dVector& pivot) const
{
	node->SetPivot(pivot);
}

void dInverseKinematic::SetAxis(dIKNode* const node, const dVector& pin) const
{
	node->SetPin(pin);
}
