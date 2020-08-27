/* Copyright (c) <2003-2019> <Julio Jerez, Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 
* 3. This notice may not be removed or altered from any source distribution.
*/

#ifndef __D_BROADPHASE_NODE_H__
#define __D_BROADPHASE_NODE_H__

#include "dNewtonStdafx.h"

D_MSC_VECTOR_ALIGNMENT
class dBroadPhaseNode
{
	public:
	dBroadPhaseNode(dBroadPhaseNode* const parent)
		//:m_minBox(dFloat32(-1.0e15f))
		//,m_maxBox(dFloat32(1.0e15f))
		//,m_parent(parent)
		//,m_surfaceArea(dFloat32(1.0e20f))
		//,m_criticalSectionLock(0)
	{
	}

	virtual ~dBroadPhaseNode()
	{
	}

/*
	virtual bool IsSegregatedRoot() const
	{
		return false;
	}

	virtual bool IsLeafNode() const
	{
		return false;
	}

	virtual bool IsAggregate() const
	{
		return false;
	}

	void SetAABB(const dVector& minBox, const dVector& maxBox)
	{
		dAssert(minBox.m_x <= maxBox.m_x);
		dAssert(minBox.m_y <= maxBox.m_y);
		dAssert(minBox.m_z <= maxBox.m_z);

		dVector p0(minBox * m_broadPhaseScale);
		dVector p1(maxBox * m_broadPhaseScale + dVector::m_one);

		m_minBox = p0.Floor() * m_broadInvPhaseScale;
		m_maxBox = p1.Floor() * m_broadInvPhaseScale;

		dAssert(m_minBox.m_w == dFloat32(0.0f));
		dAssert(m_maxBox.m_w == dFloat32(0.0f));

		dVector side0(m_maxBox - m_minBox);
		m_surfaceArea = side0.DotProduct(side0.ShiftTripleRight()).m_x;
	}

	virtual dBody* GetBody() const
	{
		return nullptr;
	}

	virtual dBroadPhaseNode* GetLeft() const
	{
		return nullptr;
	}

	virtual dBroadPhaseNode* GetRight() const
	{
		return nullptr;
	}

	dVector m_minBox;
	dVector m_maxBox;
	dBroadPhaseNode* m_parent;
	dFloat32 m_surfaceArea;
	dInt32 m_criticalSectionLock;

	static dVector m_broadPhaseScale;
	static dVector m_broadInvPhaseScale;
*/
} D_GCC_VECTOR_ALIGNMENT;

D_MSC_VECTOR_ALIGNMENT
class dBroadPhaseBodyNode: public dBroadPhaseNode
{
	public:
	dBroadPhaseBodyNode(dBody* const body)
		:dBroadPhaseNode(nullptr)
		//,m_body(body)
		//,m_updateNode(nullptr)
	{
		//SetAABB(body->m_minAABB, body->m_maxAABB);
		//m_body->SetBroadPhase(this);
	}

	virtual ~dBroadPhaseBodyNode()
	{
		//m_body->SetBroadPhase(nullptr);
	}
/*
	virtual bool IsLeafNode() const
	{
		return true;
	}

	virtual dBody* GetBody() const
	{
		return m_body;
	}

	dBody* m_body;
	dList<dBroadPhaseNode*>::dListNode* m_updateNode;
*/
}  D_GCC_VECTOR_ALIGNMENT;

#if 0
class dBroadPhaseTreeNode: public dBroadPhaseNode
{
	public:
	dBroadPhaseTreeNode()
		:dBroadPhaseNode(nullptr)
		,m_left(nullptr)
		,m_right(nullptr)
		,m_fitnessNode(nullptr)
	{
	}

	dBroadPhaseTreeNode(dBroadPhaseNode* const sibling, dBroadPhaseNode* const myNode)
		:dBroadPhaseNode(sibling->m_parent)
		,m_left(sibling)
		,m_right(myNode)
		,m_fitnessNode(nullptr)
	{
		if (m_parent) {
			dBroadPhaseTreeNode* const myParent = (dBroadPhaseTreeNode*)m_parent;
			if (myParent->m_left == sibling) {
				myParent->m_left = this;
			} else {
				dAssert(myParent->m_right == sibling);
				myParent->m_right = this;
			}
		}

		sibling->m_parent = this;
		myNode->m_parent = this;

		dBroadPhaseNode* const left = m_left;
		dBroadPhaseNode* const right = m_right;

		m_minBox = left->m_minBox.GetMin(right->m_minBox);
		m_maxBox = left->m_maxBox.GetMax(right->m_maxBox);
		dVector side0(m_maxBox - m_minBox);
		m_surfaceArea = side0.DotProduct(side0.ShiftTripleRight()).m_x;
	}

	virtual ~dBroadPhaseTreeNode()
	{
		if (m_left) {
			delete m_left;
		}
		if (m_right) {
			delete m_right;
		}
	}
	
	virtual dBroadPhaseNode* GetLeft() const
	{
		return m_left;
	}

	virtual dBroadPhaseNode* GetRight() const
	{
		return m_right;
	}

	dBroadPhaseNode* m_left;
	dBroadPhaseNode* m_right;
	dList<dBroadPhaseTreeNode*>::dListNode* m_fitnessNode;
} D_GCC_VECTOR_ALIGNMENT;
#endif

#endif
