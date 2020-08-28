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

#include "dNewtonStdafx.h"
#include "dBody.h"
#include "dBroadPhaseNode.h"

#define D_AABB_QUANTIZATION		dFloat32 (8.0f)
#define D_AABB_INV_QUANTIZATION	(dFloat32 (1.0f) / D_AABB_QUANTIZATION)

dVector dBroadPhaseNode::m_aabbQuantization(D_AABB_QUANTIZATION, D_AABB_QUANTIZATION, D_AABB_QUANTIZATION, dFloat32 (0.0f));
dVector dBroadPhaseNode::m_aabbInvQuantization(D_AABB_INV_QUANTIZATION, D_AABB_INV_QUANTIZATION, D_AABB_INV_QUANTIZATION, dFloat32(0.0f));

dBroadPhaseBodyNode::dBroadPhaseBodyNode(dBody* const body)
	:dBroadPhaseNode(nullptr)
	,m_body(body)
	//,m_updateNode(nullptr)
{
	SetAABB(body->m_minAABB, body->m_maxAABB);
	m_body->SetBroadPhaseNode(this);
}

dBroadPhaseBodyNode::~dBroadPhaseBodyNode()
{
	m_body->SetBroadPhaseNode(nullptr);
}

void dBroadPhaseNode::SetAABB(const dVector& minBox, const dVector& maxBox)
{
	dAssert(minBox.m_x <= maxBox.m_x);
	dAssert(minBox.m_y <= maxBox.m_y);
	dAssert(minBox.m_z <= maxBox.m_z);

	dVector p0(minBox * m_aabbQuantization);
	dVector p1(maxBox * m_aabbQuantization + dVector::m_one);

	m_minBox = p0.Floor() * m_aabbInvQuantization;
	m_maxBox = p1.Floor() * m_aabbInvQuantization;

	dAssert(m_minBox.m_w == dFloat32(0.0f));
	dAssert(m_maxBox.m_w == dFloat32(0.0f));

	dVector size(m_maxBox - m_minBox);
	m_surfaceArea = size.DotProduct(size.ShiftTripleRight()).GetScalar();
}

dBroadPhaseTreeNode::dBroadPhaseTreeNode(dBroadPhaseNode* const sibling, dBroadPhaseNode* const myNode)
	:dBroadPhaseNode(sibling->m_parent)
	,m_left(sibling)
	,m_right(myNode)
	,m_fitnessNode(nullptr)
{
	if (m_parent) 
	{
		dBroadPhaseTreeNode* const myParent = (dBroadPhaseTreeNode*)m_parent;
		if (myParent->m_left == sibling) 
		{
			myParent->m_left = this;
		} 
		else 
		{
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

dBroadPhaseTreeNode::~dBroadPhaseTreeNode()
{
	if (m_left) 
	{
		delete m_left;
	}
	if (m_right) 
	{
		delete m_right;
	}
}

