/* Copyright (c) <2003-2021> <Julio Jerez, Newton Game Dynamics>
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

#ifndef __D_SCENE_NODE_H__
#define __D_SCENE_NODE_H__

#include "ndCollisionStdafx.h"

class ndBodyKinematic;
class ndSceneBodyNode;
class ndSceneTreeNode;

D_MSV_NEWTON_ALIGN_32
class ndSceneNode: public dClassAlloc
{
	public:
	ndSceneNode(ndSceneNode* const parent)
		:dClassAlloc()
		,m_minBox(dFloat32(-1.0e15f))
		,m_maxBox(dFloat32( 1.0e15f))
		,m_parent(parent)
		,m_surfaceArea(dFloat32(1.0e20f))
		,m_lock()
	{
	}

	virtual ~ndSceneNode()
	{
	}

	void GetAABB(dVector& minBox, dVector& maxBox) const;
	void SetAABB(const dVector& minBox, const dVector& maxBox);

	virtual ndSceneNode* GetAsSceneNode() { return this; }
	virtual ndSceneBodyNode* GetAsSceneBodyNode() { return nullptr; }
	virtual ndSceneTreeNode* GetAsSceneTreeNode() { return nullptr; }

	virtual ndBodyKinematic* GetBody() const
	{
		return nullptr;
	}

	virtual ndSceneNode* GetLeft() const
	{
		return nullptr;
	}

	virtual ndSceneNode* GetRight() const
	{
		return nullptr;
	}

	dVector m_minBox;
	dVector m_maxBox;
	ndSceneNode* m_parent;
	dFloat32 m_surfaceArea;
	dSpinLock m_lock;

	static dVector m_aabbQuantization;
	static dVector m_aabbInvQuantization;
} D_GCC_NEWTON_ALIGN_32 ;

D_MSV_NEWTON_ALIGN_32
class ndSceneBodyNode: public ndSceneNode
{
	public:
	D_COLLISION_API ndSceneBodyNode(ndBodyKinematic* const body);
	D_COLLISION_API virtual ~ndSceneBodyNode();

	virtual ndSceneBodyNode* GetAsSceneBodyNode() { return this; }

	virtual ndBodyKinematic* GetBody() const
	{
		return m_body;
	}

	ndBodyKinematic* m_body;
} D_GCC_NEWTON_ALIGN_32 ;

class ndSceneTreeNode: public ndSceneNode
{
	public:
	D_COLLISION_API ndSceneTreeNode(ndSceneNode* const sibling, ndSceneNode* const myNode);
	D_COLLISION_API virtual ~ndSceneTreeNode();

	virtual ndSceneTreeNode* GetAsSceneTreeNode() { return this; }
	
	virtual ndSceneNode* GetLeft() const
	{
		return m_left;
	}
	
	virtual ndSceneNode* GetRight() const
	{
		return m_right;
	}

	ndSceneNode* m_left;
	ndSceneNode* m_right;
	dList<ndSceneTreeNode*, dContainersFreeListAlloc<ndSceneTreeNode*>>::dNode* m_fitnessNode;
} D_GCC_NEWTON_ALIGN_32;


inline void ndSceneNode::GetAABB(dVector& minBox, dVector& maxBox) const
{
	minBox = m_minBox;
	maxBox = m_maxBox;
}

inline void ndSceneNode::SetAABB(const dVector& minBox, const dVector& maxBox)
{
	dAssert(minBox.m_x <= maxBox.m_x);
	dAssert(minBox.m_y <= maxBox.m_y);
	dAssert(minBox.m_z <= maxBox.m_z);

	const dVector p0(minBox * m_aabbQuantization);
	const dVector p1(maxBox * m_aabbQuantization + dVector::m_one);

	m_minBox = p0.Floor() * m_aabbInvQuantization;
	m_maxBox = p1.Floor() * m_aabbInvQuantization;

	dAssert(m_minBox.m_w == dFloat32(0.0f));
	dAssert(m_maxBox.m_w == dFloat32(0.0f));

	const dVector size(m_maxBox - m_minBox);
	m_surfaceArea = size.DotProduct(size.ShiftTripleRight()).GetScalar();
}
#endif
