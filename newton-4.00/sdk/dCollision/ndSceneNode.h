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

#ifndef __D_SCENE_NODE_H__
#define __D_SCENE_NODE_H__

#include "ndCollisionStdafx.h"

class ndBodyKinematic;
class ndSceneBodyNode;
class ndSceneTreeNode;
class ndSceneAggregate;

D_MSV_NEWTON_ALIGN_32
class ndSceneNode: public dClassAlloc
{
	public:
	ndSceneNode(ndSceneNode* const parent)
		:dClassAlloc()
		,m_minBox(dFloat32(-1.0e15f))
		,m_maxBox(dFloat32(1.0e15f))
		,m_parent(parent)
		,m_lock()
		,m_surfaceArea(dFloat32(1.0e20f))
	{
	}

	virtual ~ndSceneNode()
	{
	}

	void GetAABB(dVector& minBox, dVector& maxBox) const
	{
		minBox = m_minBox;
		maxBox = m_maxBox;
	}

	D_COLLISION_API void SetAABB(const dVector& minBox, const dVector& maxBox);

	virtual ndSceneNode* GetAsSceneNode() { return this; }
	virtual ndSceneBodyNode* GetAsSceneBodyNode() { return nullptr; }
	virtual ndSceneTreeNode* GetAsSceneTreeNode() { return nullptr; }
	virtual ndSceneAggregate* GetAsSceneAggregate() { return nullptr; }

/*
	virtual bool IsSegregatedRoot() const
	{
		return false;
	}

	virtual bool IsAggregate() const
	{
		return false;
	}
*/
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
	dSpinLock m_lock;
	dFloat32 m_surfaceArea;

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
	dList<ndSceneTreeNode*, dContainersFreeListAlloc<ndSceneTreeNode*>>::dListNode* m_fitnessNode;
} D_GCC_NEWTON_ALIGN_32;


#endif
