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

#include "ndCollisionStdafx.h"

class ntBodyKinematic;
class ntSceneBodyNode;
class ntSceneTreeNode;
class ntSceneAggregate;

D_MSV_NEWTON_ALIGN_32
class ntSceneNode: public dClassAlloc
{
	public:
	ntSceneNode(ntSceneNode* const parent)
		:dClassAlloc()
		,m_minBox(dFloat32(-1.0e15f))
		,m_maxBox(dFloat32(1.0e15f))
		,m_parent(parent)
		,m_lock()
		,m_surfaceArea(dFloat32(1.0e20f))
	{
	}

	virtual ~ntSceneNode()
	{
	}

	ND_COLLISION_API void SetAABB(const dVector& minBox, const dVector& maxBox);

	virtual ntSceneNode* GetAsBroadPhaseNode() { return this; }
	virtual ntSceneBodyNode* GetAsBroadPhaseBodyNode() { return nullptr; }
	virtual ntSceneTreeNode* GetAsBroadPhaseTreeNode() { return nullptr; }
	virtual ntSceneAggregate* GetAsBroadPhaseAggregate() { return nullptr; }

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
	virtual ntBodyKinematic* GetBody() const
	{
		return nullptr;
	}

	virtual ntSceneNode* GetLeft() const
	{
		return nullptr;
	}

	virtual ntSceneNode* GetRight() const
	{
		return nullptr;
	}

	dVector m_minBox;
	dVector m_maxBox;
	ntSceneNode* m_parent;
	dSpinLock m_lock;
	dFloat32 m_surfaceArea;

	static dVector m_aabbQuantization;
	static dVector m_aabbInvQuantization;
} D_GCC_NEWTON_ALIGN_32 ;

D_MSV_NEWTON_ALIGN_32
class ntSceneBodyNode: public ntSceneNode
{
	public:
	ND_COLLISION_API ntSceneBodyNode(ntBodyKinematic* const body);
	ND_COLLISION_API virtual ~ntSceneBodyNode();

	virtual ntSceneBodyNode* GetAsBroadPhaseBodyNode() { return this; }

	virtual ntBodyKinematic* GetBody() const
	{
		return m_body;
	}

	ntBodyKinematic* m_body;
	//dList<dBroadPhaseNode*>::dListNode* m_updateNode;
} D_GCC_NEWTON_ALIGN_32 ;

class ntSceneTreeNode: public ntSceneNode
{
	public:
	//ntScenereeNode()
	//	:ntSceneNode(nullptr)
	//	,m_left(nullptr)
	//	,m_right(nullptr)
	//	,m_fitnessNode(nullptr)
	//{
	//}
	
	ND_COLLISION_API ntSceneTreeNode(ntSceneNode* const sibling, ntSceneNode* const myNode);
	ND_COLLISION_API virtual ~ntSceneTreeNode();

	virtual ntSceneTreeNode* GetAsBroadPhaseTreeNode() { return this; }
	
	virtual ntSceneNode* GetLeft() const
	{
		return m_left;
	}
	
	virtual ntSceneNode* GetRight() const
	{
		return m_right;
	}

	ntSceneNode* m_left;
	ntSceneNode* m_right;
	dList<ntSceneTreeNode*, dContainersFreeListAlloc<ntSceneTreeNode*>>::dListNode* m_fitnessNode;
} D_GCC_NEWTON_ALIGN_32;


#endif
