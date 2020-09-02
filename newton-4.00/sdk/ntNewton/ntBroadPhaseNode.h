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

#include "ntStdafx.h"

class ntBody;
class ntBroadPhaseBodyNode;
class ntBroadPhaseTreeNode;
class ntBroadPhaseAggregate;

D_MSV_NEWTON_ALIGN_32
class ntBroadPhaseNode: public dClassAlloc
{
	public:
	ntBroadPhaseNode(ntBroadPhaseNode* const parent)
		:dClassAlloc()
		,m_minBox(dFloat32(-1.0e15f))
		,m_maxBox(dFloat32(1.0e15f))
		,m_parent(parent)
		,m_lock()
		,m_surfaceArea(dFloat32(1.0e20f))
	{
	}

	virtual ~ntBroadPhaseNode()
	{
	}

	D_NEWTON_API void SetAABB(const dVector& minBox, const dVector& maxBox);

	virtual ntBroadPhaseNode* GetAsBroadPhaseNode() { return this; }
	virtual ntBroadPhaseBodyNode* GetAsBroadPhaseBodyNode() { return nullptr; }
	virtual ntBroadPhaseTreeNode* GetAsBroadPhaseTreeNode() { return nullptr; }
	virtual ntBroadPhaseAggregate* GetAsBroadPhaseAggregate() { return nullptr; }

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
	virtual ntBody* GetBody() const
	{
		return nullptr;
	}

	virtual ntBroadPhaseNode* GetLeft() const
	{
		return nullptr;
	}

	virtual ntBroadPhaseNode* GetRight() const
	{
		return nullptr;
	}

	dVector m_minBox;
	dVector m_maxBox;
	ntBroadPhaseNode* m_parent;
	dSpinLock m_lock;
	dFloat32 m_surfaceArea;

	static dVector m_aabbQuantization;
	static dVector m_aabbInvQuantization;
} D_GCC_NEWTON_ALIGN_32 ;

D_MSV_NEWTON_ALIGN_32
class ntBroadPhaseBodyNode: public ntBroadPhaseNode
{
	public:
	D_NEWTON_API ntBroadPhaseBodyNode(ntBody* const body);
	D_NEWTON_API virtual ~ntBroadPhaseBodyNode();

	virtual ntBroadPhaseBodyNode* GetAsBroadPhaseBodyNode() { return this; }

	virtual ntBody* GetBody() const
	{
		return m_body;
	}

	ntBody* m_body;
	//dList<dBroadPhaseNode*>::dListNode* m_updateNode;
} D_GCC_NEWTON_ALIGN_32 ;

class ntBroadPhaseTreeNode: public ntBroadPhaseNode
{
	public:
	//dBroadPhaseTreeNode()
	//	:dBroadPhaseNode(nullptr)
	//	,m_left(nullptr)
	//	,m_right(nullptr)
	//	,m_fitnessNode(nullptr)
	//{
	//}
	
	D_NEWTON_API ntBroadPhaseTreeNode(ntBroadPhaseNode* const sibling, ntBroadPhaseNode* const myNode);
	D_NEWTON_API virtual ~ntBroadPhaseTreeNode();

	virtual ntBroadPhaseTreeNode* GetAsBroadPhaseTreeNode() { return this; }
	
	virtual ntBroadPhaseNode* GetLeft() const
	{
		return m_left;
	}
	
	virtual ntBroadPhaseNode* GetRight() const
	{
		return m_right;
	}

	ntBroadPhaseNode* m_left;
	ntBroadPhaseNode* m_right;
	dList<ntBroadPhaseTreeNode*, dContainersFreeListAlloc<ntBroadPhaseTreeNode*>>::dListNode* m_fitnessNode;
} D_GCC_NEWTON_ALIGN_32;


#endif
