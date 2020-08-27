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

class dBody;
class dBroadPhaseBodyNode;
class dBroadPhaseTreeNode;
class dBroadPhaseAggregate;

D_MSC_VECTOR_ALIGNMENT
class dBroadPhaseNode: public dClassAlloc
{
	public:
	dBroadPhaseNode(dBroadPhaseNode* const parent)
		:dClassAlloc()
		,m_minBox(dFloat32(-1.0e15f))
		,m_maxBox(dFloat32(1.0e15f))
		,m_parent(parent)
		,m_surfaceArea(dFloat32(1.0e20f))
		//,m_criticalSectionLock(0)
	{
	}

	virtual ~dBroadPhaseNode()
	{
	}

	D_NEWTON_API void SetAABB(const dVector& minBox, const dVector& maxBox);

	virtual dBroadPhaseNode* GetAsBroadPhaseNode() { return this; }
	virtual dBroadPhaseBodyNode* GetAsBroadPhaseBodyNode() { return nullptr; }
	virtual dBroadPhaseTreeNode* GetAsBroadPhaseTreeNode() { return nullptr; }
	virtual dBroadPhaseAggregate* GetAsBroadPhaseAggregate() { return nullptr; }

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

	virtual dBody* GetBody() const
	{
		return nullptr;
	}
*/
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
	//dInt32 m_criticalSectionLock;

	static dVector m_aabbQuantization;
	static dVector m_aabbInvQuantization;
} D_GCC_VECTOR_ALIGNMENT;

D_MSC_VECTOR_ALIGNMENT
class dBroadPhaseBodyNode: public dBroadPhaseNode
{
	public:
	D_NEWTON_API dBroadPhaseBodyNode(dBody* const body);
	D_NEWTON_API virtual ~dBroadPhaseBodyNode();

	virtual dBroadPhaseBodyNode* GetAsBroadPhaseBodyNode() { return this; }
/*
	virtual bool IsLeafNode() const
	{
		return true;
	}

	virtual dBody* GetBody() const
	{
		return m_body;
	}
*/
	dBody* m_body;
	//dList<dBroadPhaseNode*>::dListNode* m_updateNode;
} D_GCC_VECTOR_ALIGNMENT;

class dBroadPhaseTreeNode: public dBroadPhaseNode
{
	public:
	//dBroadPhaseTreeNode()
	//	:dBroadPhaseNode(nullptr)
	//	,m_left(nullptr)
	//	,m_right(nullptr)
	//	,m_fitnessNode(nullptr)
	//{
	//}
	
	D_NEWTON_API dBroadPhaseTreeNode(dBroadPhaseNode* const sibling, dBroadPhaseNode* const myNode);
	D_NEWTON_API virtual ~dBroadPhaseTreeNode();

	virtual dBroadPhaseTreeNode* GetAsBroadPhaseTreeNode() { return this; }
	
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
