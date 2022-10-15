/* Copyright (c) <2003-2022> <Julio Jerez, Newton Game Dynamics>
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

#ifndef __ND_SCENE_NODE_H__
#define __ND_SCENE_NODE_H__

#include "ndCollisionStdafx.h"

//#define D_NEW_SCENE

class ndBodyKinematic;
class ndBvhLeafNode;
class ndBvhInternalNode;

D_MSV_NEWTON_ALIGN_32
class ndBvhNode : public ndContainersFreeListAlloc<ndBvhNode>
{
	public:
	ndBvhNode(ndBvhNode* const parent);
	ndBvhNode(const ndBvhNode& src);
	virtual ~ndBvhNode();
	virtual ndBvhNode* Clone() const;

	void Kill();
	void GetAabb(ndVector& minBox, ndVector& maxBox) const;
	void SetAabb(const ndVector& minBox, const ndVector& maxBox);

	virtual ndBvhNode* GetAsSceneNode() const;
	virtual ndBvhLeafNode* GetAsSceneBodyNode() const;
	virtual ndBvhInternalNode* GetAsSceneTreeNode() const;
	
	virtual ndBvhNode* GetLeft() const;
	virtual ndBvhNode* GetRight() const;
	virtual ndBodyKinematic* GetBody() const;

	virtual bool SanityCheck(ndUnsigned32 level) const;

	ndVector m_minBox;
	ndVector m_maxBox;
	ndBvhNode* m_parent;
#ifdef D_NEW_SCENE
	ndBvhNode* m_buildNode;
#endif
	ndSpinLock m_lock;
	ndInt32 m_depthLevel;
	ndUnsigned8 m_isDead;
	ndUnsigned8 m_bhvLinked;
#ifdef _DEBUG
	ndInt32 m_nodeId;
#endif

	static ndVector m_aabbQuantization;
	static ndVector m_aabbInvQuantization;
} D_GCC_NEWTON_ALIGN_32 ;

class ndBvhInternalNode: public ndBvhNode
{
	public:
	ndBvhInternalNode();
	ndBvhInternalNode(const ndBvhInternalNode& src);
	virtual ~ndBvhInternalNode();
	virtual ndBvhNode* Clone() const;

	virtual ndBvhNode* GetLeft() const;
	virtual ndBvhNode* GetRight() const;
	virtual ndBvhInternalNode* GetAsSceneTreeNode() const;

	bool SanityCheck(ndUnsigned32 level) const;

	ndBvhNode* m_left;
	ndBvhNode* m_right;
} D_GCC_NEWTON_ALIGN_32;

class ndBvhLeafNode : public ndBvhNode
{
	public:
	ndBvhLeafNode(ndBodyKinematic* const body);
	ndBvhLeafNode(const ndBvhLeafNode& src);
	virtual ~ndBvhLeafNode();

	virtual ndBvhNode* Clone() const;
	virtual ndBodyKinematic* GetBody() const;
	virtual ndBvhLeafNode* GetAsSceneBodyNode() const;

	ndBodyKinematic* m_body;
};

class ndBottomUpCell
{
	public:
	ndInt32 m_x;
	ndInt32 m_y;
	ndInt32 m_z;
	ndBvhNode* m_node;
};

class ndCellScanPrefix
{
	public:
	ndInt32 m_location : 30;
	ndUnsigned32 m_cellTest : 1;
};

class ndBuildBvhTreeBuildState
{
	public:
	enum ndState
	{
		m_beginBuild,
		m_calculateBoxes,
		m_buildLayer,
		m_enumerateLayers,
		m_endBuild,
	};

	ndBuildBvhTreeBuildState();

	void Init(ndInt32 maxCount);

	ndVector m_size;
	ndVector m_origin;
	ndArray<ndBottomUpCell> m_cellBuffer0;
	ndArray<ndBottomUpCell> m_cellBuffer1;
	ndArray<ndCellScanPrefix> m_cellCounts0;
	ndArray<ndCellScanPrefix> m_cellCounts1;
	ndArray<ndBvhNode*> m_tempNodeBuffer;

	ndBvhNode* m_root;
	ndBvhNode** m_srcArray;
	ndBvhNode** m_tmpArray;
	ndBvhNode** m_parentsArray;

	ndInt32 m_depthLevel;
	ndInt32 m_leafNodesCount;
	ndState m_state;
};

class ndBvhNodeArray : public ndArray<ndBvhNode*>
{
	public:
	ndBvhNodeArray();
	ndBvhNodeArray(const ndBvhNodeArray& src);
	~ndBvhNodeArray();

	void CleanUp();
	void Swap(ndBvhNodeArray& src);

	ndUnsigned32 m_isDirty;
	ndUnsigned32 m_scansCount;
	ndUnsigned32 m_scans[256];
};

class ndBvhSceneManager
{
	public:
	ndBvhSceneManager();
	ndBvhSceneManager(const ndBvhSceneManager& src);
	~ndBvhSceneManager();

	void CleanUp();
	ndBvhNode* AddBody(ndBodyKinematic* const body, ndBvhNode* root);
	void RemoveBody(ndBodyKinematic* const body);

	void UpdateScene(ndThreadPool& threadPool);
	ndBvhNode* BuildBvhTree(ndThreadPool& threadPool);

	ndBvhNodeArray& GetNodeArray();
	ndBvhLeafNode* GetLeafNode(ndBodyKinematic* const body) const;

	private:
	void Update(ndThreadPool& threadPool);
	bool BuildBvhTreeInitNodes(ndThreadPool& threadPool);
	void BuildBvhTreeSetNodesDepth(ndThreadPool& threadPool);
	void BuildBvhGenerateLayerGrids(ndThreadPool& threadPool);
	void BuildBvhTreeCalculateLeafBoxes(ndThreadPool& threadPool);
	
	ndBvhNode* BuildIncrementalBvhTree(ndThreadPool& threadPool);
	ndInt32 BuildSmallBvhTree(ndThreadPool& threadPool, ndBvhNode** const parentsArray, ndInt32 bashCount);

	void BuildBvhTreeSwapBuffers(ndThreadPool& threadPool);

	ndBvhNodeArray m_workingArray;
#ifdef D_NEW_SCENE
	ndBvhNodeArray m_buildArray;
#endif

	ndBuildBvhTreeBuildState m_bvhBuildState;
};


// **************************************************************
//
//  inline functions
//
// **************************************************************
inline ndBvhNode::ndBvhNode(ndBvhNode* const parent)
	:ndContainersFreeListAlloc<ndBvhNode>()
	,m_minBox(ndFloat32(-1.0e15f))
	,m_maxBox(ndFloat32(1.0e15f))
	,m_parent(parent)
#ifdef D_NEW_SCENE
	,m_buildNode(nullptr)
#endif
	,m_lock()
	,m_depthLevel(0)
	,m_isDead(0)
	,m_bhvLinked(0)
{
#ifdef _DEBUG
	m_nodeId = 0;
#endif
}

inline ndBvhNode::ndBvhNode(const ndBvhNode& src)
	:ndContainersFreeListAlloc<ndBvhNode>()
	,m_minBox(src.m_minBox)
	,m_maxBox(src.m_maxBox)
	,m_parent(nullptr)
#ifdef D_NEW_SCENE
	,m_buildNode((ndBvhNode*)&src)
#endif
	,m_lock()
	,m_depthLevel(0)
	,m_isDead(0)
	,m_bhvLinked(0)
{
#ifdef D_NEW_SCENE
	m_buildNode->m_buildNode = this;
#endif

#ifdef _DEBUG
	m_nodeId = 0;
#endif
}

inline ndBvhNode::~ndBvhNode()
{
}

inline ndBvhNode* ndBvhNode::GetAsSceneNode() const
{ 
	return (ndBvhNode*)this;
}

inline ndBvhLeafNode* ndBvhNode::GetAsSceneBodyNode() const
{ 
	return nullptr; 
}

inline ndBvhInternalNode* ndBvhNode::GetAsSceneTreeNode() const
{ 
	return nullptr; 
}

inline ndBodyKinematic* ndBvhNode::GetBody() const
{
	return nullptr;
}

inline ndBvhNode* ndBvhNode::GetLeft() const
{
	return nullptr;
}

inline ndBvhNode* ndBvhNode::GetRight() const
{
	return nullptr;
}

inline void ndBvhNode::Kill()
{
	m_isDead = 1;
#ifdef D_NEW_SCENE
	m_buildNode->m_isDead = 1;
#endif
}

inline void ndBvhNode::GetAabb(ndVector& minBox, ndVector& maxBox) const
{
	minBox = m_minBox;
	maxBox = m_maxBox;
}

inline void ndBvhNode::SetAabb(const ndVector& minBox, const ndVector& maxBox)
{
	ndAssert(minBox.m_x <= maxBox.m_x);
	ndAssert(minBox.m_y <= maxBox.m_y);
	ndAssert(minBox.m_z <= maxBox.m_z);

	const ndVector p0(minBox * m_aabbQuantization);
	const ndVector p1(maxBox * m_aabbQuantization + ndVector::m_one);

	m_minBox = p0.Floor() * m_aabbInvQuantization;
	m_maxBox = p1.Floor() * m_aabbInvQuantization;

	ndAssert(m_minBox.m_w == ndFloat32(0.0f));
	ndAssert(m_maxBox.m_w == ndFloat32(0.0f));
}

inline ndBvhNode* ndBvhNode::Clone() const
{
	ndAssert(0);
	return nullptr;
}

inline ndBvhLeafNode* ndBvhLeafNode::GetAsSceneBodyNode() const
{
	return (ndBvhLeafNode*)this;
}

inline ndBodyKinematic* ndBvhLeafNode::GetBody() const
{
	return m_body;
}

inline ndBvhInternalNode* ndBvhInternalNode::GetAsSceneTreeNode() const
{
	return (ndBvhInternalNode*)this;
}

inline ndBvhNode* ndBvhInternalNode::GetLeft() const
{
	return m_left;
}

inline ndBvhNode* ndBvhInternalNode::GetRight() const
{
	return m_right;
}

inline ndBvhNodeArray& ndBvhSceneManager::GetNodeArray()
{
	return m_workingArray;
}

#endif
