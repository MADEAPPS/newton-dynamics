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

#include "ndCoreStdafx.h"
#include "ndCollisionStdafx.h"
#include "ndBvhNode.h"
#include "ndBodyKinematic.h"

#define D_AABB_QUANTIZATION		ndFloat32 (4.0f)
#define D_AABB_INV_QUANTIZATION	(ndFloat32 (1.0f) / D_AABB_QUANTIZATION)

ndVector ndBvhNode::m_aabbQuantization(D_AABB_QUANTIZATION, D_AABB_QUANTIZATION, D_AABB_QUANTIZATION, ndFloat32 (0.0f));
ndVector ndBvhNode::m_aabbInvQuantization(D_AABB_INV_QUANTIZATION, D_AABB_INV_QUANTIZATION, D_AABB_INV_QUANTIZATION, ndFloat32(0.0f));

ndBvhLeafNode::ndBvhLeafNode(ndBodyKinematic* const body)
	:ndBvhNode(nullptr)
	,m_body(body)
{
#ifdef _DEBUG
	static ndInt32 nodeId = 0;
	m_nodeId = nodeId;
	nodeId++;
#endif

	SetAabb(body->m_minAabb, body->m_maxAabb);
}

ndBvhLeafNode::ndBvhLeafNode(const ndBvhLeafNode& src)
	:ndBvhNode(src)
	,m_body(src.m_body)
{
#ifdef _DEBUG
	m_nodeId = src.m_nodeId;
#endif
	SetAabb(src.m_minBox, src.m_maxBox);
}

ndBvhLeafNode::~ndBvhLeafNode()
{
}

ndBvhNode* ndBvhLeafNode::Clone() const
{
	ndBvhNode* const node = new ndBvhLeafNode(*this);
	return node;
}

ndBvhInternalNode::ndBvhInternalNode()
	:ndBvhNode(nullptr)
	,m_left(nullptr)
	,m_right(nullptr)
{
#ifdef _DEBUG
	static ndInt32 nodeId = 1000000;
	m_nodeId = nodeId;
	nodeId++;
#endif
}

ndBvhInternalNode::ndBvhInternalNode(const ndBvhInternalNode& src)
	:ndBvhNode(src)
	,m_left(nullptr)
	,m_right(nullptr)
{
#ifdef _DEBUG
	m_nodeId = src.m_nodeId;
#endif
}

ndBvhNode* ndBvhInternalNode::Clone() const
{
	ndBvhNode* const node = new ndBvhInternalNode(*this);
	return node;
}

ndBvhInternalNode::~ndBvhInternalNode()
{
}

bool ndBvhInternalNode::SanityCheck(ndUnsigned32 level) const
{
	ndAssert(m_left->m_parent == this);
	ndAssert(m_right->m_parent == this);

	ndBvhNode::SanityCheck(level);
	m_left->SanityCheck(level + 1);
	m_right->SanityCheck(level + 1);
	return true;
}

#ifdef _DEBUG
bool ndBvhNode::SanityCheck(ndUnsigned32 level) const
{
	char margin[512];
	level = ndMin(level, ndUnsigned32(sizeof(margin) / 2 - 1));

	for (ndUnsigned32 i = 0; i < level; ++i)
	{
		margin[i * 2] = ' ';
		margin[i * 2 + 1] = ' ';
	}
	margin[level * 2] = 0;
	//if (GetBody())
	//{
	//	ndTrace(("%s nodeId:%d  depth:%d  id:%d\n", margin, m_nodeId, m_depthLevel, GetBody()->GetId()));
	//}
	//else
	//{
	//	ndTrace(("%s nodeId:%d  depth:%d\n", margin, m_nodeId, m_depthLevel));
	//}
	ndAssert(!m_parent || (m_depthLevel < m_parent->m_depthLevel));
	ndAssert(!m_parent || ndBoxInclusionTest(m_minBox, m_maxBox, m_parent->m_minBox, m_parent->m_maxBox));
	return true;
}

#else
bool ndBvhNode::SanityCheck(ndUnsigned32) const
{
	return true;
}
#endif


ndBvhNodeArray::ndBvhNodeArray()
	:ndArray<ndBvhNode*>(1024)
	,m_isDirty(1)
	,m_scansCount(0)
{
}

ndBvhNodeArray::ndBvhNodeArray(const ndBvhNodeArray& src)
	:ndArray<ndBvhNode*>(1024)
	,m_isDirty(1)
	,m_scansCount(0)
{
	Swap((ndBvhNodeArray&)src);
}

ndBvhNodeArray::~ndBvhNodeArray()
{
	CleanUp();
}

void ndBvhNodeArray::CleanUp()
{
	for (ndInt32 i = ndInt32(GetCount()) - 1; i >= 0; --i)
	{
		ndBvhNode* const node = (*this)[i];
		ndAssert(node->m_isDead);
		delete node;
	}
	SetCount(0);
}

void ndBvhNodeArray::Swap(ndBvhNodeArray& src)
{
	ndArray<ndBvhNode*>::Swap(src);

	ndSwap(m_isDirty, src.m_isDirty);
	ndSwap(m_scansCount, src.m_scansCount);
	for (ndInt32 i = 0; i < ndInt32 (sizeof(m_scans) / sizeof(m_scans[0])); ++i)
	{
		ndSwap(m_scans[i], src.m_scans[i]);
	}
}

ndBvhSceneManager::ndBvhSceneManager()
	:m_workingArray()
	,m_bvhBuildState()
{
}

ndBvhSceneManager::ndBvhSceneManager(const ndBvhSceneManager& src)
	:m_workingArray(src.m_workingArray)
	,m_bvhBuildState(src.m_bvhBuildState)
{
}

ndBvhSceneManager::~ndBvhSceneManager()
{
	CleanUp();
}

ndBvhNode* ndBvhSceneManager::AddBody(ndBodyKinematic* const body, ndBvhNode* root)
{
	m_workingArray.m_isDirty = 1;
	ndBvhLeafNode* const bodyNode = new ndBvhLeafNode(body);
	ndBvhInternalNode* sceneNode = new ndBvhInternalNode();

	sceneNode->m_isDead = 0;
	m_workingArray.PushBack(sceneNode);
	body->m_sceneNodeIndex = ndInt32(m_workingArray.GetCount()) - 1;
	
	bodyNode->m_isDead = 0;
	m_workingArray.PushBack(bodyNode);
	body->m_bodyNodeIndex = ndInt32(m_workingArray.GetCount()) - 1;

	if (m_workingArray.GetCount() > 2)
	{
		ndAssert(root);
		ndBvhInternalNode* childNode = sceneNode;
		childNode->m_minBox = bodyNode->m_minBox;
		childNode->m_maxBox = bodyNode->m_maxBox;
		childNode->m_left = bodyNode;
		bodyNode->m_parent = childNode;

		ndUnsigned32 depth = 0;
		ndBvhNode* rootNode = root;
		ndBvhNode* parent = rootNode;
		while (1)
		{
			sceneNode = parent->GetAsSceneTreeNode();
			if (sceneNode && ndBoxInclusionTest(childNode->m_minBox, childNode->m_maxBox, parent->m_minBox, parent->m_maxBox))
			{
				const ndVector minLeftBox (sceneNode->m_left->m_minBox.GetMin(childNode->m_minBox));
				const ndVector maxLeftBox (sceneNode->m_left->m_maxBox.GetMax(childNode->m_maxBox));
				const ndVector minRightBox(sceneNode->m_right->m_minBox.GetMin(childNode->m_minBox));
				const ndVector maxRightBox(sceneNode->m_right->m_maxBox.GetMax(childNode->m_maxBox));
				const ndVector leftSize(maxLeftBox - minLeftBox);
				const ndVector rightSize(maxRightBox - minRightBox);
				const ndFloat32 leftArea = leftSize.DotProduct(leftSize.ShiftTripleRight()).GetScalar();
				const ndFloat32 rightArea = rightSize.DotProduct(rightSize.ShiftTripleRight()).GetScalar();

				parent = (leftArea < rightArea) ? sceneNode->m_left : sceneNode->m_right;
				depth++;
			}
			else
			{
				if (parent->m_parent)
				{
					if (parent->m_parent->GetLeft() == parent)
					{
						parent->m_parent->GetAsSceneTreeNode()->m_left = childNode;
					}
					else
					{
						parent->m_parent->GetAsSceneTreeNode()->m_right = childNode;
					}
					childNode->m_right = parent;
					childNode->m_parent = parent->m_parent;
					parent->m_parent = childNode;

					const ndVector minBox(childNode->m_left->m_minBox.GetMin(childNode->m_right->m_minBox));
					const ndVector maxBox(childNode->m_left->m_maxBox.GetMax(childNode->m_right->m_maxBox));
					childNode->m_minBox = minBox;
					childNode->m_maxBox = maxBox;
				}
				else
				{
					const ndVector minBox(parent->m_minBox.GetMin(childNode->m_minBox));
					const ndVector maxBox(parent->m_maxBox.GetMax(childNode->m_maxBox));
					childNode->m_minBox = minBox;
					childNode->m_maxBox = maxBox;
					childNode->m_right = parent;
					childNode->m_parent = nullptr;
					parent->m_parent = childNode;
					rootNode = childNode;
				}
				break;
			}
		}
		#ifdef _DEBUG
		//ndAssert(depth < 128);
		if (depth >= 256)
		{
			ndTrace(("This may be a pathological scene, consider balancing the scene\n"));
		}
		#endif
		return rootNode;
	}
	else
	{
		return bodyNode;
	}
}

void ndBvhSceneManager::RemoveBody(ndBodyKinematic* const body)
{
	m_workingArray.m_isDirty = 1;
	ndBvhLeafNode* const bodyNode = (ndBvhLeafNode*)m_workingArray[body->m_bodyNodeIndex];
	ndBvhInternalNode* const sceneNode = (ndBvhInternalNode*)m_workingArray[body->m_sceneNodeIndex];
	ndAssert(bodyNode->GetAsSceneBodyNode());
	ndAssert(sceneNode->GetAsSceneTreeNode());
	bodyNode->Kill();
	sceneNode->Kill();
}

ndBvhLeafNode* ndBvhSceneManager::GetLeafNode(ndBodyKinematic* const body) const
{
	ndAssert(m_workingArray[body->m_bodyNodeIndex] && m_workingArray[body->m_bodyNodeIndex]->GetAsSceneBodyNode());
	return (ndBvhLeafNode*)m_workingArray[body->m_bodyNodeIndex];
}


void ndBvhSceneManager::CleanUp()
{
	m_workingArray.CleanUp();
}

void ndBvhSceneManager::Update(ndThreadPool& threadPool)
{
	ndBvhNodeArray& nodeArray = m_workingArray;

	if (nodeArray.m_isDirty && nodeArray.GetCount())
	{
		D_TRACKTIME();
		const ndInt32 count = ndInt32(nodeArray.GetCount());
		nodeArray.SetCount(count * 2);
		ndBvhNode** const src = &nodeArray[0];
		ndBvhNode** const tmp = &nodeArray[count];

		class ndSortSceneNodeKey
		{
			public:
			ndSortSceneNodeKey(const void* const)
			{
				m_keyCode[0] = 0;
				m_keyCode[1] = 1;
				m_keyCode[2] = 2;
				m_keyCode[3] = 2;
			}

			ndInt32 GetKey(const ndBvhNode* const node) const
			{
				ndUnsigned32 code = ndUnsigned32(node->m_isDead * 2 + (((ndBvhNode*)node)->GetAsSceneBodyNode() ? 1 : 0));
				return m_keyCode[code];
			}

			ndInt32 m_keyCode[4];
		};

		ndUnsigned32 scans[5];
		ndCountingSortInPlace<ndBvhNode*, ndSortSceneNodeKey, 2>(threadPool, src, tmp, count, scans, nullptr);

		const ndInt32 alivedStart = ndInt32(scans[2]);
		const ndInt32 deadCount = ndInt32(scans[3]) - alivedStart;
		for (ndInt32 i = 0; i < deadCount; ++i)
		{
			ndBvhNode* const node = nodeArray[alivedStart + i];
			delete node;
		}
		nodeArray.SetCount(alivedStart);

		if (nodeArray.GetCount())
		{
			ndAtomic<ndInt32> iterator(0);
			auto EnumerateNodes = ndMakeObject::ndFunction([&iterator, &nodeArray](ndInt32, ndInt32)
			{
				D_TRACKTIME_NAMED(MarkCellBounds);
				ndBvhNode** const nodes = &nodeArray[0];
				const ndInt32 baseCount = ndInt32(nodeArray.GetCount()) / 2;
				for (ndInt32 i = iterator.fetch_add(D_WORKER_BATCH_SIZE); i < baseCount; i = iterator.fetch_add(D_WORKER_BATCH_SIZE))
				{
					const ndInt32 maxSpan = ((baseCount - i) >= D_WORKER_BATCH_SIZE) ? D_WORKER_BATCH_SIZE : baseCount - i;
					for (ndInt32 j = 0; j < maxSpan; ++j)
					{
						ndBvhLeafNode* const bodyNode = (ndBvhLeafNode*)nodes[baseCount + i + j];
						ndAssert(nodes[i + j]->GetAsSceneTreeNode());
						ndAssert(nodes[baseCount + i + j]->GetAsSceneBodyNode());

						ndBodyKinematic* const body = bodyNode->m_body;
						body->m_sceneNodeIndex = i + j;
						body->m_bodyNodeIndex = baseCount + i + j;
					}
				}
			});
			threadPool.ParallelExecute(EnumerateNodes);
		}
		nodeArray.m_isDirty = 0;
	}
}

ndBuildBvhTreeBuildState::ndBuildBvhTreeBuildState()
	:m_size(ndVector::m_zero)
	,m_origin(ndVector::m_zero)
	,m_cellBuffer0(1024)
	,m_cellBuffer1(1024)
	,m_cellCounts0(1024)
	,m_cellCounts1(1024)
	,m_tempNodeBuffer(1024)
	,m_root(nullptr)
	,m_srcArray(nullptr)
	,m_tmpArray(nullptr)
	,m_parentsArray(nullptr)
	,m_depthLevel(0)
	,m_leafNodesCount(0)
	,m_state(m_beginBuild)
{
}

void ndBuildBvhTreeBuildState::Init(ndInt32 maxCount)
{
	m_depthLevel = 1;
	m_tempNodeBuffer.SetCount(ndInt32(4 * (maxCount + 4)));

	m_root = nullptr;
	m_srcArray = &m_tempNodeBuffer[0];
	m_tmpArray = &m_srcArray[2 * (maxCount + 4)];
	m_parentsArray = &m_srcArray[maxCount];
	m_leafNodesCount = maxCount;
}

void ndBvhSceneManager::UpdateScene(ndThreadPool& threadPool)
{
	D_TRACKTIME();

	ndInt32 start = 0;
	ndInt32 count = 0;
	ndAtomic<ndInt32> iterator(0);
	auto UpdateSceneBvh = ndMakeObject::ndFunction([this, &iterator, &start, &count](ndInt32, ndInt32)
	{
		D_TRACKTIME_NAMED(UpdateSceneBvh);
		ndBvhInternalNode** const nodes = (ndBvhInternalNode**)&m_workingArray[start];
		const ndInt32 itemsCount = count;
		for (ndInt32 i = iterator.fetch_add(D_WORKER_BATCH_SIZE); i < itemsCount; i = iterator.fetch_add(D_WORKER_BATCH_SIZE))
		{
			const ndInt32 maxSpan = ((itemsCount - i) >= D_WORKER_BATCH_SIZE) ? D_WORKER_BATCH_SIZE : itemsCount - i;
			for (ndInt32 j = 0; j < maxSpan; ++j)
			{
				ndBvhInternalNode* const node = nodes[i + j];
				ndAssert(node && node->GetAsSceneNode());

				const ndVector minBox(node->m_left->m_minBox.GetMin(node->m_right->m_minBox));
				const ndVector maxBox(node->m_left->m_maxBox.GetMax(node->m_right->m_maxBox));
				if (!ndBoxInclusionTest(minBox, maxBox, node->m_minBox, node->m_maxBox))
				{
					node->m_minBox = minBox;
					node->m_maxBox = maxBox;
				}
			}
		}
	});

	const ndBvhNodeArray& array = m_workingArray;
	for (ndInt32 i = 0; i < ndInt32(array.m_scansCount); ++i)
	{
		start = ndInt32(array.m_scans[i]);
		count = ndInt32(array.m_scans[i + 1] - start);
		threadPool.ParallelExecute(UpdateSceneBvh);
	}
}

bool ndBvhSceneManager::BuildBvhTreeInitNodes(ndThreadPool& threadPool)
{
	D_TRACKTIME();
	ndAtomic<ndInt32> iterator(0);
	auto CopyBodyNodes = ndMakeObject::ndFunction([this, &iterator](ndInt32, ndInt32)
	{
		D_TRACKTIME_NAMED(CopyBodyNodes);

		ndBvhNodeArray& nodeArray = m_workingArray;
		const ndInt32 baseCount = ndInt32(nodeArray.GetCount()) / 2;
		ndBvhNode** const srcArray = m_bvhBuildState.m_srcArray;
		ndBvhLeafNode** const bodySceneNodes = (ndBvhLeafNode**)&nodeArray[baseCount];

		for (ndInt32 i = iterator.fetch_add(D_WORKER_BATCH_SIZE); i < baseCount; i = iterator.fetch_add(D_WORKER_BATCH_SIZE))
		{
			const ndInt32 maxSpan = ((baseCount - i) >= D_WORKER_BATCH_SIZE) ? D_WORKER_BATCH_SIZE : baseCount - i;
			for (ndInt32 j = 0; j < maxSpan; ++j)
			{
				ndBvhLeafNode* const node = bodySceneNodes[i + j];
				ndAssert(node && node->GetAsSceneBodyNode());
				ndBodyKinematic* const body = node->GetBody();

				node->m_bhvLinked = 0;
				node->m_depthLevel = 0;
				node->m_parent = nullptr;
				node->SetAabb(body->m_minAabb, body->m_maxAabb);
				srcArray[i + j] = node;
			}
		}
	});

	ndAtomic<ndInt32> iterator1(0);
	auto CopySceneNode = ndMakeObject::ndFunction([this, &iterator1](ndInt32, ndInt32)
	{
		D_TRACKTIME_NAMED(CopySceneNode);
		ndBvhNodeArray& nodeArray = m_workingArray;
		
		ndBvhInternalNode** const sceneNodes = (ndBvhInternalNode**)&nodeArray[0];
		ndBvhNode** const parentsArray = m_bvhBuildState.m_parentsArray;

		const ndInt32 baseCount = ndInt32(nodeArray.GetCount()) / 2;
		for (ndInt32 i = iterator1.fetch_add(D_WORKER_BATCH_SIZE); i < baseCount; i = iterator1.fetch_add(D_WORKER_BATCH_SIZE))
		{
			const ndInt32 maxSpan = ((baseCount - i) >= D_WORKER_BATCH_SIZE) ? D_WORKER_BATCH_SIZE : baseCount - i;
			for (ndInt32 j = 0; j < maxSpan; ++j)
			{
				ndBvhInternalNode* const node = sceneNodes[i + j];
				ndAssert(node && node->GetAsSceneTreeNode());

				parentsArray[i + j] = node;
				node->m_bhvLinked = 0;
				node->m_depthLevel = 0;
				node->m_parent = nullptr;
			}
		}
	});

	Update(threadPool);

	bool ret = false;
	ndBvhNodeArray& nodeArray = m_workingArray;

	if (nodeArray.GetCount())
	{
		ret = true;
		m_bvhBuildState.Init(ndInt32(nodeArray.GetCount()) / 2);
		threadPool.ParallelExecute(CopyBodyNodes);
		threadPool.ParallelExecute(CopySceneNode);
	}
	return ret;
}

void ndBvhSceneManager::BuildBvhTreeCalculateLeafBoxes(ndThreadPool& threadPool)
{
	D_TRACKTIME();
	ndVector boxes[D_MAX_THREADS_COUNT][2];
	ndFloat32 boxSizes[D_MAX_THREADS_COUNT];

	ndAtomic<ndInt32> iterator(0);
	auto CalculateBoxSize = ndMakeObject::ndFunction([this, &iterator, &boxSizes, &boxes](ndInt32 threadIndex, ndInt32)
	{
		D_TRACKTIME_NAMED(CalculateBoxSize);
		ndVector minP(ndFloat32(1.0e15f));
		ndVector maxP(ndFloat32(-1.0e15f));
		ndFloat32 minSize = ndFloat32(1.0e15f);

		ndBvhNode** const srcArray = m_bvhBuildState.m_srcArray;

		const ndInt32 leafNodesCount = m_bvhBuildState.m_leafNodesCount;
		for (ndInt32 i = iterator.fetch_add(D_WORKER_BATCH_SIZE); i < leafNodesCount; i = iterator.fetch_add(D_WORKER_BATCH_SIZE))
		{
			const ndInt32 maxSpan = ((leafNodesCount - i) >= D_WORKER_BATCH_SIZE) ? D_WORKER_BATCH_SIZE : leafNodesCount - i;
			for (ndInt32 j = 0; j < maxSpan; ++j)
			{
				const ndBvhNode* const node = srcArray[i + j];
				ndAssert(((ndBvhNode*)node)->GetAsSceneBodyNode());
				minP = minP.GetMin(node->m_minBox);
				maxP = maxP.GetMax(node->m_maxBox);
				const ndVector size(node->m_maxBox - node->m_minBox);
				ndFloat32 maxDim = ndMax(ndMax(size.m_x, size.m_y), size.m_z);
				minSize = ndMin(maxDim, minSize);
			}
		}
		boxes[threadIndex][0] = minP;
		boxes[threadIndex][1] = maxP;
		boxSizes[threadIndex] = minSize;
	});
	threadPool.ParallelExecute(CalculateBoxSize);

	ndVector minP(ndFloat32(1.0e15f));
	ndVector maxP(ndFloat32(-1.0e15f));
	ndFloat32 minBoxSize = ndFloat32(1.0e15f);
	const ndInt32 threadCount = threadPool.GetThreadCount();
	for (ndInt32 i = 0; i < threadCount; ++i)
	{
		minP = minP.GetMin(boxes[i][0]);
		maxP = maxP.GetMax(boxes[i][1]);
		minBoxSize = ndMin(minBoxSize, boxSizes[i]);
	}

	m_bvhBuildState.m_origin = minP;
	m_bvhBuildState.m_size = ndVector::m_triplexMask & ndVector(minBoxSize);
}

ndInt32 ndBvhSceneManager::BuildSmallBvhTree(ndThreadPool& threadPool, ndBvhNode** const parentsArray, ndInt32 bashCount)
{
	ndInt32 depthLevel[D_MAX_THREADS_COUNT];
	ndAtomic<ndInt32> iterator(0);
	auto SmallBhvNodes = ndMakeObject::ndFunction([this, &iterator, parentsArray, bashCount, &depthLevel](ndInt32 threadIndex, ndInt32)
	{
		D_TRACKTIME_NAMED(SmallBhvNodes);

		const ndCellScanPrefix* const srcCellNodes = &m_bvhBuildState.m_cellCounts0[0];
		const ndCellScanPrefix* const newParentsDest = &m_bvhBuildState.m_cellCounts1[0];
		const ndBottomUpCell* const nodesCells = &m_bvhBuildState.m_cellBuffer0[0];

		ndAssert(m_bvhBuildState.m_cellCounts0.GetCount() == m_bvhBuildState.m_cellCounts1.GetCount());

		auto MakeTwoNodesTree = [](ndBvhInternalNode* const root, ndBvhNode* const left, ndBvhNode* const right)
		{
			left->m_bhvLinked = 1;
			right->m_bhvLinked = 1;
			root->m_bhvLinked = 1;
			root->m_left = left;
			root->m_right = right;

			left->m_parent = root;
			right->m_parent = root;
			root->m_parent = nullptr;

			root->m_minBox = left->m_minBox.GetMin(right->m_minBox);
			root->m_maxBox = left->m_maxBox.GetMax(right->m_maxBox);
		};

		auto MakeThreeNodesTree = [](ndBvhInternalNode* const root, ndBvhInternalNode* const subRoot, ndBvhNode* const node0, ndBvhNode* const node1, ndBvhNode* const node2)
		{
			class ndNodeOrder
			{
				public:
				ndVector m_p0;
				ndVector m_p1;
				ndBvhNode* m_node0;
				ndBvhNode* m_node1;
				ndBvhNode* m_node2;
				ndFloat32 m_area;
			};

			ndNodeOrder order[3];

			order[0].m_node0 = node0;
			order[0].m_node1 = node1;
			order[0].m_node2 = node2;

			order[1].m_node0 = node1;
			order[1].m_node1 = node2;
			order[1].m_node2 = node0;

			order[2].m_node0 = node2;
			order[2].m_node1 = node0;
			order[2].m_node2 = node1;

			for (ndInt32 i = 0; i < 3; ++i)
			{
				order[i].m_p0 = order[i].m_node0->m_minBox.GetMin(order[i].m_node1->m_minBox);
				order[i].m_p1 = order[i].m_node0->m_maxBox.GetMax(order[i].m_node1->m_maxBox);
				const ndVector size(order[i].m_p1 - order[i].m_p0);
				order[i].m_area = size.DotProduct(size.ShiftTripleRight()).GetScalar();
			}

			if (order[2].m_area < order[1].m_area)
			{
				ndSwap(order[1], order[2]);
			}
			if (order[1].m_area < order[0].m_area)
			{
				ndSwap(order[1], order[0]);
			}

			root->m_bhvLinked = 1;
			node0->m_bhvLinked = 1;
			node1->m_bhvLinked = 1;
			node2->m_bhvLinked = 1;
			subRoot->m_bhvLinked = 1;

			subRoot->m_parent = root;
			subRoot->m_left = order[0].m_node0;
			subRoot->m_right = order[0].m_node1;
			subRoot->m_minBox = order[0].m_p0;
			subRoot->m_maxBox = order[0].m_p1;
			subRoot->m_left->m_parent = subRoot;
			subRoot->m_right->m_parent = subRoot;

			root->m_parent = nullptr;
			root->m_right = subRoot;
			root->m_left = order[0].m_node2;
			root->m_left->m_parent = root;

			root->m_minBox = root->m_left->m_minBox.GetMin(root->m_right->m_minBox);
			root->m_maxBox = root->m_left->m_maxBox.GetMax(root->m_right->m_maxBox);
		};

		ndInt32 maxDepth = 0;
		const ndInt32 count = bashCount;
		for (ndInt32 i = iterator.fetch_add(D_WORKER_BATCH_SIZE); i < count; i = iterator.fetch_add(D_WORKER_BATCH_SIZE))
		{
			const ndInt32 maxSpan = ((count - i) >= D_WORKER_BATCH_SIZE) ? D_WORKER_BATCH_SIZE : count - i;
			for (ndInt32 j = 0; j < maxSpan; ++j)
			{
				const ndInt32 k = i + j;
				const ndInt32 nodesCount = newParentsDest[k + 1].m_location - newParentsDest[k].m_location;

				ndInt32 depth = 0;
				if (nodesCount == 1)
				{
					const ndInt32 childIndex = srcCellNodes[k].m_location;
					const ndInt32 parentIndex = newParentsDest[k].m_location;
					ndBvhNode* const node0 = nodesCells[childIndex + 0].m_node;
					ndBvhNode* const node1 = nodesCells[childIndex + 1].m_node;
					ndBvhInternalNode* const root = parentsArray[parentIndex]->GetAsSceneTreeNode();
					ndAssert(root);
					ndAssert(!root->m_bhvLinked);
					MakeTwoNodesTree(root, node0, node1);
					root->m_bhvLinked = 0;
				}
				else if (nodesCount == 2)
				{
					const ndInt32 childIndex = srcCellNodes[k].m_location;
					const ndInt32 parentIndex = newParentsDest[k].m_location;

					ndBvhNode* const node0 = nodesCells[childIndex + 0].m_node;
					ndBvhNode* const node1 = nodesCells[childIndex + 1].m_node;
					ndBvhNode* const node2 = nodesCells[childIndex + 2].m_node;

					ndBvhInternalNode* const root = parentsArray[parentIndex + 0]->GetAsSceneTreeNode();
					ndBvhInternalNode* const subParent = parentsArray[parentIndex + 1]->GetAsSceneTreeNode();

					ndAssert(root);
					ndAssert(!root->m_bhvLinked);
					MakeThreeNodesTree(root, subParent, node0, node1, node2);
					root->m_bhvLinked = 0;
					depth += 1;
				}
				else if (nodesCount > 2)
				{
					class ndBlockSegment
					{
						public:
						ndInt32 m_start;
						ndInt32 m_count;
						ndInt32 m_rootNodeIndex;
						ndInt32 m_depth;
					};

					ndBlockSegment stackPool[32];

					ndInt32 stack = 1;
					ndInt32 rootNodeIndex = newParentsDest[k].m_location;
					stackPool[0].m_depth = 0;
					stackPool[0].m_rootNodeIndex = rootNodeIndex;
					stackPool[0].m_start = srcCellNodes[k].m_location;
					stackPool[0].m_count = srcCellNodes[k + 1].m_location - srcCellNodes[k].m_location;

					ndBvhNode* const rootNode = parentsArray[rootNodeIndex];
					rootNodeIndex++;

					ndInt32 maxStack = 0;
					while (stack)
					{
						stack--;
						const ndBlockSegment block(stackPool[stack]);
						ndAssert(block.m_count > 2);

						maxStack = ndMax(maxStack, stack);

						ndVector minP(ndFloat32(1.0e15f));
						ndVector maxP(ndFloat32(-1.0e15f));
						ndVector median(ndVector::m_zero);
						ndVector varian(ndVector::m_zero);

						for (ndInt32 m = 0; m < block.m_count; ++m)
						{
							ndBvhNode* const node = nodesCells[block.m_start + m].m_node;
							minP = minP.GetMin(node->m_minBox);
							maxP = maxP.GetMax(node->m_maxBox);
							ndVector p(ndVector::m_half * (node->m_minBox + node->m_maxBox));
							median += p;
							varian += p * p;
						}

						ndBvhInternalNode* const root = parentsArray[block.m_rootNodeIndex]->GetAsSceneTreeNode();
						root->m_minBox = minP;
						root->m_maxBox = maxP;
						root->m_bhvLinked = 1;

						ndInt32 index = 0;
						ndFloat32 maxVarian = ndFloat32(-1.0e15f);
						varian = varian.Scale(ndFloat32(block.m_count)) - median * median;
						for (ndInt32 m = 0; m < 3; ++m)
						{
							if (varian[m] > maxVarian)
							{
								index = m;
								maxVarian = varian[m];
							}
						}

						ndInt32 index0 = block.m_start + block.m_count / 2;
						if (maxVarian > ndFloat32(1.0e-3f))
						{
							class ndCompareContext
							{
							public:
								ndFloat32 m_midPoint;
								ndInt32 m_index;
							};

							class ndCompareKey
							{
								public:
								ndCompareKey(const void* const context)
								{
									const ndCompareContext* const info = (ndCompareContext*)context;

									m_index = info->m_index;
									m_midPoint = info->m_midPoint;
								}

								ndInt32 GetKey(const ndBottomUpCell& cell)
								{
									const ndBvhNode* const node = cell.m_node;
									const ndVector p(ndVector::m_half * (node->m_minBox + node->m_maxBox));
									ndInt32 key = p[m_index] >= m_midPoint;
									return key;
								}

								ndFloat32 m_midPoint;
								ndInt32 m_index;
							};

							ndCompareContext info;
							ndUnsigned32 scan[8];

							info.m_index = index;
							info.m_midPoint = median[index] / ndFloat32(block.m_count);
							ndCountingSortInPlace<ndBottomUpCell, ndCompareKey, 2>(&m_bvhBuildState.m_cellBuffer0[block.m_start], &m_bvhBuildState.m_cellBuffer1[block.m_start], block.m_count, scan, &info);
							index0 = block.m_start + ndInt32(scan[1]);
							if (index0 == block.m_start)
							{
								index0 = block.m_start + block.m_count / 2;
							}
							if (index0 == (block.m_start + block.m_count))
							{
								index0 = block.m_start + block.m_count / 2;
							}
						}

						ndAssert(index0 > block.m_start);
						ndAssert(index0 < (block.m_start + block.m_count));

						ndInt32 count0 = index0 - block.m_start;

						ndAssert(count0);
						if (count0 == 1)
						{
							ndBvhNode* const node = m_bvhBuildState.m_cellBuffer0[block.m_start].m_node;
							node->m_bhvLinked = 1;
							node->m_parent = root;
							root->m_left = node;
						}
						else if (count0 == 2)
						{
							ndBvhNode* const node0 = m_bvhBuildState.m_cellBuffer0[block.m_start + 0].m_node;
							ndBvhNode* const node1 = m_bvhBuildState.m_cellBuffer0[block.m_start + 1].m_node;
							ndBvhInternalNode* const parent = parentsArray[rootNodeIndex]->GetAsSceneTreeNode();
							rootNodeIndex++;

							ndAssert(root);
							MakeTwoNodesTree(parent, node0, node1);
							parent->m_parent = root;
							root->m_left = parent;
							maxStack = ndMax(maxStack, block.m_depth + 1);
						}
						else if (count0 == 3)
						{
							ndBvhNode* const node0 = m_bvhBuildState.m_cellBuffer0[block.m_start + 0].m_node;
							ndBvhNode* const node1 = m_bvhBuildState.m_cellBuffer0[block.m_start + 1].m_node;
							ndBvhNode* const node2 = m_bvhBuildState.m_cellBuffer0[block.m_start + 2].m_node;

							ndBvhInternalNode* const grandParent = parentsArray[rootNodeIndex]->GetAsSceneTreeNode();
							rootNodeIndex++;

							ndBvhInternalNode* const parent = parentsArray[rootNodeIndex]->GetAsSceneTreeNode();
							rootNodeIndex++;

							ndAssert(root);
							MakeThreeNodesTree(grandParent, parent, node0, node1, node2);
							grandParent->m_parent = root;
							root->m_left = grandParent;
							maxStack = ndMax(maxStack, block.m_depth + 2);
						}
						else
						{
							ndBvhInternalNode* const parent = parentsArray[rootNodeIndex]->GetAsSceneTreeNode();
							parent->m_bhvLinked = 1;
							parent->m_parent = root;
							parent->m_left = nullptr;
							parent->m_right = nullptr;
							root->m_left = parent;

							stackPool[stack].m_count = count0;
							stackPool[stack].m_start = block.m_start;
							stackPool[stack].m_depth = block.m_depth + 1;
							stackPool[stack].m_rootNodeIndex = rootNodeIndex;

							stack++;
							rootNodeIndex++;
							ndAssert(stack < sizeof(stackPool) / sizeof(stackPool[0]));
						}

						ndInt32 count1 = block.m_start + block.m_count - index0;
						ndAssert(count1);
						if (count1 == 1)
						{
							ndBvhNode* const node = m_bvhBuildState.m_cellBuffer0[index0].m_node;
							node->m_bhvLinked = 1;
							node->m_parent = root;
							root->m_right = node;
						}
						else if (count1 == 2)
						{
							ndBvhNode* const node0 = m_bvhBuildState.m_cellBuffer0[index0 + 0].m_node;
							ndBvhNode* const node1 = m_bvhBuildState.m_cellBuffer0[index0 + 1].m_node;
							ndBvhInternalNode* const parent = parentsArray[rootNodeIndex]->GetAsSceneTreeNode();
							rootNodeIndex++;

							ndAssert(root);
							MakeTwoNodesTree(parent, node0, node1);
							parent->m_parent = root;
							root->m_right = parent;
							maxStack = ndMax(maxStack, block.m_depth + 1);
						}
						else if (count1 == 3)
						{
							ndBvhNode* const node0 = m_bvhBuildState.m_cellBuffer0[index0 + 0].m_node;
							ndBvhNode* const node1 = m_bvhBuildState.m_cellBuffer0[index0 + 1].m_node;
							ndBvhNode* const node2 = m_bvhBuildState.m_cellBuffer0[index0 + 2].m_node;

							ndBvhInternalNode* const grandParent = parentsArray[rootNodeIndex]->GetAsSceneTreeNode();
							rootNodeIndex++;

							ndBvhInternalNode* const parent = parentsArray[rootNodeIndex]->GetAsSceneTreeNode();
							rootNodeIndex++;

							ndAssert(root);
							MakeThreeNodesTree(grandParent, parent, node0, node1, node2);
							grandParent->m_parent = root;
							root->m_right = grandParent;
							maxStack = ndMax(maxStack, block.m_depth + 2);
						}
						else
						{
							ndBvhInternalNode* const parent = parentsArray[rootNodeIndex]->GetAsSceneTreeNode();
							parent->m_bhvLinked = 1;
							parent->m_parent = root;
							parent->m_left = nullptr;
							parent->m_right = nullptr;
							root->m_right = parent;

							stackPool[stack].m_start = index0;
							stackPool[stack].m_count = count1;
							stackPool[stack].m_depth = block.m_depth + 1;
							stackPool[stack].m_rootNodeIndex = rootNodeIndex;

							stack++;
							rootNodeIndex++;
							ndAssert(stack < sizeof(stackPool) / sizeof(stackPool[0]));
						}
					}
					rootNode->m_bhvLinked = 0;
					depth = maxStack;
				}
				#ifdef _DEBUG
				else if (nodesCount == 0)
				{
					ndInt32 index = srcCellNodes[k].m_location;
					ndBvhNode* const node = nodesCells[index].m_node;
					ndAssert(!node->m_bhvLinked);
				}
				#endif		

				maxDepth = ndMax(maxDepth, depth);
			}
		}
		depthLevel[threadIndex] = maxDepth;
	});
	threadPool.ParallelExecute(SmallBhvNodes);

	ndInt32 depth = 0;
	for (ndInt32 i = threadPool.GetThreadCount() - 1; i >= 0; --i)
	{
		depth = ndMax(depth, depthLevel[i]);
	}
	return depth;
}

void ndBvhSceneManager::BuildBvhTreeSetNodesDepth(ndThreadPool& threadPool)
{
	D_TRACKTIME();
	class ndSortGetDethpKey
	{
		public:
		ndSortGetDethpKey(const void* const)
		{
		}

		ndInt32 GetKey(const ndBvhInternalNode* const node) const
		{
			return node->m_depthLevel;
		}
	};

	ndBvhNodeArray& nodeArray = m_workingArray;

	ndInt32 sceneNodeCount = ndInt32(nodeArray.GetCount()) / 2 - 1;
	ndBvhInternalNode** const view = (ndBvhInternalNode**)&nodeArray[0];
	ndBvhInternalNode** tmpBuffer = (ndBvhInternalNode**)&m_bvhBuildState.m_tempNodeBuffer[0];

	ndUnsigned32 scans[257];
	ndCountingSortInPlace<ndBvhInternalNode*, ndSortGetDethpKey, 8>(threadPool, &view[0], tmpBuffer, sceneNodeCount, scans, nullptr);

	nodeArray.m_scansCount = 0;
	for (ndInt32 i = 1; (i < 257) && (ndInt32(scans[i]) < sceneNodeCount); ++i)
	{
		nodeArray.m_scans[i - 1] = scans[i];
		nodeArray.m_scansCount++;
	}
	nodeArray.m_scans[nodeArray.m_scansCount] = scans[nodeArray.m_scansCount + 1];
	ndAssert(nodeArray.m_scans[0] == 0);
}

void ndBvhSceneManager::BuildBvhGenerateLayerGrids(ndThreadPool& threadPool)
{
	D_TRACKTIME();
	enum
	{
		m_linkedCell,
		m_insideCell,
		m_outsideCell,
	};

	class ndGridClassifier
	{
		public:
		ndGridClassifier(void* const data)
		{
			const ndBuildBvhTreeBuildState* const info = (ndBuildBvhTreeBuildState*)data;
			m_size = info->m_size;
			m_origin = info->m_origin;
			m_invSize = ndVector::m_triplexMask & ndVector(ndFloat32(1.0f) / m_size.m_x);

			m_code[0] = m_outsideCell;
			m_code[1] = m_insideCell;
			m_code[2] = m_linkedCell;
			m_code[3] = m_linkedCell;
		}

		ndInt32 GetKey(const ndBvhNode* const node) const
		{
			const ndVector minPosit((m_invSize * (node->m_minBox - m_origin)).GetInt());
			const ndVector maxPosit((m_invSize * (node->m_maxBox - m_origin)).GetInt());
			const ndInt32 x0 = ndInt32 (minPosit.m_ix);
			const ndInt32 y0 = ndInt32 (minPosit.m_iy);
			const ndInt32 z0 = ndInt32 (minPosit.m_iz);
			const ndInt32 x1 = ndInt32 (maxPosit.m_ix);
			const ndInt32 y1 = ndInt32 (maxPosit.m_iy);
			const ndInt32 z1 = ndInt32 (maxPosit.m_iz);

			ndAssert(x0 >= 0);
			ndAssert(y0 >= 0);
			ndAssert(z0 >= 0);
			ndAssert(x1 >= 0);
			ndAssert(y1 >= 0);
			ndAssert(z1 >= 0);

			const ndUnsigned32 test_x = ndUnsigned32((((x1 - x0)) >> 1) == 0);
			const ndUnsigned32 test_y = ndUnsigned32((((y1 - y0)) >> 1) == 0);
			const ndUnsigned32 test_z = ndUnsigned32((((z1 - z0)) >> 1) == 0);
			const ndUnsigned32 test = test_x & test_y & test_z;
			const ndUnsigned32 codeIndex = node->m_bhvLinked * 2 + test;
			return m_code[codeIndex];
		}

		ndVector m_size;
		ndVector m_invSize;
		ndVector m_origin;
		ndInt32 m_code[5];
	};

	class ndSortCell_x
	{
		public:
		ndSortCell_x(const void* const shift)
		{
			m_shift = *((ndInt32*)shift);
		}

		ndInt32 GetKey(const ndBottomUpCell& cell) const
		{
			ndInt32 val = cell.m_x >> m_shift;
			return val & 0xff;
		}

		ndInt32 m_shift;
	};

	class ndSortCell_y
	{
		public:
		ndSortCell_y(const void* const shift)
		{
			m_shift = *((ndInt32*)shift);
		}

		ndInt32 GetKey(const ndBottomUpCell& cell) const
		{
			ndInt32 val = cell.m_y >> m_shift;
			return val & 0xff;
		}

		ndInt32 m_shift;
	};

	class ndSortCell_z
	{
		public:
		ndSortCell_z(const void* const shift)
		{
			m_shift = *((ndInt32*)shift);
		}

		ndInt32 GetKey(const ndBottomUpCell& cell) const
		{
			ndInt32 val = cell.m_z >> m_shift;
			return val & 0xff;
		}

		ndInt32 m_shift;
	};

	class ndSortCellCount
	{
		public:
		ndSortCellCount(const void* const)
		{
		}

		ndInt32 GetKey(const ndCellScanPrefix& cell) const
		{
			return ndInt32 (cell.m_cellTest);
		}
	};

	ndUnsigned32 prefixScan[8];
	ndInt32 maxGrids[D_MAX_THREADS_COUNT][3];

	ndCountingSortInPlace<ndBvhNode*, ndGridClassifier, 2>(threadPool, m_bvhBuildState.m_srcArray, m_bvhBuildState.m_tmpArray, m_bvhBuildState.m_leafNodesCount, prefixScan, &m_bvhBuildState);
	ndInt32 insideCellsCount = ndInt32(prefixScan[m_insideCell + 1] - prefixScan[m_insideCell]);
	if (insideCellsCount)
	{
		m_bvhBuildState.m_cellBuffer0.SetCount(insideCellsCount);
		m_bvhBuildState.m_cellBuffer1.SetCount(insideCellsCount);
		const ndUnsigned32 linkedNodes = prefixScan[m_linkedCell + 1] - prefixScan[m_linkedCell];
		m_bvhBuildState.m_srcArray += linkedNodes;
		m_bvhBuildState.m_leafNodesCount -= linkedNodes;

		ndAtomic<ndInt32> iterator(0);
		auto MakeGrids = ndMakeObject::ndFunction([this, &iterator, &maxGrids](ndInt32 threadIndex, ndInt32)
		{
			D_TRACKTIME_NAMED(MakeGrids);

			const ndGridClassifier gridClassifier(&m_bvhBuildState);
			const ndVector origin(gridClassifier.m_origin);
			const ndVector invSize(gridClassifier.m_invSize);

			ndBvhNode** const srcArray = m_bvhBuildState.m_srcArray;

			ndInt32 max_x = 0;
			ndInt32 max_y = 0;
			ndInt32 max_z = 0;

			const ndInt32 count = ndInt32(m_bvhBuildState.m_cellBuffer0.GetCount());
			for (ndInt32 i = iterator.fetch_add(D_WORKER_BATCH_SIZE); i < count; i = iterator.fetch_add(D_WORKER_BATCH_SIZE))
			{
				const ndInt32 maxSpan = ((count - i) >= D_WORKER_BATCH_SIZE) ? D_WORKER_BATCH_SIZE : count - i;
				for (ndInt32 j = 0; j < maxSpan; ++j)
				{
					const ndInt32 k = i + j;
					ndBvhNode* const node = srcArray[k];
					const ndVector dist(node->m_minBox - origin);
					const ndVector posit(invSize * dist);
					const ndVector intPosit(posit.GetInt());
					m_bvhBuildState.m_cellBuffer0[k].m_x = ndInt32(intPosit.m_ix);
					m_bvhBuildState.m_cellBuffer0[k].m_y = ndInt32(intPosit.m_iy);
					m_bvhBuildState.m_cellBuffer0[k].m_z = ndInt32(intPosit.m_iz);
					m_bvhBuildState.m_cellBuffer0[k].m_node = node;
					max_x = ndMax(ndInt32(intPosit.m_ix), max_x);
					max_y = ndMax(ndInt32(intPosit.m_iy), max_y);
					max_z = ndMax(ndInt32(intPosit.m_iz), max_z);
				}
			}
			maxGrids[threadIndex][0] = max_x;
			maxGrids[threadIndex][1] = max_y;
			maxGrids[threadIndex][2] = max_z;
		});
		threadPool.ParallelExecute(MakeGrids);

		const ndInt32 threadCount = threadPool.GetThreadCount();
		for (ndInt32 i = 1; i < threadCount; ++i)
		{
			maxGrids[0][0] = ndMax(maxGrids[i][0], maxGrids[0][0]);
			maxGrids[0][1] = ndMax(maxGrids[i][1], maxGrids[0][1]);
			maxGrids[0][2] = ndMax(maxGrids[i][2], maxGrids[0][2]);
		}

		ndAssert(maxGrids[0][0] < 0x7fffffff);
		ndAssert(maxGrids[0][1] < 0x7fffffff);
		ndAssert(maxGrids[0][2] < 0x7fffffff);
		ndInt64 shift = 0;
		do {
			ndCountingSort<ndBottomUpCell, ndSortCell_x, 8>(threadPool, m_bvhBuildState.m_cellBuffer0, m_bvhBuildState.m_cellBuffer1, nullptr, &shift);
			shift += 8;
		} while (ndInt64(maxGrids[0][0]) > ndInt64(1) << shift);

		shift = 0;
		do {
			ndCountingSort<ndBottomUpCell, ndSortCell_y, 8>(threadPool, m_bvhBuildState.m_cellBuffer0, m_bvhBuildState.m_cellBuffer1, nullptr, &shift);
			shift += 8;
		} while (ndInt64(maxGrids[0][1]) > ndInt64(1) << shift);

		shift = 0;
		do {
			ndCountingSort<ndBottomUpCell, ndSortCell_z, 8>(threadPool, m_bvhBuildState.m_cellBuffer0, m_bvhBuildState.m_cellBuffer1, nullptr, &shift);
			shift += 8;
		} while (ndInt64(maxGrids[0][2]) > ndInt64(1) << shift);

		ndBottomUpCell sentinelCell;
		sentinelCell.m_x = ndUnsigned32(-1);
		sentinelCell.m_y = ndUnsigned32(-1);
		sentinelCell.m_z = ndUnsigned32(-1);
		sentinelCell.m_node = nullptr;

		m_bvhBuildState.m_cellBuffer0.PushBack(sentinelCell);
		m_bvhBuildState.m_cellBuffer1.PushBack(sentinelCell);
		m_bvhBuildState.m_cellCounts0.SetCount(m_bvhBuildState.m_cellBuffer0.GetCount());
		m_bvhBuildState.m_cellCounts1.SetCount(m_bvhBuildState.m_cellBuffer1.GetCount());

		ndAtomic<ndInt32> iterator1(0);
		auto MarkCellBounds = ndMakeObject::ndFunction([this, &iterator1](ndInt32, ndInt32)
		{
			D_TRACKTIME_NAMED(MarkCellBounds);
			ndCellScanPrefix* const dst = &m_bvhBuildState.m_cellCounts0[0];

			const ndInt32 count = ndInt32(m_bvhBuildState.m_cellBuffer0.GetCount()) - 1;
			for (ndInt32 i = iterator1.fetch_add(D_WORKER_BATCH_SIZE); i < count; i = iterator1.fetch_add(D_WORKER_BATCH_SIZE))
			{
				const ndInt32 maxSpan = ((count - i) >= D_WORKER_BATCH_SIZE) ? D_WORKER_BATCH_SIZE : count - i;
				for (ndInt32 j = 0; j < maxSpan; ++j)
				{
					const ndInt32 k = i + j;
					const ndBottomUpCell& cell0 = m_bvhBuildState.m_cellBuffer0[k + 1];
					const ndBottomUpCell& cell1 = m_bvhBuildState.m_cellBuffer0[k];
					const ndUnsigned8 test = ndUnsigned8((cell0.m_x == cell1.m_x) & (cell0.m_y == cell1.m_y) & (cell0.m_z == cell1.m_z) & (cell1.m_node != nullptr));
					dst[k + 1].m_cellTest = test;
					dst[k + 1].m_location = k + 1;
				}
			}
		});
		threadPool.ParallelExecute(MarkCellBounds);

		m_bvhBuildState.m_cellCounts0[0].m_cellTest = 0;
		m_bvhBuildState.m_cellCounts0[0].m_location = 0;
		ndCountingSort<ndCellScanPrefix, ndSortCellCount, 1>(threadPool, m_bvhBuildState.m_cellCounts0, m_bvhBuildState.m_cellCounts1, prefixScan, nullptr);

		ndUnsigned32 sum = 0;
		const ndInt32 bashCount = ndInt32(prefixScan[1] - 1);
		for (ndInt32 i = 0; i < bashCount; ++i)
		{
			const ndInt32 count = m_bvhBuildState.m_cellCounts0[i + 1].m_location - m_bvhBuildState.m_cellCounts0[i].m_location - 1;
			m_bvhBuildState.m_cellCounts1[i].m_location = ndInt32(sum);
			sum += count;
		}
		if (sum)
		{
			m_bvhBuildState.m_cellCounts1[bashCount].m_location = ndInt32(sum);
			ndInt32 subTreeDepth = BuildSmallBvhTree(threadPool, m_bvhBuildState.m_parentsArray, bashCount);
			m_bvhBuildState.m_depthLevel += subTreeDepth;

			ndAtomic<ndInt32> iterator2(0);
			auto EnumerateSmallBvh = ndMakeObject::ndFunction([this, &iterator2, sum](ndInt32, ndInt32)
			{
				D_TRACKTIME_NAMED(EnumerateSmallBvh);

				ndInt32 depthLevel = m_bvhBuildState.m_depthLevel;
				ndBvhNode** const parentsArray = m_bvhBuildState.m_parentsArray;

				const ndInt32 count = ndInt32(sum);
				for (ndInt32 i = iterator2.fetch_add(D_WORKER_BATCH_SIZE); i < count; i = iterator2.fetch_add(D_WORKER_BATCH_SIZE))
				{
					const ndInt32 maxSpan = ((count - i) >= D_WORKER_BATCH_SIZE) ? D_WORKER_BATCH_SIZE : count - i;
					for (ndInt32 j = 0; j < maxSpan; ++j)
					{
						ndBvhInternalNode* const root = parentsArray[i + j]->GetAsSceneTreeNode();
						ndAssert(root);
						if (!root->m_parent)
						{
							ndAssert(root->m_depthLevel == 0);
							class StackLevel
							{
							public:
								ndBvhInternalNode* m_node;
								ndInt32 m_depthLevel;
							};

							ndInt32 stack = 1;
							StackLevel m_stackPool[32];

							m_stackPool[0].m_node = root;
							m_stackPool[0].m_depthLevel = depthLevel;

							while (stack)
							{
								stack--;
								const StackLevel level(m_stackPool[stack]);

								ndBvhInternalNode* const node = level.m_node;
								node->m_depthLevel = level.m_depthLevel;

								ndBvhInternalNode* const left = node->m_left->GetAsSceneTreeNode();
								if (left && left->m_depthLevel == 0)
								{
									m_stackPool[stack].m_node = left;
									m_stackPool[stack].m_depthLevel = level.m_depthLevel - 1;
									stack++;
									ndAssert(stack < ndInt32(sizeof(m_stackPool) / sizeof(m_stackPool[0])));
								}

								ndBvhInternalNode* const right = node->m_right->GetAsSceneTreeNode();
								if (right && right->m_depthLevel == 0)
								{
									m_stackPool[stack].m_node = right;
									m_stackPool[stack].m_depthLevel = level.m_depthLevel - 1;
									stack++;
									ndAssert(stack < ndInt32(sizeof(m_stackPool) / sizeof(m_stackPool[0])));
								}
							}
						}
					}
				}
			});
			threadPool.ParallelExecute(EnumerateSmallBvh);

			m_bvhBuildState.m_parentsArray += sum;
			m_bvhBuildState.m_leafNodesCount += sum;
			m_bvhBuildState.m_depthLevel++;
		}
	}
}

ndBvhNode* ndBvhSceneManager::BuildIncrementalBvhTree(ndThreadPool& threadPool)
{
	D_TRACKTIME();
	ndBvhNode* root = nullptr;
	switch (m_bvhBuildState.m_state)
	{
		case ndBuildBvhTreeBuildState::m_beginBuild:
		{
			if (BuildBvhTreeInitNodes(threadPool))
			{
				m_bvhBuildState.m_state = m_bvhBuildState.m_calculateBoxes;
			}
			break;
		}

		case ndBuildBvhTreeBuildState::m_calculateBoxes:
		{
			BuildBvhTreeCalculateLeafBoxes(threadPool);
			m_bvhBuildState.m_state = m_bvhBuildState.m_buildLayer;
			break;
		}

		case ndBuildBvhTreeBuildState::m_buildLayer:
		{
			if (m_bvhBuildState.m_leafNodesCount > 1)
			{
				m_bvhBuildState.m_size = m_bvhBuildState.m_size * ndVector::m_two;
				BuildBvhGenerateLayerGrids(threadPool);
			}
			else
			{
				m_bvhBuildState.m_root = m_bvhBuildState.m_srcArray[0];
				m_bvhBuildState.m_state = m_bvhBuildState.m_enumerateLayers;
			}
			break;
		}

		case ndBuildBvhTreeBuildState::m_enumerateLayers:
		{
			BuildBvhTreeSetNodesDepth(threadPool);
			m_bvhBuildState.m_state = m_bvhBuildState.m_endBuild;
			break;
		}

		case ndBuildBvhTreeBuildState::m_endBuild:
		{
			root = m_bvhBuildState.m_root;
			ndAssert(m_bvhBuildState.m_root->SanityCheck(0));
			m_bvhBuildState.m_state = m_bvhBuildState.m_beginBuild;
			break;
		}

		default:
			ndAssert(0);
	}
	return root;
}

ndBvhNode* ndBvhSceneManager::BuildBvhTree(ndThreadPool& threadPool)
{
	D_TRACKTIME();

	//while (!BuildIncrementalBvhTree());

	if (!BuildBvhTreeInitNodes(threadPool))
	{
		return nullptr;
	}

	BuildBvhTreeCalculateLeafBoxes(threadPool);
	while (m_bvhBuildState.m_leafNodesCount > 1)
	{
		m_bvhBuildState.m_size = m_bvhBuildState.m_size * ndVector::m_two;
		BuildBvhGenerateLayerGrids(threadPool);
	}

	m_bvhBuildState.m_root = m_bvhBuildState.m_srcArray[0];

	BuildBvhTreeSetNodesDepth(threadPool);
	ndAssert(m_bvhBuildState.m_root->SanityCheck(0));
	
	return m_bvhBuildState.m_root;
}
