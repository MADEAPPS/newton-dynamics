#include "stdafx.h"
#include "exportMeshNode.h"

exportMeshNode::exportMeshNode()
	:m_localMatrix()
	,m_globalEuler(0.0f, 0.0f, 0.0f, 0.0f)
	,m_globalPosit(0.0f, 0.0f, 0.0f, 1.0f)
	,m_fbxNode(nullptr)
	,m_parent(nullptr)
	,m_positionsKeys()
	,m_rotationsKeys()
{
}

exportMeshNode::exportMeshNode(exportMeshNode* const parent)
	:m_localMatrix()
	,m_globalEuler(0.0f, 0.0f, 0.0f, 0.0f)
	,m_globalPosit(0.0f, 0.0f, 0.0f, 1.0f)
	,m_fbxNode(nullptr)
	,m_parent(parent)
{
	_ASSERT(parent);
	m_parent->m_children.push_back(this);
}

exportMeshNode::~exportMeshNode()
{
	while (m_children.size())
	{
		exportMeshNode* const child = m_children.back();
		m_children.pop_back();
		delete child;
	}
}

