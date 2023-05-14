#include "stdafx.h"
#include "exportMeshNode.h"

exportMeshNode::exportMeshNode()
	:m_matrix()
	,m_tPoseMatrix()
	,m_fbxNode(nullptr)
	,m_parent(nullptr)
	,m_animDof()
	,m_positionsKeys()
	,m_rotationsKeys()
{
}

exportMeshNode::exportMeshNode(exportMeshNode* const parent)
	:m_matrix()
	,m_tPoseMatrix()
	,m_fbxNode(nullptr)
	,m_parent(parent)
	,m_animDof()
	,m_positionsKeys()
	,m_rotationsKeys()
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

float exportMeshNode::CalculateDeltaAngle(float angle1, float angle0) const
{
	float s1 = sin(angle1);
	float c1 = cos(angle1);
	float s0 = sin(angle0);
	float c0 = cos(angle0);

	float s = s1 * c0 - s0 * c1;
	float c = c1 * c0 + s0 * s1;
	float delta = atan2(s, c);
	return delta;
}