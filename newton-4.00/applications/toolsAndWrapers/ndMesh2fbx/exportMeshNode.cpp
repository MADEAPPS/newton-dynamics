#include "stdafx.h"
#include "exportMeshNode.h"

exportMeshNode::exportMeshNode()
	:m_matrix()
	,m_meshMatrix()
	,m_parent(nullptr)
	,m_fbxNode(nullptr)
{
}

exportMeshNode::exportMeshNode(exportMeshNode* const parent)
	:m_matrix()
	,m_meshMatrix()
	,m_parent(parent)
	,m_fbxNode(nullptr)
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
	float s1 = (float)sin(angle1);
	float c1 = (float)cos(angle1);
	float s0 = (float)sin(angle0);
	float c0 = (float)cos(angle0);

	float s = s1 * c0 - s0 * c1;
	float c = c1 * c0 + s0 * s1;
	float delta = (float)atan2(s, c);
	return delta;
}