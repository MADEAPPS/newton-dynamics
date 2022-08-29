#include "stdafx.h"
#include "BvhNode.h"

BvhNode::BvhNode()
	:m_matrix()
	,m_eulers(0.0f, 0.0f, 0.0f, 0.0f)
	,m_parent(nullptr)
{
}

BvhNode::BvhNode(BvhNode* const parent)
	:m_matrix()
	,m_eulers(0.0f, 0.0f, 0.0f, 0.0f)
	,m_parent(parent)
{
	_ASSERT(parent);
	m_parent->m_children.push_back(this);
}

BvhNode::~BvhNode()
{
	while (m_children.size())
	{
		BvhNode* const child = m_children.back();
		m_children.pop_back();
		delete child;
	}
}

BvhNode* BvhNode::LoadSkeleton(const char* const name)
{
	char filename[256];
	sprintf(filename, "%s", name);
	_strlwr(filename);

	BvhNode* root = nullptr;
	const char* const ext = strrchr(filename, '.');
	if (ext)
	{
		if (strcmp(ext, ".bvh") == 0)
		{
			root = ImportBvhSkeleton(name);
		}
		else if (strcmp(ext, ".asf") == 0)
		{
			root = ImportAsfSkeleton(name);
		}
	}
	return root;
}

