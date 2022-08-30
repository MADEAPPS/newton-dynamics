#include "stdafx.h"
#include "exportMeshNode.h"

exportMeshNode::exportMeshNode()
	:m_matrix()
	,m_eulers(0.0f, 0.0f, 0.0f, 0.0f)
	,m_parent(nullptr)
{
}

exportMeshNode::exportMeshNode(exportMeshNode* const parent)
	:m_matrix()
	,m_eulers(0.0f, 0.0f, 0.0f, 0.0f)
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

//exportMeshNode* exportMeshNode::LoadSkeleton(const char* const name)
//{
//	char filename[256];
//	sprintf(filename, "%s", name);
//	_strlwr(filename);
//
//	exportMeshNode* root = nullptr;
//	const char* const ext = strrchr(filename, '.');
//	if (ext)
//	{
//		if (strcmp(ext, ".bvh") == 0)
//		{
//			root = ImportBvhSkeleton(name);
//		}
//		else if (strcmp(ext, ".asf") == 0)
//		{
//			root = ImportAsfSkeleton(name);
//		}
//	}
//	return root;
//}

