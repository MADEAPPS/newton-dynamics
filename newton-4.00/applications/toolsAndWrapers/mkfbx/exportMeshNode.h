#pragma once
#include "exportMatrix.h"

class exportMeshNode
{
	public:
	exportMeshNode();
	exportMeshNode(exportMeshNode* const parent);

	virtual ~exportMeshNode();

	exportMatrix m_matrix;
	exportVector m_eulers;
	std::string m_name;
	exportMeshNode* m_parent;
	std::list<exportMeshNode*> m_children;

	static exportMeshNode* LoadSkeleton(const char* const name);
	private:
	static exportMeshNode* ImportBvhSkeleton(const char* const name);
	static exportMeshNode* ImportAsfSkeleton(const char* const name);
};

