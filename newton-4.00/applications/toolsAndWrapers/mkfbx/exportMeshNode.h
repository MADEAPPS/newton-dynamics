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

	static exportMeshNode* ImportBvhSkeleton(const char* const name);
	static exportMeshNode* ImportAsfSkeleton(const char* const asfName, const char* const amcName);

	private:
	void ImportAmcAnimation(const char* const amcName);
};

