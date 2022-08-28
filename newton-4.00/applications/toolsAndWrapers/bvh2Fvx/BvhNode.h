#pragma once
#include "bvhMatrix.h"

class BvhNode
{
	public:
	BvhNode();
	BvhNode(BvhNode* const parent);

	virtual ~BvhNode();

	bvhMatrix m_matrix;
	std::string m_name;
	BvhNode* m_parent;
	std::list<BvhNode*> m_children;

	static BvhNode* LoadBvhSkeleton(const char* const name);
};

