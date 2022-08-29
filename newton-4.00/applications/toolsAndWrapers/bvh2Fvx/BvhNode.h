#pragma once
#include "bvhMatrix.h"

class BvhNode
{
	public:
	BvhNode();
	BvhNode(BvhNode* const parent);

	virtual ~BvhNode();

	bvhMatrix m_matrix;
	bvhVector m_eulers;
	std::string m_name;
	BvhNode* m_parent;
	std::list<BvhNode*> m_children;

	static BvhNode* LoadSkeleton(const char* const name);
	private:
	static BvhNode* ImportBvhSkeleton(const char* const name);
	static BvhNode* ImportAsfSkeleton(const char* const name);
};

