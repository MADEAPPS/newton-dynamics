#pragma once
#include "exportMatrix.h"

class exportMeshNode
{
	public:
	class animDof
	{
		public:
		animDof()
		{
			m_channel[0] = -1;
			m_channel[1] = -1;
			m_channel[2] = -1;
			m_channel[3] = -1;
			m_channel[4] = -1;
			m_channel[5] = -1;
		}
		int m_channel[6];
	};

	exportMeshNode();
	exportMeshNode(exportMeshNode* const parent);

	virtual ~exportMeshNode();

	static exportMeshNode* ImportBvhSkeleton(const char* const name);

	public:
	float CalculateDeltaAngle(float angle1, float angle0) const;

	void CalculateTpose();
	void SetFrame(int index);

	void DeleteEffector();

	std::string m_name;
	exportMatrix m_matrix;
	exportMatrix m_tPoseMatrix;
	mutable FbxNode* m_fbxNode;
	exportMeshNode* m_parent;
	std::list<exportMeshNode*> m_children;

	animDof m_animDof;
	std::vector<exportVector> m_positionsKeys;
	std::vector<exportVector> m_rotationsKeys;
};

