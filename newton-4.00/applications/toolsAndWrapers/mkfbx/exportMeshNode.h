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
	void AlignFrames();
	void GenerateTpose();
	void DeleteEffectors();
	void SetFrame(int index);
	float CalculateDeltaAngle(float angle1, float angle0) const;

	std::string m_name;
	exportMatrix m_matrix;
	exportMeshNode* m_parent;
	mutable FbxNode* m_fbxNode;
	std::list<exportMeshNode*> m_children;

	animDof m_animDof;
	std::vector<exportMatrix> m_keyFrame;
};

