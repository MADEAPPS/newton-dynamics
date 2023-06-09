#pragma once
#include "exportMatrix.h"

class exportMeshNode
{
	public:
	exportMeshNode();
	exportMeshNode(exportMeshNode* const parent);

	virtual ~exportMeshNode();

	static exportMeshNode* ImportNdm(const char* const name);

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
	std::vector<exportMatrix> m_keyFrame;
};

