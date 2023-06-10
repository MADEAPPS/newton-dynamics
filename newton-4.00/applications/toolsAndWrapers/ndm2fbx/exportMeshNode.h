#pragma once
#include "exportMatrix.h"

class exportMesh;
class exportMeshNode
{
	public:
	exportMeshNode();
	exportMeshNode(exportMeshNode* const parent);

	virtual ~exportMeshNode();

	static exportMeshNode* ImportNdm(const char* const name);

	public:
	void AlignFrames();
	void SetFrame(int index);
	float CalculateDeltaAngle(float angle1, float angle0) const;

	void LoadNdmSkeleton(FILE* const file, std::map <int, std::shared_ptr<exportMesh>>& meshEffects);

	std::string m_name;
	exportMatrix m_matrix;
	exportMatrix m_meshMatrix;
	std::shared_ptr<exportMesh> m_mesh;
	exportMeshNode* m_parent;
	mutable FbxNode* m_fbxNode;
	std::list<exportMeshNode*> m_children;
	//std::vector<exportMatrix> m_keyFrame;
	std::list<exportVector> m_positionCurve;
	std::list<exportVector> m_rotationCurve;
};

