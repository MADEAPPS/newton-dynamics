#pragma once
#include "exportMatrix.h"

class exportMesh
{
	public:
	exportMesh();
	~exportMesh();

	std::vector<exportVector> m_positions;
	std::vector<exportVector> m_normals;
	std::vector<exportVector> m_uvs;

	std::vector<int> m_positionsIndex;
	std::vector<int> m_normalsIndex;
	std::vector<int> m_uvIndex;
};

