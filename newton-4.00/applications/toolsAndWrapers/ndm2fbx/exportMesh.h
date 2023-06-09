#pragma once
#include "exportMatrix.h"

class exportMaterial
{
	public:
	exportMaterial();
	~exportMaterial();

	exportVector m_ambient;
	exportVector m_diffuse;
	exportVector m_specular;
	float m_opacity;
	float m_shiness;
	std::string m_texName;
	std::vector<int> m_indexArray;
	std::vector<int> m_faceIndexCount;
};

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

	std::vector<exportMaterial> m_materials;
};

