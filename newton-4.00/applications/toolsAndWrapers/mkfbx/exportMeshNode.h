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
			m_channels[0] = false;
			m_channels[1] = false;
			m_channels[2] = false;
			m_channels[3] = false;
			m_channels[4] = false;
			m_channels[5] = false;
			m_values[0] = 0.0f;
			m_values[1] = 0.0f;
			m_values[2] = 0.0f;
			m_values[3] = 0.0f;
			m_values[4] = 0.0f;
			m_values[5] = 0.0f;
		}
		float m_values[6];
		bool m_channels[6];
	};

	exportMeshNode();
	exportMeshNode(exportMeshNode* const parent);

	virtual ~exportMeshNode();

	static exportMeshNode* ImportBvhSkeleton(const char* const name);
	static exportMeshNode* ImportAsfSkeleton(const char* const asfName, const char* const amcName);

	static void ConvertToLocal(exportMeshNode* const root);
	private:
	void ImportAmcAnimation(const char* const amcName, const std::map<std::string, exportMeshNode*>& map, const std::map<std::string, animDof>& animBlueprint);

	public:
	exportMatrix m_matrix;
	exportVector m_eulers;
	std::string m_name;
	mutable FbxNode* m_fbxNode;
	exportMeshNode* m_parent;
	std::list<exportMeshNode*> m_children;

	std::vector<exportVector> m_positionsKeys;
	std::vector<exportVector> m_rotationsKeys;
};

