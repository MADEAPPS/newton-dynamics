#include "stdafx.h"
#include "exportMeshNode.h"

exportMeshNode* exportMeshNode::ImportBvhSkeleton(const char* const name)
{
	exportMeshNode* entity = nullptr;

	FILE* const fp = fopen(name, "rt");

	char token[256];
	auto ReadToken = [fp, &token]()
	{
		fscanf(fp, "%s", token);
	};

	auto ReadFloat = [fp]()
	{
		float value;
		fscanf(fp, "%f", &value);
		return value;
	};

	auto ReadInt = [fp]()
	{
		int value;
		fscanf(fp, "%d", &value);
		return value;
	};

	int stack = 0;
	exportMeshNode* stackPool[128];
	std::vector<exportMeshNode*> entityList;
	
	float scale = 6.0f;
	if (fp)
	{
		ReadToken();
		if (!strcmp(token, "HIERARCHY"))
		{
			while (!feof(fp))
			{
				ReadToken();
				if (!strcmp(token, "ROOT"))
				{
					entity = new exportMeshNode();
					entityList.push_back(entity);
					ReadToken();
					entity->m_name = token;
					stackPool[stack] = entity;
					stack = 1;
					ReadToken();
				}
				else if (!strcmp(token, "JOINT"))
				{
					exportMeshNode* const child = new exportMeshNode(stackPool[stack - 1]);
					entityList.push_back(child);
					ReadToken();
					if (child->m_parent->m_parent)
					{
						strcat(token, "_bone");
					}
					else
					{
						strcat(token, "_ref");
					}
					child->m_name = token;
					stackPool[stack] = child;
					stack++;
					ReadToken();
				}
				else if (!strcmp(token, "End"))
				{
					exportMeshNode* const child = new exportMeshNode(stackPool[stack - 1]);
					entityList.push_back(child);
					ReadToken();
					child->m_name = "effector";
					stackPool[stack] = child;
					stack++;
					ReadToken();
				}
				else if (!strcmp(token, "OFFSET"))
				{
					exportMeshNode* const node = stackPool[stack - 1];
					//node->m_matrix.m_posit.m_x = ReadFloat() * scale;
					//node->m_matrix.m_posit.m_y = ReadFloat() * scale;
					//node->m_matrix.m_posit.m_z = ReadFloat() * scale;
					node->m_globalPosit.m_x = ReadFloat() * scale;
					node->m_globalPosit.m_y = ReadFloat() * scale;
					node->m_globalPosit.m_z = ReadFloat() * scale;
					node->m_globalPosit.m_w = 1.0f;
				}
				else if (!strcmp(token, "CHANNELS"))
				{
					int skips = ReadInt();
					for (int i = 0; i < skips; ++i)
					{
						ReadToken();
					}
				}
				else if (!strcmp(token, "}"))
				{
					stack--;
				}
				else if (!strcmp(token, "MOTION"))
				{
					break;
				}
			}
		}

		fclose(fp);
	}

	//bvhMatrix rotation(ndYawMatrix(90.0f * ndDegreeToRad));
	//bvhMatrix invRotation(rotation.Inverse());
	//for (int i = 0; i < entityList.GetCount(); ++i)
	//{
	//	exportMeshNode* const child = entityList[i];
	//	bvhMatrix matrix(invRotation * child->GetRenderMatrix() * rotation);
	//	child->ResetMatrix(matrix);
	//}
	
	return entity;
}


