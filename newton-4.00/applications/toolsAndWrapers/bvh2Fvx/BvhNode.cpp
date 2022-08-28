#include "stdafx.h"
#include "BvhNode.h"

BvhNode::BvhNode()
	:m_matrix()
	,m_parent(nullptr)
{
}

BvhNode::BvhNode(BvhNode* const parent)
	:m_matrix()
	,m_parent(parent)
{
	_ASSERT(parent);
	m_parent->m_children.push_back(this);
}

BvhNode::~BvhNode()
{
	while (m_children.size())
	{
		BvhNode* const child = m_children.back();
		m_children.pop_back();
		delete child;
	}
}


BvhNode* BvhNode::LoadBvhSkeleton(const char* const name)
{
	BvhNode* ent = nullptr;

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
	BvhNode* stackPool[128];
	std::vector<BvhNode*> entityList;
	
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
					ent = new BvhNode();
					entityList.push_back(ent);
					ReadToken();
					ent->m_name = token;
					stackPool[stack] = ent;
					stack = 1;
					ReadToken();
				}
				else if (!strcmp(token, "JOINT"))
				{
					BvhNode* const child = new BvhNode(stackPool[stack - 1]);
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
					BvhNode* const child = new BvhNode(stackPool[stack - 1]);
					entityList.push_back(child);
					ReadToken();
					child->m_name = "effector";
					stackPool[stack] = child;
					stack++;
					ReadToken();
				}
				else if (!strcmp(token, "OFFSET"))
				{
					BvhNode* const node = stackPool[stack - 1];
					node->m_matrix.m_posit.m_x = ReadFloat() * scale;
					node->m_matrix.m_posit.m_y = ReadFloat() * scale;
					node->m_matrix.m_posit.m_z = ReadFloat() * scale;
					
					//if (!strcmp(node->GetName().GetStr(), "ltibia"))
					//{
					//	break;
					//}
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
	//	BvhNode* const child = entityList[i];
	//	bvhMatrix matrix(invRotation * child->GetRenderMatrix() * rotation);
	//	child->ResetMatrix(matrix);
	//}
	
	return ent;
}
