#include "stdafx.h"
#include "BvhNode.h"

BvhNode::BvhNode()
	:m_matrix()
	,m_eulers(0.0f, 0.0f, 0.0f, 0.0f)
	,m_parent(nullptr)
{
}

BvhNode::BvhNode(BvhNode* const parent)
	:m_matrix()
	,m_eulers(0.0f, 0.0f, 0.0f, 0.0f)
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

BvhNode* BvhNode::LoadSkeleton(const char* const name)
{
	char filename[256];
	sprintf(filename, "%s", name);
	_strlwr(filename);

	BvhNode* root = nullptr;
	const char* const ext = strrchr(filename, '.');
	if (ext)
	{
		if (strcmp(ext, ".bvh") == 0)
		{
			root = LoadBvhSkeleton(name);
		}
		else if (strcmp(ext, ".asf") == 0)
		{
			root = LoadAsfSkeleton(name);
		}
	}
	return root;
}

BvhNode* BvhNode::LoadBvhSkeleton(const char* const name)
{
	BvhNode* entity = nullptr;

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
					entity = new BvhNode();
					entityList.push_back(entity);
					ReadToken();
					entity->m_name = token;
					stackPool[stack] = entity;
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
	
	return entity;
}


BvhNode* BvhNode::LoadAsfSkeleton(const char* const name)
{
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

	std::map<std::string, BvhNode*> map;

	BvhNode* entity = nullptr;
	float scale = 6.0f;
	float distScale = 1.0f;
	float angleScale = 1.0f;
	if (fp)
	{
		while (!feof(fp))
		{
			ReadToken();
			if (!strcmp(token, ":units"))
			{
				for (int i = 0; i < 3; i++)
				{
					ReadToken();
					if (!strcmp(token, "length"))
					{
						distScale = ReadFloat();
					}
					else if (!strcmp(token, "angle"))
					{
						ReadToken();
						if (!strcmp(token, "deg"))
						{
							angleScale = 3.14159265f / 180.0f;
						}
					}
					else
					{
						ReadToken();
					}
				}
			}
			else if (!strcmp(token, ":root"))
			{
				entity = new BvhNode();
				entity->m_name = "root";
				map[entity->m_name] = entity;
			}
			else if (!strcmp(token, "begin"))
			{
				ReadToken();
				ReadToken();
				ReadToken();
				ReadToken();
				BvhNode* const bone = new BvhNode();
				bone->m_name = token;
				map[bone->m_name] = bone;

				ReadToken();
				_ASSERT(!strcmp(token, "direction"));
				bvhVector dir;
				dir.m_x = ReadFloat();
				dir.m_y = ReadFloat();
				dir.m_z = ReadFloat();
				dir.m_w = 0.0f;

				ReadToken();
				_ASSERT(!strcmp(token, "length"));
				float length = ReadFloat();
				if (dir.DotProduct(dir) > 1.e-5f)
				{
					//dir = dir.Normalize().Scale(length * distScale);
					dir = dir.Normalize().Scale(length * scale);
				}
				dir.m_w = 1.0f;

				ReadToken();
				_ASSERT(!strcmp(token, "axis"));
				bvhVector euler;
				euler.m_x = ReadFloat();
				euler.m_y = ReadFloat();
				euler.m_z = ReadFloat();
				euler.m_w = 0.0f;
				bone->m_eulers = euler.Scale(angleScale);
				//if (bone->m_name != "ltibia") bone->m_eulers = euler.Scale(0.0f);

				bone->m_matrix.m_posit = dir;
			}
			else if (!strcmp(token, ":hierarchy"))
			{
				break;
			}
		}

		while (!feof(fp))
		{
			ReadToken();
			if (!strcmp(token, "begin"))
			{
			}
			else if (!strcmp(token, "end"))
			{
				break;
			}
			else
			{
				BvhNode* const parent = map[token];
				for (;1;)
				{
					int ch = getc(fp);
					if (ch == '\n')
					{
						break;
					}
					ungetc(ch, fp);
					ReadToken();
					BvhNode* const child = map[token];
					child->m_parent = parent;
					parent->m_children.push_back(child);
				}
			}
		}
		fclose(fp);

		int stack = 1;
		BvhNode* stackPool[128];
		bvhMatrix parentMatrixPool[128];
		stackPool[0] = entity;
		parentMatrixPool[0] = bvhMatrix();
		while (stack)
		{
			stack--;
			BvhNode* const bone = stackPool[stack];
			bvhMatrix pitch(ndPitchMatrix(bone->m_eulers.m_x));
			bvhMatrix yaw(ndYawMatrix(bone->m_eulers.m_y));
			bvhMatrix roll(ndRollMatrix(bone->m_eulers.m_z));
			bvhMatrix globalMatrix(pitch * yaw * roll);
			globalMatrix.m_posit = bone->m_matrix.m_posit + parentMatrixPool[stack].m_posit;
			globalMatrix.m_posit.m_w = 1.0f;
			bone->m_matrix = globalMatrix;
			for (std::list<BvhNode*>::const_iterator iter = bone->m_children.begin();
				iter != bone->m_children.end(); iter++)
			{
				stackPool[stack] = *iter;
				parentMatrixPool[stack] = globalMatrix;
				stack++;
			}
		}

		stack = 1;
		stackPool[0] = entity;
		parentMatrixPool[0] = bvhMatrix();
		while (stack)
		{
			stack--;
			BvhNode* const bone = stackPool[stack];
			bvhMatrix globalMatrix(bone->m_matrix);
			bvhMatrix localMatrix(globalMatrix * parentMatrixPool[stack].Inverse());
			bone->m_eulers = localMatrix.CalcPitchYawRoll();
			bone->m_matrix = localMatrix;
			for (std::list<BvhNode*>::const_iterator iter = bone->m_children.begin();
				iter != bone->m_children.end(); iter++)
			{
				stackPool[stack] = *iter;
				parentMatrixPool[stack] = globalMatrix;
				stack++;
			}
		}
	}

	return entity;
}