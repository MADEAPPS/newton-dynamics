#include "stdafx.h"
#include "exportMeshNode.h"

void exportMeshNode::SetFrame(int index)
{
	int stack = 1;
	exportMeshNode* stackPool[128];

	stack = 1;
	stackPool[0] = this;
	while (stack)
	{
		stack--;

		exportMeshNode* const node = stackPool[stack];
		node->m_matrix = node->m_keyFrame[index];

		for (std::list<exportMeshNode*>::const_iterator iter = node->m_children.begin();
			iter != node->m_children.end(); iter++)
		{
			stackPool[stack] = *iter;
			stack++;
		}
	}
}

void exportMeshNode::AlignFrames()
{
	int stack = 1;
	exportMeshNode* stackPool[128];
	exportMatrix alignFrame[128];

	stack = 0;
	for (std::list<exportMeshNode*>::const_iterator iter = m_children.begin();
		iter != m_children.end(); iter++)
	{
		stackPool[stack] = *iter;
		alignFrame[stack] = exportMatrix();
		stack++;
	}

	while (stack)
	{
		stack--;

		exportMeshNode* const node = stackPool[stack];

		exportMatrix parentAlign (alignFrame[stack]);

		if (node->m_keyFrame.size())
		{
			exportMatrix childAlign;
			if (node->m_children.size())
			{
				exportMeshNode* const childNode = *node->m_children.begin();
				childAlign = exportMatrix(childNode->m_matrix.m_posit);
			}

			for (int i = 0; i < node->m_keyFrame.size(); ++i)
			{
				exportMatrix animMatrix(node->m_keyFrame[i]);
				exportMatrix matrix(childAlign * animMatrix * parentAlign);
				node->m_keyFrame[i] = matrix;
			}
			parentAlign = childAlign.Inverse();
		}

		for (std::list<exportMeshNode*>::const_iterator iter = node->m_children.begin();
			iter != node->m_children.end(); iter++)
		{
			stackPool[stack] = *iter;
			alignFrame[stack] = parentAlign;
			stack++;
		}
	}
}

void exportMeshNode::GenerateTpose()
{
	int stack = 1;
	exportMeshNode* stackPool[128];
	exportMatrix alignFrame[128];

	stack = 0;
	for (std::list<exportMeshNode*>::const_iterator iter = m_children.begin();
		iter != m_children.end(); iter++)
	{
		stackPool[stack] = *iter;
		alignFrame[stack] = exportMatrix();
		stack++;
	}

	while (stack)
	{
		stack--;

		exportMeshNode* const node = stackPool[stack];

		exportMatrix childAlign;
		exportMatrix parentAlign(alignFrame[stack]);
		
		if (node->m_children.size())
		{
			exportMeshNode* const childNode = *node->m_children.begin();
			childAlign = exportMatrix(childNode->m_matrix.m_posit);
		}

		exportMatrix animMatrix(node->m_matrix);
		exportMatrix matrix(childAlign * animMatrix * parentAlign);
		node->m_matrix = matrix;
		parentAlign = childAlign.Inverse();

		for (std::list<exportMeshNode*>::const_iterator iter = node->m_children.begin();
			iter != node->m_children.end(); iter++)
		{
			stackPool[stack] = *iter;
			alignFrame[stack] = parentAlign;
			stack++;
		}
	}
}

void exportMeshNode::DeleteEffectors()
{
	int stack = 1;
	exportMeshNode* stackPool[128];

	stack = 1;
	stackPool[0] = this;
	while (stack)
	{
		stack--;
		exportMeshNode* const node = stackPool[stack];

		if (node->m_name == "effector")
		{
			node->m_parent->m_children.clear();
			delete node;
		}
		else
		{
			for (std::list<exportMeshNode*>::const_iterator iter = node->m_children.begin();
				iter != node->m_children.end(); iter++)
			{
				stackPool[stack] = *iter;
				stack++;
			}
		}
	}
}

exportMeshNode* exportMeshNode::ImportNdm(const char* const name)
{
	exportMeshNode* entity = nullptr;

	FILE* const file = fopen(name, "rt");
	if (file == nullptr)
	{
		return entity;
	}
	
	//char token[256];
	//auto ReadToken = [fp, &token]()
	//{
	//	fscanf(fp, "%s", token);
	//};
	//
	//auto ReadFloat = [fp]()
	//{
	//	float value;
	//	fscanf(fp, "%f", &value);
	//	return value;
	//};
	//
	//auto ReadInt = [fp]()
	//{
	//	int value;
	//	fscanf(fp, "%d", &value);
	//	return value;
	//};
	//
	//int stack = 0;
	//exportMeshNode* stackPool[128];
	//std::vector<exportMeshNode*> nodeIndex;
	//
	//float scale = 1.0f;
	//if (fp)
	//{
	//	ReadToken();
	//	if (!strcmp(token, "HIERARCHY"))
	//	{
	//		while (!feof(fp))
	//		{
	//			ReadToken();
	//			if (!strcmp(token, "ROOT"))
	//			{
	//				entity = new exportMeshNode();
	//				nodeIndex.push_back(entity);
	//				ReadToken();
	//				entity->m_name = token;
	//				stackPool[stack] = entity;
	//				stack = 1;
	//				ReadToken();
	//			}
	//			else if (!strcmp(token, "JOINT"))
	//			{
	//				exportMeshNode* const node = new exportMeshNode(stackPool[stack - 1]);
	//				nodeIndex.push_back(node);
	//				ReadToken();
	//				if (node->m_parent->m_parent)
	//				{
	//					//strcat(token, "_bone");
	//				}
	//				else
	//				{
	//					//strcat(token, "_ref");
	//				}
	//				node->m_name = token;
	//				stackPool[stack] = node;
	//				stack++;
	//				ReadToken();
	//			}
	//			else if (!strcmp(token, "End"))
	//			{
	//				exportMeshNode* const node = new exportMeshNode(stackPool[stack - 1]);
	//				ReadToken();
	//				node->m_name = "effector";
	//				stackPool[stack] = node;
	//				stack++;
	//				ReadToken();
	//				//while (strcmp(token, "}")) ReadToken();
	//			}
	//			else if (!strcmp(token, "OFFSET"))
	//			{
	//				exportMeshNode* const node = stackPool[stack - 1];
	//				node->m_matrix.m_posit.m_x = ReadFloat() * scale;
	//				node->m_matrix.m_posit.m_y = ReadFloat() * scale;
	//				node->m_matrix.m_posit.m_z = ReadFloat() * scale;
	//				node->m_matrix.m_posit.m_w = 1.0f;
	//			}
	//			else if (!strcmp(token, "CHANNELS"))
	//			{
	//				int count = ReadInt();
	//				exportMeshNode* const node = stackPool[stack - 1];
	//				for (int i = 0; i < count; ++i)
	//				{
	//					ReadToken();
	//					if (!strcmp(token, "Xposition"))
	//					{
	//						node->m_animDof.m_channel[i] = 3;
	//					} 
	//					else if (!strcmp(token, "Yposition"))
	//					{
	//						node->m_animDof.m_channel[i] = 4;
	//					}
	//					else if (!strcmp(token, "Zposition"))
	//					{
	//						node->m_animDof.m_channel[i] = 5;
	//					}
	//					else if (!strcmp(token, "Xrotation"))
	//					{
	//						node->m_animDof.m_channel[i] = 0;
	//					}
	//					else if (!strcmp(token, "Yrotation"))
	//					{
	//						node->m_animDof.m_channel[i] = 1;
	//					}
	//					else if (!strcmp(token, "Zrotation"))
	//					{
	//						node->m_animDof.m_channel[i] = 2;
	//					}
	//				}
	//			}
	//			else if (!strcmp(token, "}"))
	//			{
	//				stack--;
	//			}
	//			else if (!strcmp(token, "MOTION"))
	//			{
	//				break;
	//			}
	//		}
	//	}
	//
	//	ReadToken();
	//	int framesCount = ReadInt();
	//	ReadToken();
	//	ReadToken();
	//	float frameTime = ReadFloat();
	//	
	//	//framesCount = 0;
	//	for (int i = 0; i < framesCount; ++i)
	//	{
	//		for (int j = 0; j < nodeIndex.size(); ++j)
	//		{
	//			float values[6];
	//			exportMeshNode* const node = nodeIndex[j];
	//			
	//			values[0] = 0.0f;
	//			values[1] = 0.0f;
	//			values[2] = 0.0f;
	//			values[3] = node->m_matrix.m_posit.m_x;
	//			values[4] = node->m_matrix.m_posit.m_y;
	//			values[5] = node->m_matrix.m_posit.m_z;
	//			for (int k = 0; k < 6; ++k)
	//			{
	//				int index = node->m_animDof.m_channel[k];
	//				if (index != -1)
	//				{
	//					float value = ReadFloat();
	//					values[index] = value;
	//				}
	//			}
	//
	//			exportMatrix pitch(ndPitchMatrix(values[0] * M_PI / 180.0f));
	//			exportMatrix yaw(ndYawMatrix(values[1] * M_PI / 180.0f));
	//			exportMatrix roll(ndRollMatrix(values[2] * M_PI / 180.0f));
	//			exportMatrix matrix(yaw * pitch * roll);
	//			matrix.m_posit = node->m_matrix.m_posit;
	//			if (node->m_parent == entity)
	//			{
	//				matrix.m_posit = exportVector(values[3], values[4], values[5], 1.0f);
	//			}
	//			node->m_keyFrame.push_back(matrix);
	//		}
	//	}
	//
	//	fclose(fp);
	//}
	//
	//entity->DeleteEffectors();
	////entity->GenerateTpose();
	//entity->AlignFrames();

	char token[256];
	auto ReadToken = [file, &token]()
	{
		fscanf(file, "%s", token);
	};

	fgets(token, sizeof(token) - 1, file);
	ReadToken();
	//ndTree<ndSharedPtr<ndMeshEffect>, int> meshEffects;
	if (!strcmp(token, "geometries:"))
	{
		int geometryCount;
		fscanf(file, "%d", &geometryCount);
		ReadToken();
		ReadToken();
	
		for (int k = 0; k < geometryCount; k++)
		{
			int meshId;
			fscanf(file, "%d", &meshId);
	//		ndSharedPtr<ndMeshEffect> effectMesh(new ndMeshEffect());
	//		meshEffects.Insert(effectMesh, meshId);
			ReadToken();
	
	//		ndArray<exportVector> positions;
	//		ndArray<ndMeshEffect::ndUV> uvs;
	//		ndArray<ndMeshEffect::ndNormal> normals;
	//		ndArray<ndMeshEffect::ndVertexWeight> vertexWeights;
	//
	//		ndArray<int> uvIndex;
	//		ndArray<int> faceArray;
	//		ndArray<int> indexArray;
	//		ndArray<int> normalsIndex;
	//		ndArray<int> materialArray;
	//		ndArray<int> positionsIndex;
	//
	//		ndMeshEffect::ndMeshVertexFormat format;
	//
			ReadToken();
			int vertexCount = 0;
			while (strcmp(token, "}"))
			{
				if (!strcmp(token, "vertex:"))
				{
					ReadToken();
					ReadToken();
					fscanf(file, "%d", &vertexCount);
					ReadToken();
					for (int i = 0; i < vertexCount; ++i)
					{
						double x;
						double y;
						double z;
						fscanf(file, "%lf %lf %lf", &x, &y, &z);
						//positions.PushBack(exportVector(x, y, z, double(0.0f)));
					}
					ReadToken();
	
					int indexCount;
					ReadToken();
					fscanf(file, "%d", &indexCount);
					ReadToken();
					for (int i = 0; i < indexCount; ++i)
					{
						int index;
						fscanf(file, "%d", &index);
	//					positionsIndex.PushBack(index);
					}
					ReadToken();
	
	//				format.m_vertex.m_data = &positions[0].m_x;
	//				format.m_vertex.m_indexList = &positionsIndex[0];
	//				format.m_vertex.m_strideInBytes = sizeof(exportVector);
					ReadToken();
				}
				else if (!strcmp(token, "normal:"))
				{
					int vCount;
					ReadToken();
					ReadToken();
					fscanf(file, "%d", &vCount);
	
					ReadToken();
					for (int i = 0; i < vCount; ++i)
					{
						float x;
						float y;
						float z;
						fscanf(file, "%f %f %f", &x, &y, &z);
	//					normals.PushBack(ndMeshEffect::ndNormal(x, y, z));
					}
					ReadToken();
	
					int indexCount;
					ReadToken();
					fscanf(file, "%d", &indexCount);
					ReadToken();
					for (int i = 0; i < indexCount; ++i)
					{
						int index;
						fscanf(file, "%d", &index);
	//					normalsIndex.PushBack(index);
					}
					ReadToken();
	
	//				format.m_normal.m_data = &normals[0].m_x;
	//				format.m_normal.m_indexList = &normalsIndex[0];
	//				format.m_normal.m_strideInBytes = sizeof(ndMeshEffect::ndNormal);
					ReadToken();
				}
				else if (!strcmp(token, "uv:"))
				{
					int vCount;
					ReadToken();
					ReadToken();
					fscanf(file, "%d", &vCount);
	
					ReadToken();
					for (int i = 0; i < vCount; ++i)
					{
						float x;
						float y;
						fscanf(file, "%f %f", &x, &y);
	//					uvs.PushBack(ndMeshEffect::ndUV(x, y));
					}
					ReadToken();
	
					int indexCount;
					ReadToken();
					fscanf(file, "%d", &indexCount);
					ReadToken();
					for (int i = 0; i < indexCount; ++i)
					{
						int index;
						fscanf(file, "%d", &index);
	//					uvIndex.PushBack(index);
					}
					ReadToken();
	//
	//				format.m_uv0.m_data = &uvs[0].m_u;
	//				format.m_uv0.m_indexList = &uvIndex[0];
	//				format.m_uv0.m_strideInBytes = sizeof(ndMeshEffect::ndUV);
					ReadToken();
				}
				else if (!strcmp(token, "material:"))
				{
					exportVector val;
	//				ndMeshEffect::ndMaterial material;
	
					ReadToken();
					ReadToken();
					fscanf(file, "%f %f %f %f", &val.m_x, &val.m_y, &val.m_z, &val.m_w);
	//				material.m_ambient = ndVector(val);
	
					ReadToken();
					fscanf(file, "%f %f %f %f", &val.m_x, &val.m_y, &val.m_z, &val.m_w);
	//				material.m_diffuse = ndVector(val);
	//
					ReadToken();
					fscanf(file, "%f %f %f %f", &val.m_x, &val.m_y, &val.m_z, &val.m_w);
	//				material.m_specular = ndVector(val);
	//
					ReadToken();
					fscanf(file, "%f", &val.m_x);
	//				material.m_opacity = ndFloat32(val.m_x);
	//
					ReadToken();
					fscanf(file, "%f", &val.m_x);
	//				material.m_shiness = ndFloat32(val.m_x);
	//
					ReadToken();
					ReadToken();
	//				strcpy(material.m_textureName, token);
	//
	//				ndArray<ndMeshEffect::ndMaterial>& materials = effectMesh->GetMaterials();
	//				int materialIndex = materials.GetCount();
	//
					ReadToken();
					int faceCount;
					fscanf(file, "%d", &faceCount);
	
					ReadToken();
					for (int i = 0; i < faceCount; ++i)
					{
						int vCount;
						fscanf(file, "%d:", &vCount);
						for (int j = 0; j < vCount; ++j)
						{
							int index;
							fscanf(file, "%d ", &index);
	//						indexArray.PushBack(index);
						}
	//					faceArray.PushBack(vCount);
	//					materialArray.PushBack(materialIndex);
					}
					ReadToken();
					ReadToken();
	//				materials.PushBack(material);
				}
				else if (!strcmp(token, "vertexWeightsCluster:"))
				{
					_ASSERT(0);
	//				if (!vertexWeights.GetCount())
	//				{
	//					vertexWeights.SetCount(vertexCount);
	//					for (int i = 0; i < vertexWeights.GetCount(); ++i)
	//					{
	//						vertexWeights[i].Clear();
	//					}
	//					format.m_vertexWeight.m_data = &vertexWeights[0];
	//					format.m_vertexWeight.m_indexList = &positionsIndex[0];
	//					format.m_vertexWeight.m_strideInBytes = sizeof(ndMeshEffect::ndVertexWeight);
	//				}
	//
	//				char boneName[128];
	//				fscanf(file, "%s", boneName);
	//				int indexCount;
	//				ReadToken();
	//				ReadToken();
	//				fscanf(file, "%d", &indexCount);
	//
	//				int hashId = int(ndCRC64(boneName) & 0xffffffff);
	//				ReadToken();
	//
	//				ndArray<int> boneIndex;
	//				for (int i = 0; i < indexCount; ++i)
	//				{
	//					int index;
	//					fscanf(file, "%d", &index);
	//					boneIndex.PushBack(index);
	//				}
	//
	//				ReadToken();
	//				ndArray<float> vertexWeight;
	//				for (int i = 0; i < indexCount; ++i)
	//				{
	//					float weight;
	//					fscanf(file, "%f", &weight);
	//					vertexWeight.PushBack(weight);
	//				}
	//				ReadToken();
	//
	//				for (int i = 0; i < indexCount; ++i)
	//				{
	//					int index = boneIndex[i];
	//					vertexWeights[index].SetWeight(hashId, float(vertexWeight[i]));
	//				}
				}
				ReadToken();
			}
			ReadToken();
	
	//		format.m_faceCount = faceArray.GetCount();
	//		format.m_faceIndexCount = &faceArray[0];
	//		format.m_faceMaterial = &materialArray[0];
	//		effectMesh->BuildFromIndexList(&format);
		}
		ReadToken();
	}
	
	if (!strcmp(token, "node:"))
	{
	//	root = new ndMesh(nullptr);
	//	root->Load(file, meshEffects);
	}

	fclose(file);
	return entity;
}

