#include "stdafx.h"
#include "exportMesh.h"
#include "exportMeshNode.h"

void exportMeshNode::SetFrame(int index)
{
	_ASSERT(0);
	//int stack = 1;
	//exportMeshNode* stackPool[128];
	//
	//stack = 1;
	//stackPool[0] = this;
	//while (stack)
	//{
	//	stack--;
	//
	//	exportMeshNode* const node = stackPool[stack];
	//	node->m_matrix = node->m_keyFrame[index];
	//
	//	for (std::list<exportMeshNode*>::const_iterator iter = node->m_children.begin();
	//		iter != node->m_children.end(); iter++)
	//	{
	//		stackPool[stack] = *iter;
	//		stack++;
	//	}
	//}
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

		_ASSERT(0);
		//if (node->m_keyFrame.size())
		//{
		//	exportMatrix childAlign;
		//	if (node->m_children.size())
		//	{
		//		exportMeshNode* const childNode = *node->m_children.begin();
		//		childAlign = exportMatrix(childNode->m_matrix.m_posit);
		//	}
		//
		//	for (int i = 0; i < node->m_keyFrame.size(); ++i)
		//	{
		//		exportMatrix animMatrix(node->m_keyFrame[i]);
		//		exportMatrix matrix(childAlign * animMatrix * parentAlign);
		//		node->m_keyFrame[i] = matrix;
		//	}
		//	parentAlign = childAlign.Inverse();
		//}
		//
		//for (std::list<exportMeshNode*>::const_iterator iter = node->m_children.begin();
		//	iter != node->m_children.end(); iter++)
		//{
		//	stackPool[stack] = *iter;
		//	alignFrame[stack] = parentAlign;
		//	stack++;
		//}
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

	char token[256];
	auto ReadToken = [file, &token]()
	{
		fscanf(file, "%s", token);
	};

	fgets(token, sizeof(token) - 1, file);
	ReadToken();
	std::map <int, std::shared_ptr<exportMesh>> meshEffects;
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
			std::shared_ptr<exportMesh> effectMesh(new exportMesh());
			meshEffects[meshId] = effectMesh;
			ReadToken();
	
	//		std::vector<ndMeshEffect::ndVertexWeight> vertexWeights;

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
						float x;
						float y;
						float z;
						fscanf(file, "%f %f %f", &x, &y, &z);
						effectMesh->m_positions.push_back(exportVector(x, y, z, float(0.0f)));
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
						effectMesh->m_positionsIndex.push_back(index);
					}
					ReadToken();
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
						effectMesh->m_normals.push_back(exportVector(x, y, z, float(0.0f)));
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
						effectMesh->m_normalsIndex.push_back(index);
					}
					ReadToken();
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
						effectMesh->m_uvs.push_back(exportVector(x, y, float(0.0f), float(0.0f)));
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
						effectMesh->m_uvIndex.push_back(index);
					}
					ReadToken();
					ReadToken();
				}
				else if (!strcmp(token, "material:"))
				{
					exportVector val;
					exportMaterial material;
	
					ReadToken();
					ReadToken();
					fscanf(file, "%f %f %f %f", &val.m_x, &val.m_y, &val.m_z, &val.m_w);
					material.m_ambient = val;
	
					ReadToken();
					fscanf(file, "%f %f %f %f", &val.m_x, &val.m_y, &val.m_z, &val.m_w);
					material.m_diffuse = val;
	
					ReadToken();
					fscanf(file, "%f %f %f %f", &val.m_x, &val.m_y, &val.m_z, &val.m_w);
					material.m_specular = val;
	
					ReadToken();
					fscanf(file, "%f", &val.m_x);
					material.m_opacity = val.m_x;
	
					ReadToken();
					fscanf(file, "%f", &val.m_x);
					material.m_shiness = val.m_x;
	
					ReadToken();
					ReadToken();
					material.m_texName = token;
	
					ReadToken();
					int faceCount;
					fscanf(file, "%d", &faceCount);
					ReadToken();
					for (int i = 0; i < faceCount; ++i)
					{
						int vCount;
						fscanf(file, "%d:", &vCount);
						material.m_faceIndexCount.push_back(vCount);
					}
					ReadToken();

					ReadToken();
					int indexCount;
					fscanf(file, "%d", &indexCount);
					ReadToken();
					for (int i = 0; i < indexCount; ++i)
					{
						int index;
						fscanf(file, "%d:", &index);
						material.m_indexArray.push_back(index);
					}
					ReadToken();
					ReadToken();
					effectMesh->m_materials.push_back(material);
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
	//				std::vector<int> boneIndex;
	//				for (int i = 0; i < indexCount; ++i)
	//				{
	//					int index;
	//					fscanf(file, "%d", &index);
	//					boneIndex.push_back(index);
	//				}
	//
	//				ReadToken();
	//				std::vector<float> vertexWeight;
	//				for (int i = 0; i < indexCount; ++i)
	//				{
	//					float weight;
	//					fscanf(file, "%f", &weight);
	//					vertexWeight.push_back(weight);
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
		}
		ReadToken();
	}
	
	if (!strcmp(token, "node:"))
	{
		entity = new exportMeshNode();
		entity->LoadNdmSkeleton(file, meshEffects);
	}

	fclose(file);
	return entity;
}


void exportMeshNode::LoadNdmSkeleton(FILE* const file, std::map <int, std::shared_ptr<exportMesh>>& meshEffects)
{
	char token[256];
	auto ReadToken = [file, &token]()
	{
		fscanf(file, "%s", token);
	};

	ReadToken();
	ReadToken();
	while (strcmp(token, "}"))
	{
		if (!strcmp(token, "name:"))
		{
			ReadToken();
			m_name = token;
		}
		else if (!strcmp(token, "eulers:"))
		{
			exportVector eulers;
			fscanf(file, "%f %f %f", &eulers.m_x, &eulers.m_y, &eulers.m_z);
			exportMatrix matrix(ndPitchMatrix(eulers.m_x * ndDegreeToRad) * ndYawMatrix(eulers.m_y * ndDegreeToRad) * ndRollMatrix(eulers.m_z * ndDegreeToRad));
			matrix.m_posit = m_matrix.m_posit;
			m_matrix = matrix;
		}
		else if (!strcmp(token, "position:"))
		{
			exportVector posit;
			fscanf(file, "%f %f %f", &posit.m_x, &posit.m_y, &posit.m_z);
			posit.m_w = 1.0f;
			m_matrix.m_posit = posit;
		}
		else if (!strcmp(token, "geometryEulers:"))
		{
			exportVector eulers;
			fscanf(file, "%f %f %f", &eulers.m_x, &eulers.m_y, &eulers.m_z);
			exportMatrix matrix(ndPitchMatrix(eulers.m_x * ndDegreeToRad) * ndYawMatrix(eulers.m_y * ndDegreeToRad) * ndRollMatrix(eulers.m_z * ndDegreeToRad));
			matrix.m_posit = m_meshMatrix.m_posit;
			m_meshMatrix = matrix;
		}
		else if (!strcmp(token, "geometryPosition:"))
		{
			exportVector posit;
			fscanf(file, "%f %f %f", &posit.m_x, &posit.m_y, &posit.m_z);
			posit.m_w = 1.0f;
			m_meshMatrix.m_posit = posit;
		}
		else if (!strcmp(token, "geometry:"))
		{
			int nodeId;
			fscanf(file, "%d", &nodeId);
			_ASSERT(meshEffects.find(nodeId) != meshEffects.end());
			//m_mesh = meshEffects.Find(nodeId)->GetInfo();
			m_mesh = meshEffects[nodeId];
		}
		else if (!strcmp(token, "keyFramePosits:"))
		{
			int keyFramesCount;
			fscanf(file, "%d\n", &keyFramesCount);
		
			ReadToken();
			for (int i = 0; i < keyFramesCount; ++i)
			{
				exportVector keyframe;
				fscanf(file, "%f %f %f %f\n", &keyframe.m_x, &keyframe.m_y, &keyframe.m_z, &keyframe.m_w);
				m_positionCurve.push_back(keyframe);
			}
			ReadToken();
		}
		else if (!strcmp(token, "keyFrameRotations:"))
		{
			int keyFramesCount;
			fscanf(file, "%d\n", &keyFramesCount);
		
			ReadToken();
			for (int i = 0; i < keyFramesCount; ++i)
			{
				exportVector keyframe;
				fscanf(file, "%f %f %f %f\n", &keyframe.m_x, &keyframe.m_y, &keyframe.m_z, &keyframe.m_w);
				keyframe.m_x = keyframe.m_x * ndDegreeToRad;
				keyframe.m_y = keyframe.m_y * ndDegreeToRad;
				keyframe.m_z = keyframe.m_z * ndDegreeToRad;
				m_rotationCurve.push_back(keyframe);
			}
			ReadToken();
		}
		else if (!strcmp(token, "node:"))
		{
			exportMeshNode* const child = new exportMeshNode(this);
			child->LoadNdmSkeleton(file, meshEffects);
		}
		else
		{
			break;
		}
		ReadToken();
	}

	//entity->DeleteEffectors();
	////entity->GenerateTpose();
	//entity->AlignFrames();

}
