/* Copyright (c) <2003-2022> <Julio Jerez, Newton Game Dynamics>
*
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
*
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
*
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
*
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
*
* 3. This notice may not be removed or altered from any source distribution.
*/

#include "ndModelStdafx.h"
#include "ndMesh.h"
#include "ndMeshFile.h"

ndMeshFile::ndMeshFile()
	:ndClassAlloc()
{
}

ndMeshFile::~ndMeshFile()
{
}

//void ndMeshFile::Export(const ndMesh* const mesh, const char* const fullPathName)
void ndMeshFile::Export(const ndMesh* const, const char* const)
{
	ndAssert(0);
	//FILE* const file = fopen(fullPathName, "wb");
	//if (file)
	//{
	//	fprintf(file, "# Newton Dynamics Mesh file format.\n\n");
	//	ndTree<ndInt32, const ndMeshEffect*> meshEffects;
	//	for (ndMesh* node = mesh->GetFirstIterator(); node; node = node->GetNextIterator())
	//	{
	//		if (node->m_mesh)
	//		{
	//			meshEffects.Insert(meshEffects.GetCount(), *node->m_mesh);
	//		}
	//	}
	//
	//	if (meshEffects.GetCount())
	//	{
	//		auto PrintVertexChannel = [file](const ndMeshEffect::ndMeshVertexFormat::ndData<ndFloat64>& channel, ndInt32 vertexCount)
	//		{
	//			fprintf(file, "\t\tvertex:\n");
	//			fprintf(file, "\t\t{\n");
	//
	//			ndInt32 positCount = 0;
	//			for (ndInt32 i = 0; i < vertexCount; ++i)
	//			{
	//				positCount = ndMax(positCount, channel.m_indexList[i] + 1);
	//			}
	//
	//			fprintf(file, "\t\t\tposition: %d\n", positCount);
	//			fprintf(file, "\t\t\t{\n");
	//			ndInt32 stride = channel.m_strideInBytes / ndInt32(sizeof(ndFloat64));
	//			for (ndInt32 i = 0; i < positCount; ++i)
	//			{
	//				ndFloat64 x;
	//				ndFloat64 y;
	//				ndFloat64 z;
	//				x = channel.m_data[i * stride + 0];
	//				y = channel.m_data[i * stride + 1];
	//				z = channel.m_data[i * stride + 2];
	//				fprintf(file, "\t\t\t\t%lg %lg %lg\n", x, y, z);
	//			}
	//			fprintf(file, "\t\t\t}\n");
	//
	//			fprintf(file, "\t\t\tindices: %d\n", vertexCount);
	//			fprintf(file, "\t\t\t{\n");
	//			fprintf(file, "\t\t\t\t");
	//			for (ndInt32 i = 0; i < vertexCount; ++i)
	//			{
	//				fprintf(file, "%d ", channel.m_indexList[i]);
	//			}
	//			fprintf(file, "\n");
	//			fprintf(file, "\t\t\t}\n");
	//
	//			fprintf(file, "\t\t}\n");
	//		};
	//
	//		auto PrintNormalChannel = [file](const ndMeshEffect::ndMeshVertexFormat::ndData<ndReal>& channel, ndInt32 vertexCount)
	//		{
	//			fprintf(file, "\t\tnormal:\n");
	//			fprintf(file, "\t\t{\n");
	//
	//			ndInt32 positCount = 0;
	//			for (ndInt32 i = 0; i < vertexCount; ++i)
	//			{
	//				positCount = ndMax(positCount, channel.m_indexList[i] + 1);
	//			}
	//
	//			fprintf(file, "\t\t\tposition: %d\n", positCount);
	//			fprintf(file, "\t\t\t{\n");
	//			ndInt32 stride = channel.m_strideInBytes / ndInt32(sizeof(ndReal));
	//			for (ndInt32 i = 0; i < positCount; ++i)
	//			{
	//				ndReal x;
	//				ndReal y;
	//				ndReal z;
	//				x = channel.m_data[i * stride + 0];
	//				y = channel.m_data[i * stride + 1];
	//				z = channel.m_data[i * stride + 2];
	//				fprintf(file, "\t\t\t\t%g %g %g\n", x, y, z);
	//			}
	//			fprintf(file, "\t\t\t}\n");
	//
	//			fprintf(file, "\t\t\tindices: %d\n", vertexCount);
	//			fprintf(file, "\t\t\t{\n");
	//			fprintf(file, "\t\t\t\t");
	//			for (ndInt32 i = 0; i < vertexCount; ++i)
	//			{
	//				fprintf(file, "%d ", channel.m_indexList[i]);
	//			}
	//			fprintf(file, "\n");
	//			fprintf(file, "\t\t\t}\n");
	//
	//			fprintf(file, "\t\t}\n");
	//		};
	//
	//		auto PrintUVChannel = [file](const ndMeshEffect::ndMeshVertexFormat::ndData<ndReal>& channel, ndInt32 vertexCount)
	//		{
	//			fprintf(file, "\t\tuv:\n");
	//			fprintf(file, "\t\t{\n");
	//
	//			ndInt32 positCount = 0;
	//			for (ndInt32 i = 0; i < vertexCount; ++i)
	//			{
	//				positCount = ndMax(positCount, channel.m_indexList[i] + 1);
	//			}
	//
	//			fprintf(file, "\t\t\tposition: %d\n", positCount);
	//			fprintf(file, "\t\t\t{\n");
	//			ndInt32 stride = channel.m_strideInBytes / ndInt32(sizeof(ndReal));
	//			for (ndInt32 i = 0; i < positCount; ++i)
	//			{
	//				ndReal x;
	//				ndReal y;
	//				x = channel.m_data[i * stride + 0];
	//				y = channel.m_data[i * stride + 1];
	//				fprintf(file, "\t\t\t\t%g %g\n", x, y);
	//			}
	//			fprintf(file, "\t\t\t}\n");
	//
	//			fprintf(file, "\t\t\tindices: %d\n", vertexCount);
	//			fprintf(file, "\t\t\t{\n");
	//			fprintf(file, "\t\t\t\t");
	//			for (ndInt32 i = 0; i < vertexCount; ++i)
	//			{
	//				fprintf(file, "%d ", channel.m_indexList[i]);
	//			}
	//			fprintf(file, "\n");
	//			fprintf(file, "\t\t\t}\n");
	//
	//			fprintf(file, "\t\t}\n");
	//		};
	//
	//		fprintf(file, "geometries: %d\n", meshEffects.GetCount());
	//		fprintf(file, "{\n");
	//
	//		ndTree<ndInt32, const ndMeshEffect*>::Iterator it(meshEffects);
	//		for (it.Begin(); it; it++)
	//		{
	//			fprintf(file, "\tgeometry: %d\n", it.GetNode()->GetInfo());
	//			fprintf(file, "\t{\n");
	//
	//			ndMeshEffect* effectMesh = (ndMeshEffect*)it.GetKey();
	//
	//			ndArray<ndUnsigned8> tmpBuffer;
	//			ndMeshEffect::ndMeshVertexFormat format;
	//			ndInt32 vertexCount = effectMesh->GenerateVertexFormat(format, tmpBuffer);
	//			PrintVertexChannel(format.m_vertex, vertexCount);
	//			if (format.m_normal.m_data)
	//			{
	//				PrintNormalChannel(format.m_normal, vertexCount);
	//			}
	//			if (format.m_uv0.m_data)
	//			{
	//				PrintUVChannel(format.m_uv0, vertexCount);
	//			}
	//
	//			const ndArray<ndMeshEffect::ndMaterial>& materialArray = effectMesh->GetMaterials();
	//			for (ndInt32 i = 0; i < materialArray.GetCount(); ++i)
	//			{
	//				fprintf(file, "\t\tmaterial:\n");
	//				fprintf(file, "\t\t{\n");
	//			
	//				const ndMeshEffect::ndMaterial& material = materialArray[i];
	//			
	//				fprintf(file, "\t\t\tambience: %g %g %g %g\n", material.m_ambient.m_x, material.m_ambient.m_y, material.m_ambient.m_z, material.m_ambient.m_w);
	//				fprintf(file, "\t\t\tdiffuse: %g %g %g %g\n", material.m_diffuse.m_x, material.m_diffuse.m_y, material.m_diffuse.m_z, material.m_diffuse.m_w);
	//				fprintf(file, "\t\t\tm_specular: %g %g %g %g\n", material.m_specular.m_x, material.m_specular.m_y, material.m_specular.m_z, material.m_specular.m_w);
	//				fprintf(file, "\t\t\topacity: %g\n", material.m_opacity);
	//				fprintf(file, "\t\t\tshiness: %g\n", material.m_shiness);
	//				fprintf(file, "\t\t\ttexture: %s\n", material.m_textureName);
	//
	//				ndInt32 faceCount = 0;
	//				for (ndInt32 j = 0; j < format.m_faceCount; ++j)
	//				{
	//					if (format.m_faceMaterial[j] == i)
	//					{
	//						faceCount++;
	//					}
	//				}
	//			
	//				ndInt32 indexAcc = 0;
	//				fprintf(file, "\t\t\tfacesVertexCount: %d\n", faceCount);
	//				fprintf(file, "\t\t\t{\n");
	//				fprintf(file, "\t\t\t\t");
	//				for (ndInt32 j = 0; j < format.m_faceCount; ++j)
	//				{
	//					if (format.m_faceMaterial[j] == i)
	//					{
	//						fprintf(file, "%d ", format.m_faceIndexCount[j]);
	//						indexAcc += format.m_faceIndexCount[j];
	//					}
	//				}
	//				fprintf(file, "\n");
	//				fprintf(file, "\t\t\t}\n");
	//
	//				fprintf(file, "\t\t\tfacesIndexList: %d\n", indexAcc);
	//				fprintf(file, "\t\t\t{\n");
	//				fprintf(file, "\t\t\t\t");
	//				indexAcc = 0;
	//				for (ndInt32 j = 0; j < format.m_faceCount; ++j)
	//				{
	//					if (format.m_faceMaterial[j] == i)
	//					{
	//						for (ndInt32 k = 0; k < format.m_faceIndexCount[j]; ++k)
	//						{
	//							fprintf(file, "%d ", indexAcc + k);
	//						}
	//					}
	//					indexAcc += format.m_faceIndexCount[j];
	//				}
	//				fprintf(file, "\n");
	//				fprintf(file, "\t\t\t}\n");
	//
	//
	//				fprintf(file, "\t\t}\n");
	//			}
	//
	//			if (format.m_vertexWeight.m_data)
	//			{
	//				ndInt32 weightsCount = 0;
	//				for (ndInt32 i = 0; i < vertexCount; ++i)
	//				{
	//					weightsCount = ndMax(weightsCount, format.m_vertexWeight.m_indexList[i] + 1);
	//				}
	//
	//				for (ndMesh* node = mesh->GetFirstIterator(); node; node = node->GetNextIterator())
	//				{
	//					ndInt32 count = 0;
	//					ndInt32 hash = ndInt32(ndCRC64(node->m_name.GetStr()) & 0xffffffff);
	//					ndAssert(hash);
	//					for (ndInt32 j = 0; j < weightsCount; ++j)
	//					{
	//						for (ndInt32 k = 0; k < ND_VERTEX_WEIGHT_SIZE; ++k)
	//						{
	//							count += (format.m_vertexWeight.m_data[j].m_boneId[k] == hash) ? 1 : 0;
	//						}
	//					}
	//					if (count)
	//					{
	//						fprintf(file, "\t\tvertexWeightsCluster: %s\n", node->m_name.GetStr());
	//						fprintf(file, "\t\t{\n");
	//						
	//						fprintf(file, "\t\t\tindexCount: %d\n", count);
	//						
	//						fprintf(file, "\t\t\tvertexIndex: ");
	//						for (ndInt32 j = 0; j < weightsCount; ++j)
	//						{
	//							for (ndInt32 k = 0; k < ND_VERTEX_WEIGHT_SIZE; ++k)
	//							{
	//								if (format.m_vertexWeight.m_data[j].m_boneId[k] == hash)
	//								{
	//									fprintf(file, "%d ", j);
	//								}
	//							}
	//						}
	//						fprintf(file, "\t\t\t\n");
	//
	//						fprintf(file, "\t\t\tvertexWeight: ");
	//						for (ndInt32 j = 0; j < weightsCount; ++j)
	//						{
	//							for (ndInt32 k = 0; k < ND_VERTEX_WEIGHT_SIZE; ++k)
	//							{
	//								if (format.m_vertexWeight.m_data[j].m_boneId[k] == hash)
	//								{
	//									fprintf(file, "%g ", format.m_vertexWeight.m_data[j].m_weight[k]);
	//								}
	//							}
	//						}
	//						fprintf(file, "\t\t\t\n");
	//						
	//						fprintf(file, "\t\t}\n");
	//
	//					}
	//				}
	//			}
	//			
	//			fprintf(file, "\t}\n");
	//		}
	//
	//		fprintf(file, "}\n");
	//		fprintf(file, "\n");
	//	}
	//
	//	mesh->Save(file, meshEffects);
	//	fclose(file);
	//}
}

//void ndMeshFile::Save(FILE* const file, const ndTree<ndInt32, const ndMeshEffect*>& meshEffects, ndInt32 level) const
//{
//	auto PrintTabs = [file](ndInt32 level)
//	{
//		for (ndInt32 i = 0; i < level; i++)
//		{
//			fprintf(file, "\t");
//		}
//	};
//
//	PrintTabs(level);
//	fprintf(file, "node:\n");
//
//	PrintTabs(level);
//	fprintf(file, "{\n");
//
//	PrintTabs(level);
//	fprintf(file, "\tname: %s\n", GetName().GetStr());
//
//	ndVector euler1;
//	ndVector euler(m_matrix.CalcPitchYawRoll(euler1).Scale(ndRadToDegree));
//	PrintTabs(level);
//	fprintf(file, "\teulers: %g %g %g\n", euler.m_x, euler.m_y, euler.m_z);
//	PrintTabs(level);
//	fprintf(file, "\tposition: %g %g %g\n", m_matrix.m_posit.m_x, m_matrix.m_posit.m_y, m_matrix.m_posit.m_z);
//
//	PrintTabs(level);
//	euler = m_meshMatrix.CalcPitchYawRoll(euler1).Scale(ndRadToDegree);
//	fprintf(file, "\tgeometryEulers: %g %g %g\n", euler.m_x, euler.m_y, euler.m_z);
//	PrintTabs(level);
//	fprintf(file, "\tgeometryPosition: %g %g %g\n", m_meshMatrix.m_posit.m_x, m_meshMatrix.m_posit.m_y, m_meshMatrix.m_posit.m_z);
//
//	if (m_mesh)
//	{
//		ndTree<ndInt32, const ndMeshEffect*>::ndNode* const meshNode = meshEffects.Find(*m_mesh);
//		if (meshNode)
//		{
//			PrintTabs(level);
//			fprintf(file, "\tgeometry: %d\n", meshNode->GetInfo());
//		}
//	}
//
//	const ndMesh::ndCurve& positCurve = GetPositCurve();
//	if (positCurve.GetCount())
//	{
//		PrintTabs(level);
//		fprintf(file, "\tkeyFramePosits: %d\n", positCurve.GetCount());
//
//		PrintTabs(level);
//		fprintf(file, "\t{\n");
//		for (ndCurve::ndNode* keyFrameNode = positCurve.GetFirst(); keyFrameNode; keyFrameNode = keyFrameNode->GetNext())
//		{
//			const ndCurveValue& keyframe = keyFrameNode->GetInfo();
//			PrintTabs(level);
//			fprintf(file, "\t\t%g %g %g %g\n", keyframe.m_x, keyframe.m_y, keyframe.m_z, keyframe.m_time);
//		}
//		PrintTabs(level);
//		fprintf(file, "\t}\n");
//	}
//
//	const ndMesh::ndCurve& rotationCurve = GetRotationCurve();
//	if (rotationCurve.GetCount())
//	{
//		PrintTabs(level);
//		fprintf(file, "\tkeyFrameRotations: %d\n", rotationCurve.GetCount());
//
//		PrintTabs(level);
//		fprintf(file, "\t{\n");
//		for (ndCurve::ndNode* keyFrameNode = rotationCurve.GetFirst(); keyFrameNode; keyFrameNode = keyFrameNode->GetNext())
//		{
//			const ndCurveValue& keyframe = keyFrameNode->GetInfo();
//			PrintTabs(level);
//			ndReal pitch = ndReal(keyframe.m_x * ndRadToDegree);
//			ndReal yaw = ndReal(keyframe.m_y * ndRadToDegree);
//			ndReal roll = ndReal(keyframe.m_z * ndRadToDegree);
//			fprintf(file, "\t\t%g %g %g %g\n", pitch, yaw, roll, keyframe.m_time);
//		}
//		PrintTabs(level);
//		fprintf(file, "\t}\n");
//	}
//
//	for (ndMesh* child = GetFirstChild(); child; child = child->GetNext())
//	{
//		child->Save(file, meshEffects, level + 1);
//	}
//
//	PrintTabs(level);
//	fprintf(file, "}\n");
//}

//ndMesh* ndMeshFile::Import(const char* const fullPathName)
ndMesh* ndMeshFile::Import(const char* const)
{
	ndAssert(0);
	return nullptr;
	//ndMesh* root = nullptr;
	//FILE* const file = fopen(fullPathName, "rb");
	//if (file)
	//{
	//	char token[256];
	//	auto ReadToken = [file, &token]()
	//	{
	//		ndInt32 error = 0;
	//		error = fscanf(file, "%s", token);
	//	};
	//
	//	ndInt32 error = 0;
	//	char* error1 = nullptr;
	//	error1 = fgets(token, sizeof(token) - 1, file);
	//	ReadToken();
	//	ndTree<ndSharedPtr<ndMeshEffect>, ndInt32> meshEffects;
	//	if (!strcmp(token, "geometries:"))
	//	{
	//		ndInt32 geometryCount;
	//		error = fscanf(file, "%d", &geometryCount);
	//		ReadToken();
	//		ReadToken();
	//
	//		for (ndInt32 k = 0; k < geometryCount; k++)
	//		{
	//			ndInt32 meshId;
	//			error = fscanf(file, "%d", &meshId);
	//			ndSharedPtr<ndMeshEffect> effectMesh(new ndMeshEffect());
	//			meshEffects.Insert(effectMesh, meshId);
	//			ReadToken();
	//			
	//			ndArray<ndBigVector> positions;
	//			ndArray<ndMeshEffect::ndUV> uvs;
	//			ndArray<ndMeshEffect::ndNormal> normals;
	//			ndArray<ndMeshEffect::ndVertexWeight> vertexWeights;
	//
	//			ndArray<ndInt32> uvIndex;
	//			ndArray<ndInt32> faceArray;
	//			ndArray<ndInt32> indexArray;
	//			ndArray<ndInt32> normalsIndex;
	//			ndArray<ndInt32> materialArray;
	//			ndArray<ndInt32> positionsIndex;
	//			
	//			ndMeshEffect::ndMeshVertexFormat format;
	//
	//			ReadToken();
	//			ndInt32 vertexCount = 0;
	//			while (strcmp(token, "}"))
	//			{
	//				if (!strcmp(token, "vertex:"))
	//				{
	//					ReadToken();
	//					ReadToken();
	//					error = fscanf(file, "%d", &vertexCount);
	//					ReadToken();
	//					for (ndInt32 i = 0; i < vertexCount; ++i)
	//					{
	//						ndFloat64 x;
	//						ndFloat64 y;
	//						ndFloat64 z;
	//						error = fscanf(file, "%lf %lf %lf", &x, &y, &z);
	//						positions.PushBack(ndBigVector(x, y, z, ndFloat64(0.0f)));
	//					}
	//					ReadToken();
	//
	//					int indexCount;
	//					ReadToken();
	//					error = fscanf(file, "%d", &indexCount);
	//					ReadToken();
	//					for (ndInt32 i = 0; i < indexCount; ++i)
	//					{
	//						ndInt32 index;
	//						error = fscanf(file, "%d" , &index);
	//						positionsIndex.PushBack(index);
	//					}
	//					ReadToken();
	//
	//					format.m_vertex.m_data = &positions[0].m_x;
	//					format.m_vertex.m_indexList = &positionsIndex[0];
	//					format.m_vertex.m_strideInBytes = sizeof(ndBigVector);
	//					ReadToken();
	//				}
	//				else if (!strcmp(token, "normal:"))
	//				{
	//					ndInt32 vCount;
	//					ReadToken();
	//					ReadToken();
	//					error = fscanf(file, "%d", &vCount);
	//
	//					ReadToken();
	//					for (ndInt32 i = 0; i < vCount; ++i)
	//					{
	//						ndReal x;
	//						ndReal y;
	//						ndReal z;
	//						error = fscanf(file, "%f %f %f", &x, &y, &z);
	//						normals.PushBack(ndMeshEffect::ndNormal(x, y, z));
	//					}
	//					ReadToken();
	//
	//					int indexCount;
	//					ReadToken();
	//					error = fscanf(file, "%d", &indexCount);
	//					ReadToken();
	//					for (ndInt32 i = 0; i < indexCount; ++i)
	//					{
	//						ndInt32 index;
	//						error = fscanf(file, "%d", &index);
	//						normalsIndex.PushBack(index);
	//					}
	//					ReadToken();
	//
	//					format.m_normal.m_data = &normals[0].m_x;
	//					format.m_normal.m_indexList = &normalsIndex[0];
	//					format.m_normal.m_strideInBytes = sizeof(ndMeshEffect::ndNormal);
	//					ReadToken();
	//				}
	//				else if (!strcmp(token, "uv:"))
	//				{
	//					ndInt32 vCount;
	//					ReadToken();
	//					ReadToken();
	//					error = fscanf(file, "%d", &vCount);
	//
	//					ReadToken();
	//					for (ndInt32 i = 0; i < vCount; ++i)
	//					{
	//						ndReal x;
	//						ndReal y;
	//						error = fscanf(file, "%f %f", &x, &y);
	//						uvs.PushBack(ndMeshEffect::ndUV(x, y));
	//					}
	//					ReadToken();
	//
	//					int indexCount;
	//					ReadToken();
	//					error = fscanf(file, "%d", &indexCount);
	//					ReadToken();
	//					for (ndInt32 i = 0; i < indexCount; ++i)
	//					{
	//						ndInt32 index;
	//						error = fscanf(file, "%d", &index);
	//						uvIndex.PushBack(index);
	//					}
	//					ReadToken();
	//
	//					format.m_uv0.m_data = &uvs[0].m_u;
	//					format.m_uv0.m_indexList = &uvIndex[0];
	//					format.m_uv0.m_strideInBytes = sizeof(ndMeshEffect::ndUV);
	//					ReadToken();
	//				}
	//				else if (!strcmp(token, "material:"))
	//				{
	//					ndBigVector val;
	//					ndMeshEffect::ndMaterial material;
	//
	//					ReadToken();
	//					ReadToken();
	//					error = fscanf(file, "%lf %lf %lf %lf", &val.m_x, &val.m_y, &val.m_z, &val.m_w);
	//					material.m_ambient = ndVector(val);
	//
	//					ReadToken();
	//					error = fscanf(file, "%lf %lf %lf %lf", &val.m_x, &val.m_y, &val.m_z, &val.m_w);
	//					material.m_diffuse = ndVector(val);
	//
	//					ReadToken();
	//					error = fscanf(file, "%lf %lf %lf %lf", &val.m_x, &val.m_y, &val.m_z, &val.m_w);
	//					material.m_specular = ndVector(val);
	//
	//					ReadToken();
	//					error = fscanf(file, "%lf", &val.m_x);
	//					material.m_opacity = ndFloat32(val.m_x);
	//
	//					ReadToken();
	//					error = fscanf(file, "%lf", &val.m_x);
	//					material.m_shiness = ndFloat32(val.m_x);
	//
	//					ReadToken();
	//					ReadToken();
	//					strcpy(material.m_textureName, token);
	//
	//					ndArray<ndMeshEffect::ndMaterial>& materials = effectMesh->GetMaterials();
	//					ndInt32 materialIndex = ndInt32(materials.GetCount());
	//
	//					ReadToken();
	//					ndInt32 faceCount;
	//					error = fscanf(file, "%d", &faceCount);
	//					ReadToken();
	//					for (ndInt32 i = 0; i < faceCount; ++i)
	//					{
	//						ndInt32 vCount;
	//						error = fscanf(file, "%d:", &vCount);
	//						faceArray.PushBack(vCount);
	//						materialArray.PushBack(materialIndex);
	//					}
	//					ReadToken();
	//
	//					ReadToken();
	//					ndInt32 indexCount;
	//					error = fscanf(file, "%d", &indexCount);
	//					ReadToken();
	//					for (ndInt32 i = 0; i < indexCount; ++i)
	//					{
	//						ndInt32 index;
	//						error = fscanf(file, "%d ", &index);
	//						indexArray.PushBack(index);
	//					}
	//					ReadToken();
	//					ReadToken();
	//					materials.PushBack(material);
	//				}
	//				else if (!strcmp(token, "vertexWeightsCluster:"))
	//				{
	//					if (!vertexWeights.GetCount())
	//					{
	//						vertexWeights.SetCount(vertexCount);
	//						for (ndInt32 i = 0; i < vertexWeights.GetCount(); ++i)
	//						{
	//							vertexWeights[i].Clear();
	//						}
	//						format.m_vertexWeight.m_data = &vertexWeights[0];
	//						format.m_vertexWeight.m_indexList = &positionsIndex[0];
	//						format.m_vertexWeight.m_strideInBytes = sizeof(ndMeshEffect::ndVertexWeight);
	//					}
	//
	//					char boneName[128];
	//					error = fscanf(file, "%s", boneName);
	//					ndInt32 indexCount;
	//					ReadToken();
	//					ReadToken();
	//					error = fscanf(file, "%d", &indexCount);
	//					
	//					ndInt32 hashId = ndInt32(ndCRC64(boneName) & 0xffffffff);
	//					ReadToken();
	//
	//					ndArray<ndInt32> boneIndex;
	//					for (ndInt32 i = 0; i < indexCount; ++i)
	//					{
	//						ndInt32 index;
	//						error = fscanf(file, "%d", &index);
	//						boneIndex.PushBack(index);
	//					}
	//					
	//					ReadToken();
	//					ndArray<ndReal> vertexWeight;
	//					for (ndInt32 i = 0; i < indexCount; ++i)
	//					{
	//						ndReal weight;
	//						error = fscanf(file, "%f", &weight);
	//						vertexWeight.PushBack(weight);
	//					}
	//					ReadToken();
	//
	//					for (ndInt32 i = 0; i < indexCount; ++i)
	//					{
	//						ndInt32 index = boneIndex[i];
	//						vertexWeights[index].SetWeight(hashId, ndReal(vertexWeight[i]));
	//					}
	//				}
	//				ReadToken();
	//			}
	//			ReadToken();
	//
	//			format.m_faceCount = ndInt32(faceArray.GetCount());
	//			format.m_faceIndexCount = &faceArray[0];
	//			format.m_faceMaterial = &materialArray[0];
	//			effectMesh->BuildFromIndexList(&format);
	//		}
	//		ReadToken();
	//	}
	//
	//	if (!strcmp(token, "node:"))
	//	{
	//		root = new ndMesh(nullptr);
	//		root->Load(file, meshEffects);
	//	}
	//
	//	fclose(file);
	//}
	//return root;
}

//void ndMeshFile::Load(FILE* const file, const ndTree<ndSharedPtr<ndMeshEffect>, ndInt32>& meshEffects)
//{
//	char token[256];
//	ndInt32 error = 0;
//
//	auto ReadToken = [file, &token]()
//	{
//		ndInt32 error = 0;
//		error = fscanf(file, "%s", token);
//	};
//
//	ReadToken();
//	ReadToken();
//	while (strcmp(token, "}"))
//	{
//		if (!strcmp(token, "name:"))
//		{
//			ReadToken();
//			SetName(token);
//		}
//		else if (!strcmp(token, "eulers:"))
//		{
//			ndBigVector eulers;
//			error = fscanf(file, "%lf %lf %lf", &eulers.m_x, &eulers.m_y, &eulers.m_z);
//			ndMatrix matrix(ndPitchMatrix(ndFloat32(eulers.m_x) * ndDegreeToRad) * ndYawMatrix(ndFloat32(eulers.m_y) * ndDegreeToRad) * ndRollMatrix(ndFloat32(eulers.m_z) * ndDegreeToRad));
//			matrix.m_posit = m_matrix.m_posit;
//			m_matrix = matrix;
//		}
//		else if (!strcmp(token, "position:"))
//		{
//			ndBigVector posit;
//			error = fscanf(file, "%lf %lf %lf", &posit.m_x, &posit.m_y, &posit.m_z);
//			posit.m_w = ndFloat32(1.0f);
//			m_matrix.m_posit = ndVector(posit);
//		}
//		else if (!strcmp(token, "geometryEulers:"))
//		{
//			ndBigVector eulers;
//			error = fscanf(file, "%lf %lf %lf", &eulers.m_x, &eulers.m_y, &eulers.m_z);
//			ndMatrix matrix(ndPitchMatrix(ndFloat32(eulers.m_x) * ndDegreeToRad) * ndYawMatrix(ndFloat32(eulers.m_y) * ndDegreeToRad) * ndRollMatrix(ndFloat32(eulers.m_z) * ndDegreeToRad));
//			matrix.m_posit = m_meshMatrix.m_posit;
//			m_meshMatrix = matrix;
//		}
//		else if (!strcmp(token, "geometryPosition:"))
//		{
//			ndBigVector posit;
//			error = fscanf(file, "%lf %lf %lf", &posit.m_x, &posit.m_y, &posit.m_z);
//			posit.m_w = ndFloat32(1.0f);
//			m_meshMatrix.m_posit = ndVector(posit);
//		}
//		else if (!strcmp(token, "node:"))
//		{
//			ndMesh* const child = new ndMesh(this);
//			child->Load(file, meshEffects);
//		}
//		else if (!strcmp(token, "geometry:"))
//		{
//			ndInt32 nodeId;
//			error = fscanf(file, "%d", &nodeId);
//			ndAssert(meshEffects.Find(nodeId));
//			m_mesh = meshEffects.Find(nodeId)->GetInfo();
//		}
//		else if (!strcmp(token, "keyFramePosits:"))
//		{
//			ndInt32 keyFramesCount;
//			error = fscanf(file, "%d\n", &keyFramesCount);
//			
//			ReadToken();
//			ndMesh::ndCurve& curve = GetPositCurve();
//			for (ndInt32 i = 0; i < keyFramesCount; ++i)
//			{
//				ndCurveValue keyframe;
//				error = fscanf(file, "%f %f %f %f\n", &keyframe.m_x, &keyframe.m_y, &keyframe.m_z, &keyframe.m_time);
//				curve.Append(keyframe);
//			}
//			ReadToken();
//		}
//		else if (!strcmp(token, "keyFrameRotations:"))
//		{
//			ndInt32 keyFramesCount;
//			error = fscanf(file, "%d\n", &keyFramesCount);
//
//			ReadToken();
//			ndMesh::ndCurve& curve = GetRotationCurve();
//			for (ndInt32 i = 0; i < keyFramesCount; ++i)
//			{
//				ndCurveValue keyframe;
//				error = fscanf(file, "%f %f %f %f\n", &keyframe.m_x, &keyframe.m_y, &keyframe.m_z, &keyframe.m_time);
//				keyframe.m_x = ndReal(keyframe.m_x * ndDegreeToRad);
//				keyframe.m_y = ndReal(keyframe.m_y * ndDegreeToRad);
//				keyframe.m_z = ndReal(keyframe.m_z * ndDegreeToRad);
//				curve.Append(keyframe);
//			}
//			ReadToken();
//		}
//		else
//		{
//			break;
//		}
//		ReadToken();
//	}
//}