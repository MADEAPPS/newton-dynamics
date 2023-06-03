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

#include "ndCoreStdafx.h"
#include "ndCollisionStdafx.h"
#include "ndMesh.h"

ndMesh::ndMesh(ndMesh* const parent)
	:ndNodeHierarchy<ndMesh>()
	,m_matrix(ndGetIdentityMatrix())
	,m_meshMatrix(ndGetIdentityMatrix())
	,m_name()
	,m_mesh()
	,m_scale()
	,m_posit()
	,m_rotation()
{
	if (parent)
	{
		Attach(parent);
	}
}

ndMesh::ndMesh(const ndMesh& src)
	:ndNodeHierarchy<ndMesh>(src)
	,m_matrix(src.m_matrix)
	,m_meshMatrix(src.m_meshMatrix)
	,m_name(src.m_name)
	,m_mesh(src.m_mesh)
	,m_scale()
	,m_posit()
	,m_rotation()
{
	for (ndCurve::ndNode* node = src.m_scale.GetFirst(); node; node = node->GetNext())
	{
		m_scale.Append(node->GetInfo());
	}

	for (ndCurve::ndNode* node = src.m_posit.GetFirst(); node; node = node->GetNext())
	{
		m_posit.Append(node->GetInfo());
	}

	for (ndCurve::ndNode* node = src.m_rotation.GetFirst(); node; node = node->GetNext())
	{
		m_rotation.Append(node->GetInfo());
	}
}

ndMesh::~ndMesh()
{
}

const ndString& ndMesh::GetName() const
{
	return m_name;
}

ndMesh::ndCurve& ndMesh::GetScaleCurve()
{
	return m_scale;
}

const ndMesh::ndCurve& ndMesh::GetScaleCurve() const
{
	return m_scale;
}

ndMesh::ndCurve& ndMesh::GetPositCurve()
{
	return m_posit;
}

const ndMesh::ndCurve& ndMesh::GetPositCurve() const
{
	return m_posit;
}

ndMesh::ndCurve& ndMesh::GetRotationCurve()
{
	return m_rotation;
}

const ndMesh::ndCurve& ndMesh::GetRotationCurve() const
{
	return m_rotation;
}

void ndMesh::SetName(const ndString& name)
{
	m_name = name;
}

ndMesh* ndMesh::CreateClone() const
{
	return new ndMesh(*this);
}

ndSharedPtr<ndMeshEffect>& ndMesh::GetMesh()
{
	return m_mesh;
}

const ndSharedPtr<ndMeshEffect>& ndMesh::GetMesh() const
{
	return m_mesh;
}

void ndMesh::SetMesh(const ndSharedPtr<ndMeshEffect>& mesh)
{
	m_mesh = mesh;
}

void ndMesh::ApplyTransform(const ndMatrix& transform)
{
	ndInt32 stack = 1;
	ndMesh* entBuffer[1024];

	auto GetKeyframe = [](const ndCurveValue& scale, const ndCurveValue& position, const ndCurveValue& rotation)
	{
		ndMatrix scaleMatrix(ndGetIdentityMatrix());
		scaleMatrix[0][0] = scale.m_x;
		scaleMatrix[1][1] = scale.m_y;
		scaleMatrix[2][2] = scale.m_z;
		ndMatrix matrix(scaleMatrix * ndPitchMatrix(rotation.m_x) * ndYawMatrix(rotation.m_y) * ndRollMatrix(rotation.m_z));
		matrix.m_posit = ndVector(position.m_x, position.m_y, position.m_z, 1.0f);
		return matrix;
	};

	entBuffer[0] = this;
	ndMatrix invTransform(transform.Inverse4x4());
	while (stack)
	{
		stack--;
		ndMesh* const node = entBuffer[stack];

		ndMatrix entMatrix(invTransform * node->m_matrix * transform);
		node->m_matrix = entMatrix;

		ndSharedPtr<ndMeshEffect> mesh (node->GetMesh());
		if (*mesh)
		{
			ndMatrix meshMatrix(invTransform * node->m_meshMatrix * transform);
			node->m_meshMatrix = meshMatrix;
			mesh->ApplyTransform(transform);
		}

		ndMesh::ndCurve& positCurve = node->GetPositCurve();
		ndMesh::ndCurve& rotationCurve = node->GetRotationCurve();
		if (positCurve.GetCount() || rotationCurve.GetCount())
		{
			ndMesh::ndCurve::ndNode* positNode = node->GetPositCurve().GetFirst();
			ndMesh::ndCurve::ndNode* rotationNode = node->GetRotationCurve().GetFirst();

			ndMesh::ndCurveValue scaleValue;
			scaleValue.m_x = 1.0f;
			scaleValue.m_y = 1.0f;
			scaleValue.m_z = 1.0f;
			for (ndInt32 i = 0; i < positCurve.GetCount(); ++i)
			{
				ndMesh::ndCurveValue& positValue = positNode->GetInfo();
				ndMesh::ndCurveValue& rotationValue = rotationNode->GetInfo();

				ndVector animScale;
				ndMatrix stretchAxis;
				ndMatrix animTransformMatrix;
				ndMatrix keyframe(invTransform * GetKeyframe(scaleValue, positValue, rotationValue) * transform);
				keyframe.PolarDecomposition(animTransformMatrix, animScale, stretchAxis);

				ndVector euler0;
				ndVector euler(animTransformMatrix.CalcPitchYawRoll(euler0));

				rotationValue.m_x = euler.m_x;
				rotationValue.m_y = euler.m_y;
				rotationValue.m_z = euler.m_z;

				positValue.m_x = animTransformMatrix.m_posit.m_x;
				positValue.m_y = animTransformMatrix.m_posit.m_y;
				positValue.m_z = animTransformMatrix.m_posit.m_z;

				positNode = positNode->GetNext();
				rotationNode = rotationNode->GetNext();
			}
		}

		for (ndMesh* child = node->GetFirstChild(); child; child = child->GetNext())
		{
			entBuffer[stack] = child;
			stack++;
			ndAssert(stack < sizeof(entBuffer) / sizeof (entBuffer[0]));
		}
	}
}

void ndMesh::Save(const ndMesh* const mesh, const char* const fullPathName)
{
	FILE* const file = fopen(fullPathName, "wb");
	if (file)
	{
		fprintf(file, "# Newton Dynamics Mesh file format.\n\n");
		ndTree<ndInt32, const ndMeshEffect*> meshEffects;
		for (ndMesh* node = mesh->GetFirstIterator(); node; node = node->GetNextIterator())
		{
			if (*node->m_mesh)
			{
				meshEffects.Insert(meshEffects.GetCount(), *node->m_mesh);
			}
		}

		if (meshEffects.GetCount())
		{
			auto PrintVertexChannel = [file](const ndMeshEffect::ndMeshVertexFormat::ndData<ndFloat64>& channel, ndInt32 vertexCount)
			{
				fprintf(file, "\t\tvertex:\n");
				fprintf(file, "\t\t{\n");

				ndInt32 positCount = 0;
				for (ndInt32 i = 0; i < vertexCount; ++i)
				{
					positCount = ndMax(positCount, channel.m_indexList[i] + 1);
				}

				fprintf(file, "\t\t\tposition: %d\n", positCount);
				fprintf(file, "\t\t\t{\n");
				ndInt32 stride = channel.m_strideInBytes / ndInt32(sizeof(ndFloat64));
				for (ndInt32 i = 0; i < positCount; ++i)
				{
					ndFloat64 x;
					ndFloat64 y;
					ndFloat64 z;
					x = channel.m_data[i * stride + 0];
					y = channel.m_data[i * stride + 1];
					z = channel.m_data[i * stride + 2];
					fprintf(file, "\t\t\t\t%lg %lg %lg\n", x, y, z);
				}
				fprintf(file, "\t\t\t}\n");

				fprintf(file, "\t\t\tindices: %d\n", vertexCount);
				fprintf(file, "\t\t\t{\n");
				fprintf(file, "\t\t\t\t");
				for (ndInt32 i = 0; i < vertexCount; ++i)
				{
					fprintf(file, "%d ", channel.m_indexList[i]);
				}
				fprintf(file, "\n");
				fprintf(file, "\t\t\t}\n");

				fprintf(file, "\t\t}\n");
			};

			auto PrintNormalChannel = [file](const ndMeshEffect::ndMeshVertexFormat::ndData<ndReal>& channel, ndInt32 vertexCount)
			{
				fprintf(file, "\t\tnormal:\n");
				fprintf(file, "\t\t{\n");

				ndInt32 positCount = 0;
				for (ndInt32 i = 0; i < vertexCount; ++i)
				{
					positCount = ndMax(positCount, channel.m_indexList[i] + 1);
				}

				fprintf(file, "\t\t\tposition: %d\n", positCount);
				fprintf(file, "\t\t\t{\n");
				ndInt32 stride = channel.m_strideInBytes / ndInt32(sizeof(ndReal));
				for (ndInt32 i = 0; i < positCount; ++i)
				{
					ndReal x;
					ndReal y;
					ndReal z;
					x = channel.m_data[i * stride + 0];
					y = channel.m_data[i * stride + 1];
					z = channel.m_data[i * stride + 2];
					fprintf(file, "\t\t\t\t%g %g %g\n", x, y, z);
				}
				fprintf(file, "\t\t\t}\n");

				fprintf(file, "\t\t\tindices: %d\n", vertexCount);
				fprintf(file, "\t\t\t{\n");
				fprintf(file, "\t\t\t\t");
				for (ndInt32 i = 0; i < vertexCount; ++i)
				{
					fprintf(file, "%d ", channel.m_indexList[i]);
				}
				fprintf(file, "\n");
				fprintf(file, "\t\t\t}\n");

				fprintf(file, "\t\t}\n");
			};

			auto PrintUVChannel = [file](const ndMeshEffect::ndMeshVertexFormat::ndData<ndReal>& channel, ndInt32 vertexCount)
			{
				fprintf(file, "\t\tuv:\n");
				fprintf(file, "\t\t{\n");

				ndInt32 positCount = 0;
				for (ndInt32 i = 0; i < vertexCount; ++i)
				{
					positCount = ndMax(positCount, channel.m_indexList[i] + 1);
				}

				fprintf(file, "\t\t\tposition: %d\n", positCount);
				fprintf(file, "\t\t\t{\n");
				ndInt32 stride = channel.m_strideInBytes / ndInt32(sizeof(ndReal));
				for (ndInt32 i = 0; i < positCount; ++i)
				{
					ndReal x;
					ndReal y;
					x = channel.m_data[i * stride + 0];
					y = channel.m_data[i * stride + 1];
					fprintf(file, "\t\t\t\t%g %g\n", x, y);
				}
				fprintf(file, "\t\t\t}\n");

				fprintf(file, "\t\t\tindices: %d\n", vertexCount);
				fprintf(file, "\t\t\t{\n");
				fprintf(file, "\t\t\t\t");
				for (ndInt32 i = 0; i < vertexCount; ++i)
				{
					fprintf(file, "%d ", channel.m_indexList[i]);
				}
				fprintf(file, "\n");
				fprintf(file, "\t\t\t}\n");

				fprintf(file, "\t\t}\n");
			};

			//ndTree<ndMesh*, ndInt32> nodeMap;
			//for (ndMesh* node = mesh->GetFirstIterator(); node; node = node->GetNextIterator())
			//{
			//	nodeMap.Insert(node, ndInt32(ndCRC64(node->m_name.GetStr()) & 0xffffffff));
			//}

			fprintf(file, "geometries: %d\n", meshEffects.GetCount());
			fprintf(file, "{\n");

			ndTree<ndInt32, const ndMeshEffect*>::Iterator it(meshEffects);
			for (it.Begin(); it; it++)
			{
				fprintf(file, "\tgeometry: %d\n", it.GetNode()->GetInfo());
				fprintf(file, "\t{\n");

				ndMeshEffect* effectMesh = (ndMeshEffect*)it.GetKey();

				ndArray<ndUnsigned8> tmpBuffer;
				ndMeshEffect::ndMeshVertexFormat format;
				ndInt32 vertexCount = effectMesh->GenerateVertexFormat(format, tmpBuffer);
				PrintVertexChannel(format.m_vertex, vertexCount);
				if (format.m_normal.m_data)
				{
					PrintNormalChannel(format.m_normal, vertexCount);
				}
				if (format.m_uv0.m_data)
				{
					PrintUVChannel(format.m_uv0, vertexCount);
				}

				const ndArray<ndMeshEffect::ndMaterial>& materialArray = effectMesh->GetMaterials();
				for (ndInt32 i = 0; i < materialArray.GetCount(); ++i)
				{
					fprintf(file, "\t\tmaterial:\n");
					fprintf(file, "\t\t{\n");
				
					const ndMeshEffect::ndMaterial& material = materialArray[i];
				
					fprintf(file, "\t\t\tambience: %g %g %g %g\n", material.m_ambient.m_x, material.m_ambient.m_y, material.m_ambient.m_z, material.m_ambient.m_w);
					fprintf(file, "\t\t\tdiffuse: %g %g %g %g\n", material.m_diffuse.m_x, material.m_diffuse.m_y, material.m_diffuse.m_z, material.m_diffuse.m_w);
					fprintf(file, "\t\t\tm_specular: %g %g %g %g\n", material.m_specular.m_x, material.m_specular.m_y, material.m_specular.m_z, material.m_specular.m_w);
					fprintf(file, "\t\t\topacity: %g\n", material.m_opacity);
					fprintf(file, "\t\t\tshiness: %g\n", material.m_shiness);
					fprintf(file, "\t\t\ttexture: %s\n", material.m_textureName);

					ndInt32 faceCount = 0;
					for (ndInt32 j = 0; j < format.m_faceCount; ++j)
					{
						if (format.m_faceMaterial[j] == i)
						{
							faceCount++;
						}
					}
				
					ndInt32 indexAcc = 0;
					fprintf(file, "\t\t\tfaces: %d\n", faceCount);
					fprintf(file, "\t\t\t{\n");
					for (ndInt32 j = 0; j < format.m_faceCount; ++j)
					{
						if (format.m_faceMaterial[j] == i)
						{
							fprintf(file, "\t\t\t\t%d: ", format.m_faceIndexCount[j]);
							for (ndInt32 k = 0; k < format.m_faceIndexCount[j]; ++k)
							{
								fprintf(file, "%d ", format.m_vertex.m_indexList[indexAcc + k]);
							}
							fprintf(file, "\n");
						}
						indexAcc += format.m_faceIndexCount[j];
					}
					fprintf(file, "\t\t\t}\n");

					fprintf(file, "\t\t}\n");
				}

				if (format.m_vertexWeight.m_data)
				{
					ndInt32 weightsCount = 0;
					for (ndInt32 i = 0; i < vertexCount; ++i)
					{
						weightsCount = ndMax(weightsCount, format.m_vertexWeight.m_indexList[i] + 1);
					}

					for (ndMesh* node = mesh->GetFirstIterator(); node; node = node->GetNextIterator())
					{
						ndInt32 count = 0;
						ndInt32 hash = ndInt32(ndCRC64(node->m_name.GetStr()) & 0xffffffff);
						ndAssert(hash);
						for (ndInt32 j = 0; j < weightsCount; ++j)
						{
							for (ndInt32 k = 0; k < ND_VERTEX_WEIGHT_SIZE; ++k)
							{
								count += (format.m_vertexWeight.m_data[j].m_boneId[k] == hash) ? 1 : 0;
							}
						}
						if (count)
						{
							fprintf(file, "\t\tskinCluster: %s\n", node->m_name.GetStr());
							fprintf(file, "\t\t{\n");
							
							fprintf(file, "\t\t\tindexCount: %d\n", count);
							
							fprintf(file, "\t\t\tvertexIndex: ");
							for (ndInt32 j = 0; j < weightsCount; ++j)
							{
								for (ndInt32 k = 0; k < ND_VERTEX_WEIGHT_SIZE; ++k)
								{
									if (format.m_vertexWeight.m_data[j].m_boneId[k] == hash)
									{
										fprintf(file, "%d ", j);
									}
								}
							}
							fprintf(file, "\t\t\t\n");

							fprintf(file, "\t\t\tvertexWeight: ");
							for (ndInt32 j = 0; j < weightsCount; ++j)
							{
								for (ndInt32 k = 0; k < ND_VERTEX_WEIGHT_SIZE; ++k)
								{
									if (format.m_vertexWeight.m_data[j].m_boneId[k] == hash)
									{
										fprintf(file, "%g ", format.m_vertexWeight.m_data[j].m_weight[k]);
									}
								}
							}
							fprintf(file, "\t\t\t\n");
							
							fprintf(file, "\t\t}\n");

						}
					}
				}
				
				fprintf(file, "\t}\n");
			}

			fprintf(file, "}\n");
			fprintf(file, "\n");
		}

		mesh->Save(file, meshEffects);
		fclose(file);
	}
}

void ndMesh::Save(FILE* const file, const ndTree<ndInt32, const ndMeshEffect*>& meshEffects, ndInt32 level) const
{
	auto PrintTabs = [file](ndInt32 level)
	{
		for (ndInt32 i = 0; i < level; i++)
		{
			fprintf(file, "\t");
		}
	};

	PrintTabs(level);
	fprintf(file, "node:\n");

	PrintTabs(level);
	fprintf(file, "{\n");

	PrintTabs(level);
	fprintf(file, "\tname: %s\n", GetName().GetStr());

	ndVector euler1;
	ndVector euler(m_matrix.CalcPitchYawRoll(euler1).Scale(ndRadToDegree));
	PrintTabs(level);
	fprintf(file, "\teulers: %g %g %g\n", euler.m_x, euler.m_y, euler.m_z);
	PrintTabs(level);
	fprintf(file, "\tposition: %g %g %g\n", m_matrix.m_posit.m_x, m_matrix.m_posit.m_y, m_matrix.m_posit.m_z);

	PrintTabs(level);
	euler = m_meshMatrix.CalcPitchYawRoll(euler1).Scale(ndRadToDegree);
	fprintf(file, "\tgeometryEulers: %g %g %g\n", euler.m_x, euler.m_y, euler.m_z);
	PrintTabs(level);
	fprintf(file, "\tgeometryPosition: %g %g %g\n", m_meshMatrix.m_posit.m_x, m_meshMatrix.m_posit.m_y, m_meshMatrix.m_posit.m_z);

	if (*m_mesh)
	{
		ndTree<ndInt32, const ndMeshEffect*>::ndNode* const meshNode = meshEffects.Find(*m_mesh);
		if (meshNode)
		{
			PrintTabs(level);
			fprintf(file, "\tgeometry: %d\n", meshNode->GetInfo());
		}
	}

	if (GetPositCurve().GetCount())
	{
		//ndAssert(0);
	}

	if (GetRotationCurve().GetCount())
	{
		//ndAssert(0);
	}

	for (ndMesh* child = GetFirstChild(); child; child = child->GetNext())
	{
		child->Save(file, meshEffects, level + 1);
	}

	PrintTabs(level);
	fprintf(file, "}\n");
}

void ndMesh::Load(FILE* const file, const ndTree<ndSharedPtr<ndMeshEffect>, ndInt32>& meshEffects)
{
	char token[256];
	auto ReadToken = [file, &token]()
	{
		fscanf(file, "%s", token);
	};

	ReadToken();
	while (strcmp(token, "}"))
	{
		ReadToken();
		if (!strcmp(token, "name:"))
		{
			ReadToken();
			SetName(token);
		}
		else if (!strcmp(token, "eulers:"))
		{
			ndBigVector eulers;
			fscanf(file, "%lf %lf %lf", &eulers.m_x, &eulers.m_y, &eulers.m_z);
			ndMatrix matrix(ndPitchMatrix(ndFloat32(eulers.m_x) * ndDegreeToRad) * ndYawMatrix(ndFloat32(eulers.m_y) * ndDegreeToRad) * ndRollMatrix(ndFloat32(eulers.m_z) * ndDegreeToRad));
			matrix.m_posit = m_matrix.m_posit;
			m_matrix = matrix;
		}
		else if (!strcmp(token, "position:"))
		{
			ndBigVector posit;
			fscanf(file, "%lf %lf %lf", &posit.m_x, &posit.m_y, &posit.m_z);
			posit.m_w = ndFloat32(1.0f);
			m_matrix.m_posit = ndVector(posit);
		}
		else if (!strcmp(token, "geometryEulers:"))
		{
			ndBigVector eulers;
			fscanf(file, "%lf %lf %lf", &eulers.m_x, &eulers.m_y, &eulers.m_z);
			ndMatrix matrix(ndPitchMatrix(ndFloat32(eulers.m_x) * ndDegreeToRad) * ndYawMatrix(ndFloat32(eulers.m_y) * ndDegreeToRad) * ndRollMatrix(ndFloat32(eulers.m_z) * ndDegreeToRad));
			matrix.m_posit = m_meshMatrix.m_posit;
			m_meshMatrix = matrix;
		}
		else if (!strcmp(token, "geometryPosition:"))
		{
			ndBigVector posit;
			fscanf(file, "%lf %lf %lf", &posit.m_x, &posit.m_y, &posit.m_z);
			posit.m_w = ndFloat32(1.0f);
			m_meshMatrix.m_posit = ndVector(posit);
		}
		else if (!strcmp(token, "node:"))
		{
			ndMesh* const child = new ndMesh(this);
			child->Load(file, meshEffects);
		}
		else if (!strcmp(token, "geometry:"))
		{
			ndInt32 nodeId;
			fscanf(file, "%d", &nodeId);
			ndAssert(meshEffects.Find(nodeId));
			m_mesh = meshEffects.Find(nodeId)->GetInfo();
		}
		else
		{
			break;
		}
	}
}

ndMesh* ndMesh::Load(const char* const fullPathName)
{
	ndMesh* root = nullptr;
	FILE* const file = fopen(fullPathName, "rb");
	if (file)
	{
		char token[256];
		auto ReadToken = [file, &token]()
		{
			fscanf(file, "%s", token);
		};

		ReadToken();
		ndTree<ndSharedPtr<ndMeshEffect>, ndInt32> meshEffects;
		if (!strcmp(token, "geometries:"))
		{
			ndInt32 geomntryCount;
			fscanf(file, "%d", &geomntryCount);
			ReadToken();

			for (ndInt32 k = 0; k < geomntryCount; k++)
			{
				ndInt32 meshId;
				ReadToken();
				fscanf(file, "%d", &meshId);
				ndSharedPtr<ndMeshEffect> effectMesh(new ndMeshEffect());
				meshEffects.Insert(effectMesh, meshId);
				ReadToken();
				
				ndArray<ndBigVector> positions;
				ndArray<ndMeshEffect::ndUV> uvs;
				ndArray<ndMeshEffect::ndNormal> normals;

				ndArray<ndInt32> uvIndex;
				ndArray<ndInt32> faceArray;
				ndArray<ndInt32> indexArray;
				ndArray<ndInt32> normalsIndex;
				ndArray<ndInt32> materialArray;
				ndArray<ndInt32> positionsIndex;
				ndMeshEffect::ndMeshVertexFormat format;

				ReadToken();
				while (strcmp(token, "}"))
				{
					if (!strcmp(token, "vertex:"))
					{
						ndInt32 vertexCount;
						ReadToken();
						ReadToken();
						fscanf(file, "%d", &vertexCount);
						ReadToken();
						for (ndInt32 i = 0; i < vertexCount; ++i)
						{
							ndFloat64 x;
							ndFloat64 y;
							ndFloat64 z;
							fscanf(file, "%lf %lf %lf", &x, &y, &z);
							positions.PushBack(ndBigVector(x, y, z, ndFloat64(0.0f)));
						}
						ReadToken();

						int indexCount;
						ReadToken();
						fscanf(file, "%d", &indexCount);
						ReadToken();
						for (ndInt32 i = 0; i < indexCount; ++i)
						{
							ndInt32 index;
							fscanf(file, "%d" , &index);
							positionsIndex.PushBack(index);
						}
						ReadToken();

						format.m_vertex.m_data = &positions[0].m_x;
						format.m_vertex.m_indexList = &positionsIndex[0];
						format.m_vertex.m_strideInBytes = sizeof(ndBigVector);
						ReadToken();
					}
					else if (!strcmp(token, "normal:"))
					{
						ndInt32 vertexCount;
						ReadToken();
						ReadToken();
						fscanf(file, "%d", &vertexCount);

						ReadToken();
						for (ndInt32 i = 0; i < vertexCount; ++i)
						{
							ndReal x;
							ndReal y;
							ndReal z;
							fscanf(file, "%f %f %f", &x, &y, &z);
							normals.PushBack(ndMeshEffect::ndNormal(x, y, z));
						}
						ReadToken();

						int indexCount;
						ReadToken();
						fscanf(file, "%d", &indexCount);
						ReadToken();
						for (ndInt32 i = 0; i < indexCount; ++i)
						{
							ndInt32 index;
							fscanf(file, "%d", &index);
							normalsIndex.PushBack(index);
						}
						ReadToken();

						format.m_normal.m_data = &normals[0].m_x;
						format.m_normal.m_indexList = &normalsIndex[0];
						format.m_normal.m_strideInBytes = sizeof(ndMeshEffect::ndNormal);
						ReadToken();
					}
					else if (!strcmp(token, "uv:"))
					{
						ndInt32 vertexCount;
						ReadToken();
						ReadToken();
						fscanf(file, "%d", &vertexCount);

						ReadToken();
						for (ndInt32 i = 0; i < vertexCount; ++i)
						{
							ndReal x;
							ndReal y;
							fscanf(file, "%f %f", &x, &y);
							uvs.PushBack(ndMeshEffect::ndUV(x, y));
						}
						ReadToken();

						int indexCount;
						ReadToken();
						fscanf(file, "%d", &indexCount);
						ReadToken();
						for (ndInt32 i = 0; i < indexCount; ++i)
						{
							ndInt32 index;
							fscanf(file, "%d", &index);
							uvIndex.PushBack(index);
						}
						ReadToken();

						format.m_uv0.m_data = &uvs[0].m_u;
						format.m_uv0.m_indexList = &uvIndex[0];
						format.m_uv0.m_strideInBytes = sizeof(ndMeshEffect::ndUV);
						ReadToken();
					}
					else if (!strcmp(token, "material:"))
					{
						ndBigVector val;
						ndMeshEffect::ndMaterial material;

						ReadToken();
						ReadToken();
						fscanf(file, "%lf %lf %lf %lf", &val.m_x, &val.m_y, &val.m_z, &val.m_w);
						material.m_ambient = ndVector(val);

						ReadToken();
						fscanf(file, "%lf %lf %lf %lf", &val.m_x, &val.m_y, &val.m_z, &val.m_w);
						material.m_diffuse = ndVector(val);

						ReadToken();
						fscanf(file, "%lf %lf %lf %lf", &val.m_x, &val.m_y, &val.m_z, &val.m_w);
						material.m_specular = ndVector(val);

						ReadToken();
						fscanf(file, "%lf", &val.m_x);
						material.m_opacity = ndFloat32(val.m_x);

						ReadToken();
						fscanf(file, "%lf", &val.m_x);
						material.m_shiness = ndFloat32(val.m_x);

						ReadToken();
						ReadToken();
						strcpy(material.m_textureName, token);

						ndArray<ndMeshEffect::ndMaterial>& materials = effectMesh->GetMaterials();
						ndInt32 materialIndex = materials.GetCount();

						ReadToken();
						ndInt32 faceCount;
						fscanf(file, "%d", &faceCount);

						ReadToken();
						for (ndInt32 i = 0; i < faceCount; ++i)
						{
							ndInt32 vertexCount;
							fscanf(file, "%d:", &vertexCount);
							for (ndInt32 j = 0; j < vertexCount; ++j)
							{
								ndInt32 index;
								fscanf(file, "%d ", &index);
								indexArray.PushBack(index);
							}
							faceArray.PushBack(vertexCount);
							materialArray.PushBack(materialIndex);
						}
						ReadToken();
						ReadToken();
						materials.PushBack(material);
					}
					else if (!strcmp(token, "skinCluster:"))
					{
						ndAssert(0);
						//char boneName[128];
						//fscanf(file, "%s", boneName);
						//ndInt32 indexCount;
						//ReadToken();
						//ReadToken();
						//fscanf(file, "%d", &indexCount);
						//
						//ndMeshEffect::ndVertexCluster* const cluster = effectMesh->CreateCluster(boneName);
						//ReadToken();
						//for (ndInt32 i = 0; i < indexCount; ++i)
						//{
						//	ndInt32 index;
						//	fscanf(file, "%d", &index);
						//	cluster->m_vertexIndex.PushBack(index);
						//}
						//
						//ReadToken();
						//for (ndInt32 i = 0; i < indexCount; ++i)
						//{
						//	ndReal weight;
						//	fscanf(file, "%f", &weight);
						//	cluster->m_vertexWeigh.PushBack(ndReal(weight));
						//}
						//ReadToken();
					}
					ReadToken();
				}
				ReadToken();
				format.m_faceCount = faceArray.GetCount();
				format.m_faceIndexCount = &faceArray[0];
				format.m_faceMaterial = &materialArray[0];
				effectMesh->BuildFromIndexList(&format);
			}
			ReadToken();
		}

		if (!strcmp(token, "node:"))
		{
			root = new ndMesh(nullptr);
			root->Load(file, meshEffects);
		}

		fclose(file);
	}
	return root;
}