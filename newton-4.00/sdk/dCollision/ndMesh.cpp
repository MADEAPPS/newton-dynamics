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

ndMesh* ndMesh::Load(const char* const fullPathName)
{
	ndMesh* root = nullptr;
	FILE* const file = fopen(fullPathName, "rb");
	if (file)
	{
		char token[256];
		fscanf(file, "%s", token);
		if (!strcmp(token, "node:"))
		{
			root = new ndMesh(nullptr);
			root->Load(file);
		}

		fclose(file);
	}
	return root;
}


void ndMesh::Load(FILE* const file)
{
	char token[256];
	auto ReadToken = [file, &token]()
	{
		fscanf(file, "%s", token);
	};

	auto LoadGeometry = [file, &token](ndMesh* const node)
	{
		auto ReadToken = [file, &token]()
		{
			fscanf(file, "%s", token);
		};

		auto SkipToken = [file, &token]()
		{
			char temp[128];
			fscanf(file, "%s", temp);
		};

		node->m_mesh = ndSharedPtr<ndMeshEffect>(new ndMeshEffect());

		struct ndTmpVertex
		{
			ndFloat64 m_posit[3];
		};
		struct ndNornalUV
		{
			ndFloat32 m_normal[3];
			ndFloat32 m_uv[2];
		};

		ndArray<ndTmpVertex> vertex;
		ndArray<ndNornalUV> normalUV;
		ndArray<ndInt32> indexArray;

		ReadToken();
		while (strcmp(token, "}"))
		{
			ReadToken();
			if (!strcmp(token, "eulers:"))
			{
				ndBigVector eulers;
				fscanf(file, "%lf %lf %lf", &eulers.m_x, &eulers.m_y, &eulers.m_z);
				ndMatrix matrix(ndPitchMatrix(ndFloat32(eulers.m_x) * ndDegreeToRad) * ndYawMatrix(ndFloat32(eulers.m_y) * ndDegreeToRad) * ndRollMatrix(ndFloat32(eulers.m_z) * ndDegreeToRad));
				matrix.m_posit = node->m_meshMatrix.m_posit;
				node->m_meshMatrix = matrix;
			}
			else if (!strcmp(token, "position:"))
			{
				ndBigVector posit;
				fscanf(file, "%lf %lf %lf", &posit.m_x, &posit.m_y, &posit.m_z);
				posit.m_w = ndFloat32(1.0f);
				node->m_meshMatrix.m_posit = ndVector(posit);
			}
			else if (!strcmp(token, "points(x"))
			{
				fscanf(file, "%s", token);
				fscanf(file, "%s", token);
				fscanf(file, "%s", token);
				fscanf(file, "%s", token);
				fscanf(file, "%s", token);
				fscanf(file, "%s", token);
				fscanf(file, "%s", token);
			
				ndInt32 pointsCount;
				fscanf(file, "%d", &pointsCount);
				ReadToken();

				for (ndInt32 i = 0; i < pointsCount; ++i)
				{
					ndNornalUV nuv;
					ndTmpVertex point;
			
					ndBigVector uv;
					ndBigVector normal;
					fscanf(file, "%lf %lf %lf", &point.m_posit[0], &point.m_posit[1], &point.m_posit[2]);
					fscanf(file, "%lf %lf %lf", &normal.m_x, &normal.m_y, &normal.m_z);
					fscanf(file, "%lf %lf", &uv.m_x, &uv.m_y);
			
					nuv.m_uv[0] = ndFloat32(uv.m_x);
					nuv.m_uv[1] = ndFloat32(uv.m_y);
					nuv.m_normal[0] = ndFloat32(normal.m_x);
					nuv.m_normal[1] = ndFloat32(normal.m_y);
					nuv.m_normal[2] = ndFloat32(normal.m_z);
			
					vertex.PushBack(point);
					normalUV.PushBack(nuv);
				}
				SkipToken();
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
				
				ndArray<ndMeshEffect::ndMaterial>& materialArray = node->m_mesh->GetMaterials();
				ndInt32 triangleCount;
				ndInt32 materialIndex = materialArray.GetCount();
				
				ReadToken();
				fscanf(file, "%d", &triangleCount);

				ReadToken();
				indexArray.SetCount(0);
				for (ndInt32 i = 0; i < triangleCount; ++i)
				{
					ndInt32 i0;
					ndInt32 i1;
					ndInt32 i2;
					fscanf(file, "%d %d %d", &i0, &i1, &i2);
					indexArray.PushBack(i0);
					indexArray.PushBack(i1);
					indexArray.PushBack(i2);
					indexArray.PushBack(materialIndex);
				}
				SkipToken();
				SkipToken();
				materialArray.PushBack(material);
			}
			else if (!strcmp(token, "skinCluster:"))
			{
				char boneName[126];
				fscanf(file, "%s", boneName);

				ndInt32 indexCount;
				ReadToken();
				ReadToken();
				fscanf(file, "%d", &indexCount);
			
				ndMeshEffect::ndVertexCluster* const cluster = node->m_mesh->CreateCluster(boneName);
				ReadToken();
				for (ndInt32 i = 0; i < indexCount; ++i)
				{
					ndInt32 index;
					fscanf(file, "%d", &index);
					cluster->m_vertexIndex.PushBack(index);
				}
			
				ReadToken();
				for (ndInt32 i = 0; i < indexCount; ++i)
				{
					ndFloat64 weight;
					fscanf(file, "%lf", &weight);
					cluster->m_vertexWeigh.PushBack(ndFloat32 (weight));
				}
				SkipToken();
			}
		}

		node->m_mesh->BeginBuild();
		for (ndInt32 i = 0; i < indexArray.GetCount(); i += 4)
		{
			node->m_mesh->BeginBuildFace();
			ndInt32 matIndex = indexArray[i + 3];
			for (ndInt32 j = 0; j < 3; j++)
			{
				ndInt32 index = indexArray[i + j];
				node->m_mesh->AddMaterial(matIndex);
				node->m_mesh->AddPoint(vertex[index].m_posit[0], vertex[index].m_posit[1], vertex[index].m_posit[2]);
				node->m_mesh->AddNormal(normalUV[index].m_normal[0], normalUV[index].m_normal[1], normalUV[index].m_normal[2]);
				node->m_mesh->AddUV0(normalUV[index].m_uv[0], normalUV[index].m_uv[1]);
			}
			node->m_mesh->EndBuildFace();
		}
		node->m_mesh->EndBuild();

		ReadToken();
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
		else if (!strcmp(token, "node:"))
		{
			ndMesh* const child = new ndMesh(this);
			child->Load(file);
		}
		else if (!strcmp(token, "geometry:"))
		{
			LoadGeometry(this);
		}
		else
		{
			break;
		}
	}
}

void ndMesh::Save(const ndMesh* const mesh, const char* const fullPathName)
{
	FILE* const file = fopen(fullPathName, "wb");
	if (file)
	{
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
			auto PrintVertexChannel = [file](const ndMeshEffect::ndMeshVertexFormat::ndDoubleData& channel, ndInt32 vertexCount)
			{
				fprintf(file, "\t\tvertex:\n");
				fprintf(file, "\t\t{\n");

				fprintf(file, "\t\t\tindices: %d\n", vertexCount);
				fprintf(file, "\t\t\t{\n");
				ndInt32 positCount = 0;
				fprintf(file, "\t\t\t\t");
				for (ndInt32 i = 0; i < vertexCount; ++i)
				{
					fprintf(file, "%d ", channel.m_indexList[i]);
					positCount = ndMax(positCount, channel.m_indexList[i] + 1);
				}
				fprintf(file, "\n");
				fprintf(file, "\t\t\t}\n");

				fprintf(file, "\t\t\tposits: %d\n", positCount);
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
				fprintf(file, "\t\t}\n");
			};

			auto PrintNormalChannel = [file](const ndMeshEffect::ndMeshVertexFormat::ndFloatData& channel, ndInt32 vertexCount)
			{
				fprintf(file, "\t\tnormal\n");
				fprintf(file, "\t\t{\n");

				fprintf(file, "\t\t\tindices: %d\n", vertexCount);
				fprintf(file, "\t\t\t{\n");
				ndInt32 positCount = 0;
				fprintf(file, "\t\t\t\t");
				for (ndInt32 i = 0; i < vertexCount; ++i)
				{
					fprintf(file, "%d ", channel.m_indexList[i]);
					positCount = ndMax(positCount, channel.m_indexList[i] + 1);
				}
				fprintf(file, "\n");
				fprintf(file, "\t\t\t}\n");

				fprintf(file, "\t\t\tposits: %d\n", positCount);
				fprintf(file, "\t\t\t{\n");
				ndInt32 stride = channel.m_strideInBytes / ndInt32(sizeof(ndFloat32));
				for (ndInt32 i = 0; i < positCount; ++i)
				{
					ndFloat32 x;
					ndFloat32 y;
					ndFloat32 z;
					x = channel.m_data[i * stride + 0];
					y = channel.m_data[i * stride + 1];
					z = channel.m_data[i * stride + 2];
					fprintf(file, "\t\t\t\t%g %g %g\n", x, y, z);
				}
				fprintf(file, "\t\t\t}\n");
				fprintf(file, "\t\t}\n");
			};

			auto PrintUVChannel = [file](const ndMeshEffect::ndMeshVertexFormat::ndFloatData& channel, ndInt32 vertexCount)
			{
				fprintf(file, "\t\tuv:");
				fprintf(file, "\t\t{\n");

				fprintf(file, "\t\t\tindices %d\n", vertexCount);
				fprintf(file, "\t\t\t{\n");
				ndInt32 positCount = 0;
				fprintf(file, "\t\t\t\t");
				for (ndInt32 i = 0; i < vertexCount; ++i)
				{
					fprintf(file, "%d ", channel.m_indexList[i]);
					positCount = ndMax(positCount, channel.m_indexList[i] + 1);
				}
				fprintf(file, "\n");
				fprintf(file, "\t\t\t}\n");

				fprintf(file, "\t\t\tposits: %d\n", positCount);
				fprintf(file, "\t\t\t{\n");
				ndInt32 stride = channel.m_strideInBytes / ndInt32(sizeof(ndFloat32));
				for (ndInt32 i = 0; i < positCount; ++i)
				{
					ndFloat32 x;
					ndFloat32 y;
					x = channel.m_data[i * stride + 0];
					y = channel.m_data[i * stride + 1];
					fprintf(file, "\t\t\t\t%g %g\n", x, y);
				}
				fprintf(file, "\t\t\t}\n");
				fprintf(file, "\t\t}\n");
			};

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
				
				const ndMeshEffect::ndClusterMap& clusters = effectMesh->GetCluster();
				if (clusters.GetCount())
				{
					ndMeshEffect::ndClusterMap::Iterator clusterIter(clusters);
					for (clusterIter.Begin(); clusterIter; clusterIter++)
					{
						fprintf(file, "\t\tskinCluster: %s\n", clusterIter.GetKey().GetStr());
						fprintf(file, "\t\t{\n");
				
						ndMeshEffect::ndVertexCluster& cluster = clusterIter.GetNode()->GetInfo();
				
						fprintf(file, "\t\t\tindexCount: %d\n", cluster.m_vertexIndex.GetCount());
				
						fprintf(file, "\t\t\tvertexIndex: ");
						for (ndInt32 i = 0; i < cluster.m_vertexIndex.GetCount(); ++i)
						{
							fprintf(file, "%d ", cluster.m_vertexIndex[i]);
						}
						fprintf(file, "\t\t\t\n");
				
						fprintf(file, "\t\t\tvertexWeight: ");
						for (ndInt32 i = 0; i < cluster.m_vertexWeigh.GetCount(); ++i)
						{
							fprintf(file, "%g ", cluster.m_vertexWeigh[i]);
						}
						fprintf(file, "\t\t\t\n");
				
						fprintf(file, "\t\t}\n");
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
		PrintTabs(level);
		fprintf(file, "\tgeometry: %d\n", meshEffects.Find(*m_mesh)->GetInfo());
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