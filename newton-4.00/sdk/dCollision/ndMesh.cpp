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
		mesh->Save(file);
		fclose(file);
	}
}

void ndMesh::Save(FILE* const file, ndInt32 level) const
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
	fprintf(file, "\teulers: %f %f %f\n", euler.m_x, euler.m_y, euler.m_z);
	PrintTabs(level);
	fprintf(file, "\tposition: %f %f %f\n", m_matrix.m_posit.m_x, m_matrix.m_posit.m_y, m_matrix.m_posit.m_z);

	if (*m_mesh)
	{ 
		PrintTabs(level);
		fprintf(file, "\tmeshData:\n");
		PrintTabs(level);
		fprintf(file, "\t{\n");
		PrintTabs(level);
		fprintf(file, "\t\teulers: %f %f %f\n", euler.m_x, euler.m_y, euler.m_z);
		PrintTabs(level);
		fprintf(file, "\t\tposition: %f %f %f\n", m_meshMatrix.m_posit.m_x, m_meshMatrix.m_posit.m_y, m_meshMatrix.m_posit.m_z);
	
		ndSharedPtr<ndMeshEffect> effectMesh (GetMesh());
		ndIndexArray* const geometryHandle = effectMesh->MaterialGeometryBegin();
		
		// extract vertex data  from the newton mesh		
		ndInt32 indexCount = 0;
		ndInt32 vertexCount = effectMesh->GetPropertiesCount();
		ndInt32 materialsCount = 0;
		for (ndInt32 handle = effectMesh->GetFirstMaterial(geometryHandle); handle != -1; handle = effectMesh->GetNextMaterial(geometryHandle, handle))
		{
			materialsCount++;
			indexCount += effectMesh->GetMaterialIndexCount(geometryHandle, handle);
		}
		
		struct dTmpData
		{
			ndReal m_posit[3];
			ndReal m_normal[3];
			ndReal m_uv[2];
		};
		
		ndArray<dTmpData> tmp;
		ndArray<ndInt32> indices;
		
		tmp.SetCount(vertexCount);
		indices.SetCount(indexCount);
		
		effectMesh->GetVertexChannel(sizeof(dTmpData), &tmp[0].m_posit[0]);
		effectMesh->GetNormalChannel(sizeof(dTmpData), &tmp[0].m_normal[0]);
		effectMesh->GetUV0Channel(sizeof(dTmpData), &tmp[0].m_uv[0]);

		PrintTabs(level);
		fprintf(file, "\t\tpoints(x y x nx ny nz u v): %d\n", vertexCount);

		PrintTabs(level);
		fprintf(file, "\t\t{\n");

		for (ndInt32 i = 0; i < vertexCount; ++i)
		{
			PrintTabs(level);
			fprintf(file, "\t\t\t%f %f %f %f %f %f %f %f\n",
				tmp[i].m_posit[0], tmp[i].m_posit[1], tmp[i].m_posit[2],
				tmp[i].m_normal[0], tmp[i].m_normal[1], tmp[i].m_normal[2],
				tmp[i].m_uv[0], tmp[i].m_uv[1]);
		}
		PrintTabs(level);
		fprintf(file, "\t\t}\n");
	
		//PrintTabs(level);
		//fprintf(file, "\t\tmaterials: %d\n", materialsCount);

		ndInt32 segmentStart = 0;
		const ndArray<ndMeshEffect::ndMaterial>& materialArray = effectMesh->GetMaterials();
		for (ndInt32 handle = effectMesh->GetFirstMaterial(geometryHandle); handle != -1; handle = effectMesh->GetNextMaterial(geometryHandle, handle))
		{
			PrintTabs(level);
			fprintf(file, "\t\tmaterial:\n");
			PrintTabs(level);
			fprintf(file, "\t\t{\n");

			ndInt32 materialIndex = effectMesh->GetMaterialID(geometryHandle, handle);
		
			const ndMeshEffect::ndMaterial& material = materialArray[materialIndex];
	
			PrintTabs(level);
			fprintf(file, "\t\t\tambience: %f %f %f %f\n", material.m_ambient.m_x, material.m_ambient.m_y, material.m_ambient.m_z, material.m_ambient.m_w);
			PrintTabs(level);
			fprintf(file, "\t\t\tdiffuse: %f %f %f %f\n", material.m_diffuse.m_x, material.m_diffuse.m_y, material.m_diffuse.m_z, material.m_diffuse.m_w);
			PrintTabs(level);
			fprintf(file, "\t\t\topacity: %f\n", material.m_opacity);
			PrintTabs(level);
			fprintf(file, "\t\t\tshiness: %f\n", material.m_shiness);
			PrintTabs(level);
			fprintf(file, "\t\t\ttexture: %s\n", material.m_textureName);
		
			ndInt32 triangleIndexCount = effectMesh->GetMaterialIndexCount(geometryHandle, handle);
			effectMesh->GetMaterialGetIndexStream(geometryHandle, handle, &indices[segmentStart]);
	
			PrintTabs(level);
			fprintf(file, "\t\t\ttriangles: %d\n", triangleIndexCount / 3);
			PrintTabs(level);
			fprintf(file, "\t\t\t{\n");
			for (ndInt32 i = 0; i < triangleIndexCount; i += 3)
			{
				PrintTabs(level);
				fprintf(file, "\t\t\t\t%d %d %d\n", indices[segmentStart + i], indices[segmentStart + i + 1], indices[segmentStart + i + 2]);
			}
			PrintTabs(level);
			fprintf(file, "\t\t\t}\n");
			segmentStart += triangleIndexCount;

			PrintTabs(level);
			fprintf(file, "\t\t}\n");
		}
		effectMesh->MaterialGeometryEnd(geometryHandle);
	
		if (GetPositCurve().GetCount())
		{
			//ndAssert(0);
		}
		
		if (GetRotationCurve().GetCount())
		{
			//ndAssert(0);
		}

		PrintTabs(level);
		fprintf(file, "\t}\n");
	}

	for (ndMesh* child = GetFirstChild(); child; child = child->GetNext())
	{
		child->Save(file, level + 1);
	}

	PrintTabs(level);
	fprintf(file, "}\n");
}

ndMesh* ndMesh::Load(const char* const fullPathName)
{
	FILE* const file = fopen(fullPathName, "rb");
	if (file)
	{
		char token[256];
		fscanf(file, "%s", token);
		ndMesh* root = nullptr;
		if (!strcmp(token, "node:"))
		{
			root = new ndMesh(nullptr);
			root->Load(file);
		}

		fclose(file);
		return root;
	}

/*
	ndInt32 stack = 1;
	ndMesh* parentMesh[1024];

	char token[256];
	auto ReadToken = [file, &token]()
	{
		fscanf(file, "%s", token);
	};

	auto ReadMatrix = [file, &token]()
	{
		ndVector posit;
		ndVector eulers;
		fscanf(file, "%s", token);
		fscanf(file, "%f %f %f", &eulers.m_x, &eulers.m_y, &eulers.m_z);
		fscanf(file, "%s", token);
		fscanf(file, "%f %f %f", &posit.m_x, &posit.m_y, &posit.m_z);
		posit.m_w = ndFloat32(1.0f);

		ndMatrix matrix(ndPitchMatrix(eulers.m_x * ndDegreeToRad) * ndYawMatrix(eulers.m_y * ndDegreeToRad) * ndRollMatrix(eulers.m_z * ndDegreeToRad));
		matrix.m_posit = posit;
		return matrix;
	};

	ndMesh* rootMesh = nullptr;
	ndMesh* meshNode = nullptr;

	stack = 1;
	parentMesh[0] = rootMesh;
	while (!feof(file))
	{
		ReadToken();
		if (!strcmp(token, "node:"))
		{
			ReadToken();
			meshNode = new ndMesh(nullptr);
			map.Insert(meshNode, nodeNumber);
		}
		else if (!strcmp(token, "parentNode:"))
		{
			ndInt32 parentNumber;
			fscanf(file, "%d", &parentNumber);
			if (parentNumber == -1)
			{
				rootMesh = meshNode;
			}
			else
			{
				ndTree<ndMesh*, ndInt32>::ndNode* const parentNode = map.Find(parentNumber);
				ndAssert(parentNode);
				meshNode->Attach(parentNode->GetInfo());
			}
		}
		else if (!strcmp(token, "nodeName:"))
		{
			ReadToken();
			meshNode->SetName(token);
			meshNode->m_matrix = ReadMatrix();
		}
		else if (!strcmp(token, "meshData:"))
		{
			ReadToken();
			meshNode->m_meshMatrix = ReadMatrix();
			fscanf(file, "%s", token);
			fscanf(file, "%s", token);
			fscanf(file, "%s", token);
			fscanf(file, "%s", token);
			fscanf(file, "%s", token);
			fscanf(file, "%s", token);
			fscanf(file, "%s", token);
			fscanf(file, "%s", token);

			ndInt32 pointsCount;
			fscanf(file, "%d", &pointsCount);

			struct dTmpData
			{
				ndReal m_posit[3];
				ndReal m_normal[3];
				ndReal m_uv[2];
			};
			ndArray<dTmpData> tmp;
			for (ndInt32 i = 0; i < pointsCount; ++i)
			{
				dTmpData point;
				fscanf(file, "%f %f %f", &point.m_posit[0], &point.m_posit[1], &point.m_posit[2]);
				fscanf(file, "%f %f %f", &point.m_normal[0], &point.m_normal[1], &point.m_normal[2]);
				fscanf(file, "%f %f", &point.m_uv[0], &point.m_uv[1]);
				tmp.PushBack(point);
			}

			ndInt32 materialsCount;
			ReadToken();
			ndAssert(!strcmp(token, "materials:"));
			fscanf(file, "%d", &materialsCount);
			for (ndInt32 i = 0; i < materialsCount; ++i)
			{
				ReadToken();



				ReadToken();
			}

		}
		else if (!strcmp(token, "}"))
		{
			// close bracket, do nothing;
		}
		else
		{
			ndAssert(0);
		}
	}
*/

	return nullptr;
}

void ndMesh::Load(FILE* const file)
{
	char token[256];
	auto ReadToken = [file, &token]()
	{
		fscanf(file, "%s", token);
	};

	auto ReadMatrix = [file, &token]()
	{
		ndVector posit;
		ndVector eulers;
		fscanf(file, "%s", token);
		fscanf(file, "%f %f %f", &eulers.m_x, &eulers.m_y, &eulers.m_z);
		fscanf(file, "%s", token);
		fscanf(file, "%f %f %f", &posit.m_x, &posit.m_y, &posit.m_z);
		posit.m_w = ndFloat32(1.0f);

		ndMatrix matrix(ndPitchMatrix(eulers.m_x * ndDegreeToRad) * ndYawMatrix(eulers.m_y * ndDegreeToRad) * ndRollMatrix(eulers.m_z * ndDegreeToRad));
		matrix.m_posit = posit;
		return matrix;
	};

	struct dTmpData
	{
		ndReal m_posit[3];
		ndReal m_normal[3];
		ndReal m_uv[2];
	};
	ndArray<dTmpData> tmp;

	ReadToken();
	ReadToken();
	while (strcmp(token, "}"))
	{
		if (!strcmp(token, "node:"))
		{
			ndMesh* const child = new ndMesh(this);
			child->Load(file);
		}
		else if (!strcmp(token, "name:"))
		{
			ReadToken();
			SetName(token);
			m_matrix = ReadMatrix();
		}
		else if (!strcmp(token, "meshData:"))
		{
			ReadToken();
			m_meshMatrix = ReadMatrix();
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
				dTmpData point;
				fscanf(file, "%f %f %f", &point.m_posit[0], &point.m_posit[1], &point.m_posit[2]);
				fscanf(file, "%f %f %f", &point.m_normal[0], &point.m_normal[1], &point.m_normal[2]);
				fscanf(file, "%f %f", &point.m_uv[0], &point.m_uv[1]);
				tmp.PushBack(point);
			}
			ReadToken();
		}
		else if (!strcmp(token, "material:"))
		{
			ReadToken();
			ndAssert(0);

			ReadToken();
		}
		else
		{
			ndAssert(0);
		}
		ReadToken();
	}
}