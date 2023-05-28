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
	FILE* const file = fopen(fullPathName, "rb");
	if (!file)
	{
		ndAssert(0);
		return nullptr;
	}
	ndAssert(0);

	fclose(file);
	return nullptr;
}

void ndMesh::Save(const ndMesh* const mesh, const char* const fullPathName)
{
	ndInt32 stack = 1;
	ndInt32 indexStack[1024];
	const ndMesh* entBuffer[1024];

	FILE* const file = fopen(fullPathName, "wb");
	if (!file)
	{
		ndAssert(0);
		return;
	}

	ndInt32 index = 0;
	indexStack[0] = -1;
	entBuffer[0] = mesh;
	while (stack)
	{
		stack--;
		const ndMesh* const node = entBuffer[stack];
		
		fprintf(file, "node: %d\n", index);
		fprintf(file, "{\n");
		fprintf(file, "\tparentNode: %d\n", indexStack[stack]);
		fprintf(file, "\tnodeName: %s\n", node->GetName().GetStr());

		ndVector euler1;
		ndVector euler(node->m_matrix.CalcPitchYawRoll(euler1).Scale(ndRadToDegree));
		fprintf(file, "\teulers: %f %f %f\n", euler.m_x, euler.m_y, euler.m_z);
		fprintf(file, "\tposition: %f %f %f\n", node->m_matrix.m_posit.m_x, node->m_matrix.m_posit.m_y, node->m_matrix.m_posit.m_z);

		if (*node->m_mesh)
		{ 
			euler = node->m_meshMatrix.CalcPitchYawRoll(euler1).Scale(ndRadToDegree);

			fprintf(file, "\tmeshData: 1\n");
			fprintf(file, "\t{\n");
			fprintf(file, "\t\tmeshEulers: %f %f %f\n", euler.m_x, euler.m_y, euler.m_z);
			fprintf(file, "\t\tmeshPosition: %f %f %f\n", node->m_meshMatrix.m_posit.m_x, node->m_meshMatrix.m_posit.m_y, node->m_meshMatrix.m_posit.m_z);

			ndSharedPtr<ndMeshEffect> effectMesh (node->GetMesh());
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
			
			fprintf(file, "\t\tpoints(x y x nx ny nz u v): %d\n", vertexCount);
			for (ndInt32 i = 0; i < vertexCount; ++i)
			{
				fprintf(file, "\t\t\t%f %f %f %f %f %f %f %f\n",
					tmp[i].m_posit[0], tmp[i].m_posit[1], tmp[i].m_posit[2],
					tmp[i].m_normal[0], tmp[i].m_normal[1], tmp[i].m_normal[2],
					tmp[i].m_uv[0], tmp[i].m_uv[1]);
			}
			
			fprintf(file, "\t\tmaterials: %d\n", materialsCount);
			fprintf(file, "\t\t{\n");
			ndInt32 segmentStart = 0;
			const ndArray<ndMeshEffect::ndMaterial>& materialArray = effectMesh->GetMaterials();
			for (ndInt32 handle = effectMesh->GetFirstMaterial(geometryHandle); handle != -1; handle = effectMesh->GetNextMaterial(geometryHandle, handle))
			{
				ndInt32 materialIndex = effectMesh->GetMaterialID(geometryHandle, handle);
			
				const ndMeshEffect::ndMaterial& material = materialArray[materialIndex];

				fprintf(file, "\t\t\tambience: %f %f %f %f\n", material.m_ambient.m_x, material.m_ambient.m_y, material.m_ambient.m_z, material.m_ambient.m_w);
				fprintf(file, "\t\t\tdiffuse: %f %f %f %f\n", material.m_diffuse.m_x, material.m_diffuse.m_y, material.m_diffuse.m_z, material.m_diffuse.m_w);
				fprintf(file, "\t\t\topacity: %f\n", material.m_opacity);
				fprintf(file, "\t\t\tshiness: %f\n", material.m_shiness);
				fprintf(file, "\t\t\ttexture: %s\n", material.m_textureName);
			
				ndInt32 triangleIndexCount = effectMesh->GetMaterialIndexCount(geometryHandle, handle);
				effectMesh->GetMaterialGetIndexStream(geometryHandle, handle, &indices[segmentStart]);

				fprintf(file, "\t\t\ttriangles: %d\n", triangleIndexCount / 3);
				for (ndInt32 i = 0; i < triangleIndexCount; i += 3)
				{
					fprintf(file, "\t\t\t\t%d %d %d\n", indices[segmentStart + i], indices[segmentStart + i + 1], indices[segmentStart + i + 2]);
				}

				segmentStart += triangleIndexCount;
			}
			fprintf(file, "\t\t}\n");
			effectMesh->MaterialGeometryEnd(geometryHandle);

			fprintf(file, "\t}\n");
		}

		if (node->GetScaleCurve().GetCount())
		{
			ndAssert(0);
			//ndMesh::ndCurve::ndNode* positNode = ent->GetPositCurve().GetFirst();
			//ndMesh::ndCurve::ndNode* rotationNode = ent->GetRotationCurve().GetFirst();
			//
			//ndMesh::ndCurveValue scaleValue;
			//scaleValue.m_x = 1.0f;
			//scaleValue.m_y = 1.0f;
			//scaleValue.m_z = 1.0f;
			//for (ndInt32 i = 0; i < ent->GetScaleCurve().GetCount(); ++i)
			//{
			//	ndMesh::ndCurveValue& positValue = positNode->GetInfo();
			//	ndMesh::ndCurveValue& rotationValue = rotationNode->GetInfo();
			//
			//	ndVector animScale;
			//	ndMatrix stretchAxis;
			//	ndMatrix animTransformMatrix;
			//	ndMatrix keyframe(invTransform * GetKeyframe(scaleValue, positValue, rotationValue) * transform);
			//	keyframe.PolarDecomposition(animTransformMatrix, animScale, stretchAxis);
			//
			//	ndVector euler0;
			//	ndVector euler(animTransformMatrix.CalcPitchYawRoll(euler0));
			//
			//	rotationValue.m_x = euler.m_x;
			//	rotationValue.m_y = euler.m_y;
			//	rotationValue.m_z = euler.m_z;
			//
			//	positValue.m_x = animTransformMatrix.m_posit.m_x;
			//	positValue.m_y = animTransformMatrix.m_posit.m_y;
			//	positValue.m_z = animTransformMatrix.m_posit.m_z;
			//
			//	positNode = positNode->GetNext();
			//	rotationNode = rotationNode->GetNext();
			//}
		}

		if (node->GetPositCurve().GetCount())
		{
			//ndAssert(0);
		}

		if (node->GetRotationCurve().GetCount())
		{
			//ndAssert(0);
		}

		fprintf(file, "}\n\n");
		for (ndMesh* child = node->GetFirstChild(); child; child = child->GetNext())
		{
			entBuffer[stack] = child;
			indexStack[stack] = index;
			stack++;
			ndAssert(stack < sizeof(entBuffer) / sizeof(entBuffer[0]));
		}
		index++;
	}

	fclose(file);
}
