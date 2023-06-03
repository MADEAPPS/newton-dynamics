/* Copyright (c) <2003-2022> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/

#include "ndSandboxStdafx.h"
#include "ndDemoMesh.h"
#include "ndDemoCamera.h"
#include "ndDemoEntity.h"
#include "ndDemoSkinMesh.h"
#include "ndTargaToOpenGl.h"
#include "ndDemoEntityManager.h"

class glSkinVertex : public glPositionNormalUV
{
	public:
	glVector4 m_weighs;
	glVector4 m_boneIndex;
};

ndDemoSkinMesh::ndDemoSkinMesh(ndDemoEntity* const owner, ndMeshEffect* const meshNode, const ndShaderCache& shaderCache)
	:ndDemoMeshInterface()
	,m_shareMesh(new ndDemoMesh(owner->GetName().GetStr()))
	,m_ownerEntity(owner)
	,m_bindingMatrixArray()
	,m_shader(0)
	,m_nodeCount(0)
	,m_matrixPalette(0)
{
	m_name = owner->GetName();
	m_shader = shaderCache.m_skinningDiffuseEffect;

	ndDemoEntity* root = owner;
	while (root->GetParent()) 
	{
		root = root->GetParent();
	}

	ndInt32 stack = 1;
	ndDemoEntity* pool[128];
	ndMatrix parentMatrix[128];
	ndArray<ndMatrix> bindMatrixArray;
	ndArray<ndDemoEntity*> entityArray;

	pool[0] = root;
	parentMatrix[0] = ndGetIdentityMatrix();
	ndMatrix shapeBindMatrix(m_ownerEntity->GetMeshMatrix() * m_ownerEntity->CalculateGlobalMatrix());

	//ndTree<ndInt32, ndString> boneClusterRemapIndex;
	//const ndMeshEffect::ndClusterMap& clusterMap = meshNode->GetCluster();
	
	ndTree<ndInt32, ndInt32> boneHashIdMap;
	while (stack) 
	{
		stack--;
		ndDemoEntity* const entity = pool[stack];
	//	ndMeshEffect::ndClusterMap::ndNode* const clusterNode = clusterMap.Find(entity->GetName());
	//	if (clusterNode) 
	//	{
	//		boneClusterRemapIndex.Insert(entityArray.GetCount(), entity->GetName());
	//	}
		ndInt32 hash = ndInt32(ndCRC64(entity->GetName().GetStr()) & 0xffffffff);
		boneHashIdMap.Insert(entityArray.GetCount(), hash);
	
		const ndMatrix boneMatrix(entity->GetCurrentMatrix() * parentMatrix[stack]);
		entityArray.PushBack(entity);
		bindMatrixArray.PushBack(shapeBindMatrix * boneMatrix.OrthoInverse());
	
		for (ndDemoEntity* node = entity->GetFirstChild(); node; node = node->GetNext()) 
		{
			pool[stack] = node;
			parentMatrix[stack] = boneMatrix;
			stack++;
			ndAssert(stack < sizeof(pool) / sizeof(pool[0]));
		}
	}
	
	m_nodeCount = entityArray.GetCount();
	m_bindingMatrixArray.SetCount(m_nodeCount);
	for (ndInt32 i = 0; i < m_nodeCount; ++i)
	{
		m_bindingMatrixArray[i] = bindMatrixArray[i];
	}
	
	//ndArray<ndVector> weight;
	//ndArray<ndWeightBoneIndex> skinBone;
	//weight.SetCount(meshNode->GetVertexCount());
	//skinBone.SetCount(meshNode->GetVertexCount());
	//
	//ndMemSet(&weight[0], ndVector::m_zero, meshNode->GetVertexCount());
	//ndMemSet(&skinBone[0], ndWeightBoneIndex(), meshNode->GetVertexCount());
	//
	//ndInt32 vCount = 0;
	//ndMeshEffect::ndClusterMap::Iterator iter(clusterMap);
	//for (iter.Begin(); iter; iter++) 
	//{
	//	const ndMeshEffect::ndVertexCluster* const cluster = &iter.GetNode()->GetInfo();
	//	ndInt32 boneIndex = boneClusterRemapIndex.Find(iter.GetKey())->GetInfo();
	//	for (ndInt32 i = 0; i < cluster->m_vertexIndex.GetCount(); ++i) 
	//	{
	//		ndInt32 vertexIndex = cluster->m_vertexIndex[i];
	//		vCount = ndMax(vertexIndex + 1, vCount);
	//		ndFloat32 vertexWeight = cluster->m_vertexWeigh[i];
	//		if (vertexWeight >= weight[vertexIndex][3]) 
	//		{
	//			weight[vertexIndex][3] = vertexWeight;
	//			skinBone[vertexIndex].m_boneIndex[3] = boneIndex;
	//		
	//			for (ndInt32 j = 2; j >= 0; --j)
	//			{
	//				if (weight[vertexIndex][j] < weight[vertexIndex][j + 1]) 
	//				{
	//					ndSwap(weight[vertexIndex][j], weight[vertexIndex][j + 1]);
	//					ndSwap(skinBone[vertexIndex].m_boneIndex[j], skinBone[vertexIndex].m_boneIndex[j + 1]);
	//				}
	//			}
	//		}
	//	}
	//}
	//
	//ndInt32 weightcount = 0;
	//for (ndInt32 i = 0; i < weight.GetCount(); ++i)
	//{
	//	ndVector w(weight[i]);
	//	ndFloat32 invMag = w.AddHorizontal().GetScalar();
	//	ndAssert(invMag > 0.0f);
	//	weight[i] = w.Scale(ndFloat32 (1.0f) / invMag);
	//
	//	ndAssert(skinBone[i].m_boneIndex[0] != -1);
	//	for (ndInt32 j = 0; j < 4; ++j) 
	//	{
	//		if (skinBone[i].m_boneIndex[j] != -1) 
	//		{
	//			weightcount = ndMax(weightcount, j + 1);
	//		}
	//		else 
	//		{
	//			skinBone[i].m_boneIndex[j] = 0;
	//		}
	//	}
	//}
	
	// extract the materials index array for mesh
	ndIndexArray* const geometryHandle = meshNode->MaterialGeometryBegin();
	
	// extract vertex data  from the newton mesh		
	ndInt32 indexCount = 0;
	ndInt32 vertexCount = meshNode->GetPropertiesCount();
	for (ndInt32 handle = meshNode->GetFirstMaterial(geometryHandle); handle != -1; handle = meshNode->GetNextMaterial(geometryHandle, handle))
	{
		indexCount += meshNode->GetMaterialIndexCount(geometryHandle, handle);
	}
	
	struct ndTmpData
	{
		ndFloat32 m_posit[3];
		ndFloat32 m_normal[3];
		ndFloat32 m_uv[2];
		ndMeshEffect::ndVertexWeight m_weights;
	};
	
	ndArray<ndTmpData> tmp;
	ndArray<ndInt32> indices;
	ndArray<ndInt32> vertexIndex;
	ndArray<glSkinVertex> points;
	
	indices.SetCount(indexCount);
	points.SetCount(vertexCount);
	tmp.SetCount(vertexCount);
	vertexIndex.SetCount(vertexCount);
	
	meshNode->GetVertexIndexChannel(&vertexIndex[0]);
	meshNode->GetVertexChannel(sizeof(ndTmpData), &tmp[0].m_posit[0]);
	meshNode->GetNormalChannel(sizeof(ndTmpData), &tmp[0].m_normal[0]);
	meshNode->GetUV0Channel(sizeof(ndTmpData), &tmp[0].m_uv[0]);
	meshNode->GetVertexWeightChannel(sizeof(ndTmpData), &tmp[0].m_weights);
	
	for (ndInt32 i = 0; i < vertexCount; ++i)
	{
		points[i].m_posit.m_x = GLfloat(tmp[i].m_posit[0]);
		points[i].m_posit.m_y = GLfloat(tmp[i].m_posit[1]);
		points[i].m_posit.m_z = GLfloat(tmp[i].m_posit[2]);
		points[i].m_normal.m_x = GLfloat(tmp[i].m_normal[0]);
		points[i].m_normal.m_y = GLfloat(tmp[i].m_normal[1]);
		points[i].m_normal.m_z = GLfloat(tmp[i].m_normal[2]);
		points[i].m_uv.m_u = GLfloat(tmp[i].m_uv[0]);
		points[i].m_uv.m_v = GLfloat(tmp[i].m_uv[1]);
	
		//ndInt32 k = vertexIndex[i];
		for (ndInt32 j = 0; j < 4; ++j)
		{
			//points[i].m_weighs[j] = GLfloat(weight[k][j]);
			//points[i].m_boneIndex[j] = GLfloat(skinBone[k].m_boneIndex[j]);

			ndInt32 boneIndex = 0;
			ndInt32 hashId = tmp[i].m_weights.m_boneId[j];
			if (hashId != -1)
			{
				ndTree<ndInt32, ndInt32>::ndNode* const entNode = boneHashIdMap.Find(hashId);
				ndAssert(entNode);
				boneIndex = entNode->GetInfo();
			}
			points[i].m_boneIndex[j] = GLfloat(boneIndex);
			points[i].m_weighs[j] = GLfloat(tmp[i].m_weights.m_weight[j]);
		}
	}
	
	ndInt32 segmentStart = 0;
	bool hasTransparency = false;
	const ndArray<ndMeshEffect::ndMaterial>& materialArray = meshNode->GetMaterials();
	
	ndDemoMesh* const shareMesh = (ndDemoMesh*)*m_shareMesh;
	for (ndInt32 handle = meshNode->GetFirstMaterial(geometryHandle); handle != -1; handle = meshNode->GetNextMaterial(geometryHandle, handle))
	{
		ndInt32 materialIndex = meshNode->GetMaterialID(geometryHandle, handle);
		ndDemoSubMesh* const segment = shareMesh->AddSubMesh();
	
		const ndMeshEffect::ndMaterial& material = materialArray[materialIndex];
		segment->m_material.m_ambient = glVector4(material.m_ambient);
		segment->m_material.m_diffuse = glVector4(material.m_diffuse);
		segment->m_material.m_specular = glVector4(material.m_specular);
		segment->m_material.m_opacity = GLfloat(material.m_opacity);
		segment->m_material.m_shiness = GLfloat(material.m_shiness);
		//strcpy(segment->m_material.m_textureName, material.m_textureName);
		segment->m_material.SetTextureName(material.m_textureName);
		ndInt32 tex = ndInt32(LoadTexture(material.m_textureName));
		segment->m_material.SetTexture(tex);
		ReleaseTexture(GLuint(tex));
		segment->SetOpacity(material.m_opacity);
		hasTransparency = hasTransparency | segment->m_hasTranparency;
	
		segment->m_indexCount = meshNode->GetMaterialIndexCount(geometryHandle, handle);
	
		segment->m_segmentStart = segmentStart;
		meshNode->GetMaterialGetIndexStream(geometryHandle, handle, &indices[segmentStart]);
		segmentStart += segment->m_indexCount;
	}
	meshNode->MaterialGeometryEnd(geometryHandle);
	shareMesh->m_hasTransparency = hasTransparency;
	
	// optimize this mesh for hardware buffers if possible
	CreateRenderMesh(&points[0], vertexCount, &indices[0], indexCount);
}

ndDemoSkinMesh::ndDemoSkinMesh(const ndDemoSkinMesh& source, ndDemoEntity* const owner)
	:ndDemoMeshInterface()
	,m_shareMesh(source.m_shareMesh)
	,m_ownerEntity(owner)
	,m_bindingMatrixArray()
	,m_shader(source.m_shader)
	,m_nodeCount(source.m_nodeCount)
	,m_matrixPalette(source.m_matrixPalette)
{
	m_bindingMatrixArray.SetCount(source.m_bindingMatrixArray.GetCount());
	ndMemCpy(&m_bindingMatrixArray[0], &source.m_bindingMatrixArray[0], source.m_bindingMatrixArray.GetCount());
}

ndDemoSkinMesh::~ndDemoSkinMesh()
{
}

ndDemoSkinMesh* ndDemoSkinMesh::GetAsDemoSkinMesh()
{ 
	return this; 
}

ndDemoMeshInterface* ndDemoSkinMesh::Clone(ndDemoEntity* const owner)
{
	return new ndDemoSkinMesh(*this, owner);
}

void ndDemoSkinMesh::CreateRenderMesh(
	const glSkinVertex* const points, ndInt32 pointCount,
	const ndInt32* const indices, ndInt32 indexCount)
{
	ndDemoMesh* const mesh = (ndDemoMesh*)*m_shareMesh;
	glGenVertexArrays(1, &mesh->m_vertextArrayBuffer);
	glBindVertexArray(mesh->m_vertextArrayBuffer);
	
	glGenBuffers(1, &mesh->m_vertexBuffer);
	glBindBuffer(GL_ARRAY_BUFFER, mesh->m_vertexBuffer);
	glBufferData(GL_ARRAY_BUFFER, GLsizeiptr(pointCount * sizeof(glSkinVertex)), &points[0], GL_STATIC_DRAW);
	
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glSkinVertex), (void*)OFFSETOF(glSkinVertex, m_posit));
	
	glEnableVertexAttribArray(1);
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(glSkinVertex), (void*)OFFSETOF(glSkinVertex, m_normal));
	
	glEnableVertexAttribArray(2);
	glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(glSkinVertex), (void*)OFFSETOF(glSkinVertex, m_uv));

	glEnableVertexAttribArray(3);
	glVertexAttribPointer(3, 4, GL_FLOAT, GL_FALSE, sizeof(glSkinVertex), (void*)OFFSETOF(glSkinVertex, m_weighs));

	glEnableVertexAttribArray(4);
	glVertexAttribPointer(4, 4, GL_FLOAT, GL_FALSE, sizeof(glSkinVertex), (void*)OFFSETOF(glSkinVertex, m_boneIndex));

	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindVertexArray(0);

	glDisableVertexAttribArray(4);
	glDisableVertexAttribArray(3);
	glDisableVertexAttribArray(2);
	glDisableVertexAttribArray(1);
	glDisableVertexAttribArray(0);
	
	glGenBuffers(1, &mesh->m_indexBuffer);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, mesh->m_indexBuffer);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, GLsizeiptr(indexCount * sizeof(GLuint)), &indices[0], GL_STATIC_DRAW);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
	
	glUseProgram(m_shader);
	mesh->m_textureLocation = glGetUniformLocation(m_shader, "texture");
	mesh->m_transparencyLocation = glGetUniformLocation(m_shader, "transparency");
	mesh->m_normalMatrixLocation = glGetUniformLocation(m_shader, "normalMatrix");
	mesh->m_projectMatrixLocation = glGetUniformLocation(m_shader, "projectionMatrix");
	mesh->m_viewModelMatrixLocation = glGetUniformLocation(m_shader, "viewModelMatrix");
	mesh->m_directionalLightDirLocation = glGetUniformLocation(m_shader, "directionalLightDir");
	
	mesh->m_materialAmbientLocation = glGetUniformLocation(m_shader, "material_ambient");
	mesh->m_materialDiffuseLocation = glGetUniformLocation(m_shader, "material_diffuse");
	mesh->m_materialSpecularLocation = glGetUniformLocation(m_shader, "material_specular");
	m_matrixPalette = glGetUniformLocation(m_shader, "matrixPallete");
	
	glUseProgram(0);
	mesh->m_vertexCount = pointCount;
	mesh->m_indexCount = indexCount;
}

ndInt32 ndDemoSkinMesh::CalculateMatrixPalette(ndMatrix* const bindMatrix) const
{
	int stack = 1;
	ndDemoEntity* pool[128];
	ndMatrix parentMatrix[128];

	ndDemoEntity* root = m_ownerEntity;
	while (root->GetParent()) 
	{
		root = root->GetParent();
	}

	int count = 0;
	pool[0] = root;
	parentMatrix[0] = ndGetIdentityMatrix();
	ndMatrix shapeBindMatrix((m_ownerEntity->GetMeshMatrix() * m_ownerEntity->CalculateGlobalMatrix()).OrthoInverse());
	while (stack) 
	{
		stack--;
		ndDemoEntity* const entity = pool[stack];
		ndMatrix boneMatrix(entity->GetCurrentMatrix() * parentMatrix[stack]);
		bindMatrix[count] = m_bindingMatrixArray[count] * boneMatrix * shapeBindMatrix;

		count++;
		ndAssert(count <= 128);
		ndAssert(count <= m_nodeCount);
		for (ndDemoEntity* node = entity->GetFirstChild(); node; node = node->GetNext()) 
		{
			pool[stack] = node;
			parentMatrix[stack] = boneMatrix;
			stack++;
		}
	}
	return count;
}

void ndDemoSkinMesh::Render(ndDemoEntityManager* const scene, const ndMatrix& modelMatrix)
{
	ndMatrix* const bindMatrix = ndAlloca(ndMatrix, m_nodeCount);
	ndInt32 count = CalculateMatrixPalette(bindMatrix);
	glMatrix* const glMatrixPallete = ndAlloca(glMatrix, count);
	for (ndInt32 i = 0; i < count; ++i)
	{
		glMatrixPallete[i] = bindMatrix[i];
	}

	glUseProgram(m_shader);

	ndDemoCamera* const camera = scene->GetCamera();

	const ndMatrix& viewMatrix = camera->GetViewMatrix();
	const glMatrix& projectionMatrix(camera->GetProjectionMatrix());
	const glMatrix viewModelMatrix(modelMatrix * viewMatrix);
	const glVector4 directionaLight(viewMatrix.RotateVector(ndVector(-1.0f, 1.0f, 0.0f, 0.0f)).Normalize());

	ndDemoMesh* const mesh = (ndDemoMesh*)*m_shareMesh;
	glUniform1i(mesh->m_textureLocation, 0);
	glUniform1f(mesh->m_transparencyLocation, 1.0f);
	glUniform4fv(mesh->m_directionalLightDirLocation, 1, &directionaLight[0]);
	glUniformMatrix4fv(mesh->m_normalMatrixLocation, 1, false, &viewModelMatrix[0][0]);
	glUniformMatrix4fv(mesh->m_projectMatrixLocation, 1, false, &projectionMatrix[0][0]);
	glUniformMatrix4fv(mesh->m_viewModelMatrixLocation, 1, false, &viewModelMatrix[0][0]);
	glUniformMatrix4fv(m_matrixPalette, count, GL_FALSE, &glMatrixPallete[0][0][0]);

	glBindVertexArray(mesh->m_vertextArrayBuffer);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, mesh->m_indexBuffer);

	glActiveTexture(GL_TEXTURE0);
	for (ndDemoMesh::ndNode* node = mesh->GetFirst(); node; node = node->GetNext())
	{
		ndDemoSubMesh& segment = node->GetInfo();
		if (!segment.m_hasTranparency)
		{
			glUniform3fv(mesh->m_materialAmbientLocation, 1, &segment.m_material.m_ambient[0]);
			glUniform3fv(mesh->m_materialDiffuseLocation, 1, &segment.m_material.m_diffuse[0]);
			glUniform3fv(mesh->m_materialSpecularLocation, 1, &segment.m_material.m_specular[0]);

			glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
			glBindTexture(GL_TEXTURE_2D, GLuint(segment.m_material.GetTexture()));
			glDrawElements(GL_TRIANGLES, segment.m_indexCount, GL_UNSIGNED_INT, (void*)(segment.m_segmentStart * sizeof(GL_UNSIGNED_INT)));
		}
	}

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
	glBindVertexArray(0);
	glUseProgram(0);
}
