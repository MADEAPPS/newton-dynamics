/* Copyright (c) <2003-2021> <Newton Game Dynamics>
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

ndDemoSkinMesh::ndDemoSkinMesh(ndDemoEntity* const owner, ndMeshEffect* const meshNode, const ndShaderPrograms& shaderCache)
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

	dInt32 stack = 1;
	ndDemoEntity* pool[128];
	ndMatrix parentMatrix[128];
	ndArray<ndMatrix> bindMatrixArray;
	ndArray<ndDemoEntity*> entityArray;

	pool[0] = root;
	parentMatrix[0] = dGetIdentityMatrix();
	ndMatrix shapeBindMatrix(m_ownerEntity->GetMeshMatrix() * m_ownerEntity->CalculateGlobalMatrix());

	ndTree<dInt32, ndString> boneClusterRemapIndex;
	const ndMeshEffect::dClusterMap& clusterMap = meshNode->GetCluster();
	
	while (stack) 
	{
		stack--;
		ndDemoEntity* const entity = pool[stack];
		const ndMatrix boneMatrix(entity->GetCurrentMatrix() * parentMatrix[stack]);
	
		const ndMatrix bindMatrix(shapeBindMatrix * boneMatrix.Inverse());
		entityArray.PushBack(entity);
		bindMatrixArray.PushBack(bindMatrix);
	
		ndMeshEffect::dClusterMap::ndNode* const clusterNode = clusterMap.Find(entity->GetName());
		if (clusterNode) 
		{
			boneClusterRemapIndex.Insert(entityArray.GetCount() - 1, entity->GetName());
		}
	
		for (ndDemoEntity* node = entity->GetChild(); node; node = node->GetSibling()) 
		{
			pool[stack] = node;
			parentMatrix[stack] = boneMatrix;
			stack++;
		}
	}
	
	m_nodeCount = entityArray.GetCount();
	m_bindingMatrixArray.SetCount(m_nodeCount);
	for (dInt32 i = 0; i < m_nodeCount; i++)
	{
		m_bindingMatrixArray[i] = bindMatrixArray[i];
	}
	
	ndArray<ndVector> weight;
	ndArray<dWeightBoneIndex> skinBone;
	weight.SetCount(meshNode->GetVertexCount());
	skinBone.SetCount(meshNode->GetVertexCount());
	memset(&weight[0], 0, meshNode->GetVertexCount() * sizeof(ndVector));
	memset(&skinBone[0], -1, meshNode->GetVertexCount() * sizeof(dWeightBoneIndex));
	
	dInt32 vCount = 0;
	ndMeshEffect::dClusterMap::Iterator iter(clusterMap);
	for (iter.Begin(); iter; iter++) 
	{
		const ndMeshEffect::dVertexCluster* const cluster = &iter.GetNode()->GetInfo();
		dInt32 boneIndex = boneClusterRemapIndex.Find(iter.GetKey())->GetInfo();
		for (dInt32 i = 0; i < cluster->m_vertexIndex.GetCount(); i++) 
		{
			dInt32 vertexIndex = cluster->m_vertexIndex[i];
			vCount = dMax(vertexIndex + 1, vCount);
			dFloat32 vertexWeight = cluster->m_vertexWeigh[i];
			if (vertexWeight >= weight[vertexIndex][3]) 
			{
				weight[vertexIndex][3] = vertexWeight;
				skinBone[vertexIndex].m_boneIndex[3] = boneIndex;
			
				for (dInt32 j = 2; j >= 0; j--) 
				{
					if (weight[vertexIndex][j] < weight[vertexIndex][j + 1]) 
					{
						dSwap(weight[vertexIndex][j], weight[vertexIndex][j + 1]);
						dSwap(skinBone[vertexIndex].m_boneIndex[j], skinBone[vertexIndex].m_boneIndex[j + 1]);
					}
				}
			}
		}
	}
	
	dInt32 weightcount = 0;
	for (dInt32 i = 0; i < weight.GetCount(); i++)
	{
		ndVector w(weight[i]);
		dFloat32 invMag = w.m_x + w.m_y + w.m_z + w.m_w;
		dAssert(invMag > 0.0f);
		invMag = 1.0f / invMag;
		weight[i].m_x = w.m_x * invMag;
		weight[i].m_y = w.m_y * invMag;
		weight[i].m_z = w.m_z * invMag;
		weight[i].m_w = w.m_w * invMag;
	
		dAssert(skinBone[i].m_boneIndex[0] != -1);
		for (dInt32 j = 0; j < 4; j++) 
		{
			if (skinBone[i].m_boneIndex[j] != -1) 
			{
				weightcount = dMax(weightcount, j + 1);
			}
			else 
			{
				skinBone[i].m_boneIndex[j] = 0;
			}
		}
	}

	// extract the materials index array for mesh
	ndIndexArray* const geometryHandle = meshNode->MaterialGeometryBegin();

	// extract vertex data  from the newton mesh		
	dInt32 indexCount = 0;
	dInt32 vertexCount = meshNode->GetPropertiesCount();
	for (dInt32 handle = meshNode->GetFirstMaterial(geometryHandle); handle != -1; handle = meshNode->GetNextMaterial(geometryHandle, handle))
	{
		indexCount += meshNode->GetMaterialIndexCount(geometryHandle, handle);
	}
	
	struct dTmpData
	{
		dFloat32 m_posit[3];
		dFloat32 m_normal[3];
		dFloat32 m_uv[2];
	};
	
	ndArray<dTmpData> tmp;
	ndArray<dInt32> indices;
	ndArray<dInt32> vertexIndex;
	ndArray<glSkinVertex> points;
	
	indices.SetCount(indexCount);
	points.SetCount(vertexCount);
	tmp.SetCount(vertexCount);
	vertexIndex.SetCount(vertexCount);
	
	meshNode->GetVertexChannel(sizeof(dTmpData), &tmp[0].m_posit[0]);
	meshNode->GetNormalChannel(sizeof(dTmpData), &tmp[0].m_normal[0]);
	meshNode->GetUV0Channel(sizeof(dTmpData), &tmp[0].m_uv[0]);
	meshNode->GetVertexIndexChannel(&vertexIndex[0]);

	for (dInt32 i = 0; i < vertexCount; i++)
	{
		points[i].m_posit.m_x = GLfloat(tmp[i].m_posit[0]);
		points[i].m_posit.m_y = GLfloat(tmp[i].m_posit[1]);
		points[i].m_posit.m_z = GLfloat(tmp[i].m_posit[2]);
		points[i].m_normal.m_x = GLfloat(tmp[i].m_normal[0]);
		points[i].m_normal.m_y = GLfloat(tmp[i].m_normal[1]);
		points[i].m_normal.m_z = GLfloat(tmp[i].m_normal[2]);
		points[i].m_uv.m_u = GLfloat(tmp[i].m_uv[0]);
		points[i].m_uv.m_v = GLfloat(tmp[i].m_uv[1]);

		dInt32 k = vertexIndex[i];
		for (dInt32 j = 0; j < 4; j++)
		{
			points[i].m_weighs[j] = GLfloat(weight[k][j]);
			points[i].m_boneIndex[j] = GLfloat(skinBone[k].m_boneIndex[j]);
		}
	}
	
	dInt32 segmentStart = 0;
	bool hasTransparency = false;
	const ndArray<ndMeshEffect::dMaterial>& materialArray = meshNode->GetMaterials();
	for (dInt32 handle = meshNode->GetFirstMaterial(geometryHandle); handle != -1; handle = meshNode->GetNextMaterial(geometryHandle, handle))
	{
		dInt32 materialIndex = meshNode->GetMaterialID(geometryHandle, handle);
		ndDemoSubMesh* const segment = m_shareMesh->AddSubMesh();
	
		const ndMeshEffect::dMaterial& material = materialArray[materialIndex];
		segment->m_material.m_ambient = glVector4(material.m_ambient);
		segment->m_material.m_diffuse = glVector4(material.m_diffuse);
		segment->m_material.m_specular = glVector4(material.m_specular);
		segment->m_material.m_opacity = GLfloat(material.m_opacity);
		segment->m_material.m_shiness = GLfloat(material.m_shiness);
		strcpy(segment->m_material.m_textureName, material.m_textureName);

		segment->m_material.m_textureHandle = LoadTexture(material.m_textureName);
		segment->SetOpacity(material.m_opacity);
		hasTransparency = hasTransparency | segment->m_hasTranparency;
	
		segment->m_indexCount = meshNode->GetMaterialIndexCount(geometryHandle, handle);
	
		segment->m_segmentStart = segmentStart;
		meshNode->GetMaterialGetIndexStream(geometryHandle, handle, &indices[segmentStart]);
		segmentStart += segment->m_indexCount;
	}
	meshNode->MaterialGeometryEnd(geometryHandle);
	m_shareMesh->m_hasTransparency = hasTransparency;

	// optimize this mesh for hardware buffers if possible
	CreateRenderMesh(&points[0], vertexCount, &indices[0], indexCount);
}

ndDemoSkinMesh::ndDemoSkinMesh(const ndDemoSkinMesh& source, ndDemoEntity* const owner)
	:ndDemoMeshInterface()
	,m_shareMesh((ndDemoMesh*)source.m_shareMesh->AddRef())
	,m_ownerEntity(owner)
	,m_bindingMatrixArray()
	,m_shader(source.m_shader)
	,m_nodeCount(source.m_nodeCount)
	,m_matrixPalette(source.m_matrixPalette)
{
	m_bindingMatrixArray.SetCount(source.m_bindingMatrixArray.GetCount());
	memcpy(&m_bindingMatrixArray[0], &source.m_bindingMatrixArray[0], source.m_bindingMatrixArray.GetCount() * sizeof(ndMatrix));
}

ndDemoSkinMesh::~ndDemoSkinMesh()
{
	m_shareMesh->Release();
}

ndDemoMeshInterface* ndDemoSkinMesh::Clone(ndDemoEntity* const owner)
{
	return (ndDemoSkinMesh*)new ndDemoSkinMesh(*this, owner);
}

void ndDemoSkinMesh::CreateRenderMesh(
	const glSkinVertex* const points, dInt32 pointCount,
	const dInt32* const indices, dInt32 indexCount)
{
	glGenVertexArrays(1, &m_shareMesh->m_vertextArrayBuffer);
	glBindVertexArray(m_shareMesh->m_vertextArrayBuffer);
	
	glGenBuffers(1, &m_shareMesh->m_vertexBuffer);
	glBindBuffer(GL_ARRAY_BUFFER, m_shareMesh->m_vertexBuffer);
	glBufferData(GL_ARRAY_BUFFER, pointCount * sizeof(glSkinVertex), &points[0], GL_STATIC_DRAW);
	
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
	
	glGenBuffers(1, &m_shareMesh->m_indexBuffer);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_shareMesh->m_indexBuffer);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, indexCount * sizeof(GLuint), &indices[0], GL_STATIC_DRAW);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
	
	glUseProgram(m_shader);
	m_shareMesh->m_textureLocation = glGetUniformLocation(m_shader, "texture");
	m_shareMesh->m_transparencyLocation = glGetUniformLocation(m_shader, "transparency");
	m_shareMesh->m_normalMatrixLocation = glGetUniformLocation(m_shader, "normalMatrix");
	m_shareMesh->m_projectMatrixLocation = glGetUniformLocation(m_shader, "projectionMatrix");
	m_shareMesh->m_viewModelMatrixLocation = glGetUniformLocation(m_shader, "viewModelMatrix");
	m_shareMesh->m_directionalLightDirLocation = glGetUniformLocation(m_shader, "directionalLightDir");
	
	m_shareMesh->m_materialAmbientLocation = glGetUniformLocation(m_shader, "material_ambient");
	m_shareMesh->m_materialDiffuseLocation = glGetUniformLocation(m_shader, "material_diffuse");
	m_shareMesh->m_materialSpecularLocation = glGetUniformLocation(m_shader, "material_specular");
	m_matrixPalette = glGetUniformLocation(m_shader, "matrixPallete");
	
	glUseProgram(0);
	m_shareMesh->m_vertexCount = pointCount;
	m_shareMesh->m_indexCount = indexCount;
}

dInt32 ndDemoSkinMesh::CalculateMatrixPalette(ndMatrix* const bindMatrix) const
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
	parentMatrix[0] = dGetIdentityMatrix();
	ndMatrix shapeBindMatrix((m_ownerEntity->GetMeshMatrix() * m_ownerEntity->CalculateGlobalMatrix()).Inverse());
	while (stack) 
	{
		stack--;
		ndDemoEntity* const entity = pool[stack];
		ndMatrix boneMatrix(entity->GetCurrentMatrix() * parentMatrix[stack]);
		bindMatrix[count] = m_bindingMatrixArray[count] * boneMatrix * shapeBindMatrix;

		count++;
		dAssert(count <= 128);
		dAssert(count <= m_nodeCount);
		for (ndDemoEntity* node = entity->GetChild(); node; node = node->GetSibling()) 
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
	ndMatrix* const bindMatrix = dAlloca(ndMatrix, m_nodeCount);
	dInt32 count = CalculateMatrixPalette(bindMatrix);
	glMatrix* const glMatrixPallete = dAlloca(glMatrix, count);
	for (dInt32 i = 0; i < count; i++)
	{
		glMatrixPallete[i] = bindMatrix[i];
	}

	glUseProgram(m_shader);

	ndDemoCamera* const camera = scene->GetCamera();

	const ndMatrix& viewMatrix = camera->GetViewMatrix();
	const glMatrix& projectionMatrix(camera->GetProjectionMatrix());
	const glMatrix viewModelMatrix(modelMatrix * viewMatrix);
	const glVector4 directionaLight(viewMatrix.RotateVector(ndVector(-1.0f, 1.0f, 0.0f, 0.0f)).Normalize());

	glUniform1i(m_shareMesh->m_textureLocation, 0);
	glUniform1f(m_shareMesh->m_transparencyLocation, 1.0f);
	glUniform4fv(m_shareMesh->m_directionalLightDirLocation, 1, &directionaLight[0]);
	glUniformMatrix4fv(m_shareMesh->m_normalMatrixLocation, 1, false, &viewModelMatrix[0][0]);
	glUniformMatrix4fv(m_shareMesh->m_projectMatrixLocation, 1, false, &projectionMatrix[0][0]);
	glUniformMatrix4fv(m_shareMesh->m_viewModelMatrixLocation, 1, false, &viewModelMatrix[0][0]);
	glUniformMatrix4fv(m_matrixPalette, count, GL_FALSE, &glMatrixPallete[0][0][0]);

	glBindVertexArray(m_shareMesh->m_vertextArrayBuffer);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_shareMesh->m_indexBuffer);

	glActiveTexture(GL_TEXTURE0);
	for (ndDemoMesh::ndNode* node = m_shareMesh->GetFirst(); node; node = node->GetNext())
	{
		ndDemoSubMesh& segment = node->GetInfo();
		if (!segment.m_hasTranparency)
		{
			glUniform3fv(m_shareMesh->m_materialAmbientLocation, 1, &segment.m_material.m_ambient[0]);
			glUniform3fv(m_shareMesh->m_materialDiffuseLocation, 1, &segment.m_material.m_diffuse[0]);
			glUniform3fv(m_shareMesh->m_materialSpecularLocation, 1, &segment.m_material.m_specular[0]);

			glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
			glBindTexture(GL_TEXTURE_2D, segment.m_material.m_textureHandle);
			glDrawElements(GL_TRIANGLES, segment.m_indexCount, GL_UNSIGNED_INT, (void*)(segment.m_segmentStart * sizeof(GL_UNSIGNED_INT)));
		}
	}

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
	glBindVertexArray(0);
	glUseProgram(0);
}
