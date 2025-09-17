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
#include "ndPngToOpenGl.h"
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

	ndFixSizeArray<ndDemoEntity*, 128> pool;
	ndFixSizeArray<ndMatrix, 128> parentMatrix;
	ndFixSizeArray<ndMatrix, 128> bindMatrixArray;
	ndFixSizeArray<ndDemoEntity*, 128> entityArray;

	parentMatrix.PushBack(ndGetIdentityMatrix());
	pool.PushBack((ndDemoEntity*)owner->GetRoot());
	ndMatrix shapeBindMatrix(m_ownerEntity->GetMeshMatrix() * m_ownerEntity->CalculateGlobalMatrix());
	
	ndTree<ndInt32, ndInt32> boneHashIdMap;
	while (pool.GetCount())
	{
		ndDemoEntity* const entity = pool.Pop();
		ndInt32 hash = ndInt32(ndCRC64(entity->GetName().GetStr()) & 0xffffffff);
		boneHashIdMap.Insert(ndInt32(entityArray.GetCount()), hash);

		const ndMatrix boneMatrix(entity->GetCurrentMatrix() * parentMatrix.Pop());
		const ndMatrix palleteMatrix(shapeBindMatrix * boneMatrix.OrthoInverse());
		entityArray.PushBack(entity);
		bindMatrixArray.PushBack(palleteMatrix);

		for (ndList<ndSharedPtr<ndDemoEntity>>::ndNode* node = entity->GetChildren().GetFirst(); node; node = node->GetNext())
		{
			pool.PushBack(*node->GetInfo());
			parentMatrix.PushBack(boneMatrix);
		}
	}
	
	m_nodeCount = ndInt32(entityArray.GetCount());
	m_bindingMatrixArray.SetCount(m_nodeCount);
	for (ndInt32 i = 0; i < m_nodeCount; ++i)
	{
		m_bindingMatrixArray[i] = bindMatrixArray[i];
	}
	
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
		glSkinVertex& glPoint = points[i];
		glPoint.m_posit.m_x = GLfloat(tmp[i].m_posit[0]);
		glPoint.m_posit.m_y = GLfloat(tmp[i].m_posit[1]);
		glPoint.m_posit.m_z = GLfloat(tmp[i].m_posit[2]);
		glPoint.m_normal.m_x = GLfloat(tmp[i].m_normal[0]);
		glPoint.m_normal.m_y = GLfloat(tmp[i].m_normal[1]);
		glPoint.m_normal.m_z = GLfloat(tmp[i].m_normal[2]);
		glPoint.m_uv.m_u = GLfloat(tmp[i].m_uv[0]);
		glPoint.m_uv.m_v = GLfloat(tmp[i].m_uv[1]);
		glPoint.m_weighs = glVector4();
		glPoint.m_boneIndex = glVector4();

		ndAssert(ND_VERTEX_WEIGHT_SIZE == 4);
		const ndMeshEffect::ndVertexWeight& weights = tmp[i].m_weights;

		for (ndInt32 j = 0; j < ND_VERTEX_WEIGHT_SIZE; ++j)
		{
			ndInt32 boneIndex = 0;
			ndInt32 hashId = weights.m_boneId[j];
			if (hashId != -1)
			{
				ndTree<ndInt32, ndInt32>::ndNode* const entNode = boneHashIdMap.Find(hashId);
				ndAssert(entNode);
				boneIndex = entNode->GetInfo();

				glPoint.m_boneIndex[j] = GLfloat(boneIndex);
				glPoint.m_weighs[j] = GLfloat(weights.m_weight[j]);
				ndAssert(weights.m_weight[j] <= ndFloat32(1.001f));
			}
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

	glGenBuffers(1, &mesh->m_indexBuffer);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, mesh->m_indexBuffer);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, GLsizeiptr(indexCount * sizeof(GLuint)), &indices[0], GL_STATIC_DRAW);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
	
	glUseProgram(m_shader);
	mesh->m_textureLocation = glGetUniformLocation(m_shader, "texture0");
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
	ndFixSizeArray <ndDemoEntity*, 128> pool;
	ndFixSizeArray <ndMatrix, 128> parentMatrix;

	ndInt32 count = 0;
	parentMatrix.PushBack(ndGetIdentityMatrix());
	pool.PushBack ((ndDemoEntity*)m_ownerEntity->GetRoot());
	ndMatrix shapeBindMatrix((m_ownerEntity->GetMeshMatrix() * m_ownerEntity->CalculateGlobalMatrix()).OrthoInverse());
	while (pool.GetCount())
	{
		ndDemoEntity* const entity = pool.Pop();
		ndMatrix boneMatrix(entity->GetCurrentMatrix() * parentMatrix.Pop());
		bindMatrix[count] = m_bindingMatrixArray[count] * boneMatrix * shapeBindMatrix;

		count++;
		ndAssert(count <= 128);
		ndAssert(count <= m_nodeCount);
		for (ndList<ndSharedPtr<ndDemoEntity>>::ndNode* node = entity->GetChildren().GetFirst(); node; node = node->GetNext())
		{
			pool.PushBack(*node->GetInfo());
			parentMatrix.PushBack(boneMatrix);
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

	const ndMatrix& viewMatrix = camera->GetInvViewMatrix();
	const glMatrix& projectionMatrix(camera->GetProjectionMatrix());
	const glMatrix viewModelMatrix(modelMatrix * viewMatrix);
	const glVector4 directionaLight(viewMatrix.RotateVector(scene->GetDirectionsLight()));

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

			//glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
			glBindTexture(GL_TEXTURE_2D, GLuint(segment.m_material.GetTexture()));
			glDrawElements(GL_TRIANGLES, segment.m_indexCount, GL_UNSIGNED_INT, (void*)(segment.m_segmentStart * sizeof(GL_UNSIGNED_INT)));
		}
	}

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
	glBindVertexArray(0);
	glUseProgram(0);
}
