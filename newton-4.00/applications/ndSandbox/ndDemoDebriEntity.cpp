/* Copyright (c) <2003-2019> <Newton Game Dynamics>
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
#include "ndDemoCamera.h"
#include "ndTargaToOpenGl.h"
#include "ndDemoDebriEntity.h"
#include "ndDemoEntityManager.h"


ndDemoDebriMesh::ndDemoDebriMesh(const char* const name, ndMeshEffect* const meshNode, const ndShaderPrograms& shaderCache)
	:ndDemoMesh(name)
{
	m_name = name;
	m_shader = shaderCache.m_diffuseDebriEffect;

	// extract the materials index array for mesh
	ndIndexArray* const geometryHandle = meshNode->MaterialGeometryBegin();

	// extract vertex data  from the newton mesh		
	dInt32 indexCount = 0;
	dInt32 vertexCount = meshNode->GetPropertiesCount();
	for (dInt32 handle = meshNode->GetFirstMaterial(geometryHandle); handle != -1; handle = meshNode->GetNextMaterial(geometryHandle, handle))
	{
		indexCount += meshNode->GetMaterialIndexCount(geometryHandle, handle);
	}

	dInt32* const indices = dAlloca(dInt32, indexCount);
	DebriPoint* const points = dAlloca(DebriPoint, vertexCount);

	meshNode->GetVertexChannel(sizeof(DebriPoint), &points[0].m_posit.m_x);
	meshNode->GetNormalChannel(sizeof(DebriPoint), &points[0].m_normal.m_x);
	meshNode->GetUV0Channel(sizeof(DebriPoint), &points[0].m_uv.m_u);
	
	dInt32 segmentStart = 0;
	dInt32 materialCount = 0;
	const dArray<ndMeshEffect::dMaterial>& materialArray = meshNode->GetMaterials();
	for (dInt32 handle = meshNode->GetFirstMaterial(geometryHandle); handle != -1; handle = meshNode->GetNextMaterial(geometryHandle, handle))
	{
		dInt32 materialIndex = meshNode->GetMaterialID(geometryHandle, handle);
		const ndMeshEffect::dMaterial& material = materialArray[materialIndex];

		m_material[materialCount].m_ambient = material.m_ambient;
		m_material[materialCount].m_diffuse = material.m_diffuse;
		m_material[materialCount].m_specular = material.m_specular;
		m_material[materialCount].m_opacity = material.m_opacity;
		m_material[materialCount].m_shiness = material.m_shiness;
		strcpy(m_material[materialCount].m_textureName, material.m_textureName);
		m_material[materialCount].m_textureHandle = LoadTexture(material.m_textureName);

		dInt32 subIndexCount = meshNode->GetMaterialIndexCount(geometryHandle, handle);
		meshNode->GetMaterialGetIndexStream(geometryHandle, handle, &indices[segmentStart]);

		dFloat32 blend = materialCount ? 0.0f : 1.0f;
		for (dInt32 i = 0; i < subIndexCount; i++)
		{
			dInt32 index = indices[segmentStart + i];
			points[index].m_posit.m_w = blend;
		}
		
		materialCount++;
		segmentStart += subIndexCount;
	}

	meshNode->MaterialGeomteryEnd(geometryHandle);

	glGenVertexArrays(1, &m_vertextArrayBuffer);
	glBindVertexArray(m_vertextArrayBuffer);

	glGenBuffers(1, &m_vertexBuffer);
	glBindBuffer(GL_ARRAY_BUFFER, m_vertexBuffer);
	glBufferData(GL_ARRAY_BUFFER, vertexCount * sizeof(DebriPoint), &points[0], GL_STATIC_DRAW);

	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, sizeof(DebriPoint), (void*)0);

	glEnableVertexAttribArray(1);
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(DebriPoint), (void*)sizeof(ndMeshVector4));

	glEnableVertexAttribArray(2);
	glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(DebriPoint), (void*)(sizeof (ndMeshVector4) + sizeof(ndMeshVector)));
	glBindBuffer(GL_ARRAY_BUFFER, 0);

	glBindVertexArray(0);
	glDisableVertexAttribArray(2);
	glDisableVertexAttribArray(1);
	glDisableVertexAttribArray(0);

	glGenBuffers(1, &m_indexBuffer);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_indexBuffer);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, indexCount * sizeof(GLuint), &indices[0], GL_STATIC_DRAW);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

	glUseProgram(m_shader);
	m_textureLocation = glGetUniformLocation(m_shader, "texture0");
	m_textureLocation1 = glGetUniformLocation(m_shader, "texture1");
	m_transparencyLocation = glGetUniformLocation(m_shader, "transparency");
	m_normalMatrixLocation = glGetUniformLocation(m_shader, "normalMatrix");
	m_projectMatrixLocation = glGetUniformLocation(m_shader, "projectionMatrix");
	m_viewModelMatrixLocation = glGetUniformLocation(m_shader, "viewModelMatrix");
	m_directionalLightDirLocation = glGetUniformLocation(m_shader, "directionalLightDir");

	m_materialAmbientLocation = glGetUniformLocation(m_shader, "material_ambient");
	m_materialDiffuseLocation = glGetUniformLocation(m_shader, "material_diffuse");
	m_materialSpecularLocation = glGetUniformLocation(m_shader, "material_specular");

	glUseProgram(0);

	m_indexCount = indexCount;
	m_vertexCount = vertexCount;
}

void ndDemoDebriMesh::Render(ndDemoEntityManager* const scene, const dMatrix& modelMatrix)
{
	glUseProgram(m_shader);

	ndDemoCamera* const camera = scene->GetCamera();

	const dMatrix& viewMatrix = camera->GetViewMatrix();
	const dMatrix& projectionMatrix = camera->GetProjectionMatrix();
	dMatrix viewModelMatrix(modelMatrix * viewMatrix);
	dVector directionaLight(viewMatrix.RotateVector(dVector(-1.0f, 1.0f, 0.0f, 0.0f)).Normalize());

	glUniform1i(m_textureLocation, 0);
	glUniform1i(m_textureLocation1, 1);
	glUniform1f(m_transparencyLocation, 1.0f);
	glUniform4fv(m_directionalLightDirLocation, 1, &directionaLight.m_x);
	glUniformMatrix4fv(m_normalMatrixLocation, 1, false, &viewModelMatrix[0][0]);
	glUniformMatrix4fv(m_projectMatrixLocation, 1, false, &projectionMatrix[0][0]);
	glUniformMatrix4fv(m_viewModelMatrixLocation, 1, false, &viewModelMatrix[0][0]);

	glBindVertexArray(m_vertextArrayBuffer);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_indexBuffer);

	glUniform3fv(m_materialAmbientLocation, 1, &m_material[0].m_ambient.m_x);
	glUniform3fv(m_materialDiffuseLocation, 1, &m_material[0].m_diffuse.m_x);
	glUniform3fv(m_materialSpecularLocation, 1, &m_material[0].m_specular.m_x);

	// these call make the font display wrong
	glActiveTexture(GL_TEXTURE1);
	glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
	glBindTexture(GL_TEXTURE_2D, m_material[1].m_textureHandle);

	glActiveTexture(GL_TEXTURE0);
	glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
	glBindTexture(GL_TEXTURE_2D, m_material[0].m_textureHandle);

	glDrawElements(GL_TRIANGLES, m_indexCount, GL_UNSIGNED_INT, 0);

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
	glBindVertexArray(0);
	glUseProgram(0);
}

//ndDemoDebriMesh2::ndDemoDebriMesh2(const char* const name, dArray<ndMeshPointUV>& vertexArray, dArray<dInt32>& indexArray, ndMeshEffect* const meshNode, const ndShaderPrograms& shaderCache)
ndDemoDebriMesh2::ndDemoDebriMesh2(const char* const name, ndMeshEffect* const meshNode, const ndShaderPrograms& shaderCache)
	:ndDemoMesh(name)
{
/*
	dInt32 indexCount = 0;
	ndIndexArray* const geometryHandle = meshNode->MaterialGeometryBegin();
	dInt32 vertexCount = meshNode->GetPropertiesCount();
	for (dInt32 handle = meshNode->GetFirstMaterial(geometryHandle); handle != -1; handle = meshNode->GetNextMaterial(geometryHandle, handle))
	{
		indexCount += meshNode->GetMaterialIndexCount(geometryHandle, handle);
	}

	const dInt32 base = vertexArray.GetCount();
	dInt32 segmentStart = indexArray.GetCount();
	indexArray.SetCount(indexArray.GetCount() + indexCount);
	vertexArray.SetCount(vertexArray.GetCount() + vertexCount);

	meshNode->GetVertexChannel(sizeof(ndMeshPointUV), &vertexArray[base].m_posit.m_x);
	meshNode->GetNormalChannel(sizeof(ndMeshPointUV), &vertexArray[base].m_normal.m_x);
	meshNode->GetUV0Channel(sizeof(ndMeshPointUV), &vertexArray[base].m_uv.m_u);
	
	for (dInt32 handle = meshNode->GetFirstMaterial(geometryHandle); handle != -1; handle = meshNode->GetNextMaterial(geometryHandle, handle))
	{
		dInt32 material = meshNode->GetMaterialID(geometryHandle, handle);
		ndDemoSubMesh* const segment = AddSubMesh();
	
		segment->m_material.m_textureHandle = (GLuint)material;
		segment->m_indexCount = meshNode->GetMaterialIndexCount(geometryHandle, handle);
	
		segment->m_segmentStart = segmentStart;
		meshNode->GetMaterialGetIndexStream(geometryHandle, handle, &indexArray[segmentStart]);
		for (dInt32 i = 0; i < segment->m_indexCount; i++)
		{
			indexArray[segmentStart + i] += base;
		}
		segmentStart += segment->m_indexCount;
	}
	meshNode->MaterialGeomteryEnd(geometryHandle);
*/

	m_name = name;
	m_shader = shaderCache.m_diffuseDebriEffect;

	// extract the materials index array for mesh
	ndIndexArray* const geometryHandle = meshNode->MaterialGeometryBegin();

	// extract vertex data  from the newton mesh		
	dInt32 indexCount = 0;
	for (dInt32 handle = meshNode->GetFirstMaterial(geometryHandle); handle != -1; handle = meshNode->GetNextMaterial(geometryHandle, handle))
	{
		indexCount += meshNode->GetMaterialIndexCount(geometryHandle, handle);
	}

	dInt32* const indices = dAlloca(dInt32, indexCount);

	dInt32 segmentStart = 0;
	dInt32 materialCount = 0;
	const dArray<ndMeshEffect::dMaterial>& materialArray = meshNode->GetMaterials();
	for (dInt32 handle = meshNode->GetFirstMaterial(geometryHandle); handle != -1; handle = meshNode->GetNextMaterial(geometryHandle, handle))
	{
		dInt32 materialIndex = meshNode->GetMaterialID(geometryHandle, handle);
		const ndMeshEffect::dMaterial& material = materialArray[materialIndex];

		m_material[materialCount].m_ambient = material.m_ambient;
		m_material[materialCount].m_diffuse = material.m_diffuse;
		m_material[materialCount].m_specular = material.m_specular;
		m_material[materialCount].m_opacity = material.m_opacity;
		m_material[materialCount].m_shiness = material.m_shiness;
		strcpy(m_material[materialCount].m_textureName, material.m_textureName);
		m_material[materialCount].m_textureHandle = LoadTexture(material.m_textureName);

		dInt32 subIndexCount = meshNode->GetMaterialIndexCount(geometryHandle, handle);
		meshNode->GetMaterialGetIndexStream(geometryHandle, handle, &indices[segmentStart]);

		//dFloat32 blend = materialCount ? 0.0f : 1.0f;
		//for (dInt32 i = 0; i < subIndexCount; i++)
		//{
		//	dInt32 index = indices[segmentStart + i];
		//	points[index].m_posit.m_w = blend;
		//}

		materialCount++;
		segmentStart += subIndexCount;
	}

	meshNode->MaterialGeomteryEnd(geometryHandle);

	
	glGenBuffers(1, &m_indexBuffer);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_indexBuffer);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, indexCount * sizeof(GLuint), &indices[0], GL_STATIC_DRAW);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
	
	glUseProgram(m_shader);
	m_textureLocation = glGetUniformLocation(m_shader, "texture0");
	m_textureLocation1 = glGetUniformLocation(m_shader, "texture1");
	m_transparencyLocation = glGetUniformLocation(m_shader, "transparency");
	m_normalMatrixLocation = glGetUniformLocation(m_shader, "normalMatrix");
	m_projectMatrixLocation = glGetUniformLocation(m_shader, "projectionMatrix");
	m_viewModelMatrixLocation = glGetUniformLocation(m_shader, "viewModelMatrix");
	m_directionalLightDirLocation = glGetUniformLocation(m_shader, "directionalLightDir");
	
	m_materialAmbientLocation = glGetUniformLocation(m_shader, "material_ambient");
	m_materialDiffuseLocation = glGetUniformLocation(m_shader, "material_diffuse");
	m_materialSpecularLocation = glGetUniformLocation(m_shader, "material_specular");
	
	glUseProgram(0);

	m_indexCount = indexCount;
}

ndDemoDebriMesh2::~ndDemoDebriMesh2()
{
}

ndDemoDebriEntity::ndDemoDebriEntity(ndMeshEffect* const meshNode, dArray<DebriPoint>& vertexArray, ndDemoDebriEntityRoot* const parent, const ndShaderPrograms& shaderCache)
	:ndDemoEntity(dGetIdentityMatrix(), parent)
	,m_vertexOffestBase(vertexArray.GetCount())
{
	dInt32 vertexCount = meshNode->GetPropertiesCount();
	vertexArray.SetCount(m_vertexOffestBase + vertexCount);
	meshNode->GetVertexChannel(sizeof(DebriPoint), &vertexArray[m_vertexOffestBase].m_posit.m_x);
	meshNode->GetNormalChannel(sizeof(DebriPoint), &vertexArray[m_vertexOffestBase].m_normal.m_x);
	meshNode->GetUV0Channel(sizeof(DebriPoint), &vertexArray[m_vertexOffestBase].m_uv.m_u);

	ndDemoDebriMesh2* const mesh = new ndDemoDebriMesh2("fracture", meshNode, shaderCache);
	SetMesh(mesh, dGetIdentityMatrix());
	mesh->Release();

	dInt32 indices[1024 * 4];
	dInt32 segmentStart = 0;
	dInt32 materialCount = 0;
	ndIndexArray* const geometryHandle = meshNode->MaterialGeometryBegin();
	for (dInt32 handle = meshNode->GetFirstMaterial(geometryHandle); handle != -1; handle = meshNode->GetNextMaterial(geometryHandle, handle))
	{
		dInt32 subIndexCount = meshNode->GetMaterialIndexCount(geometryHandle, handle);
		meshNode->GetMaterialGetIndexStream(geometryHandle, handle, &indices[segmentStart]);
	
		dFloat32 blend = materialCount ? 0.0f : 1.0f;
		for (dInt32 i = 0; i < subIndexCount; i++)
		{
			dInt32 index = indices[segmentStart + i];
			vertexArray[m_vertexOffestBase + index].m_posit.m_w = blend;
		}
		materialCount++;
		segmentStart += subIndexCount;
	}
	meshNode->MaterialGeomteryEnd(geometryHandle);
}

ndDemoDebriEntity::ndDemoDebriEntity(const ndDemoDebriEntity& copyFrom)
	:ndDemoEntity(copyFrom)
	,m_vertexOffestBase(copyFrom.m_vertexOffestBase)
{
}

ndDemoDebriEntity::~ndDemoDebriEntity()
{

}

dNodeBaseHierarchy* ndDemoDebriEntity::CreateClone() const
{
	return new ndDemoDebriEntity(*this);
}

ndDemoDebriEntityRoot::ndDemoDebriEntityRoot(const ndDemoDebriEntityRoot& copyFrom)
	:ndDemoEntity(copyFrom)
	,m_vertexCount(copyFrom.m_vertexCount)
	,m_buffRefCount(copyFrom.m_vertexCount + 1)
	,m_vertexBuffer(copyFrom.m_vertexBuffer)
	,m_vertextArrayBuffer(copyFrom.m_vertextArrayBuffer)
{
}

ndDemoDebriEntityRoot::ndDemoDebriEntityRoot()
	:ndDemoEntity(dGetIdentityMatrix(), nullptr)
	,m_vertexCount(0)
	,m_buffRefCount(0)
	,m_vertexBuffer(0)
	,m_vertextArrayBuffer(0)
{
}

ndDemoDebriEntityRoot::~ndDemoDebriEntityRoot(void)
{
	m_buffRefCount--;
	if ((m_buffRefCount <= 0) && m_vertextArrayBuffer)
	{
		glDeleteBuffers(1, &m_vertexBuffer);
		glDeleteVertexArrays(1, &m_vertextArrayBuffer);
	}
}

void ndDemoDebriEntityRoot::FinalizeConstruction(dArray<DebriPoint>& vertexArray)
{
//	dArray<dInt32> remapIndex;
//	remapIndex.SetCount(vertexArray.GetCount());
//	dInt32 vertexCount = dVertexListToIndexList(&vertexArray[0].m_posit.m_x, sizeof(ndMeshPointUV), sizeof(ndMeshPointUV), 0, vertexArray.GetCount(), &remapIndex[0], dFloat32(1.0e-6f));
//
//	for (dInt32 i = 0; i < indexArray.GetCount(); i++)
//	{
//		dInt32 index = indexArray[i];
//		dAssert(index < remapIndex.GetCount());
//		indexArray[i] = remapIndex[index];
//	}

	m_buffRefCount++;
	glGenVertexArrays(1, &m_vertextArrayBuffer);
	glBindVertexArray(m_vertextArrayBuffer);
	
	m_vertexCount = vertexArray.GetCount();
	glGenBuffers(1, &m_vertexBuffer);
	glBindBuffer(GL_ARRAY_BUFFER, m_vertexBuffer);
	glBufferData(GL_ARRAY_BUFFER, m_vertexCount * sizeof(DebriPoint), &vertexArray[0].m_posit.m_x, GL_STATIC_DRAW);
	
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, sizeof(DebriPoint), (void*)0);
	
	glEnableVertexAttribArray(1);
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(DebriPoint), (void*)sizeof(ndMeshVector4));
	
	glEnableVertexAttribArray(2);
	glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(DebriPoint), (void*)(sizeof(ndMeshVector4) + sizeof(ndMeshVector)));
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	
	glBindVertexArray(0);
	glDisableVertexAttribArray(2);
	glDisableVertexAttribArray(1);
	glDisableVertexAttribArray(0);
}

void ndDemoDebriEntityRoot::Render(dFloat32 timestep, ndDemoEntityManager* const scene, const dMatrix& matrix) const
{
	//ndDemoEntity::Render(timestep, scene, matrix);
}
