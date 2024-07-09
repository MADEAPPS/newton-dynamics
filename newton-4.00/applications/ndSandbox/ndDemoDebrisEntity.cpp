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
#include "ndPngToOpenGl.h"
#include "ndDemoDebrisEntity.h"
#include "ndDemoEntityManager.h"

ndDemoDebrisMesh::ndDemoDebrisMesh(ndDemoDebrisMesh* const srcMesh, const ndArray<glDebrisPoint>& vertexArray)
	:ndDemoMesh("vertexBufferMesh")
{
	m_indexCount = 0;
	m_shader = srcMesh->m_shader;
	m_vertexCount = ndInt32(vertexArray.GetCount());
	m_textureLocation = srcMesh->m_textureLocation;
	m_transparencyLocation = srcMesh->m_transparencyLocation;
	m_normalMatrixLocation = srcMesh->m_normalMatrixLocation;
	m_projectMatrixLocation = srcMesh->m_projectMatrixLocation;
	m_viewModelMatrixLocation = srcMesh->m_viewModelMatrixLocation;
	m_directionalLightDirLocation = srcMesh->m_directionalLightDirLocation;
	
	m_materialAmbientLocation = srcMesh->m_materialAmbientLocation;
	m_materialDiffuseLocation = srcMesh->m_materialDiffuseLocation;
	m_materialSpecularLocation = srcMesh->m_materialSpecularLocation;
	
	m_textureLocation1 = srcMesh->m_textureLocation1;
	memcpy(m_material, srcMesh->m_material, sizeof(m_material));
	
	glGenVertexArrays(1, &m_vertextArrayBuffer);
	glBindVertexArray(m_vertextArrayBuffer);
	
	glGenBuffers(1, &m_vertexBuffer);
	glBindBuffer(GL_ARRAY_BUFFER, m_vertexBuffer);
	glBufferData(GL_ARRAY_BUFFER, GLsizeiptr(m_vertexCount * sizeof(glDebrisPoint)), &vertexArray[0].m_posit.m_x, GL_STATIC_DRAW);
	
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, sizeof(glDebrisPoint), (void*)OFFSETOF(glDebrisPoint, m_posit));
	
	glEnableVertexAttribArray(1);
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(glDebrisPoint), (void*)OFFSETOF(glDebrisPoint, m_normal));
	
	glEnableVertexAttribArray(2);
	glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(glDebrisPoint), (void*)OFFSETOF(glDebrisPoint, m_uv));
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	
	glBindVertexArray(0);
}

ndDemoDebrisMesh::ndDemoDebrisMesh(const char* const name, ndMeshEffect* const meshNode, const ndShaderCache& shaderCache, ndInt32 offsetBase, ndArray<glDebrisPoint>& vertexArray)
	:ndDemoMesh(name)
{
	m_name = name;
	m_shader = shaderCache.m_diffuseDebrisEffect;
	
	// extract the materials index array for mesh
	ndIndexArray* const geometryHandle = meshNode->MaterialGeometryBegin();
	
	// extract vertex data  from the newton mesh		
	ndInt32 indexCount = 0;
	for (ndInt32 handle = meshNode->GetFirstMaterial(geometryHandle); handle != -1; handle = meshNode->GetNextMaterial(geometryHandle, handle))
	{
		indexCount += meshNode->GetMaterialIndexCount(geometryHandle, handle);
	}
	
	ndInt32* const indices = ndAlloca(ndInt32, indexCount);
	
	ndInt32 segmentStart = 0;
	ndInt32 materialCount = 0;
	const ndArray<ndMeshEffect::ndMaterial>& materialArray = meshNode->GetMaterials();
	for (ndInt32 handle = meshNode->GetFirstMaterial(geometryHandle); handle != -1; handle = meshNode->GetNextMaterial(geometryHandle, handle))
	{
		ndInt32 materialIndex = meshNode->GetMaterialID(geometryHandle, handle);
		const ndMeshEffect::ndMaterial& material = materialArray[materialIndex];
	
		m_material[materialCount].m_ambient = glVector4 (material.m_ambient);
		m_material[materialCount].m_diffuse = glVector4(material.m_diffuse);
		m_material[materialCount].m_specular = glVector4(material.m_specular);
		m_material[materialCount].m_opacity = GLfloat(material.m_opacity);
		m_material[materialCount].m_shiness = GLfloat(material.m_shiness);
		//strcpy(m_material[materialCount].m_textureName, material.m_textureName);
		m_material[materialCount].SetTextureName(material.m_textureName);
		ndInt32 tex = ndInt32(LoadTexture(material.m_textureName));
		m_material[materialCount].SetTexture(tex);
		ReleaseTexture(GLuint(tex));
	
		ndInt32 subIndexCount = meshNode->GetMaterialIndexCount(geometryHandle, handle);
		meshNode->GetMaterialGetIndexStream(geometryHandle, handle, &indices[segmentStart]);
	
		GLfloat blend = materialCount ? 0.0f : 1.0f;
		for (ndInt32 i = 0; i < subIndexCount; ++i)
		{
			ndInt32 index = indices[segmentStart + i] + offsetBase;
			indices[segmentStart + i] = index;
			vertexArray[index].m_posit.m_w = blend;
		}
	
		materialCount++;
		segmentStart += subIndexCount;
	}
	meshNode->MaterialGeometryEnd(geometryHandle);
	
	
	glGenBuffers(1, &m_indexBuffer);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_indexBuffer);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, GLsizeiptr(indexCount * sizeof(GLuint)), &indices[0], GL_STATIC_DRAW);
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

ndDemoDebrisMesh::~ndDemoDebrisMesh()
{
}

void ndDemoDebrisMesh::Render(ndDemoEntityManager* const scene, const ndMatrix& modelMatrix)
{
	ndDemoCamera* const camera = scene->GetCamera();

	const ndMatrix& viewMatrix = camera->GetInvViewMatrix();
	const glMatrix viewModelMatrix(modelMatrix * viewMatrix);

	glUniformMatrix4fv(m_normalMatrixLocation, 1, false, &viewModelMatrix[0][0]);
	glUniformMatrix4fv(m_viewModelMatrixLocation, 1, false, &viewModelMatrix[0][0]);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_indexBuffer);

	glDrawElements(GL_TRIANGLES, m_indexCount, GL_UNSIGNED_INT, 0);
}

ndDemoDebrisEntity::ndDemoDebrisEntity(ndMeshEffect* const meshNode, ndArray<glDebrisPoint>& vertexArray, ndDemoDebrisRootEntity* const parent, const ndShaderCache& shaderCache)
	:ndDemoEntity(ndGetIdentityMatrix(), parent)
{
	ndInt32 vertexCount = meshNode->GetPropertiesCount();

	ndInt32 const vertexOffsetBase = ndInt32(vertexArray.GetCount());
	vertexArray.SetCount(vertexOffsetBase + vertexCount);

	struct dTmpData
	{
		ndFloat32 m_posit[4];
		ndFloat32 m_normal[3];
		ndFloat32 m_uv[2];
	};

	ndArray<dTmpData> tmp;
	tmp.SetCount(vertexCount);

	//meshNode->GetVertexChannel(sizeof(glDebrisPoint), &vertexArray[vertexOffsetBase].m_posit.m_x);
	//meshNode->GetNormalChannel(sizeof(glDebrisPoint), &vertexArray[vertexOffsetBase].m_normal.m_x);
	//meshNode->GetUV0Channel(sizeof(glDebrisPoint), &vertexArray[vertexOffsetBase].m_uv.m_u);
	meshNode->GetVertexChannel(sizeof(dTmpData), &tmp[0].m_posit[0]);
	meshNode->GetNormalChannel(sizeof(dTmpData), &tmp[0].m_normal[0]);
	meshNode->GetUV0Channel(sizeof(dTmpData), &tmp[0].m_uv[0]);

	for (ndInt32 i = 0; i < vertexCount; ++i)
	{
		vertexArray[vertexOffsetBase + i].m_posit.m_x = GLfloat(tmp[i].m_posit[0]);
		vertexArray[vertexOffsetBase + i].m_posit.m_y = GLfloat(tmp[i].m_posit[1]);
		vertexArray[vertexOffsetBase + i].m_posit.m_z = GLfloat(tmp[i].m_posit[2]);
		vertexArray[vertexOffsetBase + i].m_posit.m_w = GLfloat(tmp[i].m_posit[3]);
		vertexArray[vertexOffsetBase + i].m_normal.m_x = GLfloat(tmp[i].m_normal[0]);
		vertexArray[vertexOffsetBase + i].m_normal.m_y = GLfloat(tmp[i].m_normal[1]);
		vertexArray[vertexOffsetBase + i].m_normal.m_z = GLfloat(tmp[i].m_normal[2]);
		vertexArray[vertexOffsetBase + i].m_uv.m_u = GLfloat(tmp[i].m_uv[0]);
		vertexArray[vertexOffsetBase + i].m_uv.m_v = GLfloat(tmp[i].m_uv[1]);
	}

	ndSharedPtr<ndDemoMeshInterface> mesh(new ndDemoDebrisMesh("fracture", meshNode, shaderCache, vertexOffsetBase, vertexArray));
	SetMesh(mesh);
}

ndDemoDebrisEntity::ndDemoDebrisEntity(const ndDemoDebrisEntity& copyFrom)
	:ndDemoEntity(copyFrom)
{
}

ndDemoDebrisEntity::~ndDemoDebrisEntity()
{
}

ndDemoEntity* ndDemoDebrisEntity::CreateClone() const
{
	return new ndDemoDebrisEntity(*this);
}

void ndDemoDebrisEntity::Render(ndFloat32, ndDemoEntityManager* const scene, const ndMatrix& matrix) const
{
	const ndMatrix modelMatrix(m_matrix * matrix);
	ndDemoMeshInterface* const mesh = (ndDemoMeshInterface*)*m_mesh;
	mesh->Render(scene, modelMatrix);
}

ndDemoDebrisRootEntity::ndDemoDebrisRootEntity()
	:ndDemoEntity(ndGetIdentityMatrix(), nullptr)
{
}

ndDemoDebrisRootEntity::ndDemoDebrisRootEntity(const ndDemoDebrisRootEntity& copyFrom)
	:ndDemoEntity(copyFrom)
{
}

ndDemoDebrisRootEntity::~ndDemoDebrisRootEntity(void)
{
}

void ndDemoDebrisRootEntity::FinalizeConstruction(const ndArray<glDebrisPoint>& vertexArray)
{
	ndDemoDebrisMesh* const shaderMesh = (ndDemoDebrisMesh*)*GetFirstChild()->GetMesh();
	ndSharedPtr<ndDemoMeshInterface> mesh(new ndDemoDebrisMesh(shaderMesh, vertexArray));
	SetMesh(mesh);
}

void ndDemoDebrisRootEntity::Render(ndFloat32 timestep, ndDemoEntityManager* const scene, const ndMatrix& matrix) const
{
	ndDemoDebrisMesh* const shaderMesh = ((ndDemoDebrisMesh*)*m_mesh);
	glUseProgram(shaderMesh->m_shader);
	glBindVertexArray(shaderMesh->m_vertextArrayBuffer);
	
	ndDemoCamera* const camera = scene->GetCamera();
	const ndMatrix& viewMatrix = camera->GetInvViewMatrix();
	const glMatrix projectionMatrix (camera->GetProjectionMatrix());
	const glVector4 directionaLight(viewMatrix.RotateVector(scene->GetDirectionsLight()));
	
	glUniform1i(shaderMesh->m_textureLocation, 0);
	glUniform1i(shaderMesh->m_textureLocation1, 1);
	glUniform1f(shaderMesh->m_transparencyLocation, 1.0f);
	glUniform4fv(shaderMesh->m_directionalLightDirLocation, 1, &directionaLight[0]);
	glUniformMatrix4fv(shaderMesh->m_projectMatrixLocation, 1, false, &projectionMatrix[0][0]);
	
	glUniform3fv(shaderMesh->m_materialAmbientLocation, 1, &shaderMesh->m_material[0].m_ambient[0]);
	glUniform3fv(shaderMesh->m_materialDiffuseLocation, 1, &shaderMesh->m_material[0].m_diffuse[0]);
	glUniform3fv(shaderMesh->m_materialSpecularLocation, 1, &shaderMesh->m_material[0].m_specular[0]);
	
	// these call make the font display wrong
	glActiveTexture(GL_TEXTURE1);
	//glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
	glBindTexture(GL_TEXTURE_2D, GLuint(shaderMesh->m_material[1].GetTexture()));
	
	glActiveTexture(GL_TEXTURE0);
	//glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
	glBindTexture(GL_TEXTURE_2D, GLuint(shaderMesh->m_material[0].GetTexture()));
	
	const ndMatrix nodeMatrix(m_matrix * matrix);
	for (ndDemoEntity* child = GetFirstChild(); child; child = child->GetNext())
	{
		child->Render(timestep, scene, nodeMatrix);
	}
	
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
	glBindVertexArray(0);
	glUseProgram(0);
}
