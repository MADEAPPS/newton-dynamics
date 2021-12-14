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
#include "ndDemoCamera.h"
#include "ndTargaToOpenGl.h"
#include "ndDemoEntityManager.h"
#include "ndDemoInstanceEntity.h"

ndArray<ndMatrix> ndDemoInstanceEntity::m_matrixStack;

ndDemoMeshIntance::ndDemoMeshIntance(const char* const name, const ndShaderPrograms& shaderCache, const ndShapeInstance* const collision, const char* const texture0, const char* const, const char* const, ndFloat32 opacity, const ndMatrix& uvMatrix)
	:ndDemoMesh(name)
	,m_offsets(nullptr)
	,m_instanceCount(0)
	,m_maxInstanceCount(1024)
	,m_matrixOffsetBuffer(0)
{
	ndShapeInstanceMeshBuilder mesh(*collision);

	ndMatrix aligmentUV(uvMatrix);
	m_shader = shaderCache.m_diffuseIntanceEffect;

	// apply uv projections
	ndShapeInfo info(collision->GetShapeInfo());
	switch (info.m_collisionType)
	{
		case ndShapeID::m_sphere:
		case ndShapeID::m_capsule:
		{
			mesh.SphericalMapping(LoadTexture(texture0), &aligmentUV[0][0]);
			break;
		}

		case ndShapeID::m_box:
		{
			ndInt32 tex0 = LoadTexture(texture0);
			mesh.UniformBoxMapping(tex0, aligmentUV);
			break;
		}

		default:
		{
			ndInt32 tex0 = LoadTexture(texture0);
			mesh.UniformBoxMapping(tex0, aligmentUV);
		}
	}

	// extract the materials index array for mesh
	ndIndexArray* const geometryHandle = mesh.MaterialGeometryBegin();

	// extract vertex data  from the newton mesh		
	ndInt32 vertexCount = mesh.GetPropertiesCount();
	ndInt32 indexCount = 0;
	for (ndInt32 handle = mesh.GetFirstMaterial(geometryHandle); handle != -1; handle = mesh.GetNextMaterial(geometryHandle, handle))
	{
		indexCount += mesh.GetMaterialIndexCount(geometryHandle, handle);
	}

	struct dTmpData
	{
		ndFloat32 m_posit[3];
		ndFloat32 m_normal[3];
		ndFloat32 m_uv[2];
	};

	ndArray<dTmpData> tmp;
	ndArray<ndInt32> indices;
	ndArray<glPositionNormalUV> points;
	ndArray<glMatrix> offsets;

	tmp.SetCount(vertexCount);
	points.SetCount(vertexCount);
	indices.SetCount(indexCount);
	offsets.SetCount(m_maxInstanceCount);
	
	mesh.GetVertexChannel(sizeof(dTmpData), &tmp[0].m_posit[0]);
	mesh.GetNormalChannel(sizeof(dTmpData), &tmp[0].m_normal[0]);
	mesh.GetUV0Channel(sizeof(dTmpData), &tmp[0].m_uv[0]);

	for (ndInt32 i = 0; i < vertexCount; i++)
	{
		points[i].m_posit.m_x = GLfloat(tmp[i].m_posit[0]);
		points[i].m_posit.m_y = GLfloat(tmp[i].m_posit[1]);
		points[i].m_posit.m_z = GLfloat(tmp[i].m_posit[2]);
		points[i].m_normal.m_x = GLfloat(tmp[i].m_normal[0]);
		points[i].m_normal.m_y = GLfloat(tmp[i].m_normal[1]);
		points[i].m_normal.m_z = GLfloat(tmp[i].m_normal[2]);
		points[i].m_uv.m_u = GLfloat(tmp[i].m_uv[0]);
		points[i].m_uv.m_v = GLfloat(tmp[i].m_uv[1]);
	}

	ndInt32 segmentStart = 0;
	bool hasTransparency = false;
	for (ndInt32 handle = mesh.GetFirstMaterial(geometryHandle); handle != -1; handle = mesh.GetNextMaterial(geometryHandle, handle))
	{
		ndInt32 material = mesh.GetMaterialID(geometryHandle, handle);
		ndDemoSubMesh* const segment = AddSubMesh();

		segment->m_material.m_textureHandle = (GLuint)material;
		segment->SetOpacity(opacity);
		hasTransparency = hasTransparency | segment->m_hasTranparency;

		segment->m_indexCount = mesh.GetMaterialIndexCount(geometryHandle, handle);

		segment->m_segmentStart = segmentStart;
		mesh.GetMaterialGetIndexStream(geometryHandle, handle, &indices[segmentStart]);
		segmentStart += segment->m_indexCount;
	}

	mesh.MaterialGeometryEnd(geometryHandle);

	m_hasTransparency = hasTransparency;

	// load index buffer.
	glGenBuffers(1, &m_indexBuffer);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_indexBuffer);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.GetCount() * sizeof(GLuint), &indices[0], GL_STATIC_DRAW);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

	// create vertex semantic layout
	glGenVertexArrays(1, &m_vertextArrayBuffer);
	glBindVertexArray(m_vertextArrayBuffer);

	glGenBuffers(1, &m_vertexBuffer);
	glBindBuffer(GL_ARRAY_BUFFER, m_vertexBuffer);
	glBufferData(GL_ARRAY_BUFFER, points.GetCount() * sizeof(glPositionNormalUV), &points[0], GL_STATIC_DRAW);

	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glPositionNormalUV), (void*)OFFSETOF(glPositionNormalUV, m_posit));

	glEnableVertexAttribArray(1);
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(glPositionNormalUV), (void*)OFFSETOF(glPositionNormalUV, m_normal));

	glEnableVertexAttribArray(2);
	glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(glPositionNormalUV), (void*)OFFSETOF(glPositionNormalUV, m_uv));
	glBindBuffer(GL_ARRAY_BUFFER, 0);

	// set vertex buffer for matrix instances
	glGenBuffers(1, &m_matrixOffsetBuffer);
	glBindBuffer(GL_ARRAY_BUFFER, m_matrixOffsetBuffer);
	glBufferData(GL_ARRAY_BUFFER, m_maxInstanceCount * sizeof(glMatrix), &offsets[0], GL_STATIC_DRAW);

	glEnableVertexAttribArray(3);
	glVertexAttribPointer(3, 4, GL_FLOAT, GL_FALSE, sizeof(glMatrix), (void*) (0 * sizeof(glVector4)));
	glVertexAttribDivisor(3, 1);
	
	glEnableVertexAttribArray(4);
	glVertexAttribPointer(4, 4, GL_FLOAT, GL_FALSE, sizeof(glMatrix), (void*)(1 * sizeof(glVector4)));
	glVertexAttribDivisor(4, 1);
	
	glEnableVertexAttribArray(5);
	glVertexAttribPointer(5, 4, GL_FLOAT, GL_FALSE, sizeof(glMatrix), (void*)(2 * sizeof(glVector4)));
	glVertexAttribDivisor(5, 1);
	
	glEnableVertexAttribArray(6);
	glVertexAttribPointer(6, 4, GL_FLOAT, GL_FALSE, sizeof(glMatrix), (void*)(3 * sizeof(glVector4)));
	glVertexAttribDivisor(6, 1);

	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindVertexArray(0);
	glDisableVertexAttribArray(3);
	glDisableVertexAttribArray(2);
	glDisableVertexAttribArray(1);
	glDisableVertexAttribArray(0);

	glUseProgram(m_shader);
	m_textureLocation = glGetUniformLocation(m_shader, "texture");
	m_transparencyLocation = glGetUniformLocation(m_shader, "transparency");
	m_normalMatrixLocation = glGetUniformLocation(m_shader, "normalMatrix");
	m_projectMatrixLocation = glGetUniformLocation(m_shader, "projectionMatrix");
	m_viewModelMatrixLocation = glGetUniformLocation(m_shader, "viewModelMatrix");
	m_directionalLightDirLocation = glGetUniformLocation(m_shader, "directionalLightDir");

	m_materialAmbientLocation = glGetUniformLocation(m_shader, "material_ambient");
	m_materialDiffuseLocation = glGetUniformLocation(m_shader, "material_diffuse");
	m_materialSpecularLocation = glGetUniformLocation(m_shader, "material_specular");

	glUseProgram(0);

	m_vertexCount = points.GetCount();
	m_indexCount = indices.GetCount();
}

ndDemoMeshIntance::~ndDemoMeshIntance()
{
	glDeleteBuffers(1, &m_matrixOffsetBuffer);
}

void ndDemoMeshIntance::SetTransforms(ndInt32 count, const ndMatrix* const matrixArray)
{
	m_offsets = matrixArray;
	m_instanceCount = count;
}

void ndDemoMeshIntance::RenderBatch(ndInt32 start, ndDemoEntityManager* const scene, const ndMatrix& modelMatrix)
{
	glBindBuffer(GL_ARRAY_BUFFER, m_matrixOffsetBuffer);

	glMatrix* const matrixBuffer = (glMatrix*)glMapBuffer(GL_ARRAY_BUFFER, GL_WRITE_ONLY);

	const ndInt32 base = start * m_maxInstanceCount;
	const ndInt32 count = ((base + m_maxInstanceCount) > m_instanceCount) ? m_instanceCount - base : m_maxInstanceCount;
	for (ndInt32 i = 0; i < count; i++)
	{
		ndMatrix matrix(m_offsets[base + i]);
		const ndFloat32* const src = &matrix[0][0];
		GLfloat* const dst = &matrixBuffer[i][0][0];
		for (ndInt32 j = 0; j < 16; j++) 
		{
			dst[j] = GLfloat(src[j]);
		}
	}

	glUnmapBuffer(GL_ARRAY_BUFFER);

	glBindBuffer(GL_ARRAY_BUFFER, 0);

	glUseProgram(m_shader);

	ndDemoCamera* const camera = scene->GetCamera();

	const ndMatrix& viewMatrix = camera->GetViewMatrix();
	const glMatrix& projectionMatrix (camera->GetProjectionMatrix());
	const glMatrix viewModelMatrix (modelMatrix * viewMatrix);
	const glVector4 directionaLight (viewMatrix.RotateVector(ndVector(-1.0f, 1.0f, 0.0f, 0.0f)).Normalize());

	glUniform1i(m_textureLocation, 0);
	glUniform1f(m_transparencyLocation, 1.0f);
	glUniform4fv(m_directionalLightDirLocation, 1, &directionaLight[0]);
	glUniformMatrix4fv(m_projectMatrixLocation, 1, false, &projectionMatrix[0][0]);
	glUniformMatrix4fv(m_viewModelMatrixLocation, 1, false, &viewModelMatrix[0][0]);

	glBindVertexArray(m_vertextArrayBuffer);
	glBindBuffer(GL_ARRAY_BUFFER, m_vertexBuffer);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_indexBuffer);

	glActiveTexture(GL_TEXTURE0);
	for (ndNode* node = GetFirst(); node; node = node->GetNext())
	{
		ndDemoSubMesh& segment = node->GetInfo();
		if (!segment.m_hasTranparency)
		{
			glUniform3fv(m_materialAmbientLocation, 1, &segment.m_material.m_ambient[0]);
			glUniform3fv(m_materialDiffuseLocation, 1, &segment.m_material.m_diffuse[0]);
			glUniform3fv(m_materialSpecularLocation, 1, &segment.m_material.m_specular[0]);

			glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
			glBindTexture(GL_TEXTURE_2D, segment.m_material.m_textureHandle);
			glDrawElementsInstanced(GL_TRIANGLES, segment.m_indexCount, GL_UNSIGNED_INT, (void*)(segment.m_segmentStart * sizeof(GL_UNSIGNED_INT)), m_instanceCount);
		}
	}

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindVertexArray(0);
	glUseProgram(0);
}

void ndDemoMeshIntance::Render(ndDemoEntityManager* const scene, const ndMatrix& modelMatrix)
{
	ndInt32 segments = (m_instanceCount - 1) / m_maxInstanceCount;
	for (ndInt32 i = 0; i < segments; i++)
	{
		RenderBatch(i, scene, modelMatrix);
	}
	RenderBatch(segments, scene, modelMatrix);
}

ndDemoInstanceEntity::ndDemoInstanceEntity(const ndDemoInstanceEntity& copyFrom)
	:ndDemoEntity(copyFrom)
	,m_instanceMesh(copyFrom.m_instanceMesh)
{
	dAssert(0);
	m_instanceMesh->AddRef();
}

ndDemoInstanceEntity::ndDemoInstanceEntity(ndDemoMeshIntance* const instanceMesh)
	:ndDemoEntity(dGetIdentityMatrix(), nullptr)
	,m_instanceMesh(instanceMesh)
{
	m_instanceMesh->AddRef();
}

ndDemoInstanceEntity::~ndDemoInstanceEntity(void)
{
	m_instanceMesh->Release();
}

void ndDemoInstanceEntity::Render(ndFloat32, ndDemoEntityManager* const scene, const ndMatrix& matrix) const
{
	D_TRACKTIME();
	//count active instances 
	ndInt32 count = 0;
	for (ndDemoEntity* child = GetChild(); child; child = child->GetSibling())
	{
		count++;
	}
	
	// prepare the transforms buffer form all the children matrices
	ndInt32 index = 0;
	//ndMatrix* const matrixStack = dAlloca(ndMatrix, count);
	m_matrixStack.SetCount(count);
	
	for (ndDemoEntity* child = GetChild(); child; child = child->GetSibling())
	{
		m_matrixStack[index] = child->GetCurrentMatrix();
		index++;
	}
	m_instanceMesh->SetTransforms(count, &m_matrixStack[0]);
	
	ndMatrix nodeMatrix(m_matrix * matrix);
	m_instanceMesh->Render(scene, nodeMatrix);
}
