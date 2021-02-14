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
#include "ndDemoEntityManager.h"
#include "ndDemoInstanceEntity.h"

ndDemoMeshIntance::ndDemoMeshIntance(const char* const name, const ndShaderPrograms& shaderCache, const ndShapeInstance* const collision, const char* const texture0, const char* const texture1, const char* const texture2, dFloat32 opacity, const dMatrix& uvMatrix)
	:ndDemoMesh(name)
	,m_offsets(nullptr)
	,m_instanceCount(0)
	,m_maxInstanceCount(1024)
	,m_matrixOffsetBuffer(0)
{
	ndShapeInstanceMeshBuilder mesh(*collision);

	dMatrix aligmentUV(uvMatrix);
	m_shader = shaderCache.m_diffuseIntanceEffect;

	// apply uv projections
	ndShapeInfo info(collision->GetShapeInfo());
	switch (info.m_collisionType)
	{
		case ndShapeID::m_sphereCollision:
		case ndShapeID::m_capsuleCollision:
		{
			mesh.SphericalMapping(LoadTexture(texture0), &aligmentUV[0][0]);
			break;
		}

		case ndShapeID::m_boxCollision:
		{
			dInt32 tex0 = LoadTexture(texture0);
			mesh.UniformBoxMapping(tex0, aligmentUV);
			break;
		}

		default:
		{
			dInt32 tex0 = LoadTexture(texture0);
			mesh.UniformBoxMapping(tex0, aligmentUV);
		}
	}

	// extract the materials index array for mesh
	ndIndexArray* const geometryHandle = mesh.MaterialGeometryBegin();

	// extract vertex data  from the newton mesh		
	dInt32 vertexCount = mesh.GetPropertiesCount();
	dInt32 indexCount = 0;
	for (dInt32 handle = mesh.GetFirstMaterial(geometryHandle); handle != -1; handle = mesh.GetNextMaterial(geometryHandle, handle))
	{
		indexCount += mesh.GetMaterialIndexCount(geometryHandle, handle);
	}

	dArray<dInt32> indices(indexCount);
	dArray<ndMeshPointUV> points(vertexCount);
	dArray<ndMeshMatrix> offsets(m_maxInstanceCount);

	points.SetCount(vertexCount);
	indices.SetCount(indexCount);
	offsets.SetCount(m_maxInstanceCount);

	mesh.GetVertexChannel(sizeof(ndMeshPointUV), &points[0].m_posit.m_x);
	mesh.GetNormalChannel(sizeof(ndMeshPointUV), &points[0].m_normal.m_x);
	mesh.GetUV0Channel(sizeof(ndMeshPointUV), &points[0].m_uv.m_u);

	dInt32 segmentStart = 0;
	bool hasTransparency = false;
	for (dInt32 handle = mesh.GetFirstMaterial(geometryHandle); handle != -1; handle = mesh.GetNextMaterial(geometryHandle, handle))
	{
		dInt32 material = mesh.GetMaterialID(geometryHandle, handle);
		ndDemoSubMesh* const segment = AddSubMesh();

		segment->m_material.m_textureHandle = (GLuint)material;
		segment->SetOpacity(opacity);
		hasTransparency = hasTransparency | segment->m_hasTranparency;

		segment->m_indexCount = mesh.GetMaterialIndexCount(geometryHandle, handle);

		segment->m_segmentStart = segmentStart;
		mesh.GetMaterialGetIndexStream(geometryHandle, handle, &indices[segmentStart]);
		segmentStart += segment->m_indexCount;
	}

	mesh.MaterialGeomteryEnd(geometryHandle);

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
	glBufferData(GL_ARRAY_BUFFER, points.GetCount() * sizeof(ndMeshPointUV), &points[0], GL_STATIC_DRAW);

	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(ndMeshPointUV), (void*)0);

	glEnableVertexAttribArray(1);
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(ndMeshPointUV), (void*)sizeof(ndMeshVector));

	glEnableVertexAttribArray(2);
	glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(ndMeshPointUV), (void*)(2 * sizeof(ndMeshVector)));
	glBindBuffer(GL_ARRAY_BUFFER, 0);


	// set vertex buffer for matrix instances
	glGenBuffers(1, &m_matrixOffsetBuffer);
	glBindBuffer(GL_ARRAY_BUFFER, m_matrixOffsetBuffer);
	glBufferData(GL_ARRAY_BUFFER, m_maxInstanceCount * sizeof(ndMeshMatrix), &offsets[0], GL_STATIC_DRAW);

	glEnableVertexAttribArray(3);
	glVertexAttribPointer(3, 4, GL_FLOAT, GL_FALSE, sizeof(ndMeshMatrix), (void*) (0 * sizeof(ndMeshVector4)));
	glVertexAttribDivisor(3, 1);
	
	glEnableVertexAttribArray(4);
	glVertexAttribPointer(4, 4, GL_FLOAT, GL_FALSE, sizeof(ndMeshMatrix), (void*)(1 * sizeof(ndMeshVector4)));
	glVertexAttribDivisor(4, 1);
	
	glEnableVertexAttribArray(5);
	glVertexAttribPointer(5, 4, GL_FLOAT, GL_FALSE, sizeof(ndMeshMatrix), (void*)(2 * sizeof(ndMeshVector4)));
	glVertexAttribDivisor(5, 1);
	
	glEnableVertexAttribArray(6);
	glVertexAttribPointer(6, 4, GL_FLOAT, GL_FALSE, sizeof(ndMeshMatrix), (void*)(3 * sizeof(ndMeshVector4)));
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

void ndDemoMeshIntance::SetTransforms(dInt32 count, const dMatrix* const matrixArray)
{
	m_offsets = matrixArray;
	m_instanceCount = count;
}

void ndDemoMeshIntance::RenderBatch(dInt32 start, ndDemoEntityManager* const scene, const dMatrix& modelMatrix)
{
	glBindBuffer(GL_ARRAY_BUFFER, m_matrixOffsetBuffer);

	ndMeshMatrix* const matrixBuffer = (ndMeshMatrix*)glMapBuffer(GL_ARRAY_BUFFER, GL_WRITE_ONLY);

	const dInt32 base = start * m_maxInstanceCount;
	const dInt32 count = ((base + m_maxInstanceCount) > m_instanceCount) ? m_instanceCount - base : m_maxInstanceCount;
	for (dInt32 i = 0; i < count; i++)
	{
		dMatrix matrix(m_offsets[base + i]);
		const dFloat32* const src = &matrix[0][0];
		dFloat32* const dst = &matrixBuffer[i].m_array[0].m_x;
		for (dInt32 j = 0; j < 16; j++) 
		{
			dst[j] = GLfloat(src[j]);
		}
	}

	glUnmapBuffer(GL_ARRAY_BUFFER);

	glBindBuffer(GL_ARRAY_BUFFER, 0);

	glUseProgram(m_shader);

	ndDemoCamera* const camera = scene->GetCamera();

	const dMatrix& viewMatrix = camera->GetViewMatrix();
	const dMatrix& projectionMatrix = camera->GetProjectionMatrix();
	dMatrix viewModelMatrix(modelMatrix * viewMatrix);
	dVector directionaLight(viewMatrix.RotateVector(dVector(-1.0f, 1.0f, 0.0f, 0.0f)).Normalize());

	glUniform1i(m_textureLocation, 0);
	glUniform1f(m_transparencyLocation, 1.0f);
	glUniform4fv(m_directionalLightDirLocation, 1, &directionaLight.m_x);
	glUniformMatrix4fv(m_projectMatrixLocation, 1, false, &projectionMatrix[0][0]);
	glUniformMatrix4fv(m_viewModelMatrixLocation, 1, false, &viewModelMatrix[0][0]);

	glBindVertexArray(m_vertextArrayBuffer);
	glBindBuffer(GL_ARRAY_BUFFER, m_vertexBuffer);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_indexBuffer);

	glActiveTexture(GL_TEXTURE0);
	for (dListNode* node = GetFirst(); node; node = node->GetNext())
	{
		ndDemoSubMesh& segment = node->GetInfo();
		if (!segment.m_hasTranparency)
		{
			glUniform3fv(m_materialAmbientLocation, 1, &segment.m_material.m_ambient.m_x);
			glUniform3fv(m_materialDiffuseLocation, 1, &segment.m_material.m_diffuse.m_x);
			glUniform3fv(m_materialSpecularLocation, 1, &segment.m_material.m_specular.m_x);

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

void ndDemoMeshIntance::Render(ndDemoEntityManager* const scene, const dMatrix& modelMatrix)
{
	dInt32 segments = (m_instanceCount - 1) / m_maxInstanceCount;
	for (dInt32 i = 0; i < segments; i++)
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

void ndDemoInstanceEntity::Render(dFloat32 timestep, ndDemoEntityManager* const scene, const dMatrix& matrix) const
{
	//count active instances 
	dInt32 count = 0;
	for (ndDemoEntity* child = GetChild(); child; child = child->GetSibling())
	{
		count++;
	}
	
	// prepare the transforms buffer form all the children matrices
	dInt32 index = 0;
	dMatrix* const matrixStack = dAlloca(dMatrix, count);
	
	for (ndDemoEntity* child = GetChild(); child; child = child->GetSibling())
	{
		matrixStack[index] = child->GetCurrentMatrix();
		index++;
	}
	m_instanceMesh->SetTransforms(count, &matrixStack[0]);
	
	dMatrix nodeMatrix(m_matrix * matrix);
	m_instanceMesh->Render(scene, nodeMatrix);
}
