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

#include "ndRenderStdafx.h"
#include "ndRender.h"
#include "ndRenderContext.h"
#include "ndRenderTexture.h"
#include "ndRenderOpenGlUtil.h"
#include "ndRenderShaderCache.h"
#include "ndRenderSceneCamera.h"
#include "ndRenderTextureImage.h"
#include "ndRenderPrimitiveMesh.h"

#include "ndRenderPrimitiveMeshImplement.h"

ndRenderPrimitiveMeshImplement::ndRenderPrimitiveMeshImplement(
	const ndRender* const render,
	const ndShapeInstance* const collision, 
	const ndRenderPrimitiveMeshMaterial& material,
	ndRenderPrimitiveMesh::ndUvMapingMode mapping,
	const ndMatrix& uvMatrix, 
	bool stretchMaping)
	:ndContainersFreeListAlloc<ndRenderPrimitiveMeshImplement>()
	,m_context(*render->m_context)
	,m_segments()
	,m_indexCount(0)
	,m_vertexCount(0)
	,m_indexBuffer(0)
	,m_vertexBuffer(0)
	,m_vertextArrayBuffer(0)
	,m_normalMatrixLocation(-1)
	,m_projectMatrixLocation(-1)
	,m_viewModelMatrixLocation(-1)
{
	ndMeshEffect mesh(*collision);

	ndRenderTextureImageCommon* const image = (ndRenderTextureImageCommon*)*material.m_texture;
	ndInt32 textureId = ndInt32(image->m_texture);
	switch (mapping)
	{
		case ndRenderPrimitiveMesh::m_spherical:
		case ndRenderPrimitiveMesh::m_cylindrical:
		{
			ndMatrix flipMatrix(ndGetIdentityMatrix());
			flipMatrix[0][0] = ndFloat32(-1.0f);
			ndMatrix aligmentUV(flipMatrix * uvMatrix);
			mesh.SphericalMapping(textureId, aligmentUV);
			break;
		}

		case ndRenderPrimitiveMesh::m_box:
		{
			if (stretchMaping)
			{
				mesh.BoxMapping(textureId, textureId, textureId, uvMatrix);
			}
			else
			{
				mesh.UniformBoxMapping(textureId, uvMatrix);
			}
			break;
		}
		default:
			//ndInt32 tex0 = LoadTexture(texture0);
			//ndInt32 tex1 = LoadTexture(texture1);
			//ndInt32 tex2 = LoadTexture(texture2);
			//NewtonMeshApplyBoxMapping(mesh, tex0, tex1, tex2, &aligmentUV[0][0]);
			mesh.UniformBoxMapping(textureId, uvMatrix);
	}

	ndIndexArray* const geometryHandle = mesh.MaterialGeometryBegin();

	// extract vertex data  from the newton mesh
	ndInt32 indexCount = 0;
	ndInt32 vertexCount = mesh.GetPropertiesCount();
	for (ndInt32 handle = mesh.GetFirstMaterial(geometryHandle); handle != -1; handle = mesh.GetNextMaterial(geometryHandle, handle))
	{
		indexCount += mesh.GetMaterialIndexCount(geometryHandle, handle);
	}

	struct dTmpData
	{
		ndReal m_posit[3];
		ndReal m_normal[3];
		ndReal m_uv[2];
	};
	
	ndArray<dTmpData> tmp;
	ndArray<ndInt32> indices;
	ndArray<glPositionNormalUV> points;
	
	tmp.SetCount(vertexCount);
	points.SetCount(vertexCount);
	indices.SetCount(indexCount);
	
	mesh.GetVertexChannel(sizeof(dTmpData), &tmp[0].m_posit[0]);
	mesh.GetNormalChannel(sizeof(dTmpData), &tmp[0].m_normal[0]);
	mesh.GetUV0Channel(sizeof(dTmpData), &tmp[0].m_uv[0]);
	
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
	}
	
	ndInt32 segmentStart = 0;
	//bool hasTransparency = false;
	//const ndArray<ndMeshEffect::ndMaterial>& materialArray = mesh.GetMaterials();
	for (ndInt32 handle = mesh.GetFirstMaterial(geometryHandle); handle != -1; handle = mesh.GetNextMaterial(geometryHandle, handle))
	{
		//ndInt32 materialId = mesh.GetMaterialID(geometryHandle, handle);
		ndRenderPrimitiveMeshSegment& segment = m_segments.Append()->GetInfo();
	
		//const ndMeshEffect::ndMaterial& segMaterial = materialArray[materialId];
		//segment->m_material.SetTexture(material);
		//segment->SetOpacity(opacity);
		//hasTransparency = hasTransparency | segment->m_hasTranparency;
		segment.m_material.m_texture = material.m_texture;
		segment.m_material.m_diffuse = material.m_diffuse;
		segment.m_material.m_specular = material.m_specular;
		segment.m_material.m_opacity = material.m_opacity;
		segment.m_material.m_specularPower = material.m_specularPower;
	
		segment.m_indexCount = mesh.GetMaterialIndexCount(geometryHandle, handle);
	
		segment.m_segmentStart = segmentStart;
		mesh.GetMaterialGetIndexStream(geometryHandle, handle, &indices[segmentStart]);
		segmentStart += segment.m_indexCount;
	}
	mesh.MaterialGeometryEnd(geometryHandle);
	
	//m_hasTransparency = hasTransparency;
	
	// optimize this mesh for hardware buffers if possible
	OptimizeForRender(&points[0], vertexCount, &indices[0], indexCount);
	
	//ReleaseTexture(GLuint(tex0));
}

ndRenderPrimitiveMeshImplement::~ndRenderPrimitiveMeshImplement()
{
	ResetOptimization();
}

void ndRenderPrimitiveMeshImplement::ResetOptimization()
{
	if (m_vertextArrayBuffer)
	{
		glDeleteBuffers(1, &m_indexBuffer);
		glDeleteBuffers(1, &m_vertexBuffer);
		glDeleteVertexArrays(1, &m_vertextArrayBuffer);
	}
}

void ndRenderPrimitiveMeshImplement::OptimizeForRender(
	const glPositionNormalUV* const points, ndInt32 pointCount,
	const ndInt32* const indices, ndInt32 indexCount)
{
	// first make sure the previous optimization is removed
	ResetOptimization();

	glGenVertexArrays(1, &m_vertextArrayBuffer);
	glBindVertexArray(m_vertextArrayBuffer);

	glGenBuffers(1, &m_vertexBuffer);
	glBindBuffer(GL_ARRAY_BUFFER, m_vertexBuffer);
	glBufferData(GL_ARRAY_BUFFER, GLsizeiptr(pointCount * sizeof(glPositionNormalUV)), &points[0], GL_STATIC_DRAW);

	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glPositionNormalUV), (void*)OFFSETOF(glPositionNormalUV, m_posit));

	glEnableVertexAttribArray(1);
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(glPositionNormalUV), (void*)OFFSETOF(glPositionNormalUV, m_normal));

	glEnableVertexAttribArray(2);
	glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(glPositionNormalUV), (void*)OFFSETOF(glPositionNormalUV, m_uv));
	glBindBuffer(GL_ARRAY_BUFFER, 0);

	glBindVertexArray(0);

	glGenBuffers(1, &m_indexBuffer);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_indexBuffer);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, GLsizeiptr(indexCount * sizeof(GLuint)), &indices[0], GL_STATIC_DRAW);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

	GLuint shader = m_context->m_shaderCache->m_diffuseEffect;
	glUseProgram(shader);
	//m_textureLocation = glGetUniformLocation(shader, "texture0");
	//m_transparencyLocation = glGetUniformLocation(shader, "transparency");
	m_normalMatrixLocation = glGetUniformLocation(shader, "normalMatrix");
	m_projectMatrixLocation = glGetUniformLocation(shader, "projectionMatrix");
	m_viewModelMatrixLocation = glGetUniformLocation(shader, "viewModelMatrix");

	m_diffuseColor = glGetUniformLocation(shader, "diffuseColor");
	m_specularColor = glGetUniformLocation(shader, "specularColor");
	m_directionalLightAmbient = glGetUniformLocation(shader, "directionalLightAmbient");
	m_directionalLightIntesity = glGetUniformLocation(shader, "directionalLightIntesity");
	m_directionalLightDirection = glGetUniformLocation(shader, "directionalLightDirection");
	m_specularAlpha = glGetUniformLocation(shader, "specularAlpha");

	glUseProgram(0);

	//glUseProgram(m_shaderShadow);
	//m_textureLocationShadow = glGetUniformLocation(m_shaderShadow, "texture0");
	//m_transparencyLocationShadow = glGetUniformLocation(m_shaderShadow, "transparency");
	//m_projectMatrixLocationShadow = glGetUniformLocation(m_shaderShadow, "projectionMatrix");
	//m_viewModelMatrixLocationShadow = glGetUniformLocation(m_shaderShadow, "viewModelMatrix");
	//m_materialAmbientLocationShadow = glGetUniformLocation(m_shaderShadow, "material_ambient");
	//m_materialDiffuseLocationShadow = glGetUniformLocation(m_shaderShadow, "material_diffuse");
	//m_materialSpecularLocationShadow = glGetUniformLocation(m_shaderShadow, "material_specular");
	//m_directionalLightDirLocationShadow = glGetUniformLocation(m_shaderShadow, "directionalLightDir");
	//
	//m_shadowSlices = glGetUniformLocation(m_shaderShadow, "shadowSlices");
	//m_worldMatrix = glGetUniformLocation(m_shaderShadow, "modelWorldMatrix");
	//m_depthMapTexture = glGetUniformLocation(m_shaderShadow, "depthMapTexture");
	//m_directionLightViewProjectionMatrixShadow = glGetUniformLocation(m_shaderShadow, "directionaLightViewProjectionMatrix");
	//
	//glUseProgram(0);

	m_vertexCount = pointCount;
	m_indexCount = indexCount;
}

void ndRenderPrimitiveMeshImplement::Render(const ndRender* const render, const ndMatrix& modelMatrix) const
{
	const ndSharedPtr<ndRenderSceneCamera>& camera = render->GetCamera();

	GLuint shader = m_context->m_shaderCache->m_diffuseEffect;
	glUseProgram(shader);

	const ndMatrix viewMatrix (camera->m_invViewMatrix);
	const ndMatrix modelViewMatrix (modelMatrix * viewMatrix);

	const glMatrix glViewModelMatrix(modelViewMatrix);
	const glMatrix glProjectionMatrix(camera->m_projectionMatrix);

	const glVector4 glSunlightAmbient(render->m_sunLightAmbient);
	const glVector4 glSunlightIntensity(render->m_sunLightIntesity);
	const glVector4 glSunlightDir(viewMatrix.RotateVector(render->m_sunLightDir));
	
	glUniform3fv(m_directionalLightDirection, 1, &glSunlightDir[0]);
	glUniform3fv(m_directionalLightAmbient, 1, &glSunlightAmbient[0]);
	glUniform3fv(m_directionalLightIntesity, 1, &glSunlightIntensity[0]);

	glUniformMatrix4fv(m_normalMatrixLocation, 1, false, &glViewModelMatrix[0][0]);
	glUniformMatrix4fv(m_projectMatrixLocation, 1, false, &glProjectionMatrix[0][0]);
	glUniformMatrix4fv(m_viewModelMatrixLocation, 1, false, &glViewModelMatrix[0][0]);
	
	glBindVertexArray(m_vertextArrayBuffer);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_indexBuffer);
	
	for (ndList<ndRenderPrimitiveMeshSegment>::ndNode* node = m_segments.GetFirst(); node; node = node->GetNext())
	{
		ndRenderPrimitiveMeshSegment& segment = node->GetInfo();
		//if (!segment.m_hasTranparency)
		{
			const ndRenderPrimitiveMeshMaterial* const material = &segment.m_material;
			const ndRenderTextureImageCommon* const image = (ndRenderTextureImageCommon*) *material->m_texture;
			ndAssert(image);

			const glVector4 diffuse(material->m_diffuse);
			const glVector4 specular(material->m_specular);
			glUniform3fv(m_diffuseColor, 1, &diffuse[0]);
			glUniform3fv(m_specularColor, 1, &specular[0]);
			glUniform1fv(m_specularAlpha, 1, &material->m_specularPower);
			
			glBindTexture(GL_TEXTURE_2D, image->m_texture);
			glDrawElements(GL_TRIANGLES, segment.m_indexCount, GL_UNSIGNED_INT, (void*)(segment.m_segmentStart * sizeof(GL_UNSIGNED_INT)));
		}
	}
	
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
	glBindVertexArray(0);
	glUseProgram(0);
}