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
#include "ndRenderPassEnvironment.h"
#include "ndRenderPassShadowsImplement.h"
#include "ndRenderPrimitiveMeshImplement.h"

ndRenderPrimitiveMeshImplement::ndRenderPrimitiveMeshImplement(const ndRender* const render, const ndShapeInstance* const collision)
	:ndContainersFreeListAlloc<ndRenderPrimitiveMeshImplement>()
	,m_context(*render->m_context)
	,m_segments()
	,m_indexCount(0)
	,m_vertexCount(0)
	,m_indexBuffer(0)
	,m_vertexBuffer(0)
	,m_vertextArrayBuffer(0)
{
	class ndDrawShape : public ndShapeDebugNotify
	{
		public:
		ndDrawShape()
			:ndShapeDebugNotify()
			,m_triangles(1024)
		{
		}
	
		virtual void DrawPolygon(ndInt32 vertexCount, const ndVector* const faceVertex, const ndEdgeType* const)
		{
			ndVector p0(faceVertex[0]);
			ndVector p1(faceVertex[1]);
			ndVector p2(faceVertex[2]);
	
			ndVector normal((p1 - p0).CrossProduct(p2 - p0));
			normal = normal.Normalize();
			for (ndInt32 i = 2; i < vertexCount; ++i)
			{
				glPositionNormal point;
				point.m_posit.m_x = GLfloat(faceVertex[0].m_x);
				point.m_posit.m_y = GLfloat(faceVertex[0].m_y);
				point.m_posit.m_z = GLfloat(faceVertex[0].m_z);
				point.m_normal.m_x = GLfloat(normal.m_x);
				point.m_normal.m_y = GLfloat(normal.m_y);
				point.m_normal.m_z = GLfloat(normal.m_z);
				m_triangles.PushBack(point);
	
				point.m_posit.m_x = GLfloat(faceVertex[i - 1].m_x);
				point.m_posit.m_y = GLfloat(faceVertex[i - 1].m_y);
				point.m_posit.m_z = GLfloat(faceVertex[i - 1].m_z);
				point.m_normal.m_x = GLfloat(normal.m_x);
				point.m_normal.m_y = GLfloat(normal.m_y);
				point.m_normal.m_z = GLfloat(normal.m_z);
				m_triangles.PushBack(point);
	
				point.m_posit.m_x = GLfloat(faceVertex[i].m_x);
				point.m_posit.m_y = GLfloat(faceVertex[i].m_y);
				point.m_posit.m_z = GLfloat(faceVertex[i].m_z);
				point.m_normal.m_x = GLfloat(normal.m_x);
				point.m_normal.m_y = GLfloat(normal.m_y);
				point.m_normal.m_z = GLfloat(normal.m_z);
				m_triangles.PushBack(point);
			}
		}
	
		ndArray<glPositionNormal> m_triangles;
	};
	
	ndDrawShape drawShapes;
	collision->DebugShape(ndGetIdentityMatrix(), drawShapes);
	if (drawShapes.m_triangles.GetCount())
	{
		ndArray<ndInt32> m_triangles(drawShapes.m_triangles.GetCount());
		m_triangles.SetCount(drawShapes.m_triangles.GetCount());

		m_indexCount = ndInt32(m_triangles.GetCount());
		ndInt32 vertexCount = ndVertexListToIndexList(&drawShapes.m_triangles[0].m_posit.m_x, sizeof(glPositionNormal), 6, ndInt32(drawShapes.m_triangles.GetCount()), &m_triangles[0], GLfloat(1.0e-6f));

		//m_shader = shaderCache.m_flatShaded;
		//m_color.m_x = 1.0f;
		//m_color.m_y = 1.0f;
		//m_color.m_z = 1.0f;
		//m_color.m_w = 1.0f;
	
		glGenVertexArrays(1, &m_vertextArrayBuffer);
		glBindVertexArray(m_vertextArrayBuffer);
	
		glGenBuffers(1, &m_vertexBuffer);
		glBindBuffer(GL_ARRAY_BUFFER, m_vertexBuffer);
	
		glBufferData(GL_ARRAY_BUFFER, GLsizeiptr(vertexCount * sizeof(glPositionNormal)), &drawShapes.m_triangles[0].m_posit.m_x, GL_STATIC_DRAW);
	
		glEnableVertexAttribArray(0);
		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glPositionNormal), (void*)OFFSETOF(glPositionNormal, m_posit));
	
		glEnableVertexAttribArray(1);
		glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(glPositionNormal), (void*)OFFSETOF(glPositionNormal, m_normal));
	
		glGenBuffers(1, &m_indexBuffer);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_indexBuffer);
		glBufferData(GL_ELEMENT_ARRAY_BUFFER, GLsizeiptr(m_indexCount * sizeof(GLuint)), &m_triangles[0], GL_STATIC_DRAW);
	
		glBindVertexArray(0);
	
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
		glBindBuffer(GL_ARRAY_BUFFER, 0);

		ndRenderPrimitiveMeshSegment& segment = m_segments.Append()->GetInfo();

		segment.m_material.m_specular = ndVector::m_zero;
		segment.m_material.m_reflection = ndVector::m_zero;

		segment.m_segmentStart = 0;
		segment.m_indexCount = m_indexCount;
		
		//mesh.GetMaterialGetIndexStream(geometryHandle, handle, &indices[segmentStart]);
		//segmentStart += segment.m_indexCount;

		GLuint shader = m_context->m_shaderCache->m_debugDiffuseSolidEffect;
		glUseProgram(shader);
		m_debugSolidColorBlock.m_projectMatrixLocation = glGetUniformLocation(shader, "projectionMatrix");
		m_debugSolidColorBlock.m_viewModelMatrixLocation = glGetUniformLocation(shader, "viewModelMatrix");
		m_debugSolidColorBlock.m_diffuseColor = glGetUniformLocation(shader, "diffuseColor");
		m_debugSolidColorBlock.m_directionalLightAmbient = glGetUniformLocation(shader, "directionalLightAmbient");
		m_debugSolidColorBlock.m_directionalLightIntesity = glGetUniformLocation(shader, "directionalLightIntesity");
		m_debugSolidColorBlock.m_directionalLightDirection = glGetUniformLocation(shader, "directionalLightDirection");
		glUseProgram(0);
	}
}

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
	for (ndInt32 handle = mesh.GetFirstMaterial(geometryHandle); handle != -1; handle = mesh.GetNextMaterial(geometryHandle, handle))
	{
		ndRenderPrimitiveMeshSegment& segment = m_segments.Append()->GetInfo();
	
		segment.m_material.m_texture = material.m_texture;
		segment.m_material.m_diffuse = material.m_diffuse;
		segment.m_material.m_opacity = material.m_opacity;
		segment.m_material.m_specular = material.m_specular;
		segment.m_material.m_castShadows = material.m_castShadows;
		segment.m_material.m_specularPower = material.m_specularPower;
	
		segment.m_indexCount = mesh.GetMaterialIndexCount(geometryHandle, handle);
	
		segment.m_segmentStart = segmentStart;
		mesh.GetMaterialGetIndexStream(geometryHandle, handle, &indices[segmentStart]);
		segmentStart += segment.m_indexCount;
	}
	mesh.MaterialGeometryEnd(geometryHandle);

	// optimize this mesh for hardware buffers if possible
	OptimizeForRender(&points[0], vertexCount, &indices[0], indexCount);
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

	{
		// solid color no shadow
		GLuint shader = m_context->m_shaderCache->m_diffuseEffect;
		glUseProgram(shader);
		//m_normalMatrixLocation = glGetUniformLocation(shader, "normalMatrix");
		m_solidColorBlock.m_texture = glGetUniformLocation(shader, "texture0");
		m_solidColorBlock.m_environmentMap = glGetUniformLocation(shader, "environmentMap");

		m_solidColorBlock.m_projectMatrixLocation = glGetUniformLocation(shader, "projectionMatrix");
		m_solidColorBlock.m_viewModelMatrixLocation = glGetUniformLocation(shader, "viewModelMatrix");
		m_solidColorBlock.m_diffuseColor = glGetUniformLocation(shader, "diffuseColor");
		m_solidColorBlock.m_specularColor = glGetUniformLocation(shader, "specularColor");
		m_solidColorBlock.m_reflectionColor = glGetUniformLocation(shader, "reflectionColor");
		m_solidColorBlock.m_directionalLightAmbient = glGetUniformLocation(shader, "directionalLightAmbient");
		m_solidColorBlock.m_directionalLightIntesity = glGetUniformLocation(shader, "directionalLightIntesity");
		m_solidColorBlock.m_directionalLightDirection = glGetUniformLocation(shader, "directionalLightDirection");
		m_solidColorBlock.m_specularAlpha = glGetUniformLocation(shader, "specularAlpha");
		glUseProgram(0);
	}

	{
		// solid color plus shadows
		GLuint shader = m_context->m_shaderCache->m_diffuseShadowEffect;
		glUseProgram(shader);
		//m_normalMatrixLocation = glGetUniformLocation(shader, "normalMatrix");

		m_solidShadowColorBlock.m_texture = glGetUniformLocation(shader, "texture0");
		m_solidShadowColorBlock.m_environmentMap = glGetUniformLocation(shader, "environmentMap");
		m_solidShadowColorBlock.m_depthMapTexture = glGetUniformLocation(shader, "shadowMapTexture");

		m_solidShadowColorBlock.m_projectMatrixLocation = glGetUniformLocation(shader, "projectionMatrix");
		m_solidShadowColorBlock.m_viewModelMatrixLocation = glGetUniformLocation(shader, "viewModelMatrix");
		m_solidShadowColorBlock.m_diffuseColor = glGetUniformLocation(shader, "diffuseColor");
		m_solidShadowColorBlock.m_specularColor = glGetUniformLocation(shader, "specularColor");
		m_solidShadowColorBlock.m_reflectionColor = glGetUniformLocation(shader, "reflectionColor");
		m_solidShadowColorBlock.m_directionalLightAmbient = glGetUniformLocation(shader, "directionalLightAmbient");
		m_solidShadowColorBlock.m_directionalLightIntesity = glGetUniformLocation(shader, "directionalLightIntesity");
		m_solidShadowColorBlock.m_directionalLightDirection = glGetUniformLocation(shader, "directionalLightDirection");
		m_solidShadowColorBlock.m_specularAlpha = glGetUniformLocation(shader, "specularAlpha");

		m_solidShadowColorBlock.m_shadowSlices = glGetUniformLocation(shader, "shadowSlices");
		m_solidShadowColorBlock.m_worldMatrix = glGetUniformLocation(shader, "modelWorldMatrix");
		m_solidShadowColorBlock.m_directionLightViewProjectionMatrixShadow = glGetUniformLocation(shader, "directionaLightViewProjectionMatrix");
		glUseProgram(0);
	}

	{
		// transparent color plus shadowes
		GLuint shader = m_context->m_shaderCache->m_diffuseTransparentEffect;
		glUseProgram(shader);
		//m_normalMatrixLocation = glGetUniformLocation(shader, "normalMatrix");
		m_transparencyColorBlock.m_texture = glGetUniformLocation(shader, "texture0");
		m_transparencyColorBlock.m_environmentMap = glGetUniformLocation(shader, "environmentMap");

		m_transparencyColorBlock.m_projectMatrixLocation = glGetUniformLocation(shader, "projectionMatrix");
		m_transparencyColorBlock.m_viewModelMatrixLocation = glGetUniformLocation(shader, "viewModelMatrix");
		m_transparencyColorBlock.m_diffuseColor = glGetUniformLocation(shader, "diffuseColor");
		m_transparencyColorBlock.m_specularColor = glGetUniformLocation(shader, "specularColor");
		m_transparencyColorBlock.m_reflectionColor = glGetUniformLocation(shader, "reflectionColor");
		m_transparencyColorBlock.m_directionalLightAmbient = glGetUniformLocation(shader, "directionalLightAmbient");
		m_transparencyColorBlock.m_directionalLightIntesity = glGetUniformLocation(shader, "directionalLightIntesity");
		m_transparencyColorBlock.m_directionalLightDirection = glGetUniformLocation(shader, "directionalLightDirection");
		m_transparencyColorBlock.m_specularAlpha = glGetUniformLocation(shader, "specularAlpha");
		m_transparencyColorBlock.m_opacity = glGetUniformLocation(shader, "opacity");
		glUseProgram(0);
	}

	m_vertexCount = pointCount;
	m_indexCount = indexCount;
}

void ndRenderPrimitiveMeshImplement::Render(const ndRender* const render, const ndMatrix& modelMatrix, ndRenderPassMode renderMode) const
{
	switch (renderMode)
	{
		case m_shadowMap:
			RenderShadowMap(render, modelMatrix);
			break;

		case m_solidColor:
			RenderSolidColor(render, modelMatrix);
			break;

		case m_shadowSolidColor:
			RenderShadowSolidColor(render, modelMatrix);
			break;

		case m_transparencyBackface:
			RenderTransparency(render, modelMatrix, true);
			break;

		case m_transparencyFrontface:
			RenderTransparency(render, modelMatrix, false);
			break;

		case m_debugDisplaySolidMesh:
			RenderDebugShape(render, modelMatrix);
			break;

		default:
		ndAssert(0);
	}
}

void ndRenderPrimitiveMeshImplement::RenderShadowMap(const ndRender* const render, const ndMatrix& lightMatrix) const
{
	ndRenderPassShadowsImplement* const owner = render->m_cachedShadowPass;
	ndAssert(owner);

	bool castShadow = true;
	for (ndList<ndRenderPrimitiveMeshSegment>::ndNode* node = m_segments.GetFirst(); node && castShadow; node = node->GetNext())
	{
		ndRenderPrimitiveMeshSegment& segment = node->GetInfo();
		castShadow = castShadow && segment.m_material.m_castShadows;
	}

	if (castShadow)
	{
		glMatrix matrix(lightMatrix);
		glUniformMatrix4fv(GLint(owner->m_modelProjectionMatrixLocation), 1, false, &matrix[0][0]);

		glBindVertexArray(m_vertextArrayBuffer);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_indexBuffer);

		for (ndList<ndRenderPrimitiveMeshSegment>::ndNode* node = m_segments.GetFirst(); node; node = node->GetNext())
		{
			ndRenderPrimitiveMeshSegment& segment = node->GetInfo();
			if (segment.m_material.m_castShadows)
			{
				glDrawElements(GL_TRIANGLES, segment.m_indexCount, GL_UNSIGNED_INT, (void*)(segment.m_segmentStart * sizeof(GL_UNSIGNED_INT)));
			}
		}
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
		glBindVertexArray(0);
	}
}

void ndRenderPrimitiveMeshImplement::RenderSolidColor(const ndRender* const render, const ndMatrix& modelMatrix) const
{
	bool castShadow = true;
	for (ndList<ndRenderPrimitiveMeshSegment>::ndNode* node = m_segments.GetFirst(); node && castShadow; node = node->GetNext())
	{
		ndRenderPrimitiveMeshSegment& segment = node->GetInfo();
		castShadow = castShadow && segment.m_material.m_castShadows;
	}

	if (castShadow)
	{
		return;
	}

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
	
	glUniform3fv(m_solidColorBlock.m_directionalLightDirection, 1, &glSunlightDir[0]);
	glUniform3fv(m_solidColorBlock.m_directionalLightAmbient, 1, &glSunlightAmbient[0]);
	glUniform3fv(m_solidColorBlock.m_directionalLightIntesity, 1, &glSunlightIntensity[0]);

	//glUniformMatrix4fv(m_normalMatrixLocation, 1, false, &glViewModelMatrix[0][0]);
	glUniformMatrix4fv(m_solidColorBlock.m_projectMatrixLocation, 1, false, &glProjectionMatrix[0][0]);
	glUniformMatrix4fv(m_solidColorBlock.m_viewModelMatrixLocation, 1, false, &glViewModelMatrix[0][0]);

	ndRenderPassEnvironment* const environment = render->m_cachedEnvironmentPass;
	ndAssert(environment);
	ndRenderTextureImage* const environmentTexture = (ndRenderTextureImage*)*environment->m_cubeMap;

	glActiveTexture(GL_TEXTURE1);
	glBindTexture(GL_TEXTURE_CUBE_MAP, environmentTexture->m_texture);

	glBindVertexArray(m_vertextArrayBuffer);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_indexBuffer);
	
	glActiveTexture(GL_TEXTURE0);
	for (ndList<ndRenderPrimitiveMeshSegment>::ndNode* node = m_segments.GetFirst(); node; node = node->GetNext())
	{
		ndRenderPrimitiveMeshSegment& segment = node->GetInfo();
		if (!segment.m_material.m_castShadows && (segment.m_material.m_opacity > ndFloat32 (0.99f)))
		{
			const ndRenderPrimitiveMeshMaterial* const material = &segment.m_material;
			const ndRenderTextureImageCommon* const image = (ndRenderTextureImageCommon*) *material->m_texture;
			ndAssert(image);

			const glVector4 diffuse(material->m_diffuse);
			const glVector4 specular(material->m_specular);
			const glVector4 reflection(material->m_reflection);

			glUniform3fv(m_solidColorBlock.m_diffuseColor, 1, &diffuse[0]);
			glUniform3fv(m_solidColorBlock.m_specularColor, 1, &specular[0]);
			glUniform3fv(m_solidColorBlock.m_reflectionColor, 1, &reflection[0]);
			glUniform1fv(m_solidColorBlock.m_specularAlpha, 1, &material->m_specularPower);
			
			glBindTexture(GL_TEXTURE_2D, image->m_texture);
			glDrawElements(GL_TRIANGLES, segment.m_indexCount, GL_UNSIGNED_INT, (void*)(segment.m_segmentStart * sizeof(GL_UNSIGNED_INT)));
		}
	}
	
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
	glBindVertexArray(0);
	glUseProgram(0);
}

void ndRenderPrimitiveMeshImplement::RenderShadowSolidColor(const ndRender* const render, const ndMatrix& modelMatrix) const
{
	//bool castShadow = true;
	//for (ndList<ndRenderPrimitiveMeshSegment>::ndNode* node = m_segments.GetFirst(); node && castShadow; node = node->GetNext())
	//{
	//	ndRenderPrimitiveMeshSegment& segment = node->GetInfo();
	//	castShadow = castShadow && segment.m_material.m_castShadows;
	//}
	//
	//if (!castShadow)
	//{
	//	return;
	//}
	
	const ndSharedPtr<ndRenderSceneCamera>& camera = render->GetCamera();
	
	GLuint shader = m_context->m_shaderCache->m_diffuseShadowEffect;
	glUseProgram(shader);
	
	const ndMatrix viewMatrix(camera->m_invViewMatrix);
	const ndMatrix modelViewMatrix(modelMatrix * viewMatrix);

	const glMatrix worldMatrix(modelMatrix);
	const glMatrix glViewModelMatrix(modelViewMatrix);
	const glMatrix glProjectionMatrix(camera->m_projectionMatrix);
	
	const glVector4 glSunlightAmbient(render->m_sunLightAmbient);
	const glVector4 glSunlightIntensity(render->m_sunLightIntesity);
	const glVector4 glSunlightDir(viewMatrix.RotateVector(render->m_sunLightDir));
	
	glUniform3fv(m_solidShadowColorBlock.m_directionalLightDirection, 1, &glSunlightDir[0]);
	glUniform3fv(m_solidShadowColorBlock.m_directionalLightAmbient, 1, &glSunlightAmbient[0]);
	glUniform3fv(m_solidShadowColorBlock.m_directionalLightIntesity, 1, &glSunlightIntensity[0]);
	
	//glUniformMatrix4fv(m_normalMatrixLocation, 1, false, &glViewModelMatrix[0][0]);
	glUniformMatrix4fv(m_solidShadowColorBlock.m_projectMatrixLocation, 1, false, &glProjectionMatrix[0][0]);
	glUniformMatrix4fv(m_solidShadowColorBlock.m_viewModelMatrixLocation, 1, false, &glViewModelMatrix[0][0]);

	ndRenderPassShadowsImplement* const shadowPass = render->m_cachedShadowPass;
	ndAssert(shadowPass);

	glVector4 cameraSpaceSplits(shadowPass->m_cameraSpaceSplits);
	glMatrix lightViewProjectMatrix[4];
	for (ndInt32 i = 0; i < 4; ++i)
	{
		lightViewProjectMatrix[i] = shadowPass->m_lighProjectionMatrix[i];
	}
	glUniform1i(m_solidShadowColorBlock.m_depthMapTexture, 1);
	glUniformMatrix4fv(m_solidShadowColorBlock.m_worldMatrix, 1, GL_FALSE, &worldMatrix[0][0]);
	glUniformMatrix4fv(m_solidShadowColorBlock.m_directionLightViewProjectionMatrixShadow, 4, GL_FALSE, &lightViewProjectMatrix[0][0][0]);
	glUniform4fv(m_solidShadowColorBlock.m_shadowSlices, 1, &cameraSpaceSplits[0]);

	ndRenderPassEnvironment* const environment = render->m_cachedEnvironmentPass;
	ndAssert(environment);
	ndRenderTextureImage* const environmentTexture = (ndRenderTextureImage*)*environment->m_cubeMap;

	glActiveTexture(GL_TEXTURE2);
	glBindTexture(GL_TEXTURE_CUBE_MAP, environmentTexture->m_texture);

	glActiveTexture(GL_TEXTURE1);
	glBindTexture(GL_TEXTURE_2D, shadowPass->m_shadowMapTexture);

	glBindVertexArray(m_vertextArrayBuffer);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_indexBuffer);
	
	glActiveTexture(GL_TEXTURE0);
	for (ndList<ndRenderPrimitiveMeshSegment>::ndNode* node = m_segments.GetFirst(); node; node = node->GetNext())
	{
		ndRenderPrimitiveMeshSegment& segment = node->GetInfo();
		if (segment.m_material.m_opacity > ndFloat32(0.99f))
		{
			const ndRenderPrimitiveMeshMaterial* const material = &segment.m_material;
			const ndRenderTextureImageCommon* const image = (ndRenderTextureImageCommon*)*material->m_texture;
			ndAssert(image);
	
			const glVector4 diffuse(material->m_diffuse);
			const glVector4 specular(material->m_specular);
			const glVector4 reflection(material->m_reflection);

			glUniform3fv(m_solidShadowColorBlock.m_diffuseColor, 1, &diffuse[0]);
			glUniform3fv(m_solidShadowColorBlock.m_specularColor, 1, &specular[0]);
			glUniform3fv(m_solidShadowColorBlock.m_reflectionColor, 1, &reflection[0]);
			glUniform1fv(m_solidShadowColorBlock.m_specularAlpha, 1, &material->m_specularPower);
	
			glBindTexture(GL_TEXTURE_2D, image->m_texture);
			glDrawElements(GL_TRIANGLES, segment.m_indexCount, GL_UNSIGNED_INT, (void*)(segment.m_segmentStart * sizeof(GL_UNSIGNED_INT)));
		}
	}
	
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
	glBindVertexArray(0);
	glUseProgram(0);
}

void ndRenderPrimitiveMeshImplement::RenderTransparency(const ndRender* const render, const ndMatrix& modelMatrix, bool backface) const
{
	bool isOpaque = true;
	for (ndList<ndRenderPrimitiveMeshSegment>::ndNode* node = m_segments.GetFirst(); node && isOpaque; node = node->GetNext())
	{
		ndRenderPrimitiveMeshSegment& segment = node->GetInfo();
		isOpaque = isOpaque && (segment.m_material.m_opacity > ndFloat32 (0.99f));
	}
	if (isOpaque)
	{
		return;
	}

	const ndSharedPtr<ndRenderSceneCamera>& camera = render->GetCamera();
	
	GLuint shader = m_context->m_shaderCache->m_diffuseTransparentEffect;
	glUseProgram(shader);
	
	const ndMatrix viewMatrix(camera->m_invViewMatrix);
	const ndMatrix modelViewMatrix(modelMatrix * viewMatrix);
	
	const glMatrix glViewModelMatrix(modelViewMatrix);
	const glMatrix glProjectionMatrix(camera->m_projectionMatrix);
	
	const glVector4 glSunlightAmbient(render->m_sunLightAmbient);
	const glVector4 glSunlightIntensity(render->m_sunLightIntesity);
	const glVector4 glSunlightDir(viewMatrix.RotateVector(render->m_sunLightDir));
	
	glUniform3fv(m_transparencyColorBlock.m_directionalLightDirection, 1, &glSunlightDir[0]);
	glUniform3fv(m_transparencyColorBlock.m_directionalLightAmbient, 1, &glSunlightAmbient[0]);
	glUniform3fv(m_transparencyColorBlock.m_directionalLightIntesity, 1, &glSunlightIntensity[0]);
	
	//glUniformMatrix4fv(m_normalMatrixLocation, 1, false, &glViewModelMatrix[0][0]);
	glUniformMatrix4fv(m_transparencyColorBlock.m_projectMatrixLocation, 1, false, &glProjectionMatrix[0][0]);
	glUniformMatrix4fv(m_transparencyColorBlock.m_viewModelMatrixLocation, 1, false, &glViewModelMatrix[0][0]);
	
	ndRenderPassEnvironment* const environment = render->m_cachedEnvironmentPass;
	ndAssert(environment);
	ndRenderTextureImage* const environmentTexture = (ndRenderTextureImage*)*environment->m_cubeMap;

	glDepthMask(GL_FALSE);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	backface ? glFrontFace(GL_CW) : glFrontFace(GL_CCW);
	
	glActiveTexture(GL_TEXTURE1);
	glBindTexture(GL_TEXTURE_CUBE_MAP, environmentTexture->m_texture);
	
	glBindVertexArray(m_vertextArrayBuffer);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_indexBuffer);
	
	glActiveTexture(GL_TEXTURE0);
	for (ndList<ndRenderPrimitiveMeshSegment>::ndNode* node = m_segments.GetFirst(); node; node = node->GetNext())
	{
		ndRenderPrimitiveMeshSegment& segment = node->GetInfo();
		if (segment.m_material.m_opacity <= ndFloat32(0.99f))
		{
			const ndRenderPrimitiveMeshMaterial* const material = &segment.m_material;
			const ndRenderTextureImageCommon* const image = (ndRenderTextureImageCommon*)*material->m_texture;
			ndAssert(image);
	
			const glVector4 diffuse(material->m_diffuse);
			const glVector4 specular(material->m_specular);
			const glVector4 reflection(material->m_reflection);
	
			glUniform3fv(m_transparencyColorBlock.m_diffuseColor, 1, &diffuse[0]);
			glUniform3fv(m_transparencyColorBlock.m_specularColor, 1, &specular[0]);
			glUniform3fv(m_transparencyColorBlock.m_reflectionColor, 1, &reflection[0]);
			glUniform1fv(m_transparencyColorBlock.m_specularAlpha, 1, &material->m_specularPower);
			glUniform1fv(m_transparencyColorBlock.m_opacity, 1, &material->m_opacity);
	
			glBindTexture(GL_TEXTURE_2D, image->m_texture);
			glDrawElements(GL_TRIANGLES, segment.m_indexCount, GL_UNSIGNED_INT, (void*)(segment.m_segmentStart * sizeof(GL_UNSIGNED_INT)));
		}
	}
	
	glDisable(GL_BLEND);
	glDepthMask(GL_TRUE);

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
	glBindVertexArray(0);
	glUseProgram(0);
}

void ndRenderPrimitiveMeshImplement::RenderDebugShape(const ndRender* const render, const ndMatrix& modelViewMatrix) const
{

}