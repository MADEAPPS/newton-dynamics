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
#include "ndRenderShader.h"
#include "ndRenderContext.h"
#include "ndRenderPassDebug.h"
#include "ndRenderOpenGlUtil.h"
#include "ndRenderSceneCamera.h"
#include "ndRenderShaderCache.h"
#include "ndRenderPassShadows.h"
#include "ndRenderTextureImage.h"
#include "ndRenderPassEnvironment.h"
#include "ndRenderPrimitiveImplement.h"
#include "ndRenderPassShadowsImplement.h"

ndRenderShaderBlock::ndRenderShaderBlock()
	:m_shader(0)
{
}

ndRenderShaderBlock::~ndRenderShaderBlock()
{
}

void ndRenderShaderBlock::SetParameters(GLuint shader)
{
	m_shader = shader;
	glUseProgram(m_shader);
}

void ndRenderShaderBlock::EndParameters()
{
	glUseProgram(0);
}

// *********************************************************************
// 
// *********************************************************************
void ndRenderShaderSetZbufferCleanBlock::GetShaderParameters(const ndRenderShaderCache* const shaderCache)
{
	SetParameters(shaderCache->m_setZbufferEffect);
	EndParameters();
}

void ndRenderShaderSetZbufferCleanBlock::SetParameters(GLuint shader)
{
	ndRenderShaderBlock::SetParameters(shader);
	m_viewModelProjectionMatrix = glGetUniformLocation(m_shader, "viewModelProjectionMatrix");
}

void ndRenderShaderSetZbufferCleanBlock::Render(const ndRenderPrimitiveImplement* const self, const ndRender* const render, const ndMatrix& modelMatrix) const
{
	const ndRenderSceneCamera* const camera = render->GetCamera()->FindCameraNode();

	const ndMatrix modelViewProjectionMatrixMatrix(modelMatrix * camera->m_invViewRrojectionMatrix);
	const glMatrix glViewModelProjectionMatrix(modelViewProjectionMatrixMatrix);

	glCullFace(GL_BACK);
	glFrontFace(GL_CCW);
	glDisable(GL_BLEND);
	glDepthFunc(GL_LEQUAL);
	glEnable(GL_CULL_FACE);
	glEnable(GL_DEPTH_TEST);

	glUseProgram(m_shader);

	glUniformMatrix4fv(m_viewModelProjectionMatrix, 1, false, &glViewModelProjectionMatrix[0][0]);

	glBindVertexArray(self->m_vertextArrayBuffer);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, self->m_indexBuffer);

	glDrawElements(GL_TRIANGLES, self->m_indexCount, GL_UNSIGNED_INT, (void*)0);

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
	glBindVertexArray(0);
	glUseProgram(0);
}

// *********************************************************************
// 
// *********************************************************************
void ndRenderShaderGenerateShadowMapBlock::GetShaderParameters(const ndRenderShaderCache* const shaderCache)
{
	SetParameters(shaderCache->m_generateShadowMapsEffect);
	EndParameters();
}

void ndRenderShaderGenerateShadowMapBlock::SetParameters(GLuint shader)
{
	ndRenderShaderSetZbufferCleanBlock::SetParameters(shader);
}

void ndRenderShaderGenerateShadowMapBlock::Render(const ndRenderPrimitiveImplement* const self, const ndRender* const, const ndMatrix& modelMatrix) const
{
	glUseProgram(m_shader);

	const glMatrix matrix(modelMatrix);
	glUniformMatrix4fv(m_viewModelProjectionMatrix, 1, false, &matrix[0][0]);

	glBindVertexArray(self->m_vertextArrayBuffer);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, self->m_indexBuffer);
		
	for (ndList<ndRenderPrimitiveSegment>::ndNode* node = self->m_owner->m_segments.GetFirst(); node; node = node->GetNext())
	{
		ndRenderPrimitiveSegment& segment = node->GetInfo();
		if (segment.m_material.m_castShadows)
		{
			glDrawElements(GL_TRIANGLES, segment.m_indexCount, GL_UNSIGNED_INT, (void*)(segment.m_segmentStart * sizeof(GL_UNSIGNED_INT)));
		}
	}
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
	glBindVertexArray(0);

	ndAssert(glCheckFramebufferStatus(GL_FRAMEBUFFER) == GL_FRAMEBUFFER_COMPLETE);
	glUseProgram(0);
}	

// *********************************************************************
// 
// *********************************************************************
void ndRenderShaderGenerateInstanceShadowMapBlock::GetShaderParameters(const ndRenderShaderCache* const shaderCache)
{
	SetParameters(shaderCache->m_generateInstancedShadowMapsEffect);
	EndParameters();
}

void ndRenderShaderGenerateInstanceShadowMapBlock::SetParameters(GLuint shader)
{
	ndRenderShaderGenerateShadowMapBlock::SetParameters(shader);
}

void ndRenderShaderGenerateInstanceShadowMapBlock::Render(const ndRenderPrimitiveImplement* const self, const ndRender* const, const ndMatrix& modelMatrix) const
{
	glUseProgram(m_shader);

	const glMatrix matrix(modelMatrix);
	glUniformMatrix4fv(m_viewModelProjectionMatrix, 1, false, &matrix[0][0]);

	//glPolygonOffset(GLfloat(1.0f), GLfloat(1024.0f * 8.0f));
	//glEnable(GL_POLYGON_OFFSET_FILL);

	glBindVertexArray(self->m_vertextArrayBuffer);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, self->m_indexBuffer);

	for (ndList<ndRenderPrimitiveSegment>::ndNode* node = self->m_owner->m_segments.GetFirst(); node; node = node->GetNext())
	{
		ndRenderPrimitiveSegment& segment = node->GetInfo();
		if (segment.m_material.m_castShadows)
		{
			glDrawElementsInstanced(GL_TRIANGLES, segment.m_indexCount, GL_UNSIGNED_INT, (void*)(segment.m_segmentStart * sizeof(GL_UNSIGNED_INT)), GLsizei(self->m_instanceMatrixArray.GetCount()));
		}
	}
	
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
	glBindVertexArray(0);

	//glDisable(GL_POLYGON_OFFSET_FILL);
	ndAssert(glCheckFramebufferStatus(GL_FRAMEBUFFER) == GL_FRAMEBUFFER_COMPLETE);
	glUseProgram(0);
}

// *********************************************************************
// 
// *********************************************************************
void ndRenderShaderDebugFlatShadedDiffusedBlock::GetShaderParameters(const ndRenderShaderCache* const shaderCache)
{
	SetParameters(shaderCache->m_debugFlatShadedDiffuseEffect);
	EndParameters();
}

void ndRenderShaderDebugFlatShadedDiffusedBlock::SetParameters(GLuint shader)
{
	ndRenderShaderBlock::SetParameters(shader);
	m_diffuseColor = glGetUniformLocation(m_shader, "diffuseColor");
	m_projectMatrixLocation = glGetUniformLocation(m_shader, "projectionMatrix");
	m_viewModelMatrixLocation = glGetUniformLocation(m_shader, "viewModelMatrix");
	m_directionalLightAmbient = glGetUniformLocation(m_shader, "directionalLightAmbient");
	m_directionalLightIntesity = glGetUniformLocation(m_shader, "directionalLightIntesity");
	m_directionalLightDirection = glGetUniformLocation(m_shader, "directionalLightDirection");
}

void ndRenderShaderDebugFlatShadedDiffusedBlock::Render(const ndRenderPrimitiveImplement* const self, const ndRender* const render, const ndMatrix& modelMatrix) const
{
	const ndRenderSceneCamera* const camera = render->GetCamera()->FindCameraNode();
	
	const ndMatrix viewMatrix(camera->m_invViewMatrix);
	const ndMatrix modelViewMatrix(modelMatrix * viewMatrix);
	
	const glMatrix glViewModelMatrix(modelViewMatrix);
	const glMatrix glProjectionMatrix(camera->m_projectionMatrix);
	
	ndRenderPrimitiveSegment& segment = self->m_owner->m_segments.GetFirst()->GetInfo();
	const ndRenderPrimitiveMaterial* const material = &segment.m_material;
	
	const glVector4 diffuse(material->m_diffuse);
	const glVector4 glSunlightAmbient(render->m_sunLightAmbient);
	const glVector4 glSunlightIntensity(render->m_sunLightIntesity);
	const glVector4 glSunlightDir(viewMatrix.RotateVector(render->m_sunLightDir));
	
	glCullFace(GL_BACK);
	glFrontFace(GL_CCW);
	glDisable(GL_BLEND);
	glDepthFunc(GL_LEQUAL);
	glEnable(GL_CULL_FACE);
	glEnable(GL_DEPTH_TEST);
	
	glUseProgram(m_shader);
	
	glUniformMatrix4fv(m_projectMatrixLocation, 1, false, &glProjectionMatrix[0][0]);
	glUniformMatrix4fv(m_viewModelMatrixLocation, 1, false, &glViewModelMatrix[0][0]);
	glUniform3fv(m_diffuseColor, 1, &diffuse[0]);
	glUniform3fv(m_directionalLightDirection, 1, &glSunlightDir[0]);
	glUniform3fv(m_directionalLightAmbient, 1, &glSunlightAmbient[0]);
	glUniform3fv(m_directionalLightIntesity, 1, &glSunlightIntensity[0]);
	
	glBindVertexArray(self->m_vertextArrayBuffer);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, self->m_indexBuffer);
	
	glDrawElements(GL_TRIANGLES, self->m_indexCount, GL_UNSIGNED_INT, (void*)0);
	
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
	glBindVertexArray(0);
	glUseProgram(0);
}

// *********************************************************************
// 
// *********************************************************************
void ndRenderShaderDebugWireframeDiffuseBlock::Render(const ndRenderPrimitiveImplement* const self, const ndRender* const render, const ndMatrix& modelMatrix) const
{
	const ndRenderSceneCamera* const camera = render->GetCamera()->FindCameraNode();
	
	const ndMatrix viewMatrix(camera->m_invViewMatrix);
	const ndMatrix modelViewMatrix(modelMatrix * viewMatrix);
	
	const glMatrix glViewModelMatrix(modelViewMatrix);
	const glMatrix glProjectionMatrix(camera->m_projectionMatrix);
	
	ndRenderPrimitiveSegment& segment = self->m_owner->m_segments.GetFirst()->GetInfo();
	const ndRenderPrimitiveMaterial* const material = &segment.m_material;
	
	const glVector4 diffuse(material->m_diffuse);
	const glVector4 glSunlightAmbient(render->m_sunLightAmbient);
	const glVector4 glSunlightIntensity(render->m_sunLightIntesity);
	const glVector4 glSunlightDir(viewMatrix.RotateVector(render->m_sunLightDir));
	
	glCullFace(GL_BACK);
	glFrontFace(GL_CCW);
	glDisable(GL_BLEND);
	glDepthFunc(GL_LEQUAL);
	glEnable(GL_CULL_FACE);
	glEnable(GL_DEPTH_TEST);
	
	glUseProgram(m_shader);
	
	glUniformMatrix4fv(m_projectMatrixLocation, 1, false, &glProjectionMatrix[0][0]);
	glUniformMatrix4fv(m_viewModelMatrixLocation, 1, false, &glViewModelMatrix[0][0]);
	glUniform3fv(m_diffuseColor, 1, &diffuse[0]);
	glUniform3fv(m_directionalLightDirection, 1, &glSunlightDir[0]);
	glUniform3fv(m_directionalLightAmbient, 1, &glSunlightAmbient[0]);
	glUniform3fv(m_directionalLightIntesity, 1, &glSunlightIntensity[0]);
	
	glBindVertexArray(self->m_vertextArrayBuffer);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, self->m_indexBuffer);
	
	glDrawElements(GL_LINES, self->m_indexCount, GL_UNSIGNED_INT, (void*)0);
	
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
	glBindVertexArray(0);
	glUseProgram(0);
}

// *********************************************************************
// 
// *********************************************************************
void ndRenderShaderOpaqueDiffusedColorBlock::GetShaderParameters(const ndRenderShaderCache* const shaderCache)
{
	SetParameters(shaderCache->m_diffuseEffect);
	EndParameters();
}

void ndRenderShaderOpaqueDiffusedColorBlock::SetParameters(GLuint shader)
{
	ndRenderShaderDebugFlatShadedDiffusedBlock::SetParameters(shader);
	m_texture = glGetUniformLocation(m_shader, "texture0");
	m_specularColor = glGetUniformLocation(m_shader, "specularColor");
	m_cameraToWorld = glGetUniformLocation(m_shader, "cameraToWorld");
	m_specularAlpha = glGetUniformLocation(m_shader, "specularAlpha");
	m_environmentMap = glGetUniformLocation(m_shader, "environmentMap");
	m_reflectionColor = glGetUniformLocation(m_shader, "reflectionColor");
}

void ndRenderShaderOpaqueDiffusedColorBlock::Render(const ndRenderPrimitiveImplement* const self, const ndRender* const render, const ndMatrix& modelMatrix) const
{
	const ndRenderSceneCamera* const camera = render->GetCamera()->FindCameraNode();

	glUseProgram(m_shader);

	const ndMatrix viewMatrix(camera->m_invViewMatrix);
	const ndMatrix modelViewMatrix(modelMatrix * viewMatrix);

	const glMatrix glViewModelMatrix(modelViewMatrix);
	const glMatrix glCameraToWorld(viewMatrix.OrthoInverse());
	const glMatrix glProjectionMatrix(camera->m_projectionMatrix);

	const glVector4 glSunlightAmbient(render->m_sunLightAmbient);
	const glVector4 glSunlightIntensity(render->m_sunLightIntesity);
	const glVector4 glSunlightDir(viewMatrix.RotateVector(render->m_sunLightDir));

	glUniform3fv(m_directionalLightDirection, 1, &glSunlightDir[0]);
	glUniform3fv(m_directionalLightAmbient, 1, &glSunlightAmbient[0]);
	glUniform3fv(m_directionalLightIntesity, 1, &glSunlightIntensity[0]);

	//glUniformMatrix4fv(m_normalMatrixLocation, 1, false, &glViewModelMatrix[0][0]);
	glUniformMatrix4fv(m_cameraToWorld, 1, false, &glCameraToWorld[0][0]);
	glUniformMatrix4fv(m_projectMatrixLocation, 1, false, &glProjectionMatrix[0][0]);
	glUniformMatrix4fv(m_viewModelMatrixLocation, 1, false, &glViewModelMatrix[0][0]);

	ndRenderPassEnvironment* const environment = render->m_cachedEnvironmentPass;
	ndAssert(environment);
	ndRenderTextureImage* const environmentTexture = (ndRenderTextureImage*)*environment->m_cubeMap;

	glActiveTexture(GL_TEXTURE1);
	glBindTexture(GL_TEXTURE_CUBE_MAP, environmentTexture->m_texture);

	glBindVertexArray(self->m_vertextArrayBuffer);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, self->m_indexBuffer);

	glActiveTexture(GL_TEXTURE0);
	for (ndList<ndRenderPrimitiveSegment>::ndNode* node = self->m_owner->m_segments.GetFirst(); node; node = node->GetNext())
	{
		ndRenderPrimitiveSegment& segment = node->GetInfo();
		if (!segment.m_material.m_castShadows && (segment.m_material.m_opacity > ndFloat32(0.99f)))
		{
			const ndRenderPrimitiveMaterial* const material = &segment.m_material;
			const ndRenderTextureImageCommon* const image = (ndRenderTextureImageCommon*)*material->m_texture;
			ndAssert(image);

			const glVector4 diffuse(material->m_diffuse);
			const glVector4 specular(material->m_specular);
			const glVector4 reflection(material->m_reflection);

			glUniform3fv(m_diffuseColor, 1, &diffuse[0]);
			glUniform3fv(m_specularColor, 1, &specular[0]);
			glUniform3fv(m_reflectionColor, 1, &reflection[0]);
			glUniform1fv(m_specularAlpha, 1, &material->m_specularPower);

			glBindTexture(GL_TEXTURE_2D, image->m_texture);
			glDrawElements(GL_TRIANGLES, segment.m_indexCount, GL_UNSIGNED_INT, (void*)(segment.m_segmentStart * sizeof(GL_UNSIGNED_INT)));
		}
	}

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
	glBindVertexArray(0);
	glUseProgram(0);
}

// *********************************************************************
// 
// *********************************************************************
void ndRenderShaderOpaqueDiffusedShadowColorBlock::GetShaderParameters(const ndRenderShaderCache* const shaderCache)
{
	SetParameters(shaderCache->m_diffuseShadowEffect);
	EndParameters();
}

void ndRenderShaderOpaqueDiffusedShadowColorBlock::SetParameters(GLuint shader)
{
	ndRenderShaderOpaqueDiffusedColorBlock::SetParameters(shader);
	m_shadowSlices = glGetUniformLocation(m_shader, "shadowSlices");
	m_worldMatrix = glGetUniformLocation(m_shader, "modelWorldMatrix");
	m_depthMapTexture = glGetUniformLocation(m_shader, "shadowMapTexture");
	m_directionLightViewProjectionMatrixShadow = glGetUniformLocation(m_shader, "directionaLightViewProjectionMatrix");
}

void ndRenderShaderOpaqueDiffusedShadowColorBlock::Render(const ndRenderPrimitiveImplement* const self, const ndRender* const render, const ndMatrix& modelMatrix) const
{
	const ndRenderSceneCamera* const camera = render->GetCamera()->FindCameraNode();
	
	const ndMatrix viewMatrix(camera->m_invViewMatrix);
	const ndMatrix modelViewMatrix(modelMatrix * viewMatrix);
	
	const glMatrix worldMatrix(modelMatrix);
	const glMatrix glViewModelMatrix(modelViewMatrix);
	const glMatrix glCameraToWorld(viewMatrix.OrthoInverse());
	const glMatrix glProjectionMatrix(camera->m_projectionMatrix);
	
	const glVector4 glSunlightAmbient(render->m_sunLightAmbient);
	const glVector4 glSunlightIntensity(render->m_sunLightIntesity);
	const glVector4 glSunlightDir(viewMatrix.RotateVector(render->m_sunLightDir));

	glUseProgram(m_shader);

	glUniform3fv(m_directionalLightDirection, 1, &glSunlightDir[0]);
	glUniform3fv(m_directionalLightAmbient, 1, &glSunlightAmbient[0]);
	glUniform3fv(m_directionalLightIntesity, 1, &glSunlightIntensity[0]);
	
	glUniformMatrix4fv(m_projectMatrixLocation, 1, false, &glProjectionMatrix[0][0]);
	glUniformMatrix4fv(m_viewModelMatrixLocation, 1, false, &glViewModelMatrix[0][0]);
	
	ndRenderPassShadowsImplement* const shadowPass = render->m_cachedShadowPass;
	ndAssert(shadowPass);
	
	glVector4 cameraSpaceSplits(shadowPass->m_cameraSpaceSplits);
	glMatrix lightViewProjectMatrix[4];
	for (ndInt32 i = 0; i < 4; ++i)
	{
		lightViewProjectMatrix[i] = shadowPass->m_lighProjectionMatrix[i];
	}
	glUniform1i(m_depthMapTexture, 1);
	glUniformMatrix4fv(m_worldMatrix, 1, GL_FALSE, &worldMatrix[0][0]);
	glUniformMatrix4fv(m_cameraToWorld, 1, false, &glCameraToWorld[0][0]);
	glUniformMatrix4fv(m_directionLightViewProjectionMatrixShadow, 4, GL_FALSE, &lightViewProjectMatrix[0][0][0]);
	glUniform4fv(m_shadowSlices, 1, &cameraSpaceSplits[0]);
	
	ndRenderPassEnvironment* const environment = render->m_cachedEnvironmentPass;
	ndAssert(environment);
	ndRenderTextureImage* const environmentTexture = (ndRenderTextureImage*)*environment->m_cubeMap;
	
	glActiveTexture(GL_TEXTURE2);
	glBindTexture(GL_TEXTURE_CUBE_MAP, environmentTexture->m_texture);
	
	glActiveTexture(GL_TEXTURE1);
	glBindTexture(GL_TEXTURE_2D, shadowPass->m_shadowMapTexture);
	
	glBindVertexArray(self->m_vertextArrayBuffer);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, self->m_indexBuffer);
	
	glActiveTexture(GL_TEXTURE0);
	for (ndList<ndRenderPrimitiveSegment>::ndNode* node = self->m_owner->m_segments.GetFirst(); node; node = node->GetNext())
	{
		ndRenderPrimitiveSegment& segment = node->GetInfo();
		if (segment.m_material.m_opacity > ndFloat32(0.99f))
		{
			const ndRenderPrimitiveMaterial* const material = &segment.m_material;
			const ndRenderTextureImageCommon* const image = (ndRenderTextureImageCommon*)*material->m_texture;
			ndAssert(image);
	
			const glVector4 diffuse(material->m_diffuse);
			const glVector4 specular(material->m_specular);
			const glVector4 reflection(material->m_reflection);
	
			glUniform3fv(m_diffuseColor, 1, &diffuse[0]);
			glUniform3fv(m_specularColor, 1, &specular[0]);
			glUniform3fv(m_reflectionColor, 1, &reflection[0]);
			glUniform1fv(m_specularAlpha, 1, &material->m_specularPower);
	
			glBindTexture(GL_TEXTURE_2D, image->m_texture);
			glDrawElements(GL_TRIANGLES, segment.m_indexCount, GL_UNSIGNED_INT, (void*)(segment.m_segmentStart * sizeof(GL_UNSIGNED_INT)));
		}
	}
	
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
	glBindVertexArray(0);
	glUseProgram(0);
}

// *********************************************************************
// 
// *********************************************************************
void ndRenderShaderTransparentDiffusedShadowColorBlock::GetShaderParameters(const ndRenderShaderCache* const shaderCache)
{
	// transparent color plus shadows
	SetParameters(shaderCache->m_diffuseTransparentEffect);
	EndParameters();
}

void ndRenderShaderTransparentDiffusedShadowColorBlock::SetParameters(GLuint shader)
{
	ndRenderShaderOpaqueDiffusedColorBlock::SetParameters(shader);
	m_opacity = glGetUniformLocation(m_shader, "opacity");
}

void ndRenderShaderTransparentDiffusedShadowColorBlock::SetWidingMode(bool clockwise) const
{
	clockwise ? glFrontFace(GL_CW) : glFrontFace(GL_CCW);
}

void ndRenderShaderTransparentDiffusedShadowColorBlock::Render(const ndRenderPrimitiveImplement* const self, const ndRender* const render, const ndMatrix& modelMatrix) const
{
	const ndRenderSceneCamera* const camera = render->GetCamera()->FindCameraNode();
	
	glUseProgram(m_shader);

	const ndMatrix viewMatrix(camera->m_invViewMatrix);
	const ndMatrix modelViewMatrix(modelMatrix * viewMatrix);

	const glMatrix glViewModelMatrix(modelViewMatrix);
	const glMatrix glCameraToWorld(viewMatrix.OrthoInverse());
	const glMatrix glProjectionMatrix(camera->m_projectionMatrix);

	const glVector4 glSunlightAmbient(render->m_sunLightAmbient);
	const glVector4 glSunlightIntensity(render->m_sunLightIntesity);
	const glVector4 glSunlightDir(viewMatrix.RotateVector(render->m_sunLightDir));

	glUniform3fv(m_directionalLightDirection, 1, &glSunlightDir[0]);
	glUniform3fv(m_directionalLightAmbient, 1, &glSunlightAmbient[0]);
	glUniform3fv(m_directionalLightIntesity, 1, &glSunlightIntensity[0]);

	//glUniformMatrix4fv(m_normalMatrixLocation, 1, false, &glViewModelMatrix[0][0]);
	glUniformMatrix4fv(m_cameraToWorld, 1, false, &glCameraToWorld[0][0]);
	glUniformMatrix4fv(m_projectMatrixLocation, 1, false, &glProjectionMatrix[0][0]);
	glUniformMatrix4fv(m_viewModelMatrixLocation, 1, false, &glViewModelMatrix[0][0]);

	ndRenderPassEnvironment* const environment = render->m_cachedEnvironmentPass;
	ndAssert(environment);
	ndRenderTextureImage* const environmentTexture = (ndRenderTextureImage*)*environment->m_cubeMap;

	glDepthMask(GL_FALSE);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	glActiveTexture(GL_TEXTURE1);
	glBindTexture(GL_TEXTURE_CUBE_MAP, environmentTexture->m_texture);

	glBindVertexArray(self->m_vertextArrayBuffer);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, self->m_indexBuffer);

	glActiveTexture(GL_TEXTURE0);
	for (ndList<ndRenderPrimitiveSegment>::ndNode* node = self->m_owner->m_segments.GetFirst(); node; node = node->GetNext())
	{
		ndRenderPrimitiveSegment& segment = node->GetInfo();
		if (segment.m_material.m_opacity <= ndFloat32(0.99f))
		{
			const ndRenderPrimitiveMaterial* const material = &segment.m_material;
			const ndRenderTextureImageCommon* const image = (ndRenderTextureImageCommon*)*material->m_texture;
			ndAssert(image);

			const glVector4 diffuse(material->m_diffuse);
			const glVector4 specular(material->m_specular);
			const glVector4 reflection(material->m_reflection);

			glUniform3fv(m_diffuseColor, 1, &diffuse[0]);
			glUniform3fv(m_specularColor, 1, &specular[0]);
			glUniform3fv(m_reflectionColor, 1, &reflection[0]);
			glUniform1fv(m_specularAlpha, 1, &material->m_specularPower);
			glUniform1fv(m_opacity, 1, &material->m_opacity);

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

// *********************************************************************
// 
// *********************************************************************
void ndRenderShaderInstancedOpaqueDiffusedShadowBlock::GetShaderParameters(const ndRenderShaderCache* const shaderCache)
{
	SetParameters(shaderCache->m_diffuseShadowIntanceEffect);
	EndParameters();
}

void ndRenderShaderInstancedOpaqueDiffusedShadowBlock::SetParameters(GLuint shader)
{
	ndRenderShaderOpaqueDiffusedShadowColorBlock::SetParameters(shader);
}

void ndRenderShaderInstancedOpaqueDiffusedShadowBlock::Render(const ndRenderPrimitiveImplement* const self, const ndRender* const render, const ndMatrix& modelMatrix) const
{
	const ndRenderSceneCamera* const camera = render->GetCamera()->FindCameraNode();

	const ndMatrix viewMatrix(camera->m_invViewMatrix);
	const ndMatrix modelViewMatrix(modelMatrix * viewMatrix);

	const glMatrix worldMatrix(modelMatrix);
	const glMatrix glViewModelMatrix(modelViewMatrix);
	const glMatrix glCameraToWorld(viewMatrix.OrthoInverse());
	const glMatrix glProjectionMatrix(camera->m_projectionMatrix);

	const glVector4 glSunlightAmbient(render->m_sunLightAmbient);
	const glVector4 glSunlightIntensity(render->m_sunLightIntesity);
	const glVector4 glSunlightDir(viewMatrix.RotateVector(render->m_sunLightDir));

	glUseProgram(m_shader);

	// upload matrix palette to the gpu vertex buffer
	glBindBuffer(GL_ARRAY_BUFFER, self->m_instanceMatrixBuffer);
	glMatrix* const matrixBuffer = (glMatrix*)glMapBuffer(GL_ARRAY_BUFFER, GL_WRITE_ONLY);
	ndMemCpy(matrixBuffer, &self->m_instanceMatrixArray[0], self->m_instanceMatrixArray.GetCount());
	glUnmapBuffer(GL_ARRAY_BUFFER);

	glUniform3fv(m_directionalLightDirection, 1, &glSunlightDir[0]);
	glUniform3fv(m_directionalLightAmbient, 1, &glSunlightAmbient[0]);
	glUniform3fv(m_directionalLightIntesity, 1, &glSunlightIntensity[0]);

	//glUniformMatrix4fv(m_normalMatrixLocation, 1, false, &glViewModelMatrix[0][0]);
	glUniformMatrix4fv(m_cameraToWorld, 1, false, &glCameraToWorld[0][0]);
	glUniformMatrix4fv(m_projectMatrixLocation, 1, false, &glProjectionMatrix[0][0]);
	glUniformMatrix4fv(m_viewModelMatrixLocation, 1, false, &glViewModelMatrix[0][0]);

	ndRenderPassShadowsImplement* const shadowPass = render->m_cachedShadowPass;
	ndAssert(shadowPass);

	glVector4 cameraSpaceSplits(shadowPass->m_cameraSpaceSplits);
	glMatrix lightViewProjectMatrix[4];
	for (ndInt32 i = 0; i < 4; ++i)
	{
		lightViewProjectMatrix[i] = shadowPass->m_lighProjectionMatrix[i];
	}
	glUniform1i(m_depthMapTexture, 1);
	glUniformMatrix4fv(m_worldMatrix, 1, GL_FALSE, &worldMatrix[0][0]);
	glUniformMatrix4fv(m_directionLightViewProjectionMatrixShadow, 4, GL_FALSE, &lightViewProjectMatrix[0][0][0]);
	glUniform4fv(m_shadowSlices, 1, &cameraSpaceSplits[0]);

	ndRenderPassEnvironment* const environment = render->m_cachedEnvironmentPass;
	ndAssert(environment);
	ndRenderTextureImage* const environmentTexture = (ndRenderTextureImage*)*environment->m_cubeMap;

	glActiveTexture(GL_TEXTURE2);
	glBindTexture(GL_TEXTURE_CUBE_MAP, environmentTexture->m_texture);

	glActiveTexture(GL_TEXTURE1);
	glBindTexture(GL_TEXTURE_2D, shadowPass->m_shadowMapTexture);

	glBindVertexArray(self->m_vertextArrayBuffer);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, self->m_indexBuffer);

	glActiveTexture(GL_TEXTURE0);
	for (ndList<ndRenderPrimitiveSegment>::ndNode* node = self->m_owner->m_segments.GetFirst(); node; node = node->GetNext())
	{
		ndRenderPrimitiveSegment& segment = node->GetInfo();
		if (segment.m_material.m_opacity > ndFloat32(0.99f))
		{
			const ndRenderPrimitiveMaterial* const material = &segment.m_material;
			const ndRenderTextureImageCommon* const image = (ndRenderTextureImageCommon*)*material->m_texture;
			ndAssert(image);

			const glVector4 diffuse(material->m_diffuse);
			const glVector4 specular(material->m_specular);
			const glVector4 reflection(material->m_reflection);

			glUniform3fv(m_diffuseColor, 1, &diffuse[0]);
			glUniform3fv(m_specularColor, 1, &specular[0]);
			glUniform3fv(m_reflectionColor, 1, &reflection[0]);
			glUniform1fv(m_specularAlpha, 1, &material->m_specularPower);

			glBindTexture(GL_TEXTURE_2D, image->m_texture);
			glDrawElementsInstanced(GL_TRIANGLES, segment.m_indexCount, GL_UNSIGNED_INT, (void*)(segment.m_segmentStart * sizeof(GL_UNSIGNED_INT)), GLsizei(self->m_instanceMatrixArray.GetCount()));
		}
	}

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindVertexArray(0);
	glUseProgram(0);
}

// *********************************************************************
// 
// *********************************************************************
void ndRenderShaderOpaqueDiffusedShadowSkinColorBlock::GetShaderParameters(const ndRenderShaderCache* const shaderCache)
{
	SetParameters(shaderCache->m_diffuseShadowSkinEffect);
	EndParameters();
}

void ndRenderShaderOpaqueDiffusedShadowSkinColorBlock::SetParameters(GLuint shader)
{
	ndRenderShaderOpaqueDiffusedShadowColorBlock::SetParameters(shader);
	m_matrixPalette = glGetUniformLocation(m_shader, "matrixPalette");
}

void ndRenderShaderOpaqueDiffusedShadowSkinColorBlock::Render(const ndRenderPrimitiveImplement* const self, const ndRender* const render, const ndMatrix& modelMatrix) const
{
	const ndRenderSceneCamera* const camera = render->GetCamera()->FindCameraNode();

	const ndMatrix viewMatrix(camera->m_invViewMatrix);
	const ndMatrix modelViewMatrix(modelMatrix * viewMatrix);

	const glMatrix worldMatrix(modelMatrix);
	const glMatrix glViewModelMatrix(modelViewMatrix);
	const glMatrix glCameraToWorld(viewMatrix.OrthoInverse());
	const glMatrix glProjectionMatrix(camera->m_projectionMatrix);

	const glVector4 glSunlightAmbient(render->m_sunLightAmbient);
	const glVector4 glSunlightIntensity(render->m_sunLightIntesity);
	const glVector4 glSunlightDir(viewMatrix.RotateVector(render->m_sunLightDir));

	glUseProgram(m_shader);

	glUniform3fv(m_directionalLightDirection, 1, &glSunlightDir[0]);
	glUniform3fv(m_directionalLightAmbient, 1, &glSunlightAmbient[0]);
	glUniform3fv(m_directionalLightIntesity, 1, &glSunlightIntensity[0]);

	glUniformMatrix4fv(m_projectMatrixLocation, 1, GL_FALSE, &glProjectionMatrix[0][0]);
	glUniformMatrix4fv(m_viewModelMatrixLocation, 1, GL_FALSE, &glViewModelMatrix[0][0]);

	ndRenderPassShadowsImplement* const shadowPass = render->m_cachedShadowPass;
	ndAssert(shadowPass);

	glVector4 cameraSpaceSplits(shadowPass->m_cameraSpaceSplits);
	glMatrix lightViewProjectMatrix[4];
	for (ndInt32 i = 0; i < 4; ++i)
	{
		lightViewProjectMatrix[i] = shadowPass->m_lighProjectionMatrix[i];
	}
	glUniform1i(m_depthMapTexture, 1);
	glUniformMatrix4fv(m_worldMatrix, 1, GL_FALSE, &worldMatrix[0][0]);
	glUniformMatrix4fv(m_cameraToWorld, 1, false, &glCameraToWorld[0][0]);
	glUniformMatrix4fv(m_directionLightViewProjectionMatrixShadow, 4, GL_FALSE, &lightViewProjectMatrix[0][0][0]);
	glUniform4fv(m_shadowSlices, 1, &cameraSpaceSplits[0]);
	glUniformMatrix4fv(m_matrixPalette, ndInt32(self->m_skinPaletteMatrixArray.GetCount()), GL_FALSE, &self->m_skinPaletteMatrixArray[0][0][0]);

	ndRenderPassEnvironment* const environment = render->m_cachedEnvironmentPass;
	ndAssert(environment);
	ndRenderTextureImage* const environmentTexture = (ndRenderTextureImage*)*environment->m_cubeMap;

	glActiveTexture(GL_TEXTURE2);
	glBindTexture(GL_TEXTURE_CUBE_MAP, environmentTexture->m_texture);

	glActiveTexture(GL_TEXTURE1);
	glBindTexture(GL_TEXTURE_2D, shadowPass->m_shadowMapTexture);
	
	glBindVertexArray(self->m_vertextArrayBuffer);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, self->m_indexBuffer);
	
	glActiveTexture(GL_TEXTURE0);
	for (ndList<ndRenderPrimitiveSegment>::ndNode* node = self->m_owner->m_segments.GetFirst(); node; node = node->GetNext())
	{
		ndRenderPrimitiveSegment& segment = node->GetInfo();
		if (segment.m_material.m_opacity > ndFloat32(0.99f))
		{
			const ndRenderPrimitiveMaterial* const material = &segment.m_material;
			const ndRenderTextureImageCommon* const image = (ndRenderTextureImageCommon*)*material->m_texture;
			ndAssert(image);
	
			const glVector4 diffuse(material->m_diffuse);
			const glVector4 specular(material->m_specular);
			const glVector4 reflection(material->m_reflection);
	
			glUniform3fv(m_diffuseColor, 1, &diffuse[0]);
			glUniform3fv(m_specularColor, 1, &specular[0]);
			glUniform3fv(m_reflectionColor, 1, &reflection[0]);
			glUniform1fv(m_specularAlpha, 1, &material->m_specularPower);
	
			glBindTexture(GL_TEXTURE_2D, image->m_texture);
			glDrawElements(GL_TRIANGLES, segment.m_indexCount, GL_UNSIGNED_INT, (void*)(segment.m_segmentStart * sizeof(GL_UNSIGNED_INT)));
		}
	}
	
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
	glBindVertexArray(0);
	glUseProgram(0);
}

// *********************************************************************
// 
// *********************************************************************
void ndRenderShaderGenerateSkinShadowMapBlock::GetShaderParameters(const ndRenderShaderCache* const shaderCache)
{
	SetParameters(shaderCache->m_generateShadowMapsSkinEffect);
	EndParameters();
}

void ndRenderShaderGenerateSkinShadowMapBlock::SetParameters(GLuint shader)
{
	ndRenderShaderGenerateShadowMapBlock::SetParameters(shader);
	m_matrixPalette = glGetUniformLocation(m_shader, "matrixPalette");
}

void ndRenderShaderGenerateSkinShadowMapBlock::Render(const ndRenderPrimitiveImplement* const self, const ndRender* const, const ndMatrix& modelMatrix) const
{
	glUseProgram(m_shader);
	
	const glMatrix matrix(modelMatrix);
	glUniformMatrix4fv(m_viewModelProjectionMatrix, 1, false, &matrix[0][0]);
	glUniformMatrix4fv(m_matrixPalette, ndInt32(self->m_skinPaletteMatrixArray.GetCount()), GL_FALSE, &self->m_skinPaletteMatrixArray[0][0][0]);
	
	glBindVertexArray(self->m_vertextArrayBuffer);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, self->m_indexBuffer);
	
	for (ndList<ndRenderPrimitiveSegment>::ndNode* node = self->m_owner->m_segments.GetFirst(); node; node = node->GetNext())
	{
		ndRenderPrimitiveSegment& segment = node->GetInfo();
		if (segment.m_material.m_castShadows)
		{
			glDrawElements(GL_TRIANGLES, segment.m_indexCount, GL_UNSIGNED_INT, (void*)(segment.m_segmentStart * sizeof(GL_UNSIGNED_INT)));
		}
	}
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
	glBindVertexArray(0);
	
	ndAssert(glCheckFramebufferStatus(GL_FRAMEBUFFER) == GL_FRAMEBUFFER_COMPLETE);
	glUseProgram(0);
}

// *********************************************************************
// 
// *********************************************************************
void ndRenderShaderStaticLinesArrayBlock::GetShaderParameters(const ndRenderShaderCache* const shaderCache)
{
	SetParameters(shaderCache->m_wireFrameEffect);
	EndParameters();
}

void ndRenderShaderStaticLinesArrayBlock::SetParameters(GLuint shader)
{
	ndRenderShaderBlock::SetParameters(shader);
	m_viewModelProjectionMatrix = glGetUniformLocation(m_shader, "viewModelProjectionMatrix");
}

void ndRenderShaderStaticLinesArrayBlock::Render(const ndRenderPrimitiveImplement* const self, const ndRender* const render, const ndMatrix& modelMatrix) const
{
	const ndRenderSceneCamera* const camera = render->GetCamera()->FindCameraNode();

	glUseProgram(m_shader);
	const ndMatrix modelViewProjectionMatrixMatrix(modelMatrix * camera->m_invViewMatrix * camera->m_projectionMatrix);
	const glMatrix glViewModelProjectionMatrix(modelViewProjectionMatrixMatrix);
	glUniformMatrix4fv(m_viewModelProjectionMatrix, 1, false, &glViewModelProjectionMatrix[0][0]);

	glBindVertexArray(self->m_vertextArrayBuffer);
	glDrawArrays(GL_LINES, 0, self->m_vertexCount);

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
	glBindVertexArray(0);
	glUseProgram(0);
}

// *********************************************************************
// 
// *********************************************************************
void ndRenderShaderDynamicLinesArrayBlock::GetShaderParameters(const ndRenderShaderCache* const shaderCache)
{
	SetParameters(shaderCache->m_lineEffect);
	EndParameters();
}

void ndRenderShaderDynamicLinesArrayBlock::SetParameters(GLuint shader)
{
	ndRenderShaderBlock::SetParameters(shader);
	m_viewModelProjectionMatrix = glGetUniformLocation(m_shader, "viewModelProjectionMatrix");
}

void ndRenderShaderDynamicLinesArrayBlock::Render(const ndRenderPrimitiveImplement* const self, const ndRender* const render, const ndMatrix& modelMatrix) const
{
	const ndRenderSceneCamera* const camera = render->GetCamera()->FindCameraNode();
	
	glUseProgram(m_shader);
	ndRenderPassDebug* const debugPass = render->m_cachedDebugPass;
	ndAssert(debugPass);

	const ndMatrix modelViewProjectionMatrixMatrix(modelMatrix * camera->m_invViewMatrix * camera->m_projectionMatrix);
	const glMatrix glViewModelProjectionMatrix(modelViewProjectionMatrixMatrix);
	glUniformMatrix4fv(m_viewModelProjectionMatrix, 1, false, &glViewModelProjectionMatrix[0][0]);

	glBindVertexArray(self->m_vertextArrayBuffer);
	glBindBuffer(GL_ARRAY_BUFFER, self->m_vertexBuffer);
	const ndArray<ndRenderPassDebug::ndPoint>& points = debugPass->GetVertex();

	glLineWidth(ndReal(2.0f));
	for (ndInt32 j = 0; j < points.GetCount(); j += self->m_vertexCount)
	{
		glPointColor* const bufferData = (glPointColor*)glMapBuffer(GL_ARRAY_BUFFER, GL_WRITE_ONLY);
		ndAssert(bufferData);
		
		const ndInt32 pointCount = ((j + self->m_vertexCount) > points.GetCount()) ? ndInt32(points.GetCount() - j) : self->m_vertexCount;
		for (ndInt32 i = 0; i < pointCount; ++ i)
		{
			bufferData[i].m_point = glVector3(points[i + j].m_point);
			bufferData[i].m_color = glVector3(points[i + j].m_color);
		}
		glUnmapBuffer(GL_ARRAY_BUFFER);
		glDrawArrays(GL_LINES, 0, pointCount);
	}

	glLineWidth(ndReal(1.0f));
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindVertexArray(0);
	glUseProgram(0);
}

// *********************************************************************
// 
// *********************************************************************
void ndRenderShaderDynamicPointsArrayBlock::GetShaderParameters(const ndRenderShaderCache* const shaderCache)
{
	SetParameters(shaderCache->m_pointEffect);
	EndParameters();
}

void ndRenderShaderDynamicPointsArrayBlock::SetParameters(GLuint shader)
{
	ndRenderShaderBlock::SetParameters(shader);
	m_viewModelProjectionMatrix = glGetUniformLocation(m_shader, "viewModelProjectionMatrix");
}

void ndRenderShaderDynamicPointsArrayBlock::Render(const ndRenderPrimitiveImplement* const self, const ndRender* const render, const ndMatrix& modelMatrix) const
{
	const ndRenderSceneCamera* const camera = render->GetCamera()->FindCameraNode();
	
	glUseProgram(m_shader);
	ndRenderPassDebug* const debugPass = render->m_cachedDebugPass;
	ndAssert(debugPass);
	
	const ndMatrix modelViewProjectionMatrixMatrix(modelMatrix * camera->m_invViewMatrix * camera->m_projectionMatrix);
	const glMatrix glViewModelProjectionMatrix(modelViewProjectionMatrixMatrix);
	glUniformMatrix4fv(m_viewModelProjectionMatrix, 1, false, &glViewModelProjectionMatrix[0][0]);
	
	glBindVertexArray(self->m_vertextArrayBuffer);
	glBindBuffer(GL_ARRAY_BUFFER, self->m_vertexBuffer);
	const ndArray<ndRenderPassDebug::ndPoint>& points = debugPass->GetPoints();
	
	glPointSize(ndReal(4.0f));
	for (ndInt32 j = 0; j < points.GetCount(); j += self->m_vertexCount)
	{
		glPointColor* const bufferData = (glPointColor*)glMapBuffer(GL_ARRAY_BUFFER, GL_WRITE_ONLY);
		ndAssert(bufferData);
	
		const ndInt32 pointCount = ((j + self->m_vertexCount) > points.GetCount()) ? ndInt32(points.GetCount() - j) : self->m_vertexCount;
		for (ndInt32 i = 0; i < pointCount; ++i)
		{
			bufferData[i].m_point = glVector3(points[i + j].m_point);
			bufferData[i].m_color = glVector3(points[i + j].m_color);
		}
		glUnmapBuffer(GL_ARRAY_BUFFER);
		glDrawArrays(GL_POINTS, 0, pointCount);
	}
	
	glPointSize(ndReal(1.0f));
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindVertexArray(0);
	glUseProgram(0);
}
