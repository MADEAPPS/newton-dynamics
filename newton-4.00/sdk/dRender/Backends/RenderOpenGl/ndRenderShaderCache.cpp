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
#include "ndRenderOpenGlUtil.h"
#include "ndRenderSceneCamera.h"
#include "ndRenderShaderCache.h"
#include "ndRenderPassShadows.h"
#include "ndRenderTextureImage.h"
#include "ndRenderPassEnvironment.h"
#include "ndRenderPassShadowsImplement.h"
#include "ndRenderPrimitiveMeshImplement.h"

ndRenderShaderBlock::ndRenderShaderBlock()
	:m_shader(0)
{
}

ndRenderShaderBlock::~ndRenderShaderBlock()
{
}

// *********************************************************************
// 
// *********************************************************************
ndRenderShaderCache::ndRenderShaderCache(void)
{
	ndMemSet(m_shaders, GLuint(0), sizeof(m_shaders) / sizeof(GLuint));
	CreateAllEffects();
}

ndRenderShaderCache::~ndRenderShaderCache(void)
{
}

bool ndRenderShaderCache::CreateAllEffects()
{
	m_skyBoxEffect = CreateShaderEffect(m_skyBoxVertex, m_skyBoxPixel);
	m_setZbufferEffect = CreateShaderEffect(m_setZbufferVertex, m_doNothingPixel);
	m_diffuseEffect = CreateShaderEffect(m_directionalDiffuseVertex, m_directionalDiffusePixel);
	m_generateShadowMapsEffect = CreateShaderEffect(m_generateShadowMapVertex, m_doNothingPixel);
	m_diffuseIntanceEffect = CreateShaderEffect(m_directionalDiffuseInstanceVertex, m_directionalDiffusePixel);
	m_diffuseShadowEffect = CreateShaderEffect(m_directionalDiffuseShadowVertex, m_directionalDiffuseShadowPixel);
	m_diffuseTransparentEffect = CreateShaderEffect(m_directionalDiffuseVertex, m_directionalDiffuseTransparentPixel);

	m_debugFlatShadedDiffuseEffect = CreateShaderEffect(m_debugFlatShadedDiffuseVertex, m_debugFlatShadedDiffusePixel);

	//m_wireFrame = CreateShaderEffect("WireFrame", "FlatShaded");
	//m_flatShaded = CreateShaderEffect("FlatShaded", "FlatShaded");
	//m_colorPoint = CreateShaderEffect("ColorPoint", "FlatShaded");
	//m_texturedDecal = CreateShaderEffect ("TextureDecal", "TextureDecal");
	//m_diffuseDebrisEffect = CreateShaderEffect("DirectionalDebriDiffuse", "DirectionalDebriDiffuse");
	//m_skinningDiffuseEffect = CreateShaderEffect ("SkinningDirectionalDiffuse", "DirectionalDiffuse");
	//m_diffuseIntanceEffect = CreateShaderEffect ("DirectionalDiffuseInstance", "DirectionalDiffuse");
	//m_thickPoints = CreateShaderEffect("ThickPoint", "ThickPoint", "ThickPoint");
	//m_spriteSpheres = CreateShaderEffect("DirectionalDiffuseSprite", "DirectionalDiffuseSprite", "DirectionalDiffuseSprite");

	return true;
}

GLuint ndRenderShaderCache::CreateShaderEffect (const char* const vertexShaderCode, const char* const pixelShaderCode, const char* const geometryShaderCode)
{
	GLint state;
	char errorLog[GL_INFO_LOG_LENGTH];

	GLuint program = glCreateProgram();
	GLuint vertexShader = glCreateShader(GL_VERTEX_SHADER);
	
	glShaderSource(vertexShader, 1, &vertexShaderCode, nullptr);
	glCompileShader(vertexShader);
	glGetShaderiv(vertexShader, GL_COMPILE_STATUS, &state); 
	if (state != GL_TRUE ) 
	{
		GLsizei length;  
		glGetShaderInfoLog(vertexShader, sizeof (errorLog), &length, errorLog);
		ndTrace ((errorLog));
		ndAssert(0);
	}
	glAttachShader(program, vertexShader);
	
	// load and compile geometry shader, if any
	GLuint geometryShader = 0;
	if (geometryShaderCode)
	{
		ndAssert(0);
		//snprintf(tmpName, sizeof(tmpName), "shaders/%s.gs", geometryShaderCode);
		//LoadShaderCode(tmpName, buffer);
		//geometryShader = glCreateShader(GL_GEOMETRY_SHADER);
	
		//glShaderSource(geometryShader, 1, &vPtr, nullptr);
		//glCompileShader(geometryShader);
		//glGetShaderiv(geometryShader, GL_COMPILE_STATUS, &state);
		//if (state != GL_TRUE)
		//{
		//	GLsizei length;
		//	glGetShaderInfoLog(geometryShader, sizeof(buffer), &length, errorLog);
		//	ndTrace((errorLog));
		//}
		//glAttachShader(program, geometryShader);
	}
	
	GLuint pixelShader = glCreateShader(GL_FRAGMENT_SHADER);
	glShaderSource(pixelShader, 1, &pixelShaderCode, nullptr);
	glCompileShader(pixelShader);
	glGetShaderiv(pixelShader, GL_COMPILE_STATUS, &state); 
	if (state != GL_TRUE ) 
	{
		GLsizei length;  
		glGetShaderInfoLog(pixelShader, sizeof(errorLog), &length, errorLog);
		ndTrace((errorLog));
		ndAssert(0);
	}
	glAttachShader(program, pixelShader);
	
	glLinkProgram(program);
	glGetProgramiv(program, GL_LINK_STATUS, &state);   
	if (state != GL_TRUE ) 
	{
		GLsizei length;  
		glGetProgramInfoLog(program, sizeof(errorLog), &length, errorLog);
		ndTrace((errorLog));
		ndAssert(0);
	}
	
	glValidateProgram(program);
	glGetProgramiv(program,  GL_VALIDATE_STATUS, &state);   
	if (state != GL_TRUE)
	{
		GLsizei length;
		glGetProgramInfoLog(program, sizeof(errorLog), &length, errorLog);
		ndTrace((errorLog));
		ndAssert(0);
	}
	
	if (geometryShader)
	{
		glDeleteShader(geometryShader);
	}

	glDeleteShader(pixelShader);
	glDeleteShader(vertexShader);
	return program;
}

void ndRenderShaderCache::Cleanup()
{
	for (ndInt32 i = 0; i < ndInt32(sizeof(m_shaders) / sizeof(m_shaders[0])); ++i)
	{
		if (m_shaders[i])
		{
			glDeleteProgram(m_shaders[i]);
			m_shaders[i] = 0;
		}
	}
}

// *********************************************************************
// 
// *********************************************************************
void ndRenderShaderSetZbufferCleanBlock::GetShaderParameters(const ndRenderShaderCache* const shaderCache)
{
	m_shader = shaderCache->m_setZbufferEffect;
	glUseProgram(m_shader);
	viewModelProjectionMatrix = glGetUniformLocation(m_shader, "viewModelProjectionMatrix");
	glUseProgram(0);
}

void ndRenderShaderSetZbufferCleanBlock::Render(const ndRenderPrimitiveMeshImplement* const self, const ndRender* const render, const ndMatrix& modelMatrix) const
{
	const ndSharedPtr<ndRenderSceneCamera>& camera = render->GetCamera();

	//const ndMatrix modelViewProjectionMatrixMatrix(modelMatrix * camera->m_invViewMatrix * camera->m_projectionMatrix);
	const ndMatrix modelViewProjectionMatrixMatrix(modelMatrix * camera->m_invViewRrojectionMatrix);
	const glMatrix glViewModelProjectionMatrix(modelViewProjectionMatrixMatrix);

	glCullFace(GL_BACK);
	glFrontFace(GL_CCW);
	glDisable(GL_BLEND);
	glDepthFunc(GL_LEQUAL);
	glEnable(GL_CULL_FACE);
	glEnable(GL_DEPTH_TEST);

	glUseProgram(m_shader);

	glUniformMatrix4fv(viewModelProjectionMatrix, 1, false, &glViewModelProjectionMatrix[0][0]);

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
	m_shader = shaderCache->m_setZbufferEffect;
	glUseProgram(m_shader);
	viewModelProjectionMatrix = glGetUniformLocation(m_shader, "viewModelProjectionMatrix");
	glUseProgram(0);
}

void ndRenderShaderGenerateShadowMapBlock::BeginRender()
{
	glDisable(GL_SCISSOR_TEST);
	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LEQUAL);
	
	glClear(GL_DEPTH_BUFFER_BIT);
	
	glUseProgram(m_shader);
	
	glPolygonOffset(GLfloat(1.0f), GLfloat(1024.0f * 8.0f));
	glEnable(GL_POLYGON_OFFSET_FILL);
}

void ndRenderShaderGenerateShadowMapBlock::EndRender()
{
	glDisable(GL_POLYGON_OFFSET_FILL);
	ndAssert(glCheckFramebufferStatus(GL_FRAMEBUFFER) == GL_FRAMEBUFFER_COMPLETE);
	glUseProgram(0);
}

void ndRenderShaderGenerateShadowMapBlock::Render(const ndRenderPrimitiveMeshImplement* const self, const ndRender* const, const ndMatrix& modelMatrix) const
{
	glMatrix matrix(modelMatrix);
	glUniformMatrix4fv(viewModelProjectionMatrix, 1, false, &matrix[0][0]);

	glBindVertexArray(self->m_vertextArrayBuffer);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, self->m_indexBuffer);
		
	for (ndList<ndRenderPrimitiveMeshSegment>::ndNode* node = self->m_owner->m_segments.GetFirst(); node; node = node->GetNext())
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

// *********************************************************************
// 
// *********************************************************************
void ndRenderShaderDebugFlatShadedDiffusedBlock::GetShaderParameters(const ndRenderShaderCache* const shaderCache)
{
	m_shader = shaderCache->m_debugFlatShadedDiffuseEffect;
	glUseProgram(m_shader);
	m_diffuseColor = glGetUniformLocation(m_shader, "diffuseColor");
	m_projectMatrixLocation = glGetUniformLocation(m_shader, "projectionMatrix");
	m_viewModelMatrixLocation = glGetUniformLocation(m_shader, "viewModelMatrix");
	m_directionalLightAmbient = glGetUniformLocation(m_shader, "directionalLightAmbient");
	m_directionalLightIntesity = glGetUniformLocation(m_shader, "directionalLightIntesity");
	m_directionalLightDirection = glGetUniformLocation(m_shader, "directionalLightDirection");
	glUseProgram(0);
}

void ndRenderShaderDebugFlatShadedDiffusedBlock::Render(const ndRenderPrimitiveMeshImplement* const self, const ndRender* const render, const ndMatrix& modelMatrix) const
{
	const ndSharedPtr<ndRenderSceneCamera>& camera = render->GetCamera();
	
	const ndMatrix viewMatrix(camera->m_invViewMatrix);
	const ndMatrix modelViewMatrix(modelMatrix * viewMatrix);
	
	const glMatrix glViewModelMatrix(modelViewMatrix);
	const glMatrix glProjectionMatrix(camera->m_projectionMatrix);
	
	ndRenderPrimitiveMeshSegment& segment = self->m_owner->m_segments.GetFirst()->GetInfo();
	const ndRenderPrimitiveMeshMaterial* const material = &segment.m_material;
	
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
void ndRenderShaderDebugWireframeDiffuseBlock::Render(const ndRenderPrimitiveMeshImplement* const self, const ndRender* const render, const ndMatrix& modelMatrix) const
{
	const ndSharedPtr<ndRenderSceneCamera>& camera = render->GetCamera();
	
	const ndMatrix viewMatrix(camera->m_invViewMatrix);
	const ndMatrix modelViewMatrix(modelMatrix * viewMatrix);
	
	const glMatrix glViewModelMatrix(modelViewMatrix);
	const glMatrix glProjectionMatrix(camera->m_projectionMatrix);
	
	ndRenderPrimitiveMeshSegment& segment = self->m_owner->m_segments.GetFirst()->GetInfo();
	const ndRenderPrimitiveMeshMaterial* const material = &segment.m_material;
	
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
	m_shader = shaderCache->m_diffuseEffect;
	glUseProgram(m_shader);
	//m_normalMatrixLocation = glGetUniformLocation(m_shader, "normalMatrix");
	m_texture = glGetUniformLocation(m_shader, "texture0");
	m_environmentMap = glGetUniformLocation(m_shader, "environmentMap");
	m_projectMatrixLocation = glGetUniformLocation(m_shader, "projectionMatrix");
	m_viewModelMatrixLocation = glGetUniformLocation(m_shader, "viewModelMatrix");
	m_diffuseColor = glGetUniformLocation(m_shader, "diffuseColor");
	m_specularColor = glGetUniformLocation(m_shader, "specularColor");
	m_reflectionColor = glGetUniformLocation(m_shader, "reflectionColor");
	m_directionalLightAmbient = glGetUniformLocation(m_shader, "directionalLightAmbient");
	m_directionalLightIntesity = glGetUniformLocation(m_shader, "directionalLightIntesity");
	m_directionalLightDirection = glGetUniformLocation(m_shader, "directionalLightDirection");
	m_specularAlpha = glGetUniformLocation(m_shader, "specularAlpha");
	glUseProgram(0);
}

void ndRenderShaderOpaqueDiffusedColorBlock::Render(const ndRenderPrimitiveMeshImplement* const self, const ndRender* const render, const ndMatrix& modelMatrix) const
{
	const ndSharedPtr<ndRenderSceneCamera>& camera = render->GetCamera();

	glUseProgram(m_shader);

	const ndMatrix viewMatrix(camera->m_invViewMatrix);
	const ndMatrix modelViewMatrix(modelMatrix * viewMatrix);

	const glMatrix glViewModelMatrix(modelViewMatrix);
	const glMatrix glProjectionMatrix(camera->m_projectionMatrix);

	const glVector4 glSunlightAmbient(render->m_sunLightAmbient);
	const glVector4 glSunlightIntensity(render->m_sunLightIntesity);
	const glVector4 glSunlightDir(viewMatrix.RotateVector(render->m_sunLightDir));

	glUniform3fv(m_directionalLightDirection, 1, &glSunlightDir[0]);
	glUniform3fv(m_directionalLightAmbient, 1, &glSunlightAmbient[0]);
	glUniform3fv(m_directionalLightIntesity, 1, &glSunlightIntensity[0]);

	//glUniformMatrix4fv(m_normalMatrixLocation, 1, false, &glViewModelMatrix[0][0]);
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
	for (ndList<ndRenderPrimitiveMeshSegment>::ndNode* node = self->m_owner->m_segments.GetFirst(); node; node = node->GetNext())
	{
		ndRenderPrimitiveMeshSegment& segment = node->GetInfo();
		if (!segment.m_material.m_castShadows && (segment.m_material.m_opacity > ndFloat32(0.99f)))
		{
			const ndRenderPrimitiveMeshMaterial* const material = &segment.m_material;
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
	m_shader = shaderCache->m_diffuseShadowEffect;
	glUseProgram(m_shader);
	//m_normalMatrixLocation = glGetUniformLocation(m_shader, "normalMatrix");

	m_texture = glGetUniformLocation(m_shader, "texture0");
	m_environmentMap = glGetUniformLocation(m_shader, "environmentMap");
	m_depthMapTexture = glGetUniformLocation(m_shader, "shadowMapTexture");

	m_projectMatrixLocation = glGetUniformLocation(m_shader, "projectionMatrix");
	m_viewModelMatrixLocation = glGetUniformLocation(m_shader, "viewModelMatrix");
	m_diffuseColor = glGetUniformLocation(m_shader, "diffuseColor");
	m_specularColor = glGetUniformLocation(m_shader, "specularColor");
	m_reflectionColor = glGetUniformLocation(m_shader, "reflectionColor");
	m_directionalLightAmbient = glGetUniformLocation(m_shader, "directionalLightAmbient");
	m_directionalLightIntesity = glGetUniformLocation(m_shader, "directionalLightIntesity");
	m_directionalLightDirection = glGetUniformLocation(m_shader, "directionalLightDirection");
	m_specularAlpha = glGetUniformLocation(m_shader, "specularAlpha");

	m_shadowSlices = glGetUniformLocation(m_shader, "shadowSlices");
	m_worldMatrix = glGetUniformLocation(m_shader, "modelWorldMatrix");
	m_directionLightViewProjectionMatrixShadow = glGetUniformLocation(m_shader, "directionaLightViewProjectionMatrix");
	glUseProgram(0);
}

void ndRenderShaderOpaqueDiffusedShadowColorBlock::Render(const ndRenderPrimitiveMeshImplement* const self, const ndRender* const render, const ndMatrix& modelMatrix) const
{
	const ndSharedPtr<ndRenderSceneCamera>& camera = render->GetCamera();
	
	const ndMatrix viewMatrix(camera->m_invViewMatrix);
	const ndMatrix modelViewMatrix(modelMatrix * viewMatrix);
	
	const glMatrix worldMatrix(modelMatrix);
	const glMatrix glViewModelMatrix(modelViewMatrix);
	const glMatrix glProjectionMatrix(camera->m_projectionMatrix);
	
	const glVector4 glSunlightAmbient(render->m_sunLightAmbient);
	const glVector4 glSunlightIntensity(render->m_sunLightIntesity);
	const glVector4 glSunlightDir(viewMatrix.RotateVector(render->m_sunLightDir));

	glUseProgram(m_shader);

	glUniform3fv(m_directionalLightDirection, 1, &glSunlightDir[0]);
	glUniform3fv(m_directionalLightAmbient, 1, &glSunlightAmbient[0]);
	glUniform3fv(m_directionalLightIntesity, 1, &glSunlightIntensity[0]);
	
	//glUniformMatrix4fv(m_normalMatrixLocation, 1, false, &glViewModelMatrix[0][0]);
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
	for (ndList<ndRenderPrimitiveMeshSegment>::ndNode* node = self->m_owner->m_segments.GetFirst(); node; node = node->GetNext())
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
	m_shader = shaderCache->m_diffuseTransparentEffect;
	glUseProgram(m_shader);
	//m_normalMatrixLocation = glGetUniformLocation(m_shader, "normalMatrix");
	m_texture = glGetUniformLocation(m_shader, "texture0");
	m_environmentMap = glGetUniformLocation(m_shader, "environmentMap");

	m_projectMatrixLocation = glGetUniformLocation(m_shader, "projectionMatrix");
	m_viewModelMatrixLocation = glGetUniformLocation(m_shader, "viewModelMatrix");
	m_diffuseColor = glGetUniformLocation(m_shader, "diffuseColor");
	m_specularColor = glGetUniformLocation(m_shader, "specularColor");
	m_reflectionColor = glGetUniformLocation(m_shader, "reflectionColor");
	m_directionalLightAmbient = glGetUniformLocation(m_shader, "directionalLightAmbient");
	m_directionalLightIntesity = glGetUniformLocation(m_shader, "directionalLightIntesity");
	m_directionalLightDirection = glGetUniformLocation(m_shader, "directionalLightDirection");
	m_specularAlpha = glGetUniformLocation(m_shader, "specularAlpha");
	m_opacity = glGetUniformLocation(m_shader, "opacity");
	glUseProgram(0);
}

void ndRenderShaderTransparentDiffusedShadowColorBlock::SetWidingMode(bool clockwise) const
{
	clockwise ? glFrontFace(GL_CW) : glFrontFace(GL_CCW);
}

void ndRenderShaderTransparentDiffusedShadowColorBlock::Render(const ndRenderPrimitiveMeshImplement* const self, const ndRender* const render, const ndMatrix& modelMatrix) const
{
	const ndSharedPtr<ndRenderSceneCamera>& camera = render->GetCamera();
	
	glUseProgram(m_shader);

	const ndMatrix viewMatrix(camera->m_invViewMatrix);
	const ndMatrix modelViewMatrix(modelMatrix * viewMatrix);

	const glMatrix glViewModelMatrix(modelViewMatrix);
	const glMatrix glProjectionMatrix(camera->m_projectionMatrix);

	const glVector4 glSunlightAmbient(render->m_sunLightAmbient);
	const glVector4 glSunlightIntensity(render->m_sunLightIntesity);
	const glVector4 glSunlightDir(viewMatrix.RotateVector(render->m_sunLightDir));

	glUniform3fv(m_directionalLightDirection, 1, &glSunlightDir[0]);
	glUniform3fv(m_directionalLightAmbient, 1, &glSunlightAmbient[0]);
	glUniform3fv(m_directionalLightIntesity, 1, &glSunlightIntensity[0]);

	//glUniformMatrix4fv(m_normalMatrixLocation, 1, false, &glViewModelMatrix[0][0]);
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
	for (ndList<ndRenderPrimitiveMeshSegment>::ndNode* node = self->m_owner->m_segments.GetFirst(); node; node = node->GetNext())
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
